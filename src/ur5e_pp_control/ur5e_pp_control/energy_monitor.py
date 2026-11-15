"""Live energy monitor (the presentation view).

    ros2 run ur5e_pp_control energy_monitor                       # box task, live
    ros2 run ur5e_pp_control energy_monitor --ros-args -p scenario:=cans -p label:=AFTER \\
        -p compare:=~/ur5e_pp_ws/results/cans/cans_baseline        # live, with the "before" run behind it
    ros2 run ur5e_pp_control energy_monitor --ros-args -p scenario:=cans \\
        -p replay:=~/ur5e_pp_ws/results/cans/cans_energyopt -p compare:=~/ur5e_pp_ws/results/cans/cans_baseline
                                                                   # replay a recorded run, no Gazebo needed

Live: subscribes to /joint_states (q, q-dot) and /pp/phase ('<cycle> <segment>',
published by pp_runner). Replay: plays a recorded run (<tag>.csv) at
replay_speed. Three panels:
  1. power now: electrical, mechanical and copper loss
  2. energy used in this cycle; grey: the same cycle (same carton slot) of the
     compare run, i.e. the "before"
  3. total energy since the start, this run vs the compare run, with the saving
Torque and energy model are the ones validate.py uses (inverse dynamics of the
motion, sampled every 10 ms like the runner's log).
"""
import json
import os
import signal
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from . import cans
from .dynamics import ATTACHED, BOX_SIZE, Dynamics
from .kinematics import JOINT_NAMES
from .validate import CU, ELEC, GRID, INK, MECH, MUTED, PHASE_FILL, _plt, csv_of, cumulative, load_run, power, select

SAMPLE_DT = 0.0095    # s, ~100 Hz like the runner's log (q-dot is differentiated at this step)


def _run_path(p):
    """Results path without the .csv / .json extension."""
    base, ext = os.path.splitext(os.path.expanduser(p))
    return base if ext in ('.csv', '.json') else base + ext


class EnergyMonitor(Node):
    def __init__(self):
        super().__init__('energy_monitor')
        p = self.declare_parameter
        self.scenario = p('scenario', 'box').value
        self.box_mass = p('box_mass', cans.CAN_MASS if self.scenario == 'cans' else 0.5).value
        self.label = p('label', '').value
        self.style = p('style', 'detail').value     # detail (engineering view) | video (for a screen recording)
        compare = p('compare', '').value
        replay = p('replay', '').value
        self.replay_speed = p('replay_speed', 1.0).value
        if self.scenario == 'cans':
            self.dyn = Dynamics(obj_height=cans.CAN_HEIGHT, obj_radius=cans.CAN_RADIUS)
        else:
            self.dyn = Dynamics(obj_height=BOX_SIZE)
        self.n_slots = cans.N_SLOTS if self.scenario == 'cans' else 1
        self.qd_prev = None
        self.lock = threading.Lock()
        self.cycle = 0
        self.segment = 'waiting for pp_runner'
        self.t_now = None
        self.totals = []            # E_elec of every finished cycle
        self.prev = None            # (t, E_elec) of the last finished cycle
        self._reset(None)
        self.compare = self._load_compare(compare) if compare else None
        if replay:
            threading.Thread(target=self._replay, args=(_run_path(replay),), daemon=True).start()
        else:
            self.create_subscription(JointState, '/joint_states', self._on_joints, 50)
            self.create_subscription(String, '/pp/phase', self._on_phase, 10)

    # -------------------------------------------------------------- compare run
    def _load_compare(self, path):
        """Energy curve of every cycle of a recorded run (the "before")."""
        base = _run_path(path)
        models = {}
        meta, run = load_run(base + '.json', lambda h, r: models.setdefault((h, r), Dynamics(obj_height=h, obj_radius=r)))
        out = {'tag': meta['tag'], 'cycle_E': {}, 'slot_curve': {}}
        for c in meta['cycles']:
            r = select(run, c['cycle'])
            if len(r['t']) < 2:
                continue
            P = np.maximum(power(r['tau'], r['qd'])[0].sum(1), 0.0)
            E = cumulative(P, r['t'])
            out['cycle_E'][c['cycle']] = float(E[-1])
            slot = c.get('slot', 0)
            if slot not in out['slot_curve'] and (c['cycle'] > 1 or self.scenario == 'cans'):
                out['slot_curve'][slot] = (r['t'], E)
        self.get_logger().info(f"compare run {meta['tag']}: {len(out['cycle_E'])} cycles")
        return out

    # -------------------------------------------------------------- input
    def _reset(self, t0):
        self.t0 = t0
        self.t, self.P, self.Pm, self.Pc = [], [], [], []
        self.E, self.Em, self.Ec = [0.0], [0.0], [0.0]
        self.marks = []             # (t, segment) where each phase starts

    def _on_phase(self, msg):
        cycle, seg = msg.data.split(' ', 1)
        cycle = int(cycle)
        with self.lock:
            if cycle != self.cycle:
                if self.cycle > 0 and len(self.t) > 1:
                    self.totals.append(self.E[-1])
                    self.prev = (np.array(self.t), np.array(self.E))
                self.cycle = cycle
                self._reset(self.t_now)
            self.segment = seg
            if self.t_now is not None and seg != 'start':     # 'start' is the single t=0 sample
                self.marks.append((self.t_now - self.t0, seg))

    def _on_joints(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(n in idx for n in JOINT_NAMES) or not len(msg.velocity):
            return
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self._sample(t, np.array([[msg.position[idx[n]] for n in JOINT_NAMES]]),
                     np.array([[msg.velocity[idx[n]] for n in JOINT_NAMES]]))

    def _sample(self, t, q, qd):
        if self.t_now is not None and t - self.t_now < SAMPLE_DT:
            return                          # keep 10 ms samples (duplicates, reset clock, 500 Hz stream)
        qdd = (qd - self.qd_prev) / (t - self.t_now) if self.qd_prev is not None else np.zeros_like(qd)
        self.qd_prev = qd
        box = self.box_mass if self.segment in ATTACHED else 0.0
        p_el, p_mech, p_cu = power(self.dyn.tau(q, qd, qdd, box), qd)
        P, Pm, Pc = max(p_el.sum(), 0.0), max(p_mech.sum(), 0.0), float(p_cu.sum())
        with self.lock:
            self.t_now = t
            if self.t0 is None:
                self.t0 = t
            if self.t:
                dt = (t - self.t0) - self.t[-1]
                self.E.append(self.E[-1] + 0.5 * (P + self.P[-1]) * dt)
                self.Em.append(self.Em[-1] + 0.5 * (Pm + self.Pm[-1]) * dt)
                self.Ec.append(self.Ec[-1] + 0.5 * (Pc + self.Pc[-1]) * dt)
            self.t.append(t - self.t0)
            self.P.append(P)
            self.Pm.append(Pm)
            self.Pc.append(Pc)

    def _replay(self, base):
        """Play a recorded run as if it were live (same messages, same timing)."""
        d = np.genfromtxt(csv_of(base), delimiter=',', names=True, dtype=None, encoding=None)
        with open(base + '.json') as f:
            self.get_logger().info(f"replaying {json.load(f)['tag']} at {self.replay_speed}x")
        cyc, seg, tt = np.asarray(d['cycle']), np.asarray(d['segment'], str), np.asarray(d['t'], float)
        q = np.stack([np.asarray(d[f'q_{j}'], float) for j in range(6)], 1)
        qd = np.stack([np.asarray(d[f'qd_{j}'], float) for j in range(6)], 1)
        offset, prev, t_abs = 0.0, (None, None), 0.0
        wall0, sim0 = time.monotonic(), None
        for k in range(len(tt)):
            if k and cyc[k] != cyc[k - 1]:
                offset = t_abs + 0.01           # time keeps running from one cycle to the next
            t_abs = offset + tt[k]
            if (cyc[k], seg[k]) != prev:
                self._on_phase(String(data=f'{cyc[k]} {seg[k]}'))
                prev = (cyc[k], seg[k])
            self._sample(t_abs, q[k:k + 1], qd[k:k + 1])
            sim0 = t_abs if sim0 is None else sim0
            lag = (t_abs - sim0) / self.replay_speed - (time.monotonic() - wall0)
            if lag > 0:
                time.sleep(lag)
        with self.lock:
            if len(self.t) > 1:
                self.totals.append(self.E[-1])
            self.segment = 'replay finished'

    def snapshot(self):
        with self.lock:
            n = len(self.t)
            return {'t': np.array(self.t), 'P': np.array(self.P), 'Pm': np.array(self.Pm),
                    'Pc': np.array(self.Pc), 'E': np.array(self.E[:n]), 'Em': np.array(self.Em[:n]),
                    'Ec': np.array(self.Ec[:n]), 'marks': list(self.marks), 'cycle': self.cycle,
                    'segment': self.segment, 'totals': list(self.totals), 'prev': self.prev}


def _detail_view(plt, node, cmp):
    """Engineering view: power, energy of this cycle, running totals."""
    fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(11.5, 9.2), gridspec_kw={'hspace': 0.42,
                                                                          'height_ratios': [1, 1, 0.9]})
    fig.canvas.manager.set_window_title('UR5e energy monitor' + (f' - {node.label}' if node.label else ''))
    lP, = a1.plot([], [], color=ELEC, label='electrical  max(0, Σ Pᵢ)')
    lPm, = a1.plot([], [], color=MECH, label='mechanical  max(0, Σ τᵢq̇ᵢ)')
    lPc, = a1.plot([], [], color=CU, label='copper loss  Σ τᵢ²/(N kₘ)²')
    lref, = a2.plot([], [], color=GRID, lw=5, label=f"before: {cmp['tag']}" if cmp else 'previous cycle')
    lE, = a2.plot([], [], color=ELEC, label='E_elec, this cycle')
    lEc, = a2.plot([], [], color=CU, lw=1.5, label='copper-loss part')
    lrun_ref, = a3.plot([], [], color=MUTED, lw=2.5, ls='--', marker='o', ms=4, label='before (compare run)')
    lrun, = a3.plot([], [], color=ELEC, lw=2.5, marker='o', ms=5, label='this run')
    a1.set_ylabel('power (W)')
    a1.set_xlabel('time in cycle (s)')
    a2.set_ylabel('energy in this cycle (J)')
    a2.set_xlabel('time in cycle (s)')
    a3.set_ylabel('total energy (J)')
    a3.set_xlabel('cans packed' if node.scenario == 'cans' else 'cycles done')
    from matplotlib.ticker import MaxNLocator
    a3.xaxis.set_major_locator(MaxNLocator(integer=True))
    a1.legend(loc='upper right', fontsize=9, labelcolor=INK)
    a2.legend(loc='upper left', fontsize=9, labelcolor=INK)
    a3.legend(loc='upper left', fontsize=9, labelcolor=INK)
    if not cmp:
        lrun_ref.set_visible(False)
    fig.text(0.01, 0.995, node.label or 'UR5e energy monitor', va='top', fontsize=16, color=INK,
             fontweight='bold')
    status = fig.text(0.01, 0.962, '', va='top', fontsize=11, color=INK)
    saving = a3.text(0.99, 0.08, '', transform=a3.transAxes, ha='right', va='bottom', fontsize=13,
                     fontweight='bold', color=INK)
    shades = []

    def update(_):
        s = node.snapshot()
        for a in shades:
            a.remove()
        shades.clear()
        t = s['t']
        if len(t) < 2:
            status.set_text(f"cycle {s['cycle']} · {s['segment']} · waiting for /joint_states")
            return []
        slot = (s['cycle'] - 1) % node.n_slots if s['cycle'] else 0
        lP.set_data(t, s['P'])
        lPm.set_data(t, s['Pm'])
        lPc.set_data(t, s['Pc'])
        lE.set_data(t, s['E'])
        lEc.set_data(t, s['Ec'])
        ref = cmp['slot_curve'].get(slot) if cmp else s['prev']
        if ref is not None:
            lref.set_data(*ref)
        marks = s['marks'] + [(t[-1], None)]
        for k, ((t0, name), (t1, _)) in enumerate(zip(marks[:-1], marks[1:])):
            for ax in (a1, a2):
                shades.append(ax.axvspan(t0, t1, color=PHASE_FILL[k % 2], lw=0, zorder=0))
            shades.append(a1.text((t0 + t1) / 2, 1.0 + 0.06 * (k % 2), name.replace('_', ' '),
                                  transform=a1.get_xaxis_transform(), ha='center', va='bottom',
                                  fontsize=8, color=MUTED))
        t_end = max(t[-1], ref[0][-1] if ref is not None else 0.0, 1.0)
        for ax in (a1, a2):
            ax.set_xlim(0, t_end * 1.02)
        a1.set_ylim(0, max(s['P'].max(), 1.0) * 1.15)
        a2.set_ylim(0, max(s['E'][-1], ref[1][-1] if ref is not None else 0.0, 1.0) * 1.15)

        # panel 3: running totals, including the cycle in progress
        run = np.concatenate([[0.0], np.cumsum(s['totals'] + [s['E'][-1]])])
        n = np.arange(len(run)) if len(s['totals']) == 0 else np.append(np.arange(len(s['totals']) + 1),
                                                                         len(s['totals']) + t[-1] / t_end)
        lrun.set_data(n, run)
        top = run[-1]
        txt = ''
        if cmp:
            ce = [cmp['cycle_E'].get(c, np.nan) for c in range(1, len(s['totals']) + 2)]
            ref_run = np.concatenate([[0.0], np.nancumsum(ce)])
            lrun_ref.set_data(np.arange(len(ref_run)), ref_run)
            top = max(top, ref_run[-1])
            done = len(s['totals'])
            if done:
                before, after = ref_run[done], run[done]
                txt = f'after {done} {"cans" if node.scenario == "cans" else "cycles"}: ' \
                      f'{after:.0f} J vs {before:.0f} J before  →  {100 * (after - before) / before:+.1f} %'
        saving.set_text(txt)
        a3.set_xlim(0, max(len(s['totals']) + 1, 3))
        a3.set_ylim(0, max(top, 1.0) * 1.15)

        this = s['E'][-1]
        extra = ''
        if cmp and slot in cmp['slot_curve']:
            b = cmp['slot_curve'][slot][1][-1]
            extra = f" · before this {'can' if node.scenario == 'cans' else 'cycle'}: {b:.1f} J"
        where = f" · slot {slot + 1}/{node.n_slots}" if node.scenario == 'cans' else ''
        status.set_text(f"cycle {s['cycle']}{where} · {s['segment'].replace('_', ' ')} · "
                        f"E_elec {this:.1f} J · P {s['P'][-1]:.1f} W{extra}")
        return []

    return fig, update


# video view: light surface; "after" is categorical slot 1 (blue), "before" a neutral reference grey
V_SURF, V_INK, V_INK2, V_MUTED, V_GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#8a8984', '#e7e6e2'
V_AFTER, V_BEFORE = '#2a78d6', '#b9b7b0'


def _video_view(plt, node, cmp):
    """Large, uncluttered view for a screen recording: saving in %, this can, running total."""
    from matplotlib.lines import Line2D
    from matplotlib.ticker import MaxNLocator
    unit = 'can' if node.scenario == 'cans' else 'cycle'
    plt.rcParams['toolbar'] = 'None'                   # no zoom/save buttons in the recording
    fig = plt.figure(figsize=(8.0, 10.0), facecolor=V_SURF)
    fig.canvas.manager.set_window_title('UR5e energy monitor' + (f' - {node.label}' if node.label else ''))
    fig.text(0.06, 0.968, 'UR5e can-packing line' if node.scenario == 'cans' else 'UR5e pick and place',
             fontsize=22, fontweight='bold', color=V_INK, va='top')
    fig.text(0.06, 0.928, 'Electrical energy of the robot, measured live', fontsize=13, color=V_INK2, va='top')
    handles = [Line2D([], [], color=V_AFTER, lw=3, label='after: energy-optimal controller')]
    if cmp:
        handles.insert(0, Line2D([], [], color=V_BEFORE, lw=4, label='before: existing controller'))
    fig.legend(handles=handles, loc='upper left', bbox_to_anchor=(0.045, 0.908), frameon=False,
               fontsize=12, labelcolor=V_INK, handlelength=1.8)

    # headline numbers
    hero = fig.text(0.06, 0.795, '', fontsize=54, fontweight='bold', color=V_INK, va='center')
    hero_cap = fig.text(0.06, 0.728, '', fontsize=13, color=V_INK2, va='center')
    fig.text(0.58, 0.822, f'this {unit}', fontsize=12, color=V_MUTED, va='center')
    now_val = fig.text(0.58, 0.786, '', fontsize=30, fontweight='bold', color=V_INK, va='center')
    now_cap = fig.text(0.58, 0.743, '', fontsize=13, color=V_INK2, va='center')

    def style(ax):
        ax.set_facecolor(V_SURF)
        for side in ('top', 'right', 'left'):
            ax.spines[side].set_visible(False)
        ax.spines['bottom'].set_color(V_GRID)
        ax.grid(axis='y', color=V_GRID, lw=0.8)
        ax.set_axisbelow(True)
        ax.tick_params(colors=V_INK2, labelsize=11, length=0)
        ax.yaxis.set_major_locator(MaxNLocator(5))

    t1 = fig.text(0.06, 0.668, '', fontsize=14, fontweight='bold', color=V_INK, va='center')
    phase = fig.text(0.94, 0.668, '', fontsize=12, color=V_MUTED, va='center', ha='right')
    ax1 = fig.add_axes([0.11, 0.395, 0.83, 0.245])
    style(ax1)
    ax1.set_xlabel('time (s)', color=V_INK2, fontsize=11)
    b1, = ax1.plot([], [], color=V_BEFORE, lw=4, solid_capstyle='round')
    a1, = ax1.plot([], [], color=V_AFTER, lw=2.5, solid_capstyle='round')
    a1dot, = ax1.plot([], [], 'o', color=V_AFTER, ms=9, mec=V_SURF, mew=2)
    b1lab = ax1.annotate('', (0, 0), xytext=(-4, 9), textcoords='offset points', ha='right', fontsize=12,
                         color=V_INK2)
    a1lab = ax1.annotate('', (0, 0), xytext=(-6, 10), textcoords='offset points', ha='right', fontsize=12,
                         fontweight='bold', color=V_INK)

    fig.text(0.06, 0.335, 'Total energy since the start (J)', fontsize=14, fontweight='bold', color=V_INK,
             va='center')
    ax2 = fig.add_axes([0.11, 0.075, 0.83, 0.225])
    style(ax2)
    ax2.set_xlabel(f'{unit}s packed' if node.scenario == 'cans' else 'cycles done', color=V_INK2, fontsize=11)
    ax2.xaxis.set_major_locator(MaxNLocator(integer=True))
    b2, = ax2.plot([], [], color=V_BEFORE, lw=3, marker='o', ms=6, mec=V_SURF, mew=1.5)
    a2, = ax2.plot([], [], color=V_AFTER, lw=2.5, solid_capstyle='round')
    a2pts, = ax2.plot([], [], 'o', color=V_AFTER, ms=8, mec=V_SURF, mew=2)
    b2lab = ax2.annotate('', (0, 0), xytext=(-4, 9), textcoords='offset points', ha='right', fontsize=12,
                         color=V_INK2)
    a2lab = ax2.annotate('', (0, 0), xytext=(8, -4), textcoords='offset points', ha='left', va='top',
                         fontsize=12, fontweight='bold', color=V_INK)
    fig.text(0.06, 0.012, 'Gazebo simulation · energy from the identified UR5e motor model '
             '(Clochiatti et al., Robotica 2024)', fontsize=9, color=V_MUTED)
    n_axis = max(node.n_slots, 3)

    def update(_):
        s = node.snapshot()
        t = s['t']
        if len(t) < 2:
            hero.set_text('…')
            hero_cap.set_text('waiting for the robot')
            return []
        cycle = max(s['cycle'], 1)
        slot = (cycle - 1) % node.n_slots
        done, E_now = len(s['totals']), float(s['E'][-1])
        ref = cmp['slot_curve'].get(slot) if cmp else None

        # headline: energy so far vs the "before" run at the same point (same cans, same time in the can)
        if cmp and ref is not None and (done or t[-1] > 5.0):
            after = float(np.sum(s['totals'])) + E_now
            before = float(np.nansum([cmp['cycle_E'].get(c, np.nan) for c in range(1, done + 1)])
                           + np.interp(t[-1], ref[0], ref[1]))
            pct = 100.0 * (after - before) / before
            hero.set_text(f'{abs(pct):.1f} %')
            hero_cap.set_text(f"{'less' if pct < 0 else 'more'} energy than the existing controller")
        elif cmp:
            hero.set_text('…')
            hero_cap.set_text('saving appears in a few seconds')
        else:
            hero.set_text(f'{float(np.sum(s["totals"])) + E_now:.0f} J')
            hero_cap.set_text('total energy so far')
        now_val.set_text(f'{E_now:.1f} J')
        now_cap.set_text(f'before: {ref[1][-1]:.1f} J' if ref is not None else '')

        # this can
        t1.set_text(f'Energy used for {unit} {cycle} (J)')
        phase.set_text(s['segment'].replace('_', ' '))
        a1.set_data(t, s['E'])
        a1dot.set_data([t[-1]], [E_now])
        a1lab.xy = (t[-1], E_now)
        a1lab.set_text(f'{E_now:.1f} J')
        t_end = max(t[-1], 1.0)
        top = E_now
        if ref is not None:
            b1.set_data(*ref)
            b1lab.xy = (ref[0][-1], ref[1][-1])
            b1lab.set_text(f'before {ref[1][-1]:.1f} J')
            t_end, top = max(t_end, ref[0][-1]), max(top, ref[1][-1])
        ax1.set_xlim(0, t_end * 1.02)
        ax1.set_ylim(0, max(top, 1.0) * 1.25)

        # running total, including the can in progress
        run = np.concatenate([[0.0], np.cumsum(s['totals'])])
        frac = min(t[-1] / t_end, 1.0)
        a2.set_data(np.append(np.arange(done + 1), done + frac), np.append(run, run[-1] + E_now))
        a2pts.set_data(np.arange(1, done + 1), run[1:])
        a2lab.xy = (done + frac, run[-1] + E_now)
        near_end = done + frac > max(n_axis, done + 1) - 1.5          # keep the label inside the plot
        a2lab.xyann, _ = ((-6, -10), a2lab.set_ha('right')) if near_end else ((8, -4), a2lab.set_ha('left'))
        a2lab.set_text(f'{run[-1] + E_now:.0f} J')
        top, n_max = run[-1] + E_now, max(n_axis, done + 1)
        if cmp:
            ce = [cmp['cycle_E'].get(c, np.nan) for c in range(1, n_max + 1)]
            ref_run = np.concatenate([[0.0], np.nancumsum(ce)])
            b2.set_data(np.arange(len(ref_run)), ref_run)
            b2lab.xy = (len(ref_run) - 1, ref_run[-1])
            b2lab.set_text(f'before {ref_run[-1]:.0f} J')
            top = max(top, ref_run[-1])
        ax2.set_xlim(0, n_max + 0.3)
        ax2.set_ylim(0, max(top, 1.0) * 1.18)
        return []

    return fig, update


def main():
    plt = _plt('TkAgg')
    from matplotlib.animation import FuncAnimation

    # Ctrl+C closes the window (the animation timer gives Python the chance to run the
    # handler); rclpy's own handler would only stop the ROS thread and leave the window open
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: plt.close('all'))
    node = EnergyMonitor()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    cmp = node.compare

    view = _video_view if node.style == 'video' else _detail_view
    fig, update = view(plt, node, cmp)
    anim = FuncAnimation(fig, update, interval=150 if node.style == 'video' else 250, cache_frame_data=False)  # noqa: F841 (keep a reference)
    try:
        plt.show()
    finally:
        executor.shutdown()
        spinner.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
