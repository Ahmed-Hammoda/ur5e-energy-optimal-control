"""Energy validation report for logged pick-and-place runs.

    ros2 run ur5e_pp_control validate                    # every run in ~/ur5e_pp_ws/results
    ros2 run ur5e_pp_control validate --run nominal_scaled
    ros2 run ur5e_pp_control validate --dir some/dir --cycle 5

Reads <tag>.csv / <tag>.json written by pp_runner and writes, into
<dir>/validation/: summary.md, summary.json and per run
<tag>_cycle.png, <tag>_phases.png, <tag>_joints.png, <tag>_cycles.png.

Energy model (electrical energy per cycle), from the identified UR5e model of
Clochiatti et al., "Electro-mechanical modeling and identification of the
UR5 e-series robot", Robotica 42 (2024), Eq. (23):

  P_i   = (kb/kt) * tau_i * qd_i  +  tau_i^2 / (N * km_i)^2        [W]
          mechanical (motor side)    copper loss R*I^2
  P     = sum_i P_i                         joints share one DC bus
  E_elec = integral of max(0, P) dt         braking energy is burnt in the
                                            brake resistor, not recovered
  N  = 100 (harmonic drive), kt/kb = 0.89,
  km = 0.27426 Nm/sqrt(W) (size-3 motors, joints 1-3),
       0.06632 Nm/sqrt(W) (size-1 motors, joints 4-6).

Joint friction is zero in the simulation, so the motor torque is tau/N and
the motor speed N*qd. Also reported: E_mech = integral of max(0, sum tau*qd),
E_cu = integral of sum tau^2/(N km)^2, peak and average power.

tau is the inverse-dynamics torque M(q) qdd + C(q, qd) qd + g(q) of the logged
motion (dynamics.py): Gazebo's own joint torque contains only g(q), so it would
hide the cost of accelerating. Gazebo's torque is kept as a cross-check of the
gravity part and of the energy (E_elec with Gazebo tau).
"""
import argparse
import glob
import json
import os

import numpy as np

from .dynamics import BOX_SIZE, GRIPPER_MASS, Dynamics, box_mass_per_sample

GEAR = 100.0
KM = np.array([0.27426] * 3 + [0.06632] * 3)       # Nm/sqrt(W)
KB_OVER_KT = 1.0 / 0.89
C_CU = 1.0 / (GEAR * KM) ** 2                      # W per (N.m)^2 of joint torque
JOINTS = ['base', 'shoulder', 'elbow', 'wrist 1', 'wrist 2', 'wrist 3']
TAU_MAX = np.array([150.0] * 3 + [28.0] * 3)       # UR5e joint torque limits

# acceptance rules for a fair comparison (step 2 definition)
REF_CYCLE_TIME = 16.02    # s, baseline cycle time
PLACE_TOL_MM = 5.0
FIRST_CYCLE = 2           # cycle 1 starts from rest and is not counted

# chart styling: categorical slots in fixed order, recessive axes
ELEC, MECH, CU = '#2a78d6', '#eb6834', '#1baf7a'
INK, MUTED, GRID, SURFACE = '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'
PHASE_FILL = ['#f1f0ec', '#fcfcfb']


# ------------------------------------------------------------------ model
def power(tau, qd):
    """Per-joint electrical, mechanical and copper power, shape (N, 6)."""
    mech = tau * qd
    cu = C_CU * tau ** 2
    return KB_OVER_KT * mech + cu, mech, cu


def integrate(y, t):
    return float(np.sum(0.5 * (y[1:] + y[:-1]) * np.diff(t)))


def cumulative(y, t):
    return np.concatenate([[0.0], np.cumsum(0.5 * (y[1:] + y[:-1]) * np.diff(t))])


def cycle_metrics(t, tau, qd):
    p_el, p_mech, p_cu = power(tau, qd)
    P = np.maximum(p_el.sum(1), 0.0)
    E = integrate(P, t)
    return {
        'E_elec_J': E,
        'E_mech_J': integrate(np.maximum(p_mech.sum(1), 0.0), t),
        'E_cu_J': integrate(p_cu.sum(1), t),
        'P_peak_W': float(P.max()),
        'P_avg_W': E / (t[-1] - t[0]),
        'E_joint_J': [integrate(np.maximum(p_el[:, j], 0.0), t) for j in range(6)],
        'E_joint_cu_J': [integrate(p_cu[:, j], t) for j in range(6)],
        'tau_peak_ratio': float(np.max(np.abs(tau) / TAU_MAX)),
    }


# ------------------------------------------------------------------ data
def csv_of(base):
    """Log file of a run: <base>.csv, or the compressed <base>.csv.xz (numpy reads it directly)."""
    return base + '.csv' if os.path.exists(base + '.csv') or not os.path.exists(base + '.csv.xz') \
        else base + '.csv.xz'


def load_run(json_path, dyn):
    """Log of one run; run['tau'] is the model torque, run['tau_gz'] Gazebo's."""
    with open(json_path) as f:
        meta = json.load(f)
    d = np.genfromtxt(csv_of(json_path[:-5]), delimiter=',', names=True, dtype=None, encoding=None)
    run = {'t': np.asarray(d['t'], float), 'cycle': np.asarray(d['cycle'], int),
           'segment': np.asarray(d['segment'], str), 'tcp_err': np.asarray(d['tcp_err'], float)}
    run['q'] = np.stack([np.asarray(d[f'q_{j}'], float) for j in range(6)], 1)
    run['qd'] = np.stack([np.asarray(d[f'qd_{j}'], float) for j in range(6)], 1)
    run['tau_gz'] = np.stack([np.asarray(d[f'tau_{j}'], float) for j in range(6)], 1)
    dyn = dyn(meta.get('object_height', BOX_SIZE), meta.get('object_radius'))
    box = box_mass_per_sample(run['segment'], meta.get('payload_kg', GRIPPER_MASS + 0.5) - GRIPPER_MASS)
    run['tau'] = np.zeros_like(run['q'])
    run['tau_g'] = np.zeros_like(run['q'])
    for c in np.unique(run['cycle']):                  # t restarts every cycle: differentiate per cycle
        m = run['cycle'] == c
        if m.sum() < 2:
            continue
        qdd = np.gradient(run['qd'][m], run['t'][m], axis=0)
        run['tau'][m] = dyn.tau(run['q'][m], run['qd'][m], qdd, box[m])
        run['tau_g'][m] = dyn.gravity(run['q'][m], box[m])
    return meta, run


def select(run, c):
    m = run['cycle'] == c
    return {k: v[m] for k, v in run.items()}


def phases(seg, t):
    """Contiguous (name, t_start, t_end) blocks of the task segments."""
    out, start = [], 0
    for k in range(1, len(seg) + 1):
        if k == len(seg) or seg[k] != seg[start]:
            out.append((seg[start], t[start], t[min(k, len(t) - 1)]))
            start = k
    return [p for p in out if p[0] != 'start']


def evaluate(meta, run):
    cycles = []
    # cans: every cycle starts and ends at rest at the pick, and a carton needs all its cans
    first = 1 if meta.get('scenario') == 'cans' else FIRST_CYCLE
    takt = meta.get('takt_s', REF_CYCLE_TIME)
    for c in meta['cycles']:
        if c['cycle'] < first or c['fault']:
            continue
        r = select(run, c['cycle'])
        gz = cycle_metrics(r['t'], r['tau_gz'], r['qd'])
        cycles.append({'cycle': c['cycle'], 'cycle_time_s': c['cycle_time_s'],
                       'max_tcp_err_mm': c['max_tcp_err_mm'], 'placement_err_mm': c['placement_err_mm'],
                       **cycle_metrics(r['t'], r['tau'], r['qd']),
                       'E_elec_gazebo_J': gz['E_elec_J'],
                       'gravity_rms_err_Nm': float(np.sqrt(np.mean((r['tau_gz'] - r['tau_g']) ** 2)))})
    if not cycles:
        raise SystemExit(f"{meta['tag']}: no complete cycles after cycle {first - 1}")
    arr = lambda k: np.array([c[k] for c in cycles], float)
    s = {k: {'mean': float(arr(k).mean()), 'std': float(arr(k).std())}
         for k in ('E_elec_J', 'E_mech_J', 'E_cu_J', 'P_peak_W', 'P_avg_W', 'cycle_time_s', 'E_elec_gazebo_J')}
    s['E_joint_J'] = np.mean([c['E_joint_J'] for c in cycles], 0).tolist()
    s['E_joint_cu_J'] = np.mean([c['E_joint_cu_J'] for c in cycles], 0).tolist()
    s['max_tcp_err_mm'] = float(arr('max_tcp_err_mm').max())
    s['placement_max_mm'] = float(arr('placement_err_mm').max())
    s['tau_peak_ratio'] = float(arr('tau_peak_ratio').max())
    s['gravity_rms_err_Nm'] = float(arr('gravity_rms_err_Nm').max())
    faults = sum(c['fault'] for c in meta['cycles'])
    s['checks'] = {
        'no fault': faults == 0,
        f'cycle time <= {takt:.2f} s': s['cycle_time_s']['mean'] <= takt + 0.05,
        f'placement error <= {PLACE_TOL_MM:.0f} mm': s['placement_max_mm'] <= PLACE_TOL_MM,
        'torque within limits': s['tau_peak_ratio'] <= 1.0,
    }
    if meta.get('scenario') == 'cans':               # energy of every full carton (12 cans)
        per = {}
        for c in cycles:
            per.setdefault((c['cycle'] - 1) // 12 + 1, []).append(c['E_elec_J'])
        s['carton_E_J'] = {n: float(sum(v)) for n, v in per.items() if len(v) == 12}
    return {'tag': meta['tag'], 'controller': meta.get('controller', meta['mode']),
            'cycles_used': len(cycles), 'cycles': cycles, 'summary': s}


# ------------------------------------------------------------------ figures
def _plt(backend='Agg'):
    import matplotlib
    matplotlib.use(backend)
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        'font.size': 10.5, 'axes.edgecolor': '#9a9993', 'axes.labelcolor': INK, 'xtick.color': MUTED,
        'ytick.color': MUTED, 'axes.titlelocation': 'left', 'axes.titleweight': 'bold',
        'axes.titlesize': 12, 'axes.titlecolor': INK, 'axes.spines.top': False, 'axes.spines.right': False,
        'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.8, 'axes.axisbelow': True,
        'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'legend.frameon': False,
        'lines.linewidth': 2.0, 'lines.solid_capstyle': 'round'})
    return plt


def _shade_phases(ax, blocks):
    for k, (_, t0, t1) in enumerate(blocks):
        ax.axvspan(t0, t1, color=PHASE_FILL[k % 2], lw=0, zorder=0)


def plot_cycle(res, run, c, out):
    """Power and cumulative energy over one cycle: the time / energy curve."""
    plt = _plt()
    r = select(run, c)
    t = r['t']
    p_el, p_mech, p_cu = power(r['tau'], r['qd'])
    P = np.maximum(p_el.sum(1), 0.0)
    Pm = np.maximum(p_mech.sum(1), 0.0)
    Pc = p_cu.sum(1)
    blocks = phases(r['segment'], t)
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(11.5, 7.4), dpi=170, sharex=True,
                                 gridspec_kw={'height_ratios': [1, 1], 'hspace': 0.12})
    for ax in (a1, a2):
        _shade_phases(ax, blocks)
    for k, (name, t0, t1) in enumerate(blocks):        # two rows so short phases don't collide
        a1.text((t0 + t1) / 2, 1.0 + 0.045 * (k % 2), name.replace('_', ' '), transform=a1.get_xaxis_transform(),
                ha='center', va='bottom', fontsize=8.5, color=MUTED)
    a1.plot(t, P, color=ELEC, label='electrical  max(0, Σ Pᵢ)')
    a1.plot(t, Pm, color=MECH, label='mechanical  max(0, Σ τᵢq̇ᵢ)')
    a1.plot(t, Pc, color=CU, label='copper loss  Σ τᵢ²/(N kₘ)²')
    a1.set_ylabel('power (W)')
    a1.set_ylim(bottom=0)
    a1.set_title(f"{res['tag']}: power during cycle {c}", pad=34)
    a1.legend(loc='upper right', fontsize=9.5, labelcolor=INK)

    curves = [(cumulative(P, t), ELEC, 'E_elec'), (cumulative(Pm, t), MECH, 'E_mech'),
              (cumulative(Pc, t), CU, 'E_cu')]
    for y, col, name in curves:
        a2.plot(t, y, color=col)
        a2.annotate(f'{name} = {y[-1]:.1f} J', (t[-1], y[-1]), xytext=(6, 0), textcoords='offset points',
                    va='center', fontsize=9.5, color=INK)
        a2.plot(t[-1], y[-1], 'o', ms=6, color=col, mec=SURFACE, mew=2)
    a2.set_ylabel('cumulative energy (J)')
    a2.set_xlabel('time in cycle (s)')
    a2.set_ylim(bottom=0)
    a2.set_xlim(t[0], t[-1] * 1.1)
    a2.set_title('energy used so far in the cycle', pad=8)
    fig.savefig(out, bbox_inches='tight')
    plt.close(fig)


def _grouped_bars(ax, labels, e_elec, e_cu):
    """E_elec and E_cu side by side. While lowering, the bus gets energy back,
    so E_elec can be smaller than the copper loss alone."""
    x = np.arange(len(labels))
    w = 0.36
    for dx, vals, col, name in ((-w / 2, e_elec, ELEC, 'E_elec (total electrical)'),
                                (w / 2, e_cu, CU, 'E_cu (copper loss part)')):
        bars = ax.bar(x + dx, vals, w * 0.92, color=col, label=name, edgecolor=SURFACE, lw=2)
        ax.bar_label(bars, fmt='%.1f', fontsize=9, color=INK, padding=2)
    ax.set_xticks(x, labels)
    ax.set_ylim(0, max(max(e_elec), max(e_cu)) * 1.2)
    ax.grid(axis='x', visible=False)
    ax.legend(loc='upper right', fontsize=9.5, labelcolor=INK)


def plot_phases(res, run, c, out):
    """Electrical and copper-loss energy of each task phase."""
    plt = _plt()
    r = select(run, c)
    t = r['t']
    p_el, _, p_cu = power(r['tau'], r['qd'])
    P = np.maximum(p_el.sum(1), 0.0)
    labels, e_elec, e_cu = [], [], []
    for name, t0, t1 in phases(r['segment'], t):
        m = (t >= t0) & (t <= t1)
        labels.append(f"{name.replace('_', ' ')}\n{t1 - t0:.2f} s")
        e_elec.append(integrate(P[m], t[m]))
        e_cu.append(integrate(p_cu.sum(1)[m], t[m]))
    fig, ax = plt.subplots(figsize=(11.5, 4.6), dpi=170)
    _grouped_bars(ax, labels, e_elec, e_cu)
    ax.set_ylabel('energy (J)')
    ax.set_title(f"{res['tag']}: energy per task phase (cycle {c})")
    fig.savefig(out, bbox_inches='tight')
    plt.close(fig)


def plot_joints(res, out):
    """Mean electrical and copper-loss energy per joint (each joint on its own,
    so the sum is larger than E_elec, where the joints share the bus)."""
    plt = _plt()
    s = res['summary']
    fig, ax = plt.subplots(figsize=(11.5, 4.4), dpi=170)
    _grouped_bars(ax, JOINTS, s['E_joint_J'], s['E_joint_cu_J'])
    ax.set_ylabel('energy per cycle (J)')
    ax.set_title(f"{res['tag']}: energy per joint (mean of {res['cycles_used']} cycles)")
    fig.savefig(out, bbox_inches='tight')
    plt.close(fig)


def plot_cycles(res, out):
    """E_elec of every counted cycle: shows the run is repeatable."""
    plt = _plt()
    cyc = res['cycles']
    n = [c['cycle'] for c in cyc]
    e = np.array([c['E_elec_J'] for c in cyc])
    m = res['summary']['E_elec_J']
    fig, ax = plt.subplots(figsize=(11.5, 3.8), dpi=170)
    ax.axhspan(m['mean'] - m['std'], m['mean'] + m['std'], color=GRID, lw=0)
    ax.hlines(m['mean'], n[0], n[-1], color=MUTED, lw=1.2, ls='--')
    ax.plot(n, e, color=ELEC, marker='o', ms=6, mec=SURFACE, mew=2)
    ax.text(n[-1] + 0.6, m['mean'], f"mean {m['mean']:.2f} ± {m['std']:.2f} J", va='center',
            fontsize=9.5, color=INK)
    ax.set_xlim(n[0] - 0.5, n[-1] + 7)
    pad = max(3 * m['std'], 0.02 * m['mean'])
    ax.set_ylim(m['mean'] - pad, m['mean'] + pad)
    ax.set_xlabel('cycle')
    ax.set_ylabel('E_elec (J)')
    ax.set_title(f"{res['tag']}: electrical energy of each cycle")
    fig.savefig(out, bbox_inches='tight')
    plt.close(fig)


# ------------------------------------------------------------------ report
def report(results):
    f = lambda d: f"{d['mean']:.2f} ± {d['std']:.2f}"
    lines = ['# Energy validation', '',
             'Electrical energy model of Clochiatti et al. (Robotica 2024), Eq. (23), with the '
             'inverse-dynamics torque of the logged motion; '
             f'cycles {FIRST_CYCLE} onward, mean ± std.', '',
             '| run | cycles | E_elec (J) | E_mech (J) | E_cu (J) | P_peak (W) | P_avg (W) | cycle time (s) |'
             ' max TCP err (mm) | placement (mm) |', '|---|---|---|---|---|---|---|---|---|---|']
    for r in results:
        s = r['summary']
        lines.append(f"| {r['tag']} | {r['cycles_used']} | {f(s['E_elec_J'])} | {f(s['E_mech_J'])} | "
                     f"{f(s['E_cu_J'])} | {f(s['P_peak_W'])} | {f(s['P_avg_W'])} | {f(s['cycle_time_s'])} | "
                     f"{s['max_tcp_err_mm']:.2f} | {s['placement_max_mm']:.2f} |")
    lines += ['', '## Energy per joint (J per cycle)', '', '| run | ' + ' | '.join(JOINTS) + ' |',
              '|---|' + '---|' * 6]
    for r in results:
        lines.append(f"| {r['tag']} | " + ' | '.join(f'{v:.2f}' for v in r['summary']['E_joint_J']) + ' |')
    if any('carton_E_J' in r['summary'] for r in results):
        lines += ['', '## Energy per full carton (12 cans)', '', '| run | ' + ' | '.join(
            f'carton {n}' for n in range(1, 1 + max(len(r['summary'].get('carton_E_J', {})) for r in results))) + ' |',
            '|---|' + '---|' * max(len(r['summary'].get('carton_E_J', {})) for r in results)]
        for r in results:
            if 'carton_E_J' in r['summary']:
                lines.append(f"| {r['tag']} | " + ' | '.join(f'{v:.1f} J' for v in r['summary']['carton_E_J'].values()) + ' |')
    lines += ['', '## Cross-check with Gazebo torque (gravity part only)', '',
              '| run | E_elec, model τ (J) | E_elec, Gazebo τ (J) | Gazebo τ − model g(q), rms (N·m) |',
              '|---|---|---|---|']
    for r in results:
        s = r['summary']
        lines.append(f"| {r['tag']} | {f(s['E_elec_J'])} | {f(s['E_elec_gazebo_J'])} | "
                     f"{s['gravity_rms_err_Nm']:.3f} |")
    lines += ['', '## Fair-comparison checks', '']
    for r in results:
        checks = ', '.join(f"{k}: {'pass' if ok else 'FAIL'}" for k, ok in r['summary']['checks'].items())
        lines.append(f"- **{r['tag']}**: {checks}")
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dir', default=os.path.expanduser('~/ur5e_pp_ws/results'))
    ap.add_argument('--run', action='append', help='run tag (repeatable); default: every run in --dir')
    ap.add_argument('--cycle', type=int, default=FIRST_CYCLE, help='cycle shown in the time plots')
    args = ap.parse_args()

    paths = ([os.path.join(args.dir, r + '.json') for r in args.run] if args.run
             else sorted(glob.glob(os.path.join(args.dir, '*.json'))))
    out_dir = os.path.join(args.dir, 'validation')
    os.makedirs(out_dir, exist_ok=True)
    models = {}

    def dyn(height, radius):                           # one dynamics model per carried object
        if (height, radius) not in models:
            models[height, radius] = Dynamics(obj_height=height, obj_radius=radius)
        return models[height, radius]
    results = []
    for jp in paths:
        if not os.path.exists(csv_of(jp[:-5])):
            continue
        meta, run = load_run(jp, dyn)
        res = evaluate(meta, run)
        c = args.cycle if any(x['cycle'] == args.cycle for x in res['cycles']) else res['cycles'][0]['cycle']
        base = os.path.join(out_dir, res['tag'])
        plot_cycle(res, run, c, base + '_cycle.png')
        plot_phases(res, run, c, base + '_phases.png')
        plot_joints(res, base + '_joints.png')
        plot_cycles(res, base + '_cycles.png')
        results.append(res)
    if not results:
        raise SystemExit(f'No runs found in {args.dir}')

    text = report(results)
    with open(os.path.join(out_dir, 'summary.md'), 'w') as f:
        f.write(text + '\n')
    with open(os.path.join(out_dir, 'summary.json'), 'w') as f:
        json.dump([{k: v for k, v in r.items()} for r in results], f, indent=2)
    print(text)
    print(f'\nFigures and tables written to {out_dir}')


if __name__ == '__main__':
    main()
