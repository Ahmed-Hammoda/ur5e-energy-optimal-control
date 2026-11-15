"""Energy-optimal pick-and-place trajectory (the new controller, replaces Layer 1).

The task (pick, place, object mass and height, takt time) is the input. The
cycle always lasts exactly the takt time (fixed production rhythm: one part
every 16.02 s, like the baseline): when the motion is shorter the robot waits,
and waiting costs energy too (holding the arm against gravity). The motion is
described by a few "knobs"; the optimizer picks the knob values with the lowest
predicted electrical energy E_elec per cycle (validate.py model, torque from
dynamics.py):

  lift            lift height above pick / place [m]
  tr_dx, tr_dz    via-point of the transfer (box carried), offset from the
                  midpoint of the two lift tops [m]; dx < 0 is towards the base
  rt_dx, rt_dz    via-point of the return (empty gripper), same meaning
  wait_dx/dz      where the robot waits for the next part: offset from the
                  top of the pick lift, dx < 0 towards the base [m]
  T_*             duration of each move [s]; the wait is what is left of the takt

Cycle: grasp, lift, transfer, lower, release, lift_back, return (to the wait
pose), wait, approach (to the top of the pick lift), lower_back.

Path builder: IK only at the key poses (pick, lift middles and tops, place,
via-points, wait pose); between them every joint moves along a smooth spline in
joint space, timed by a minimum-jerk law, so velocity and acceleration are zero
at every stop and q, qd, qdd are analytic (no IK per sample, no numerical
differentiation). Forward kinematics checks the tool path: vertical lift /
lower, the box and the gripper above the table and the boxes, the tool away
from the robot's own base, TCP speed limit.
"""
from dataclasses import dataclass, field

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import minimize

from .dynamics import ATTACHED, BOX_SIZE, Dynamics
from .kinematics import R_TOOL_DOWN, fk_all, ik
from .trajectory import IK_SEED, LIFT_HEIGHT, PICK, PLACE, CycleReference
from .validate import power

TABLE_TOP = 0.08
GRIPPER_LEN = 0.10
V_TCP_MAX = 0.20          # m/s, the project's TCP speed limit (trajectory.py)
CLEARANCE = 0.01          # m, object / gripper above table and boxes while moving
LATERAL_TOL = 0.003       # m, allowed sideways drift of lift / lower (vertical approach)
GRIPPER_RADIUS = 0.035    # m, gripper cylinder
MIN_RADIUS = 0.25         # m, tool kept this far from the base axis (simple self-collision rule)
DWELL = 0.5               # s, grasp and release pauses
TAKT = 16.02              # s, baseline cycle time = production rhythm
KNOBS = ['lift', 'tr_dx', 'tr_dz', 'rt_dx', 'rt_dz', 'wait_dx', 'wait_dz',
         'T_lift', 'T_transfer', 'T_lower', 'T_lift_back', 'T_return', 'T_approach', 'T_lower_back']
LO = np.array([0.03, -0.25, -0.10, -0.25, -0.10, -0.25, 0.00] + [0.3] * 5 + [0.0, 0.3])
HI = np.array([0.25, 0.15, 0.20, 0.15, 0.20, 0.10, 0.40] + [8.0] * 7)


@dataclass
class Task:
    pick: np.ndarray = field(default_factory=PICK.copy)        # tool0 at the pick
    place: np.ndarray = field(default_factory=PLACE.copy)      # tool0 at the release
    box_mass: float = 0.5
    box_height: float = BOX_SIZE
    obj_radius: float = None        # None: the 7 cm cube; else a can of this radius
    takt: float = TAKT
    # boxes (xmin, xmax, ymin, ymax, ztop) to pass over: carton walls, packed cans, end stop
    obstacles: list = field(default_factory=list)

    def footprint(self):
        return BOX_SIZE / 2 * np.sqrt(2) if self.obj_radius is None else self.obj_radius


def baseline_knobs(task=None, speed=0.15, lift=LIFT_HEIGHT):
    """Straight moves, min-jerk timing at `speed` peak, no wait detour (the baseline at 0.15)."""
    task = task or Task()
    T = lambda L: 1.875 * L / speed
    span = np.linalg.norm(task.place - task.pick)
    return dict(lift=lift, tr_dx=0.0, tr_dz=0.0, rt_dx=0.0, rt_dz=0.0, wait_dx=0.0, wait_dz=0.0,
                T_lift=T(lift), T_transfer=T(span), T_lower=T(lift), T_lift_back=T(lift),
                T_return=T(span), T_approach=0.0, T_lower_back=T(lift))


def _quant(T):
    return round(T / 0.01) * 0.01          # whole 10 ms steps: the runner's reference is uniform


class Planner:
    def __init__(self, task=None, dyn=None):
        self.task = task or Task()
        self.dyn = dyn or Dynamics(obj_height=self.task.box_height, obj_radius=self.task.obj_radius)
        self.n_eval = 0

    # -------------------------------------------------------------- path
    def key_poses(self, k):
        """Joint angles at the key poses, or None if a pose is not reachable."""
        tk = self.task
        up = np.array([0.0, 0.0, k['lift']])
        top_pick, top_place = tk.pick + up, tk.place + up
        mid = 0.5 * (top_pick + top_place)
        radial = lambda p: np.append(p[:2] / np.linalg.norm(p[:2]), 0.0)   # towards / away from the base
        points = {'pick': tk.pick, 'mid_pick': tk.pick + up / 2, 'top_pick': top_pick,
                  'via_tr': mid + k['tr_dx'] * radial(mid) + [0, 0, k['tr_dz']],
                  'top_place': top_place, 'mid_place': tk.place + up / 2, 'place': tk.place,
                  'via_rt': mid + k['rt_dx'] * radial(mid) + [0, 0, k['rt_dz']],
                  'wait': top_pick + k['wait_dx'] * radial(top_pick) + [0, 0, k['wait_dz']]}
        q, seed = {}, IK_SEED
        for name in ('pick', 'mid_pick', 'top_pick', 'via_tr', 'top_place', 'mid_place', 'place',
                     'via_rt', 'wait'):
            sol, ok = ik(points[name], R_TOOL_DOWN, seed)
            if not ok:
                return None
            q[name], seed = sol, sol
        return q

    def legs(self, k):
        Q = self.key_poses(k)
        if Q is None:
            return None
        moves = [('grasp', [Q['pick']], DWELL),
                 ('lift', [Q['pick'], Q['mid_pick'], Q['top_pick']], k['T_lift']),
                 ('transfer', [Q['top_pick'], Q['via_tr'], Q['top_place']], k['T_transfer']),
                 ('lower', [Q['top_place'], Q['mid_place'], Q['place']], k['T_lower']),
                 ('release', [Q['place']], DWELL),
                 ('lift_back', [Q['place'], Q['mid_place'], Q['top_place']], k['T_lift_back']),
                 ('return', [Q['top_place'], Q['via_rt'], Q['wait']], k['T_return']),
                 ('wait', [Q['wait']], 0.0),
                 ('approach', [Q['wait'], Q['top_pick']], k['T_approach']),
                 ('lower_back', [Q['top_pick'], Q['mid_pick'], Q['pick']], k['T_lower_back'])]
        moves = [(n, kn, _quant(T)) for n, kn, T in moves]
        t_wait = _quant(self.task.takt - sum(T for _, _, T in moves))
        return [(n, kn, t_wait if n == 'wait' else T) for n, kn, T in moves], t_wait

    def build(self, k, dt):
        """Sampled cycle for knob dict k: t, q, qd, qdd, segment, t_wait (or None if unreachable)."""
        out = self.legs(k)
        if out is None:
            return None
        legs, t_wait = out
        q0 = legs[0][1][0]
        ts, qs, qds, qdds, segs = [np.zeros(1)], [q0[None]], [np.zeros((1, 6))], [np.zeros((1, 6))], ['start']
        t0 = 0.0
        for name, knots, T in legs:
            if T < 0.005:
                continue                                     # zero-length leg (no wait, no approach)
            n = max(int(round(T / dt)), 1)
            tau = np.arange(1, n + 1) / n
            if len(knots) == 1:                              # dwell
                q, qd, qdd = np.repeat(knots[0][None], n, 0), np.zeros((n, 6)), np.zeros((n, 6))
            else:
                # path q(s), s in [0, 1], through the knots; timing s(t) minimum jerk
                spline = CubicSpline(np.linspace(0, 1, len(knots)), np.array(knots), bc_type='natural')
                s = 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5
                sd = (30 * tau ** 2 - 60 * tau ** 3 + 30 * tau ** 4) / T
                sdd = (60 * tau - 180 * tau ** 2 + 120 * tau ** 3) / T ** 2
                q, dq, ddq = spline(s), spline(s, 1), spline(s, 2)
                qd = dq * sd[:, None]
                qdd = ddq * sd[:, None] ** 2 + dq * sdd[:, None]
            ts.append(t0 + tau * T)
            qs.append(q); qds.append(qd); qdds.append(qdd); segs += [name] * n
            t0 += T
        return dict(t=np.concatenate(ts), q=np.vstack(qs), qd=np.vstack(qds), qdd=np.vstack(qdds),
                    segment=np.array(segs), t_wait=t_wait)

    # -------------------------------------------------------------- cost
    def evaluate(self, k, dt=0.05):
        """(E_elec, penalty, details) for knob dict k. Penalty 0 means every rule holds."""
        self.n_eval += 1
        tk = self.task
        path = self.build(k, dt)
        if path is None:
            return np.inf, np.inf, {}
        t, q, qd, qdd, seg = path['t'], path['q'], path['qd'], path['qdd'], path['segment']
        carried = np.isin(seg, ATTACHED)
        tau = self.dyn.tau(q, qd, qdd, np.where(carried, tk.box_mass, 0.0))
        P = np.maximum(power(tau, qd)[0].sum(1), 0.0)
        E = float(np.sum(0.5 * (P[1:] + P[:-1]) * np.diff(t)))

        tool = np.array([fk_all(qi)[-1][:3, 3] for qi in q])
        pen = [max(-path['t_wait'], 0.0) * 10]                                  # motion fits in the takt
        v = np.linalg.norm(np.diff(tool, axis=0), axis=1) / np.diff(t)
        pen.append(max(v.max() - V_TCP_MAX, 0.0) * 10)                          # TCP speed
        obj_bottom = tool[:, 2] - GRIPPER_LEN - tk.box_height                  # carried object
        tip = tool[:, 2] - GRIPPER_LEN                                          # empty gripper
        tr = seg == 'transfer'
        pen.append(max(TABLE_TOP + CLEARANCE - obj_bottom[tr].min(), 0.0) * 100)
        empty = np.isin(seg, ['return', 'wait', 'approach'])                   # over the next object at pick
        if empty.any():
            pen.append(max(tk.pick[2] - GRIPPER_LEN + CLEARANCE - tip[empty].min(), 0.0) * 100)
        for xmin, xmax, ymin, ymax, ztop in tk.obstacles:                       # pass over walls and packed cans
            for m, bottom, r in ((tr, obj_bottom, tk.footprint()), (empty, tip, GRIPPER_RADIUS)):
                over = m & (tool[:, 0] > xmin - r) & (tool[:, 0] < xmax + r) \
                    & (tool[:, 1] > ymin - r) & (tool[:, 1] < ymax + r)
                if over.any():
                    pen.append(max(ztop + CLEARANCE - bottom[over].min(), 0.0) * 100)
        for name, xy in (('lift', tk.pick), ('lower_back', tk.pick), ('lower', tk.place), ('lift_back', tk.place)):
            m = seg == name
            drift = np.linalg.norm(tool[m, :2] - xy[:2], axis=1).max()
            pen.append(max(drift - LATERAL_TOL, 0.0) * 100)                     # vertical approach
        pen.append(max(MIN_RADIUS - np.linalg.norm(tool[:, :2], axis=1).min(), 0.0) * 100)
        pen.append(max(np.max(np.abs(tau) / np.array([150.0] * 3 + [28.0] * 3)) - 1.0, 0.0) * 10)
        return E, float(sum(pen)), dict(t_cycle=float(t[-1]), t_wait=float(path['t_wait']),
                                        v_max=float(v.max()), obj_clear=float(obj_bottom[tr].min() - TABLE_TOP))

    # -------------------------------------------------------------- search
    def optimize(self, starts=None, max_eval=1500, dt=0.05, callback=None, n_refine=3):
        """Minimise E_elec under the rules. First every start motion and a coarse grid
        are rated (~20 evaluations, so a good motion is known after a fraction of a
        second), then Nelder-Mead (bounded) refines from the n_refine best of them.
        callback(knobs, E) is called whenever a better valid motion is found."""
        starts = starts or default_starts(self.task) + coarse_grid(self.task)
        best = {'E': np.inf}

        def f(x):
            xc = np.clip(x, LO, HI)
            k = dict(zip(KNOBS, xc))
            E, pen, _ = self.evaluate(k, dt)
            if pen == 0 and E < best['E']:
                best.update(E=E, k=k)
                if callback:
                    callback(k, E)
            return E + 1000 * pen + 1000 * float(np.sum(np.abs(x - xc)))

        x0s = [np.array([k0[n] for n in KNOBS]) for k0 in starts]
        rated = sorted(zip([f(x) for x in x0s], range(len(x0s))))       # best first
        for _, i in rated[:n_refine]:
            minimize(f, x0s[i], method='Nelder-Mead',
                     options=dict(maxfev=max_eval, xatol=1e-4, fatol=1e-3, adaptive=True))
        return best.get('k'), best['E']

    def reference(self, k, dt=0.01):
        """CycleReference (what the runner sends to the JTC) for knob dict k."""
        path = self.build(k, dt)
        tool = np.array([fk_all(qi)[-1][:3, 3] for qi in path['q']])
        seg = list(path['segment'])
        return CycleReference(dt=dt, t=path['t'], q=path['q'], qd=path['qd'], p=tool, segment=seg,
                              place_index=max(i for i, s in enumerate(seg) if s == 'release'),
                              pick_index=len(seg) - 1)


def default_starts(task=None):
    """Starting motions for the search: the baseline, a fast low-lift motion, and
    the same with the wait moved in towards the base and up."""
    fast = baseline_knobs(task, speed=0.19, lift=0.05)
    folded = dict(fast, wait_dx=-0.15, wait_dz=0.15, T_approach=1.5)
    high = dict(fast, wait_dx=-0.20, wait_dz=0.30, T_approach=2.5)
    return [baseline_knobs(task), fast, folded, high]


def coarse_grid(task=None):
    """Rough grid over the knobs that matter most: lift height, speed, wait pose."""
    out = []
    for lift in (0.03, 0.05, 0.08, 0.12, 0.15):
        for speed in (0.17, 0.19):
            for dx, dz in ((0.0, 0.0), (-0.10, 0.05), (-0.15, 0.15), (-0.20, 0.25)):
                k = baseline_knobs(task, speed=speed, lift=lift)
                k.update(wait_dx=dx, wait_dz=dz, T_approach=1.875 * np.hypot(dx, dz) / speed)
                out.append(k)
    return out


def background_search(queue, tasks, max_eval=1500, quick_eval=200, init=None):
    """Run in a separate, low-priority process while the robot works (pp_runner
    planner:=energy_opt). tasks: list of (key, Task), e.g. one per carton slot.
    First a quick search for every task (so each one soon has a good path), then
    the full search for every task. Puts (key, E_pred, knobs, CycleReference) on
    the queue each time a task's best motion improves by more than 1 %, and
    'done' at the end. init: {key: knobs} already learned (warm start); the
    search starts from them and only sends what is better."""
    import os
    import signal
    os.nice(10)                     # never slow down the runner or Gazebo
    signal.signal(signal.SIGINT, signal.SIG_IGN)    # Ctrl+C is for pp_runner, which stops this process
    dyn = None
    planners, sent, best = {}, {}, {}
    for key, task in tasks:
        planners[key] = Planner(task, dyn)
        dyn = planners[key].dyn     # same robot and object: share the model
        sent[key] = np.inf
        if init and key in init:
            E, pen, _ = planners[key].evaluate(init[key])
            if pen == 0:
                best[key], sent[key] = dict(init[key]), E

    def send(key, k, E):
        queue.put((key, float(E), {n: float(v) for n, v in k.items()}, planners[key].reference(k)))
        sent[key] = E

    # grid: only rate the start motions and the coarse grid (~1 s per task), so every
    # task has a good path within the first cycles; then quick and full refinement
    for phase, n_eval, n_refine in (('grid', 0, 0), ('quick', quick_eval, 1), ('full', max_eval, 3)):
        for key, task in tasks:
            pl = planners[key]
            starts = ([best[key]] if key in best else []) + default_starts(task) + coarse_grid(task)
            improved = lambda k, E, key=key: send(key, k, E) if E < 0.99 * sent[key] else None
            k, E = pl.optimize(starts=starts, max_eval=n_eval, n_refine=n_refine, callback=improved)
            if k is not None:
                best[key] = k
                if E < sent[key]:
                    send(key, k, E)
    queue.put('done')
