"""Reference trajectory for one pick-and-place cycle.

The cycle follows the pick-and-place task:
    Pick -> Lift -> Transfer -> Place  (and the same path back to Pick)
Each move is a straight Cartesian line with a minimum-jerk (quintic) time
law whose PEAK speed equals the requested TCP speed, so the TCP never goes
faster than the 0.20 m/s limit. Every sample is converted to joint space
with IK, and the joint velocity reference comes from the Jacobian, so
q_ref and qd_ref are exactly consistent.

The same reference is replayed for every controller and every cycle,
so the energy comparison is fair.
"""
from dataclasses import dataclass, field

import numpy as np

from .kinematics import R_TOOL_DOWN, fk, ik, jacobian

# Task geometry (world frame, metres)
PICK = np.array([0.45, -0.20, 0.25])
# At z = 0.25 the carried box (gripper 0.10 m + box 0.07 m) would touch the
# table top (0.08 m) exactly; the position-held arm then presses it into the
# table up to the torque limit. 5 mm clearance: the box is released just above.
PLACE = np.array([0.45, 0.20, 0.255])
LIFT_HEIGHT = 0.10

# Elbow-up, wrist-down IK seed (well conditioned, far from singularities)
IK_SEED = np.array([-0.69, -1.43, 2.07, -2.21, -1.571, -2.26])


@dataclass
class CycleReference:
    dt: float
    t: np.ndarray            # (N,)
    q: np.ndarray            # (N, 6) joint positions
    qd: np.ndarray           # (N, 6) joint velocities
    p: np.ndarray            # (N, 3) TCP (tool0) positions
    segment: list = field(default_factory=list)   # (N,) segment name per sample
    place_index: int = 0     # sample at which the box is released
    pick_index: int = 0      # sample at which the box is grasped (end of the cycle)

    @property
    def duration(self):
        return self.t[-1]


def _min_jerk(L, v_max, dt):
    """Return s(t) and s_dot(t) for a quintic profile with peak speed v_max."""
    T = 1.875 * L / v_max          # peak speed of a quintic = 1.875 L / T
    n = max(int(np.ceil(T / dt)), 1)
    tau = np.linspace(0.0, 1.0, n + 1)[1:]
    s = L * (10 * tau**3 - 15 * tau**4 + 6 * tau**5)
    sd = L / T * (30 * tau**2 - 60 * tau**3 + 30 * tau**4)
    return s, sd


def build_cycle(speed=0.15, dt=0.01, dwell=0.5, pick=PICK, place=PLACE,
                lift=LIFT_HEIGHT, q_seed=IK_SEED, takt=None):
    """Build one full cycle starting and ending at the pick pose. With a takt time
    (fixed production rhythm) the robot then holds at the pick pose ('wait')
    until the next part is due."""
    up_pick = pick + np.array([0.0, 0.0, lift])
    up_place = place + np.array([0.0, 0.0, lift])
    plan = [
        ('grasp', pick, pick),
        ('lift', pick, up_pick),
        ('transfer', up_pick, up_place),
        ('lower', up_place, place),
        ('release', place, place),
        ('lift_back', place, up_place),
        ('return', up_place, up_pick),
        ('lower_back', up_pick, pick),
    ]

    q_start, ok = ik(pick, R_TOOL_DOWN, q_seed)
    if not ok:
        raise RuntimeError('Pick pose is not reachable')

    qs, qds, ps, segs = [q_start], [np.zeros(6)], [pick.copy()], ['start']
    q = q_start
    place_index = 0
    for name, a, b in plan:
        if np.allclose(a, b):                       # dwell (gripper close / open)
            n = int(round(dwell / dt))
            for _ in range(n):
                qs.append(q.copy()); qds.append(np.zeros(6)); ps.append(a.copy()); segs.append(name)
            if name == 'release':
                place_index = len(qs) - 1
            continue
        L = np.linalg.norm(b - a)
        u = (b - a) / L
        for s, sd in zip(*_min_jerk(L, speed, dt)):
            p = a + s * u
            q, ok = ik(p, R_TOOL_DOWN, q)
            if not ok:
                raise RuntimeError(f'IK failed in segment {name}')
            v = np.hstack([sd * u, np.zeros(3)])
            qd = np.linalg.solve(jacobian(q), v)
            qs.append(q.copy()); qds.append(qd); ps.append(p); segs.append(name)

    q_arr = np.array(qs)
    ref = CycleReference(
        dt=dt,
        t=np.arange(len(qs)) * dt,
        q=q_arr,
        qd=np.array(qds),
        p=np.array(ps),
        segment=segs,
        place_index=place_index,
        pick_index=len(qs) - 1,
    )
    return ref if takt is None else with_takt(ref, takt)


def with_takt(ref, takt):
    """The cycle followed by a hold at its last pose ('wait') until the takt is over."""
    n = int(round(takt / ref.dt)) + 1 - len(ref.t)
    if n < 0:
        raise RuntimeError(f'Cycle ({ref.duration:.2f} s) does not fit in the takt ({takt:.2f} s)')
    return CycleReference(
        dt=ref.dt,
        t=np.arange(len(ref.t) + n) * ref.dt,
        q=np.vstack([ref.q, np.repeat(ref.q[-1:], n, 0)]),
        qd=np.vstack([ref.qd, np.zeros((n, 6))]),
        p=np.vstack([ref.p, np.repeat(ref.p[-1:], n, 0)]),
        segment=list(ref.segment) + ['wait'] * n,
        place_index=ref.place_index,
        pick_index=ref.pick_index,
    )


def offset_start(ref, offset_m, direction=(1.0, 1.0, 0.0)):
    """Joint configuration of the pick pose shifted by offset_m (start-offset test)."""
    d = np.asarray(direction, dtype=float)
    d /= np.linalg.norm(d)
    q, ok = ik(ref.p[0] + offset_m * d, R_TOOL_DOWN, ref.q[0])
    if not ok:
        raise RuntimeError('Offset start pose is not reachable')
    return q


def tcp_position(q):
    return fk(q)[:3, 3]
