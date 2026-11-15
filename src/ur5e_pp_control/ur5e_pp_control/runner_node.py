"""Experiment runner: N pick-and-place cycles with one existing UR controller,
measuring tracking accuracy and ENERGY consumption.

mode:=trajectory  joint_trajectory_controller         (open source, ros2_controllers)
mode:=scaled      scaled_joint_trajectory_controller  (open source, UR's default)

Flow:
  1. release the gripper, put a box on the red pick zone, activate the
     controller and move to the start pose (pick pose, optionally shifted
     by start_offset_mm)
  2. run the cycles: the same pick -> lift -> transfer -> place -> return
     path is sent to the controller every cycle. The gripper attaches the
     box after 'grasp' and releases it on the green zone after 'release';
     a new box is put on the red zone during 'return'
  3. log joint positions, velocities and torques at 100 Hz (validate.py
     computes the energy from this log)
  4. fault (TCP error > fault_limit_mm for fault_samples samples in a row) -> stop
  5. write <results_dir>/<tag>.csv (every sample) and <tag>.json (per cycle)

planner:=energy_opt (the new controller): cycle 1 runs the baseline path while
energy_opt.background_search looks for the energy-optimal path in its own
process; every cycle starts with the best path found so far.
"""
import json
import multiprocessing
import os
import queue
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from rclpy.action import ActionClient
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from . import cans
from .dynamics import BOX_SIZE, GRIPPER_MASS
from .energy_opt import Planner, Task, background_search
from .kinematics import JOINT_NAMES
from .trajectory import PICK, build_cycle, offset_start, tcp_position, with_takt

CONTROLLERS = {
    'trajectory': 'joint_trajectory_controller',
    'scaled': 'scaled_joint_trajectory_controller',
}
BOX_HALF = 0.035          # box is a 7 cm cube
TABLE_TOP = 0.08


def _dur(seconds):
    sec = int(seconds)
    return Duration(sec=sec, nanosec=int((seconds - sec) * 1e9))


def _cached(key, make):
    """make() once, then from ~/.cache/ur5e_pp_control (IK at every 10 ms sample takes ~1 s per
    path). The key includes the path code's modification times, so edits rebuild the cache."""
    import hashlib
    import pickle
    from . import kinematics, trajectory
    stamp = [os.path.getmtime(m.__file__) for m in (kinematics, trajectory)]
    name = hashlib.sha1(repr((key, stamp)).encode()).hexdigest()[:16]
    path = os.path.join(os.path.expanduser('~/.cache/ur5e_pp_control'), name + '.pkl')
    try:
        with open(path, 'rb') as f:
            return pickle.load(f)
    except (OSError, pickle.UnpicklingError, EOFError):
        value = make()
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'wb') as f:
            pickle.dump(value, f)
        return value


def scenario_setup(scenario, speed=0.15, payload=1.2):
    """Baseline references, takt, carried object and planner tasks of a scenario.
    One reference per place ("key"): the box has one, the carton one per slot."""
    obj_mass = max(payload - GRIPPER_MASS, 0.0)
    if scenario == 'cans':
        # fixed takt: the line rate the baseline can keep at its slowest (farthest) slot
        moves = _cached(('cans', speed, cans.PICK.tolist(), [cans.place_pose(i).tolist() for i in range(cans.N_SLOTS)],
                         cans.BASELINE_LIFT),
                        lambda: [build_cycle(speed=speed, pick=cans.PICK, place=cans.place_pose(i),
                                             lift=cans.BASELINE_LIFT) for i in range(cans.N_SLOTS)])
        takt = float(np.ceil(max(m.duration for m in moves) * 10) / 10)
        refs = [with_takt(m, takt) for m in moves]
        tasks = [(i, Task(pick=cans.PICK, place=cans.place_pose(i), box_mass=obj_mass,
                          box_height=cans.CAN_HEIGHT, obj_radius=cans.CAN_RADIUS, takt=takt,
                          obstacles=cans.obstacles(i))) for i in range(cans.N_SLOTS)]
        return refs, takt, cans.CAN_HEIGHT, cans.CAN_RADIUS, tasks
    if scenario == 'box':
        refs = [build_cycle(speed=speed)]
        takt = float(refs[0].duration)
        return refs, takt, BOX_SIZE, None, [(0, Task(box_mass=obj_mass, takt=takt))]
    raise ValueError("scenario must be 'box' or 'cans'")


class Runner(Node):
    def __init__(self):
        super().__init__('pp_runner')
        p = self.declare_parameter
        self.mode = p('mode', 'scaled').value
        self.cycles = p('cycles', 30).value
        self.speed = p('speed', 0.15).value
        self.offset_mm = p('start_offset_mm', 0.0).value
        self.fault_limit = p('fault_limit_mm', 10.0).value / 1000.0
        self.fault_samples = p('fault_samples', 3).value    # consecutive samples above the limit
        self.settle_tol = p('settle_tol_mm', 1.0).value / 1000.0
        self.settle_timeout = p('settle_timeout_s', 2.0).value
        self.payload = p('payload_kg', 1.2).value      # for the record only
        self.use_gripper = p('use_gripper', True).value
        self.planner = p('planner', 'baseline').value     # baseline | energy_opt
        self.scenario = p('scenario', 'box').value        # box | cans (can-packing line)
        self.warm_start = os.path.expanduser(p('warm_start', '').value)   # results of an earlier energy_opt run
        self.tag = p('tag', '').value or (self.mode if self.planner == 'baseline' else f'{self.mode}_{self.planner}')
        self.results_dir = os.path.expanduser(p('results_dir', '~/ur5e_pp_ws/results').value)
        if self.mode not in CONTROLLERS:
            raise ValueError(f"mode must be one of {list(CONTROLLERS)}")
        self.ctrl = CONTROLLERS[self.mode]

        self.refs, self.takt, self.obj_height, self.obj_radius, tasks = scenario_setup(
            self.scenario, self.speed, self.payload)
        self.ref = self.refs[0]
        self.dt = self.ref.dt
        # energy_opt: the first cycles run the baseline paths while the optimizer searches
        # in its own process; every cycle starts with the best path found so far for its place
        self.plan_version = [0] * len(self.refs)
        self.plan_E = [None] * len(self.refs)
        self.plan_knobs = {}
        self.search_done, self.worker = True, None
        if self.planner == 'energy_opt':
            if self.warm_start:
                self._load_plans(self.warm_start, tasks)
            ctx = multiprocessing.get_context('spawn')
            self.plan_q = ctx.Queue()
            self.worker = ctx.Process(target=background_search, daemon=True,
                                      args=(self.plan_q, tasks), kwargs=dict(init=dict(self.plan_knobs)))
            self.worker.start()
            self.search_done = False
        elif self.planner != 'baseline':
            raise ValueError("planner must be 'baseline' or 'energy_opt'")

        self.q = None
        self.qd = None
        self.tau = None
        self.q_time = None          # stamp of the joint state in self.q
        self.last_t = None
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        self.err_pub = self.create_publisher(Float64, '/pp/tcp_error_mm', 10)
        self.phase_pub = self.create_publisher(String, '/pp/phase', 10)    # '<cycle> <segment>'
        self.switch_cli = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.traj_cli = ActionClient(self, FollowJointTrajectory, f'/{self.ctrl}/follow_joint_trajectory')
        self.attach_pub = self.create_publisher(Empty, '/pp/gripper/attach', 10)
        self.detach_pub = self.create_publisher(Empty, '/pp/gripper/detach', 10)
        self.pose_cli = self.create_client(SetEntityPose, '/world/workcell/set_pose')
        self.prev_seg = None
        self.t_start_wait = None
        self.slot, self.prev_slot = 0, None
        self.belt_models = ['can'] + [f'queue_{k}' for k in range(cans.QUEUE)]   # cans at belt places 0, 1, ...
        self.belt_t0, self.belt_last = None, -1.0

        self.done = False
        self.rows = []
        self.cycle_stats = []
        self.state = 'wait_joints'
        self.pending = None
        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f'controller={self.ctrl} scenario={self.scenario} planner={self.planner} cycles={self.cycles} '
            f'speed={self.speed} m/s offset={self.offset_mm} mm, takt {self.takt:.2f} s')

    # ------------------------------------------------------------------ io
    def _on_joints(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(n in idx for n in JOINT_NAMES):
            return
        pick = lambda arr: np.array([arr[idx[n]] for n in JOINT_NAMES]) if len(arr) else np.zeros(6)
        self.q = pick(msg.position)
        self.qd = pick(msg.velocity)
        self.tau = pick(msg.effort)
        self.q_time = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------ gripper
    def _attach(self):
        if self.use_gripper:
            self.attach_pub.publish(Empty())

    def _detach(self):
        if self.use_gripper:
            self.detach_pub.publish(Empty())

    def _set_pose(self, name, xyz):
        if not self.use_gripper:
            return
        req = SetEntityPose.Request()
        req.entity.name = name
        req.entity.type = Entity.MODEL
        req.pose.position.x, req.pose.position.y, req.pose.position.z = (float(v) for v in xyz)
        req.pose.orientation.w = 1.0
        self.pose_cli.call_async(req)

    def _new_box(self):
        """A new part arrives: the box on the red zone, or the next can at the conveyor end stop.
        Cans: the can just released becomes 'packed_<slot>' and the working can goes back to the belt."""
        if self.scenario == 'cans':
            if self.prev_slot is None:               # start: the row of cans on the belt
                for k, name in enumerate(self.belt_models):
                    self._set_pose(name, cans.belt_pose(k))
                return
            self._set_pose(f'packed_{self.prev_slot}', cans.packed_pose(self.prev_slot))
            self._set_pose('can', cans.belt_pose(cans.QUEUE + 1))    # enters at the back of the belt
            self.belt_t0, self.belt_last = self._now(), -1.0
        else:
            self._set_pose('box', [PICK[0], PICK[1], TABLE_TOP + BOX_HALF])

    def _move_belt(self):
        """Belt moving: every can one place forward, then the working can swaps places
        with the (identical) can that reached the end stop."""
        if self.belt_t0 is None:
            return
        now = self._now()
        s = min((now - self.belt_t0) / cans.BELT_TIME, 1.0)
        if s < 1.0 and now - self.belt_last < 0.05:
            return                                   # 20 Hz is smooth enough to watch
        self.belt_last = now
        row = self.belt_models[1:] + ['can']         # cans at places 1 .. QUEUE+1 before the move
        for k, name in enumerate(row):
            self._set_pose(name, cans.belt_pose(k + 1 - s))
        if s >= 1.0:
            front = row[0]
            self._set_pose('can', cans.belt_pose(0))
            self._set_pose(front, cans.belt_pose(len(row) - 1))
            self.belt_models = ['can'] + row[1:-1] + [front]
            self.belt_t0 = None

    def _empty_carton(self):
        """Full carton leaves, an empty one arrives: the packed cans go back to storage."""
        for i in range(cans.N_SLOTS):
            self._set_pose(f'packed_{i}', cans.storage_pose(i))

    def _gripper_events(self, seg):
        """Fire gripper actions when the reference enters a new segment."""
        if seg == self.prev_seg:
            return
        if self.prev_seg == 'grasp':
            self._attach()           # end of grasp pause: close the gripper
        elif self.prev_seg == 'release':
            self._detach()           # end of release pause: open the gripper on the green zone
        if seg == 'return':
            self.prev_slot = self.slot
            self._new_box()          # robot is away from both zones: next part arrives
        self.prev_seg = seg

    # ------------------------------------------------------------ controller
    def _switch(self):
        """Activate the controller under test, deactivate the other one."""
        req = SwitchController.Request()
        req.activate_controllers = [self.ctrl]
        req.deactivate_controllers = [c for c in CONTROLLERS.values() if c != self.ctrl]
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        req.timeout = _dur(2.0)
        return self.switch_cli.call_async(req)

    def _send_trajectory(self, q_list, qd_list, t_list, start_delay=0.2):
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        for q, qd, t in zip(q_list, qd_list, t_list):
            traj.points.append(JointTrajectoryPoint(
                positions=[float(x) for x in q], velocities=[float(x) for x in qd],
                time_from_start=_dur(t)))
        start = self.get_clock().now() + rclpy.duration.Duration(seconds=start_delay)
        traj.header.stamp = start.to_msg()
        goal = FollowJointTrajectory.Goal(trajectory=traj)
        return self.traj_cli.send_goal_async(goal), start.nanoseconds * 1e-9

    # --------------------------------------------------------------- state machine
    def _tick(self):
        getattr(self, f'_st_{self.state}')()

    def _st_wait_joints(self):
        if self.q is None or not self.switch_cli.service_is_ready() or not self.traj_cli.server_is_ready():
            return
        if self.use_gripper and not self.pose_cli.service_is_ready():
            if self.t_start_wait is None:
                self.t_start_wait = time.monotonic()
            if time.monotonic() - self.t_start_wait < 10.0:
                return
            self.get_logger().warn('Gripper/box services not found: running without the box.')
            self.use_gripper = False
        self._detach()
        self._new_box()
        if self.scenario == 'cans':
            self._empty_carton()
        self.pending = self._switch()
        self.state = 'to_start_switch'

    def _st_to_start_switch(self):
        if not self.pending.done():
            return
        if not self.pending.result().ok:
            # right after the simulation starts, the controllers may still be loading: retry
            self.switch_tries = getattr(self, 'switch_tries', 0) + 1
            if self.switch_tries < 30:
                if self.switch_tries == 1:
                    self.get_logger().info(f'{self.ctrl} not ready yet, retrying ...')
                self.t_retry = time.monotonic() + 1.0
                self.state = 'switch_retry'
                return
            self.get_logger().error(f'Could not activate {self.ctrl}')
            return self._finish('switch_failed')
        q_goal = offset_start(self.ref, self.offset_mm / 1000.0) if self.offset_mm else self.ref.q[0]
        self.pending, self.t_wait = self._send_trajectory([q_goal], [np.zeros(6)], [4.0])
        self.t_wait += 4.5
        self.get_logger().info('Moving to start pose ...')
        self.state = 'to_start_wait'

    def _st_switch_retry(self):
        if time.monotonic() >= self.t_retry:
            self.pending = self._switch()
            self.state = 'to_start_switch'

    def _st_to_start_wait(self):
        if self._now() >= self.t_wait:
            self._start_cycle(1)

    def _load_plans(self, path, tasks):
        """Warm start: use the paths an earlier energy_opt run learned, from cycle 1."""
        path = path if path.endswith('.json') else path + '.json'
        with open(path) as f:
            meta = json.load(f)
        if abs(meta.get('takt_s', self.takt) - self.takt) > 1e-6 or meta.get('scenario', 'box') != self.scenario:
            raise ValueError(f'{path}: other scenario or takt, cannot warm start from it')
        dyn = None
        for key, task in tasks:
            plan = meta.get('plans', {}).get(str(key))
            if plan:
                pl = Planner(task, dyn)
                dyn = pl.dyn                     # same robot and object: build the model once
                self.refs[key] = pl.reference(plan['knobs'])
                self.plan_knobs[key], self.plan_E[key] = plan['knobs'], plan['E_pred_J']
                self.plan_version[key] = 1
        self.get_logger().info(f'warm start: {len(self.plan_knobs)} learned paths from {path}')

    def _take_plan(self):
        """Store the newest optimized paths the background search sent."""
        while self.worker is not None:
            try:
                item = self.plan_q.get_nowait()
            except queue.Empty:
                return
            if item == 'done':
                self.search_done = True
                continue
            key, self.plan_E[key], self.plan_knobs[key], self.refs[key] = item
            self.plan_version[key] += 1

    def _start_cycle(self, n):
        self.slot = (n - 1) % len(self.refs)
        if self.scenario == 'cans' and self.slot == 0 and n > 1:
            self._empty_carton()
            self.prev_slot = None
        prev = self.plan_version[self.slot]
        self._take_plan()
        self.ref = self.refs[self.slot]
        if self.plan_version[self.slot] != prev:
            self.get_logger().info(f'cycle {n} (slot {self.slot}): optimized path v{self.plan_version[self.slot]}, '
                                   f'predicted {self.plan_E[self.slot]:.2f} J'
                                   f'{" (search finished)" if self.search_done else ""}')
        self.cycle = n
        self.max_err = 0.0
        self.place_err = None
        self.prev_seg = None
        self.fault_count = 0
        step = 5   # send every 5th sample (20 Hz); the controller interpolates with splines
        idx = list(range(0, len(self.ref.t), step))
        if idx[-1] != len(self.ref.t) - 1:
            idx.append(len(self.ref.t) - 1)
        _, self.t0 = self._send_trajectory(self.ref.q[idx], self.ref.qd[idx], self.ref.t[idx])
        self.state = 'run'

    def _st_run(self):
        if self.scenario == 'cans':
            self._move_belt()
        # time of the measurement, not of this tick: if the node stalls, q is
        # still compared with the reference point it belongs to
        t = self.q_time - self.t0
        if t < 0 or t == self.last_t:
            return                                   # start delay, or no new joint state
        self.last_t = t
        n_ref = len(self.ref.t)
        i = min(int(round(t / self.dt)), n_ref - 1)
        q_ref, qd_ref, p_ref = self.ref.q[i], self.ref.qd[i], self.ref.p[i]
        seg = self.ref.segment[i]
        if seg != self.prev_seg:
            self.phase_pub.publish(String(data=f'{self.cycle} {seg}'))
        self._gripper_events(seg)

        p = tcp_position(self.q)
        err = float(np.linalg.norm(p - p_ref))
        self.max_err = max(self.max_err, err)
        if i >= self.ref.place_index and self.place_err is None:
            self.place_err = err
        self.err_pub.publish(Float64(data=err * 1000))
        self.rows.append([t, self.cycle, self.ref.segment[i], *q_ref, *self.q,
                          *qd_ref, *self.qd, *self.tau, *p_ref, *p, err])

        self.fault_count = self.fault_count + 1 if err > self.fault_limit else 0
        if self.fault_count >= self.fault_samples:
            self.get_logger().error(f'FAULT: TCP error {err * 1000:.1f} mm > limit, stopping.')
            self._record_cycle(t, fault=True)
            return self._finish('fault')

        if i == n_ref - 1:
            settled = err < self.settle_tol
            timeout = t - self.ref.duration > self.settle_timeout
            if settled or timeout:
                self._record_cycle(t, fault=False, settled=settled)
                if self.cycle >= self.cycles:
                    return self._finish('done')
                self._start_cycle(self.cycle + 1)

    def _record_cycle(self, t, fault, settled=False):
        s = {'cycle': self.cycle, 'cycle_time_s': t, 'max_tcp_err_mm': self.max_err * 1000,
             'placement_err_mm': None if self.place_err is None else self.place_err * 1000,
             'settled': settled, 'fault': fault,
             'slot': self.slot, 'plan_version': self.plan_version[self.slot],
             'plan_E_pred_J': self.plan_E[self.slot]}
        self.cycle_stats.append(s)
        self.get_logger().info(
            f"cycle {self.cycle}/{self.cycles}: time {t:.2f} s, max TCP err {s['max_tcp_err_mm']:.2f} mm")

    def _finish(self, reason):
        self.timer.cancel()
        if self.worker is not None and self.worker.is_alive():
            self.worker.terminate()
        os.makedirs(self.results_dir, exist_ok=True)
        base = os.path.join(self.results_dir, self.tag)
        header = (['t', 'cycle', 'segment']
                  + [f'q_ref_{j}' for j in range(6)] + [f'q_{j}' for j in range(6)]
                  + [f'qd_ref_{j}' for j in range(6)] + [f'qd_{j}' for j in range(6)]
                  + [f'tau_{j}' for j in range(6)]
                  + ['px_ref', 'py_ref', 'pz_ref', 'px', 'py', 'pz', 'tcp_err'])
        with open(base + '.csv', 'w') as f:
            f.write(','.join(header) + '\n')
            for r in self.rows:
                f.write(','.join(str(x) for x in r) + '\n')
        meta = {'mode': self.mode, 'controller': self.ctrl, 'tag': self.tag, 'end_reason': reason,
                'cycles_requested': self.cycles, 'speed': self.speed, 'start_offset_mm': self.offset_mm,
                'planner': self.planner, 'scenario': self.scenario, 'takt_s': self.takt,
                'plans': {str(k): {'knobs': v, 'E_pred_J': self.plan_E[k]} for k, v in self.plan_knobs.items()},
                'object_height': self.obj_height, 'object_radius': self.obj_radius,
                'payload_kg': self.payload, 'dt': self.dt,
                'ref_cycle_time_s': float(self.ref.duration), 'cycles': self.cycle_stats,
                'wall_time': time.strftime('%Y-%m-%d %H:%M:%S')}
        with open(base + '.json', 'w') as f:
            json.dump(meta, f, indent=2)
        self.get_logger().info(f'Finished ({reason}). Results: {base}.csv / .json')
        self.done = True


def learn_offline():
    """Learn the energy-optimal paths of every place without Gazebo and save them,
    for planner:=energy_opt warm_start:=<file>.

        ros2 run ur5e_pp_control learn_paths --scenario cans --out ~/ur5e_pp_ws/results/cans/learned
    """
    import argparse
    ap = argparse.ArgumentParser(description=learn_offline.__doc__)
    ap.add_argument('--scenario', default='cans')
    ap.add_argument('--speed', type=float, default=0.15)
    ap.add_argument('--payload', type=float, default=None, help='gripper + object [kg]')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    payload = a.payload if a.payload is not None else GRIPPER_MASS + (cans.CAN_MASS if a.scenario == 'cans' else 0.5)
    _, takt, h, r, tasks = scenario_setup(a.scenario, a.speed, payload)
    plans, t0 = {}, time.time()

    class Collect:
        def put(self, item):
            if item == 'done':
                return
            key, E, knobs, _ = item
            plans[str(key)] = {'knobs': knobs, 'E_pred_J': E}
            print(f'{time.time() - t0:7.1f} s  place {key}: {E:.2f} J', flush=True)

    background_search(Collect(), tasks)
    out = os.path.expanduser(a.out)
    out = out if out.endswith('.json') else out + '.json'
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w') as f:
        json.dump({'scenario': a.scenario, 'takt_s': takt, 'speed': a.speed, 'payload_kg': payload,
                   'object_height': h, 'object_radius': r, 'plans': plans}, f, indent=2)
    print(f'saved {len(plans)} learned paths to {out}')


def main():
    rclpy.init()
    node = Runner()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:                                         # a second Ctrl+C during shutdown: no traceback
            if node.worker is not None and node.worker.is_alive():
                node.worker.terminate()
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
