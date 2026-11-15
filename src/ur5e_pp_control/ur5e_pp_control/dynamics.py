"""UR5e inverse dynamics: tau = M(q) qdd + C(q, qd) qd + g(q).

Gazebo's reported joint torque contains only the gravity part g(q) (the joints
are velocity servos), so the energy is computed from this model instead, as in
Clochiatti et al. (Robotica 2024). Gazebo's torque stays a cross-check of g(q).

Recursive Newton-Euler over the workcell URDF (UR5e masses and inertias from
ur_description + the 0.7 kg gripper), vectorised over N samples. The carried
object (the 7 cm box, or a can: a cylinder along the tool axis) is added at
tool0 while it is attached to the gripper.
"""
import os
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

from .kinematics import JOINT_NAMES

GRAVITY = np.array([0.0, 0.0, -9.81])
GRIPPER_MASS = 0.7
BOX_SIZE = 0.07                                       # m, cube (sim.launch.py)
GRIPPER_LEN = 0.10                                    # m, tool0 to the gripper tip
ATTACHED = ('lift', 'transfer', 'lower', 'release')   # segments with the box in the gripper


def workcell_urdf():
    """URDF text of the workcell robot, expanded from the installed xacro."""
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory('ur5e_pp_control'), 'urdf', 'ur5e_workcell.urdf.xacro')
    return subprocess.run(['xacro', path], check=True, capture_output=True, text=True).stdout


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _origin(el):
    o = el.find('origin')
    if o is None:
        return np.zeros(3), np.eye(3)
    return (np.array([float(v) for v in o.get('xyz', '0 0 0').split()]),
            _rpy(*[float(v) for v in o.get('rpy', '0 0 0').split()]))


def _skew(k):
    return np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])


class Dynamics:
    def __init__(self, urdf_xml=None, obj_height=BOX_SIZE, obj_radius=None):
        """obj_radius None: a cube of side obj_height; else a cylinder (can)."""
        self.obj_com = np.array([0.0, 0.0, GRIPPER_LEN + obj_height / 2])
        if obj_radius is None:
            self.obj_inertia = obj_height ** 2 / 6 * np.eye(3)                 # per kg
        else:
            side = (3 * obj_radius ** 2 + obj_height ** 2) / 12
            self.obj_inertia = np.diag([side, side, obj_radius ** 2 / 2])      # per kg, axis = tool z
        root = ET.fromstring(urdf_xml or workcell_urdf())
        self.links = {}
        for link in root.findall('link'):
            inert = link.find('inertial')
            if inert is None:
                continue
            c, R = _origin(inert)
            a = inert.find('inertia').attrib
            I = np.array([[a['ixx'], a['ixy'], a['ixz']], [a['ixy'], a['iyy'], a['iyz']],
                          [a['ixz'], a['iyz'], a['izz']]], dtype=float)
            self.links[link.get('name')] = (float(inert.find('mass').get('value')), c, R @ I @ R.T)
        children = {}
        for j in root.findall('joint'):
            xyz, R = _origin(j)
            ax = j.find('axis')
            axis = np.array([float(v) for v in ax.get('xyz').split()]) if ax is not None else np.zeros(3)
            children.setdefault(j.find('parent').get('link'), []).append(
                dict(name=j.get('name'), revolute=j.get('type') == 'revolute', child=j.find('child').get('link'),
                     xyz=xyz, R=R, axis=axis))
        self.order = []                                # joints, parents before children

        def walk(link):
            for j in children.get(link, []):
                j['parent'] = link
                self.order.append(j)
                walk(j['child'])
        walk('world')
        idx = {n: i for i, n in enumerate(JOINT_NAMES)}
        for j in self.order:
            j['q_index'] = idx.get(j['name'])

    def tau(self, q, qd, qdd, box_mass=0.0):
        """Joint torques, shape (N, 6), for q, qd, qdd of shape (N, 6).
        box_mass: scalar or (N,) array, 0 where the box is not attached."""
        q, qd, qdd = (np.atleast_2d(np.asarray(x, float)) for x in (q, qd, qdd))
        n = len(q)
        box = np.broadcast_to(np.asarray(box_mass, float), (n,))
        zero = np.zeros((n, 3))
        st = {'world': dict(T=np.broadcast_to(np.eye(3), (n, 3, 3)), p=zero, w=zero, al=zero,
                            a=np.broadcast_to(-GRAVITY, (n, 3)))}
        for j in self.order:                           # forward: velocities and accelerations
            s = st[j['parent']]
            R = s['T'] @ j['R']
            p = s['p'] + s['T'] @ j['xyz']
            if j['revolute'] and j['q_index'] is not None:
                k = j['q_index']
                z = R @ j['axis']
                K = _skew(j['axis'] / np.linalg.norm(j['axis']))
                sq, cq = np.sin(q[:, k])[:, None, None], np.cos(q[:, k])[:, None, None]
                R = R @ (np.eye(3) + sq * K + (1 - cq) * (K @ K))
                vd, va = z * qd[:, k:k + 1], z * qdd[:, k:k + 1]
            else:
                z, vd, va = zero, zero, zero
            d = p - s['p']
            a = s['a'] + np.cross(s['al'], d) + np.cross(s['w'], np.cross(s['w'], d))
            st[j['child']] = dict(T=R, p=p, z=z, w=s['w'] + vd, al=s['al'] + va + np.cross(s['w'], vd), a=a,
                                  f=np.zeros((n, 3)), m=np.zeros((n, 3)))
        st['world'].update(f=np.zeros((n, 3)), m=np.zeros((n, 3)))
        out = np.zeros((n, 6))
        for j in reversed(self.order):                 # backward: forces and moments
            s = st[j['child']]
            bodies = []                                # (mass (N,1), COM in link frame, world inertia (N,3,3))
            if j['child'] in self.links:
                m, c, I = self.links[j['child']]
                bodies.append((np.full((n, 1), m), c, s['T'] @ I @ np.swapaxes(s['T'], 1, 2)))
            if j['child'] == 'tool0':                  # the carried object
                Iw = s['T'] @ self.obj_inertia @ np.swapaxes(s['T'], 1, 2)
                bodies.append((box[:, None], self.obj_com, box[:, None, None] * Iw))
            F, N = s['f'].copy(), s['m'].copy()
            for m, c, Iw in bodies:
                cw = s['T'] @ c
                ac = s['a'] + np.cross(s['al'], cw) + np.cross(s['w'], np.cross(s['w'], cw))
                F = F + m * ac
                N = N + np.einsum('nij,nj->ni', Iw, s['al']) + np.cross(s['w'], np.einsum('nij,nj->ni', Iw, s['w'])) \
                    + np.cross(cw, m * ac)
            par = st[j['parent']]
            par['f'] = par['f'] + F
            par['m'] = par['m'] + N + np.cross(s['p'] - par['p'], F)
            if j['q_index'] is not None:
                out[:, j['q_index']] = np.einsum('ni,ni->n', s['z'], N)
        return out

    def gravity(self, q, box_mass=0.0):
        z = np.zeros_like(np.atleast_2d(q), dtype=float)
        return self.tau(q, z, z, box_mass)


def box_mass_per_sample(segments, box_mass):
    """box_mass where the box is in the gripper, else 0."""
    return np.where(np.isin(np.asarray(segments, str), ATTACHED), box_mass, 0.0)
