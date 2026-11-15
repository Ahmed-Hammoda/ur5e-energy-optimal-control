"""UR5e kinematics: forward kinematics, geometric Jacobian and numerical IK.

Uses the official UR5e DH parameters. The DH chain is expressed in the UR
'base' frame, which is rotated 180 deg about Z with respect to 'base_link'
(the frame Gazebo/RViz show as the world origin). All public functions take
and return poses in the WORLD / base_link frame so the rest of the project
never has to think about this rotation.
"""
import numpy as np

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# UR5e DH parameters (Universal Robots, "DH parameters for calculations of kinematics")
D = np.array([0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996])
A = np.array([0.0, -0.425, -0.3922, 0.0, 0.0, 0.0])
ALPHA = np.array([np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0])

# base_link (world) <- base (DH frame 0): rotation of pi about Z
T_WORLD_BASE = np.diag([-1.0, -1.0, 1.0, 1.0])

# Tool pointing straight down, used for every pick-and-place pose
R_TOOL_DOWN = np.array([[1.0, 0.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [0.0, 0.0, -1.0]])


def _dh(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0.0, sa, ca, d],
                     [0.0, 0.0, 0.0, 1.0]])


def fk_all(q):
    """Return the list of 7 frames (world <- frame i), i = 0..6 (6 = tool0)."""
    T = T_WORLD_BASE.copy()
    frames = [T]
    for i in range(6):
        T = T @ _dh(q[i], D[i], A[i], ALPHA[i])
        frames.append(T)
    return frames


def fk(q):
    """Pose of tool0 in the world frame as a 4x4 matrix."""
    return fk_all(q)[-1]


def jacobian(q):
    """6x6 geometric Jacobian of tool0 in the world frame ([v; w])."""
    frames = fk_all(q)
    p_e = frames[-1][:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        z = frames[i][:3, 2]
        p = frames[i][:3, 3]
        J[:3, i] = np.cross(z, p_e - p)
        J[3:, i] = z
    return J


def _rot_error(R_des, R):
    """Orientation error as a rotation vector (world frame)."""
    Re = R_des @ R.T
    angle = np.arccos(np.clip((np.trace(Re) - 1.0) / 2.0, -1.0, 1.0))
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
    return axis * angle / (2.0 * np.sin(angle))


def ik(p_des, R_des, q_seed, tol=1e-6, max_iter=200, damping=1e-3):
    """Damped-least-squares IK. Returns (q, converged)."""
    q = np.array(q_seed, dtype=float)
    for _ in range(max_iter):
        T = fk(q)
        err = np.hstack([p_des - T[:3, 3], _rot_error(R_des, T[:3, :3])])
        if np.linalg.norm(err) < tol:
            return q, True
        J = jacobian(q)
        dq = J.T @ np.linalg.solve(J @ J.T + damping * np.eye(6), err)
        q += dq
    return q, False
