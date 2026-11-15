"""Can-packing cell: plain 330 ml cans (no logo) come on a conveyor to an end
stop and the robot packs them into a 3 x 4 carton (scenario:=cans).

Top view (world frame, metres; robot base at the origin):

    conveyor  ====================> | stop        +-----------+  y = 0.37
    (y -0.80 .. -0.17, x = 0.45)    (pick at        | o  o  o   |  row 3  filled first
                                     y = -0.20)     | o  o  o   |  row 2
                                                    | o  o  o   |  row 1
                                                    | o  o  o   |  row 0  filled last
                                                    +-----------+  y = 0.07

The far row is filled first, so the robot never carries a can over cans that
are already packed. Gazebo: one working can ('can') is carried by the gripper;
when it is released in slot i, the packed can 'packed_<i>' is put exactly there.
The belt carries a row of cans: the working can at the end stop and QUEUE
waiting cans ('queue_<k>') behind it. While the robot is away, the belt moves
every can one place forward (BELT_PITCH, in BELT_TIME); the working can enters
at the back. As all cans look the same, the working can then swaps places with
the can that reached the end stop, so it is always the one the gripper takes.
"""
import numpy as np

CAN_MASS = 0.35           # kg, full 330 ml can
CAN_RADIUS = 0.033        # m  (66 mm)
CAN_HEIGHT = 0.115        # m
GRIPPER_LEN = 0.10        # m, tool0 to the gripper tip
TABLE_TOP = 0.08
BELT_TOP = 0.081          # conveyor belt surface (1 mm above the table it runs over)
PICK_XY = np.array([0.45, -0.20])

CARTON_CENTER = np.array([0.45, 0.22])
PITCH = 0.075             # slot spacing (66 mm can + 9 mm)
COLS, ROWS = 3, 4         # x, y
FLOOR_TOP = TABLE_TOP + 0.004
WALL_TOP = FLOOR_TOP + 0.06
WALL = 0.005              # wall thickness
INNER = np.array([COLS * PITCH, ROWS * PITCH])
END_STOP = (0.45 - 0.06, 0.45 + 0.06, -0.20 + CAN_RADIUS + 0.002, -0.20 + CAN_RADIUS + 0.012, BELT_TOP + 0.03)
RELEASE_GAP = 0.005       # m, can bottom above the carton floor at release
STORAGE = np.array([-40.0, -0.6])  # where unused packed cans wait (far away, out of view)
QUEUE = 3                 # cans waiting on the belt behind the one at the end stop
BELT_PITCH = 0.12         # m between cans on the belt
BELT_TIME = 2.0           # s for the belt to move one can forward
BASELINE_LIFT = 0.15      # m: the carried can must clear packed neighbours (top 0.199) next to its slot

PICK = np.array([PICK_XY[0], PICK_XY[1], BELT_TOP + CAN_HEIGHT + GRIPPER_LEN])   # tool0 at the pick


def slot_xy(i):
    """Centre of slot i in fill order (0 = first: far row)."""
    row = ROWS - 1 - i // COLS
    col = i % COLS
    return CARTON_CENTER + [(col - (COLS - 1) / 2) * PITCH, (row - (ROWS - 1) / 2) * PITCH]


N_SLOTS = COLS * ROWS


def place_pose(i):
    """tool0 position that releases the can in slot i, RELEASE_GAP above the floor."""
    x, y = slot_xy(i)
    return np.array([x, y, FLOOR_TOP + RELEASE_GAP + CAN_HEIGHT + GRIPPER_LEN])


def obstacles(i):
    """Boxes (xmin, xmax, ymin, ymax, ztop) the moving can / gripper must pass over
    when packing slot i: the carton walls, the cans already packed, the end stop."""
    cx, cy = CARTON_CENTER
    hx, hy = INNER / 2
    out = [(cx - hx - WALL, cx + hx + WALL, cy - hy - WALL, cy - hy, WALL_TOP),
           (cx - hx - WALL, cx + hx + WALL, cy + hy, cy + hy + WALL, WALL_TOP),
           (cx - hx - WALL, cx - hx, cy - hy, cy + hy, WALL_TOP),
           (cx + hx, cx + hx + WALL, cy - hy, cy + hy, WALL_TOP),
           END_STOP]
    for j in range(i):
        x, y = slot_xy(j)
        out.append((x - CAN_RADIUS, x + CAN_RADIUS, y - CAN_RADIUS, y + CAN_RADIUS, FLOOR_TOP + CAN_HEIGHT))
    return out


def packed_pose(i):
    """Gazebo pose (x, y, z of the can centre) of packed can i in its slot."""
    x, y = slot_xy(i)
    return np.array([x, y, FLOOR_TOP + CAN_HEIGHT / 2 + 0.001])


def storage_pose(i):
    return np.array([STORAGE[0], STORAGE[1] + 0.08 * i, CAN_HEIGHT / 2 + 0.001])


def belt_pose(k):
    """Gazebo pose of the can at belt place k (0 = end stop, larger = further upstream)."""
    return np.array([PICK_XY[0], PICK_XY[1] - k * BELT_PITCH, BELT_TOP + CAN_HEIGHT / 2 + 0.001])


def conveyor_pose():
    """Gazebo pose of the working can standing at the end stop."""
    return belt_pose(0)
