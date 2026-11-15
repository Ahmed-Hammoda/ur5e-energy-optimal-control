"""Start Gazebo with the UR5e workcell and the ros2_control controllers.

    ros2 launch ur5e_pp_control sim.launch.py                 # with GUI
    ros2 launch ur5e_pp_control sim.launch.py gui:=false      # headless (faster)
    ros2 launch ur5e_pp_control sim.launch.py box_mass:=0.3   # lighter box
    ros2 launch ur5e_pp_control sim.launch.py scenario:=cans  # can-packing line (conveyor + carton)

The box is a separate object on the red pick zone. The gripper attaches and
detaches it through Gazebo's DetachableJoint (ROS topics /pp/gripper/attach
and /pp/gripper/detach); the runner puts a fresh box on the pick zone
through the /world/workcell/set_pose service.

scenario:=cans: a conveyor brings plain cans (no logo) to an end stop and the
robot packs them into a 3 x 4 carton (layout in ur5e_pp_control/cans.py). The
gripper carries the working can 'can'; the runner puts 'packed_<i>' into slot
i when that can is released.
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, IfElseSubstitution,
                                  LaunchConfiguration, PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

BOX_SIZE = 0.07                     # m (cube)
PICK_XY = (0.45, -0.20)             # red zone
TABLE_TOP = 0.08                    # m


def box_sdf(mass, a=BOX_SIZE):
    i = mass * a * a / 6.0          # solid cube inertia
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="box">
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
        <inertia><ixx>{i}</ixx><iyy>{i}</iyy><izz>{i}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{a} {a} {a}</size></box></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
      <visual name="visual">
        <geometry><box><size>{a} {a} {a}</size></box></geometry>
        <material><ambient>0.95 0.5 0.1 1</ambient><diffuse>0.95 0.5 0.1 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


def _static_boxes(name, boxes, rgba):
    """Static model made of boxes: (cx, cy, cz, sx, sy, sz) each."""
    links = ''.join(f"""
      <collision name="c{k}"><pose>{cx} {cy} {cz} 0 0 0</pose><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
      <visual name="v{k}"><pose>{cx} {cy} {cz} 0 0 0</pose><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material></visual>"""
                    for k, (cx, cy, cz, sx, sy, sz) in enumerate(boxes))
    return f"""<?xml version="1.0"?>
<sdf version="1.9"><model name="{name}"><static>true</static><link name="link">{links}
</link></model></sdf>"""


def can_sdf(name, mass):
    """Plain can, no logo: red body, aluminium lid and base."""
    from ur5e_pp_control.cans import CAN_HEIGHT as h, CAN_RADIUS as r
    ixx, izz = mass * (3 * r * r + h * h) / 12, mass * r * r / 2
    return f"""<?xml version="1.0"?>
<sdf version="1.9"><model name="{name}"><link name="link">
  <inertial><mass>{mass}</mass>
    <inertia><ixx>{ixx}</ixx><iyy>{ixx}</iyy><izz>{izz}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
  <collision name="collision"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
    <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface></collision>
  <visual name="body"><geometry><cylinder><radius>{r}</radius><length>{h - 0.01}</length></cylinder></geometry>
    <material><ambient>0.75 0.1 0.1 1</ambient><diffuse>0.8 0.12 0.12 1</diffuse><specular>0.5 0.5 0.5 1</specular></material></visual>
  <visual name="lid"><pose>0 0 {h / 2 - 0.0025} 0 0 0</pose><geometry><cylinder><radius>{r * 0.95}</radius><length>0.005</length></cylinder></geometry>
    <material><ambient>0.75 0.75 0.78 1</ambient><diffuse>0.8 0.8 0.82 1</diffuse><specular>0.9 0.9 0.9 1</specular></material></visual>
  <visual name="base"><pose>0 0 {-h / 2 + 0.0025} 0 0 0</pose><geometry><cylinder><radius>{r * 0.95}</radius><length>0.005</length></cylinder></geometry>
    <material><ambient>0.75 0.75 0.78 1</ambient><diffuse>0.8 0.8 0.82 1</diffuse></material></visual>
</link></model></sdf>"""


def _create(sdf, name, xyz):
    return Node(package='ros_gz_sim', executable='create', output='screen',
                arguments=['-string', sdf, '-name', name, '-allow_renaming', 'false',
                           '-x', str(xyz[0]), '-y', str(xyz[1]), '-z', str(xyz[2])])


def spawn_cans(mass):
    from ur5e_pp_control import cans as c
    belt = (0.0, -0.485, c.BELT_TOP / 2, 0.14, 0.63, c.BELT_TOP)               # y -0.80 .. -0.17
    rails = [(sx * 0.075, -0.485, c.BELT_TOP + 0.01, 0.01, 0.63, 0.02) for sx in (-1, 1)]
    x0, x1, y0, y1, zt = c.END_STOP
    stop = ((x0 + x1) / 2 - c.PICK_XY[0], (y0 + y1) / 2, (c.BELT_TOP + zt) / 2, x1 - x0, y1 - y0, zt - c.BELT_TOP)
    hx, hy = c.INNER / 2
    hz, wz = c.WALL_TOP - c.TABLE_TOP, (c.WALL_TOP + c.TABLE_TOP) / 2
    carton = [(0, 0, (c.TABLE_TOP + c.FLOOR_TOP) / 2, 2 * hx + 2 * c.WALL, 2 * hy + 2 * c.WALL, c.FLOOR_TOP - c.TABLE_TOP),
              (0, -hy - c.WALL / 2, wz, 2 * hx + 2 * c.WALL, c.WALL, hz),
              (0, hy + c.WALL / 2, wz, 2 * hx + 2 * c.WALL, c.WALL, hz),
              (-hx - c.WALL / 2, 0, wz, c.WALL, 2 * hy, hz),
              (hx + c.WALL / 2, 0, wz, c.WALL, 2 * hy, hz)]
    out = [_create(_static_boxes('conveyor', [belt], '0.15 0.15 0.17 1'), 'conveyor', (c.PICK_XY[0], 0, 0)),
           _create(_static_boxes('rails', rails + [stop], '0.6 0.6 0.62 1'), 'rails', (c.PICK_XY[0], 0, 0)),
           _create(_static_boxes('carton', carton, '0.72 0.55 0.35 1'), 'carton', (*c.CARTON_CENTER, 0)),
           _create(can_sdf('can', mass), 'can', c.conveyor_pose())]
    out += [_create(can_sdf(f'packed_{i}', mass), f'packed_{i}', c.storage_pose(i)) for i in range(c.N_SLOTS)]
    out += [_create(can_sdf(f'queue_{k}', mass), f'queue_{k}', c.belt_pose(k + 1)) for k in range(c.QUEUE)]
    return out


def spawn_objects(context):
    mass = float(LaunchConfiguration('box_mass').perform(context))
    if LaunchConfiguration('scenario').perform(context) == 'cans':
        return spawn_cans(mass)
    return [Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-string', box_sdf(mass), '-name', 'box', '-allow_renaming', 'false',
                            '-x', str(PICK_XY[0]), '-y', str(PICK_XY[1]),
                            '-z', str(TABLE_TOP + BOX_SIZE / 2 + 0.001)])]


def generate_launch_description():
    pkg = FindPackageShare('ur5e_pp_control')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    controllers = PathJoinSubstitution([pkg, 'config', 'controllers.yaml'])
    scenario = LaunchConfiguration('scenario')
    world = PathJoinSubstitution([pkg, 'worlds', IfElseSubstitution(
        PythonExpression(["'", scenario, "' == 'cans'"]),
        if_value=IfElseSubstitution(LaunchConfiguration('fast'), if_value='cans_cell_demo.sdf', else_value='cans_cell.sdf'),
        else_value='workcell.sdf')])

    description = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([pkg, 'urdf', 'ur5e_workcell.urdf.xacro']),
        ' simulation_controllers:=', controllers,
        ' gripped_model:=', IfElseSubstitution(PythonExpression(["'", scenario, "' == 'cans'"]),
                                               if_value='can', else_value='box'),
    ]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': IfElseSubstitution(
            gui, if_value=[' -r -v 1 ', world], else_value=[' -s -r -v 1 ', world])}.items(),
    )

    def spawner(name, *extra):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '-c', '/controller_manager', *extra], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Show the Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='false', description='Also start RViz'),
        DeclareLaunchArgument('box_mass', default_value='0.5', description='Box (or can) mass in kg (gripper is 0.7 kg)'),
        DeclareLaunchArgument('scenario', default_value='box', description='box (original task) | cans (can-packing line)'),
        DeclareLaunchArgument('fast', default_value='false',
                              description='cans: 2 ms physics step and no shadows (presentation; not for measurements)'),
        # Own Gazebo partition per launch: a second simulation can never spawn
        # its robot into this world (that gives two overlapping robots).
        SetEnvironmentVariable('GZ_PARTITION', os.environ.get('GZ_PARTITION', f'ur5e_pp_{os.getpid()}')),
        gazebo,
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'use_sim_time': True, 'robot_description': description}]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-topic', 'robot_description', '-name', 'ur', '-allow_renaming', 'false']),
        OpaqueFunction(function=spawn_objects),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                        '/pp/gripper/attach@std_msgs/msg/Empty]gz.msgs.Empty',
                        '/pp/gripper/detach@std_msgs/msg/Empty]gz.msgs.Empty',
                        '/world/workcell/set_pose@ros_gz_interfaces/srv/SetEntityPose']),
        spawner('joint_state_broadcaster'),
        spawner('joint_trajectory_controller'),
        spawner('scaled_joint_trajectory_controller', '--inactive'),
        Node(package='rviz2', executable='rviz2', output='log', condition=IfCondition(rviz),
             arguments=['-d', PathJoinSubstitution([FindPackageShare('ur_description'), 'rviz', 'view_robot.rviz'])],
             parameters=[{'use_sim_time': True}]),
    ])
