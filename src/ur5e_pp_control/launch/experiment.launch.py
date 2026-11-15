"""Run one experiment (N pick-and-place cycles with one controller).

Start the simulation first (sim.launch.py), then for example:
    ros2 launch ur5e_pp_control experiment.launch.py mode:=scaled cycles:=30
    ros2 launch ur5e_pp_control experiment.launch.py mode:=trajectory speed:=0.20 start_offset_mm:=5 tag:=traj_varied
    ros2 launch ur5e_pp_control experiment.launch.py planner:=energy_opt     # the new controller
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = {
        'mode': ('scaled', 'scaled (UR default) | trajectory', str),
        'cycles': ('30', 'number of pick-and-place cycles', int),
        'speed': ('0.15', 'peak TCP speed in m/s (max 0.20)', float),
        'start_offset_mm': ('0.0', 'start pose offset from the pick pose', float),
        'payload_kg': ('1.2', 'gripper + box mass (recorded in the results)', float),
        'use_gripper': ('true', 'pick and place the real box (false: no box)', bool),
        'planner': ('baseline', 'baseline (straight path) | energy_opt (new energy-optimal controller)', str),
        'scenario': ('box', 'box (original task) | cans (can-packing line, start sim.launch.py scenario:=cans)', str),
        'warm_start': ('', 'energy_opt: start from the paths learned in an earlier run (its results path)', str),
        'tag': ('', 'results file name (default: mode)', str),
        'results_dir': ('~/ur5e_pp_ws/results', 'where CSV/JSON results go', str),
    }
    decls = [DeclareLaunchArgument(k, default_value=v[0], description=v[1]) for k, v in args.items()]
    params = {k: ParameterValue(LaunchConfiguration(k), value_type=v[2]) for k, v in args.items()}
    params['use_sim_time'] = True
    runner = Node(package='ur5e_pp_control', executable='pp_runner', output='screen', parameters=[params])
    return LaunchDescription(decls + [
        DeclareLaunchArgument('shutdown_when_done', default_value='true',
                              description='end the launch when the runner finishes (false: keep e.g. the demo open)'),
        runner,
        RegisterEventHandler(OnProcessExit(target_action=runner, on_exit=[EmitEvent(event=Shutdown())]),
                             condition=IfCondition(LaunchConfiguration('shutdown_when_done'))),
    ])
