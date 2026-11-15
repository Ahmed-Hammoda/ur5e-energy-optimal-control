"""Presentation demo of the can-packing line: Gazebo (GUI) + the energy monitor + one run.

    ros2 launch ur5e_pp_control demo.launch.py version:=before        # existing controller (baseline)
    ros2 launch ur5e_pp_control demo.launch.py version:=after         # new energy-optimal controller,
                                                                      # with the "before" run drawn behind it
    ros2 launch ur5e_pp_control demo.launch.py version:=after replay:=true replay_speed:=2.0
                                                                      # no simulation: replay the recorded runs

Options: cans:=12 (one carton), style:=video (monitor for a screen recording), replay_speed:=1.0, compare:=<results run>
(the "before" run shown in the monitor; default: the recorded 36-can baseline),
learned:=true (after: start from the learned paths in results/cans/learned.json,
so the robot is optimal from the first can; false: watch it learn from scratch).
Live runs are saved to ~/ur5e_pp_ws/results/demo/demo_<version>.csv / .json.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

RESULTS = os.path.expanduser('~/ur5e_pp_ws/results/cans')
RECORDED = {'before': os.path.join(RESULTS, 'cans_baseline'), 'after': os.path.join(RESULTS, 'cans_after')}
LEARNED = os.path.join(RESULTS, 'learned')     # paths learned offline: ros2 run ur5e_pp_control learn_paths


def setup(context):
    version = LaunchConfiguration('version').perform(context)
    if version not in RECORDED:
        raise RuntimeError("version must be 'before' or 'after'")
    replay = LaunchConfiguration('replay').perform(context).lower() == 'true'
    compare = LaunchConfiguration('compare').perform(context)
    if compare == 'auto':
        compare = RECORDED['before'] if version == 'after' else ''
    label = {'before': 'BEFORE: existing controller (straight paths)',
             'after': 'AFTER: energy-optimal controller'}[version]
    monitor = Node(package='ur5e_pp_control', executable='energy_monitor', output='screen',
                   parameters=[{'scenario': 'cans', 'label': label, 'compare': compare,
                                'style': LaunchConfiguration('style').perform(context),
                                'replay': RECORDED[version] if replay else '',
                                'replay_speed': float(LaunchConfiguration('replay_speed').perform(context)),
                                'use_sim_time': not replay}])
    if replay:
        return [monitor]
    pkg = FindPackageShare('ur5e_pp_control')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', 'sim.launch.py'])),
        launch_arguments={'scenario': 'cans', 'box_mass': '0.35', 'gui': 'true',
                          'fast': LaunchConfiguration('fast')}.items())
    run = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', 'experiment.launch.py'])),
        launch_arguments={'scenario': 'cans', 'payload_kg': '1.05',
                          'planner': 'baseline' if version == 'before' else 'energy_opt',
                          'cycles': LaunchConfiguration('cans').perform(context),
                          'tag': f'demo_{version}',
                          'shutdown_when_done': 'false',
                          'warm_start': LEARNED if version == 'after' and
                          LaunchConfiguration('learned').perform(context).lower() == 'true' else '',
                          'results_dir': os.path.expanduser('~/ur5e_pp_ws/results/demo')}.items())
    return [sim, monitor, run]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('version', default_value='before', description='before | after'),
        DeclareLaunchArgument('replay', default_value='false',
                              description='true: no simulation, replay the recorded run in the monitor'),
        DeclareLaunchArgument('cans', default_value='12', description='cans to pack (12 = one carton)'),
        DeclareLaunchArgument('fast', default_value='true',
                              description='2 ms physics step and no shadows, so it runs closer to real time'),
        DeclareLaunchArgument('learned', default_value='true',
                              description='after: start from the learned paths (false: learn live from scratch)'),
        DeclareLaunchArgument('style', default_value='detail',
                              description='monitor look: detail (engineering view) | video (large, for a screen recording)'),
        DeclareLaunchArgument('replay_speed', default_value='1.0', description='replay speed (2.0 = twice as fast)'),
        DeclareLaunchArgument('compare', default_value='auto',
                              description='results run drawn as "before" in the monitor (auto: the recorded baseline)'),
        OpaqueFunction(function=setup),
    ])
