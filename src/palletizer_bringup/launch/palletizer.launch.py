"""Top-level launch for the palletizing cell.

  mode:=manual   simulation + perception + packing + MoveIt/motion + spawner/conveyor
                 (drive it with services/actions, or run tests)
  mode:=auto     everything above + BehaviorTree orchestrator + metrics logger:
                 fully autonomous palletizing

Args: gui:=true|false  rviz:=true|false (default: gui)  planner:=heuristic|naive
      seed:=<int> (box sequence, -1 random)  spawn_rate:=<attempts/s>  rtf:=<float>
      max_pallets:=<int> (auto: stop after N pallets, 0 = run forever)
      run_name:=<str> results_dir:=<dir> (metrics CSVs)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def setup(context, *args, **kwargs):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    cell_config = os.path.join(
        get_package_share_directory('palletizer_bringup'), 'config', 'cell_config.yaml')
    with open(cell_config) as f:
        cell = yaml.safe_load(f)
    auto = lc('mode') == 'auto'
    planner = lc('planner')
    seed = int(lc('seed'))
    spawn_rate = lc('spawn_rate') or str(cell['spawner']['rate'] if auto else 0.0)

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('palletizer_gazebo'), 'launch', 'sim.launch.py')),
        launch_arguments={'gui': lc('gui'), 'rviz': lc('rviz'), 'rtf': lc('rtf'),
                          'cell_config': cell_config}.items(),
    )
    common = {'use_sim_time': True, 'cell_config': cell_config}
    node = lambda pkg, exe, extra=None, **kw: Node(  # noqa: E731
        package=pkg, executable=exe, output='screen',
        parameters=[common] + ([extra] if extra else []), **kw)

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('palletizer_moveit_config'), 'launch',
            'move_group.launch.py')),
        launch_arguments={'cell_config': cell_config}.items(),
    )
    actions = [
        sim,
        node('palletizer_tools', 'conveyor_controller'),
        node('palletizer_tools', 'sim_ops'),
        node('palletizer_perception', 'box_perception'),
        node('palletizer_packing', 'packing_node', {'planner': planner}),
        TimerAction(period=5.0, actions=[move_group, node('palletizer_motion', 'motion_server')]),
        # Give Gazebo time to load the world before the first spawn.
        TimerAction(period=8.0, actions=[node('palletizer_tools', 'box_spawner',
                                              {'rate': float(spawn_rate), 'seed': seed})]),
    ]
    if auto:
        pal = cell['pallet']
        run_name = lc('run_name') or f'{planner}_seed{seed}'
        orchestrator = node('palletizer_orchestrator', 'orchestrator', {
            'tree_file': os.path.join(get_package_share_directory('palletizer_orchestrator'),
                                      'trees', 'palletize.xml'),
            'planner': planner,
            'max_pallets': int(lc('max_pallets')),
            'pallet_area': pal['size'][0] * pal['size'][1],
            'pallet_top': pal['size'][2],
            'usable_height': usable_height(cell),
        })
        metrics = node('palletizer_orchestrator', 'metrics_logger', {
            'results_dir': lc('results_dir'), 'run_name': run_name, 'planner': planner,
            'seed': seed})
        actions += [metrics, TimerAction(period=10.0, actions=[orchestrator])]
        if int(lc('max_pallets')) > 0:
            # headless benchmark runs end when the orchestrator finishes
            actions.append(RegisterEventHandler(OnProcessExit(
                target_action=orchestrator,
                on_exit=[TimerAction(period=3.0, actions=[EmitEvent(event=Shutdown(
                    reason='orchestrator finished'))])])))
    return actions


def usable_height(cell):
    import numpy as np
    path = os.path.join(get_package_share_directory('palletizer_motion'), 'config',
                        cell['reachability']['file'])
    usable = float(np.load(path)['usable_top_height'])
    return min(usable, cell['pallet']['max_stack_height'])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='manual', choices=['manual', 'auto']),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value=''),
        DeclareLaunchArgument('planner', default_value='heuristic',
                              choices=['heuristic', 'naive']),
        DeclareLaunchArgument('seed', default_value='-1'),
        DeclareLaunchArgument('spawn_rate', default_value='',
                              description='spawn attempts/s (default: 0 manual, config auto)'),
        DeclareLaunchArgument('rtf', default_value='1.0'),
        DeclareLaunchArgument('max_pallets', default_value='0'),
        DeclareLaunchArgument('run_name', default_value=''),
        DeclareLaunchArgument('results_dir', default_value=os.path.expanduser(
            '~/palletizer_ws/results')),
        OpaqueFunction(function=setup),
    ])
