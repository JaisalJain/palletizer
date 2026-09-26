"""Top-level launch for the palletizing cell (headless by default).

  mode:=manual      simulation + perception (conveyor + pallet camera) + packing + MoveIt/motion
                    + spawner/conveyor + pallet conveyor (a first pallet is delivered)
                    (drive it with services/actions, or run tests)
  mode:=mixed       everything above + BehaviorTree orchestrator + metrics logger: autonomous
                    palletizing of random boxes with the v1 planners (planner:=heuristic|naive)
                    (mode:=auto is an alias)
  mode:=single_sku  autonomous single-SKU palletizing with layer patterns; the spawner sends
                    sku:=<catalog key | LxWxH mm> or order:="<sku>:<n>, <sku>:<n>"

Args: gui:=true|false (default false: headless)  rviz:=true|false (default: gui)
      planner:=heuristic|naive  seed:=<int> (-1 random)  spawn_rate:=<attempts/s>  rtf:=<float>
      max_pallets:=<int> (0 = forever)  finish_with_next_box:=true|false (after max_pallets,
      also place the first box of the next pallet)  run_name:=  results_dir:=
      sku:=  order:=  pallets:="EUR,HALF"  exchange_mode:=auto|operator
      pattern_priority:=count|interlock  packing:=mixed|single_sku (manual mode)
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
    mode = {'auto': 'mixed'}.get(lc('mode'), lc('mode'))
    auto = mode in ('mixed', 'single_sku')
    packing_mode = 'single_sku' if mode == 'single_sku' else \
        (lc('packing') if mode == 'manual' else 'mixed')
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
    spawner = {'rate': float(spawn_rate), 'seed': seed, 'sku': lc('sku'), 'order': lc('order')}
    exchanger = {'exchange_mode': lc('exchange_mode'), 'initial': not auto,
                 'seed': seed}
    if lc('pallets'):
        exchanger['pallets'] = lc('pallets')
    packing = {'mode': packing_mode, 'planner': planner}
    if lc('pattern_priority'):
        packing['pattern_priority'] = lc('pattern_priority')
    actions = [
        sim,
        node('palletizer_tools', 'conveyor_controller'),
        node('palletizer_tools', 'sim_ops'),
        node('palletizer_tools', 'pallet_exchanger', exchanger),
        node('palletizer_perception', 'box_perception'),
        node('palletizer_perception', 'pallet_perception'),
        node('palletizer_packing', 'packing_node', packing),
        node('palletizer_orchestrator', 'status_overlay'),
        TimerAction(period=5.0, actions=[move_group, node('palletizer_motion', 'motion_server')]),
        # Give Gazebo time to load the world before the first spawn.
        TimerAction(period=8.0, actions=[node('palletizer_tools', 'box_spawner', spawner)]),
    ]
    if auto:
        tag = lc('sku') or ('order' if lc('order') else planner)
        run_name = lc('run_name') or f'{mode}_{tag}_seed{seed}'
        orchestrator = node('palletizer_orchestrator', 'orchestrator', {
            'tree_file': os.path.join(get_package_share_directory('palletizer_orchestrator'),
                                      'trees', 'palletize.xml'),
            'mode': mode,
            'planner': 'pattern' if mode == 'single_sku' else planner,
            'max_pallets': int(lc('max_pallets')),
            'finish_with_next_box': lc('finish_with_next_box') == 'true',
        })
        metrics = node('palletizer_orchestrator', 'metrics_logger', {
            'results_dir': lc('results_dir'), 'run_name': run_name,
            'planner': 'pattern' if mode == 'single_sku' else planner, 'seed': seed,
            'mode': mode})
        actions += [metrics, TimerAction(period=10.0, actions=[orchestrator])]
        if int(lc('max_pallets')) > 0:
            # headless benchmark runs end when the orchestrator finishes
            actions.append(RegisterEventHandler(OnProcessExit(
                target_action=orchestrator,
                on_exit=[TimerAction(period=3.0, actions=[EmitEvent(event=Shutdown(
                    reason='orchestrator finished'))])])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='manual',
                              choices=['manual', 'mixed', 'single_sku', 'auto']),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value=''),
        DeclareLaunchArgument('planner', default_value='heuristic',
                              choices=['heuristic', 'naive']),
        DeclareLaunchArgument('packing', default_value='mixed', choices=['mixed', 'single_sku'],
                              description='packing mode in manual mode'),
        DeclareLaunchArgument('seed', default_value='-1'),
        DeclareLaunchArgument('spawn_rate', default_value='',
                              description='spawn attempts/s (default: 0 manual, config auto)'),
        DeclareLaunchArgument('rtf', default_value='1.0'),
        DeclareLaunchArgument('max_pallets', default_value='0'),
        DeclareLaunchArgument('finish_with_next_box', default_value='false'),
        DeclareLaunchArgument('run_name', default_value=''),
        DeclareLaunchArgument('results_dir', default_value=os.path.expanduser(
            '~/palletizer_ws/results')),
        DeclareLaunchArgument('sku', default_value=''),
        DeclareLaunchArgument('order', default_value=''),
        DeclareLaunchArgument('pallets', default_value=''),
        DeclareLaunchArgument('exchange_mode', default_value='auto',
                              choices=['auto', 'operator']),
        DeclareLaunchArgument('pattern_priority', default_value=''),
        OpaqueFunction(function=setup),
    ])
