"""Top-level launch: simulation + box spawner + conveyor controller.

Args: gui:=true|false, rviz:=true|false (default: gui), spawn_rate:=<boxes/s> (0 = on demand)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cell_config = os.path.join(
        get_package_share_directory('palletizer_bringup'), 'config', 'cell_config.yaml')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('palletizer_gazebo'), 'launch', 'sim.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'cell_config': cell_config,
        }.items(),
    )

    common = {'use_sim_time': True, 'cell_config': cell_config}
    conveyor = Node(
        package='palletizer_tools', executable='conveyor_controller',
        output='screen', parameters=[common])
    spawner = Node(
        package='palletizer_tools', executable='box_spawner',
        output='screen',
        parameters=[common, {'rate': LaunchConfiguration('spawn_rate'),
                             'seed': LaunchConfiguration('seed')}])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value=''),
        DeclareLaunchArgument('spawn_rate', default_value='0.5',
                              description='Spawn attempts/s (0 = only via ~/spawn_box)'),
        DeclareLaunchArgument('seed', default_value='-1',
                              description='Box sequence seed (-1 = random)'),
        sim,
        conveyor,
        # Give Gazebo time to load the world before the first spawn.
        TimerAction(period=8.0, actions=[spawner]),
    ])
