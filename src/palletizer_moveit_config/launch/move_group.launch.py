"""MoveIt move_group for the palletizing cell (Pilz PTP/LIN + OMPL fallback).

Args: cell_config:=<path> (default: palletizer_bringup/config/cell_config.yaml)
Uses sim time and the joint_trajectory_controller started by palletizer_gazebo/sim.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from palletizer_moveit_config.config import moveit_config


def _default_cell_config():
    try:
        return os.path.join(
            get_package_share_directory('palletizer_bringup'), 'config', 'cell_config.yaml')
    except Exception:  # noqa: BLE001
        return ''


def launch_setup(context, *args, **kwargs):
    cfg = moveit_config(LaunchConfiguration('cell_config').perform(context))
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='log',
        parameters=[
            cfg.to_dict(),
            {
                'use_sim_time': True,
                'publish_robot_description_semantic': True,
                'monitor_dynamics': False,
                'capabilities': 'pilz_industrial_motion_planner/MoveGroupSequenceAction '
                                'pilz_industrial_motion_planner/MoveGroupSequenceService',
            },
        ],
    )
    return [move_group]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cell_config', default_value=_default_cell_config()),
        OpaqueFunction(function=launch_setup),
    ])
