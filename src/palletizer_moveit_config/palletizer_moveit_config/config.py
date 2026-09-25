"""MoveItConfigs for the cell, shared by move_group and RViz launch files."""

import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def moveit_config(cell_config):
    desc = get_package_share_directory('palletizer_description')
    share = get_package_share_directory('palletizer_moveit_config')
    cfg = lambda name: os.path.join(share, 'config', name)  # noqa: E731
    return (
        MoveItConfigsBuilder('palletizer', package_name='palletizer_moveit_config')
        .robot_description(
            file_path=os.path.join(desc, 'urdf', 'palletizer.urdf.xacro'),
            mappings={'cell_config': cell_config, 'sim_gazebo': 'false'})
        .robot_description_semantic(file_path=cfg('palletizer.srdf.xacro'))
        .robot_description_kinematics(file_path=cfg('kinematics.yaml'))
        .joint_limits(file_path=cfg('joint_limits.yaml'))
        .pilz_cartesian_limits(file_path=cfg('pilz_cartesian_limits.yaml'))
        .trajectory_execution(file_path=cfg('moveit_controllers.yaml'))
        .planning_pipelines(pipelines=['pilz_industrial_motion_planner', 'ompl'],
                            default_planning_pipeline='pilz_industrial_motion_planner')
        .planning_scene_monitor(publish_robot_description=False,
                                publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
