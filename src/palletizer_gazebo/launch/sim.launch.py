"""Palletizing cell simulation: Gazebo world + UR10e + controllers + bridge + TF + RViz.

Structure follows ur_simulation_gz/launch/ur_sim_control.launch.py.

Args:
  gui:=true|false          Gazebo GUI (default true; false = headless server, headless rendering)
  rviz:=true|false         RViz (defaults to the value of gui)
  cell_config:=<path>      cell_config.yaml (default: palletizer_bringup/config/cell_config.yaml)
  rtf:=<float>             Gazebo real-time factor (default 1.0)
  world:=plain|warehouse   plain cell (default) or the cell inside the Blender-built warehouse
"""

import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_cell_config():
    # palletizer_bringup depends on this package, so look it up at runtime instead of
    # declaring a dependency (which would create a cycle).
    try:
        return os.path.join(
            get_package_share_directory('palletizer_bringup'), 'config', 'cell_config.yaml')
    except Exception:  # noqa: BLE001
        return ''


def _static_tf(name, xyz, rpy, parent, child):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        output='log',
        arguments=[
            '--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
            '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
            '--frame-id', parent, '--child-frame-id', child,
        ],
        parameters=[{'use_sim_time': True}],
    )


def launch_setup(context, *args, **kwargs):
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    rviz_arg = LaunchConfiguration('rviz').perform(context).lower()
    rviz = gui if rviz_arg == '' else rviz_arg == 'true'
    cell_config = LaunchConfiguration('cell_config').perform(context)
    if not cell_config or not os.path.isfile(cell_config):
        raise RuntimeError(f'cell_config not found: "{cell_config}"')

    with open(cell_config) as f:
        cell = yaml.safe_load(f)

    gz_share = get_package_share_directory('palletizer_gazebo')
    desc_share = get_package_share_directory('palletizer_description')

    # Render world and robot from the single config file.
    out_dir = os.path.join(tempfile.gettempdir(), 'palletizer')
    os.makedirs(out_dir, exist_ok=True)
    world_file = os.path.join(out_dir, 'palletizing_cell.sdf')
    world_xml = xacro.process_file(
        os.path.join(gz_share, 'worlds', 'palletizing_cell.sdf.xacro'),
        mappings={'cell_config': cell_config,
                  'real_time_factor': LaunchConfiguration('rtf').perform(context),
                  'environment': LaunchConfiguration('world').perform(context)}).toxml()
    with open(world_file, 'w') as f:
        f.write(world_xml)

    robot_description = xacro.process_file(
        os.path.join(desc_share, 'urdf', 'palletizer.urdf.xacro'),
        mappings={
            'cell_config': cell_config,
            'simulation_controllers': os.path.join(desc_share, 'config', 'controllers.yaml'),
        }).toxml()

    gz_args = f'-r -v 3 {world_file}' if gui else f'-s -r --headless-rendering -v 3 {world_file}'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_description, '-name', 'palletizer', '-allow_renaming', 'false'],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='log',
        parameters=[{
            'config_file': os.path.join(gz_share, 'config', 'bridge.yaml'),
            'use_sim_time': True,
        }],
    )

    cloud_relay = Node(
        package='palletizer_tools',
        executable='cloud_relay',
        output='log',
        parameters=[{'use_sim_time': True}],
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager',
                   '--controller-manager-timeout', '120'],
        output='screen',
    )
    jtc_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '-c', '/controller_manager',
                   '--controller-manager-timeout', '120'],
        output='screen',
    )
    # Start the arm controller once the state broadcaster is up.
    jtc_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[jtc_spawner]))

    # Static cell frames, all from cell_config.yaml.
    # (the pallet frame is published by pallet_perception from the MEASURED pallet)
    cam = cell['camera']
    pcam = cell['pallet_camera']
    static_tfs = [
        _static_tf('pick_zone_tf', cell['pick_zone']['xyz'], [0, 0, 0], 'world', 'pick_zone'),
        _static_tf('overhead_camera_tf', cam['xyz'], cam['rpy'], 'world', 'overhead_camera_link'),
        # REP-103 optical frame: z forward, x right, y down (relative to camera link x forward).
        _static_tf('overhead_camera_optical_tf', [0, 0, 0], [-1.5707963, 0, -1.5707963],
                   'overhead_camera_link', 'overhead_camera_optical_frame'),
        _static_tf('pallet_camera_tf', pcam['xyz'], pcam['rpy'], 'world', 'pallet_camera_link'),
        _static_tf('pallet_camera_optical_tf', [0, 0, 0], [-1.5707963, 0, -1.5707963],
                   'pallet_camera_link', 'pallet_camera_optical_frame'),
    ]

    # model:// lookup for the warehouse (also exported by the package's environment hook)
    resources = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(gz_share, 'models'))

    nodes = [resources, gazebo, spawn_robot, robot_state_publisher, bridge, cloud_relay,
             jsb_spawner, jtc_after_jsb]
    nodes += static_tfs

    if rviz:
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', os.path.join(gz_share, 'rviz', 'cell.rviz')],
            parameters=[{'use_sim_time': True}],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start the Gazebo GUI (false: headless)'),
        DeclareLaunchArgument('rviz', default_value='',
                              description='Start RViz (default: same as gui)'),
        DeclareLaunchArgument('rtf', default_value='1.0',
                              description='Gazebo real_time_factor (0 = as fast as possible)'),
        DeclareLaunchArgument('world', default_value='plain', choices=['plain', 'warehouse'],
                              description='plain cell or the cell inside the warehouse'),
        DeclareLaunchArgument('cell_config', default_value=_default_cell_config(),
                              description='Path to cell_config.yaml'),
        OpaqueFunction(function=launch_setup),
    ])
