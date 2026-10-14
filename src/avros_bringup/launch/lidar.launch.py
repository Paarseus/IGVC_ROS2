"""
Individual LiDAR launch: see the STVL 3D voxels in RViz without the full Nav2 stack.

Starts:
  - sensors.launch.py with only the Velodyne enabled (optional: enable_sensors)
  - avros_perception lidar_preprocessor -> /perception/lidar/obstacles
  - static odom->base_link and map->odom stand-ins (robot is stationary)
  - Nav2 controller_server (creates the local costmap incl. the STVL layer) using
    the SAME nav2_params as the real run, plus config/lidar_overrides.yaml
    (only drops the ZED semantic layer, which needs the cameras)
  - RViz (lidar.rviz): obstacle cloud, /local_costmap/voxel_grid, costmap

Do not run together with localization.launch.py / navigation.launch.py: the static
odom->base_link here would fight the EKF. Use enable_sensors:=false when the
Velodyne driver is already running (never run two drivers).

Usage:
  ros2 launch avros_bringup lidar.launch.py
  ros2 launch avros_bringup lidar.launch.py enable_sensors:=false rviz_display:=:1001
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _rviz(context, *_args, **_kwargs):
    cfg = context.launch_configurations
    if cfg['rviz'].lower() != 'true':
        return []
    env = {}
    if cfg['rviz_display']:
        env['DISPLAY'] = cfg['rviz_display']
    if cfg['software_gl'].lower() == 'true':
        env.update({
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'GALLIUM_DRIVER': 'llvmpipe',
            'QT_X11_NO_MITSHM': '1',
        })
    rviz_config = os.path.join(
        get_package_share_directory('avros_bringup'), 'rviz', 'lidar.rviz')
    return [Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        additional_env=env,
        output='screen',
    )]


def generate_launch_description():
    pkg_dir = get_package_share_directory('avros_bringup')
    perception_dir = get_package_share_directory('avros_perception')
    use_sim_time = LaunchConfiguration('use_sim_time')
    cyclonedds_file = os.path.join(pkg_dir, 'config', 'cyclonedds.xml')

    nav2_params = os.path.join(
        pkg_dir, 'config', 'nav2_params_igvc_autonav.yaml')
    overrides = os.path.join(pkg_dir, 'config', 'lidar_overrides.yaml')

    return LaunchDescription([
        # Same DDS setup as every other avros_bringup launch: CLI tools and the other
        # nodes are on CycloneDDS, and mixing RMWs breaks services (parameters).
        SetEnvironmentVariable(
            name='RMW_IMPLEMENTATION', value='rmw_cyclonedds_cpp'),
        SetEnvironmentVariable(
            name='CYCLONEDDS_URI', value='file://' + cyclonedds_file),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use simulation clock'),
        DeclareLaunchArgument(
            'enable_sensors', default_value='true',
            description='Start sensors.launch.py (Velodyne driver, URDF TF). '
                        'false if the drivers are already running'),
        DeclareLaunchArgument(
            'enable_ntrip', default_value='false',
            description='Start the NTRIP client (RTK corrections for the Xsens GNSS)'),
        DeclareLaunchArgument(
            'lidar_input_topic', default_value='/velodyne_points',
            description="lidar_preprocessor input. Webots: '/velodyne_points/point_cloud'"),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Launch RViz'),
        DeclareLaunchArgument(
            'rviz_display', default_value='',
            description='X display for RViz (e.g. :1001); empty = $DISPLAY'),
        DeclareLaunchArgument(
            'software_gl', default_value='true',
            description='Software OpenGL for RViz (NoMachine displays)'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'sensors.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'enable_ntrip': LaunchConfiguration('enable_ntrip'),
                'enable_velodyne': 'true',
            }.items(),
            condition=IfCondition(LaunchConfiguration('enable_sensors')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(perception_dir, 'launch', 'lidar_preprocessor.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'lidar_input_topic': LaunchConfiguration('lidar_input_topic'),
            }.items(),
        ),

        # Stationary-robot stand-ins for the EKF transforms.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='bench_static_odom',
             arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='bench_static_map',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom']),

        # controller_server creates the local costmap (and STVL) internally.
        # Later parameter files override earlier ones key by key.
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=[nav2_params, overrides, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_lidar', output='screen',
             parameters=[{'autostart': True, 'bond_timeout': 0.0,
                          'use_sim_time': use_sim_time,
                          'node_names': ['controller_server']}]),

        OpaqueFunction(function=_rviz),
    ])
