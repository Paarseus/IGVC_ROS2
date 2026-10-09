"""Dual EKF + navsat_transform for GPS-based localization.

Based on: https://github.com/ros-navigation/navigation2_tutorials
  nav2_gps_waypoint_follower_demo/launch/dual_ekf_navsat.launch.py

Launches:
  - Everything from sensors.launch.py
  - EKF #1 (odom): IMU only -> odom -> base_link (smooth, local)
  - EKF #2 (map):  IMU + GPS -> map -> odom (GPS-anchored, global)
  - navsat_transform_node: GPS -> Cartesian odometry for EKF #2

TF tree:
  map -> odom -> base_link
  (EKF map)  (EKF odom)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('avros_bringup')
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    navsat_config = os.path.join(pkg_dir, 'config', 'navsat.yaml')
    cyclonedds_file = os.path.join(pkg_dir, 'config', 'cyclonedds.xml')

    return LaunchDescription([
        # Force CycloneDDS for every node spawned from this launch (and any
        # child includes). Without this, directly-instantiated nodes (EKFs,
        # navsat_transform) come up on the shell default — usually FastDDS —
        # which doesn't interop cleanly with the CycloneDDS sensor stack and
        # corrupts action goal payloads. See CLAUDE.md known-issues.
        SetEnvironmentVariable(
            name='RMW_IMPLEMENTATION',
            value='rmw_cyclonedds_cpp'
        ),
        SetEnvironmentVariable(
            name='CYCLONEDDS_URI',
            value='file://' + cyclonedds_file
        ),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'
        ),

        DeclareLaunchArgument(
            'enable_ntrip', default_value='true',
            description='Enable NTRIP client for RTK corrections'
        ),

        DeclareLaunchArgument(
            'enable_velodyne', default_value='true',
            description='Enable Velodyne VLP-16 LiDAR'
        ),

        DeclareLaunchArgument(
            'enable_zed_front', default_value='false',
            description='Enable front ZED X camera'
        ),

        DeclareLaunchArgument(
            'enable_zed_left', default_value='false',
            description='Enable left ZED X camera'
        ),

        DeclareLaunchArgument(
            'enable_zed_right', default_value='false',
            description='Enable right ZED X camera'
        ),

        # Include sensors launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'sensors.launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'enable_ntrip': LaunchConfiguration('enable_ntrip'),
                'enable_velodyne': LaunchConfiguration('enable_velodyne'),
                'enable_zed_front': LaunchConfiguration('enable_zed_front'),
                'enable_zed_left': LaunchConfiguration('enable_zed_left'),
                'enable_zed_right': LaunchConfiguration('enable_zed_right'),
            }.items(),
        ),

        # EKF #1: Local odometry (odom -> base_link)
        # Fuses IMU only for smooth local navigation
        # Output: /odometry/filtered (used by Nav2 controller + local costmap)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            parameters=[
                ekf_config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                ('odometry/filtered', '/odometry/filtered'),
            ],
            output='screen',
        ),

        # EKF #2: Global map (map -> odom)
        # Fuses IMU + GPS for GPS-anchored global position
        # Output: /odometry/global (consumed by navsat_transform_node)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            parameters=[
                ekf_config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                ('odometry/filtered', '/odometry/global'),
            ],
            output='screen',
        ),

        # navsat_transform_node: GPS -> Cartesian odometry
        # Reads global EKF output, publishes /odometry/gps for EKF #2
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            parameters=[
                navsat_config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                ('imu/data', '/imu/data'),
                ('gps/fix', '/gnss'),
                ('gps/filtered', '/gps/filtered'),
                ('odometry/gps', '/odometry/gps'),
                ('odometry/filtered', '/odometry/global'),
            ],
            output='screen',
        ),
    ])
