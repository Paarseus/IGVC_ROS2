"""Launch costmap test: sensors + static TF + Nav2 controller_server.

Tests the local costmap without localization or the full Nav2 stack.
Uses controller_server which creates the local costmap internally.
Static odom/map TFs stand in until EKF and navsat are ready.

Usage:
  # Live sensors + costmap + RViz
  ros2 launch avros_bringup costmap_test.launch.py

  # Headless
  ros2 launch avros_bringup costmap_test.launch.py rviz:=false

  # Bag playback (disable live drivers, use bag clock, start lane detector):
  #   Terminal 1: ros2 bag play <bag> --clock --loop
  #   Terminal 2:
  ros2 launch avros_bringup costmap_test.launch.py \\
      use_sim_time:=true \\
      enable_velodyne:=false \\
      enable_realsense:=false \\
      enable_lane_detector:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('avros_bringup')
    cyclonedds_file = os.path.join(pkg_dir, 'config', 'cyclonedds.xml')

    # Select distro-specific Nav2 params (matches navigation.launch.py).
    # Humble and Jazzy have different plugin naming and param schemas;
    # notably, always_send_full_costmap lives only in the Humble file,
    # so using the wrong one leaves /local_costmap/costmap as a one-shot
    # transient_local publisher and breaks downstream consumers.
    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    if ros_distro == 'humble':
        nav2_config = os.path.join(pkg_dir, 'config', 'nav2_params_humble.yaml')
    else:
        nav2_config = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'costmap_test.rviz')
    lane_detector_config = os.path.join(
        pkg_dir, 'config', 'lane_detector_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_lane_detector = LaunchConfiguration('enable_lane_detector')

    return LaunchDescription([
        # Set RMW explicitly at this level (defensive). The included
        # sensors.launch.py sets the same values, but relying on that
        # side-effect leaves controller_server briefly exposed to the
        # FastDDS default if the launch traversal order changes.
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
            description='Use simulation clock (set true for bag playback)'
        ),

        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz (set false for headless)'
        ),

        DeclareLaunchArgument(
            'enable_velodyne', default_value='true',
            description='Enable live Velodyne driver (set false for bag playback)'
        ),

        DeclareLaunchArgument(
            'enable_realsense', default_value='true',
            description='Enable live RealSense driver (set false for bag playback)'
        ),

        DeclareLaunchArgument(
            'enable_lane_detector', default_value='false',
            description='Start lane_detector_node (needed to see lane cells in costmap)'
        ),

        # Sensors: robot_state_publisher + velodyne + realsense + xsens
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'sensors.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'enable_ntrip': 'false',
                'enable_velodyne': LaunchConfiguration('enable_velodyne'),
                'enable_realsense': LaunchConfiguration('enable_realsense'),
            }.items(),
        ),

        # Static odom -> base_link (replaced by EKF later)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_publisher',
            arguments=[
                '--frame-id', 'odom',
                '--child-frame-id', 'base_link',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # Static map -> odom (replaced by navsat_transform later)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_publisher',
            arguments=[
                '--frame-id', 'map',
                '--child-frame-id', 'odom',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # Lane detector (optional; required to see lane cells in costmap)
        Node(
            package='avros_perception',
            executable='lane_detector_node',
            name='lane_detector_node',
            parameters=[
                lane_detector_config,
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
            condition=IfCondition(enable_lane_detector),
        ),

        # controller_server creates and manages the local costmap internally
        # Publishes /local_costmap/costmap from sensor data
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[
                nav2_config,
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),

        # Lifecycle manager activates controller_server (and its costmap)
        # bond_timeout 0.0: required for standalone use without full Nav2
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap',
            parameters=[{
                'autostart': True,
                'bond_timeout': 0.0,
                'node_names': ['controller_server'],
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),

        # RViz (disable with rviz:=false for headless)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
