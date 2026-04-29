"""Localization + perception + costmap + manual drive — full pipeline minus Nav2.

Wraps:
  - localization.launch.py     (sensors + dual EKF + GPS, front camera only)
  - perception.launch.py       (HSV pipeline on the front ZED)
  - webui.launch.py            (actuator_node + phone joystick on https://<jetson>:8000)
  - controller_server          (hosts local_costmap with kiwicampus semantic_layer)
  - lifecycle_manager          (autostarts controller_server)
  - foxglove_bridge            (laptop-side viz on ws://<jetson>:8765)

Compared to perception_test.launch.py:
  Uses real EKF localization instead of identity static TFs, so the robot
  actually translates in the costmap as you drive. Use perception_test
  for HSV calibration when the robot is parked; this one for actual drive
  testing.

Compared to navigation.launch.py:
  Skips planner/smoother/route_server/BT/behaviors. Driver is the human
  on the phone joystick, not Nav2. Use navigation.launch.py once you want
  autonomous goal-following.

Usage:
  ros2 launch avros_bringup localization_perception_test.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('avros_bringup')
    nav2_config = os.path.join(pkg_dir, 'config', 'perception_test_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'
        ),

        # Localization — sensors + EKF + GPS, front camera only
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'localization.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'enable_velodyne': 'false',
                'enable_realsense': 'false',
                'enable_zed_front': 'true',
            }.items(),
        ),

        # Perception — HSV pipeline (default in perception.yaml) on front ZED
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('avros_perception'),
                '/launch/perception.launch.py',
            ]),
            launch_arguments={
                'cameras': 'front',
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        # WebUI — actuator_node + webui_node (phone joystick at https://<jetson>:8000)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'webui.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # controller_server hosts local_costmap with semantic_layer + inflation_layer
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[nav2_config, {'use_sim_time': use_sim_time}],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),

        # Lifecycle manager — autostarts controller_server (configure -> activate)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization_perception_test',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['controller_server'],
            }],
            output='screen',
        ),

        # Foxglove bridge — connect from Studio at ws://<jetson-ip>:8765
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',
                'tls': False,
                'use_compression': False,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
    ])
