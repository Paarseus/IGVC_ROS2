"""Standalone perception launcher for HSV lane detector tuning.

No Nav2, no localization, no actuator. Starts the lane detector plus a
foxglove_bridge for remote visualization. Use with either:

  # Live camera
  ros2 launch avros_bringup sensors.launch.py enable_velodyne:=false &
  ros2 launch avros_bringup perception.launch.py

  # Bag replay (use_sim_time must match the bag)
  ros2 bag play grass_noon --loop --clock --rate 0.5 &
  ros2 launch avros_bringup perception.launch.py use_sim_time:=true

Tune HSV bounds live:
  ros2 run rqt_reconfigure rqt_reconfigure
  rqt_image_view  # /lane_detector/debug_mask
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('avros_bringup')
    lane_detector_config = os.path.join(
        pkg_dir, 'config', 'lane_detector_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock (set true for bag replay)'
        ),

        Node(
            package='avros_perception',
            executable='lane_detector_node',
            name='lane_detector_node',
            parameters=[
                lane_detector_config,
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': 8765,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
    ])
