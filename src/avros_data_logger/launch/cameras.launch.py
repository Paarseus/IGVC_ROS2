"""Launch the two USB camera ROS bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('avros_data_logger'), 'config',
        'cameras.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('capture_rate_hz', default_value='30.0'),
        Node(
            package='avros_data_logger',
            executable='avros_camera_bridge',
            name='avros_camera_bridge',
            parameters=[config, {
                'capture_rate_hz': LaunchConfiguration('capture_rate_hz'),
            }],
            output='screen'),
    ])
