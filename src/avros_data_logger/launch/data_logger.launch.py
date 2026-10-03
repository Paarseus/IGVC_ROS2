"""Launch the AVROS data logger and optional browser preview."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rgb_camera_index', default_value='-1'),
        DeclareLaunchArgument('thermal_camera_index', default_value='-1'),
        DeclareLaunchArgument('episode_id', default_value='1'),
        DeclareLaunchArgument('output_directory', default_value='data_logger'),
        DeclareLaunchArgument('capture_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('web_preview', default_value='true'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument(
            'web_bind_address', default_value='127.0.0.1'),
        Node(
            package='avros_data_logger',
            executable='avros_dataLogger',
            name='avros_dataLogger',
            output='screen',
            parameters=[{
                'rgb_camera_index': LaunchConfiguration('rgb_camera_index'),
                'thermal_camera_index': LaunchConfiguration('thermal_camera_index'),
                'episode_id': LaunchConfiguration('episode_id'),
                'output_directory': LaunchConfiguration('output_directory'),
                'capture_rate_hz': LaunchConfiguration('capture_rate_hz'),
                'web_preview': LaunchConfiguration('web_preview'),
                'web_port': LaunchConfiguration('web_port'),
                'web_bind_address': LaunchConfiguration('web_bind_address'),
            }],
        ),
    ])
