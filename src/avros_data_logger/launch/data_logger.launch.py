"""Launch the ROS-topic based raw data logger."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output_directory', default_value='data_logger'),
        DeclareLaunchArgument('rgb_topic', default_value='/rgb_camera/image_raw'),
        DeclareLaunchArgument('thermal_topic', default_value='/thermal_camera/image_raw'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('pair_tolerance_ms', default_value='10.0'),
        DeclareLaunchArgument('capture_rate_hz', default_value='30.0'),
        DeclareLaunchArgument('web_preview', default_value='true'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        Node(
            package='avros_data_logger',
            executable='avros_dataLogger',
            name='avros_dataLogger',
            output='screen',
            parameters=[{
                'output_directory': LaunchConfiguration('output_directory'),
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'thermal_topic': LaunchConfiguration('thermal_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'pair_tolerance_ms': LaunchConfiguration('pair_tolerance_ms'),
                'capture_rate_hz': LaunchConfiguration('capture_rate_hz'),
                'web_preview': LaunchConfiguration('web_preview'),
                'web_port': LaunchConfiguration('web_port'),
            }],
        ),
    ])
