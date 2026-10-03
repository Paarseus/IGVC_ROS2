"""Launch the browser-based camera test without logging."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rgb_camera_index', default_value='0'),
        DeclareLaunchArgument('thermal_camera_index', default_value='1'),
        DeclareLaunchArgument('web_port', default_value='8081'),
        Node(
            package='avros_data_logger',
            executable='avros_cameraTest',
            output='screen',
            arguments=[
                '--rgb-index', LaunchConfiguration('rgb_camera_index'),
                '--thermal-index', LaunchConfiguration('thermal_camera_index'),
                '--port', LaunchConfiguration('web_port'),
            ],
        ),
    ])
