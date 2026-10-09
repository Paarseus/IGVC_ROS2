"""Launch web UI with actuator_node.

Launches:
  - actuator_node (ActuatorCommand/cmd_vel -> Teensy serial -> SparkMAX)
  - webui_node (phone WebSocket -> ActuatorCommand)

Phone: open https://<jetson-ip>:8000

Note: heading-hold activates if /imu/data is publishing. Launch
sensors.launch.py alongside to get the Xsens feed; otherwise the node
passes webui throttle/steer through without IMU correction.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('avros_bringup')
    actuator_config = os.path.join(pkg_dir, 'config', 'actuator_params.yaml')
    webui_config = os.path.join(pkg_dir, 'config', 'webui_params.yaml')
    cyclonedds_file = os.path.join(pkg_dir, 'config', 'cyclonedds.xml')

    return LaunchDescription([
        # Force CycloneDDS so the webui-driven actuator interops with the
        # sensor stack (sees /imu/data from Xsens, etc.).
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

        # Actuator bridge node
        Node(
            package='avros_control',
            executable='actuator_node',
            name='actuator_node',
            parameters=[
                actuator_config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            output='screen',
        ),

        # Web UI node
        Node(
            package='avros_webui',
            executable='webui_node',
            name='webui_node',
            parameters=[
                webui_config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            output='screen',
        ),
    ])
