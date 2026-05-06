"""Launch actuator_node standalone for bench testing.

Launches:
  - actuator_node (cmd_vel / ActuatorCommand -> Teensy serial -> SparkMAX,
    with IMU heading-hold on straight commands)

Subscribes: /cmd_vel, /avros/actuator_command, /imu/data
Publishes:  /avros/actuator_state, /wheel_odom
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
    cyclonedds_file = os.path.join(pkg_dir, 'config', 'cyclonedds.xml')

    return LaunchDescription([
        # Force CycloneDDS so this node interops with the sensor stack.
        # Without this, the actuator runs on the shell default (FastDDS by
        # default on Humble) and won't see /imu/data from the CycloneDDS
        # Xsens driver.
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
    ])
