"""Launch keyboard teleop with actuator_node.

Launches:
  - actuator_node (cmd_vel -> Teensy serial -> SparkMAX)
  - teleop_twist_keyboard (keyboard -> cmd_vel)

Keys: i=forward, ,=backward, j=left, l=right, k=stop, q/z=speed up/down

Note: for IMU heading-hold to be active while driving, also launch
`sensors.launch.py` in another terminal (starts the Xsens /imu/data).
Without the IMU, the node passes cmd_vel through without heading correction.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('avros_bringup')
    actuator_config = os.path.join(pkg_dir, 'config', 'actuator_params.yaml')

    return LaunchDescription([
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

        # Keyboard teleop — publishes Twist on /cmd_vel
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            parameters=[{'speed': 0.3, 'turn': 0.3}],
            prefix='xterm -e',
            output='screen',
        ),
    ])
