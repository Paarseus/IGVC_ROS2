"""Launch the perception node for a single ZED camera.

For Phase 2 / 3 this spawns exactly one perception_node bound to the front
camera. Phase 5 extends this to a cameras-list loop for 3-camera setups.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('avros_perception')
    perception_config = os.path.join(pkg_dir, 'config', 'perception.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'
        ),

        DeclareLaunchArgument(
            'camera_name', default_value='front',
            description='Logical camera name (drives topic namespaces)'
        ),

        Node(
            package='avros_perception',
            executable='perception_node',
            name='perception_node',
            parameters=[
                perception_config,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'camera_name': LaunchConfiguration('camera_name'),
                },
            ],
            output='screen',
        ),
    ])
