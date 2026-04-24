"""
Launch one perception_node per camera.

Default is a single node bound to the front ZED. Pass `cameras:=front,left,right`
to spawn one node per camera — each owns a /perception/{cam}/* namespace.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _spawn_nodes(context, *_args, **_kwargs):
    pkg_dir = get_package_share_directory('avros_perception')
    perception_config = os.path.join(pkg_dir, 'config', 'perception.yaml')

    cameras = [c.strip() for c in context.launch_configurations['cameras'].split(',') if c.strip()]
    use_sim_time = context.launch_configurations['use_sim_time'].lower() == 'true'

    return [
        Node(
            package='avros_perception',
            executable='perception_node',
            name=f'perception_{cam}',
            parameters=[
                perception_config,
                {
                    'use_sim_time': use_sim_time,
                    'camera_name': cam,
                },
            ],
            output='screen',
        )
        for cam in cameras
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'cameras', default_value='front',
            description='Comma-separated list of ZED camera names (e.g. front,left,right)',
        ),
        # Kept for backward compatibility; ignored when `cameras` has >1 entry.
        DeclareLaunchArgument(
            'camera_name', default_value='front',
            description='(Legacy) Single camera; prefer `cameras`',
        ),
        OpaqueFunction(function=_spawn_nodes),
    ])
