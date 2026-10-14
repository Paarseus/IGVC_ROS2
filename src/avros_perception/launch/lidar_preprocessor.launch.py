"""
Launch the LiDAR preprocessor (feeds /perception/lidar/obstacles to Nav2 STVL).

Drivers, static TFs and the Velodyne sensor setup live in avros_bringup
(sensors.launch.py); this file starts only the preprocessing node.
"""

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('avros_perception'), 'config', 'lidar.yaml')

    # CycloneDDS like the rest of the stack (mixing RMWs breaks services/parameters).
    # The shared config lives in avros_bringup; skip it if that package is absent.
    dds = [SetEnvironmentVariable(
        name='RMW_IMPLEMENTATION', value='rmw_cyclonedds_cpp')]
    try:
        dds.append(SetEnvironmentVariable(
            name='CYCLONEDDS_URI',
            value='file://' + os.path.join(
                get_package_share_directory('avros_bringup'),
                'config', 'cyclonedds.xml')))
    except PackageNotFoundError:
        pass

    return LaunchDescription(dds + [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'lidar_input_topic', default_value='/velodyne_points',
            description="lidar_preprocessor input. Webots publishes "
                        "'/velodyne_points/point_cloud' (suffix added by "
                        'webots_ros2_driver)'),
        Node(
            package='avros_perception',
            executable='lidar_preprocessor',
            name='lidar_preprocessor',
            parameters=[config, {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_topic': LaunchConfiguration('lidar_input_topic'),
            }],
            # Tiny matrices: numpy's BLAS thread pool only adds contention on a busy CPU.
            additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'},
            output='screen',
        ),
    ])
