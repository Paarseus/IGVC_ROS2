"""Launch Webots simulation with the LiDAR preprocessing + STVL voxel bench + RViz.

Exercises avros_perception's lidar_preprocessor and the bench STVL costmap
(3D voxels) end-to-end without a real Velodyne attached. Includes:
  - Base sim.launch.py (Webots cpp_campus world + vehicle + robot_state_publisher,
    which publishes the full TF tree incl. base_link->velodyne/imu_link)
  - avros_bringup lidar.launch.py (preprocessor + bench costmap + RViz), with
    enable_sensors:=false (no real Velodyne driver / duplicate TF -- Webots
    already provides both) and lidar_input_topic pointed at Webots' LiDAR topic
  - RViz (avros_bringup lidar.rviz), showing /perception/lidar/obstacles and
    the STVL /local_costmap/voxel_grid

Webots' Lidar device plugin appends /point_cloud to the configured topic
name, so /velodyne_points -> /velodyne_points/point_cloud.

NOTE: the bench costmap publishes static odom->base_link / map->odom. If
sim.launch.py also publishes odom->base_link (sim odometry), the two will
conflict: use avros_perception's lidar.launch.py directly with the sim's own
Nav2 costmap instead of lidar.launch.py.

Usage:
  ros2 launch avros_sim sim_lidar.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_pkg = get_package_share_directory('avros_sim')
    bringup_pkg = get_package_share_directory('avros_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Launch RViz'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_pkg, 'launch', 'sim.launch.py')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_pkg, 'launch', 'lidar.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'enable_sensors': 'false',
                'rviz': LaunchConfiguration('rviz'),
                'lidar_input_topic': '/velodyne_points/point_cloud',
            }.items(),
        ),
    ])
