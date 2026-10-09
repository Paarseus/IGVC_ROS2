"""Launch Webots simulation with the LiDAR voxel-mapping stack + RViz.

For exercising avros_lidar's voxel_mapper (including its optional GPU/CUDA
raycasting path, see voxel_grid.py's use_gpu_raycast) end-to-end without a
real Velodyne attached. Includes:
  - Base sim.launch.py (Webots cpp_campus world + vehicle + robot_state_publisher,
    which publishes the full TF tree incl. base_link->velodyne/imu_link)
  - lidar_preprocessor + voxel_mapper_node via avros_lidar's lidar.launch.py,
    with enable_sensors:=false (no real Velodyne driver / duplicate TF --
    Webots already provides both) and lidar_input_topic pointed at Webots'
    actual LiDAR topic (see below)
  - RViz (avros_lidar's lidar.rviz config), showing /perception/lidar/voxel_markers
    and /perception/lidar/obstacle_costmap_2d

Webots' Lidar device plugin appends /point_cloud to the configured topic
name, so /velodyne_points -> /velodyne_points/point_cloud (same quirk
sim_navigation.launch.py works around for Nav2's costmaps).

Usage:
  ros2 launch avros_sim sim_lidar.launch.py
  ros2 launch avros_sim sim_lidar.launch.py use_gpu_raycast:=false  # CPU path, for comparison
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_pkg = get_package_share_directory('avros_sim')
    lidar_pkg = get_package_share_directory('avros_lidar')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gpu_raycast', default_value='true',
            description='Run voxel_mapper raycasting on the GPU (CUDA via numba)'),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Launch RViz'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_pkg, 'launch', 'sim.launch.py')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lidar_pkg, 'launch', 'lidar.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'enable_sensors': 'false',
                'use_gpu_raycast': LaunchConfiguration('use_gpu_raycast'),
                'rviz': LaunchConfiguration('rviz'),
                'lidar_input_topic': '/velodyne_points/point_cloud',
            }.items(),
        ),
    ])
