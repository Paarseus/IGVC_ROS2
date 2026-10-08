"""Launch teleop video: MediaMTX + webrtc_node.

Launches:
  - mediamtx (stock binary + stock config from scripts/install_deps.sh)
  - webrtc_node (ZED image -> NVENC H.264 -> rtsp://127.0.0.1:8554/zed_front)

Browser (sim chair): http://<jetson-tailscale-ip>:8889/zed_front

Does NOT start the camera. Run alongside:
  ros2 launch avros_bringup sensors.launch.py enable_zed_front:=true
Set use_mediamtx:=false if MediaMTX already runs as a service.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('avros_bringup')
    webrtc_config = os.path.join(pkg_dir, 'config', 'webrtc_params.yaml')
    cyclonedds_file = os.path.join(pkg_dir, 'config', 'cyclonedds.xml')

    return LaunchDescription([
        # Same RMW as the rest of the stack so the ZED topic is visible.
        SetEnvironmentVariable(
            name='RMW_IMPLEMENTATION',
            value='rmw_cyclonedds_cpp'
        ),
        SetEnvironmentVariable(
            name='CYCLONEDDS_URI',
            value='file://' + cyclonedds_file
        ),

        DeclareLaunchArgument(
            'use_mediamtx', default_value='true',
            description='Start MediaMTX from this launch file'
        ),
        DeclareLaunchArgument(
            'mediamtx_bin', default_value='/usr/local/bin/mediamtx',
            description='MediaMTX binary'
        ),
        DeclareLaunchArgument(
            'mediamtx_config', default_value='/usr/local/etc/mediamtx.yml',
            description='MediaMTX config (stock file, unmodified)'
        ),

        ExecuteProcess(
            cmd=[LaunchConfiguration('mediamtx_bin'),
                 LaunchConfiguration('mediamtx_config')],
            name='mediamtx',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_mediamtx')),
        ),

        Node(
            package='avros_webrtc',
            executable='webrtc_node',
            name='webrtc_node',
            parameters=[webrtc_config],
            output='screen',
        ),
    ])
