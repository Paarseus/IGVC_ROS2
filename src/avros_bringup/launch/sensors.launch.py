"""Launch sensor drivers and robot_state_publisher for the AV2 platform.

Launches:
  - robot_state_publisher (URDF -> static TF)
  - velodyne VLP-16 driver + pointcloud transform
  - xsens MTi-680G IMU/GNSS
  - NTRIP client for RTK corrections (optional)
"""

import fcntl
import os
import socket
import struct
import urllib.parse
import urllib.request

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _iface_ip(iface):
    """IPv4 address currently assigned to a network interface, or None."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            packed = fcntl.ioctl(
                sock.fileno(), 0x8915,  # SIOCGIFADDR
                struct.pack('256s', iface.encode()[:15]))
            return socket.inet_ntoa(packed[20:24])
        except OSError:
            return None


def _point_lidar(context, *_args, **_kwargs):
    """Make the VLP-16 stream (unicast) to this machine's current IP.

    The sensor stores its UDP destination. When it is on broadcast
    (255.255.255.255) every packet arrives twice and out of order, which makes
    the driver's cut_angle logic emit thin partial scans (100+ msgs/s) instead of
    one full 360 deg scan at 10 Hz. Re-pointing it on every launch keeps it on
    unicast across sensor power cycles and DHCP address changes.
    """
    cfg = context.launch_configurations
    if (cfg['enable_velodyne'].lower() != 'true'
            or cfg['set_lidar_host'].lower() != 'true'):
        return []
    lidar_ip, iface = cfg['lidar_ip'], cfg['lidar_iface']
    host = _iface_ip(iface)
    if host is None:
        print(f'[sensors.launch] {iface} has no IPv4 address; '
              'not touching the lidar destination')
        return []
    try:
        data = urllib.parse.urlencode(
            {'addr': host, 'dport': 2368, 'tport': 8308}).encode()
        urllib.request.urlopen(
            f'http://{lidar_ip}/cgi/setting/host', data, timeout=3).read()
        print(f'[sensors.launch] lidar {lidar_ip} now streams to {host}:2368')
    except Exception as exc:  # sensor unreachable: keep launching
        print(f'[sensors.launch] could not set lidar destination: {exc}')
    return []


def generate_launch_description():
    pkg_dir = get_package_share_directory('avros_bringup')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'avros.urdf.xacro')
    cyclonedds_file = os.path.join(pkg_dir, 'config', 'cyclonedds.xml')
    velodyne_config = os.path.join(pkg_dir, 'config', 'velodyne.yaml')
    zed_front_config = os.path.join(pkg_dir, 'config', 'zed_front.yaml')
    zed_left_config = os.path.join(pkg_dir, 'config', 'zed_left.yaml')
    zed_right_config = os.path.join(pkg_dir, 'config', 'zed_right.yaml')
    xsens_config = os.path.join(pkg_dir, 'config', 'xsens.yaml')
    ntrip_config = os.path.join(pkg_dir, 'config', 'ntrip_params.yaml')

    return LaunchDescription([
        # CycloneDDS shared memory
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

        DeclareLaunchArgument(
            'enable_ntrip', default_value='true',
            description='Enable NTRIP client for RTK corrections (public '
                        'MDOT CORS caster). Default true: IGVC 2026 §I.2 '
                        'forbids running your OWN base station, but a public '
                        'NTRIP caster is permitted (user-confirmed 2026-05-30 '
                        '— get the judge ruling in writing). Set false for '
                        'unaided GPS.'
        ),

        DeclareLaunchArgument(
            'enable_velodyne', default_value='true',
            description='Enable Velodyne VLP-16 LiDAR'
        ),

        DeclareLaunchArgument(
            'set_lidar_host', default_value='true',
            description="Point the VLP-16's UDP destination (unicast) at this "
                        "machine's current IP on lidar_iface before starting the "
                        'driver. Avoids broadcast duplicates / fragmented scans.'
        ),
        DeclareLaunchArgument(
            'lidar_ip', default_value='192.168.13.11',
            description='VLP-16 IP address (its web interface)'
        ),
        DeclareLaunchArgument(
            'lidar_iface', default_value='eno1',
            description='Network interface the VLP-16 is connected to'
        ),

        DeclareLaunchArgument(
            'enable_zed_front', default_value='false',
            description='Enable front ZED X camera (requires ZED SDK + ZED Link Quad)'
        ),

        DeclareLaunchArgument(
            'enable_zed_left', default_value='false',
            description='Enable left ZED X camera (requires ZED SDK + ZED Link Quad)'
        ),

        DeclareLaunchArgument(
            'enable_zed_right', default_value='false',
            description='Enable right ZED X camera (requires ZED SDK + ZED Link Quad)'
        ),

        # robot_state_publisher: URDF -> static TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', urdf_file]), value_type=str
                ),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen',
        ),

        OpaqueFunction(function=_point_lidar),

        # Velodyne VLP-16 driver
        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            name='velodyne_driver_node',
            parameters=[velodyne_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_velodyne')),
        ),

        # Velodyne raw packets -> PointCloud2
        Node(
            package='velodyne_pointcloud',
            executable='velodyne_transform_node',
            name='velodyne_transform_node',
            parameters=[velodyne_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_velodyne')),
        ),

        # ZED X Front (GMSL via ZED Link Quad). Launched via the wrapper's own
        # launch file — zed_wrapper is a metapackage; the camera is a composable
        # component (stereolabs::ZedCamera) loaded by zed_camera.launch.py.
        #
        # Canonical args per zed-ros2-examples: pass only camera_model +
        # camera_name; wrapper sets namespace=camera_name, node_name=zed_node,
        # and our URDF provides the frame chain via zed_macro.urdf.xacro.
        # Passing `namespace` + `node_name` explicitly triggers a collision
        # (wrapper overwrites node_name with camera_name → /zed_front/zed_front/...).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('zed_wrapper'),
                '/launch/zed_camera.launch.py',
            ]),
            launch_arguments={
                'camera_model': 'zedx',
                'camera_name': 'zed_front',
                # Verified 2026-04-24 via per-port enumeration:
                #   GMSL port 0 -> SN 42569280  (physical front, confirmed by user)
                #   GMSL port 1 -> SN 49910017
                #   GMSL port 2 -> SN 43779087
                'serial_number': '42569280',
                'publish_tf': 'false',           # robot_localization owns odom→base_link
                'publish_urdf': 'false',         # our URDF already includes zed_macro
                'ros_params_override_path': zed_front_config,
            }.items(),
            condition=IfCondition(LaunchConfiguration('enable_zed_front')),
        ),

        # ZED X Left (GMSL via ZED Link Quad)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('zed_wrapper'),
                '/launch/zed_camera.launch.py',
            ]),
            launch_arguments={
                'camera_model': 'zedx',
                'camera_name': 'zed_left',
                'serial_number': '43779087',
                'publish_tf': 'false',
                'publish_urdf': 'false',
                'ros_params_override_path': zed_left_config,
            }.items(),
            condition=IfCondition(LaunchConfiguration('enable_zed_left')),
        ),

        # ZED X Right (GMSL via ZED Link Quad)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('zed_wrapper'),
                '/launch/zed_camera.launch.py',
            ]),
            launch_arguments={
                'camera_model': 'zedx',
                'camera_name': 'zed_right',
                # TODO: confirm right-camera serial (was 42569280, now repurposed to front).
                'serial_number': '49910017',
                'publish_tf': 'false',
                'publish_urdf': 'false',
                'ros_params_override_path': zed_right_config,
            }.items(),
            condition=IfCondition(LaunchConfiguration('enable_zed_right')),
        ),

        # Xsens MTi-680G IMU/GNSS
        # Package: xsens_mti_ros2_driver (built from source via avros.repos)
        # Fork: https://github.com/Paarseus/Xsens_MTi_ROS_Driver_and_Ntrip_Client (ros2 branch)
        Node(
            package='xsens_mti_ros2_driver',
            executable='xsens_mti_node',
            name='xsens_mti_node',
            parameters=[xsens_config],
            output='screen',
        ),

        # NTRIP client for RTK corrections (GPGGA -> caster -> RTCM3)
        # Package: ntrip (same fork as xsens_mti_ros2_driver — monorepo)
        Node(
            package='ntrip',
            executable='ntrip',
            name='ntrip_client',
            parameters=[ntrip_config],
            remappings=[
                ('nmea', '/nmea'),
                ('rtcm', '/rtcm'),
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_ntrip')),
        ),
    ])
