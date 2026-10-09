"""Launch the full LiDAR voxel-mapping stack against real sensors.

Launches:
  - base_link->velodyne / imu_link TF: static transforms with the values from
    avros_bringup's URDF (tf_source:=static, default), or robot_state_publisher
    with that URDF (tf_source:=urdf, needs zed_wrapper built for its xacro)
  - Velodyne VLP-16 driver + pointcloud transform (avros_bringup velodyne.yaml)
  - Xsens IMU driver (optional, avros_bringup xsens.yaml)
  - lidar_preprocessor + voxel_mapper_node
  - RViz (optional)

Sets CycloneDDS like avros_bringup so the nodes see the sensor topics.
Use enable_sensors:=false when the drivers/TF are already running elsewhere
(e.g. avros_bringup sensors.launch.py) -- never run two Velodyne drivers.

RViz shows on the display given by rviz_display, or on $DISPLAY when empty
(run from a terminal inside the NoMachine session).
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
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _iface_ip(iface):
    """IPv4 address currently assigned to a network interface, or None."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            packed = fcntl.ioctl(
                s.fileno(), 0x8915,  # SIOCGIFADDR
                struct.pack('256s', iface.encode()[:15]))
            return socket.inet_ntoa(packed[20:24])
        except OSError:
            return None


def _point_lidar(context, *_args, **_kwargs):
    """Make the VLP-16 stream to this machine's current IP on lidar_iface.

    The sensor sends UDP to a stored host address. When the Jetson's DHCP
    lease changes, the driver sees nothing ('poll() timeout'). Re-pointing
    the sensor at the current address on every launch avoids that.
    """
    cfg = context.launch_configurations
    if (cfg['enable_sensors'].lower() != 'true'
            or cfg['set_lidar_host'].lower() != 'true'):
        return []
    lidar_ip, iface = cfg['lidar_ip'], cfg['lidar_iface']
    host = _iface_ip(iface)
    if host is None:
        print(f'[lidar.launch] {iface} has no IPv4 address; '
              'not touching the lidar destination')
        return []
    try:
        data = urllib.parse.urlencode(
            {'addr': host, 'dport': 2368, 'tport': 8308}).encode()
        urllib.request.urlopen(
            f'http://{lidar_ip}/cgi/setting/host', data, timeout=3).read()
        print(f'[lidar.launch] lidar {lidar_ip} now streams to {host}:2368')
    except Exception as exc:  # sensor unreachable: keep launching
        print(f'[lidar.launch] could not set lidar destination: {exc}')
    return []


def _rviz(context, *_args, **_kwargs):
    if context.launch_configurations['rviz'].lower() != 'true':
        return []
    env = {}
    display = context.launch_configurations['rviz_display']
    if display:
        env['DISPLAY'] = display
    if context.launch_configurations['software_gl'].lower() == 'true':
        env.update({
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'GALLIUM_DRIVER': 'llvmpipe',
            'QT_X11_NO_MITSHM': '1',
        })
    rviz_config = os.path.join(
        get_package_share_directory('avros_lidar'), 'rviz', 'lidar.rviz')
    return [Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        additional_env=env,
        output='screen',
    )]


def generate_launch_description():
    lidar_dir = get_package_share_directory('avros_lidar')
    bringup_dir = get_package_share_directory('avros_bringup')
    params = os.path.join(lidar_dir, 'config', 'lidar_params.yaml')
    urdf_file = os.path.join(bringup_dir, 'urdf', 'avros.urdf.xacro')
    velodyne_config = os.path.join(bringup_dir, 'config', 'velodyne.yaml')
    xsens_config = os.path.join(bringup_dir, 'config', 'xsens.yaml')
    cyclonedds_file = os.path.join(bringup_dir, 'config', 'cyclonedds.xml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_sensors = LaunchConfiguration('enable_sensors')
    urdf_tf = IfCondition(PythonExpression([
        "'", enable_sensors, "' == 'true' and '",
        LaunchConfiguration('tf_source'), "' == 'urdf'"]))
    static_tf = IfCondition(PythonExpression([
        "'", enable_sensors, "' == 'true' and '",
        LaunchConfiguration('tf_source'), "' == 'static'"]))

    return LaunchDescription([
        SetEnvironmentVariable(
            name='RMW_IMPLEMENTATION', value='rmw_cyclonedds_cpp'),
        SetEnvironmentVariable(
            name='CYCLONEDDS_URI', value='file://' + cyclonedds_file),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock (bag replay)'),
        DeclareLaunchArgument(
            'enable_sensors', default_value='true',
            description='Start robot_state_publisher + Velodyne driver. Set '
                        'false if they are already running elsewhere'),
        DeclareLaunchArgument(
            'tf_source', default_value='static',
            description="'static' = fixed transforms copied from avros_bringup's "
                        "URDF; 'urdf' = robot_state_publisher with that URDF"),
        DeclareLaunchArgument(
            'set_lidar_host', default_value='true',
            description="Point the VLP-16's UDP destination at this machine's "
                        'current IP on lidar_iface before starting the driver '
                        '(survives DHCP address changes)'),
        DeclareLaunchArgument(
            'lidar_ip', default_value='192.168.13.11',
            description='VLP-16 IP address (its web interface)'),
        DeclareLaunchArgument(
            'lidar_iface', default_value='eno1',
            description='Network interface the VLP-16 is connected to'),
        DeclareLaunchArgument(
            'deskew_frame', default_value='velodyne',
            description="Velodyne per-packet deskew frame. avros_bringup's "
                        "velodyne.yaml deskews against 'odom', which needs the "
                        "localization EKF running. 'velodyne' disables deskew "
                        "(no odom needed); pass 'odom' when localization runs"),
        DeclareLaunchArgument(
            'enable_xsens', default_value='false',
            description='Start the Xsens IMU driver'),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Launch RViz'),
        DeclareLaunchArgument(
            'rviz_display', default_value='',
            description='X display for RViz (e.g. :0 or :1001); empty = $DISPLAY'),
        DeclareLaunchArgument(
            'software_gl', default_value='true',
            description='Software OpenGL for RViz (NoMachine displays)'),
        DeclareLaunchArgument(
            'voxel_resolution', default_value='0.1',
            description='Voxel size in metres'),
        DeclareLaunchArgument(
            'enable_raycasting', default_value='true',
            description='Raycast free space between sensor and hits'),
        DeclareLaunchArgument(
            'use_gpu_raycast', default_value='true',
            description='Run the raycast Bresenham walk as a CUDA kernel '
                        '(numba). Falls back to CPU automatically (with a '
                        'one-time warning) if numba/CUDA is unavailable'),
        DeclareLaunchArgument(
            'lidar_input_topic', default_value='/velodyne_points',
            description="lidar_preprocessor's input_topic override. Webots' "
                        'Lidar device plugin publishes on '
                        "'/velodyne_points/point_cloud' (suffix appended by "
                        'webots_ros2_driver), not /velodyne_points -- pass '
                        'that when feeding this stack from sim.launch.py'),

        OpaqueFunction(function=_point_lidar),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', urdf_file]), value_type=str),
                'use_sim_time': use_sim_time,
            }],
            output='screen',
            condition=urdf_tf,
        ),
        # avros.urdf.xacro: velodyne_joint (0.089, 0, xsens_height + 0.159) and
        # imu_joint (0, 0, xsens_height), xsens_height = 0.5556, both rpy 0.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_velodyne_tf',
            arguments=['--x', '0.089', '--y', '0.0', '--z', '0.7146',
                       '--frame-id', 'base_link', '--child-frame-id', 'velodyne'],
            condition=static_tf,
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.5556',
                       '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
            condition=static_tf,
        ),
        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            name='velodyne_driver_node',
            parameters=[velodyne_config, {'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(enable_sensors),
        ),
        Node(
            package='velodyne_pointcloud',
            executable='velodyne_transform_node',
            name='velodyne_transform_node',
            # velodyne.yaml swaps fixed/target on purpose (upstream bug), so
            # its 'target_frame' is the frame deskewed against at runtime.
            parameters=[velodyne_config, {
                'target_frame': LaunchConfiguration('deskew_frame'),
                'use_sim_time': use_sim_time,
            }],
            output='screen',
            condition=IfCondition(enable_sensors),
        ),
        Node(
            package='xsens_mti_ros2_driver',
            executable='xsens_mti_node',
            name='xsens_mti_node',
            parameters=[xsens_config, {'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_xsens')),
        ),

        Node(
            package='avros_lidar',
            executable='lidar_preprocessor',
            name='lidar_preprocessor',
            parameters=[params, {
                'use_sim_time': use_sim_time,
                'input_topic': LaunchConfiguration('lidar_input_topic'),
            }],
            output='screen',
        ),
        Node(
            package='avros_lidar',
            executable='voxel_mapper_node',
            name='voxel_mapper',
            parameters=[params, {
                'use_sim_time': use_sim_time,
                'enable_raycasting': LaunchConfiguration('enable_raycasting'),
                'resolution': LaunchConfiguration('voxel_resolution'),
                'use_gpu_raycast': LaunchConfiguration('use_gpu_raycast'),
            }],
            output='screen',
        ),

        OpaqueFunction(function=_rviz),
    ])
