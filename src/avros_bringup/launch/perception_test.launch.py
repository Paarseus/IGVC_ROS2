"""Front camera -> perception -> semantic_segmentation_layer -> costmap.

Standalone launch for verifying the camera-only path into the local costmap.
Drops sensors we don't need for that test: no Velodyne, no RealSense, no
Xsens, no NTRIP, no EKF/navsat, no planner/route/BT/behaviors/smoother.

Optionally includes the actuator + WebUI joystick (`enable_drive:=true`,
default true) so you can drive the robot outdoors while watching the
costmap update in Foxglove. Disable with `enable_drive:=false` if you're
running webui in a separate terminal or don't want motor power.

Exposes:
  /zed_front/zed_node/...        ZED X front camera
  /perception/front/...          avros_perception (HSV pipeline by default)
  /local_costmap/costmap         semantic_layer + inflation_layer only
  /local_costmap/front/tile_map  kiwicampus internal viz
  /avros/actuator_state          when enable_drive:=true
  ws://<jetson>:8765             foxglove_bridge
  https://<jetson>:8000          webui (when enable_drive:=true)

Static TFs are published in place of EKF: map -> odom -> base_link, all
identity. base_link's URDF children (ZED frames, etc) come from
robot_state_publisher.

NOTE: with identity TFs, base_link does NOT move when you drive — odom
stays at (0,0,0). The local costmap is rolling around base_link, so it
also stays anchored at the origin. Cells appear/decay correctly as the
camera sees new things, but they don't translate as the robot moves.
For full driving with proper odometry use navigation.launch.py instead.

Usage:
  ros2 launch avros_bringup perception_test.launch.py
  # drive via phone joystick at https://<jetson-ip>:8000
  # watch costmap in Foxglove at ws://<jetson-ip>:8765
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('avros_bringup')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'avros.urdf.xacro')
    zed_front_config = os.path.join(pkg_dir, 'config', 'zed_front.yaml')
    nav2_config = os.path.join(pkg_dir, 'config', 'perception_test_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'
        ),

        DeclareLaunchArgument(
            'enable_drive', default_value='true',
            description='Include actuator_node + webui_node so you can drive '
                        'the robot from the phone joystick at '
                        'https://<jetson-ip>:8000 while testing the costmap. '
                        'Set false if you run webui.launch.py separately.'
        ),

        # robot_state_publisher: URDF -> TF (gives us the zed_front_*
        # frame chain via the embedded zed_macro.urdf.xacro)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', urdf_file]), value_type=str
                ),
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),

        # Static TFs in place of EKF/navsat: map -> odom -> base_link.
        # Costmap operates in odom; without these the rolling window
        # has nowhere to anchor.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='static_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen',
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='static_odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen',
        ),

        # ZED X front via the wrapper's own launch file.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('zed_wrapper'),
                '/launch/zed_camera.launch.py',
            ]),
            launch_arguments={
                'camera_model': 'zedx',
                'camera_name': 'zed_front',
                'serial_number': '42569280',     # GMSL port 0 (front)
                'publish_tf': 'false',
                'publish_urdf': 'false',
                'ros_params_override_path': zed_front_config,
            }.items(),
        ),

        # avros_perception — HSV pipeline by default (per perception.yaml).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('avros_perception'),
                '/launch/perception.launch.py',
            ]),
            launch_arguments={
                'cameras': 'front',
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        # controller_server hosts the local_costmap. Lifecycle managed below.
        Node(
            package='nav2_controller', executable='controller_server',
            name='controller_server',
            parameters=[nav2_config, {'use_sim_time': use_sim_time}],
            output='screen',
            respawn=True, respawn_delay=2.0,
        ),

        # Lifecycle manager — transitions controller_server through
        # configure -> activate. Without this, controller_server stays in
        # 'unconfigured' state and never publishes /local_costmap/costmap.
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_perception_test',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['controller_server'],
            }],
            output='screen',
        ),

        # Foxglove WebSocket — connect from Studio: ws://<jetson-ip>:8765
        Node(
            package='foxglove_bridge', executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',
                'tls': False,
                'use_compression': False,
                'capabilities': ['clientPublish', 'parameters', 'parametersSubscribe',
                                 'services', 'connectionGraph', 'assets'],
            }],
            output='screen',
        ),

        # Actuator + WebUI joystick — drive the robot via phone WebSocket.
        # Brings up actuator_node (cmd_vel/ActuatorCommand -> Teensy serial ->
        # SparkMAX) and webui_node (FastAPI server on port 8000).
        # Phone URL: https://<jetson-ip>:8000  (accept the self-signed cert)
        # Note: heading-hold needs /imu/data which this slim launch doesn't
        # provide — webui throttle/steer pass through the slew limiter but
        # without IMU correction. Fine for short course tests; for longer
        # straight-line driving consider running sensors.launch.py too so
        # the Xsens publishes /imu/data.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('avros_bringup'),
                '/launch/webui.launch.py',
            ]),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(LaunchConfiguration('enable_drive')),
        ),
    ])
