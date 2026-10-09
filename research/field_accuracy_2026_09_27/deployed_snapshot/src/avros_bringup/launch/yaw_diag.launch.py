"""Launch the minimal stack for yaw-diagnostic field testing.

Launches:
  - Everything from localization.launch.py (sensors + EKF + navsat)
  - actuator_node (cmd_vel -> Teensy serial)
  - foxglove_bridge (remote visualization)

Does NOT launch any Nav2 components (controller_server, velocity_smoother,
planner_server, bt_navigator, etc.) — these compete on /cmd_vel and
contaminate direct-cmd_vel test data. See issue #13 session log
2026-05-27 docs/yaw_diag_session_2026_05_27/repeatability_failure.md
for the discovery of the 4-publisher /cmd_vel contention.

Use this launch file for:
  - Issue #13 multi-source IMU/wheel/EKF yaw comparison
  - Phase 2 motion baseline characterization (M1b/c/d, M2)
  - Any direct /cmd_vel test where MPPI must be out of the loop

For the full Nav2 stack, use navigation.launch.py instead.

Defaults: LiDAR + cameras + NTRIP all OFF (no obstacle data needed,
no RTK per IGVC §I.2). Override per launch arg if needed.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('avros_bringup')
    actuator_config = os.path.join(pkg_dir, 'config', 'actuator_params.yaml')
    cyclonedds_file = os.path.join(pkg_dir, 'config', 'cyclonedds.xml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # Force CycloneDDS so every node spawned here (and any children) uses
        # the same RMW as the sensor stack. CLI tools still read shell env;
        # see CLAUDE.md known-issues.
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
            'enable_ntrip', default_value='false',
            description='Enable NTRIP client for RTK corrections. '
                        'Default false per IGVC §I.2 (no base stations).'
        ),

        DeclareLaunchArgument(
            'enable_velodyne', default_value='false',
            description='Enable Velodyne VLP-16 LiDAR. Default false for '
                        'yaw-diag tests (no obstacle data needed).'
        ),

        DeclareLaunchArgument(
            'enable_zed_front', default_value='false',
            description='Enable front ZED X camera. Default false for '
                        'yaw-diag tests. Enable to record ZED VIO odom as '
                        'a 3rd-source yaw reference (M2 tiebreaker).'
        ),

        DeclareLaunchArgument(
            'enable_zed_left', default_value='false',
            description='Enable left ZED X camera (Phase 5).'
        ),

        DeclareLaunchArgument(
            'enable_zed_right', default_value='false',
            description='Enable right ZED X camera (Phase 5).'
        ),

        # Localization (sensors + EKF + navsat). This brings in
        # robot_state_publisher, xsens_mti_node, ekf_filter_node_odom,
        # ekf_filter_node_map, navsat_transform_node — and optionally
        # velodyne / ZED / ntrip per the flags above.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'localization.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'enable_ntrip': LaunchConfiguration('enable_ntrip'),
                'enable_velodyne': LaunchConfiguration('enable_velodyne'),
                'enable_zed_front': LaunchConfiguration('enable_zed_front'),
                'enable_zed_left': LaunchConfiguration('enable_zed_left'),
                'enable_zed_right': LaunchConfiguration('enable_zed_right'),
            }.items(),
        ),

        # Actuator bridge — listens on /cmd_vel + /avros/actuator_command,
        # talks to Teensy on /dev/ttyACM0.
        Node(
            package='avros_control',
            executable='actuator_node',
            name='actuator_node',
            parameters=[
                actuator_config,
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),

        # Foxglove bridge for remote monitoring (laptop side).
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': 8765,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),

        # NO Nav2 nodes — that is intentional and the whole point of this
        # launch file. /cmd_vel will have exactly one publisher (the test
        # script) and one subscriber (actuator_node). No velocity_smoother,
        # no controller_server, no behavior_server interference.
    ])
