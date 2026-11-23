"""
Launch the LiDAR white-line detector (Ouster reflectivity -> /perception/lidar/lines).

Needs /ouster/points and the base_link -> os_sensor TF (sensors.launch.py, or a
bag that contains /tf_static). Examples:
  ros2 launch avros_perception lidar_lines.launch.py
  ros2 launch avros_perception lidar_lines.launch.py use_sim_time:=true   # bag --clock
  ros2 launch avros_perception lidar_lines.launch.py rviz:=true   # + RViz (NoMachine :1001)
"""

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _rviz(context, *_args, **_kwargs):
    cfg = context.launch_configurations
    if cfg['rviz'].lower() != 'true':
        return []
    env = {}
    if cfg['rviz_display']:
        env['DISPLAY'] = cfg['rviz_display']
    elif not os.environ.get('DISPLAY'):
        env['DISPLAY'] = ':1001'  # launched over SSH: the NoMachine session
    if cfg['software_gl'].lower() == 'true':
        env.update({'LIBGL_ALWAYS_SOFTWARE': '1', 'GALLIUM_DRIVER': 'llvmpipe',
                    'QT_X11_NO_MITSHM': '1'})
    rviz_config = os.path.join(
        get_package_share_directory('avros_bringup'), 'rviz', 'lidar_lines.rviz')
    return [Node(
        package='rviz2', executable='rviz2', name='rviz_lidar_lines',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        additional_env=env, output='log',
    )]


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('avros_perception'), 'config', 'lidar_lines.yaml')

    dds = [SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_cyclonedds_cpp')]
    try:
        dds.append(SetEnvironmentVariable(
            name='CYCLONEDDS_URI',
            value='file://' + os.path.join(
                get_package_share_directory('avros_bringup'), 'config', 'cyclonedds.xml')))
    except PackageNotFoundError:
        pass

    return LaunchDescription(dds + [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_input_topic', default_value='/ouster/points'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'rviz_display', default_value='',
            description='X display for RViz; empty = $DISPLAY, else :1001'),
        DeclareLaunchArgument(
            'software_gl', default_value='true',
            description='Software OpenGL for RViz (NoMachine displays)'),
        Node(
            package='avros_perception',
            executable='lidar_line_detector',
            name='lidar_line_detector',
            parameters=[config, {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_topic': LaunchConfiguration('lidar_input_topic'),
            }],
            additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'},
            output='screen',
        ),
        OpaqueFunction(function=_rviz),
    ])
