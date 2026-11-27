"""Raw ROS image/sensor recorder for RGB, thermal, IMU, and GNSS data."""

import csv
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, NavSatFix
from avros_msgs.msg import ActuatorCommand


CAMERA_COLUMNS = [
    'pair_id', 'rgb_stamp_ns', 'thermal_stamp_ns', 'rgb_receive_ns',
    'thermal_receive_ns', 'pair_delta_ns', 'pair_valid', 'rgb_image',
    'thermal_image', 'throttle', 'steering', 'brake', 'mode', 'estop',
    'vehicle_stopped', 'speed_mps', 'yaw_rate_rps',
]

IMU_COLUMNS = [
    'timestamp_ns', 'receive_ns', 'frame_id', 'orientation_x',
    'orientation_y', 'orientation_z', 'orientation_w', 'angular_velocity_x',
    'angular_velocity_y', 'angular_velocity_z', 'linear_acceleration_x',
    'linear_acceleration_y', 'linear_acceleration_z',
]

GNSS_COLUMNS = [
    'timestamp_ns', 'receive_ns', 'frame_id', 'status', 'service',
    'latitude', 'longitude', 'altitude', 'position_covariance_0',
    'position_covariance_1', 'position_covariance_2',
]


class RosDataLoggerNode(Node):
    """Record raw ROS camera messages and original-rate vehicle sensors."""

    def __init__(self):
        super().__init__('avros_dataLogger')
        self.declare_parameter('output_directory', 'data_logger')
        self.declare_parameter('rgb_topic', '/rgb_camera/image_raw')
        self.declare_parameter('thermal_topic', '/thermal_camera/image_raw')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('gps_topic', '/gnss')
        self.declare_parameter('odometry_topic', '/wheel_odom')
        self.declare_parameter('command_topic', '/avros/actuator_command')
        self.declare_parameter('pair_tolerance_ms', 10.0)

        root = Path(str(self.get_parameter('output_directory').value)).expanduser()
        self._root = root if root.is_absolute() else Path.cwd() / root
        self._rgb_dir = self._root / 'images' / 'rgb'
        self._thermal_dir = self._root / 'images' / 'thermal'
        self._rgb_dir.mkdir(parents=True, exist_ok=True)
        self._thermal_dir.mkdir(parents=True, exist_ok=True)
        self._camera_file, self._camera_writer = self._csv(
            'camera_log.csv', CAMERA_COLUMNS)
        self._imu_file, self._imu_writer = self._csv('imu_log.csv', IMU_COLUMNS)
        self._gnss_file, self._gnss_writer = self._csv('gnss_log.csv', GNSS_COLUMNS)

        self._lock = threading.Lock()
        self._bridge = CvBridge()
        self._rgb_queue = deque(maxlen=30)
        self._thermal_queue = deque(maxlen=30)
        self._commands = deque(maxlen=200)
        self._odom = deque(maxlen=200)
        self._pair_id = 0
        self._slop_ns = int(float(self.get_parameter('pair_tolerance_ms').value) * 1e6)

        self.create_subscription(
            Image, str(self.get_parameter('rgb_topic').value), self._on_rgb,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, str(self.get_parameter('thermal_topic').value),
            self._on_thermal, qos_profile_sensor_data)
        self.create_subscription(
            Imu, str(self.get_parameter('imu_topic').value), self._on_imu,
            qos_profile_sensor_data)
        self.create_subscription(
            NavSatFix, str(self.get_parameter('gps_topic').value), self._on_gnss,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, str(self.get_parameter('odometry_topic').value),
            self._on_odom, qos_profile_sensor_data)
        self.create_subscription(
            ActuatorCommand, str(self.get_parameter('command_topic').value),
            self._on_command, qos_profile_sensor_data)
        self.get_logger().info(f'Raw ROS recording to {self._root}')

    def _csv(self, name, columns):
        file = (self._root / name).open('w', newline='', encoding='utf-8')
        writer = csv.DictWriter(file, fieldnames=columns)
        writer.writeheader()
        file.flush()
        return file, writer

    @staticmethod
    def _stamp(msg):
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    @staticmethod
    def _utc(stamp_ns):
        return datetime.fromtimestamp(stamp_ns / 1e9, timezone.utc).isoformat()

    @staticmethod
    def _nearest(history, stamp_ns):
        if not history:
            return None
        prior = [item for item in history if item[0] <= stamp_ns]
        return prior[-1][1] if prior else None

    def _on_rgb(self, msg):
        self._rgb_queue.append((self._stamp(msg), time.monotonic_ns(), msg))
        self._try_pair()

    def _on_thermal(self, msg):
        self._thermal_queue.append((self._stamp(msg), time.monotonic_ns(), msg))
        self._try_pair()

    def _try_pair(self):
        if not self._rgb_queue or not self._thermal_queue:
            return
        rgb_index, thermal_index = min(
            ((i, j) for i, rgb in enumerate(self._rgb_queue)
             for j, thermal in enumerate(self._thermal_queue)),
            key=lambda pair: abs(self._rgb_queue[pair[0]][0] -
                                 self._thermal_queue[pair[1]][0]))
        rgb = self._rgb_queue[rgb_index]
        thermal = self._thermal_queue[thermal_index]
        delta = abs(rgb[0] - thermal[0])
        if delta > self._slop_ns:
            if rgb[0] < thermal[0]:
                self._rgb_queue.popleft()
            else:
                self._thermal_queue.popleft()
            return
        del self._rgb_queue[rgb_index]
        del self._thermal_queue[thermal_index]
        try:
            rgb_image = self._bridge.imgmsg_to_cv2(rgb[2], 'passthrough')
            thermal_image = self._bridge.imgmsg_to_cv2(thermal[2], 'passthrough')
            stamp = max(rgb[0], thermal[0])
            stem = f'{self._pair_id:06d}_{stamp}'
            rgb_name = f'rgb_{stem}.png'
            thermal_name = f'thermal_{stem}.png'
            if not cv2.imwrite(str(self._rgb_dir / rgb_name), rgb_image):
                raise IOError(rgb_name)
            if not cv2.imwrite(str(self._thermal_dir / thermal_name), thermal_image):
                raise IOError(thermal_name)
            command = self._nearest(self._commands, stamp)
            odom = self._nearest(self._odom, stamp)
            row = {
                'pair_id': self._pair_id,
                'rgb_stamp_ns': rgb[0], 'thermal_stamp_ns': thermal[0],
                'rgb_receive_ns': rgb[1], 'thermal_receive_ns': thermal[1],
                'pair_delta_ns': delta, 'pair_valid': True,
                'rgb_image': f'images/rgb/{rgb_name}',
                'thermal_image': f'images/thermal/{thermal_name}',
                'throttle': getattr(command, 'throttle', ''),
                'steering': getattr(command, 'steer', ''),
                'brake': getattr(command, 'brake', ''),
                'mode': getattr(command, 'mode', ''),
                'estop': getattr(command, 'estop', ''),
                'vehicle_stopped': command is None or (
                    getattr(command, 'throttle', 0.0) == 0.0 and
                    getattr(command, 'brake', 0.0) == 0.0),
                'speed_mps': getattr(getattr(getattr(odom, 'twist', None),
                                             'twist', None), 'linear', None),
                'yaw_rate_rps': getattr(getattr(getattr(odom, 'twist', None),
                                                'twist', None), 'angular', None),
            }
            row['speed_mps'] = getattr(row['speed_mps'], 'x', '')
            row['yaw_rate_rps'] = getattr(row['yaw_rate_rps'], 'z', '')
            with self._lock:
                self._camera_writer.writerow(row)
                self._camera_file.flush()
            self._pair_id += 1
        except Exception as exc:
            self.get_logger().error(f'Could not save image pair: {exc}')

    def _on_imu(self, msg):
        row = {
            'timestamp_ns': self._stamp(msg),
            'receive_ns': time.monotonic_ns(),
            'frame_id': msg.header.frame_id,
            'orientation_x': msg.orientation.x, 'orientation_y': msg.orientation.y,
            'orientation_z': msg.orientation.z, 'orientation_w': msg.orientation.w,
            'angular_velocity_x': msg.angular_velocity.x,
            'angular_velocity_y': msg.angular_velocity.y,
            'angular_velocity_z': msg.angular_velocity.z,
            'linear_acceleration_x': msg.linear_acceleration.x,
            'linear_acceleration_y': msg.linear_acceleration.y,
            'linear_acceleration_z': msg.linear_acceleration.z,
        }
        with self._lock:
            self._imu_writer.writerow(row)

    def _on_gnss(self, msg):
        row = {
            'timestamp_ns': self._stamp(msg), 'receive_ns': time.monotonic_ns(),
            'frame_id': msg.header.frame_id, 'status': msg.status.status,
            'service': msg.status.service, 'latitude': msg.latitude,
            'longitude': msg.longitude, 'altitude': msg.altitude,
            'position_covariance_0': msg.position_covariance[0],
            'position_covariance_1': msg.position_covariance[1],
            'position_covariance_2': msg.position_covariance[2],
        }
        with self._lock:
            self._gnss_writer.writerow(row)

    def _on_command(self, msg):
        self._commands.append((self._stamp(msg), msg))

    def _on_odom(self, msg):
        self._odom.append((self._stamp(msg), msg))

    def destroy_node(self):
        for file in (self._camera_file, self._imu_file, self._gnss_file):
            file.flush()
            file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RosDataLoggerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
