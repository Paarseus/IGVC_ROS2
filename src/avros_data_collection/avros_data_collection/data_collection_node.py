"""Capture synchronized camera frames and the latest vehicle state to disk."""

import csv
import os
import signal
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from threading import Lock
from typing import Optional

import cv2
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, NavSatFix
from nav_msgs.msg import Odometry


CSV_COLUMNS = [
    'image_count', 'capture_time_utc', 'image_stamp_ns',
    'rgb_image', 'thermal_image',
    'imu_orientation_x', 'imu_orientation_y', 'imu_orientation_z', 'imu_orientation_w',
    'imu_angular_velocity_x', 'imu_angular_velocity_y', 'imu_angular_velocity_z',
    'imu_linear_acceleration_x', 'imu_linear_acceleration_y', 'imu_linear_acceleration_z',
    'gps_latitude', 'gps_longitude', 'gps_altitude', 'gps_status',
    'linear_velocity_x', 'linear_velocity_y', 'linear_velocity_z',
    'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
]


class DataCollectionNode(Node):
    """Save matched RGB/thermal pairs and state snapshots at a fixed rate."""

    def __init__(self) -> None:
        super().__init__('avros_dataCollection')

        self.declare_parameter('output_directory', 'data_collection')
        self.declare_parameter('rgb_topic', '/zed_front/zed_node/rgb/image_rect_color')
        self.declare_parameter('thermal_topic', '/thermal/image_raw')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('gps_topic', '/gnss')
        self.declare_parameter('odometry_topic', '/wheel_odom')
        self.declare_parameter('capture_rate_hz', 10.0)
        self.declare_parameter('sync_slop_seconds', 0.05)
        self.declare_parameter('state_max_age_seconds', 0.5)
        self.declare_parameter('jpeg_quality', 95)

        output = Path(str(self.get_parameter('output_directory').value)).expanduser()
        self._output = output if output.is_absolute() else Path.cwd() / output
        self._rgb_dir = self._output / 'imgs' / 'rgb'
        self._thermal_dir = self._output / 'imgs' / 'thermal'
        self._rgb_dir.mkdir(parents=True, exist_ok=True)
        self._thermal_dir.mkdir(parents=True, exist_ok=True)
        self._csv_path = self._output / 'data.csv'

        self._rate_hz = float(self.get_parameter('capture_rate_hz').value)
        if self._rate_hz <= 0.0:
            raise ValueError('capture_rate_hz must be greater than zero')
        self._period_ns = int(1e9 / self._rate_hz)
        self._slop = float(self.get_parameter('sync_slop_seconds').value)
        self._max_state_age_ns = int(float(
            self.get_parameter('state_max_age_seconds').value) * 1e9)
        self._jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self._bridge = CvBridge()
        self._lock = Lock()
        self._last_capture_stamp_ns = None
        self._image_count = 0
        self._rows = []
        self._closed = False

        # Keep short histories so the state associated with an image is chosen
        # by timestamp, rather than simply whichever callback happened last.
        self._imu_history = deque(maxlen=200)
        self._gps_history = deque(maxlen=50)
        self._odom_history = deque(maxlen=200)

        self._imu_sub = self.create_subscription(
            Imu, str(self.get_parameter('imu_topic').value), self._on_imu,
            qos_profile_sensor_data)
        self._gps_sub = self.create_subscription(
            NavSatFix, str(self.get_parameter('gps_topic').value), self._on_gps,
            qos_profile_sensor_data)
        self._odom_sub = self.create_subscription(
            Odometry, str(self.get_parameter('odometry_topic').value), self._on_odom,
            qos_profile_sensor_data)

        self._rgb_sub = Subscriber(
            self, Image, str(self.get_parameter('rgb_topic').value),
            qos_profile=qos_profile_sensor_data)
        self._thermal_sub = Subscriber(
            self, Image, str(self.get_parameter('thermal_topic').value),
            qos_profile=qos_profile_sensor_data)
        self._sync = ApproximateTimeSynchronizer(
            [self._rgb_sub, self._thermal_sub], queue_size=30, slop=self._slop)
        self._sync.registerCallback(self._on_image_pair)

        self.get_logger().info(
            f'Collecting at {self._rate_hz:g} Hz into {self._output}; '
            f'RGB={self._rgb_sub.topic}, thermal={self._thermal_sub.topic}')

    @staticmethod
    def _stamp_ns(msg) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _on_imu(self, msg: Imu) -> None:
        with self._lock:
            self._imu_history.append((self._stamp_ns(msg), msg))

    def _on_gps(self, msg: NavSatFix) -> None:
        with self._lock:
            self._gps_history.append((self._stamp_ns(msg), msg))

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._odom_history.append((self._stamp_ns(msg), msg))

    @staticmethod
    def _nearest(history, stamp_ns: int, max_age_ns: int):
        if not history:
            return None
        candidate = min(history, key=lambda item: abs(item[0] - stamp_ns))
        return candidate[1] if abs(candidate[0] - stamp_ns) <= max_age_ns else None

    def _on_image_pair(self, rgb_msg: Image, thermal_msg: Image) -> None:
        rgb_stamp = self._stamp_ns(rgb_msg)
        thermal_stamp = self._stamp_ns(thermal_msg)
        pair_stamp = max(rgb_stamp, thermal_stamp)
        with self._lock:
            if (self._last_capture_stamp_ns is not None and
                    pair_stamp - self._last_capture_stamp_ns < self._period_ns):
                return
            self._last_capture_stamp_ns = pair_stamp
            imu = self._nearest(self._imu_history, pair_stamp, self._max_state_age_ns)
            gps = self._nearest(self._gps_history, pair_stamp, self._max_state_age_ns)
            odom = self._nearest(self._odom_history, pair_stamp, self._max_state_age_ns)

        try:
            rgb = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            thermal = self._thermal_to_bgr(thermal_msg)
        except CvBridgeError as exc:
            self.get_logger().error(f'Could not convert image pair: {exc}')
            return

        count = self._image_count
        stamp_date = datetime.fromtimestamp(pair_stamp / 1e9, timezone.utc).strftime(
            '%Y%m%dT%H%M%S.%fZ')
        rgb_name = f'rgb_{count}_{stamp_date}.jpg'
        thermal_name = f'thermal_{count}_{stamp_date}.jpg'
        flags = [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
        if not cv2.imwrite(str(self._rgb_dir / rgb_name), rgb, flags):
            self.get_logger().error(f'Failed to write {rgb_name}')
            return
        if not cv2.imwrite(str(self._thermal_dir / thermal_name), thermal, flags):
            self.get_logger().error(f'Failed to write {thermal_name}')
            return

        self._rows.append(self._make_row(
            count, pair_stamp, stamp_date, rgb_name, thermal_name, imu, gps, odom))
        self._image_count += 1

    def _thermal_to_bgr(self, msg: Image):
        image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if image is None:
            raise CvBridgeError('thermal image is empty')
        if image.dtype != 'uint8':
            image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
        if len(image.shape) == 2:
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        if image.shape[2] == 4:
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        return image

    @staticmethod
    def _make_row(count, stamp_ns, date, rgb_name, thermal_name, imu, gps, odom):
        def value(obj, path: str, default=''):
            for part in path.split('.'):
                if obj is None:
                    return default
                obj = getattr(obj, part, None)
            return default if obj is None else obj

        return {
            'image_count': count, 'capture_time_utc': date,
            'image_stamp_ns': stamp_ns, 'rgb_image': f'imgs/rgb/{rgb_name}',
            'thermal_image': f'imgs/thermal/{thermal_name}',
            'imu_orientation_x': value(imu, 'orientation.x'),
            'imu_orientation_y': value(imu, 'orientation.y'),
            'imu_orientation_z': value(imu, 'orientation.z'),
            'imu_orientation_w': value(imu, 'orientation.w'),
            'imu_angular_velocity_x': value(imu, 'angular_velocity.x'),
            'imu_angular_velocity_y': value(imu, 'angular_velocity.y'),
            'imu_angular_velocity_z': value(imu, 'angular_velocity.z'),
            'imu_linear_acceleration_x': value(imu, 'linear_acceleration.x'),
            'imu_linear_acceleration_y': value(imu, 'linear_acceleration.y'),
            'imu_linear_acceleration_z': value(imu, 'linear_acceleration.z'),
            'gps_latitude': value(gps, 'latitude'), 'gps_longitude': value(gps, 'longitude'),
            'gps_altitude': value(gps, 'altitude'), 'gps_status': value(gps, 'status.status'),
            'linear_velocity_x': value(odom, 'twist.twist.linear.x'),
            'linear_velocity_y': value(odom, 'twist.twist.linear.y'),
            'linear_velocity_z': value(odom, 'twist.twist.linear.z'),
            'angular_velocity_x': value(odom, 'twist.twist.angular.x'),
            'angular_velocity_y': value(odom, 'twist.twist.angular.y'),
            'angular_velocity_z': value(odom, 'twist.twist.angular.z'),
        }

    def write_csv(self) -> None:
        if self._closed:
            return
        self._closed = True
        with self._csv_path.open('w', newline='', encoding='utf-8') as file:
            writer = csv.DictWriter(file, fieldnames=CSV_COLUMNS)
            writer.writeheader()
            writer.writerows(self._rows)
        self.get_logger().info(f'Wrote {len(self._rows)} rows to {self._csv_path}')

    def destroy_node(self):
        self.write_csv()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DataCollectionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
