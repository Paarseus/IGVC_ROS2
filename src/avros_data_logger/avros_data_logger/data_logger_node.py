"""Capture synchronized camera frames and the latest vehicle state to disk."""

import csv
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from threading import Lock

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from avros_msgs.msg import ActuatorCommand, ActuatorState


RGB_CAMERA_INDEX = 0 # Camera device index for the RGB camera.
THERMAL_CAMERA_INDEX = 1 # Camera device index for the thermal camera.
CAPTURE_RATE_HZ = 10.0 # Number of image pairs to capture per second.


CSV_COLUMNS = [
    'image_count', 'capture_time_utc', 'image_stamp_ns',
    'rgb_image', 'thermal_image',
    'command_throttle', 'command_steering', 'command_brake',
    'command_mode', 'command_estop',
    'actual_throttle', 'actual_steering', 'actual_brake',
    'actual_mode', 'actual_estop', 'watchdog_active',
    'imu_orientation_x', 'imu_orientation_y', 'imu_orientation_z', 'imu_orientation_w',
    'imu_angular_velocity_x', 'imu_angular_velocity_y', 'imu_angular_velocity_z',
    'imu_linear_acceleration_x', 'imu_linear_acceleration_y', 'imu_linear_acceleration_z',
    'gps_valid', 'gps_latitude', 'gps_longitude', 'gps_altitude',
    'gps_status', 'gps_covariance_xx',
    'linear_velocity_x', 'linear_velocity_y', 'linear_velocity_z',
    'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
]


class DataLoggerNode(Node):
    """Save matched RGB/thermal pairs and state snapshots at a fixed rate."""

    def __init__(self) -> None:
        super().__init__('avros_dataLogger')

        self.declare_parameter('output_directory', 'data_logger')
        self.declare_parameter('rgb_camera_index', RGB_CAMERA_INDEX)
        self.declare_parameter('thermal_camera_index', THERMAL_CAMERA_INDEX)
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('command_topic', '/avros/actuator_command')
        self.declare_parameter('actuator_state_topic', '/avros/actuator_state')
        self.declare_parameter('gps_topic', '/gnss')
        self.declare_parameter('odometry_topic', '/wheel_odom')
        self.declare_parameter('capture_rate_hz', CAPTURE_RATE_HZ)
        self.declare_parameter('state_max_age_seconds', 0.5)
        self.declare_parameter('jpeg_quality', 95)

        output = Path(str(self.get_parameter('output_directory').value)).expanduser()
        self._output = output if output.is_absolute() else Path.cwd() / output
        self._rgb_dir = self._output / 'imgs' / 'rgb'
        self._thermal_dir = self._output / 'imgs' / 'thermal'
        self._rgb_dir.mkdir(parents=True, exist_ok=True)
        self._thermal_dir.mkdir(parents=True, exist_ok=True)
        self._csv_path = self._output / 'data.csv'
        self._csv_file = self._csv_path.open('w', newline='', encoding='utf-8')
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=CSV_COLUMNS)
        self._csv_writer.writeheader()
        self._csv_file.flush()

        self._rate_hz = float(self.get_parameter('capture_rate_hz').value)
        if self._rate_hz <= 0.0:
            raise ValueError('capture_rate_hz must be greater than zero')
        self._period_ns = int(1e9 / self._rate_hz)
        self._max_state_age_ns = int(float(
            self.get_parameter('state_max_age_seconds').value) * 1e9)
        self._jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self._rgb_camera = cv2.VideoCapture(int(self.get_parameter('rgb_camera_index').value))
        self._thermal_camera = cv2.VideoCapture(int(self.get_parameter('thermal_camera_index').value))
        if not self._rgb_camera.isOpened():
            raise RuntimeError('Could not open the RGB camera')
        if not self._thermal_camera.isOpened():
            self._rgb_camera.release()
            raise RuntimeError('Could not open the thermal camera')
        self._lock = Lock()
        self._last_capture_stamp_ns = None
        self._image_count = 0
        self._closed = False

        # Keep short histories so the state associated with an image is chosen
        # by timestamp, rather than simply whichever callback happened last.
        self._imu_history = deque(maxlen=200)
        self._gps_history = deque(maxlen=50)
        self._odom_history = deque(maxlen=200)
        self._command_history = deque(maxlen=200)
        self._actuator_state_history = deque(maxlen=200)

        self._imu_sub = self.create_subscription(
            Imu, str(self.get_parameter('imu_topic').value), self._on_imu,
            qos_profile_sensor_data)
        self._gps_sub = self.create_subscription(
            NavSatFix, str(self.get_parameter('gps_topic').value), self._on_gps,
            qos_profile_sensor_data)
        self._odom_sub = self.create_subscription(
            Odometry, str(self.get_parameter('odometry_topic').value), self._on_odom,
            qos_profile_sensor_data)
        self._command_sub = self.create_subscription(
            ActuatorCommand, str(self.get_parameter('command_topic').value),
            self._on_command, qos_profile_sensor_data)
        self._actuator_state_sub = self.create_subscription(
            ActuatorState, str(self.get_parameter('actuator_state_topic').value),
            self._on_actuator_state, qos_profile_sensor_data)

        self._capture_timer = self.create_timer(1.0 / self._rate_hz, self._capture_pair)

        self.get_logger().info(
            f'Collecting at {self._rate_hz:g} Hz into {self._output}; '
            f'RGB camera index={self.get_parameter("rgb_camera_index").value}, '
            f'thermal camera index={self.get_parameter("thermal_camera_index").value}')

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

    def _on_command(self, msg: ActuatorCommand) -> None:
        with self._lock:
            self._command_history.append((self._stamp_ns(msg), msg))

    def _on_actuator_state(self, msg: ActuatorState) -> None:
        with self._lock:
            self._actuator_state_history.append((self._stamp_ns(msg), msg))

    @staticmethod
    def _nearest(history, stamp_ns: int, max_age_ns: int):
        if not history:
            return None
        candidate = min(history, key=lambda item: abs(item[0] - stamp_ns))
        return candidate[1] if abs(candidate[0] - stamp_ns) <= max_age_ns else None

    def _capture_pair(self) -> None:
        # The reads occur in the same callback. Hardware triggering is needed
        # for truly simultaneous exposure; this gives ordinary USB cameras a
        # consistent software capture cadence.
        rgb_ok, rgb = self._rgb_camera.read()
        thermal_ok, thermal = self._thermal_camera.read()
        if not rgb_ok or not thermal_ok:
            self.get_logger().warning('Could not read both external cameras')
            return
        pair_stamp = self.get_clock().now().nanoseconds
        with self._lock:
            if (self._last_capture_stamp_ns is not None and
                    pair_stamp - self._last_capture_stamp_ns < self._period_ns):
                return
            self._last_capture_stamp_ns = pair_stamp
            imu = self._nearest(self._imu_history, pair_stamp, self._max_state_age_ns)
            gps = self._nearest(self._gps_history, pair_stamp, self._max_state_age_ns)
            odom = self._nearest(self._odom_history, pair_stamp, self._max_state_age_ns)
            command = self._nearest(self._command_history, pair_stamp, self._max_state_age_ns)
            actuator_state = self._nearest(
                self._actuator_state_history, pair_stamp, self._max_state_age_ns)

        thermal = self._thermal_to_bgr(thermal)

        count = self._image_count
        stamp_date = datetime.fromtimestamp(pair_stamp / 1e9, timezone.utc).strftime(
            '%Y%m%dT%H%M%S.%fZ')
        padded_count = f'{count:0{IMAGE_COUNT_WIDTH}d}'
        rgb_name = f'rgb_{padded_count}_{stamp_date}.jpg'
        thermal_name = f'thermal_{padded_count}_{stamp_date}.jpg'
        flags = [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
        if not cv2.imwrite(str(self._rgb_dir / rgb_name), rgb, flags):
            self.get_logger().error(f'Failed to write {rgb_name}')
            return
        if not cv2.imwrite(str(self._thermal_dir / thermal_name), thermal, flags):
            self.get_logger().error(f'Failed to write {thermal_name}')
            return

        row = self._make_row(
            count, pair_stamp, stamp_date, rgb_name, thermal_name, imu, gps, odom,
            command, actuator_state)
        self._csv_writer.writerow(row)
        self._csv_file.flush()
        self._image_count += 1

    @staticmethod
    def _thermal_to_bgr(image):
        if image is None:
            raise ValueError('thermal image is empty')
        if image.dtype != 'uint8':
            image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
        if len(image.shape) == 2:
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        if image.shape[2] == 4:
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        return image

    @staticmethod
    def _make_row(count, stamp_ns, date, rgb_name, thermal_name, imu, gps, odom,
                  command, actuator_state):
        def value(obj, path: str, default=''):
            for part in path.split('.'):
                if obj is None:
                    return default
                if part.isdigit():
                    try:
                        obj = obj[int(part)]
                    except (IndexError, TypeError):
                        return default
                else:
                    obj = getattr(obj, part, None)
            return default if obj is None else obj

        return {
            'image_count': count, 'capture_time_utc': date,
            'image_stamp_ns': stamp_ns, 'rgb_image': f'imgs/rgb/{rgb_name}',
            'thermal_image': f'imgs/thermal/{thermal_name}',
            'command_throttle': value(command, 'throttle'),
            'command_steering': value(command, 'steer'),
            'command_brake': value(command, 'brake'),
            'command_mode': value(command, 'mode'),
            'command_estop': value(command, 'estop'),
            'actual_throttle': value(actuator_state, 'throttle'),
            'actual_steering': value(actuator_state, 'steer'),
            'actual_brake': value(actuator_state, 'brake'),
            'actual_mode': value(actuator_state, 'mode'),
            'actual_estop': value(actuator_state, 'estop'),
            'watchdog_active': value(actuator_state, 'watchdog_active'),
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
            'gps_valid': (value(gps, 'status.status', -1) >= 0),
            'gps_latitude': value(gps, 'latitude'),
            'gps_longitude': value(gps, 'longitude'),
            'gps_altitude': value(gps, 'altitude'),
            'gps_status': value(gps, 'status.status'),
            'gps_covariance_xx': value(gps, 'position_covariance.0'),
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
        self._csv_file.flush()
        self._csv_file.close()
        self.get_logger().info(
            f'Wrote {self._image_count} rows to {self._csv_path}')

    def destroy_node(self):
        self.write_csv()
        self._rgb_camera.release()
        self._thermal_camera.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DataLoggerNode()
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
