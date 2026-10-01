"""Log external camera data and high-rate vehicle sensor streams."""

import csv
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from threading import Lock

import cv2
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
from avros_msgs.msg import ActuatorCommand, ActuatorState


# Camera device index for the RGB camera.
RGB_CAMERA_INDEX = 0
# Camera device index for the thermal camera.
THERMAL_CAMERA_INDEX = 1
# Number of image pairs to capture per second.
CAPTURE_RATE_HZ = 10.0
# Number of digits used for zero-padding image counters.
IMAGE_COUNT_WIDTH = 6


CAMERA_COLUMNS = [
    'image_count', 'episode_id', 'capture_time_utc', 'image_stamp_ns',
    'rgb_image', 'thermal_image',
    'command_throttle', 'command_steering', 'command_brake',
    'command_mode', 'command_estop',
    'actual_throttle', 'actual_steering', 'actual_brake',
    'actual_mode', 'actual_estop', 'watchdog_active',
    'linear_velocity_x', 'linear_velocity_y', 'linear_velocity_z',
    'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
]

IMU_COLUMNS = [
    'timestamp_ns', 'timestamp_utc', 'frame_id',
    'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
    'orientation_covariance_0', 'orientation_covariance_1',
    'orientation_covariance_2', 'orientation_covariance_3',
    'orientation_covariance_4', 'orientation_covariance_5',
    'orientation_covariance_6', 'orientation_covariance_7',
    'orientation_covariance_8',
    'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
    'angular_velocity_covariance_0', 'angular_velocity_covariance_1',
    'angular_velocity_covariance_2', 'angular_velocity_covariance_3',
    'angular_velocity_covariance_4', 'angular_velocity_covariance_5',
    'angular_velocity_covariance_6', 'angular_velocity_covariance_7',
    'angular_velocity_covariance_8',
    'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z',
    'linear_acceleration_covariance_0', 'linear_acceleration_covariance_1',
    'linear_acceleration_covariance_2', 'linear_acceleration_covariance_3',
    'linear_acceleration_covariance_4', 'linear_acceleration_covariance_5',
    'linear_acceleration_covariance_6', 'linear_acceleration_covariance_7',
    'linear_acceleration_covariance_8',
]

GNSS_COLUMNS = [
    'timestamp_ns', 'timestamp_utc', 'frame_id', 'status', 'service',
    'latitude', 'longitude', 'altitude',
    'position_covariance_0', 'position_covariance_1', 'position_covariance_2',
    'position_covariance_3', 'position_covariance_4', 'position_covariance_5',
    'position_covariance_6', 'position_covariance_7', 'position_covariance_8',
    'position_covariance_type',
]

def _open_camera(index: int):
    """Open an OpenCV camera and fail clearly when it is unavailable."""
    camera = cv2.VideoCapture(int(index))
    if not camera.isOpened():
        camera.release()
        raise RuntimeError(f'Could not open camera index {index}')
    return camera


def _image_names(count: int, stamp_date: str):
    """Return matching RGB and thermal filenames for one capture."""
    padded_count = f'{count:0{IMAGE_COUNT_WIDTH}d}'
    return (
        f'rgb_{padded_count}_{stamp_date}.jpg',
        f'thermal_{padded_count}_{stamp_date}.jpg',
    )



class DataLoggerNode(Node):
    """Save camera frames and sensor messages in rate-appropriate logs."""

    def __init__(self) -> None:
        super().__init__('avros_dataLogger')

        self.declare_parameter('output_directory', 'data_logger')
        self.declare_parameter('episode_id', 1)
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
        self._rgb_dir = self._output / 'images' / 'rgb_image'
        self._thermal_dir = self._output / 'images' / 'thermal_image'
        self._rgb_dir.mkdir(parents=True, exist_ok=True)
        self._thermal_dir.mkdir(parents=True, exist_ok=True)

        self._camera_file, self._camera_writer = self._open_csv(
            self._output / 'camera_log.csv', CAMERA_COLUMNS)
        self._imu_file, self._imu_writer = self._open_csv(
            self._output / 'imu_log.csv', IMU_COLUMNS)
        self._gnss_file, self._gnss_writer = self._open_csv(
            self._output / 'gnss_log.csv', GNSS_COLUMNS)

        self._episode_id = int(self.get_parameter('episode_id').value)
        self._rate_hz = float(self.get_parameter('capture_rate_hz').value)
        if self._rate_hz <= 0.0:
            raise ValueError('capture_rate_hz must be greater than zero')
        self._period_ns = int(1e9 / self._rate_hz)
        self._max_state_age_ns = int(float(
            self.get_parameter('state_max_age_seconds').value) * 1e9)
        self._jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        try:
            self._rgb_camera = _open_camera(
                self.get_parameter('rgb_camera_index').value)
            self._thermal_camera = _open_camera(
                self.get_parameter('thermal_camera_index').value)
        except RuntimeError:
            if hasattr(self, '_rgb_camera'):
                self._rgb_camera.release()
            self._close_logs()
            raise

        self._lock = Lock()
        self._last_capture_stamp_ns = None
        self._id = 0
        self._closed = False
        self._command_history = deque(maxlen=200)
        self._actuator_state_history = deque(maxlen=200)
        self._odom_history = deque(maxlen=200)

        self.create_subscription(
            Imu, str(self.get_parameter('imu_topic').value), self._on_imu,
            qos_profile_sensor_data)
        self.create_subscription(
            NavSatFix, str(self.get_parameter('gps_topic').value), self._on_gnss,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, str(self.get_parameter('odometry_topic').value), self._on_odom,
            qos_profile_sensor_data)
        self.create_subscription(
            ActuatorCommand, str(self.get_parameter('command_topic').value),
            self._on_command, qos_profile_sensor_data)
        self.create_subscription(
            ActuatorState, str(self.get_parameter('actuator_state_topic').value),
            self._on_actuator_state, qos_profile_sensor_data)

        self._capture_timer = self.create_timer(
            1.0 / self._rate_hz, self._capture_pair)
        self.get_logger().info(
            f'Logging to {self._output} at {self._rate_hz:g} Hz; '
            f'RGB index={self.get_parameter("rgb_camera_index").value}, '
            f'thermal index={self.get_parameter("thermal_camera_index").value}')

    @staticmethod
    def _open_csv(path: Path, columns):
        file = path.open('w', newline='', encoding='utf-8')
        writer = csv.DictWriter(file, fieldnames=columns)
        writer.writeheader()
        file.flush()
        return file, writer

    @staticmethod
    def _stamp_ns(msg) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    @staticmethod
    def _utc_stamp(stamp_ns: int) -> str:
        return datetime.fromtimestamp(stamp_ns / 1e9, timezone.utc).strftime(
            '%Y-%m-%dT%H:%M:%S.%fZ')

    def _on_imu(self, msg: Imu) -> None:
        stamp_ns = self._stamp_ns(msg)
        row = {
            'timestamp_ns': stamp_ns,
            'timestamp_utc': self._utc_stamp(stamp_ns),
            'frame_id': msg.header.frame_id,
            'orientation_x': msg.orientation.x,
            'orientation_y': msg.orientation.y,
            'orientation_z': msg.orientation.z,
            'orientation_w': msg.orientation.w,
            'angular_velocity_x': msg.angular_velocity.x,
            'angular_velocity_y': msg.angular_velocity.y,
            'angular_velocity_z': msg.angular_velocity.z,
            'linear_acceleration_x': msg.linear_acceleration.x,
            'linear_acceleration_y': msg.linear_acceleration.y,
            'linear_acceleration_z': msg.linear_acceleration.z,
        }
        row.update(self._covariance_fields('orientation_covariance',
                                            msg.orientation_covariance))
        row.update(self._covariance_fields('angular_velocity_covariance',
                                            msg.angular_velocity_covariance))
        row.update(self._covariance_fields('linear_acceleration_covariance',
                                            msg.linear_acceleration_covariance))
        with self._lock:
            self._imu_writer.writerow(row)
            self._imu_file.flush()

    def _on_gnss(self, msg: NavSatFix) -> None:
        stamp_ns = self._stamp_ns(msg)
        row = {
            'timestamp_ns': stamp_ns,
            'timestamp_utc': self._utc_stamp(stamp_ns),
            'frame_id': msg.header.frame_id,
            'status': msg.status.status,
            'service': msg.status.service,
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'position_covariance_type': msg.position_covariance_type,
        }
        row.update(self._covariance_fields('position_covariance',
                                            msg.position_covariance))
        with self._lock:
            self._gnss_writer.writerow(row)
            self._gnss_file.flush()

    @staticmethod
    def _covariance_fields(prefix: str, values):
        return {f'{prefix}_{index}': value for index, value in enumerate(values)}

    def _on_command(self, msg: ActuatorCommand) -> None:
        with self._lock:
            self._command_history.append((self._stamp_ns(msg), msg))

    def _on_actuator_state(self, msg: ActuatorState) -> None:
        with self._lock:
            self._actuator_state_history.append((self._stamp_ns(msg), msg))

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._odom_history.append((self._stamp_ns(msg), msg))

    @staticmethod
    def _nearest(history, stamp_ns: int, max_age_ns: int):
        if not history:
            return None
        candidate = min(history, key=lambda item: abs(item[0] - stamp_ns))
        return candidate[1] if abs(candidate[0] - stamp_ns) <= max_age_ns else None

    def _capture_pair(self) -> None:
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
            command = self._nearest(
                self._command_history, pair_stamp, self._max_state_age_ns)
            actuator_state = self._nearest(
                self._actuator_state_history, pair_stamp, self._max_state_age_ns)
            odom = self._nearest(
                self._odom_history, pair_stamp, self._max_state_age_ns)

        thermal = self._thermal_to_bgr(thermal)
        count = self._id
        stamp_date = datetime.fromtimestamp(pair_stamp / 1e9, timezone.utc).strftime(
            '%Y%m%dT%H%M%S.%fZ')
        rgb_name, thermal_name = _image_names(count, stamp_date)
        flags = [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
        if not cv2.imwrite(str(self._rgb_dir / rgb_name), rgb, flags):
            self.get_logger().error(f'Failed to write {rgb_name}')
            return
        if not cv2.imwrite(str(self._thermal_dir / thermal_name), thermal, flags):
            self.get_logger().error(f'Failed to write {thermal_name}')
            return

        row = self._make_camera_row(
            count, self._episode_id, pair_stamp, stamp_date,
            rgb_name, thermal_name, command, actuator_state, odom)
        with self._lock:
            self._camera_writer.writerow(row)
            self._camera_file.flush()
        self._id += 1

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
    def _make_camera_row(count, episode_id, stamp_ns, date,
                         rgb_name, thermal_name, command, actuator_state, odom):
        def value(obj, path: str, default=''):
            for part in path.split('.'):
                if obj is None:
                    return default
                obj = getattr(obj, part, None)
            return default if obj is None else obj

        return {
            'image_count': count,
            'episode_id': episode_id,
            'capture_time_utc': date,
            'image_stamp_ns': stamp_ns,
            'rgb_image': f'images/rgb/{rgb_name}',
            'thermal_image': f'images/thermal/{thermal_name}',
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
            'linear_velocity_x': value(odom, 'twist.twist.linear.x'),
            'linear_velocity_y': value(odom, 'twist.twist.linear.y'),
            'linear_velocity_z': value(odom, 'twist.twist.linear.z'),
            'angular_velocity_x': value(odom, 'twist.twist.angular.x'),
            'angular_velocity_y': value(odom, 'twist.twist.angular.y'),
            'angular_velocity_z': value(odom, 'twist.twist.angular.z'),
        }

    def _close_logs(self) -> None:
        for file in (self._camera_file, self._imu_file, self._gnss_file):
            if file is not None and not file.closed:
                file.flush()
                file.close()

    def destroy_node(self):
        if self._closed:
            return super().destroy_node()
        self._closed = True
        self._close_logs()
        self._rgb_camera.release()
        self._thermal_camera.release()
        self.get_logger().info(
            f'Closed logs after saving {self._id} image pairs')
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = DataLoggerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
