"""Log external camera data and high-rate vehicle sensor streams."""

import csv
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
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
from avros_msgs.msg import ActuatorCommand


# Default RGB index; -1 disables RGB until a ROS parameter is supplied.
RGB_CAMERA_INDEX = -1
# Default thermal index; -1 disables thermal until a ROS parameter is supplied.
THERMAL_CAMERA_INDEX = -1
# Number of image pairs to capture per second.
CAPTURE_RATE_HZ = 10.0
# Number of digits used for zero-padding image counters.
IMAGE_COUNT_WIDTH = 6
# Enable local contrast enhancement for camera frames by default.
AUTO_CONTRAST_ENABLED = True
# CLAHE contrast limit used for local camera contrast enhancement.
AUTO_CONTRAST_CLIP_LIMIT = 2.0
# CLAHE tile size used for local camera contrast enhancement.
AUTO_CONTRAST_TILE_GRID_SIZE = (8, 8)


CAMERA_COLUMNS = [
    'image_count', 'episode_id', 'capture_time_utc', 'image_stamp_ns',
    'time_seconds',
    'rgb_image', 'thermal_image',
    'throttle', 'steering', 'brake', 'mode', 'estop',
    'vehicle_stopped',
    'speed_mps', 'yaw_rate_rps',
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



_PREVIEW_HTML = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AV ROS Imitation Learning Data Logger</title>
<style>
body { margin: 0; background: #181818; color: white; font-family: sans-serif; }
header { padding: 12px 16px; background: #282828; font-size: 20px; }
controls { display: flex; align-items: center; gap: 10px; padding: 12px 16px; background: #222; }
button { padding: 8px 16px; font-size: 16px; cursor: pointer; }
#recording { color: #ffcc66; }
main { display: flex; flex-direction: column; gap: 12px; padding: 12px; }
.panel { position: relative; height: calc(50vh - 58px); min-height: 180px;
         background: black; display: flex; align-items: center;
         justify-content: center; overflow: hidden; }
.panel img { width: 100%; height: 100%; object-fit: contain; }
.label { position: absolute; top: 8px; left: 8px; background: #000b;
         padding: 5px 8px; z-index: 1; }
.unavailable { color: #aaa; font-size: 22px; }
</style>
</head>
<body>
<header>AV ROS Imitation Learning Data Logger — Images captured: <span id="count">0</span></header>
<controls>
  <button id="start" onclick="setRecording(true)">Start Recording</button>
  <button id="stop" onclick="setRecording(false)" disabled>Stop Recording</button>
  <span id="recording">Preview only — recording has not started</span>
</controls>
<main>
  <section class="panel"><span class="label">RGB</span>
    <img id="rgb" alt="RGB camera"><span id="rgb-unavailable" class="unavailable">Unavailable</span>
  </section>
  <section class="panel"><span class="label">Thermal</span>
    <img id="thermal" alt="Thermal camera"><span id="thermal-unavailable" class="unavailable">Unavailable</span>
  </section>
</main>
<script>
function updateFrame(name, available) {
  const image = document.getElementById(name);
  const message = document.getElementById(name + '-unavailable');
  image.style.display = available ? 'block' : 'none';
  message.style.display = available ? 'none' : 'block';
  if (available) image.src = '/frame/' + name + '.jpg?t=' + Date.now();
}
async function setRecording(enabled) {
  await fetch(enabled ? '/record/start' : '/record/stop', {method: 'POST'});
  await refresh();
}
async function refresh() {
  try {
    const response = await fetch('/status?t=' + Date.now());
    const status = await response.json();
    document.getElementById('count').textContent = status.image_count;
    document.getElementById('start').disabled = status.recording;
    document.getElementById('stop').disabled = !status.recording;
    document.getElementById('recording').textContent = status.recording
      ? 'Recording active' : 'Preview only — recording has stopped';
    updateFrame('rgb', status.rgb_available);
    updateFrame('thermal', status.thermal_available);
  } catch (error) {
    updateFrame('rgb', false);
    updateFrame('thermal', false);
  }
}
refresh();
setInterval(refresh, 250);
</script>
</body>
</html>"""


class _PreviewHTTPServer(HTTPServer):
    allow_reuse_address = True

    def __init__(self, address, handler, logger_node):
        self.logger_node = logger_node
        super().__init__(address, handler)


class _PreviewHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path == '/':
            self._send(200, 'text/html; charset=utf-8', _PREVIEW_HTML.encode())
            return
        if path == '/status':
            body = json.dumps(self.server.logger_node._preview_status()).encode()
            self._send(200, 'application/json', body)
            return
        if path in ('/frame/rgb.jpg', '/frame/thermal.jpg'):
            name = 'rgb' if path.endswith('rgb.jpg') else 'thermal'
            body = self.server.logger_node._preview_frame(name)
            if body is None:
                self._send(404, 'text/plain', b'Unavailable')
            else:
                self._send(200, 'image/jpeg', body)
            return
        self._send(404, 'text/plain', b'Not found')

    def do_POST(self):
        path = self.path.split('?', 1)[0]
        if path == '/record/start':
            self.server.logger_node.start_recording()
            self._send(200, 'text/plain', b'Recording started')
            return
        if path == '/record/stop':
            self.server.logger_node.stop_recording()
            self._send(200, 'text/plain', b'Recording stopped')
            return
        self._send(404, 'text/plain', b'Not found')

    def _send(self, status, content_type, body):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_args):
        return


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
        self.declare_parameter('gps_topic', '/gnss')
        self.declare_parameter('odometry_topic', '/wheel_odom')
        self.declare_parameter('capture_rate_hz', CAPTURE_RATE_HZ)
        self.declare_parameter('state_max_age_seconds', 0.5)
        self.declare_parameter('jpeg_quality', 95)
        self.declare_parameter('auto_contrast', AUTO_CONTRAST_ENABLED)
        self.declare_parameter(
            'auto_contrast_clip_limit', AUTO_CONTRAST_CLIP_LIMIT)
        self.declare_parameter('web_preview', True)
        self.declare_parameter('web_port', 8080)
        self.declare_parameter('web_bind_address', '127.0.0.1')

        output = Path(str(self.get_parameter('output_directory').value)).expanduser()
        self._output = output if output.is_absolute() else Path.cwd() / output
        self._rgb_dir = self._output / 'images' / 'rgb'
        self._thermal_dir = self._output / 'images' / 'thermal'
        self._rgb_dir.mkdir(parents=True, exist_ok=True)
        self._thermal_dir.mkdir(parents=True, exist_ok=True)

        self._camera_file, self._camera_writer = self._open_csv(
            self._output / 'camera_log.csv', CAMERA_COLUMNS)
        self._imu_file, self._imu_writer = self._open_csv(
            self._output / 'imu_log.csv', IMU_COLUMNS)
        self._gnss_file, self._gnss_writer = self._open_csv(
            self._output / 'gnss_log.csv', GNSS_COLUMNS)

        self._episode_id = int(self.get_parameter('episode_id').value)
        self._run_start_ns = self.get_clock().now().nanoseconds
        self._rate_hz = float(self.get_parameter('capture_rate_hz').value)
        if self._rate_hz <= 0.0:
            raise ValueError('capture_rate_hz must be greater than zero')
        self._period_ns = int(1e9 / self._rate_hz)
        self._max_state_age_ns = int(float(
            self.get_parameter('state_max_age_seconds').value) * 1e9)
        self._jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self._auto_contrast_enabled = bool(
            self.get_parameter('auto_contrast').value)
        self._auto_contrast_clip_limit = float(
            self.get_parameter('auto_contrast_clip_limit').value)
        if self._auto_contrast_clip_limit <= 0.0:
            raise ValueError('auto_contrast_clip_limit must be greater than zero')
        self._clahe = cv2.createCLAHE(
            clipLimit=self._auto_contrast_clip_limit,
            tileGridSize=AUTO_CONTRAST_TILE_GRID_SIZE)
        self._web_preview = bool(self.get_parameter('web_preview').value)
        self._web_port = int(self.get_parameter('web_port').value)
        self._web_bind_address = str(
            self.get_parameter('web_bind_address').value)
        self._preview_lock = Lock()
        self._preview_rgb = None
        self._preview_thermal = None
        self._preview_server = None
        self._preview_thread = None

        self._rgb_camera = self._try_open_camera(
            self.get_parameter('rgb_camera_index').value, 'RGB')
        self._thermal_camera = self._try_open_camera(
            self.get_parameter('thermal_camera_index').value, 'thermal')
        if self._rgb_camera is None and self._thermal_camera is None:
            self.get_logger().warning('No camera opened; image logging is disabled')

        self._lock = Lock()
        self._last_capture_stamp_ns = None
        self._id = 0
        self._closed = False
        self._recording = False
        if self._web_preview:
            self._start_web_preview()
        self._command_history = deque(maxlen=200)
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

    def _try_open_camera(self, index: int, name: str):
        if int(index) < 0:
            self.get_logger().info(
                f'{name} camera disabled because its index parameter is {index}')
            return None
        try:
            return _open_camera(index)
        except RuntimeError as exc:
            self.get_logger().warning(f'{name} camera unavailable: {exc}')
            return None

    @staticmethod
    def _stamp_ns(msg) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    @staticmethod
    def _utc_stamp(stamp_ns: int) -> str:
        return datetime.fromtimestamp(stamp_ns / 1e9, timezone.utc).strftime(
            '%Y-%m-%dT%H:%M:%S.%fZ')

    def _on_imu(self, msg: Imu) -> None:
        if not getattr(self, '_recording', True):
            return
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
        if not getattr(self, '_recording', True):
            return
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
        rgb_ok, rgb = (False, None)
        thermal_ok, thermal = (False, None)
        if self._rgb_camera is not None:
            rgb_ok, rgb = self._rgb_camera.read()
        if self._thermal_camera is not None:
            thermal_ok, thermal = self._thermal_camera.read()
        if thermal_ok:
            thermal = self._thermal_to_bgr(thermal)
        if self._auto_contrast_enabled:
            if rgb_ok:
                rgb = self._auto_contrast(rgb)
            if thermal_ok:
                thermal = self._auto_contrast(thermal)
        self._update_preview_frames(rgb if rgb_ok else None,
                                     thermal if thermal_ok else None)

        if not rgb_ok and not thermal_ok:
            return

        if not self._recording:
            return

        pair_stamp = self.get_clock().now().nanoseconds
        with self._lock:
            if (self._last_capture_stamp_ns is not None and
                    pair_stamp - self._last_capture_stamp_ns < self._period_ns):
                return
            self._last_capture_stamp_ns = pair_stamp
            command = self._nearest(
                self._command_history, pair_stamp, self._max_state_age_ns)
            odom = self._nearest(
                self._odom_history, pair_stamp, self._max_state_age_ns)

        count = self._id
        stamp_date = datetime.fromtimestamp(pair_stamp / 1e9, timezone.utc).strftime(
            '%Y%m%dT%H%M%S.%fZ')
        rgb_name, thermal_name = _image_names(count, stamp_date)
        flags = [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
        if rgb_ok and not cv2.imwrite(str(self._rgb_dir / rgb_name), rgb, flags):
            self.get_logger().error(f'Failed to write {rgb_name}')
            rgb_name = ''
        if thermal_ok and not cv2.imwrite(
                str(self._thermal_dir / thermal_name), thermal, flags):
            self.get_logger().error(f'Failed to write {thermal_name}')
            thermal_name = ''
        if not rgb_name and not thermal_name:
            return

        time_seconds = (pair_stamp - self._run_start_ns) / 1e9
        row = self._make_camera_row(
            count, self._episode_id, pair_stamp, stamp_date, time_seconds,
            rgb_name, thermal_name, command, odom)
        with self._lock:
            self._camera_writer.writerow(row)
            self._camera_file.flush()
        self._id += 1

    def _start_web_preview(self) -> None:
        try:
            self._preview_server = _PreviewHTTPServer(
                (self._web_bind_address, self._web_port), _PreviewHandler, self)
            self._preview_thread = threading.Thread(
                target=self._preview_server.serve_forever,
                name='data_logger_preview', daemon=True)
            self._preview_thread.start()
            self.get_logger().info(
                f'Web preview available at http://{self._web_bind_address}:{self._web_port}')
        except OSError as exc:
            self._web_preview = False
            self.get_logger().error(f'Could not start web preview: {exc}')

    def _update_preview_frames(self, rgb, thermal) -> None:
        def encode(image):
            if image is None:
                return None
            ok, buffer = cv2.imencode('.jpg', image)
            return buffer.tobytes() if ok else None

        with self._preview_lock:
            self._preview_rgb = encode(rgb)
            self._preview_thermal = encode(thermal)

    def _auto_contrast(self, image):
        """Enhance local luminance contrast without changing image size."""
        if image is None:
            return None
        if len(image.shape) == 2:
            return self._clahe.apply(image)
        if image.shape[2] == 4:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        lightness, channel_a, channel_b = cv2.split(lab)
        lightness = self._clahe.apply(lightness)
        return cv2.cvtColor(
            cv2.merge((lightness, channel_a, channel_b)), cv2.COLOR_LAB2BGR)

    def _preview_status(self):
        with self._preview_lock:
            return {
                'image_count': self._id,
                'recording': self._recording,
                'rgb_available': self._preview_rgb is not None,
                'thermal_available': self._preview_thermal is not None,
            }

    def _preview_frame(self, camera_name):
        with self._preview_lock:
            return (self._preview_rgb if camera_name == 'rgb'
                    else self._preview_thermal)

    def start_recording(self):
        """Start writing data after browser confirmation."""
        with self._lock:
            if not self._recording:
                self._run_start_ns = self.get_clock().now().nanoseconds
                self._last_capture_stamp_ns = None
                self._recording = True
                self.get_logger().info('Recording started from web interface')

    def stop_recording(self):
        """Stop writing data while keeping the camera preview available."""
        with self._lock:
            if self._recording:
                self._recording = False
                for file in (self._camera_file, self._imu_file, self._gnss_file):
                    file.flush()
                self.get_logger().info('Recording stopped from web interface')

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
    def _make_camera_row(count, episode_id, stamp_ns, date, time_seconds,
                         rgb_name, thermal_name, command, odom):
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
            'time_seconds': time_seconds,
            'rgb_image': f'images/rgb/{rgb_name}' if rgb_name else '',
            'thermal_image': f'images/thermal/{thermal_name}' if thermal_name else '',
            'throttle': value(command, 'throttle'),
            'steering': value(command, 'steer'),
            'brake': value(command, 'brake'),
            'mode': value(command, 'mode'),
            'estop': value(command, 'estop'),
            'vehicle_stopped': (
                command is None
                or bool(value(command, 'estop', False))
                or (float(value(command, 'throttle', 0.0)) == 0.0
                    and float(value(command, 'brake', 0.0)) == 0.0)),
            'speed_mps': value(odom, 'twist.twist.linear.x'),
            'yaw_rate_rps': value(odom, 'twist.twist.angular.z'),
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
        if self._rgb_camera is not None:
            self._rgb_camera.release()
        if self._thermal_camera is not None:
            self._thermal_camera.release()
        if self._preview_server is not None:
            self._preview_server.shutdown()
            self._preview_server.server_close()
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
