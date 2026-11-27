"""Publish two USB cameras as timestamped ROS Image topics."""

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraBridgeNode(Node):
    """Small transition bridge for UVC cameras without vendor ROS drivers."""

    def __init__(self) -> None:
        super().__init__('avros_camera_bridge')
        self.declare_parameter('rgb_device', -1)
        self.declare_parameter('thermal_device', -1)
        self.declare_parameter('rgb_topic', '/rgb_camera/image_raw')
        self.declare_parameter('thermal_topic', '/thermal_camera/image_raw')
        self.declare_parameter('rgb_frame_id', 'rgb_camera_optical_frame')
        self.declare_parameter('thermal_frame_id', 'thermal_camera_optical_frame')
        self.declare_parameter('capture_rate_hz', 30.0)
        self.declare_parameter('rgb_width', 680)
        self.declare_parameter('rgb_height', 480)
        self.declare_parameter('thermal_width', 640)
        self.declare_parameter('thermal_height', 512)

        self._rgb = self._open(
            'rgb_device', 'RGB', 'rgb_width', 'rgb_height')
        self._thermal = self._open(
            'thermal_device', 'thermal', 'thermal_width', 'thermal_height')
        self._rgb_pub = self.create_publisher(
            Image, str(self.get_parameter('rgb_topic').value),
            qos_profile_sensor_data)
        self._thermal_pub = self.create_publisher(
            Image, str(self.get_parameter('thermal_topic').value),
            qos_profile_sensor_data)
        rate = float(self.get_parameter('capture_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('capture_rate_hz must be greater than zero')
        self.create_timer(1.0 / rate, self._capture)

    def _open(self, parameter, name, width_parameter, height_parameter):
        device = self.get_parameter(parameter).value
        if isinstance(device, str) and device.lstrip('-').isdigit():
            device = int(device)
        if device is None or device == -1 or device == '':
            self.get_logger().info(f'{name} camera disabled')
            return None
        camera = cv2.VideoCapture(device)
        width = int(self.get_parameter(width_parameter).value)
        height = int(self.get_parameter(height_parameter).value)
        if width > 0:
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height > 0:
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not camera.isOpened():
            camera.release()
            raise RuntimeError(f'Could not open {name} camera: {device}')
        actual_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(
            f'{name} camera requested {width}x{height}; '
            f'driver selected {actual_width}x{actual_height}')
        return camera

    @staticmethod
    def _message(frame, stamp, frame_id):
        if not frame.flags['C_CONTIGUOUS']:
            frame = frame.copy()
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.height, message.width = frame.shape[:2]
        if frame.ndim == 2:
            message.encoding = 'mono16' if frame.dtype == 'uint16' else 'mono8'
        elif frame.shape[2] == 3:
            message.encoding = 'bgr8'
        elif frame.shape[2] == 4:
            message.encoding = 'bgra8'
        else:
            raise ValueError(f'Unsupported camera shape: {frame.shape}')
        message.is_bigendian = 0
        message.step = frame.strides[0]
        message.data = frame.tobytes()
        return message

    def _capture_one(self, camera, frame_id, publisher):
        if camera is None:
            return
        ok = camera.grab()
        if not ok:
            self.get_logger().warning(
                'Camera frame read failed', throttle_duration_sec=2.0)
            return
        stamp = self.get_clock().now().to_msg()
        ok, frame = camera.retrieve()
        if not ok:
            self.get_logger().warning(
                'Camera frame retrieve failed', throttle_duration_sec=2.0)
            return
        publisher.publish(self._message(frame, stamp, frame_id))

    def _capture(self):
        # Grab both devices before retrieving either frame. This minimizes
        # software-induced exposure skew; hardware triggering is still needed
        # for true simultaneous exposure.
        rgb_ok = self._rgb is not None and self._rgb.grab()
        rgb_stamp = self.get_clock().now().to_msg() if rgb_ok else None
        thermal_ok = self._thermal is not None and self._thermal.grab()
        thermal_stamp = self.get_clock().now().to_msg() if thermal_ok else None
        if rgb_ok:
            ok, frame = self._rgb.retrieve()
            if ok:
                self._rgb_pub.publish(self._message(
                    frame, rgb_stamp,
                    str(self.get_parameter('rgb_frame_id').value)))
        if thermal_ok:
            ok, frame = self._thermal.retrieve()
            if ok:
                self._thermal_pub.publish(self._message(
                    frame, thermal_stamp,
                    str(self.get_parameter('thermal_frame_id').value)))

    def destroy_node(self):
        for camera in (self._rgb, self._thermal):
            if camera is not None:
                camera.release()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
