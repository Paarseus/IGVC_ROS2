"""AVROS WebRTC node: ROS image topic -> Jetson NVENC H.264 -> MediaMTX.

Isolated from webui_node so encoder load / crashes never touch the
control path. This node only encodes; MediaMTX (stock binary, stock
config) serves the stream to browsers over WebRTC (WHEP):

  /zed_front/zed_node/rgb/color/rect/image   (BEST_EFFORT, depth 1)
    -> appsrc -> nvvidconv (VIC, not CPU) -> NV12 (NVMM)
    -> nvv4l2h264enc (Baseline, CBR, short IDR) -> h264parse
    -> rtspclientsink rtsp://127.0.0.1:8554/<path>
  MediaMTX -> http://<jetson>:8889/<path>  (browser page + /whep)

Subscribes to the RAW image only. Never subscribe to the ZED
/compressed or /theora sub-topics: those encode on the CPU inside the
ZED process (the CPU budget that already starved MPPI on 2026-05-29).
"""

import numpy as np

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst  # noqa: E402

import rclpy  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
)
from sensor_msgs.msg import Image  # noqa: E402


# Latest frame only, never back-pressure the ZED wrapper (it publishes
# RELIABLE QoS(10); a BEST_EFFORT subscriber is compatible with that).
SENSOR_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)

# ROS encoding -> (GStreamer raw format, bytes per pixel, needs CPU convert).
# nvvidconv accepts BGRx/RGBA/GRAY8 from system memory directly; 24-bit
# formats need videoconvert first. ZED default is bgra8 -> BGRx (alpha
# ignored, same byte order), so the default path has no CPU conversion.
ENCODINGS = {
    'bgra8': ('BGRx', 4, False),
    'rgba8': ('RGBA', 4, False),
    'mono8': ('GRAY8', 1, False),
    'bgr8': ('BGR', 3, True),
    'rgb8': ('RGB', 3, True),
}


class WebRTCNode(Node):
    """Encode one image topic with NVENC and publish it to MediaMTX."""

    def __init__(self):
        super().__init__('webrtc_node')

        self.declare_parameter(
            'image_topic', '/zed_front/zed_node/rgb/color/rect/image')
        self.declare_parameter(
            'rtsp_url', 'rtsp://127.0.0.1:8554/zed_front')
        # Must match the publisher rate (zed_front.yaml pub_frame_rate);
        # the encoder uses it to size CBR frames.
        self.declare_parameter('frame_rate', 8.0)
        self.declare_parameter('bitrate_bps', 1500000)
        # Frames between IDRs. Short GOP = fast recovery after packet loss
        # and fast first frame for a newly connected browser.
        self.declare_parameter('idr_interval', 8)
        self.declare_parameter('restart_backoff_s', 2.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.rtsp_url = self.get_parameter('rtsp_url').value
        self.frame_rate = float(self.get_parameter('frame_rate').value)
        self.bitrate = int(self.get_parameter('bitrate_bps').value)
        self.idr_interval = int(self.get_parameter('idr_interval').value)
        self.backoff = float(self.get_parameter('restart_backoff_s').value)

        Gst.init(None)
        self._pipeline = None
        self._appsrc = None
        self._shape = None          # (width, height, encoding) of pipeline
        self._retry_at = None       # rclpy Time; None = may build now
        self._frames = 0

        self.create_subscription(
            Image, self.image_topic, self._on_image, SENSOR_QOS)
        self.create_timer(0.2, self._poll_bus)
        self.create_timer(30.0, self._log_stats)

        self.get_logger().info(
            f'WebRTC node started ({self.image_topic} -> {self.rtsp_url}, '
            f'{self.bitrate // 1000} kbps, IDR every {self.idr_interval})'
        )

    # ----- pipeline -------------------------------------------------------

    def _build(self, width: int, height: int, encoding: str):
        gst_format, _, cpu_convert = ENCODINGS[encoding]
        fps_num, fps_den = int(round(self.frame_rate * 1000)), 1000
        convert = 'videoconvert ! video/x-raw,format=BGRx ! ' if cpu_convert else ''
        desc = (
            'appsrc name=src is-live=true format=time do-timestamp=true '
            'block=false max-buffers=1 leaky-type=downstream '
            f'caps=video/x-raw,format={gst_format},width={width},'
            f'height={height},framerate={fps_num}/{fps_den} ! '
            f'{convert}'
            'nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! '
            'nvv4l2h264enc profile=0 control-rate=1 '
            f'bitrate={self.bitrate} '
            f'iframeinterval={self.idr_interval} '
            f'idrinterval={self.idr_interval} '
            'insert-sps-pps=true insert-vui=true poc-type=2 '
            'preset-level=1 maxperf-enable=true ! '
            'h264parse config-interval=-1 ! '
            f'rtspclientsink location={self.rtsp_url} protocols=tcp'
        )
        pipeline = Gst.parse_launch(desc)
        appsrc = pipeline.get_by_name('src')
        if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            pipeline.set_state(Gst.State.NULL)
            raise RuntimeError('pipeline failed to start')
        self._pipeline, self._appsrc = pipeline, appsrc
        self._shape = (width, height, encoding)
        self.get_logger().info(
            f'Streaming {width}x{height} {encoding} @ {self.frame_rate:g} Hz'
            + (' (CPU videoconvert)' if cpu_convert else '')
        )

    def _teardown(self, reason: str):
        if self._pipeline is not None:
            self._pipeline.set_state(Gst.State.NULL)
        self._pipeline = None
        self._appsrc = None
        self._shape = None
        self._retry_at = self.get_clock().now() + Duration(
            seconds=self.backoff)
        self.get_logger().warn(
            f'Pipeline stopped ({reason}); retry in {self.backoff:g} s')

    def _poll_bus(self):
        # No GLib main loop: drain the bus from a ROS timer instead.
        if self._pipeline is None:
            return
        bus = self._pipeline.get_bus()
        while True:
            msg = bus.pop_filtered(
                Gst.MessageType.ERROR | Gst.MessageType.EOS
                | Gst.MessageType.WARNING)
            if msg is None:
                return
            if msg.type == Gst.MessageType.WARNING:
                warn, _ = msg.parse_warning()
                self.get_logger().warn(f'GStreamer: {warn.message}')
                continue
            if msg.type == Gst.MessageType.ERROR:
                err, dbg = msg.parse_error()
                self.get_logger().error(f'GStreamer: {err.message} ({dbg})')
                self._teardown('error')
            else:
                self._teardown('EOS')
            return

    # ----- frames ---------------------------------------------------------

    def _on_image(self, msg: Image):
        if msg.encoding not in ENCODINGS:
            self.get_logger().error(
                f'Unsupported encoding {msg.encoding!r}',
                throttle_duration_sec=10.0)
            return

        shape = (msg.width, msg.height, msg.encoding)
        if self._pipeline is not None and shape != self._shape:
            self._teardown(f'image changed to {shape}')
            self._retry_at = None   # rebuild immediately for a new shape
        if self._pipeline is None:
            if self._retry_at is not None and self.get_clock().now() < self._retry_at:
                return
            try:
                self._build(*shape)
            except Exception as e:  # GLib.Error from parse_launch, RuntimeError
                self.get_logger().error(f'Pipeline build failed: {e}')
                self._teardown('build failed')
                return

        bpp = ENCODINGS[msg.encoding][1]
        row = msg.width * bpp
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.step != row:   # strip row padding
            data = data.reshape(msg.height, msg.step)[:, :row]
        self._appsrc.emit('push-buffer', Gst.Buffer.new_wrapped(data.tobytes()))
        self._frames += 1

    def _log_stats(self):
        self.get_logger().info(
            f'{self._frames} frames pushed in last 30 s'
            + ('' if self._pipeline is not None else ' (pipeline down)'))
        self._frames = 0

    def destroy_node(self):
        if self._pipeline is not None:
            self._pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebRTCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
