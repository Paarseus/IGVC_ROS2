"""Lane detector node: RealSense color + aligned depth -> /lane_points PointCloud2.

Classical HSV threshold on the D455 color stream, masked into the aligned depth
image, deprojected to 3D via the camera's pinhole intrinsics, published as a
PointCloud2 in camera_color_optical_frame. Consumed by Nav2 as a dedicated
ObstacleLayer observation source (see nav2_params.yaml -> lane_obstacle_layer).

HSV masking pattern is Apache-2.0 and adapted in spirit from ROBOTIS
turtlebot3_autorace_detect (https://github.com/ROBOTIS-GIT/turtlebot3_autorace).
The downstream half (sliding window / polynomial fit / Float64 center offset)
is discarded; we replace it with depth deprojection and PointCloud2 output
for the standard Nav2 costmap contract.

Subscribes:
  /camera/camera/color/image_raw                 sensor_msgs/Image  (synced)
  /camera/camera/aligned_depth_to_color/image_raw sensor_msgs/Image (synced)
  /camera/camera/color/camera_info               sensor_msgs/CameraInfo (cached)

Publishes:
  /lane_points                                   sensor_msgs/PointCloud2
  /lane_detector/debug_mask                      sensor_msgs/Image (mono8)

Parameters: see declare_parameter block in __init__. All HSV/morphology/ROI/
range params are live-tunable via rqt_reconfigure.
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from rcl_interfaces.msg import SetParametersResult


class LaneDetectorNode(Node):
    """HSV lane detector + depth deprojection."""

    def __init__(self):
        super().__init__('lane_detector_node')

        self._declare_params()
        self._cache_params()
        self.add_on_set_parameters_callback(self._on_params_change)

        self._bridge = CvBridge()
        self._model = PinholeCameraModel()
        self._model_ready = False

        color_topic = self.get_parameter('color_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        info_topic = self.get_parameter('camera_info_topic').value

        # camera_info is cached once (intrinsics are stable for a fixed D455
        # config). It is NOT in the sync — some RealSense configs publish it
        # at a different cadence than the image streams, which would starve a
        # 3-way ApproximateTimeSync.
        self._info_sub = self.create_subscription(
            CameraInfo, info_topic, self._info_cb, qos_profile_sensor_data)

        color_sub = Subscriber(
            self, Image, color_topic, qos_profile=qos_profile_sensor_data)
        depth_sub = Subscriber(
            self, Image, depth_topic, qos_profile=qos_profile_sensor_data)
        self._sync = ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=10, slop=self._sync_slop)
        self._sync.registerCallback(self._on_image)

        self._cloud_pub = self.create_publisher(PointCloud2, '/lane_points', 10)
        self._debug_pub = self.create_publisher(
            Image, '/lane_detector/debug_mask', 1)

        self.get_logger().info(
            f'lane_detector_node ready (color={color_topic}, depth={depth_topic})')

    # ---- parameters ----------------------------------------------------

    def _declare_params(self):
        # Topics (fixed at launch, not live-tunable)
        self.declare_parameter(
            'color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter(
            'depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter(
            'camera_info_topic', '/camera/camera/color/camera_info')

        # White HSV bounds (IGVC primary)
        self.declare_parameter('h_min_white', 0)
        self.declare_parameter('h_max_white', 179)
        self.declare_parameter('s_min_white', 0)
        self.declare_parameter('s_max_white', 60)
        self.declare_parameter('v_min_white', 180)
        self.declare_parameter('v_max_white', 255)

        # Yellow HSV bounds (disabled by default; toggle for road use)
        self.declare_parameter('enable_yellow', False)
        self.declare_parameter('h_min_yellow', 15)
        self.declare_parameter('h_max_yellow', 40)
        self.declare_parameter('s_min_yellow', 80)
        self.declare_parameter('s_max_yellow', 255)
        self.declare_parameter('v_min_yellow', 100)
        self.declare_parameter('v_max_yellow', 255)

        # Morphology + ROI
        self.declare_parameter('erode_kernel', 3)
        self.declare_parameter('close_kernel', 7)
        self.declare_parameter('roi_y_start_frac', 0.45)

        # Depth filter + subsample
        self.declare_parameter('min_depth_m', 0.3)
        self.declare_parameter('max_lane_range_m', 5.0)
        self.declare_parameter('pixel_stride', 4)

        # Sync
        self.declare_parameter('sync_slop_s', 0.033)

    def _cache_params(self):
        gp = self.get_parameter
        self._white_lo = np.array(
            [gp('h_min_white').value, gp('s_min_white').value, gp('v_min_white').value],
            dtype=np.uint8)
        self._white_hi = np.array(
            [gp('h_max_white').value, gp('s_max_white').value, gp('v_max_white').value],
            dtype=np.uint8)
        self._yellow_lo = np.array(
            [gp('h_min_yellow').value, gp('s_min_yellow').value, gp('v_min_yellow').value],
            dtype=np.uint8)
        self._yellow_hi = np.array(
            [gp('h_max_yellow').value, gp('s_max_yellow').value, gp('v_max_yellow').value],
            dtype=np.uint8)
        self._enable_yellow = gp('enable_yellow').value

        erode_k = max(1, int(gp('erode_kernel').value)) | 1  # force odd
        close_k = max(1, int(gp('close_kernel').value)) | 1
        self._erode_mat = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (erode_k, erode_k))
        self._close_mat = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (close_k, close_k))

        self._roi_y_start = float(gp('roi_y_start_frac').value)
        self._min_depth = float(gp('min_depth_m').value)
        self._max_range = float(gp('max_lane_range_m').value)
        self._stride = max(1, int(gp('pixel_stride').value))
        self._sync_slop = float(gp('sync_slop_s').value)

    def _on_params_change(self, _params):
        # Re-cache on any change; sync slop takes effect on next restart.
        self._cache_params()
        return SetParametersResult(successful=True)

    # ---- callbacks -----------------------------------------------------

    def _info_cb(self, msg):
        if not self._model_ready:
            self._model.fromCameraInfo(msg)
            self._model_ready = True
            self.get_logger().info(
                f'CameraInfo cached: fx={self._model.fx():.1f}, '
                f'fy={self._model.fy():.1f}, '
                f'cx={self._model.cx():.1f}, cy={self._model.cy():.1f}')

    def _on_image(self, color_msg, depth_msg):
        if not self._model_ready:
            return

        try:
            bgr = self._bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            depth = self._bridge.imgmsg_to_cv2(depth_msg)
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge decode failed: {exc}')
            return

        h, w = bgr.shape[:2]
        y0 = int(h * self._roi_y_start)
        roi = bgr[y0:, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._white_lo, self._white_hi)
        if self._enable_yellow:
            mask |= cv2.inRange(hsv, self._yellow_lo, self._yellow_hi)

        mask = cv2.erode(mask, self._erode_mat, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._close_mat)

        # Full-size debug mask (ROI painted back into the original frame).
        full_mask = np.zeros((h, w), dtype=np.uint8)
        full_mask[y0:, :] = mask
        dbg = self._bridge.cv2_to_imgmsg(full_mask, 'mono8')
        dbg.header = color_msg.header
        self._debug_pub.publish(dbg)

        ys, xs = np.nonzero(mask)
        if ys.size == 0:
            return
        if self._stride > 1:
            ys = ys[::self._stride]
            xs = xs[::self._stride]
        ys_full = ys + y0

        # Depth units: RealSense aligned_depth is uint16 mm; float32 m on some
        # configs. Branch on actual dtype rather than assuming.
        raw = depth[ys_full, xs]
        if depth.dtype == np.uint16:
            z = raw.astype(np.float32) * 0.001
        else:
            z = raw.astype(np.float32)

        valid = (z > self._min_depth) & (z < self._max_range) & np.isfinite(z)
        if not valid.any():
            return
        z = z[valid]
        u = xs[valid].astype(np.float32)
        v = ys_full[valid].astype(np.float32)

        fx = self._model.fx()
        fy = self._model.fy()
        cx = self._model.cx()
        cy = self._model.cy()
        x3 = (u - cx) * z / fx
        y3 = (v - cy) * z / fy
        points = np.column_stack((x3, y3, z))

        hdr = Header()
        hdr.stamp = color_msg.header.stamp
        hdr.frame_id = color_msg.header.frame_id
        self._cloud_pub.publish(create_cloud_xyz32(hdr, points.tolist()))


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
