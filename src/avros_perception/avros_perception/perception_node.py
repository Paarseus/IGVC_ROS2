"""
Camera perception bridge.

Subscribes to one ZED camera's rectified RGB + organized PointCloud2, runs a
pluggable Pipeline to produce a mono8 class-ID mask, and publishes the
contract kiwicampus/semantic_segmentation_layer consumes:

    mask        (sensor_msgs/Image, mono8, H x W)
    confidence  (sensor_msgs/Image, mono8, H x W)
    cloud       (sensor_msgs/PointCloud2, organized, H x W) — relayed
    labels      (vision_msgs/LabelInfo, latched once)

All four outputs carry the SAME header.stamp (the image's) — kiwicampus
message-filters them and drops frames with mismatched stamps.
"""

import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import LabelInfo

from avros_perception.pipelines import build_pipeline
from avros_perception.utils.class_map import build_label_info, load_class_map


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # ---- parameters ----
        self.declare_parameter('camera_name', 'front')
        self.declare_parameter('rgb_topic', '')
        self.declare_parameter('cloud_topic', '')
        self.declare_parameter('pipeline', 'stub')
        self.declare_parameter('sync_slop', 0.02)
        self.declare_parameter('sync_queue', 10)
        self.declare_parameter('class_map_path', '')
        # Stub-pipeline runtime knobs; ignored by other pipelines.
        self.declare_parameter('inject_stripe_width', 0)
        self.declare_parameter('inject_stripe_start', -1)
        self.declare_parameter('inject_class_id', 1)

        cam = self.get_parameter('camera_name').value
        # zed-ros2-wrapper v5.x topic names:
        #   rgb/color/rect/image       (not rgb/image_rect_color — v4.x only)
        #   point_cloud/cloud_registered
        rgb_topic = self.get_parameter('rgb_topic').value or \
            f'/zed_{cam}/zed_node/rgb/color/rect/image'
        cloud_topic = self.get_parameter('cloud_topic').value or \
            f'/zed_{cam}/zed_node/point_cloud/cloud_registered'
        pipeline_name = self.get_parameter('pipeline').value
        sync_slop = float(self.get_parameter('sync_slop').value)
        sync_queue = int(self.get_parameter('sync_queue').value)

        class_map_path = self.get_parameter('class_map_path').value
        if not class_map_path:
            class_map_path = os.path.join(
                get_package_share_directory('avros_perception'),
                'config', 'class_map.yaml'
            )

        # ---- class map ----
        self._classes = load_class_map(class_map_path)
        self.get_logger().info(
            f'Loaded {len(self._classes)} classes from {class_map_path}'
        )
        # OpenCV uses BGR; class_map.yaml stores RGB. Flip once at init.
        self._class_colors_bgr = {
            e.id: (int(e.rgb[2]), int(e.rgb[1]), int(e.rgb[0]))
            for e in self._classes if e.id != 0
        }

        # ---- pipeline ----
        self._pipeline_params = {
            'inject_stripe_width': self.get_parameter('inject_stripe_width').value,
            'inject_stripe_start': self.get_parameter('inject_stripe_start').value,
            'inject_class_id': self.get_parameter('inject_class_id').value,
        }
        self._pipeline = build_pipeline(
            pipeline_name, self._pipeline_params, self.get_logger()
        )
        self._pipeline.warmup()
        self.get_logger().info(f"Pipeline active: {pipeline_name}")

        # React to runtime param changes (only the stub knobs for now)
        self._stub_param_names = {
            'inject_stripe_width', 'inject_stripe_start', 'inject_class_id',
        }
        self.add_on_set_parameters_callback(self._on_set_params)

        # ---- publishers ----
        ns = f'/perception/{cam}'
        self._mask_pub = self.create_publisher(
            Image, f'{ns}/semantic_mask', qos_profile_sensor_data
        )
        self._confidence_pub = self.create_publisher(
            Image, f'{ns}/semantic_confidence', qos_profile_sensor_data
        )
        self._cloud_pub = self.create_publisher(
            PointCloud2, f'{ns}/semantic_points', qos_profile_sensor_data
        )
        # Debug: RGB with mask pixels tinted per class. Useful in Foxglove
        # to see WHAT is being classified without splitting panels.
        self._overlay_pub = self.create_publisher(
            Image, f'{ns}/overlay', qos_profile_sensor_data
        )
        label_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._label_pub = self.create_publisher(
            LabelInfo, f'{ns}/label_info', label_qos
        )

        # Latched LabelInfo — published once, new subscribers get the last value.
        label_msg = build_label_info(self._classes)
        label_msg.header.stamp = self.get_clock().now().to_msg()
        self._label_pub.publish(label_msg)
        self.get_logger().info(f'Latched LabelInfo on {ns}/label_info')

        # ---- subscribers + time-sync ----
        self._bridge = CvBridge()
        self._rgb_sub = Subscriber(
            self, Image, rgb_topic, qos_profile=qos_profile_sensor_data
        )
        self._cloud_sub = Subscriber(
            self, PointCloud2, cloud_topic, qos_profile=qos_profile_sensor_data
        )
        self._sync = ApproximateTimeSynchronizer(
            [self._rgb_sub, self._cloud_sub],
            queue_size=sync_queue,
            slop=sync_slop,
        )
        self._sync.registerCallback(self._on_synced)

        self.get_logger().info(
            f'Subscribed: {rgb_topic} + {cloud_topic} (slop {sync_slop}s)'
        )

    def _on_set_params(self, params):
        # Update the pipeline's shared params dict in-place; pipeline reads
        # on next run(). Only touch keys that belong to the stub pipeline.
        for p in params:
            if p.name in self._stub_param_names:
                self._pipeline_params[p.name] = p.value
        return SetParametersResult(successful=True)

    def _on_synced(self, image: Image, cloud: PointCloud2):
        try:
            # BGR matches OpenCV convention (cv2.cvtColor COLOR_BGR2HSV etc).
            bgr = self._bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge failed: {e}')
            return

        # ZED wrapper may publish image and cloud at different resolutions
        # (image respects pub_downscale_factor; cloud uses point_cloud_res).
        # kiwicampus indexes cloud[v,u] to look up 3D per mask pixel, so mask
        # and cloud MUST have identical HxW. Resize the image down to the
        # cloud's shape when they differ — cloud is the costmap-side truth.
        if bgr.shape[:2] != (cloud.height, cloud.width):
            bgr = cv2.resize(
                bgr, (cloud.width, cloud.height),
                interpolation=cv2.INTER_AREA,
            )
        h, w = bgr.shape[:2]

        result = self._pipeline.run(bgr)
        if result.mask.shape != (h, w):
            self.get_logger().error(
                f'pipeline produced mask shape {result.mask.shape} != image {(h, w)}'
            )
            return

        stamp = image.header.stamp
        frame = image.header.frame_id

        mask_msg = self._bridge.cv2_to_imgmsg(result.mask, encoding='mono8')
        mask_msg.header.stamp = stamp
        mask_msg.header.frame_id = frame
        self._mask_pub.publish(mask_msg)

        conf_msg = self._bridge.cv2_to_imgmsg(result.confidence, encoding='mono8')
        conf_msg.header.stamp = stamp
        conf_msg.header.frame_id = frame
        self._confidence_pub.publish(conf_msg)

        # Overlay: 50/50 blend of BGR with per-class tint where mask > 0.
        overlay = bgr.copy()
        for cid, color in self._class_colors_bgr.items():
            sel = result.mask == cid
            if sel.any():
                overlay[sel] = (
                    0.5 * overlay[sel].astype(np.float32)
                    + 0.5 * np.array(color, dtype=np.float32)
                ).astype(np.uint8)
        overlay_msg = self._bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        overlay_msg.header.stamp = stamp
        overlay_msg.header.frame_id = frame
        self._overlay_pub.publish(overlay_msg)

        # Relay the organized cloud under our namespace, stamped identically
        # to the mask so kiwicampus's time-sync passes.
        cloud.header.stamp = stamp
        self._cloud_pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
