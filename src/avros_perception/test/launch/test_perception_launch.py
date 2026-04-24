"""
Launch test for perception_node with the stub pipeline.

Spawns perception_node in a subprocess, waits for the four output topics
to appear, and asserts that the latched LabelInfo publishes before any
subscriber connects late (kiwicampus's real-world behavior).
"""

import time

import launch
import launch_pytest
import launch_ros.actions
import pytest
import rclpy
from launch_pytest.actions import ReadyToTest
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import LabelInfo


CAM = 'test_front'


@launch_pytest.fixture
def generate_test_description():
    """Spawn perception_node wired to test-only topic names."""
    perception = launch_ros.actions.Node(
        package='avros_perception',
        executable='perception_node',
        name='perception_node',
        parameters=[{
            'camera_name': CAM,
            'rgb_topic': f'/test/{CAM}/image',
            'cloud_topic': f'/test/{CAM}/cloud',
            'pipeline': 'stub',
            'sync_slop': 0.1,
        }],
        output='screen',
    )
    return launch.LaunchDescription([
        perception,
        ReadyToTest(),
    ])


class _TopicWatcher(Node):
    """Subscribes to all four perception outputs and records first-seen."""

    def __init__(self):
        super().__init__('test_topic_watcher')
        self.seen = {'mask': False, 'conf': False, 'points': False, 'label': False}
        self.label_msg = None

        ns = f'/perception/{CAM}'
        self.create_subscription(
            Image, f'{ns}/semantic_mask',
            lambda m: self.seen.__setitem__('mask', True),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, f'{ns}/semantic_confidence',
            lambda m: self.seen.__setitem__('conf', True),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2, f'{ns}/semantic_points',
            lambda m: self.seen.__setitem__('points', True),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LabelInfo, f'{ns}/label_info',
            self._on_label,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def _on_label(self, msg):
        self.seen['label'] = True
        self.label_msg = msg


@pytest.mark.launch(fixture=generate_test_description)
def test_label_info_latches_on_late_subscriber():
    """
    Verify latched LabelInfo publication (transient_local QoS).

    A subscriber created after the node started must still receive the
    latched message — this is how kiwicampus picks it up when the costmap
    plugin loads after perception_node.
    """
    rclpy.init()
    try:
        # Give the launched node a couple seconds to declare its topics.
        time.sleep(2.0)

        watcher = _TopicWatcher()
        executor = SingleThreadedExecutor()
        executor.add_node(watcher)

        deadline = time.time() + 10.0
        while time.time() < deadline and not watcher.seen['label']:
            executor.spin_once(timeout_sec=0.1)

        assert watcher.seen['label'], 'LabelInfo not received within 10 s'
        assert watcher.label_msg is not None
        # Shipped class_map.yaml has at least free (0) and unknown (255).
        names = [v.class_name for v in watcher.label_msg.class_map]
        assert 'free' in names
        assert 'unknown' in names

        watcher.destroy_node()
    finally:
        rclpy.shutdown()
