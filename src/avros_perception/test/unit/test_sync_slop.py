"""
ApproximateTimeSynchronizer slop behavior tests.

Mirrors the pattern in ros2/message_filters test/test_approxsync.py.
Pushes mock Image and PointCloud2 messages through a synchronizer with
controlled stamp offsets, asserting the callback fires for stamps within
slop and drops for stamps beyond slop. Catches stamp-drift regressions
that would silently desync mask + cloud at runtime and cause kiwicampus
to miss observations.
"""

import pytest
from builtin_interfaces.msg import Time
from message_filters import ApproximateTimeSynchronizer

from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header


SLOP = 0.02  # matches perception.yaml default


class _DummySubscriber:
    """
    Minimal stand-in for message_filters.Subscriber.

    message_filters.SimpleFilter exposes signalMessage(); we emulate it by
    holding the registered callbacks in a list and letting the test push
    messages through the synchronizer directly. Importing SimpleFilter is
    fragile across rolling/humble so we re-implement the tiny interface.
    """

    def __init__(self):
        self.callbacks = []

    def registerCallback(self, cb, *args):
        self.callbacks.append((cb, args))
        return 0

    def signalMessage(self, msg):
        for cb, args in self.callbacks:
            cb(msg, *args)


def _make_header(sec: int, nanosec: int, frame_id: str = 'test'):
    h = Header()
    h.frame_id = frame_id
    h.stamp = Time(sec=sec, nanosec=nanosec)
    return h


def _make_image(sec: int, nanosec: int):
    msg = Image()
    msg.header = _make_header(sec, nanosec)
    msg.height = 10
    msg.width = 10
    msg.encoding = 'bgr8'
    msg.step = 30
    msg.data = bytes(300)
    return msg


def _make_cloud(sec: int, nanosec: int):
    msg = PointCloud2()
    msg.header = _make_header(sec, nanosec)
    msg.height = 10
    msg.width = 10
    msg.point_step = 12
    msg.row_step = 120
    msg.is_dense = True
    msg.data = bytes(1200)
    return msg


@pytest.fixture
def sync_pair():
    """Build an ApproximateTimeSynchronizer over two dummy subscribers."""
    img_sub = _DummySubscriber()
    cloud_sub = _DummySubscriber()
    ts = ApproximateTimeSynchronizer([img_sub, cloud_sub], queue_size=10, slop=SLOP)
    paired = []
    ts.registerCallback(lambda img, cloud: paired.append((img, cloud)))
    return img_sub, cloud_sub, paired


def test_identical_stamps_pair_immediately(sync_pair):
    """Image + cloud with the exact same stamp must pair on arrival."""
    img_sub, cloud_sub, paired = sync_pair
    img_sub.signalMessage(_make_image(1000, 0))
    cloud_sub.signalMessage(_make_cloud(1000, 0))
    assert len(paired) == 1
    assert paired[0][0].header.stamp.sec == 1000
    assert paired[0][1].header.stamp.sec == 1000


def test_stamps_within_slop_pair(sync_pair):
    """Stamps offset by < slop must still pair."""
    img_sub, cloud_sub, paired = sync_pair
    # Image at t=1000.000, cloud at t=1000.010 (10 ms < 20 ms slop)
    img_sub.signalMessage(_make_image(1000, 0))
    cloud_sub.signalMessage(_make_cloud(1000, 10_000_000))
    assert len(paired) == 1


def test_stamps_beyond_slop_do_not_pair(sync_pair):
    """Stamps offset by > slop must not produce a pair."""
    img_sub, cloud_sub, paired = sync_pair
    # 30 ms apart — well beyond 20 ms slop
    img_sub.signalMessage(_make_image(1000, 0))
    cloud_sub.signalMessage(_make_cloud(1000, 30_000_000))
    assert paired == []


def test_second_image_replaces_unmatched_first(sync_pair):
    """
    Newer image supersedes an earlier one when the matching cloud arrives.

    Image frames arriving faster than cloud frames must pair against the
    most recent image at match time. This is the behavior kiwicampus
    relies on when perception outpaces the depth pipeline.
    """
    img_sub, cloud_sub, paired = sync_pair
    img_sub.signalMessage(_make_image(1000, 0))
    img_sub.signalMessage(_make_image(1000, 100_000_000))   # +100 ms
    cloud_sub.signalMessage(_make_cloud(1000, 105_000_000))  # matches 2nd img
    assert len(paired) == 1
    assert paired[0][0].header.stamp.nanosec == 100_000_000


def test_cloud_arrives_first_then_matching_image(sync_pair):
    """Order-independence: cloud first, image second, pair on match."""
    img_sub, cloud_sub, paired = sync_pair
    cloud_sub.signalMessage(_make_cloud(1000, 0))
    img_sub.signalMessage(_make_image(1000, 5_000_000))
    assert len(paired) == 1


def test_slop_boundary_is_inclusive_of_small_offset():
    """Stamps offset by exactly slop/2 (well inside) must always pair."""
    img_sub = _DummySubscriber()
    cloud_sub = _DummySubscriber()
    paired = []
    ts = ApproximateTimeSynchronizer([img_sub, cloud_sub], queue_size=10, slop=SLOP)
    ts.registerCallback(lambda img, cloud: paired.append((img, cloud)))

    half_slop_ns = int(SLOP / 2 * 1e9)
    img_sub.signalMessage(_make_image(1000, 0))
    cloud_sub.signalMessage(_make_cloud(1000, half_slop_ns))
    assert len(paired) == 1
