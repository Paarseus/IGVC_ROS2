"""Unit tests for the lidar_preprocessor motion-deskew helpers."""

import numpy as np
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from avros_perception.lidar_preprocessor import (
    deskew_buckets,
    deskew_points,
    pointcloud2_to_xyz_t_arrays,
    range_filter,
    range_mask,
)


def _yaw_matrix(yaw, tx=0.0):
    c, s = np.cos(yaw), np.sin(yaw)
    m = np.eye(4)
    m[:2, :2] = [[c, -s], [s, c]]
    m[0, 3] = tx
    return m


def test_buckets_cover_span_evenly():
    t = np.linspace(0.0, 0.1, 1001)
    idx, centres = deskew_buckets(t, 16)
    assert idx.min() == 0 and idx.max() == 15
    assert np.allclose(centres, (np.arange(16) + 0.5) * 0.1 / 16)
    # every point is within half a bucket of its centre
    assert np.max(np.abs(t - centres[idx])) <= 0.1 / 32 + 1e-12


def test_buckets_degenerate_cases():
    idx, centres = deskew_buckets(np.zeros(0), 16)
    assert idx.shape == (0,) and centres.shape == (0,)
    idx, centres = deskew_buckets(np.full(5, 0.02), 16)  # zero span
    assert (idx == 0).all() and np.allclose(centres, [0.02])
    idx, centres = deskew_buckets(np.linspace(0, 0.1, 10), 1)
    assert (idx == 0).all() and centres.shape == (1,)


def test_identity_matrices_leave_points_unchanged():
    pts = np.random.default_rng(0).normal(size=(200, 3))
    idx = np.random.default_rng(1).integers(0, 4, size=200)
    out = deskew_points(pts, idx, [np.eye(4)] * 4)
    assert np.allclose(out, pts)


def test_each_bucket_gets_its_own_transform():
    pts = np.array([[5.0, 0.0, 0.0], [5.0, 0.0, 0.0], [5.0, 0.0, 0.0]])
    idx = np.array([0, 1, 2])
    mats = [np.eye(4), _yaw_matrix(np.pi / 2), _yaw_matrix(0.0, tx=-0.3)]
    out = deskew_points(pts, idx, mats)
    assert np.allclose(out[0], [5.0, 0.0, 0.0])
    assert np.allclose(out[1], [0.0, 5.0, 0.0])
    assert np.allclose(out[2], [4.7, 0.0, 0.0])


def test_turning_scan_of_a_wall_is_straightened():
    # Robot yaws at w while scanning a point fixed in the world at (5, 0).
    # In the sensor frame at time t the point appears rotated by -w*t; the
    # transform sensor@t -> sensor@0 is a +w*t yaw, which must undo it.
    w = 1.5
    t = np.linspace(0.0, 0.1, 64)
    world = np.array([5.0, 0.0, 0.0])
    seen = np.stack([_yaw_matrix(-w * ti)[:3, :3] @ world for ti in t])
    idx, centres = deskew_buckets(t, 16)
    mats = [_yaw_matrix(w * c) for c in centres]
    out = deskew_points(seen, idx, mats)
    raw_err = np.linalg.norm(seen - world, axis=1).max()
    fixed_err = np.linalg.norm(out - world, axis=1).max()
    assert raw_err > 0.7          # ~0.75 m smear at 5 m, as documented
    assert fixed_err < 0.03       # half-bucket residual only


def test_range_mask_matches_range_filter():
    pts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [20.0, 0.0, 0.0]])
    mask = range_mask(pts, 0.7, 15.0)
    assert mask.tolist() == [False, True, False]
    assert np.allclose(range_filter(pts, 0.7, 15.0), pts[mask])


def _cloud(fields, rows):
    return point_cloud2.create_cloud(Header(frame_id="os_sensor"), fields, rows)


def test_reads_ouster_time_field_in_seconds():
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="t", offset=12, datatype=PointField.UINT32, count=1),
    ]
    rows = [(1.0, 2.0, 3.0, 0), (float("nan"), 0.0, 0.0, 5), (4.0, 5.0, 6.0, 50_000_000)]
    xyz, t = pointcloud2_to_xyz_t_arrays(_cloud(fields, rows))
    assert xyz.shape == (2, 3)
    assert np.allclose(t, [0.0, 0.05])


def test_missing_time_field_returns_none():
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    xyz, t = pointcloud2_to_xyz_t_arrays(_cloud(fields, [(1.0, 2.0, 3.0)]))
    assert t is None and xyz.shape == (1, 3)
