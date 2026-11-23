"""Unit tests for lidar_line_detector on a synthetic Ouster-like scan of flat asphalt."""

import numpy as np

from avros_perception.lidar_line_detector import (
    LineParams,
    detect_lines,
    filter_runs,
    line_candidates,
    ring_background,
)

SENSOR_H = 0.67
H, W = 64, 1024


def _scan(stripes=(), boxes=(), asphalt=10.0, noise=2.0, seed=0):
    """
    Organized scan of flat ground seen from SENSOR_H (sensor frame = base_link
    shifted up by SENSOR_H, no rotation).

    stripes: (y_min, y_max, reflectivity) painted bands parallel to x.
    boxes:   (x_min, x_max, y_min, y_max, height, reflectivity) solid boxes.
    """
    rng = np.random.default_rng(seed)
    elev = np.radians(np.linspace(21.05, -21.39, H))[:, None]
    az = np.linspace(0, 2 * np.pi, W, endpoint=False)[None, :]
    with np.errstate(divide="ignore"):
        r_ground = np.where(elev < 0, SENSOR_H / np.sin(-elev), np.inf)
    r = np.broadcast_to(r_ground, (H, W)).copy()
    dirs = np.stack(
        (np.cos(elev) * np.cos(az), np.cos(elev) * np.sin(az),
         np.broadcast_to(np.sin(elev), (H, W))), axis=2)
    refl = asphalt + noise * rng.standard_normal((H, W))
    # Boxes: vertical faces approximated by intersecting rays with the box volume.
    for x0, x1, y0, y1, hgt, rf in boxes:
        for rr in np.linspace(0.5, 20, 400):
            p = dirs * rr
            pz = p[..., 2] + SENSOR_H
            inside = ((p[..., 0] >= x0) & (p[..., 0] <= x1) & (p[..., 1] >= y0)
                      & (p[..., 1] <= y1) & (pz >= 0) & (pz <= hgt) & (rr < r))
            r[inside] = rr
            refl[inside] = rf
    r[~np.isfinite(r)] = 0.0  # no return
    xyz = dirs * r[..., None]
    ground_hit = (r > 0) & np.isclose(xyz[..., 2], -SENSOR_H, atol=1e-6)
    for y0, y1, rf in stripes:
        on = ground_hit & (xyz[..., 1] >= y0) & (xyz[..., 1] <= y1)
        refl[on] = rf
    t = np.eye(4)
    t[2, 3] = SENSOR_H
    return xyz, np.clip(refl, 0, 255), t


def test_background_ignores_bright_paint():
    refl = np.full((1, 200), 10.0)
    refl[0, 100:104] = 80.0
    bg = ring_background(refl, np.ones_like(refl, bool), 61, 1.5, 8)
    assert np.allclose(bg[0, 100:104], 10.0)


def test_background_nan_without_support():
    refl = np.full((1, 100), 10.0)
    ground = np.zeros_like(refl, bool)
    ground[0, :3] = True
    bg = ring_background(refl, ground, 21, 1.5, 8)
    assert np.isnan(bg).all()


def test_candidates_need_absolute_and_relative_contrast():
    bg = np.array([[10.0, 10.0, 60.0, np.nan]])
    refl = np.array([[15.0, 40.0, 90.0, 90.0]])
    m = line_candidates(refl, bg, np.ones_like(refl, bool), 20.0, 2.0, 12.0)
    assert m.tolist() == [[False, True, False, False]]


def test_filter_runs_drops_speckle_and_wide_patches():
    mask = np.zeros((1, 1024), bool)
    mask[0, 10] = True              # single point
    mask[0, 100:104] = True         # 4 points at 3 m: ~7 cm
    mask[0, 500:700] = True         # 200 points at 3 m: ~3.7 m
    out = filter_runs(mask, np.full(mask.shape, 3.0), 2, 2.5)
    assert not out[0, 10]
    assert out[0, 100:104].all()
    assert not out[0, 500:700].any()


def test_detects_painted_line_beside_robot():
    xyz, refl, t = _scan(stripes=[(1.5, 1.576, 60.0)])
    res = detect_lines(xyz, refl, t, LineParams())
    assert res.lines_xyz.shape[0] > 20
    y = res.lines_xyz[:, 1]
    assert np.all((y > 1.45) & (y < 1.63))
    assert np.allclose(res.lines_xyz[:, 2], 0.0, atol=0.02)
    assert abs(res.plane[2]) > 0.99


def test_no_lines_on_plain_asphalt():
    xyz, refl, t = _scan()
    res = detect_lines(xyz, refl, t, LineParams())
    assert res.lines_xyz.shape[0] == 0


def test_uniformly_bright_surface_is_not_a_line():
    xyz, refl, t = _scan(asphalt=60.0, noise=3.0)
    res = detect_lines(xyz, refl, t, LineParams())
    assert res.lines_xyz.shape[0] == 0


def test_bright_obstacle_is_not_a_line():
    # White barrel-like box: bright but well above the ground.
    xyz, refl, t = _scan(boxes=[(3.0, 3.5, -0.3, 0.3, 1.0, 90.0)])
    res = detect_lines(xyz, refl, t, LineParams())
    near_box = (
        (res.lines_xyz[:, 0] > 2.9) & (res.lines_xyz[:, 0] < 3.6)
        & (np.abs(res.lines_xyz[:, 1]) < 0.4)
    )
    assert not near_box.any()


def test_handles_tilted_mount():
    # 3 deg pitch of the sensor: the fitted plane should absorb it.
    xyz, refl, t = _scan(stripes=[(-2.076, -2.0, 70.0)])
    a = np.radians(3.0)
    rot = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
    xyz_tilted = xyz @ rot  # sensor sees the world rotated by -a
    t_tilt = t.copy()
    t_tilt[:3, :3] = rot  # base_link <- sensor undoes it
    res = detect_lines(xyz_tilted, refl, t_tilt, LineParams())
    assert res.lines_xyz.shape[0] > 20
    assert np.all(np.abs(res.lines_xyz[:, 1] + 2.04) < 0.1)


def test_rear_half_is_ignored():
    # Painted lines on both sides; one crosses in front, one only behind.
    xyz, refl, t = _scan(stripes=[(1.5, 1.576, 60.0)])
    behind = (xyz[..., 0] < 0)
    refl_rear = refl.copy()
    refl_rear[~behind] = 10.0          # paint only behind the robot
    res = detect_lines(xyz, refl_rear, t, LineParams())
    assert res.lines_xyz.shape[0] == 0
    assert np.all(res.ground_xyz[:, 0] >= -1e-6)
    res360 = detect_lines(xyz, refl_rear, t, LineParams(fov_deg=360.0))
    assert res360.lines_xyz.shape[0] > 10


def test_points_within_10_inches_are_ignored():
    # Footprint box shrunk away so only the range gate can remove these points.
    no_body = dict(footprint_x_min=0.0, footprint_x_max=0.0,
                   footprint_y_min=0.0, footprint_y_max=0.0, ground_tolerance=10.0)
    xyz, refl, t = _scan()
    xyz = xyz.copy()
    xyz[63, :10] = [0.20, 0.0, -0.10]   # 0.224 m from the sensor -> dropped
    xyz[63, 10:20] = [0.25, 0.0, -0.10]  # 0.269 m -> kept
    res = detect_lines(xyz, refl, t, LineParams(**no_body))
    near_x = res.ground_xyz[np.isclose(res.ground_xyz[:, 2], 0.57), 0]
    assert not np.isclose(near_x, 0.20).any()
    assert np.isclose(near_x, 0.25).sum() == 10
