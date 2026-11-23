"""
LiDAR white-line detection on asphalt from Ouster OS1 reflectivity.

Painted lines (and especially lines with retroreflective glass beads) return
much more light than asphalt, and the Ouster reports a range-calibrated
`reflectivity` for every point. Within one beam (one row of the organized
64 x 1024 scan) the asphalt next to a line was hit at almost the same range
and incidence angle, so the line shows up as a bright run against a local
background. The detector therefore works ring by ring:

  1. Organized cloud -> per-point x/y/z/reflectivity/t arrays (H x W).
  2. Optional motion deskew (shared helpers from lidar_preprocessor).
  3. Transform to base_link, fit and smooth the ground plane (RANSAC), and
     keep points within ground_tolerance of it (painted lines have no height).
  4. Per ring: a circular rolling mean of ground reflectivity, recomputed
     with bright outliers removed (so the line itself does not raise its own
     background).
  5. A point is a line candidate if it is ground and its reflectivity is
     >= min_reflectivity, >= contrast_ratio x background and
     >= background + contrast_delta.
  6. Runs of candidates along each ring are kept only if they have at least
     min_run_points points and are at most max_run_width_m wide, which
     removes single-point speckle and large bright patches (concrete pads,
     painted areas).

Topics:
  in   /ouster/points                         organized PointCloud2 (native)
  out  /perception/lidar/lines                base_link, x y z intensity(=reflectivity)
  out  /perception/lidar/lines_accumulated    odom, last accumulate_sec seconds
  out  /perception/lidar/line_debug/ground    base_link, ground points,
                                              intensity = reflectivity / background
                                              (publish_debug: true)

The Ouster cannot see the ground closer than ~1.7 m (lowest beam -21.4 deg
from ~0.67 m up), so near-field lines come only from accumulation or from the
camera pipelines.

detect_lines() is pure (numpy in, numpy out) so the offline bag tool and the
unit tests run exactly the same code as the node.
"""

from collections import deque
from dataclasses import dataclass, fields
import time
from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)

from avros_perception.lidar_preprocessor import (
    _cap_memory,
    apply_transform,
    deskew_buckets,
    deskew_points,
    fit_ground_plane_ransac,
    smooth_plane,
    transform_matrix_from_stamped,
)


# ---- parameters -------------------------------------------------------------


@dataclass
class LineParams:
    """Detection parameters. Every field is also a ROS parameter of the same name."""

    # Sensor-frame range gate (m): ignore the first 10 in (0.254 m) from the
    # LiDAR. (Flat ground is only hit from ~1.7 m out anyway: lowest beam
    # -21.4 deg.) Beyond ~12 m a 7.6 cm line is under 2 columns wide.
    range_min: float = 0.254
    range_max: float = 15.0
    # Horizontal field of view kept, in base_link (0 deg = vehicle front = the
    # Ouster-logo side, +90 = left). 180 = front half only; 360 = everything.
    fov_deg: float = 180.0
    fov_center_deg: float = 0.0
    # Ground (base_link, which sits at ground level).
    plane_fit_band: float = 0.2       # |z| eligible for the RANSAC fit
    plane_fit_max_dist: float = 10.0  # horizontal distance eligible for the fit
    plane_fit_stride: int = 4         # use every Nth column for the fit
    ransac_iterations: int = 60
    ransac_distance_threshold: float = 0.06
    plane_smoothing_alpha: float = 0.3
    plane_max_tilt_deg: float = 10.0  # reject fits steeper than this
    # Reject fits whose height under base_link is further than this from 0
    # (base_link is at ground level; clutter such as kerbs, low boxes or grass
    # can otherwise out-vote a partly hidden ground).
    plane_max_offset: float = 0.15
    ground_tolerance: float = 0.08    # |distance to plane| that counts as ground
    # Object feet: a ground point is dropped if one of the next obstacle_rows
    # higher beams in the same column hits something > obstacle_height_m above
    # the plane within obstacle_clearance_m (horizontally) of it. Removes the
    # bottom few cm of white barrels, posts and people's shoes.
    obstacle_rows: int = 4
    obstacle_height_m: float = 0.12
    obstacle_clearance_m: float = 0.35
    # Robot body (base_link box) to ignore.
    footprint_x_min: float = -0.23
    footprint_x_max: float = 0.78
    footprint_y_min: float = -0.46
    footprint_y_max: float = 0.46
    # Per-ring background.
    background_window: int = 61       # columns (1024 per rev -> ~21 deg)
    background_outlier_ratio: float = 1.5
    background_min_support: int = 8   # ground points needed in the window
    # Thresholds. UNTUNED first guesses: asphalt ~5-20, paint ~30-100+.
    min_reflectivity: float = 20.0
    contrast_ratio: float = 2.0
    contrast_delta: float = 12.0
    # Run gate along each ring.
    min_run_points: int = 2
    max_run_width_m: float = 2.5

    @classmethod
    def names(cls):
        return [f.name for f in fields(cls)]


# ---- pure functions ---------------------------------------------------------


def organized_arrays(
    msg: PointCloud2, names=("x", "y", "z", "reflectivity", "t")
) -> Dict[str, np.ndarray]:
    """Return {field: (H, W) float64 array} for an organized cloud (H > 1)."""
    if msg.height <= 1:
        raise ValueError(
            f"cloud is not organized (height={msg.height}); set organized: true "
            "in ouster.yaml"
        )
    available = {f.name for f in msg.fields}
    wanted = tuple(n for n in names if n in available)
    missing = {"x", "y", "z", "reflectivity"} - set(wanted)
    if missing:
        raise ValueError(f"cloud lacks fields {sorted(missing)} (point_type: native?)")
    cloud = point_cloud2.read_points(msg, field_names=wanted, skip_nans=False)
    return {
        n: cloud[n].astype(np.float64).reshape(msg.height, msg.width) for n in wanted
    }


def _circular_window_sum(values: np.ndarray, window: int) -> np.ndarray:
    """Sum over a centred window of `window` columns, wrapping around each row."""
    h = window // 2
    w = 2 * h + 1
    padded = np.concatenate([values[:, -h:], values, values[:, :h]], axis=1) if h else values
    cs = np.concatenate(
        [np.zeros((values.shape[0], 1)), np.cumsum(padded, axis=1)], axis=1
    )
    return cs[:, w:] - cs[:, :-w]


def ring_background(
    refl: np.ndarray,
    ground: np.ndarray,
    window: int,
    outlier_ratio: float,
    min_support: int,
) -> np.ndarray:
    """
    Robust local background reflectivity per point, along its own ring.

    First pass: mean reflectivity of the ground points in the window. Second
    pass: the same, ignoring ground points brighter than outlier_ratio x the
    first estimate (the paint itself). NaN where fewer than min_support ground
    points remain.
    """
    window = max(1, min(int(window), refl.shape[1] - 1 if refl.shape[1] > 1 else 1))
    g = ground.astype(np.float64)
    s1 = _circular_window_sum(np.where(ground, refl, 0.0), window)
    c1 = _circular_window_sum(g, window)
    with np.errstate(invalid="ignore", divide="ignore"):
        bg1 = s1 / c1
    keep = ground & ~(refl > outlier_ratio * np.nan_to_num(bg1, nan=np.inf))
    s2 = _circular_window_sum(np.where(keep, refl, 0.0), window)
    c2 = _circular_window_sum(keep.astype(np.float64), window)
    with np.errstate(invalid="ignore", divide="ignore"):
        bg = s2 / c2
    bg[c2 < min_support] = np.nan
    return bg


def line_candidates(
    refl: np.ndarray,
    background: np.ndarray,
    ground: np.ndarray,
    min_reflectivity: float,
    contrast_ratio: float,
    contrast_delta: float,
) -> np.ndarray:
    """Ground points that are bright both absolutely and against their background."""
    finite = np.isfinite(background)
    bg = np.where(finite, background, np.inf)
    return (
        ground
        & finite
        & (refl >= min_reflectivity)
        & (refl >= contrast_ratio * bg)
        & (refl >= bg + contrast_delta)
    )


def filter_runs(
    mask: np.ndarray,
    range_m: np.ndarray,
    min_points: int,
    max_width_m: float,
) -> np.ndarray:
    """
    Keep runs of consecutive True columns (per row) that are long enough in
    points and narrow enough in metres. Width ~ mean range x columns x the
    column angle (2 pi / W). Runs that wrap past column 0 are treated as two.
    """
    h, w = mask.shape
    padded = np.zeros((h, w + 2), dtype=np.int8)
    padded[:, 1:-1] = mask
    d = np.diff(padded, axis=1)
    rs, cs = np.nonzero(d == 1)   # run start column
    _, ce = np.nonzero(d == -1)  # run end column (exclusive)
    if rs.size == 0:
        return np.zeros_like(mask)
    length = ce - cs
    rsum = np.concatenate(
        [np.zeros((h, 1)), np.cumsum(np.where(mask, range_m, 0.0), axis=1)], axis=1
    )
    mean_r = (rsum[rs, ce] - rsum[rs, cs]) / length
    width = mean_r * length * (2.0 * np.pi / w)
    keep = (length >= min_points) & (width <= max_width_m)
    marker = np.zeros((h, w + 1), dtype=np.int32)
    np.add.at(marker, (rs[keep], cs[keep]), 1)
    np.add.at(marker, (rs[keep], ce[keep]), -1)
    return np.cumsum(marker[:, :w], axis=1) > 0


def object_feet(
    raised: np.ndarray, horiz: np.ndarray, rows: int, clearance: float
) -> np.ndarray:
    """
    Mask of points that have a raised point (above the ground band) in one of
    the `rows` beams above them (lower row index = higher beam on the Ouster)
    in the same column, within `clearance` metres of horizontal distance.
    """
    out = np.zeros_like(raised)
    for k in range(1, max(0, int(rows)) + 1):
        if k >= raised.shape[0]:
            break
        near = raised[:-k] & (np.abs(horiz[:-k] - horiz[k:]) <= clearance)
        out[k:] |= near
    return out


def plane_tilt_deg(coeffs: np.ndarray) -> float:
    """Angle between the plane normal and +Z, in degrees (sign-agnostic)."""
    return float(np.degrees(np.arccos(min(1.0, abs(coeffs[2])))))


@dataclass
class LineResult:
    lines_xyz: np.ndarray          # (N, 3) base_link
    lines_refl: np.ndarray         # (N,)
    ground_xyz: np.ndarray         # (M, 3) base_link
    ground_contrast: np.ndarray    # (M,) reflectivity / background (nan -> 0)
    ground_refl: np.ndarray        # (M,)
    plane: Optional[np.ndarray]    # [a, b, c, d] used, or None
    candidates: int                # before the run gate


def detect_lines(
    xyz_sensor: np.ndarray,
    refl: np.ndarray,
    t_base_sensor: np.ndarray,
    params: LineParams,
    previous_plane: Optional[np.ndarray] = None,
    rng_seed: int = 42,
) -> LineResult:
    """
    Run the whole detector on one organized scan.

    xyz_sensor: (H, W, 3) points in the sensor frame, already deskewed
    (no-return points may be 0 or NaN). refl: (H, W). t_base_sensor: 4x4
    base_link <- sensor. previous_plane: last smoothed plane, or None.
    """
    p = params
    hgt, wid = refl.shape
    finite = np.isfinite(xyz_sensor).all(axis=2)
    xyz_s = np.where(finite[..., None], xyz_sensor, 0.0)
    range_m = np.linalg.norm(xyz_s, axis=2)
    valid = finite & (range_m >= p.range_min) & (range_m <= p.range_max)

    flat = xyz_s.reshape(-1, 3)
    base = apply_transform(flat, t_base_sensor).reshape(hgt, wid, 3)
    bx, by, bz = base[..., 0], base[..., 1], base[..., 2]
    in_body = (
        (bx >= p.footprint_x_min) & (bx <= p.footprint_x_max)
        & (by >= p.footprint_y_min) & (by <= p.footprint_y_max)
    )
    valid &= ~in_body
    if p.fov_deg < 360.0:
        az = np.degrees(np.arctan2(by, bx)) - p.fov_center_deg
        az = (az + 180.0) % 360.0 - 180.0
        valid &= np.abs(az) <= 0.5 * p.fov_deg

    # Ground plane: RANSAC on a strided, near, low subset; blend with the last.
    horiz = np.hypot(bx, by)
    fit_mask = valid & (np.abs(bz) <= p.plane_fit_band) & (horiz <= p.plane_fit_max_dist)
    fit_mask[:, np.arange(wid) % max(1, p.plane_fit_stride) != 0] = False
    plane = previous_plane
    if fit_mask.sum() >= 50:
        cand, _ = fit_ground_plane_ransac(
            base[fit_mask],
            iterations=p.ransac_iterations,
            distance_threshold=p.ransac_distance_threshold,
            candidate_band=2.0 * p.plane_fit_band,
            seed=rng_seed,
        )
        if cand is not None:
            if cand[2] < 0:
                cand = -cand
            if (plane_tilt_deg(cand) <= p.plane_max_tilt_deg
                    and abs(cand[3] / cand[2]) <= p.plane_max_offset):
                plane = smooth_plane(previous_plane, cand, p.plane_smoothing_alpha)
    if plane is None:
        # No usable fit yet: assume base_link's z = 0 is the ground.
        plane = np.array([0.0, 0.0, 1.0, 0.0])

    dist = base @ plane[:3] + plane[3]
    ground = valid & (np.abs(dist) <= p.ground_tolerance)
    ground &= ~object_feet(
        valid & (dist > p.obstacle_height_m), horiz, p.obstacle_rows,
        p.obstacle_clearance_m,
    )

    bg = ring_background(
        refl, ground, p.background_window, p.background_outlier_ratio,
        p.background_min_support,
    )
    cand_mask = line_candidates(
        refl, bg, ground, p.min_reflectivity, p.contrast_ratio, p.contrast_delta
    )
    line_mask = filter_runs(cand_mask, range_m, p.min_run_points, p.max_run_width_m)

    with np.errstate(invalid="ignore", divide="ignore"):
        contrast = np.nan_to_num(refl / bg, nan=0.0, posinf=0.0)
    return LineResult(
        lines_xyz=base[line_mask],
        lines_refl=refl[line_mask],
        ground_xyz=base[ground],
        ground_contrast=contrast[ground],
        ground_refl=refl[ground],
        plane=plane,
        candidates=int(cand_mask.sum()),
    )


def make_xyzi_cloud(header: Header, xyz: np.ndarray, intensity: np.ndarray) -> PointCloud2:
    """PointCloud2 with float32 x, y, z, intensity."""
    fields_ = [
        PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i, n in enumerate(("x", "y", "z", "intensity"))
    ]
    pts = np.empty((xyz.shape[0], 4), dtype=np.float32)
    pts[:, :3] = xyz
    pts[:, 3] = intensity
    return point_cloud2.create_cloud(header, fields_, pts)


# ---- node -------------------------------------------------------------------


class LidarLineDetectorNode(Node):
    def __init__(self):
        super().__init__("lidar_line_detector")
        self.declare_parameter("input_topic", "/ouster/points")
        self.declare_parameter("sensor_frame", "os_sensor")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("deskew_enable", True)
        self.declare_parameter("deskew_fixed_frame", "odom")
        self.declare_parameter("deskew_buckets", 16)
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("accumulate_sec", 2.0)
        self.declare_parameter("accumulate_frame", "odom")
        self.declare_parameter("accumulate_voxel", 0.04)
        defaults = LineParams()
        for name in LineParams.names():
            self.declare_parameter(name, getattr(defaults, name))

        self.sensor_frame = self.get_parameter("sensor_frame").value
        self.target_frame = self.get_parameter("target_frame").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self.create_subscription(
            PointCloud2, self.get_parameter("input_topic").value, self._on_cloud, qos
        )
        self.lines_pub = self.create_publisher(PointCloud2, "/perception/lidar/lines", 5)
        self.accum_pub = self.create_publisher(
            PointCloud2, "/perception/lidar/lines_accumulated", 5
        )
        self.debug_pub = self.create_publisher(
            PointCloud2, "/perception/lidar/line_debug/ground", 2
        )

        self._plane: Optional[np.ndarray] = None
        self._history: deque = deque()  # (stamp_sec, xyz_odom, refl)
        self._times = deque(maxlen=50)
        self._n = 0

    def _params(self) -> LineParams:
        return LineParams(**{n: self.get_parameter(n).value for n in LineParams.names()})

    def _has_frame(self, frame: str) -> bool:
        """Non-blocking: is `frame` connected to the sensor in TF at all?"""
        if self.tf_buffer.can_transform(frame, self.sensor_frame, Time()):
            return True
        self.get_logger().warn(
            f"no '{frame}' TF (localization not running?): deskew and "
            "accumulation are off until it appears",
            throttle_duration_sec=30.0,
        )
        return False

    def _lookup(self, target, source, stamp) -> Optional[np.ndarray]:
        try:
            tf = self.tf_buffer.lookup_transform(
                target, source, Time.from_msg(stamp), timeout=Duration(seconds=0.02)
            )
        except (LookupException, ExtrapolationException, ConnectivityException) as exc:
            self.get_logger().warn(
                f"TF {target}<-{source} failed: {exc}", throttle_duration_sec=5.0
            )
            return None
        return transform_matrix_from_stamped(tf)

    def _deskew(self, flat: np.ndarray, t_rel: np.ndarray, stamp) -> np.ndarray:
        """Same scheme as lidar_preprocessor; returns input unchanged without odom TF."""
        idx, centres = deskew_buckets(t_rel, int(self.get_parameter("deskew_buckets").value))
        fixed = self.get_parameter("deskew_fixed_frame").value
        ref = Time.from_msg(stamp)
        mats = []
        for c in centres:
            src = ref + Duration(nanoseconds=int(round(c * 1e9)))
            try:
                tf = self.tf_buffer.lookup_transform_full(
                    self.sensor_frame, ref, self.sensor_frame, src, fixed
                )
            except ExtrapolationException:
                try:
                    tf = self.tf_buffer.lookup_transform_full(
                        self.sensor_frame, ref, self.sensor_frame, Time(), fixed
                    )
                except (LookupException, ExtrapolationException, ConnectivityException):
                    return flat
            except (LookupException, ConnectivityException):
                return flat
            mats.append(transform_matrix_from_stamped(tf))
        return deskew_points(flat, idx, mats)

    def _on_cloud(self, msg: PointCloud2):
        t0 = time.perf_counter()
        try:
            arr = organized_arrays(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=10.0)
            return
        h, w = arr["x"].shape
        xyz = np.stack((arr["x"], arr["y"], arr["z"]), axis=2)

        if (self.get_parameter("deskew_enable").value and "t" in arr
                and self._has_frame(self.get_parameter("deskew_fixed_frame").value)):
            flat = xyz.reshape(-1, 3)
            ok = np.isfinite(flat).all(axis=1) & (np.abs(flat).sum(axis=1) > 0)
            if ok.any():
                flat = flat.copy()
                flat[ok] = self._deskew(flat[ok], arr["t"].reshape(-1)[ok] * 1e-9,
                                        msg.header.stamp)
                xyz = flat.reshape(h, w, 3)

        t_bs = self._lookup(self.target_frame, self.sensor_frame, msg.header.stamp)
        if t_bs is None:
            return

        res = detect_lines(xyz, arr["reflectivity"], t_bs, self._params(), self._plane)
        self._plane = res.plane

        header = Header(stamp=msg.header.stamp, frame_id=self.target_frame)
        self.lines_pub.publish(make_xyzi_cloud(header, res.lines_xyz, res.lines_refl))
        if self.get_parameter("publish_debug").value:
            self.debug_pub.publish(
                make_xyzi_cloud(header, res.ground_xyz, res.ground_contrast)
            )
        self._accumulate(res, msg.header.stamp)

        self._times.append((time.perf_counter() - t0) * 1e3)
        self._n += 1
        if self._n % 100 == 0:
            self.get_logger().info(
                f"lines: {res.lines_xyz.shape[0]} pts ({res.candidates} candidates), "
                f"ground {res.ground_xyz.shape[0]}, mean {np.mean(self._times):.1f} ms/scan"
            )

    def _accumulate(self, res: LineResult, stamp):
        horizon = float(self.get_parameter("accumulate_sec").value)
        if horizon <= 0.0:
            return
        frame = self.get_parameter("accumulate_frame").value
        if not self._has_frame(frame):
            return
        t_ob = self._lookup(frame, self.target_frame, stamp)
        if t_ob is None:
            return
        now = stamp.sec + stamp.nanosec * 1e-9
        if self._history and now < self._history[-1][0]:
            self._history.clear()  # time went backwards (bag loop)
        self._history.append((now, apply_transform(res.lines_xyz, t_ob), res.lines_refl))
        while self._history and now - self._history[0][0] > horizon:
            self._history.popleft()
        xyz = np.concatenate([h[1] for h in self._history])
        refl = np.concatenate([h[2] for h in self._history])
        voxel = float(self.get_parameter("accumulate_voxel").value)
        if voxel > 0 and xyz.shape[0]:
            keys = np.floor(xyz / voxel).astype(np.int64)
            _, first = np.unique(keys, axis=0, return_index=True)
            xyz, refl = xyz[first], refl[first]
        self.accum_pub.publish(
            make_xyzi_cloud(Header(stamp=stamp, frame_id=frame), xyz, refl)
        )


def main(args=None):
    _cap_memory()
    rclpy.init(args=args)
    node = LidarLineDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
