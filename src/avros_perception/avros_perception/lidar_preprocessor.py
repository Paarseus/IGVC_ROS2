"""
LiDAR point-cloud preprocessing: deskew, filter, downsample, transform, remove ground.

Pure functions are module-level and take/return numpy arrays or plain
values so they're testable without a live ROS node or TF listener.
LidarPreprocessorNode wires them to subscriptions/publishers/TF.
"""

import os
import resource
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header
from tf2_ros import (
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    Buffer,
    TransformListener,
)


# ---- pure functions ---------------------------------------------------------


def pointcloud2_to_xyz_array(msg: PointCloud2, skip_nans: bool = True) -> np.ndarray:
    """Extract an (N, 3) xyz array from a PointCloud2, dropping NaN rows."""
    cloud = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)
    if cloud.shape[0] == 0:
        return np.empty((0, 3), dtype=np.float64)
    arr = np.stack((cloud["x"], cloud["y"], cloud["z"]), axis=1).astype(np.float64)
    if skip_nans:
        arr = arr[np.isfinite(arr).all(axis=1)]
    return arr


def pointcloud2_to_xyz_t_arrays(
    msg: PointCloud2, time_field: str = "t", time_scale: float = 1e-9
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """
    Extract (N, 3) xyz and (N,) per-point time offsets in seconds.

    The offsets are relative to the cloud's header stamp. For ouster_ros
    `native` clouds `t` is uint32 nanoseconds since the start of the scan,
    hence time_scale=1e-9. Returns (xyz, None) if the cloud has no
    time_field (e.g. the Webots sim cloud), so callers can skip deskew.
    NaN rows are dropped from both arrays.
    """
    if time_field not in [f.name for f in msg.fields]:
        return pointcloud2_to_xyz_array(msg), None
    cloud = point_cloud2.read_points(
        msg, field_names=("x", "y", "z", time_field), skip_nans=False
    )
    if cloud.shape[0] == 0:
        return np.empty((0, 3), dtype=np.float64), np.empty(0, dtype=np.float64)
    xyz = np.stack((cloud["x"], cloud["y"], cloud["z"]), axis=1).astype(np.float64)
    t_rel = cloud[time_field].astype(np.float64) * time_scale
    finite = np.isfinite(xyz).all(axis=1)
    return xyz[finite], t_rel[finite]


def range_mask(
    points_xyz: np.ndarray, min_range: float, max_range: float
) -> np.ndarray:
    """Boolean mask of points whose distance from the sensor origin is in range."""
    dist = np.linalg.norm(points_xyz, axis=1)
    return (dist >= min_range) & (dist <= max_range)


def range_filter(
    points_xyz: np.ndarray, min_range: float, max_range: float
) -> np.ndarray:
    """
    Keep points whose distance from the sensor origin (0,0,0) is in range.

    Must be called on points still in the sensor's own frame (e.g. `os_sensor`)
    -- filtering after transforming into base_link would measure range from
    the wrong origin, since the sensor is offset from base_link.
    """
    if points_xyz.shape[0] == 0:
        return points_xyz
    return points_xyz[range_mask(points_xyz, min_range, max_range)]


def deskew_buckets(
    t_rel: np.ndarray, n_buckets: int
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Split the scan's time span into n_buckets equal slices.

    Returns (bucket index per point, bucket centre times in seconds). One TF
    lookup per bucket instead of per point: with 16 buckets over a 0.1 s scan
    a point is at most ~3 ms from its bucket centre, i.e. <0.3 deg of yaw at
    1.5 rad/s (~2 cm at 5 m).
    """
    n = t_rel.shape[0]
    if n == 0:
        return np.zeros(0, dtype=np.int64), np.zeros(0, dtype=np.float64)
    t0 = float(t_rel.min())
    span = float(t_rel.max()) - t0
    if span <= 0.0 or n_buckets <= 1:
        return np.zeros(n, dtype=np.int64), np.array([t0 + 0.5 * span])
    width = span / n_buckets
    idx = np.minimum(((t_rel - t0) / width).astype(np.int64), n_buckets - 1)
    centres = t0 + (np.arange(n_buckets) + 0.5) * width
    return idx, centres


def deskew_points(
    points_xyz: np.ndarray, bucket_idx: np.ndarray, matrices: List[np.ndarray]
) -> np.ndarray:
    """
    Move every point into the sensor pose at the reference (header) time.

    matrices[k] is the 4x4 transform from the sensor frame at bucket k's time
    to the sensor frame at the reference time; it is applied to the points
    whose bucket_idx is k.
    """
    out = np.empty_like(points_xyz)
    for k, m in enumerate(matrices):
        sel = bucket_idx == k
        if sel.any():
            out[sel] = points_xyz[sel] @ m[:3, :3].T + m[:3, 3]
    return out


def vertical_fov_filter(
    points_xyz: np.ndarray, min_deg: float, max_deg: float
) -> np.ndarray:
    """
    Keep points whose elevation angle from the sensor origin is in range.

    Must be called on points still in the sensor's own frame (e.g.
    `velodyne`) for the same reason as range_filter -- elevation is the
    angle off the sensor's own boresight, not base_link's. The VLP-16's
    16 lasers already span exactly -15 to +15 degrees, so with the
    defaults this is a no-op safety net rather than an actual crop; it
    only starts cropping real data if the sensor is ever swapped for one
    with a wider vertical spread.
    """
    if points_xyz.shape[0] == 0:
        return points_xyz
    horizontal = np.linalg.norm(points_xyz[:, :2], axis=1)
    elevation_deg = np.degrees(np.arctan2(points_xyz[:, 2], horizontal))
    mask = (elevation_deg >= min_deg) & (elevation_deg <= max_deg)
    return points_xyz[mask]


def downsample_voxel(points_xyz: np.ndarray, voxel_size: float) -> np.ndarray:
    """Reduce to one representative point (the first seen) per voxel_size cube."""
    if points_xyz.shape[0] == 0:
        return points_xyz
    keys = np.floor(points_xyz / voxel_size).astype(np.int64)
    keys -= keys.min(axis=0)
    span = keys.max(axis=0) + 1
    packed = (keys[:, 0] * span[1] + keys[:, 1]) * span[2] + keys[:, 2]
    _, first_indices = np.unique(packed, return_index=True)
    return points_xyz[np.sort(first_indices)]


def fit_ground_plane_ransac(
    points_xyz: np.ndarray,
    iterations: int = 50,
    distance_threshold: float = 0.1,
    candidate_band: float = 0.5,
    seed: Optional[int] = None,
) -> Tuple[Optional[np.ndarray], np.ndarray]:
    """
    Hand-rolled RANSAC plane fit for ground removal.

    candidate_band restricts which points are eligible to seed/vote for the
    plane (a coarse z-band near the lowest points), separate from the
    per-point inlier distance_threshold used to score a candidate plane.

    Returns (plane_coeffs, inlier_mask) where plane_coeffs is [a, b, c, d]
    for ax+by+cz+d=0, normalized so (a,b,c) is unit length. Returns
    (None, all-False mask) if fitting isn't possible (too few points).
    """
    n = points_xyz.shape[0]
    inlier_mask = np.zeros(n, dtype=bool)
    if n < 3:
        return None, inlier_mask

    rng = np.random.default_rng(seed)

    z_min = np.min(points_xyz[:, 2])
    candidate_mask = points_xyz[:, 2] <= (z_min + candidate_band)
    candidate_indices = np.where(candidate_mask)[0]
    if candidate_indices.shape[0] < 3:
        candidate_indices = np.arange(n)

    # Draw all hypotheses at once; duplicate picks give a zero normal and are dropped.
    sample = candidate_indices[
        rng.integers(0, candidate_indices.shape[0], size=(iterations, 3))
    ]
    p1, p2, p3 = (points_xyz[sample[:, k]] for k in range(3))
    normals = np.cross(p2 - p1, p3 - p1)
    norms = np.linalg.norm(normals, axis=1)
    valid = norms >= 1e-9
    if not valid.any():
        return None, inlier_mask
    normals = normals[valid] / norms[valid, None]
    d = -np.einsum("ij,ij->i", normals, p1[valid])

    dist = np.abs(points_xyz @ normals.T + d)  # (n, valid hypotheses)
    counts = (dist <= distance_threshold).sum(axis=0)
    best = int(np.argmax(counts))  # first maximum, like the original strict '>'
    best_coeffs = np.array([*normals[best], d[best]])

    dist = np.abs(points_xyz @ best_coeffs[:3] + best_coeffs[3])
    inlier_mask = dist <= distance_threshold
    return best_coeffs, inlier_mask


def remove_ground(points_xyz: np.ndarray, inlier_mask: np.ndarray) -> np.ndarray:
    """Return non-ground (obstacle) points, i.e. those NOT in inlier_mask."""
    if points_xyz.shape[0] == 0:
        return points_xyz
    return points_xyz[~inlier_mask]


def align_plane_sign(coeffs: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """
    Flip coeffs' sign if needed so its normal points the same way as reference's.

    RANSAC's cross-product normal direction depends on sample order and is
    arbitrary per fit -- two fits of the same physical plane can come back
    as [a,b,c,d] or [-a,-b,-c,-d]. Averaging those directly would cancel
    instead of reinforcing, so callers doing temporal blending must align
    signs first.
    """
    if np.dot(coeffs[:3], reference[:3]) < 0:
        return -coeffs
    return coeffs


def smooth_plane(
    previous: Optional[np.ndarray], candidate: np.ndarray, alpha: float
) -> np.ndarray:
    """
    Exponentially blend a new RANSAC plane fit onto a running estimate.

    A ground vehicle's ground plane changes gradually (ramps, curbs, terrain
    undulation) even though an unseeded per-frame RANSAC fit on essentially
    the same points can jitter between fits from run to run. Blending
    trades a bit of lag for a stable plane, which is what should be used to
    classify ground vs. obstacle points, rather than each frame's raw fit.

    alpha is the weight given to the new candidate fit (0 < alpha <= 1);
    previous=None (first fit) returns candidate unchanged.
    """
    if previous is None:
        return candidate
    aligned = align_plane_sign(candidate, previous)
    blended = alpha * aligned + (1.0 - alpha) * previous
    norm = np.linalg.norm(blended[:3])
    if norm < 1e-9:
        return previous
    return blended / norm


def classify_ground_inliers(
    points_xyz: np.ndarray, plane_coeffs: np.ndarray, distance_threshold: float
) -> np.ndarray:
    """Boolean mask of points within distance_threshold of plane_coeffs."""
    if points_xyz.shape[0] == 0:
        return np.zeros(0, dtype=bool)
    dist = np.abs(points_xyz @ plane_coeffs[:3] + plane_coeffs[3])
    return dist <= distance_threshold


def apply_height_band(
    points_xyz: np.ndarray, z_min: float, z_max: float
) -> np.ndarray:
    if points_xyz.shape[0] == 0:
        return points_xyz
    mask = (points_xyz[:, 2] >= z_min) & (points_xyz[:, 2] <= z_max)
    return points_xyz[mask]


def remove_self_footprint(
    points_xyz: np.ndarray,
    x_range: Tuple[float, float],
    y_range: Tuple[float, float],
    z_range: Tuple[float, float],
) -> np.ndarray:
    """
    Drop points (already in base_link) inside the vehicle's own bounding box.

    Without this, the vehicle's own chassis -- the closest, most persistent,
    most geometrically detailed thing any beam can see -- is eligible to be
    detected as an obstacle sitting next to itself, and is a likely
    contributor to voxel flicker right around the sensor (see
    ground_plane_ransac_seed / the grid log-odds: this is a different,
    upstream cause those don't address). The box defaults come from
    IGVC_ROS2's measured chassis dimensions, not this repo's own placeholder
    URDF -- see node docstring for the derivation.
    """
    if points_xyz.shape[0] == 0:
        return points_xyz
    inside = (
        (points_xyz[:, 0] >= x_range[0]) & (points_xyz[:, 0] <= x_range[1])
        & (points_xyz[:, 1] >= y_range[0]) & (points_xyz[:, 1] <= y_range[1])
        & (points_xyz[:, 2] >= z_range[0]) & (points_xyz[:, 2] <= z_range[1])
    )
    return points_xyz[~inside]


def transform_matrix_from_stamped(tf: TransformStamped) -> np.ndarray:
    """
    Build a 4x4 homogeneous transform matrix from a TransformStamped.

    Hand-rolled quaternion->rotation-matrix instead of adding
    tf_transformations/transforms3d as a dependency.
    """
    t = tf.transform.translation
    q = tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w

    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if norm > 0:
        x, y, z, w = x / norm, y / norm, z / norm, w / norm

    rot = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )

    matrix = np.eye(4)
    matrix[:3, :3] = rot
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


def apply_transform(points_xyz: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    if points_xyz.shape[0] == 0:
        return points_xyz
    homogeneous = np.hstack([points_xyz, np.ones((points_xyz.shape[0], 1))])
    transformed = homogeneous @ matrix.T
    return transformed[:, :3]


def is_sensor_healthy(
    last_msg_time: Optional[float], now: float, timeout_sec: float
) -> bool:
    if last_msg_time is None:
        return False
    return (now - last_msg_time) <= timeout_sec


# ---- ROS node -------------------------------------------------------------


class LidarPreprocessorNode(Node):
    def __init__(self):
        super().__init__("lidar_preprocessor")

        # 2026-11-04: Ouster OS1 replaced the VLP-16 (was /velodyne_points, velodyne).
        # self.declare_parameter("input_topic", "/velodyne_points")
        # self.declare_parameter("sensor_frame", "velodyne")
        self.declare_parameter("input_topic", "/ouster/points")
        self.declare_parameter("sensor_frame", "os_sensor")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("range_min", 0.5)
        self.declare_parameter("range_max", 50.0)
        self.declare_parameter("vertical_fov_min_deg", -15.0)
        self.declare_parameter("vertical_fov_max_deg", 15.0)
        self.declare_parameter("downsample_voxel_size", 0.2)
        self.declare_parameter("ransac_iterations", 50)
        self.declare_parameter("ransac_candidate_band", 0.5)
        self.declare_parameter("ransac_distance_threshold", 0.1)
        self.declare_parameter("ground_plane_ransac_seed", 42)
        self.declare_parameter("ground_plane_smoothing_alpha", 0.25)
        self.declare_parameter("height_band_min", -0.8)
        self.declare_parameter("height_band_max", 3.0)
        self.declare_parameter("sensor_timeout_sec", 0.5)
        # The Velodyne driver can emit many small partial-revolution clouds. Fragments
        # are filtered on arrival and merged over this window (header-stamp time, one
        # revolution at 600 rpm = 0.1 s) before the expensive stages run once.
        # 0 disables merging: every incoming cloud is processed on its own.
        self.declare_parameter("merge_window_sec", 0.08)
        # Motion deskew (2026-11-06). The VLP-16 driver deskewed per packet against
        # odom; ouster_ros does not, so a 0.1 s scan taken while turning at 1.5 rad/s
        # is smeared by ~8.6 deg. Each point is moved into the sensor pose at the
        # header stamp using its per-point time field and TF (sensor_frame through
        # deskew_fixed_frame). Skipped automatically for clouds without the field.
        self.declare_parameter("deskew_enable", True)
        self.declare_parameter("deskew_fixed_frame", "odom")
        self.declare_parameter("deskew_time_field", "t")
        self.declare_parameter("deskew_buckets", 16)

        # Self-footprint exclusion box, in base_link. Defaults are the union
        # bounding box of IGVC_ROS2's measured chassis geometry (main body +
        # top sensor platform + both tread enclosures, from avros.urdf.xacro,
        # resurveyed 2026-05-06) plus a 0.05m margin, NOT this repo's own
        # placeholder vehicle.urdf.xacro (which has no collision geometry).
        # Re-derive these if the real chassis is measured directly, or if
        # this vehicle's dimensions differ from IGVC_ROS2's.
        self.declare_parameter("footprint_x_min", -0.23)
        self.declare_parameter("footprint_x_max", 0.78)
        self.declare_parameter("footprint_y_min", -0.46)
        self.declare_parameter("footprint_y_max", 0.46)
        self.declare_parameter("footprint_z_min", -0.05)
        self.declare_parameter("footprint_z_max", 0.72)

        self.sensor_frame = self.get_parameter("sensor_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.sensor_timeout_sec = self.get_parameter("sensor_timeout_sec").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            self.get_parameter("input_topic").value,
            self._cloud_callback,
            sensor_qos,
        )

        self.obstacles_pub = self.create_publisher(
            PointCloud2, "/perception/lidar/obstacles", 10
        )
        self.health_pub = self.create_publisher(Bool, "/perception/lidar/health", 10)

        self._last_msg_time: Optional[float] = None
        self._smoothed_ground_plane: Optional[np.ndarray] = None
        self._fragments = []  # sensor-frame (range/FOV filtered) point arrays
        self._window_start: Optional[float] = None
        self._last_stamp = None
        self._timeout_timer = self.create_timer(0.2, self._check_timeout)

    def _check_timeout(self):
        now = self.get_clock().now().nanoseconds / 1e9
        healthy = is_sensor_healthy(self._last_msg_time, now, self.sensor_timeout_sec)
        if not healthy:
            self.health_pub.publish(Bool(data=False))

    def _cloud_callback(self, msg: PointCloud2):
        """Filter an incoming (possibly partial) cloud and merge it into the window."""
        self._last_msg_time = self.get_clock().now().nanoseconds / 1e9
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        window = self.get_parameter("merge_window_sec").value

        # Stamps going backwards (bag loop / sim reset): drop the stale window.
        if self._window_start is not None and t < self._window_start:
            self._fragments = []
            self._window_start = None

        if self.get_parameter("deskew_enable").value:
            points, t_rel = pointcloud2_to_xyz_t_arrays(
                msg, self.get_parameter("deskew_time_field").value
            )
        else:
            points, t_rel = pointcloud2_to_xyz_array(msg), None
        # Range-filter BEFORE deskew: no-return points sit at (0,0,0) and must
        # not be moved off the origin by the deskew transform.
        keep = range_mask(
            points,
            self.get_parameter("range_min").value,
            self.get_parameter("range_max").value,
        )
        points = points[keep]
        if t_rel is not None and points.shape[0]:
            points = self._deskew(points, t_rel[keep], stamp)
        points = vertical_fov_filter(
            points,
            self.get_parameter("vertical_fov_min_deg").value,
            self.get_parameter("vertical_fov_max_deg").value,
        )
        if points.shape[0]:
            self._fragments.append(points)
        if self._window_start is None:
            self._window_start = t
        self._last_stamp = stamp

        if t - self._window_start >= window:
            merged = (
                np.concatenate(self._fragments)
                if self._fragments
                else np.empty((0, 3), dtype=np.float64)
            )
            self._fragments = []
            self._window_start = None
            self._process(merged, stamp)

    def _deskew(self, points: np.ndarray, t_rel: np.ndarray, stamp) -> np.ndarray:
        """Move each point into the sensor pose at `stamp`; unchanged if TF is missing."""
        idx, centres = deskew_buckets(
            t_rel, int(self.get_parameter("deskew_buckets").value)
        )
        fixed = self.get_parameter("deskew_fixed_frame").value
        ref_time = Time.from_msg(stamp)
        matrices = []
        for centre in centres:
            src_time = ref_time + Duration(nanoseconds=int(round(centre * 1e9)))
            try:
                tf = self.tf_buffer.lookup_transform_full(
                    self.sensor_frame, ref_time, self.sensor_frame, src_time, fixed
                )
            except ExtrapolationException:
                # The end of the scan can be newer than the latest odom TF (EKF
                # lag). Use the latest pose for those buckets: error ~ the lag.
                try:
                    tf = self.tf_buffer.lookup_transform_full(
                        self.sensor_frame, ref_time, self.sensor_frame, Time(), fixed
                    )
                except (LookupException, ExtrapolationException,
                        ConnectivityException) as exc:
                    self._warn_deskew(exc)
                    return points
            except (LookupException, ConnectivityException) as exc:
                self._warn_deskew(exc)
                return points
            matrices.append(transform_matrix_from_stamped(tf))
        return deskew_points(points, idx, matrices)

    def _warn_deskew(self, exc):
        self.get_logger().warn(
            f"deskew skipped (no {self.get_parameter('deskew_fixed_frame').value}"
            f"<-{self.sensor_frame} TF over the scan): {exc}",
            throttle_duration_sec=5.0,
        )

    def _process(self, points: np.ndarray, stamp):
        """Downsample, transform, remove body and ground, and publish obstacles."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, self.sensor_frame, stamp
            )
        except (LookupException, ExtrapolationException, ConnectivityException) as exc:
            self.get_logger().warn(
                f"TF lookup {self.target_frame}<-{self.sensor_frame} failed: {exc}",
                throttle_duration_sec=2.0,
            )
            self.health_pub.publish(Bool(data=False))
            return

        points = downsample_voxel(
            points, self.get_parameter("downsample_voxel_size").value
        )

        matrix = transform_matrix_from_stamped(tf)
        points_base = apply_transform(points, matrix)
        points_base = remove_self_footprint(
            points_base,
            x_range=(
                self.get_parameter("footprint_x_min").value,
                self.get_parameter("footprint_x_max").value,
            ),
            y_range=(
                self.get_parameter("footprint_y_min").value,
                self.get_parameter("footprint_y_max").value,
            ),
            z_range=(
                self.get_parameter("footprint_z_min").value,
                self.get_parameter("footprint_z_max").value,
            ),
        )

        distance_threshold = self.get_parameter("ransac_distance_threshold").value
        candidate_coeffs, _ = fit_ground_plane_ransac(
            points_base,
            iterations=self.get_parameter("ransac_iterations").value,
            distance_threshold=distance_threshold,
            candidate_band=self.get_parameter("ransac_candidate_band").value,
            seed=self.get_parameter("ground_plane_ransac_seed").value,
        )
        if candidate_coeffs is not None:
            self._smoothed_ground_plane = smooth_plane(
                self._smoothed_ground_plane,
                candidate_coeffs,
                self.get_parameter("ground_plane_smoothing_alpha").value,
            )

        if self._smoothed_ground_plane is not None:
            inlier_mask = classify_ground_inliers(
                points_base, self._smoothed_ground_plane, distance_threshold
            )
        else:
            inlier_mask = np.zeros(points_base.shape[0], dtype=bool)

        obstacles = remove_ground(points_base, inlier_mask)
        obstacles = apply_height_band(
            obstacles,
            self.get_parameter("height_band_min").value,
            self.get_parameter("height_band_max").value,
        )

        header = Header()
        header.stamp = stamp
        header.frame_id = self.target_frame

        cloud_msg = point_cloud2.create_cloud_xyz32(header, obstacles)
        self.obstacles_pub.publish(cloud_msg)

        self.health_pub.publish(Bool(data=True))


def _cap_memory():
    """Cap this process's address space so a runaway allocation fails alone.

    The node normally uses under 1 GB of address space. On 2026-10-09 it once grew
    to ~56 GB within a couple of minutes and was OOM-killed by the kernel (cause not
    found, not reproducible), which puts the rest of the robot's processes at risk.
    With the cap it raises MemoryError (and a traceback) instead. Override with
    AVROS_LIDAR_MAX_VMEM_GB; 0 disables the cap.
    """
    gb = float(os.environ.get('AVROS_LIDAR_MAX_VMEM_GB', '4'))
    if gb > 0:
        limit = int(gb * 2 ** 30)
        resource.setrlimit(resource.RLIMIT_AS, (limit, limit))


def main(args=None):
    _cap_memory()
    rclpy.init(args=args)
    node = LidarPreprocessorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
