# 05 — State Estimation (robot_localization dual EKF)

Config: `evidence/deployed_snapshot/src/avros_bringup/config/ekf.yaml`, `navsat.yaml`.

## 1. Configuration as deployed

| | Local EKF (`odom`) | Global EKF (`map`) |
|---|---|---|
| Publishes | `odom→base_link`, `/odometry/filtered` | `map→odom`, `/odometry/global` |
| Rate | 30 Hz (measured 29.9) | 30 Hz (measured 27.0) |
| `two_d_mode` | true | true |
| IMU `/imu/data` | roll, pitch, yaw (absolute) + angular rates; 5σ pose and twist gates | same |
| Wheel `/wheel_odom` | vx | vx |
| ZED VIO | Δyaw (differential); inactive while the camera is off | — |
| GPS `/odometry/gps` | — | x, y absolute; gate `13.8` |
| Process noise x, y / yaw / vx / vyaw | 1e-3 / 0.01 / 0.5 / 0.3 | 0.5 / 0.01 / 0.5 / 0.3 |

This follows the Nav2 GPS tutorial pattern (both EKFs share the continuous sensors; only the global EKF fuses GPS). The history in the config comments is sound: wheel vyaw removed because track yaw is unreliable; GPS switched to absolute once RTK was available.

## 2. Measured behavior (stationary, 150 s)

| Output | Position span | Yaw drift | Max \|vy\| | Reported σ (x / yaw) |
|---|---|---|---|---|
| `/odometry/filtered` | 0.00 cm | +0.12° (follows IMU) | 0.000 m/s | 3.7e6 m (unbounded) / 1.0° |
| `/odometry/global` | **37.6 × 33.6 cm** | +0.12° | **0.082 m/s** | 0.29 m / 1.0° |
| `map→odom` TF | 37.0 × 33.3 cm | 0.02° | — | — |

Source: `evidence/live_measurements/static_2026_09_25/SUMMARY.txt`.

## 3. Findings

**E1 — The global pose follows GNSS noise one-to-one (Accuracy, High).**
The stationary robot's map position moved 37 × 34 cm, matching the span of `/odometry/gps` (36.0 × 33.3 cm) in the same window. Three configuration choices combine:
1. GNSS variance reported as ~(2 cm)² while the real scatter was ~(10 cm)² (06, G2).
2. High position process noise (0.5), so the prior is weak and each fix pulls the state almost fully.
3. No lateral constraint (E2), so corrections can also move the state sideways.
For a waypoint tolerance of 2 m this is acceptable. For costmap consistency in the map frame (global costmap, obstacle persistence) it is not: a static obstacle marked in the map frame is smeared by the same 30–40 cm.
Fix order: G1 (lever arm), G2 (covariance by RTK state), E2 (vy), then re-tune process noise with V7 data.

**E2 — Lateral velocity is unconstrained (Estimation, Medium).**
No input measures vy, so it is driven only by process noise and cross-correlation with position updates. Measured: up to 0.082 m/s lateral velocity on the global EKF while stationary, with vy variance 1.0 (m/s)² reported. Fuse vy = 0 from wheel odometry with σ 0.05–0.1 m/s (03, O3).

**E3 — IMU measurements enter as near-perfect, so the IMU gates cannot catch a gyro bias (Estimation, High, confirmed).**
The driver's `orientation_stddev`, `angular_velocity_stddev` and `linear_acceleration_stddev` default to zero, and our `xsens.yaml` does not set them (04, I1; measured zero on the robot). robot_localization clamps any variance below 1e-9 to 1e-9 (`ekf.cpp` L140–146; `external/03` §7). Consequences:
- Both EKFs pass the IMU yaw and yaw rate through almost unchanged. The reported yaw σ (1.0°) reflects process noise, not IMU accuracy.
- The 5σ IMU gates added on 2026-05-26 for the stuck-bias incident can reject sudden steps only. A slowly drifting or constant gyro bias is not rejected: the filter owns yaw rate through the IMU itself, so the state follows the bias and the innovation stays small. The gate does not do what it was added for.
- A large legitimate heading correction from the Xsens (e.g. convergence after start-up) is rejected at first, then admitted as P grows, which produces a delayed step in both frames.
Fix: publish realistic covariances (04, I1), and detect stuck bias directly (04, I3) rather than relying on the gate.

**E4 — The GPS rejection gate is effectively disabled (Robustness, Medium, confirmed).**
robot_localization interprets `*_rejection_threshold` as a Mahalanobis distance in σ and squares it internally (`filter_base.cpp` `checkMahalanobisThreshold`; `external/03` §4). The configured `13.8` is a χ² value (2 DOF, 99.9%) placed in a σ parameter, so the gate is 13.8σ (χ² ≈ 190): it rejects almost nothing. The config comment ("6σ²") and CLAUDE.md ("4σ") both misdescribe it. The intended 99.9% gate is **3.72**.
Do not tighten it on its own. The map EKF sets no `initial_estimate_covariance`, so P(x,y) starts at 1e-9 and the first GPS fix far from the datum is accepted only once P has grown. At a 3.72 gate with Q = 0.5, a robot 20 m from the datum waits ~1 min; 100 m, ~24 min (`external/03`, implication 2). Order: set x/y initial covariance (~1e4) and realistic GPS covariance (06, G2) first, then set the gate to 3.72, then test by injecting a 5 m jump from a bag.

**E5 — The odom frame is not guaranteed continuous (Correctness, Low–Medium).**
(Absolute IMU yaw in both EKFs is the canonical robot_localization/Nav2 pattern; `external/03` §1.)
The local EKF fuses absolute IMU yaw. If the Xsens internal filter corrects its heading (for example when GNSS heading becomes observable after start-up, I2), `odom→base_link` yaw jumps. REP-105 requires the odom frame to be continuous. The Nav2 tutorial also fuses absolute IMU yaw in both filters, so this is common practice. The risk here is higher because the IMU heading is known to shift after the first motion. Option: fuse only angular velocity (plus ZED Δyaw) in the local EKF and keep absolute yaw in the global EKF only.

**E6 — Local EKF position covariance grows without bound (Low).**
Expected for a velocity-only filter (3.7e6 m after 15 min). Harmless for MPPI, which reads only pose and twist. Any consumer that uses `/odometry/filtered` covariance should use the global output instead.

**E7 — Global EKF misses its rate (Low).**
27.0 Hz against 30 Hz configured, under the full navigation load. MPPI runs at 20 Hz, so this does not starve the controller, but it indicates CPU pressure. Re-check with the ZED and perception enabled.

**E8 — The ZED differential yaw is over-weighted by design (Estimation, Low while the camera is off).**
robot_localization computes the variance of a differentially fused pose as `(σ²_t + σ²_{t−1})·Δt` instead of dividing by Δt² (open issue #356, `external/03` §2). A differential source therefore carries far more weight than its noise justifies. This is moot today (camera off, IMU treated as perfect), but must be checked with NIS once E3 is fixed and the camera is enabled.

**E9 — Comments in `navsat.yaml` misdescribe the node (Documentation, Low).**
With `wait_for_datum: true`, `navsat_transform` never subscribes to the IMU; the transform heading comes from the datum heading (`navsat_transform.cpp` L186). The comment "use IMU yaw directly" is therefore inaccurate. Datum heading 0 (map +x = east) is correct because the EKFs fuse true-north ENU yaw from the Xsens. The comment that `broadcast_cartesian_transform` would create a TF loop is also wrong: it publishes `map→utm`, not `map→odom`. Disabling it is harmless.

## 4. Items verified as correct
- Frame layout (`map→odom→base_link`, `broadcast_cartesian_transform: false`) avoids the TF loop.
- Wheel vyaw is excluded; heading comes from the IMU gyro, which is the right choice given measured track slip.
- `imu0_remove_gravitational_acceleration: true` with acceleration not fused: no effect, harmless.
- `map→odom` yaw stable to 0.02° over 150 s.

## 5. Assessment
The EKF structure is standard and appropriate. Its accuracy is limited by its inputs' covariances rather than by its structure: the IMU is treated as perfect, the GNSS is treated as cm-accurate even when it is not, and lateral motion is unconstrained. These are configuration fixes with measurable pass criteria (08: V0, V7, V8).
