# 04 — IMU (Xsens MTi-680G)

Config: `evidence/deployed_snapshot/src/avros_bringup/config/xsens.yaml`. Filter profile: General_RTK (flashed to the device; `enable_deviceConfig: false`, so ROS does not change it).

## 1. Role in the stack
The Xsens is the only heading source in both EKFs and the only input to the actuator heading-hold. Wheel vyaw was removed from both EKFs on 2026-05-30, and the ZED yaw input is inactive while the camera is off. Every yaw-dependent output (odom frame, map frame, heading-hold, navsat rotation into the map frame) therefore depends on this one device.

## 2. Measured performance (stationary, 150 s, 2026-09-25)

| Quantity | Measured | Comment |
|---|---|---|
| Output rate | 100.0 Hz, stamp spacing 10 ms (max 20 ms) | nominal |
| Transport latency | median 25.8 ms, p95 40 ms | stamps are MTi UTC time |
| Gyro z bias | −0.0008 °/s | healthy; stuck-bias incident value was −2.86 °/s |
| Gyro noise (100 Hz, 1σ) | z 0.107, x 0.134, y 0.140 °/s | |
| Yaw drift | +0.124° over 150 s (+0.05 °/min) | healthy |
| Yaw output noise (1σ) | 0.28° | peak-to-peak 1.1° seen in `odom→base_link` |
| Gravity vector | (−0.51, 0.05, 9.78) m/s² | ~3.0° pitch: ground slope or mount tilt (not separated) |
| Reported covariances | **all zero** (orientation, angular velocity, acceleration) | see I1 |

Source: `evidence/live_measurements/static_2026_09_25/SUMMARY.txt`.

## 3. Findings

**I1 — The driver publishes zero covariance for every field (Estimation, High).**
The EKFs receive orientation and angular velocity with zero variance, i.e. "perfect" measurements. Consequences:
- The EKF yaw follows the IMU exactly. Its uncertainty comes only from process noise (reported yaw variance 3.2e-4 rad², σ ≈ 1.0°).
- robot_localization replaces each zero variance with 1e-9 (`external/03` §7). The 5σ gates added for the stuck-bias failure therefore reject only sudden steps, never a constant bias (05, E3; I3 below).
- The measured values give defensible numbers to put in instead: yaw σ ≈ 0.3° (7.5e-5 rad², stationary; larger in motion), gyro σ ≈ 0.11 °/s (3.5e-6 (rad/s)²).
Fix: set the covariances in the driver fork, or in a small relay, from measured noise, with margin for dynamic conditions.

**I2 — Heading after power-up is not reliable until the robot has moved (Operations, Medium).**
With General_RTK and a single antenna, heading is observable from GNSS only while moving. The team has already recorded phantom yaw drift on the first motion leg after launch (`feedback_xsens_quat_needs_motion_warmup` memory). There is no automated check that heading has converged before a goal is accepted.
Fix: a pre-mission check (drive straight 5–10 m, compare the GNSS course with the IMU yaw) gated on RTK state. See 08, V6.

**I3 — The stuck-bias failure is not detected at runtime (Robustness, Medium).**
The recorded incident (gyro bias 70× normal, cleared only by USB power-cycle) has no runtime detection. The 5σ EKF gates cannot provide it. The IMU is the only yaw-rate source in both EKFs, so the filter state follows a constant bias and the innovation stays small, whether or not the covariance is realistic (05, E3). Detection needs an independent reference, for example:
- the mean gyro rate over 10 s while the wheels report zero motion, against a threshold;
- IMU yaw rate against the track-differential yaw rate while driving straight;
- IMU heading against GNSS course under RTK FIXED.

**I4 — Yaw noise is fed straight into heading-hold (Behavior, Low).**
0.28° yaw noise × kp 1.5 produces ~0.007 rad/s of ω noise, which is small. Larger IMU yaw steps, for example from the internal filter's GNSS heading corrections, pass through with no filtering and no rate limit (see 02, D3).

**I5 — GNSS lever arm is configured on the device (Correct).**
`GNSS_LeverArm: [0.74, 0, 0]` is set, so the MTi's internal position and velocity solution refers to the IMU. The raw `/gnss` topic the stack consumes does not apply it (see 06, G1).

## 4. Assessment
The device performs well when healthy: bias < 0.001 °/s and drift 0.05 °/min stationary. The integration defects are that the stack treats it as noiseless (zero covariance), depends on it alone for heading, and has no runtime health check for the documented failure mode.
