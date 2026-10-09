# 03 — Wheel Odometry (`/wheel_odom`)

Source: `actuator_node._publish_odom`, called from the 20 Hz state timer.

## 1. Current implementation

| Aspect | Implementation |
|---|---|
| Input | Latest SPARK MAX reported velocity (RPM) per track, from the Teensy `E` line |
| Rate | 20 Hz (the docstring and `ekf.yaml` comment say 50 Hz; measured 20.0 Hz) |
| Kinematics | v = (L + R)/2; ω = (R − L) / 0.7366 m (physical track, no multiplier; intentional) |
| Integration | Midpoint yaw, dt from host clock; a tick with dt > 0.5 s is dropped |
| Timestamp | Host time when the timer fires |
| Covariance | vx 1e-4 (σ 1 cm/s), vyaw 1e-4, vy 1e6 (not measured) |
| What the EKFs fuse | vx only, in both EKFs. vyaw was removed from both on 2026-05-30. vy is not fused. |

## 2. Measured properties

| Property | Value | Evidence |
|---|---|---|
| Reported velocity lag behind real wheel motion | **185 ms**, both tracks, two independent runs, correlation ≥ 0.996 | `evidence/live_measurements/velocity_lag_2026_05_logs/RESULT.txt` |
| Command → real wheel motion delay | 15–45 ms | same |
| Steady-state accuracy of reported velocity | 98.7–99.9% of position-derived velocity | same |
| Stationary output | exactly zero, no drift over 150 s | `static_2026_09_25/SUMMARY.txt` |
| L/R agreement, wheels up, 73 s | 0.43% | `docs/motor_sync_logs/REPORT.md` |
| Closed-loop 1×1 m square with EKF | 6 cm / 3.6° closure (1.5% of distance) | `docs/skid_steer_kinematics_findings_2026_05_18.md` |

## 3. Findings

**O1 — Odometry integrates lagged velocity instead of encoder position (Accuracy, High).**
The `E` line carries both velocity and cumulative position. The node stores position (`_l_meas_pos`, `_r_meas_pos`) but never uses it; `_l_pos_prev/_r_pos_prev` are declared and unused. Integrating reported velocity has three costs:
1. The twist sent to the EKF is 185 ms old but stamped as current. Velocity error = acceleration × 0.185 s: 5.6 cm/s while accelerating at 0.3 m/s², 24 cm/s while braking at 1.3 m/s².
2. Position lags real motion by v × 0.185 s during motion (13 cm at 0.7 m/s). The accumulated distance recovers when the robot stops, because the delay is linear.
3. Any interval longer than 0.5 s between timer ticks (executor stall) is discarded, and that motion is lost permanently. Position deltas are immune to this.

Standard practice (ros2_controllers `diff_drive_controller` with position feedback, WPILib `DifferentialDriveOdometry`) is to integrate encoder position deltas and derive velocity from them. That removes the measurement delay from both the pose and the twist.
Fix: compute ΔL, ΔR from `pos_rev` differences, derive v and ω from ΔL, ΔR over the actual interval, and publish at the E-line rate (50 Hz).

**O2 — The timestamp is the publish time, not the measurement time (Accuracy, Medium).**
The Teensy does not timestamp E-lines, and the node stamps the odometry at timer time. robot_localization orders measurements by stamp, so the wheel data is fused as if it were newer than it is. With O1 fixed, the remaining delay is CAN status period + serial + E-line sampling (~20–40 ms). Adding a Teensy `micros()` field to the E-line, or stamping on E-line receipt, would bound this.

**O3 — Lateral velocity is not constrained (Estimation, Medium).**
`vy` is published with covariance 1e6 and is not fused. The robot is non-holonomic; fusing vy = 0 with a realistic variance (skid-steer side-slip on grass is not zero, so σ ≈ 0.05–0.1 m/s) gives the EKF a lateral constraint. The static capture shows why this matters: with the robot stationary, the global EKF reported lateral velocity up to **0.08 m/s**, driven by GPS corrections (see 05). robot_localization's documentation recommends fusing vy for non-holonomic platforms.

**O4 — The vx variance does not reflect slip (Estimation, Low).**
σ = 1 cm/s is appropriate for encoder resolution but not for track slip. Measured motor delivery is 93–96% in translation and skid on grass is uncalibrated, so a slip-dominated error of 2–5% of speed is more realistic (σ ≈ 1.5–3.5 cm/s at 0.7 m/s). A speed-proportional variance is a common approach.

**O5 — The ground-travel constant is nominal, not measured (Calibration, Medium).**
0.01994 m/rev comes from the nominal pulley pitch diameter (80.85 mm) and gear ratio. Track sinkage and belt tension change the effective radius, especially on grass. No straight-line distance test against an independent reference is on record. RTK-FIXED GNSS now provides one (see 08, test V4).

## 4. Assessment
The wheel odometry is precise (repeatable, zero drift at rest, L/R matched) and its distance scale has not been independently verified. Its timing is the main defect: the 185 ms reported-velocity lag passes straight into the EKF. The accuracy the square test demonstrates (1.5% of distance) comes from the IMU carrying yaw, which is also why removing wheel vyaw from the EKFs was correct.
