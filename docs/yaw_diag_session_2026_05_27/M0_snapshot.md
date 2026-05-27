# M0 Snapshot — 2026-05-27 ~13:46 UTC

5-second sample taken while robot stationary, ~3 min after launch.

## Metrics

| Metric | Observed | Threshold (healthy) | Verdict | Reference |
|---|---|---|---|---|
| `/imu/data.wz` mean (gyro bias) | **+0.0143 °/s** | < 0.05 °/s | ✅ PASS | #12 M4 baseline was 0.0042°/s |
| `/imu/data.wz` σ (gyro noise) | 0.097 °/s (0.001694 rad/s) | < 0.15 °/s | ✅ PASS | normal Xsens noise floor |
| `/wheel_odom.wz` mean | 0.000000 exact | == 0 | ✅ PASS | encoder correctly silent at rest |
| `/wheel_odom.wz` σ | 0.000000 exact | == 0 | ✅ PASS | |
| `/odometry/filtered.wz` mean | -0.000012 rad/s | ~ 0 | ✅ PASS | EKF correctly settling near zero |
| `/odometry/filtered.wz` σ | 0.089 °/s | < 0.15 °/s | ✅ PASS | EKF noise tracking IMU as expected |
| IMU yaw 5s span | 0.4° | n/a (5s window only) | ✅ acceptable | extrapolated bias drift ≈ 0.86°/60s, within 1° tolerance |

## Verdict

**M0 PASS.** No environmental issues; sensors healthy; EKF behaving as expected at rest. The IMU bias is ~3× the #12 M4 baseline but still well within the healthy-threshold (~3.5× safety margin). Possibly explained by:
- Different power-on time / thermal state
- Slightly different orientation than 2026-05-26
- Both are normal Xsens session-to-session variation

**Wheel encoders correctly publishing exact zero** — confirms no encoder noise leak into vyaw. This is critical for the #13 hypothesis: if the wheel signal is corrupted with motion, the bias appears under motion only.

## Conditions
- Robot: stationary, level
- Cameras: OFF
- LiDAR: OFF
- IMU: Xsens MTi-680G, General_RTK profile, /dev/ttyUSB0 @ 921600 baud
- Wheel encoders: SparkMAX-reported, Mandow-corrected (multiplier 1.19)
- Stack uptime when sampled: ~3 min (xsens drift well-converged)

## Sample window in bag
Roughly t=0..170s of `/tmp/yaw_diag_20260527_134442` (bag started 13:44:42; snapshot at ~13:47:30).
