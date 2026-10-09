| Area | Measurement | Result | Target | Pass |
|---|---|---|---|---|
| Recording | Length | 11.0 min | ≥ 10 min | PASS |
| Recording | Robot stayed still | yes | yes | PASS |
| RTK | Corrections start → first FIXED | never | < 3 min | FAIL |
| RTK | Time FIXED after first FIXED | n/a | ≥ 95 % | FAIL |
| RTK | Drops out of FIXED (not explained by a correction gap) | 0 (0) | 0 unexplained | PASS |
| RTK | FLOAT: reported accuracy vs measured error | 55.4 vs 62.5 cm (×1.1) | within 2× | PASS |
| RTK | Correction messages per second | 5.3 | steady | — |
| RTK | Longest correction gap | 1.9 s | ≤ 2 s | PASS |
| IMU | Gyro bias x / y / z | +0.005 / +0.057 / -0.007 °/s | < 0.05 °/s | FAIL |
| IMU | Gyro noise x / y / z | 0.124 / 0.142 / 0.103 °/s | record | — |
| IMU | Heading drift, whole recording | -1.273 °/min | record | — |
| IMU | Heading drift, second half (settled) | -1.361 °/min | < 0.2 °/min | FAIL |
| IMU | Total heading change vs what the gyro bias explains | -2.67° vs -4.50° | record | — |
| IMU | Heading change per minute | +9.14, +3.98, -7.75, +1.57, -1.33, -0.40, -4.20, -1.47, +0.56, -2.68 ° | record | — |
| IMU | Roll / pitch | +1.40° / -0.15° | record | — |
| IMU | Gravity | 9.795 m/s² | 9.81 ± 0.05 | PASS |
| IMU | Reported covariance (orientation / gyro / accel) | 0 / 0 / 0 | non-zero | FAIL |
| Odometry | Wheel speed while parked (max) | 0 RPM | 0 | PASS |
| Odometry | Wheel odometry movement while parked | 0.0 cm | 0 | PASS |
| Filters | Local position drift (largest) | 0.0 cm | < 1 cm | PASS |
| Filters | Local heading change (follows the IMU heading) | -2.67° | < 0.1° | FAIL |
| Timing | /imu/data rate / longest gap | 100.0 Hz / 40 ms | ≥ 95 Hz | PASS |
| Timing | /wheel_odom rate / longest gap | 20.0 Hz / 61 ms | ≥ 19 Hz | PASS |
| Timing | /odometry/filtered rate / longest gap | 24.6 Hz / 71 ms | ≥ 28 Hz | FAIL |
| Timing | /odometry/global rate / longest gap | 24.2 Hz / 70 ms | ≥ 28 Hz | FAIL |
| Timing | /gnss rate / longest gap | 4.0 Hz / 281 ms | ≥ 3.5 Hz | PASS |
| Timing | /filter/positionlla rate / longest gap | 100.0 Hz / 40 ms | ≥ 3.5 Hz | PASS |
| Timing | /odometry/gps rate / longest gap | 4.0 Hz / 301 ms | ≥ 3.5 Hz | PASS |
| Timing | /rtcm rate / longest gap | 5.3 Hz / 1900 ms | ≥ 0.8 Hz | PASS |
| Timing | /status rate / longest gap | 104.0 Hz / 40 ms | ≥ 3.5 Hz | PASS |
| Timing | /imu/data delay (median / 95th pct) | 38 / 58 ms | < 50 ms | PASS |
| Timing | /wheel_odom delay (median / 95th pct) | 1 / 1 ms | < 50 ms | PASS |
| Timing | /gnss delay (median / 95th pct) | 79 / 99 ms | < 50 ms | FAIL |
