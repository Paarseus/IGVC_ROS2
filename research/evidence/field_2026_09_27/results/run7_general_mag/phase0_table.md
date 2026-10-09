| Area | Measurement | Result | Target | Pass |
|---|---|---|---|---|
| Recording | Length | 11.0 min | ≥ 10 min | PASS |
| Recording | Robot stayed still | yes | yes | PASS |
| RTK | Corrections start → first FIXED | 0.0 min | < 3 min | PASS |
| RTK | Launch → first FIXED | 0.0 min | record | — |
| RTK | Time FIXED after first FIXED | 100.0 % | ≥ 95 % | PASS |
| RTK | Drops out of FIXED (not explained by a correction gap) | 0 (0) | 0 unexplained | PASS |
| RTK | FIXED spread (std east / north) | 0.8 / 0.9 cm | ≤ 1.5 cm | PASS |
| RTK | FIXED largest distance from average | 4.4 cm | ≤ 4 cm | FAIL |
| RTK | FIXED: reported accuracy vs measured error | 1.4 vs 1.2 cm (×0.8) | within 2× | PASS |
| RTK | Satellites while FIXED (min / median) | 21 / 24 | ≥ 15 | PASS |
| RTK | HDOP while FIXED (median / max) | 0.7 / 0.8 | ≤ 1.2 | PASS |
| RTK | Correction messages per second | 6.4 | steady | — |
| RTK | Longest correction gap | 1.9 s | ≤ 2 s | PASS |
| Antenna | Antenna ↔ IMU distance (Xsens outputs) | 0.741 m | 0.74 ± 0.03 m | PASS |
| Antenna | Offset direction vs IMU heading | +0.1° | within ±5° | PASS |
| IMU | Gyro bias x / y / z | -0.008 / +0.053 / -0.004 °/s | < 0.05 °/s | FAIL |
| IMU | Gyro noise x / y / z | 0.142 / 0.143 / 0.111 °/s | record | — |
| IMU | Heading drift, whole recording | +0.003 °/min | record | — |
| IMU | Heading drift, second half (settled) | +0.008 °/min | < 0.2 °/min | PASS |
| IMU | Total heading change vs what the gyro bias explains | -0.17° vs -2.50° | record | — |
| IMU | Heading change per minute | -0.18, -0.04, +0.01, -0.04, +0.03, +0.03, +0.03, -0.01, -0.02, +0.02 ° | record | — |
| IMU | Roll / pitch | -0.26° / -0.33° | record | — |
| IMU | Gravity | 9.793 m/s² | 9.81 ± 0.05 | PASS |
| IMU | Reported covariance (orientation / gyro / accel) | 0 / 0 / 0 | non-zero | FAIL |
| Odometry | Wheel speed while parked (max) | 6 RPM | 0 | FAIL |
| Odometry | Wheel odometry movement while parked | 0.0 cm | 0 | PASS |
| Filters | Local position drift (largest) | 0.0 cm | < 1 cm | PASS |
| Filters | Local heading change (follows the IMU heading) | +141.72° | < 0.1° | FAIL |
| Filters | Map position spread while FIXED (std / largest) | 1.2 / 4.4 cm | ≤ 2 cm largest | FAIL |
| Filters | Map→odom correction spread while FIXED (largest) | 4.4 cm | ≤ 2 cm | FAIL |
| Timing | /imu/data rate / longest gap | 100.0 Hz / 50 ms | ≥ 95 Hz | PASS |
| Timing | /wheel_odom rate / longest gap | 20.0 Hz / 62 ms | ≥ 19 Hz | PASS |
| Timing | /odometry/filtered rate / longest gap | 20.1 Hz / 70 ms | ≥ 28 Hz | FAIL |
| Timing | /odometry/global rate / longest gap | 20.1 Hz / 69 ms | ≥ 28 Hz | FAIL |
| Timing | /gnss rate / longest gap | 4.0 Hz / 298 ms | ≥ 3.5 Hz | PASS |
| Timing | /filter/positionlla rate / longest gap | 100.0 Hz / 40 ms | ≥ 3.5 Hz | PASS |
| Timing | /odometry/gps rate / longest gap | 4.0 Hz / 301 ms | ≥ 3.5 Hz | PASS |
| Timing | /rtcm rate / longest gap | 6.4 Hz / 1919 ms | ≥ 0.8 Hz | PASS |
| Timing | /status rate / longest gap | 104.0 Hz / 50 ms | ≥ 3.5 Hz | PASS |
| Timing | /imu/data delay (median / 95th pct) | 70 / 86 ms | < 50 ms | FAIL |
| Timing | /wheel_odom delay (median / 95th pct) | 1 / 1 ms | < 50 ms | PASS |
| Timing | /gnss delay (median / 95th pct) | 143 / 163 ms | < 50 ms | FAIL |
