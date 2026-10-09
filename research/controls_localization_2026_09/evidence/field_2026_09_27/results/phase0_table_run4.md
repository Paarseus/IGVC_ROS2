| Area | Measurement | Result | Target | Pass |
|---|---|---|---|---|
| Recording | Length | 25.0 min | ≥ 10 min | PASS |
| Recording | Robot stayed still | NO — results invalid | yes | FAIL |
| RTK | Corrections start → first FIXED | never | < 3 min | FAIL |
| RTK | Time FIXED after first FIXED | n/a | ≥ 95 % | FAIL |
| RTK | Drops out of FIXED (not explained by a correction gap) | 0 (0) | 0 unexplained | PASS |
| RTK | FLOAT: reported accuracy vs measured error | 81.2 vs 1084.1 cm (×13.4) | within 2× | FAIL |
| RTK | Correction messages per second | 6.3 | steady | — |
| RTK | Longest correction gap | 3.3 s | ≤ 2 s | FAIL |
| IMU | Gyro bias x / y / z | -0.006 / +0.062 / +0.227 °/s | < 0.05 °/s | FAIL |
| IMU | Gyro noise x / y / z | 0.776 / 1.161 / 4.353 °/s | record | — |
| IMU | Heading drift, whole recording | +17.583 °/min | record | — |
| IMU | Heading drift, second half (settled) | +7.552 °/min | < 0.2 °/min | FAIL |
| IMU | Total heading change vs what the gyro bias explains | +350.66° vs +340.91° | record | — |
| IMU | Heading change per minute | +0.19, +0.10, -0.07, -0.22, +0.12, +174.72, -0.26, +0.04, -0.14, +0.02, +0.09, -0.07, -0.01, +162.97, +12.90, +0.04, +0.04, +0.01, +0.23, -0.07, -0.06, -0.07, +0.00, +0.15 ° | record | — |
| IMU | Roll / pitch | +0.22° / +2.06° | record | — |
| IMU | Gravity | 9.806 m/s² | 9.81 ± 0.05 | PASS |
| IMU | Reported covariance (orientation / gyro / accel) | 0 / 0 / 0 | non-zero | FAIL |
| Odometry | Wheel speed while parked (max) | 4766 RPM | 0 | FAIL |
| Odometry | Wheel odometry movement while parked | 1962.8 cm | 0 | FAIL |
| Filters | Local position drift (largest) | 1975.5 cm | < 1 cm | FAIL |
| Filters | Local heading change (follows the IMU heading) | +350.66° | < 0.1° | FAIL |
| Timing | /imu/data rate / longest gap | 100.0 Hz / 79 ms | ≥ 95 Hz | PASS |
| Timing | /wheel_odom rate / longest gap | 20.0 Hz / 64 ms | ≥ 19 Hz | PASS |
| Timing | /odometry/filtered rate / longest gap | 21.3 Hz / 75 ms | ≥ 28 Hz | FAIL |
| Timing | /odometry/global rate / longest gap | 21.0 Hz / 74 ms | ≥ 28 Hz | FAIL |
| Timing | /gnss rate / longest gap | 4.0 Hz / 500 ms | ≥ 3.5 Hz | PASS |
| Timing | /filter/positionlla rate / longest gap | 100.0 Hz / 79 ms | ≥ 3.5 Hz | PASS |
| Timing | /odometry/gps rate / longest gap | 4.0 Hz / 500 ms | ≥ 3.5 Hz | PASS |
| Timing | /rtcm rate / longest gap | 6.3 Hz / 3331 ms | ≥ 0.8 Hz | PASS |
| Timing | /status rate / longest gap | 104.0 Hz / 79 ms | ≥ 3.5 Hz | PASS |
| Timing | /imu/data delay (median / 95th pct) | 70 / 96 ms | < 50 ms | FAIL |
| Timing | /wheel_odom delay (median / 95th pct) | 1 / 2 ms | < 50 ms | PASS |
| Timing | /gnss delay (median / 95th pct) | 112 / 138 ms | < 50 ms | FAIL |
