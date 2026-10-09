# Phase 0: Setup and Static Health

**Purpose:** confirm that every sensor and estimator works correctly while the robot stands still, and record baseline numbers, before any driving test. If a check fails here, later tests would measure the wrong thing.

**Robot state:** parked in open sky, not moving, motors idle. Nobody touches it during the 10-minute recording.

## What we measure and why

### 1. Power and firmware (Teensy status line)
| Measurement | Why it matters | Target |
|---|---|---|
| Battery voltage at idle | The Jetson and motors share one 12 V supply; low voltage causes brown-outs under load | ≥ 12.0 V |
| Firmware version (ramp value `M`) | Confirms the new firmware is the one running | `M=100` present |
| Motor mode at idle | New firmware holds motors in velocity mode at 0 RPM | `mode=VEL` |

### 2. Jetson health
| Measurement | Why it matters | Target |
|---|---|---|
| Clock synchronized (NTP) | GPS and IMU messages carry GPS time; a wrong Jetson clock makes every message look late | synchronized |
| CPU load, temperature, free disk | Control loop needs spare CPU; recordings need disk | load < 2 at idle, temp < 70 °C, disk > 50 GB |

### 3. RTK GPS (10-minute static recording)
| Measurement | Why it matters | Target |
|---|---|---|
| Time from corrections start to first FIXED | How long to wait before every run | < 3 min (last measured: 11 min) |
| Share of time FIXED after the first FIXED | Drops to FLOAT move the position by up to tens of cm | ≥ 95 % |
| Number of drops out of FIXED, and whether corrections stopped before each drop | Separates a sky or antenna problem from an internet problem | Every drop explained by a correction gap > 2 s, or none |
| Position spread while FIXED (standard deviation and largest distance from the average) | This is the best accuracy the robot can have | spread ≤ 1.5 cm, largest ≤ 4 cm |
| Accuracy the receiver reports vs the spread we measure, per fix type | The position filter trusts the reported value; if it is too optimistic, the filter follows noise | reported and measured within 2× of each other |
| Satellites used and HDOP (satellite geometry; lower is better) | Explains poor fixes | ≥ 15 satellites, HDOP ≤ 1.2 |
| Correction messages: rate and longest gap | Corrections must arrive steadily | about 1 set per second, no gap > 2 s |

### 4. Antenna offset (static)
| Measurement | Why it matters | Target |
|---|---|---|
| Distance between the GPS antenna position and the IMU position reported by the Xsens | Checks that the 0.74 m offset stored in the Xsens is applied | 0.74 ± 0.03 m |
| Direction of that offset vs the IMU heading | The antenna is straight ahead of the IMU; a mismatch means a heading or mounting error | within 5° |
| Tape measurement, antenna centre to IMU centre (done by hand) | Independent check of the stored value | record |

### 5. IMU (Xsens), static
| Measurement | Why it matters | Target |
|---|---|---|
| Gyro bias (average rotation rate while still) | A bias makes the robot think it is turning | < 0.05 °/s on every axis |
| Gyro noise | Sets how much the filter should trust the gyro | record |
| Heading drift over the recording | Detects the known "stuck bias" failure | < 0.2° per minute |
| Roll and pitch | Robot should be level; checks mounting | record |
| Gravity magnitude from the accelerometer | Sanity check of the accelerometer | 9.81 ± 0.05 m/s² |
| Covariance values the IMU reports | Zero means the filter treats the IMU as perfect | record (known problem: zero) |

### 6. Wheel odometry and position filters, static
| Measurement | Why it matters | Target |
|---|---|---|
| Wheel speed and distance while parked | Must be zero, or odometry drifts at rest | 0 |
| Local position estimate (odom frame) drift | The robot must not "move" while parked | < 1 cm, < 0.1° |
| Map position spread while FIXED | Shows whether the map position follows GPS noise | ≤ 2 cm |
| Size of each jump in the map position when RTK re-fixes | A jump smears obstacles in the map | < 10 cm |
| Map-to-odom correction spread | Same, as the navigation stack sees it | ≤ 2 cm |

### 7. Message timing
| Measurement | Why it matters | Target |
|---|---|---|
| Rate and longest gap for each topic | Missing or slow data degrades everything | IMU 100 Hz, wheel odometry ≥ 20 Hz, filters 30 Hz, GPS ≥ 4 Hz, LiDAR 10 Hz |
| Delay between measurement time and arrival (IMU, wheel odometry, GPS) | Late data puts the estimate behind the real robot | < 50 ms (GPS measured at about 90 ms before) |

## Procedure
1. **Stop** the running web UI launch (motors stop; robot is parked). Read the Teensy status line. Record Jetson health.
2. **Start** the web UI launch and `localization.launch.py`. Note the start time.
3. **Record** 10 minutes of all topics listed in `phase0_record.sh` while the robot stands still.
4. **Analyze** with `phase0_analyze.py`; results go to `results/`.
5. **By hand:** tape-measure antenna centre to IMU centre; mark the ground under the IMU and under the antenna (used in later phases).
6. **Warm-up:** drive 5 m out and back with the joystick (the IMU heading needs motion to settle). Not recorded.

## Files
| File | Purpose |
|---|---|
| `phase0_record.sh` | Records the Phase 0 topics to a bag (read-only) |
| `teensy_status.py` | Reads one Teensy status line (only while `actuator_node` is stopped) |
| `phase0_analyze.py` | Computes every number above from the bag and writes the results table |
| `results/` | Raw outputs and [PHASE0_RESULTS.md](results/PHASE0_RESULTS.md) |
