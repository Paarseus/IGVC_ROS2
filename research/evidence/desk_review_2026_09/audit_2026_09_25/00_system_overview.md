# 00 — System Overview

Scope: everything between the Nav2 velocity command and the pose estimate Nav2 consumes. Perception, planning and costmaps are out of scope.

All values below are from the configuration deployed on the Jetson on 2026-09-25 (`evidence/deployed_snapshot/`), not from the repository defaults. Where the two differ it is noted.

## 1. Signal chain

```
COMMAND PATH (downstream)
  Nav2 MPPI controller ............ 20 Hz, publishes /cmd_vel (v, ω)
        │   phone web UI → /avros/actuator_command (manual path, same node)
        ▼
  actuator_node (Jetson, Python) .. 50 Hz timer
        │  1. command selection (freshness < 0.5 s)
        │  2. slew-rate limit  (v: +0.3 / −1.3 m/s², ω: ±1.2 rad/s²)
        │  3. heading-hold     (|ω| < 0.05 rad/s and |v| > 0.02 m/s → P on IMU yaw, kp 1.5)
        │  4. inverse kinematics, effective track = 0.7366 m × 1.19
        │  5. m/s → motor RPM   (0.01994 m per motor rev)
        ▼  USB serial 115200 baud, "L<rpm> R<rpm>" every 20 ms
  Teensy 4.1 firmware ............. 50 Hz tick
        │  setpoint ramp 100 RPM/tick (≈1.66 m/s² at the track), clamp ±4600 RPM
        │  300 ms host watchdog → 0 RPM
        ▼  CAN 1 Mbit/s: heartbeat + VELOCITY_SETPOINT per tick
  SPARK MAX ×2 (FW 26.1.4) ........ onboard velocity PID, slot 0
        │  kFF 0.000197, kP 0.0007, kI 2.5e-7, kD 0, IZone 600 RPM
        ▼
  NEO → 12.75:1 → chain 1:1 → 20T pulley → track

FEEDBACK PATH (upstream)
  NEO hall encoder → SPARK MAX velocity + position (STATUS_2)
        ▼  CAN
  Teensy → "E L<rpm> <rev> R<rpm> <rev>" at 50 Hz (no timestamp)
        ▼  USB serial
  actuator_node → /wheel_odom at 20 Hz (integrates velocity, not position)
  Xsens MTi-680G → /imu/data 100 Hz, /gnss 4 Hz (raw receiver PVT)
        │  NTRIP client → /rtcm → MTi (RTK corrections, EarthScope PSDM, 3.9 km baseline)
        ▼
  navsat_transform → /odometry/gps (map frame)
  EKF local  (odom frame, 30 Hz): IMU roll/pitch/yaw + angular rate, wheel vx, ZED Δyaw
  EKF global (map frame,  30 Hz): same IMU + wheel vx, GPS x/y absolute
        ▼
  TF map→odom→base_link, /odometry/filtered → MPPI
```

## 2. Component inventory

| Layer | Component | Deployed setting | Source |
|---|---|---|---|
| Chassis | AndyMark Raptor, tracked, timing-belt tracks | gauge 0.7366 m | `actuator_params.yaml` |
| Drive | NEO brushless ×2, ToughBox Mini 12.75:1, 22T:22T #35 chain, 20T 0.5" pulley | 0.01994 m / motor rev, 1.89 m/s at free speed | CLAUDE.md, `actuator_params.yaml` |
| Motor controller | SPARK MAX FW 26.1.4, CAN IDs 1 (L) / 2 (R), Brake idle | slot-0 gains above | `actuator_params.yaml`, firmware |
| Bridge | Teensy 4.1, FlexCAN_T4, 1 Mbit/s | 50 Hz control + feedback | `teensy_diff_drive.ino` (Jetson working copy) |
| Host control | `actuator_node` (rclpy) | 50 Hz loop, 20 Hz odometry | `actuator_node.py` |
| IMU/GNSS | Xsens MTi-680G, General_RTK filter, lever arm [0.74, 0, 0] m | 100 Hz IMU, 4 Hz GNSS | `xsens.yaml` |
| Corrections | NTRIP → EarthScope `PSDM_RTCM3P3` | MSM7 GPS/GLO/GAL, 1 Hz | `ntrip_params.yaml` |
| Estimation | robot_localization dual EKF + navsat_transform | 30 Hz, `two_d_mode` | `ekf.yaml`, `navsat.yaml` |
| Consumer | Nav2 MPPI, DiffDrive model | 20 Hz, 56 × 50 ms horizon, vx ≤ 0.7, ωz ≤ 1.9 | `nav2_params_igvc_autonav.yaml` |

## 3. Measured rates and latencies (static capture, 2026-09-25)

Latency = ROS receive time minus message header stamp. It measures transport plus any stamping offset; it does not capture sensor-internal filter delay.

| Topic | Rate | Latency median / p95 | Stamp source |
|---|---|---|---|
| `/imu/data` | 100.0 Hz | 25.8 / 40.0 ms | MTi UTC time |
| `/gnss` | 4.0 Hz | 91.9 / 107.2 ms | MTi UTC time |
| `/odometry/gps` | 4.0 Hz | 106.8 / 136.3 ms | copied from `/gnss` |
| `/wheel_odom` | 20.0 Hz | 3.6 / 6.1 ms | host time at publish (not at measurement) |
| `/odometry/filtered` | 29.9 Hz | 24.8 / 43.4 ms | EKF |
| `/odometry/global` | 27.0 Hz (target 30) | 19.8 / 30.6 ms | EKF |

Source: `evidence/live_measurements/static_2026_09_25/SUMMARY.txt`.

## 4. Command-to-motion latency budget (analytical)

| Stage | Worst-case added delay |
|---|---|
| MPPI period | 50 ms |
| actuator_node timer phase | 20 ms |
| USB serial + parse | ~1–2 ms |
| Teensy tick phase | 20 ms |
| Teensy ramp (only when the step exceeds 100 RPM/tick) | 0 ms for host-slewed commands |
| SPARK MAX velocity loop response | dominated by velocity-measurement filtering (see 01) |

Worst case before the motor loop acts: ~90 ms. The host slew limiter, not the transport, dominates the time to reach a new speed (0.3 m/s² → 2.3 s to reach 0.7 m/s).

## 5. Differences between the repository and the robot

| Item | Repository (`origin/main`) | Jetson working copy |
|---|---|---|
| Teensy firmware | PR #23 version (setpoint ramp) | Source identical to `origin/main`; shows as an uncommitted edit only because the Jetson checkout (`d40e18a`) predates PR #23. Flashed binary not verified. |
| `actuator_params.yaml` | Teensy serial 18639150 | Teensy serial 20383890 (replacement board) |
| `navsat.yaml` | Michigan datum | local datum set 2026-09-25 |
| `ntrip_params.yaml` | MDOT CORS | EarthScope |
| `max_angular_rps` | CLAUDE.md states 1.0 | 1.5 |

The firmware actually running on the Teensy cannot be confirmed from the host. A firmware version string in the `D` diagnostic line would close this gap.
