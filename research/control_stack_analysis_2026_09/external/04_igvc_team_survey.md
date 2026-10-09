# External Research — IGVC Team Control and Localization Stacks

Collected 2026-09-25 from the team's local archives (`docs/winners_research/`, `igvc_winners_research/`, `robojackets/`), 22 igvc.org design reports and team repositories. Items not confirmed in code or a report are marked UNVERIFIED.

## A. Local archives

- **Sooner (1st, 2023–2025)** won without robot_localization. They use a 750-particle filter in a local ENU frame anchored at the first GPS fix. The motion model is encoder deltas and the measurement is GPS (σ 0.45 m), resampled on every fix.
  - 2023 had no tested IMU; heading recovered from GPS-encoder consistency "within the first few seconds".
  - 2025 uses VN-200 yaw.
  - Their low-level control changed every year:
    - 2023: Teensy PID with quadrature encoders over CAN.
    - 2024: RP2040 PID on output-shaft encoders driving SPARK MAX by PWM.
    - 2025: SPARK MAX onboard PID over CAN on a dedicated bus.
  - All years use "CONBus", a typed register protocol for live tuning of gains and kinematic constants.
- **TnTech:** VESC over CAN. A local EKF fuses wheel odometry plus ZED IMU yaw rate. `map ≡ odom` is an identity transform, and GPS generates waypoint goals only; the global EKF is not launched.
- **Hosei:** custom Python EKF fusing FAST-LIO with dual ZED-F9P moving-base heading.
- **RoboJackets (Georgia Tech, ROS 1, ~2020):**
  - mbed velocity PID on encoder ticks (~10 ms loop, UNVERIFIED), Sabertooth output. Their own design notes criticize running networking and PID in one thread ([igvc-firmware](https://github.com/RoboJackets/igvc-firmware)).
  - Single robot_localization EKF at 30 Hz: wheel vx/vy/vyaw, IMU roll/pitch/vyaw/accel, magnetometer yaw, GPS x/y/z absolute, rejection thresholds commented out.
  - A GTSAM iSAM2 smoother exists but is not launched (UNVERIFIED).
  - Their IMU needed ~1 min after driver start to converge.

## B. Teams

| Team / source | Drive, motor control | Velocity loop | Odometry / slip | Estimation | GPS | Published numbers | Validation |
|---|---|---|---|---|---|---|---|
| Sooner 2025 (1st) [report](http://www.igvc.org/design/2025/7.pdf), [code](https://github.com/SoonerRobotics/autonav_software_2025) | Swerve, 8× NEO + SPARK MAX | onboard SPARK MAX over CAN | swerve chosen to reduce slip | particle filter, GPS σ 0.45 m | VN-200, ENU from first fix | 2 m waypoint radius; 2:20 official course time | CONBus live tuning, simulator |
| Sooner 2024 (1st) [report](http://www.igvc.org/design/2024/17.pdf) | 6-wheel diff, 2× NEO + SPARK MAX | MCU PID, output-shaft encoders, PWM | fixed encoder scale factors (0.95/0.8) | particle filter, IMU not fused | VN-200 | 4.9 mph max, 4.1 mph average | web GUI, simulator |
| Wayne State 2025 [report](http://www.igvc.org/design/2025/26.pdf) | diff, RoboClaw 2x60 | inner 1 kHz on RoboClaw (motor encoders) + outer 100 Hz in ROS 2 on post-gearbox encoders | post-gearbox encoders via ESP32 | dual EKF (local encoders + IMU; global RTK + local) | RTK | spec-sheet only | Gazebo twin; injected encoder noise cross-checked against IMU |
| Oakland U 2024/2025 [2024](http://www.igvc.org/design/2024/10.pdf), [2025](https://igvc.secs.oakland.edu/design/2025/14.pdf) | diff; 2024 NEO on ODrive S1 | onboard ODrive | added external encoders to NEOs (built-in "less accurate") | dual EKF (global: GPS + IMU + ZED VIO + wheels); slam_toolbox | ZED-F9P/F9R RTK, 20 Hz | spec-sheet only | Gazebo |
| LTU 2025 [report](http://www.igvc.org/design/2025/12.pdf) | diff, Arduino encoders | not stated | slip / actuator-lag monitor against expected progress (implementation UNVERIFIED) | robot_localization EKF + navsat, DWB | Atlas | **waypoint 0.5–0.8 m RMS** | replayed recorded GPS in Gazebo to tune the EKF; fixed motor noise on GPS by grounding |
| U. Michigan 2025-26 [nav_stack](https://github.com/umigv/nav_stack_2526), [embedded](https://github.com/umigv/embedded_ros_marvin_2426) | diff, ODrive | onboard ODrive | encoder twist | `ekf_local` 100 Hz: encoder vx only + IMU vyaw only; `map→odom` from INS (UNVERIFIED) | VN-300 dual antenna; automatic datum = median of 60–90 s of fixes, rejecting σ > 1 m | none | sensor simulator |
| Virginia Tech 2024/2025 [2024](http://www.igvc.org/design/2024/19.pdf), [2025](http://www.igvc.org/design/2025/19.pdf) | diff + casters, 24 V BLDC | host computes wheel speeds | wheel encoders | robot_localization EKF → slam_toolbox; navsat for waypoints | Novatel | **stopped within 6 in of waypoint** (GPS σ 0.5–0.75 m) | Gazebo course from video |
| U. Toronto 2025 [report](http://www.igvc.org/design/2025/8.pdf) | diff, BLD-750 | PID in a ROS node on a Raspberry Pi | Hall + encoders | robot_localization dual EKF (+ stereo VO, GPS) | Columbus P-7 | **wheel odometry drift ≥ 5% of distance** | remounted IMU after slip found |
| UCF 2024 [report](http://www.igvc.org/design/2024/14.pdf) | diff, Victor SPX PWM | not stated | dropped ZED VO (failed with lighting/obstacles), built wheel odometry | multiple robot_localization EKFs, two IMUs | Spatial GNSS/INS | none | simulation |
| Cedarville 2023 [report](http://www.igvc.org/design/2023/26.pdf) | diff, Sabertooth + Teensy | not stated | optical shaft encoders | custom weighted heading (wheel, IMU, GPS), GPS weight grows near waypoint | ZED-F9P / Emlid RTK | **15 cm ground plane halved static GPS σ** | static GPS logging |
| Bob Jones 2023 [report](http://www.igvc.org/design/2023/27.pdf) | diff, RoboClaw + US Digital encoders | on RoboClaw | encoders | robot_localization + RPP | not stated | none | custom simulator |
| Hosei 2025 [report](http://www.igvc.org/design/2025/21.pdf) | diff, ZLAC8015D | on driver | wheels + FAST-LIO | custom EKF | dual F9P moving-base heading | none | replica course, bags |
| MANAS 2025 (3rd) [code](https://github.com/asmit-mit/autonav-ws) | diff, Cytron + STM32 | STM32 | LiDAR odometry instead of wheels | custom (private) | WIT GPS | none | simulation |

Also read, thin on localization detail: TnTech 2025, UT Austin 2025, LTU Schoolbus 2025 ([2025/16](http://www.igvc.org/design/2025/16.pdf), [2025/9](http://www.igvc.org/design/2025/9.pdf), [2025/1](http://www.igvc.org/design/2025/1.pdf)).

## C. Practices observed that we do not use

1. Automatic GPS datum from a median of fixes with an accuracy gate (U. Michigan).
2. Offline EKF tuning by replaying recorded sensor data (LTU).
3. Measurement-based wheel odometry covariance (Toronto measured ≥ 5% drift on plain wheels).
4. Slip / actuator-lag monitor comparing command, encoders, IMU and GNSS (LTU, Wayne State).
5. Odometry independent of drive slip: external or post-gearbox encoders (Oakland, Wayne State). For tracks, an encoder on an unpowered idler or a trailing wheel (standard in tracked robotics; not seen at IGVC, UNVERIFIED).
6. Dual-antenna GNSS heading (Hosei, U. Michigan VN-300). Hardware change; MTi-680G dual-antenna support not verified.
7. Antenna ground plane (Cedarville).
8. Readiness gate for IMU/GNSS convergence before the first goal (RoboJackets waited a fixed minute).
9. Typed register protocol with read-back for motor-controller parameters (Sooner CONBus).

## D. Caveat
Winning teams used simpler estimators (Sooner: particle filter; TnTech: no map-frame EKF). Estimator sophistication alone does not predict competition performance. Keeping GPS out of the costmap's transform (TnTech, U. Michigan) is the main architectural difference from our stack.
