# Control Stack Analysis — September 2026

Question: is the drive and localization stack of the Raptor tracked vehicle accurate (no systematic error) and precise (small, bounded random error), from the motor controllers up to the global pose estimate?

Scope: SPARK MAX/NEO velocity control, Teensy CAN bridge, `actuator_node` kinematics, wheel odometry, Xsens MTi-680G, robot_localization dual EKF, `navsat_transform` and RTK GNSS. Out of scope: perception, planning, costmaps (except where they consume the pose).

## Method
1. **Deployed-state snapshot.** Configuration and code copied from the Jetson on 2026-09-25, not from the repository (`evidence/deployed_snapshot/`).
2. **Code and configuration audit**, layer by layer (documents 01–06).
3. **Measurement.** A read-only 150 s static capture of every relevant topic, a static antenna lever-arm check, and a velocity-lag analysis of the 2026-05 motor logs (`evidence/live_measurements/`, scripts in `evidence/scripts/`).
4. **External research.** Vendor documentation, the robot_localization and Nav2 documentation, the skid-steer literature, and other teams' open-source stacks (`external/`, compared in 07).
5. **Validation plan** with pass criteria for everything that could not be proven from existing data (08).

## Result

The stack is **precise** where it has been measured: motor tracking repeatable to ±0.25% and matched L/R to 0.4%, IMU bias below 0.001 °/s, local odometry closing a square to 1.5% of distance, RTK FIXED to ~1 cm. It is **not yet shown to be accurate**, and the global pose is the weakest output. Most defects sit at layer boundaries (units, filters, covariances, frames) rather than in the architecture, which matches robot_localization, Nav2 and Clearpath reference designs.

| # | Finding | Status |
|---|---|---|
| 1 | SPARK MAX feedforward (param 16) is volts/RPM on firmware 2026; our value supplies ~9% of the intended feedforward. Explains the long-standing under-delivery. | Probable; bench test defined |
| 2 | GNSS position is the antenna, 0.76 m ahead of `base_link`, uncorrected. The map pose is biased along the heading. | Measured |
| 3 | Reported wheel velocity lags real motion by 185 ms; odometry integrates it instead of encoder position. | Measured |
| 4 | IMU covariances are zero: the EKFs treat the IMU as perfect, and the stuck-bias gates cannot detect a bias. | Measured + source |
| 5 | GNSS covariance is optimistic outside RTK FIXED; the stationary map pose wandered 37 × 34 cm. | Measured |
| 6 | Web UI and Nav2 commands have no enforced priority. | Source |
| 7 | GPS rejection gate `13.8` is 13.8σ (effectively off); intended value 3.72. | Source |
| 8 | MPPI acceleration limits are ignored on Humble. | Binary + parameter query |

Full list, verdict by layer and fix order: [09](09_findings_and_recommendations.md).

## Reproducing the measurements
On the Jetson with the stack running (all scripts subscribe only; nothing is published):
```bash
python3 evidence/scripts/capture_static.py 150 /tmp/ctrl_capture     # static capture
python3 evidence/scripts/leverarm_check.py 60                          # antenna offset
```
On any machine:
```bash
gzip <capture_dir>/*.jsonl && python3 evidence/scripts/analyze_static.py <capture_dir>
python3 evidence/scripts/velocity_lag.py docs/motor_sync_logs/csv/avros_wheel_debug.csv
```

## Documents

| # | Document | Content |
|---|---|---|
| 00 | [System overview](00_system_overview.md) | Signal chain, inventory, measured rates and latencies, repository vs robot |
| 01 | [Motor control layer](01_motor_control_layer.md) | SPARK MAX velocity PID, NEO encoder, Teensy bridge |
| 02 | [Drive kinematics layer](02_drive_kinematics_layer.md) | `actuator_node`: arbitration, slew, heading-hold, skid correction |
| 03 | [Wheel odometry](03_wheel_odometry.md) | `/wheel_odom` implementation, timing, covariance |
| 04 | [IMU](04_imu_xsens.md) | Xsens performance, covariance, failure modes |
| 05 | [State estimation](05_state_estimation.md) | Dual EKF configuration and measured behavior |
| 06 | [GNSS and global localization](06_gnss_localization.md) | RTK, lever arm, covariance, map alignment |
| 07 | [External comparison](07_external_comparison.md) | Our stack against published practice and other teams |
| 08 | [Validation plan](08_validation_plan.md) | Tests, pass criteria, status |
| 09 | [Findings and recommendations](09_findings_and_recommendations.md) | Prioritized list with evidence and fix order |

Supporting material:
- `evidence/deployed_snapshot/`: files as running on the Jetson (NTRIP credentials redacted).
- `evidence/live_measurements/`: raw captures (gzipped JSON lines) and results.
- `evidence/scripts/`: capture and analysis scripts; each runs read-only.
- `external/`: research reports with source links.
