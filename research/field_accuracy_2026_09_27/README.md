# Field Accuracy Review: 2026-09-27

**Question.** Can the robot move and localize precisely enough to thread IGVC obstacle gaps? The minimum passage is 5 ft (1.524 m) for a robot 0.83 m wide, so there is 0.35 m of slack per side. Which parts of the RTK, PID, odometry, localization and Nav2 stack are misconfigured, and how do we test them outdoors today?

**Method.**
- Four parallel desk reviews of the configuration and code copied from the Jetson today (`deployed_snapshot/`).
- They build on the 2026-09 audit (`../control_stack_analysis_2026_09/`), plus vendor documentation, upstream source and the team's earlier logs.
- The headline claims were re-checked against the deployed files and the live robot (marked ✔ below).
- No field test has been run yet. Nothing on the robot or in the repository was changed.

| # | Scope | Report | Script (read-only, never publishes) |
|---|---|---|---|
| 1 | Motor velocity loop, PID, Teensy ramp | [01_motor_pid_and_ramp.md](01_motor_pid_and_ramp.md) | `motor_step_log.py` |
| 2 | RTK GNSS and global localization | [02_rtk_and_global_localization.md](02_rtk_and_global_localization.md) | `rtk_field_analyze.py` |
| 3 | Kinematics, wheel odometry, IMU, local EKF | [03_odometry_kinematics_local_ekf.md](03_odometry_kinematics_local_ekf.md) | `odom_kinematics_analyze.py` |
| 4 | Nav2 fine motion in tight obstacle fields | [04_fine_motion_tight_obstacles.md](04_fine_motion_tight_obstacles.md) | — |

## Top findings (all scopes, deduplicated)

✔ = verified during this review against the deployed files or the live robot. Otherwise the status is as the scope report states.

| # | Finding | Severity | Status | Effect on tight-gap driving | Scope |
|---|---|---|---|---|---|
| 1 | **`base_link` is at the IMU, 0.31 m behind the track centre.** Chassis and treads sit at x = +0.3143 in `avros.urdf.xacro:43,71,85`, while the line 8 comment claims base_link is the chassis centre. MPPI, footprint rotation and odometry all assume the robot spins about `base_link`. | High | ✔ geometry; magnitude needs test | A 90° pivot moves base_link about 0.44 m without the estimator knowing. Rear corners sweep 0.72 m radius against 0.50 m predicted. The error cancels over a full loop, which is why the square test missed it. | 3, 4, 2 |
| 2 | **Feedforward (param 16) is in volts/RPM.** 0.000197 supplies about 9% of what is needed. Team bench data: feedforward alone moved the tracks 8 RPM instead of the ~1540 expected. The correct value is about 0.0021. | Critical | Confirmed from bench logs | Speed depends on the integrator (~3.5 s time constant). This causes overshoot after each ramp, slow low-speed response, and inflates the 1.19 skid multiplier. | 1 |
| 3 | **Heading-hold replaces MPPI's small turn commands.** Below \|ω\| 0.05 rad/s while moving, the command becomes 1.5 × yaw error and skips the angular slew (`actuator_node.py:446-455`). The IMU "fresh" flag is never cleared (`:234,348`). | High | ✔ | Discards the small corrections MPPI uses to centre in a gap. A stale IMU leaves the robot steering toward an old heading. | 3, 4 |
| 4 | **GNSS fix is the antenna, 0.74 m ahead of base_link, uncorrected.** The MTi lever arm only moves `/filter/positionlla`; navsat uses raw `/gnss`. | High | Confirmed (source + measurement) | Goals land about 0.74 m short. After a 180° turn, map-frame obstacles shift up to 1.5 m. | 2 |
| 5 | **Stop is now actively driven.** New firmware: `S`, E-STOP and the watchdog command 0 RPM in velocity mode (`teensy_diff_drive.ino:326-337`), and idle sends `S` at 50 Hz. | Medium–High | ✔ code; effect needs test T4 | The PID fights any motion at stop: possible reverse kick and current spike on the shared 12 V rail. Brake idle never engages while `actuator_node` runs. | 1 |
| 6 | **Local costmap is 50 × 50 m at 0.2 m cells** (`nav2_params_igvc_autonav.yaml:276-278`). | Medium | ✔ | Rounding can eat up to 0.2 m per side of a 1.524 m gap. 0.1 m cells in a 20 × 20 m window is fewer cells than today. | 4 |
| 7 | **Humble MPPI's cost critic samples cost only at `base_link`**, 0.81 m behind the nose. | Medium | Confirmed (upstream source) | The front relies only on the binary footprint check, so it centres in a gap late. | 4 |
| 8 | **The RTK state is ignored everywhere** (driver, navsat, mission_manager). The GPS gate is 13.8σ, effectively off. There's no initial covariance. The map pose jumped 35 cm at FLOAT→FIXED in the 09-25 bag. `/gnss` is about 92 ms old and applied without lag compensation. | High | Confirmed (bag + source) | The map pose steps and lags under the robot; global obstacles and the path move with it. | 2 |
| 9 | **The local EKF fuses absolute Xsens yaw at 1e-9 variance; IMU covariances are zero.** Wheel yaw rate is fused in neither EKF (`ekf.yaml:61,163`; CLAUDE.md is stale). | Medium | ✔ config | Any Xsens heading correction rotates every remembered obstacle (2° ≈ 10 cm at 3 m). | 3 |
| 10 | **Command priority is not enforced** between web UI and Nav2 (both write the same target). | High | ✔ (earlier this session) | Joystick takeover mid-run mixes with Nav2; E-STOP is the only reliable override. | 1, 3 |
| 11 | **The skid multiplier of 1.19 was calibrated in spins on pavement and mostly compensates motor under-delivery (#2).** Distance scale was checked once on asphalt (±3%), never on grass. | Medium | Probable | Over- or under-turns on grass. It should fall to about 1.04 once feedforward is fixed. | 1, 3 |
| 12 | **MPPI acceleration limits are not read on Humble,** while the actuator slews ω at 1.2 rad/s². | Medium | Confirmed | About 0.4 rad heading lag through a slalom reversal. | 4 |
| 13 | **`actuator_node` logs every Teensy ack at INFO:** 49 lines/s at idle. `~/webui.log` reached 37 MB. | Low | ✔ live | Disk and CPU waste; also buries real warnings. | 1 |
| 14 | **`actuator_node` pushes the yaml gains on every start,** overwriting anything burned to SPARK flash. The yaml is the real source of truth. | Info | ✔ (earlier this session) | New gains must go into `actuator_params.yaml`. | 1 |

Also verified today:
- Live `actuator_node` params match the yaml.
- The control loop runs at **50 Hz**, not the 20 Hz some documents state.
- The angular cap is **1.5 rad/s**, not the 1.0 in CLAUDE.md.

Scope 2 also confirmed:
- Map heading and datum handling are correct (`/fromLL` matches independent ENU to 0.001°).
- The RTCM message set is fine for the F9P.

## Is PID re-tuning needed?

**Yes, and it's the first thing to do.** The current gains aren't wrong because they're badly tuned; they're wrong because feedforward is about 11× too small (finding 2). The P and I terms have been stretched to cover for it, which makes low-speed response slow and integrator-driven. That is the worst case for fine control. Also, kP 0.0007 is near the oscillation point set by the 185 ms velocity lag.

Tuning order, detailed in scope 1:
1. Feedforward about 0.0021, verified by test T1.
2. kI down to about 1e-7 with kIZone 200.
3. Try a lower kP (0.0005, then 0.0004).
4. Re-measure the skid multiplier.
5. Write the values into `actuator_params.yaml`, restart, and BURN.

Leave the Teensy ramp at M = 100; the host slew is the binding limit.

## Field session plan (merged, in order)

Each scope report has the exact commands, record topics and numeric pass/fail criteria for its tests. This orders them so each stage validates what the next depends on, and shares bring-ups.

**Rules for every test:**
- One person on the E-STOP.
- CycloneDDS exported in every shell.
- No RViz on the Jetson.
- The web UI stays connected in **AUTO ON**, so the CLI `/cmd_vel` isn't fought and E-STOP stays live.
- Heading-hold is set explicitly per test with `ros2 param set /actuator_node heading_hold_deadband 0.0` (off) or `0.05` (default).
- RTK-referenced runs count only while `rtk_status == 2`.

**Stage 0: bring-up and preconditions (15 min).**
- Bring-up A: `webui.launch.py` (actuator + web UI) plus `localization.launch.py` (sensors, EKFs, navsat).
- Preconditions:
  - Clear sky, RTK FIXED, 30 s IMU stationary drift < 0.2°.
  - Motion warm-up: drive 5 m out and back.
- Run RTK test **A** (static repeatability and time-to-FIXED) during the warm-up wait.
- Mark the ground under the IMU and the antenna, and tape-measure the lever arm.

**Stage 1: motor layer, current gains (baseline, 30 min).**
- Scope 1 **T1** (feedforward-only delivery; start at kFF 0.0005 as a safety step).
- **T2** (low-speed steps at 0.1–0.4 m/s).
- **T4** (normal stop, E-STOP, idle push test).
- **T5** (L/R matching).
- Log with `motor_step_log.py`. **If T1 confirms finding 2, run the tuning procedure here,** then repeat T2 with the new gains before moving on. Everything downstream depends on it.

**Stage 2: kinematics and odometry, final gains (40 min).**
- Scope 3 **T1** (10 m straight, both directions, against RTK and tape) together with scope 2 **C** (the same runs give heading offset, lever arm and GPS lag).
- Scope 3 **T2** (360°/720° spins) together with scope 2 **D** and scope 4 **T1**. Chalk mark under the IMU; the spin-circle fits give the true rotation centre (finding 1) and α on grass.
- Scope 3 **T3** (small-ω arcs, heading-hold on/off: finding 3).
- Scope 3 **T4** (square / figure-8 / L-path against RTK; the L-path exposes finding 1).
- Scope 2 **B** (known-point return at 3 headings).

**Stage 3: Nav2 fine motion (45 min).**
- Bring-up B: stop bring-up A, then `navigation.launch.py enable_mission_manager:=false`, plus the web UI node on its own (not `webui.launch.py`, which starts a second `actuator_node`).
- Scope 4:
  - **T7** (creep at 0.05–0.2 m/s).
  - **T2** (heading-hold on/off, start 0.3 m off-centre).
  - **T3** gap ladder at 2.0 / 1.524 / 1.3 / 1.2 / 1.0 m; the 1.0 m gap must be refused.
  - **T4** (barrel dead-centre).
  - **T5** (4-barrel slalom).
  - **T6** (barrel fade / clear-around).
  - CPU check: `/cmd_vel` ≥ 18 Hz.
- Scope 2 **E**: short GPS goal, tape-measured.

**Optional:** scope 1 **T6** (Teensy-side ramp/BURN checks). Needs `actuator_node` stopped.

## Changes to decide after the tests

Proposals only; nothing is applied. Each is gated on the test named in its scope report.

**Safe to try at runtime today** (`ros2 param set`, reverts on restart; confirm each reports success):
- `heading_hold_deadband 0.0` while Nav2 drives.
- MPPI: vx_max 0.5 in obstacle sections, PathAlign weight 12→8, cost critic 5→7, near_goal_distance 1.0→0.5, wz_std/vx_std 0.4/0.2.
- New motor gains via `ros2 param set /actuator_node kFF/kP/kI/kIZone`.

**Need a file change and restart:**
- `actuator_params.yaml`: the tuned gains, and the grass skid multiplier and `m_per_motor_rev` from the tests.
- Local costmap at 0.1 m in a 20 × 20 m window, STVL decay 5 s.
- Local EKF fusing gyro rate instead of absolute yaw.
- IMU and GNSS covariances set by RTK state.
- Map EKF: initial covariance, gate 5σ, `smooth_lagged_data`, lower Q_xy.

**Code changes:**
- Command priority in `actuator_node`.
- Heading-hold limited to the manual path, with IMU staleness handled.
- Log `OK` acks at DEBUG.
- A GNSS antenna frame (`gnss_link`) plus relay, and an RTK gate in `mission_manager`.
- Move `base_link` to the measured rotation centre, only if the spin test confirms, and not right before competition.

**Documentation:**
- CLAUDE.md is stale on: wheel vyaw fusion, the 4σ gate, MPPI accel matching, the 1.0 angular cap, and `S` meaning duty-0 brake.
- `avros.urdf.xacro` line 8 comment.

## Caveats
- All magnitudes marked Probable or Hypothesis in the scope reports are desk estimates until the tests run.
- The analysis scripts compile and pass synthetic self-tests, but haven't been run on real field bags. `motor_step_log.py` did reproduce the May logs.
- Scope 2 notes that NTRIP credentials are stored in plain text in the git-tracked `ntrip_params.yaml`.
