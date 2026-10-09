# 03: Odometry, drive kinematics, IMU and local EKF: field accuracy (2026-09-27)

Scope: when we command (v, ω), how accurately the robot moves, and how accurately the odom frame knows where it moved over 0–20 m.
Builds on `research/control_stack_analysis_2026_09/` (02, 03, 04, 05, 08, 09). Deployed files: `deployed_snapshot/` in this folder. It is byte-identical to the audit snapshot for `actuator_node.py`, `actuator_params.yaml`, `ekf.yaml`, `xsens.yaml` and `avros.urdf.xacro`.
Analysis script (read-only, bag in → metrics out): `odom_kinematics_analyze.py` in this folder.

## Summary

- **New and most important: `base_link` is probably not the chassis rotation centre.** base_link sits under the IMU. The URDF places the tread centroid 0.314 m ahead of it, and the measured footprint centre is 0.27 m ahead. The EKF model assumes vy = 0 at base_link. It is not: in a turn, base_link slides sideways at about −ω·0.3 m/s. The resulting pose error depends only on the net heading change: about 0.42 m after a 90° turn and about 0.60 m after 180°. The error cancels over a closed loop, which is why the 2026-05 square test (net 360°) closed at 6 cm and did not show it. For a tight course it matters. Obstacles seen before a 90° turn are misplaced by up to about 0.4 m relative to the robot, and the error also adds to the known rotation-phantom smear. Test T2 measures the offset directly from the RTK antenna circle.
- **Heading-hold replaces MPPI's ω (it does not add to it).** It does this whenever |ω_slewed| < 0.05 rad/s and |v| > 0.02 m/s (`actuator_node.py:447-455`). It also has no IMU staleness check, and it makes the chassis physically follow Xsens heading drift. That was seen on 2026-05-28: a −13° heading drift over 9 s on the first leg after start-up.
- **The local EKF's heading is the Xsens absolute yaw at 1e-9 variance.** Every Xsens heading correction therefore rotates the odom frame, and with it every remembered obstacle: 1° is 5 cm at 3 m.
- **Wheel ω is not fused in either EKF.** It was removed 2026-05-30; `ekf.yaml` header lines 5-7 and CLAUDE.md are stale. The multiplier and α therefore affect only how well commands are delivered, not the state estimate.
- **The distance scale has only been checked once, coarsely.** That check was on asphalt: a 3 m tape mark, "ended near 3.8 m", about ±3%. It has never been checked on grass.
- **The 185 ms vx lag has a small effect on MPPI but a real one on the pose.** On Humble, MPPI uses the odometry speed only for the first 50 ms step of each rollout (verified in the source). The pose, however, lags by v × 0.185 s (13 cm at 0.7 m/s). That is non-conservative after re-accelerating near obstacles that were marked while slow.
- Issue #17 is still open: forward delivery drops to 43% right after a reverse leg. It matters for back-up recovery followed by forward motion in tight spots.

## Findings

| # | Finding | Severity | Status | Evidence | New / audit |
|---|---|---|---|---|---|
| K1 | base_link (IMU, x=0) is ~0.27–0.31 m behind the track/footprint centre, and the EKF and MPPI DiffDrive both assume rotation about base_link. In a spin, base_link truly moves on a circle of radius x_c; the EKF shows 0. Pose error = 2·x_c·sin(Δψ/2): 0.42 m at 90°, 0.60 m at 180°. MPPI also mispredicts the footprint sweep: the rear corners swing wider (~0.72 m, not 0.50 m) and the front narrower. It adds to the rotation-phantom LiDAR arc: the velodyne really orbits x_c at ~0.22 m, not 0.089 m. | **High** | **Probable** (geometry confirmed; the true x_c depends on the centre of mass, measured in T2) | `avros.urdf.xacro:43,71,85` (chassis and tread at x=0.3143), `:131` (IMU at base_link x=0); footprint `nav2_params_igvc_autonav.yaml:287` (+0.81/−0.28); `obstacle_stall_rca_2026_05_30.md:136`; `lidar_rotation_phantom_diagnosis_2026_05_29.md` C2 assumes rotation about base_link | **New** |
| K2 | Heading-hold sets ω = 1.5·(yaw_lock − yaw), clamped ±0.75 rad/s. It **replaces** the MPPI command for any \|ω_slewed\| < 0.05 at \|v\| > 0.02, and its output bypasses the angular slew. At 0.3–0.7 m/s the 0.05 rad/s deadband covers every path with a turn radius above 6–14 m. MPPI's gentle corrections are discarded until they exceed 0.05, so it behaves like a relay with ±cm lateral limit cycles, and the lock may push against MPPI (opposite sign). | Medium | Mechanism **Confirmed**; impact Hypothesis (T3) | `actuator_node.py:447-455` | Audit D2/D3; extended |
| K3 | `_imu_fresh` is set True once and never cleared (`:349`). If the IMU stops publishing during a lock, the yaw error is frozen and constant: a steady turn of up to 0.75 rad/s with no warning. | Medium | Confirmed (code) | `actuator_node.py:233,349,446` | **New** |
| K4 | Heading-hold makes the chassis physically follow drift in the Xsens quaternion. On 2026-05-28 it injected +0.0177 rad/s for 9 s (9.1° commanded) while the IMU heading converged: −13° drift on pass 1, −1.7° on pass 2. | Medium | Confirmed (bag) | `docs/yaw_diag_session_2026_05_28/validation_results.md` §"Per-leg yaw drift" | Audit I2; extended |
| K5 | The local EKF fuses **absolute** IMU yaw with zero covariance (clamped to 1e-9), so odom yaw is the Xsens yaw exactly. Warm-up convergence and GNSS-driven heading corrections step the odom frame and rotate the local costmap's memory: δψ·d, i.e. 2° gives 10 cm at 3 m. | Medium–High for tight gaps | Confirmed (config); magnitude per session | `ekf.yaml:36-41`, audit I1/E3/E5 | Audit E5; impact on gap threading new |
| K6 | Wheel vyaw is **not** fused in either EKF: odom `odom0_config` = vx only (`ekf.yaml:58-62`), map `odom1_config` = vx only (`:160-164`). The comments at `ekf.yaml:5-7,13` and CLAUDE.md ("both EKFs fuse vx,vyaw") are stale. Wheel ω therefore has **no** effect on the state. On pavement /wheel_odom ω ≈ 0.96–0.97 × true ω (α_meas); on grass it is unknown and may read high. | Low (doc) / informative | Confirmed | `ekf.yaml`; `skid_steer_kinematics_findings_2026_05_18.md` §2 | Audit (doc item 22); α on grass new test |
| K7 | Multiplier 1.19 was calibrated in **spins on indoor/smooth concrete**. It is ~85% motor-delivery compensation and ~4% skid. Grass raises both lateral scrub (larger effective track) and motor load, so the needed multiplier probably **rises** (Hypothesis: 1.2–1.5). Arcs likely need less than spins. With MPPI closing the loop on IMU heading, a wrong α shows up as curvature mismatch and overshoot in in-place turns near obstacles, not as estimator error. | Medium | Probable | skid doc §4, §7; audit D7 | Audit D7/V10 |
| K8 | Distance scale 0.01994 m/rev was "confirmed" only by EKF 3.84 m against a tape (">3 m mark, ended near 3.8 m") on asphalt. Resolution is ~±3%, it was not repeated, and it was never done on grass. Expect wheel odometry to over-count by 1–5% on grass (longitudinal slip), more during the 0.3 m/s² ramps and on soft ground (Hypothesis). | Medium | Partly refutes audit O5 ("never measured"): a coarse check exists | `docs/yaw_diag_session_2026_05_28/validation_results.md:14,47-53` | Refines audit O5 |
| K9 | The 185 ms lag in reported velocity is integrated into /wheel_odom, and the local EKF has no other vx source, so its position lags true by v·0.185 (6.5 cm at 0.35 m/s, 13 cm at 0.7 m/s). The error is consistent relative to obstacles marked at the same speed. It is **non-conservative** relative to obstacles marked while stopped or slow: after re-accelerating, the robot is ~v·0.185 closer to them than it believes. Effect on MPPI rollouts is small: Humble `MotionModel::predict` copies `cvx[i-1]` into `vx[i]`, and odometry speed seeds only column 0. | Medium | Confirmed (MPPI source); pose effect derived | audit O1/V1; `nav2_mppi_controller` humble `optimizer.cpp` / `motion_models.hpp` | Audit O1; MPPI nuance new |
| K10 | /wheel_odom is published at 20 Hz (state timer, `:314,553`), stamped at timer time, not at E-line receipt. The YAML and docstring say 50 Hz. | Low–Medium | Confirmed | `actuator_node.py:314,576` | Audit O2 |
| K11 | Issue #17 (OPEN): forward delivery is 43% for ~9 s after a reverse leg when v=0 was streamed between them (velocity mode kept the reverse I-term). It matters after a BT BackUp recovery followed by forward motion. | Medium | Confirmed (bags), cause Probable (integrator, fresh_audit N21) | GitHub #17; `docs/fresh_audit_2026_05_29.md:116` | Not in audit |
| K12 | vy in the **local** EKF: nothing observes position, so vy stays at its initial 0. The "lateral constraint" fix (audit O3/E2) matters for the global EKF only. Given K1, the correct base_link vy is −ω·x_c, not 0. | Low | Derived (rl state transition) | `ekf.yaml:58-62` | Refines audit O3/E2 |
| K13 | IMU stamps are Xsens UTC; /wheel_odom and cmd use host time. If the Jetson clock is not disciplined in the field (no internet), the EKF mis-orders IMU against wheel measurements. It was 25.8 ms on 09-25; it is not re-checked per session. | Medium if skewed | Hypothesis | audit 04 §2; `xsens.yaml` (pub_utctime) | **New** check |
| K14 | IMU twist gate 5σ at 1e-9 R: the gate width is ~5·sqrt(Q_vyaw·dt) ≈ 0.27 rad/s per 10 ms step, so gyro spikes on rough grass may be dropped. It self-recovers (P grows). | Low | Hypothesis | `ekf.yaml:52-53,115` | New (minor) |
| K15 | No track desaturation (audit D5). Nav2 at vx 0.7 plus the ω clamp 1.5 at multiplier 1.19 gives 4090 RPM, but the bus-sag ceiling is ~3000 RPM (~1.0 m/s per track). A grass multiplier above 1.19 pushes it further, so one track clips in tight fast turns and curvature distorts. | Medium (grows with K7) | Confirmed (code); sag Probable | `actuator_node.py:461-465`; audit D5 | Audit D5; interaction new |
| K16 | Local EKF config otherwise sane: `two_d_mode: true`, 30 Hz, vx-only wheel, IMU rates, `world_frame: odom`, process noise vx 0.5 / vyaw 0.3 / yaw 0.01. `navsat` consumes `/odometry/global`, not local, so changing local yaw fusion does not affect navsat. The wheel vx covariance (1e-4) is irrelevant to the local mean because vx has a single source. | — | Confirmed | `ekf.yaml`, `localization.launch.py:126,141` | Confirms audit |

## Field tests outdoors today

**Common setup (every test)**
1. Bring up **without Nav2**: `ros2 launch avros_bringup localization.launch.py`, then `actuator.launch.py` and `webui.launch.py`. Leave the web UI connected with **AUTONOMOUS ON**. In that mode the page stops streaming `control` messages, so they cannot fight the CLI; D1 means there is no source priority. The **E-STOP button stays live**. Do **not** close the page mid-test: a disconnect latches an e-stop, which only a non-estop ActuatorCommand clears (toggle E-STOP off in the UI).
2. In every CLI shell: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$HOME/IGVC_ROS2/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml` (without it, CLI publishes on FastDDS).
3. Stop options:
   - Ctrl-C the publisher: the command goes stale in 0.5 s, decelerates at 1.3 m/s², then `S` is sent.
   - Hardware E-stop or the web UI E-STOP.
   - `ros2 topic pub --once /avros/actuator_command avros_msgs/msg/ActuatorCommand '{estop: true}'`.
4. Preconditions:
   - `ros2 topic echo /status --field rtk_status` shows 2 (FIXED) for the whole run. Discard runs that drop to FLOAT.
   - 30 s stationary: IMU yaw drift < 0.2° (stuck-bias check; if it fails, USB power-cycle the Xsens).
   - **Motion warm-up**: drive 5 m out and back by joystick before any recorded run (K4).
5. Record one bag per test:
   ```bash
   ros2 bag record -o T<n>_<surface>_<run> /wheel_odom /imu/data /odometry/filtered /odometry/global /gnss \
     /filter/positionlla /status /avros/actuator_state /avros/wheel_debug /cmd_vel /tf /tf_static
   ```
6. Run the clock check once: `python3 odom_kinematics_analyze.py <bag> --test clock`. Pass: \|median stamp−receive\| < 0.05 s for `/imu/data` and `/wheel_odom` (K13).
7. Publish pattern (bounded duration): `timeout -s INT <sec> ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "<twist>"`. Let each leg go stale (which sends `S`) and wait 3 s before the next leg. Never stream v=0 between reverse and forward (K11), except in T6.

**T1: Straight-line distance scale (K8), grass and pavement if available, 3× each direction**
- Purpose: m_per_motor_rev on the competition surface; heading consistency (IMU yaw vs RTK course); lateral drift with heading-hold.
- Setup: 12 m clear straight lane. Tape the ground at the front edge of the left track at the start.
- Forward: `timeout -s INT 21.5 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"`. This is a 1.7 s ramp at 0.3 m/s² plus ~19.8 s at 0.5, ≈ 10.3 m including the 0.1 m stop.
- Tape the end position at the same track edge and measure with a tape. Then reverse with `x: -0.5` (same duration) back to the start. 3 forward + 3 reverse.
- Also run once at `x: 0.3` for 35 s (slip against speed).
- Analysis: `--test straight`. It gives the wheel distance from encoder revs (lag-free) and from integrated /wheel_odom, EKF path length, RTK displacement (antenna displacement equals base displacement when yaw is constant), scale = wheel/RTK, RTK course − IMU yaw, and maximum lateral deviation.
- Pass:
  - Scale within 1.00 ± 0.01 on pavement and ± 0.02 on grass; std over 3 runs < 0.005.
  - Tape and RTK agree within 3 cm.
  - Course − IMU yaw < 2° and the same sign in both directions (a constant offset is a map-yaw offset for the GNSS scope).
  - Lateral deviation < 0.10 m over 10 m.
  - If the scale is off, the new `m_per_motor_rev = 0.01994 × RTK/wheel` applies to that surface.

**T2: Rotate in place 360° and 720°, both directions, 3× each, on grass (K1, K6, K7)**
- Purpose: re-measure ω delivery (multiplier) on grass; α_meas = IMU/encoder ω; wheel_odom ω error; and the **rotation-centre offset x_c**.
- Commands at the deployed multiplier 1.19:
  - 360° CCW: `timeout -s INT 13.0 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"` (0.42 s slew, then 12.6 s × 0.5).
  - 720°: 25.6 s.
  - CW: `z: -0.5`.
  - Also 360° at `z: 1.0` (6.7 s): delivery against rate.
- Physical check: chalk a line along the left track at the start. After 720°, measure the residual angle with a long straightedge and a phone inclinometer on the ground mark. This catches an IMU scale error, which "field-verified 1.000" claims is absent.
- Mark the ground under the IMU (base_link) and under the track centre at the start, and watch which point stays put. That is a cheap visual check of x_c.
- Analysis: `--test spin`. It reports delivered/commanded (IMU/w_slewed), IMU/encoder (α_meas), wheel_odom/IMU, the antenna circle radius r_a, x_c = 0.76 − r_a, EKF base_link motion (≈0), and the true base_link excursion from RTK.
- Pass:
  - Delivery 0.95–1.05. Otherwise new multiplier = 1.19 / delivery for spins, and note it separately from arcs (T3).
  - wheel_odom/IMU recorded per surface (informational, K6).
  - **x_c < 0.10 m.** Otherwise K1 is confirmed: the true base_link excursion is ≥ 0.2 m while the EKF shows 0.
  - Repeatability: delivery std < 0.02 across 3 runs, and CW and CCW within 0.03 (asymmetry means track friction or motor mismatch).

**T3: Small-ω arcs, heading-hold interference (K2) and arc delivery (K7), 3×**
- Legs, 15 s each, waiting 3 s between legs:
  - A: `{linear: {x: 0.4}, angular: {z: 0.03}}`, inside the deadband. With no override, expect 25.8° of turn.
  - B: `{linear: {x: 0.4}, angular: {z: 0.06}}`, outside the deadband. Expect 51.6°.
  - C: `{linear: {x: 0.4}, angular: {z: 0.3}}`, R = 1.33 m, typical gap-threading curvature. Expect 258°. Use 10 s instead of 15 s if space is short.
  - D (A/B comparison): repeat A with `ros2 param set /actuator_node heading_hold_deadband 0.0` (DYNAMIC; disables heading-hold), then restore to 0.05.
- Analysis: `--test arc`. It gives the lock fraction, lock output sign against w_target, and IMU ω / w_target per |ω| band. Also integrate IMU yaw per leg.
- Pass (D2 refuted): leg A turns ≥ 20° with heading-hold on. **If leg A turns < 5° while leg D turns ≈ 26°, D2 is confirmed**, and heading-hold must be disabled on the /cmd_vel path. Arc delivery (legs B, C) within 0.9–1.1. If it is above 1.05 while spins are at ~1.0, the multiplier over-drives arcs (K7).
- Optional under Nav2 later: a 15 m goal with a 1 m lateral offset. Pass: lock active < 10% of the time while w_target ≠ 0.

**T4: Closed loop with RTK truth for the local EKF (K1, K5, K9), 3×**
- Drive by web UI joystick (AUTONOMOUS OFF during this test only) or by timed CLI legs:
  - (a) 5×5 m square with **pivot turns** at the corners (v=0, ω=±0.5), CCW then CW;
  - (b) figure-8 with ~2 m radius loops at 0.4 m/s;
  - (c) an **L path**: 8 m straight, a 90° pivot, 8 m straight, then stop. The net heading change is 90°, so the K1 error does **not** cancel here.
- Analysis: `--test loop`. It reconstructs RTK base_link = antenna − R(yaw)·[0.76, 0] and compares it with /odometry/filtered after an SE2 fit: ATE RMS/max, the 5 m relative-distance error, and closure against RTK closure.
- Pass:
  - ATE max < 1% of path (< 0.20 m on 20 m); 5 m relative p95 < 0.10 m.
  - L-path end error < 0.10 m. **If it is ≈ 2·x_c·sin 45° (~0.4 m) while the square closes well, K1 is confirmed.**
  - Figure-8 < 0.15 m.

**T5: Repeatability.** Every test above runs 3× per direction and surface. Report mean ± std. A metric passes only if all 3 runs pass and std < ⅓ of the tolerance.

**T6 (optional, 5 min): issue #17 repro in the recovery pattern.** Reverse at `x: -0.25` for 3 s (as BackUp does), then stream `x: 0.0` for 2 s, then forward at `x: 0.4` for 8 s. Compare the forward distance with a clean forward leg at the same speed (`--test straight`). Pass: forward delivery > 90% of the clean leg.

## Analysis script specs (`odom_kinematics_analyze.py`, read-only)

Inputs: one rosbag2 (sqlite3 or mcap). It needs a sourced Humble overlay (rosbag2_py and xsens msgs), and runs on the Jetson after the test or on a laptop with the overlay. It publishes nothing.

| Mode | Inputs | Outputs |
|---|---|---|
| `clock` | header stamps against bag time | per-topic median/p95 offset (K13) |
| `straight` | wheel_debug pos_rev, /wheel_odom, /odometry/filtered, /gnss, /imu/data, /status | per segment: encoder, integrated, EKF and RTK distance, scale, course−yaw, lateral deviation, FIXED fraction |
| `spin` | as above plus w_slewed | commanded, IMU, wheel_odom and encoder angle; delivery; α_meas; antenna circle fit → x_c; EKF against true base_link excursion |
| `arc` | wheel_debug (w_target, w_after_imu, heading_locked), /imu/data | lock fraction, lock-overrides-small-ω fraction, opposite-sign fraction, IMU/w_target per ω band |
| `loop` | /gnss, /imu/data, /odometry/filtered | ATE (SE2-aligned), 5 m relative error, closure against RTK |

Segments are detected automatically from /wheel_odom (|v| > 0.03 m/s, or |ω| > 0.03 rad/s for spins, lasting ≥ 2 s). The antenna offset defaults to `--antenna-x 0.76`; use 0.74 to match `GNSS_LeverArm`. The spin x_c fit assumes the antenna and rotation centre lie on the centreline.

## Proposed config changes (not applied; each gated on the test named)

1. **K1, after T2 shows x_c ≥ 0.10 m.** Move `base_link` to the measured rotation centre: shift every child joint in `avros.urdf.xacro` by −x_c (IMU to `x=-x_c`), and shift both Nav2 `footprint` polygons by −x_c (e.g. with x_c = 0.30: `[[0.513,0.415],[0.513,-0.415],[-0.579,-0.415],[-0.579,0.415]]`).
   - `GNSS_LeverArm` is IMU-relative and stays unchanged; navsat and the EKFs need no change (the IMU angular rate is frame-invariant).
   - Interim option without the URDF change: publish `twist.linear.y = −ω·x_c` in /wheel_odom (code change) and set `odom0_config` vy true with vy variance (0.05)².
   - The URDF route is preferred because it also fixes MPPI's DiffDrive footprint prediction.
2. **K2/K3, if T3 confirms.** For Nav2 runs, set `heading_hold_deadband: 0.0` (DYNAMIC, no code change; heading-hold never engages). The proper fix is a code change: apply heading-hold only on the ActuatorCommand path, add IMU staleness (disable if no IMU for > 0.2 s), and rate-limit its output through the angular slew.
3. **K5, after T2 confirms the gyro scale within 0.5%.** In the local EKF set `imu0_config` yaw → `false` and keep vroll/vpitch/vyaw `true` (roll/pitch are ignored in 2D). Odom heading is then gyro-integrated, continuous per REP-105, and immune to Xsens heading steps. Keep absolute yaw in the map EKF.
4. **K7.** `wheel_separation_multiplier = 1.19 / delivery_grass(spin)` from T2. If T3 arc delivery exceeds 1.05 at that value, pick the value for the typical arc (ω 0.3 at 0.4 m/s) as the compromise, and record both.
5. **K8.** `m_per_motor_rev = 0.01994 × (RTK/encoder)` from T1 on grass, only if the error is > 1.5% and repeatable.
6. **IMU covariances (audit I1).** Measured noise is gyro 0.11°/s and yaw 0.28°. Set the Xsens driver's stddev parameters to about angular velocity 0.004 rad/s and orientation 0.01/0.01/0.02 rad (verify the parameter names in the driver source before setting). Keep the 5σ gates.
7. **Docs.** Fix `ekf.yaml:5-8,13` (vyaw not fused), `actuator_params.yaml:7` and the docstring (/wheel_odom is 20 Hz), and `actuator_params.yaml:61-62` (MPPI ax limits are not read on Humble).

## Sources

- Nav2 MPPI (Humble) `optimizer.cpp` / `motion_models.hpp`, where `predict` copies `cvx[i-1]` into `vx[i]` with no acceleration model: https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/include/nav2_mppi_controller/motion_models.hpp
- robot_localization state estimation docs (2D mode, fusing velocities, covariance): https://docs.ros.org/en/humble/p/robot_localization/ and REP-105 (continuity of the odom frame): https://www.ros.org/reps/rep-0105.html
- `diff_drive_controller` `wheel_separation_multiplier`: https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html
- Mandow et al. 2007, skid-steer experimental kinematics, IROS, doi:10.1109/IROS.2007.4399139. Martínez et al. 2005, tracked ICR kinematics, Robotica, doi:10.1017/S0263574704001067. Pentzer et al. 2014, J. Field Robotics, doi:10.1002/rob.21509.
- WPILib `desaturateWheelSpeeds`: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/kinematics/DifferentialDriveWheelSpeeds.html
- Repo: `docs/skid_steer_kinematics_findings_2026_05_18.md`, `docs/yaw_diag_session_2026_05_28/validation_results.md`, `docs/lidar_rotation_phantom_diagnosis_2026_05_29.md`, `docs/obstacle_stall_rca_2026_05_30.md`, `docs/fresh_audit_2026_05_29.md`, GitHub issue #17; audit `control_stack_analysis_2026_09/02–05, 08, 09`.
