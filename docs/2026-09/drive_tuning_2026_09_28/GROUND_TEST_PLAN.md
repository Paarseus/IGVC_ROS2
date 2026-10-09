# Ground Test Plan: Drive, Odometry and Controller Interface Before MPPI

**Date:** 2026-09-29 · **Status:** plan (nothing run on the ground yet) · **Replaces:** `GROUND_ACCEPTANCE.md`

**Purpose.** Before MPPI is tuned, prove on the ground that:
- every layer under MPPI does what MPPI assumes;
- every tunable value on that path was measured, not guessed.

**Test order.** The tests go bottom-up and change one thing at a time:
- Scopes 1–5 command the Teensy directly. ROS 2 runs only for the sensors (IMU, GNSS).
- Scope 6 adds `actuator_node`.
- Scopes 7–8 check what MPPI receives and what it assumes.

**Evidence base.**
- Research topics `research/topics/` C1–C4 and L1–L5 (verified); X1 and X2 (draft).
- Four reviews made for this plan (§9): the control research, the localization research, a full parameter audit of the code, and an outside-literature review.
- Bench results: `results/ACCEPTANCE_2026_09_28.md`, `results/FW26_BENCH_2026_09_28.md`.

---

## 0. Fix or decide before the first ground session

These were found while building this plan. **None of them has been changed.** Each needs your approval.

| # | Finding | Evidence | Proposed action |
|---|---|---|---|
| **F1** | **MPPI probably receives no odometry.** `controller_server` has no `odom_topic`, so it listens to `odom`, which nothing publishes. The EKF publishes `/odometry/filtered`. MPPI then starts every prediction from zero speed. | `nav2_params_igvc_autonav.yaml:73-82` (no `odom_topic`; set only for bt_navigator :43 and the smoother :658). Humble `OdomSubscriber` default `"odom"` (`research/topics/C4…/sources/nav2_2026_humble_odom_subscriber.hpp:64-69`). | Check on the robot: `ros2 param get /controller_server odom_topic` and `ros2 topic info /odom`. If confirmed, add `odom_topic: /odometry/filtered` under `controller_server`. |
| **F2** | MPPI may plan turns up to 1.9 rad/s. `actuator_node` silently cuts them to 1.5. | `nav2_params_igvc_autonav.yaml:134`, `actuator_params.yaml:36` | Set `wz_max` to at most the measured turn limit (test 8.3). |
| **F3** | Heading-hold is **on** in navigation. Any MPPI turn command below 0.05 rad/s is replaced by a hidden heading loop. | `actuator_node.py:459-468`; `actuator_params.yaml:68` | Keep it off (`heading_hold_deadband: 0.0`) for all tests. Decide in the MPPI phase with an on/off comparison. |
| **F4** | The GNSS antenna offset (about 0.76 m forward of the IMU) is not in the ROS pipeline. `/gnss` is published in `imu_link`, and navsat does not correct it. | `xsens.yaml:47` (device only); URDF has no antenna link | Add the antenna frame. Test 7.6 verifies it. |
| **F5** | The IMU publishes zero covariance, so the EKF trusts it completely. Its 5σ rejection gates therefore cannot reject a bad reading correctly. | `xsens.yaml` (no `*_stddev`); `ekf.yaml:52-53` | Set measured values (test 7.4). |
| **F6** | Stale statements in the docs: CLAUDE.md gives the turn cap as 1.0 (it is 1.5), says both EKFs fuse wheel yaw rate (vx only), and says `/wheel_odom` runs at 50 Hz (it is 20 Hz). There are also out-of-date comments in `ekf.yaml:13`, `actuator_params.yaml:61`, and the `vx_max` comment in the nav2 yaml. | parameter audit | Correct the text only. |
| F7 | If the yaml fails to load, `actuator_node`'s fallback values are firmware-25 gains (kFF 0.000197, about 12× too weak on firmware 26) and multiplier 1.0. | `actuator_node.py:142-216` | Low priority: update the defaults. |
| F8 | The documented rule "web UI command beats `/cmd_vel`" is not in the code. The latest message wins. | `actuator_node.py:337-356` | Never run the web UI and Nav2 together. Fix it later. |

---

## 1. Common rules (all scopes)

| Rule | Detail | Source |
|---|---|---|
| Surface | Asphalt first (the AutoNav course surface), then grass. Values are per surface: a floor calibration on grass gave about 6× the error. | C2 p8, C2-S33 |
| Robot configuration | Record battery position, payload and track tension at the start of each session. A change to any of them repeats Scopes 2–4. | X2 p4; C2 (CoM dependence) |
| Heading-hold | Off (`heading_hold_deadband 0.0`) in every scope. | F3; C3 p1 |
| Repeats | Measured values: **5 per condition and direction**. Report mean ± 2·s/√5. Pass/fail safety items: **10 of 10**, which is 80 % reliability at 85 % confidence. | UMBmark (5+5); NIST/ASTM E54 (0/10); X2-S11 (3 repeats resolve only large effects) |
| Calibrate, then validate | Fit on repeats 1–3 and pass or fail on repeats 4–5. Never accept on the data used to fit. | C1 p9; C2 p5; X2 |
| Reference accuracy | Each reference must be at least 10× better than the limit it checks (Scope 0). | X2 p2; NIST AGV |
| No instant steps | Ramp every speed change. Instant jumps caused gate-driver faults 3 of 3 times. Use the Teensy ramp `M` at 100–300, never ≥ 1000. | FW26_BENCH T4 |
| Logging | For every run, record the raw bag (`/cmd_vel`, `/wheel_odom`, `/imu/data`, `/gnss`, `/odometry/filtered`, `/avros/wheel_debug`), the Teensy X-line log, bus voltage at start and end, surface and air temperature. | ASTM F3218/F3327; X2 |
| Randomise | Randomise the run order inside each test so that battery drain and warm-up do not line up with one condition. | NIST DOE (X2-S13) |

### 1.1 Which reference each test uses (RTK is not required for Sessions 1–2)

Measure on the ground with a tape and chalk wherever possible. Use RTK only where a test needs continuous position or heading in the world.

**Robot reference marks.**
- Fix a downward pointer or plumb line at the `base_link` point, and a second one about 1 m ahead of it on the robot's centre line.
- Every tape measurement is taken from these two points, marked on the ground with chalk.

| Reference | Accuracy | Enough for |
|---|---|---|
| Steel tape + chalk marks from the pointers | about ±5 mm over 20 m (tape ±4 mm, each mark ±3 mm) = **0.03 %** | distance per motor turn (1 % limit: 30× margin), stops, square return error, spin centre |
| Chalk heading lines (two points 1 m apart along the centre line, before and after) | ±0.3° per line; over 5 turns (1800°) = **0.02 %** | gyro scale (0.3 % limit), heading change on straights |
| Encoder position | exact for motor speed (no slip in the measurement) | everything in Scopes 2–3 (motor speed is what is tested there) |
| Gyro, after 0.2 and 0.3 pass | bias ≤ 0.02 °/s; scale ≤ 0.3 % | turn rate in spins and arcs (3 % limit: 10× margin) |
| RTK FIXED | about 1–2 cm per fix, 10 Hz or slower; wrong fixes possible; antenna 0.76 m ahead of `base_link` | cross-check in Sessions 1–2; **required** only where marked below |

| Test | Main reference | RTK |
|---|---|---|
| 0.2 IMU bias, 2.x motor fit, 3.x speed loop, 6.5 latency, 7.1–7.2 odometry rate and lag | encoder, gyro, clocks | not needed |
| 0.3 gyro scale | chalk heading lines + counted turns | optional cross-check |
| 4.1 distance per motor turn | tape between start and end marks (straight distance), plus the sideways offset at the end | optional cross-check |
| 4.2 left/right match | chalk heading lines at start and end (and the gyro) | not needed |
| 4.3–4.4 spins and arcs (turn multiplier) | gyro (checked in 0.3) | optional: driven radius of arcs |
| 4.5 spin centre | chalk mark of the `base_link` pointer at 0° and after half a turn: half the distance between them is the offset of the spin centre | optional: circle fit |
| 4.6 square | tape from the start marks to the end marks (as in UMBmark) | not needed |
| 5.1, 6.2 stopping distance | encoder distance; tape check on some runs | not needed |
| 0.4 RTK integrity, 7.5 heading vs world, 7.6 antenna offset, map-frame checks | RTK | **required** |
| 7.3–7.4 odometry noise and covariance | tape-timed runs or RTK speed | preferred |
| 8.1–8.2 MPPI model: predicted vs real 2.8 s path | RTK path | **required** (or lidar-based localization) |

---

## 2. Scopes and tests

**How to read the tables.**
- "Sets" is the parameter the test decides.
- "Current" is today's value.
- Pass limits marked *(ours)* were derived in this plan; the others come from the cited source.

### Scope 0: References (no driving). Run at the start of every session.
Every later result depends on these. Nothing else is judged until they pass.

| ID | Test | Method | Pass | Why |
|---|---|---|---|---|
| 0.1 | Boot check | `bench.py CHK` | `CHK OK` | Motor type, idle mode, gains and filter are as saved |
| 0.2 | IMU warm-up and bias | Power on. Wait **≥ 10 min**, then take 3 min stationary. Repeat after the first drive. | Gyro-z bias **≤ 0.02 °/s** *(ours)*; datasheet stability 0.002 °/s; a healthy unit measured 0.0008 °/s | The current gate of 0.1 °/s allows 6°/min of heading drift. Research asks for 5–10 min of warm-up (L2-S38). |
| 0.3 | Gyro scale | About 5 slow turns each way. Reference: counted turns plus the angle between chalk heading lines drawn before and after (§1.1, about 0.02 %). With RTK FIXED, the antenna angle around the spin centre is a cross-check. | Scale error **≤ 0.3 %** (10× better than the ±3 % turn limit) | The "1.000 field-verified" in `ekf.yaml:61` has no record behind it. The datasheet allows 0.5–1.5 %. |
| 0.4 | RTK integrity | 2 min static FIXED. Re-occupy a marked point in each session. Log the correction age and the `/rtcm` rate. | 100 % FIXED; static σ ≤ 2 cm; re-occupation within 3 cm; correction age ≤ 2 s; no `/rtcm` gap > 5 s | GGA quality 4 alone does not prove a correct fix (L3 §9). The NTRIP client cannot detect a silent stream (L3). |
| 0.5 | Clocks | `chronyc tracking` in the field. Histogram received time − stamp for `/imu/data`, `/gnss`, `/wheel_odom`. | Offset < 5 ms *(ours: 10 % of the 50 ms odometry budget)* | The Xsens stamps with its own UTC clock and `/wheel_odom` with the Jetson clock (L5) |
| 0.6 | USB serial buffering | `cat /sys/bus/usb-serial/devices/ttyUSB*/latency_timer`; `tools/ground/imu_timing.py 20` | 1 ms; IMU interval sd < 2 ms | The FTDI default of 16 ms adds lag to IMU data (L5-S39). **Fixed 2026-09-30** by a udev rule: `docs/imu_usb_latency_2026_09_30.md` |
| 0.7 | IMU timestamps | compare the `/imu/data` stamp with `date +%s` | within 0.1 s | Before a GPS fix the stamps are Xsens power-on time (same doc) |

### Scope 1: Safety stops (motor layer; ROS only for sensors)
| ID | Test | Pass | Source |
|---|---|---|---|
| 1.1 | Hardware e-stop from 0.5 and 1.0 m/s | Motion stops. Record the stopping distance. **10/10.** | IGVC qualification; C3 p8–9 |
| 1.2 | Teensy watchdog: stop sending while driving at 0.5 m/s | Stops at 250–350 ms. 10/10. | FW `WATCHDOG_MS 300` |
| 1.3 | Braked stop (`S`) from 0.7 m/s | No backward motion, no fault. 10/10. | FW26_BENCH T4 |

### Scope 2: Motor model per track (motor layer)
Sets **kV (`kFF`)** and **kS per track** (`kS_left`, `kS_right`), which today are 0.0023 V/RPM and 0.18 V on both tracks.

| ID | Test | Method | Pass | Source |
|---|---|---|---|---|
| 2.1 | Slow voltage ramp | 0 → 7 V at 0.5 V/s, forward and reverse, each track logged separately. Over ≥ 6 m of ground. | — | C1 p9, p15; SysId (C1-S18) |
| 2.2 | Voltage steps | 2 → 4 V and 4 → 6 V, forward and reverse. These are small steps, not SysId's default 7 V, because of the fault finding. | — | C1 p16; FW26_BENCH T4 |
| 2.3 | Fit | V = kS·sign(v) + kV·v (+ kA·a) per track and per direction. Fit on repeats 1–3; validate on 4–5. | Fit r² > 0.9 on validation data | C1 p16, C1-S20 |
| 2.4 | Direction difference | Compare forward kS with reverse kS | If they differ by > 20 % *(ours)*, report it. The Teensy has one kS per track, not per direction. | C1 common mistakes (C1-S32: 36 % method spread) |

**Decisions:**
- Change `kFF` if the fitted kV differs from 0.0023 by > 5 %.
- Set `kS_left` and `kS_right` separately: the right track has about 8 % more friction.

### Scope 3: Speed loop on the ground (motor layer, final gains)
Confirms **kP 0.0002, kI 0** under load. Sets the current limit if needed.

| ID | Test | Method | Pass | Source |
|---|---|---|---|---|
| 3.1 | Steady delivery per track | Hold 150, 300, 900, 1500, 2100, 3000 RPM, forward and reverse. Speed taken from position, not the SPARK's speed report. | Error **≤ 2 %** at ≥ 300 RPM; **≤ 10 %** at 150 RPM | MPPI assumes the mean command is delivered (C4-S01). A fixed gain error leaves a permanent offset (C4-S05, C4-S52). |
| 3.2 | Crawl | 150 RPM (0.05 m/s) and the inner track of a 0.1 rad/s spin, 10 s each | Never stuck (no 100 ms window at zero); variation ≤ 20 % *(ours)* | Bench A6; T7 |
| 3.3 | Ramped step response | 900 → 2100 RPM over 0.25 s, and back | Settles within 0.4 s after the ramp ends; overshoot ≤ 5 %; delay to first motion ≤ 100 ms | STRATEGY T3, T4, T8; X2 p7 |
| 3.4 | Load disturbance | Hold 0.5 m/s straight onto the ≤ 15 % ramp | Speed dip ≤ 10 %; back within 1 s *(ours)* | X2 p7 |
| 3.5 | Top-speed headroom | Hold 4600 RPM (the clamp) on the surface | Reached with power < 0.95 of maximum | Only 11 % headroom off the ground (T3.6) |
| 3.6 | Current in spins | Log current during Scope 4 spins, on grass | Peak < 50 A limit, or the limit is raised with a reason | Limit set from the REV range, never measured (parameter audit) |

**Decision:** if 3.1 fails on one surface only, the choice is between per-surface kV/kS and a small kI with iZone (C1 p5, p18). With I = 0 and this P, the steady error is about half the feedforward error.

### Scope 4: Chassis kinematics (motor layer, host-computed track speeds)
Sets **`m_per_motor_rev`** (0.01994), **`wheel_separation_multiplier`** (1.19) and the spin centre.

| ID | Test | Method | Pass | Source |
|---|---|---|---|---|
| 4.1 | Distance scale | 20 m straights at 0.3 and 0.7 m/s, both directions. Tape between the chalk marks of the `base_link` pointer at start and end, and the sideways offset at the end (§1.1, about 0.03 %). RTK averaged 10 s at each end is a cross-check. | Scale error **≤ 1 %** | UMBmark E_s first (Borenstein 1996); C2 p3–4; 20 m legs, not 10 m, so the reference is 10× better |
| 4.2 | Left/right match | The same straights, open loop. Heading change per metre from the gyro. | Heading drift **≤ 0.65 °/m**, which equals ≤ 1 % left/right mismatch *(ours: 0.01/0.877 m)* | Replaces the old "5 cm end offset", which needed a 0.1 % match and could not be measured |
| 4.3 | Spin calibration | Spins at 0.1, 0.3, 0.6, 1.0 rad/s, CW and CCW, 5 turns counted on a ground mark | Turn rate **±3 %** of the command after calibration | MPPI turn-error budget (plan G2); Mandow 2007 |
| 4.4 | Arcs | Radius 1.52 m (the IGVC minimum) and 3 m, at 0.45 and 0.7 m/s, both directions | Multiplier from arcs within 3 % of the spin value, otherwise a radius-dependent value is needed | C2 p9 (varies with radius, speed and acceleration) |
| 4.5 | Spin centre | Mark the `base_link` pointer on the ground, spin half a turn slowly, mark it again: the spin centre is half that distance from `base_link` (0 = turns about base_link; about 0.31 m = about the track centre). RTK circle fit of the antenna as a cross-check. | Report the offset | MPPI and the footprint assume rotation about base_link. The URDF puts the tracks 0.31 m ahead of it. |
| 4.6 | Bi-directional square (UMBmark) | 4 × 4 m, 5 CW + 5 CCW, slow | Systematic return error ≤ 3× its standard error; otherwise recalibrate | Borenstein & Feng 1996; X2 p6 |

**Expected result:** the multiplier falls from 1.19 toward about 1.0–1.05. 1.19 was measured when the motors delivered only about 85 % of commanded speed; they now deliver about 100 %. This is a hypothesis for the test to settle.

### Scope 5: Stopping distance (motor layer)
| ID | Test | Pass | Source |
|---|---|---|---|
| 5.1 | From 0.5, 0.7, 1.0 m/s: braked stop (`S`) and watchdog stop. 5 each per surface. | Report mean + 3σ. No backward motion. < 1 cm creep after 2 s. | ISO 18646-1 (stopping distance); ISO 9283 (mean + 3σ); C3 p15 |

### Scope 6: The ROS 2 command path (`/cmd_vel` → `actuator_node` → motors)
Starts `actuator_node`. Everything below it is now fixed by Scopes 2–5.

| ID | Test | Method | Pass | Source |
|---|---|---|---|---|
| 6.1 | Delivery through the node | `/cmd_vel` 0.05, 0.1, 0.3, 0.5, 0.7, 1.0 m/s and spins | Same limits as 3.1 and 4.3. Any extra error is in the node. | C4 p2 |
| 6.2 | Stops through the node | 0.7 → 0 command, and `/cmd_vel` going silent (0.5 s timeout) | **≤ 0.30 m** from 0.7 m/s; no backward motion; 10/10 | Plan G9 (0.7²/(2·1.3) + 0.7·0.1) |
| 6.3 | Turn reversals at speed | ω +0.46 ↔ −0.46 rad/s at v = 0.4 | No hunting around zero > ±30 RPM | IGVC course is mostly S-curves |
| 6.4 | Combined slow-down and turn | Decel while turning | Log whether the Teensy ramp (1.66 m/s² per track) binds; it binds above that rate | Parameter audit: needs up to 1.83 m/s² |
| 6.5 | Latency | Time `/cmd_vel` → track motion, 200 steps at random phase | Command-to-motion ≤ 100 ms; report p95 | C3 p10, p13; STRATEGY T8 |
| 6.6 | Curvature at the limits | v + ω near the caps | Log whether one track clamps at 4600 RPM (curvature changes silently). Fine at `vx_max` 0.7. | C1-S51; Clearpath `preserve_turning_radius` |

### Scope 7: Odometry and localization (what MPPI and Nav2 receive)
| ID | Test | Method | Pass | Source |
|---|---|---|---|---|
| 7.1 | `/wheel_odom` rate and gaps | full stack running | ≥ 20 Hz; no gap > 100 ms | C4-S27 |
| 7.2 | Odometry lag | Cross-correlate `/wheel_odom` speed with speed from encoder position; include the EKF | Total ≤ 50 ms. **Expected to fail:** estimated 75–130 ms (filter 50 + 20 Hz timer + EKF). | C4-S26 (seeds every prediction); L1 §5 |
| 7.3 | Noise and standstill | Steady windows; 60 s stopped after a drive | σ_v ≤ 0.03 m/s, σ_ω ≤ 0.05 rad/s; stopped \|v\|, \|ω\| < 0.01 in 99 % of samples | MPPI threshold 0.01; plan G8 |
| 7.4 | Covariances | Measured variance ÷ declared variance for wheel vx and IMU | 0.5–2 | L1 §10; L4 §5; Moore (robot_localization docs) |
| 7.5 | Heading | (a) 20 m straights N/E/S/W: IMU yaw vs RTK course. (b) 10 m forward then 10 m reverse: look for heading jumps. (c) odom yaw continuity. | (a) offset < 1°, spread < 1°; (b) no jump > 1°; (c) no steps | L2-S15, L2-S33; REP-105 |
| 7.6 | Antenna offset | After F4, repeat the 4.3 spin | `/odometry/gps` stays within 5 cm of a fixed point during the spin | L3 p10 |
| 7.7 | EKF rate under load | Full stack with perception on | ≥ 20 Hz, no gap > 100 ms (measured 27 Hz earlier) | L4 |
| 7.8 | Rejection gates | Replay bags with an injected 5 m GPS jump and a slow IMU bias ramp | The jump is rejected; the bias is caught | L4 §6 |

**Decisions after 7.2:**
- If the lag fails, compute `/wheel_odom` speed from encoder position and publish at 50 Hz. The position data is already received and unused.
- This is a code change, and it needs your approval.

### Scope 8: The model MPPI assumes (MPPI running, costs not tuned)
Humble MPPI assumes each command is reached within one 0.05 s step, with no lag and no acceleration limit. Its `ax_max`/`az_max` are not read on Humble (C4-S26).

| ID | Test | Method | Pass | Source |
|---|---|---|---|---|
| 8.0 | MPPI reads odometry | Confirm F1 is fixed | `controller_server` subscribed to `/odometry/filtered` | F1 |
| 8.1 | Random command holds | Random (v, ω) over the reachable range, 6 s holds (2 s transient + 2 × 2 s steady) | Delay + time constant per axis fitted; fit r² > 0.9 on held-out data | DRIVE (Baril 2024); C4-S21 |
| 8.2 | Prediction error over MPPI's horizon | Replay MPPI-like fast random commands. Compare the real 2.8 s path with MPPI's model: (a) as MPPI assumes, (b) with the slews, (c) with slews + lag. | (a) p95 lateral error **≤ 0.10 m** on R ≥ 1.52 m at 0.7 m/s; effective delay ≤ 0.28 s (goal 0.14 s) | Plan §1.3 budget; Seegmiller 2013 (judge by multi-second prediction) |
| 8.3 | Reachable limits | Largest v and ω actually reached per surface | `vx_max` ≤ reached v; `wz_max` ≤ min(1.5, reached ω) | C4 p3; DRIVE step 1 |

**Decision after 8.2.** The slews are expected to be the largest mismatch:
- accelerating 0 → 0.7 m/s takes 2.3 s, up to 0.82 m of along-track error from standstill;
- reaching the IGVC turn rate takes 0.38 s.

If (a) fails and (b) passes, choose between faster slews (after the 12 V rail fix) and a newer Nav2 that models acceleration.

---

## 3. Parameters: what each test decides

| Parameter | Where | Current | Measured by | Changes if |
|---|---|---|---|---|
| kV (`kFF`) | actuator_params.yaml | 0.0023 V/RPM | 2.3 | fit differs > 5 % |
| kS left / right | actuator_params.yaml | 0.18 / 0.18 V | 2.3, 3.2 | fit per track |
| kP / kI | actuator_params.yaml | 0.0002 / 0 | 3.1, 3.3 | 3.1 fails on a surface |
| current limit | SPARK (saved) | 50 A | 3.6 | spins reach the limit |
| `m_per_motor_rev` | actuator_params.yaml | 0.01994 | 4.1 | error > 1 % |
| `wheel_separation_multiplier` | actuator_params.yaml | 1.19 | 4.3, 4.4 | error > 3 % (expected) |
| `track_width_m` | actuator_params.yaml | 0.7366 (URDF: 0.7306) | absorbed by 4.3 | — |
| spin centre / footprint origin | URDF, nav2 footprint | base_link assumed | 4.5 | offset > 0.1 m *(ours)* |
| decel, accel, turn slews | actuator_params.yaml | 1.3 / 0.3 / 1.2 | 5.1, 6.2, 8.2 | stop > 0.30 m, or model error |
| Teensy ramp `M` | firmware default | 100 RPM/tick | 6.4 | it binds in normal driving |
| `/wheel_odom` rate and method | actuator_node | 20 Hz, SPARK speed | 7.1, 7.2 | lag > 50 ms |
| wheel-odom and IMU covariances | actuator_node, xsens.yaml | 1e-4 / zero | 7.4 | ratio outside 0.5–2 |
| GPS antenna frame | URDF / navsat | missing | 7.6 | F4 |
| `odom_topic` (controller_server) | nav2 yaml | missing | 8.0 | F1 |
| `vx_max`, `wz_max` | nav2 yaml | 0.7, 1.9 | 8.3 | above measured / 1.5 |
| heading-hold | actuator_params.yaml | on (0.05) | MPPI phase A/B | not better with it on |

---

## 4. Sessions

| Session | Surface | Content | Time |
|---|---|---|---|
| 1 | asphalt | Scope 0, 1, 2, 3.1–3.3, 3.5 | about 3 h |
| 2 | asphalt | Scope 4, 5, 3.4 (ramp), 3.6 | about 3 h |
| 3 | asphalt | Scope 6, 7 (after F1, F4, F5 are decided) | about 3 h |
| 4 | asphalt | Scope 8; then the MPPI phase | about 2 h |
| 5 | grass | Repeat 2.3, 3.1–3.2, 4.1–4.4, 5.1, 8.3 | about 3 h |

**Gate to start tuning MPPI:**
- Scopes 0–7 pass on asphalt.
- 8.0 passes.
- 8.2 passes, or a slew decision has been made and 8.2 repeated.

---

## 5. Tools

| Need | Tool | Status |
|---|---|---|
| Voltage ramps and steps, speed holds, v/ω sequences | `tools/drive_tuner.py ramp / steps / vel` | exists |
| kS/kV fit, lag, steps, distance, turn | `tools/analyze.py ff / lag / steps / distance / turn` | exists. Add validation on held-out repeats and heading drift per metre. |
| Pipeline stops | `tools/bench/actuator_stop_test.py` | exists. Add distance from GNSS. |
| Gyro scale, spin-circle fit, stationary checks | `analyze.py gyroscale / circle / still` | **done** (tested on synthetic data) |
| Session record: config snapshot, journal, ROS bags, field checklist | `tools/ground/` (`session_start.sh`, `config_snapshot.py`, `bag.sh`, `README.md`) | **done**; not yet run on the Jetson |
| Odometry rate, lag, noise, covariance | new `odom_quality.py` | to write |
| Random command holds, horizon replay | new `random_cmd.py`, `fopdt_fit.py` | to write |

---

## 6. How this plan compares with the research and the previous plans

| Topic | Research says | Previous plans | This plan |
|---|---|---|---|
| Repeats | 3 repeats resolve only large effects (43/11/5 runs for 0.5/1/1.5σ); 10/10 for 80 % reliability; UMBmark 5+5 | 3 repeats; 3/3 for the final run | 5 per condition; 10/10 for safety and end-to-end |
| Validation data | Validate on data not used for fitting | Calibrate and accept on the same runs | Fit on 1–3, judge on 4–5 |
| Reference accuracy | 10× better than the limit | 10 m RTK chords (about 3.5×); gyro with a 0.1 °/s bias and unverified scale | 20 m legs with averaged ends (14×); counted turns; 0.02 °/s bias gate |
| End offset over 10 m | — | ≤ 5 cm (needs 0.1 % left/right match; not measurable) | Heading drift per metre, tied to the 1 % match target |
| Step tests | SysId default 7 V steps | Instant steps with `M ≥ 1000` | Small ramped steps; instant steps faulted 3/3 |
| Turn calibration | Spins and arcs, both directions, per surface, radius-dependent | Arcs 1/2/4 m | IGVC radii 1.52/3 m plus spins; spin centre measured |
| Multiplier on odometry | ros2_controllers applies it to commands and odometry | Commands only (our choice) | Unchanged: the EKF uses wheel speed only, not wheel turn rate, so the choice does not affect the EKF |
| kS per direction | Identify friction per direction | Not tested | Tested (2.4); per track in firmware |
| Odometry into MPPI | High rate, low lag; MPPI seeds every prediction from it | Dropped from the ground plan | Scope 7 + F1 (MPPI may not be reading it at all) |
| IMU and GNSS | Warm-up 5–10 min; covariances measured; antenna offset corrected; wrong-fix checks | Only "RTK FIXED" and "0.1 °/s bias" | Scope 0 + 7.4–7.8 |
| Acceleration limits | The last limiter should rarely act; controller limits ≥ downstream | Actuator slews; MPPI's limits unread on Humble | Measured as model error (8.2) before deciding |
| Heading-hold | A hidden loop between MPPI and the motors risks fighting (4–10× separation rule) | On by default | Off; decided by A/B in the MPPI phase |
| Motor temperature ≤ 60 °C | No source | Pass limit | Removed; log the temperature trend instead |

**Where the research could not help** (open questions, measured here instead):
- tracked-vehicle slip on grass;
- whether one multiplier fits all radii;
- the Xsens heading when reversing;
- a latency budget for MPPI on a real robot.

---

## 7. What is already proven (not repeated)
Bench acceptance 16/16 covers:
- configuration and persistence;
- CAN timing;
- watchdog;
- input limits;
- off-ground speed accuracy, low speed and stops;
- supply sag on ramped starts;
- the `/cmd_vel` path timing.

See `results/ACCEPTANCE_2026_09_28.md`.

## 8. Records
Raw logs go to `~/drive_tuning_2026_09_28/runs/`. Results go to `results/GROUND_<date>.md`, as a table with these columns:
- test;
- measured, with its uncertainty;
- limit;
- pass/fail;
- the parameter changed, if any.

## 9. Sources behind this plan
- Research topics: `research/topics/C1`–`C4`, `L1`–`L5` (verified), `X1`, `X2` (draft).
- Earlier plans: `MPPI_READINESS_TEST_PLAN.md` (targets and budgets; its firmware-25 values are out of date) and `GROUND_ACCEPTANCE.md` (replaced).
- Outside literature:
  - Borenstein & Feng 1996 (UMBmark).
  - Martínez et al. 2005 and Mandow et al. 2007 (tracked and skid-steer kinematics).
  - Baril et al. 2024 (DRIVE).
  - Seegmiller et al. 2013 (multi-second prediction error).
  - Nav2 MPPI README; robot_localization docs.
  - ISO 18646-1, ISO 9283; NIST/ASTM E54 and F45 test methods.
