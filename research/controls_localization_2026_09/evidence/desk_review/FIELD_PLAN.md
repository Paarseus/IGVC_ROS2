# Field Test Plan: 2026-09-27

**Goal:** by the end of this session, have a measured number, with pass/fail against the target below, for every subsystem:
- motors / PID
- kinematics / odometry
- IMU
- RTK / GNSS
- local and global localization
- Nav2 fine motion

**Order:** the phases go bottom-up, so each layer is measured on a layer already known good. Motor gains are decided before any odometry is measured, and odometry before Nav2.

Test IDs refer to the scope reports: M = [01 motor](01_motor_pid_and_ramp.md), G = [02 RTK/global](02_rtk_and_global_localization.md), K = [03 odometry/kinematics](03_odometry_kinematics_local_ekf.md), N = [04 Nav2 fine motion](04_fine_motion_tight_obstacles.md). Each report has the exact commands and full criteria.

## Before going out

**Equipment:**
- 30 m tape measure
- Chalk or marking paint
- 6+ barrels or tall cones: LiDAR marks obstacles only between 0.4 and 1.0 m height, so short cones are invisible
- Stakes or tape for marks
- Laptop with Foxglove
- Phone with the web UI

**Area:**
- Open sky, for RTK FIXED
- A straight 15 m lane
- A 10 × 25 m open area for the Nav2 course
- Grass, and pavement if available (competition is asphalt)

**Roles:**
- **Safety:** holds the E-STOP (phone web UI) at all times, and calls stop.
- **Measurer:** tape and chalk marks, video of gap passes.
- **Operator (Claude, via SSH):** starts bring-ups, sends every `/cmd_vel` leg and goal, records bags, and runs the analysis between tests.

**Rules for every run:**
- Web UI connected and in **AUTO ON**: the E-STOP stays live and the CLI isn't fought.
- RTK-referenced numbers count only while `rtk_status == 2`.
- One bag per run, named `<test>_<surface>_<run>`, for example `K1_grass_2`.
- Every motion test runs **3×** (both directions where applicable). A metric passes only if all 3 pass and std < ⅓ of the tolerance.
- No RViz on the Jetson.

## Phase 0: setup and static health (20 min)

**Bring-up A:**
- `webui.launch.py` (actuator + web UI), already running
- `localization.launch.py` (sensors + EKFs + navsat)

| Measure | How | Target |
|---|---|---|
| Time to RTK FIXED; FIXED σ and max radius; FIXED→FLOAT drops; re-fix step | G-A, static 10 min, `rtk_field_analyze.py static` | < 3 min; σ ≤ 1.5 cm, max ≤ 4 cm; ≥ 95% FIXED after first fix; step < 10 cm |
| IMU stationary yaw drift | 30 s static | < 0.2° (else USB power-cycle the Xsens) |
| Sensor clock skew (`/imu/data`, `/wheel_odom`) | `odom_kinematics_analyze.py --test clock` | < 0.05 s |
| Bus voltage at idle | `/avros/actuator_state` | ≥ 12.0 V |
| Antenna-to-IMU lever arm | Plumb marks under IMU and antenna, tape | Record (config 0.74 m; audit measured 0.764 m) |
| IMU motion warm-up | Joystick 5 m out and back | Done before any recorded run |

## Phase 1: motors / PID baseline, current gains (35 min)

Heading-hold **off** (`heading_hold_deadband 0.0`). Log every run with `motor_step_log.py record`.

| Test | Measures | Target |
|---|---|---|
| M-T2 low-speed steps 0.1 / 0.2 / 0.3 / 0.4 m/s | Delivery at 1 s and at segment end; overshoot; t95; steady-state CoV | ≥ 95% at 1 s, ≥ 98% at end; overshoot < 8%; t95 < 0.6 s; CoV < 3% |
| N-T7 creep 0.05–0.20 m/s, pivots ω 0.1 / 0.2 | Start delay, delivery at low speed, stall threshold | Start < 0.5 s; ±10% after 1.5 s for v ≥ 0.10; turns at ω = 0.1 |
| M-T4 stops: normal, E-STOP, idle push | Stop distance, reverse kick, drift, buzz, Jetson survives | ≤ 0.10 m from 0.4 m/s; reverse < 5 mm (normal) / < 10 mm (E-STOP); no reboot |
| M-T5 L/R matching, 10 m straight | Track delivery mismatch, yaw drift | < 2%; < 3° per 10 m |
| M-T1 feedforward-only (kP = kI = 0; kFF 0.0005 → 0.0021) | Delivery vs kFF (decides the unit error) | Delivery at 0.0021 ≥ 3× delivery at 0.0005 → confirmed |

**Result:** baseline motor numbers, plus a yes/no on the feedforward unit error.

## Phase 2: PID re-tune (30 min; only if M-T1 confirms)

All changes are live via `ros2 param set /actuator_node …`, with no restart. Follow the procedure in the M report:
1. **kFF:** pick the value where the left track reaches 97–100% at 0.4 m/s (expect about 0.0021).
2. **kI / kIZone:** about 1e-7 and 200.
3. **kP:** try 0.0005, then 0.0004; keep the lowest value that passes M-T2.
4. **Re-run M-T2, N-T7, M-T4 and M-T5** with the new gains. Their pass criteria must now hold.

**Result:** the final gain set, with before/after numbers for every motor metric. The gains are written to `actuator_params.yaml` only after review; nothing is saved to files during the session.

## Phase 3: kinematics and odometry, final gains (45 min)

| Test | Measures | Target |
|---|---|---|
| K-T1 + G-C: 10 m straight, both directions, grass (and pavement) | Distance scale (odom vs RTK vs tape); lateral drift; IMU heading vs GNSS course; antenna offset `a`; GNSS lag τ | Scale 1.00 ± 0.02 grass (± 0.01 pavement); tape vs RTK ≤ 3 cm; drift < 0.10 m; heading offset < 0.5°, same both ways; record `a` (expect 0.74) and τ (expect 70–100 ms) |
| K-T2 + G-D + N-T1: 360° / 720° spins, CW and CCW, chalk under IMU | Delivered vs commanded rotation (α on grass); true spin centre `x_c`; IMU vs wheel ω; CW/CCW asymmetry | Delivery 0.95–1.05 (else new multiplier = 1.19 / delivery); **x_c < 0.10 m, else the base_link offset is confirmed**; CW vs CCW within 0.03 |
| M-T3 + K-T3: slow pivots and small-ω arcs (0.03 / 0.06 / 0.3), heading-hold on vs off | Stick-slip; arc delivery; whether heading-hold suppresses small turns | Yaw-rate CoV < 10%; IMU ω / cmd 0.95–1.05; arc 0.9–1.1; **leg at ω 0.03 turns < 5° with hold on vs ≈ 26° off → heading-hold interference confirmed** |
| Verify the new multiplier (if changed): one 360° spin each way | Delivery with the new value | 0.98–1.02 |

**Result:** grass distance scale, grass skid multiplier, the true spin-centre offset, and the heading-hold verdict.

## Phase 4: localization (30 min)

| Test | Measures | Target |
|---|---|---|
| G-B: known-point return from 3 headings | Map pose error vs RTK truth and tape; repeatability | Truth repeatability ≤ 3 cm; EKF−truth ≈ +0.74 m forward at each heading (confirms the antenna bias; target after fix ≤ 5 cm) |
| K-T4: 5 × 5 m square (pivot turns), figure-8, L-path, vs RTK | Local EKF trajectory error; loop closure; L-path end error | Max error < 1% of path; 5 m relative p95 < 0.10 m; figure-8 < 0.15 m; **L-path end ≈ 0.4 m while the square closes → base_link offset confirmed** |

**Result:** local and global pose accuracy in centimetres, and the magnitude of the antenna bias.

## Phase 5: Nav2 fine motion (60 min)

**Bring-up B:**
1. Stop bring-up A.
2. Start `navigation.launch.py enable_mission_manager:=false`.
3. Start the web UI node on its own (not `webui.launch.py`).

**Checks:** RTK FIXED, `/cmd_vel` ≥ 18 Hz, Foxglove on the laptop.

Goals are short map-frame goals sent by the operator.

| Test | Measures | Target |
|---|---|---|
| T-CPU (during all runs) | Control-loop rate, load | `/cmd_vel` ≥ 18 Hz; no "missed rate" warnings; load < 6 |
| N-T2 heading-hold A/B, 1.524 m gap, start 0.3 m off-centre | Heading-hold lock %, lateral error at gap entry, ω chatter | Entry error ≤ 0.10 m; ≤ 2 ω reversals; deadband 0 equal or better |
| N-T3 gap ladder 2.0 / 1.524 / 1.3 / 1.2 / 1.0 m | Pass rate, clearance per side, time, stops, recoveries | 2.0 and 1.524: 3/3, clearance ≥ 0.10 m, ≤ 38 s, no recovery; **1.0 must be refused** |
| N-T4 barrel dead-centre on path | Go-around without freezing | 3/3; clearance ≥ 0.3 m; no stop > 2 s |
| N-T5 4-barrel slalom, baseline then runtime A/B (vx_max 0.5, PathAlign 8, CostCritic 7, wz_std 0.4) | Completion time, contact, ω reversals, rear-corner clearance | No contact or recovery; ≤ 45 s; ≤ 2 reversals per barrel; rear clearance ≥ 0.10 m |
| N-T6 barrel beside parked robot, then clear-around | Blind-zone fade; false gap after costmap clear | Cells stay lethal for 15 s |
| G-E short GPS goal (~15 m), tape-measured | Goal arrival vs marked point | Record arrival error (expect ~0.74 m short with the current antenna bias) |

**Result:** the real minimum passable gap, slalom performance, and whether the runtime tweaks help.

## Phase 6: wrap-up (15 min)

- Stop `actuator_node`, then **M-T6** (optional):
  - Teensy ramp check, and bus voltage under a 1200 RPM load (≥ 11.5 V).
  - If gains were tuned: `BURN` → expect `result L=0 R=0`.
- Copy every bag off the Jetson.
- Restore the default runtime params.
- Operator runs all analyses and fills in the measurement sheet below.

## Measurement sheet (filled in by the end)

| Subsystem | Numbers we will have |
|---|---|
| **Motors / PID** | Delivery %, overshoot %, t95 and CoV per speed (before/after tune); creep threshold; stop distance and reverse kick; L/R mismatch; final kFF/kP/kI/kIZone; feedforward unit verdict |
| **Kinematics / odometry** | Distance scale (grass, pavement); skid multiplier (grass); spin-centre offset x_c; CW/CCW asymmetry; arc delivery; heading-hold verdict |
| **IMU** | Static drift; heading vs GNSS course offset; IMU ω vs delivered ω |
| **RTK / GNSS** | Time to FIXED; FIXED σ and max; FIXED availability %; re-fix step; antenna lever arm (tape, RTK fit, config); GNSS lag τ |
| **Local localization** | Square / figure-8 / L-path error; 5 m relative p95 |
| **Global localization** | Known-point error per heading; goal-arrival error |
| **Nav2 fine motion** | Minimum passable gap; clearance per gap; slalom time and reversals; control-loop rate; blind-zone and clear-around verdict; A/B tweak verdict |

**Total:** about 4 h with setup. If time is short, **Phases 0–3 are the minimum**. They cover motors, odometry and the spin-centre question, which everything else depends on.
