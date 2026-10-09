> **Status: partly superseded (2026-09-28).** Written for SPARK firmware 25. The test order and principles still hold; the values do not (kV is now in volts per RPM, final gains differ). Current state: `README.md`. Ground procedure: `GROUND_ACCEPTANCE.md`.

# Drive Tuning Strategy: Straight-Line and Turning Parameters

**Date:** 2026-09-28 · **Robot:** tracked skid-steer (AndyMark Raptor), 2 × SPARK MAX (**FW 25.0.4, confirmed on the bench**; see `results/BENCH_2026_09_28.md`) + NEO, Teensy 4.1 CAN bridge
**Goal:** the robot delivers the commanded speed and turn rate within **±2 % (speed)** and **±3 % (turn rate)** on grass. It is then an accurate plant for Nav2 MPPI, which can absorb the small remaining error by re-planning.

Research behind each step is cited by topic ID. Topics are in `research/topics/`, and the firmware review is in `research/evidence/firmware_can_review_2026_09_27/`.

---

> **Bench update (2026-09-28):** the controllers run **FW 25.0.4**. Parameter 16 is **kF in duty per RPM**, and our 0.000197 is correct (feedforward alone gave about 1000 RPM for a 1000 RPM command). **kS/kA (204/205) do not exist on FW 25**, so per-side static friction must go in as arbitrary feedforward in the setpoint frame, or wait for a FW 26 update. Idle mode reads **Coast** and the current limit **80 A stall**; both need a deliberate decision. BURN works with firmware v2.

## 1. Principles

| # | Principle | Why | Source |
|---|---|---|---|
| P1 | **One unknown per step, in signal-chain order:** motor speed → distance scale → turning | A later step's result is only meaningful once the earlier layer is correct. The old multiplier (1.19) was absorbing motor under-delivery. | C1, C2 recommended practice; C3 §2 (cascade) |
| P2 | **Identify open-loop first, then close the loop** | Feedforward comes from a model; feedback trims what is left. Closed-loop data biases naive fits. | C1 practice 1, 9, 15, 18 |
| P3 | **Each quantity gets an independent reference:** encoders for track speed, RTK for distance, gyro for rotation | Encoders cannot see slip; GPS heading is poor when turning slowly; the gyro measures real rotation | C2 practice 4; L1 practice 8 |
| P4 | **On the competition surface, at competition weight, per side, both directions** | Load, surface and direction all change friction and slip | C1 practice 6, 15; C2 practice 8; L1 practice 11 |
| P5 | **Every test is repeated 3×**; report mean and spread | Tuning is meaningless if results vary (target T10) | L1 practice 3, 5; X2 |
| P6 | **Verify every write** by reading back the controller's reply; nothing is persisted until validated | v1 firmware could not confirm writes. BURN is only done at the end, with the controllers disabled. | Firmware review §1.3, §6 |
| P7 | **Log raw data for every run**; decide from plots and numbers, not feel | Graph setpoint and measurement for every test | C1 practice 21 |

## 2. Targets

Targets T1–T11 are from the Phase 1 motor plan (derived from MPPI's 2.8 s horizon and IGVC gap widths). K1–K3 are added for kinematics.

| # | Target | Value |
|---|---|---|
| T1 | Steady speed, each track, 0.1–1.0 m/s | within ±2 % |
| T2 | Feedforward + P only (no I) | within ±5 % |
| T3 | Speed step, 95 % response | ≤ 0.4 s |
| T4 | Overshoot | ≤ 5 % |
| T5 | Left/right mismatch at the same command | ≤ 1 % |
| T7 | Slowest controllable | 0.05 m/s straight, 0.1 rad/s turning, no stick-slip |
| T8 | Command-to-motion delay | ≤ 100 ms |
| T9 | Stop from 0.7 m/s | ≤ 0.30 m (physics floor with the 1.3 m/s² decel cap plus 100 ms delay: 0.7²/(2·1.3) + 0.7·0.1 = 0.26 m) |
| T10 | Repeatability (3 runs) | spread ≤ 1 % |
| T11 | Battery sensitivity (charged vs low battery) | ≤ 2 % speed change. The SPARKs run from a 48→12 V buck, so first check in the `X` line (`bus_V`) whether the motor bus actually sags with the battery; if it doesn't, T11 is met by design |
| K1 | Distance scale (encoder vs RTK), straight 10 m | error ≤ 1 % |
| K2 | Straightness at equal commands, 10 m | **end-point offset** perpendicular to the starting heading (first 1 m of the RTK track) ≤ 5 cm. `analyze.py distance` also prints the fitted sagitta; K2 uses the end offset because it is what the robot ends up doing |
| K3 | Turn-rate delivery (gyro vs command), spins and arcs 1–4 m radius | within ±3 % |

## 3. What gets tuned, and which step sets it

| Layer | Parameter | Today | Set in step |
|---|---|---|---|
| SPARK MAX | kS per side (param 204, volts) | not used | B, C |
| SPARK MAX | kV per side (param 16, **volts per RPM** on FW 26) | 0.000197 (sized as duty/RPM, probably ~12× too weak) | B, C |
| SPARK MAX | kA (205) | not used | B (only if MAXMotion is used later) |
| SPARK MAX | kP, kI, kIZone, kIMaxAccum | 0.0007, 2.5e-7, 600, — | D |
| SPARK MAX | Hall velocity filter period and depth (136/137) | 32 ms × 8 (≈112 ms lag) | B (measure), D (choose) |
| SPARK MAX | STATUS_2 period | 20 ms default | A |
| SPARK MAX | Smart current limit (59–61), idle mode (6), voltage compensation (74/75), closed-loop ramp (114), output min/max (19/20) | unknown / Hardware Client | A (read back), C (set) |
| Teensy | Setpoint slew `M` (RPM per 20 ms) | 100 (5000 RPM/s) | D |
| actuator_node | `m_per_motor_rev` (possibly per side) | 0.01994 | E |
| actuator_node | `wheel_separation_multiplier` (possibly radius-dependent) | 1.19 | F |
| actuator_node | heading-hold (`heading_hold_deadband`, `heading_kp`) | on | **off** for all tuning (C3: extra loops under the planner fight it) |
| actuator_node | accel/decel caps | 0.3 / 1.3 m/s², 1.2 rad/s² | G (confirm). Humble MPPI has no `ax_max`/`az_max` (C4 §11, "Nav2 MPPI — Humble"), so the actuator slew is the only accel limit; match MPPI `vx_max`/`wz_max` to measured capability (C4 practice 3) |

## 4. Test sequence

About 3.5 h on grass including warm-up, split into two sessions: **Session 1** = steps A, B, C, E; **Session 2** = D, F, G, H (both start with the IMU warm-up drive and RTK FIXED). Run with RTK FIXED, the battery fully charged at the start, and the voltage logged on every run.

### Step A: Preflight (15 min, tracks off the ground for A3–A5)
1. Flash firmware v2 (after bench check). `FV` must report 26.1.4 on both controllers.
2. **Read back everything:** each tuning parameter, via read (`PR`) or via the write response (`PWR`). Record current limits, idle mode, voltage compensation, ramp, output limits, feedback sensor and conversion factors. Clear faults (`CF`) and check there are no active faults.
3. Direction and sign: `L+ R+` moves both tracks forward; encoder signs match.
4. Free-running check: equal voltage on both sides → compare RPM (drivetrain friction per side, in the air).
5. Breakaway per side in the air: slow voltage ramp until the track moves. Compare with step B's on-ground value to tell drivetrain friction from ground friction.

### Step B: Open-loop identification (40 min, on grass)
*Actuator_node stopped; the tuning tool owns the serial port. Feedback gains zero. Voltage mode (`UVL/UVR`) if available, otherwise duty × bus voltage.*
1. **Quasistatic ramp**, both tracks the same voltage: 0 → 7 V at 0.5 V/s, forward, then backward. ×3. (About 1.1 m/s at the end, about 5 m of travel.)
2. **Dynamic steps**: 3, 5, 7 V held 2.5 s, forward and backward. ×3.
3. **Spin ramp** (L = −V, R = +V) 0 → 6 V at 0.5 V/s, both directions. This gives the turning load.
4. **Fit per side and direction:** V = kS·sgn(v) + kV·v + kA·a, by SysId's least squares on acceleration (C1-S20 `FeedforwardAnalysis.cpp`), with velocity from encoder **position** (C1 practice 15–16). Checks: **simulated-velocity r² > 0.9**, acceleration r² > ~0.2 for a usable kA (C1-S18). Fit on 2 of the 3 repeats and validate on the third (C1 practice 9).
5. **Measure the speed-reading lag:** cross-correlate the reported STATUS_2 velocity with velocity differentiated from position. Compare with the 112 ms predicted for the default filter (C1 S18/S20). Repeat with the filter at 16 ms × 2 to see the noise vs lag trade-off (C1 practice 19).

### Step C: Feedforward in (15 min)
1. Write kS and kV **per side** (volts). Write the current limit (REV: 40–60 A for NEO, C1 practice 13) and brake idle mode. Decide on voltage compensation from step B's voltage data. Confirm every write with `PWR res=0`.
2. **Check T2** with P = I = 0: closed-loop velocity steps 0.1 / 0.3 / 0.5 / 0.7 / 1.0 m/s, forward and backward. Delivered speed must be within ±5 %. If not, the feedforward fit is wrong: stop and refit rather than tuning P.

### Step D: Feedback (30 min)
1. Choose the hall filter from step B.5 (shortest setting whose RPM noise does not make P chatter).
2. **P sweep** on 0.3 → 0.7 m/s steps: raise P until overshoot reaches T4 or oscillation appears, then back off 30 %. Delay limits P, so record the lag used (C1 practice 17).
3. **I only if T1 fails:** smallest kI with kIZone and kIMaxAccum bounding it (C1 practice 5, 18).
4. **Teensy slew `M`:** set so the ramped setpoint stays within what the motors can follow (from step B's acceleration fit). This is not an overshoot fix.
5. Low speed: crawl at 0.05 m/s and spin at 0.1 rad/s (T7). If there is stick-slip, revisit kS.

### Step E: Distance calibration (25 min)
*Closed loop, direct track commands (equal RPM), heading-hold off.*
1. Straight 10–15 m at 0.5 and 1.0 m/s, forward and backward, ×3. Use the **raw `/gnss` antenna position**, with RTK FIXED on the whole run (`analyze.py distance` prints the GGA quality-4 share: accept only 100 %) and the first and last second trimmed (`--trim`).
2. `m_per_motor_rev = RTK distance ÷ mean motor revolutions` (L1 practice 1; C2 practice 3). **K1.**
3. Lateral drift from RTK plus gyro heading change → left/right scale ratio (UMBmark-style, both directions; L1 practice 2). **K2, T5.**

### Step F: Turning calibration (30 min)
*Closed loop, direct track commands, heading-hold off. Gate: the stationary gyro-z bias printed by `analyze.py turn` must be below 0.1 °/s, otherwise USB power-cycle the Xsens (CLAUDE.md known issue). The Xsens does not remove its bias estimate from the published rate of turn (L2 §12, L2-S38), and the IMU needs a ≥ 5–10 min warm-up before the bias is meaningful (L2 practice 5). `drive_tuner.py vel` holds still for `--pre` 3 s first for this.*
1. **Spins** at 0.3 / 0.6 / 1.0 rad/s (from nominal geometry), CW and CCW, 2 full turns each, ×3.
2. **Arcs** of radius 1 / 2 / 4 m at 0.5 m/s, both directions, ×3.
3. Per run: effective width = (v_R − v_L) ÷ ω_gyro, using track speeds from step E's scale. Multiplier = effective width ÷ 0.7366 (C2 practice 3, 13).
4. **Decision:** if spins and arcs agree within 3 %, use one constant. Otherwise use a radius-dependent value (C2 practice 9) or weight it toward IGVC-typical radii. **K3.**

### Step G: Full-system validation (30 min)
*actuator_node and webui back on, with the new yaml values, heading-hold off.*
1. `/cmd_vel` steps and turns: check T1, T4 and K3 through the real pipeline, including actuator_node slew. T3 (≤ 0.4 s) is a motor-layer target (steps C–D): through actuator_node a 0.3→0.7 m/s step is slewed over 1.3 s at 0.3 m/s², so here measure the lag behind the **slewed** setpoint instead.
2. **UMBmark square** (L1 practice 2): 4 m sides, stop and spin 90° on the spot at each corner, CW and CCW, **5 runs each direction**, with RTK ground truth. Run it with `drive_tuner.py vel` and the calibrated `--m-per-rev` and `--mult`, e.g. `--seq "0.4,0:10 0,0:1 0,0.5:3.14 0,0:1"` repeated 4×. Compare the RTK return-to-start error with wheel odometry (L1 practice 2, 12).
3. T9 stops from 0.7 m/s. T8 delay from the step logs.
4. **T11:** repeat one 0.5 m/s step and one spin at the end of the session (lower battery).

### Step H: Lock in (10 min)
1. Final values into `actuator_params.yaml` (actuator_node pushes them at every start, so this file is the source of truth).
2. v2 safe BURN (controllers disabled first). Confirm `result=0`, power-cycle, and read back.
3. Results table: target | measured | pass/fail, with plots, in `results/`.

## 5. Data recorded per run
- **Teensy telemetry (`X` line, 50 Hz):** Teensy µs timestamp, and per side velocity, position, STATUS_2 arrival time, applied output, current, bus voltage and temperature. Plus the commands sent.
- **ROS bag:**
  - `/imu/data` (gyro)
  - `/gnss` (raw antenna)
  - `/filter/*`
  - RTK status
  Sensors launch only; no actuator_node during steps B–F.
- A run sheet with test ID, repeat #, direction, surface notes and battery voltage.

## 6. Safety
- Clear area of at least 15 × 15 m. One person holds the physical E-stop the whole time.
- The tuning tool: Space = stop. The Teensy 300 ms watchdog stops the motors if the tool or the link dies. The voltage and duty cap (`MD`) is set per test, never above 7.2 V.
- Steps B–F run with actuator_node stopped (one writer on the serial port). The webui cannot drive during these steps.
- If the Teensy USB drops (seen 2026-09-28), the tool stops and must be restarted. Note the time and the motor current just before it, as it points to a supply dip.

## 7. Tools
| Tool | Purpose | Status |
|---|---|---|
| `firmware/teensy_diff_drive_v2/` | Confirmed typed writes, read-back, per-side gains, faults, telemetry, voltage mode, safe BURN | being written; then independent review against the REV spec |
| `docs/drive_tuning_2026_09_28/tools/drive_tuner.py` | Runs the scripted tests of steps A–F, owns the serial port, logs CSV (`--ros` adds `/imu/data`, `/gnss`, `/nmea` GGA) | written; reviewed 2026-09-28 (`REVIEW.md`); not yet run on hardware |
| `docs/drive_tuning_2026_09_28/tools/analyze.py` (`ff`, `lag`, `steps`, `distance`, `turn`) | Analysis per step | written; reviewed and checked on synthetic data 2026-09-28 (`REVIEW.md`) |
