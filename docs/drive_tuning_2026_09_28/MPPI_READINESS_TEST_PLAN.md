> **Status: reference.** The ground tests are now in `GROUND_TEST_PLAN.md`. Firmware-25 values below (FV 25.0.4, 140 ms lag, v2b) are out of date. The research-based targets behind `GROUND_ACCEPTANCE.md`. Bench-scope results on firmware 26 are in `results/FW26_BENCH_2026_09_28.md` and `results/ACCEPTANCE_2026_09_28.md`.

# MPPI Readiness: Measurement and Test Plan for the Drive Stack

**Date:** 2026-09-28 · **Status:** plan only (nothing run, no code changed)
**Question:** are the motors, firmware (Teensy v2b), SPARK MAX loops (FW 25.0.4) and `actuator_node` good enough to be the plant that Nav2 MPPI (Humble) assumes?
**Builds on:** `STRATEGY.md` (targets T1–T11, K1–K3), `RUNBOOK.md`, `REVIEW.md`, `results/BENCH_2026_09_28.md`, and the research topics in `research/topics/` (cited as [C4-Snn] etc.; "C4 p3" = C4 Recommended practice 3).

**Source keys used below (non-research):**

| Key | Meaning |
|---|---|
| [STRAT Tn] | Target in `STRATEGY.md` §2 |
| [BENCH] | `results/BENCH_2026_09_28.md` |
| [BENCH-b] | Bench findings of 2026-09-28 that are not yet written in any file (listed in §0) |
| [YAML-N] | `src/avros_bringup/config/nav2_params_igvc_autonav.yaml`, the Humble default that `navigation.launch.py:56` loads (not `nav2_params_humble.yaml`) |
| [YAML-A] | `src/avros_bringup/config/actuator_params.yaml`; [YAML-E] = `ekf.yaml` |
| [ACT] | `src/avros_control/avros_control/actuator_node.py` (line numbers) |
| [FW] | `firmware/teensy_diff_drive_v2/` (PROTOCOL.md, `.ino` line numbers) |
| [IGVC] | `docs/igvc_rules/IGVC_2026_rules.txt` §II.2 (AutoNav course) |
| (ours) | A number derived in this plan from the sources, not given by a source. The derivation is shown. |

---

## 0. Bench findings of 2026-09-28 not yet in the result files ([BENCH-b])

| # | Finding |
|---|---|
| b1 | All 23 tunable parameters change, confirm (`PWR res=0`) and read back. Effects measured for kF, outputMax, closed-loop ramp (raw = rate, full scale per second), status2Period, and hall depth: speed-reading lag **140 ms at depth 3 → 46 ms at depth 1**. |
| b2 | BURN persists across a power cycle. |
| b3 | Old stop (`S` = velocity 0) slams the PID to full reverse: 46–57 A, bus sags to ≈7–8 V, DRV sticky faults, ≥ 2 s of ringing. |
| b4 | "Standard" setup (P 0.0003, I 0, Brake, 50 A, stop-to-idle) through actuator_node: standstill in 0.7–1.2 s, ≈ −200 RPM dip, 0–1 reversals. Current setup (P 0.0007, I 2.5e-7, Coast, 80 A): 2.1–2.6 s, up to 3 reversals. |
| b5 | Speed ripple off the ground: P 0.0007 → ±100–200 RPM; P ≤ 0.0004 → ±6–17 RPM. |
| b6 | Hall depth 1 with the old gains is violently unstable. |
| b7 | FW 25.0.4: kF is duty/RPM, 0.000197 correct (FF alone gives ≈1000 RPM at a 1000 command). No kS/kA. |

---

## 1. What MPPI needs from the layers below it

### 1.1 What Humble MPPI assumes and reads

| Item | Fact | Consequence for the drive stack | Source |
|---|---|---|---|
| Noise model | The command MPPI sends is only the **mean** of what the vehicle receives; it "has to pass through a lower level of control". | The lower layers must deliver the commanded mean without bias. A constant gain error is not "noise". | [C4-S01 p. 2] |
| Motion model | `DiffDrive`: each rollout step's velocity **equals the previous sampled command**, reached after one `model_dt` (0.05 s), with **no acceleration limit and no lag**. | Every delay, first-order lag and slew in our stack is unmodelled. | [C4-S26 `predict`, `updateInitialStateVelocities`]; [YAML-N `motion_model: DiffDrive`] |
| Initial state | Step 0 of every rollout is seeded with the **latest odometry twist**, dead-banded by `getThresholdedTwist` (0.01 m/s, 0.01 rad/s), with no averaging and no age check. | Odometry lag and noise go straight into every rollout. Standstill noise must stay below 0.01. | [C4-S26]; [C4-S34 controller_server.cpp:478]; [YAML-N `min_x/theta_velocity_threshold: 0.01`] |
| Parameters read on Humble | `vx_max`, `vx_min`, `wz_max`, `vx_std`, `wz_std`, `model_dt`, `time_steps`, `batch_size`, `temperature`, `gamma`, `iteration_count`, `prune_distance`, critics. **Not** read: `ax_max`, `ax_min`, `az_max`, `open_loop`, `model_delay_*`. | The `ax_max 0.4 / ax_min −1.5 / az_max 1.5` lines in [YAML-N] are **dead**. The actuator slew is the only acceleration limit, and MPPI does not know it. No delay compensation exists on Humble. | [C4-S26 `getParams`]; [C4-S30]; [C4-S37 "not backported"] |
| Output clipping | Final sequence clipped to `vx_max`, `vx_min`, `wz_max`. | `wz_max 1.9` > actuator `max_angular_rps 1.5` → anything above 1.5 rad/s is silently clipped downstream (under-delivery MPPI does not see). | [C4-S26]; [YAML-N]; [YAML-A]; C4 p3 |
| Gain mismatch | A persistent fraction-of-command error leaves a steady tracking offset in a fixed-model predictive controller. | Delivery accuracy must be fixed below MPPI (calibration), not left to MPPI. | [C4-S05 pp. 52–53]; [C4-S52]; [C4-S50 pp. 2186–2187]; [C4-S51 p. 5] |
| Delay | Unmodelled actuator delay causes overshoot, oscillation after turns and late turn-in. | Measure the delay per layer; keep it well under the horizon. | [C4-S16 p. 5]; [C4-S37]; [C4-S38] |
| Odometry | Closed-loop feedback needs "high rate and low latency" odometry, at least as fast as the control rate, ideally much faster. | `/odometry/filtered` rate and lag are readiness criteria. | [C4-S31]; [C4-S27]; [C4-S35] |
| Cascade | Each lower loop 4× (minimum) to 10× (preferable) faster than the loop above; ≤ 3× the layers "fight". | Heading-hold is an extra loop between MPPI and the motor loop (see §5). | [C3-S03 p. 9]; C3 p1; C1 p4 |
| Horizon vs costmap | `time_steps × model_dt × vx_max` must fit the local costmap; `model_dt` ≤ control period. | 56 × 0.05 = **2.8 s**, × 0.7 m/s = 1.96 m ≪ 25 m half-width. `model_dt` = 1/20 Hz. OK. | [C4-S25]; [YAML-N] |

### 1.2 How odometry reaches MPPI in this stack

| Signal in `/odometry/filtered` (local EKF, 30 Hz) | Comes from | Known lag/quality issue | Source |
|---|---|---|---|
| `twist.linear.x` | `/wheel_odom` vx only (50 Hz) ← E-line **STATUS_2 velocity** (hall filter) | 140 ms at depth 3, 46 ms at depth 1; stamped with host `now()`, not measurement time; declared σ_vx = 0.01 m/s | [YAML-E odom0_config]; [ACT 540–586, 624]; [BENCH-b b1]; L5 p1 |
| `twist.angular.z` | Xsens gyro `/imu/data` vyaw (100 Hz); wheel vyaw is **not** fused | Gyro bias can stick (CLAUDE.md known issue); IMU rejection gate 5σ | [YAML-E imu0_config, odom0 comment]; L2 p5 |

### 1.3 Quantitative targets derived from the horizon and IGVC geometry

Inputs:
- Horizon T = 56 × 0.05 s = **2.8 s** [YAML-N].
- Cruise speed v = `vx_max` **0.7 m/s** [YAML-N].
- Minimum course radius R = 5 ft = **1.52 m** → ω = v/R = **0.46 rad/s** [IGVC].
- Minimum passage between line and obstacle = 5 ft = **1.524 m** [IGVC]; robot width 0.83 m [YAML-N footprint] → lateral slack ≈ (1.524 − 0.83)/2 = **0.35 m per side** when centred.
- Minimum average speed 1 mph = **0.447 m/s** [IGVC].
- Unit conversions: 1 m/s track speed = 60/0.01994 = **3009 motor RPM**. 1 rad/s body rate = 0.877/2 m/s per track = **1319 RPM** [YAML-A].

**Budget (ours):** at most **0.10 m** of the 0.35 m slack (≈ 30 %) may be used by drive-stack prediction error over one horizon. The rest is left for localisation, perception and MPPI sampling.

| Target | Formula (ours unless cited) | Value | Why / source |
|---|---|---|---|
| **G1** steady v delivery | along-track error over horizon = ε_v·v·T = 1.96·ε_v m | **±2 %** → 0.04 m | [STRAT T1]; MPPI mean assumption [C4-S01]; offset otherwise persists [C4-S05, C4-S52] |
| **G2** steady ω delivery | open-loop lateral error on the R = 1.52 m arc ≈ ½·v·ε_ω·ω·T² = 1.26·ε_ω m | **±3 %** → 0.04 m (ε_ω = 8 % would use the whole 0.10 m budget) | [STRAT K3]; [C4-S50] (×0.5 turn-rate error → 0.8–0.95 m lateral error on a fixed-model MPC) |
| **G3** L/R match at equal command | curvature error from a speed split | **≤ 1 %** | [STRAT T5] |
| **G4** effective delay, full pipeline (τd + τc, per axis) | an unmodelled equivalent delay τ shifts a turn by ≈ v·τ | **goal ≤ 0.14 s** (= 0.10 m / 0.7 m/s); **hard ≤ 0.28 s** (= T/10) | Budget (ours); hard limit = 10× time-scale separation [C3-S03] taking the horizon as the outer time scale (ours). Humble assumes ≈ one `model_dt` [C4-S26] |
| **G5** motor-layer response | 95 % step response; overshoot | **t95 ≤ 0.4 s, overshoot ≤ 5 %**, command-to-motion delay **≤ 100 ms** | [STRAT T3, T4, T8]; C1 p4 (inner loop much faster) |
| **G6** odometry lag into MPPI | seed error = a·L. At the ω slew of 1.2 rad/s²: L = 140 ms → 0.17 rad/s (37 % of 0.46); L = 50 ms → 0.06 rad/s | **L ≤ 50 ms** for both v and ω | (ours) from [C4-S26] seeding + [YAML-A] slew; Nav2 main now predicts one control period forward to cover this [C4-S28] |
| **G7** odometry rate / gaps | — | **≥ 20 Hz, no gap > 100 ms** | [C4-S27]; [YAML-N `controller_frequency: 20`] |
| **G8** odometry noise (steady) | ≤ 1/10 of the sampling std (`vx_std 0.3`, `wz_std 0.5`) | **σ_v ≤ 0.03 m/s, σ_ω ≤ 0.05 rad/s**; at standstill \|v\|, \|ω\| < 0.01 in 99 % of samples | (ours) from [YAML-N]; noise is not modelled by MPPI [C4-S48]; dead-band [C4-S34] |
| **G9** stop from 0.7 m/s | 0.7²/(2·1.3) + 0.7·0.1 | **≤ 0.30 m**, 0 reversals, no sticky fault | [STRAT T9]; C3 p15 [C3-S46]; [BENCH-b b3, b4] |
| **G10** control loop | MPPI must meet its period | `/cmd_vel` **≥ 18 Hz** under full load, 0 "Optimizer fail" | [YAML-N time_steps comment]; `model_dt` ≤ period [C4-S25] |
| **G11** min-speed gate | 44 ft in ≤ 30 s from standstill, through the 0.3 m/s² slew | average **≥ 0.447 m/s** over the first 13.4 m | [IGVC]; [YAML-A] |

**Known conflict to settle in S8 (ours):**
- The angular slew (1.2 rad/s²) alone takes 0.46/1.2 = 0.38 s to reach the IGVC turn rate. That is an equivalent delay of ≈ 0.19 s, which already exceeds the G4 goal (0.14 s), though not the hard limit (0.28 s).
- The linear slew (0.3 m/s²) takes 2.3 s to reach 0.7 m/s. From standstill, the pass-through model is wrong by up to v²/(2a) = **0.82 m along-track** over a horizon.
- On Humble, only the actuator slew can close this gap: MPPI has no `ax_max`, `az_max` or delay parameter to read [C4-S26].

---

## 2. Scopes

Each scope measures one layer. Upstream layers are held fixed; downstream layers are removed or bypassed.

| ID | Scope | Isolation (what is bypassed/fixed) |
|---|---|---|
| **S0** | Reference instruments and clocks | nothing driven; validates the rulers |
| **S1** | Firmware and CAN timing integrity | drive_tuner owns the port; no actuator_node |
| **S2** | Controller configuration and persistence | motors idle |
| **S3** | Per-motor velocity loop | Teensy `L/R` direct, Teensy slew `M` raised; no ROS |
| **S4** | Stopping and direction reversal | S4a motor layer (direct); S4b pipeline (`/cmd_vel`) |
| **S5** | Command-to-motion latency per layer | each hop timed separately |
| **S6** | Electrical under load | logged during S3/S4/S7 runs, plus one soak |
| **S7** | Chassis delivery on the ground | direct track commands, heading-hold off |
| **S8** | Plant model as MPPI sees it | S8a motor+chassis; S8b full `/cmd_vel` pipeline |
| **S9** | Odometry feedback quality | `/wheel_odom` and `/odometry/filtered` vs references |
| **S10** | End-to-end with MPPI | full stack |

Changes to the suggested list:
- **Added S0.** Every scope needs a reference ≥ 10× better than the quantity under test, with time offsets calibrated [X2 p2–p3].
- **Split S4 and S8** into motor-layer and pipeline parts. Otherwise the host slew hides the motor behaviour (§5).

---

## 3. Scope details

Common rules for all scopes:
- **Repeats:** 3 per condition (5 for UMBmark) in randomised order, reported as mean ± s/√n [STRAT P5; X2 p5; L1 p3].
- **Logging:** raw data logged, and setpoint and measurement plotted for every run [C1 p21; X2 p10].
- **Velocity signal:** velocity is taken from the **position** (`pos_rot`, central difference over unique STATUS_2 samples), never from the lagged STATUS_2 velocity, unless the lag itself is being measured [REVIEW M0, M2].
- **Battery:** bus V is recorded at the start and end of every run.

### S0 Reference instruments and clocks
| Item | Content |
|---|---|
| Measures / why | Accuracy of each reference, and the time offsets between Teensy µs, host clock, `/imu/data` and `/gnss`. Every later scope depends on these rulers [X2 p2, p3]. |
| Method | (a) Teensy↔host clock: 10 min of `X1` telemetry; fit host arrival time vs `teensy_us` (lowest-latency envelope) → offset and skew [L5 p2]. (b) Gyro: 3 min stationary bias after a ≥ 5–10 min warm-up and one motion warm-up [L2 p5; memory note]. (c) RTK: 2 min static, GGA quality 4 share. |
| Instrument | Teensy `micros()`; Xsens gyro (field scale 1.000 per [YAML-E] comment); RTK GNSS. |
| Metric | Clock skew (ppm) and residual (p99); gyro bias (°/s); RTK static σ (cm). |
| Pass | Clock residual p99 ≤ 2 ms. Gyro bias < 0.1 °/s, otherwise USB power-cycle the Xsens [STRAT F gate; REVIEW M10]. RTK 100 % FIXED, σ ≤ 2 cm, so that 10 m runs resolve 1 % at ≥ 10× [X2-S16 pp. 2, 5]. |
| Depends on | — |
| Tools | `drive_tuner.py listen`, `--ros`; `analyze.py turn` (bias). **New:** `clock_map.py` (≈ 40 lines: offset + skew fit from `teensy.csv`). |

### S1 Firmware and CAN timing integrity
| Item | Content |
|---|---|
| Measures / why | Whether the bridge delivers setpoints and feedback on time, every time. Jitter and gaps look like delay and noise to the loops above [C3-S42 p. 48]. |
| Method (bench) | (1) 10 min at 1000 RPM with `X1`: STATUS_2 inter-arrival per side (`s2_rx_us`); setpoint TX period (`sp_tx_us`); DIAG counters `txfail`, `sdrop`, `foreign`, `rxage` before and after. (2) Host→Teensy round trip: 2000 × `L0 R0` → `OK L=` echo time. (3) Watchdog: stream `L1000 R1000`, stop sending; time to applied = 0 (X `applied`) and to position standstill; ×10. (4) `HB0` test (already PASS [BENCH]); repeat once after the final BURN. (5) Heartbeat lock `hblock=1/1` continuous over the 10 min. |
| Instrument | X-line Teensy timestamps; host monotonic clock. |
| Metrics | Inter-arrival mean, σ, p99, max; missed frames = count(Δt > 1.5 × period); RTT p50/p99/max; watchdog latency = t(applied=0) − t(last command). |
| Pass | STATUS_2 period 20 ms with jitter σ < 2 ms (< 10 % of period [C3-S42]) and 0 gaps > 60 ms (SPARK signal-loss window [C3-S48]). Setpoint period 20 ms ± 2 ms. `txfail = sdrop = 0`. RTT p99 ≤ 10 ms (ours; report the distribution per L5 p14, C3 p10). Watchdog stop ≤ 340 ms (300 ms + one tick, [FW .ino:93]). Note C3 p14 / [C3-S43] recommends ≈ 100 ms per actuator: record as a finding, not a fail. |
| Depends on | S0 (clock) |
| Tools | `drive_tuner.py vel`/`listen` (X log); `bench/bench.py` (DIAG, HB0). **New:** `timing_audit.py` (inter-arrival stats, gap count, RTT loop, watchdog timing). |

### S2 Controller configuration and persistence
| Item | Content |
|---|---|
| Measures / why | The controllers run exactly the decided configuration, and it survives a power cycle. Unverified CAN writes are the main risk in this stack [STRAT P6; C1 "common mistakes": not burning]. |
| Method (bench) | Read all 23 tunables on both sides (`PR` / `PWR` values) → manifest JSON. Diff against the decided set: kF per side, P (≤ 0.0004 per [BENCH-b b5]), I = 0, idle Brake, smart stall 50 A (REV 40–60 A [C1-S13]), hall period/depth, status2Period, closed-loop ramp, outputMin/Max, voltComp. Then BURN (disabled) → power-cycle → read back → diff. `FV` on both. `CF`, then check STATUS_1 stays clean for 10 min. |
| Instrument | `PWR`/`PRD` replies (controller's own value). |
| Metric | Mismatch count; BURN result code; sticky-fault flags. |
| Pass | 0 mismatches before and after power cycle; `OK BURN result L=0 R=0`; FV 25.0.4 on both; no new sticky faults. b1/b2 already show the mechanism works [BENCH-b]; this scope freezes the **final** set. |
| Depends on | S3/S4 decisions for the values; re-run after every change. |
| Tools | `drive_tuner.py preflight`, `cmd`; `bench/bench.py`. **New:** `config_manifest.py` (dump + diff against a golden JSON, also against `actuator_params.yaml` gains). |

### S3 Per-motor velocity loop
Conditions: velocity mode direct (`L/R`). Teensy slew **`M` raised to ≥ 1000** so the setpoint is a true step (at the default M = 100, 5000 RPM/s, a 1200 RPM step is itself ramped over 0.24 s). Both sides, both directions. The bench runs first; the ground runs repeat the fit and the gain checks. SysId warns against characterising on blocks [C1 p15; C1-S18].

| Sub-test | Input | Metric (formula) | Pass | Source |
|---|---|---|---|---|
| S3.1 FF accuracy | P = I = D = 0; hold 300, 900, 1500, 2100, 3000 RPM, 4 s each, ± | e_FF = (mean v_pos over last 50 % of hold − cmd)/cmd | \|e_FF\| ≤ 5 % | [STRAT T2]; C1 p15–16 |
| S3.2 FF identification (ground) | Quasistatic ramp 0→7 V at 0.5 V/s, and steps at 3/5/7 V, ± | kS, kV, kA per side/direction by SysId OLS; sim-velocity r² | r² > 0.9, accel r² > 0.2; validate on a held-out repeat | C1 p9, p16; [C1-S18, S20]; [REVIEW M1] |
| S3.3 steady error | final gains; same levels | e_ss as above | ≤ 2 % for ≥ 300 RPM (0.1 m/s) | [STRAT T1] |
| S3.4 ripple | final gains; 2 s steady windows | σ and ½·p2p of v_pos (100 ms window) | ½·p2p ≤ 20 RPM off-ground (achieved 6–17 at P ≤ 0.0004 [BENCH-b b5]); on ground σ ≤ 0.01 m/s (30 RPM) so that odometry meets G8 with margin | (ours) from G8; C1-S04 "without oscillation" |
| S3.5 step response | 900→2100 RPM (≥ 1200 RPM step [REVIEW M4]) and back, ± | t95 from **setpoint TX** time (`sp_tx_us`); overshoot = (peak − target)/span; IAE and input effort TV of `applied` | t95 ≤ 0.4 s, OS ≤ 5 % | [STRAT T3, T4]; X2 p7 [X2-S15] |
| S3.6 delay | same steps | τd = first v_pos departure > 3σ_noise after `sp_tx_us` | ≤ 100 ms (motor part of T8) | [STRAT T8]; C1 p1 |
| S3.7 measurement lag | same steps; hall depth 3, 2, 1 (gains re-tuned per depth [C1 "mistakes": filter change]) | cross-correlation lag of STATUS_2 velocity vs v_pos | choose the shortest depth whose loop still meets S3.4 and S3.5; expect ≤ 50 ms (G6) | C1 p17, p19; [C1-S20/S21]; [BENCH-b b1, b6] |
| S3.8 low speed | 150 RPM (0.05 m/s) and 132 RPM (0.1 rad/s spin), 10 s, ± | fraction of 100 ms windows with v_pos = 0; CV = σ/mean | no zero windows; CV ≤ 20 % (ours) | [STRAT T7]; C1 p6 |
| S3.9 saturation | request 5500 RPM (clamped at 4600 [FW]) then return to 2000 | max delivered; overshoot on return | overshoot on return ≤ 5 % (no windup; trivially true with I = 0) | C1 p5 |
| S3.10 load step (ground) | constant 0.5 m/s, drive onto a ramp ≤ 15 % [IGVC] | speed dip, recovery time, IAE | dip ≤ 10 %, recovered < 1 s (ours) | X2 p7 |

- **Depends on:** S1, S2 (values in RAM).
- **Tools:** `drive_tuner.py ramp/steps/vel`, `analyze.py ff/lag/steps`, `bench/standard_setup_test.py`. **New:** add `--ripple` and `--delay` outputs to `analyze.py steps` (small extension).

### S4 Stopping and direction reversal
| Item | Content |
|---|---|
| Measures / why | Stops and zero crossings are where the old configuration failed: full-reverse slam, faults, reversals [BENCH-b b3, b4]. MPPI's sinusoidal course means frequent ω sign changes [IGVC "primarily sinusoidal curves"]. |
| S4a motor layer (bench → ground) | From 2100 RPM (0.7 m/s): (i) `S` (stop-to-idle), (ii) watchdog timeout, (iii) SS1 = ramp to 0 then `S`. ×5 each. |
| S4b pipeline (bench → ground) | `/cmd_vel` 0.7 → 0 (decel slew 1.3 m/s²); cmd_vel goes stale (0.5 s timeout); ω +0.46 → −0.46 rad/s at v = 0.4; v +0.4 → −0.3 (BackUp recovery only, since `vx_min 0` [YAML-N]). ×5 each. |
| Instrument | X-line position, current, bus V, fault lines (`F`); `/avros/wheel_debug` pos_rev for S4b; RTK for distance on the ground. |
| Metrics | Stopping distance = Δpos × m/rev (encoder) and RTK chord; time to standstill (\|v_pos\| < 30 RPM for 0.2 s); reversals = sign changes of v_pos with \|v\| > 30 RPM; peak current; min bus V; new sticky faults; for ω flips: time to zero crossing vs slew-predicted (0.46·2/1.2 = 0.77 s), and hunting amplitude near zero. |
| Pass | G9: ≤ 0.30 m from 0.7 m/s (pipeline, ground). 0 reversals. Peak current ≤ smart limit. No DRV sticky fault. ω zero-crossing within ± 1 control tick (20 ms) of the slew prediction plus the S3 delay. No hunting > ±30 RPM around zero (bench finding 3 in [BENCH] showed ±50). |
| Depends on | S2 (idle Brake, current limit), S3 (gains) |
| Tools | `bench/stoptest.py`, `stopspike.py`, `actuator_stop_test.py`, `run_stop_comparison.sh`, `drv_hunt.py`. **New:** `actuator_stop_test.py --omega-flip` mode (small extension). |

### S5 Command-to-motion latency per layer
| Hop | How it is timed | Pass | Source |
|---|---|---|---|
| H1 MPPI `/cmd_vel` publish → actuator_node tick | bag receive time of `/cmd_vel` vs the change in `v_req` in `/avros/wheel_debug`. The 50 Hz tick adds 0–20 ms of zero-order hold. | p99 ≤ 25 ms | C3 p10, p13 |
| H2 host serial write → Teensy | RTT/2 from S1 | p99 ≤ 5 ms | L5 p11 |
| H3 Teensy receipt → CAN setpoint | `sp_tx_us` − receipt; ≤ one 20 ms tick | ≤ 20 ms | [FW §4] |
| H4 CAN setpoint → motion | S3.6 τd | ≤ 100 ms | [STRAT T8] |
| H5 motion → reported velocity | S3.7 lag | ≤ 50 ms (G6) | C1 p17 |
| H6 E-line → `/wheel_odom` → EKF → `/odometry/filtered` | bag arrival chain; EKF at 30 Hz adds 0–33 ms | total odometry age ≤ 50 ms (G6) | [C4-S35]; [YAML-E] |
| **Sum H1–H4** | command-to-motion | **≤ 100 ms** (T8; also within the G4 goal) | [STRAT T8]; [C4-S44 p. 616] (0.1 s input delay typical) |

- **Method:** bench first (H1–H3 and H5 are load-independent), then H4 on the ground. 200 steps with random timing relative to the 20 ms tick, so the distribution is sampled [L5 p14].
- **Instrument:** the S0 clock map joins Teensy and host times.
- **Depends on:** S0, S1, S3.
- **Tools:** new `latency_chain.py`. Pipeline runs need Teensy timestamps while actuator_node owns the port, so they need either an X-line passthrough/log in actuator_node (a future code change, flag-gated) or a serial tee (pty). **Until then, H4–H5 in the pipeline come from `/avros/wheel_debug` pos_rev with host receive times (≈ ±10 ms resolution).**

### S6 Electrical under load
| Item | Content |
|---|---|
| Measures / why | Bus sag, current, temperature and faults. Sag changes the delivered speed (T11), and brown-outs caused USB drops and the Jetson crashes (CLAUDE.md). |
| Method | Log X `bus_V`, `current_A`, `temp_C` and `F` lines in every S3/S4/S7 run. Plus one **6-minute soak** (IGVC run length [IGVC §II.2 "Six (6) minutes"]) of MPPI-like commands (S8 random sequence) on the ground, at full charge and again at low charge. |
| Metrics | Min bus V; ΔV/ΔA (source impedance); peak and RMS current per side; temperature rise and slope over the last 2 min; fault/warning events; Teensy USB drops. |
| Pass | 0 sticky faults, 0 USB drops. Peak current ≤ smart limit (50 A). Temperature slope → plateau (< 1 °C/min in the last 2 min, ours). Speed change charged vs low ≤ 2 % [STRAT T11] (or voltage compensation on, C1 p20). Min bus V: record, and set the pass value to the measured Jetson/Teensy brown-out threshold + margin (no source gives it). |
| Depends on | S2 |
| Tools | `drive_tuner.py` logs these; `drv_hunt.py`. **New:** `analyze.py electrical` subcommand (min/peak/slope/fault summary). |

### S7 Chassis delivery on the ground
| Item | Content |
|---|---|
| Measures / why | Body v and ω vs command (the MPPI "mean" assumption at the chassis level [C4-S01]) on the **competition surface**. IGVC AutoNav is on **asphalt** [IGVC §II.2]; grass is practice only. Slip parameters are per terrain [C2 p8, p18; L1 p4]. |
| Method | `STRATEGY.md` steps E, F, with heading-hold **off** (`heading_hold_deadband 0.0`; setting `heading_kp 0` instead would zero MPPI's small ω, [ACT 444–452]). Straights of 10 m at 0.45 and 0.7 m/s, ±. Spins at 0.3/0.6/1.0 rad/s CW/CCW. **Arcs at the IGVC radius 1.52 m and at 3 m** (in place of STRATEGY's 1/2/4 m), at 0.45 and 0.7 m/s. ×3 each. |
| Instrument | RTK chord distance (K1); gyro ω with bias removed (K3); encoder track speeds. |
| Metrics | ε_v = v_RTK/v_cmd − 1; ε_ω = ω_gyro/ω_cmd − 1; L/R ratio; end offset over 10 m (K2); effective width = (v_R − v_L)/ω_gyro. |
| Pass | G1 ±2 %, G2 ±3 %, G3 ≤ 1 %, K1 ≤ 1 %, K2 ≤ 5 cm. Spin vs arc multiplier within 3 %, otherwise radius-dependent [C2 p9]. |
| Depends on | S0, S3 (G5 met), S4 |
| Tools | `drive_tuner.py vel`, `analyze.py distance/turn` (reviewed). No new tool. |

### S8 Plant model as MPPI sees it
| Item | Content |
|---|---|
| Measures / why | First-order-plus-delay (FOPDT) models for v and ω, and effective accel/decel limits: (a) motor + chassis (direct track commands, Teensy M high); (b) full pipeline (`/cmd_vel` → slew → heading-hold off → inverse → motor). Humble MPPI's pass-through model [C4-S26] is then scored against reality over its own horizon [C4 p1; C4-S17]. |
| Method | (1) **Unloaded calibration (bench):** command the full (v, ω) grid on blocks; encoder speed = commanded speed across the space [C4-S21 pp. 15–16; C2 p10]. (2) **Steps (ground):** v 0→0.45→0.7→0.3→0; ω 0→±0.46→±1.0 at v = 0.4; hold ≥ 4 s. (3) **DRIVE-style random commands:** uniform random (v, ω) in the reachable space, 6 s holds (2 s transient + 2×2 s steady) [C4-S21 p. 7]. (4) **MPPI-like rapid commands:** random walk at 20 Hz with steps drawn from N(0, vx_std²) and N(0, wz_std²) [C4-S46 p. 5]. (5) Effective accel: large v and ω steps with the actuator slew temporarily raised (dynamic params, ground, current-limited). This measures the chassis' own limit, separate from the slew. |
| Instrument | v from RTK (ground) and encoder position; ω from gyro; commands from bag. |
| Metrics | Per axis/direction: gain K, τd, τc (least squares on the step data; validated on (3)–(4) held-out data); sim r². Multi-step prediction error at t = 2.8 s over all windows of (4), for three models: M0 Humble pass-through, M1 slew only, M2 slew + FOPDT → lateral and along-track RMS/p95 [C4 p1; C4-S01 p. 17]. Measured a_max, a_min, α_max vs actuator caps. |
| Pass | FOPDT sim r² > 0.9 (C1-S18 analogue). Pipeline τd + τc: hard ≤ 0.28 s, goal ≤ 0.14 s (G4). **M0 horizon-end lateral error p95 ≤ 0.10 m** on the R ≥ 1.52 m, 0.7 m/s sequences (budget, §1.3). If M0 fails but M1/M2 pass, the mismatch is the slew → decide on slew caps vs brown-out, or on a Nav2 version with `ax_max`/`model_delay` ([C4-S30]). Also: `wz_max` ≤ min(actuator cap, measured α-limited reach) and `vx_max` ≤ measured deliverable (C4 p3). |
| Depends on | S3, S5, S7 (calibrated m/rev and multiplier) |
| Tools | `drive_tuner.py vel --seq` (steps, direct); `bench/actuator_stop_test.py` pattern for `/cmd_vel`. **New:** `random_cmd.py` (DRIVE and MPPI-like `/cmd_vel` generators with a SPACE stop) and `fopdt_fit.py` (fit + M0/M1/M2 horizon replay). |

### S9 Odometry feedback quality
| Item | Content |
|---|---|
| Measures / why | What MPPI's step 0 is seeded with [C4-S26, C4-S34]. Rate, age, lag, noise, scale, standstill dead-band, and covariance realism [L1 p9]. |
| Method | Bench: rate, gaps, stamp age, standstill noise, `/wheel_odom` v lag vs encoder position (hall depth candidates from S3.7). Ground: S7/S8 runs; `/odometry/filtered` vs RTK velocity (v) and raw gyro (ω); 60 s standstill after motion. |
| Instrument | Encoder position (v truth on bench), RTK (v on ground), gyro (ω). S0 clock map. |
| Metrics | Rate and inter-arrival p99. Age = receive − `header.stamp`. Lag = cross-correlation of odometry twist vs reference. Noise σ in steady windows. Scale error. Standstill share with \|v\|, \|ω\| < 0.01. Covariance ratio = measured var / declared var (NEES-style [X2-S26]; declared σ_vx = 0.01 [ACT 297–300]). |
| Pass | G7 (≥ 20 Hz, no gap > 100 ms); G6 (lag ≤ 50 ms for v and ω); G8 (σ_v ≤ 0.03, σ_ω ≤ 0.05; standstill 99 % under 0.01); scale G1/G2; covariance ratio 0.5–2 (ours). **Expected fail today:** v lag ≈ 140 ms at hall depth 3 [BENCH-b b1]. The fix candidates (depth 1 with retuned gains, or position-differenced velocity) are measured, not assumed. |
| Depends on | S0, S3.7, S7 |
| Tools | `drive_tuner.py --ros`; `ros2 bag record`. **New:** `odom_quality.py`. |

### S10 End-to-end validation with MPPI
| Item | Content |
|---|---|
| Measures / why | Whether the stack that passed S1–S9 tracks MPPI's plans on an IGVC-like course. Metrics follow the tracker literature [C4 "How it is tested"; C4-S41]. |
| Method | Course on asphalt (then grass): 3.05 m lane [IGVC], sinusoid with R ≥ 1.52 m, barrel pairs leaving 1.524 m passages, a 13.4 m start segment. Goals in the odom/global frame per the known-issue notes. Heading-hold **A/B** (on vs off), 10 runs each. Optional B: hall depth candidate. Full stack, **no RViz on the Jetson** (CLAUDE.md). |
| Instrument | RTK track; bag of `/cmd_vel`, `/local_plan`, `/odometry/filtered`, `/avros/wheel_debug`, `/rosout`. |
| Metrics | Success rate. Cross-track error vs global path (mean, max) and vs MPPI's own predicted trajectory at +1 s and +2.8 s. Command-vs-delivered RMS (v, ω). ω sign changes per metre on straights (oscillation). Fraction of ticks where the actuator slew is active (\|v_req − v_slewed\| > 1e-3). Fraction with heading-hold locked. `/cmd_vel` rate. "Optimizer fail" count. Min-speed segment average. |
| Pass | 10/10 runs without collision or line cross (80 % reliability at 85 % confidence [X2-S16 p. 5]). Max XTE ≤ 0.35 m, mean ≤ 0.10 m (slack and budget, §1.3). G10 (≥ 18 Hz, 0 optimizer fails). G11 (≥ 0.447 m/s). Slew active ≤ 10 % of moving time (C3 p6 says "rarely"; 10 % is ours). If heading-hold "on" is not better than "off" on XTE and oscillation, it stays off under MPPI (C3 p1; C4 Nav2 layering). |
| Depends on | S1–S9 pass |
| Tools | `analyze.py distance` (RTK prep). **New:** `mppi_run_metrics.py` (bag → the metrics above). |

---

## 4. Execution order

| Order | Test | Where | Can run NOW? | Gate before continuing |
|---|---|---|---|---|
| 1 | S0a Teensy↔host clock map | bench | **yes** | residual p99 ≤ 2 ms |
| 2 | S1 timing (1)–(5) | bench | **yes** | jitter, gaps, watchdog pass |
| 3 | S2 manifest of the current state (baseline) | bench | **yes** | mismatches listed |
| 4 | S3.1 FF check, S3.4 ripple, S3.5 steps (M ≥ 1000), S3.6 delay, S3.8 low speed, S3.9 saturation, both sides and directions | bench (off ground) | **yes** | provisional P and hall depth chosen |
| 5 | S3.7 hall depth 3/2/1 with gains re-tuned per depth | bench | **yes** | depth whose lag ≤ 50 ms with S3.4/S3.5 met, or record why not |
| 6 | S4a motor-layer stops; S4b pipeline stops and ω flips (off ground) | bench | **yes** | 0 reversals, no DRV fault |
| 7 | S5 hops H1–H3, H5, H6 | bench | **yes** | sum within budget |
| 8 | S8(1) unloaded command-space calibration; pipeline software transfer (cmd_vel → L/R setpoint in wheel_debug is exact and checkable) | bench | **yes** | commanded = encoder across space |
| 9 | S9 bench part (rate, gaps, age, standstill noise, v lag) | bench | **yes** | G7; G6 candidate identified |
| 10 | S10 compute check: full stack, a goal sent, tracks off ground; measure `/cmd_vel` rate only | bench | **yes** (rate only) | G10 rate |
| 11 | S2 write decided values → BURN → power cycle → diff | bench | **yes**, after 4–6 | 0 mismatches |
| 12 | S0b/c gyro bias, RTK static | ground | no | bias < 0.1 °/s, 100 % FIXED |
| 13 | S3.2 ground FF identification, S3.3, S3.5 repeat, S3.10 load step | ground | no | T1–T4 on ground |
| 14 | S4 on ground (stopping distance G9) | ground | no | ≤ 0.30 m |
| 15 | S7 straights, spins, arcs (asphalt, then grass) | ground | no | G1–G3, K1–K2 |
| 16 | S8(2)–(5) model identification and horizon replay | ground | no | M0 ≤ 0.10 m, or a mitigation decision |
| 17 | S9 ground; S6 6-min soak (full and low charge) | ground | no | G6–G8; T11 |
| 18 | S10 end-to-end, heading-hold A/B | ground | no | readiness |

---

## 5. Risks and confounders

| Risk | Effect on measurements | Control |
|---|---|---|
| **Off-ground vs loaded** | Off the ground there is no track–ground friction, less inertia and no slip. FF (kS, kV), ripple, stop reversals and step times all differ; SysId says do not characterise on blocks [C1 p15; C1-S18]. The right side already has ≈ 10 % more drivetrain friction [BENCH §3.5]. | Use bench results only for timing, config, stability, and relative comparisons (P, depth). Every numeric pass for S3/S4/S7/S8 is re-measured on the ground, on the competition surface (asphalt [IGVC]). |
| **SPARK velocity measurement lag** (140 ms at depth 3, 46 ms at depth 1) | Reported velocity makes the loop look 3× slower than it is; it also delays `/wheel_odom` v and hence MPPI's seed. A shorter depth adds quantisation noise and destabilised the old gains [BENCH-b b1, b6; C1 p19]. | Judge motion with position-derived velocity [REVIEW M2]; measure the lag explicitly (S3.7); re-tune gains after any depth change [C1-S18 "Measurement Delays"]. |
| **Host slew masks motor behaviour** | Through actuator_node, 0.3 m/s², 1.3 m/s² and 1.2 rad/s² ramps are slower than the motor loop, so pipeline tests measure the slew, not the motor [STRATEGY §4 G.1]. The slew is also the largest unmodelled term for Humble MPPI (§1.3). | Motor-layer scopes (S3, S4a, S8a) bypass actuator_node. Pipeline scopes report lag **behind the slewed setpoint** (`v_slewed` in wheel_debug). S8 separates M0/M1/M2. |
| **Teensy ramp `M`** (100 RPM per 20 ms = 5000 RPM/s ≈ 1.66 m/s²) | Inactive under the host slew (host rates ≤ 3912 RPM/s), but it ramps direct steps in S3, adding ≈ 0.24 s to a 1200 RPM step; `S` bypasses it. | Set `M` ≥ 1000 for S3/S8a and time from `sp_tx_us`. Restore `M100` afterwards and record `M` in each run's meta. |
| **Battery state** | Bus sag (7–8 V seen on the old stop [BENCH-b b3]) changes delivered speed without voltage compensation (off today [BENCH §2]); SysId notes battery voltage as a hidden variable [C4-S01 p. 13]. | Start each session fully charged. Log `bus_V` per run. Repeat one step and one spin at the end of the session (T11). Decide on voltage compensation from S6 (C1 p20). |
| **Heading-hold interference** | Engages whenever \|ω\| < 0.05 rad/s and \|v\| > 0.02 m/s, and replaces ω with 1.5·yaw_err (clamp 0.75 rad/s) [ACT 444–452]. Under MPPI this silently overrides small ω commands, adds a loop in the cascade [C3-S03] and imports Xsens bias faults. `heading_kp = 0` does **not** disable it: it zeroes ω instead. | Off (`heading_hold_deadband 0.0`) in S3–S9. S10 A/B decides whether it stays on. Log `heading_locked` from wheel_debug. |
| Surface mismatch | Tuning on grass, competing on asphalt: slip parameters learned on one surface gave ≈ 6× larger translational error on another [C2-S33]. | S7/S8 on both surfaces; asphalt values ship. |
| IMU gyro bias | Corrupts K3, the S9 ω truth and heading-hold. | S0 gate before every turning block [STRAT F]. |
| Two writers on the serial port | Apparent motor stepping (memory note). | One owner per test; `fuser` check [RUNBOOK §1]. |
| `/wheel_odom` stamped at host time | Stamp age hides the real measurement age [L5 p1]. | S9 measures age against Teensy `s2_rx_us` through the S0 clock map. |
| Dead MPPI accel params | `ax_max`/`az_max` in [YAML-N] suggest a matched limit that Humble never reads [C4-S26]. | Treat the actuator slew as the only accel limit in every analysis. |
| Small n | 3 repeats resolve only large effects (43/11/5 runs for 0.5σ/1σ/1.5σ [X2-S11]). | Report the uncertainty with each result; add repeats where a result is near its limit. |
