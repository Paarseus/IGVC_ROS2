# Drive Tuning Log (from 2026-10-02)

The record of the speed-loop tuning: the **baseline** (every parameter and result before tuning), the **method**, and one row per **experiment**. Nothing here changes a value without a row in §4.

Related: `GROUND_TEST_PLAN.md` (what must pass before MPPI), `results/GROUND_2026_10_02.md` (the delivery measurements), `results/ACCEPTANCE_2026_09_28.md` (bench).

---

## 1. Parameter register (baseline, before tuning)

### 1.1 SPARK MAX motor controllers (firmware 26.1.5, saved in flash, read back after a power cycle)
| Parameter (id) | Left / Right | Unit / meaning | Tuned here? |
|---|---|---|---|
| motorType (2) | 1 / 1 | brushless (NEO) | no |
| idleMode (6) | 1 / 1 | Brake | no |
| inverted (45) | 0 / 1 | so that `L+ R+` drives both tracks forward | no |
| **kV** (16) | 0.0023 | feedforward **volts per RPM** of the setpoint; divided by the measured bus voltage to give duty | **yes** |
| **kP** (13) | 0.0002 | **duty per RPM** of error | **yes** |
| **kI** (14) | 0 | duty per (RPM·s) | **yes (if needed)** |
| kD (15), kDFilter (18) | 0, 0 | derivative | no |
| **kIZone** (17) | 0 | RPM. 0 = no zone. When \|error\| > zone the integral is **reset to 0** | **yes (with kI)** |
| **kIMaxAccum** (96) | 0 | limit on the integral term (windup cap) | **yes (with kI)** |
| allowedClosedLoopError (97) | 0 | below this error only feedforward acts | no |
| kS (204), kA (205) | 0, 0 | native static/acceleration feedforward, kept 0 (kS is sent per track by the Teensy) | no |
| outputMin / outputMax (19/20) | −1 / +1 | duty clamp | no |
| closedLoopRamp (114), openLoopRamp (56) | 0, 0 | SPARK-side ramps (off) | no |
| voltCompMode (74), nominalVoltage (75) | 0, 0 | voltage compensation off (kV volts are already divided by bus voltage) | no |
| smartStallA / smartFreeA / smartLimitRpm (59/60/61) | 50 A / 50 A / 10000 | current limit | watch (spin current peaked at 33 A) |
| hallSamplePeriod (136) / hallAvgDepth (137) | 0.016 s / 2 | speed measurement filter (about 50 ms lag) | no |
| posConvFactor / velConvFactor (112/113) | 1 / 1 | units: rotations, RPM | no |
| status0/1/2/7/8 period | 20 / 250 / 20 / 20 / 20 ms | telemetry rates | no |

Control law on firmware 26 (REV, simulated in `research/evidence/firmware_can_review_2026_09_27/references/fw26_2026_09_28/README.md`):
`duty = kI-accumulator + kP × error + (kS_arb·sign + kV × setpoint) / bus_voltage`, with `accumulator += error × kI × dt` while \|error\| ≤ kIZone (or kIZone = 0), otherwise reset to 0.

### 1.2 Teensy firmware v2d (`firmware/teensy_diff_drive_v2/`, unchanged during tuning)
| Constant | Value | Meaning |
|---|---|---|
| control tick / feedback tick | 20 ms / 20 ms | setpoints and wheel telemetry at 50 Hz |
| host watchdog | 300 ms | commands stop, then duty 0 (Brake) |
| MAX_RPM | 4600 | per-track clamp (1.53 m/s) |
| M (speed ramp) | 100 RPM per 20 ms (5000 RPM/s) | default; **set to 20 during tests** (about 0.33 m/s², like `actuator_node`) |
| kS (arbitrary feedforward) | per track, volts, 0 at a zero setpoint; deadband 1 RPM; max 2 V | set by `KSL` / `KSR` |
| stop | `S` and the watchdog: duty 0, Brake | **not changed** |
| duty / voltage cap `MD` | 0.30 default, 0.60 hard | only for the voltage test modes |

### 1.3 Host: `src/avros_bringup/config/actuator_params.yaml`
| Parameter | Value | Tuned here? |
|---|---|---|
| **kFF** (= kV) | 0.0023 | **yes** |
| **kP** | 0.0002 | **yes** |
| **kI / kD / kIZone** | 0 / 0 / 0 | **kI, kIZone yes** |
| **kS_left / kS_right** | 0.18 V / 0.18 V | **yes** |
| track_width_m | 0.7366 m | later (turn calibration) |
| m_per_motor_rev | 0.01994 m | later (distance test) |
| wheel_separation_multiplier | 1.19 | later (turn calibration) |
| max linear / angular speed | 1.5 m/s / 1.5 rad/s | no |
| accel / decel / angular slew | 0.3 m/s² / 1.3 m/s² / 1.2 rad/s² | no |
| heading_hold_deadband / heading_kp | 0.05 rad/s / 1.5 | no (off for tests) |
| cmd_timeout_s, control_rate_hz, state_pub_rate_hz | 0.5 s, 50 Hz, 20 Hz | no |

---

## 2. Results before tuning

### 2.1 Bench, tracks off the ground (2026-09-28): 16 of 16 passed
Speed within 5 % from 1000 to 3500 RPM, slow speeds (0.033–0.1 m/s) within 10 %, braked stops under 0.11 s without faults, odometry lag 50 ms. Index: `results/ACCEPTANCE_2026_09_28.md`.

### 2.2 Ground, concrete sidewalk (2026-10-02)
**Electrical:** no controller faults in 26 delivery runs; bus never below 10.1 V; peak current 27 A straight, 33 A in spins. Instantaneous voltage steps (2 V then +1.5 V twice) sagged the bus to 8.3 V at 39 A per motor, so only ramps are used.

**Braked stop (`S`) from speed:** stops in 0.2–0.8 s over 2.5–8 cm, rollback under 8 mm, but 0.7–1.2 g of deceleration and one gate-driver fault flag (right controller, 1.0 m/s reverse). Left unchanged by decision.

**Slow voltage ramps (1 repeat):** kV about 0.0021 V/RPM (in use 0.0023); kS about 0.36 V forward, 0.26 V reverse (in use 0.18 V). Preliminary.

**Speed delivery, baseline gains (kV 0.0023, kP 0.0002, kI 0, kS 0.18), 5 repeats, mean error of the steady speed vs command:**
| Command | Forward L / R | Reverse L / R |
|---|---|---|
| 0.05 m/s | −32.6 / −30.5 % | −11.2 / −8.4 % |
| 0.10 m/s | −12.9 / −11.5 % | −3.3 / −1.9 % |
| 0.30 m/s | −1.0 / −0.5 % | +1.8 / +2.4 % |
| 0.50 m/s | +0.3 / +0.9 % | +2.3 / +3.4 % |
| 0.70 m/s | +1.9 / +2.2 % | +3.5 / +3.9 % |

Left/right match within 0.3–1.1 % at 0.3 m/s and above; speed noise 10–15 RPM (the encoder resolution), no oscillation.

**Slow spins in place (3 repeats):** ω = 0.3 rad/s delivered **60–70 %** of the commanded track speed (−31 to −43 %); ω = 0.1 rad/s: **tracks did not move**.

---

## 3. Method

### 3.1 Principles
1. **One parameter at a time.** Every experiment changes exactly one thing from a named reference set; everything else is read back from the controllers and logged.
2. **Same protocol, same conditions.** Concrete sidewalk, same direction pairs, `M20`, same speeds, 3 repeats (5 for the final set). The first experiment (E0) re-measures the baseline with the new protocol, so later changes are compared with a fresh reference and the run-to-run noise is known.
3. **Decide with numbers.** The metrics and pass limits below are fixed before the runs. A change is kept only if it improves the target metric by more than the 95 % band and does not worsen another.
4. **Reversible.** All experiment changes are RAM-only on the Teensy and SPARKs. The yaml and the saved (flash) values change only at the end, and flash (BURN) only with your approval.

### 3.2 Protocol (per parameter set; `tools/ground/tune_set.sh`)
| Part | Runs per repeat | What it measures |
|---|---|---|
| Straight A (forward and reverse) | 0.05, 0.10, 0.20, 0.30 m/s, 3.5–4 s each | crawl and low-speed delivery |
| Straight C (forward and reverse) | 0.50 then 0.70 m/s, 3.2 s each | cruise delivery |
| Spin (ccw and cw) | ω 0.1, 0.3, 0.6 rad/s, 6 s each | delivery under scrub load |

Metrics from `analyze.py delivery`: mean error vs command with its 95 % band, worst run, ripple (speed noise), stuck share, left/right match, and the **stop behaviour** (opposite-direction excursion and settling time after the final slow-down).

### 3.3 Guards (any one aborts the set and restores the baseline gains)
controller fault flag, bus voltage below 9.5 V, motor current above 45 A, speed ripple above 60 RPM (oscillation).

### 3.4 Targets for the speed loop
| Condition | Limit |
|---|---|
| 0.3 m/s and above, straight, both directions, both tracks | within ±2 % |
| 0.05 and 0.10 m/s, straight | within ±10 %, never stuck |
| spins 0.1 / 0.3 / 0.6 rad/s | within ±10 % (the multiplier absorbs the rest) |
| ripple | no more than 1.5× the baseline; no oscillation |
| left/right match | within 2 % |
| stops | no opposite-direction excursion beyond 30 RPM |

### 3.5 Experiment order
| ID | Isolated parameter | Reference | Purpose |
|---|---|---|---|
| E0 | none | baseline | new-protocol reference and noise |
| E1 | kP = 0, kI = 0 (pure feedforward) | baseline | identify true kV and kS per track and direction (closed form) |
| E2 | kV and kS set from E1, kP = 0 | E1 | confirm the feedforward alone delivers; keep one value per track |
| E3 | kP sweep | E2 | most delivery without oscillation (final = 0.7 × oscillation onset) |
| E4 | kI with kIZone and kIMaxAccum | E3 | spin delivery, only if E3 leaves spins out of limit; check stops for windup |
| E5 | none (final set) | E4 | 5-repeat confirmation, then yaml |

---

## 4. Experiment log
| ID | Date | Changed (from reference) | Result | Decision |
|---|---|---|---|---|
| baseline | 2026-10-02 | see §1, §2 | delivery table above | reference for E0 |
| E0 | 2026-10-02 20:34 | nothing (baseline gains, new protocol, 3 repeats; `S_ccw_r3` excluded) | Reproduces the first campaign. **Forward** 0.3 / 0.5 / 0.7 m/s: −1.4…−1.8 / +0.4…+0.9 / +1.4…+2.0 % (pass). **Reverse** 0.3 / 0.5 / 0.7: +1.7…+2.5 / +3.1…+3.6 / +3.8…+4.5 % (fail). **Crawl** forward 0.05 / 0.10 m/s: −35 / −16 % (fail); reverse −20…−17 / −8…−5 % (0.05 fails). **Spins** 0.6 / 0.3 / 0.1 rad/s: −13…−17 / −32…−37 / 100 % (stuck). Ripple 10–23 RPM. Stops (smooth slow-down then S): rollback ≤ 3.3 mm, pass. Left/right within 0.8 % at speed, up to 3 % in reverse crawl. | reference for all later sets |
| incident | 2026-10-02 20:41 | none (user pressed the e-stop during `S_ccw_r3`) | Both tracks' position counters stepped by about −5000 rotations in one sample: the **motor drivers were power-cycled** by the e-stop, which resets the counters **and any gains held in RAM on the SPARKs to their saved values**. Run excluded from E0. | Guard added: a position step above 20 rotations per sample marks the run void and aborts the set (no automatic re-drive); resume with `REP_FROM=<rep>`. Gains are re-verified at the start of every set. |
| E1 | 2026-10-02 20:44 | kP 0 and kI 0 (pure feedforward; kV 0.0023, kS 0.18 unchanged) | Straight runs, 3 repeats, 12 runs, no faults. Closed-form fit of delivered vs commanded speed (r² 0.9993–0.9999): **kV true 0.00210 / 0.00209 / 0.00214 / 0.00209 V/RPM** (L fwd / L rev / R fwd / R rev; free NEO 12/5676 = 0.00211), **kS true 0.398 / 0.406 / 0.405 / 0.382 V**. kS is the same in both directions (about 0.40 V); the earlier slow-ramp fit (0.36 forward, 0.26 reverse) was biased by ramp acceleration and is superseded. Recommended kV 0.00211, kS_left 0.402, kS_right 0.394. | use for E2 |
| E2 | 2026-10-02 20:49 | kV 0.0023 → **0.00211**, kS_left 0.18 → **0.40**, kS_right 0.18 → **0.39** (P and I still 0; spins included) | 3 repeats, 18 runs, no faults. **Reverse**: 0.7 / 0.5 / 0.3 m/s +1.4…+1.7 / +1.7…+2.1 / −0.1…+0.1 %; 0.1 m/s +4.5…+6.0 %; **0.05 m/s −1.9 / −1.2 % (was −20 / −17 %)**. **Forward**: left −0.6 / −0.4 / −2.3 %, right −2.7 / −2.8 / −6.1 % at 0.7 / 0.5 / 0.3; 0.1 m/s −12 / −19 %; 0.05 m/s −30 / −35 % (large scatter). Measured errors at ≥ 0.5 m/s are within about 1 % of the E1 model. The forward-minus-reverse difference is a **constant in RPM** (about 20 RPM left, 35 RPM right at every speed), i.e. a constant extra force of about 0.04–0.07 V in the forward direction (sidewalk slope and/or direction-dependent friction). **Spins** (feedforward only): 0.6 rad/s −23…−42 %, 0.3 rad/s −54…−72 %, 0.1 rad/s stuck. Stops: rollback ≤ 4.3 mm. | feedforward adopted as the base for E3; remaining errors are disturbances for P / I |
| E3a | 2026-10-02 20:58 | kP 0 → **0.0004** (feedforward as E2) | 2 repeats, 12 runs, no faults, ripple ≤ 26 RPM (no oscillation), bus ≥ 9.7 V, peak 40 A (spins). **Reverse**: within ±3 % at every speed, 0.7–0.2 m/s within ±1.1 %. **Forward**: left −0.5…−1.0 % at 0.3–0.7 m/s, right −3.1 / −2.2 / −1.4 % at 0.3 / 0.5 / 0.7; crawl 0.05 m/s −12 / −20 % (was −30 / −35 %). **Spins**: 0.6 rad/s −8…−16 %, 0.3 rad/s −16…−29 %, 0.1 rad/s still partly stuck. Stops clean. | P helps in every condition; continue upward (E3b 0.0008) |
| E3b | 2026-10-02 21:04 | kP 0.0004 → **0.0008** | **Aborted by the guard on the first run** (forward, 0.05–0.3 m/s): bus **8.0 V**, current **52 A**, speed noise **81 RPM** (oscillation), peak 1071 RPM against a 903 RPM command (19 % overshoot). Baseline gains restored and verified by the harness. The Jetson stayed up. | **kP 0.0008 rejected.** Oscillation onset lies between 0.0004 and 0.0008 |
| E3c | 2026-10-02 21:06 | kP 0.0008 → **0.0006** (probe, 1 repeat, straight) | First two runs clean (bus ≥ 10.2 V, ≤ 29 A, noise ≤ 22 RPM). **Aborted on the cruise run (0.5 → 0.7 m/s): bus 9.0 V** at 33 A, noise 32 RPM, no faults. Resting bus 12.12 V afterwards (battery not the cause). Minimum bus in the cruise run vs kP: **10.3 V (0.0002), 10.2 V (0), 9.9–10.1 V (0.0004), 9.0 V (0.0006)**. | **E3 conclusion: kP = 0.0004.** Oscillation onset is above 0.0006 and below 0.0008, but the supply dip limits kP first; 0.0004 is the highest value with bus ≥ 9.7 V in every run |
| E4a | 2026-10-02 21:08 | kI 0 → **0.0001**, kIZone 0 → 700 RPM, kIMaxAccum 0 → 0.08 (kP 0.0004, feedforward as E2) | **Aborted on the first run and the robot was shut down by the operator** (first run lasted 11.3 s of 21 s). Crawl 0.05 m/s: speed swung between 22 and 491 RPM against a 150 RPM command, duty flipped between −0.16 and +0.32 (**reversing**), current 51 A, bus 8.5 V, noise 102 RPM, travel 0.7 m instead of 2.3 m. This is the classic integrator limit cycle with stiction: the integral winds up while the track sticks, then breaks away and overshoots. The abort path restored the gains; the `kIMaxAccum` restore got no reply (drivers had lost power, so it reverts to the saved 0). | **kI 0.0001 rejected** (zone 700 RPM is far too wide for the crawl). Any further integral test needs a much smaller kI, a narrow zone and a small cap, and only with the operator ready |

---

## 5. Status snapshot (2026-10-02, 21:30)

**Best verified set (RAM only, not applied):** kV 0.00211 V/RPM, kS_left 0.40 V, kS_right 0.39 V, kP 0.0004, kI 0 (E1 → E3a). **The robot and `actuator_params.yaml` still hold the baseline** (kV 0.0023, kS 0.18 / 0.18, kP 0.0002, kI 0); the controllers also revert to their saved values after any power cycle.

| Area | State |
|---|---|
| Speed delivery, best set | reverse within ±3 %; forward 0.3–0.7 m/s: left −0.5…−1.0 %, right −1.4…−3.1 %; crawl 0.05 m/s forward −12 / −20 %; spins 0.3 rad/s −16…−29 %, 0.1 rad/s partly stuck |
| Limits found | kP above about 0.0004 sags the bus (9.0 V at 0.0006; oscillation, 52 A and 8.0 V at 0.0008); integral term with a 700 RPM zone limit-cycles (E4a, operator shut the robot down) |
| Open | right track is the weak one (friction about 8–10 % higher, noisier, stalls more, bearing failure in May at kP 0.0012): inspect before more tuning; which motor took the 51–52 A peaks in E3b/E4a is not yet checked |
| Not done | friction map and feedforward correction table (`TUNING_TEST_PLAN.md` phases B–C), bounded integral test, ground-truth (tape) check, confirmation through `actuator_node`, turn calibration |
| Data | raw runs for E0–E4a are on the Jetson only (`~/ground_tests/2026-09-30_1059_concrete/`): **copy to the laptop when it is reachable** |
| Decision pending | adopt the best set in `actuator_params.yaml` (yes/no) before the navigation tests |
