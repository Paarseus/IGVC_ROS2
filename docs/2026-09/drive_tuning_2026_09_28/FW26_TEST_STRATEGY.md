# Test Strategy: SPARK MAX Firmware 26.1.5 + Teensy Firmware v2d

**Date:** 2026-09-28. **Goal:** prove, one isolated and controlled test at a time, that the drive stack is correct on SPARK firmware 26.1.5, then tune it for MPPI.

**Inputs:**
- Research: `research/evidence/firmware_can_review_2026_09_27/FW26_CHANGES_2026_09_28.md` (§9 has each bench test's full inputs and expected results; this plan orders them and adds controls and gates).
- MPPI targets: `MPPI_READINESS_TEST_PLAN.md`.
- Earlier results: `results/BENCH_*.md`.

## Status (2026-09-28, end of bench day)
| Tier | Status |
|---|---|
| T0 config | ✔ `CHK OK` after fixes (motor type, idle, kV) |
| T1 safety | ✔ interlock (T1.1), HB0 (T1.2), watchdog (T1.3), clamps (T1.4) |
| T2 feedforward | ✔ B1 kV volts, B3 native kS unsafe at 0, B5 additive, B4 kA ignored, T2.6 arbFF kS works; B2 part 2 open (second battery level) |
| T3 speed loop | ✔ FF delivery, P/depth/period chosen (P 0.0002, depth 2, 0.016 s), STATUS_7/8, saturation |
| T4 stops | ✔ brake/coast/ramp-brake 0/18 faults; fault root cause = instant setpoint steps |
| T5 timing | ✔ 20.0 ms, 0 gaps, cmd→motion ≤ 40 ms, odom lag 50 ms |
| T6 persistence | ✔ final config BURNed + `CHK OK`; **T6.3 ✔: after a power cycle (HAS_RESET on both) boot `CHK OK` and 0 of 37 parameters differ from the final snapshot** |
| T7 host changes | ✔ yaml gains + kS L/R push (verified acks); kS_left/right = 0.18 V interim after low-speed acceptance |
| **Bench acceptance** | **✔ 16/16** (`results/ACCEPTANCE_2026_09_28.md`) |
| T8 ground | not started: `GROUND_ACCEPTANCE.md` |
Details: `results/FW26_BENCH_2026_09_28.md`.

## 1. Rules for every test (the controls)
| Rule | Why |
|---|---|
| **One variable per test.** Everything else is held at a recorded baseline (the `CHK` output plus the S2 manifest) | So a result can only come from the variable under test |
| **RAM only**, and the baseline is restored and confirmed (`PWR res=0`) at the end of each test | So tests cannot contaminate each other. Flash writes only in the persistence tier (T6) |
| **One writer on the serial port** (actuator_node stopped for firmware-level tests) | Two writers cause false steps (memory: serial collision) |
| **Tracks off the ground** for T1–T6; the ground only in T8 | Separates motor and controller behaviour from terrain |
| **Teensy timestamps** (`X` line µs, CAN arrival time) as the time reference, never host arrival | S0 showed ±2.8 ms host jitter |
| **Speed from wheel position**, not the SPARK's filtered speed; ramped setpoints (M100 or the actuator slew) for step metrics | The filtered speed lags 70–150 ms; instant steps overshoot 85–135 % |
| **Every result is logged** (raw JSON on the Jetson in `~/bench_scopes_*`), and pass/fail against a stated criterion is written in `results/` | Traceability |
| **Stop conditions:** current > 40 A at < 20 % duty with speed ≈ 0 (stall), a DRV fault, or bus < 9 V → stop, `S`, investigate | Today's brushed-mode stall showed why |

## 2a. Results after the motor-type fix (2026-09-28)
- **Motor type set to 1 (brushless) over CAN** on both (confirmed `PWR res=0`, read back 1). The interlock released. At 5 % duty: 169 / 183 RPM at 0.2 A (it was about 30 A stalled).
- **Idle mode = Brake on both** (`CHK B idleMode L=1 R=1 ok`). **Saved with BURN: `result L=0 R=0`.** CHK now fails only on `kV<5e-4`, which is expected until kV is retuned.
- **T2.1 / B1 kV units: ✔ volts per RPM on FW 26.1.5.** P = I = D = 0, kS 0, 1000 RPM command:

| kV (param 16) | Applied duty | Bus V | Applied volts | Speed L / R |
|---|---|---|---|---|
| 0.000197 (the FW 25 value) | 0.0164 / 0.0166 | 11.98 / 11.82 | 0.197 V | **0 / 0** (below friction) |
| 0.00211 | 0.179 / 0.181 | 11.76 / 11.66 | 2.10 V | 918 / 928 RPM |
| 0.00236 | 0.201 / 0.203 | 11.74 / 11.61 | 2.36 V | 1037 / 1048 RPM |

- **T2.2 / B2 (partial): the controller divides the feedforward volts by the MEASURED bus voltage.** Duty 0.179 × 11.76 V = 2.104 V = kV × 1000, and dividing by a fixed 12 V would give 0.176. So feedforward on FW 26 is battery-independent. A second battery level will confirm it.
- Off the ground, the effective kV ≈ **0.00225–0.00229 V/RPM** (including friction). The ground value comes from Session 1.

## 2. Blocker found before any motion test
**Motor type (param 2) = 0 (brushed) on both controllers after the update.** An 8 % duty command drew about 30 A into stalled NEOs, with position and speed at 0. It must be set to **1 (brushless)** before any motion test; it is a protected parameter, and the owner decides how. v2d adds an interlock that blocks motion whenever motor type ≠ 1, and a `CHK` command that checks it at every boot.

## 3. Tiers and gates (each tier must pass before the next)

### T0 Configuration check (read-only; runs now)
| Test | Method | Pass | Status |
|---|---|---|---|
| T0.1 firmware | `FV` | 26.1.5 on L and R | ✔ 26.1.5 / 26.1.5, model 2 (MAX) |
| T0.2 motor config | `CHK` (v2d, runs automatically at boot) | `CHK OK` | **✘ `CHK FAIL L:motorType=0 L:kV<5e-4 R:motorType=0 R:kV<5e-4 idleMode_L!=R`** (v2d boot, 2026-09-28): it caught all three open problems |
| T0.3 manifest diff | read all 31 table params, diff against `s2_manifest_baseline.json` | only intended differences | idle mode L changed 0 → 1 during the update; everything else unchanged |
| T0.4 B8 raw param 136 | `PR B 136` | record raw hex | ✔ raw `0x3EA00000` = 0.3125 on both (REVLib default is 0.03125 = `0x3D000000`); units unconfirmed, **do not write until T3.4** |
| T0.5 B7 status timing | 20 s of STATUS_2 at idle | no 40 ms gaps | ✔ 1000 of 1000 intervals at 20.0 ms (p99 20.1). The 26.1.3 fix is confirmed; FW 25 had about 1 % gaps |

### T1 Safety (tracks off the ground; T1.1 runs before and after the motor type fix, T1.2–T1.5 after)
| Test | Method | Pass |
|---|---|---|
| T1.1 v2d interlock **✔ PASS (before the fix)**: streamed duty 0.05, velocity 300 RPM and voltage 0.6 V; applied 0.000, current 0.0 A, speed 0 on both sides; `blkn=1090/1090` frames converted to duty 0. Boot message `!! MOTOR TYPE 0 … motion blocked` within the first second. **After the fix:** repeat with `CHK OK`. | Before the motor type is fixed: `CHK` must report `FAIL motor type 0` and any motion command must be blocked, so there is **no** current draw. This is a safe, direct test of the interlock on today's real fault. After the fix: `CHK OK` and motion allowed | blocked with type 0; allowed with type 1 |
| T1.2 B10 disabled heartbeat | `HB0` while streaming 6 % duty | tracks stop ≤ 0.3 s, applied 0 |
| T1.3 watchdog | stream, then silence | stop to idle about 300 ms after the last command |
| T1.4 limits and clamps | `L9000`, `UL0.9`, `UVL20`, NaN inputs | clamped values acknowledged |
| T1.5 stall guard (manual) | watch current in every motion test | no stall signature |

### T2 Units and feedforward on FW 26 (one term at a time)
| Test | Variable | Method (details: FW26 §9) | Pass / decision |
|---|---|---|---|
| T2.1 **B1 kV units** | param 16 | P = I = D = 0, all kS = 0; kV 0.000197 at 1000 RPM; only if about 0 RPM, then kV 0.00236 | volts: 0 then about 1000 RPM → **retune kV ≈ 0.00236** |
| T2.2 B2 bus-voltage division | battery charge; voltage compensation on/off | applied × Vbus at a fixed kV, two charge levels; then 74 = 2, 75 = 11 | applied × Vbus constant → feedforward is battery-independent |
| T2.3 B3 kS sign and zero | param 204 | 204 = 0.30 V; setpoints +100, −100, **0** | expect +kS at 0 (no deadband): **keep kS in the arbitrary feedforward (v2c/v2d), param 204 = 0** |
| T2.4 B5 arbFF + kS | both | 204 = 0.30 and `KS0.30` together | additive → confirms "one source only" |
| T2.5 B4 kA | param 205 | kA only, ramped setpoint | ignored in velocity mode → do not use 205 |
| T2.6 kS value (off the ground) | `KS` | low-speed scope (50–300 RPM) with kS 0 / 0.18 / 0.30 V | tracking within ±3 %, no sticking (as on FW 25; retest on FW 26) |

### T3 Speed loop and measurement (single motor, isolated)
| Test | Variable | Method | Pass |
|---|---|---|---|
| T3.1 FF-only delivery | — | `scopes.py s3ff` with kV from T2.1 | ratio 0.95–1.05 across 1000–3500 RPM, both directions |
| T3.2 P sweep | kP | ripple and ramped-step overshoot, P 0.0001–0.0005 | pick P = 0.7 × the oscillation onset |
| T3.3 low speed | kS | `scopes.py s3low` | T7: 0.05 m/s with no stick-slip |
| T3.4 **B8 filter** | 137, then 136 | step-lag test: depth 3 → 1; then 136 = 0.03125 (the documented default) vs the current 0.3125 | 137 honoured (lag 150 → about 50 ms); decide whether 136 is really seconds |
| T3.5 STATUS_7 / STATUS_8 (v2d) | — | compare the decoded setpoint and I accumulator with the commands | setpoint matches, I = 0 with kI = 0 |
| T3.6 saturation | — | 4600 RPM | reached with headroom, no DRV fault |

### T4 Stopping and transitions
| Test | Method | Pass |
|---|---|---|
| T4.1 B12 idle mode | write **both** sides to the chosen mode (Brake), read back | L = R |
| T4.2 stop comparison | `stoptest.py` equivalents on FW 26 | duty 0 + Brake: no reversals, ≤ 0.3 s, no DRV |
| T4.3 pipeline stops and ω flips | `run_stop_comparison.sh` / `pipeline_scopes.py` | ≤ 1 reversal, standstill ≤ 1.2 s |
| T4.4 high-speed brake | stop from 4600 RPM through the watchdog path | no DRV fault. Otherwise make the watchdog path ramp or coast |

### T5 Timing and latency (repeat on FW 26)
`scopes.py s1`, `s5`, `pipeline_scopes.py`: 50 Hz loop, CAN rates, RTT, command → motion ≤ 100 ms, `/wheel_odom` rate and lag (currently 130–170 ms, a code fix is pending).

### T6 Persistence (writes flash; last)
| Test | Method | Pass |
|---|---|---|
| T6.1 B13 (optional) | one PERSIST with the enabled heartbeat | refused (non-zero) |
| T6.2 final configuration BURN | write the decided values (motor type 1, Brake on both, current limit, kV, P, I = 0, IZone 0, 204 = 0), then BURN | `result 0/0` |
| T6.3 power-cycle diff | power-cycle, `CHK`, manifest diff | 0 unintended differences, HAS_RESET seen |

### T7 Tooling and code changes validated on the bench
- **v2d:** STATUS_7/8 in the X line, `CHK`, the interlock, the KA warning. Also host changes: actuator_params kV in V/RPM, kS_left/right pushed at startup, and position-derived 50 Hz `/wheel_odom`. Each is validated with the relevant T2–T5 test before and after.

### T8 Ground (after T0–T7)
This is the ground part of `MPPI_READINESS_TEST_PLAN.md`:
- identification per side (kS, kV) on the competition surface;
- stopping distance ≤ 0.30 m;
- v/ω delivery ±2 % / ±3 %;
- distance and turning calibration;
- the MPPI model check;
- end-to-end with the heading-hold A/B.

Asphalt first (IGVC 2026 surface), then grass.

## 4. What changes between FW 25 and FW 26 for tuning
| Item | FW 25.0.4 | FW 26.1.5 |
|---|---|---|
| param 16 | kF, duty/RPM (0.000197 correct) | **kV, V/RPM (0.000197 is about 12× weak; start at about 0.00236)** |
| kS | no parameter (arbFF workaround) | param 204 exists, but applies +kS at a zero setpoint → keep the arbFF kS |
| kA | none | only in MAXMotion |
| feedforward vs battery | duty, so it weakens as the battery sags | volts ÷ measured bus voltage → battery-independent (T2.2 confirms) |
| status jitter | about 1 % 40 ms gaps | none (T0.5) |
| new diagnostics | — | STATUS_7 I accumulator, STATUS_8 setpoint / at-setpoint |
| tools | Hardware Client 1 | Hardware Client 2 only (USB is SLCan) |
