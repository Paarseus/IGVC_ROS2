# Bench Acceptance Report (2026-09-28)

**Result: 16 of 16 checks passed.** The firmware, motor controllers and actuator_node path are ready for the ground acceptance tests (`../GROUND_ACCEPTANCE.md`).

**Configuration under test:**
- **Motor controllers:** SPARK MAX firmware 26.1.5 (both). Brushless, Brake, kV 0.0023 V/RPM, P 0.0002, I 0, D 0, speed filter depth 2 / 16 ms, 50 A current limit.
- **Teensy firmware:** v2d.
- **Host:** `actuator_params.yaml` (kFF 0.0023, kP 0.0002, kI 0, kS_left/right 0.18 V).

**Conditions:** tracks off the ground, battery about 12 V.
**Run with:** `tools/bench/run_acceptance.sh`. Raw results: `raw/acceptance_serial.json`, `raw/acceptance_pipeline.json`.

## Firmware and motor layer (Teensy direct)
| ID | Check | Measured | Limit | Result |
|---|---|---|---|---|
| A1 | configuration check at the controllers | `CHK OK` | `CHK OK` | PASS |
| A2a | speed/position report interval | 20.0 ms mean, 0 gaps > 30 ms | 20 ± 1 ms, 0 gaps | PASS |
| A2b | speed command interval | 20.0 ms, max 20.0 ms | max ≤ 25 ms | PASS |
| A2c | watchdog stop after commands stop | 257 ms | 250–350 ms | PASS |
| A3 | disable test (power streamed, heartbeat disabled) | 0 RPM, 0.0 A | ≤ 10 RPM, ≤ 0.5 A | PASS |
| A4 | input limits: speed, power, voltage, invalid number | 4600 RPM / 0.30 / 3.6 V / 0 | same | PASS |
| A5 | speed accuracy 1000–3500 RPM, both directions | worst 4.8 %, noise σ 24 RPM | ≤ 5 % (off the ground), σ ≤ 40 RPM | PASS |
| A6a | slow speed 100–300 RPM (0.033–0.1 m/s), kS 0.18 V | worst 4.7 % | ≤ 10 % | PASS |
| A6b | crawl 50 RPM (0.017 m/s), kS 0.18 V | never stuck; error 13 % (reported) | never stuck | PASS |
| A7 | braked stop from 2000/3500 RPM (4 stops) | ≤ 0.11 s, supply ≥ 11.2 V, no faults | ≤ 0.40 s, ≥ 10 V, no faults | PASS |
| A8 | ramped start 0 → 3500 RPM | supply ≥ 10.8 V, peak 15 A, no faults | ≥ 9.5 V, no faults | PASS |

## Full command path (`/cmd_vel` → actuator_node → Teensy → controllers)
| ID | Check | Measured | Limit | Result |
|---|---|---|---|---|
| P1 | stops (0.6 m/s timeout, 0.6 and 1.0 m/s to 0, turn to 0) | worst dip −18 RPM, 0 reversals, still in 0.92 s | ≥ −100 RPM, 0 reversals, ≤ 1.3 s | PASS |
| P2 | turn reversal +0.8 ↔ −0.8 rad/s | 0 direction changes beyond the commanded one | 0 | PASS |
| P3 | `/cmd_vel` to motor command | median 11 ms, max 18 ms | max ≤ 40 ms | PASS |
| P4 | odometry speed lag (what MPPI receives) | 50 ms, 20 Hz | ≤ 60 ms (target 50), ≥ 20 Hz | PASS |
| P5 | slow speed via `/cmd_vel`: 0.05 / 0.1 m/s, ±0.1 rad/s | worst 5 %, never stuck | ≤ 10 %, never stuck | PASS |

## Persistence (separate run, same configuration)
After a power cycle (both controllers reported HAS_RESET): boot `CHK OK`, and **0 of 37 settings differ** from the saved configuration (`raw/s2_manifest_fw26_final.json` vs `raw/s2_manifest_fw26_after_powercycle.json`).

## What changed during acceptance
| Finding | Action |
|---|---|
| Slow speed failed with kS = 0 through actuator_node (31 % error, sticking at 0.05 m/s and 0.1 rad/s) | `kS_left`/`kS_right` set to **0.18 V** (bench-proven: 5–8 %, never stuck). **Interim value**; replaced by the ground fit (G1) |
| kS 0.22 / 0.26 V overshot slow speeds by 15–40 % | 0.18 V kept |
| Two checks first failed because of measurement bugs in the test script: stop direction read from a single sample; the commanded direction change counted as an error | both metrics fixed; the robot behaviour was unchanged (dip −18 RPM, 0 extra direction changes) |

## Not covered by bench acceptance
Everything that needs load, friction or ground truth: speed delivery ±2 % on the ground, distance and turn calibration, stopping distance, low battery, and the MPPI model and end-to-end tests. These are in `../GROUND_ACCEPTANCE.md`.
