# Bench Test Programs

All run on the Jetson with the tracks off the ground. Stop the web UI / actuator_node first (`RUNBOOK.md` §2), except for the `run_*.sh` scripts, which do it themselves. Settings are changed in RAM only and restored to the final configuration afterwards; nothing is saved unless the script says so. Results go to `~/bench_scopes_2026_09_28/`.

| Program | Purpose | Test IDs |
|---|---|---|
| `run_acceptance.sh` | **full bench acceptance** (both parts, pass/fail); `run_acceptance.sh part2` runs the actuator_node part only | A1–A8, P1–P5 |
| `acceptance_serial.py` | acceptance part 1: firmware and motor layer | A1–A8 |
| `acceptance_pipeline.py` | acceptance part 2: `/cmd_vel` → actuator_node path | P1–P5 |
| `run_lowspeed.sh` | low-speed kS sweep (no argument) or low speed through actuator_node with kS 0 vs a value (`run_lowspeed.sh 0.18`) | T2.6, P5 |
| `bench.py` | send any Teensy commands and print the replies, e.g. `python3 bench.py CHK "#sleep 3"` | — |
| `tio.py` | shared serial helper (telemetry parsing, final configuration) | — |
| `scopes.py` | single isolated scopes: timing, feedforward-only, speed grid, filter sweep, low speed, saturation, latency | S0–S5, T3 |
| `fw26_units.py` | firmware-26 feedforward units test | T2.1 (B1) |
| `fw26_tests.py` | kS sign, kS stacking, kA, status reports | T2.3–T2.5, T3.5 |
| `fw26_safety_b8.py` | disable test, input limits, speed-filter period | T1.2, T1.4, T3.4 |
| `ks_zero_variants.py` | controller kS at a zero command: controlled variants | kS-at-zero study |
| `stop_fw26.py` | stop methods and faults (brake / coast / ramp-then-brake) | T4.2, T4.4 |
| `pipeline_scopes.py`, `actuator_stop_test.py` | actuator_node path: latency, odometry, turn reversals, stops | T4.3, T5 |
| `run_pipeline_fw26.sh` | compare P / filter settings through actuator_node, e.g. `run_pipeline_fw26.sh 0.0002_2_0.016` | T3 decision |
| `historical/` | firmware-25 one-off scripts. **Do not run `historical/run_stop_comparison.sh`: it flashes old firmware** | — |

Field tools (ground tests) are one level up: `../drive_tuner.py` (runs tests, logs CSV) and `../analyze.py` (feedforward fit, lag, steps, distance, turn calibration).
