# Drive Tuning and Verification

Motor control, firmware and drive verification for the tracked robot: 2 × REV SPARK MAX + NEO, Teensy 4.1 CAN bridge, ROS 2 actuator_node.

## Current state (2026-09-28)
| Item | State |
|---|---|
| Motor controller firmware | SPARK MAX **26.1.5** (updated from 25.0.4 with REV Hardware Client 2) |
| Teensy firmware | **v2d**, `firmware/teensy_diff_drive_v2/` |
| Controller settings (saved, survive power loss) | brushless, Brake, kV 0.0023 V/RPM, P 0.0002, I 0, D 0, speed filter depth 2 / 16 ms, 50 A limit |
| Host settings | `src/avros_bringup/config/actuator_params.yaml`: kFF 0.0023, kP 0.0002, kI 0, kS_left / kS_right 0.18 V (interim) |
| Bench acceptance | **16 / 16 passed**: `results/ACCEPTANCE_2026_09_28.md` |
| Ground acceptance | **not started**: `GROUND_ACCEPTANCE.md` |
| Ready for MPPI | **not yet**: needs the ground acceptance (on-ground speed/turn accuracy, calibration, stopping distance, MPPI model check) |

## Documents
| File | Purpose | Status |
|---|---|---|
| `README.md` | this index | current |
| `GROUND_ACCEPTANCE.md` | final measurements on the ground, with pass limits | **current: next step** |
| `RUNBOOK.md` | exact commands for bench and ground tests | current |
| `FW26_TEST_STRATEGY.md` | test tiers, controls and status for firmware 26 | current |
| `MPPI_READINESS_TEST_PLAN.md` | the research-based targets behind the pass limits | reference |
| `results/ACCEPTANCE_2026_09_28.md` | bench acceptance report | **current** |
| `results/FW26_BENCH_2026_09_28.md` | all firmware-26 bench measurements and decisions | current |
| `results/raw/` | raw JSON of every bench run and the configuration snapshots | data |
| `STRATEGY.md` | the first tuning plan (firmware 25) | partly superseded |
| `results/BENCH_2026_09_28*.md`, `results/BENCH_SCOPES_2026_09_28.md` | firmware 25 bench results | superseded |
| `FIRMWARE_COMPARISON.md`, `REVIEW.md` | firmware 25 comparison with sparkcan; review of the first plan | historical |
| `tools/` | test and analysis programs (see `tools/bench/README.md`) | current |

## Related
- Firmware and its protocol: `firmware/README.md`, `firmware/teensy_diff_drive_v2/PROTOCOL.md`, `REVIEW.md`
- Firmware research: `research/evidence/firmware_can_review_2026_09_27/` (`REPORT.md`, `FW26_CHANGES_2026_09_28.md`, `KS_AT_ZERO_2026_09_28.md`)
- Background research topics: `research/topics/` (C1 motor control, C2 kinematics, C3 command pipeline, C4 controller interface, L1–L5 localization)

## Key decisions and why
| Decision | Evidence |
|---|---|
| Stop = power 0 with Brake idle, not a speed-0 command | a speed-0 stop drove the controller to full reverse (46–57 A, supply down to 7–8 V, faults, 2+ s of back-and-forth); power 0 + Brake stops in about 0.3 s with no faults |
| P 0.0002, I 0 | P 0.0003+ oscillated with the default filter; I causes windup; feedforward does the main work |
| Speed filter depth 2 / 16 ms | odometry lag 150 → 50 ms; cleanest stops |
| kS sent by the Teensy (not the controller's kS setting) | the controller's kS pushes +kS at a zero command (REV's design); ours is 0 at zero, so it can use the full measured value |
| Speed commands always ramped | instant jumps sag the supply to about 7 V and trip gate-driver faults; ramped starts do not |
| Boot configuration check + motor-type block | the 25 → 26 update silently set motor type to brushed (stalled motors at about 30 A) |
