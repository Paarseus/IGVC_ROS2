> **Superseded (2026-09-29) by `GROUND_TEST_PLAN.md`.** Kept for history.

# Ground Acceptance: Final Measurements Before MPPI

**Purpose:** prove on the ground that the drive system does what the MPPI controller assumes. The bench acceptance (`results/ACCEPTANCE_2026_09_28.md`) proves the firmware and motor layer; these tests add weight, ground friction and the real chassis.

**Surfaces:** asphalt first (the IGVC 2026 AutoNav surface), then grass. Each test is run on both unless marked.
**Pass limits** come from `MPPI_READINESS_TEST_PLAN.md` §1.3 (MPPI plans 2.8 s ahead; IGVC passages leave about 0.35 m of room per side).
**Commands and tools:** `RUNBOOK.md`.

## Before every session (G0)
| Check | Pass |
|---|---|
| Teensy boot line | `CHK OK` |
| RTK | FIXED (GGA quality 4) for the whole run |
| IMU | one warm-up drive done; gyro still-bias ≤ 0.1 °/s |
| Battery | voltage recorded at the start and end |
| heading-hold | off (`heading_hold_deadband 0.0`) for G1–G7 |

## Tests
| ID | What is measured | How | Pass limit | Sets / decides |
|---|---|---|---|---|
| **G1** | motor model per track, on the surface | voltage ramps and steps, forward and reverse, 3 repeats (`drive_tuner.py ramp/steps`, `analyze.py ff`) | fit quality ≥ 0.9 | `kFF` (kV) if it differs > 5 % from 0.0023; `kS_left` / `kS_right` |
| **G2** | speed delivery | `/cmd_vel` **0.05**, 0.1, 0.3, 0.5, 0.7, 1.0 m/s through actuator_node, 3 repeats | track speed within **±2 %** at 0.3–1.0 m/s; **within ±10 % and no stick-slip at 0.05 and 0.1 m/s** | confirms G1 values |
| **G3** | distance scale and straightness | 10–15 m straight at 0.5 and 1.0 m/s, both directions, 3 repeats, RTK (`analyze.py distance`) | distance scale error ≤ **1 %**; end offset ≤ **5 cm** | `m_per_motor_rev` |
| **G4** | turn rate | spins **0.1**/0.3/0.6/1.0 rad/s (0.1 = slow alignment turns) and arcs of 1/2/4 m radius, both directions, 3 repeats, gyro (`analyze.py turn`) | turn rate within **±3 %** of command after calibration | `wheel_separation_multiplier` (the old 1.19 partly compensated weak motors) |
| **G5** | stopping | 0.7 m/s then command 0 through actuator_node, 5 repeats | distance ≤ **0.30 m**; no backward motion; no faults | confirms decel limit |
| **G6** | electrical endurance | 6 minutes of mixed driving, then one G2 step and one G4 spin at low battery | no faults; motor temperature ≤ 60 °C; G2/G4 results change ≤ 2 % | confirms feedforward is battery-independent |
| **G7** | the model MPPI assumes | IGVC-like command sequences through actuator_node; compare commanded and delivered v and ω over 2.8 s windows | effective delay ≤ **0.28 s** (goal 0.14) per axis; predicted vs actual path ≤ **0.10 m** (95th percentile) | whether to change the actuator slews and MPPI `wz_max` (1.9 > the 1.5 cap) |
| **G8** | end to end | Nav2 MPPI waypoint course with obstacles, heading-hold off vs on, 3 runs each | all waypoints reached; path error within the lane margin; no stalls | heading-hold setting for Nav2 |

## Ready for MPPI when
1. G0–G6 pass on asphalt (and on grass, for grass events).
2. G7 passes, or the model mismatch is fixed and G7 is repeated.
3. G8 completes the course in 3 of 3 runs.

## Records
Each test writes to `~/drive_tuning_2026_09_28/runs/` (raw logs), and its result goes in `results/GROUND_<date>.md` as a table: test | measured | limit | pass/fail.
