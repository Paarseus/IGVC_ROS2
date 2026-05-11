# IGVC_ROS2 — Comprehensive Code Review

**Date:** 2026-05-01
**Reviewer:** Claude Code, multi-agent orchestration
**Scope:** Every Python / C++ / firmware / config / launch / URDF / BT / shell file in the IGVC_ROS2 workspace at `/home/mspacman/IGVC_ROS2/`, excluding vendored upstream sources (`src/realsense-ros/`, `src/xsens_mti/`, `src/zed-ros2-wrapper/`, `src/semantic_segmentation_layer/`).
**Competition deadline:** ~1 month (early June 2026).

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Methodology](#2-methodology)
3. [Severity Rubric](#3-severity-rubric)
4. [Top 10 P0 Items — IGVC-Blocking](#4-top-10-p0-items--igvc-blocking)
5. [Full P0 Punch List](#5-full-p0-punch-list)
6. [Failure-Mode Matrix](#6-failure-mode-matrix)
7. [Inspection-Day Checklist](#7-inspection-day-checklist)
8. [Risk Register](#8-risk-register)
9. [One-Month Timeline](#9-one-month-timeline)
10. [Per-Package Verdict Cards](#10-per-package-verdict-cards)
11. [P1 — Professional Quality Issues](#11-p1--professional-quality-issues)
12. [P2 — Nice-to-haves](#12-p2--nice-to-haves)
13. [Documentation Drift Table](#13-documentation-drift-table)
14. [Build-System Issues](#14-build-system-issues)
15. [Test Coverage & CI](#15-test-coverage--ci)
16. [Positives](#16-positives)
17. [Recommendations](#17-recommendations)
18. [Working Artifacts](#18-working-artifacts)

---

## 1. Executive Summary

**Overall verdict.** The IGVC_ROS2 workspace is closer to competition-ready than the punch list will suggest, but the team must execute a focused one-month plan to clear inspection-day blockers. The control architecture is conceptually correct, the perception package's tests are the workspace high-water mark, the firmware bring-up trail is exemplary, and Nav2 has been assembled with deliberate engineering choices. **What is dangerous is the gap between the intended architecture and the safety guarantees the code currently provides** — many failure modes that *should* stop the chassis don't, the IGVC-mandated hardware e-stop chain has not been built, and the front-door README describes a fundamentally different vehicle than what exists.

**The single most important item:** the vehicle has **not** driven autonomously end-to-end. Sustained-speed field testing is blocked by a power-rail brown-out (Jetson and SparkMAXes share a 12 V buck). Until that is fixed, the entire stack remains untested at speed.

**Top finding by severity (full lists in §4 and §5):**

- **15+ P0 items spanning safety, security, build, and functional correctness.**
- **80+ P1 items** (professional-quality issues that would fail a code review).
- **~120 P2 items** (style, naming, polish).

**One-line action plan:**

1. **This week:** wire hardware e-stop, install dedicated Jetson buck, rotate NTRIP credentials, fix `_estop` latch, fix Teensy watchdog `ctrl_mode`, fix `_imu_fresh` stale-yaw, add CSV-input mode to `generate_graph.py`, fix the README.
2. **Week 2:** first autonomous 30 m run; voxel_layer onto Humble local costmap; webui auth; HSV hue wraparound.
3. **Week 3:** mock-course drills; `nav2_collision_monitor`; `test_kinematics.py`; sensor mount measurements.
4. **Week 4:** freeze main; dress rehearsal; runbook.

**What this review delivers:** a top-10 P0 inspection-blocker list with file:line citations, a failure-mode matrix (21 rows, 12 P0s, 6 OKs), an inspection-day checklist for the physical vehicle, a 17-item code-vs-docs drift table, a risk register with mitigations, and per-package verdict cards. All findings cross-reference the 16 working artifacts under `docs/review_2026-05/working/`.

---

## 2. Methodology

This review was produced by a four-phase multi-agent process:

**Phase 1 — Standards research (4 parallel agents).** Each agent established what professional / upstream standards look like in one domain and produced a reference document with citations to authoritative sources (docs.ros.org, design.ros2.org, REPs, navigation.ros.org, robot_localization, vision_msgs, ZED ROS 2 wrapper, IGVC rules, ISO 13849-1, NASA Power-of-10, REV Robotics, FlexCAN_T4). Outputs: `standards_ros2_python.md` (3.5k words, 18-item checklist), `standards_nav2_localization.md` (3.9k words, 25-item checklist), `standards_perception.md` (3.5k words, 25-item checklist), `standards_firmware_safety.md` (4.3k words, 25-item checklist).

**Phase 2 — Per-package deep review (8 parallel agents).** Each agent read every file in one package, scored it against the Phase 1 standards, and produced a per-package report with file:line citations, severity-tagged findings, and a P0/P1/P2 punch list. Packages: `avros_msgs`, `avros_bringup` (largest), `avros_control` (most safety-critical), `avros_webui`, `avros_navigation`, `avros_perception`, `avros_sim`, `firmware/`. After several agent timeouts, the surviving prompts use an iterative-write strategy (write skeleton first, then Edit section-by-section).

**Phase 3 — Cross-cutting review (4 parallel agents).** Each agent consolidated Phase 2 findings around one workspace-wide concern: build system + colcon, documentation quality, end-to-end safety chain, test coverage + IGVC competition readiness. Each produced a report with cross-references back to Phase 2.

**Phase 4 — Synthesis (this document).** Consolidated all 16 working reports into a single severity-prioritized master report.

**Scope.** Every Python / C++ / firmware / config / launch / URDF / BT / shell file in the IGVC_ROS2 workspace, excluding vendored upstream sources (`src/realsense-ros/`, `src/xsens_mti/`, `src/zed-ros2-wrapper/`, `src/semantic_segmentation_layer/`). Total source reviewed: ~5,884 LOC of Python/C++ + ~2,700 LOC of YAML/XML/Python config + ~440 lines of CLAUDE.md + 7 changelogs + 5 .tex files.

**Review type.** Static. No code was executed, no Teensy was flashed, no `colcon build` was attempted during the review.

---

## 3. Severity Rubric

**P0 — IGVC-blocking.** Either (a) the vehicle would fail IGVC AutoNav inspection or be disqualified, (b) the chassis is unsafe to drive autonomously in its current state, (c) the workspace fails to build / launch on a fresh checkout, or (d) competition functionality (e.g. course-graph navigation) cannot work without a code change.

**P1 — Professional quality.** Convention violations a professional code review would block on. Typically: missing parameter declarations, undeclared dependencies, blocking I/O in callbacks, untested invariants, code-vs-docs drift, ambiguous semantics, missing tests for safety-critical code.

**P2 — Polish.** Style, naming, documentation, future-proofing, hygiene. Address opportunistically; never blocking.

---

## 4. Top 10 P0 Items — IGVC-Blocking

These are the items most likely to block the team at judges' inspection or prevent completion of the AutoNav course. Each links to the deeper Phase 2 / Phase 3 source.

| # | Item | Where | Effort | Source |
|---|---|---|---|---|
| 1 | **Hardware mechanical e-stop** at center-rear (2–4 ft) + **wireless e-stop** (≥ 50 ft) + **battery-to-motor contactor**. Quoted IGVC rule: *"Vehicle E-Stops must be hardware based and not controlled through software."* **Without this the team fails inspection on day 0.** | Vehicle hardware + `firmware/teensy_diff_drive/teensy_diff_drive.ino` GPIO + new latched-fault state machine | ~1 day HW + ~2 hr FW | `package_firmware.md` P0 #1; `crosscut_safety.md` Links 1–3 |
| 2 | **Dedicated Jetson power rail** (48 V → 19 V buck, separate from motor rail). *Currently shared 12 V buck; motor inrush brown-outs the Jetson — blocks any sustained-speed field test.* | Vehicle hardware | ~0.5 day + parts | TODO.md "High priority"; `crosscut_safety.md` Link 15 |
| 3 | **Course graph for IGVC AutoNav venue.** Committed `cpp_campus_graph.geojson` is 16,651 nodes / 17,492 edges (CPP campus, not the AutoNav course); `generate_graph.py` has no CSV-input or GPS-walk mode. CLAUDE.md claims "52 / 113" — false. **route_server has no goal-able plan today.** | `src/avros_navigation/scripts/generate_graph.py`; `src/avros_bringup/config/cpp_campus_graph.geojson` | ~1 afternoon for `--from-csv`; ~30 min/graph | `package_avros_navigation.md` P0 #1, #2 |
| 4 | **NTRIP credentials committed to public GitHub** (username `stoic_panini`, password `z0OYEP3Bwg0hdxvN`). Rotate immediately and move to env-var or `.gitignore`d local override. | `src/avros_bringup/config/ntrip_params.yaml:14-15` | ~1 hr | `package_avros_bringup.md` P0 SECURITY |
| 5 | **Latch `_estop` and bound the e-stop deceleration.** Today: `_estop` auto-clears on the next non-estop ActuatorCommand, AND e-stop snaps `_slew_v=0` (infinite jerk — saturates SparkMAX PID, regen-spikes 12V rail, snaps drivetrain backlash). | `src/avros_control/avros_control/actuator_node.py:217, 256-259` | ~45 min | `package_avros_control.md` F6, F18 |
| 6 | **Serial-link liveness + IMU staleness** in `actuator_node`. Today: no detection of dead serial handle (control loop writes into nothing while wheels keep moving); `_imu_fresh` set once and never reset (stale yaw drives heading-hold forever if Xsens dies). | `actuator_node.py:107-110, 369-398` (serial); `:225-228, 274-289` (IMU) | ~1 afternoon (serial) + ~30 min (IMU) | `package_avros_control.md` F3, F4, F5 |
| 7 | **Firmware watchdog leaves `ctrl_mode = MODE_VELOCITY`** on host loss. Wheels stop via velocity PID but Brake-idle never engages → chassis rolls on slope. **One-line fix.** | `firmware/teensy_diff_drive/teensy_diff_drive.ino:355-362` (add `ctrl_mode = MODE_DUTY;` after wheel zeroing) | ~5 min code + reflash | `package_firmware.md` P0 #2 |
| 8 | **Humble local costmap has no LiDAR layer.** `plugins:` is `["semantic_layer", "inflation_layer"]` — `voxel_layer` block exists in YAML but isn't activated. Velodyne data wasted; only camera-classified hazards appear. Camera misclassification = collision. Plus: no `nav2_collision_monitor` for a >1 m/s vehicle. | `src/avros_bringup/config/nav2_params_humble.yaml:142, 144-176` | ~30 min (voxel_layer) + ~2 hr (collision_monitor) | `package_avros_bringup.md` P0; `crosscut_safety.md` Link 14 |
| 9 | **WebSocket has no authentication.** `webui_node` binds `0.0.0.0`, no token, no Origin check. Anyone on venue WiFi or Tailscale can `wss://<jetson>:8000/ws` and drive. Single-controller mutex loses to reconnect-loop. | `src/avros_webui/avros_webui/webui_node.py:200, 110-125` | ~1 hr | `package_avros_webui.md` P0 #1 |
| 10 | **HSV barrel hue wraparound unhandled.** Single-range `cv2.inRange` on `H ∈ [5, 25]` silently misses pixels when sunset shifts orange across H=0/179 boundary. AutoNav is run outdoors. | `src/avros_perception/avros_perception/pipelines/hsv.py:103-104` | ~2 hr (incl. regression test) | `package_avros_perception.md` P0 #2 |

**Rationale on ordering.** Items 1–3 are structurally hardest — hardware integration and a missing course graph. They cannot be parallelized away. Items 4–10 are concentrated software fixes the team can land in a single intense week.

**Compounding risk.** Items 5, 6, 8 all assume hardware e-stop works. Without item 1, every other layer of defense collapses. Fix item 1 first.

---

## 5. Full P0 Punch List

This is the consolidated, deduplicated P0 list across all 8 packages and 4 cross-cutting reports. Grouped by category.

### 5.1 Safety & Control (15 items)

| # | Issue | Location | Source |
|---|---|---|---|
| S1 | No hardware mushroom-button e-stop wired | Vehicle / firmware GPIO | `package_firmware.md` P0; `crosscut_safety.md` L1 |
| S2 | No wireless e-stop receiver (≥ 50 ft) | Vehicle | `crosscut_safety.md` L2 |
| S3 | No motor-power contactor | Vehicle | `crosscut_safety.md` L3 |
| S4 | No dedicated compute power rail (Jetson brown-outs on motor inrush) | Vehicle | TODO.md; `crosscut_safety.md` L15 |
| S5 | `_estop` auto-clears on next non-estop ActuatorCommand (no latch) | `actuator_node.py:217` | `package_avros_control.md` F6 |
| S6 | E-stop snaps `_slew_v=0` (infinite jerk decel) | `actuator_node.py:256-259` | `package_avros_control.md` F18 |
| S7 | No serial-link liveness — dead handle silently writes-to-void | `actuator_node.py:107-110, 369-398` | `package_avros_control.md` F3, F4 |
| S8 | `_imu_fresh` set once and never reset → stale yaw heading-hold | `actuator_node.py:225-228, 274-289` | `package_avros_control.md` F5 |
| S9 | Teensy host-loss watchdog leaves `ctrl_mode = MODE_VELOCITY` (Brake-idle never engages) | `teensy_diff_drive.ino:355-362` | `package_firmware.md` P0 #2 |
| S10 | No `nav2_collision_monitor` for a >1 m/s vehicle | `nav2_params_humble.yaml` | `package_avros_bringup.md` P0; `crosscut_safety.md` L14 |
| S11 | Humble local costmap missing `voxel_layer` in `plugins:` (LiDAR unused locally) | `nav2_params_humble.yaml:142` | `package_avros_bringup.md` P0 |
| S12 | `inflation_radius: 0.5` < robot_radius 0.8 (planner plans into hard collision) | `nav2_params_humble.yaml` | `package_avros_bringup.md` P0 |
| S13 | WebUI WebSocket binds 0.0.0.0 with no auth/token/Origin | `webui_node.py:200, 110-125` | `package_avros_webui.md` P0 #1 |
| S14 | HSV barrel hue wraparound unhandled (silent miss at sunset) | `pipelines/hsv.py:103-104` | `package_avros_perception.md` P0 #2 |
| S15 | `LabelInfo.header.frame_id` published empty (kiwicampus contract violation) | `perception_node.py:212` | `package_avros_perception.md` P0 #1 |

### 5.2 Build / Deployment / Security (10 items)

| # | Issue | Location | Source |
|---|---|---|---|
| B1 | NTRIP credentials committed to public repo | `config/ntrip_params.yaml:14-15` | `package_avros_bringup.md` P0 SECURITY |
| B2 | `scripts/deploy.sh` deploys to deprecated `~/AVROS` (not `~/IGVC`) | `scripts/deploy.sh:11` | `crosscut_build.md` P0 #1 |
| B3 | `scripts/deploy.sh` masks colcon exit code (silently launches stale install) | `scripts/deploy.sh:44` (no `set -o pipefail`) | `crosscut_build.md` P0 #2 |
| B4 | `avros_bringup/package.xml` missing 6 `<exec_depend>` for packages it launches (`xsens_mti_ros2_driver`, `zed_wrapper`, `nav2_route`, `nav2_lifecycle_manager`, `avros_perception`, `semantic_segmentation_layer`) | `src/avros_bringup/package.xml` | `crosscut_build.md` P0 #3 |
| B5 | `avros_webui` missing pip exec_depends (`python3-fastapi`, `python3-uvicorn`, `python3-websockets`) | `src/avros_webui/package.xml` | `crosscut_build.md` P0 #4 |
| B6 | `avros_navigation/package.xml` declares 7 dead ROS deps + 0 actual pip deps | `src/avros_navigation/package.xml:10-16` | `crosscut_build.md` P0 #5 |
| B7 | `requirements.txt` floors-only — `osmnx>=1.3.0` will pull breaking 2.x; `numpy<2` Jetson pin encoded nowhere | `requirements.txt` | `crosscut_build.md` P0 #6 |
| B8 | `scripts/apply_kiwicampus_patches.sh` referenced by CLAUDE.md does not exist | `CLAUDE.md` "Build & Test" | `crosscut_build.md` P0 #7 |
| B9 | `avros_msgs/srv/PlanRoute.srv` is dead IDL bloating rebuilds | `src/avros_msgs/CMakeLists.txt:16` | `crosscut_build.md` P0 #8 |
| B10 | No top-level repo `LICENSE` file (GitHub flags as missing) | repo root | `crosscut_documentation.md` P0 #5 |

### 5.3 Functional / Course (4 items)

| # | Issue | Location | Source |
|---|---|---|---|
| F1 | Committed graph is 16,651 nodes (CPP campus); IGVC course has no graph; `generate_graph.py` has no CSV-input mode | `scripts/generate_graph.py` + `cpp_campus_graph.geojson` | `package_avros_navigation.md` P0 #1, #2 |
| F2 | Sim is **Ackermann car**; real is **tracked diff-drive** — different control problems | `avros_sim/avros_sim/avros_vehicle_driver.py:79`; `avros_webots.urdf` | `package_avros_sim.md` P0 #1 |
| F3 | Sim `avros_webots.urdf` references `camera_color_optical_frame` / `camera_depth_optical_frame` that aren't statically declared in URDF (image→base_link transforms fail with LookupException in sim) | `src/avros_sim/resource/avros_webots.urdf:31, 40` | `package_avros_sim.md` P0 #2 |
| F4 | Both sim EKFs load the same `ekf.yaml` — dual-EKF pattern collapses (both publish odom→base_link). *Likely also affects `avros_bringup` per Phase 2.* | `sim_navigation.launch.py:114, 130` | `package_avros_sim.md` P0 #3 |

### 5.4 Documentation (6 items)

| # | Issue | Location | Source |
|---|---|---|---|
| D1 | README.md describes "Ackermann + UDP + steering/throttle/brake/gear" — none exist on this platform | `README.md:16, 19, 40` | `crosscut_documentation.md` P0 #1; drift D1–D3 |
| D2 | README.md says `cd ~/AVROS` (forbidden by CLAUDE.md) | `README.md:46` | `crosscut_documentation.md` P0 #2; drift D4 |
| D3 | CLAUDE.md "52 nodes, 113 edges" route-graph claim — actual file is 16,651 / 17,492 | `CLAUDE.md:279` | `crosscut_documentation.md` P0 #3; drift D5 |
| D4 | CLAUDE.md `zed_front.yaml HD720@15fps` row contradicts CLAUDE.md's own Known Issues row noting HD720 deprecated for ZED X v5.2 | `CLAUDE.md:389` | `crosscut_documentation.md` P0 #4; drift D6 |
| D5 | `realsense_d455_setup.tex` (225 lines) duplicates `REALSENSE_SETUP.md` (96 lines) — two sources of truth | `docs/realsense_d455_setup.tex` | `crosscut_documentation.md` P0 |
| D6 | `docs/avros_nav_demo.gif` is 38 MB committed to repo (bloats every clone); may show old Ackermann sim | `docs/avros_nav_demo.gif` | `crosscut_documentation.md` P0 |

### 5.5 Messages (1 item)

| # | Issue | Location | Source |
|---|---|---|---|
| M1 | `string mode` field in ActuatorCommand/State is duplicated across 5 files (.msg ×2 + Python ×2 + JS ×1) with no central source — typo-driven silent failures | `src/avros_msgs/msg/{ActuatorCommand,ActuatorState}.msg:4` | `package_avros_msgs.md` P0 |

**P0 totals: 36 items.** Of these, 7 are pure hardware (S1–S4 + S5/S6/S7 vehicle integration); the remaining 29 are software/config/documentation, all afternoon-scale or smaller fixes.

---

## 6. Failure-Mode Matrix

End-to-end safety chain trace from operator to motors. **Severity legend:** P0 = vehicle is unsafe / inspection fails today. OK = link works. Source: `crosscut_safety.md` Links 1–15.

| # | Failure | Should stop? | Today | Time / outcome | Severity |
|---|---|---|---|---|---|
| F1 | Mushroom button pressed | YES (Cat-0 — power off) | NO — not wired | ∞ | **P0 HW** |
| F2 | Wireless e-stop pressed | YES (Cat-0) | NO — not wired | ∞ | **P0 HW** |
| F3 | WebUI e-stop pressed | YES (assist) | YES but auto-clears + infinite-jerk decel | ~50 ms, re-arm risk | **P0 SW** |
| F4 | WebUI WS disconnect | YES (assist) | YES — `finally:` publishes estop | ~50 ms | OK |
| F5 | Hostile WiFi user connects to webui | Reject | NO auth — full control granted | n/a | **P0 SW** |
| F6 | Nav2 stops publishing cmd_vel | YES (controlled) | YES — 500 ms timeout + 1.0 s decel | ~1.5 s, ~0.75 m | OK (P1 polish) |
| F7 | USB to Teensy unplugged | YES (immediate) | No host detect; firmware WDT stops PID-zero (no brake) | ~0.6 s + slope risk | **P0 SW** |
| F8 | Teensy firmware lockup | YES | Heartbeat ceases → SparkMAX coast | ~100 ms | OK (P1 — RTWDOG) |
| F9 | Host watchdog trip | YES (Cat-1 brake) | YES PID-zero but no brake-idle | slope risk | **P0** (1-line fix) |
| F10 | One CAN cable disconnect | YES (symmetric) | NO — chassis pivots uncontrolled | ∞ asymmetric | **P1 SW** |
| F11 | Both SparkMAX lose heartbeat | YES | YES — both coast | ~100 ms + coast | OK |
| F12 | IMU disconnect / freeze | Degrade | NO — heading-hold uses frozen yaw | curves off course | **P0 SW** |
| F13 | NaN quaternion | Ignore | Silently treated as yaw=0 | drift | **P2 SW** |
| F14 | RTK FIXED → SPS | Warn | NO warning | off-course | **P1 SW** |
| F15 | Person 1 m ahead at 1.5 m/s | YES | NO collision_monitor; no LiDAR on local costmap | collision likely | **P0 SW** |
| F16 | Camera misclassifies obstacle | YES (LiDAR fallback) | NO — LiDAR not on Humble local costmap | collision likely | **P0 SW** |
| F17 | 12 V rail sag during hard accel | Warn | Jetson reboots, firmware WDT stops | 30–60 s blind | **P0 HW** |
| F18 | Bus voltage < 11 V | Soft stop | NOT DETECTED — DIAG not parsed | n/a | **P1 SW** |
| F19 | Op presses Ctrl-C twice | Graceful | Mostly OK | ~100 ms | OK (P2) |
| F20 | Nav2 wants software estop | YES | No path — Nav2 publishes only cmd_vel | n/a | **P1 SW** |
| F21 | webui_node crashes mid-drive | YES (cmd_vel takeover) | YES — 500 ms watchdog | ~1.5 s | OK |

**Tally: 6 P0 HW + 6 P0 SW + 6 OK + 4 P1 + 1 P2.**

**Most urgent compound risk:** F12 (IMU loss → curves off course) × F15/F16 (no collision monitor + camera-only costmap). Chassis drifts off course and doesn't see what's in the new direction. Realistic worst case for a 1.5 m/s vehicle around spectators.

---

## 7. Inspection-Day Checklist

Verify these on the *physical vehicle* before competition. Print this; sign each item off as a team. Source: `crosscut_safety.md` "Inspection-day checklist".

### Mechanical / electrical

- [ ] Mushroom button on the rear of the vehicle, between 2 ft and 4 ft (tape measure).
- [ ] Mushroom is latching (twist-and-pull to release) and uses NC contacts (multimeter: continuity present in idle, broken when pressed).
- [ ] Wireless e-stop receiver mounted with clear line-of-sight antenna; transmitter has fresh batteries; latching switch.
- [ ] **Wireless range test at 50 ft** in open space; verify motors lose power on press.
- [ ] Both mushroom and wireless feed the *same* contactor (in series).
- [ ] Contactor is between battery and SparkMAX 12 V — not just signaling firmware. Press mushroom: SparkMAX LEDs must go off.
- [ ] After release, vehicle does NOT auto-resume — explicit re-arm required.
- [ ] **Slope test:** chassis on 5° ramp, command stop, confirm it holds (not rolls). If rolls, brake-idle isn't engaging.
- [ ] Compute rail separate from motor rail (multimeter Jetson supply during hard accel; should stay above 18 V).
- [ ] Battery cutoff disconnects everything.
- [ ] All power cables have strain relief.

### Software pre-flight (every test day)

- [ ] `colcon build --symlink-install` clean.
- [ ] `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` + `CYCLONEDDS_URI=...` exported in the shell.
- [ ] `ros2 topic hz /imu/data` ≈ 100 Hz, `/velodyne_points` ≈ 70 Hz, `/gnss` reports RTK FIXED.
- [ ] On jacks: `ros2 topic pub /cmd_vel ...` confirms both wheels drive at same speed.
- [ ] Watchdog test: stop publishing cmd_vel, confirm wheels stop in < 1.5 s.
- [ ] E-stop test (webui + hardware): both stop wheels within their respective windows.

### Bring-up regression

- [ ] Rerun `phase6c_pid_verify.py`; assert max RPM within 5% of 2026-04-23 baseline (5532 L / 5072 R). > 20 % drop = chain wear / belt slip.
- [ ] CAN continuity: 60 Ω across CAN_H/CAN_L (two 120 Ω terminators in parallel).
- [ ] Both SparkMAX FW 26.1.4 (REV Hardware Client).
- [ ] Both SparkMAX idle mode = Brake (Hardware Client, persisted in flash).
- [ ] Motor inversion configured at the SparkMAX, not the firmware.

### What to bring

This checklist (empty boxes), IGVC rules PDF (current year), multimeter, tape measure, plywood for slope test, wireless e-stop transmitter (fresh batteries), spare mushroom button + spare contactor, SSH access + operator laptop.

---

## 8. Risk Register

Source: `crosscut_tests_and_readiness.md` "Risk register".

| # | Risk | Likelihood | Impact | Mitigation if not fixed |
|---|---|---|---|---|
| 1 | Mechanical / wireless e-stop missing | Certainty | **Disqualification** | None — team fails inspection. |
| 2 | Jetson brown-out under motor load | High (already observed) | Mid-run reset, run forfeit | Speed-cap to 0.7 m/s; accept slower time. |
| 3 | Wrong route graph / no IGVC graph | Certainty (if route_server is BT path) | No goal-able plan | Use `navigate_to_pose_simple_humble.xml` instead; lose pre-built waypoint logic. |
| 4 | NTRIP credentials leaked | Medium (repo visibility unknown) | Account banned mid-competition | Rotate + env-var. ~1 hr. |
| 5 | Webui reconnect silently un-stops motors | Medium (Wi-Fi blip is common) | Vehicle resumes after operator believed stopped | Train operators to use hardware e-stop. |
| 6 | Serial link to Teensy fails mid-run | Medium (USB-C is known failure mode) | Vehicle keeps moving until 300 ms firmware watchdog | Visual monitoring + hardware e-stop. |
| 7 | Watchdog leaves vehicle in MODE_VELOCITY on slope | Low (course is flat) | Rolls after host crash | Flat course mitigates; hardware e-stop is ground truth. |
| 8 | LiDAR not used in Humble local costmap | High (any non-colored obstacle the camera misses) | Vehicle plows into hard obstacle | Speed-cap reduces severity; no software mitigation. |
| 9 | Spectator drives robot via webui | Low (requires venue Wi-Fi) | Run aborted | Bind to localhost on the day. ~1 minute change. |
| 10 | HSV misses barrels at sunset | Medium (afternoon AutoNav slot) | Vehicle hits barrel | Schedule near solar noon; live-tune at venue. |

**Compounding risk.** Items 5, 6, 7, 9 all assume hardware e-stop works. Without it (item 1), every layered defense collapses. The first three items are the foundation everything else relies on.

---

## 9. One-Month Timeline

Today is 2026-05-01. Plan assumes a four-person team (mechanical/power, perception, controls/firmware, nav/integration leads). Source: `crosscut_tests_and_readiness.md` "Realistic 1-month timeline".

### Week 1 (May 1–8) — Foundations

Remove items that block all subsequent field testing.

**Mechanical lead:**
- Order + install hardware e-stop (P0 #1: mushroom + contactor + safety circuit).
- Order + install dedicated Jetson buck (P0 #2: 48V→19V, separate from motor rail).
- Both during the same shop day.

**Controls lead:**
- Land four `actuator_node` software fixes:
  - Latched estop (P0 #5).
  - Serial liveness watchdog (P0 #6 — serial side).
  - IMU staleness watchdog (P0 #6 — IMU side).
  - Teensy watchdog `ctrl_mode = MODE_DUTY;` (P0 #7).
- Add `test/test_kinematics.py` (~120 LOC: `yaw_from_quaternion`, `wrap_angle`, diff-drive inverse, slew limiter, midpoint integrator).

**Nav lead:**
- Rotate NTRIP credentials (P0 #4).
- Set up `colcon build` GitHub Action (~30 LOC YAML).
- Fix README.md (rewrite as thin trampoline to CLAUDE.md).

**Perception lead:**
- Land HSV hue wraparound fix + regression test (P0 #10).
- Tighten pothole HSV per TODO.md (concrete misclassified at ~14 % outdoor).

**End-of-week gate:** 50 m teleop drive with no brown-out + verified mechanical e-stop. *If brown-out persists, escalate — this is the structural blocker.*

### Week 2 (May 9–15) — Course-level integration; first autonomous run

**Nav lead:**
- Add CSV-input mode to `generate_graph.py` (P0 #3).
- Pre-author an AutoNav arena graph from organizer-provided map.
- Wire the `voxel_layer` into Humble local costmap (P0 #8).
- Bump `inflation_radius` 0.5 → 0.8 to match `robot_radius`.

**Controls lead:**
- Add `add_on_set_parameters_callback` for runtime tuning (P1).
- Run Phase 7 BURN verification (firmware bring-up regression).
- Wire `msg.watchdog_active` (currently hard-coded false).

**Perception lead:**
- Finalize pothole vs lane separation.
- Re-tighten `max_obstacle_distance` to 5–8 m.
- Run `perception_test.launch.py` at the bench course.

**Mechanical lead:**
- Measure sensor mounts (ZED, Velodyne, IMU, GNSS lever arm).
- Run 5 m `m_per_rev` calibration.

**End-of-week gate:** first autonomous 30 m navigation run via `navigation.launch.py`.

### Week 3 (May 16–22) — Robustness and operational practice

Mock IGVC AutoNav course on a quad/field with taped lanes + barrels + 5 waypoints. ≥ 3 runs daily, rosbags captured.

**Controls lead:**
- P1 polish: IMU QoS, measured-`dt` slew, watchdog state on actuator_state.

**Perception lead:**
- Land integration test that publishes synthetic image+cloud (test P1 #5).
- Live-tune HSV for venue lighting.

**Nav lead:**
- WebSocket auth (P0 #9).
- `nav2_collision_monitor` config + launch.

Daily 15-min stand-ups; every failure into TODO.md.

**End-of-week gate:** 5 consecutive autonomous course runs without operator intervention.

### Week 4 (May 23–29) — Polish + dress rehearsal

Freeze `main`. Any change requires code review *and* a successful mock-course run.

- Mechanical lead: prep spares (batteries, USB cables, pre-flashed Teensy).
- Perception lead: compile a "lighting playbook" with HSV ranges per time-of-day.
- Nav lead: write operator runbook, pre-stage the IGVC course graph, verify RTK FIXED outdoors.
- Two days before competition: full dress rehearsal on the vehicle. **No software changes after.**

### Buffer / decisions to make in week 1

This schedule has no buffer for "route_server times out at 16k nodes"-class surprises. Build buffer by descoping P1s from week 2 onward.

**The hardest early decision** — make in week 1 — is whether to use the route_server BT path or skip it for `navigate_to_pose_simple_humble.xml` with hand-clicked goals. Deciding early removes the riskiest dependency.

---

## 10. Per-Package Verdict Cards

### 10.1 avros_msgs

**Files:** `CMakeLists.txt`, `package.xml`, `msg/ActuatorCommand.msg`, `msg/ActuatorState.msg`, `srv/PlanRoute.srv` (~58 LOC).

**Verdict:** structurally clean, semantically dangerous. Skeleton (format-3, `member_of_group rosidl_interface_packages`, modern `rosidl_generate_interfaces`, build-time deps correctly wired) is professional. Where it goes wrong is field semantics.

**P0:** `string mode` field in both ActuatorCommand and ActuatorState (literal "N"/"D"/"S"/"R") instead of `uint8 mode` with constants — duplicated across 5 files (.msg ×2, Python ×2, JS ×1) with no central source of truth. Typo-driven silent failures.

**P1:** `srv/PlanRoute.srv` is dead code — only references are in itself + CMakeLists.txt; project uses `nav2_route`'s ComputeRoute action. `bool watchdog_active` field hardcoded false in `actuator_node.py:327` — never set. Placeholder `<version>0.0.0</version>` and `avlab@cpp.edu` maintainer. Incomplete LICENSE file (claims MIT, missing copyright line).

**Positives:** correctly uses package format 3, `member_of_group`, modern `rosidl_generate_interfaces`, properly split `buildtool_depend rosidl_default_generators` + `exec_depend rosidl_default_runtime`, downstream packages consume it correctly. The REP-103 steer-sign comment in `ActuatorCommand.msg:6` is the kind of prose convention that prevents bugs.

**Source:** `package_avros_msgs.md` (P0×2 / P1×4 / P2×3).

### 10.2 avros_bringup

**Files:** 9 launch files, 16 YAMLs, 3 BT XML, 1 URDF/xacro, 3 RViz configs, 1 GeoJSON (~2,704 LOC). Largest package by file count.

**Verdict:** the most heavily-touched package and where most P0 surface area lives. The launch tree is conceptually correct (dual-EKF + navsat_transform pattern, ZED wrapper gotchas avoided, route_server.global_frame set explicitly) but riddled with deployment hazards.

**P0 (13):** NTRIP credentials committed to public repo (P0 SECURITY). GeoJSON / CLAUDE.md mismatch (16,651 vs 52 nodes). `RMW_IMPLEMENTATION` only set in sensors.launch.py — every other launch (incl. navigation) inherits user shell defaults. Humble local costmap has no `voxel_layer` in plugins (LiDAR unused locally). No `nav2_collision_monitor` for >1 m/s. `zed_back.yaml` uses v4 wrapper syntax + deprecated HD720/PERFORMANCE enums. Missing 6 `<exec_depend>` entries for packages it launches. Drift between `nav2_params.yaml` (Jazzy) and `_humble.yaml`. Sensor URDF mounts are sketch-derived (every TODO marker); GNSS lever arm `[0,0,0]`. `inflation_radius` 0.5 < `robot_radius` 0.8.

**Positives (24):** every launch file has a strong docstring; ZED wrapper gotchas correctly avoided (namespace/node_name, v5 enums, optical-frame chain via `zed_macro.urdf.xacro`); dual-EKF pattern correct (continuous-only sources in odom EKF, GPS in map EKF); `actuator_params.yaml` is exemplary (every constant has a unit and derivation comment); PR3 raytrace-clear applied per kiwicampus fix.

**Source:** `package_avros_bringup.md` (807 lines, 13 P0 / 27 P1 / 23 P2).

### 10.3 avros_control

**Files:** `actuator_node.py` (426 LOC, single file), package.xml, setup.py (~456 LOC).

**Verdict:** the most safety-critical Python in the workspace. **Architecture is conceptually right; safety hardening is broken across the board.** Five P0s share a root cause: no liveness/staleness model for the node's three external dependencies (Teensy serial, Xsens IMU, e-stop latch). Each fix is 15–60 minutes; together they're an afternoon.

**P0 (5):**
- F3 (`:376-398`) — no serial reconnect / dead-handle detection (chassis can keep moving while logs spam at 50 Hz).
- F4 (`:376-398`) — no E-line freshness check (Teensy hang invisible).
- F5 (`:137-141, 225-229, 274-289`) — `_imu_fresh` set once, never reset (stale yaw drives heading-hold forever if IMU dies).
- F6 (`:209-223`) — `_estop` auto-clears on next non-estop ActuatorCommand (no latch).
- F18 (`:256-259`) — on e-stop, slew steps to 0 in one tick (infinite jerk) — contradicts inline comment.

**P1 (12):** zero behavioral unit tests on a 426-LOC safety-critical control node. Suggested `test_kinematics.py` would be ~120 LOC for `wrap_angle`, `yaw_from_quaternion`, diff-drive inverse, slew limiter, midpoint integrator.

**Positives (multiple):** midpoint pose integration correct (2nd-order accurate); asymmetric accel/decel logic correct; parameter coverage matches YAML 1:1; slew limiter design right modulo dt; last-message-wins priority architecture correct (webui and Nav2 produce identical chassis behavior); lock discipline on `_fb_lock` right; `actuator_params.yaml` 1:1 derivation comments are the workspace standard.

**Source:** `package_avros_control.md` (913 lines, 5 P0 / 12 P1 / 22 P2).

### 10.4 avros_webui

**Files:** `webui_node.py` (211 LOC), package.xml, setup.py (~246 LOC).

**Verdict:** the safety property that matters (disconnect → e-stop) **does** fire correctly via `finally:` on every WS exit path. Concurrency split is canonical (uvicorn main thread, rclpy.spin daemon, lock-guarded shared state). What's broken is the access boundary.

**P0 (2):**
- No authentication on the control WebSocket — `host='0.0.0.0'` (`webui_node.py:200`); WS handler accepts every handshake (lines 110-125) with only a single-controller mutex which a hostile client can race-loop. Anyone reachable to TCP 8000 (vehicle LAN, Tailscale, IGVC venue Wi-Fi) can drive. Mitigations: bind localhost / Tailscale-only, URL token, Basic Auth.
- Software-only e-stop (project-level concern — IGVC requires hardware).

**P1 (12):** no server-side joystick-frame watchdog (`receive_json()` blocks indefinitely on half-open TCP); NaN propagates through `float()` with no `math.isfinite` guard; FastAPI/uvicorn/websockets undeclared in `package.xml` (rosdep can't satisfy); TOCTOU race on single-controller lock; no Origin header check (drive-by attack); no test for disconnect → e-stop safety contract; nipplejs CDN-loaded so offline operation breaks; `max_throttle: 1.0` overrides safer 0.55 default.

**Positives:** Disconnect → e-stop **does** fire via `finally:` (`:161-165`). Server-side input clamping in `publish_command` (`:79-81`). 20 Hz send rate (10× margin on actuator_node's 500 ms freshness window). Full-width always-visible e-stop button. QoS verified: both ends of `/avros/actuator_command` use depth-10 default reliable (volatile) — match.

**Source:** `package_avros_webui.md` (165 lines, 2 P0 / 12 P1 / ~10 P2).

### 10.5 avros_navigation

**Files:** `scripts/generate_graph.py` (332 LOC), package.xml, setup.py, tests (~360 LOC).

**Verdict:** **No P0 schema bug.** GeoJSON output matches nav2_route's expectations (frame: 'map' on nodes, cost on edges, bidirectional edges preserved); coordinate-system handling is correct (`generate_graph.py:191-192, 220-221` lifts the datum to UTM and subtracts → produces same map-frame meters that navsat_transform_node would emit at runtime). The pitfall the brief warned about is avoided.

**P0 (workflow-shaped, 2):**
- No way to author the IGVC course graph — OSM has no data for the AutoNav venue, generator supports neither CSV input nor GPS-walk import.
- Committed graph is **16,651 nodes / 17,492 edges**, NOT the "52 / 113" CLAUDE.md claims. The `--spacing 5.0` densification of CPP-wide OSM produces ~300× what the docs imply.

**P1:** Third-party deps (`osmnx`, `networkx`, `pyproj`, `shapely`) undeclared; `package.xml` declares 7 ROS deps that are entirely unused. `scripts/generate_graph.py` not installed by `setup.py`. `osmnx` not version-pinned (2.0+ broke many APIs). No unit test for schema generator. `overridable` flag missing on edges. Datum vs UTM-grid-north skew (~0.5° at CPP, sub-meter cross-track over 1 km).

**Positives:** schema correct, datum handling correct, bidirectional edges correct. Densification follows OSM road curves via `LineString.interpolate` (not chord-cutting). `mph → m/s` unit conversion correct. Boilerplate (`setup.cfg`, format-3 `package.xml`, resource marker) canonical.

**Source:** `package_avros_navigation.md` (315 lines, 2 P0 / ~7 P1 / ~5 P2).

### 10.6 avros_perception

**Files:** `perception_node.py` (344 LOC), `pipelines/{base,stub,hsv}.py`, `utils/class_map.py`, configs, launch, tests (~727 source LOC + 1,018 test LOC). **Highest test:source ratio in workspace (~1.4).**

**Verdict:** the integration contract with kiwicampus is mostly right (latched LabelInfo with TRANSIENT_LOCAL+RELIABLE QoS at `:202-209`, stamp/frame inheritance bit-exact onto mask/cloud/conf/overlay at `:301-302 → 305-327`, HxW resize *before* `pipeline.run()` at `:287-291` — the canonical ZED `pub_downscale_factor` vs `point_cloud_res` fix). Two real P0s.

**P0 (2):**
- `LabelInfo.header.frame_id` published empty at `:212` — `build_label_info(self._classes)` called without a `frame_id`. kiwicampus tolerates today but standards §1 contract violated.
- HSV barrel hue wraparound unhandled at `hsv.py:103-104` — single-range `cv2.inRange` on `H ∈ [5, 25]` silently misses pixels when sunset shifts orange across H=0/179 boundary.

**P1:** `_on_synced` callback not exception-safe (only cv_bridge wrapped — pipeline crashes propagate through message_filters). Pipeline hot-swap documented but unimplemented (read once at `:145`). Numeric class IDs duplicated 3 ways (`class_map.yaml`, `perception.yaml`, `hsv.py:31`). `adaptive_k=0` doesn't disable the bright-AND despite YAML comment claiming it does. Lane-bound defaults disagree between in-code (`:123-124`) and YAML (`:53-54`). Integration test never publishes synthetic image+cloud — `_on_synced` header propagation unverified. No HxW-resize regression test. `launch_pytest` missing from `<test_depend>`.

**Positives:** ZED v5 topic names baked in correctly (`rgb/color/rect/image`, `point_cloud/cloud_registered`) with v4 deprecation comment. Live-tunable HSV params via validated `add_on_set_parameters_callback` with atomic stage-then-merge pattern. Test infrastructure is workspace high-water mark — six reusable patterns (deterministic conftest, `launch_pytest` integration test with late-joining transient_local subscriber, SHA-256 threshold-drift guard, mock `ApproximateTimeSynchronizer`, live-tuning callback test, synthetic-numpy behavioral tests). Setup.py is the model for the workspace.

**Source:** `package_avros_perception.md` (279 lines, 2 P0 / ~10 P1 / ~10 P2).

### 10.7 avros_sim

**Files:** `avros_vehicle_driver.py` (145 LOC), 3 launch files, `nav2_sim_overrides.yaml`, `avros_webots.urdf`, `worlds/run_osm_import.sh` (~535 LOC). No tests at all.

**Verdict:** sim stack works for what it tests, but it tests the *wrong vehicle*. Sim is an Ackermann car; real is a tracked diff-drive. Multiple sim/real divergences mean sim success ≠ real success.

**P0 (4):**
- **Sim is Ackermann** (`Car.proto`, bicycle-model in `avros_vehicle_driver.py:79`); real is tracked diff-drive (track gauge 0.7366 m). Different control problems. Driver's `WHEELBASE = 1.23` (`:23`) is URDF chassis-length, not an axle distance.
- `avros_webots.urdf:31, 40` stamps images with `camera_color_optical_frame` / `camera_depth_optical_frame` — those are runtime-published by the realsense driver (which doesn't run in sim); `avros.urdf.xacro` doesn't declare them statically. Image-to-base_link transforms fail with LookupException.
- `sim_navigation.launch.py:114, 130` — both EKFs load the same `ekf.yaml` → dual-EKF pattern collapses; both publish same TF edge. **Likely also affects `avros_bringup`.**
- `worlds/run_osm_import.sh` hardcodes `$HOME/AVROS/` paths (deprecated workspace name).

**P1:** No wheel odometry publisher (sim's odom-EKF has only IMU input → unbounded drift). No cmd_vel deadman. No slew-rate / heading-hold matching real `actuator_node`. GPS datum mismatch (`34.059 -117.823` vs real `34.059270 -117.820934` — ~78 m). `nav2_sim_overrides.yaml:14` disables RPP collision detection entirely instead of setting voxel-layer `min_obstacle_height`. Local costmap sized 100×100 m at 0.5 m. No ZED stand-in. `sim.launch.py:50` xacro inflation needs `zed_wrapper` (not in package.xml exec_depends). `rclpy.init()` in driver `init()` not guarded against double-init. `scripts/diagnose_sim.py:13` claims `ros2 run avros_sim diagnose_sim` but `entry_points` empty.

**Positives:** topic names match real exactly. Configs reused from `avros_bringup` (not forked). Minimal `RewrittenYaml` override delta (23 lines). Cross-distro launch logic. Correct quaternion order. Correct lifecycle order. Proper `WebotsLauncher`+`WebotsController` pattern. `use_sim_time` set everywhere. Shutdown event handler closes the stack cleanly when Webots exits.

**Source:** `package_avros_sim.md` (174 lines, 4 P0 / ~12 P1 / ~10 P2).

### 10.8 firmware/

**Files:** `teensy_diff_drive/teensy_diff_drive.ino` (392 LOC), `teensy_diff_drive/teensy_bridge.py` (238 LOC), `phase{1-7}_*.py` (1,000+ LOC of bring-up scripts), `test.py`, three .md docs, `teensy_diag/teensy_diag.ino` (~2,280 LOC total).

**Verdict:** **the most engineered part of the workspace.** PID gains tuned to competition grade (99-100 % tracking, 0.83 % L/R sync). Three subtle SparkMAX FW 26.1.4 protocol bugs caught and fixed (kFF=16, cls=0 idx=0 velocity, cls=14 PARAMETER_WRITE). Correct `volatile` discipline on most ISR-shared state. No `String`/`malloc`/recursion/goto. `millis()` rollover-safe subtraction throughout. Bring-up trail (CLAUDE/BRING_UP/FINDINGS) is the model for the whole workspace. **The killer remaining work is hours of FW effort + e-stop wiring.**

**P0 (3):**
- No hardware e-stop wired into firmware. Firmware has no GPIO input for the mushroom-button signal, no latched-fault state. **Fails IGVC inspection day 0.**
- Watchdog trip leaves `ctrl_mode = MODE_VELOCITY` (`:360-361`). Wheels stop via velocity PID but Brake-idle never engages — chassis rolls on slope. **One-line fix:** add `ctrl_mode = MODE_DUTY;` after wheel zeroing.
- No hardware MCU watchdog (RTWDOG) as a backstop against firmware lockup.

**P1 (15):** auto-rearming watchdog, missing `volatile` on `rx_count`, torn-pair reads of `meas_rpm`/`meas_pos`, `delay(5)` and `delay(50)` in command handlers, silent serial-buffer overflow, `atof` returning 0 on garbage input (`KP<garbage>` writes 0 to flash), unconditional `BURN`, `test.py` raw `"BURN"` not gated.

**P2 (~25):** duplicate `Teensy` class, scattered magic numbers, dead `teensy_diag/` sketch, no CI for `arduino-cli compile`, stale `~/AVROS` references in BRING_UP.md.

**Positives:** competition-grade PID. Multiple SparkMAX protocol bugs caught. CAN heartbeat at 50 Hz, sent during watchdog trips. Documented bring-up trail is the workspace model. Reproducible bring-up (Phase 1–7 scripts + timestamped CSVs).

**Source:** `package_firmware.md` (285 lines, 3 P0 / 15 P1 / ~25 P2).

---

## 11. P1 — Professional Quality Issues

The full P1 list across all reports totals ~80 items. Grouped below by domain. For exhaustive lists with file:line citations, see the per-package reports under `docs/review_2026-05/working/`.

### 11.1 Safety & control (12)

P1 items from `crosscut_safety.md` punch list (#12–30):

1. Enable Teensy RTWDOG (~2 s timeout via `Watchdog_t4`).
2. Tighten `cmd_timeout_s` 0.5 → 0.3 (`actuator_params.yaml:29`).
3. Wire `msg.watchdog_active` (currently hard-coded false at `actuator_node.py:327`).
4. Bus-voltage host-side monitor — parse Teensy DIAG, soft stop on < 11 V.
5. Stop firmware watchdog auto-rearm — require explicit `S` to clear.
6. Server-side joystick watchdog in webui (`asyncio.wait_for(receive_json, 0.5)`).
7. Lower webui `max_throttle` 1.0 → 0.5.
8. NaN check on joystick floats (`webui_node.py:138-139`).
9. RTK status monitoring (subscribe `/gnss` → `NavSatStatus`, EKF outlier gate).
10. Verify LiDAR contributes to local costmap after voxel_layer P0 fix.
11. Nav2-callable software e-stop topic (`/avros/software_estop`) for BT-driven halts.
12. Set `RMW_IMPLEMENTATION` + `CYCLONEDDS_URI` in every launch file (currently only sensors.launch.py).

### 11.2 Build / deployment (13)

P1 items from `crosscut_build.md`:

13. `avros_navigation/setup.py` doesn't install `scripts/generate_graph.py` (reachable only by path-relative invocation).
14. `scripts/diagnose_sim.py` docstring claims `ros2 run avros_sim diagnose_sim` but file is at workspace root.
15. `scripts/deploy.sh` selectively builds 3 packages — no warning when others change.
16. `avros_perception/package.xml` missing `<test_depend>launch_pytest</test_depend>`.
17. `avros_sim/package.xml` missing exec_depends for `tf2_ros`, `nav_msgs`, `sensor_msgs`, `zed_wrapper`.
18. No CI/CD — drift accumulated unobserved.
19. `avros.repos:6-9` pins `xsens_mti` Paarseus fork without explanation comment.
20. `requirements.txt` no lockfile.
21. 3 packages missing LICENSE files (perception/sim/webui).
22. Top-level repo LICENSE missing.
23. `avros_control` duplicated `pyserial` dep (manifest + setup.py).
24. `.gitignore` missing common turds (`*.swp`, `.DS_Store`, `.idea/`, `.vscode/`, `*.bak`, `*~`, `.coverage`, `.pytest_cache/`, firmware bring-up `*.csv`).
25. No documented `colcon clean` workflow.

### 11.3 Documentation (20)

See §13 for the doc drift table. Highlights from `crosscut_documentation.md`:

26. CLAUDE.md missing rows: `avros_sim` not in Packages table; kiwicampus PR2/PR3 not in Known Issues.
27. `~/AVROS/` references in `xsens_mt_manager_setup.md` (`:87, 110`) and `firmware/teensy_diff_drive/BRING_UP.md:348`.
28. Consolidate the 7 dated changelog files (2,844 lines) into a single `CHANGELOG.md`.
29. All `.py` files lack copyright headers (`test_copyright.py` is `pytest.mark.skip` everywhere as a result).
30. Node `__init__` docstrings absent describing pubs/subs/params.
31. `webui_node.py` doesn't document the e-stop contract (the only software auto-e-stop path).
32. `PerceptionNode` class has no docstring.
33. TODO.md "Done" section grows unbounded — migrate to CHANGELOG.
34. Phase 2 reports not cross-referenced from TODO.md.
35. `PLAN_perception_phase4_phase5.md` needs a "what's executed" header.
36. No diagrams (vehicle photo, sensor placement, ROS topic graph, rendered TF tree).
37. Mismatched comments: `actuator_node.py` "closes the loop on ω", `nav2_sim_overrides.yaml` VelodynePuck-as-obstacles, `test.py` MAX_DUTY = 0.6.
38. HSV class IDs duplicated in 3 places.
39. Type hints sporadic — annotate every node callback method signature.
40. Magic numbers in actuator_node.py / hsv.py / firmware without named constants.
41. No Makefile for `.tex → .pdf` (committed PDFs go stale).
42. Hardware e-stop status invisible in CLAUDE.md despite being P0 inspection blocker.
43. Add "verified on YYYY-MM-DD" stamps to setup docs.
44. Track-width drift not propagated (TODO.md notes effective ≈ 0.7416 m vs configured 0.7366 m; CLAUDE.md still cites 0.7366).
45. `generate_graph.py` workflow undocumented as the only path.

### 11.4 Test coverage (8)

From `crosscut_tests_and_readiness.md`:

46. Extend `test_perception_launch.py` with synthetic image+cloud publishers (~50 LOC).
47. `test_hsv_hue_wraparound.py` — synthetic H=178 and H=2 both classified as barrel.
48. `avros_navigation/test/test_generate_graph_schema.py` — ~30 LOC, mock osmnx.
49. `avros_bringup/test/test_sensors_launch_smokes.py` — ~30 LOC.
50. `avros_sim/test/` — add 3 boilerplate linters at minimum (today silently a no-op).
51. `test_on_synced_header_propagation.py` — kiwicampus contract regression.
52. Pre-commit hook for `test_hsv_thresholds.py` SHA-256 hash guard.
53. `firmware/test/regression_check.py` — load Phase 4/6c CSV baselines, re-run.

### 11.5 Per-package P1s

For exhaustive per-package P1 lists, see:

- `package_avros_msgs.md` (4 items — placeholder version, dead PlanRoute.srv, hardcoded watchdog_active, incomplete LICENSE).
- `package_avros_bringup.md` (27 items — sensor URDF mounts, GNSS lever arm, nav2 vs nav2_humble drift, perception_test config staleness, RViz consistency).
- `package_avros_control.md` (12 items — no parameter callback, no QoS for IMU, blocking I/O, missing logger throttling, no unit tests).
- `package_avros_webui.md` (12 items — listed above).
- `package_avros_navigation.md` (~7 items — listed above).
- `package_avros_perception.md` (~10 items — listed above).
- `package_avros_sim.md` (~12 items — listed above).
- `package_firmware.md` (15 items — listed above).

---

## 12. P2 — Nice-to-haves

The full P2 list totals ~120 items. Categories with item counts:

| Category | Count | Examples |
|---|---|---|
| Style / naming | ~25 | All packages `<version>0.0.0</version>`; `avlab@cpp.edu` maintainer everywhere; broad globs in setup.py |
| Magic numbers | ~15 | `actuator_node.py` 0.02 / `_max_w * 0.5` / 1e-6 / `dt > 0.5`; firmware heartbeat bytes; sim driver constants |
| Doc grammar / formatting | ~10 | CLAUDE.md no "last-updated" header; changelog file naming inconsistency; .tex files lack obsoleted-by headers |
| Type hints | ~10 | Inconsistent across nodes; callback methods unannotated; no mypy config |
| Test depth | ~10 | `wrap_angle`/`yaw_from_quaternion` unit tests; CSV-baseline regression for firmware; launch_testing for actuator/webui/localization |
| Telemetry / diagnostics | ~10 | `diagnostic_msgs/DiagnosticArray` from all nodes; rqt_diagnostics view; safety-state UI panel |
| Refactor opportunities | ~15 | Phase 1–7 scripts duplicate boilerplate; could share a base class; class IDs single source of truth via LabelInfo |
| Future-proofing | ~10 | Wireless e-stop verification protocol; firmware compiled with `-Wall -Wextra -Wpedantic`; firmware-version banner; udev rule for Teensy |
| Documentation polish | ~10 | Mermaid TF tree; CONTRIBUTING.md / CODE_OF_CONDUCT.md / `.github/` templates; `docs/foxglove_layout.json` reference |
| License / hygiene | ~5 | `<author>` tags; `cmake_minimum_required(3.14)` in avros_msgs; remove `-Wall` etc. from pure-IDL package |

For exhaustive lists, see the "Punch list — P2" sections in each per-package report.

---

## 13. Documentation Drift Table

Source: `crosscut_documentation.md`. Every row is a place where a doc claims X but the code does Y.

| # | Doc claim | Code reality | Severity |
|---|---|---|---|
| D1 | README "Ackermann" / "SmacPlannerHybrid (Ackermann-aware)" | tracked diff-drive | **P0** |
| D2 | README "over UDP to a Teensy" | USB-CDC serial 115200 baud | **P0** |
| D3 | README "steering, throttle, brake, gear" | per-wheel velocity setpoints | **P0** |
| D4 | README `cd ~/AVROS` | workspace `~/IGVC` per CLAUDE.md:7 | **P0** |
| D5 | CLAUDE.md "52 nodes, 113 edges" | 16,651 nodes / 17,492 edges | **P0** |
| D6 | CLAUDE.md "zed_front.yaml HD720@15fps" | HD720 deprecated; YAML uses HD1080 | **P0** |
| D7 | actuator_node.py docstring "/wheel_odom @ 50 Hz" | 20 Hz | P2 |
| D8 | actuator_node.py comment "closes the loop on ω" | feed-forward boost only | P1 |
| D9 | xsens_mt_manager_setup.md `~/AVROS/install/setup.bash` | `~/IGVC` | P1 |
| D10 | firmware BRING_UP.md `~/AVROS/install/setup.bash` | same | P1 |
| D11 | CLAUDE.md Known Issues lists kiwicampus PR1 only | PR2 (mutex) + PR3 (raytrace-clear) also applied | P1 |
| D12 | CLAUDE.md track gauge 0.7366 m | measured ≈ 0.7416 m per TODO.md | P1 |
| D13 | perception class IDs "single source" | 3 sources: class_map.yaml + perception.yaml + hsv.py:31 | P1 |
| D14 | CLAUDE.md "Packages" lists 5 | repo has 7 (avros_sim missing) | P1 |
| D15 | scripts/diagnose_sim.py docstring `ros2 run avros_sim diagnose_sim` | not installed via console_scripts | P1 |
| D16 | nav2_sim_overrides.yaml "VelodynePuck sees ground" | `min_obstacle_height` problem | P1 |
| D17 | test.py "MAX_DUTY = 0.6" | firmware MAX_DUTY = 0.30 | P1 |

**Pattern:** drift concentrated in (a) public-face README (D1–D4), (b) numeric/version claims that age (D5, D6, D11, D12), (c) docstrings describing partially-realized behavior (D8, D13). CLAUDE.md is more current than README.md; TODO.md is more current than CLAUDE.md.

---

## 14. Build-System Issues

Source: `crosscut_build.md`. The build skeleton is correct — every package uses `<package format="3">`, build types match REP 149, resource markers and `setup.cfg` files exist in every ament_python package, and `avros_msgs` is correctly scheduled first by colcon. **What's not correct is everything between the skeleton bones.**

### 14.1 Missing exec_depends (causes runtime failures)

| Package | Missing |
|---|---|
| avros_bringup | `xsens_mti_ros2_driver`, `zed_wrapper`, `avros_perception`, `nav2_route`, `nav2_lifecycle_manager`, `semantic_segmentation_layer`, `tf2_ros`, `foxglove_bridge` |
| avros_webui | `python3-fastapi`, `python3-uvicorn`, `python3-websockets` (valid Humble rosdep keys exist) |
| avros_navigation | `python3-networkx`, `python3-pyproj`, `python3-shapely` (osmnx is pip-only — needs `requirements-graph-tool.txt`) |
| avros_sim | `tf2_ros`, `nav_msgs`, `sensor_msgs`, `zed_wrapper` |
| avros_perception | `launch_pytest` (test_depend) |

### 14.2 Dead exec_depends (build bloat)

| Package | Dead deps to remove |
|---|---|
| avros_navigation | `rclpy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `nav2_msgs`, `robot_localization`, `avros_msgs` (none imported — package has empty `__init__.py`; only `scripts/generate_graph.py` exists) |
| avros_webui | `std_msgs` (unused) |

### 14.3 Deploy script issues

`scripts/deploy.sh` (61 lines):

- Line 11 hardcodes `~/AVROS` (deprecated path per CLAUDE.md). Every "field deploy" today overwrites the wrong tree. **P0.**
- Line 44 `colcon build ... 2>&1 | tail -5` masks exit code (no `set -o pipefail`); failed builds treated as success → launch step runs against stale `install/`. **P0.**
- Line 44 selectively builds only 3 packages — changes to msgs/perception/navigation/sim go silently un-rebuilt. **P1.**
- Kill-everything-with-"ros"-in-cmdline (line 15) — footgun on shared dev machines. **P2.**
- **Positive:** correctly exports `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` + `CYCLONEDDS_URI` (lines 49, 52, 55).

### 14.4 Version pinning gaps

CLAUDE.md "Known Issues" mentions `numpy<2` constraint for JetPack 6, but it's enforced **nowhere** — not in requirements.txt, not in any setup.py, not in any package.xml `<depend>python3-numpy>`. Same story for `osmnx<2`. **P0 reproducibility.**

No lockfile (`Pipfile.lock` / `poetry.lock` / `requirements.lock`) anywhere. Every `pip install -r requirements.txt` resolves dynamically. **P1.**

`avros.repos:6-9` pins `xsens_mti` to a Paarseus fork branch `ros2` with **zero** explanation comment of why a fork is required. Compare to the gold-standard `semantic_segmentation_layer` block (lines 17-31) with a detailed patch-list comment. **P1.**

### 14.5 CI / automation

**No CI infrastructure exists.** No `.github/workflows/`, no `.gitlab-ci.yml`, no `Dockerfile`, no `pre-commit` config, no `tox.ini`, no `pyproject.toml`, no `Makefile`. The only test is "did `colcon build` succeed on the Jetson when the developer last ran `./scripts/deploy.sh`."

The 2026-04-29 changelog captures one such silent regression: the kiwicampus PR3 patch migration to `Paarseus/avros-fixes` required a manual `vcs import` step; anyone who didn't pull the new `avros.repos` got a build failure with no root-cause signal.

**Minimum viable CI** (~half-day work):
1. GitHub Action: `colcon build` on every PR (~30 LOC YAML using `osrf/ros:humble-desktop`).
2. GitHub Action: `colcon test`.
3. GitHub Action: `arduino-cli compile` for firmware (~10 LOC).
4. Pre-commit hook for HSV SHA-256 hash guard (~5 LOC).

---

## 15. Test Coverage & CI

Source: `crosscut_tests_and_readiness.md`.

### 15.1 Test inventory

| Package | `test/` contents | Real behavioral tests | Source LOC | Test LOC | Coverage |
|---|---|---|---|---|---|
| `avros_msgs` | (none) | 0 | 0 (msg/srv only) | 0 | N/A |
| `avros_bringup` | 3 boilerplate | 0 | 1167 | 73 | None — 9 launch files, 16 YAMLs entirely uncovered |
| `avros_control` | 3 boilerplate | 0 | 456 | 73 | **Worst gap** — 426 LOC of safety-critical code with zero behavioral assertions |
| `avros_navigation` | 3 boilerplate | 0 | 360 | 73 | 332-LOC `generate_graph.py` has no schema test |
| `avros_perception` | 3 boilerplate + 8 unit + 1 launch | 9 | 727 | 1018 | **Workspace high-water mark.** Ratio > 1.0 |
| `avros_webui` | 3 boilerplate | 0 | 246 | 29 | Disconnect → e-stop safety contract untested |
| `avros_sim` | (none) | 0 | 480 | 0 | No tests at all |
| `firmware/` | (none) | 0 | ~2280 | 0 | No `arduino-cli compile` CI hook |

**Workspace totals:** ~3,436 source LOC against ~1,264 test LOC, of which 1,018 LOC (81 %) live in `avros_perception`. Subtracting boilerplate: 1,018 LOC of real behavioral test code total, all of it covering one node in one package.

### 15.2 avros_perception as template

Six reusable patterns to extract to other packages:

1. **`conftest.py` for determinism** (`test/conftest.py:17-20`) — pins `cv2.setNumThreads(1)` and `np.random.seed(0)` before any test runs.
2. **`launch_pytest` integration test** (`test/launch/test_perception_launch.py`) — spawns real node in subprocess, late-joins a `_TopicWatcher` with the exact `transient_local + reliable` QoS, asserts the latched message arrives within 10 s.
3. **SHA-256 threshold-drift guard** (`test/unit/test_hsv_thresholds.py`, 78 LOC) — hashes the HSV tuning tuple and asserts against a checked-in `EXPECTED_HASH`.
4. **Mock `ApproximateTimeSynchronizer` tests** (`test/unit/test_sync_slop.py`, 149 LOC) — pure-Python mock subscribers push synthetic stamped messages through the real `message_filters` synchronizer.
5. **Live-tuning callback test** (`test/unit/test_hsv_live_tuning.py`, 115 LOC) — proves the shared mutable params dict + atomic validate-then-mutate pattern.
6. **Behavior tests on synthetic numpy frames** (`test/unit/test_hsv_pipeline.py`, 11 cases).

A single afternoon per package gets `conftest.py` + 5–10 unit tests. Every other Python package can crib `conftest.py` verbatim.

### 15.3 The single most important missing test

**`avros_control/test/test_kinematics.py`** (~120 LOC):
- 3 cases of `yaw_from_quaternion`
- 3 cases of `wrap_angle`
- 2 cases of diff-drive inverse (pure translation, pure rotation)
- 4 cases of asymmetric slew limiter (accel-clamped, decel-toward-zero, sign-change, at-target)
- 2 cases of midpoint-pose integrator

Per Phase 2: the slew limiter has subtle correctness ("took three reads to convince myself") and the midpoint integrator was a regression fix in CHANGELOG_2026-04-28. **No regression guard exists.** This is the single highest-leverage missing artifact in the workspace.

---

## 16. Positives

It would be unfair and inaccurate to leave the impression that this is a workspace in poor shape. The findings list is long because the review was thorough, not because the codebase is bad. Many parts demonstrate real engineering maturity worth preserving while the P0s are fixed.

### Architecture

- **Control architecture is conceptually right.** Last-wins between `cmd_vel` and `ActuatorCommand`; both feed the same slew limiter, IMU corrections, and freshness watchdog. WebUI and Nav2 produce identical chassis behavior — exactly the contract you want.
- **Slew limiting with separate accel/decel caps** (`actuator_node.py:263-269`) matches the trapezoidal-velocity-profile standard.
- **Midpoint pose integration** in `_publish_odom` is 2nd-order accurate; replaces a buggy forward-Euler that was a regression fix in CHANGELOG_2026-04-28.
- **Heading-hold gating** (`|w| < deadband AND |v| > 0.02`) correctly avoids stationary engagement.
- **Dual-EKF localization correct on the real-vehicle side** — local fuses IMU + wheel odom, global adds GPS; `broadcast_cartesian_transform: false` prevents TF loop. (Sim-side has a bug where both EKFs share one config — flagged P0.)
- **Nav2 stack assembled with deliberate engineering choices.** ZED wrapper gotchas avoided (namespace/node_name, v5 enums, optical-frame chain via `zed_macro.urdf.xacro`); RewrittenYaml two-stage merge for sim overrides.
- **`vcs import` source management is professional.** Heavy drivers pinned in `avros.repos`; the `Paarseus/avros-fixes` fork stacks four kiwicampus patches as commits — no `git am` step at install time.

### Firmware

- **The most engineered part of the workspace.** Competition-grade PID tune (99–100 % tracking, 0.83 % L/R sync). Multiple subtle SparkMAX FW 26.x protocol bugs caught and fixed pre-bring-up. Correct `volatile` discipline. No `String`/`malloc`/recursion/`goto`. `millis()` rollover-safe subtraction throughout. MAX_RPM/MAX_DUTY clamps.
- **CAN heartbeat at 50 Hz, sent during watchdog trips** — keeps SparkMAX enabled long enough for controlled stop.
- **Reproducible bring-up.** Phase 1–7 scripts + timestamped CSVs + `BRING_UP.md` form a model record.

### Perception

- **kiwicampus integration contract correct in `avros_perception`.** Latched LabelInfo (TRANSIENT_LOCAL + RELIABLE), bit-identical stamp/frame_id propagation, mono8 mask, organized HxW-matched cloud, image resize before pipeline. Every silent failure mode the standards doc warns about is guarded.
- **Live-tunable HSV params via validated `add_on_set_parameters_callback`** with atomic stage-then-merge — the only place in the workspace this canonical Humble pattern is exercised. Becomes the test template once `actuator_node` adds its missing parameter callback.
- **Test infrastructure is workspace high-water mark.** Six reusable patterns (above).

### Documentation

- **CLAUDE.md is the de facto README** — comprehensive, mostly correct. Most operations succeed by following it alone.
- **CLAUDE.md "Known Issues" table** (~20 rows, file:line cited) preserves institutional knowledge that competition teams typically lose between cohorts. The single biggest institutional asset.
- **`firmware/teensy_diff_drive/` 3-doc pattern** (CLAUDE / BRING_UP / FINDINGS) is a model that should be replicated in `avros_control` and `avros_perception`.
- **`actuator_node.py` module docstring** sets the bar for the workspace; every parameter has a units comment.
- **`perception_node.py` module docstring states the kiwicampus bit-identical-stamp contract** at lines 13-15 — the most load-bearing constraint of the perception path.
- **TODO.md** items reference file:line and capture measured ground truth ("effective track ≈ 0.7416 m vs configured 0.7366 m"). Healthiest doc in the workspace.
- **Setup docs are pinned and replayable** (REALSENSE_SETUP.md cites versions, issue links, build flags).
- **Changelogs preserve wrong-then-right reasoning** — CHANGELOG_2026-04-27 records that the GPU anomaly was `depth.depth_stabilization` not NEURAL_LIGHT.

### Build system

- **Every package uses `<package format="3">`** with correct build types. Resource markers and uniform `setup.cfg` files exist in every ament_python package.
- **`avros.repos` is well-curated** — five pinned repositories, gold-standard patch-list comment block on the `semantic_segmentation_layer` fork.
- **`avros_msgs` build-time deps correctly wired** — colcon schedules it first automatically.
- **`scripts/deploy.sh` correctly exports `RMW_IMPLEMENTATION` and `CYCLONEDDS_URI`** on every launch — addresses the known FastDDS/CycloneDDS interop bug.
- **`.gitignore` excludes the four upstream-cloned source trees** — prevents accidental commits of hundreds of MB of upstream code.
- **`avros_perception/setup.py` is the model for the workspace** — specific globs, correct entry_points, manifest matches imports.

### Per-package Phase 2 review trail

- **Every package has a technical-debt inventory with file:line pointers.** New contributors can ramp fast.
- **Persistent journald + bring-up CSVs** mean post-failure forensics will always have data.

---

## 17. Recommendations

### 17.1 This week (May 1–8)

- **Hardware:** order mushroom button + wireless e-stop kit + contactor + dedicated 48V→19V Jetson buck. Schedule one shop day to install all four.
- **Software:**
  - `actuator_node.py`: latch `_estop`; bound e-stop decel; serial liveness; IMU staleness. **Test on jacks before driving.**
  - `teensy_diff_drive.ino`: add `ctrl_mode = MODE_DUTY;` after watchdog wheel-zero (one line). Reflash. Slope test.
  - `webui_node.py`: bind to localhost or Tailscale-only; add token auth.
  - `pipelines/hsv.py`: second-range hue band for orange wraparound; add regression test.
  - `generate_graph.py`: add `--from-csv` mode.
  - **Rotate NTRIP credentials** (use env-var or `.gitignore`d local YAML).
- **Documentation:**
  - Rewrite README.md as a thin trampoline to CLAUDE.md (or replace its body). The Ackermann/UDP/throttle text is misleading every visitor.
  - Update CLAUDE.md drift (52→16,651, HD720→HD1080, missing `avros_sim`, kiwicampus PR2/PR3).
  - Create top-level `LICENSE` file.
- **CI:** stand up a minimal GitHub Action (`colcon build` on every PR, ~30 LOC YAML).

### 17.2 Adopt the iterative-write pattern for future agent reviews

Every Phase 2 / Phase 3 agent that **collected findings in memory and wrote at the end timed out** at the 30-minute stream watchdog. The agents that wrote a skeleton early and Edited section-by-section all completed cleanly. If you re-run this review or commission similar audits, instruct agents up-front to write incrementally.

### 17.3 Adopt `avros_perception` as the test template for every other package

A single afternoon per package, copying:
- `conftest.py` verbatim
- the `launch_pytest` pattern
- the SHA-256 threshold-drift guard pattern
- synthetic-input behavioral tests

Especially urgent for `avros_control` (`test_kinematics.py`).

### 17.4 Adopt the 3-doc firmware pattern (`CLAUDE.md` / `BRING_UP.md` / `FINDINGS.md`) for `avros_control` and `avros_perception`

- `CLAUDE.md` per package — the stable spec.
- `BRING_UP.md` — phase-by-phase test plan with safety gates.
- `FINDINGS.md` — post-mortem of bugs hit and fixed.

`avros_control` has safety-critical actuator code but no BRING_UP/FINDINGS equivalent. The bugs that motivated the slew limiter, midpoint integrator, and gyro-stabilized turns live only in commit messages.

### 17.5 Consolidate the changelogs

7 dated changelog files totaling 2,844 lines over 7 days projects to ~12,000 lines by competition. Roll into a single `CHANGELOG.md` at repo root (one `## YYYY-MM-DD` H2 per session, most-recent-first, ~10 lines each); keep detailed files in `docs/sessions/` for deep-dive readers; link from CLAUDE.md.

### 17.6 Decide week-1 whether to use `route_server` at competition

The committed `cpp_campus_graph.geojson` is the wrong graph and `generate_graph.py` has no IGVC-course path. The team can either (a) add `--from-csv` to `generate_graph.py` and hand-author from the IGVC organizer-provided map, or (b) skip route_server entirely and use `navigate_to_pose_simple_humble.xml` with hand-clicked Nav2 goals in RViz. Decide early. Option (b) drops the riskiest dependency.

### 17.7 Don't drive the vehicle until the safety chain is verified

The `crosscut_safety.md` failure-mode matrix has 12 P0 items. **Every P0 must be closed (or explicitly accepted with a written waiver and operational mitigation) before any non-jacked field test.** The "Inspection-day checklist" in §7 lists what to verify on the physical vehicle.

### 17.8 Out of scope for this review

This review is **static** — no code was executed. Items the team should also do:

- Run `colcon build` on a clean clone and document errors.
- Run `colcon test` and triage failures.
- Run a soak test of the safety chain (mushroom button + WS disconnect + cmd_vel timeout) with rosbag recording.
- Field-walk the IGVC course (or a representative venue) and capture a GPS path for the route graph.
- Verify `magnetic_declination_radians` with a known east-pointing path test.
- Audit hardware wiring, especially the safety circuit.

---

## 18. Working Artifacts

Detailed working reports for each phase live under `docs/review_2026-05/working/`:

**Standards research (Phase 1):**
- [`standards_ros2_python.md`](review_2026-05/working/standards_ros2_python.md) — ament_python conventions
- [`standards_nav2_localization.md`](review_2026-05/working/standards_nav2_localization.md) — Nav2 + robot_localization
- [`standards_perception.md`](review_2026-05/working/standards_perception.md) — vision_msgs + ZED + sync
- [`standards_firmware_safety.md`](review_2026-05/working/standards_firmware_safety.md) — embedded + safety control

**Per-package deep reviews (Phase 2):**
- [`package_avros_msgs.md`](review_2026-05/working/package_avros_msgs.md)
- [`package_avros_bringup.md`](review_2026-05/working/package_avros_bringup.md)
- [`package_avros_control.md`](review_2026-05/working/package_avros_control.md)
- [`package_avros_webui.md`](review_2026-05/working/package_avros_webui.md)
- [`package_avros_navigation.md`](review_2026-05/working/package_avros_navigation.md)
- [`package_avros_perception.md`](review_2026-05/working/package_avros_perception.md)
- [`package_avros_sim.md`](review_2026-05/working/package_avros_sim.md)
- [`package_firmware.md`](review_2026-05/working/package_firmware.md)

**Cross-cutting reviews (Phase 3):**
- [`crosscut_build.md`](review_2026-05/working/crosscut_build.md)
- [`crosscut_documentation.md`](review_2026-05/working/crosscut_documentation.md)
- [`crosscut_safety.md`](review_2026-05/working/crosscut_safety.md)
- [`crosscut_tests_and_readiness.md`](review_2026-05/working/crosscut_tests_and_readiness.md)
