# Next PR: Ground Test Plan, Field Kit, IMU Latency Fix

A running record of the work since PR #24 (`drive/fw26-motor-control`). It is also the draft PR description. Add each new change here as it is made.

**Base:** PR #24. Merge it first, or stack this PR on its branch.

## Summary
- A ground test plan for the drive, odometry and controller interface, to finish before MPPI is tuned.
- A field kit that records every ground session: settings, logs, journal, ROS bags.
- New analyses for spin centre, gyro scale, stationary checks and tape-measured distance.
- A permanent fix for bursty IMU data: the USB latency timer goes from 16 ms to 1 ms.

## Changes

### Ground test plan
| File | Change |
|---|---|
| `docs/drive_tuning_2026_09_28/GROUND_TEST_PLAN.md` | **New.** It contains:<br>- 8 scopes that run bottom-up;<br>- pass limits;<br>- which parameter each test decides;<br>- which reference each test uses (tape and chalk for Sessions 1–2; RTK only where required);<br>- a comparison with the research topics and the earlier plans;<br>- findings F1–F8. |
| `docs/drive_tuning_2026_09_28/GROUND_ACCEPTANCE.md` | Marked superseded. |
| `docs/drive_tuning_2026_09_28/MPPI_READINESS_TEST_PLAN.md` | Marked reference-only (its firmware-25 values are out of date). |
| `docs/drive_tuning_2026_09_28/README.md` | Index updated. |

### Field kit (`docs/drive_tuning_2026_09_28/tools/`)
| File | Change |
|---|---|
| `ground/README.md` | **New.** It covers:<br>- what gets recorded;<br>- equipment and robot reference marks;<br>- exact commands for each session. |
| `ground/session_start.sh` | **New.** Creates the session folder and saves the git version, config files, clocks, USB settings and controller configuration. |
| `ground/config_snapshot.py` | **New.** Reads every motor-controller setting (read only) and lists any difference from the saved setup. |
| `ground/bag.sh` | **New.** Records a ROS bag for a test and adds a row to the session journal. |
| `ground/imu_timing.py` | **New.** Measures IMU message spacing and timestamp age. |
| `ground/GROUND_RESULTS_TEMPLATE.md` | **New.** Template for the results file of each session. |
| `ground/delivery_campaign.sh` | **New.** The speed-delivery campaign (5 repeats of 0.05–0.7 m/s forward and reverse, 3 slow-spin repeats each way) with fault and bus-voltage guards. |
| `drive_tuner.py` | New `--rampdown` (smooth voltage ramp-down) and `--chain` for `ramp`/`steps`, `--m` (runtime speed ramp, restored to 100) for `vel`, `S:<seconds>` step in `vel --seq` (braked stop with logging continued). New `--test`, `--rep` and `--note` options. Writes `journal.csv` (one row per run). Saves runs into the session folder when `GT_SESSION` is set. |
| `analyze.py` | New subcommands:<br>- `circle`: spin centre and driven radius from GNSS;<br>- `gyroscale`: gyro scale against the RTK spin angle or counted turns plus chalk lines;<br>- `still`: IMU bias and RTK scatter;<br>- `tape`: distance per motor turn from tape-measured start and end marks;<br>- `stops`: braked-stop time, distance, rollback, bus voltage and current;<br>- `delivery`: steady speed vs command over many repeats (mean error, 95 % band, ripple, stuck share, left/right, pass/fail). |

### Xsens firmware 1.16.0 and configuration tools
| File | Change |
|---|---|
| `scripts/xsens/` | **New.** `xsens_inspect.py` (read the full device configuration), `xsens_live.py` (live GNSS status), `xsens_restore.py` (write the reference configuration + 921600 baud), `README.md` (setup, reference configuration, procedure after a firmware update). |
| `CLAUDE.md` | Firmware 1.16.0; Known Issues row: a firmware update resets baud and settings, and how to restore them. |

### Navigation test (2026-10-02)
| File | Change |
|---|---|
| `docs/nav_test_2026_10_02/RESULTS.md` | **New.** Four runs (95 m, 75 m, return as one goal, return in 8 m legs), findings, settings used, mistakes, open items. |
| `docs/nav_test_2026_10_02/{apply_test_config,make_waypoints,send_goal,course_watch,legs_from_fix}.py` | **New.** Reversible test settings, waypoint legs from a target, goal sender with metrics (not run live), GPS-course watcher with auto-cancel. |
| `docs/nav_test_2026_10_02/logs/` | **New.** Filtered Nav2 log, per-run mission logs, waypoint files. |
| `docs/obstacle_waypoint_test_plan_2026_10_02.md` | **New.** Plan for the lidar obstacle and waypoint test (stages 0-4). Stages 1-3 (static box, clean 6-8 m goal, obstacle) were NOT run. |

### Drive tuning (2026-10-02)
| File | Change |
|---|---|
| `drive_tuning_2026_09_28/TUNING_LOG.md` | **New.** Parameter register (SPARK, Teensy, host), results before tuning, method, experiment log E0–E4a, status snapshot. |
| `drive_tuning_2026_09_28/TUNING_TEST_PLAN.md` | **New.** Pre-registered plan for the next tuning phase (friction map, feedforward correction, bounded integral, confirmation, ground truth), with targets, statistics, confounders, safety. |

### Root-cause analysis (2026-10-08)
| File | Change |
|---|---|
| `drive_tuning_2026_09_28/ROOT_CAUSE_ANALYSIS_2026_10_08.md` | **New, then extended same evening.** Re-read the existing E0-E4a/GROUND_2026_10_02 data against FMEA/fishbone/DOE methodology (sourced). Headline finding: the E2 kS/kV correction fixed reverse crawl (-11%→-1.9%) but left forward crawl unchanged (-32%→-30%) — a direction-specific residual no existing doc explains. **Resolved same evening by N1 (§1 update, §7.1): it was a test-order/warm-up confound; alternating order passes ALL targets across the full 0.03-0.7 m/s grid.** Also surfaces: right-track L/R asymmetry is session-history-dependent, not fixed (3-9% mid-session, <1% late-session, 24.86% in one historical session) — needs the bench isolation test (N3), not more ground data; turning/spin failure is a separate axis from straight-line crawl (0.1 rad/s = fully stuck), untested tonight, still the top open item; `allowedClosedLoopError` (SPARK param 97) is untested; Hall-sensor quantization confirmed present (~22 RPM discrete steps observed live) but unlikely to explain the mean bias. §7 documents the firmware reflash/verification and the actuator_node gain-push fix (below). Adds tests N0-N6 and a run-manifest/session logging schema. |
| `firmware/teensy_diff_drive_v2/PROTOCOL.md`, `firmware/README.md` | Status updated: v2d marked **FINAL, flashed and ground-verified 2026-10-08** (was "compiled and desk-tested only, not flashed"). Deliberately reflashed from the exact source on disk (byte-identical to orphaned commit `3a3aecc`, never merged to `main`) to remove any doubt; boot banner read directly off hardware and recorded. |
| `src/avros_control/avros_control/actuator_node.py` | **Bug fix.** The startup SparkMAX-gain push ran before the serial-reader thread started and never verified the SPARK's own `PWR` confirmation — confirmed tonight to silently fail for `kFF`/`kP` (CAN round trip) while `kS_left`/`kS_right` (Teensy-local, no CAN round trip) reliably took. Added `_write_gain_verified()` (waits for/retries on the SPARK's `PWR res=0` reply, logs ERROR on failure instead of unconditional success) to both the startup push and the runtime `ros2 param set` path (`_on_param_change`); added `PWR` line parsing to the serial-reader thread. Deployed to the Jetson, rebuilt (`colcon build --symlink-install --packages-select avros_control`), verified end-to-end: fresh `ros2 run` now logs "confirmed by PWR replies" and an independent `PR B 16/13` readback matches the yaml (kV 0.00211, kP 0.0004) for the first time through the real launch path, not just the manual tuning harness. |
| `drive_tuning_2026_09_28/tools/ground/tune_set.sh` | **New.** Runs one parameter set end to end: verifies gains from the controllers' replies, runs the protocol, guards each run (faults, bus, current, oscillation, power-cycle), restores baseline on abort, appends results to a CSV. |
| `drive_tuning_2026_09_28/tools/analyze.py` | `delivery` (multi-repeat accuracy, stop behaviour, CSV), `ffid` (closed-form kV/kS identification), `guard` (per-run safety check). |
| `drive_tuning_2026_09_28/results/GROUND_2026_10_02.md` | **New.** Delivery campaign on concrete. |

### IMU USB latency fix
| File | Change |
|---|---|
| `scripts/udev/99-avros-xsens-latency.rules` | **New.** Sets the FTDI latency timer of the Xsens USB converter (VID 2639) to 1 ms at every boot and replug. |
| `docs/imu_usb_latency_2026_09_30.md` | **New.** Covers:<br>- why the chip holds data;<br>- measurements before and after;<br>- why the Xsens driver does not set it;<br>- the fix options compared;<br>- install, check and undo steps. |
| `CLAUDE.md` | Two Known Issues rows: the IMU burst fix, and IMU/GNSS timestamps before a GPS fix. |

## Verification
| Item | Result |
|---|---|
| IMU timing on the Jetson, 16 ms → 1 ms timer | spread 12.85 → 0.71 ms; messages < 2 ms apart 65 % → 0 %; max gap 38 → 19 ms; driver ran on with no errors |
| udev rule | installed on the Jetson 2026-09-30; `latency_timer` = 1 after `udevadm trigger` **and after a reboot** (confirmed 10:5x) |
| New analyses (`circle`, `gyroscale`, `still`, `tape`, `stops`) | tested on synthetic data: spin centre 0.313 m (0.31 set), gyro scale +0.37 % (+0.4 % set), arc length exact, stop distance exact (0.098 m) |
| `drive_tuner.py` journal | tested (rows written with test ID, repeat and stop flag) |
| Xsens firmware 1.12.0 → 1.16.0 | The update reset baud (115200), outputs, lever arm, platform, option flags and receiver options. Restored with `xsens_restore.py`; read back at 921600 matches the pre-update snapshot on every row. **Still to check on the Jetson: driver, topics, RTK.** |
| Field kit on the Jetson | copied; imports, help texts and shell syntax checked. **Not yet used in a real session.** |

## Deployment notes
- For a new Jetson, install the udev rule with the steps in `docs/imu_usb_latency_2026_09_30.md`.
- The files above were copied to the Jetson with `rsync` (2026-09-30). They are not committed there yet.

## Open items (not in this PR, need a decision)
| # | Item | Where |
|---|---|---|
| F1 | MPPI probably reads no odometry: `controller_server` has no `odom_topic`. Verify live, then add `odom_topic: /odometry/filtered`. | GROUND_TEST_PLAN §0 |
| F2 | MPPI `wz_max` 1.9 is above the actuator cap of 1.5. | same |
| F3 | Heading-hold is on under navigation. | same |
| F4 | GNSS antenna offset is not in the ROS pipeline. | same |
| F5 | IMU covariances are zero. | same |
| F6 | Stale statements in CLAUDE.md and comments (turn cap 1.0, EKF wheel yaw rate, `/wheel_odom` 50 Hz). | same |
| — | **Web UI stops driving after the physical e-stop is pressed and released** (e-stop cuts only the motor drivers; Jetson and Teensy stay powered). Cause not found yet: needs a controlled test with Teensy diagnostics. Related gap: `actuator_node` never reconnects if the Teensy USB drops (seen after a manual replug: `[Errno 5]` on the dead port). | session log 2026-09-30 |
| — | **ROS 2 messaging breaks when the Jetson's Wi-Fi disappears (2026-10-02).** With Ethernet and Wi-Fi both on 192.168.13.0/24, CycloneDDS autodetect bound nodes to the Wi-Fi; after the robot left range every write to 192.168.13.103 failed (hundreds of `ddsi_udp_conn_write` errors), topics vanished and the web UI did not move the robot. Recovered by restarting the sensor launch and web UI on Ethernet only (0 DDS errors afterwards). **Done:** AVL auto-join disabled and Wi-Fi disconnected (Ethernet only); web UI + sensors restarted, 0 DDS errors. Still optional: pin `cyclonedds.xml` to `eno1`; RMS and Sonesta Guest profiles still auto-join. | session log 2026-10-02 |
| — | **Jetson Tailscale offline off the robot network (2026-10-02).** Campus Wi-Fi (CPPGuest) had the preferred default route and its captive portal broke Tailscale's HTTPS; Jetson clock was 27 h behind with NTP stuck in backoff. Done: campus Wi-Fi profiles deleted. **Clock fixed (timesyncd restarted, +1 d 3 h) and Tailscale is back online.** New: the Jetson now has Ethernet `eno1` (192.168.13.10) and Wi-Fi AVL (192.168.13.103) on the same subnet, Wi-Fi preferred (metric 600), which causes asymmetric routing and stalling direct SSH. **Done:** Ethernet only (AVL autoconnect disabled). Consider keeping NetworkManager from ever preferring Wi-Fi over `eno1` (route metrics) and syncing time from GPS (chrony + the Xsens UTC) so the clock does not depend on the router passing NTP. | session log 2026-10-02 |
| — | Jetson clock is 60–140 ms off GPS time (systemd-timesyncd over a slow link: offset −138 ms, jitter 85 ms). The EKF mixes GPS-time IMU/GNSS stamps with Jetson-time wheel stamps. Fix: sync the Jetson to GPS time (chrony + gpsd/PPS), or stamp everything with the Jetson clock. | session log 2026-09-30 |
| — | IMU/GNSS stamps are Xsens power-on time until GPS time is valid (`time_option` 0), then jump. Options: `time_option` 1 or 2. | `docs/imu_usb_latency_2026_09_30.md` |

| — | Drive tuning (see `TUNING_LOG.md` §5): best set verified but not applied; right track is the weak one; the controller gains tested above kP 0.0004 sag the supply; integral term rejected at kI 0.0001. | `TUNING_LOG.md` |

## Log (add entries as work is done)
- 2026-09-29: ground test plan; field kit; new analyses.
- 2026-09-30: web UI stops driving after a physical e-stop (cause open); actuator_node has no serial reconnect (seen after a manual Teensy replug).
- 2026-10-02: Tuning experiments E0–E4a (feedforward identified: kV 0.00211, kS 0.40; kP 0.0004 best; integral rejected); test plan written; right-track concern recorded.
- 2026-10-02: Speed-delivery campaign on concrete (26 runs, no faults, bus >= 10.1 V): straight lines within limits at 0.3-0.5 m/s forward, reverse over-delivers +2-4 %, forward crawl -31 %; slow spins fail badly (0.3 rad/s delivers 60-70 %, 0.1 rad/s does not move). Speed loop needs re-tuning before the turn multiplier. See results/GROUND_2026_10_02.md.
- 2026-10-02: RTK FIXED reached outside (first at 19:36 local, 14 sats; over the first 10 min 39 % FIXED / 60 % FLOAT, 6 switches, mean 17 sats, HDOP 0.7-0.8) after the Xsens firmware 1.16.0 restore, a clean single-interface network and an open-sky location.
- 2026-10-02: Tailscale offline off-site traced to campus Wi-Fi route + stale clock; Wi-Fi profiles removed; clock synced and Tailscale back; dual-interface (Ethernet + AVL Wi-Fi) routing issue found, decision pending.
- 2026-09-30: Xsens firmware 1.12.0 → 1.16.0; update reset baud and settings; restored and verified with the new `scripts/xsens/` tools.
- 2026-09-30: Session 1 started on concrete (`~/ground_tests/2026-09-30_1059_concrete`): CHK OK, IMU bias −0.0095 °/s PASS, IMU spacing PASS; RTK stayed FLOAT; Jetson clock off 60–140 ms (internet time sync) — new open item.
- 2026-09-30: `drive_tuner` braked-stop step and `analyze stops` added.
- 2026-09-30: tape and chalk made the main references for Sessions 1–2; IMU USB latency fixed (udev rule), documented; timestamp issue found.
\n- 2026-10-02: Navigation test: lidar destination had reverted to .105 (fixed by the user); legs longer than the 45 s BT timeout abort; yaw in the map frame unstable (+-100 deg swings) with correct positions; return to start worked with 8.4 m legs. Details in `docs/nav_test_2026_10_02/RESULTS.md`.\n- 2026-10-08: Root-cause pass over the drive-accuracy data (`ROOT_CAUSE_ANALYSIS_2026_10_08.md`): found the E2 kS/kV fix is direction-asymmetric (reverse crawl fixed, forward crawl untouched) and right-track asymmetry is far worse in reverse (24.86%) than assumed; added tests N0-N6 and a run-manifest logging schema.
- 2026-10-08 (same evening): Ran N1 on the ground (`~/ground_tests/2026-10-08_1846_outdoor`, R0 gains reconfirmed live via `PR`: kV 0.00211, kP 0.0004, kS 0.400/0.390 — SPARK RAM had reverted to the old 0.0023/0.0002 baseline since `actuator_node` only pushes yaml gains on a live `ros2 param set`, never automatically at launch, a previously-unknown gap worth fixing). Alternating-order crawl (5 reps/direction, 0.05 m/s) gave L fwd -1.49%, L rev -4.35%, R fwd +2.27%, R rev +3.02% — ALL PASS, forward/reverse gap from E2 is gone. Confirms the gap was a test-order/warm-up artifact, not a real asymmetry; A2b/A2c/A2d and N2 are no longer needed. New finding: L/R match 3.8-7.7%, fails the 1.5% A4 target — feeds the still-pending right-track check (N3).
- 2026-10-08 (later same evening): Extended the alternating-order grid to the full Phase B range (0.03-0.7 m/s, both directions, both tracks, 76 runs) — ALL PASS against A1/A2/A3/A8. The planned Phase C speed-dependent correction table is very likely unnecessary for straight-line motion; the real remaining open items are the low-speed L/R and fwd/rev mismatch (3-9% below 0.4 m/s) and spin/turning (untested tonight, still confirmed-severe from Oct 2). Reflashed `firmware/teensy_diff_drive_v2/` deliberately to remove all doubt about source/flash correspondence — boot banner confirmed `# avros diff-drive bridge v2d ready (SPARK MAX FW 26.1.5)`, CHK OK, gains re-verified via PR, retest matched pre-reflash numbers (though the L/R mismatch shrank to <1% post-reflash — likely a session-warm-up artifact, not the reflash itself; strengthens the case for N3). Root-caused and **fixed** a real bug in `actuator_node.py`: the startup SparkMAX-gain push ran before the serial-reader thread started and never verified the SPARK's own PWR confirmation (only `kS_left/kS_right`, which has no CAN round trip, reliably took; `kFF/kP` could silently fail under load — confirmed happening for ~90 minutes of tonight's own webui session). Fix adds `_write_gain_verified()` (waits for and retries on the SPARK's PWR reply, logs ERROR on failure) to both the startup push and the runtime `ros2 param set` path; deployed, rebuilt, and verified end-to-end (`ros2 run` + independent `PR` readback both show kV=0.00211/kP=0.0004 post-fix). Full writeup: `drive_tuning_2026_09_28/ROOT_CAUSE_ANALYSIS_2026_10_08.md` §7.\n