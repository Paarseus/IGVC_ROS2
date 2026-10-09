# Navigation test results, 2026-10-02 (evening, concrete, RTK FIXED)

Plan: `../obstacle_waypoint_test_plan_2026_10_02.md`. Logs: `logs/` (filtered Nav2 log, one mission log per run, waypoint files). Tools in this folder: `apply_test_config.py` (reversible test settings), `make_waypoints.py` (split a target into legs), `send_goal.py` (one goal with metrics; **never run live**), `course_watch.py` (GPS course vs needed direction, auto-cancel), `legs_from_fix.py`.

## What was run
| Run | What | Result |
|---|---|---|
| 0 | Stack start, pre-flight | IMU 100 Hz, odometry 20 Hz, lidar **no packets** (see 1.), then 10 Hz; RTK FIXED (GGA quality 4, 22-26 satellites, HDOP 0.6, sigma 1 cm) |
| 1 | 95 m to 34.059635,-117.821028 in 4 legs of 23.8 m | Operator stopped it: it first drove about 7 m southeast, away from the target (started facing -35 deg, target at +23 deg). wp0 aborted |
| 2 | 75 m to **34.059554,-117.821200**, first leg 8 m then about 22 m legs | Drove the right way (GPS course within 5-13 deg of the line) but **all 4 legs ABORTED** (skip-on-failure), robot stopped about 17 m from where the watcher began. Mission manager still printed "mission complete" |
| 3 | Return to the run-2 start as ONE 38 m goal | Drove 14.6 m toward it (38 -> 25 m, course 220 vs 241 deg), then ABORTED at the 45 s BT timeout |
| 4 | Return again as **3 legs of 8.4 m** | **Worked**: wp0 reached (1.95 m), wp1 aborted after recovery moves (25 s), wp2 reached (1.94 m); robot back at the start. (My monitor started after it finished, so "not moving" was a misreading.) |

## Findings
1. **Lidar destination reverts after a robot restart.** The VLP-16 sent to 192.168.13.105 (not the Jetson .10): the driver logged "poll() timeout" every second and `/velodyne_points` had no data. The lidar only keeps a destination across a power cycle if "Save Configuration" was clicked. It was back at .10 later (changed by the user; the Jetson then received 10 Hz). Fix to make permanent: set host .10 and Save Configuration in the lidar web page.
2. **The BT has a hard 45 s timeout per goal** (`navigate_igvc_autonav_2026.xml`, competition value). At the 0.33 m/s test speed that is at most about 15 m, so legs of 22-24 m can never finish. Use legs of about 10 m or less at this speed (25 m at 0.7 m/s). Memory note: `project_bt_45s_timeout_leg_length`.
3. **`mission_manager` reports "mission complete" even when every leg ABORTED** (skip-on-failure). Check each `wpN result` line.
4. **`controller_server` had no `odom_topic`** (finding F1 confirmed in the file). Added `/odometry/filtered`; live parameter confirmed. Whether it improved MPPI is not measured yet.
5. **Map yaw is unstable.** While the GPS course was steady, `/odometry/global` yaw swung between about -100 and +58 deg within seconds (several times per run). Map positions are correct (the start fix converts exactly to the map pose), so the problem is heading only. Xsens General_RTK has no magnetometer yaw. Likely contributor to the first run's wrong start and to the 360 deg turns the operator saw. **Cause not yet isolated.**
6. **Turns:** commanded turn rates reached -1.3 rad/s (cap `wz_max` 1.5). Measured open-loop slow-spin delivery is 16-29 % short (0.3 rad/s), 0.1 rad/s does not move; the 1.19 multiplier was calibrated with older gains. Under-delivery makes turns slower, so it does not explain fast turns or 360s.
7. **No reverse by design:** `vx_min` 0 (set 2026-06-01 after a 10 s backward drive). The only reverse is the 0.25 m BackUp recovery.
8. Control loop: 1 missed 20 Hz deadline in the whole session; load 1.2-4.7; RTK stayed FIXED except two brief single-fix dips.

## Settings applied for these tests (reversible)
`apply_test_config.py apply` with backups `*.navtest.orig` next to each file: actuator `heading_hold_deadband` 0.0, `max_linear_mps` 0.4, `kFF` 0.00211, `kP` 0.0004, `kS_left` 0.40, `kS_right` 0.39; Nav2 `vx_max` 0.35, `wz_max` 1.5, `controller_server.odom_topic` `/odometry/filtered`. Revert: `python3 apply_test_config.py revert --actuator <yaml> --nav2 <yaml>`. **Not committed.** The web UI and any later launch also use these values (speed cap 0.4 m/s).

## Mistakes during the session (for the record)
- I told the operator to aim the robot using the unconverged map yaw.
- I planned legs longer than the 45 s timeout allows.
- A stop command accidentally launched an empty `send_goal` (killed, no goal active, robot already stopped) and a `pkill -f` pattern killed my own SSH session once.

## Open items (bottom to top)
- Motor control: crawl under-delivery (forward -12/-20 % at 0.05 m/s), spin delivery, friction map and feedforward correction, bounded integral term, right-track inspection (51-52 A peaks, May bearing failure), stop harshness, gains only in RAM/yaml (not burned).
- Actuator node: turn multiplier recalibration with a gyro on the ground, separate angular decel, serial reconnect after a Teensy drop.
- EKF: yaw instability above, zero IMU covariances, 1970 timestamps before GPS time, GNSS antenna offset 0.76 m not applied, wheel-odom scale not verified with a tape, wheel-odom covariance not derived from data.
- MPPI/Nav2: measure the effect of `odom_topic`; cap `wz_max` (suggest 0.6-0.8) and tune turning costs; Humble ignores `ax_max`/`az_max`; `mission_manager` completion message; leg length vs 45 s timeout.
- Next test: record a bag, find the cause of the 360 deg turns, with `wz_max` lowered first.
