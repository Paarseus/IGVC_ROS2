# Vehicle Integration Test Plan — V0 through V11

> **SUPERSEDED** by `docs/W_vehicle_integration_test_plan.md` (14 phases P0-P13, ~80 micro-steps). The W-plan reflects three post-V changes: (1) STVL migration on local_costmap — commits `4450003`+`4d04e3a` — clearing semantics totally rewritten; (2) NTRIP disabled per IGVC §I.2; (3) `observation_persistence` reverted to 0.0 (commit `9ea082d`). V3 and V5 here are obsolete for STVL — see P5 in W. V6-V11 fold into P13.1-P13.6 of W. **Read W first.**

**Purpose:** validate the full IGVC_ROS2 stack on the real vehicle, from minimal subsystem checks to full IGVC-like scenarios. Each step is small, has a clear pass/fail, and unlocks the next.

**Supersedes:** docs/field_test_plan.md (T1-T10) — that plan was written before the firmware MAX_RPM fix, the EKF VIO yaw-only fusion, the mission_manager build, and the latest perception/Nav2 tuning. The V-ladder reflects the current state.

**Already validated (don't re-run, just briefly re-verify in V0):**
- Localization at rest (T3.0b passed: 6 mm map-EKF drift in 60 s, 100% SBAS lock)
- Motor calibration + slew + heading-hold (T3.2 / F1 / F1b confirmed symmetry)
- E-stop on ground (T3.0a confirmed, user-reported "robot moved + e-stop worked")
- Firmware sustained 1.5 m/s commanded → 1.06 m/s actual peak / 0.58 m/s mean (F1c)
- mission_manager skeleton + waypoint sequencing on fake odom (M6 verified)

**Untested on real vehicle:** Velodyne costmap marking on grass, HSV outdoor daylight, local-costmap dynamics with live obstacles, **Nav2 actually driving the robot**, BT recovery actions, lane-as-LETHAL behavior, obstacle avoidance, combined IGVC scenarios.

---

## Safety contract

- **Wireless e-stop in your hand is the primary safety.** Foxglove/Tailscale is secondary — assume the network can drop.
- **One operator at a time** with visual line of sight to the robot.
- **Speed cap stays low** until V8 passes. Default `vx_max: 0.7` (current setting) gives ~0.5 m/s achievable. Don't ramp until everything below works.
- **Fail-closed.** Any unexpected behavior → e-stop → rollback to the prior rung → debug → retry.
- **No chaining rungs in one launch.** Stop and re-launch fresh between rungs.

---

## Materials & site

- Open empty parking lot or grass field, ≥30×30 m, ≥10 m forward clearance, ≥5 m radius for rotations. Sky view required for GPS.
- White tape (2-3 in athletic or painter's), ~50-100 ft total for lane corridor (V10/V11).
- 1-2 traffic cones or small barrels, heavy enough not to blow over.
- Charged 24 V battery (>70 % for any session).
- Phone or laptop with Foxglove Studio + Tailscale. Webui as tertiary backup.
- Wireless e-stop accessible at all times.
- Run log: timestamp, rung, args used, outcome, observations.

---

## Universal pre-session checklist (run every session)

1. Battery >70 %.
2. E-stop button physically tested today.
3. `ssh jetson` succeeds.
4. `ping -c 1 192.168.13.11` → Velodyne reachable.
5. `ros2 topic hz /gnss` → ≥4 Hz, status ≥1 (SBAS or better). **If `/gnss` is empty or status=0, do not proceed.**
6. `ZED_Explorer` (via NoMachine) sees serial `49910017`.
7. `navsat.yaml` datum is within ~100 m of the test site. If you've moved sites, update `datum: [lat, lon, ...]` first or `/fromLL` gives bogus offsets.
8. Briefly re-verify V0 and V1 once per session before doing anything new.

---

## Session bundles (suggested)

| Session | Rungs | Duration | What you prove |
|---|---|---|---|
| **A** | V0 + V1 + V2 | ~45 min | Baseline: stack alive, e-stop kills, motors drive |
| **B** | V3 + V4 + V5 | ~60 min | Sensors tuned for outdoor + costmap reacts to real obstacles |
| **C** | PRE-V6 + V6 + V7 + V8 | ~60 min | **Nav2 actually drives the robot** — the gating test, then mission_manager |
| **D** | V9 + V10 | ~60 min | Obstacle avoidance + lane following, separately |
| **E** | V11 (+ contingency for any failed rung) | ~30-60 min | IGVC integration |

Total: ~4-5 sessions, ~3 hours of motion time across them. Sessions are NOT same-day — pace by daylight, battery, operator fatigue.

---

## V0 — Pre-flight, no motion

**Goal:** confirm every subsystem is alive and publishing at the right rate.

**Pre-conditions:**
- Universal checklist passed
- Nav stack launched: `ros2 launch avros_bringup navigation.launch.py enable_velodyne:=true enable_zed_front:=true enable_perception:=true perception_cameras:=front enable_mission_manager:=false bt_xml:=navigate_igvc_autonav_humble.xml`
- Wait 30 s for full lifecycle activation + GPS first-fix

**What I'll do (verify each, all should pass):**

| Topic | Expected rate | Tolerance |
|---|---|---|
| `/gnss` | 4 Hz, status ≥ 1 | drop = HARD FAIL |
| `/imu/data` | 100 Hz | ±10 Hz |
| `/wheel_odom` | 20 Hz | ±2 Hz |
| `/avros/wheel_debug` | 50 Hz | ±5 Hz |
| `/odometry/global` | ~30 Hz | ±5 Hz |
| `/odometry/filtered` | ~30 Hz | ±5 Hz |
| `/zed_front/zed_node/rgb/color/rect/image` | 10 Hz | ±2 Hz |
| `/zed_front/zed_node/pose` | 15 Hz | ±3 Hz |
| `/velodyne_points` | ~10 Hz | ±2 Hz |
| `/perception/front/semantic_mask` | 10 Hz | ±2 Hz |
| `/perception/front/label_info` | latched (once) | present = pass |
| `/local_costmap/costmap` | 10 Hz | ±2 Hz |

**Success:** all 12 topics at their expected rates. `/odometry/global` initial drift < 0.5 m over 30 s static. Foxglove shows ZED image + Velodyne cloud + costmap layers.

**What you do:** Stand by, observe Foxglove.

**Failure modes & abort:**
- ZED image black or grossly under/overexposed → ZED exposure config issue, debug at bench
- /velodyne_points absent → Velodyne network / driver problem
- /perception/front/semantic_mask empty → perception_node failure
- /odometry/global jumps > 2 m static → bad GPS multipath; reposition

**Time budget:** 10 minutes including the launch.

---

## V1 — E-stop verify on ground

**Goal:** confirm the wireless e-stop kills motor command instantly with the robot on the ground (lower-risk than wheels-lifted; we already verified end-to-end working in T3.0a).

**Pre-conditions:** V0 passed. Robot oriented into ≥5 m clear forward area.

**What I'll do:**
1. Bag-record `/cmd_vel /avros/actuator_state /avros/wheel_debug /odometry/filtered` for 6 s.
2. Publish `/cmd_vel linear.x=0.3` for 2 s.
3. Stop publishing; motors brake on cmd_vel timeout.

**What you do:** As soon as the robot begins to move, **press the physical e-stop**. After ~2 seconds total of motion, the robot should be stopped.

**Success:** robot moves < 30 cm; motors stop within ~1 s of button press (or by cmd_vel timeout if you don't press — that's also a valid safety path).

**Failure modes:** motors keep spinning > 1 s after press → HARD STOP whole session. Bench-debug e-stop chain.

**Time budget:** 5 minutes.

---

## V2 — Manual teleop drive (webui)

**Goal:** confirm motors balanced, heading-hold engaging, on this surface today.

**Pre-conditions:** V1 passed.

**What I'll do:** Stop nav launch. Launch webui:
```
ros2 launch avros_bringup webui.launch.py
```
Wait for "Uvicorn running on https://0.0.0.0:8000" log.

**What you do:**
1. Open `https://<jetson-ip>:8000` on phone, accept self-signed cert.
2. Joystick: forward at ~30 % throttle for 3 s → straight line, ≤ 5° heading drift.
3. Stop. Joystick left for 2 s → rotates left.
4. Stop. Joystick right for 2 s → rotates right.
5. Stop. Joystick down for 2 s → reverses.

**Success:**
- Forward drives straight (operator eyeballs the trajectory)
- Heading drift on straight line < 5° (Foxglove `/odometry/filtered.yaw`)
- Rotations smooth, no track stall
- Reverse works cleanly

**Failure modes:**
- Veers consistently right > 10° in 3 m → motor friction asymmetry; CLAUDE.md says right track has 8% higher friction; for now accept and let MPPI close the loop. Note for V6.
- One direction won't rotate → mechanical / SparkMAX issue; bench

**Time budget:** 10 minutes.

---

## V3 — Velodyne grass-return tuning

**Goal:** find a `min_obstacle_height` that filters grass returns but keeps real obstacles (barrel, person).

**Pre-conditions:** V2 passed. Robot stationary in grass; place a barrel 3 m in front.

**What I'll do:**
1. Stop webui. Launch nav stack as V0.
2. Foxglove → visualize `/local_costmap/voxel_marked_cloud` AND `/velodyne_points`.

**Iterate (you + me):**
1. Current `min_obstacle_height: -0.5` (sensor-relative). Observe: is grass appearing as LETHAL cells around the robot?
2. If yes: raise to `-0.4`, `-0.3`, `-0.2` until grass returns vanish from costmap.
3. After each change: `ros2 param set /local_costmap/local_costmap voxel_layer.velodyne_points.min_obstacle_height <value>` (won't persist) AND/OR edit `nav2_params_humble.yaml` + rebuild for permanence.
4. STOP raising when the barrel STARTS disappearing from `/local_costmap/costmap` (the value above this is the working range).

**What you do:** Walk around the robot to see if humans appear as obstacles. Stand next to the barrel as a sanity check.

**Success:** grass not marked, barrel marked, human walking near robot is marked.

**Failure modes:** can't find a value that filters grass without losing barrels → Velodyne mount tilt issue; investigate.

**Time budget:** 20 minutes (this is a TUNING phase; allow iteration).

**Document:** record the chosen value + the conditions (grass height, sunlight) in CHANGELOG.

---

## V4 — HSV lane-detection tuning

**Goal:** find `lane_low` / `lane_high` HSV thresholds for THIS lighting that reliably detect white tape on grass/concrete without false positives.

**Pre-conditions:** V3 passed. Lay a 2 m strip of white tape ~3 m in front of the robot, on the surface you'll test on.

**What I'll do:**
1. Foxglove → `/perception/front/overlay` (the BGR + semantic-mask blend) AND `/perception/front/semantic_mask` (the raw classification image).

**Iterate (you + me):**
1. Current `lane_low: {V: 130}` (last tuned for blue-cast scenes). On sunny pavement, this may be too low (causing concrete itself to match).
2. Tighten S ceiling first (`lane_high.S`) to reject blue-cast non-tape. Then walk V up.
3. After each change: `ros2 param set /perception_front lane_low.V <value>` etc. (dynamic if the node supports it; may need relaunch).
4. PASS when ≥80 % of tape pixels classified `lane_white`, no false positives on grass / concrete / robot shadow.

**Document:** record the chosen values + conditions (cloudy / sunny / shadow / sun-angle) in CHANGELOG. Expect to re-tune on different days.

**Time budget:** 20 minutes.

---

## V5 — Local costmap dynamics

**Goal:** confirm the costmap MARKS new obstacles AND CLEARS them when removed, in both LiDAR (voxel_layer) and camera (semantic_layer) channels.

**Pre-conditions:** V3 + V4 passed.

**What I'll do:** Stack already running. Foxglove on `/local_costmap/costmap`.

**Iterate (you + me):**
1. Place barrel 3 m in front. Observe: LETHAL cell appears within ~1 s (voxel_layer).
2. Move barrel 1 m sideways. Old cell clears within ~3 s; new cell appears.
3. Remove barrel entirely. All cells clear within ~3 s.
4. Lay 1 m of white tape in front. Observe: lane_white LETHAL cells appear.
5. Pick up tape. Cells clear within ~3 s (kiwicampus raytrace-clear).

**Success:** both channels mark + clear within ~3 s.

**Failure modes:**
- Voxel cells linger after barrel removed → voxel `clearing` not configured correctly; check yaml
- Tape cells linger → kiwicampus `clearing: true` and `tile_map_decay_time: 0.3` should be set; verify (we recently confirmed)
- Whole costmap red → HSV thresholds too permissive; rollback to V4 tuning

**Time budget:** 15 minutes.

---

## PRE-V6 — Param prep (one-time)

Before V6, raise MPPI `vx_max` from `0.7` to `1.0` to match the firmware-fix achievable speed (sustained 1.06 m/s peak per F1c).

Edit `src/avros_bringup/config/nav2_params_humble.yaml`:
```yaml
vx_max: 1.0   # was 0.7; firmware MAX_RPM fix (commit 875ce3c) now allows
              # sustained 1.06 m/s peak on grass
vx_min: -0.6  # was -0.4; symmetric scaling
```

Rebuild `avros_bringup`, push. Verify `ros2 param get /controller_server FollowPath.vx_max` reads `1.0` after relaunch.

**Time budget:** 10 minutes (workstation work, not field).

---

## V6 — Single Nav2 goal in open area (THE GATING TEST)

**Goal:** Nav2 actually drives the robot to a goal. **This is the gating moment** — simultaneously exercises Navfn planner, MPPI controller, IGVC BT, costmaps, and EKF localization in motion for the first time.

**Pre-conditions:** V0-V5 passed. PRE-V6 param change live. Open area, ≥8 m clear forward. No obstacles in the planned path.

**What I'll do:**
1. Launch full stack as V0 (`enable_mission_manager:=false`).
2. Foxglove panels: `/local_costmap/costmap`, `/plan` (path), `/odometry/global`, `/cmd_vel`.
3. Bag-record everything.
4. Send a single Nav2 goal 5 m in front, via Foxglove "Publish Goal" tool or:
   ```
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
     "{pose: {header: {frame_id: 'map'}, pose: {position: {x: <robot_x + 5*cos(yaw)>, y: <robot_y + 5*sin(yaw)>}, orientation: {w: 1.0}}}}"
   ```

**What you do:** Hand on e-stop. Robot will drive 5 m at up to 1.0 m/s. Approximate travel time: 7-10 s with slew.

**Success:**
- Navfn produces a path (visible as `/plan` line in Foxglove)
- MPPI commands `/cmd_vel` to follow the path
- Robot drives at < 1.0 m/s (peak < firmware ceiling)
- Robot stops within `xy_goal_tolerance: 2.0` m of the goal
- BT reports SUCCEEDED
- No recovery actions fired (no Wait / BackUp / DriveOnHeading)

**Failure modes & abort:**
- Robot doesn't move → `/cmd_vel` not flowing; check controller_server log
- Robot oscillates / hunts → MPPI tuning issue; lower CostCritic.cost_weight
- Robot stops short → costmap has phantom obstacle in path; check V3/V4 tuning
- Robot drives wrong direction → frame mismatch; STOP, debug
- BT triggers Wait or BackUp → MPPI couldn't progress; check the costmap for false LETHAL cells

**Time budget:** 20 minutes (allow for re-tries).

---

## V7 — mission_manager + 1 GPS waypoint

**Goal:** the orchestrator drives the BT through 1 real waypoint.

**Pre-conditions:** V6 passed.

**What I'll do:**
1. Edit `src/avros_bringup/config/waypoints.yaml` to a SINGLE point at lat/lon ~5 m forward of robot start.
2. Launch full stack with `enable_mission_manager:=true`.
3. Bag-record everything.

**What you do:** Hand on e-stop. Robot should drive to the waypoint and idle.

**Success:**
- mission_manager log shows: started → /fromLL converted → wp[0] sent → accepted → robot drives → proximity tick fires within 2 m of waypoint → "mission complete"
- 20 s TimerAction delay from the launch (per commit `a44220e`) gives nav2 lifecycle time to activate before mission_manager sends its first goal

**Failure modes:**
- /fromLL returns (0, 0, 0) → GPS hadn't converged at TimerAction expiry; raise the timer to 30 s
- Goal REJECTED → bt_navigator wasn't fully active; raise timer
- Robot never reaches waypoint → MPPI giving up; check log for ABORTED status

**Time budget:** 15 minutes.

---

## V8 — Multi-waypoint sequence (3-4 points)

**Goal:** cursor-advance + cancel-and-resend works for real, with no stops between waypoints.

**Pre-conditions:** V7 passed.

**What I'll do:**
1. Edit `waypoints.yaml` to 3 or 4 points forming a known shape (e.g., a triangle with 5 m sides).
2. Launch same as V7.
3. Bag-record.

**What you do:** Hand on e-stop. Robot drives through all waypoints.

**Success:**
- mission_manager log shows: wp[0] reached → wp[1] sent → wp[1] reached → ... → "all 3 waypoints reached — mission complete"
- No `Wait` or `BackUp` recovery fires between waypoints
- Total time roughly `path_length / vx_max` (no long stalls)
- ABORTED results for canceled wpN goals are correctly filtered by the `index != cursor` race fix (commit `f742565`); cursor doesn't get stuck

**Failure modes:** robot stops at each waypoint → goal_checker firing AND cancel taking too long; check timing in the bag.

**Time budget:** 20 minutes.

---

## V9 — Single static obstacle (barrel)

**Goal:** Nav2 detours around an obstacle MPPI sees.

**Pre-conditions:** V6 passed (V7/V8 not strictly required — they're orthogonal).

**What I'll do:**
1. Place a barrel 2-3 m in front of the robot, offset 0.5 m to one side. Goal point 5 m ahead.
2. Launch full stack, send goal (manual or mission_manager).
3. Confirm in Foxglove that the barrel appears as LETHAL in `/local_costmap/costmap`.
4. Bag-record.

**What you do:** Hand on e-stop.

**Success:**
- `/plan` shows path bending around the barrel
- Robot maintains ≥0.5 m clearance from the barrel (inflation_radius 0.65 + robot half-width)
- No collision
- Goal reached

**Failure modes:**
- Robot collides → MPPI didn't see the costmap update OR CostCritic.cost_weight is too low; bump to 5.0 (was already in P3 plan)
- Robot stops short → can't find a path with that inflation; lower `inflation_radius`

**Time budget:** 20 minutes.

---

## V10 — White-tape lane corridor (no obstacles)

**Goal:** lane-as-LETHAL keeps the robot inside the corridor.

**Pre-conditions:** V6, V4 (HSV) passed.

**What I'll do:**
1. Lay a 2 m wide × 6-10 m long corridor of white tape. Goal point at the far end (past the tape).
2. Confirm both tape lines appear in `/local_costmap/costmap` as LETHAL (Foxglove).
3. Place robot at one end, send goal.

**What you do:** Hand on e-stop.

**Success:**
- Robot stays between the lines (visual + Foxglove path)
- MPPI's CostCritic steers away from LETHAL cells
- Goal reached without crossing either line

**Failure modes:**
- Robot crosses tape → CostCritic.cost_weight too low; raise from 3.81 toward 5.0 (per P3 plan)
- Robot stops mid-corridor → corridor too narrow for inflation overlap; widen corridor OR shrink inflation

**Lane widths to test:** 2 m first, then 1.5 m (stress test). IGVC's actual width is 10-20 ft = 3-6 m.

**Time budget:** 25 minutes.

---

## V11 — Combined IGVC scenario

**Goal:** the real deliverable. Lane corridor with one barrel inside. Robot must either stop, wait, or cross a lane briefly to dodge.

**Pre-conditions:** V9 + V10 passed.

**What I'll do:**
1. Set up the V10 corridor.
2. Place a barrel in the middle, ~4 m from the start.
3. Send goal at the far end.

**What you do:** Watch carefully. This is the most informative single test in the whole plan.

**Success (in priority order):**
- No collision with barrel
- Robot does not get permanently stuck
- Robot reaches goal, even if it crossed a lane briefly (IGVC penalty is acceptable; failure to finish is not)

**Failure modes:**
- Stuck → tune CostCritic lower or widen lane
- Collides → STOP, debug

**This phase informs IGVC strategy:** does our stack make the right scoring trade-off (dodge with penalty vs wait vs fail)? Logging answers.

**Time budget:** 25 minutes.

---

## Speed ramp schedule (across sessions, after V8)

| Session | `vx_max` |
|---|---|
| Session C (first autonomy) | 0.7 (default after PRE-V6 → 1.0) |
| Session D | 1.0 |
| Session E | 1.0 (sustained at 1.0; firmware limit) |

Don't bump above 1.0 — firmware/SparkMAX cap is ~1.06 m/s peak / 0.58 m/s mean per F1c. Setting vx_max higher just samples rollouts the motors can't execute.

---

## Diagnosis triage (if a rung fails)

Capture:
- `ros2 bag record /odometry/global /cmd_vel /local_costmap/costmap /plan /perception/front/overlay /tf` (30 s window covering the failure)
- Foxglove screenshots at the failure moment

Most likely culprits, in order of frequency:
1. CycloneDDS / RMW mismatch in shell → check `RMW_IMPLEMENTATION` env var (CLAUDE.md Known Issues)
2. HSV thresholds wrong for current lighting → re-run V4
3. Velodyne min_obstacle_height not right → re-run V3
4. GPS multipath / poor sky view → relocate
5. EKF tuning (especially after VIO yaw-only change) → check `/odometry/global` for jumps
6. `enable_perception` accidentally false in launch args
7. The kiwicampus class fix (`c9083d7`) not built in install — `colcon build --packages-select avros_bringup --symlink-install` on Jetson

---

## What this plan does NOT cover

- Ramps / inclines (no test materials)
- Switchbacks (defer until lane following is rock-solid)
- IGVC's "No-Man's-Land" (the 4-waypoint hint section — no mock available)
- Self-Drive (high-speed track) — out of scope, AutoNav-only per memory
- Weather / dusk variation (note conditions; re-tune HSV per session)
- Long-duration thermal behavior (6-minute IGVC run on full charge)

---

## Final reminder

The robot can hurt you and damage itself. **Hand on the e-stop. Eyes on the robot. Foxglove on the side.** When in doubt: stop, return to bench, debug.
