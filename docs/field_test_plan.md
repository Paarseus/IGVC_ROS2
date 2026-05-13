# Field-test plan — IGVC AutoNav stack, piece-by-piece outdoor validation

**Purpose:** validate the full nav stack outdoors in real conditions before the full IGVC course is available. Each phase isolates one new capability with everything from prior phases as the known-good baseline. If a phase fails, you fall back to the prior phase.

**Read once end-to-end before the first session.** Phases are NOT meant to be improvised.

---

## Safety contract — non-negotiable

- **One operator at a time.** That operator has visual line of sight to the robot, hand on the wireless e-stop, and is the sole authority to initiate autonomous motion.
- **Wireless e-stop is the primary safety.** Foxglove + Tailscale is **secondary** — assume the network can drop at any moment.
- **Start at low speed.** Every phase begins with `vx_max` at 0.3-0.5 m/s. Ramp only AFTER the phase passes at the prior speed.
- **Fail closed.** Any unexpected behavior → e-stop → return to the prior known-good phase → debug → retry.
- **Stop and restart freshly between phases.** Do not chain Nav2 actions across tests in one launch.

---

## Materials & site

- Open empty parking lot ≥30×30 m. After-hours university lots are ideal. Must have sky view (no garages, no dense canopy).
- White tape (2-3 in athletic or painter's tape), ~50-100 ft total for lane corridors.
- 1-2 traffic cones or small barrels, heavy enough not to blow away.
- Charged 24V battery pack, capacity for 2+ hours.
- Phone or laptop with Foxglove Studio, Tailscale connected, browser pointed at `https://<jetson-tailscale-ip>:8000` for the webui as a tertiary backup.
- Wireless e-stop button accessible at all times.
- Notebook for run log: timestamp, phase, args used, outcome, observations.

---

## Universal pre-session checklist

Run before EVERY session, not just the first:

1. Battery >70%.
2. E-stop reachable, button physically responds (click).
3. `ssh jetson` succeeds; `ros2 daemon stop` (clean slate).
4. `ping -c 1 192.168.13.11` succeeds (Velodyne).
5. `ros2 topic hz /gnss` shows ≥4 Hz with GPS quality ≥2 (DGNSS) or better — if RTK FIX, even better. **If GPS not converged, do not proceed past T1.**
6. ZED Explorer or `ZED_Explorer` confirms serial 49910017 is online (on Jetson via NoMachine).
7. **Datum sanity check**: open `src/avros_bringup/config/navsat.yaml`. The `datum: [lat, lon, ...]` should be within 100 m of your test site, OR you need to update it before the session. Wrong datum → `/fromLL` gives bogus map-frame offsets.
8. Re-verify T1 (e-stop kill) once per session, no exceptions.

---

## Phase T1 — Wireless e-stop kill (wheels lifted)

**Goal:** confirm the e-stop instantly kills motor command, end-to-end, with measurable latency.

**Setup:** Robot on cinder blocks or stands — **wheels free-spinning, no contact with ground.** Bringup launched, motors armed.

**Procedure:**
1. `ros2 launch avros_bringup actuator.launch.py` (just the actuator, no Nav2).
2. `ros2 topic pub --rate 5 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}}'` — both tracks should turn forward.
3. Press wireless e-stop. Observe.
4. Re-arm (release e-stop), confirm motion resumes.
5. Repeat steps 3-4 three times.

**Success criteria:**
- Motors stop within visibly <1 s of button press (sub-200 ms is ideal).
- After re-arm + new `/cmd_vel`, motion resumes.
- No motor coast (Brake idle mode is set on SparkMAXes per CLAUDE.md).

**Abort criteria:** any case where motors continue spinning after e-stop press for >2 s.

**If it passes:** proceed to T2 same session.

**If it fails:** STOP. Debug at bench. Check Teensy heartbeat handling, SparkMAX `K`/`S` command, wireless receiver wiring. Do not attempt any phase past T1 until e-stop is bulletproof.

---

## Phase T2 — Manual teleop drive (open area)

**Goal:** confirm motors are balanced, robot drives straight on the actual outdoor surface, slew rates are sensible.

**Setup:** Robot on the ground in the parking lot. Open area, no obstacles within 10 m. `vx_max: 0.5` in actuator config (default).

**Procedure:**
1. `ros2 launch avros_bringup teleop.launch.py`.
2. Keys: `i` (forward) for 3-5 s. Observe — does the robot drive straight, or does it veer?
3. Stop with `k`. Visually measure heading change: should be ≤5° over 3 m of forward travel.
4. Test `j` (left turn) and `l` (right turn). Should rotate smoothly in place.
5. Test `,` (backward) for 2 s. Should reverse cleanly.
6. Drive a rough 5 m × 5 m square. Note the closure error (return to start point).

**Success criteria:**
- ≤5° heading drift on 3 m straight drive.
- In-place rotation is smooth, no jerk, no track stall.
- Closure error on a 5×5 m square is ≤1 m.

**Failure modes to watch for:**
- Veers right: right track friction higher (known per CLAUDE.md, 8% friction asymmetry). Compensate via SparkMAX PID tune at bench, not in field.
- Jerky acceleration: slew rate too aggressive — already tuned to 0.5 m/s²·linear, 1.0 rad/s² angular per recent commits; if still jerky, drop further.
- One track stalls: SparkMAX cable, CAN bus, or PID issue. Bench debug.

**If it passes:** proceed to T3.

**If it fails:** rollback to bench, fix the motor calibration. The robot must drive straight before any autonomy.

---

## Phase T3 — Localization-only outdoor verification

**Goal:** confirm EKF + GPS + IMU + ZED VIO produce sensible odometry outdoors.

**Setup:** Robot in parking lot. Foxglove on phone/laptop showing `/odometry/global` as a 3D pose + `/gnss` as fix status.

**Procedure:**
1. `ros2 launch avros_bringup navigation.launch.py enable_velodyne:=false enable_zed_front:=true enable_perception:=false enable_mission_manager:=false`.
2. Wait 30 s for everything to come up + GPS to converge.
3. **Static check:** robot stationary 60 s. `/odometry/global` should drift <0.5 m horizontal. Note the drift magnitude.
4. **Drive check:** via webui (phone) or teleop, drive a 10 m straight line at 0.5 m/s. The `/odometry/global` trajectory should be roughly straight (some GPS noise tolerable).
5. **Closure check:** drive a 10 m × 10 m square back to start. Final `/odometry/global` should be within 2 m of the start pose.
6. **VIO check:** view ZED's `/zed_front/zed_node/pose` (or its odom equivalent). Should also track motion.

**Success criteria:**
- Static drift ≤0.5 m in 60 s.
- 10 m straight drive: trajectory deviation ≤1 m laterally.
- 10×10 m square closure: ≤2 m error.

**Failure modes:**
- Large jumps in `/odometry/global` → GPS multipath; move to better-sky-view location.
- Continuous slow drift → EKF tuning issue OR magnetic interference. Try repositioning.
- VIO disagrees significantly with GPS → ZED tracking lost (low texture, glare); reduce VIO weight in ekf.yaml.

**If it passes:** localization is good enough for waypoint navigation. Proceed to T4.

**If it fails:** debug at bench. Cannot proceed to autonomous phases without trustworthy localization.

---

## Phase T4 — Sensor verification at rest

**Goal:** confirm each sensor produces valid data outdoors.

**Setup:** Robot parked. Use Foxglove for visualization. All sensors enabled.

**Procedure:**
1. `ros2 launch avros_bringup navigation.launch.py enable_velodyne:=true enable_zed_front:=true enable_perception:=true perception_cameras:=front enable_mission_manager:=false`.
2. Wait 30 s.
3. **LiDAR check:** Foxglove → /velodyne_points. Walk around the robot (5 m radius). You should appear as a moving cluster in the cloud.
4. **Camera check:** Foxglove → /zed_front/zed_node/rgb/color/rect/image. Image is live, clear, not over-exposed.
5. **Lane detection check:** lay a 1 m strip of white tape on the ground 2-3 m in front of the robot. View `/perception/front/overlay` in Foxglove. The tape should be highlighted as `lane_white` in the overlay.
6. **Class verification:** `ros2 topic echo /perception/front/label_info --once` should show 5 classes (free, lane_white, barrel_orange, pothole, unknown).
7. **Mask check:** `/perception/front/semantic_mask` should be a mostly-empty image (free=0) with a small region marked 1 (lane_white) where the tape is.

**Success criteria:**
- LiDAR points cloud renders, you appear in it.
- ZED image is clear (not blown out, not too dark).
- White tape is detected by perception with ≥80% of its pixels classified `lane_white` (eyeball check).
- No "CRITICAL ERROR" log entries (the kiwicampus fix from c9083d7 should have silenced these).

**Failure modes:**
- White tape NOT detected → HSV thresholds need recalibration for this lighting. Edit `perception.yaml` `lane_low: {V: ...}` and `lane_high: {S: ...}`. Iterate until detection is solid. **Document the values that work in your site's lighting** — different sites may need different thresholds.
- Tape detected but also grass/concrete: thresholds too permissive. Tighten.
- LiDAR shows ground returns within 1 m of robot → adjust `voxel_layer.velodyne_points.min_obstacle_height` (currently -0.5 = 0.5 m above sensor).

**If it passes:** sensors are field-ready. Proceed to T5.

**If it fails (HSV):** stay at T4 and iterate HSV until lane detection is reliable in this lighting. This is the most likely site-specific tuning step.

---

## Phase T5 — Costmap dynamics at rest

**Goal:** confirm the local costmap correctly marks AND clears obstacles.

**Setup:** Same as T4 (full stack except mission_manager).

**Procedure:**
1. Place a barrel/cone 2-3 m in front of the robot.
2. Foxglove → `/local_costmap/costmap` as a 2D image. Confirm a LETHAL (red/pink) cell appears at the barrel's position.
3. Move the barrel 1 m to the side. Confirm the costmap LETHAL cell **moves with it** (old position clears within ~2 s).
4. Remove the barrel entirely. Confirm the costmap clears (raytrace clearing + tile_map_decay_time = 0.3 s should clear within 1-2 s).
5. Repeat with white tape: lay a 2-3 ft strip of tape. Confirm LETHAL cells appear along the tape. Pick up the tape; confirm the lane cells clear within a few seconds.

**Success criteria:**
- New obstacle marks LETHAL within 1 s of placement.
- Removed obstacle clears within 3 s (kiwicampus raytrace-clear + voxel raytrace).
- No "phantom" LETHAL cells lingering in the rolling window.

**Failure modes:**
- Cells don't clear after obstacle removal → semantic_segmentation_layer's clearing is broken. Verify `clearing: true` and the fork's raytrace-clear patch is applied (it was, per the avros-fixes branch).
- Costmap is fully red (everything LETHAL) → HSV thresholds are matching too much (grass classified as lane). Tighten thresholds.

**If it passes:** the perception → costmap pipeline is healthy. Proceed to T6.

**If it fails:** stay at T5, debug clearing.

---

## Phase T6 — Autonomous drive, single waypoint, open area, NO obstacles

**This is the bridge from "stack works" to "autonomy works."** Split into two sub-phases:

### T6a — Manual goal via RViz/Foxglove

**Procedure:**
1. Full launch: `ros2 launch avros_bringup navigation.launch.py enable_velodyne:=true enable_zed_front:=true enable_perception:=true perception_cameras:=front bt_xml:=navigate_igvc_autonav_humble.xml enable_mission_manager:=false`.
2. **Keep `vx_max: 0.3` in `nav2_params_humble.yaml`** for this first run — lower than the default 0.5.
3. Send a goal pose via Foxglove (or `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose ...`) 3-5 m directly in front of the robot in the open area.
4. Hand on e-stop. Watch the robot.
5. Robot should drive forward, slow on approach, stop within 2 m of the goal.

**Success criteria:**
- Robot drives toward the goal at ≤0.4 m/s.
- Trajectory is smooth (no oscillation, no sudden turns).
- BT reports SUCCEEDED when within `xy_goal_tolerance: 2.0`.
- No recovery behaviors fired (no Wait, BackUp, DriveOnHeading).

### T6b — mission_manager-driven, same scenario

**Procedure:**
1. Edit `waypoints.yaml` to a SINGLE lat/lon point ~5 m in front of the robot's start pose.
2. Same launch as T6a but with `enable_mission_manager:=true`.
3. Robot should drive to the waypoint and the log should show "all 1 waypoints reached — mission complete".

**Success criteria:** same as T6a, plus mission_manager log shows clean cursor advance.

**If T6a passes but T6b fails:** mission_manager integration issue. Inspect launch log for the 20 s TimerAction firing correctly, `/fromLL` conversion log line.

**If T6a fails:**
- Robot doesn't move → check `/cmd_vel` topic. If empty, controller_server not commanding. Check BT log.
- Robot moves wrong direction → footprint orientation, frame_id mismatch. Bench debug.
- Robot oscillates → MPPI tuning issue. Try lowering `vx_max` further.
- BT reports ABORTED → check `/global_costmap` for missing path; verify Navfn isn't refusing.

**Speed ramp after pass:** repeat T6b at 0.5, 0.8, 1.0, 1.2, 1.5 m/s in successive sessions. Each step should produce a smooth trajectory.

---

## Phase T7 — Autonomous drive with ONE static obstacle

**Goal:** verify Nav2 plans around obstacles and MPPI avoids them.

**Setup:** Same as T6b. Goal 5 m in front. Place one barrel **2-3 m in front of the robot, slightly offset to one side** so the planner has to deviate.

**Procedure:**
1. Launch as T6b with single waypoint.
2. Confirm the barrel appears in `/local_costmap/costmap` before sending goal.
3. Robot should plan around the barrel (verifiable via `/plan` topic in Foxglove — the path bends around the barrel).
4. Robot drives, executes the curve, reaches goal.

**Success criteria:**
- Robot maintains ≥0.5 m clearance from the barrel (inflation_radius 0.65 + robot radius 0.8 = ~1.4 m total — should be visible in the costmap).
- No collisions.

**Failure modes:**
- Robot collides → MPPI didn't see the costmap update OR rolled out through it. STOP, debug. May need to increase `CostCritic.cost_weight`.
- Robot stops short (refuses to move) → barrel is inside the planner's "no path" zone. Verify planner can find any path at all.

---

## Phase T8 — Lane following with white-tape corridor (NO obstacles)

**Goal:** verify lane-as-LETHAL behavior keeps the robot in a corridor.

**Setup:** Lay a 2 m wide × 6-10 m long white-tape corridor in the parking lot. The two parallel tape lines define the lane. Place the robot at one end pointing down the corridor. Place a single waypoint at the far end of the corridor (just past the tape ends).

**Procedure:**
1. Confirm both tape lines appear in `/local_costmap/costmap` as LETHAL (Foxglove).
2. Launch T6b config + mission_manager.
3. Robot should drive down the center of the corridor without crossing either tape line.

**Success criteria:**
- Robot stays between the lines for the full length.
- MPPI's CostCritic correctly steers away from the lethal cells.
- Final position is past the tape, within 2 m of the goal.

**Failure modes:**
- Robot crosses tape → CostCritic weight too low. Raise from 3.81 toward 5.0 (per P3.1 plan).
- Robot stops mid-corridor → too much inflation overlap. May need to widen corridor or shrink inflation.
- Robot oscillates side-to-side → MPPI hunting. May need to drop `vx_std`/`wz_std`.

**Lane width to test:** 2 m, then 1.5 m. IGVC lane width is 10-20 ft (3-6 m) but tight 1.5 m is a stress test of the controller.

---

## Phase T9 — Multi-waypoint sequence (no obstacles)

**Goal:** verify mission_manager chains waypoints without stopping.

**Setup:** Open area, no obstacles. Put 3-4 lat/lon waypoints in `waypoints.yaml` forming a known shape (line of 3 spaced 5 m apart; or a triangle with 5 m sides).

**Procedure:**
1. Launch as T6b config.
2. Watch the robot — it should hit each waypoint, advance the cursor on proximity, and proceed to the next WITHOUT stopping.

**Success criteria:**
- Robot reaches all 3-4 waypoints in order.
- No `Wait` recovery fired between waypoints.
- Total mission time consistent with `path_length / vx_max` (no stalls).
- `mission complete` logged at the end.

**Failure modes:**
- Robot stops at each waypoint → goal_checker tolerance is firing AND mission_manager is slow to cancel. Verify the `_advance_cursor` path completes quickly.
- Robot drifts past a waypoint without firing proximity → `/odometry/global` not converging fast enough. Look at GPS quality.

---

## Phase T10 — Combined: lane corridor with one barrel inside

**Goal:** the IGVC-like scenario — lane following with a real obstacle to avoid.

**Setup:** Same tape corridor as T8 (2 m wide × 8 m long). Place a barrel in the middle of the corridor, ~4 m from the start.

**Procedure:**
1. The robot needs to either: (a) stop and wait, (b) cross a lane line briefly to dodge the barrel, or (c) refuse to plan.
2. Per IGVC rules, lane-crossing-to-dodge IS legal with a penalty. The robot SHOULD attempt to dodge.
3. Watch what it does. This is the most informative single test in the whole plan.

**Success criteria (in priority order):**
- Robot does not collide with the barrel.
- Robot does not get permanently stuck.
- Robot reaches the waypoint past the corridor, even if it crossed a lane.

**Failure modes:**
- Stuck (refuses to plan, recovery loops) → tune `CostCritic.cost_weight` lower or widen lane temporarily.
- Crosses lane and gets confused → expected for v1; this is where No-Man's-Land logic would help in a future iteration.
- Collides → STOP. Debug.

**This phase informs IGVC strategy:** does our stack handle the actual scoring trade-off? Does it choose dodge over wait? Logging answers.

---

## Optional Phase T11 — Recovery behavior exercise

**Goal:** trigger each recovery action deliberately to verify the BT's RoundRobin works.

**Procedure:** with a controlled wedged state (e.g., place barrels surrounding the robot leaving only one exit), send a goal that requires the robot to plan through the wedge. Observe RoundRobin: ClearAroundRobot → Wait → BackUp → DriveOnHeading. Each should fire once before the action ABORTs.

This is exercise, not validation. Useful before competition to know your recovery ladder works.

---

## Speed ramp schedule (across multiple field-day sessions)

| Session | Target `vx_max` | Phase target |
|---|---|---|
| 1 | 0.3 m/s | T1-T6a baseline |
| 2 | 0.5 m/s | T6b - T9 |
| 3 | 0.8 m/s | Repeat T6-T9 |
| 4 | 1.0 m/s | Repeat T9 - T10 |
| 5 | 1.2 m/s | T10 stress test |
| 6 | 1.5 m/s | Full speed test, all phases |

**Do not jump speed steps.** Trust the controller at the current step before bumping.

---

## If the test fails — diagnosis triage

Before re-running, capture:
- `ros2 bag record /odometry/global /cmd_vel /local_costmap/costmap /plan /perception/front/overlay` (just the failing window, ~30 s).
- Foxglove screenshots at the failure moment.
- `journalctl -n 200 -u <service>` if anything is under systemd.

Most likely culprits in order of frequency (per CLAUDE.md known issues):
1. CycloneDDS / RMW mismatch (CLI vs launch nodes) — confirm `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in shell.
2. HSV thresholds wrong for site lighting.
3. GPS multipath / poor sky view.
4. EKF tuning (especially after VIO addition).
5. `enable_perception` accidentally false in launch args.

---

## What this plan does NOT cover

- Ramps/inclines — IGVC has ramps; we have none on hand. Defer.
- Switchbacks / sharp turns — defer until lane-following is rock solid.
- Multi-day or weather variation — note weather conditions for each session; expect HSV recalibration if cloudy vs sunny.
- Wireless/Tailscale failure handling — accept that you lose Foxglove if Tailscale drops. Physical e-stop is the safety.
- Self-Drive (high-speed) tests — out of scope; we're AutoNav-only per the team memory.

---

## Final reminder

The robot can hurt you and damage itself. **Hand on the e-stop. Eyes on the robot. Foxglove on the side.** When in doubt: stop, return to bench, debug.
