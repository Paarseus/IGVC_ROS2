# 04 — Nav2 Fine Motion in Tight Obstacle Fields

Scope: MPPI, Navfn, costmaps (resolution, footprint, inflation, STVL), BT and checkers, and how they interact with `actuator_node`. Everything here comes from reading the deployed snapshot (`deployed_snapshot/`) and upstream source. No robot access was used. "Probable" and "Hypothesis" items need the field tests in §4.

## Summary

- **The costmap can thread the IGVC gap, but with little margin.** The minimum passage is 5 ft (1.524 m). That leaves 0.347 m of slack per side for the 0.83 m robot. The 0.2 m local costmap can eat up to one cell (0.2 m) of that per side, leaving about 0.15 m per side in the worst case.
- **Inflation 1.0 m does not close the gap.** The footprint's inscribed radius is only 0.279 m, so lethal and inscribed cells stop 0.28 m from each barrel. The rest of the corridor is a smooth cost basin (cost about 75 at the center of a 1.52 m passage).
- **New, probable, highest impact: `base_link` is not the rotation center.** `base_link` is at the IMU. The tracks are centered 0.314 m ahead of it (URDF tread x = +0.3143). MPPI's DiffDrive model, the footprint rotation and the wheel odometry all assume the robot spins about `base_link`, but it spins about the track center.
  - A 90° spin moves `base_link` about 0.44 m, and the EKF does not see it.
  - The real rear corners sweep a 0.72 m radius. MPPI predicts 0.50 m.
  - A 4 × 90° square test cannot show this error, because it cancels over a full 360°.
- **New, probable: the soft obstacle cost is sampled only at `base_link`.** Humble's CostCritic reads the cost at the trajectory point, which is 0.81 m behind the nose. The front 0.8 m of the robot is protected only by the binary footprint check, so centering in a gap starts late.
- **Known, still active:**
  - Heading-hold replaces MPPI ω below 0.05 rad/s.
  - MPPI assumes instant acceleration while the actuator slews at 0.3 m/s² and 1.2 rad/s². A ±0.5 rad/s reversal takes 0.83 s, which is up to 0.4 rad of heading lag in a slalom.
  - Odometry lags 185 ms.
  - Map→odom movement (0.76 m lever arm, GNSS noise) moves the global path that PathAlign (weight 12) holds the robot to.
- **New, hypothesis:** a barrel beside the robot inside the LiDAR `min_range` 0.7 m can no longer be seen. It is likely cleared within about 3 s (voxel decay, frustum acceleration). The BT's ClearCostmapAroundRobot also wipes exactly that zone.
- **Recommended:**
  - Today: tests T1–T7 plus a few runtime A/Bs (heading-hold off under Nav2, PathAlign weight down, speed limit in tight sections).
  - Next restart: local costmap at 0.1 m resolution in a 20 × 20 m window (fewer cells than today), local STVL decay raised to 5 s.
  - Later: move `base_link` to the track center.

## 1. Findings

| # | Finding | Severity | Status | Evidence | New vs known |
|---|---|---|---|---|---|
| F1 | `base_link` (IMU) is 0.314 m behind the track and contact-patch center, which is the approximate spin center. MPPI DiffDrive (vy = 0), the footprint rotation, `/wheel_odom` and the EKF all treat `base_link` as the spin center. Effects: `base_link` moves 2·0.314·sin(θ/2) per spin (0.16 m at 30°, 0.44 m at 90°), and the real rear-corner sweep is 0.724 m against 0.500 m predicted, so the rear clips barrels during in-place turns. | High | Probable (geometry confirmed; the real ICR depends on weight distribution after the battery move) | `urdf/avros.urdf.xacro:35-38,71-94` (tread 0.8255 m long, centered x = +0.3143); footprint `nav2_params_igvc_autonav.yaml:287`; `actuator_node.py` IK `v ∓ ω·L/2` gives the velocity of the track midpoint | **New.** Related to known finding 13 (unconstrained vy), but this root cause is new |
| F2 | CostCritic's soft cost is sampled at `base_link` only (`costAtPose(x, y)`, orientation-free). The footprint is used only for the binary LETHAL check, which starts once center cost ≥ 51.7 (the circumscribed cost). With `base_link` 0.81 m behind the nose, the front of the robot gets no centering gradient until it is 0.8 m into a gap. | Medium | Confirmed (source); impact is a hypothesis | Humble `cost_critic.cpp` `score()` / `inCollision()` ([source](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_mppi_controller/src/critics/cost_critic.cpp)) | New |
| F3 | The local costmap is 50 × 50 m at 0.2 m (250 × 250 = 62,500 cells, matching what was seen live). Obstacle and footprint quantization costs up to 0.2 m per side, and STVL voxel-center projection adds up to about 0.05 m. At the IGVC minimum passage the worst-case margin is 0.15 m per side. | Medium–High | Confirmed (config); margin is computed | `nav2_params_igvc_autonav.yaml:276-278`, `:313` | Known as a concern; now quantified |
| F4 | Inflation geometry is sound. Inscribed radius is 0.2794 m (the rear edge), circumscribed 0.9126 m. Local inflation 1.0 > circumscribed, so CostCritic's footprint gating works. Cost at the center of a 1.524 m passage is 75; 0.2 m off-center it is 124, so the V-basin is descendable. **Inflation 1.0 does not block gaps ≥ 0.6 m for the center point.** The binding limit is the footprint plus quantization (F3). | Info | Confirmed (computed) | `:287`, `:502-503`; §2 | Refines the RCA |
| F5 | Heading-hold engages whenever \|ω_slewed\| < 0.05 rad/s and v > 0.02 m/s, and replaces MPPI ω with 1.5·(yaw_lock − yaw). Gap-centering corrections of 0.1–0.2 m at 0.3 m/s need ω of about 0.02–0.05 rad/s, which is exactly the band being overridden. This gives a deadband limit cycle. Lock also re-engages at every ω zero-crossing in a slalom. | Medium | Probable (source); impact is a hypothesis | `actuator_node.py:447-455`; `actuator_params.yaml` `heading_hold_deadband: 0.05` | Known (D2); made specific to gaps |
| F6 | MPPI on Humble has no acceleration model (`ax_max`/`az_max` are dead). The actuator slews v at 0.3 m/s² and ω at 1.2 rad/s² (accel = decel). With `wz_std` 0.5 at 20 Hz, sampled ω changes about 10× faster than the 0.06 rad/s per 50 ms that is delivered. A ±0.5 rad/s reversal takes 0.83 s, which is up to about 0.4 rad of heading error at the transition. This drives slalom overshoot and oscillation. | Medium–High | Confirmed (params dead); impact is probable | `:146-149`; control_stack 02 D6; `actuator_node.py:424-432` | Known (D6); slalom impact is new |
| F7 | The global path (map frame, Navfn, 0.2 m, replanned at 3 Hz) moves with map→odom. Sources: the 0.76 m GNSS lever arm changes with heading (a ±15° slalom swing is about 0.39 m), and the map pose wandered 37 × 34 cm while stationary. PathAlign (12) outweighs CostCritic (5): about 2.4 against 0.7 for a 0.2 m path offset versus a centered gap. A shifted path can therefore pull the robot toward a barrel. | Medium | Probable (magnitudes computed; not seen in a bag) | `:195`, `:186`; control_stack 09 #2, #6 | New combination |
| F8 | Inside the Velodyne `min_range` 0.7 m nothing is re-observed. The local STVL has `voxel_decay` 3 s linear, `decay_acceleration` 0.5, and no frustum `min_z` set, so the blind zone is inside the frustum. A barrel alongside the robot (≤ 0.7 m from the sensor when the robot is off-center in a 1.5 m passage) may fade in ≤ 3 s. At slow threading speed (1.09 m length at 0.3 m/s ≈ 3.6 s) the rear corner can then swing into it (see F1). | Medium | Hypothesis (test T6) | `velodyne.yaml:19` (`min_range: 0.7`); `:307,362`; [STVL README](https://github.com/SteveMacenski/spatio_temporal_voxel_layer) | New (RCA #3 covered only the dead-ahead case) |
| F9 | BT recovery [0] ClearCostmapAroundRobot (`reset_distance` 1.5, about a ±0.75 m square around `base_link`) erases the barrels beside the robot. Those that are inside the 0.7 m blind radius cannot be re-marked, and lane cells beside the robot are outside the front ZED's view. In a barrel field this recovery can open a false gap. The BT comment also still cites semantic `tile_map_decay_time` 1.5 s, which is now 1e6. | Medium | Probable | `navigate_igvc_autonav_2026.xml` RoundRobin [0]; `:392` | New |
| F10 | `CostCritic.near_goal_distance` is 1.0 (default 0.5). Within 1 m of the path end, repulsion is switched off (collision check only), so the robot hugs a barrel next to a waypoint. | Low | Confirmed (source) | `:190`; `cost_critic.cpp` `near_goal` | New |
| F11 | Dead or misleading parameters: `ax_max`, `ax_min`, `az_max` (D6); `CostCritic.trajectory_point_step` (not read by the Humble `cost_critic.cpp`; verify on 1.1.20); PreferForwardCritic has no effect with `vx_min` 0.0; GoalAngleCritic (weight 3, active within 0.5 m) still steers toward a goal yaw that `yaw_goal_tolerance` 3.15 ignores. | Low | Confirmed / Probable | `:146-149,191,178-182,173-177,92` | Partly known |
| F12 | Low-speed delivery is unmeasured. The feedforward is probably about 9% of intended (M1), so 0.05–0.15 m/s commands rely on P+I against track stiction. MPPI creep commands may stall and then lurch. | Medium | Hypothesis (T7) | control_stack 01 M1 | Known cause; fine-motion impact is new |
| F13 | 185 ms odometry lag. Along-track pose lag is v·0.185: 6 cm at 0.3 m/s, 13 cm at 0.7 m/s. MPPI's initial velocity is also stale. This is bounded, because costmap and robot pose share the lag, and it matters mostly when accelerating. | Low–Medium | Confirmed (lag); impact is a hypothesis | control_stack 03 | Known |
| F14 | Controller timing is consistent: 20 Hz, `model_dt` 0.05, horizon 56 × 0.05 = 2.8 s, reach 1.96 m < `prune_distance` 5 m, `batch_size` 500. Goal checker xy 0.5 m, progress checker 0.3 m / 12 s, BT Timeout 45 s, 4 retries: these are appropriate for waypoint-to-waypoint through barrels. | Info | Confirmed | `:75,102,110,114,86-93`; BT | — |

## 2. Gap geometry

Assumptions:
- Robot width 0.83 m, half-width 0.415. Footprint relative to `base_link`: x ∈ [−0.2794, +0.8128].
- Inscribed radius 0.2794, circumscribed 0.9126.
- Inflation cost = 252·e^(−2.5(d − 0.2794)) for 0.2794 < d ≤ 1.0 (local).
- IGVC rule: minimum passage 5 ft = **1.524 m** between line and obstacle or obstacle and obstacle (`docs/cv_canny_pipeline_2026_06_01/igvc_2026_rules_fulltext.txt:295-298`). Lanes are 10–20 ft. **The 2026 course is asphalt** (line 278), so grass tests today are pessimistic for traction and slip.

| Gap G (surface to surface) | Slack per side (G − 0.83)/2 | Center-point cost | Cost 0.2 m off-center | Worst-case margin per side at res 0.2 / 0.1 / 0.05 |
|---|---|---|---|---|
| 1.0 m | 0.085 | 145 | 239 | −0.115 / −0.015 / +0.035 (refuse, correctly) |
| 1.2 m | 0.185 | 113 | 186 | −0.015 / +0.085 / +0.135 |
| 1.3 m | 0.235 | 100 | 165 | +0.035 / +0.135 / +0.185 |
| **1.524 m (IGVC min)** | **0.347** | **75** | **124** | **+0.147 / +0.247 / +0.297** |
| 2.0 m | 0.585 | 42 | 69 | +0.385 / +0.485 / +0.535 |
| 2.5 m | 0.835 | 0 | 0 | ample |

"Worst-case margin" subtracts one cell per side. The footprint outline cell and the obstacle cell can coincide while the true distance is up to one cell. This is a conservative bound; the typical loss is about half a cell.

- At 0.2 m, the margin left at the IGVC minimum (about 0.15 m per side) has to absorb all of the following:
  - LiDAR point spread (±3 cm)
  - STVL voxel projection (≤ 5 cm)
  - odometry lag (6 cm at 0.3 m/s)
  - heading-hold wobble (F5)
  - the ICR error in any turn inside the gap (F1: 0.16 m for a 30° correction)
- **The corridor is feasible only if the robot enters the gap aligned and does not rotate inside it.**
- At 0.1 m the margin is about 0.25 m.
- Navfn treats cost 75–125 as about 110–150 per cell against 50 for free space (Navfn `COST_FACTOR` 0.8 + neutral 50). It may prefer a detour up to about 2–3× longer around a cluster if one exists inside the lanes. This is expected, not a bug.
- Inflation radius: 1.0 local is right. It is above the circumscribed radius, which CostCritic footprint gating needs, and it does not block gaps. Keep `cost_scaling_factor` 2.5. Do not add `footprint_padding` (default 0.01); at 0.2 m cells any padding just rounds into a lost cell.

## 3. Answers to the scope questions

1. **Thread settings:**
   - Costmap resolution 0.1 m.
   - Inflation 1.0 / `cost_scaling_factor` 2.5 local, 0.85 global (both unchanged).
   - Footprint padding 0.
   - CostCritic 5 → 6–7 and PathAlign 12 → 8, for tight sections, as an A/B.
   - `near_goal_distance` 0.5.
   - Keep PathFollow 8 and Goal 5.
   - No TwirlingCritic: it is for holonomic robots, and in Humble it only penalizes wz.
   - PreferForward is inert while `vx_min` = 0.
2. **Resolution:** 0.2 m costs up to 0.2 m per side (F3). Use 0.1 m with voxel 0.1 m (matched) and a **20 × 20 m** window. The horizon reach is about 2 m and `obstacle_range` 8 m, so ±10 m is sufficient. That is 40,000 cells, **36% fewer than today's 62,500**. Inflation work per obstacle cell grows about 4× (kernel radius 10 cells instead of 5), and the footprint outline check grows from about 19 to about 38 cells per pose. That is still well under 1 ms per MPPI cycle at 500 × 28 points (estimate, unmeasured). 0.05 m in a 16 × 16 m window (102,400 cells, 1.6× today) is possible if T-CPU passes, but keep the voxel at 0.1 m.
3. **MPPI low-speed settings for tight fields (Humble has no acceleration model):**
   - Reduce sampling spread toward what the slew can deliver: `vx_std` 0.3 → 0.2, `wz_std` 0.5 → 0.35–0.4 (default 0.4).
   - Keep `temperature` 0.25–0.3, `model_dt` 0.05 = 1/20 Hz, `time_steps` 56.
   - Cap speed in obstacle sections: `vx_max` 0.5, or `/speed_limit` at 70%, which Humble MPPI honors. Slower means the slew lag is a smaller share of the horizon.
   - The structural fix is either MPPI with accel constraints (PR #4352) or raising the actuator ω slew to about 2.0–2.5 rad/s² once the 12 V rail is fixed.
4. **Actuator:**
   - Disable heading-hold while `/cmd_vel` (Nav2) is the source. Today: `heading_hold_deadband 0.0` A/B.
   - Add a separate angular decel (D4).
   - Measure low-speed delivery (T7); the M1 kV fix is the real cure.
   - The Teensy ramp (100 RPM per 20 ms ≈ 1.66 m/s² per track) does not bind; the host slew does.
   - The 185 ms lag is bounded (F13).
5. **BT and checkers:**
   - Goal xy 0.5 m, mission pop at 2.0 m, progress 0.3 m / 12 s, Timeout 45 s: appropriate.
   - `bt_navigator.goal_reached_tol` 3.0 is unused by this BT.
   - Weak points: recovery [0] clears barrels in the blind zone (F9), and recovery [2] DriveOnHeading 0.3 m straight is fine. Consider `reset_distance` 1.5 → about 1.0 in fields. Semantics: the square side is `reset_distance`; verify on Humble.
6. **Other:** F1 (ICR), F2 (cost sampled at the rear third), F8 (blind-zone decay), F10, F11.

## 4. Field tests — outdoors today

**Common setup**
- Launch: `ros2 launch avros_bringup navigation.launch.py enable_mission_manager:=false` (keep NTRIP on; RTK FIXED is required for the map-frame goals).
- No RViz on the Jetson. Use laptop Foxglove on `ws://<jetson>:8765`.
- In the CLI shell: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and set `CYCLONEDDS_URI`.
- Pre-checks:
  - `ros2 topic hz /cmd_vel` ≥ 18 Hz during a goal.
  - `/gnss` status FIXED.
  - `ros2 param get /actuator_node heading_hold_deadband`.

**Short map-frame goal**
1. `ros2 run tf2_ros tf2_echo map base_link` → note x, y, yaw.
2. Goal = (x + d·cos yaw, y + d·sin yaw).
3. Send it: `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: X, y: Y}, orientation: {w: 1.0}}}}"`, or Foxglove "Publish pose" on `/goal_pose` in the `map` frame.
4. Never send goals in the `odom` frame (see the extrapolation issue in CLAUDE.md).

**Foxglove panels**
- 3D: `/local_costmap/costmap`, `/local_costmap/published_footprint`, `/plan`, `/velodyne_points`.
- Plot: `/cmd_vel` linear.x and angular.z against `/odometry/filtered` twist.
- Plot: `/avros/wheel_debug` `heading_locked`.
- Log: `/behavior_tree_log`.

**Record for every run**
```
ros2 bag record -o fine_<test>_<run> /cmd_vel /odometry/filtered /wheel_odom /imu/data /gnss \
  /local_costmap/costmap /local_costmap/published_footprint /plan /tf /tf_static \
  /avros/wheel_debug /avros/actuator_state /behavior_tree_log /navigate_to_pose/_action/status /rosout
```
Add `/velodyne_points` for T3/T5/T6 only (about 4 MB/s; keep runs < 60 s). MPPI trajectories need `FollowPath.visualize: true`, which requires a restart and costs CPU. Skip it today unless the restart batch in §5 is applied.

**Barrels:** standard drums, about 0.58 m in diameter. Measure every gap surface to surface at barrel mid-height.

| Test | Purpose | Setup and procedure | Pass / fail |
|---|---|---|---|
| **T1 ICR offset** (F1; no Nav2 motion needed) | Find the real spin center relative to `base_link` | Chalk a plumb mark on the ground under the IMU. Teleop (`teleop.launch.py` or webui) spin in place at about 0.5 rad/s through 90°, stop. Measure the chalk-to-new-IMU-plumb displacement. Repeat 180° and 360°, both directions. Also log `/gnss` (antenna 0.76 m ahead): its circle radius is about 0.45 m if the ICR is at the track center, 0.76 m if at `base_link`. | IMU displacement after 90° < 0.10 m → F1 refuted. About 0.44 m → F1 confirmed; plan the `base_link` move. Record the EKF-reported displacement too (expected ≈ 0). |
| **T2 heading-hold A/B** (F5) | Does the hold fight gap centering? | Two barrels, 1.524 m gap. Start 5 m before, **0.3 m lateral offset**, heading parallel. Goal 5 m past the gap. Run 3× with deadband 0.05, then `ros2 param set /actuator_node heading_hold_deadband 0.0` and 3×. Restore to 0.05 after. | Fraction of the approach with `heading_locked` = 1 (expect > 50% with 0.05). Lateral error at the gap entry ≤ 0.10 m. No ω sign chattering (≤ 2 reversals on the approach). Deadband 0 should be equal or better. |
| **T3 gap ladder** (F3/F4) | Find the real minimum passable gap | Two barrels, centered approach, 5 m before, goal 5 m past. Gaps 2.0 → 1.524 → 1.3 → 1.2 → 1.0 m, 3 runs each. `vx_max` as deployed. | 2.0 and 1.524: 3/3 pass, no contact, measured clearance ≥ 0.10 m each side (video or tape), no stop > 1 s, no recovery, time ≤ 10 m / 0.3 m/s + 5 s ≈ 38 s. 1.3: note pass rate (prediction: marginal at 0.2 m). 1.2: prediction is refusal or detour. **1.0: must refuse or detour; any attempt that touches is a fail.** |
| **T4 barrel on the path** (RCA regression) | Go-around without freezing | One barrel dead-center 6 m ahead on a straight path; goal 10 m. Mark ±1.524 m "lane" lines with tape and cones (cones visible to LiDAR). | Completes 3/3, clearance ≥ 0.3 m, no stop > 2 s, stays within the cones, no recovery triggered. |
| **T5 slalom** (F6/F7/F1) | Fine control through repeated gaps | 4 barrels alternating L/R, 2.5 m longitudinal spacing, 1.524 m passage between each barrel and a cone line. Goal 3 m past the last barrel. 3 runs baseline; then 3 runs with the runtime A/B set (vx_max 0.5, PathAlign 8, CostCritic 7, wz_std 0.4). | No contact. No recovery. Completion ≤ 45 s. ω sign reversals on `/cmd_vel` ≤ 2 per barrel (more means oscillation). Delivered ω (`/odometry/filtered`) tracks `/cmd_vel` within 0.3 s at the peaks. Rear-corner clearance ≥ 0.10 m on video (checks F1). |
| **T6 blind-zone persistence** (F8/F9) | Does a barrel beside the robot fade? | Nav2 idle (no goal). Teleop to park alongside a barrel with about 0.2–0.3 m side clearance (barrel surface ≤ 0.7 m from the Velodyne). Watch `/local_costmap/costmap` for 15 s. Then call `ros2 service call /local_costmap/clear_around_local_costmap nav2_msgs/srv/ClearCostmapAroundRobot "{reset_distance: 1.5}"` and watch 10 s. | Barrel cells stay LETHAL the whole 15 s → F8 refuted. They fade ≤ 5 s → F8 confirmed. After the clear, cells are not re-marked within 2 s → F9 confirmed. |
| **T7 low-speed delivery** (F12) | Can it creep precisely? | Nav2 idle. `ros2 topic pub -r 20 /cmd_vel geometry_msgs/Twist "{linear: {x: V}}"` for V = 0.05, 0.10, 0.15, 0.20 m/s, 5 s each, Ctrl-C between. Then ω = 0.1 and 0.2 rad/s with v = 0. Use a clear straight area. | Motion starts within 0.5 s. Steady-state wheel speed within ±10% of command after 1.5 s for v ≥ 0.10. Rotation starts at ω = 0.1. Record the stall threshold. |
| **T-CPU** (with any change) | Budget | During T5 | `/cmd_vel` ≥ 18 Hz, no "Control loop missed its desired rate" warnings, Jetson load < 6. |

## 5. Proposed parameter changes (not applied)

**Runtime A/B today** (`ros2 param set`). MPPI and inflation parameters are dynamic in Humble per upstream parameter handlers, but that is unverified on 1.1.20. Confirm "Set parameter successful" and look for the effect. Revert after each test.

| Node / param | Current | Proposed | Why |
|---|---|---|---|
| `/actuator_node heading_hold_deadband` | 0.05 | 0.0 while Nav2 drives (restore 0.05 for webui) | F5 — stop overriding MPPI micro-corrections |
| `/controller_server FollowPath.vx_max` (or `/speed_limit` `{percentage: true, speed_limit: 70.0}`) | 0.7 | 0.5 in obstacle sections | Less slew lag and odometry lag per metre; more cycles per gap |
| `FollowPath.PathAlignCritic.cost_weight` | 12 | 8 | F7 — let the cost basin, not a shifted global path, center the robot |
| `FollowPath.CostCritic.cost_weight` | 5 | 7 (A/B vs 5) | Stronger centering in the 75–125 cost band. Watch for freeze recurrence; the footprint is now right-sized, so the freeze mechanism should not return. |
| `FollowPath.CostCritic.near_goal_distance` | 1.0 | 0.5 | F10 |
| `FollowPath.wz_std` / `vx_std` | 0.5 / 0.3 | 0.4 / 0.2 | F6 — sample closer to what the 1.2 rad/s² / 0.3 m/s² slew can deliver; less chatter |
| `/actuator_node max_angular_accel_rps2` | 1.2 | 2.0 (only if the 12 V rail is fixed; watch for Jetson brown-out) | F6 — a slalom reversal drops from 0.83 s to 0.5 s |

**Needs restart** (edit YAML; the team decides):

| Param | Current | Proposed | Why |
|---|---|---|---|
| `local_costmap.resolution` / `width` / `height` | 0.2 / 50 / 50 | 0.1 / 20 / 20 | F3 — half the quantization loss with 36% fewer cells. Check T-CPU. |
| local `stvl_layer.voxel_decay` | 3.0 | 5.0 (matches global) | F8 — barrels beside the robot must outlive a slow pass |
| local `velodyne_points.min_z` (STVL frustum near plane) | unset (0) | 0.75 | F8 — do not frustum-clear the LiDAR blind zone. Verify the parameter name in the installed STVL. |
| BT `ClearCostmapAroundRobot reset_distance` | 1.5 | 1.0 | F9 |
| `FollowPath.visualize` | false | true for test sessions only | Records MPPI optimal trajectory and samples for tuning (CPU cost) |
| Remove `ax_max`, `ax_min`, `az_max`, `CostCritic.trajectory_point_step` | set | delete, or comment as inert | F11 — config should not claim limits that are not enforced |
| URDF `base_link` → track center (+0.3143 m); footprint → [[0.4985, ±0.415], [−0.5937, ±0.415]]; IMU/LiDAR/ZED joints shifted −0.3143; Xsens lever arm and EKF re-checked | IMU origin | Only if T1 confirms | F1 — REP-105 convention; fixes the MPPI rotation model, the footprint sweep and odometry together. Structural; not for a competition-eve change. |

## 6. Sources

- Nav2 MPPI README (Humble): https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_mppi_controller/README.md
- Humble CostCritic source: https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_mppi_controller/src/critics/cost_critic.cpp
- Nav2 tuning guide (inflation potential field, footprint vs radius): https://docs.nav2.org/rolling/configuration_and_development/tuning_guide/
- CostCritic API (Humble 1.1.18): https://docs.ros.org/en/humble/p/nav2_mppi_controller/generated/classmppi_1_1critics_1_1CostCritic.html
- MPPI accel constraints PR #4352 (post-Humble): https://github.com/ros-navigation/navigation2/pull/4352
- STVL README: https://github.com/SteveMacenski/spatio_temporal_voxel_layer
- Dynamic footprint in MPPI proposal: https://github.com/ros-navigation/navigation2/issues/6273
- IGVC 2026 rules (local copy): `/home/mspacman/IGVC_ROS2/docs/cv_canny_pipeline_2026_06_01/igvc_2026_rules_fulltext.txt:278-298`
- Repo: `docs/obstacle_stall_rca_2026_05_30.md`; `research/control_stack_analysis_2026_09/02_drive_kinematics_layer.md` (D2, D6), `09_findings_and_recommendations.md`
