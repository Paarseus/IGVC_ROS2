# Next-Session Test Plan — 2026-05-19

**References:** [`W_vehicle_integration_test_plan.md`](W_vehicle_integration_test_plan.md) (canonical 14-phase ladder).
**Builds on:** [`skid_steer_kinematics_findings_2026_05_18.md`](skid_steer_kinematics_findings_2026_05_18.md) (2026-05-18 PID retune + Mandow correction + EKF validation).

This is a **scoped subset** of the W plan — not a replacement. It picks the specific W phases worth doing first given the state of the system after 2026-05-18, and notes what's deferred. Each step here links to the corresponding W section for the full Action/Go/No-go detail; this file only adds the *ordering*, the *recent-change context*, and the *commands to run*.

---

## State as of end-of-day 2026-05-18

| W Phase | Status | Notes |
|---|---|---|
| P0 Pre-flight, P1 DDS | ✅ implicit | actuator + sensors + localization launched cleanly all session |
| P3 Sensor data (Xsens) | ✅ validated | post-USB-recovery, stationary drift 0.085°/s |
| P3 Sensor data (Velodyne, ZED) | ⚠️ not stressed this session | sensors up but no field test |
| P7 Localization (EKF) | ✅ validated | 3-way comparison + closed-loop 1×1 m square, Δxy=6 cm, Δyaw=3.6° |
| P11 Drive train | ✅ validated | new burned PID + Mandow multiplier 1.19 |
| P5 STVL Local Costmap | ⏭️ **untested — the headline gating test** |
| P9 Planner (Navfn) | ⏭️ untested |
| P10 Controller MPPI | ⏭️ **untested — where 2026-05-18 work pays off most** |
| P4/P6/P8/P12/P13 | ⏭️ deferred (see below) |

## Deferred to later sessions (not next)

- **P4 / P6** (perception pipeline + semantic_layer): only matters when you're on grass for lane-following. Not relevant for next session's open-area validation.
- **P8** (costmap integration): covered indirectly by P5 + P10 below. Full integration runs after P10 passes.
- **P12** (mission_manager): waypoint orchestration. Run after MPPI single-goal works.
- **P13** (full IGVC scenarios): final integration, comes after P12.

---

## Recommended sequence — 5 steps, ~50 min total

### Step 1 — Pre-flight (10 min, BENCH)

Maps to W P0 + P1 (Sensor + DDS pre-flight). Quick re-verify of stable launches after a reboot.

**Action:**
```bash
ssh jetson
source /opt/ros/humble/setup.bash && source ~/IGVC/install/setup.bash
# Verify env vars (CLI uses CycloneDDS):
printenv RMW_IMPLEMENTATION CYCLONEDDS_URI
# Should print: rmw_cyclonedds_cpp + file:///home/dinosaur/IGVC/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml
```

If env not set, [`~/.bashrc` is missing them](W_vehicle_integration_test_plan.md#phase-1--dds-hygiene-7-steps-10-min-bench) — fix before continuing or every CLI command will use FastDDS and corrupt action goals.

**Go:** prints rmw_cyclonedds_cpp + the file:// URI. **No-go:** stop, fix `~/.bashrc`.

---

### Step 2 — Full nav stack launches cleanly (10 min, BENCH)

Maps to **W P5.1, P5.2, P9.1**. Bring up the full stack and verify lifecycle nodes activate.

**Action:**
```bash
# On Jetson — full stack with perception + ZED (perception+ZED can be omitted
# if you only want to test MPPI on open ground without lane following)
ros2 launch avros_bringup navigation.launch.py \
  enable_zed_front:=false \
  enable_perception:=false \
  enable_mission_manager:=false \
  enable_ntrip:=false
```

In a second terminal:
```bash
ros2 topic hz /local_costmap/costmap          # expect ~10 Hz
ros2 topic hz /global_costmap/costmap         # expect ~0.5 Hz
ros2 topic hz /odometry/filtered              # expect ~20 Hz
ros2 lifecycle get /local_costmap/local_costmap   # expect: active
ros2 lifecycle get /controller_server         # expect: active
ros2 lifecycle get /planner_server            # expect: active
ros2 param get /planner_server GridBased.plugin  # expect: nav2_navfn_planner/NavfnPlanner
```

**Go:** all 4 topics publishing at expected rates, 3 nodes active, planner is Navfn.
**No-go:** any node fails to activate → check launch log for plugin load errors (STVL, kiwicampus). The most common after a system update is `kiwicampus/semantic_segmentation_layer` failing to build on Humble — see [CLAUDE.md Known Issues](../CLAUDE.md) for the patch.

---

### Step 3 — STVL costmap (10 min, BENCH or FIELD)

Maps to **[W P5](W_vehicle_integration_test_plan.md#phase-5--stvl-local-costmap-10-steps-25-min-field)** — this is THE headline test of the 2026-05-12 STVL migration. Cannot skip.

**Action:**
- Robot stationary, in open area.
- Place a 0.55 m traffic barrel 3 m forward of `base_link`.
- In RViz: add `/local_costmap/costmap` as Map display.

**Go (W P5.3):** LETHAL cell (occupancy 254) appears at approximately (3, 0) within **0.4 s** of placing the barrel.
**Go (W P5.5):** snatch the barrel laterally out of the LiDAR's view. LETHAL cell decays to FREE within **1.5 s ±0.5 s** (frustum-accelerated clear).

**No-go (mark):** >0.5 s to mark → barrel height <0.2 m above ground, or velodyne points filtered out.
**No-go (clear):** >5 s to clear → `decay_acceleration: 0` or `model_type: 0` (depth-camera mode) — check `nav2_params_humble.yaml`.

This validates the STVL migration + clearing behavior + range/height filters in one test. **Don't skip.**

---

### Step 4 — Planner sanity (5 min, BENCH)

Maps to **[W P9.3](W_vehicle_integration_test_plan.md#phase-9--planner-10-steps-20-min-field)**. Verify Navfn can produce a path.

**Action:**
```bash
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 0.0}}}}"
```

**Go:** path returned within 2 s, ≥2 poses, frame_id=map.
**No-go:** ABORTED → no map→odom TF (check `ekf_filter_node_map`), or costmap not active.

If a barrel is in front from Step 3, also try with the barrel in place — path should detour with ≥0.4 m clearance.

---

### Step 5 — MPPI smoke test (10-15 min, FIELD)  ⭐ **the payoff**

Maps to **[W P10.3 + P10.4](W_vehicle_integration_test_plan.md#phase-10--controller-mppi-12-steps-25-min-field)**.

**This is where 2026-05-18's work earns its keep.** MPPI predicts each sampled trajectory assuming the chassis tracks commanded velocities. With Mandow correction live (multiplier=1.19), ω tracking is ~100% — so MPPI's rollouts should match reality, and lateral path error should be <10 cm RMS.

**Pre-conditions:**
- Open area, ≥8 m clear forward.
- Hand on the wireless e-stop.
- Visual line of sight.
- Verify gains live:
  ```bash
  ros2 param get /actuator_node wheel_separation_multiplier   # expect: 1.19
  ros2 param get /actuator_node kP                            # expect: 0.0007
  ```

**Action (P10.3 — static-pose null):**
```bash
# Send a goal AT the robot's current location — robot should not drive
# Click "2D Nav Goal" in RViz at the robot's current position, or use:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0}}}}"
```
**Go:** `/cmd_vel` stays near zero. **No-go:** robot creeps or oscillates → check goal_tolerance / xy_goal_tolerance config.

**Action (P10.4 — 3 m path tracking):**
```bash
# Open area, send goal 3 m forward
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 0.0}}}}"
```

**Go:**
- Robot reaches goal within xy_goal_tolerance (2.0 m).
- **Lateral path error < 10 cm RMS** along the path (this is the 2026-05-18 work paying off).
- No swerve, oscillation, or overshoot.
- `/cmd_vel.linear.x` ≤ MPPI's vx_max (currently 0.7 m/s).

**No-go (any of):** swerve >0.5 m, sustained vx_max clamp at 0.5 m/s for >1 s ([Nav2 #4970](https://github.com/ros-navigation/navigation2/issues/4970)), or robot stops with FAILED_TO_MAKE_PROGRESS.

**Bag protocol if it fails:**
```bash
ros2 bag record -s mcap -o mppi_smoke_fail \
  /cmd_vel /odometry/filtered /odometry/global \
  /local_costmap/costmap /local_costmap/voxel_grid \
  /plan /tf /tf_static /imu/data /avros/wheel_debug
```

---

## Optional — Step 6 — MPPI detour test (10 min, FIELD)

Maps to **W P10.7**. Same goal as Step 5, but place a barrel 0.5 m off-path. MPPI's `CostCritic` (weight 3.81) should steer around it.

**Go:** chassis maintains ≥0.5 m clearance from barrel, still reaches goal.
**No-go (too close to barrel):** bump `CostCritic.cost_weight` 3.81 → 5.0 in `nav2_params_humble.yaml`.

---

## Things to watch out for

1. **`/dev/ttyUSB0` vs `/dev/ttyUSB1`** for Xsens — if the chassis was power-cycled, the Xsens may have moved to ttyUSB1. `xsens.yaml` has `scan_for_devices: true` so the driver finds it automatically, but `sensors.launch.py` must be relaunched after USB events.

2. **Stuck IMU bias symptom:** car doesn't go straight + stationary `/imu/data.orientation` yaw drifts visibly → USB power-cycle the Xsens. ROS-level restart does NOT fix. See [skid_steer_kinematics_findings_2026_05_18.md §6](skid_steer_kinematics_findings_2026_05_18.md).

3. **Speed cap before P10 if it feels aggressive:**
   ```bash
   ros2 param set /actuator_node max_linear_mps 0.7   # indoor / safety
   ros2 param set /actuator_node max_angular_rps 0.6
   ```
   Both DYNAMIC, take effect immediately.

4. **Mandow multiplier might need recalibration on a different surface.** The 1.19 value was measured on indoor concrete. Grass may need 1.4-1.6 range. Re-calibration procedure: [skid_steer_kinematics_findings_2026_05_18.md §7](skid_steer_kinematics_findings_2026_05_18.md).

## Skip these for now

- Driving through dense lanes (P13 family) — needs perception (P4) validated first.
- Mission manager waypoint sequencing (P12) — verify single-goal MPPI works before adding the wrapper.
- Long-haul thermal test (W P13.8) — leave until competition prep.

## Success criteria for "this session worked"

After Step 5 passes:
- ✅ STVL costmap correctly marks + clears obstacles (P5)
- ✅ Navfn produces valid paths (P9)
- ✅ MPPI tracks paths within 10 cm RMS lateral error on 3 m forward goal (P10.4)
- ✅ End-to-end stack functional with 2026-05-18 tuning

That's enough to declare "the chassis is navigatable" and move to scenario tests (P10.7, P12, P13) in a follow-up session.
