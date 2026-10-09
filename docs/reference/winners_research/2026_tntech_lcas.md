# TnTech LCAS Lab IGVC2026 (active, closest HW match)

## Repo & build
- Source: https://github.com/LCAS-Lab/TnTechIGVC2026 (single workspace `isaac_ros-dev/`, custom pkg `vehicle_control_unit/` + drop-in `lane_costmap_layer/`).
- Stack: Isaac ROS 3.2 + ROS 2 Humble on Jetson AGX Orin. ZED 2i (high mount) + RealSense D435 (low mount). VESC motor controllers over CAN (socketcan). MAVROS for RC + GPS via ArduPilot/Pixhawk.

## Architecture (one ASCII diagram)
```
ZED 2i ─┐                              ┌─ Nav2 (MPPI) ── /cmd_vel ─ cmd_vel_node ─┐
        ├─ NVBlox (TSDF, voxel 0.10 m)─┤                                          ├─ vesc_can_node (L,R via socketcan)
RS D435 ┘   /nvblox_node/static_map_slice                                         │
                       │                                                          │
                       ▼ (NvbloxCostmapLayer, 2D slice)                           │
            local + global costmaps                                               │
GPS (MAVROS) ─ mission_global_frame ─ waypoint_mgr ─ carrot_target ─ carrot_to_nav2_action ─ NavigateToPose (map frame)
VESC eRPM ─ wheel_odom ─┐
ZED IMU (yaw rate) ─────┴─ ekf_local (odom) ── /odometry/local ── consumed by Nav2 & status monitor
                           (NO global EKF — map ≡ odom identity, map_odom_broadcaster)
```

## Localization (GPS fusion strategy)
**They do NOT fuse GPS into a `map` EKF.** `ekf_local` (`config/ekf_local.yaml`) fuses only `/wheel/odom` (X/Y/yaw/Vx/Vyaw) + ZED IMU (Vyaw only — yaw absolute is intentionally skipped because ZED VIO jumps). A separate `map_odom_broadcaster` node just publishes `map → odom` as identity (`launch/navigation.launch.py:329-339`). GPS enters via a **mission pipeline** — `mission_global_frame_node` → waypoint manager → carrot target → `carrot_to_nav2_action_node`, which emits NavigateToPose goals in the `map` frame. A `config/ekf_global.yaml` exists fusing `/odometry/local` + `/odometry/gps` but is **not wired into the active launch**. Compass-from-mag (`compass_heading_publisher`) is plumbed but commented out in `ekf_local.yaml`. Net: GPS drives waypoint targeting, not TF.

## Costmap / world representation
**NVBlox is used as BOTH local and global costmap layer** via `nvblox::nav2::NvbloxCostmapLayer`, fed from the same `/nvblox_node/static_map_slice` topic (`config/nav2_params.yaml:199-206, 235-242`). Voxel size **0.10 m** (`config/nvblox/nvblox_base.yaml:6`), 2D ESDF slice at min_height 0.3 / max_height 2.0, `static_tsdf` mode. Local costmap: **6×6 m, 0.05 m resolution**, plugins `[nvblox_layer, lane_layer, inflation_layer]`. Global: **40×40 m, 0.05 m resolution**, plugins `[nvblox_layer, inflation_layer]`. Their custom `lane_costmap_layer` (separate pkg) overlays a `/lane/occupancy_grid` topic in the local costmap only — same pattern as our kiwicampus semantic layer.

Compared to us: we have **STVL + Velodyne VLP-16** local 50×50 m @ 0.2 m and global 40×40 m @ 0.2 m. They have **4× finer resolution** (0.05 vs 0.2 m) but a much smaller local window (6×6 m vs 50×50). Their inflation: local 0.35 m / global 0.50 m (we run 1.0 m local / 0.65 m global — ours larger because our robot_radius is 0.8 m vs their ~0.55 m).

**Why we shouldn't switch to NVBlox from STVL**: NVBlox needs depth from stereo cameras as its primary input — we have a Velodyne VLP-16 that gives 360° geometric range that NVBlox would mostly ignore (their `use_lidar: false`). Our STVL already does time-decay clearing the way GPS-drift-prone global maps need. NVBlox shines for indoor/dense-depth dynamic scenes; for outdoor 360° LiDAR + GPS, STVL is the more appropriate choice. We'd consider NVBlox only if we added a high-mount ZED for long-range stereo and wanted GPU-accelerated voxel TSDF.

## Planner
`NavfnPlanner` (default Nav2), no SmacPlanner. Same as our current choice — convergent peer signal.

## Controller (MPPI config DELTA from ours)
File refs: theirs `isaac_ros-dev/src/vehicle_control_unit/config/nav2_params.yaml:88-165`, ours `src/avros_bringup/config/nav2_params_humble.yaml:79-183`.

| Param | TnTech | Ours | Impact |
|---|---|---|---|
| `controller_frequency` | 10.0 Hz | 20.0 Hz | We replan 2× more often; cheaper batch helps us hit 20 Hz |
| `time_steps` | 40 | 56 | Their horizon = 4.0 s; ours = 2.8 s @ model_dt 0.05 |
| `model_dt` | 0.10 | 0.05 | They take coarser 100 ms steps; we sample finer |
| `batch_size` | 2000 | 1000 | They run AGX Orin (more GPU); we halved for Jetson Orin (likely Nano/NX) to hold 20 Hz |
| `vx_std` / `wz_std` | 0.5 / 0.4 | 0.2 / 0.4 | They explore wider linear velocity space |
| `vx_max` | 1.0 m/s | 0.7 m/s | They run faster; we're motor-traction-capped on grass |
| `wz_max` | 1.5 rad/s | 1.9 rad/s | We sample wider angular (actuator clamps downstream) |
| `iteration_count` | 3 | 1 | They take 3 Newton-style optimizer passes per cycle (smoother convergence; expensive) |
| `prune_distance` | 3.0 | 1.7 | They consider path further ahead |
| `regenerate_noises` | true | false | They resample noise each tick (more exploration, more CPU) |
| `transform_tolerance` | 0.3 | 0.1 | They tolerate stale TF |
| `ObstaclesCritic` | enabled (weight 2.0/10.0, inflation 0.45) | NOT USED — we use `CostCritic` (weight 3.81, critical 300) | Different critic, both valid |
| `PathAlignCritic` weight | 4.0 | 14.0 | We weight path adherence much more heavily |
| `PathFollowCritic` weight | 8.0 | 5.0 | They lean on path-follow more |
| accel limits (`ax_max`/`az_max`) | not declared | 0.4 / 1.5 | We match chassis slew caps (their behavior_server limits acceleration_limit 0.5) |

## State machine / BT
Custom BT at `config/igvc_bt.xml`: RecoveryNode(3 retries) wrapping PipelineSequence(ComputePathToPose, FollowPath) with a `ReactiveFallback` recovery — ClearLocalCostmap + ClearGlobalCostmap + BackUp(0.5 m @ 0.05 m/s) + **RoundRobin of three Spin angles (15° L, 15° R, 30° L)**. No `Wait` recovery, no `RateController`. Replan rate is driven externally by `carrot_to_nav2_action_node` at 2 Hz. Similar shape to our `navigate_igvc_autonav_humble.xml` but with `Spin` instead of `DriveOnHeading` for recovery. They run a `goal_reach_checker_node` + `nav2_goal_cancel_node` as independent watchdogs (xy tol 0.40 m, yaw 0.35 rad, hold 0.5 s, 5 consecutive hits).

## Three lessons for us
1. **Decouple GPS from the TF tree.** They explicitly avoid `navsat_transform_node` driving `map`, instead pinning `map ≡ odom` identity and using GPS only to compute waypoint targets. This sidesteps the exact GPS-drift phantom-accumulation we fought (our 2026-05-19 STVL-global fix). Worth considering for outdoor courses where map-frame stability matters more than absolute georeference.
2. **External replan via custom action client (`carrot_to_nav2_action`) at 2 Hz** is a cleaner architecture than rate-controllers inside the BT — it sidesteps the BT recovery-loop coupling and gives you explicit control over goal updates as the waypoint carrot advances.
3. **Their MPPI is heavier per cycle (batch 2000, iter 3, regenerate noises) but at half the rate (10 Hz)** — total compute similar. Worth A/B testing on Jetson: if our 20 Hz @ batch 1000 / iter 1 ever misses deadlines under load again, switching to 10 Hz @ batch 2000 / iter 3 may give better trajectory quality with the same CPU budget.

## What was UNCLEAR
- Whether `ekf_global.yaml` is ever activated (its config exists, but `navigation.launch.py:373` hardcodes `goal_pose_topic = '/odometry/local'` and no `ekf_global` Node spawn). Possibly an indoor/outdoor mode toggle that was stripped.
- VESC `max_abs_duty: 0.3` (their `vesc_can_node.py:32`) — is that a permanent safety cap or test-bench setting? At 0.3 duty their 1.0 m/s `vx_max` implies a much lower-torque drivetrain than ours.
- Lane detection: classical OpenCV `lane_detector_node` exists but `enable_lane_detection: false` by default in the active launch — unclear if they intend to ship it.
- The unused `lidar: True` code path in `nvblox.launch.py` suggests they considered LiDAR fusion but dropped it; no rationale documented.
