# Hosei University KBKN Lab — orange_ros2 (ROS2 Humble, Tsukuba + IGVC)

## Repo & build
`KBKN-Autonomous-Robotics-Lab/orange_ros2` (15 stars). Build via `wstool`+`rosinstall` per [`orange_ros2.rosinstall`](https://github.com/KBKN-Autonomous-Robotics-Lab/orange_ros2/blob/main/orange_ros2.rosinstall) — pulls 12 sibling packages: `velodyne`, `icm_20948`, `estop_ros`, `orange_navigation` (custom waypoint follower, distinct from this repo's `orange_navigation` config dir), `kbkn_maps`, `multi_map_manager`, `linefit_ground_segmentation_ros2`, `FAST_LIO`, `fast_odom_convert`, `pcd_convert`, `livox_laser_simulation_RO2`, `serial`. Hardware: ZLAC8015D motors, ICM-20948 IMU, ZED-F9P GNSS (moving-base RTK), Livox MID360 LiDAR.

## Architecture (one ASCII diagram)
```
MID360 ─► livox_to_pointcloud2 ─► pcd_convert (ground seg) ─► /livox_scan ─► AMCL → /map→/odom
   │                                                                       └► costmaps (global+local)
   └─► FAST-LIO ─► /odom_fast ─┐
ICM-20948 ─► IMU ─► FAST-LIO   ├─► ekf_myself.py ─► /fusion/odom ─► DWB ─► waypoint_nav.cpp (custom)
ZED-F9P base+rover ─► GPSodom ─┘  (custom Py, NOT                          ↓ navigate_to_pose action
ZLAC8015D ─► /odom; estop_ros overrides /cmd_vel    robot_localization)    tandem_run_manager (toggles obstacle_layer)
```
robot_localization is commented out in [`data_processing.launch.xml:16-20`](https://github.com/KBKN-Autonomous-Robotics-Lab/orange_ros2/blob/main/orange_bringup/launch/data_processing.launch.xml).

## Localization
**AMCL on a pre-saved map** ([`navigation2_params.yaml:6-47`](https://github.com/KBKN-Autonomous-Robotics-Lab/orange_ros2/blob/main/orange_navigation/config/navigation2_params.yaml)): `likelihood_field`, 1500 max particles, `set_initial_pose: True` at (0,0,0). Odom input is `/fusion/odom` — a custom Python EKF (`ekf_myself.py`, 220 lines) fusing FAST-LIO LiDAR-inertial odom with GPS moving-base heading. No `robot_localization`, no `navsat_transform_node`; `ekf_node.yaml` is dead code.

## SLAM strategy
slam_toolbox is the production choice (README lists it first; cartographer is a fallback). Always **online async mapping** ([`online_slam_toolbox_params.yaml:17`](https://github.com/KBKN-Autonomous-Robotics-Lab/orange_ros2/blob/main/orange_slam/config/online_slam_toolbox_params.yaml) — `mode: mapping`, `#localization` commented out); `async_slam_toolbox_node` live, `sync_*` only for bag replay. **IGVC §I.2 compliance:** the AMCL+pre-saved-map flow targets **Tsukuba** (`waypoint_navigation.launch.xml:5` hardcodes `hosei/2024/nakaniwa_odomfast_GPS.yaml`). For IGVC, [`kbkn_maps/waypoints/IGVC/2025/autonav.yaml`](https://github.com/KBKN-Autonomous-Robotics-Lab/kbkn_maps/blob/main/waypoints/IGVC/2025/autonav.yaml) is **lat/lon-only** (`gps_points: [42.4009...]`) with **no companion `.pgm`** — so the IGVC run can't use the AMCL flow and must run online slam_toolbox per run (mapping mode, no `map_load`). The public repo has no launch wiring `gps_points` into nav; that consumer lives in a private competition fork.

## Costmap / world representation
8×8 m rolling local (`obstacle_layer`+`inflation_layer`, radius 0.3, `cost_scaling_factor: 25.0` — very steep), unbounded global with `static_layer`+`obstacle_layer`+`inflation_layer` (radius 1.5, scaling 2.0). All obstacles from `/livox_scan` (pcd→laserscan via `pcd_convert` ground segmentation). Rectangular footprint `[[0.20,0.33],[0.20,-0.33],[-0.65,-0.33],[-0.65,0.33]]`. No voxel/semantic/GPS layers.

## Planner + Controller
Planner: `nav2_navfn_planner/NavfnPlanner` (Dijkstra, `tolerance: 0.5`, `allow_unknown: true`). Controller: **DWB**, 8 critics (`RotateToGoal`, `PathAlign`, `GoalAlign`, `PathDist`, `GoalDist`, `BaseObstacle`, `ObstacleFootprint`, `Oscillation`), `max_vel_x: 1.4`, `max_vel_theta: 0.5`, `sim_time: 2.0 s`, 10 lin × 10 theta samples, 5 Hz loop. BT is **stock `navigate_w_replanning_and_recovery.xml`** — spin/backup/wait recoveries, no custom tree.

## Waypoint follower
**Custom C++** ([`orange_navigation/waypoint_navigation/src/waypoint_nav.cpp`](https://github.com/KBKN-Autonomous-Robotics-Lab/orange_navigation/blob/humble-devel/waypoint_navigation/src/waypoint_nav.cpp), 432 lines), **not `nav2_waypoint_follower`**. Loads YAML, fires goals one-by-one at `navigate_to_pose`, exposes `start_wp_nav`/`stop_wp_nav`/`resume_nav` services, supports `start_from_middle` (snap to nearest forward-facing waypoint), per-waypoint `vel`/`rad`/`stop` flags. Sidecar `tandem_run_manager.py` reads `tandem_start`/`tandem_end` tags and **toggles `obstacle_layer.enabled` on the global costmap** when a follower robot is in the LiDAR front cone — Tsukuba cooperative-driving feature.

## Three lessons for us
1. **Don't reinvent EKF.** `ekf_myself.py` is 220 lines hand-rolling what `robot_localization`'s two-EKF cascade already does — tying them to one sensor set. Our `robot_localization` + `navsat_transform_node` is more maintainable; the lever-arm fix we documented is smaller debt than DIY EKF.
2. **Versioned waypoint repo.** Putting `[lat, lon, vel, heading]` rows in an external `kbkn_maps` repo decouples course coords from launch code. Worth mirroring for our `cpp_campus_graph.geojson` workflow.
3. **DWB still ships in 2025.** A 15-star team picked DWB over MPPI/RPP — so when our MPPI starves the loop on grass, DWB with `sim_time: 2.0` + 10 critics is a known-good escape valve. Worth keeping `nav2_params_dwb.yaml` as a shadow config.

## What was UNCLEAR
- **No IGVC launch in public repo.** `gps_points` YAMLs exist but nothing in `orange_ros2` consumes them — IGVC entry uses a private fork. Cannot confirm whether they re-SLAM each run or pre-load a practice-day map (which §I.2 forbids).
- **`use_sim_time: True` hardcoded** on every node in `navigation2_params.yaml`; presumably `bringup_launch.py` overrides, but verify before borrowing.
- **`Position_magnification: 1.675`** in `lonlat_to_odom` ([`data_processing.launch.xml:27`](https://github.com/KBKN-Autonomous-Robotics-Lab/orange_ros2/blob/main/orange_bringup/launch/data_processing.launch.xml)) — undocumented scale factor, probably lever-arm or local-meters-per-degree hack.
