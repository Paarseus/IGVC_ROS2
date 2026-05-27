# Sooner Competitive Robotics — Twistopher (IGVC 2025, 1st place)

## Repo & build
- URL: https://github.com/SoonerRobotics/autonav_software_2025 (clone worked, public)
- ROS 2 Jazzy / Ubuntu 24.04, `ros2 launch autonav_launch competition.xml`
- **Swerve drive** (holonomic) — `autonav_hardware/src/swerve/swerve_drive.py`
- **4 cameras + ultrasonics + VectorNav IMU/GPS — no LiDAR** (grep: zero `lidar/laser/velodyne` refs)
- Compute: Intel NUC (external claim)

## Architecture (one ASCII diagram)
```
4 cams → autonav_vision/{transformations,combination}.py
           │
           ▼ /autonav/vision/combined/filtered (binary mask)
VectorNav GPS+IMU → zemlin_filters/filters.py (PF, 750 particles)
           │
           ▼ /autonav/position (x,y,θ in local ENU)
ultrasonics → autonav_feelers/feeler_node ──► /autonav/motor_input
              (16 raycasts + bias sum + PID)         │
                                                     ▼
                                  sparkmax_node.py (swerve CAN)
```

## Localization
**Particle filter, not EKF.** `zemlin_filters/src/particlefilter.py` — 750 particles, motor odom propagates each, GPS reweights via `exp(-d/(2σ²))`, `gps_noise=0.45 m`, resample w/ Gaussian odom jitter. **No global map frame**: origin = first GPS fix, positions in local ENU (`(lat-lat0)*latLength`). IMU yaw is the heading source. Publishes `/autonav/position` (`x,y,θ`).

## Costmap / world representation
**None.** No `nav2_costmap_2d`, no `OccupancyGrid`. The world IS the camera frame — a binary `cv::Mat` from `autonav_vision/combination.py` (BEV-warped + thresholded 4-cam composite). 2024's grid-A* (`zemlin_navigation/src/astar.py`) is commented out in `competition.xml`. Mask consumed pixel-wise by `Feeler::update()` (`autonav_feelers/src/feeler.cpp:295-387`), walking each ray until a white pixel.

## Planner
**2D raycast heuristic ("Feelers"), C++.** `autonav_feelers/src/feeler_node.cpp`. 16 rays (balanced fore+aft, `start_angle=25°`, `max_length=100 px`), image-centered. Per ray:
1. `update()` walks pixels until obstacle or max length
2. forward bias = dot(ray, `Feeler(10,100)`) (`feeler_node.cpp:236-241`)
3. GPS bias = `gpsBiasWeight × (ray · gpsVector)` (`l. 362-376`)
4. **All rays vector-summed** → `headingArrow` (`calculateOutputs()`, l. 481-497)

No horizon, no graph — recomputed every image frame and every GPS msg. Chosen over A* because A* on a noisy vision mask produced jagged paths (2024 failure mode).

## Controller
**Planner output IS the command.** `headingArrow.y` → `forward_velocity` (±2.0 m/s); `headingArrow.x` → P-only PID (kP=0.002) → `angular_velocity` (±1.25 rad/s). `feeler_node.cpp:531-537`. Stuck-state hack: if `|cmd| < 0.1`, reverse-left to escape (`l. 541-549`).

## State machine
Tiny enum in `autonav_shared/include/autonav_shared/types.hpp`: **DISABLED → MANUAL → AUTONOMOUS → SHUTDOWN**. Each node inherits `AutoNav::Node`, overrides `on_system_state_updated()`. `autonav_commander/commander.py` is **NOT** the state machine — it's a node-discovery/preset loader. Direction (`compNorth`/`compSouth` waypoint set) is auto-picked 100 s into AUTONOMOUS by `position.theta` (`feeler_node.cpp:317-330`). Waypoints pop within `2 m` (l. 383-395). No "lane vs GPS" mode switch — both run continuously and additively.

## Three lessons for us (Cal Poly Pomona, Jetson Orin, Nav2 MPPI today)
1. **~700 LOC of C++ on the raw mask wins** — no costmap, no TF, no global frame. Our MPPI + dual-EKF + STVL + voxel-costmap stack fights problems (RViz starving the loop, GPS smear, costmap flicker) that don't exist here. Worth prototyping a feelers-style fallback for when `ComputePathToPose` aborts.
2. **GPS as a bias vector, not a goal pose.** They never send `NavigateToPose` in map frame. The waypoint is a *force* added to heading — transient GPS jumps nudge, don't trigger replans. Our `map→odom` GPS-drift problem (9c51906) vanishes when GPS isn't a hard frame.
3. **PF beats EKF for short outdoor runs.** PF resamples every GPS fix; our Xsens stuck-bias incident would have been diluted by 750 particles instead of steering us off course.

## What was UNCLEAR / would need a deeper read
- **Whether `competition.xml` is what actually ran at IGVC 2025.** Feelers/vision/filters are commented out there; `feelers_irl.xml` looks like the real layout. The committed competition.xml may be stale.
- `autonav_vision/combination.py` (BEV stitching + thresholding) skimmed only; color thresholds unread.
- `ultrasonic_contribution` defaults to `1` but the addition is commented out (`feeler_node.cpp:493-496`) — UNCLEAR if ultrasonics contributed at competition.
- 2:20 course-time and Intel NUC claims are external, not verifiable from the repo.
