# Lidar Obstacle Avoidance and Waypoint Test Plan (2026-10-02)

Uses the existing stack only: `ros2 launch avros_bringup navigation.launch.py` (sensors + EKF + navsat + Nav2 with MPPI + STVL lidar costmaps + `actuator_node` + Foxglove bridge), the recovery BT `navigate_igvc_autonav_humble.xml`, and `mission_manager` for GPS waypoints. Earlier result: `lidar_obstacle_avoidance_test_2026_05_21.md` (avoidance passed; a clean goal-reached run was never captured).

## 1. The waypoint
Given: **34.059635, −117.821028**. From the robot's last known position (34.059302, −117.821821, 2026-10-02 19:49) it is **82 m away: 37 m north, 73 m east, bearing 063°** (85 m from the navsat datum).

| Consequence | Detail |
|---|---|
| Too far for one goal | The global costmap is a 100 × 100 m rolling window (about 50 m reach) and the Navfn planner cannot path to a goal outside it (`nav2_params_igvc_autonav.yaml`, global_costmap width/height). |
| Use intermediate legs | `mission_manager` takes a list of lat/lon points and advances at 2.0 m. Split the line into legs of 25–30 m (three legs here), computed from the robot's **live** position at the start. |
| Needs RTK FIXED | Goals are converted from GPS through `/fromLL`; the file itself says accuracy requires RTK FIXED. Today RTK flickered between FIXED and FLOAT outdoors. |
| The path must be known | The lidar sees about 3 m reliably and does not see drop-offs, stairs or curbs (negative obstacles). Someone must confirm the 82 m route is walkable for a 0.83 m wide robot. |

## 2. Findings that must be handled first
| # | Finding | Action |
|---|---|---|
| N1 | **`enable_mission_manager` defaults to true** and auto-starts the waypoint mission about 20 s after launch; the shipped `waypoints.yaml` holds the IGVC course in Michigan | launch with `enable_mission_manager:=false` for stages 1–3; for stage 4 point `mission_manager` at a local waypoint file |
| N2 | **Is the lidar alive?** The sensor log showed repeated `velodyne_driver_node: poll() timeout` warnings this session (no packets) | first check: `ros2 topic hz /velodyne_points` ≥ 15 Hz; check the lidar's power and cable if not |
| N3 | `controller_server` has no `odom_topic` in either Nav2 file (only `bt_navigator` and `velocity_smoother`), so MPPI may be seeded with zero velocity (finding F1) | verify live (`ros2 param get /controller_server odom_topic`); if default, add `odom_topic: /odometry/filtered` (needs your approval) |
| N4 | MPPI `wz_max` 1.9 vs actuator cap 1.5 (F2); heading-hold on (F3) | cap `wz_max` at 1.5; set `heading_hold_deadband` 0.0 for these tests (needs approval) |
| N5 | Drive gains: the robot holds the baseline; the tuned set (`TUNING_LOG.md` §5) is verified only through the Teensy directly | decide: baseline (as in May, comparable) or tuned set |
| N6 | Never run RViz on the Jetson; use laptop Foxglove (port 8765) | as before |
| N7 | After any e-stop the drivers power-cycle and their RAM gains revert; `actuator_node` has no serial reconnect | restart the stack after an e-stop |
| N8 | The shared 12 V rail sags under load; keep speeds modest | cap speed (below) |

## 3. Stages
Speed cap for all stages: MPPI `vx_max` 0.35 m/s (live parameter) and actuator `max_linear_mps` 0.4 as backstop (as in May). Operator at the e-stop, clear area, no one in the robot's path except where a stage says so.

| Stage | What | Pass |
|---|---|---|
| 0 Pre-flight | robot up; Jetson clock synced; `CHK OK`; IMU 100 Hz; `/velodyne_points` ≥ 15 Hz; RTK status; `tailscale`/network; load average; N3 check | all green |
| 1 Static (no driving) | launch with `enable_mission_manager:=false`; a box placed 2–3 m ahead appears as lethal cells in the local costmap within 2 s and clears when removed; control loop ≥ 18 Hz; load < 7; TF stable | detection, no missed loops |
| 2 Clean goal | map-frame goal 6–8 m straight ahead, no obstacle (action client as in May) | **SUCCEEDED** (status 4), no recovery, stops within 2 m of goal |
| 3 Obstacle | same goal; soft obstacle (cardboard box or cone, no person) placed 3–4 m ahead, centred, then offset 0.5 m left and right; 3 repeats each | no collision, min clearance recorded, reaches the goal, ≤ 2 recoveries |
| 4 GPS waypoint | custom `waypoints.yaml` with the given waypoint as the last point and legs ≤ 30 m; RTK FIXED before starting; first without an obstacle, then one obstacle on the line | reaches within 2 m of the waypoint, no collision |

## 4. Metrics and records
Per run: success status, time, path length vs straight line, minimum lidar clearance, number of recoveries, `/cmd_vel` rate (≥ 18 Hz), "control loop missed" count, bus voltage minimum, RTK state, load average. Record a ROS bag (`/cmd_vel`, `/odometry/filtered`, `/plan`, `/local_plan`, local costmap, `/tf`, `/velodyne_points`, `/diagnostics`, `/rosout`) with `tools/ground/bag.sh` (adapt the topic list), and keep notes in the session folder.

## 5. Abort rules
Any collision or near miss under 0.3 m, a person entering the path, RTK dropping to SINGLE during stage 4, bus below 9.5 V, `actuator_node` serial errors, or the operator's call: e-stop, then restart the whole stack before the next run.

## 6. Decisions needed before stage 0
1. Drive gains: baseline or tuned set (N5).
2. Approve N3 (`odom_topic`) and N4 (`wz_max`, heading-hold) configuration changes.
3. Obstacle type and a confirmed-clear route for the 82 m path (or choose a nearer goal for stage 4).
4. Robot powered up and reachable.
