# Standards Reference: Nav2 + robot_localization + nav2_route on ROS 2 Humble

This document captures what professional / upstream-standard configuration looks
like for each part of a Nav2 + robot_localization + nav2_route deployment. It is
a *reference* for downstream reviewers — Phase 2 of the review will compare the
actual `nav2_params.yaml`, BT XML, `ekf.yaml`, and `navsat.yaml` in this repo
against the standards captured here. Sources are cited inline.

---

## 1. REP-105 / REP-103 Frame Conventions

### Required frames and their semantics

- REP-105 specifies four frames for mobile platforms: `earth`, `map`, `odom`,
  `base_link`, with the chain `earth -> map -> odom -> base_link`
  ([REP-105](https://www.ros.org/reps/rep-0105.html)).
- `base_link` is rigidly attached to the robot body at an arbitrary reference
  point. `odom` is world-fixed and **continuous** (smooth, no jumps) but can
  drift unboundedly. `map` is world-fixed and **may jump discretely** when the
  localizer corrects, but minimal long-term drift
  ([REP-105](https://www.ros.org/reps/rep-0105.html)).
- `earth` is the ECEF (Earth-Centered Earth-Fixed) frame for multi-robot or
  global geo-referenced systems, and is parent of `map`. For a single-robot
  campus deployment it is rarely published explicitly
  ([REP-105](https://www.ros.org/reps/rep-0105.html)).
- `base_footprint` is **not** in REP-105. It is a community convention (used
  heavily by Nav2 / `robot_state_publisher` URDFs) for a 2D-projected,
  ground-plane base frame. When present, it is normally a child of `base_link`
  or vice-versa via a fixed (URDF) joint and is published statically by
  `robot_state_publisher`.

### Static vs dynamic, who publishes what

| Transform | Type | Publisher |
|---|---|---|
| `earth -> map` | Static (when used at all) | `static_transform_publisher` or `navsat_transform_node` `broadcast_cartesian_transform` |
| `map -> odom` | Dynamic | Localizer (AMCL, SLAM Toolbox, or robot_localization map-EKF) |
| `odom -> base_link` | Dynamic | Odometry source (wheel odometry, VIO, or robot_localization odom-EKF) |
| `base_link -> sensor_link` | Static | `robot_state_publisher` from URDF |

Citations: [REP-105](https://www.ros.org/reps/rep-0105.html);
[robot_localization GPS integration guide](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/integrating_gps.rst).

### ENU vs NED

- ROS conventions are **ENU** by default: x-east, y-north, z-up; mobile-base
  body frames are x-forward, y-left, z-up
  ([REP-103](https://www.ros.org/reps/rep-0103.html)).
- NED frames are explicitly suffixed `_ned` and are reserved for
  aerospace/legacy systems
  ([REP-103](https://www.ros.org/reps/rep-0103.html)).
- IMU data fed to robot_localization **must** be in ENU; an automotive IMU that
  reports zero-yaw at magnetic-north (NED-style) requires a `yaw_offset` of
  approximately +pi/2 in `navsat_transform_node`
  ([dual_ekf_navsat_example.yaml](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/params/dual_ekf_navsat_example.yaml)).

### Common mistakes

- **Publishing `map -> base_link` directly from a localizer.** Breaks the TF
  tree because every frame can have only one parent; `odom -> base_link` is
  already published by the odometry source. Localizer must publish
  `map -> odom` instead so the chain composes correctly
  ([REP-105](https://www.ros.org/reps/rep-0105.html)).
- Conflating `base_link` with `base_footprint` — a sensor mounted relative to
  the chassis should attach to `base_link`, not to `base_footprint`.
- Using non-ENU IMU data with no `yaw_offset` set in `navsat_transform_node`
  ([dual_ekf_navsat_example.yaml](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/params/dual_ekf_navsat_example.yaml)).
- Camera/optical frames using x-forward instead of the documented `_optical`
  convention (z-forward, x-right, y-down)
  ([REP-103](https://www.ros.org/reps/rep-0103.html)).
- Quaternions hand-written as `(w, x, y, z)`. ROS 2 (and tf2) use
  `(x, y, z, w)` order; identity is `(0, 0, 0, 1)`
  ([Quaternion Fundamentals](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html)).

---

## 2. robot_localization EKF

### Core parameters

- `frequency` (Hz): output rate. Typical 30 Hz for indoor, 30-50 Hz for outdoor
  with high-rate IMU
  ([dual_ekf_navsat_example.yaml](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/params/dual_ekf_navsat_example.yaml)).
- `sensor_timeout` (s): how long to dead-reckon when sensors stop updating.
  Default 0.1 s; should not exceed `1/frequency` by much
  ([ekf.yaml](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/params/ekf.yaml)).
- `two_d_mode`: forces z, roll, pitch, vz, vroll, vpitch and az to zero. Set
  `true` for a planar tracked/diff-drive vehicle on flat ground; set `false`
  when the platform pitches/rolls or you need full 3D state
  ([ekf.yaml](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/params/ekf.yaml)).
- `print_diagnostics`: `true` in production — surfaces stale sensors, NaN
  measurements, and frame-id mismatches via the `/diagnostics` topic
  ([ekf.yaml](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/params/ekf.yaml)).
- `publish_tf`: `true` for whichever EKF owns the `world_frame -> odom` or
  `odom -> base_link` edge; **at most one** node should publish each TF edge.
- `publish_acceleration`: true publishes a `geometry_msgs/AccelWithCovarianceStamped`.

### `*_config` 15-bool matrix semantics

The matrix is in the order `[X, Y, Z, roll, pitch, yaw, vx, vy, vz, vroll,
vpitch, vyaw, ax, ay, az]`, given in the **frame_id of the input message**, not
the EKF target frame
([preparing_sensor_data](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/preparing_sensor_data.rst)).

- Prefer fusing **velocities** (vx, vyaw, etc.) from wheel odometry rather than
  absolute pose, to avoid double-counting one source of information
  ([preparing_sensor_data](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/preparing_sensor_data.rst)).
- For nonholonomic robots, fusing `vy = 0` (with realistic covariance) acts as
  a soft sideslip constraint
  ([preparing_sensor_data](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/preparing_sensor_data.rst)).
- **Do not** fuse the same physical signal from two sources (e.g. orientation
  from both an IMU and a magnetometer-corrected IMU pose) without
  `_differential = true` on the second one
  ([preparing_sensor_data](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/preparing_sensor_data.rst)).
- Don't fuse linear acceleration from a noisy IMU on a slow ground robot —
  integration drift dominates
  ([preparing_sensor_data](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/preparing_sensor_data.rst)).

### `_differential` vs `_relative` flags

- `_differential = true` differentiates absolute pose into velocity before
  fusion. Use it when a second orientation source would otherwise fight the
  first
  ([preparing_sensor_data](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/preparing_sensor_data.rst)).
- `_relative = true` makes the **first** received measurement the zero. Useful
  when an IMU reports absolute heading but you want the EKF to start at
  `(x=0, y=0, yaw=0)` regardless of where the robot was powered on.
- The two flags are mutually exclusive in spirit — pick the one that matches
  intent.

### `world_frame` semantics and the dual-EKF pattern

- `world_frame = odom_frame`: this EKF publishes `odom -> base_link`. Inputs
  must be **continuous-only** sources (wheel odom, IMU, VIO) — no GPS, no AMCL
  poses
  ([integrating_gps](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/integrating_gps.rst)).
- `world_frame = map_frame`: this EKF publishes `map -> odom` (computed by
  composing its `map -> base_link` estimate against the odom-EKF's
  `odom -> base_link`). Inputs may include all sensors, plus
  `navsat_transform_node`'s `/odometry/gps` for absolute corrections
  ([integrating_gps](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/integrating_gps.rst);
  [Nav2 GPS tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html)).
- The convention: name the nodes `ekf_filter_node_odom` and
  `ekf_filter_node_map` and have them publish `/odometry/filtered` and
  `/odometry/filtered/global` respectively
  ([Nav2 GPS tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html)).

### Common mistakes

- Feeding GPS into the same EKF as wheel odometry. GPS jumps would propagate
  into the `odom -> base_link` transform and break Nav2 controllers (which
  assume `odom` is continuous)
  ([integrating_gps](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/integrating_gps.rst)).
- Two nodes both setting `publish_tf = true` for `odom -> base_link`. Only one
  publisher per TF edge.
- Inflating covariance to "ignore" a state variable instead of disabling it in
  `*_config`
  ([preparing_sensor_data](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/preparing_sensor_data.rst)).
- Zero variance entries in input covariances — the filter adds 1e-6 to avoid
  numerical issues, but a real covariance is preferable
  ([preparing_sensor_data](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/preparing_sensor_data.rst)).
- Forgetting that `*_config` is in the **input message's frame_id** — common
  when an IMU reports angular velocity in `imu_link` rather than `base_link`.

---

## 3. navsat_transform_node

### Purpose

- Converts a `sensor_msgs/NavSatFix` (lat/lon) into a UTM-anchored
  `nav_msgs/Odometry` published as `/odometry/gps`, expressed in your local
  `map` frame, so a downstream EKF can fuse it as a positional correction
  ([integrating_gps](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/integrating_gps.rst);
  [Nav2 GPS tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html)).
- Consumes three topics: GPS fix, IMU (for absolute heading), and the **map-EKF
  output** `/odometry/filtered/global` (for closing the loop on robot pose)
  ([integrating_gps](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/integrating_gps.rst)).

### `datum` vs first-fix

- Default (`wait_for_datum: false`): the **first** GPS fix becomes the origin
  of the local cartesian frame
  ([integrating_gps](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/integrating_gps.rst)).
- `wait_for_datum: true` + `datum: [lat, lon, yaw, world_frame, base_link_frame]`
  forces a fixed origin — required for **multi-session repeatability** (e.g.,
  the same campus origin every run, so a pre-built route graph in `map` frame
  stays valid)
  ([integrating_gps](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/integrating_gps.rst)).
- Alternatively, call the `/datum` (`SetDatum.srv`) service at runtime
  ([integrating_gps](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/integrating_gps.rst)).

### `magnetic_declination_radians` and `yaw_offset`

- `magnetic_declination_radians`: local declination at the operating site
  (look up at e.g. NOAA's geomagnetic calculator). Sign matters — see the
  example value `0.0429` rad for Edinburgh
  ([dual_ekf_navsat_example.yaml](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/params/dual_ekf_navsat_example.yaml)).
- `yaw_offset`: rotates the IMU's reference direction. If the IMU reports
  zero-yaw at magnetic north (NED convention), set `1.5707963` to convert to
  ENU (zero at east)
  ([dual_ekf_navsat_example.yaml](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/params/dual_ekf_navsat_example.yaml)).

### Other knobs

- `broadcast_cartesian_transform` / `broadcast_utm_transform`: whether to
  publish `utm -> map` on `/tf`. Useful for visualizing GPS in RViz.
- `publish_filtered_gps`: re-projects the EKF's `map`-frame estimate back to
  lat/lon for monitoring or logging.
- `use_odometry_yaw`: ignores IMU heading and uses the EKF-fused yaw instead.
  Only enable once odometry yaw has stabilized.

### Pitfalls

- IMU `frame_id` not in REP-103 ENU. Without `yaw_offset`, the GPS-derived
  odometry will be rotated 90 degrees off
  ([dual_ekf_navsat_example.yaml](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/params/dual_ekf_navsat_example.yaml)).
- Setting `wait_for_datum: false` while having a pre-built route graph in
  `map` frame — the map-frame origin moves between runs, so the graph drifts.
- Forgetting `magnetic_declination_radians` (treating mag-north as true-north)
  will rotate the global frame several degrees.

---

## 4. Nav2 Controller / Planner Choices

### When SmacPlannerHybrid is appropriate

- Hybrid-A* planner for **non-holonomic vehicles** with a finite turning
  radius (Ackermann, car-like, large skid-steer). Produces kinematically
  feasible paths and supports forward-only (DUBIN) or reversing (REEDS_SHEPP)
  motion ([SmacPlanner](https://docs.nav2.org/configuration/packages/configuring-smac-planner.html)).
- For circular differential drives, prefer SmacPlanner2D or NavfnPlanner.
- For arbitrary control sets (omni, complex kinematics), use the State Lattice
  variant ([SmacPlanner](https://docs.nav2.org/configuration/packages/configuring-smac-planner.html)).

### Key SmacPlannerHybrid knobs

- `motion_model_for_search`: `DUBIN` (forward-only), `REEDS_SHEPP` (with
  reverse), `MOORE` (omni)
  ([SmacPlanner](https://docs.nav2.org/configuration/packages/configuring-smac-planner.html)).
- `minimum_turning_radius`: must match the vehicle's actual physical limit; if
  set too small, the planner produces infeasible curves; too large and it
  rejects valid paths.
- `allow_unknown`: usually `true` outdoors so the planner can route through
  not-yet-mapped regions.
- `analytic_expansion_ratio` and `lookup_table_dim` trade speed for accuracy.
- `cost_travel_multiplier`: bias toward staying away from obstacles.

### Regulated Pure Pursuit (RPP) tuning

- `lookahead_dist` (fixed) **or** `min/max_lookahead_dist` with
  `use_velocity_scaled_lookahead_dist = true` — short lookahead for tight
  navigation, long lookahead for highway-style following
  ([RPP](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html)).
- `regulated_linear_scaling_min_radius` / `regulated_linear_scaling_min_speed`
  + `use_regulated_linear_velocity_scaling = true` — slow down on tight turns
  ([RPP](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html)).
- `use_cost_regulated_linear_velocity_scaling = true` slows near costmap
  obstacles ([RPP](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html)).
- `max_robot_pose_search_dist`: cap on how far back along the path the
  controller will look for a starting point — keep just larger than typical
  pose drift to avoid finding a stale closer-to-start point
  ([RPP](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html)).
- `allow_reversing`: enable only for vehicles whose planner can produce
  reversing paths (e.g. SmacPlannerHybrid + REEDS_SHEPP).
- `use_collision_detection` + `max_allowed_time_to_collision_up_to_carrot` —
  forward-simulate the next ~1-2 s and abort if collision predicted.

### Goal checker tolerances

- `xy_goal_tolerance` and `yaw_goal_tolerance` live in the goal-checker
  plugin, not the controller. Outdoor / large-vehicle deployments commonly
  use 0.5-2.0 m and 0.25-1.0 rad
  ([Controller Server](https://docs.nav2.org/configuration/packages/configuring-controller-server.html)).
- `stateful: true` means the goal-checker latches once tolerance is hit (no
  oscillation around the goal); `stateful: false` re-evaluates each tick.
- `SimpleProgressChecker` parameters `required_movement_radius` and
  `movement_time_allowance` define stuck detection.

### BT with vs without recoveries — a competition trade-off

- Default Nav2 BT (`navigate_to_pose_w_replanning_and_recovery.xml`) wraps the
  primary navigation in a `RecoveryNode` that triggers Spin / BackUp / Wait
  on failure ([BT default](https://github.com/ros-navigation/navigation2/blob/main/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml)).
- Competition BTs (e.g. for IGVC AutoNav) often **omit** Spin and BackUp:
  spinning in place can violate "always-forward" judging rules, and a
  large-radius vehicle cannot Spin on a 2 m course.
- The trade-off: without recoveries, **any** transient sensor dropout aborts
  the run. Mitigations: longer `failure_tolerance`, `Wait` recovery instead
  of motion recoveries, and tighter Collision Monitor for the forward path.

---

## 5. Costmap 2D Layers

### Standard layer ordering

The conventional `plugins:` order is **StaticLayer -> ObstacleLayer (or
VoxelLayer) -> InflationLayer**
([Costmaps](https://docs.nav2.org/configuration/packages/configuring-costmaps.html)).
Plugins run sequentially; later ones may override earlier values
([Costmaps](https://docs.nav2.org/configuration/packages/configuring-costmaps.html)).

- **StaticLayer** loads a precomputed map (e.g. from SLAM) — usually only on
  the global costmap.
- **ObstacleLayer** marks 2D laser-scan / pointcloud hits as occupied and
  raytraces clear-space.
- **VoxelLayer** does the same with 3D voxels — required when the LiDAR/depth
  cloud has Z information (Velodyne, RealSense depth) so overhanging branches
  don't get smeared into the ground plane
  ([Costmaps](https://docs.nav2.org/configuration/packages/configuring-costmaps.html)).
- **InflationLayer** must always be **last** — it blurs the cost field
  outward by `inflation_radius`, and any layer running after it would corrupt
  the gradient. Custom plugin layers should generally sit between ObstacleLayer
  and InflationLayer.

### Custom plugin layers (e.g., kiwicampus/semantic_segmentation_layer)

- A semantic-segmentation layer typically marks pixels classified as
  `road / grass / barrier / obstacle` with per-class costs into the costmap.
  It needs the same lifecycle, transform, and parameter contract as the
  built-in layers.
- Position it **after** ObstacleLayer (so geometric obstacles still register)
  and **before** InflationLayer (so semantic costs get smoothed too).
- Topic contracts matter: the kiwicampus plugin needs a mask, an organized
  pointcloud sharing the mask's `header.stamp`, and a transient-local
  `vision_msgs/LabelInfo` so a late-joining costmap subscriber sees the class
  table.

### `raytrace_max_range` vs `obstacle_max_range`

- `obstacle_max_range`: max distance at which sensor returns are allowed to
  **mark** new obstacles. Conservative — typically the sensor's reliable
  range ([Costmaps](https://docs.nav2.org/configuration/packages/configuring-costmaps.html)).
- `raytrace_max_range`: max distance at which the layer will **clear** stale
  obstacles by tracing rays. Should be **larger** than `obstacle_max_range`
  so the layer can clear noise it would not have marked
  ([Costmaps](https://docs.nav2.org/configuration/packages/configuring-costmaps.html)).
- A common bug: `raytrace_max_range < obstacle_max_range` -> obstacles get
  marked but never cleared, costmap accumulates ghosts.

### Footprint vs `robot_radius`

- `robot_radius` (a single double) gives a circular collision model — fast,
  fine for round/square robots
  ([Costmaps](https://docs.nav2.org/configuration/packages/configuring-costmaps.html)).
- `footprint` (polygon) is required for elongated platforms (cars, tracked
  robots, IGVC-style vehicles where length >> width) — circle inscribed
  collision-checks are too pessimistic.
- Pick **one** — setting both leads to undefined behavior.

### `inflation_radius` and `cost_scaling_factor`

- `inflation_radius` should be `~0.5-1.0 *` the largest body half-extent, so
  the planner has a soft buffer beyond hard collision.
- `cost_scaling_factor` controls the exponential decay rate of inflated cost.
  Higher = sharper drop-off; the navigation2_tutorials example uses 3.0,
  which is a good default
  ([Costmaps](https://docs.nav2.org/configuration/packages/configuring-costmaps.html)).
- Sizing rule: with default `cost_scaling_factor = 10.0`, inflation cost
  decays to LETHAL/4 at roughly `0.1 m` from the obstacle. Tuning these two
  together changes how aggressively the controller hugs corridors.
- `inflate_unknown`: usually `false` outdoors so the planner can traverse
  unknown space at low cost.

### VoxelLayer params

- `z_resolution`, `z_voxels`, `mark_threshold`, `unknown_threshold` —
  determine the 3D occupancy slab. `publish_voxel_map: true` is invaluable
  for debugging height-thresholding (visible in RViz)
  ([Costmaps](https://docs.nav2.org/configuration/packages/configuring-costmaps.html)).
- `min_obstacle_height` / `max_obstacle_height` per observation source filter
  pointcloud points by Z before marking — set above the ground plane and
  below the vehicle height.

### Rolling-window pattern

- **Local costmap**: `rolling_window: true`, sized 5-10 m square — a moving
  window centered on `base_link` for the controller to react to dynamic
  obstacles ([Costmaps](https://docs.nav2.org/configuration/packages/configuring-costmaps.html)).
- **Global costmap**: typically `rolling_window: false` for indoor with a
  fixed map; `rolling_window: true` with a 50-200 m window outdoors where
  there's no full prebuilt map.

### `expected_update_rate` and stale data

- Each observation source should have `expected_update_rate` set just below
  its real sensor rate. The layer will stop trusting the source if updates
  fall behind, and `print_diagnostics` will surface the warning
  ([Costmaps](https://docs.nav2.org/configuration/packages/configuring-costmaps.html)).

---

## 6. Behavior Trees

### Standard BTs in nav2_bt_navigator

- `navigate_to_pose_w_replanning_and_recovery.xml` — the canonical default.
  Outer `RecoveryNode` (6 retries) wraps a `PipelineSequence`:
  selectors -> `RateController(1 Hz) ComputePathToPose` -> `FollowPath`, with
  fallback recoveries `Spin (1.57 rad)`, `Wait (5 s)`, `BackUp (0.30 m at
  0.15 m/s)` cycled by `RoundRobin`
  ([nav2_bt_navigator default BT](https://github.com/ros-navigation/navigation2/blob/main/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml)).
- `navigate_through_poses_w_replanning_and_recovery.xml` — same pattern but
  uses `ComputePathThroughPoses` and `TruncatePathLocal` so the controller
  always sees a forward window ([Nav2 BT docs](https://docs.nav2.org/behavior_trees/index.html)).
- Both register against the BT navigator via the `navigators` parameter
  (`['navigate_to_pose', 'navigate_through_poses']`)
  ([BT Navigator](https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html)).

### When to write a custom BT

- You need a non-standard goal-finding step (e.g., `ComputeRoute` from
  `nav2_route` instead of `ComputePathToPose`).
- Recoveries must be domain-specific (e.g., "honk before backing up", "open
  door service call before crossing edge").
- The mission has multi-stage goals (pick-place, dock-undock, waypoint
  cycles) that don't decompose into NavigateThroughPoses.
- The `default_nav_to_pose_bt_xml` parameter accepts `$(find-pkg-share ...)`
  substitution so a custom BT can ship from any package
  ([BT Navigator](https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html)).

### How recoveries chain

- `RecoveryNode` (Nav2-specific control node) takes a primary child and a
  recovery child. On primary failure, it ticks the recovery, then retries the
  primary up to `number_of_retries` times
  ([Nav2 BT docs](https://docs.nav2.org/behavior_trees/index.html)).
- Recovery actions are typically wrapped in a `RoundRobin` so successive
  failures cycle through `Spin -> Wait -> BackUp` rather than always running
  the same one
  ([Nav2 BT default](https://github.com/ros-navigation/navigation2/blob/main/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml)).
- Recoveries clear costmaps (`ClearEntireCostmap-Local`,
  `ClearEntireCostmap-Global`) before motion recoveries to deal with stuck
  ghost obstacles.

### Why omit recoveries (competition trade-off)

- Recovery primitives like `Spin` and `BackUp` violate forward-only judging
  rules at competitions like IGVC AutoNav.
- A bare BT (`PipelineSequence` -> `RateController(ComputeRoute)` ->
  `FollowPath`) gives deterministic behavior at the cost of robustness:
  any planner failure aborts the run.
- Mitigations when omitting recoveries: increase `failure_tolerance` on the
  controller server, add a longer `Wait` action in front of `ComputeRoute`,
  use `KeepRunningUntilFailure` decorator to retry forever instead of giving
  up.

### Group, ReactiveSequence, PipelineSequence patterns

- `Sequence` — strict left-to-right; each child must SUCCESS before the next
  ticks. Used for setup-then-execute flows.
- `ReactiveSequence` — re-ticks all earlier children on every iteration; the
  whole sequence aborts if any child fails. Used to combine continuous
  conditions (e.g., "while goal-not-updated, follow path")
  ([Nav2 BT docs](https://docs.nav2.org/behavior_trees/index.html)).
- `PipelineSequence` — Nav2-specific; runs children concurrently like a
  pipeline. Allows replanning to run in parallel with following the current
  plan
  ([Nav2 BT docs](https://docs.nav2.org/behavior_trees/index.html)).
- `Fallback` / `RoundRobin` — try-the-next-thing-on-failure patterns for
  recoveries ([Nav2 BT docs](https://docs.nav2.org/behavior_trees/index.html)).

---

## 7. nav2_route

### What it does

- Layer **on top of** standard Nav2 — replaces or augments
  `ComputePathToPose` with graph-constrained routing through a predefined
  network of nodes and edges
  ([nav2_route README](https://github.com/ros-navigation/navigation2/tree/main/nav2_route);
  [Route Server](https://docs.nav2.org/configuration/packages/configuring-route-server.html)).
- Two action interfaces:
  - `ComputeRoute` — drop-in for `ComputePathToPose`, returns sparse
    node/edge route + dense `nav_msgs/Path`
    ([nav2_route README](https://github.com/ros-navigation/navigation2/tree/main/nav2_route)).
  - `ComputeAndTrackRoute` — computes the route **and** tracks robot
    progress, firing route-operation triggers (door open, speed limits) as
    the robot crosses edges
    ([nav2_route README](https://github.com/ros-navigation/navigation2/tree/main/nav2_route)).
- Ideal when navigation must follow a fixed road graph (campus, warehouse
  aisles) instead of free-space planning.

### GeoJSON schema

- **Nodes**: `id` + `coordinates: [x, y]` required; `frame` recommended (the
  TF frame of the coordinates — often `map`)
  ([nav2_route README](https://github.com/ros-navigation/navigation2/tree/main/nav2_route)).
- **Edges**: `id`, `startid`, `endid` required; `cost` (fixed) and
  `overridable` (boolean: may scorers modify cost?) recommended
  ([nav2_route README](https://github.com/ros-navigation/navigation2/tree/main/nav2_route)).
- Both nodes and edges support a `metadata` blob for arbitrary
  application-specific keys (`speed_limit`, `class`, `penalty`, `door_id`)
  consumed by scorer / operation plugins
  ([nav2_route README](https://github.com/ros-navigation/navigation2/tree/main/nav2_route)).

### Edge cost expressions

Plugins compose multiplicatively/additively per the server's
`edge_cost_function`:

| Plugin | What it does |
|---|---|
| `DistanceScorer` | L2 distance, optionally scaled by `speed_limit %` metadata |
| `TimeScorer` | seconds = length / `abs_speed_limit` (or recorded `abs_time_taken`) |
| `PenaltyScorer` | static penalty from `metadata.penalty` |
| `SemanticScorer` | per-`class` weight (`grass: 5.0`, `road: 1.0`) |
| `CostmapScorer` | live costmap sample along edge — invalidates blocked edges |
| `DynamicEdgesScorer` | external service marks edges open/closed at runtime |

Citations: [nav2_route README](https://github.com/ros-navigation/navigation2/tree/main/nav2_route),
[Route Server](https://docs.nav2.org/configuration/packages/configuring-route-server.html).

### Why `route_server`'s `global_frame` matters

- Defaults to `map` ([Route Server](https://docs.nav2.org/configuration/packages/configuring-route-server.html)).
- Internally `getRobotPose()` calls `tf_buffer->lookupTransform("", "map")`
  if the parameter is empty / missing — this throws "Failed to transform from
  '' to map" and the server fails silently. Always set it explicitly.
- All graph nodes are transformed into `route_frame` at load time. Nodes can
  be authored in different `frame`s (e.g., `utm`, `gps`) and will be
  reprojected if TF resolves the chain
  ([nav2_route README](https://github.com/ros-navigation/navigation2/tree/main/nav2_route)).

### How `ComputeRoute` differs from `ComputePathToPose`

- `ComputePathToPose` plans free-space from current pose to goal pose,
  ignoring any topology.
- `ComputeRoute` snaps both ends to the nearest graph nodes and runs
  Dijkstra/A* on the graph, returning a sparse route. Path is then
  interpolated at `path_density` (default 0.05 m)
  ([Route Server](https://docs.nav2.org/configuration/packages/configuring-route-server.html);
  [nav2_route README](https://github.com/ros-navigation/navigation2/tree/main/nav2_route)).
- Useful when the planner's free-space output would cut across closed areas
  (lawns, no-go zones) that the graph excludes by construction.

### Sub-graph operators

- **CollisionMonitor** route operation — forward-checks edges against the
  global costmap, marks them invalid, triggers reroute
  ([nav2_route README](https://github.com/ros-navigation/navigation2/tree/main/nav2_route)).
- **ReroutingService** — external service can request a fresh route at any
  time.
- **AdjustSpeedLimit** — publishes a controller speed override on entering
  edges with a `speed_limit` metadata value.
- **TriggerEvent** — calls an arbitrary ROS service when an edge / node is
  entered (open door, ring elevator, etc.)
  ([nav2_route README](https://github.com/ros-navigation/navigation2/tree/main/nav2_route)).

---

## 8. Common Nav2 Anti-Patterns

- **Wrong `frame_id`s on sensor messages.** A pointcloud arriving as
  `frame_id: ""` or `velodyne` instead of the URDF-published `velodyne_link`
  silently fails costmap raycasts.
- **Missing `transient_local` on `/map` (or any latched topic).** Nav2's
  Static Layer subscribes with `transient_local` durability — if the map
  publisher is `volatile`, late-joining costmaps never receive the map.
  Same applies to `vision_msgs/LabelInfo` for the kiwicampus semantic layer.
- **Costmap topic mismatches.** `nav2_route`'s `CostmapScorer`,
  `collision_monitor`, and the `controller_server` may each subscribe to a
  different costmap topic — always cross-check the YAML.
- **Regulated Pure Pursuit without acceleration limits / Velocity Smoother.**
  RPP outputs raw cmd_vel; if the chassis can't follow step changes, you get
  oscillation. Add a `nav2_velocity_smoother` between the controller and the
  base controller, with `max_accel` and `max_decel` matching the platform's
  real limits ([Velocity Smoother](https://docs.nav2.org/configuration/packages/configuring-velocity-smoother.html)).
- **Forgetting `collision_monitor` for high-speed platforms.** The costmap
  is updated at ~5 Hz; at >1 m/s, that's 200 mm of travel between updates.
  `collision_monitor` runs at sensor rate (>20 Hz) and bypasses the costmap
  for emergency stops / slowdowns
  ([Collision Monitor](https://docs.nav2.org/configuration/packages/configuring-collision-monitor.html)).
- **Missing parameter overrides for sim vs real.** Sim usually has perfect
  odometry and zero sensor noise; real platforms need higher
  `transform_tolerance`, larger `sensor_timeout`, looser goal tolerances.
  Use ROS 2 launch composition to layer a `_sim.yaml` over `_base.yaml`.
- **Two TF publishers fighting for the same edge.** E.g., a static
  `map -> odom` from a launch file plus a localizer also publishing
  `map -> odom`. Symptom: TF_REPEATED_DATA warnings, jittery RViz.
- **Hard-coded goal tolerances tighter than the controller can hit.** A
  0.05 m `xy_goal_tolerance` on a 0.8 m radius vehicle in 0.05 m
  costmap-resolution causes oscillation and `failure_tolerance` triggers.
- **Mixing `RMW_IMPLEMENTATION` between launch and CLI.** Nav2 launch
  setting `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` while the user shell
  defaults to FastDDS leads to corrupted action goals (poses arrive zeroed)
  and cross-RMW interop bugs.
- **`controller_frequency` too low for the chassis.** RPP at 5 Hz on a
  1.5 m/s platform produces 30 cm of error per tick — set 20-30 Hz minimum.
- **Voxel layer with `min_obstacle_height: 0.0`.** Marks the ground plane as
  obstacle. Set above your floor's typical noise floor (e.g., 0.1 m).
- **Single EKF fusing GPS plus wheel odometry.** Discussed in section 2 —
  GPS jumps corrupt `odom -> base_link` continuity and break the controller.

---

## Quick Checklist for Reviewers

Boolean items — answer Y/N for each when reviewing a Nav2 stack.

1. TF chain matches REP-105: `map -> odom -> base_link` published by exactly
   one source per edge; no direct `map -> base_link`.
2. `base_footprint` (if present) is a static URDF child, not a dynamic
   localizer output.
3. IMU data published in ENU; non-ENU IMUs have `yaw_offset` set.
4. Two EKFs: `ekf_filter_node_odom` (`world_frame=odom`) for continuous
   sources only, `ekf_filter_node_map` (`world_frame=map`) including GPS.
5. Exactly one `publish_tf=true` per TF edge across all localizers.
6. EKF `*_config` matrices fuse velocities (not absolute pose) from wheel
   odometry; orientation from a single source with `_differential=false`.
7. EKF covariances are non-zero and reflect actual sensor noise.
8. `navsat_transform_node` either has `wait_for_datum: true` with a fixed
   `datum`, **or** the system tolerates a moving map origin between runs.
9. `magnetic_declination_radians` is set to the local declination (not 0).
10. Planner choice matches kinematics: SmacPlannerHybrid only for
    Ackermann/large-radius vehicles, with `minimum_turning_radius` matching
    the real platform.
11. RPP `lookahead_dist` (or min/max range) matches typical speed; velocity
    and cost regulation enabled for safety.
12. Goal tolerances (`xy_goal_tolerance`, `yaw_goal_tolerance`) are larger
    than `controller_frequency`-determined steady-state error.
13. Costmap `plugins:` ordered StaticLayer -> Obstacle/VoxelLayer -> custom
    layers -> InflationLayer (Inflation always last).
14. `raytrace_max_range > obstacle_max_range` for every observation source.
15. Either `footprint` (polygon) or `robot_radius` is set, not both.
16. Local costmap `rolling_window: true`; global costmap window sized for
    the operating area.
17. Each observation source has an `expected_update_rate` near sensor rate.
18. `voxel_layer` has `publish_voxel_map: true` enabled at least during
    bring-up for debugging.
19. `vision_msgs/LabelInfo` (and other latched costmap inputs) published
    with `transient_local` + `reliable` QoS.
20. BT either is the standard recovery BT, or the custom BT has a
    documented reason for omitting Spin/BackUp/Wait, with mitigations.
21. `route_server.global_frame` set explicitly (not empty/missing).
22. `nav2_route` GeoJSON has `frame: map` on every node, matches
    `route_frame` config.
23. Velocity Smoother (or equivalent) sits between controller and base,
    enforcing platform `max_accel` / `max_decel`.
24. Collision Monitor active for any platform exceeding ~1 m/s.
25. `RMW_IMPLEMENTATION` consistent across launch files, BT navigator, and
    user shell.
