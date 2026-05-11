# avros_sim — Review

## Summary

`avros_sim` is a small (~535 LOC), well-shaped Webots simulation package: webots_ros2_driver plugin, campus `.wbt`, device-mapping URDF, Nav2 override delta, three launches, one diagnostic script. Structure is correct (ament_python layout, sim time, RewrittenYaml two-stage override, cross-distro config selection), and topic names match real — the most important parity goal.

But four P0 defects undermine sim's ability to validate the real stack: (1) Ackermann sim vs diff-drive real (different control problems); (2) camera frames stamped on sim images don't exist in sim TF (image-to-base_link fails); (3) both EKFs load the same `ekf.yaml`, so the dual-EKF pattern collapses (likely a stack-wide defect, not just sim's); (4) `worlds/run_osm_import.sh` hardcodes the deprecated `~/AVROS/` path. P1 issues fill in the picture: no wheel odometry, no cmd_vel deadman, no slew-rate limit, no ZED stand-in for the kiwicampus semantic layer, sim disables collision detection rather than fixing `min_obstacle_height`, GPS datum mismatch (~78 m). P2 issues are mostly five-minute fixes.

With one month to IGVC AutoNav, the priority is to (a) fix the EKF dual-config and the camera-frame TF gap so sim doesn't lie, and (b) decide explicitly whether sim is for trajectory-shape validation or full controller/EKF tuning. Right now it tries to do both and does neither well.

## Per-file findings

### avros_vehicle_driver.py

`src/avros_sim/avros_sim/avros_vehicle_driver.py:1-145`. A `webots_ros2_driver` plugin class (loaded by Webots's controller via `<plugin>` in `avros_webots.urdf:13`), not a standalone Node — `init()` runs once, `step()` every tick. `rclpy.spin_once(timeout_sec=0)` in `step()` (line 90) is the correct idiom.

**Hardcoded constants (lines 23-26):** `WHEELBASE = 1.23`, `TRACK_FRONT = 0.9`, `MAX_STEERING_RAD = 0.489`, `WHEEL_RADIUS = 0.36`. The wheelbase and track values are copied from the URDF chassis bounding-box (`chassis_length`/`chassis_width` in `avros.urdf.xacro:5-6`), but those are envelope dimensions, not axle distances. More importantly: the bicycle-model formula at line 79 implements **Ackermann** kinematics, while the real platform is a **diff-drive tracked chassis** (CLAUDE.md "Diff-Drive Parameters"). Sim is solving a different control problem than real. (P0 — see parity section.) None of these are declared parameters; they should be (P2).

**cmd_vel callback (lines 72-87):** Bicycle-model inversion is correct for Ackermann. The `elif abs(omega) > 0.01` branch hard-applies max steering when `v ~ 0` — fine in sim but unrealistic for hardware (P2). Subscriber depth=1 default-reliable is acceptable for command topics.

**IMU publisher (lines 67, 111-145):** Combines Webots InertialUnit + Gyro + Accel into one `sensor_msgs/Imu` on `/imu/data` — topic matches CLAUDE.md table, `frame_id='imu_link'` matches the URDF. Quaternion `[x,y,z,w]` ordering is correct (avoids the standards-doc anti-pattern). Default reliable depth-10 QoS instead of `qos_profile_sensor_data` is a minor mismatch with real-side QoS (P1). Covariances hardcoded at 0.01/0.01/0.1 (P2). Sim time comes from `self.__node.get_clock().now()` with `use_sim_time=True` — correct.

**Lifecycle / shutdown (P1):** `rclpy.init(args=None)` is called unguarded at line 57; on Webots controller restart it raises `RCLError`. Should be `if not rclpy.ok(): rclpy.init()`. No `destroy_node()` / `rclpy.shutdown()` anywhere — leaks DDS entities on teardown (standards doc anti-pattern).

**Missing:** No cmd_vel timeout / deadman (real `actuator_node` has 500 ms freshness gate). No wheel-odometry publisher. No slew-rate limit / heading-hold equivalent. (All P1 — see parity section.)

**Style nits (P2):** `__double_underscore` name mangling everywhere is hostile to subclassing the plugin. `from math import atan2` is fine but `import math` is more common. `rclpy.parameter` could be `from rclpy.parameter import Parameter`.

### launch/*.launch.py

**`sim.launch.py` (1-73)** — `WebotsLauncher(world=..., ros2_supervisor=True)` + `WebotsController(robot_name='avros', parameters=[{'robot_description': <urdf-path>}, ...])` is the correct webots_ros2_driver pattern. The `webots._supervisor` private-attribute access (line 58) is fragile across driver versions (P2). `robot_state_publisher` inflates the real `avros.urdf.xacro` via `Command(['xacro ', urdf_file])`, which `<xacro:include>`s `zed_wrapper/urdf/zed_macro.urdf.xacro` — but `zed_wrapper` is not in `package.xml` exec_depends, so xacro fails at launch on a clean install (P1). The full ZED-front/left/right macro tree gets published in sim TF as dead frames with no devices — wasteful, not broken. Shutdown event handler (lines 63-72) emits `Shutdown` when Webots exits, clean. Missing: any `DeclareLaunchArgument` (world, robot_name) (P2); `SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')` (P1, per CLAUDE.md known issue).

**`sim_navigation.launch.py` (1-179)** — ROS_DISTRO branch (lines 33-40) picks `nav2_params_humble.yaml` + matching BT XML (both exist, verified). `RewrittenYaml` two-stage merge (`configured_params`, `sim_override_params`) at line 94 — rightmost wins per key, documented Nav2 behavior. Lifecycle order list (lines 56-64) preserves activation order, route_server before bt_navigator — correct. LiDAR remap `('/velodyne_points', '/velodyne_points/point_cloud')` at line 83 papers over webots_ros2_driver's auto-suffix; relies on Nav2 costmap layers inheriting parent-node remappings (works in practice but fragile, P1).

**Critical defect (P0):** Both EKFs load the same `ekf.yaml` (lines 114, 130). Standards Section 2 dual-EKF pattern requires distinct configs — `world_frame: odom` on the local EKF, `world_frame: map` on the global. Single config means either both publish `odom→base_link` (TF_REPEATED_DATA) or neither sets up the dual-EKF chain. Same defect may exist in `avros_bringup`'s `localization.launch.py`; needs cross-package check.

`navsat_transform_node` (140-156) — standard remap set, fuses against `/odometry/global`. `TimerAction(period=20.0)` (line 163) for lifecycle manager bring-up — long fixed delay; standards doc prefers `OnProcessStart` (P1). Sim has no wheel-odometry source, so the odom-EKF runs IMU-only — double-integrated drift (P1).

**`sim_teleop.launch.py` (1-42)** — includes `sim.launch.py`, adds `teleop_twist_keyboard` with `prefix='xterm -e'` (P2: undeclared system dep on `xterm`). Hardcoded `speed: 2.0`, `turn: 0.5`. No LaunchArguments. Minimal, fine.

### config/nav2_sim_overrides.yaml

`src/avros_sim/config/nav2_sim_overrides.yaml:1-23`. Minimal override file — good approach (delta over base, not a fork). Sets relaxed progress checker, `desired_linear_vel: 1.0`, drops camera_depth from voxel_layer observation_sources.

Two P1 issues:
1. **`use_collision_detection: false`** (line 14) — disables RPP forward collision-checking entirely. Comment cites "VelodynePuck sees ground as obstacles," which is actually a `min_obstacle_height` problem. Right fix: set `min_obstacle_height` on the voxel layer, restore collision detection.
2. **Local costmap is global-sized** — `width: 100, height: 100, resolution: 0.5` (lines 19-21). Real is 10×10 m per CLAUDE.md. The controller will plan around obstacles 50 m away — changes Nav2 behavior substantially.

Missing: no `min_obstacle_height` override (the actual fix for issue 1); no `route_server.global_frame: "map"` (CLAUDE.md known-issue, but probably set in base config).

### resource/avros_webots.urdf

`src/avros_sim/resource/avros_webots.urdf:1-54`. The webots_ros2_driver device-mapping file (not the kinematic URDF — that stays in `avros_bringup/urdf/avros.urdf.xacro`).

Mappings: `<plugin>` loads the driver class (line 13); velodyne → `/velodyne_points` with `frameName=velodyne` (matches `avros.urdf.xacro:77`); camera_color → `/camera/camera/color/image_raw` with `camera_color_optical_frame`; camera_depth → `/camera/camera/aligned_depth_to_color/image_raw` with `camera_depth_optical_frame`; gps → `/gnss` with `frameName=imu_link`.

**P0 issue:** `camera_color_optical_frame` and `camera_depth_optical_frame` (lines 31, 40) are **not** static URDF links in `avros.urdf.xacro` — on real hardware the realsense2_camera driver publishes them at runtime. In sim that driver isn't running, so the frames never appear in TF. Any consumer transforming a sim camera image into `base_link` fails with `LookupException`. Fix in parity section.

GPS `frameName=imu_link` (line 49) is loose — should be a GPS-antenna frame — but `xsens.yaml` lever arm is `[0,0,0]` per CLAUDE.md, so sim ≈ real (both zero-lever) (P2).

**Missing:** No ZED mapping (real uses ZED front for `kiwicampus/semantic_segmentation_layer` — sim has no input for that layer, P1). No wheel-encoder mapping (P1).

### worlds/run_osm_import.sh

`src/avros_sim/worlds/run_osm_import.sh:1-7`. Five-line wrapper for the Webots OSM importer.

**P0 usability:** All paths hardcode `$HOME/AVROS/...` — the deprecated workspace path explicitly called out by CLAUDE.md ("the live Jetson workspace lives at `~/IGVC/`. ... `~/AVROS/` directory ... do **not** use it"). This script will fail on every machine.

P2 issues: no `set -euo pipefail`; hardcoded `$HOME/webots` install path (should default-substitute); output file `cpp_campus_osm.wbt` differs from the world used by `sim.launch.py` (`cpp_campus.wbt`) — a one-time import scaffold workflow that isn't documented.

### package.xml / setup.py

**`package.xml` (1-31):** Format 3, build_type ament_python — correct. License `MIT` — inconsistent with rest of repo (P2). `<depend>` used instead of `<exec_depend>` for `rclpy`/`geometry_msgs`/`webots_ros2_driver` (P2 — works but stylistically wrong for ament_python). Missing exec_depends for `tf2_ros`, `nav_msgs`, `sensor_msgs` actually imported by the driver and diagnose script (P2 — masked when those are pulled in transitively). The orchestration exec_depends (`avros_bringup`, `avros_navigation`, `robot_localization`, `nav2_bringup`, `teleop_twist_keyboard`, `xacro`) are correct for a sim metapackage. Test deps boilerplate present, but no `test/` directory exists (P2).

**`setup.py` (1-41):** Standard skeleton. `data_files` installs launch (`*launch.[pxy][yma]*` glob is unusual but works), worlds (`.wbt` + `.wbproj`, **misses `run_osm_import.sh`** — fine, it's a developer script), resource URDF, config YAML. `entry_points.console_scripts` empty — **correct** (the driver loads via `<plugin>`, not `ros2 run`). No `setup.cfg` — fine when entry_points is empty.

### scripts/diagnose_sim.py (out-of-tree)

`scripts/diagnose_sim.py:1-169`. Lives at repo root, not inside the package. Subscribes to `/imu/data`, `/gnss`, `/odometry/filtered`, `/odometry/global`, `/odometry/gps`, `/cmd_vel` and TF `map→base_link`, prints a structured report every 2 s.

Topic names match `sim_navigation.launch.py` (verified). Node structure matches standards doc Section 2 (subclass Node, callbacks, timer, `try/finally` in main). BEST_EFFORT depth 1 QoS — correct for sensor streams, downgrades cleanly from default-reliable publishers.

P1: docstring (line 13) claims `ros2 run avros_sim diagnose_sim` works; it doesn't (entry_points empty, file outside the package). Either add the entry point or fix the docstring.

P2: bare `except Exception` (line 135) for the tf2 lookup — should catch the LookupException/ConnectivityException/ExtrapolationException triple. Hardcoded "EXPECTED" spawn values (147-150) couple the tool to a specific world layout.

Verdict: useful diagnostic, works as documented when run via `python3 scripts/diagnose_sim.py`.

## Sim-vs-real parity analysis

The point of `avros_sim` is to validate the real Nav2 + EKF + perception stack pre-competition. Parity defects make sim look green while real-world configuration is broken. Inventory:

**1. Kinematics (P0).** Real is a track diff-drive (CLAUDE.md "Diff-Drive Parameters", track gauge 0.7366 m). Sim is an Ackermann car (Webots `Car.proto`, `avros_vehicle_driver.py:36-41` — front-wheel-drive bicycle inversion at line 79). Same `/cmd_vel` input, fundamentally different kinematics. Nav2 controller tuning won't transfer. The repo's choice of `SmacPlannerHybrid (DUBIN, min_radius 2.31 m)` (CLAUDE.md "Nav2 Config") is right for the sim but wrong for real — should be `SmacPlanner2D`/`NavfnPlanner` for a circular diff-drive.

**2. Wheelbase value (P1).** `WHEELBASE = 1.23` (`avros_vehicle_driver.py:23`) is the URDF chassis-length, not an axle-to-axle distance. Decoupled from the Webots Car PROTO geometry too.

**3. Frame names — mostly OK.** `base_link`, `base_footprint`, `imu_link`, `velodyne` all match across sim and real (sim launches the same `avros.urdf.xacro` via `robot_state_publisher`). **Camera frames break (P0):** `camera_color_optical_frame` and `camera_depth_optical_frame` (`avros_webots.urdf:31, 40`) are not in the static URDF — on real hardware the realsense driver provides them at runtime, but in sim that driver isn't running. Image-to-base_link transforms fail. Fix: add static joints in `avros.urdf.xacro` gated behind a sim arg, or a sim-side `static_transform_publisher`. ZED frames exist in TF (URDF macro published) but have no Webots devices stamping into them — dead frames, not broken.

**4. GPS datum (P1).** Webots `gpsReference 34.059 -117.823` (cpp_campus.wbt:25) vs real `navsat.yaml` datum `34.059270, -117.820934` (CLAUDE.md). ~78 m offset at this latitude. If `wait_for_datum: true` (CLAUDE.md says fixed), the route graph appears 78 m off from the sim vehicle.

**5. Wheel odometry (P1).** Real EKF fuses `/wheel_odom` from Teensy E-line. Sim publishes no wheel odometry — odom-EKF runs IMU-only, drift on the order of meters per 30 seconds.

**6. cmd_vel deadman (P1).** Real `actuator_node` enforces 500 ms freshness (CLAUDE.md). Sim driver has no timeout — last command persists. Failure-mode behavior diverges.

**7. ZED / perception (P1).** Real uses ZED X front + `kiwicampus/semantic_segmentation_layer` for road/grass/barrier costs. Sim has no ZED topic — semantic layer silently disabled. Costmap behavior diverges where it's hardest to validate manually.

**8. Voxel ground-noise workaround (P1).** Sim disables RPP `use_collision_detection` (`nav2_sim_overrides.yaml:14`) instead of fixing `min_obstacle_height`. Sim never exercises the real collision-monitor code path.

**9. No slew-rate / heading-hold in sim driver (P1).** Real `actuator_node` ramps over ~100 ms (max_linear_accel = 1.0 m/s²) and gyro-stabilizes turns. Sim applies cmd_vel directly. Controllers that converge in sim may oscillate on hardware.

## Cross-cutting issues

- **No tests at all** — `<test_depend>` entries in `package.xml` but no `test/` directory. Standards checklist item 9 fails.
- **No CI hook for sim.** Breakage from neighbor packages (ZED v5 changes, new `nav2_params.yaml` keys) surfaces only on manual launch.
- **No `DeclareLaunchArgument`s** anywhere. Standards checklist item 17 fails.
- **Documentation drift:** `diagnose_sim.py:13` claims `ros2 run avros_sim diagnose_sim` works; it doesn't.
- **Stale workspace path:** `run_osm_import.sh` references `~/AVROS/` (deprecated, CLAUDE.md).
- **Webots private API:** `webots._supervisor` (`sim.launch.py:58`) — fragile across driver versions.
- **Same-config dual-EKF:** see launch section. Likely a stack-wide problem (real `localization.launch.py` may have the same defect).

## Punch list (P0 / P1 / P2)

### P0 — sim/real divergence that breaks validation

1. **Ackermann sim vs diff-drive real** (`avros_vehicle_driver.py:23-87`). Either swap to a `Robot.proto` with two diff-drive wheels at `track_gauge=0.7366`, or scope sim to "trajectory-shape only."
2. **Camera frames missing in sim TF** (`avros_webots.urdf:31, 40`). Add static joints in `avros.urdf.xacro` for `camera_link → camera_color_optical_frame` (gated behind a sim arg), or a sim-side static_transform_publisher.
3. **Two EKFs share one `ekf.yaml`** (`sim_navigation.launch.py:114, 130`). Need separate `ekf_odom.yaml` / `ekf_map.yaml` per standards Section 2. Same defect likely exists in `avros_bringup` — check there.
4. **`run_osm_import.sh` references `~/AVROS/`** (deprecated path per CLAUDE.md). Update to `~/IGVC/` or compute relative to `$0`.

### P1 — silent gaps that make sim less useful than it appears

5. **Add wheel-odometry publisher.** Webots `PositionSensor` on the driven wheels → `/wheel_odom`.
6. **Align GPS datum.** Edit `cpp_campus.wbt:25` `gpsReference` to `34.059270 -117.820934 0`.
7. **Add `<exec_depend>zed_wrapper</exec_depend>` to `package.xml`** — sim.launch.py xacro-inflates the real URDF which `<xacro:include>`s the ZED macro.
8. **Add cmd_vel 500 ms deadman** to driver matching `actuator_node`.
9. **`SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')`** in sim launch files.
10. **Replace `use_collision_detection: false`** with voxel-layer `min_obstacle_height`.
11. **Shrink the local costmap** — current 100×100 m at 0.5 m is global-sized.
12. **ZED stand-in or document its absence.** Otherwise `kiwicampus/semantic_segmentation_layer` is silently dead in sim.
13. **Replace 20 s lifecycle TimerAction** with an `OnProcessStart` event handler.
14. **Fix `scripts/diagnose_sim.py` docstring** or add the entry_point.
15. **Guard `rclpy.init()`** with `if not rclpy.ok():` for Webots controller restarts.
16. **Add slew-rate limiter / heading-hold** to sim driver matching real `actuator_node`.

### P2 — style / hygiene

17. No `test/` directory (boilerplate ament linters).
18. No `DeclareLaunchArgument`s in any launch file.
19. Driver magic numbers → declared parameters.
20. `<depend>` should be `<exec_depend>` in `package.xml:10-12`.
21. Missing exec_depends for `tf2_ros`, `nav_msgs`, `sensor_msgs`.
22. `__double_underscore` name mangling in driver.
23. Undeclared `xterm` system dep in `sim_teleop.launch.py:39`.
24. `run_osm_import.sh`: hardcoded `$HOME/webots`, no `set -euo pipefail`.
25. Bare `except Exception` in `diagnose_sim.py:135`.
26. IMU publisher uses default QoS instead of `qos_profile_sensor_data`.
27. No `setup.cfg`.
28. MIT vs Apache-2.0 license inconsistency.
29. `os.path.join` instead of `PathJoinSubstitution`.

## Positives

- **Tiny, focused package** (~535 LOC) — easy to read end-to-end.
- **Topic names match real platform exactly** — `/velodyne_points`, `/imu/data`, `/gnss`, `/camera/camera/...`. The right primary parity goal, well executed.
- **Reuses real configs from `avros_bringup`** (`ekf.yaml`, `navsat.yaml`, `nav2_params.yaml`, route graph). Configs evolve once, sim follows.
- **`RewrittenYaml` two-stage merge** — base + minimal delta override (23 lines), not a full `nav2_params.yaml` fork.
- **Cross-distro launch logic** — `sim_navigation.launch.py:33-40` ROS_DISTRO branch. Forward-thinking.
- **Quaternion `[x,y,z,w]` order correct** in driver IMU publisher (`avros_vehicle_driver.py:122-125`).
- **Lifecycle bring-up order documented and correct** (route_server before bt_navigator).
- **Proper Webots integration pattern** — `WebotsLauncher` + `WebotsController`, not a hand-rolled `Node()`.
- **`use_sim_time: true` set everywhere** — driver, robot_state_publisher, EKFs, navsat, every Nav2 server, teleop, diagnose.
- **`diagnose_sim.py` is genuinely useful** — fixed-rate report covering all the streams a bring-up debugger needs.
- **Shutdown event handler in `sim.launch.py`** — no orphan Nav2/EKF/RViz processes when Webots dies.
- **Inline comments explain non-obvious choices** — lidar topic suffix, override semantics, lifecycle-delay rationale. Future-team friendly.
