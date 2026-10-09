# avros_bringup — Review

## Summary

`avros_bringup` is a pure `ament_python` bringup package containing 9 launch files, 16 YAML/XML config files, 1 URDF/xacro, and a 16,651-node GeoJSON route graph. It is the keystone for the entire IGVC AutoNav autonomous stack — every sensor driver, EKF, Nav2 server, perception pipeline, and actuator bridge is composed from this package's launches and configs.

**Overall grade: B+.** The package shows clear engineering maturity (excellent docstrings, dual-EKF pattern done correctly, ZED wrapper gotchas avoided, deliberate BT recovery choices for IGVC) but has a meaningful set of P0 issues that need immediate attention before competition:

- **P0 SECURITY**: NTRIP credentials are committed to git in `config/ntrip_params.yaml`. If this repo is public on GitHub, the password is leaked.
- **P0 SAFETY**: No `nav2_collision_monitor` configured for a >1 m/s vehicle.
- **P0 SAFETY**: Humble local costmap has no `voxel_layer` — Velodyne data does not influence local-costmap obstacle avoidance.
- **P0 RUNTIME**: `RMW_IMPLEMENTATION` env var only set in `sensors.launch.py`; every other launch (including `navigation.launch.py`) leaks default RMW into Nav2 nodes — exact bug already documented in CLAUDE.md.
- **P0 ROSDEP**: `package.xml` is missing several `<exec_depend>` entries (`nav2_route`, `kiwicampus_semantic_segmentation_layer`, `avros_perception`, `zed_wrapper`, `xsens_mti_ros2_driver`, etc.).
- **P0 DRIFT**: `nav2_params.yaml` (Jazzy) and `nav2_params_humble.yaml` have substantial undocumented drift beyond the header-noted differences.
- **P0 MISMEASURE**: `cpp_campus_graph.geojson` has 16,651 nodes / 17,492 edges — not the 52/113 documented in CLAUDE.md. Plan-time and disk impact unverified.
- **P0 MISMEASURE**: Every sensor mount in URDF is sketch-derived ("TODO: measure on real vehicle"). GNSS lever arm is `[0,0,0]`.
- **P0 BROKEN CONFIG**: `zed_back.yaml` uses v4 wrapper syntax + invalid v4 enums; will throw `InvalidParameterValueException` if ever enabled.

The 27-item P1 list is mostly polish, doc-fix, and DRY refactoring. P2 items are pure quality-of-life. Once P0s are addressed, this package will be in solid shape for IGVC.

**Standards-rubric score** (per `standards_ros2_python.md` quick checklist, 18 items): roughly 14/18 — passes the threshold. Item failures: missing exec_depends (#7), no real unit tests beyond linters (#9 partially), some env-var inconsistency (#17 partial). Per `standards_nav2_localization.md` 25-item checklist: roughly 18/25 — REP-105 frame chain correct, dual EKF correct, navsat datum correct, BT recovery choice deliberate; misses are no collision_monitor, no voxel layer on Humble local, missing transient-local on label_info subscribers (out of scope), `magnetic_declination_radians: 0.0` assumption needs verification.

**Recommendation:** address every P0 in the punch list before the IGVC competition; tackle high-value P1s (DRY refactor, env-var fix, sensor measurements, missing collision_monitor) in the next 1-2 weeks; defer P2s.

## Per-file findings

### package.xml / setup.py

**`package.xml`** (`src/avros_bringup/package.xml`):

- Format 3, `ament_python` build type — correct (line 3, 33). Matches REP-149.
- License is `MIT` (line 8). The standards reference notes `Apache-2.0` is the de-facto default for ROS 2 demos, but MIT is fine if a matching `LICENSE` file exists at repo root. **P2:** confirm `LICENSE` file is present at repo root with MIT text — `<license>` field alone is not enough for `ament_copyright`.
- Maintainer email `avlab@cpp.edu` (line 7) — institutional, fine.
- **P1 — `<depend>` for runtime-only deps**: lines 10-14 use `<depend>` for `rclpy`, `launch`, `launch_ros`, `robot_state_publisher`, `xacro`. Per the standards rubric (`standards_ros2_python.md` §1, dependency tag table), `ament_python` packages should prefer `<exec_depend>` for runtime deps and reserve `<depend>` for things you genuinely build against. `rclpy`/`launch`/`launch_ros` are not used by anything in this package's Python code (this is a launch-only package — no `rclpy.Node` here), so they should be `<exec_depend>`. `robot_state_publisher` and `xacro` are also runtime-only.
- `<exec_depend>` block (lines 16-25) is reasonably comprehensive but **incomplete** for what the launch files actually need:
  - **P0 — Missing `<exec_depend>nav2_route</exec_depend>`**: `navigation.launch.py` runs the route_server node (`navigation.launch.py` referenced in CLAUDE.md) and `nav2_params.yaml` configures `route_server`. Without an `<exec_depend>`, `rosdep install --from-paths src` won't pull `nav2_route`.
  - **P0 — Missing `<exec_depend>kiwicampus_semantic_segmentation_layer</exec_depend>` (or whatever the package name is)**: nav2_params.yaml references the plugin per CLAUDE.md.
  - **P1 — Missing `<exec_depend>avros_perception</exec_depend>`**: `navigation.launch.py` includes the perception node per CLAUDE.md launch table.
  - **P1 — Missing `<exec_depend>zed_wrapper</exec_depend>`**: `sensors.launch.py` includes `zed_camera.launch.py` per CLAUDE.md.
  - **P1 — Missing `<exec_depend>xsens_mti_ros2_driver</exec_depend>`**: same — sensors.launch.py launches it.
  - **P1 — Missing `<exec_depend>tf2_ros</exec_depend>`**: any `static_transform_publisher` invocation.
  - **P1 — Missing `<exec_depend>nav2_lifecycle_manager</exec_depend>`**: required for Nav2 servers; bringup typically uses it.
  - **P2 — Missing `<exec_depend>rviz2</exec_depend>`** if launch files invoke rviz.
- **P2 — Empty `<author>` and missing `<url>`**: package.xml could include `<url type="repository">https://github.com/Paarseus/IGVC_ROS2</url>` and an `<author>` line for traceability.
- **P2 — `<version>0.0.0</version>` (line 5)**: never bumped. For a competition-bound codebase, bump to `0.1.0` once the bring-up stack is stable.

**`setup.py`** (`src/avros_bringup/setup.py`):

- Standard scaffold, `find_packages(exclude=['test'])` — fine (line 11).
- **P1 — Glob patterns are loose**:
  - Line 17: `glob('launch/*launch.[pxy][yma]*')` — this exotic glob tries to match `launch.py`/`launch.xml`/`launch.yaml` in one shot. It works (matches `*.launch.py`), but the canonical idiom from the standards reference (and `lifecycle_py` upstream) is `glob('launch/*.launch.py')`. The current pattern would also match a typo-named file like `foolaunch.pyma` and copy it — minor footgun. Tighten to `glob('launch/*.launch.py')`.
  - Line 19: `glob('config/*')` — too broad. This will install editor swap files (`.swp`), backups (`*.bak`), and any subdirectories. Restrict per file type: `glob('config/*.yaml') + glob('config/*.xml')`.
  - Line 21: `glob('urdf/*')` — same issue. Should be `glob('urdf/*.xacro') + glob('urdf/*.urdf')`.
  - Line 23: `glob('rviz/*')` — should be `glob('rviz/*.rviz')`.
- **P1 — No GeoJSON glob**: per CLAUDE.md the route graph is `cpp_campus_graph.geojson` and lives in `config/`. The current `glob('config/*')` will catch it (one of the few cases where the broad glob saves us), but the proper fix is to add an explicit `glob('config/*.geojson')` line so the install rules document the intent.
- **P2 — `entry_points['console_scripts'][]` is empty (lines 37-38)**: correct — this package has no Python entry points, only data files. Standard for a pure-bringup package.
- **P2 — `setup.cfg` not seen**: need to check whether `setup.cfg` exists with the `[develop]`/`[install]` `script_dir` lines required by ament_python (per `standards_ros2_python.md` §1, item 5). Even with no entry points, `setup.cfg` is conventionally present.
- **P2 — `resource/avros_bringup` not verified**: ament marker file required by index — should be present. Easy to overlook.

**`<test_depend>` declarations** (package.xml lines 27-30): `ament_copyright`, `ament_flake8`, `ament_pep257`, `python3-pytest` — correct boilerplate. The actual tests will be reviewed in GROUP 4.

### launch/sensors.launch.py

The sensor bring-up file. 218 lines, structured well, with a Python docstring header (lines 1-9). Conditional `IfCondition` gating per sensor.

**Positives:**

- Module docstring (lines 1-9) explains what the file launches and the role of NTRIP.
- `SetEnvironmentVariable` for both `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (line 41-44) and `CYCLONEDDS_URI` (line 45-48). This matches the explicit recommendation in `standards_ros2_python.md` §4 and the issue logged in CLAUDE.md ("CLI commands get (0,0) goals / RMW mismatch").
- Every optional sensor is gated by an `IfCondition(LaunchConfiguration('enable_*'))` with sensible defaults (lines 50-83). Velodyne/realsense default `true`, ZED defaults `false`.
- `robot_state_publisher` correctly invokes `xacro` via `Command(['xacro ', urdf_file])` and wraps in `ParameterValue(..., value_type=str)` (lines 89-94). Standard pattern.
- ZED includes pass only `camera_name`/`camera_model`/`serial_number` and **explicitly do not pass `namespace` or `node_name`** — comment at lines 128-136 cites the wrapper-overwrite gotcha. This is the single most-likely-to-bite bug avoided. `publish_tf: false` and `publish_urdf: false` (lines 150-151) correctly delegate TF to `robot_localization` and URDF to our `robot_state_publisher`.
- ZED `ros_params_override_path` (line 152) is the right way to layer user config over the wrapper's `common_stereo.yaml`/`zedx.yaml` defaults.
- NTRIP `remappings` at lines 210-213 correctly bridge default node-relative names (`nmea`, `rtcm`) to global topics.

**Issues:**

- **P1 — Missing log/output uniformity**: `velodyne_transform_node` (line 110) is the upstream package's executable name. CLAUDE.md says the executable is `velodyne_convert_node` (CLAUDE.md lines `Nodes:` for Velodyne). **Verify** — `ros-humble-velodyne` apt package ships *both* `velodyne_transform_node` and `velodyne_convert_node` historically, but newer Humble ships `velodyne_transform_node`. Either is fine; just flag the mismatch with CLAUDE.md prose so docs stay synced.
- **P1 — `enable_xsens` argument missing**: every other sensor has an `enable_*` toggle but the Xsens node (lines 195-201) is unconditional. For bench testing without the IMU plugged in, you can't disable it. Add `DeclareLaunchArgument('enable_xsens', default_value='true', ...)` and gate with `IfCondition`.
- **P1 — ZED right serial conflict with CLAUDE.md**: line 184 hard-codes `serial_number: '49910017'` for `enable_zed_right`, but CLAUDE.md "Left / Right / Back ZED X (future)" section says serial 49910017 was repurposed to **front**. The code comment on line 183 acknowledges "TODO: confirm right-camera serial (was 42569280, now repurposed to front)" — but the actual serial assignment is now front=42569280 (line 149) and right=49910017 (line 184). One of these is wrong. Reading sensors.launch.py more carefully: line 145-149 says "GMSL port 0 -> SN 42569280 (physical front)" so that part is consistent. Line 184 may be old. **Action: cross-check Stereolabs ZED_Explorer output against this file before competition.**
- **P1 — IncludeLaunchDescription path concatenation via list with leading slash**: lines 138-141 build the path as `[get_package_share_directory('zed_wrapper'), '/launch/zed_camera.launch.py']` — this works because `PythonLaunchDescriptionSource` happens to accept a list-of-strings concatenation. The standards reference (`standards_ros2_python.md` §4) recommends `PathJoinSubstitution([FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'])` — robust to symlink-install, more idiomatic, and avoids the leading-slash-on-second-list-element pattern. Same concern at lines 159-162, 175-178, 137-141 of `navigation.launch.py`, and similar in the test launches.
- **P2 — No `output='screen'` on every Node**: most do, but the include actions inherit defaults — fine for now.
- **P2 — Comment at line 40 ("CycloneDDS shared memory") is mis-leading**: shared memory is *disabled* per CLAUDE.md and `cyclonedds.xml`, but the comment header says "CycloneDDS shared memory" without the "disabled" qualifier. Reword to "CycloneDDS configuration (shared mem disabled — see cyclonedds.xml)".

### launch/navigation.launch.py

The full autonomous stack. 213 lines. Includes localization, perception (conditional), actuator, all Nav2 servers + lifecycle manager + foxglove bridge.

**Positives:**

- Module docstring (lines 1-17) is excellent: lists what is launched, explains why nav2 servers are launched directly rather than through `nav2_bringup` (lifecycle manager ordering for route_server), and notes the cross-distro path selection.
- **Distro-specific config switching** (lines 36-43): reads `os.environ.get('ROS_DISTRO', 'humble')` and selects `nav2_params_humble.yaml` + `navigate_route_graph_humble.xml` for Humble vs the non-`_humble` files for newer distros. This is a thoughtful pattern, but flag as **P2** that the variable defaults to `'humble'` so non-Humble systems must explicitly set `ROS_DISTRO`. (Most ROS environments do.)
- `RewrittenYaml` (lines 53-62) substitutes `default_nav_to_pose_bt_xml`, `default_nav_through_poses_bt_xml`, `graph_filepath`, and `use_sim_time` into the loaded params at launch time — clean. The BT path is `[pkg_dir, '/config/', bt_xml_filename]`, so users supply just the filename via `bt_xml:=foo.xml`. Also `convert_types=True` ensures YAML coercion happens.
- **Lifecycle manager owns ordered list** (lines 66-76, 190-199): `nav2_servers` tuple list goes controller → smoother → planner → route_server → behavior → velocity_smoother → bt_navigator. The order is meaningful: route_server activates before bt_navigator validates the BT XML (which references `ComputeRoute`). This matches the comment in the docstring and is the right thing to do.
- `respawn=True, respawn_delay=2.0` on every Nav2 server (lines 85-86) — robustness if a node crashes.
- **`bt_xml` launch arg** (lines 50-51, 97-103) lets users swap between graph-based BT and `navigate_to_pose_simple_humble.xml` for free-space planning. Useful for IGVC: graph for the road network portion, free-space for the autonav arena.
- Foxglove bridge included (lines 202-211) — sensible default for remote viz on a competition robot.

**Issues:**

- **P0 — `RMW_IMPLEMENTATION` and `CYCLONEDDS_URI` are NOT set in this launch**: `sensors.launch.py` sets them, but `navigation.launch.py` includes `localization.launch.py`, which in turn includes `sensors.launch.py`. Env vars set in a child `IncludeLaunchDescription` propagate to that child's launched processes only — they do **not** apply to nodes added later in the parent's `LaunchDescription` (the Nav2 servers, lifecycle manager, foxglove bridge, actuator_node). This means **the Nav2 servers run with whatever the user's shell defaults are** (FastDDS unless the user has the env var globally). Per CLAUDE.md "CLI commands get (0,0) goals / RMW mismatch", this is the exact bug already known. **Fix: re-set `RMW_IMPLEMENTATION`/`CYCLONEDDS_URI` at the top of this file too.**
- **P0 — `default_nav_to_pose_bt_xml` value type**: `RewrittenYaml` is fed `[pkg_dir, '/config/', bt_xml_filename]` (line 57). `bt_xml_filename` is a `LaunchConfiguration` substitution. `RewrittenYaml.param_rewrites` accepts substitutions but does not always concatenate list-of-mixed-types cleanly. Verify by `ros2 launch ... navigation.launch.py --show-args` and inspect the actually-rewritten YAML. If the concatenation produces a literal `'/config/'` middle segment alongside a `LaunchConfiguration` object that doesn't get .perform()'d, the BT path is broken. (This may already be tested; flag for verification.)
- **P1 — `actuator_node` placement**: lines 175-184. It's launched from `navigation.launch.py` but **not** from `localization.launch.py` (which it should arguably be, since teleop and webui both rely on it). Currently `actuator.launch.py`, `teleop.launch.py`, `webui.launch.py`, `navigation.launch.py` each instantiate a separate `actuator_node` with the same params — DRY violation. **Recommendation:** factor `actuator_node` into a reusable launch include (e.g., `actuator_only.launch.py` that's just the node + arg) and `IncludeLaunchDescription` it from each of the 4 sites.
- **P1 — Missing args propagation**: `navigation.launch.py` declares `enable_zed_left` and `enable_zed_right` (lines 125-133) but the `localization.launch.py` include only forwards them on line 156-157. Fine. However `localization.launch.py` itself launches Xsens unconditionally — same Xsens-disabling gap.
- **P1 — `enable_perception`** (lines 135-138, 162-172): default `false`. For competition we expect this to default `true` once perception is verified — flag as a configuration-readiness item.
- **P1 — Foxglove bridge has no `enable_foxglove` argument**: always-on. For competition where you may want to kill the WebSocket server to shed Jetson load, a toggle is cheap. Also no TLS/auth — anyone on the network can connect.
- **P2 — `nav2_servers` package names**: `'nav2_velocity_smoother'` (line 72) — verify the upstream package name. `nav2_velocity_smoother` is correct on Humble. Also `'nav2_route'` (line 70) — correct, but again must be in `<exec_depend>`.
- **P2 — `lifecycle_manager_navigation` is the only lifecycle manager**: this is correct for the main launch, but the test launches each declare their own (`lifecycle_manager_perception_test`, `lifecycle_manager_costmap`, etc.). If you ever launch navigation + a test launch in the same shell, you'd have two lifecycle managers fighting. Document the constraint.
- **P2 — `node_names: lifecycle_nodes`**: clean. Good.

### launch/localization.launch.py

Dual EKF + navsat_transform. 138 lines. Cleanly structured.

**Positives:**

- Module docstring (lines 1-15) cites the upstream `nav2_gps_waypoint_follower_demo/launch/dual_ekf_navsat.launch.py` reference. Excellent traceability for review.
- Two EKFs as required by `standards_nav2_localization.md` §2 ("dual-EKF pattern"):
  - `ekf_filter_node_odom` (lines 87-99) — local odometry EKF, output remapped to `/odometry/filtered`. Both EKFs read from the same `ekf_config` (single `ekf.yaml`); the differentiation must be inside the YAML via the `ekf_filter_node_odom`/`ekf_filter_node_map` ROS-namespace blocks. **Verify in ekf.yaml review (GROUP 2).**
  - `ekf_filter_node_map` (lines 104-116) — global EKF, output remapped to `/odometry/global`.
- `navsat_transform_node` (lines 120-136) consumes IMU + GNSS + global EKF and publishes `/odometry/gps` for the global EKF — exactly the upstream pattern. Topic remappings are correct: `gps/fix → /gnss` (the Xsens topic name), `imu/data → /imu/data`, `odometry/filtered → /odometry/global` (closing the loop).

**Issues:**

- **P0 — Missing `RMW_IMPLEMENTATION`/`CYCLONEDDS_URI`**: same as `navigation.launch.py`. The included `sensors.launch.py` sets them but only for *its* nodes. The EKFs and navsat_transform launched here run with shell defaults. **Add `SetEnvironmentVariable` at top of `LaunchDescription`**.
- **P1 — Single config file for two EKFs**: `ekf.yaml` is loaded twice (lines 92, 109). This works only if the YAML uses node-name-scoped sections. The cleaner pattern (used by upstream `nav2_gps_waypoint_follower_demo`) is two separate files: `ekf_local.yaml` and `ekf_global.yaml`. **GROUP 2 will check the YAML structure.**
- **P1 — Missing `args` for ekf nodes**: it's good practice to pass `arguments=['--ros-args', '--log-level', 'info']` or similar for runtime debugging. Currently no args at all.
- **P2 — `ekf_filter_node_odom` and `_map` node names hard-coded**: fine, matches Nav2 GPS tutorial convention, just be aware that any other launch file referencing these names has to use the same string.
- **P2 — No `enable_*` toggles for EKF/navsat**: not strictly needed but for bench-bringup having an `enable_ekf` toggle would let you launch sensors-only without bringing up the EKFs.
- **P2 — `navsat_transform_node` runs even if GPS is not yet locked**: this is fine in practice (`wait_for_datum: true` in `navsat.yaml` per CLAUDE.md "Datum: 34.059270, -117.820934 (fixed in navsat.yaml)") but worth documenting that the EKF map output will be all zeros until the first datum-corrected fix.

### launch/{actuator,teleop,webui,perception_test,localization_perception_test,costmap_test}.launch.py

#### actuator.launch.py (42 lines)

Trivial wrapper — only runs `actuator_node` with `actuator_params.yaml`. Has `use_sim_time` arg. **Issues:** no env vars set (P1: bench testing on a fresh shell may pick up FastDDS). Otherwise clean. The docstring (lines 1-9) clearly documents subscribe/publish topics — exemplary minimal launch.

#### teleop.launch.py (55 lines)

Bench teleop with keyboard. Spawns `actuator_node` + `teleop_twist_keyboard` in `xterm` (line 51).

**Issues:**

- **P1 — `prefix='xterm -e'` (line 51)** assumes `xterm` is installed. JetPack 6 sometimes does not ship xterm. Consider `prefix='xterm -e'` only inside `IfCondition` for an `enable_keyboard` arg, or fall back to `gnome-terminal`. Document the `apt install xterm` requirement somewhere.
- **P1 — No env vars**: same RMW issue.
- **P2 — `teleop_twist_keyboard`'s `parameters=[{'speed': 0.3, 'turn': 0.3}]` (line 50)**: hard-coded numbers; would be cleaner as `LaunchConfiguration` args.
- **P2 — Docstring lines 9-12 explain "for IMU heading-hold to be active … also launch sensors.launch.py"**: a more user-friendly approach would be to add an `enable_imu` arg that conditionally includes `sensors.launch.py` so the user can do `ros2 launch ... teleop.launch.py enable_imu:=true` in one command.

#### webui.launch.py (57 lines)

Spawns `actuator_node` + `webui_node`. Same template as teleop.

**Issues:** identical to teleop — no env vars, no IMU include option. The docstring (lines 9-12) is clear.

#### perception_test.launch.py (185 lines)

Standalone perception → costmap test, with optional drive. The docstring (lines 1-35) is **exceptionally well-written** — describes the test envelope, the limitation (identity TFs mean robot doesn't translate), the URL endpoints, and how to actually use it.

**Positives:**

- Static identity TFs `map → odom → base_link` (lines 88-99) are correctly published — required because the local costmap operates in `odom` frame. The docstring acknowledges that the robot doesn't actually translate in this test.
- ZED include with `serial_number: '42569280'` (line 110) — consistent with sensors.launch.py for the front camera.
- Foxglove bridge with explicit `capabilities` list (line 161-162) — better introspection than the navigation launch's bare-bones config.
- `enable_drive` arg (lines 63-69) lets users turn the WebUI joystick on/off — exactly the right ergonomics for outdoor testing.

**Issues:**

- **P1 — No env vars** (RMW/CycloneDDS): same recurring bug.
- **P1 — Two static_transform_publisher nodes use positional args** (lines 91, 97) instead of the modern `--frame-id`/`--child-frame-id` flags used in `costmap_test.launch.py` (lines 56-58, 67-69). Pick one style; positional args are deprecated in newer Humble releases.
- **P2 — `nav2_config = perception_test_params.yaml`** loaded for `controller_server` only (line 53, 133). This config will be reviewed in GROUP 3.
- **P2 — `lifecycle_manager_perception_test`** name is unique, good for parallel running.

#### localization_perception_test.launch.py (120 lines)

Wraps localization + perception + webui + controller_server (no full Nav2). Excellent docstring (lines 1-24) comparing this launch to `perception_test` and `navigation.launch.py`.

**Positives:**

- Cleanly composes existing launches via `IncludeLaunchDescription`.
- Forces `enable_velodyne='false'`, `enable_realsense='false'`, `enable_zed_front='true'` (lines 55-58) — minimum-viable sensors for the camera-only path.

**Issues:**

- **P1 — Same env-var gap**.
- **P2 — `lifecycle_manager_localization_perception_test`** is a long name. Fine. No conflicts.

#### costmap_test.launch.py (107 lines)

Sensors + static TFs + `controller_server` for costmap-only verification.

**Issues:**

- **P1 — `bond_timeout: 0.0` on the lifecycle manager** (line 91): commented as "required for standalone use without full Nav2". This is **a documented Nav2 escape hatch** but it disables the bond mechanism, so if `controller_server` crashes the manager won't notice. Acceptable for a test-only launch; flag and ensure it's not copied into `navigation.launch.py`.
- **P1 — Missing env-vars** (RMW/CycloneDDS): the docstring at line 40 acknowledges "Also sets RMW_IMPLEMENTATION and CYCLONEDDS_URI env vars" but that only applies to the *included* sensors launch — the controller_server and lifecycle_manager added later in this LaunchDescription run with shell defaults. Same recurring bug.
- **P2 — RViz config** at `rviz/costmap_test.rviz` (line 26) — referenced but not reviewed in this report (RViz files are auto-generated per the task instructions).
- **P2 — `enable_ntrip='false'` is hard-coded** (line 47): right call for offline testing, but should perhaps be exposed as a launch arg here too.
- **P2 — Nodes have no `output='screen'` on the static_transform_publishers** (lines 52, 64) — silent failure mode if the executable is missing.

### config/nav2_params.yaml + nav2_params_humble.yaml

Two parallel files, one for Jazzy, one for Humble. Both ~316/340 lines.

**Drift between the two files:**

The header comments correctly call out the differences (Humble lines 1-7, Jazzy lines 1-7). The plugin string syntax (`/` vs `::`), `plugin_lib_names` requirement, and `progress_checker_plugin` singular-vs-list are all correctly handled. **However, several genuine drifts beyond the documented ones:**

- **P1 — `desired_linear_vel`**: `nav2_params.yaml` line 49 sets `1.5`; `nav2_params_humble.yaml` line 64 sets `1.0`. **Real bug or intentional tuning?** The file headers don't explain. Pick one or document.
- **P1 — `required_movement_radius`**: Jazzy `0.5` (line 38), Humble `0.3` (line 53). Different progress-checker thresholds.
- **P1 — `max_robot_pose_search_dist`**: Jazzy `15.0` (line 67), Humble `999.0` (line 84). The Humble value of 999 is effectively "search entire path" — the comment says "skips route waypoints behind the robot" but per `standards_nav2_localization.md` §4 RPP knobs, this is a footgun: too-large values let RPP find a stale "closer to start" point and lose forward progress.
- **P1 — `max_allowed_time_to_collision_up_to_carrot`**: Jazzy `2.0` (line 60), Humble `5.0` (line 75). Materially different collision-prediction window.
- **P1 — `cost_scaling_dist` / `cost_scaling_gain` (Humble lines 78-79)**: present only in Humble. RPP supports these for cost-regulated scaling — Jazzy does not have them.
- **P1 — Local costmap `update_frequency`**: Jazzy `10.0` (line 115), Humble `5.0` (line 132). Halving the rate on Humble means 200 ms of stale costmap before a new sensor fusion — at 1.5 m/s that's 30 cm of robot travel per tick. Too slow given `controller_frequency=20.0`.
- **P1 — `always_send_full_costmap: true` (Humble line 134)**: missing from Jazzy. This forces the local costmap to publish entire grid each tick (bandwidth heavy but easier to subscribe). The Jazzy default is incremental. Inconsistency.
- **P1 — Local costmap `plugins` list differs DRAMATICALLY**:
  - Jazzy (line 124): `["voxel_layer", "semantic_front", "semantic_left", "semantic_right", "inflation_layer"]` — three separate semantic plugin instances.
  - Humble (line 142): `["semantic_layer", "inflation_layer"]` — **one** semantic plugin with three nested sources, **and no voxel_layer at all**.
  - The Humble file's docstring at lines 181-191 explains the rationale (per pepisg/nav2_segmentation_demo, three plugin instances don't propagate). This is correct architecture for the kiwicampus plugin. **But Jazzy still has the broken three-instance pattern** — flag for fix when porting.
  - **P0 — Humble local costmap has NO obstacle layer at all** — purely semantic + inflation. This means **LiDAR returns are not used by the local costmap** in Humble. For competition this is a serious issue: a hard obstacle the camera fails to classify will not appear on the costmap. The Velodyne data is wasted. (The voxel_layer block at lines 144-176 of Humble is a YAML island — declared but **not in the `plugins` list**, so loaded for nothing.) **This needs urgent decision before competition.**
- **P0 — `inflation_layer` parameters wildly different**:
  - Jazzy: `cost_scaling_factor: 2.5`, `inflation_radius: 1.8` (lines 246-247).
  - Humble local: `cost_scaling_factor: 5.0`, `inflation_radius: 0.5` (lines 270-271).
  - Humble global: same as local Humble (lines 306-307).
  - Per `standards_nav2_localization.md` §5, inflation_radius should be ~0.5-1.0 × largest body half-extent. Robot radius is 0.8m (CLAUDE.md), so 0.5 is too tight (planner won't avoid 0.5m collision) and 1.8 is generous safety margin. Pick one and document the rationale.
- **P0 — `mark_confidence`**: Jazzy `0` (line 188 et al.), Humble `1` (line 218). Confidence semantics differ between releases of the kiwicampus plugin — `1` filters out hits with confidence 0. Verify against the plugin's expected range.
- **P0 — `dominant_priority`**: Jazzy `true` (line 190), Humble `false` (line 220). This controls whether semantic cost overrides geometric cost. Different choice in each file.
- **P1 — Humble adds `clearing: true`, `raytrace_max_range: 8.0`, `raytrace_min_range: 0.0` per source (lines 207-209)**: required by the kiwicampus PR3 patch (per CLAUDE.md commit `8e2c493 Add 2026-04-29 changelog: kiwicampus PR3 raytrace-clear`). Jazzy version doesn't have these — would silently fail to clear stale paint cells.

**Common to both — issues:**

- **P0 — No `obstacle_layer` topic for `camera_depth`**: the depth camera contributes to the **VoxelLayer only** in Jazzy (lines 147-158), not the obstacle_layer. In Humble there's no voxel_layer in plugins so the camera is fully unused. Discrepancy between intent and effect.
- **P0 — Goal tolerance `xy_goal_tolerance: 2.0` (lines 43, 58)**: per CLAUDE.md "Goal tolerance: 2.0 m xy, 0.5 rad yaw" — this is intentional for IGVC. Big-vehicle. Per `standards_nav2_localization.md` §4 it's within the recommended 0.5-2.0m range. OK.
- **P1 — `min_obstacle_height` choices**:
  - Jazzy line 139: `-0.8` for velodyne (sensor at z=1.0m, so -0.8 = 0.2m above ground). With `obstacle_min_range: 1.0` this filters out the car body. OK.
  - Humble line 157: `-0.5` for velodyne (sensor at z=1.0m, so -0.5 = 0.5m above ground = "knee height"). **This skips ground-level obstacles below knee height** — small obstacles (curbs, low barrels) won't be detected by the local costmap. Jazzy version is more permissive. **Per IGVC AutoNav rules, low obstacles like potholes/painted lines must be detected — knee height threshold is wrong.**
- **P1 — VoxelLayer `mark_threshold: 0` (Jazzy line 134, Humble line 152)**: any single voxel hit marks the cell as obstacle. Per `standards_nav2_localization.md` §5, this is fine for outdoor LiDAR but be aware of single-return false positives from rain/dust. Document the choice.
- **P1 — Foxglove + `always_send_full_costmap: true` (Humble line 134)**: at 5 Hz × 50m × 50m × 0.2m resolution × 1 byte = 312KB/s per costmap publish. Saturates Jetson WiFi if used remotely. Use only on wired/local viz.
- **P1 — `velocity_smoother.feedback: "OPEN_LOOP"` (line 289)**: open-loop means the smoother does not consult odometry — fine since `odom_topic: /odometry/filtered` is set but unused. Closed loop would be more accurate. Per `standards_nav2_localization.md` §8, velocity smoother is required for high-accel platforms, and you have it — good.
- **P1 — `velocity_smoother.max_velocity: [2.0, 0.0, 1.0]`** vs CLAUDE.md max_linear_mps=1.5. Smoother allows 2.0; actuator caps at 1.5. Effectively the actuator slew-rate dominates. Pick one source of truth.
- **P0 — No collision_monitor configured**: per `standards_nav2_localization.md` §8 "Forgetting collision_monitor for high-speed platforms" is a Nav2 anti-pattern. At >1 m/s the costmap is too slow. Recommend adding `collision_monitor` with the SlowDown polygon at 1.5m, Stop at 0.8m. **High priority for competition safety.**

**route_server section** (Jazzy 299-315, Humble 323-339):

- **P0 — `global_frame: "map"` is set explicitly** — exactly per CLAUDE.md known-issue note. Both files set it. Excellent.
- `route_frame: "map"` matches `global_frame`. OK.
- `path_density: 0.5` — interpolated dense path every 0.5m. Per `standards_nav2_localization.md` §7, default is 0.05m; 0.5 is sparser but fine for a campus-scale graph.
- `edge_cost_functions: ["DistanceScorer"]` — only one scorer. **P2:** consider adding `CostmapScorer` so closed/blocked edges get rerouted automatically, per standards §7.
- `smooth_corners: false` (line 307/331) — comment "dense graph nodes follow road curves" is a clear rationale.
- `graph_filepath: ""` — overridden at launch time via `RewrittenYaml`. Correct.
- **P2 — Humble `DistanceScorer` block appears BEFORE `GeoJsonGraphFileLoader` (lines 334-339)** while Jazzy lists `GeoJsonGraphFileLoader` first (lines 310-315). Cosmetic — YAML order does not matter for Nav2 plugin loading — but inconsistent.

**bt_navigator section:**

- **P1 — Humble `plugin_lib_names` (lines 26-39)**: explicit list. **Missing nodes that might be referenced**: `nav2_reactive_fallback_bt_node` is missing — used by `navigate_route_graph_humble.xml` line 19 (`<ReactiveFallback>`). Also `nav2_sequence_bt_node` not listed. Behavior_tree.cpp generally registers built-in control nodes via the engine, so these may not need explicit listing; but `nav2_compute_route_bt_node` appears, so the project is aware of explicit listing. **Verify by launching navigation.launch.py with `bt_xml:=navigate_route_graph_humble.xml` and checking for "node not registered" errors at startup.**

**behavior_server:**

- Plugins `["backup", "drive_on_heading", "wait"]` (line 104/121). `backup` is configured but the BT XMLs **do not call** BackUp action — Ackermann vehicle commit. Loading the plugin without using it is cheap but odd; consider trimming to `["drive_on_heading", "wait"]`.

### config/navigate_route_graph.xml + navigate_route_graph_humble.xml + navigate_to_pose_simple_humble.xml

#### navigate_route_graph.xml (Jazzy, 28 lines)

BT.CPP v4 format, `BTCPP_format="4"` declared (line 11). Structure:

```
RecoveryNode (n=3)
├── Sequence (NavigateSequence)
│   ├── ComputeRoute
│   └── ReactiveFallback (FollowUntilGoalReached)
│       ├── GoalReached
│       └── FollowPath
└── Sequence (RecoveryActions)
    ├── ClearLocalCostmap
    ├── ClearGlobalCostmap
    └── Wait (3s)
```

**Positives:**
- Header docstring explains design choice: `Sequence` not `PipelineSequence` so route is stable.
- `GoalReached` + `ReactiveFallback` correctly stops `FollowPath` when the original goal is hit.
- Recovery clears both costmaps, waits, retries — sensible for outdoor.

**Issues:**
- **P1 — `ComputeRoute` has `error_code_id="{compute_path_error_code}"`** (line 15). Good practice, but the error code blackboard variable is never read elsewhere in this BT.
- **P1 — `error_code_id` for `FollowPath`** (line 18) — same.

#### navigate_route_graph_humble.xml (32 lines)

BT.CPP v3 format (no `BTCPP_format` attribute on `<root>` line 14, just `main_tree_to_execute`). Structurally identical to Jazzy version, with two important differences:

- **`number_of_retries="999"`** (line 16) — versus Jazzy's `number_of_retries="3"` (line 13). 999 is effectively infinite. Per the docstring lines 11-12: "retry up to 60 times (~5 min of waiting total before giving up)" — but the code says 999, not 60. **P1 — Doc/code drift.**
- **`Wait wait_duration="5"`** (line 27) — vs Jazzy's `3`. Together with 999 retries, that's ~83 minutes of waiting. Defensible for IGVC's static-obstacle competition (a stalled obstacle clears eventually) but **document that the run will not abort**.
- **P0 — `error_code_id` is missing** from the Humble version (lines 18, 21) — Humble's BT v3 may or may not support that port. **Verify.** If the Humble version is missing it because the binding is unavailable, it will swallow planner errors silently — risky.

#### navigate_to_pose_simple_humble.xml (29 lines)

Minimal `ComputePathToPose` → `FollowPath` BT for testing.

```
PipelineSequence (NavigateWithReplanning)
├── RateController (1 Hz)
│   └── ComputePathToPose (planner_id=GridBased)
└── FollowPath (controller_id=FollowPath, goal_checker_id=general_goal_checker)
```

**Positives:**
- Excellent docstring (lines 1-18) explains exactly when and why to use this BT.
- `goal_checker_id="general_goal_checker"` matches `nav2_params_humble.yaml` line 48.

**Issues:**
- **P2 — No GoalReached condition**: `FollowPath` runs until the controller reports SUCCESS, which happens when goal_checker fires. So this is fine; `PipelineSequence` doesn't need an explicit goal-reached condition.
- **P2 — Filename**: only a `_humble.xml` version exists. If the project ports to Jazzy, a non-`_humble` companion needs to be created. Document this.

### config/ekf.yaml

Single file with two top-level node-name-scoped blocks: `ekf_filter_node_odom` and `ekf_filter_node_map`. 114 lines.

**Positives:**
- Header citation (lines 1-8) explicitly references the upstream `nav2_gps_waypoint_follower_demo`.
- Both EKFs use `two_d_mode: true` (lines 14, 67) — correct per CLAUDE.md "tracked chassis" and `standards_nav2_localization.md` §2 ("set true for a planar tracked/diff-drive vehicle on flat ground").
- Both EKFs publish TF (`publish_tf: true`, lines 15, 68). **The local EKF publishes `odom→base_link`, the global EKF publishes `map→odom` (because `world_frame: map`).** This is the correct dual-EKF pattern per `standards_nav2_localization.md` §2.
- IMU `imu0_remove_gravitational_acceleration: true` (line 32, 85) — required for IMU acceleration channels, even though those channels are disabled in the config. Defensive setting.
- IMU `imu0_config` (lines 25-29, 78-82): `[false, false, false, true, true, true, false, false, false, true, true, true, false, false, false]` — fuses orientation (roll, pitch, yaw) + angular velocity (vroll, vpitch, vyaw). **No linear acceleration fused.** This is correct per `standards_nav2_localization.md` §2 ("Don't fuse linear acceleration from a noisy IMU on a slow ground robot — integration drift dominates").
- `imu0_differential: false`, `imu0_relative: false` — IMU absolute orientation is fused directly. Per `standards_nav2_localization.md` §2 this is fine when there's only one orientation source.
- Local EKF wheel-odometry fusion `odom0_config` (lines 38-42): `[false, false, false, false, false, false, true, false, false, false, false, true, false, false, false]` — fuses **only `vx` and `vyaw`** from `/wheel_odom`, not absolute pose. Per `standards_nav2_localization.md` §2 "Prefer fusing velocities (vx, vyaw, etc.) from wheel odometry rather than absolute pose, to avoid double-counting". Excellent.
- Global EKF GPS odometry `odom0_config` (lines 90-94): `[true, true, false, false, false, false, false, false, false, false, false, false, false, false, false]` — fuses **only x, y** from `/odometry/gps`. Correct, per Nav2 GPS tutorial.
- Process noise covariances are spelled out as 15×15 matrices (lines 47-61, 100-114). Diagonal-only with reasonable values: positions 1e-3 (local) / 1.0 (global), orientation 0.3, linear vel 0.5, angular vel 0.3, accels 0.3.
  - **Insightful design**: the global EKF has higher position process noise (`1.0` vs `1e-3`) — this is correct because GPS positional uncertainty is meters whereas the local odom-EKF has only IMU drift to worry about.

**Issues:**
- **P1 — `frequency: 30.0`**: per CLAUDE.md "Output rate: 100 Hz" for the Xsens. EKF at 30 Hz means downsampling — fine, but verify against `controller_frequency: 20.0` of Nav2 (EKF should be ≥ controller, which is satisfied).
- **P1 — `sensor_timeout` is not set** in either EKF block. Per `standards_nav2_localization.md` §2, default 0.1s. With `frequency: 30.0` and IMU at 100 Hz, this means a 100ms gap → EKF reverts to dead-reckoning. For competition-robustness consider setting `sensor_timeout: 0.5` so transient sensor dropouts don't immediately drift.
- **P1 — `print_diagnostics` not set** — per `standards_nav2_localization.md` §2 ("`true` in production — surfaces stale sensors, NaN measurements, and frame-id mismatches"). **Add `print_diagnostics: true` to both EKFs.**
- **P1 — `transform_time_offset` not set**: relevant for slightly out-of-sync sensor timestamps; default 0.0 is fine but worth documenting.
- **P1 — Both EKFs have `world_frame: odom` AND `world_frame: map`** (lines 21, 74). One publishes `odom→base_link`, the other `map→odom`. **Both have `publish_tf: true`. This is correct (each owns a different TF edge).** Worth a comment in the YAML to make it obvious.
- **P0 — `/wheel_odom` source**: the local EKF subscribes to `/wheel_odom` (line 37). CLAUDE.md says "back: `/wheel_odom` (integrated from E-line positions) → EKF". So the actuator_node publishes wheel-odom. **Verify in the avros_control review that `/wheel_odom` is actually published with realistic covariance** (not zeros, which the EKF replaces with 1e-6).
- **P1 — No `odom0_pose_rejection_threshold` or Mahalanobis gating** in either EKF — large GPS jumps (e.g., RTK loses lock and falls back to GPS-only) will be absorbed without any outlier rejection. Standard rejection thresholds are 0.5-2.0 for outdoor GPS.
- **P2 — Comment at line 8 ("No wheel odometry on this vehicle — IMU provides orientation + angular velocity")**: contradicts line 37 which DOES use `/wheel_odom`. **Outdated comment** — the codebase has since added wheel odometry.
- **P2 — `imu0_queue_size` and `odom0_queue_size: 10`** (lines 33, 45, 86, 97) are fine.

**Cross-check vs CLAUDE.md "TF Tree":**
- map ← navsat_transform_node — **wrong per code**. Per ekf.yaml, `map→odom` is published by `ekf_filter_node_map` (because `world_frame=map`, `publish_tf=true`). `navsat_transform_node` publishes `/odometry/gps` for fusion, not the TF. The CLAUDE.md TF Tree section is misleading on this point. **P2 doc fix.**

### config/navsat.yaml

25 lines. Cleanly written.

**Positives:**
- Header citation (lines 1-3) references upstream tutorial.
- `delay: 3.0` (line 12) — gives the IMU enough time to settle before transformation. Reasonable.
- **`magnetic_declination_radians: 0.0`** (line 16): comment "Xsens MTi-680G outputs ENU... uses internal World Magnetic Model for declination correction" — this is correct only if the Xsens is configured to apply declination internally. **Verify in xsens.yaml review (GROUP 3).** Per `standards_nav2_localization.md` §3, "Forgetting `magnetic_declination_radians` (treating mag-north as true-north) will rotate the global frame several degrees" — be sure the assumption holds.
- **`yaw_offset: 0.0`** (line 17): correct only if IMU reports zero-yaw at east (ENU). Xsens supports both NED and ENU output; the comment claims ENU. Verify.
- **`zero_altitude: true`** (line 19): drops Z to 0. Correct for 2D mode + flat campus.
- **`use_odometry_yaw: false`** (line 20): comment "Use IMU yaw directly (avoids circular dependency with global EKF)". Per `standards_nav2_localization.md` §3, this is the right choice during initial bring-up — if you set it to `true` after odometry yaw stabilizes you remove a feedback loop.
- **`wait_for_datum: true`** (line 21) + **`datum: [34.059270, -117.820934, 0.0]`** (line 22) — fixed CPP campus origin. Per `standards_nav2_localization.md` §3, this is **mandatory for multi-session repeatability when a pre-built route graph is used in `map` frame**. Excellent.
- **`broadcast_cartesian_transform: false`** (line 24): comment "EKF #2 publishes map -> odom; navsat broadcast would create TF loop". Correct — only one publisher per TF edge per `standards_nav2_localization.md` §1.

**Issues:**
- **P1 — `datum: [lat, lon, yaw]`**: 3-element form. Standard signature per `robot_localization` is **5 elements**: `[lat, lon, yaw, world_frame, base_link_frame]`. The 3-element form is accepted because the world_frame and base_link_frame are inferable from other params. This works on Humble but verify; the safer form is the full 5-element list.
- **P1 — `publish_filtered_gps: true`** (line 23): publishes `/gps/filtered` for monitoring. Useful but adds load. Document the topic so reviewers know.
- **P1 — `frequency: 30.0`**: matches EKF rate. OK.
- **P1 — Yaw datum is `0.0`**: this means the **+x axis of the map frame points east** (ENU convention) at the datum. If the campus road graph was generated assuming +x points north, this will rotate everything 90 degrees. **Verify against `cpp_campus_graph.geojson` orientation.**
- **P2 — No `transform_timeout` parameter set** — relies on default. Outdoor RTK with occasional dropouts could benefit from a 1-2s tolerance.

### config/cyclonedds.xml

20 lines.

**Positives:**
- **`<SharedMemory><Enable>false</Enable></SharedMemory>`** (lines 15-18) — correct per CLAUDE.md known-issues table ("CycloneDDS iceoryx/RouDi errors on launch"). RouDi daemon is not running, so shared memory must be disabled.
- **`<SocketReceiveBufferSize min="10MB"/>`** (line 13) — matches CLAUDE.md "Socket receive buffer: 10 MB minimum". Required for high-rate point cloud topics.
- **`<NetworkInterface autodetermine="true"/>`** (line 8) — auto-discovers the active interface. Fine for a single-NIC Jetson.
- **`<AllowMulticast>default</AllowMulticast>`** (line 10) — uses RMW default. Most LANs allow multicast; for hostile networks (some campus WiFi) you'd switch to `false` and rely on Discovery Server, but `default` is fine on the IGVC track.

**Issues:**
- **P2 — XML is missing the standard XML declaration line** — actually it's there (line 1). OK.
- **P2 — `<LogLevel>info</LogLevel>` inside SharedMemory block** (line 17) — works but it's the shared-memory log level, not the global. May be ignored when SharedMemory is disabled. Harmless.
- **P2 — Comment header missing**: a one-line comment describing the file's purpose would help future maintainers.
- **P2 — `<SocketReceiveBufferSize max>` not set**: defaults are usually OK; document choice.
- **P2 — No `<Tracing>` block**: for production, leave default; for debugging RMW issues you'd add `<Verbosity>config</Verbosity>` here.

### config/actuator_params.yaml

43 lines, well-commented.

**Positives:**
- Comments include physical units and the formulas used to derive constants — `m_per_motor_rev: 0.01994` cites `π × 80.85 mm / 12.75:1 gearbox` (line 9). Excellent traceability.
- Speed limits, slew-rate limits, and SparkMAX PID gains all in one place. Each block has a meaningful header comment (`# ---- ... ----`).
- `kFF: 0.000197` (line 35) with comment "From Phase 6 tuning: kFF = 1 / measured_max_RPM". Exemplary.
- `cmd_timeout_s: 0.5` (line 29) — drops to zero if no command for 500 ms. Critical safety property.
- `odom_frame: "odom"`, `base_frame: "base_link"` (lines 41-42) match REP-105 and ekf.yaml.

**Issues:**
- **P1 — `track_width_m: 0.7366`** (line 8): correct per CLAUDE.md "Track gauge: 0.7366 m". But the parameter naming is **track width** in this YAML, while the diff-drive convention uses **track gauge** (centerline-to-centerline). They are typically the same thing for a tracked vehicle, but the term `track_width` can be ambiguous (could mean width of one track tread). Add a comment that this is centerline-to-centerline.
- **P1 — `cmd_timeout_s: 0.5` deserves to be 0.3-0.4** for IGVC (a one-half-second motor coast is enough for a 1.5 m/s vehicle to travel 0.75 m, possibly over a no-go boundary).
- **P2 — `serial_port: "/dev/ttyACM0"`**: hard-coded. If the Teensy enumerates as ACM1 or ACM2 (race condition with other USB CDC devices), the launch fails. Standard fix: use a `udev` rule + symlink like `/dev/teensy_diff_drive`. Document the udev rule somewhere.
- **P2 — `serial_baud: 115200`**: standard. Good.
- **P2 — `state_pub_rate_hz: 20.0`** (line 31): matches CLAUDE.md "actuator_node @ 20 Hz". Good.
- **P2 — heading-hold gains (`heading_kp: 1.5`, `yaw_rate_kp: 0.3`, lines 25-26)**: tuning values, fine.

### config/{realsense,velodyne,xsens,ntrip_params,webui_params}.yaml

#### realsense.yaml (24 lines)

- **Positives:** docstring (lines 1-4) correctly notes the librealsense version, RSUSB backend, FW pin, IMU disable rationale.
- `pointcloud.enable: true` (line 21) — comment clarifies "Publishes /camera/camera/depth/color/points for obstacle detection". Used by `nav2_params.yaml` voxel_layer (camera_depth source).
- `align_depth.enable: true` (line 20) — required for color+depth registration.
- `enable_gyro: false`, `enable_accel: false` (lines 15-16) — D455 IMU disabled per the RSUSB backend bug. Document confirmed by CLAUDE.md.
- **P1 — `camera_namespace: ""`** (line 9): default is `camera`. With this empty, topics are at `/camera/...` instead of `/camera/camera/...`. But `nav2_params.yaml` line 148 references `/camera/camera/depth/color/points`. **Verify** — the realsense2_camera node uses both `camera_name` AND `camera_namespace` in its topic prefix. With `camera_namespace=""` and `camera_name="camera"`, the topic should be `/camera/...`. **Topic mismatch with nav2_params.yaml is a P0** if confirmed.
- **P1 — `tf_publish_rate: 0.0`** (line 23): "Static TF only" comment. RealSense ROS treats 0 as "static, never republish". Fine.
- **P2 — `depth_module.profile: "1280x720x30"`** and **`rgb_camera.color_profile: "1280x720x30"`** — high resolution, high rate. Bandwidth concern.

#### velodyne.yaml (22 lines)

- Two top-level node-name-scoped blocks: `velodyne_driver_node` and `velodyne_transform_node`.
- `device_ip: "192.168.13.11"` (line 7) — matches CLAUDE.md "Network Inventory" table.
- `port: 2368` — Velodyne factory default. Correct.
- `model: "VLP16"` — match.
- `frame_id: "velodyne"` (line 11) — must match URDF (will verify in GROUP 4).
- `min_range: 1.0`, `max_range: 50.0` — matches CLAUDE.md and `nav2_params.yaml` `obstacle_min_range: 1.0`, `obstacle_max_range: 50.0`. Consistent.
- `calibration: "/opt/ros/humble/share/velodyne_pointcloud/params/VLP16db.yaml"` — apt-installed path. **P2:** this hard-codes Humble path. If the project ports to Jazzy, the path becomes `/opt/ros/jazzy/...`. Use `find_package_share` substitution if possible.
- `organize_cloud: true` — publishes organized cloud (height>1). Useful for downstream perception. Per `standards_perception.md` §5, organized clouds are required by costmap layers like kiwicampus.
- `gps_time: false` — uses ROS time, not Velodyne's GPS time. Standard.
- **P2 — Both blocks at top level**: this loads as a single YAML file passed to two different node executables, which is fine because `velodyne_driver_node` and `velodyne_transform_node` each see only their own block via the node name. But the launch file uses different executable names — verify they match. (Per the launch file, `velodyne_driver_node` and `velodyne_transform_node` — match.)

#### xsens.yaml (57 lines)

- `port: "/dev/ttyUSB0"` (line 9), `baudrate: 921600` (line 10). Match CLAUDE.md after the 2026-03-01 baudrate flash.
- `serial_timeout: 150` (line 11) — Jetson-specific bump from default 100ms. Documented.
- `frame_id: "imu_link"` (line 12) — must match URDF.
- `output_data_rate: 100` (line 16) — 100 Hz, matches CLAUDE.md.
- **`enable_deviceConfig: false`** (line 15) — comment "Set true ONCE to flash config, then false". Excellent — prevents accidental re-flashing.
- **`pub_gnsspose: false`** (line 42) — comment "Disable — creates TF loop with EKF + navsat_transform". Crucial, exactly per `standards_nav2_localization.md` §1 ("Two TF publishers fighting for the same edge").
- **`pub_odometry: false`** (line 43) — comment "Disable — publishes imu_link->base_link TF that conflicts with URDF + EKF". Same.
- **`pub_transform: false`** (line 31) — comment "URDF provides base_link -> imu_link". Same.
- **P0 — `GNSS_LeverArm: [0.0, 0.0, 0.0]`** (line 47) with `# TODO: Measure on real vehicle`. **This MUST be measured before competition** — wrong lever-arm causes systematic GPS-to-IMU offset that the Kalman filter cannot correct. At a 0.7-1m antenna offset, this is ~1m position bias on every GPS update.
- **P1 — `pub_nmea: true`** (line 41): required for NTRIP feedback (GPGGA → caster). Correct.
- **P1 — `pub_imu: true`** + standard publishers — all correct for downstream consumers.
- **P1 — `enable_orientation_smoother: true`** (line 54), `enable_position_velocity_smoother: true` (line 55), `enable_continuous_zero_rotation_update: true` (line 56) — Xsens-side filters. Reasonable.
- **P1 — `ublox_platform: 4` (Automotive)** (line 50) — correct for car-like vehicle moving at 1.5 m/s, helps the u-blox handle dynamics.
- **P2 — `enable_beidou: false`** (line 51) — disables BeiDou constellation. Saves a few channels; use GPS+GLONASS+Galileo. Fine for North America.

#### ntrip_params.yaml (16 lines)

- **P0 — REAL CREDENTIALS COMMITTED**: lines 14-15 contain `username: "stoic_panini"` and **`password: "z0OYEP3Bwg0hdxvN"`**. **This is a serious credential leak — secrets must NOT be in git.** Move to a non-tracked file, environment variable, or `.gitignore`'d local file. Rotate the password immediately if this repo has ever been pushed publicly.
- `host: "ntrip.earthscope.org"` and `mountpoint: "PSDM_RTCM3P3"` — looks like a real EarthScope mount. The CLAUDE.md docs reference `rtk2go.com` as default, but EarthScope is a perfectly fine NTRIP caster (their mounts require registration, hence the credentials).
- `update_rate: 1.0` — 1 Hz GPGGA upload to the caster. Fine.
- **P1 — Missing `frame_id` parameter for the RTCM message**: ntrip clients sometimes need a frame; check the upstream `ntrip` package's expected params.

#### webui_params.yaml (7 lines)

- `web_port: 8000` — matches CLAUDE.md and webui.launch.py.
- `ssl_certfile: /home/dinosaur/avros_certs/cert.pem` — Jetson-only path. **P1 — Hardcoded user path** breaks portability. Should be a launch arg or `$HOME/avros_certs/...` substitution.
- `max_throttle: 1.0` — full cmd_vel range; clamped by max_linear_mps in actuator. Per CLAUDE.md "WebUI max_throttle: 1.0".
- **P2 — File is very minimal**: 7 lines. No docstring, no comments. Add explanatory header.

### config/zed_*.yaml

#### zed_front.yaml (54 lines)

- **`/**:` wildcard top-level key** (line 13): correct per `standards_perception.md` §7 — required by the wrapper to apply params under any namespace.
- Sectioned config — `general`, `depth`, `pos_tracking`, `mapping`, `object_detection`, `body_tracking`, `sensors`. Correct hierarchy for the wrapper.
- `grab_resolution: 'HD1080'`, `grab_frame_rate: 15`, `pub_frame_rate: 15.0` (lines 19-21). Per CLAUDE.md: "HD1080 @ 15 fps".
- `depth_mode: 'NEURAL_LIGHT'` (line 24) — v5 enum, comment confirms valid values. Excellent — flagged in CLAUDE.md known-issues table that older `PERFORMANCE/QUALITY/ULTRA` enums were deprecated.
- `point_cloud_freq: 15.0` (line 27) — comment "match image rate for ApproximateTime sync". Per `standards_perception.md` §3, this matters for downstream sync.
- **`depth_stabilization: 1`** (line 33) with detailed multi-line comment (lines 28-32) explaining the SDK trap (forced pos_tracking, GPU usage). Outstanding documentation of a non-obvious bug.
- `pos_tracking_enabled: true` (line 40) — comment line 35-38 explains why this MUST be true even though EKF owns localization. Good defensive comment.
- `mapping_enabled: false`, `od_enabled: false`, `bt_enabled: false` — all SDK heavy modules disabled. Good for thermal headroom.
- `sensors_image_sync: true` (line 49) — important for stereo timing.
- `publish_imu: false`, `publish_mag: false`, etc. (lines 50-53) — Xsens is the IMU; ZED IMU is redundant. Correct.

**Issues:**
- **P2 — Comment line 8** says topic is `/zed_front/zed_node/rgb/image_rect_color` (v4 name). Per CLAUDE.md and standards_perception.md, v5 wrapper uses `/zed_front/zed_node/rgb/color/rect/image`. **Comment is stale, wrong topic name.** Update.
- **P2 — Comment lines 5-6** says "Currently using 49910017 (known-good unit) until the physical-front serial is verified" — but **the launch file** (sensors.launch.py line 149) sets serial to **42569280**. So the launch file disagrees with the comment, and the launch file is now correct (per its own newer comment that 42569280 is the verified front). **Update zed_front.yaml comment.**

#### zed_left.yaml (42 lines)

- Same `/**:` pattern.
- `grab_resolution: 'SVGA'` (line 16) — comment line 9-14 explains the IGVC C2 cost-model trade-off. Excellent rationale.
- `max_depth: 8.0` (line 23) — tightened from front's 15.0 because side cameras only need to reach the 5m semantic horizon.
- Otherwise same structure as front.

**Issues:**
- **P2 — Same depth_stabilization, same pos_tracking, etc.** — consistent across files.
- **P2 — No frame override**: relies on launch's `camera_name: 'zed_left'` substitution.

#### zed_right.yaml (38 lines)

- Identical to zed_left.yaml in structure.
- **Issues:** none beyond zed_left's.

#### zed_back.yaml (44 lines)

**This file is significantly out of step with the others — old structure.**

- **P0 — Uses `zed_node:` top-level node-name key** (line 6) instead of `/**:` wildcard. This was the **v4 wrapper convention**. The v5 wrapper expects `/**:`. With v4 syntax under v5, params **silently fail to apply**.
- **P0 — `general.grab_resolution: 'HD720'`** (line 17): per CLAUDE.md known-issues table, ZED X v5.2 does not support HD720. The valid enums are `HD1200 | HD1080 | SVGA | AUTO`. **This will fail at startup with `InvalidParameterValueException`** if zed_back is ever enabled.
- **P0 — `depth.depth_mode: 'PERFORMANCE'`** (line 22): deprecated v4 enum. Will throw `InvalidParameterValueException`.
- **P0 — `depth.depth_stabilization: 30`** (line 25): reasonable in v4; v5 default is `-1` (force pos_tracking) and the front-camera config explains why we use `1`. Stale.
- **P0 — `point_cloud_freq: 5.0`** (line 37): if `pos_tracking` is force-enabled, cloud generation needs higher rate to be useful. Likely will not match image rate.
- **P1 — `serial_number: 49910017`** (line 10) — but per sensors.launch.py the front camera now uses serial 42569280, and CLAUDE.md "Left/Right/Back ZED X" section says 49910017 was repurposed to front. **Stale serial assignment.**
- **P1 — `publish_tf: false`** (line 14), `publish_map_tf: false` (line 15) — fine. But `camera_frame: 'zed_back_camera_center'` (line 13) — this conflicts with the URDF's xacro chain (which uses zed_macro emitting `_camera_link`, `_left_camera_frame`, etc).
- **CONCLUSION:** **zed_back.yaml is dead code from v4 days. Either delete or rewrite to match zed_front.yaml structure.** Also `enable_zed_back` is not a launch arg in `sensors.launch.py` — there's no way to enable this camera anyway. Remove from `data_files` or properly resurrect.

#### perception_test_params.yaml (98 lines)

This is a slim Nav2 config for `perception_test.launch.py`. Reviewed because it duplicates parts of `nav2_params_humble.yaml`.

- **Positives:**
  - Excellent docstring (lines 1-15) explaining differences from full params.
  - `update_frequency: 5.0` matches Humble local costmap.
  - `width: 20`, `height: 20`, `resolution: 0.1`, `robot_radius: 0.4` — slimmer test geometry.
  - `inflation_radius: 0.3` (line 97) — comment "1.5 m radius would smear it across the whole map for testing" — sensible test rationale.
  - `clearing: true`, `raytrace_max_range: 8.0`, `raytrace_min_range: 0.0` (lines 80-82) — PR3 raytrace clearing applied.
  - `samples_to_max_cost: 0` (line 91) vs `1` in main params — instant max cost on first hit, useful for visibility during testing.
- **Issues:**
  - **P1 — `controller_plugins: ["FollowPath"]` with `desired_linear_vel: 0.0`** (lines 25, 41): comment "Placeholder controller — never invoked in this test launch". OK, but if a stray nav goal hits this controller it will publish 0-velocity commands — not actually dangerous but odd.
  - **P2 — `mark_confidence: 0`** (line 90) — same Jazzy-style. Consistent with the test purpose.

### config/perception_test_params.yaml

(Reviewed inline above with the rest of the test perception block — see preceding section.)

### urdf/avros.urdf.xacro

168 lines, well-structured xacro.

**Positives:**
- **`base_link` is at chassis geometric center at ground level** (lines 12-18) — correctly conventional. `base_footprint` exists as a coincident link (lines 42-50) so Nav2 tools that key off `base_footprint` (some costmap layers) still work.
- **Mass / inertia values** (lines 36-39) are guesstimates (`mass: 150 kg`, principal moments 10/20/25) but clearly nontrivial. Not used by Nav2 directly but useful for Gazebo/sim if ever added.
- **`xacro:include filename="$(find zed_wrapper)/urdf/zed_macro.urdf.xacro"` (line 103)** — exactly per CLAUDE.md known-issues table and `standards_perception.md` §6/§7. **This is the correct approach.** Each `<xacro:zed_camera name="zed_*" model="zedx">` (lines 104, 119, 135) emits the full body-and-optical chain. Correct REP-103 alignment guaranteed.
- **`enable_zed_back` xacro arg** (line 10) — correct conditional pattern for an optional camera.
- **All sensor mount comments** include "Sketch 2026-04-25: ..." with measured-from-sketch dimensions in inches. Excellent traceability.
- **ZED left/right rpy="0 0 ±1.5708"`** (lines 128, 143) — rotates camera to face Y direction. ZED macro's optical-frame chain handles the 90° body→optical rotation internally, so this just rotates the body frame. Correct.

**Issues:**
- **P0 — `imu_link`, `velodyne`, `zed_*_camera_link` mounts are sketch-derived**, not measured. Lines 54, 75, 101, 117, 133, 162: every sensor is annotated `TODO: Measure exact mount position on real vehicle`. Per `standards_perception.md` §6 ("Hand-rolled URDF for cameras... misses optical-frame rotations" — although ZED is fine because of the macro), and per CLAUDE.md TF-tree section ("Sensor mount positions in URDF (`avros.urdf.xacro`) are approximate — measure on real vehicle"). **For competition, every sensor offset must be measured on the actual vehicle.** A 5cm IMU offset is a 5cm bias on every IMU-derived position fix. **High priority pre-competition task.**
- **P0 — `velodyne` link has no `<collision>` block** (lines 77-86). Same for IMU (54-65). Means costmap inflation around the LiDAR housing is ignored. For a 0.5m-tall LiDAR + 0.4m chassis, the box collision geometry from base_link covers the LiDAR cylinder approximately, so this may be intentional. Document.
- **P1 — No `realsense` URDF entry**: CLAUDE.md TF-tree mentions `camera_link` (RealSense), and `realsense.yaml` has `base_frame_id: "camera_link"`. **The URDF does not declare a `camera_link` frame for the RealSense.** The realsense2_camera node will publish `camera_link` itself if `publish_tf: true` (which is set in realsense.yaml line 22). This works **only** if the wrapper publishes a fixed pose — and per the wrapper docs, it publishes an identity TF unless extrinsics are given. **Result: the RealSense floats at base_link's origin until measured.** Add a `<link name="camera_link">` block + a measured `<joint>`.
- **P1 — `zed_back_camera_center` block (lines 151-166)** is hand-rolled (not via the macro) and only emits a single link. It does NOT include the optical-frame chain. If `enable_zed_back:=true` is ever set, the back camera's image messages will lookup_transform onto `zed_back_camera_center` and miss the optical rotation. **Per `standards_perception.md` §10 anti-patterns: "Hand-rolled URDF for cameras instead of vendor xacro macro — misses optical-frame rotations".** Either delete the back-camera block (since `zed_back.yaml` is also broken, see GROUP 3) or rewrite using `<xacro:zed_camera name="zed_back" model="zedx">` like the others.
- **P1 — Chassis dimensions** (lines 5-7): `chassis_length: 1.23` (matches `nav2_params.yaml` "wheelbase 1.23m"), `chassis_width: 0.9`, `chassis_height: 0.4`. **Note the comment says "chassis box dimensions, not wheelbase"** but then the value matches the wheelbase — likely just a coincidence. If 1.23 is the chassis box length this overestimates somewhat. Verify against actual vehicle.
- **P1 — `xsens_height: 0.500` (line 19)** is used as the reference point for other sensor Z values (lines 92, 112, 128, 143). Convenient but creates a chain of dependencies — if you ever measure the IMU mount height as different, every sensor moves with it. Either bake all sensors to absolute Z values or document the convention.
- **P2 — No `material` definitions reused via `<xacro:property>`** — colors are inlined per link (e.g., line 27 `<color rgba="0.5 0.5 0.5 0.8"/>`). Minor.
- **P2 — `base_footprint` is at xyz="0 0 0"`** (line 49). Per REP-105 / Nav2 convention `base_footprint` is the ground-projected version of `base_link`, typically at the same X,Y but Z=0 of the wheelbase. Since the comment says "coincident with base_link (already on ground)", this is consistent.

**Frame chain summary (all static):**

```
base_link
├── base_footprint
├── imu_link (xsens, 0.5m up)
├── velodyne (0.089m fwd, 0.659m up)
├── zed_front_camera_link (via macro)  → full optical chain
├── zed_left_camera_link  (via macro, rotated +90deg)
├── zed_right_camera_link (via macro, rotated -90deg)
└── zed_back_camera_center (hand-rolled, optional)  ← needs macro rewrite
```

Match against CLAUDE.md "TF Tree" — the URDF chain is consistent with what CLAUDE.md describes for the `base_link` subtree.

### test/* (boilerplate)

Three boilerplate ament linter tests — `test_copyright.py`, `test_flake8.py`, `test_pep257.py`. All are unmodified from the `ros2 pkg create --build-type ament_python` template.

**Per `standards_ros2_python.md` §5:**

- `test_copyright.py` (26 lines) — has `@pytest.mark.skip(reason='No copyright header has been placed in the generated source file.')` (line 20). **This means the test does not actually check copyrights.** Per the standards reference, "skipping is acceptable but the file should at least compile and `pytest.skip()` cleanly". This file does. Acceptable but **P2**: either commit to MIT copyright headers in every `.py` and remove the skip, or delete the test.
- `test_flake8.py` (26 lines) — runs PEP-8/Flake8 on all `.py` in the package. Active.
- `test_pep257.py` (24 lines) — runs PEP-257 docstring linter. Active.

**Issues:**
- **P1 — No actual unit tests**: per `standards_ros2_python.md` §5, the `test/` directory should also contain real unit tests (`test_<module>.py`). For a launch-only package this is harder, but consider:
  - `test_launch_imports.py` — import every `*.launch.py` and verify `generate_launch_description()` returns a `LaunchDescription` without errors.
  - `test_yaml_validity.py` — yaml.safe_load every `config/*.yaml` and assert no parse errors.
  - `test_urdf_xacro.py` — invoke `xacro avros.urdf.xacro` and assert it produces valid XML.
- **P2 — No integration tests via `launch_pytest`**: per `standards_ros2_python.md` §5, `launch_pytest` exists for this. Could add a smoke test that brings up `costmap_test.launch.py` headless and checks that `/local_costmap/costmap` publishes within N seconds. Heavy, but for a competition-bound codebase this is the kind of thing reviewers look for.
- **P2 — `test_flake8.py` and `test_pep257.py` will fail loudly on any Python file added to the package** that violates style. So far, this package has launch files only — they should be flake8/pep257 clean. Verify via `colcon test --packages-select avros_bringup`.

**`setup.cfg`** (verified separately):
```
[develop]
script_dir=$base/lib/avros_bringup
[install]
install_scripts=$base/lib/avros_bringup
```
Correct boilerplate per `standards_ros2_python.md` §1, item 5. **No issues.**

**`resource/avros_bringup`** (verified separately): empty marker file, 0 bytes — correct ament index marker. **No issues.**

**`LICENSE`** file (verified separately): present at package root. Need to verify it's MIT text (matches package.xml `<license>MIT</license>`).

## Cross-cutting issues in avros_bringup

### 1. RMW_IMPLEMENTATION / CYCLONEDDS_URI environment variables (P0)

Pattern: `sensors.launch.py` sets both env vars at the top of `LaunchDescription` (lines 41-48). Every other launch file (`navigation.launch.py`, `localization.launch.py`, `actuator.launch.py`, `teleop.launch.py`, `webui.launch.py`, `perception_test.launch.py`, `localization_perception_test.launch.py`, `costmap_test.launch.py`) **does not set them**. Some include `sensors.launch.py` and inherit transitively only for that subtree's nodes.

This means **every directly-instantiated `Node()` in those parents (Nav2 servers, EKFs, navsat, foxglove, controller_server, lifecycle_managers) runs with whatever the user's shell defaults to.** If the user's shell defaults to FastDDS (the ROS 2 Humble out-of-the-box default), all those nodes go FastDDS → mismatch with CycloneDDS-running sensor stack → action goal corruption (the bug already documented in CLAUDE.md known-issues table).

**Fix: add `SetEnvironmentVariable` for both vars at the top of every launch file**, OR factor out a `cyclonedds_env.py` helper that returns the actions and is used everywhere.

### 2. cpp_campus_graph.geojson — colossal size discrepancy (P0)

CLAUDE.md states `cpp_campus_graph.geojson`: "52 nodes, 113 edges". **Actual file: 16,651 nodes and 17,492 edges (34,143 features total).** That's a 320× node-count overestimate by CLAUDE.md, and the actual graph is two orders of magnitude denser than the doc claims.

**Implications:**
- `route_server` Dijkstra over 17k edges will take much longer than over 113 — `max_planning_time: 2.0` may not be sufficient. **Verify route_server can plan in <2s for typical campus goals.**
- File is gigantic — install/deploy slower.
- The graph's coordinates use **EPSG:32611** (UTM Zone 11N) per the `crs.properties.name` field, with first node at `coordinates: [-163.269, -633.825]`. These look like meter offsets (likely from a local ENU origin) — but `route_server` consumes them via `GeoJsonGraphFileLoader` and reprojects to `route_frame: map`. **Verify the loader correctly handles EPSG:32611 + small numeric coords**, since the `frame: "map"` property on each node may be misleading the loader (it's flagged as `map` but is actually UTM-numeric).
- Edge properties have `id, startid, endid, cost, metadata` — schema is correct per `standards_nav2_localization.md` §7.

**Action: update CLAUDE.md to reflect actual graph size**, OR if the 52-node graph was intended (a hand-curated ridge graph), regenerate using the OSMnx script with appropriate filtering and replace this auto-generated dense graph.

### 3. Launch-file env var leak vs the test launches (P1)

Each test launch file (`perception_test.launch.py`, `localization_perception_test.launch.py`, `costmap_test.launch.py`) instantiates `lifecycle_manager_*` nodes. If the test launch is started in the *same shell* as a parallel `navigation.launch.py`, you'd have multiple lifecycle managers competing — but more practically, none of these files set the env vars.

### 4. nav2_params.yaml ↔ nav2_params_humble.yaml drift (P0/P1)

The two files are intended to be parallel "Jazzy version" and "Humble version" of the same configuration, but they have substantial drift beyond what the headers document:
- Different controller speeds (1.5 vs 1.0 m/s)
- Different progress checker thresholds (0.5 vs 0.3 movement radius)
- Different inflation_radius (1.8 vs 0.5)
- Humble has a single `semantic_layer` with three nested sources; Jazzy has three separate plugin instances (the latter is broken architecture per kiwicampus design)
- Humble has the `clearing/raytrace_max_range` keys for PR3; Jazzy does not
- Humble local costmap has NO `voxel_layer` in `plugins`; Jazzy does

Since the project is on Humble for IGVC, the `nav2_params.yaml` (Jazzy) is **dead config that drifts unmonitored**. **Action: either (a) regenerate the Jazzy version from the Humble version each time the Humble version is updated, (b) delete `nav2_params.yaml` until the project actually targets Jazzy, or (c) merge into a single file using `<xacro:if>`-style conditionals if a YAML pre-processor is added.**

### 5. Hardcoded NTRIP credentials in version control (P0 SECURITY)

`config/ntrip_params.yaml` lines 14-15 contain `username: "stoic_panini"` and `password: "z0OYEP3Bwg0hdxvN"`. **This is a credential leak in a public-named repo (`IGVC_ROS2` on GitHub).** Per CLAUDE.md the repo is at `https://github.com/Paarseus/IGVC_ROS2`. **If this repo is public, the credentials are already public.** Mitigations:
- Rotate the password with the EarthScope NTRIP service immediately.
- Move credentials to a non-tracked file (e.g., `~/.config/avros/ntrip.yaml`), load via env var, or use a Vault-style secret manager.
- Add `config/ntrip_params.yaml` to `.gitignore` and ship a `ntrip_params.yaml.example` template.
- Audit `git log -p` for any historical credential exposure.

### 6. Sensor mount positions are sketch-derived, not measured (P0)

Every sensor TODO in `urdf/avros.urdf.xacro`: "TODO: Measure exact mount position on real vehicle". Same for `xsens.yaml` GNSS_LeverArm `[0.0, 0.0, 0.0]`. For competition this matters:
- IMU position 5cm off → 5cm constant bias on IMU-derived positions.
- GNSS antenna offset measured from IMU center — wrong value here biases GPS position by the antenna lever-arm magnitude (typically 0.5-1m).
- LiDAR mount 5cm off → costmap obstacle locations shifted 5cm.

**Action:** dedicate a half-day to measuring with a tape measure or laser distance meter. Update URDF + `GNSS_LeverArm`. The parts most affecting accuracy are the IMU origin and GNSS antenna.

### 7. Voxel layer absent on Humble local costmap (P0)

Already flagged in nav2_params section. **The Humble local costmap (`nav2_params_humble.yaml` line 142) has `plugins: ["semantic_layer", "inflation_layer"]` — no voxel_layer, no obstacle_layer, no LiDAR contribution.** The Velodyne data is fed into the global costmap only. For close-range obstacle avoidance during drive, you have only camera-detected lane/barrel/pothole classes. Anything the camera misclassifies will be hit.

**Action: add `voxel_layer` to local costmap plugins list in Humble**, with the Velodyne and (optionally) RealSense-depth observation sources. Or document explicitly that this is intentional reliance on the global costmap.

### 8. Inconsistency in launch-file path patterns (P1)

Some launches use `os.path.join(pkg_dir, 'launch', 'foo.launch.py')` for siblings (e.g., `navigation.launch.py` line 148), while ZED includes use list-concatenation `[get_package_share_directory('zed_wrapper'), '/launch/zed_camera.launch.py']`. The standards-recommended `PathJoinSubstitution([FindPackageShare(...), 'launch', 'zed_camera.launch.py'])` is not used anywhere. **This works but is brittle** to symlink-install corner cases. Pick one idiom across the package.

### 9. actuator_node duplicated in 4 launch files (P1)

Same `actuator_node` Node block appears in `actuator.launch.py`, `teleop.launch.py`, `webui.launch.py`, `navigation.launch.py`. DRY violation. **Factor into a shared include** (`actuator_only.launch.py` or a snippet returned from a common helper).

### 10. Test boilerplate is purely lint, no integration (P1)

Per the test review section. For a competition-deadline codebase this is acceptable, but the lack of any integration smoke test means a launch-time error is detected only when someone actually runs the launch — easy to miss before competition day.

### 11. Stale doc / comment references (P2)

- `zed_front.yaml` comment line 8 references v4 topic `rgb/image_rect_color`; v5 uses `rgb/color/rect/image`.
- `zed_front.yaml` comment line 5-6 says serial is 49910017; launch file uses 42569280.
- `ekf.yaml` line 8 says "No wheel odometry on this vehicle"; line 37 fuses `/wheel_odom`.
- CLAUDE.md TF tree section: "navsat_transform_node" listed as the publisher of `map` — actually `ekf_filter_node_map` publishes `map→odom`.
- CLAUDE.md graph stats wrong by 320× (covered above).
- `navigate_route_graph_humble.xml` comment line 11 says "60 retries (~5 min)"; code uses 999.

### 12. Dead/orphaned configs (P2)

- `zed_back.yaml` is broken (uses v4 syntax + invalid HD720 enum). Either fix or delete.
- `zed_left.yaml` and `zed_right.yaml` are working but `enable_zed_left/right` defaults to `false` and they're not on the Phase 3/Phase 4 critical path. Document phase plans.
- `nav2_params.yaml` (Jazzy) is dead until project ports.

### 13. Foxglove + bandwidth (P2)

`navigation.launch.py` line 202-211 unconditionally launches Foxglove WebSocket on port 8765. Combined with `always_send_full_costmap: true` on the local costmap, this generates ~300 KB/s costmap stream alone. Add an `enable_foxglove` toggle.

### 14. Missing `<exec_depend>` entries in package.xml (P0/P1)

Already detailed in package.xml section: `nav2_route`, `kiwicampus_semantic_segmentation_layer`, `avros_perception`, `zed_wrapper`, `xsens_mti_ros2_driver`, `tf2_ros`, `nav2_lifecycle_manager`, `foxglove_bridge`, `nav2_velocity_smoother`, `nav2_smac_planner`, `nav2_regulated_pure_pursuit_controller`, `nav2_behaviors`, `rviz2`. Missing entries break `rosdep install` workflows.

### 15. No collision_monitor (P0)

Per `standards_nav2_localization.md` §8 ("Forgetting `collision_monitor` for high-speed platforms"), at >1 m/s you need `nav2_collision_monitor` running off the LiDAR scan with SlowDown + Stop polygons. Currently no collision_monitor is launched or configured. **High priority for IGVC safety.**

### 16. Test-launch lifecycle managers all autostart with `bond_timeout: 0.0` (P2)

Standalone launches use `bond_timeout: 0.0` workaround. Be aware the bond mechanism is therefore disabled — if controller_server crashes during a test, the lifecycle manager won't notice. Acceptable for tests; document.

## Punch list

### P0 (IGVC blockers)

1. **Rotate NTRIP credentials and remove from git** (`config/ntrip_params.yaml:14-15`). The password `z0OYEP3Bwg0hdxvN` is committed to a public-named repo. Move secrets out of version control before pushing again.
2. **Set `RMW_IMPLEMENTATION` and `CYCLONEDDS_URI` env vars in every launch file**, not just `sensors.launch.py`. Currently `navigation.launch.py`, `localization.launch.py`, all the test launches, `actuator.launch.py`, `teleop.launch.py`, `webui.launch.py` directly instantiate Nav2/EKF nodes that run with shell defaults (FastDDS) — exactly the bug already in CLAUDE.md known-issues.
3. **Add `voxel_layer` to Humble local costmap** (`config/nav2_params_humble.yaml:142`). Currently `plugins: ["semantic_layer", "inflation_layer"]` means LiDAR data does NOT influence local obstacle avoidance — the Velodyne is wired up but unused at the local-costmap level. A camera misclassification = collision.
4. **Add `<exec_depend>` for missing runtime packages in `package.xml`**: `nav2_route`, `kiwicampus_semantic_segmentation_layer`, `avros_perception`, `zed_wrapper`, `xsens_mti_ros2_driver`, `nav2_lifecycle_manager`, `tf2_ros`, `foxglove_bridge`, `nav2_velocity_smoother`. Otherwise `rosdep install --from-paths src --ignore-src -y` will fail to satisfy deps on a fresh system.
5. **Decide and document the GeoJSON graph density**. CLAUDE.md says 52 nodes / 113 edges; the actual `cpp_campus_graph.geojson` has **16,651 nodes / 17,492 edges**. Either regenerate a sparse graph for `route_server` performance, or update CLAUDE.md and verify `max_planning_time: 2.0` is sufficient on the dense graph.
6. **Configure `nav2_collision_monitor`** for safety on a >1 m/s vehicle. Per Nav2 anti-patterns, the costmap (5-10 Hz) is too slow at speed.
7. **Measure and update sensor mount positions in `urdf/avros.urdf.xacro`** (every TODO line). Especially the IMU `imu_link` joint (line 70) and the GNSS lever arm (`xsens.yaml:47`). Without these, GPS-fused position is biased by 0.5-1m.
8. **Resolve nav2_params.yaml ↔ nav2_params_humble.yaml drift**. Different controller speeds, different inflation_radius, different plugin lists, different `mark_confidence`. Either delete the Jazzy version or sync explicitly.
9. **Remove or rewrite `zed_back.yaml`**. Uses v4 wrapper syntax (`zed_node:` top-level), invalid v4 enums (`HD720`, `PERFORMANCE`), and stale serial. If `enable_zed_back:=true` is ever set, ZED wrapper throws `InvalidParameterValueException`.
10. **`navigate_route_graph_humble.xml` `error_code_id` ports missing** (lines 18, 21) compared to the Jazzy version. Verify that BT.CPP v3 supports this; if it doesn't, planner failures are silently swallowed.
11. **`min_obstacle_height: -0.5`** on Humble local costmap voxel_layer (line 157) — knee-height threshold means low obstacles (curbs, painted-line bumps) are missed. Lower to -0.8 (matching Jazzy's 0.2m-above-ground filter) or document why knee height is acceptable for IGVC.
12. **`mark_confidence`, `dominant_priority`, `samples_to_max_cost` differ** between Jazzy and Humble nav2_params. Pick one source of truth.
13. **`realsense.yaml: camera_namespace=""`** vs `nav2_params.yaml`/`_humble.yaml` topic `/camera/camera/depth/color/points`: the empty namespace produces `/camera/...` not `/camera/camera/...`. Verify topic names match across all configs at runtime; if mismatched, depth cloud isn't fed to costmap.

### P1 (professional quality)

1. **Switch `<depend>` tags to `<exec_depend>` in `package.xml`** for runtime-only deps (`rclpy`, `launch`, `launch_ros`, `robot_state_publisher`, `xacro`). Per `standards_ros2_python.md` §1, `ament_python` packages should prefer `<exec_depend>`.
2. **Tighten `setup.py` glob patterns**:
   - `glob('config/*')` → `glob('config/*.yaml') + glob('config/*.xml') + glob('config/*.geojson')`
   - `glob('urdf/*')` → `glob('urdf/*.xacro') + glob('urdf/*.urdf')`
   - `glob('rviz/*')` → `glob('rviz/*.rviz')`
   - `glob('launch/*launch.[pxy][yma]*')` → `glob('launch/*.launch.py')`
3. **Add `enable_xsens` launch arg** so `sensors.launch.py` can be brought up without an IMU plugged in (bench test).
4. **Add `enable_foxglove` launch arg** to `navigation.launch.py` and the test launches; default off in competition.
5. **Set `print_diagnostics: true` and `sensor_timeout: 0.5` on both EKF blocks** in `ekf.yaml`. Currently neither is set.
6. **Set `transform_timeout` (1.0-2.0 s) in `navsat.yaml`** for outdoor RTK with intermittent dropouts.
7. **Tighten `cmd_timeout_s`** in `actuator_params.yaml` from 0.5 to 0.3-0.4 for IGVC. At 1.5 m/s a 500ms coast is ~0.75m of unintended travel.
8. **Replace `prefix='xterm -e'` in `teleop.launch.py:51`** with a more portable approach or a fallback to `gnome-terminal`. JetPack 6 may not ship xterm by default.
9. **Replace hardcoded `/home/dinosaur/avros_certs/...`** in `webui_params.yaml:4-5` with a launch-arg or `$HOME/...` substitution.
10. **Factor `actuator_node` into a single shared launch include**, used from `actuator.launch.py`, `teleop.launch.py`, `webui.launch.py`, `navigation.launch.py`. Currently duplicated 4×.
11. **`bt_navigator.plugin_lib_names` (Humble)** — verify all BT nodes referenced in `navigate_route_graph_humble.xml` are listed. Currently `nav2_reactive_fallback_bt_node`, `nav2_sequence_bt_node`, `nav2_goal_reached_condition_bt_node` may be missing — empirical test required.
12. **Pick one `inflation_radius`**: Jazzy 1.8m vs Humble 0.5m, robot radius 0.8m. Per `standards_nav2_localization.md` §5, ~0.5-1.0 × robot half-extent is recommended; for a 0.8m robot, 0.4-0.8m is the sweet spot. Document the choice.
13. **Pick one `velocity_smoother.max_velocity`**: 2.0 m/s in nav2_params, but actuator caps at 1.5 m/s. Match.
14. **Remove unused `behavior_plugins` (`backup`)** from `nav2_params*.yaml:104,121`. Loaded but the BT XMLs don't call it. Trim to `["drive_on_heading", "wait"]`.
15. **Add `frame_id` parameter for the RTCM message in `ntrip_params.yaml`** if the upstream `ntrip` package supports it.
16. **Replace hardcoded `/opt/ros/humble/share/velodyne_pointcloud/...` calibration path** in `velodyne.yaml:20` with a `$(find-pkg-share velodyne_pointcloud)/...` substitution if the YAML supports it (or via a launch argument).
17. **Add `udev` rule + symlink for Teensy** so `actuator_params.yaml:4` doesn't depend on `/dev/ttyACM0` enumeration order.
18. **Update stale comments**:
    - `zed_front.yaml:8` (v4 topic name)
    - `zed_front.yaml:5-6` (serial number)
    - `ekf.yaml:8` ("No wheel odometry" but odom0 fuses /wheel_odom)
    - `navigate_route_graph_humble.xml:11-12` ("60 retries" but code is 999)
    - CLAUDE.md TF Tree (`map ← navsat_transform_node` should be `map ← ekf_filter_node_map`)
    - CLAUDE.md graph stats (52/113 → actual 16651/17492)
19. **Use `PathJoinSubstitution([FindPackageShare(...), 'launch', 'foo.launch.py'])`** consistently for launch file references, instead of mixing `os.path.join` and list-concatenation patterns.
20. **Add explicit `enable_realsense_pointcloud`** sub-toggle so the depth cloud can be disabled (saves USB bandwidth) without disabling the camera.
21. **Add `enable_collision_monitor` launch arg + node + config** (relates to P0#6).
22. **Add real unit tests** under `test/`: `test_launch_imports.py`, `test_yaml_validity.py`, `test_urdf_xacro.py`. Each is ~10 lines; would catch most "broken on competition day" launch errors.
23. **`zed_back.yaml: serial_number: 49910017`** — same as front. If keeping the file, give it a unique placeholder serial or delete.
24. **Local costmap `update_frequency: 5.0` (Humble)** is too slow for `controller_frequency: 20.0`. Bump to 10 Hz to match Jazzy.
25. **Add CostmapScorer to `route_server.edge_cost_functions`** so blocked edges don't get planned through.
26. **Document the `route_server.path_density: 0.5`** choice — wider than default 0.05; results in coarser paths that may not feed RPP smoothly. Verify visually against typical drives.
27. **`min_obstacle_height: 0.0` on RealSense camera_depth observation** (Humble line 168 — actually it's `-0.5`). Should not allow ground points; either filter aggressively or disable the depth source for the local costmap.

### P2 (nice-to-have)

1. **Bump `package.xml` version** from `0.0.0` to `0.1.0`.
2. **Add `<author>`, `<url type="repository">`, and `<url type="bugtracker">`** to `package.xml`.
3. **Add a docstring header to `webui_params.yaml` and `cyclonedds.xml`**.
4. **Convert `kP/kI/kD/kFF` parameter names** in `actuator_params.yaml` to lowercase `k_p`, `k_i`, etc. for ROS-style consistency.
5. **Make `chassis_length / chassis_width / chassis_height`** xacro args (parameterized at xacro-call time) rather than properties.
6. **Add visual material refs** in URDF as `<xacro:property name="..." />` blocks rather than inline color RGBA.
7. **Trim `velodyne.yaml`** by removing unused params (e.g., `gps_time`, `time_offset` if not needed).
8. **Move `behavior_server` plugins** to a sub-yaml or trim to actually-used.
9. **`pub_dq`, `pub_dv`, `pub_pressure`, `pub_temperature`, `pub_status`, `pub_utctime`, `pub_sampletime`** in `xsens.yaml` — set true/false explicitly per use; current settings are reasonable but worth review.
10. **Fold `enable_zed_left`/`_right`/`_back` into a list `enable_zeds: [front, left, right]`** and iterate, instead of three separate `IncludeLaunchDescription` blocks.
11. **Add a `default_value` to `bt_xml`** that varies based on `ROS_DISTRO` — currently this works because the file does it at module load time, but a cleaner approach is to pass in a `LaunchConfiguration` and resolve via `OpaqueFunction`.
12. **Add `enable_actuator` launch arg** to `navigation.launch.py` so you can stand up Nav2 without spinning the motors during indoor sim.
13. **Add docstrings to `webui_params.yaml`, `cyclonedds.xml`** as noted.
14. **`SmacPlannerHybrid.cache_obstacle_heuristic: false`** — true would speed up replans for static obstacles. Re-evaluate.
15. **`SmacPlannerHybrid.smooth_path: true`** — verify the smoother doesn't push the path inside lethal cells (it can on tight turns).
16. **Document RViz files**: add a short comment to each `.rviz` file describing its purpose.
17. **Consider `OpaqueFunction`** for the perception camera-list expansion so each comma-separated camera in `perception_cameras:=front,left,right` becomes its own perception node.
18. **`use_rotate_to_heading: false`** is correctly set for an Ackermann vehicle, but the comment "Ackermann vehicle cannot rotate in place" is technically true — IGVC is a tracked diff-drive robot per CLAUDE.md, which CAN rotate in place; the parameter is set false for IGVC AutoNav forward-only judging rules. **Update comment to reflect competition rules, not vehicle kinematics.**
19. **`controller_server.controller_plugins: ["FollowPath"]`** — only one. Could add a backup `Stop` plugin for safety.
20. **`PluginLibNames` for Humble may need additions** — verify by launch.
21. **Add `bond_timeout` defaults to lifecycle managers in `navigation.launch.py`** rather than relying on Nav2 default.
22. **Use upstream patterns from `nav2_bringup`** — `nav2_bringup/launch/navigation_launch.py` has more polished compositions; consider taking inspiration.
23. **Document `costmap_test.rviz` differences from `avros.rviz` and `three_cam.rviz`** — purpose unclear from filenames alone.

## Positives

The package as a whole is **well above average for a student-team competition codebase**. Specific strengths:

1. **Module docstrings on every launch file** are descriptive, list what's launched, link to upstream references where relevant, and call out non-obvious design choices (`navigation.launch.py:1-17` explains why nav2 servers are launched directly rather than via `nav2_bringup`; `perception_test.launch.py:1-35` explains the test envelope's limitations).
2. **Cross-distro awareness** — `navigation.launch.py:36-43` selects between Jazzy and Humble param/BT files via `ROS_DISTRO` env var. Forward-thinking even if currently Humble-only.
3. **Conditional sensor inclusion** — every sensor has an `enable_*` toggle (with the noted Xsens exception). Bench-test ergonomics are good.
4. **`SetEnvironmentVariable` for `RMW_IMPLEMENTATION` + `CYCLONEDDS_URI`** in `sensors.launch.py` directly addresses the FastDDS/CycloneDDS interop bug documented in CLAUDE.md.
5. **CycloneDDS configuration is correct**: shared memory disabled (RouDi not running), socket buffer bumped to 10MB for high-rate clouds.
6. **ZED launch include passes only `camera_name`/`camera_model`/`serial_number`** — avoids the `namespace`-overwriting-`node_name` bug in the wrapper. Comment lines 128-136 cite the gotcha.
7. **ZED v5 enums used correctly** (`NEURAL_LIGHT`, `HD1080`) per the wrapper's expected values; v4 enums avoided.
8. **URDF uses `<xacro:zed_camera>` macro from `zed_wrapper`** — the right way to get the body/optical frame chain.
9. **Dual-EKF pattern correctly implemented** in `ekf.yaml` — local EKF (`world_frame: odom`) fuses only continuous sources (IMU + wheel odom), global EKF (`world_frame: map`) adds GPS. Each publishes a different TF edge.
10. **EKF `_config` matrices are correct**: orientation + angular velocity from IMU, vx + vyaw from wheel odom, x + y from GPS. No double-counting; no acceleration fusion.
11. **`navsat.yaml` has `wait_for_datum: true` with a fixed datum** — required for multi-session repeatability with the pre-built route graph.
12. **`navsat.yaml: broadcast_cartesian_transform: false`** prevents TF loop with `ekf_filter_node_map`.
13. **`xsens.yaml` carefully disables `pub_gnsspose`, `pub_odometry`, `pub_transform`** to prevent TF and topic conflicts with EKF/URDF.
14. **`route_server: global_frame: "map"` is set explicitly** — avoids the "Failed to transform from '' to map" silent-failure bug already documented in CLAUDE.md.
15. **`actuator_params.yaml` is exemplary** — every magic number has a unit, every parameter has a derivation comment (`m_per_motor_rev: π × 80.85 mm / 12.75:1 gearbox`).
16. **Each launch file's docstring explains test envelope and limitations** (especially `perception_test.launch.py:25-29` warning about identity TFs).
17. **Recovery BT design** — `Wait` only, no `Spin`/`BackUp`, deliberately chosen for IGVC AutoNav forward-only judging rules. Documented in BT XML headers.
18. **PR3 raytrace-clear is applied** (`nav2_params_humble.yaml:208` and `perception_test_params.yaml:80-82`) per the kiwicampus/semantic_segmentation_layer fix — addresses the stale-LETHAL-cell decay issue.
19. **Foxglove bridge enables remote diagnostics** without a full ROS install on the operator laptop.
20. **`setup.cfg`, `resource/avros_bringup` marker, `LICENSE` file** are all present** — basic ament_python hygiene checks pass.
21. **Comprehensive `enable_*` args in `sensors.launch.py`** and `navigation.launch.py` make incremental bring-up natural.
22. **Lifecycle manager ordering** in `navigation.launch.py:66-76` — route_server before bt_navigator — is correct and explains itself.
23. **`ekf.yaml` has full process noise covariances spelled out** as 15×15 matrices, not relying on defaults. Reasonable diagonal values per channel.
24. **CLAUDE.md is referenced and the codebase tracks its own known-issues** in a structured way; many of the bugs already in CLAUDE.md known-issues table are visibly avoided in the actual config.
