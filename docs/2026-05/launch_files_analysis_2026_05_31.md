# Launch File Audit — IGVC_ROS2 (2026-05-31)

> **Method.** Multi-agent workflow: 14 launch files read in full (each agent also read the
> configs/URDF/included launches/BT XMLs they reference), 7 subsystem doc-research agents built
> cited cheat-sheets from official sources (ROS 2 launch, Nav2, robot_localization, Velodyne, ZED
> v5.2.2, Xsens/NTRIP, foxglove), then candidate issues were to be adversarially verified.
>
> **Confidence note (read this first).** The **Read** and **Docs** phases succeeded fully — the
> per-file findings and the documentation appendix are solid. The **adversarial-verify** phase
> largely failed: only 7 of 73 verifier agents emitted a structured verdict (the rest ran out of
> turn after their web lookups because each was handed the entire doc cheat-sheet). So the
> "confirmed/dismissed" labels below rest more on the synthesizer's judgment than on independent
> checks. **Parsa/Claude then hand-verified the competition-critical claims directly against the
> repo on 2026-05-31** — those corrections are marked `✅ VERIFIED` / `⚠️ CORRECTED` inline.

---

## Post-workflow independent verification (2026-05-31)

| Report claim | Hand-check result |
|---|---|
| **#10** RewrittenYaml `default_nav_to_pose_bt_xml`/`graph_filepath` overrides may be a silent no-op (keys missing) | **⚠️ CORRECTED — NON-ISSUE.** `nav2_params_humble.yaml:27-28` has `default_nav_to_pose_bt_xml: ""` + `default_nav_through_poses_bt_xml: ""` under `bt_navigator`; `:651` has `graph_filepath: ""`. Keys exist → override applies correctly. **No action.** |
| **#2** `smoother_server` (no params block) could stall the lifecycle autostart | **⚠️ CORRECTED — DEAD WEIGHT, NOT A HAZARD.** Structurally true (no `smoother_server:` block in `nav2_params_humble.yaml`, only `velocity_smoother:` at `:628`), but field-test history shows GPS-waypoint nav worked 2026-05-30 → the full 7-server lifecycle *does* reach `active`. `route_server` is fully configured at `:642-657`, just unused by the default BT. Optional cleanup only. |
| **#1** `costmap_test.launch.py` loads Jazzy `nav2_params.yaml`, not Humble | **✅ VERIFIED.** `costmap_test.launch.py:25` → `nav2_params.yaml`. Diagnostic-only. |
| **#12** ZED front serial `42569280` (launch) vs `49910017` (docs) | **✅ VERIFIED.** `zed_front.yaml:5` + `urdf:214` say `49910017`; `sensors.launch.py:135` + `perception_test.launch.py:127` use `42569280`. ZED off by default → core path unaffected. |
| **#7** `enable_ntrip` default `true` in `localization.launch.py` vs `false` elsewhere | **✅ VERIFIED.** `localization.launch.py:58` = `true`; `sensors.launch.py:54` = `false` (§I.2). Production includers force `false`, so competition path is safe. |
| **#8** `teleop.launch.py` undeclared `xterm` | **✅ VERIFIED.** `:63` `prefix='xterm -e'`; `package.xml` declares no `xterm`. |
| **#9** stale `bt_xml` argument description | **✅ VERIFIED.** `:53` real default `navigate_igvc_autonav_humble.xml`; `:128-129` description says `navigate_to_pose_simple_humble.xml`. |

**Net effect of verification:** the two items the workflow tagged "competition-critical / verify on
Jetson" (RewrittenYaml keys, smoother/route activation) are **both already fine**. The genuinely
actionable items are the consistency/doc/sim issues below — none break the default core competition
path (Velodyne + Xsens + dual-EKF + Nav2 MPPI/Navfn).

---

## Executive summary

The launch layer is mechanically sound: across all 14 active launch files every node
package/executable name verifies against upstream, all `IncludeLaunchDescription` argument chains
are correctly declared and forwarded, the CycloneDDS env-forcing pattern is applied correctly
(top-of-`LaunchDescription`, before any node action, never inside a scoped `GroupAction`), the ZED
includes correctly pass only `camera_name`+`camera_model`+`serial_number` (dodging the documented
tree-collapse bug) with `publish_tf=false`/`publish_urdf=false`, and the Velodyne de-skew config
exploits a confirmed upstream arg-swap correctly. **Five files are clean as-is** (actuator, webui,
teleop, perception, yaw_diag — modulo doc nits), a handful need consistency/doc fixes, and the
**sim package needs real changes** to faithfully exercise the Humble production stack. The single
most visible defect is the **ZED front-camera serial conflict** (launch `42569280` vs docs
`49910017`) — believed correct in the launch files (user-confirmed GMSL port 0 = `42569280`) but
unreconciled in docs. **No issue breaks the default core path** (Velodyne + Xsens + EKF + Nav2),
which is what runs at competition.

## Verdict table

| Launch file | Tier | Verdict | Headline issue |
|---|---|---|---|
| `sensors.launch.py` | production | ⚠️ minor | Front ZED serial `42569280` contradicts docs (`49910017`); self-contradictory right-cam TODO |
| `localization.launch.py` | production | ⚠️ minor | `enable_ntrip` defaults `true` here vs `false` in `sensors.launch.py` (§I.2 rule) |
| `navigation.launch.py` | production | ⚠️ minor | `smoother_server`/`route_server` lifecycle-managed but unused by default BT (dead weight); stale `bt_xml` description |
| `actuator.launch.py` | production | ✅ correct | None |
| `teleop.launch.py` | bench | ⚠️ minor | Undeclared `xterm` dependency required by `prefix='xterm -e'` |
| `webui.launch.py` | bench | ✅ correct | None (config/doc nits live outside this file) |
| `perception.launch.py` | production (included) | ⚠️ minor | Dead `camera_name` legacy arg with misleading comment |
| `yaw_diag.launch.py` | diagnostic | ✅ correct | Stale `/dev/ttyACM0` comment only |
| `perception_test.launch.py` | diagnostic | ✅ correct | Carries the *correct* front serial; doc nits live in `zed_front.yaml` |
| `costmap_test.launch.py` | diagnostic | ❌ needs change | Loads Jazzy `nav2_params.yaml`, not the Humble production local_costmap |
| `localization_perception_test.launch.py` | diagnostic | ⚠️ minor | Inherits `enable_ntrip=true`; `respawn=True` on a one-shot lifecycle manager |
| `sim_navigation.launch.py` | sim | ❌ needs change | Humble MPPI-DiffDrive controller mismatches the Ackermann sim car |
| `sim.launch.py` | sim | ⚠️ minor | Unconditional `zed_wrapper` xacro include breaks on a clean sim host |
| `sim_teleop.launch.py` | sim | ⚠️ minor | Undeclared `xterm` dependency |

## Confirmed issues (ranked by severity)

### 1. [HIGH] `costmap_test.launch.py` loads the stale Jazzy `nav2_params.yaml`, not the Humble production costmap
**File:** `costmap_test.launch.py:25` (`nav2_config = config/nav2_params.yaml`). ✅ verified.

This "costmap isolation test" loads `nav2_params.yaml`, whose `local_costmap` uses
`[nav2_costmap_2d::VoxelLayer, semantic_front/left/right, inflation_layer]`. The production Jetson
runs Humble, and `navigation.launch.py:45-46` selects `nav2_params_humble.yaml`, whose
`local_costmap` (`:281`) uses a structurally different set:
`[spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer (STVL), semantic_layer (single source),
inflation_layer]` with an explicit footprint polygon. **Net: the test validates a `VoxelLayer`
LiDAR behavior the competition robot does not use (the Jetson uses STVL).** The stale
`nav2_params.yaml` header still describes an Ackermann vehicle + `SmacPlannerHybrid`/DUBIN.

**Why it matters:** a green costmap-isolation test gives false confidence about the *competition*
local costmap.

**Fix:** branch on `os.environ.get('ROS_DISTRO','humble')` to select `nav2_params_humble.yaml`
exactly as `navigation.launch.py:44-46`, or add a docstring stating it intentionally tests the
Jazzy config.

### 2. [MEDIUM — was flagged HIGH] `navigation.launch.py` lifecycle-manages `smoother_server` + `route_server` that the default BT never uses
**File:** `navigation.launch.py:81` (`smoother_server`), `:83` (`route_server`), `:89` (both in the lifecycle `node_names`). ⚠️ corrected.

`nav2_params_humble.yaml` has **no `smoother_server:` block** (only `velocity_smoother:` at `:628`),
and `route_server` *is* fully configured (`:642-657`, DistanceScorer + GeoJsonGraphFileLoader) but
the default Humble BT (`navigate_igvc_autonav_humble.xml`) calls neither `SmoothPath` nor
`ComputeRoute`. **Field-test history (GPS-waypoint nav worked 2026-05-30) proves the full 7-server
lifecycle reaches `active`** — so this is **dead weight, not an autostart hazard** as the workflow
originally worried.

**Fix (optional hardening):** drop `smoother_server` and `route_server` from both `nav2_servers`
and `lifecycle_nodes` on the Humble path — fewer servers to activate, smaller attack surface for a
lifecycle stall. Leave as-is if you want them available for BT experiments.

**Docs:** lifecycle ordering/activation — `https://docs.nav2.org/configuration/packages/configuring-lifecycle.html`; nav2_route ships on Humble (`ros-humble-nav2-route` `1.1.20-1`) — `https://raw.githubusercontent.com/ros/rosdistro/master/humble/distribution.yaml`.

### 3. [HIGH, sim] `sim_navigation.launch.py` Humble MPPI-DiffDrive controller mismatches the Ackermann Webots car
**File:** `sim_navigation.launch.py:37` selects `nav2_params_humble.yaml`, whose `FollowPath` is
`nav2_mppi_controller::MPPIController` with `motion_model: DiffDrive` (`nav2_params_humble.yaml:80,134`).

The sim vehicle is Ackermann: `avros_vehicle_driver.py` uses `WHEELBASE=1.23`, steering motors,
`MAX_STEERING_RAD≈0.489` (~2.36 m min turning radius), and at `v<=0.01` sets a steering angle with
**zero** wheel velocity (no in-place rotation). MPPI-DiffDrive commands tight/in-place turns the sim
car cannot execute, so sim navigation will not match the controller's predictions. The Jazzy
fallback (`nav2_params.yaml` RPP, `regulated_linear_scaling_min_radius:2.31`) *is* Ackermann-correct
— the sim stack was authored against the Jazzy/RPP path. **Since competition runs Humble, the sim
does not faithfully exercise the production controller.** (Plugin string is correct; the mismatch is
the *vehicle model*.)

**Fix:** give the Webots vehicle a diff-drive model matching production kinematics, **or** accept
the sim as a Jazzy/RPP-only harness and document that it does not represent the Humble controller.

### 4. [HIGH, sim] `nav2_sim_overrides.yaml` `FollowPath` keys are RPP-only, silently ignored under Humble/MPPI
**File:** `nav2_sim_overrides.yaml:11-13`. Sets `FollowPath.desired_linear_vel=1.0` +
`use_collision_detection=false` — RegulatedPurePursuit params. Humble `FollowPath` is MPPI, which
has neither → the intended sim slow-down + collision-disable **don't take effect**; MPPI keeps
`vx_max=1.5` and its CostCritic stays active. The `local_costmap` size/resolution overrides
(`:15-20`) *do* apply on both distros. Compounds #3.

**Fix:** if the sim stays on MPPI, use MPPI-namespaced override keys; otherwise gate the override
file on `$ROS_DISTRO`.

### 5. [HIGH, sim] `sim.launch.py` `robot_state_publisher` xacro unconditionally requires `zed_wrapper`
**File:** `sim.launch.py:49-51` runs `xacro` on `avros.urdf.xacro`, which at `:164` has an
**unconditional** `<xacro:include filename="$(find zed_wrapper)/urdf/zed_macro.urdf.xacro"/>`.
`zed_wrapper` is built-from-source on the Jetson but is **not** an `avros_sim` dependency — on a
clean sim host `xacro` aborts with `$(find zed_wrapper)` not found → `robot_state_publisher` never
starts → no `base_link` TF → the EKF/Nav2 stack is starved.

**Fix:** add `zed_wrapper` as an `exec_depend` of `avros_sim`, **or** gate the ZED include behind a
`xacro:arg` (e.g. `enable_zed`, default true, set false for sim). Affects `sim.launch.py` and its
includers.

### 6. [MEDIUM, sim] Semantic costmap layer enabled with no `perception_node` in sim
**File:** `sim_navigation.launch.py` (starts no perception) + `nav2_params_humble.yaml`
(`semantic_layer enabled:true`, subscribing `/perception/front/semantic_*` + `label_info`).
`nav2_sim_overrides.yaml` does not disable it → those topics never publish; at best no marks, at
worst missing-topic/TF warnings. **Open question:** whether the kiwicampus `SemanticSegmentationLayer`
completes `on_activate` when its `transient_local` `LabelInfo` + mask/cloud topics never publish. If
it blocks rather than warns, this is a sim bringup failure.

**Fix:** add `semantic_layer.enabled:false` (and/or drop it from the plugin lists) in `nav2_sim_overrides.yaml`.

### 7. [MEDIUM] `localization.launch.py` `enable_ntrip` defaults `true`, contradicting `sensors.launch.py` + IGVC §I.2
**File:** `localization.launch.py:58` (`true`) vs `sensors.launch.py:54` (`false`, "§I.2 forbids base
stations"). ✅ verified. `navigation.launch.py:135` + `yaw_diag.launch.py:64` force `false`
downward, so **the competition full-stack and yaw-diag paths keep NTRIP off**. The exposure is a
direct `ros2 launch avros_bringup localization.launch.py` (a documented bench command) — it starts
`ntrip_client` by default; `localization_perception_test.launch.py` inherits this too.

**Why it matters:** inconsistent defaults around a rules-compliance knob. Per project memory, public
NTRIP (MDOT CORS) *is* intentionally used in practice, so `true` may be intended — but two files
disagree. Deliberate-decision item, not a mechanical bug.

**Fix:** pick one default and apply it consistently. Given the §I.2 rationale in `sensors.launch.py`,
set `localization.launch.py:58` to `'false'` (operators pass `enable_ntrip:=true` for RTK).

### 8. [MEDIUM] `teleop.launch.py` undeclared `xterm` dependency (`prefix='xterm -e'`)
**File:** `teleop.launch.py:63`. ✅ verified — no `xterm` in any `package.xml`. If absent on the
Jetson, the keyboard node never starts while `actuator_node` does — a silent half-launch.
`sim_teleop.launch.py:39` has the identical gap.

**Fix:** add `<exec_depend>xterm</exec_depend>` to `avros_bringup/package.xml` (and `avros_sim`), or
document the `sudo apt install xterm` prerequisite. (ROS2 launch raises `FileNotFoundError` at spawn
when a `prefix` binary is missing.)

### 9. [LOW/cosmetic] `navigation.launch.py` `bt_xml` argument description is stale
**File:** `:53` sets `default_bt='navigate_igvc_autonav_humble.xml'`, but the `DeclareLaunchArgument`
description at `:128-129` says the default is `navigate_to_pose_simple_humble.xml`, and the inline
comment at `:59-62` names a third non-existent default. ✅ verified. No functional effect
(`default_value` wins) — but `ros2 launch ... --show-args` prints a misleading default.

**Fix:** update the `:128-129` description + `:59-62` comment to the real default.

### 10. [LOW] `sensors.launch.py` front ZED serial vs docs; self-contradictory right-cam TODO
**File:** `:135` front `serial_number='42569280'`; `:170` right `='49910017'`; `:169` TODO. ✅
verified. Front uses `42569280` (comment: "GMSL port 0 → physical front, confirmed by user"), but
`zed_front.yaml:5`, `avros.urdf.xacro:214`, and `CLAUDE.md` say the front is `49910017`.
`perception_test.launch.py:127` independently uses `42569280` for the front — corroborating that
**the launch files are correct and the docs/yaml are stale**. The right-camera TODO at `:169` ("was
42569280, now repurposed to front") then conflicts with the `49910017` on the next line. ZED off by
default → core path unaffected, but first `enable_zed_front:=true` is a guess until reconciled.
(`serial_number` is correctly a *launch arg*; any serial in `zed_front.yaml` is dead.)

**Fix:** reconcile to one source of truth — if `42569280` is verified front, update `CLAUDE.md`,
`zed_front.yaml:5`, `avros.urdf.xacro:214`, and rewrite the `:169` TODO.

### 11. [LOW] `perception.launch.py` dead `camera_name` legacy arg with misleading comment
**File:** `:51-55`. `camera_name` (default `'front'`) is declared with a comment implying it's
honored when `cameras` has one entry. It isn't — `_spawn_nodes` only ever reads `cameras`. So
`camera_name:=left` silently does nothing.

**Fix:** wire `camera_name` as the fallback when `cameras` is unset, or delete the arg + fix the
comment.

### 12. [LOW, sim] `sim_navigation.launch.py` sim odom EKF has no translational velocity input
The sim publishes only `/imu/data`, `/gnss`, `/velodyne_points` — no `/wheel_odom`, no ZED VIO.
`ekf.yaml`'s odom EKF fuses IMU orientation+angular only (no `vx`/`ax`), so `odom->base_link` tracks
heading but never translates. **Does not affect the Jetson run** (where `actuator_node` supplies
`/wheel_odom`) — hence low severity.

**Fix:** publish `/wheel_odom` from the Webots driver (`twist.linear.x`, `twist.angular.z`, small
nonzero covariance) so the sim EKF translates.

### 13. [LOW/cosmetic] Stale Ackermann/RPP comments + sim-vs-prod BT divergence in the sim route-graph path
`navigate_route_graph_humble.xml:4,8` comments describe an Ackermann/RPP setup, but the Humble
controller is MPPI-DiffDrive. The sim hardcodes the route-graph BT while production defaults to the
recovery point-to-point BT — so the sim validates graph routing, not competition behavior. Comment +
sim-fidelity note only.

## Dismissed / false alarms

- **#10 RewrittenYaml silent no-op** — **dismissed by hand-check** (keys present, see top table).
- **`smoother_server` autostart stall** — **dismissed** (field runs prove activation; see #2).
- **`actuator.launch.py` "no `$ROS_DISTRO` branch"** — not a defect; the APIs it uses are stable
  across Humble/Jazzy and it has no nav2-param/BT files to swap. A branch would be dead code.
- **`actuator.launch.py` "FastDDS by default on Humble" comment** — substantively correct (Humble
  default RMW is `rmw_fastrtps_cpp`); forcing CycloneDDS is needed for interop.
- **`foxglove_bridge` executable/port "unverifiable"** — cleared. Package + executable both
  `foxglove_bridge`; integer `port` default 8765; released on Humble. Bare `Node()` is the
  documented pattern.
- **`route_server` "not on Humble"** — cleared. `nav2_route` is apt-installable on Humble (`1.1.20-1`).
- **`webui.launch.py` as a whole** — structurally identical to known-good `actuator.launch.py`;
  executables + params verify. The `max_throttle`/SSL-path/systemd nits live in config, not this file.

## Cross-cutting observations

- **`enable_ntrip` default drift is the most systemic inconsistency.** `sensors.launch.py` (false,
  §I.2) vs `localization.launch.py` (true). Production includers force false, so competition is safe;
  `localization.launch.py` + `localization_perception_test.launch.py` run NTRIP by default. Unify it.
- **CycloneDDS env handling is correct everywhere it is set.** Every `avros_bringup` launch puts the
  two `SetEnvironmentVariable` actions first, at top level (not in a scoped `GroupAction`) — which
  the docs confirm propagates into includes + child processes. The one exception is
  `costmap_test.launch.py`, which relies on the *included* `sensors.launch.py` to set the env then
  spawns Nav2 nodes after — works only because the include is the first action; brittle. Set the env
  explicitly at its top like its siblings. Sim launches set no DDS env (acceptable for self-contained
  sim, but a deliberate divergence).
- **Distro-branching:** only `navigation.launch.py` + `sim_navigation.launch.py` branch on
  `$ROS_DISTRO`, both correctly. `costmap_test.launch.py` should and doesn't (#1).
- **Stale comments are the dominant nit class:** ZED serial docs, `bt_xml` defaults, "Teensy UDP"
  (actuator is USB-serial), `/dev/ttyACM0`, "HSV pipeline" (default is now `adaptive`),
  Ackermann/RPP BT comments. None affect runtime; a single doc-sweep clears most.
- **`package.xml` dependency lint:** `avros_sim`/`avros_bringup` spawn nav2 servers (incl.
  `nav2_route`) but `exec_depend` only on `nav2_bringup`, relying on transitive deps; `xterm`
  undeclared in both teleop paths.

## Documentation reference appendix

- **ROS2 launch — `SetEnvironmentVariable`:** writes process-global `os.environ`; affects only
  actions after it in document order; propagates into `IncludeLaunchDescription` children + child
  processes when at top level (not inside a scoped `GroupAction`); does NOT reach separate CLI/shell
  invocations. `https://github.com/ros2/launch/blob/humble/launch/launch/actions/set_environment_variable.py`
- **ROS2 launch — `IncludeLaunchDescription` args:** an undeclared arg is silently set as a
  `LaunchConfiguration` (no error); a child `DeclareLaunchArgument` with no `default_value` is
  mandatory. `https://github.com/ros2/launch/blob/humble/launch/launch/actions/include_launch_description.py`
- **ROS2 launch — `prefix` missing binary:** raises `FileNotFoundError` at spawn; node marked failed.
  `https://github.com/ros2/launch/blob/humble/launch/launch/actions/execute_process.py`
- **Velodyne `velodyne_transform_node` arg-swap (CONFIRMED, 2.5.1 = apt `ros-humble-velodyne`):**
  `transform.cpp` calls `configure(min,max,target_frame,fixed_frame)` while the base declares
  `configure(...,fixed_frame,target_frame)` — so YAML `target_frame` drives the de-skew and YAML
  `fixed_frame` drives the output `frame_id`. The project's `velodyne.yaml`
  (`fixed_frame:"velodyne"`, `target_frame:"odom"`) therefore correctly de-skews against odom while
  keeping output `frame_id "velodyne"`. The de-skewing node is `velodyne_transform_node` (NOT
  `velodyne_convert_node`). `https://github.com/ros-drivers/velodyne/blob/2.5.1/velodyne_pointcloud/include/velodyne_pointcloud/datacontainerbase.hpp#L189-L199`
- **ZED v5.2.2 `zed_camera.launch.py`:** pass ONLY `camera_name`+`camera_model`+`serial_number`; a
  non-empty `namespace` overwrites `node_name` with `camera_name` (tree collapse). `camera_model`
  required (`zedx` valid). `serial_number` is injected AFTER `ros_params_override_path`, so a serial
  in the override YAML is ignored. `publish_tf=false` stops `odom->camera_link`; `publish_urdf=false`
  stops the wrapper's own RSP. `https://github.com/stereolabs/zed-ros2-wrapper/blob/v5.2.2/zed_wrapper/launch/zed_camera.launch.py`
- **Nav2 executable/package names (Humble):** `nav2_controller/controller_server`,
  `nav2_planner/planner_server`, `nav2_smoother/smoother_server`, `nav2_behaviors/behavior_server`,
  `nav2_velocity_smoother/velocity_smoother`, `nav2_bt_navigator/bt_navigator`,
  `nav2_route/route_server`, `nav2_lifecycle_manager/lifecycle_manager`.
  `https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/launch/navigation_launch.py`
- **Nav2 lifecycle `node_names`:** ordered bringup/reverse shutdown, but with `autostart=true` all
  listed nodes activate regardless of order; a listed node that fails to activate stalls the
  sequence. `https://docs.nav2.org/configuration/packages/configuring-lifecycle.html`
- **RewrittenYaml:** `param_rewrites` only **overwrites existing** keys — never injects new ones.
  ✅ The project's `nav2_params_humble.yaml` *does* pre-declare `default_nav_to_pose_bt_xml`,
  `default_nav_through_poses_bt_xml` (`:27-28`) + `graph_filepath` (`:651`), so the launch overrides
  apply. `https://github.com/ros-navigation/navigation2/blob/humble/nav2_common/nav2_common/launch/rewritten_yaml.py`
- **MPPI plugin string:** `nav2_mppi_controller::MPPIController` (identical Humble/Jazzy).
  `https://docs.ros.org/en/humble/p/nav2_mppi_controller/`
- **robot_localization:** EKF executable `ekf_node` (`ekf_filter_node*` is the node *name*); GPS
  transform `navsat_transform_node`; `broadcast_cartesian_transform:false` (current name;
  `broadcast_utm_transform` deprecated alias) correctly prevents the dual-EKF `map->odom` TF loop.
  IMU subscriber base name is `imu` — a self-identity remap on `imu/data` is the documented #749
  footgun (the project's launches were not flagged for this). `https://github.com/cra-ros-pkg/robot_localization/blob/ros2/src/navsat_transform.cpp`
- **foxglove_bridge:** package + executable both `foxglove_bridge`; `port` int default 8765; latched
  `transient_local` topics auto-matched (no whitelist needed); `use_compression` is a ROS1-only
  param (harmless no-op on ROS2). `https://docs.ros.org/en/humble/p/foxglove_bridge/standard_docs/README.html`
- **Xsens / NTRIP:** driver pkg `xsens_mti_ros2_driver`, exec `xsens_mti_node` (the launch `name=`
  remap is what binds the `xsens_mti_node:` YAML key). NTRIP pkg+exec both `ntrip`, node name
  `ntrip_client`; `port` is an INT (do not quote). `https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client/tree/ros2`

## Prioritized action list

1. **[competition-critical]** Reconcile the ZED front serial: if `42569280` is the verified front
   unit, update `CLAUDE.md`, `zed_front.yaml:5`, `avros.urdf.xacro:214`, and rewrite the
   `sensors.launch.py:169` right-camera TODO. (Issue #10)
2. **[competition-critical]** Unify the `enable_ntrip` default: set `localization.launch.py:58` to
   `'false'` (matching `sensors.launch.py` + §I.2), or document the deliberate `true`. (Issue #7)
3. **[nice-to-have]** Set `RMW_IMPLEMENTATION` + `CYCLONEDDS_URI` explicitly at the top of
   `costmap_test.launch.py`, and branch it on `$ROS_DISTRO` to load `nav2_params_humble.yaml`.
   (Issue #1 + cross-cutting)
4. **[nice-to-have]** Add `<exec_depend>xterm</exec_depend>` to `avros_bringup/package.xml` +
   `avros_sim/package.xml` (or document `apt install xterm`). (Issue #8)
5. **[nice-to-have]** Add `zed_wrapper` as an `exec_depend` of `avros_sim`, or gate the `zed_macro`
   include in `avros.urdf.xacro:164` behind a `xacro:arg`. (Issue #5)
6. **[nice-to-have, sim]** In `nav2_sim_overrides.yaml`, disable `semantic_layer` and either give the
   Webots vehicle a diff-drive model to match the Humble MPPI controller or document the sim as
   Jazzy/RPP-only (the RPP-shaped `FollowPath` overrides are dead under MPPI). (Issues #3, #4, #6)
7. **[nice-to-have, sim]** Publish `/wheel_odom` from `avros_vehicle_driver.py` so the sim odom EKF
   translates. (Issue #12)
8. **[optional]** Drop the unused `smoother_server` + `route_server` from `navigation.launch.py`'s
   lifecycle list (default BT uses neither). (Issue #2)
9. **[cosmetic]** Doc sweep: `bt_xml` description (`navigation.launch.py:128-129,59-62`), "Teensy
   UDP" (`:5`), `/dev/ttyACM0` (`yaw_diag.launch.py:111`), "HSV pipeline" → "adaptive", Ackermann/RPP
   BT comments, ZED cameras in the `sensors.launch.py` docstring, the dead `camera_name` arg in
   `perception.launch.py`. (Issues #9, #11, #13 + nits)
