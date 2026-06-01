# IGVC AutoNav Competition Tuning — Rationale + How-To (2026-06-01)

This document explains the Nav2 tuning that fixes the **"robot freezes behind a barrel
instead of going around it"** failure, why every value is grounded in official Nav2 /
STVL / IGVC documentation, and exactly how to launch and validate it.

It accompanies two new files:

- `src/avros_bringup/config/nav2_params_igvc_autonav.yaml` — the tuned Nav2 params
- `src/avros_bringup/config/navigate_igvc_autonav_2026.xml` — the tuned recovery BT

The live field-test files (`nav2_params_humble.yaml`, `navigate_igvc_autonav_humble.xml`)
are left untouched so you can A/B against them.

> **⚠️ Baseline reconciliation (read this).** These duplicates were first generated from
> the **laptop-committed** config, then re-baselined on 2026-06-01 to match the **Jetson
> live (uncommitted)** config — what the robot actually runs. The Jetson diverged from the
> committed repo in 3 spots, now reconciled INTO this competition config:
>
> | Param | Laptop committed | Jetson live (robot) | Competition config | Note |
> |---|---|---|---|---|
> | `FollowPath.vx_max` | 1.5 | **0.7** | **0.7** | 1.5 was never deployed (reverted uncommitted); 0.7 is the field-validated value, kept by decision 2026-06-01. Raise only after an asphalt speed test. |
> | STVL `decay_acceleration` (×2) | 2.0 | **0.5** | **0.5** | Jetson's slower frustum decay keeps barrels marked while you maneuver — better for IGVC. |
> | BT `RateController hz` | 1.0 | **3.0** | **3.0** | Faster replan; within planner's 5 Hz capacity. |
>
> So the "old → new" deltas in the table below are stated against the **Jetson live** baseline
> = what the robot runs today. If you instead diff against the committed `nav2_params_humble.yaml`
> you'll see three extra lines (vx_max, decay_acceleration) that are merely the live-vs-committed
> drift, not part of the freeze fix.

---

## 1. PROBLEM — the freeze, in plain language

In field testing the robot would drive up to a barrel that was sitting on its planned
line, **stop, and sit there** until the run timed out. It never tried to bulge left or
right around the barrel, and it never recovered. Three things combined to cause this:

1. **The inflation gradient was a cliff, not a ramp.** The local and global costmaps
   were set to `inflation_radius = 0.4 m`. The robot's footprint has an *inscribed
   radius* of 0.2794 m (the rear edge), and inflation cost only starts decaying
   *outside* that inscribed disc. So the graded "cost ramp" the planner can steer down
   was only `0.4 − 0.2794 = 0.12 m` wide — essentially a vertical wall of cost with a
   tiny lip. MPPI rolls out ~1000 candidate trajectories and takes a **softmax-weighted
   average** of them. With no early gradient, the on-path samples all *collide* (cost
   1e6) while the go-around samples are barely cheaper, so the cost field across samples
   goes **flat**. A flat field makes the softmax near-uniform, the weighted-average
   command collapses toward the zero-mean noise, and **the commanded velocity goes to
   zero — the freeze.** The global planner (Navfn) plans the robot as a *point* on that
   same thin field, so it also just drew the shortest line clipping the barrel and
   punted all the go-around to MPPI, which couldn't do it.

2. **Recovery could never fire.** The progress checker's
   `movement_time_allowance` was **120 s** — a frozen robot has to sit still for *two
   full minutes* before Nav2 even notices it isn't making progress. The IGVC
   Blocking/Hold-up DQ fires at **60 s**, so the robot was always disqualified before
   recovery ever triggered. On top of that the controller's `failure_tolerance` was
   unset (= 0.0), so a single transient MPPI "fail to compute" aborted FollowPath
   outright.

3. **The BT was still wearing its field-test clothes.** The behavior tree shipped with
   the field-test overrides: outer `Timeout msec=120000` (the comment literally says
   *"REVERT to 45000 for competition"*) and `number_of_retries=8`. So even when a
   failure did bubble up, the whole-action watchdog gave it 2 minutes instead of the
   45 s it needs to leave a 15 s margin before the 60 s DQ.

Net effect: **a barrel on the path froze the robot, and nothing ever un-froze it before
the DQ clock ran out.**

---

## 2. THE FIX, AT A GLANCE

Five load-bearing changes, in priority order:

1. **Local `inflation_radius` 0.4 → 0.85 m.** Restores a real, descendable cost basin
   down the centerline of a 5 ft (1.524 m) gap so MPPI has an early-steer gradient and
   starts veering ~0.85 m *before* the barrel instead of flat-field-freezing. (This is
   the single most important change — it is the only early-steer mechanism today,
   because the planned `nav2_collision_monitor` is not wired yet.)

2. **Global `inflation_radius` 0.4 → 0.65 m.** Navfn plans as a point on the inflated
   map, so this is its *only* width model — it now bows the global path around barrels
   with margin before MPPI ever sees it.

3. **`movement_time_allowance` 120 → 12 s + `failure_tolerance` unset → 0.5 s.**
   Together these finally let recovery fire on a true stall (~12 s, leaving ~48 s before
   DQ) while never false-tripping a legit slow crawl; `failure_tolerance` absorbs
   single-frame MPPI glitches first.

4. **MPPI go-around tune: `PathAlignCritic` 16 → 12, `CostCritic` 6 → 5, `wz_std`
   0.4 → 0.5.** Loosens the path-stapling that penalized off-path go-around samples,
   eases the centerline-pull that flattened the gap (valid *only* because inflation is
   now 0.85), and widens the yaw-rate sampling so a committed escape arc actually gets
   sampled.

5. **BT reverted to competition: `Timeout` 120000 → 45000, `retries` 8 → 4, RoundRobin
   reordered to forward-crawl-before-blind-reverse, `RateController` 1 → 2 Hz.** Bounds
   the damage of a freeze to under the 60 s DQ and escalates recovery gentlest-to-blindest.

---

## 3. FULL DECISION TABLE

`nav2_params` = `src/avros_bringup/config/nav2_params_igvc_autonav.yaml`.
`bt_xml` = `src/avros_bringup/config/navigate_igvc_autonav_2026.xml`.

| Param | File | Old → New | Why (IGVC rule) | Doc citation |
|---|---|---|---|---|
| `local_costmap.inflation_layer.inflation_radius` | nav2_params | 0.4 → 0.85 | IGVC minimum 5 ft passage; 10 ft lanes | https://docs.nav2.org/configuration/packages/costmap-plugins/inflation.html |
| `global_costmap.inflation_layer.inflation_radius` | nav2_params | 0.4 → 0.65 | IGVC 10 ft lanes and randomized barrels | https://docs.nav2.org/configuration/packages/configuring-navfn.html |
| `FollowPath.PathAlignCritic.cost_weight` | nav2_params | 16.0 → 12.0 | Barrels placed after the plan; not going around = Hold-up >60 s = End of Run; lane-crossing E-stop | https://docs.nav2.org/configuration/packages/configuring-mppic.html |
| `FollowPath.CostCritic.cost_weight` | nav2_params | 6.0 → 5.0 | IGVC 5 ft passages must stay low-cost at center | https://docs.ros.org/en/humble/p/nav2_mppi_controller/__README.html |
| `FollowPath.PathAngleCritic.mode` | nav2_params | remove line, add `forward_preference: true` | Consistency on the Humble binary; forward_preference keeps the front sensor on the lane | https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_mppi_controller/src/critics/path_angle_critic.cpp |
| `FollowPath.wz_std` | nav2_params | 0.4 → 0.5 | IGVC arc around randomized barrels in 5 ft passages without freezing | https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/README.md |
| `FollowPath.temperature` | nav2_params | 0.3 → 0.25 | IGVC barrels often dead-center causing symmetric escape indecision | https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_mppi_controller/src/optimizer.cpp |
| `FollowPath.ax_max` | nav2_params | 0.4 → 0.4 (UNCHANGED — workflow's 0.8 reverted: actuator caps accel at 0.3 m/s², smoother inert, freeze isn't accel-limited) | consistency with actuator + velocity_smoother | https://github.com/ros-navigation/navigation2/issues/5465 |
| `local_costmap.stvl_layer.velodyne_points.max_obstacle_height` | nav2_params | 0.8 → 1.0 | IGVC ramps up to 15%; full-height drum; Crash penalty −10 ft and E-stop | https://raw.githubusercontent.com/SteveMacenski/spatio_temporal_voxel_layer/ros2/spatio_temporal_voxel_layer/src/measurement_buffer.cpp |
| `controller_server.progress_checker.movement_time_allowance` | nav2_params | 120.0 → 12.0 | IGVC Blocking/Hold-up >60 s; recovery must fire well under 60 s | https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_controller/plugins/simple_progress_checker.cpp |
| `controller_server.failure_tolerance` | nav2_params | unset (0.0) → 0.5 | IGVC 60 s windows; 0.5 s prevents recovery churn dragging the 1 mph average | https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_controller/src/controller_server.cpp |
| `velocity_smoother.max_velocity` | nav2_params | [2.0, 0.0, 1.0] → [1.5, 0.0, 1.5] | IGVC max 5 mph / min 1 mph avg; uncapping ω keeps avoidance turns crisp | https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_velocity_smoother/src/velocity_smoother.cpp |
| `velocity_smoother.min_velocity` | nav2_params | [-0.5, 0.0, -1.0] → [-0.6, 0.0, -1.5] | Consistency; smoother matched to actuator and MPPI; reverse arcs kept off the line | https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_velocity_smoother/src/velocity_smoother.cpp |
| `velocity_smoother.max_accel` | nav2_params | [0.5, 0.0, 1.0] → [0.3, 0.0, 1.2] | Consistency; smoother accel matched to actuator; predictable reach to cruise | https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_velocity_smoother/src/velocity_smoother.cpp |
| `velocity_smoother.max_decel` | nav2_params | [-1.0, 0.0, -1.0] → [-1.3, 0.0, -1.2] | IGVC Crash penalty −10 ft and E-stop; pothole hit is End of Run | https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_velocity_smoother/src/velocity_smoother.cpp |
| `MainTree` `Timeout msec` | bt_xml | 120000 → 45000 | IGVC Blocking/Hold-up >60 s; whole-action watchdog must abort under 60 s | https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html |
| `NavigateRecovery` `RecoveryNode number_of_retries` | bt_xml | 8 → 4 | IGVC 1 mph average caps dead time; recovery under the 45 s Timeout | https://github.com/ros-navigation/navigation2/blob/humble/nav2_behavior_tree/plugins/control/recovery_node.cpp |
| `LaneAwareRecoveryActions` RoundRobin child order | bt_xml | ClearAroundRobot → Wait → BackUp 0.3 → DriveOnHeading 0.15 → ClearAroundRobot → Wait → DriveOnHeading 0.30 → BackUp 0.25 | IGVC Crossing an Internal Line is E-stop End of Run; forward crawl before blind reverse; tactile sensors forbidden | https://github.com/ros-navigation/navigation2/blob/humble/nav2_behaviors/include/nav2_behaviors/plugins/drive_on_heading.hpp |
| `CrawlForward` DriveOnHeading dist/speed/time | bt_xml | 0.15 / 0.10 / 10 → 0.30 / 0.15 / 8 | IGVC 1 mph average; a forward nudge adds down-course distance | https://github.com/ros-navigation/navigation2/blob/humble/nav2_behaviors/include/nav2_behaviors/plugins/drive_on_heading.hpp |
| `ShortBackup` BackUp dist/speed/time | bt_xml | 0.3 / 0.10 / 25 → 0.25 / 0.10 / 5 | IGVC Crossing an Internal Line; short blind reverse limits rear-lane crossing; 5 s keeps under the DQ | https://github.com/ros-navigation/navigation2/blob/humble/nav2_behaviors/plugins/back_up.cpp |
| `WaitForSemanticDecay` Wait wait_duration | bt_xml | 2 → 3 | IGVC Blocking >60 s; every Wait counts against the stall clock | https://docs.nav2.org/configuration/packages/bt-plugins/actions/Wait.html |
| `ClearAroundRobot` ClearCostmapAroundRobot reset_distance | bt_xml | 3.0 → 1.5 | IGVC Crossing an Internal Line is E-stop End of Run; surgical clear preserves the lethal lane cells | https://github.com/ros-navigation/navigation2/blob/humble/nav2_behavior_tree/include/nav2_behavior_tree/plugins/action/clear_costmap_service.hpp |
| `ComputePathToPose` RateController hz | bt_xml | 1.0 → 2.0 | IGVC repetitive barrel series; faster re-routing reduces stalls under the 60 s DQ | https://docs.nav2.org/behavior_trees/overview/nav2_specific_nodes.html |

**Note on the actual cost_scaling_factor:** the inflation math above is computed against
the **live `cost_scaling_factor = 2.5`** (a 2026-05-30 change), *not* the 3.0 an earlier
draft cited. At csf 2.5 the 0.85 m basin is slightly softer (good — earlier steering)
and still supports 0.85. `cost_scaling_factor` is **left unchanged at 2.5** on both
costmaps.

---

## 4. WHY EACH IS GROUNDED IN OFFICIAL DOCS

- **InflationLayer (the freeze fix).** Per the Nav2 inflation docs, cost decays as
  `252·exp(−cost_scaling_factor·(d − inscribed_radius))` and the layer's purpose is "a
  consistent/smooth potential field" for cost-aware planners to descend — not a thin
  wall. With inscribed 0.2794 m the 0.4 m radius gave only 0.12 m of ramp; 0.85 m
  restores a monotonic V-basin (≈253 at each wall, ≈75 minimum at the 0.762 m centerline
  of a 5 ft gap, FREE by 0.85 m). 0.85 ≥ inscribed 0.2794 (clears the Nav2
  `inflation < inscribed` ERROR) and < circumscribed 0.913.
  *(docs.nav2.org/.../inflation.html)*

- **Navfn global planner.** The Nav2 Navfn docs make clear it plans the robot as a
  **point** on the inflated costmap — it ignores the footprint polygon — so global
  `inflation_radius` is its only width model. Raising it 0.4 → 0.65 gives Navfn a 0.37 m
  graded basin to bow the global path around barrels with margin. *(docs.nav2.org/.../configuring-navfn.html)*

- **MPPI controller + critics.** The Nav2 MPPI tuning guide states *not over-weighting
  the path-align critic lets the robot deviate around obstacles not seen at plan time* —
  exactly the IGVC case where barrels are placed after the plan. Hence `PathAlignCritic`
  16 → 12 and `CostCritic` 6 → 5 (the latter valid only with the restored 0.85 gradient).
  `wz_std` is the README's most important exploration knob for a 0 m-turn chassis (the
  go-around *is* a yaw excursion) → 0.4 → 0.5; `temperature` 0.3 → 0.25 sharpens
  commitment to the cheapest go-around on a dead-center barrel. *(docs.nav2.org/.../configuring-mppic.html;
  humble nav2_mppi_controller README + optimizer.cpp)*

- **MPPI accel sampling (`ax_max`) — KEPT at 0.4 (the workflow's 0.8 was reverted).**
  Nav2 issue #5465 flags `|ax_min|/ax_max` asymmetry, and 0.8 would lower it to 1.875×.
  But the actuator hard-caps real forward accel at **0.3 m/s²** (12 V brown-out safety),
  and the velocity_smoother is **inert** on this stack (it publishes `/cmd_vel_smoothed`,
  which nothing consumes — the actuator subscribes `/cmd_vel` directly), so there is **no
  downstream re-clamp**: `ax_max 0.8` would feed MPPI accels the chassis cannot deliver
  (2.67× the real cap) — the exact "believes-in-but-cannot-execute" tracking lag the
  nav2_params accel-block warns against, and inconsistent with the velocity_smoother
  `max_accel` 0.3 set in this same table. The freeze is a cost-gradient problem, not
  accel-limited. `ax_max` stays **0.4** (matched to the actuator with 1.33× headroom).
  Revisit the #5465 asymmetry only as a *paired* `ax_max`+`ax_min` change after the 12 V
  rail fix raises the actuator accel cap. *(github.com/.../issues/5465; actuator_params.yaml)*

- **Controller server (recovery enabling).** The Humble `simple_progress_checker.cpp`
  treats `movement_time_allowance` as **seconds**; at 120 s a frozen robot never trips a
  progress failure. 12 s fires recovery ~48 s before the 60 s DQ and never false-trips a
  0.447 m/s (1 mph) crawl. `failure_tolerance` (controller_server.cpp) defaults to 0.0 =
  abort on the first MPPI compute failure; 0.5 s lets MPPI re-sample ~10 cycles and
  self-clear first. *(humble nav2_controller plugins + controller_server.cpp)*

- **STVL (LiDAR marking).** `max_obstacle_height` is a global-frame z-cutoff
  (measurement_buffer.cpp). The tight [0.4, 0.8] band clipped a ~0.9 m construction
  drum top; widening to 1.0 marks the full drum body. Phantoms are still cleared by
  `voxel_decay` + `decay_acceleration` + frustum padding. *(STVL ros2 measurement_buffer.cpp)*

- **Velocity smoother.** Per velocity_smoother.cpp the ω ceiling 1.0 silently throttled
  turning below both the actuator (1.5) and MPPI `wz_max` (1.9); matching [1.5, 0, 1.5]
  uncaps avoidance turns, and accel/decel are re-synced to the actuator caps (0.3 / 1.3
  / 1.2). **These four are inert today** (see §6/§7) — pre-positioned for the future
  cmd_vel chain. *(humble nav2_velocity_smoother/velocity_smoother.cpp)*

- **Behavior tree.** Per the Nav2 BT docs the `<Timeout>` decorator wraps the whole
  NavigateToPose action; the field-test 120000 had no DQ clock and the file itself is
  tagged *"REVERT to 45000 for competition."* `RecoveryNode` ticks exactly one RoundRobin
  child per failure (recovery_node.cpp), so `number_of_retries = 4` makes the full
  4-child ladder reachable exactly once. `DriveOnHeading` has an always-on forward
  collision check; `BackUp` reads rear cells the robot has no sensor for (reverses
  blind) — so the RoundRobin is reordered to **forward-crawl before blind-reverse**.
  No `Spin` anywhere (a rectangular-footprint sweep would cross the 3 in line = II.4
  E-stop). *(docs.nav2.org BT docs + humble nav2_behaviors back_up.cpp / drive_on_heading.hpp)*

---

## 5. HOW TO USE

### 5.1 The launch command

```bash
ros2 launch avros_bringup navigation.launch.py \
  bt_xml:=navigate_igvc_autonav_2026.xml \
  enable_perception:=true \
  enable_zed_front:=true \
  enable_velodyne:=true
```

- `bt_xml:=navigate_igvc_autonav_2026.xml` selects the tuned recovery BT. `bt_xml` **is
  already a real launch arg** (it is rewritten into `default_nav_to_pose_bt_xml` /
  `default_nav_through_poses_bt_xml`), so the BT swap needs **no code edit**.
- `enable_perception:=true` turns on `avros_perception` (the camera lane layer feeding
  the kiwicampus semantic layer). **It requires at least one camera**, so you must also
  pass `enable_zed_front:=true` (perception defaults to false and the front ZED defaults
  to false in the launch file).
- `enable_velodyne:=true` is already the default, but pass it explicitly for the
  competition stack so the LiDAR STVL barrel layer is up.

### 5.2 The one catch — the params file is hardcoded

`navigation.launch.py` selects the params file by `$ROS_DISTRO` and **hardcodes
`nav2_params_humble.yaml`** for Humble (lines ~45-53):

```python
    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    if ros_distro == 'humble':
        nav2_config = os.path.join(pkg_dir, 'config', 'nav2_params_humble.yaml')
        ...
        default_bt = 'navigate_igvc_autonav_humble.xml'
```

There is **no `nav2_params` launch arg**, so passing `bt_xml:=...` alone gets you the new
BT but **still the old field-test params**. To run the tuned params, do **one** of:

**(a) One-line edit (simplest — point the hardcoded line at the new file):**

```python
        nav2_config = os.path.join(pkg_dir, 'config', 'nav2_params_igvc_autonav.yaml')
```

(line ~46, inside the `if ros_distro == 'humble':` branch). Rebuild
(`colcon build --symlink-install --packages-select avros_bringup`) or rely on
`--symlink-install` so the share copy tracks the source.

**(b) Add a launch arg (cleaner, no per-competition edit):**

```python
    # near the other LaunchConfigurations
    nav2_params_file = LaunchConfiguration('nav2_params')
    # in generate_launch_description(), add to the returned LaunchDescription:
    DeclareLaunchArgument(
        'nav2_params',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params_humble.yaml'),
        description='Nav2 params YAML (use nav2_params_igvc_autonav.yaml for competition)'),
    # then change the RewrittenYaml source:
    #   source_file=nav2_params_file,   # was: source_file=nav2_config
```

then launch with `nav2_params:=$(ros2 pkg prefix
avros_bringup)/share/avros_bringup/config/nav2_params_igvc_autonav.yaml` (or the
in-source path with `--symlink-install`).

Until one of these is applied, **the tuned params are not active** — only the BT is.

---

## 6. nav2_collision_monitor — DOCUMENTED AS OPTIONAL (not added)

The decision **keeps the thin-inflation + collision-monitor architecture as the *end*
state**, but the monitor is **still pending / not wired** (per the config comments at
nav2_params lines ~482/622). Because of that, this tuning leans on the **wider inflation
gradient (0.85)** as the only early-steer mechanism *today*, and the monitor is **NOT
added to `navigation.launch.py`** here.

When you do add it (recommended before competition, and the trigger to re-evaluate
dialing local inflation back down toward robot half-width), here is the ready-to-paste,
Humble-correct setup. **Marked OPTIONAL — do not enable blindly; field-tune the polygon
widths and `slowdown_ratio` first.**

### 6.1 The cmd_vel wiring (critical)

The actuator subscribes to `/cmd_vel` directly. The monitor must sit **between** the
Nav2 output and the actuator: feed it from the velocity smoother's output
(`/cmd_vel_smoothed`) and have it republish the *gated* command on `/cmd_vel`. **If you
leave the controller publishing the same `/cmd_vel` the actuator reads, the monitor is
bypassed.**

Canonical chain: `controller → /cmd_vel → velocity_smoother → /cmd_vel_smoothed →
collision_monitor → /cmd_vel → actuator`.

### 6.2 Param block (OPTIONAL — Humble syntax, base_link frame, LiDAR-fed)

```yaml
collision_monitor:
  ros__parameters:
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    cmd_vel_in_topic: "cmd_vel_smoothed"   # the smoother's output
    cmd_vel_out_topic: "cmd_vel"           # what the actuator reads
    transform_tolerance: 0.5
    source_timeout: 1.0                    # NODE-level on Humble (no per-source)
    base_shift_correction: true
    stop_pub_timeout: 1.5
    polygons: ["SlowZone", "StopZone"]     # most-aggressive wins when both fire
    SlowZone:
      type: "polygon"
      points: [1.3, 0.55, 1.3, -0.55, -0.30, -0.55, -0.30, 0.55]  # FLAT array on Humble
      action_type: "slowdown"
      max_points: 3            # HUMBLE NAME (rolling uses min_points = max_points+1)
      slowdown_ratio: 0.5      # keep cruise > 0.447 m/s (1 mph) avg
      visualize: true
      polygon_pub_topic: "polygon_slowdown"
      enabled: true
    StopZone:
      type: "polygon"
      points: [0.55, 0.45, 0.55, -0.45, -0.30, -0.45, -0.30, 0.45]
      action_type: "stop"      # Humble has stop|slowdown|approach only — NO 'limit'
      max_points: 3
      visualize: true
      polygon_pub_topic: "polygon_stop"
      enabled: true
    observation_sources: ["velodyne"]      # LiDAR only — NOT the ZED lane mask
    velodyne:
      type: "pointcloud"
      topic: "/velodyne_points"
      min_height: 0.25         # reject flat ground + painted potholes
      max_height: 0.9          # capture drum body
      enabled: true
```

Humble gotchas baked in above: `max_points` (not `min_points`); flat `points` array
(not nested `[[x,y],...]`); `stop`/`slowdown`/`approach` only (no `limit`/VelocityPolygon);
node-level `source_timeout`.

### 6.3 Launch wiring (OPTIONAL — add to navigation.launch.py)

```python
# 1) Remap the velocity_smoother OUTPUT off /cmd_vel so it doesn't reach the actuator
#    directly. In the nav2_servers Node for 'velocity_smoother', add:
#        remappings=[('cmd_vel_smoothed', 'cmd_vel_smoothed')]  # (already its default)
#    and ensure the controller publishes /cmd_vel into the smoother (Nav2 default).

# 2) Add the collision monitor node:
Node(
    package='nav2_collision_monitor',
    executable='collision_monitor',
    name='collision_monitor',
    parameters=[configured_params],   # the block above must live in the params YAML
    output='screen',
    respawn=True,
    respawn_delay=2.0,
),

# 3) Add it to the lifecycle manager's ordered node list so it is configured+activated:
nav2_servers = [
    ('nav2_controller', 'controller_server'),
    ('nav2_smoother', 'smoother_server'),
    ('nav2_planner', 'planner_server'),
    ('nav2_route', 'route_server'),
    ('nav2_behaviors', 'behavior_server'),
    ('nav2_velocity_smoother', 'velocity_smoother'),
    ('nav2_collision_monitor', 'collision_monitor'),   # <-- ADD, before bt_navigator
    ('nav2_bt_navigator', 'bt_navigator'),
]
# lifecycle_nodes is derived from nav2_servers, so it is picked up automatically.
```

After wiring, confirm `/cmd_vel` is now sourced by `collision_monitor` (not the
controller) with `ros2 topic info /cmd_vel --verbose`.

---

## 7. COMPETITION GO/NO-GO CHECKLIST

Every value that differs between the field-test build and the competition build, plus
the two hard preconditions:

**Params (must be the tuned file — see §5.2):**
- [ ] `nav2_params_igvc_autonav.yaml` is the active params (one-line edit or launch arg applied)
- [ ] local `inflation_radius` = **0.85** (field-test: 0.4)
- [ ] global `inflation_radius` = **0.65** (field-test: 0.4)
- [ ] `progress_checker.movement_time_allowance` = **12.0** s (field-test: 120.0)
- [ ] `controller_server.failure_tolerance` = **0.5** (field-test: unset/0.0)
- [ ] `PathAlignCritic.cost_weight` = **12.0** (field-test: 16.0)
- [ ] `CostCritic.cost_weight` = **5.0** (field-test: 6.0) — *only valid with inflation 0.85; if you revert inflation, restore 6.0*
- [ ] `wz_std` = **0.5**, `temperature` = **0.25** (field-test: 0.4 / 0.3). `ax_max` stays **0.4** (unchanged — see note; do NOT set 0.8 until the 12 V rail fix raises the actuator accel cap)
- [ ] local STVL `max_obstacle_height` = **1.0** (field-test: 0.8)
- [ ] `PathAngleCritic.mode` line removed, `forward_preference: true` present
- [ ] `cost_scaling_factor` still **2.5** on both costmaps (do NOT raise)

**BT (must be `navigate_igvc_autonav_2026.xml`):**
- [ ] `bt_xml:=navigate_igvc_autonav_2026.xml` on the launch line
- [ ] outer `Timeout msec` = **45000** (field-test: 120000)
- [ ] `NavigateRecovery number_of_retries` = **4** (field-test: 8)
- [ ] RoundRobin order = ClearAroundRobot → Wait(3) → DriveOnHeading(0.30/0.15/8) → BackUp(0.25/0.10/5)
- [ ] `ClearCostmapAroundRobot reset_distance` = **1.5** (field-test: 3.0)
- [ ] `RateController hz` = **2.0** (field-test: 1.0)
- [ ] BT header comments updated (Wait now 3 s, RoundRobin order) — doc accuracy only

**Hard preconditions:**
- [ ] **RTK FIXED via MDOT CORS** before relying on map-frame goals. `bt_navigator.global_frame`,
      `global_costmap.global_frame`, and goal stamps are all `map` on the premise RTK keeps map
      cm-accurate. **If RTK is unavailable/forbidden at the venue, exercise the documented fallback:
      revert ALL THREE to `odom` together** — a half-move re-introduces map→odom extrapolation aborts.
- [ ] **cmd_vel Hz re-verification:** `ros2 topic hz /cmd_vel` must read **≥ 18 Hz** under full field
      load (perception + STVL + MPPI) with `batch_size 500 × time_steps 56`. If it dips below 18,
      drop `batch_size 500 → 400` **before** touching `time_steps`. The `RateController 1 → 2 Hz`
      replan runs in the separate planner_server (5 Hz capacity) and should not steal MPPI cycles —
      verify on the Jetson. **Never run RViz on the Jetson during this check** (it starves the loop).

**Build/run hygiene:**
- [ ] `colcon build --symlink-install --packages-select avros_bringup` (or full build) after any edit
- [ ] `enable_perception:=true enable_zed_front:=true enable_velodyne:=true` on the launch line
- [ ] No RViz on the Jetson during nav — use laptop Foxglove (foxglove_bridge is in the launch)

---

## 8. FIELD A/B TEST PLAN

Mirror the obstacle-stall RCA A/B sequence — change **one variable at a time**, single
barrel on a straight, record `/cmd_vel`, `/behavior_tree_log`, and odom each run.

**Test 0 — Baseline reproduction (field-test build).**
Launch with `nav2_params_humble.yaml` + `navigate_igvc_autonav_humble.xml`. Put a barrel
dead-center on the planned line. **Expected (the bug):** robot drives up, freezes, sits
~120 s, never recovers. This is the control case — confirm the freeze still reproduces.

**Test 1 — Inflation only.**
Switch to the tuned params but **temporarily revert** the two `inflation_radius` lines to
their tuned values *only* (local 0.85, global 0.65), leaving everything else at field-test
values. **Expected:** the robot now **steers early and bulges around** the barrel — no
freeze on approach. This isolates the load-bearing fix. (If it still freezes here,
inflation is not the whole story — check footprint inscribed = 0.2794 and that the
InflationLayer is ordered after the semantic layer.)

**Test 2 — Recovery enabling.**
Add `movement_time_allowance = 12` + `failure_tolerance = 0.5` (still field-test BT).
Block the robot in a *genuine dead-end* (barrel + lane lines, no go-around). **Expected:**
progress failure fires at ~12 s, recovery branch runs, robot is no longer wedged until
the 60 s DQ. Confirm via `/behavior_tree_log` which recovery child fired.

**Test 3 — Full tuned params, field-test BT.**
All `nav2_params_igvc_autonav.yaml` changes, field-test BT. Re-run the dead-center barrel
**and** a 5 ft barrel-pair gap. **Expected:** clean early go-around on the single barrel;
threads the 5 ft gap down the centerline basin without false-stopping. Watch
`ros2 topic hz /cmd_vel ≥ 18`.

**Test 4 — Full tuned params + tuned BT.**
Add `navigate_igvc_autonav_2026.xml`. Re-run the dead-end. **Expected:** recovery
escalates gentlest-to-blindest (ClearAround → Wait → forward-crawl → blind-reverse),
the 45 s whole-action watchdog bounds a true wedge, and the robot either escapes or the
action FAILs cleanly under 45 s (mission_manager can skip the waypoint) — **never** sits
to the 60 s DQ.

**Test 5 — Repetitive barrel slalom (integration).**
A short sinusoidal series of barrels in a 10 ft lane. **Expected:** sustained go-around
without freeze, cruise stays > 0.447 m/s (1 mph) average, `/cmd_vel` holds ≥ 18 Hz, no
lane-line crossing. This is the dress rehearsal for the AutoNav course.

If any test regresses, revert that one variable and re-run — the single-variable
ordering tells you exactly which change is responsible.
