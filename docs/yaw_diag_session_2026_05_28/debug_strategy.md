# IGVC AutoNav Debug & Test Strategy — 2026-05-28 PM

Synthesized from sequential-thinking analysis + 4 parallel research agents (RoboJackets architecture, Nav2 diagnostic tooling, EKF/GPS fusion, bag forensics tooling). Replaces ad-hoc tuning with a bottom-up, isolation-first test plan.

## TL;DR

After 10+ hours of testing today, the IGVC AutoNav stack has cascading failures across 5 subsystems. Tuning one knob exposes another problem. We're chasing symptoms. This doc:

1. **Catalogs every failure mode observed today**
2. **Defines 8 isolation-first test phases** — chassis up to end-to-end, each layer testable independently
3. **Codifies the fixes for the recurring problems** that we found by running multi-agent research:
   - **EKF/GPS divergence:** use `/ekf_filter_node_map/set_pose`, NOT `/datum`
   - **Costmap forensics:** specific Nav2 introspection commands per layer
   - **Bag forensics:** 9 new scripts + a cross-bag comparison workflow
   - **Architecture review:** what's borrowable from RoboJackets vs Sooner

The fundamental insight: **we were stuck in "tune live params, re-send goal" mode all afternoon.** That's the wrong loop. The right loop is: pick a phase, run its tests, identify which sublayer's broken, fix only that. Don't move on until the layer's tests pass.

---

## Failure mode catalog — what we observed today

Grouped by subsystem so each test phase has a clear scope.

### Perception
- HSV initially over-detected asphalt (V threshold too low for this surface) → entire path painted lethal
- Lane detection covers only near-field portion of tape due to ROI + V threshold
- ZED TF Exception: cloud `header.stamp` arrives older than buffer earliest → semantic layer rejects
- After TF errors flood, costmap data goes stale; planner sees old obstacles

### Localization (the biggest open issue)
- **EKF Mahalanobis gate locks out GPS forever once gap > ~6σ** — rejected updates never affect P, P never shrinks
- Hand-pushing chassis: wheel_odom reports nothing, IMU reports noise → EKF stays static while chassis physically moves → gap opens
- Map → odom drift accumulates 5-10 cm/s during motion
- `/datum` service re-frames GPS but does NOT touch EKF state → divergence persists in the new frame
- Two-EKF setup requires anchoring the **map** EKF specifically; the odom EKF can't be "lost"

### Planning
- NavfnPlanner can't compute path when goal is outside the global costmap
- Default 40 m × 40 m global was too small for 50 m goals; we bumped to 100 m
- Lifecycle cycle of global_costmap wipes accumulated obstacle data → planner sees all-unknown → goal aborts
- "Optimizer fail to compute path" ambiguous: is it Navfn returning nothing, or MPPI failing on a valid path?

### Control
- MPPI is CPU-starved above load ~8 (RViz on Jetson alone drives this)
- cmd_vel drops to 3-5 Hz instead of 20 Hz under load
- Actuator's 500 ms cmd_vel timeout fires → chassis stops/starts → "going and stopping" pattern
- Reduced `time_steps` to 25 (1.25 s horizon × 0.7 m/s = 0.88 m forward sight) — gained speed but lost lookahead

### Recovery / behavior tree
- `BackUp` recovery drives 1.5 m backward at 0.08 m/s **without checking rear costmap**
- No rear camera/LiDAR → can't see lanes behind chassis → **BackUp crosses lanes**
- `DriveOnHeading` fails with "Collision Ahead" when forward path has lethal cells nearby
- Recovery cascade fires aggressively when MPPI starves → goals abort during recovery loops

### Behavioral / cross-cutting
- Chassis went **22 m NE when goal was 5 m SE** (one of today's runs). Root cause not identified — could be EKF drift, bad global path, actuator bias, or wheel slip on the asphalt.
- Chassis "keeps turning right" pattern — possible directional bias somewhere

---

## 8-phase test plan

Bottom-up. Don't skip phases — each builds on the prior. Each phase has setup, tests, pass criteria, and **what failure mode it isolates.**

### Phase 0 — Forensics on today's bags (laptop, 60 min)

**Goal:** Understand what actually happened today before testing anything new. We have 700+ GB across 10 bags from today; the failure patterns are in there.

**Setup:** Add 9 new scripts under `scripts/` (per Agent 4):
- `bag_index.py` — bag info → JSON per bag
- `extract_goals.py` — NavigateToPose action goal table
- `extract_bt_log.py` — `/behavior_tree_log` to timeline CSV
- `extract_mask_stats.py` — per-frame lane coverage from `/perception/front/semantic_mask`
- `extract_costmap_age.py` — costmap header freshness
- `analyze_mppi_log.py` — regex `/tmp/nav_*.log` for Optimizer fails, loop-misses, TF errors
- `analyze_divergence.py` — `/odometry/global` vs `/odometry/gps` gap over time
- `analyze_goal_outcomes.py` — chassis trajectory vs commanded direction (dot product → "wrong direction" tag)
- `cross_bag_report.py` — markdown table comparing N bags

**Tests / forensic questions:**
1. **Goal outcome table** — for each goal: timestamp, status, commanded vs actual end position, Euclidean travel, "WRONG_DIR" tag if dot(goal_vec, chassis_displacement) < 0.3
2. **MPPI health timeline** — windowed cmd_vel rate; flag <15 Hz windows; correlate with Optimizer fail log lines
3. **EKF/GPS divergence over time** — per-second gap; flag when > 2 m for > 5 s
4. **Lane mask coverage** — per-frame lane pixel fraction + largest connected component
5. **Cross-bag** — compare 3 vision attempts side-by-side: goals_sent, succeeded, mean_cmd_vel_hz, mppi_opt_fails, ekf_gps_gap_max

**Pass criteria:** Per-bag dashboard PNG + cross-bag markdown table generated. We can name the dominant failure mode for each session.

**Rosbag2_py recipe** (no full bag extraction, streams from .db3):

```python
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=str(bagdir), storage_id='sqlite3'),
    rosbag2_py.ConverterOptions('', ''),
)
reader.set_filter(rosbag2_py.StorageFilter(topics=['/behavior_tree_log']))
type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
cls = get_message(type_map['/behavior_tree_log'])
while reader.has_next():
    topic, data, t = reader.read_next()
    msg = deserialize_message(data, cls)
    # process msg.event_log
```

Cross-bag workflow: extract on Jetson, `rsync --include='*.csv' --exclude='*'` to laptop.

### Phase 1 — Chassis baseline (no Nav2, no perception, no goal — pure cmd_vel)

**Goal:** Confirm the chassis hardware does what you tell it. If Phase 1 fails, Phases 2-8 are meaningless.

**Setup:**
- Stop nav2 stack
- Launch webui (handles its own actuator_node)
- Phone connected to https://100.93.121.3:8000

**Tests:**
1. **Forward delivery (tape measured)** — drive 5 m via WebUI throttle, measure with tape. Expected: 4.5-5.5 m physical (within 10%).
2. **Reverse delivery** — drive 5 m reverse. Same tolerance.
3. **In-place rotation** — drive 360° via steer. Verify yaw closes to within 10°.
4. **1×1 m closing square** — forward 1 m, right 90°, repeat 4× — verify closure < 0.2 m and yaw error < 15°.

**Pass criteria:** All 4 tests pass. Reference: `validation_results.md` morning, similar tests passed before.

**Failure modes isolated:** Actuator + wheel-odom + motor delivery + Mandow correction.

**Tools:** WebUI joystick. Optional: `bag record /cmd_vel /wheel_odom /imu/data /odometry/filtered` for post-hoc analysis.

### Phase 2 — Localization isolated (chassis + sensors only, no Nav2)

**Goal:** EKF/GPS alignment correctly tracks chassis. The afternoon's main failure mode was here.

**Setup:**
- Launch sensors only: `ros2 launch avros_bringup sensors.launch.py`
- Plus `localization.launch.py` (dual EKF + navsat_transform)
- No Nav2

**Tests:**
1. **Stationary baseline** — 60 s motionless, measure map→odom drift rate. Per the M4 baseline in `nav2_lidar_analysis.md`, should be < 0.5 cm/s.
2. **GPS-anchored stability** — confirm `/odometry/global` and `/odometry/gps` agree within 1 m for 60 s stationary.
3. **Hand-push test** — push chassis 3 m by hand, observe EKF behavior. Expected fail with current config (Mahalanobis lockout). After applying fixes below, EKF should re-anchor.
4. **`/set_pose` recovery** — deliberately push chassis out of alignment, then:
   ```bash
   # Per Agent 3:
   ros2 service call /ekf_filter_node_map/set_pose geometry_msgs/srv/SetPose \
     "{pose: {header: {frame_id: 'map'}, pose: {pose: {position: {x: GPS_X, y: GPS_Y, z: 0}, orientation: {w: 1}}, covariance: [25,0,0,0,0,0, 0,25,0,0,0,0, 0,0,25,0,0,0, 0,0,0,0.1,0,0, 0,0,0,0,0.1,0, 0,0,0,0,0,0.5]}}}"
   ```
   Verify EKF jumps to GPS-truth + covariance re-tightens via subsequent IMU updates.

**Pass criteria:** EKF/GPS gap stays < 1 m across all tests after applying `/set_pose` recovery.

**Failure modes isolated:** Mahalanobis lockout, datum confusion, IMU bias, wheel_odom.

**Fix sequence (from Agent 3) — apply BEFORE running tests:**
- Raise `odom0_pose_rejection_threshold: 4.0 → 13.8` (6σ²) in `ekf.yaml`
- Inflate GPS covariance floor to 25 m² in `navsat.yaml`
- Bump `process_noise_covariance` x,y diagonals to ~0.5 on map EKF
- Add `/set_pose` recovery script to `scripts/`: `recover_ekf.py` that auto-reads `/odometry/gps` and `/set_pose`s the map EKF

### Phase 3 — Perception isolated (camera + perception_node, no Nav2)

**Goal:** Lane mask is dense on tape, zero on asphalt, and the kiwicampus integration topics publish at expected rates.

**Setup:** Launch sensors + perception, no Nav2. Chassis stationary, tape in view.

**Tests:**
1. **HSV calibration check** — sample the asphalt center via `sample_hsv.py`. Verify V_p95 < `sooner25_upper.V` (else asphalt leaks into mask).
2. **Lane coverage** — `mask_stats.py` over 30 s. Verify lane fraction > 0.3% AND background fraction > 99%.
3. **Topic rate health** — `/perception/front/semantic_mask` at 10 Hz, `/perception/front/semantic_points` at 10 Hz, `label_info` latched.
4. **Walk in front of camera** — verify person doesn't get classified as lane (S high → S<=15 filter excludes).

**Pass criteria:** Stable lane detection on tape; zero false positives on asphalt; all topics at expected rates.

**Failure modes isolated:** HSV tuning, pipeline selection, sooner25_upper threshold, ZED itself.

### Phase 4 — Local costmap integration

**Goal:** Verify the semantic layer is actually populating the local costmap with lane cells. **Issue #18 (ZED TF Exception) lives here.**

**Setup:** Full nav2 stack (cameras + perception + Nav2 lifecycle active). Chassis stationary.

**Tests:**
1. **TF Exception count baseline** — `grep -c "TF Exception" /tmp/nav_*.log` over 60 s. Compare to bag-time TF error rates per Phase 0 forensics.
2. **Costmap snapshot** — `grab_costmap.py` ≥ once per second for 60 s. Verify lane lethal cells appear in front of chassis.
3. **Clear costmap test** — call `/local_costmap/clear_entirely_local_costmap`, watch it repopulate. Should fill within 10 s.
4. **Lane-in-costmap dwell** — once a lane cell is set lethal, how long does it persist? Test by removing the tape from view.

**Pass criteria:** Lane cells appear within 1 s of camera detection; TF Exception rate < 1/min; clear-and-repopulate works.

**Failure modes isolated:** Issue #18 (TF stamp aging), kiwicampus class_types config, costmap update_frequency.

### Phase 5 — Global costmap + NavfnPlanner

**Goal:** Verify Navfn can find paths to reachable goals.

**Setup:** Full nav2 + chassis stationary. Send `ComputePathToPose` action directly (not through bt_navigator) to test planner alone.

**Tests:**
1. **5 m straight goal** — verify path returned, path stays in costmap-free space.
2. **5 m goal through one obstacle** — verify path curves around.
3. **Goal at costmap edge** — verify path returns or planner says "outside costmap" cleanly.
4. **Goal beyond costmap** — verify planner fails predictably (not hangs).

**Pass criteria:** Path returned for all in-costmap goals; clean failure for out-of-costmap.

**Failure modes isolated:** Navfn config, costmap size, allow_unknown behavior.

### Phase 6 — MPPI controller (path provided)

**Goal:** Verify MPPI follows a known-good path with healthy cmd_vel rate.

**Setup:** Use `FollowPath` action directly with a hand-crafted Path message. Bypass planner.

**Tests:**
1. **Straight 5 m path** — chassis tracks within 0.2 m lateral error, cmd_vel rate > 18 Hz.
2. **Curved 5 m path** — same accuracy, no oscillation.
3. **CPU starvation test** — start `stress-ng --cpu 2` in another terminal, observe cmd_vel rate drop. Confirms "MPPI is CPU-bound" hypothesis.
4. **MPPI critic instrumentation** — subscribe to MPPI's diagnostic topics (per Agent 2's findings) to see which critic dominated the chosen trajectory.

**Pass criteria:** cmd_vel ≥ 18 Hz under no load, ≥ 10 Hz under stress; chassis tracks paths within 0.3 m.

**Failure modes isolated:** MPPI tuning (batch_size, time_steps, critics), CPU starvation.

### Phase 7 — Behavior tree + recovery

**Goal:** Verify recovery behaviors fire correctly and **don't cross lethals**.

**Setup:** Full nav2 + send a goal that forces MPPI to fail (e.g., goal inside a lethal blob).

**Tests:**
1. **Goal in lethal cell** — verify BT triggers ComputePathToPose → recovery sequence in order.
2. **BackUp safety** — with a known lethal cell 0.5 m behind chassis, verify BackUp aborts on Collision Ahead.
3. **DriveOnHeading safety** — same with lethal in front.
4. **Recovery sequence order** — verify ClearAroundRobot → Wait → BackUp → CrawlForward order from BT XML.

**Pass criteria:** BackUp never executes when rear costmap has lethals within `backup_dist`. **If BackUp drives over a lethal, that's a critical bug.**

**Failure modes isolated:** BT XML config, BackUp/DriveOnHeading collision checks, recovery ordering.

### Phase 8 — End-to-end IGVC scenarios

**Goal:** Full stack handles real IGVC AutoNav scenarios.

**Setup:** Full nav2 + perception + lane tape down + barrels placed + waypoint sequence.

**Tests:**
1. **10 m straight clean asphalt** — single goal forward, verify clean traversal.
2. **10 m corridor with two zigzag obstacles** — chassis stays in lane, avoids barrels.
3. **GPS waypoint sequence** — send 4 waypoints (the IGVC AutoNav pattern), use `nav2_waypoint_follower`.
4. **Recovery during course** — manually block chassis path mid-run, verify recovery fires safely and re-plans.

**Pass criteria:** Goal reached in budget time; **zero line crossings**; ≥ 1 mph average; no chassis-stuck failures.

---

## Specific tuning recommendations (apply during Phase 0-2 setup)

### ekf.yaml
```yaml
ekf_filter_node_map:
  ros__parameters:
    odom0_pose_rejection_threshold: 13.8  # 4.0 -> 13.8 (6σ² ≈ tolerate 5 m gap)
    process_noise_covariance:
      # bump x,y diagonals (entries 0,0 and 1,1)
      # from default ~0.05 to ~0.5 so P grows faster between GPS updates
```

### navsat.yaml
```yaml
navsat_transform:
  ros__parameters:
    # Floor GPS covariance at σ=5 m so the EKF's S includes realistic noise.
    # (Add input filter or scaling; r_l doesn't directly expose a covariance floor.)
```

### Existing changes from earlier today — keep them:
- `inflation_radius: 0.3` (both costmaps)
- `CostCritic.cost_weight: 6.0`
- `PathAlignCritic.cost_weight: 8.0`, `max_path_occupancy_ratio: 0.15`
- `time_steps: 25`, `batch_size: 500`
- `vx_min: -0.4` (reverse OK per user)
- `expected_planner_frequency: 5.0`
- Global costmap 100 m × 100 m

---

## Architecture decisions (informed by Agents 1 + 2)

### From Agent 1 — RoboJackets patterns worth porting

RoboJackets' stack is ROS1 (Noetic) so nothing compiles as-is on Humble — but **5 patterns are architecturally golden:**

1. **Virtual `back_circle` costmap layer** (`igvc_navigation/src/back_circle/`, `src/mapper/back_circle_layer.cpp`). Paints a semicircle of LETHAL cells behind the chassis in odom frame. **Directly solves our "BackUp recovery crosses rear-blind lanes" problem.** Port as a Nav2 costmap plugin (~150 lines C++). Their config: 1.5 m wide × 2.0 m long × offset -0.5 m.

2. **TF fallback to "latest" when stamp falls out of buffer** (`line_layer.cpp:165-175`). When `canTransform(target, source, exact_stamp)` fails, falls back to `lookupTransform(target, source, Time{0})` with a WARN log. **This is the exact fix for our Issue #18 ZED TF Exception bug** — drop-in pattern for our perception_node and semantic_segmentation_layer integration.

3. **Persistent log-odds line costmap with exponential confidence decay** (`line_layer.cpp:249-264`). Each detected pixel adds log-odds weighted by `exp(-coeff·distance)`. Old detections persist with confidence; distant/oblique observations carry less weight. Way more robust than kiwicampus's overwrite-every-frame.

4. **Cached per-pixel IPM rays** (`cnn.py:229-241`, `line_layer.cpp:145-163`). At init, precompute a full (rows × cols) array of 3D rays from CameraInfo. Per-frame projection is just: rotate ray by current TF, intersect ground plane. **90% faster than naive per-frame projection.** ZED intrinsics don't change — perfect fit.

5. **Local costmap as a passthrough mirror of global** (`rolling_layer.cpp`). Eliminates the "local sees obstacle global doesn't" bug. Single source of truth.

**Their `back_up_recovery.cpp:95-101` actually checks footprint cost before stepping back** — which means our BackUp crossing lanes is either (a) footprint mis-defined, (b) costmap was empty (perception not feeding it), or (c) we're using a different recovery node. Verify in Phase 7.

**3 deal-breakers (don't port):**
- ROS1/catkin — every file needs transcription to ROS2
- Holonomic swerve-drive + TEB — wrong kinematics for our tracked diff-drive (keep MPPI)
- GTSAM IncrementalFixedLagSmoother — overkill for our setup; dual-EKF is fine

### From Agent 2 — Nav2 diagnostic tools to wire in immediately

**Top 5 diagnostic topics + services:**

1. **`/local_costmap/costmap_raw`** (`nav2_msgs/Costmap`, transient_local) — per-cell 0-255 values, not the 0/100 occupancy view. Use to see whether STVL is writing the cell vs InflationLayer filling it.

2. **`/get_costmap` service** — synchronous snapshot at any time:
   ```bash
   ros2 service call /local_costmap/get_costmap nav2_msgs/srv/GetCostmap \
     '{specs: {resolution: 0.05, width: 200, height: 200}}'
   ```

3. **`MPPIController.visualize: true`** — publishes `/trajectories` (`visualization_msgs/MarkerArray`) showing all 500 sampled rollouts + the chosen optimal. Only publishes when subscribers exist, so no CPU cost when nobody's listening.

4. **`/behavior_tree_log`** + `bt_log_idle_transitions: true` — only authoritative source for "what recovery fired" + catches silent IDLE→IDLE transitions.

5. **`ros2 run tf2_ros tf2_monitor odom zed_front_left_camera_frame_optical`** — per-frame avg/max delay. **If max delay > 0.1 s against a 10 s buffer, that's our Issue #18 early warning.**

**For "Optimizer fail to compute path" — bump MPPIController logger to DEBUG:**
```bash
ros2 service call /controller_server/set_logger_levels rcl_interfaces/srv/SetLoggersLevels \
  '{levels: [{name: MPPIController, level: 10}]}'
```
Per-critic costs print per cycle. Tells you which critic dominated rejection.

**For BT silent aborts:** set `bt_log_idle_transitions: true` in bt_navigator params.

**For TF Exceptions:** bump tf2_ros logger to DEBUG: any failed lookupTransform prints the requested stamp vs available buffer range.

### Per-Layer Costmap Inspection (Humble limitation)

Per Agent 2: Humble has NO per-layer cost contribution topic. To debug "which layer painted this cell lethal," set `local_costmap.plugins: ["voxel_layer"]` (drop inflation, drop semantic one at a time), restart, re-record. The Jazzy `CostInspector` RViz tool is NOT backported. Live Groot monitoring is also gone on Humble.

**Architectural recommendation:** keep the dual-EKF setup (per Agent 3's analysis); add the back_circle layer (per Agent 1); add `MPPIController.visualize: true` for the session (revert before competition).

---

## New Phase 9 — Port the two RoboJackets patterns (post-test, pre-competition)

After Phases 0-8 pass and the stack is stable, do these two strategic ports:

### 9a — back_circle Nav2 costmap plugin

**Goal:** Eliminate the rear-blind problem.

**Source:** `robojackets/igvc_navigation/src/mapper/back_circle_layer.cpp` (ROS1) + `config/back_circle.yaml` (1.5 × 2.0 m, offset -0.5).

**Target:** New `nav2_costmap_2d::Layer` plugin in our `src/avros_bringup/plugins/back_circle_layer/`.

**Effort:** ~1 day.

**Verification:** Test 7.2 (BackUp safety) should now pass — BackUp aborts on Collision Ahead because the back_circle layer pre-paints lethal cells behind chassis.

### 9b — TF fallback to "latest" in perception_node

**Goal:** Eliminate Issue #18 — ZED stamps falling out of TF buffer no longer kill semantic layer updates.

**Source:** `robojackets/igvc_navigation/src/mapper/line_layer.cpp:165-175` pattern:
```cpp
try {
  tf_buffer->canTransform(target, source, stamp, timeout) ?
    tf_buffer->lookupTransform(target, source, stamp) :
    (WARN_LOG, tf_buffer->lookupTransform(target, source, Time{0}));
} catch (...) { ... }
```

**Target:** Modify `src/avros_perception/avros_perception/perception_node.py` to use latest-stamp fallback when ZED cloud stamp falls out of buffer.

**Effort:** ~2 hours.

**Verification:** Phase 4 Test 1 (TF Exception count) should drop to < 0.1/min.

---

## Specific tuning recommendations — UPDATED

Apply BEFORE Phase 2 (localization) tests:

### ekf.yaml
```yaml
ekf_filter_node_map:
  ros__parameters:
    odom0_pose_rejection_threshold: 13.8  # 4.0 -> 13.8 (6σ²)
    imu0_pose_rejection_threshold: 5.0    # keep — Xsens stuck-bias guard
    imu0_twist_rejection_threshold: 5.0   # keep
    # Bump process noise x,y diagonals from ~0.05 to ~0.5:
    # process_noise_covariance[0]  (x,x)   0.05 -> 0.5
    # process_noise_covariance[7]  (y,y)   0.05 -> 0.5
    two_d_mode: true   # halves S dimensionality; chassis is flat-course
```

### navsat.yaml
```yaml
navsat_transform:
  ros__parameters:
    # Add a covariance floor at σ=5 m to reflect real SBAS noise.
    # (No direct param; add a topic filter or scaling node if needed.)
```

### nav2_params_humble.yaml — add for session
```yaml
controller_server:
  ros__parameters:
    FollowPath:
      visualize: true   # publishes /trajectories for debugging; revert before comp
bt_navigator:
  ros__parameters:
    bt_log_idle_transitions: true   # catches silent BT aborts
```

### Existing tuning from earlier today — keep
- `inflation_radius: 0.3` (both costmaps)
- `CostCritic.cost_weight: 6.0`, `PathAlignCritic.cost_weight: 8.0`, `max_path_occupancy_ratio: 0.15`
- `time_steps: 25`, `batch_size: 500`
- `vx_min: -0.4` (reverse OK per user)
- `expected_planner_frequency: 5.0`
- Global costmap 100 m × 100 m
- HSV → sooner25 pipeline, V_upper=215

---

## Execution checklist

For each phase:
- [ ] Apply the phase's setup (launch commands, params, sensors)
- [ ] Confirm prerequisites pass (sensors at expected rate, topics flowing)
- [ ] Run each test sequentially; record pass/fail
- [ ] Capture bag of the phase test (small — just the relevant topics)
- [ ] If any test fails, **stop and fix before moving on**
- [ ] Document findings in a per-phase results doc

If we get to Phase 8 with all prior phases passing, we have a competition-ready stack.

If a fix to one phase regresses an earlier phase, that's a regression and must be diagnosed — don't move forward.

---

## Sources and citations

- Agent 3 (EKF/GPS) — `cra-ros-pkg/robot_localization` humble-devel `ros_filter.cpp:2546-2596` (setPoseCallback), `filter_base.cpp:434-444` (Mahalanobis gate), `navsat_transform.cpp:359-366` (datum re-frame)
- Agent 4 (Bag forensics) — rosbag2_py SequentialReader + StorageFilter pattern; `cross_bag_report.py` workflow
- Sequential thinking — failure mode catalog + 8-phase ordering
- This morning's `lane_following_strategy.md` — multi-agent findings from Sooner Robotics analysis
- This afternoon's session logs — failure modes observed in real-time
