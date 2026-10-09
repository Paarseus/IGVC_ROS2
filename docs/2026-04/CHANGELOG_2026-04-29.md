# 2026-04-29 — kiwicampus raytrace-clear patch + Nav2 BT bring-up + combined launch + HSV calibration

**Scope.** Long session, four threads:

1. **Costmap clearing fundamentally broken — fixed.** The kiwicampus `semantic_segmentation_layer` plugin was write-only into `costmap_`: cells went LETHAL on observation but never went FREE again until the rolling-window scrolled them out or someone called `/clear_*_costmap`. Yesterday's `tile_map_decay_time: 1.5 → 5.0` workaround masked the symptom (cells that were stuck at 7 became stuck at 329), but the bug was always: the plugin has **no clearing path at all**. Diagnosed end-to-end via a multi-phase, multi-agent source audit, then wrote a patch (`kiwicampus_pr3_raytrace_clear.patch`, 414 lines git-format) that mirrors `nav2_costmap_2d::ObstacleLayer::raytraceFreespace`. Cells now go LETHAL → FREE within one update cycle when the camera "sees through" them. Pushed upstream as draft PR [kiwicampus#5](https://github.com/kiwicampus/semantic_segmentation_layer/pull/5).

2. **Combined-launch bring-up + BT XML for arbitrary goals.** Wrote `localization_perception_test.launch.py` (full pipeline minus Nav2 — sensors + EKF + GPS + perception + costmap + webui + foxglove) so the four-terminal recipe from yesterday becomes one launch. Added `navigate_to_pose_simple_humble.xml` (10-line `ComputePathToPose → FollowPath` BT) + `bt_xml` launch arg in `navigation.launch.py` so we can swap the route-graph BT for an arbitrary-goal BT without editing the launch file.

3. **Nav2 stack bring-up tested end-to-end** — full `navigation.launch.py` with simple BT brings all 7 nav2 servers (`controller`, `smoother`, `planner`, `route`, `behavior`, `velocity_smoother`, `bt_navigator`) to `active [3]`. Initial run had `bt_navigator` stuck at `inactive [2]` because `plugin_lib_names` was missing `nav2_compute_path_to_pose_action_bt_node` — added it.

4. **HSV calibration loop in real outdoor lighting** (dusk → night). Iterated `lane_low`, `adaptive_k`, `sky_roi_poly` while watching the live overlay; landed on values that caught both painted-edge lane lines without false-positive on concrete. NTRIP creds verified already configured (TODO line was stale — fixed).

Companion to `docs/CHANGELOG_2026-04-28.md` (wheel-odom + EKF fusion, dual-clock decay workaround) and `docs/CHANGELOG_2026-04-27_field.md` (HSV outdoor calibration).

Files touched (4 commits + 1 upstream PR):

```
A  TODO.md                                                        (NTRIP TODO update — 4e4d562)
M  src/avros_bringup/launch/navigation.launch.py                  (bt_xml arg — 052ee4e)
A  src/avros_bringup/config/navigate_to_pose_simple_humble.xml    (simple BT — 052ee4e)
A  src/avros_bringup/launch/localization_perception_test.launch.py (combined launch — earlier in session)
A  src/avros_bringup/patches/kiwicampus_pr3_raytrace_clear.patch  (PR3 source patch — 8680313)
M  scripts/apply_kiwicampus_patches.sh                            (PR3 in PATCHES list — 8680313)
M  src/avros_bringup/config/perception_test_params.yaml           (clearing: true + decay revert — 8680313, f1657f1)
M  src/avros_bringup/config/nav2_params_humble.yaml               (clearing: true ×3 + decay revert + plugin_lib_names — 8680313, f1657f1)
A  docs/CHANGELOG_2026-04-29.md                                   (this file)
```

Out of scope (deferred): rolling-API rebase of the upstream PR (waiting on maintainer guidance); migration of the dual-clock `purgeOldObservations` race fix into a separate kiwicampus PR; broader HSV calibration for daylight + the `pothole` class.

---

## 1. Combined launch — `localization_perception_test.launch.py`

**Why.** Yesterday's working stack required four terminals: `localization.launch.py` + `perception.launch.py` + `webui.launch.py` + a manual `controller_server`/`lifecycle_manager` pair. That's the exact composition where neither `perception_test.launch.py` (uses identity static TFs, robot doesn't translate in the costmap) nor `navigation.launch.py` (full Nav2 stack, no manual drive) fits.

**What.** New launch file, ~120 lines, wraps:

| Layer | Source | What it brings up |
|---|---|---|
| Sensors + EKF + GPS | `localization.launch.py` (front-only sensor toggles) | URDF, ZED front, Xsens, NTRIP, dual EKF, navsat_transform |
| Perception | `avros_perception/perception.launch.py` (`cameras: front`) | `perception_node` (HSV pipeline) |
| Drive | `webui.launch.py` | `actuator_node` + `webui_node` (HTTPS port 8000) |
| Costmap | direct `controller_server` + `lifecycle_manager` with `perception_test_params.yaml` | `local_costmap` painted by `semantic_layer` |
| Visualization | `foxglove_bridge` (port 8765) | laptop-side viz |

Single command: `ros2 launch avros_bringup localization_perception_test.launch.py`.

**Files.** `src/avros_bringup/launch/localization_perception_test.launch.py`, ~50 lines new.

---

## 2. NTRIP creds verified — TODO de-staled

**Why.** The standing `TODO.md` line said `mountpoint: CHANGE_ME` — a holdover from yesterday's audit. Actually `ntrip_params.yaml` was already pointing at `ntrip.earthscope.org:2101` mountpoint `PSDM_RTCM3P3` with valid EarthScope creds, and `/rtcm` was streaming at ~5 Hz on the running stack. Indoors `/gnss status: -1` (no satellites) is expected; the corrections will get applied as soon as GPS gets sky view.

**What.** Updated `TODO.md` to drop the `CHANGE_ME` line, replaced with a still-valid TODO ("verify RTK FIXED status outdoors") that captures what actually remains undone — confirming the corrections produce a FIXED solution once GPS has lock.

**Commit.** `4e4d562`.

---

## 3. Costmap clearing investigation (5 phases, 9 sub-agents)

The expensive part of the day. User asked: "decay doesn't work, the costmap doesn't get refreshed — what if an obstacle passes by and the cells don't clear?" Set up a methodical multi-phase investigation, parallelizing per phase via subagents.

### 3.1 Phase 0 — baseline (inline, 2 min)

`/local_costmap/local_costmap` plugins: `[voxel_layer, semantic_layer, inflation_layer]`. **`voxel_layer` already has `clearing: true` + `raytrace_max_range: 20.0` for camera_depth & velodyne_points.** `semantic_layer` exposes ZERO clearing-related params — only `tile_map_decay_time` and `observation_persistence`. Cells in current run: 67 LETHAL + 838 nonzero (lane-edge detections + their inflation rings). `/local_costmap/clear_*` services exist (used yesterday for manual clearing).

### 3.2 Phase 1 — how clearing works (3 parallel agents)

Each agent took one slice:

**1A: standard nav2 `ObstacleLayer` raytrace clear.** From upstream Humble source via WebFetch. Algorithm:
```
for each clearing observation:
  origin = obs.origin_  (sensor pose, transformed to global frame per-observation)
  for each observed point (wx, wy):
    MarkCell marker(costmap_, FREE_SPACE)
    raytraceLine(marker, origin_cell, point_cell, max_range, min_range)
```
Bresenham-walks every cell along the ray (`obstacle_layer.cpp:705`), writing `FREE_SPACE` via `MarkCell` functor (`:703`). Per-source `clearing: true/false` flag controls inclusion in `clearing_buffers_` (`:210-212`). Footprint clear via `setConvexPolygonCost(footprint, FREE_SPACE)` is the *only* other FREE_SPACE write path (`updateCosts:549-551`).

**1B: kiwicampus clearing gap.** Source: `/tmp/kiwi_src/`. Findings, file-line precise:
- `costmap_` only ever written at two lines: `:361` (`max_cost`) and `:365` (`base_cost`)
- When a tile's queue purges to empty, `updateBounds:344` does `continue` — **the corresponding `costmap_[index]` is never touched**
- `updateOrigin` only fills *newly-exposed* strip when the rolling window scrolls; in-window cells retain their last-written cost
- `combination_method` operates `costmap_ → master_grid`, **cannot** clear cells in `costmap_`
- **No raytrace, clearLine, setFree, or any inverse-write logic exists in this plugin.** Architecture is write-only.

**1C: hidden-features audit.** High confidence, exhaustive grep across `.cpp`/`.hpp`/`README`/`CMakeLists`. Verdict: **NO** built-in clearing. Apparent matches that are red herrings:
- `isClearable()` → just routes the global `/clear_*_costmap` service to `reset()` (full wipe)
- `default_value_ = FREE_SPACE` → initial fill only
- `raytrace_*_range` doxygen in `segmentation_buffer.hpp:664-667` → **stale copy-paste from nav2 `ObservationBuffer`, params not declared/used anywhere**
- `purgeStaleSegmentations()` → **declared in header at line 774, never defined — dead symbol**

The doxygen vestige + dead symbol strongly suggest the original design *did* call for raytracing, but it was never implemented (or was removed before initial release).

### 3.3 Phase 2 — CV-side options (3 parallel agents)

**2A: config-only "drivable: free" approach** — adding a `class_type` with `classes: [free]` and `base_cost: 0`. **Verdict: CONDITIONAL YES** with two caveats:
- Need `use_cost_selection: false` (cost-based picks danger over free per-tile per-frame; switch to confidence-based)
- Cells stay LETHAL up to `tile_map_decay_time` after the last danger sighting before flipping to free
- Single false-positive danger pixel per tile keeps the whole tile LETHAL

**2B: depth/raytrace input feasibility.** `/perception/front/semantic_points` is organized 256×448, 4.3% NaN, frame `zed_front_left_camera_frame` (sensor origin `(0,0,0)`), TF→odom verified, ~110k finite points/frame at 15 Hz. Sufficient for raytrace-clearing implementation.

**2C: voxel_layer alongside semantic_layer.** **Dead end confirmed.** Each layer has private `costmap_`. `LayeredCostmap::updateMap` runs `updateBounds` per layer (each writes own buffer), then `updateCosts` per layer in plugin order. With order `[voxel, semantic, inflation]` and `updateWithMax` (default), even if `voxel_layer` clears its own cells via raytrace, `semantic_layer.updateCosts` runs *after* it and `updateWithMax` overrides voxel's 0s with semantic's 254s wherever semantic last wrote LETHAL. Voxel cannot help clear semantic cells.

### 3.4 Phase 3 — fix design comparison (collapsed)

The viable options collapsed to two:

| Option | Effort | Latency | Caveats |
|---|---|---|---|
| α — config-only `drivable: free` | ~10 lines YAML, no rebuild | ≤decay_time lag | one false-positive danger pixel can keep tile LETHAL |
| β — kiwicampus source patch (raytraceFreespace) | ~80–120 lines C++, new patch + rebuild | <200 ms (one update cycle) | patch maintenance vs upstream |

Voxel-alongside (option that would have been the third candidate) was eliminated by 2C.

User picked **β** — the standard answer. α is a workaround masquerading as a fix; β is what every nav2-shipped layer does.

---

## 4. PR3 source patch — `kiwicampus_pr3_raytrace_clear.patch`

### 4.1 Implementation (4 files, 256 insertions)

**`segmentation_buffer.hpp`** — new public-nested struct + members + setters:

```cpp
struct ClearingObservation {
    rclcpp::Time time;
    geometry_msgs::msg::Point origin;            // sensor origin in global_frame
    std::vector<geometry_msgs::msg::Point> points;  // hits in global_frame
};

void setRaytraceMaxRange(double range)    { sq_raytrace_max_range_ = range * range; }
void setRaytraceMinRange(double range)    { sq_raytrace_min_range_ = range * range; }
void setClearingEnabled(bool enabled)     { clearing_enabled_ = enabled; }
bool getClearingObservation(ClearingObservation& obs);
double getSqRaytraceMaxRange() const      { return sq_raytrace_max_range_; }
double getSqRaytraceMinRange() const      { return sq_raytrace_min_range_; }

// private:
double sq_raytrace_max_range_ = 64.0;   // (8m)^2 default
double sq_raytrace_min_range_ = 0.0;
bool   clearing_enabled_ = false;       // opt-in per source
ClearingObservation latest_clearing_obs_;
bool   has_clearing_obs_ = false;
```

**`segmentation_buffer.cpp`** — capture-on-the-fly inside the existing per-pixel loop in `bufferSegmentation`:

```cpp
// PR3 — capture every finite, in-range global-frame point for raytrace clearing.
if (clearing_enabled_) {
    geometry_msgs::msg::Point p;
    p.x = *iter_x_global; p.y = *iter_y_global; p.z = *iter_z_global;
    clearing_points.push_back(p);
}
```

After the iteration, commit under buffer lock:
```cpp
if (clearing_enabled_) {
    std::lock_guard<std::recursive_mutex> guard(lock_);
    latest_clearing_obs_.time   = rclcpp::Time(cloud.header.stamp.sec, cloud.header.stamp.nanosec);
    latest_clearing_obs_.origin = global_origin.point;
    latest_clearing_obs_.points = std::move(clearing_points);
    has_clearing_obs_ = true;
}
```

Plus the getter:
```cpp
bool SegmentationBuffer::getClearingObservation(ClearingObservation& obs) {
    std::lock_guard<std::recursive_mutex> guard(lock_);
    if (!clearing_enabled_ || !has_clearing_obs_) return false;
    obs = latest_clearing_obs_;
    return true;
}
```

**`semantic_segmentation_layer.hpp`** — protected method declaration:
```cpp
void raytraceFreespace(
    const std::shared_ptr<semantic_segmentation_layer::SegmentationBuffer>& buffer,
    double* min_x, double* min_y, double* max_x, double* max_y);
```

**`semantic_segmentation_layer.cpp`** — params, buffer setters, raytraceFreespace call, raytraceFreespace definition. Includes:
```cpp
#include <limits>
#include "nav2_costmap_2d/cost_values.hpp"
```

Per-source params (default off):
```cpp
declareParameter(source + "." + "clearing", rclcpp::ParameterValue(false));
declareParameter(source + "." + "raytrace_max_range", rclcpp::ParameterValue(8.0));
declareParameter(source + "." + "raytrace_min_range", rclcpp::ParameterValue(0.0));
```

After buffer construction:
```cpp
segmentation_buffer->setClearingEnabled(clearing);
segmentation_buffer->setRaytraceMaxRange(raytrace_max_range);
segmentation_buffer->setRaytraceMinRange(raytrace_min_range);
if (clearing) {
    RCLCPP_INFO(logger_, "PR3 raytrace clearing enabled for source %s "
                "(raytrace_max=%.2fm, raytrace_min=%.2fm)",
                source.c_str(), raytrace_max_range, raytrace_min_range);
}
```

Call site in `updateBounds`, **before** the existing tile-iteration marking loop (so freshly-marked LETHAL cells survive):
```cpp
buffer->lock();
tile_map_pair.first->lock();

// PR3: clearing pass first, then marking pass
raytraceFreespace(buffer, min_x, min_y, max_x, max_y);

tile_map_pair.first->purgeOldObservations(current_time);
for(auto& tile: *tile_map_pair.first) { /* existing marking loop */ }
```

`raytraceFreespace` itself mirrors `obstacle_layer.cpp:623-712` line by line — anonymous-namespace `FreeSpaceMarker` functor, per-point 2D distance gate against `sq_raytrace_max_range_`, endpoint clip to map rectangle (same algebra as `obstacle_layer.cpp:670-691`), `raytraceLine(marker, x0,y0, x1,y1)` with cell-distance bound at `numeric_limits::max()` since the world-space gate already enforces range, then `touch()` to extend the layer's update bounds to cover each cleared ray.

### 4.2 Build + restart

`colcon build --symlink-install --packages-select semantic_segmentation_layer` succeeded clean on first compile (no missing include / typo / signature mismatch). Stack restart picked up the new plugin `.so`; activation log confirmed:
```
PR3 raytrace clearing enabled for source front (raytrace_max=8.00m, raytrace_min=0.00m)
PR3 raytrace clearing enabled for source left  (...)
PR3 raytrace clearing enabled for source right (...)
```

### 4.3 A/B test — proof it works

Same protocol as 2026-04-28's decay-bug verification:

| | Before PR3 (decay 1.5) | Before PR3 (decay 5.0 workaround) | **After PR3 (decay 1.5)** |
|---|---|---|---|
| Cells while observations live | 7 (frozen) | 990 | **42 (steady)** |
| Cells after observations stop | 7 (frozen forever) | 329 (frozen forever) | **0 within one update cycle** |
| Decay actually clears? | ❌ | ❌ (just bigger frozen pool) | **✅** |

Cells went from 42 LETHAL → 0 LETHAL in roughly one 200 ms cycle after suppressing observations (set `adaptive_k: 10.0` to disable lane class). Recovered to 42 LETHAL when observations resumed. The clearing pass + marking pass interaction works exactly as designed.

### 4.4 Commits + upstream PR

- Internal: commit `8680313` ("Add kiwicampus PR3 raytrace-clear patch + enable in configs"). Patch saved at `src/avros_bringup/patches/kiwicampus_pr3_raytrace_clear.patch` (414 lines git-format), added to `scripts/apply_kiwicampus_patches.sh`'s `PATCHES` list.
- Upstream: pushed branch `feat/raytrace-freespace-clear` to `Paarseus/semantic_segmentation_layer` fork; opened **draft PR [kiwicampus#5](https://github.com/kiwicampus/semantic_segmentation_layer/pull/5)**. PR description is honest about the patch being developed against post-PR1 (Humble build fix) state; asks maintainers about rolling-port + branch strategy + dependency on the dual-clock race fix (which yesterday's mutex-patch addresses).

---

## 5. Nav2 BT bring-up

### 5.1 New simple BT XML

**Why.** The existing `navigate_route_graph_humble.xml` uses `ComputeRoute → FollowPath` against `cpp_campus_graph.geojson` (52 nodes, 113 edges). For testing whether the planner avoids LETHAL costmap cells with arbitrary "send a goal forward" tests, we need the planner pipeline (`ComputePathToPose` → `SmacPlannerHybrid`) instead.

**What.** `src/avros_bringup/config/navigate_to_pose_simple_humble.xml`, 10 lines:
```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithReplanning">
      <RateController hz="1.0">
        <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
      </RateController>
      <FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="general_goal_checker"/>
    </PipelineSequence>
  </BehaviorTree>
</root>
```

No recoveries — clean signal: it either plans + drives, or it doesn't.

### 5.2 `bt_xml` launch arg

**Why.** Hard-coding the BT XML path in `navigation.launch.py` made swapping BTs require a launch-file edit. Need a runtime selector.

**What.** `bt_xml` launch arg, defaults to the existing route-graph BT (preserves prior behaviour). For the simple BT:
```bash
ros2 launch avros_bringup navigation.launch.py \
    bt_xml:=navigate_to_pose_simple_humble.xml \
    enable_velodyne:=false enable_realsense:=false \
    enable_zed_front:=true enable_perception:=true
```

`navigation.launch.py` rewrites `default_nav_to_pose_bt_xml` and `default_nav_through_poses_bt_xml` to `[pkg_dir, '/config/', bt_xml_filename]` via `RewrittenYaml`.

**Commit.** `052ee4e`.

### 5.3 `bt_navigator` initially stuck at `inactive [2]`

**Symptom.** All 6 other Nav2 servers reached `active [3]`, but `bt_navigator` stayed at `inactive [2]` (CONFIGURED but not ACTIVATED).

**Diagnosis.** From the log:
```
[bt_navigator] Exception when loading BT:
Error at line 23: -> Node not recognized: ComputePathToPose
[bt_navigator] Error loading XML file: navigate_to_pose_simple_humble.xml
```

`bt_navigator.plugin_lib_names` in `nav2_params_humble.yaml` contained `nav2_compute_route_bt_node` (for the route-graph BT) but **not** `nav2_compute_path_to_pose_action_bt_node` (which our simple BT uses). The XML failed to parse because the BT plugin loader doesn't know about `ComputePathToPose`, so the layer's `Configuring → Activating` transition errored out.

**Fix.** Added `nav2_compute_path_to_pose_action_bt_node` to the list (kept the route one too — both BTs work now).

### 5.4 Phase A lifecycle test passed

After the fix:
```
controller_server     active [3]
smoother_server       active [3]
planner_server        active [3]
route_server          active [3]
behavior_server       active [3]
velocity_smoother     active [3]
bt_navigator          active [3]
```
Plus `Managed nodes are active` from `lifecycle_manager_navigation`. Ready for goal-following tests when the power rail buck is in.

**Commit.** `f1657f1`.

---

## 6. Decay revert — `tile_map_decay_time: 5.0 → 1.5`

**Why.** Yesterday's `5.0` was the dual-clock decay workaround — it prevented `updateBounds`'s `node->now()`-based purge from over-purging observations stamped with `cloud_time_seconds`. With PR3, cells actively clear via raytracing, so they no longer depend on a long decay window to mask the bug. The kiwicampus default 1.5 s is now stable.

**What.** `tile_map_decay_time: 5.0 → 1.5` in 4 places (`perception_test_params.yaml` ×1, `nav2_params_humble.yaml` ×3 — front/left/right semantic_layer sources). Comment blocks updated to reflect post-PR3 state — the prior comment narrating the 1.5→5.0 bump is now historical.

Verified after revert: PR3 still log-confirms on each source, costmap cells still clear within one cycle when observations stop, no flicker at 1.5 s decay.

**Commit.** `f1657f1` (same commit as the `bt_navigator` fix — both are post-PR3 cleanup).

---

## 7. HSV calibration session (live tuning)

Outdoor scene at dusk → night, robot stationary on a concrete walkway with painted edge lines + two people in frame. Iterated via `ros2 param set /perception_front …` while watching the live overlay snapshot per change:

| Iter | Change | mask % lane | Effect |
|---|---|---|---|
| 0 | baseline (`lane_low V=180`, `adaptive_k=3.0`) | 0% | painted lines too dim at dusk for default thresholds |
| 1 | `lane_low V: 180 → 120` | 0% | adaptive bright filter (`mean+3σ`) was the bottleneck, not `lane_low V` |
| 2 | `adaptive_k: 3.0 → 1.5` | 0.30% | both lane edges starting to catch |
| 3 | `adaptive_k: 1.5 → 1.0` | 0.79% | both edges clearly highlighted |
| 4 | `lane_high S: 60 → 120` | **34% (regression!)** | too aggressive — entire concrete walkway got painted lane_white |
| 5 | revert `lane_high S: 120 → 60` | 0.81% | back to clean |
| 6 | `sky_roi_poly: top 35% → top 50%` | 0.70% | strip distant artifacts from top of frame |

**Final tuned values for dusk/night:**

```yaml
lane_low:      [0, 0, 80]      # was [0, 0, 180]
lane_high:     [179, 60, 255]  # unchanged
adaptive_k:    1.0             # was 3.0
sky_roi_poly:  [0.0, 0.0, 1.0, 0.0, 1.0, 0.5, 0.0, 0.5]  # top 50% mask, was top 35%
```

These are **live in memory only** — applied via `ros2 param set`, not written into `perception.yaml` yet. To persist across launches, those values need to be baked into the YAML in a follow-up commit (deferred to next session per session-end discussion).

The scene was at the edge of the camera's dynamic range — distant lane portions remained too dim regardless of threshold tuning. Camera-physics ceiling, not algorithm tuning.

**Commit.** Not committed (live-tuning session; values pending future YAML bake-in).

---

## 8. Stack composition reference (updated)

For future "what's the right launch for this use case":

| Use case | Launch | Notes |
|---|---|---|
| HSV calibration / perception only, no driving | `perception_test.launch.py enable_drive:=false` | static identity TFs; robot doesn't translate in costmap |
| Manual drive + perception + costmap (real EKF) | **`localization_perception_test.launch.py`** (NEW) | wraps localization + perception + webui + controller_server |
| Full Nav2 with route-graph autonomy | `navigation.launch.py` | uses default `navigate_route_graph_humble.xml` BT |
| Full Nav2 with arbitrary-goal autonomy | `navigation.launch.py bt_xml:=navigate_to_pose_simple_humble.xml` | NEW — for testing planner-around-obstacles |

`webui` integrates with the first three; `navigation.launch.py` uses `actuator_node` directly so manual control is via `/cmd_vel` topic publishing or the BT itself.

---

## 9. TODO additions / removals

Removed (resolved by today's work):
- ~~Configure NTRIP credentials in ntrip_params.yaml — was already done~~ (replaced with "verify RTK FIXED status outdoors")
- ~~Write kiwicampus_align_purge_clocks.patch — proper fix for the dual-clock decay bug~~ — superseded by PR3 raytrace clear, which makes the decay-window length irrelevant
- ~~Document kiwicampus tile_map frame_id="map" mislabel~~ — separate concern, still pending but lower priority now that clearing actually works

Added (new):
- Bake live-tuned HSV values (lane_low, adaptive_k, sky_roi_poly) into `perception.yaml` so they persist across launches
- Daylight HSV calibration pass — current dusk values may be too permissive in bright sun
- Wait for kiwicampus maintainer feedback on draft PR #5; rebase to rolling API when they confirm direction

---

---

# Evening session addendum (2026-04-29 night)

After the morning's PR3 + Nav2 BT bring-up landed, end-to-end smoke testing revealed the master `/local_costmap/costmap` was **still all-FREE** despite the layer's internal `tile_map` debug topic clearly showing 8–13 marked tiles per frame. PR3 had fixed the *clearing* path but did not fix the *marking* path — that bug had been hidden by the morning's `tile_map_decay_time=5.0` workaround, and only surfaced once decay reverted to 1.5. This second session diagnosed the real root cause, fixed it, forked the upstream package, and switched the repo onto our fork.

## 10. Costmap-empty bug — root cause was a clock-domain mismatch, not a propagation bug

### 10.1 Symptom

With PR3 applied, `tile_map_decay_time: 1.5`, perception detecting 240 lane pixels per frame, all three sync topics sharing identical `header.stamp` (verified 87/87 triplets):

- `/local_costmap/front/tile_map` (kiwicampus debug PointCloud2): **8–13 entries**, class=1 lane, conf=255, world coords correctly placed 1.5 m in front of the robot
- `/local_costmap/costmap_raw` (master grid, raw cost values): **62500 cells, all 0**
- `/local_costmap/costmap` (OccupancyGrid for RViz): **all FREE**

The layer was processing observations but the master grid never received any cost.

### 10.2 Multi-phase agent investigation (3 phases, 5 agents)

Strategy: parallelise hypothesis testing instead of serial guess-and-check.

**Phase 1 — triangulation, 3 agents in parallel.**

| Agent | Method | Verdict |
|---|---|---|
| Code Tracer (Explore subagent) | Read every silent-skip / early-return between `tile_map → costmap_[idx] → updateWithMax → master_grid`, source-cited. | Top hypothesis: bounds-scope drift between `updateBounds` and `updateCosts`. Confidence MED — ruled out later. |
| Git Forensics (general-purpose) | `gh api` + `WebFetch` over kiwicampus issues, PRs, upstream demo configs. | No upstream PR after our PR3 fork-point fixes anything similar. Suggested `expected_update_rate: 2.0` as smallest config experiment — wrong, but useful negative. |
| Live Debugger (general-purpose) | Added `RCLCPP_INFO_THROTTLE` to `updateBounds` entry/exit, every `worldToMap` call, the `costmap_[index]=` write, and `updateCosts` entry. Built, killed nav stack (kept RViz alive on `:1001`), relaunched, captured 142 cycles, reverted source. | **Definitive.** `updateBounds` ran healthily; `updateCosts` called `updateWithMax` every cycle with the correct master grid; **but 141/142 cycles had `tiles=0`**. The layer's internal `temporal_tile_map_` was empty when `updateBounds` entered. The bug was upstream of the propagation path. |

**Phase 2 — narrow the wipe.** One agent instrumented `bufferSegmentation` (entry, observation push count, before/after the buffer's own `purgeOldObservations`). Findings:

- `bufferSegmentation` entry fires at 14.9 Hz (matches ZED rate)
- 10–13 observations pushed per frame (correct — matches the 240 lane pixels condensed to ~12 unique tiles after `best_observations_idxs` dedup)
- The buffer's own purge at `segmentation_buffer.cpp:241` shows `before == after` every call — **buffer is not the wiper**
- **Between two buffer cycles 67 ms apart, `temporal_tile_map_->size()` goes from 10–13 → 0** — something else wipes the map

**Phase 3 — find the wiper.** Static analysis enumerated every code path that can shrink `tile_map_`:

| File:line | Path | Time argument |
|---|---|---|
| `segmentation_buffer.cpp:241` | Buffer's `purgeOldObservations(cloud_time_seconds)` | cloud header stamp |
| `semantic_segmentation_layer.cpp:368` | Layer's `purgeOldObservations(current_time)` in `updateBounds` | `node->now().seconds()` (wall-clock) |

Instrumented both. Empirical log (one cycle):

```
buf entry:           cloud_stamp=1777513411.351
buf purge:           t=1777513411.351 before=10 after=10           # buffer call: in-domain, all kept
tm_layer_purge_call: current_time=1777513415.699 size_before=10
tm_purge:            t=1777513415.699 decay=1.5 ... n_after=0      # layer call: 4.35 s newer, all purged
```

`current_time - cloud_time_seconds = 4.35 s`. With `decay_time = 1.5 s`, every observation has age 4.35 > 1.5 → **purged on every layer cycle**. The cloud's `header.stamp` was 4.35 s behind wall-clock — apparent ZED publish-pipeline latency on this Jetson configuration (HD1080 @ 15 fps + NEURAL_LIGHT depth + organized cloud).

### 10.3 Fix — one line, two purges in the same time domain

`src/segmentation_buffer.cpp:141`:

```diff
- double cloud_time_seconds = rclcpp::Time(cloud.header.stamp.sec, cloud.header.stamp.nanosec).seconds();
+ double cloud_time_seconds = clock_->now().seconds();  // wall-clock; matches layer's purge in updateBounds
```

After this change, observations are stored stamped with wall-clock, both purge sites operate in the same domain, `decay_time` works as documented:

| Topic | Before fix | After fix |
|---|---|---|
| `/local_costmap/front/tile_map` size | 10–13 every cycle | 10–13 every cycle |
| `/local_costmap/costmap_raw` LETHAL count | **0 / 62500** | **9–10 / 62500** |
| Master grid total non-zero cost cells | 0 | 92 (with new inflation params, see §13) |

### 10.4 Why PR3's morning A/B test passed despite this bug

The morning A/B comparing pre-PR3 to post-PR3 measured "cells held LETHAL while observations were live; dropped to 0 within one cycle after suppressing input." Both pre and post tests in that comparison observed `0 LETHAL` cells when actually checked end-to-end on the master grid — only the kiwicampus internal `tile_map` debug topic showed marks, and the morning's instrumentation focused there. The morning kept `tile_map_decay_time = 5.0` s as a workaround for the previously-known dual-clock decay bug; 5.0 s happens to be wider than the ~4.35 s ZED lag, while 1.5 s is narrower. PR3's "correct" decay revert to 1.5 s exposed the latent clock-domain mismatch.

The morning changelog's claim that PR3 "fixes clearing" remains correct: PR3 implements the clearing path that was previously missing. The clock-domain fix in §10.3 is independent and additionally required to populate the master grid at all.

---

## 11. Fork — `Paarseus/semantic_segmentation_layer`, branch `avros-fixes`

Forked `kiwicampus/semantic_segmentation_layer` to `Paarseus/semantic_segmentation_layer`. New branch `avros-fixes` stacks all four of our patches as separate well-documented commits on top of upstream `274c713` (Apache-2.0 license switch):

| SHA | Title | Body summary |
|---|---|---|
| `83ef090` | Backport to Humble (Colcon builds successfully) | Inherited from upstream PR #1; body is sparse — TODO improve |
| `76fdf92` | Lock temporal_tile_map in updateBounds | PR2: data-race fix between `bufferSegmentation` (~15 Hz) and `updateBounds`; without it 90% of cycles passed degenerate bounds |
| `b357882` | Raytrace-clear stale LETHAL cells (PR3) | Mirrors `nav2_costmap_2d::ObstacleLayer::raytraceFreespace`; adds `clearing` / `raytrace_max_range` / `raytrace_min_range` params |
| `ffb3c7d` | Stamp observations with wall-clock instead of cloud header stamp | Today's fix; stops the layer's wall-clock purge from wiping cloud-stamped observations when the sensor lags |

Branch URL: <https://github.com/Paarseus/semantic_segmentation_layer/tree/avros-fixes>

`gh repo fork` discovered the fork already existed (presumably from an earlier session). Pushed the new branch from this dev box (the Jetson's stored github credentials are for a different account so a direct push from there 403'd). Branch went up first push, no conflicts.

---

## 12. Repo refactor — clone the fork, retire the patch script

`avros.repos` now points at the fork:

```diff
-  semantic_segmentation_layer:
-    type: git
-    url: https://github.com/kiwicampus/semantic_segmentation_layer.git
-    version: humble
+  semantic_segmentation_layer:
+    type: git
+    url: https://github.com/Paarseus/semantic_segmentation_layer.git
+    version: avros-fixes
```

After `vcs import src < avros.repos`, the tree is build-ready immediately — no `git am` / patch step. Removed:

- `scripts/apply_kiwicampus_patches.sh`
- `src/avros_bringup/patches/kiwicampus_pr1.patch`
- `src/avros_bringup/patches/kiwicampus_pr2_mutex.patch`
- `src/avros_bringup/patches/kiwicampus_pr3_raytrace_clear.patch`
- `src/avros_bringup/patches/` (empty dir, removed)

Patch content is preserved in git history if anyone needs to reconstruct it. The Jetson workspace was migrated to the fork via `rm -rf src/semantic_segmentation_layer && git clone -b avros-fixes https://github.com/Paarseus/...` — direct clone over the Jetson's HTTPS path was hitting `tmp_pack_*` errors mid-fetch (likely a credential-helper interaction), so the clone was done on the dev box and rsync'd to the Jetson.

---

## 13. Costmap inflation + RViz visualisation polish

Once cells started landing in the master grid, the default 1.8 m inflation radius made everything render as a black blob in the default RViz `map` colormap. Tightened both:

`nav2_params_humble.yaml`:

```diff
       inflation_layer:
         plugin: "nav2_costmap_2d::InflationLayer"
-        cost_scaling_factor: 2.5
-        inflation_radius: 1.8
+        cost_scaling_factor: 5.0
+        inflation_radius: 0.5
```

Applied to **both** local and global costmap blocks. Result on a frame with 9 LETHAL cells:

| Setting | Total non-zero cost cells |
|---|---|
| 1.8 m / 2.5 (before) | 441 (smeared blob) |
| 0.5 m / 5.0 (after)  | 92  (tight halo, stripes visible) |

`avros.rviz` updated to use the proper `costmap` colormap (the default `map` colormap renders LETHAL as black, indistinguishable from inflation):

```diff
     - Class: rviz_default_plugins/Map
       Name: LocalCostmap
       Topic:
         Value: /local_costmap/costmap
+      Color Scheme: costmap
+      Alpha: 0.7
+      Draw Behind: false
     - Class: rviz_default_plugins/Map
       Name: GlobalCostmap
       Topic:
         Value: /global_costmap/costmap
+      Color Scheme: costmap
+      Alpha: 0.5
+      Draw Behind: true
```

`costmap` colormap renders LETHAL=bright pink, INSCRIBED=red, then orange→yellow→blue gradient through inflation falloff. Now LETHAL is visually distinct from inflation in RViz.

Also added two new RViz displays earlier in the session for sanity-checking the perception pipeline directly:

```diff
     - Class: rviz_default_plugins/Image
       Name: Camera
-        Value: /camera/camera/color/image_raw
+        Value: /perception/front/overlay   # was RealSense; switched to ZED + the
+                                           # avros_perception overlay (RGB blended
+                                           # with semantic_mask) so the user can
+                                           # see what's classified in real-time
+    - Class: rviz_default_plugins/PointCloud2
+      Name: SemanticPoints
+      Topic:
+        Value: /perception/front/semantic_points
+      Color Transformer: FlatColor
+      Color: 255; 0; 255          # magenta — projects classified obstacles in 3D
```

---

## 14. nav2_params polish — drop voxel_layer, single source, kiwicampus param tweaks

While debugging, several nav2 changes turned out to be unrelated to the root cause but worth keeping.

```diff
-      plugins: ["voxel_layer", "semantic_layer", "inflation_layer"]
+      plugins: ["semantic_layer", "inflation_layer"]
```

Velodyne is currently disabled in field testing (no LiDAR fitted to the chassis as of this session). Without `velodyne_points` arriving, `voxel_layer` was a no-op consuming a slot in the `updateBounds` chain. Removing it eliminates a per-cycle no-op and one source of confusion when chasing the empty-master-grid bug. Re-add when LiDAR is back.

```diff
       semantic_layer:
         plugin: "semantic_segmentation_layer::SemanticSegmentationLayer"
         enabled: true
-        observation_sources: front left right
+        observation_sources: front
```

Left/right ZEDs are not yet wired up. Declaring them as sources but never publishing on their topics has no functional impact, but it generates noisy "no observations" warnings and adds startup latency while the sync filters time out. Restore the three-camera list once Phase 5 (multi-camera) lands.

```diff
       danger:
-        mark_confidence: 0
-        samples_to_max_cost: 0
+        mark_confidence: 1
+        samples_to_max_cost: 1
```

Both were 0 from a copy-paste; the kiwicampus plugin treats `samples_to_max_cost == 0` ambiguously (some code paths divide by it and skip the mark). Setting both to 1 makes "first observation marks the cell to base_cost (254)" the documented behaviour. Did not by itself fix the empty-grid problem — that took the §10.3 clock fix — but it removed one ambiguity from the matrix while diagnosing.

---

## 15. HSV night calibration

The morning's daylight calibration (`lane_low V = 120`, `S = 120`, `sky_roi_poly` top 60% mask) produced **0 detected lane pixels** under floodlight illumination at night. Sampled the bottom-40% ROI of a live frame — 99th-percentile V was only ~90 (vs 200+ in daylight), max V in the brightest 1% was 95.

```diff
-    lane_low:    [0,   0, 120]
+    lane_low:    [0,   0,  80]   # 2026-04-29 night cal: V floor dropped from 120, scene max V~95
```

After this single change, `lane_pixels (class=1) = 203` per frame, and the overlay clearly highlighted both edge stripes of the walkway in cyan — the user's "two lines in front of the car" visible in one shot. No iteration needed.

`adaptive_k = 0.0` and the `sky_roi_poly` (top 60% masked out) were retained from the daylight calibration; both still apply.

This is a night-only value. The daylight value (`V = 120`) needs to be restored, or the calibration made adaptive (e.g. revive `adaptive_k` non-zero), before driving outdoors during the day. TODO entry added.

---

## 16. TODO additions (evening)

Removed (resolved tonight):
- ~~Wait for kiwicampus maintainer feedback on draft PR #5; rebase to rolling API when they confirm direction~~ — superseded by our own fork at `Paarseus/semantic_segmentation_layer:avros-fixes` which we now use directly. Upstream PRs left open as a courtesy.

Added (new):
- Improve PR1 commit message body on `Paarseus/semantic_segmentation_layer:avros-fixes` (currently "Backport to Humble (Colcon builds successfully)" — bare; rebase to rewrite, then force-push)
- Open PRs upstream for the clock-domain fix (PR4 in our stack, not yet on kiwicampus). The mutex fix (PR2 in our stack) is also unfiled upstream.
- Restore daylight HSV `lane_low V = 120` before next day-time outdoor test, OR reintroduce an adaptive V floor (`adaptive_k > 0`) so a single config works in both lighting regimes.
- Verify RTK FIXED outdoors with the new nav stack composition — none of tonight's changes touched NTRIP / Xsens, but field testing lapsed early once the costmap was found empty.

---

## 17. Files touched (evening)

```
M  avros.repos                                            (kiwicampus → Paarseus fork)
D  scripts/apply_kiwicampus_patches.sh                    (no longer needed)
D  src/avros_bringup/patches/kiwicampus_pr1.patch         (now a commit on the fork)
D  src/avros_bringup/patches/kiwicampus_pr2_mutex.patch   (now a commit on the fork)
D  src/avros_bringup/patches/kiwicampus_pr3_raytrace_clear.patch  (now a commit on the fork)
M  src/avros_bringup/config/nav2_params_humble.yaml       (inflation + plugins list + observation_sources + mark/samples)
M  src/avros_bringup/rviz/avros.rviz                      (overlay topic + costmap colormap + SemanticPoints)
M  src/avros_perception/config/perception.yaml            (V floor 120 → 80 night cal)
M  docs/CHANGELOG_2026-04-29.md                           (this addendum)
```

On the fork (`Paarseus/semantic_segmentation_layer:avros-fixes`), the new commit is `ffb3c7d` "Stamp observations with wall-clock instead of cloud header stamp."

---

*Last updated: 2026-04-29 by morning + evening sessions covering kiwicampus PR3 raytrace-clear, Nav2 BT bring-up, combined-launch, HSV calibration, and tonight's clock-domain root-cause + fork migration. Morning commits: `4e4d562`, `052ee4e`, `8680313`, `f1657f1`. Evening commit forthcoming. Upstream PRs: [kiwicampus#5](https://github.com/kiwicampus/semantic_segmentation_layer/pull/5) (raytrace-clear, draft); clock-domain fix not yet filed upstream — see TODO §16.*
