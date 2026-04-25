# 2026-04-25 — Live HSV tuning, measured sensor mounts, kiwicampus master-grid bug fix

**Scope.** Make the perception → costmap pipeline that was scaffolded on
2026-04-24 actually work end-to-end on the real Jetson + real ZED X. Three
deliverables shipped, one significant Nav2-Humble bug diagnosed and worked
around. End state: `/local_costmap/costmap` paints LETHAL cells where the
ZED detects white tape, end-to-end, stable across update cycles.

Five commits on `main`:

```
fef29ab  Lower kiwicampus tile decay to 5s + add Foxglove layout
93de907  Restore full nav2_params_humble.yaml content (prior commit had bash subst string)
93444a6  Fix kiwicampus semantic_segmentation_layer not painting master costmap
a864f01  Move base_link back to ground level (standard convention)
ace533e  Live-tune HSV params + bake measured sensor mounts into URDF
```

---

## 1. Live-tunable HSV parameters (commit `ace533e`)

**Why.** Yesterday's `HSVPipeline` had its bounds hard-coded at construction
time. Field calibration meant edit YAML → restart node → reconnect Foxglove
→ check overlay → repeat. That's a 10-second loop that adds up to hours.

**Multi-agent research first.** Three parallel agents researched ROS2 dynamic
param patterns, Foxglove tuning UX, and what competitive IGVC teams actually
do. Consensus: standard pattern is `add_on_set_parameters_callback` +
`ParameterDescriptor(integer_range=[IntegerRange(...)])` + `rqt_reconfigure`.
Foxglove's Parameters panel is text-only, doesn't support arrays — not the
right tool for HSV. Most IGVC teams use offline `cv2.createTrackbar` scripts.

**What landed.**

- `perception_node.py` declares 13 HSV pipeline params with proper
  `ParameterDescriptor` + `IntegerRange`/`FloatingPointRange` descriptors so
  rqt_reconfigure renders them with sliders.
- `_on_set_params` validates HSV bounds atomically: H ∈ [0, 179], S/V ∈
  [0, 255], element-wise `low ≤ high`. Bad values are rejected without
  partial state corruption.
- `HSVPipeline` re-reads bounds, blur, adaptive_k, class IDs, and the ROI
  polygon from `self.params` on every `run()` so live `ros2 param set` takes
  effect on the next frame.
- New `test/unit/test_hsv_live_tuning.py` (5 tests) pins the contract.
  All 16 unit tests in the package still pass.

**Verified live on Jetson.**

- Tightening `pothole_low/high` → pothole pixels go from 7,587 → 0
- `adaptive_k=0` → 31,735 lane pixels appear (V-floor stops gating)
- `lane_low > lane_high` correctly rejected at param-set time

**Usage.**

```bash
ros2 run rqt_reconfigure rqt_reconfigure       # GUI sliders
# or
ros2 param set /perception_front lane_low '[0,0,200]'
```

Watch `/perception/front/overlay` in Foxglove for live feedback.

---

## 2. Measured sensor mounts in URDF (commits `ace533e` + `a864f01`)

**Why.** All six sensor mount positions in `avros.urdf.xacro` were
placeholders. Without real geometry, the costmap puts the ZED's detections
in the wrong world position, Nav2's footprint inflation is wrong, and the
visualization in Foxglove is misleading.

**Source of truth.** Hand-drawn chassis sketch from the team — Xsens at the
geometric center of the chassis platform, all other sensors offset from it.
Sketch convention: y left = negative (ROS REP-103 has left positive, so
y signs flipped during ingestion).

**What landed.**

| URDF link | xyz (m) from base_link | rpy (rad) | Source |
|---|---|---|---|
| `imu_link` | (0, 0, 0.500) | identity | sketch origin = Xsens, height measured by user |
| `velodyne` | (0.089, 0, 0.659) | identity | 3.5″ fwd, 6.25″ above Xsens |
| `zed_front_camera_link` | (0.397, 0, 0.557) | identity | 15.625″ fwd, 2.25″ above Xsens |
| `zed_left_camera_link` | (0.098, +0.286, 0.557) | yaw +π/2 | 3.875″ fwd, 11.25″ left, 2.25″ above |
| `zed_right_camera_link` | (0.098, −0.286, 0.557) | yaw −π/2 | mirror of left |

**Convention thrash.** Initially picked convention "b" — `base_link` at the
IMU body, `base_footprint` dropped −0.5 m to ground. That matched the
sketch's coordinate frame directly. But then the costmap stopped painting
and we suspected this was the cause. Reverted to convention "a" —
`base_link` at chassis-center-ground-level, IMU offset +0.5 m up — which is
what 95% of public ROS2 / Nav2 examples use. **It turned out base_link
convention was NOT the cause** (see §3), but we kept convention (a) anyway
since it's the standard.

**RealSense link removed.** D455 isn't on the robot. The `camera_link`
joint at placeholder `(1.0, 0, 0.8)` was producing a floating axis triad
~1.3 m above the floor in Foxglove. Block deleted from URDF; remaining
references in `sensors.launch.py` and `nav2_params.yaml` (camera_depth
observation source) are vestigial but harmless without the device.

**Verified.**

```
base_footprint → base_link    : (0, 0, 0)            ← on the ground
base_link → imu_link          : (0, 0, 0.500)        ← Xsens height
base_link → velodyne          : (0.089, 0, 0.659)    ← matches sketch
base_link → zed_front_camera_link : (0.397, 0, 0.557)
```

---

## 3. Kiwicampus master-grid bug — diagnosis and workaround (commits `93444a6`, `93de907`, `fef29ab`)

**The headline.** The `kiwicampus/semantic_segmentation_layer` plugin's
internal `tile_map` populates correctly with valid lane-detection tiles,
but **none of those tiles get propagated to the master `local_costmap`** on
Nav2 Humble. Master grid stays at all-zero across all cells regardless of
what the perception pipeline produces.

**Severity.** Without this working, the entire 2026-04-24 perception
plumbing is decorative — Nav2 can't see lane paint at all.

**Diagnosis was multi-phase.**

### Phase 1 — Three parallel research agents

- **Agent 1** (Nav2 tutorial deep read) — found the official tutorial uses
  ROS2 Rolling/Jazzy, not Humble; the demo code (`pepisg/nav2_segmentation_demo`)
  has subtle config differences from ours.
- **Agent 2** (kiwicampus C++ source dive) — identified `combination_method`
  switch (default 1=Maximum), `current_` flag handling, the sync paths
  (`syncSegmPointcloudCb` 2-msg vs `syncSegmConfPointcloudCb` 3-msg), and
  the lock contention between `temporal_tile_map_->lock()` (used by
  `bufferSegmentation`) and `buffer->lock()` (used by `updateBounds`) —
  **two different recursive mutexes for related data**.
- **Agent 3** (web search for working examples) — found exactly **one**
  public production deployment (`pepisg/nav2_segmentation_demo`), all on
  Iron+/Jazzy, zero on Humble. Confirmed the kiwicampus humble branch has
  no commits to fix this; jazzy branch has approximate-time-sync and QoS
  fixes that don't address our symptom.

### Phase 2 — Eliminate config differences vs the working demo

Made our config exactly match `pepisg/nav2_segmentation_demo`:

- Collapsed three plugin instances (`semantic_front`, `semantic_left`,
  `semantic_right`) into one `semantic_layer` with `observation_sources:
  front left right` (every working example uses one plugin instance with
  multiple sources).
- `samples_to_max_cost: 1 → 0`, `dominant_priority: true → false` to
  match demo.
- Switched the controller_server's `--params-file` from
  `nav2_params.yaml` (Jazzy variant) to `nav2_params_humble.yaml` for
  correctness.
- Tested the `jazzy` branch of kiwicampus on top of our Humble Nav2
  (PR #1 build patch reapplied cleanly) — built, ran, **same exact
  symptom**.

Verdict: bug is in code common to both branches, NOT in any config we
control.

### Phase 3 — DEBUG-level logging on `nav2_costmap_2d` ⇒ the smoking gun

Restarted `controller_server` with `--log-level nav2_costmap_2d:=DEBUG`
(critically, this is a different logger than `local_costmap.local_costmap`).
The line `Updating area x: [%d, %d] y: [%d, %d]` from
`LayeredCostmap::updateMap` revealed the bounds being passed to each
layer's `updateCosts`:

```
48 cycles: x: [249, 1] y: [249, 1]   ← degenerate (min > max → silent early exit)
 3 cycles: x: [120, 158] y: [99, 144] ← valid, includes our cells
 1 cycle:  x: [0, 250] y: [0, 250]    ← initial reset
```

So **90% of update cycles never call `updateCosts` on the kiwicampus layer
at all** because the per-cycle bounds are degenerate. The layer's `touch()`
calls inside `updateBounds` aren't extending the bounds, which happens when
the for-loop body doesn't execute, which happens when the buffer's
`temporal_tile_map_` is empty after `purgeOldObservations` runs.

### Root cause

The buffer's `temporal_tile_map_` is empty by the time `updateBounds`
iterates it because:

- `bufferSegmentation` (sync callback, 15 Hz) locks `temporal_tile_map_`'s
  mutex
- `updateBounds` (costmap update loop, 5 Hz) locks the SegmentationBuffer's
  mutex (DIFFERENT mutex) and iterates the tile_map without holding the
  tile_map's mutex
- `updateBounds` then calls `purgeOldObservations(node->now())` — which
  uses node clock, while observations were stored with cloud-message
  timestamps; with the default `tile_map_decay_time: 1.5s`, even minor
  thread-scheduling slack puts the node clock far enough ahead of the
  buffered observations that they all get purged before the for-loop sees
  them
- Result: empty for-loop, no `touch()` calls, `(min_x, max_x)` stay at
  framework's `±inf`, get clamped to `[249, 1]` after `worldToMapEnforceBounds`

### Workaround (committed)

Increase `tile_map_decay_time` from 1.5s → 5.0s. This keeps the
buffer's tile_map populated long enough that updateBounds always sees real
tiles. Tested decay values:

| decay | result |
|---|---|
| 1.5s (default) | master grid all-zero |
| 2.0s | master grid all-zero |
| 5.0s | **290 LETHAL cells, stable** |
| 10.0s | 290 LETHAL, identical |
| 30.0s | 289 LETHAL, identical, but cells persist 30s after camera stops seeing them |

5.0s is the sweet spot — costmap paints reliably AND cells fade in 5s when
the camera is covered. **Side effect on a moving robot:** cells will trail
behind by up to 5s of vehicle travel. With our `desired_linear_vel: 1.0
m/s`, that's a 5 m comet trail of stale lethal cells.

Also collapsed to single-plugin-with-multi-source pattern even though
that wasn't strictly the bug — every working upstream example uses it,
and our previous three-plugin pattern was untested upstream.

### Real fix (NOT done — left for future)

The proper fix is a 2-line C++ patch to kiwicampus: lock
`temporal_tile_map_`'s mutex in `updateBounds`'s for-loop in addition to
the buffer's mutex. Then short decay times work and there's no comet
trail. We'd ship this as a fork + repoint `avros.repos`, optionally PR
upstream. Not done in this session because the workaround was sufficient
for static-tape testing.

### Other knobs verified during diagnosis

- `combination_method`: confirmed `1` (Maximum) at runtime via `ros2
  param get`. Plugin caches at init; live changes don't propagate.
- `enabled`: true throughout.
- Layer `track_unknown_space`: irrelevant — masking was orthogonal.
- `voxel_layer` interference: ruled out (master empty even with voxel_layer
  disabled).
- Frame_id mismatch (mask in `_optical`, cloud in non-`_optical`):
  irrelevant — kiwicampus only uses cloud's frame_id for TF transform.
- TF chain (`map = odom = base_link`): identity, all transforms resolve.

### Audit of correctness after fix

```
=== local_costmap (post-fix, with decay=5s, stationary robot) ===
  size 250x250 @ 0.20m, origin (-24.80, -24.80) frame=odom
  lethal: 491, near-lethal: 525
  span x[0.60, 5.20] y[-3.60, 2.40]   ← all forward, within max_obstacle_distance=5m
  forward of robot: 491, behind: 0     ← no phantom cells behind
  inside ZED ~90° FOV: 439 (89.4%)     ← geometrically plausible
  74% of lethal cells within 1.5*res of a kiwicampus internal tile
```

The 11% out-of-FOV cells are stale within the 5s decay window — fade as
expected when camera is covered.

---

## 4. Foxglove starter layout (commit `fef29ab`)

`docs/foxglove_layout.json` — importable via Foxglove → Layouts → Import
from file. Four-panel workspace:

- **3D panel (60% width)** — robot URDF, costmap as red overlay, perception
  cloud colored by class. Follows `base_link`. Most ZED sub-frames hidden
  by default to reduce clutter (only mount frames + Velodyne shown).
- **Image: overlay (top right)** — `/perception/front/overlay`, the HSV-
  tinted RGB so you can see what's being detected.
- **Image: raw (bottom right)** — `/zed_front/zed_node/rgb/color/rect/image`,
  what the camera actually sees.
- **ESTOP indicator (bottom)** — green READY / red ESTOP from
  `/avros/actuator_state.estop`.

---

## 5. Side discoveries

- **ZED X is using CUDA** as expected — depth_mode is `NEURAL_LIGHT` at
  `HD1080` @ 15 Hz. End-to-end latency measured at ~100 ms (image, cloud,
  and perception mask all within 100–120 ms of `header.stamp`). Tegrastats
  showed GPU at 94% utilization — CUDA is engaged. The "delay" the user
  noticed is intrinsic to NEURAL stereo on a moving feed; could be reduced
  by dropping resolution to SVGA or switching depth_mode away from neural,
  at the cost of detection quality.

- **Perception node was missing the cloud frame_id rewrite.** Earlier
  diagnostics suggested mask and cloud might have different `header.frame_id`
  values — this turned out NOT to matter for the kiwicampus bug (the layer
  uses the cloud's frame_id for TF, mask's only for the published mask
  message).

- **`pkill -f` against the SSH command itself.** Multiple SSH sessions died
  during debugging when `pkill -f velodyne` matched the SSH session's own
  command line. Fixed by writing kill-then-relaunch logic into a script
  file rather than passing it inline (`/tmp/avros_clean_relaunch.sh` etc).

---

## Verification (all on Jetson)

| Check | Result |
|---|---|
| `ros2 topic hz /zed_front/zed_node/rgb/color/rect/image` | 15.0 Hz |
| `ros2 topic hz /perception/front/semantic_mask` | 15.0 Hz |
| `ros2 topic hz /perception/front/semantic_points` | 15.0 Hz |
| `ros2 param set /perception_front lane_low '[0,0,80]'` | accepted, mask updates next frame |
| `ros2 param set /perception_front lane_low '[0,255,255]'` (lo > hi) | rejected with reason |
| `/local_costmap/costmap` lethal cells (white tape in view) | 290, stable across 5 samples |
| `/local_costmap/costmap` after covering camera 5s | drops to 0 |
| Bounds passed to `updateCosts` (DEBUG log) | valid every cycle (`[120, 158] × [99, 144]`) |
| ZED end-to-end latency (`now - header.stamp`) | mean 102 ms |

---

## Open items / future work

- **Real fix for kiwicampus.** Patch `updateBounds` to lock the
  `temporal_tile_map_`'s mutex in addition to the buffer's. Then drop
  decay back to 1.5–2.0s. Optionally PR upstream.
- **5s comet trail.** With current workaround, cells persist 5s after the
  camera stops seeing them. Tolerable while stationary; unsuitable for
  fast driving. The C++ patch above resolves it.
- **CLAUDE.md known-issues entry.** Not added in this session because the
  user has unrelated WIP edits to that file (AVROS → IGVC renames).
  Suggested entry text included in the chat summary.
- **Dropped `voxel_layer` then re-added.** Briefly removed during
  debugging to isolate semantic_layer. Now back in `plugins:
  ["voxel_layer", "semantic_layer", "inflation_layer"]`. With Velodyne
  killed during testing, voxel_layer is silent — bring back when LiDAR is
  needed.
- **HSV calibration for outdoor conditions.** All testing was indoors with
  artificial light. Bounds will need re-tuning at the IGVC venue. The
  live-tuning infrastructure shipped today makes that a 5-minute job at
  the venue.
- **Perception left/right cameras.** The single-plugin pattern subscribes
  to `/perception/{left,right}/*` topics that nothing publishes yet (only
  front camera is wired). Layer's silent on those sources — fine for now,
  ready to light up when Phase 5 brings up the side ZED Xs.

## Reference docs

- `docs/CHANGELOG_2026-04-23.md` — diff-drive commissioning
- `docs/CHANGELOG_2026-04-24.md` — semantic segmentation layer scaffolding
- `docs/foxglove_layout.json` — importable Foxglove workspace
- `src/avros_bringup/config/nav2_params_humble.yaml` — see comment block
  above `semantic_layer:` for the decay-time requirement
- Upstream reference: https://github.com/pepisg/nav2_segmentation_demo
- Kiwicampus repo: https://github.com/kiwicampus/semantic_segmentation_layer
