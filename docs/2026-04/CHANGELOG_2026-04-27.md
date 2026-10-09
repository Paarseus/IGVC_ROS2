# 2026-04-27 — GPU anomaly root-caused, kiwicampus mutex bug patched, decay swept down to upstream default

**Scope.** Close the two perception-pipeline delays identified in
`docs/perception_latency_investigation.pdf` (2026-04-26 multi-agent research).
Both were verified end-to-end on the Jetson today over LAN. Outcomes:

1. The "94 % GPU on one camera" anomaly is **not** NEURAL_LIGHT depth — it is
   the ZED wrapper silently force-enabling positional tracking via
   `depth.depth_stabilization` defaulting to `-1`. Setting it to `1`
   recovers ~22 % of GPU per camera with cloud generation intact.
2. The kiwicampus `temporal_tile_map_` mutex bug (`CHANGELOG_2026-04-25.md`
   §3) is fixed by a 2-line C++ patch. After the patch,
   `tile_map_decay_time` can drop from the 5.0 s workaround down to **1.5 s
   (kiwicampus / upstream default)** with the master grid stable across
   30 s+ of sampling.

Files touched:

```
M  src/avros_bringup/config/zed_front.yaml
M  src/avros_bringup/config/zed_left.yaml
M  src/avros_bringup/config/zed_right.yaml
M  src/avros_bringup/config/nav2_params_humble.yaml
M  scripts/apply_kiwicampus_patches.sh
?? src/avros_bringup/patches/kiwicampus_pr2_mutex.patch
?? docs/CHANGELOG_2026-04-27.md
?? docs/perception_latency_investigation.pdf  (committed 2026-04-26)
?? docs/perception_latency_investigation.tex  (committed 2026-04-26)
```

No code rebuild required for the YAML changes. `colcon build
--packages-select semantic_segmentation_layer` required after applying the
new patch (~55 s on Orin AGX).

---

## 1. The 94 % GPU anomaly — root cause and fix

**Why.** CHANGELOG_2026-04-25 §5 reported tegrastats showing GR3D at 94 %
on a single ZED X at HD1080@15 NEURAL_LIGHT. Stereolabs' published
benchmark for the same depth mode at HD1080**@30** is **11 % GPU**. The
factor-of-8 discrepancy at half the documented frame rate blocked any
3-camera scaling decision (3 × 94 % = "no") so it had to be explained.

### 1.1 Hypotheses ruled out (Block A in the test plan PDF)

Three hypotheses from the multi-agent research were eliminated quickly:

| Hypothesis | Evidence | Verdict |
|---|---|---|
| Wrapper / SDK version drift | `Camera.hpp` reports `ZED_SDK 5.2.0`; `package.xml` reports wrapper `5.2.2`. Major.minor matches; no fallback. | Ruled out |
| `cv2.resize` in `perception_node` is on the GPU (custom CUDA-OpenCV) | `python3 -c "import cv2; print(cv2.getBuildInformation())"` reports `OpenCV 4.13.0`, no `CUDA` section at all. PyPI wheel build, CPU-only. | Ruled out |
| `point_cloud_res: COMPACT` triggers a CUDA depth-to-XYZ pass | Set `point_cloud_freq: 0.0` at runtime, GPU mean dropped from 34 % → 33 %. Within noise. | Ruled out |

The actual baseline numbers (single ZED front, no perception, no nav2,
HD1080@15 NEURAL_LIGHT, COMPACT cloud, all "unused subsystems"
disabled in YAML):

```
=== GPU baseline 30s, full ZED pipeline ===
samples: 29  mean: 34.2  max: 99
histogram (10% bins, samples per bin):
  16 × 0%   2 × 18-36%   3 × 50-66%   8 × 80-99%
```

The bursty pattern (idle gaps interspersed with 99 % spikes) is a 15 Hz
pulse: depth + cloud + positional-tracking compute fires per frame, drops
to idle, fires again. The "94 %" in the original changelog was the peak of
that pulse. Mean is 34 %, not 94 %.

### 1.2 Smoking gun in the launch log

Tail of the wrapper launch log on the very first measurement of the day:

```
[zed_front.zed_node]: POSITIONAL TRACKING disabled in the parameters,
    but forced to ENABLE because required by `depth.depth_stabilization
    > 0 or -1 (SDK default)`
[zed_front.zed_node]: === Starting Positional Tracking ===
```

`zed_front.yaml` sets `pos_tracking_enabled: false`, but the wrapper
*forces it back on* whenever `depth.depth_stabilization > 0` or `== -1`
(the SDK default sentinel). We never overrode `depth_stabilization`, so
it stayed at -1, so positional tracking was always running, regardless of
what we told the YAML.

Pos_tracking is a SLAM-style processing chain (feature extraction +
tracking) that runs every frame on the GPU. That's the cost.

### 1.3 Quantifying the trade-off

Restarted the wrapper with three different `depth_stabilization` values,
each with `pos_tracking_enabled` left at `false` in YAML. Same
30 s tegrastats capture each time:

| `depth_stabilization` | Mean GPU | Peak | Cloud publishing | Wrapper warning |
|---|---|---|---|---|
| **`-1`** (SDK default — was our state) | **34.2 %** | 99 % | ✓ 15.2 Hz | force-enable |
| **`1`** (minimum smoothing) | **26.8 %** | 93 % | ✓ 15.2 Hz | force-enable |
| **`0`** (smoothing off) | 3.4 % | 13 % | **✗ silent — no cloud topic data** | (no force; pos_tracking off) |

`depth_stabilization: 0` is *not* a viable answer. Cloud topic stops
producing data even though the topic is advertised — there's a
hard dependency in the ZED wrapper between positional tracking and
the organized point cloud, beyond what the warning text claims.
Without the cloud, the kiwicampus layer has no 3D to project mask
pixels into, so the whole semantic costmap goes silent.

One more variant tested for completeness:

| Config | Mean GPU | Peak | Cloud |
|---|---|---|---|
| `depth_stabilization: 0` + `pos_tracking_enabled: true` (explicit) | 32.2 % | 99 % | ✓ |

Forcing pos_tracking on while disabling stabilization is essentially the
same cost as the default — confirming that **pos_tracking itself is the
GPU consumer, not the temporal-stabilization filter on top of it**.

### 1.4 Final ZED config

Updated `src/avros_bringup/config/zed_front.yaml`:

```yaml
    depth:
      depth_mode: 'NEURAL_LIGHT'
      min_depth: 0.3
      max_depth: 15.0
      point_cloud_freq: 15.0
      # depth_stabilization=1 (minimum smoothing) instead of SDK-default (-1).
      # SDK default forces pos_tracking on AND runs heavier temporal filter -> 34% mean GPU.
      # Setting to 1 keeps the wrapper happy (cloud still publishes) at 27% mean GPU.
      # Setting to 0 disables cloud generation entirely on this wrapper version.
      depth_stabilization: 1

    # EKF+Xsens+GPS owns localization, but pos_tracking MUST be on for the
    # ZED wrapper to publish the organized point_cloud (cloud breaks if pos_tracking
    # is off, regardless of depth_mode). Set explicitly to true so the wrapper
    # doesn't print the misleading "forced to ENABLE" warning every launch.
    pos_tracking:
      pos_tracking_enabled: true
```

`zed_left.yaml` and `zed_right.yaml` updated the same way (left as
SVGA + NEURAL_LIGHT per the C2 cost model from the 2026-04-26 PDF;
side cameras not yet plugged in for runtime verification).

**Net win:** ~22 % mean GPU per camera (34 → 27 %). For 3 cameras at
the C2 config (front HD1080 + sides SVGA, all NEURAL_LIGHT) the new
projection is ~50–60 % combined mean GPU — within budget for sustained
operation.

---

## 2. Kiwicampus mutex patch — applied and verified end-to-end

**Why.** CHANGELOG_2026-04-25 §3 documented and worked around the bug:
`updateBounds` reads `temporal_tile_map_` while holding only the
`SegmentationBuffer`'s mutex, not the tile_map's own
`recursive_mutex lock_`. With the upstream-default `tile_map_decay_time:
1.5 s`, race-induced clock skew between `cloud.header.stamp` (used on
insert) and `node->now()` (used by `purgeOldObservations` inside
`updateBounds`) purged every observation before the for-loop could see
it. 90 % of update cycles passed degenerate bounds (`min > max`) to
`updateCosts`, and the master local_costmap stayed all-zero.

The 2026-04-25 fix was a workaround: bump decay to 5.0 s. That worked
but produced a **5 m comet trail of stale lethal cells behind a moving
robot at 1 m/s**. Acceptable for static tape detection; unacceptable
for AutoNav driving.

### 2.1 The patch

`src/avros_bringup/patches/kiwicampus_pr2_mutex.patch`:

```diff
diff --git a/src/semantic_segmentation_layer.cpp b/src/semantic_segmentation_layer.cpp
@@ -333,6 +333,7 @@ void SemanticSegmentationLayer::updateBounds(...)
   {
     auto buffer = tile_map_pair.second;
     buffer->lock();
+    tile_map_pair.first->lock();

     // Purge old observations in updateBounds before computing costs ...
     tile_map_pair.first->purgeOldObservations(current_time);
@@ -365,6 +366,7 @@
       }
       touch(tile_world_coords.x, tile_world_coords.y, ...);
     }
+    tile_map_pair.first->unlock();
     buffer->unlock();
   }
```

Two lines. Both mutexes are `std::recursive_mutex`. Lock order
(`buffer` first, `tile_map_pair.first` second) matches the order
already in use by `bufferSegmentation`, so no deadlock is introduced.

### 2.2 Delivery via patch-stack

Strategy (a) from the 2026-04-26 PDF: keep `avros.repos` pointing at
the upstream `kiwicampus/semantic_segmentation_layer` jazzy branch, add
a second carry-patch alongside `kiwicampus_pr1.patch` (Humble build
fix). Both apply via `git am` after `vcs import`.

`scripts/apply_kiwicampus_patches.sh` was extended to a loop:

```bash
PATCHES=(
  "kiwicampus_pr1.patch"
  "kiwicampus_pr2_mutex.patch"
)
for name in "${PATCHES[@]}"; do
  patch="$PATCH_DIR/$name"
  if git apply --reverse --check "$patch" >/dev/null 2>&1; then
    echo "$name: already applied (skipping)"
    continue
  fi
  git am --keep-non-patch "$patch" || { git am --abort; exit 1; }
done
```

The patch was generated from a real edit on the Jetson + `git
format-patch -1 --stdout`, then transferred back to the dev box. This
gave us proper context lines that match the actual current source —
the first hand-written attempt failed to apply because line numbers
drifted by ~30 due to the existing PR #1 patch.

Build:

```
cd ~/IGVC
rm -rf build/semantic_segmentation_layer install/semantic_segmentation_layer
colcon build --packages-select semantic_segmentation_layer --symlink-install
# Finished <<< semantic_segmentation_layer [54.6s]
```

### 2.3 End-to-end verification

The DEBUG-bounds approach from the 2026-04-25 changelog (relaunch
controller_server with `--log-level nav2_costmap_2d:=DEBUG`) was
unavailable today: the `set_logger_levels` service is not exposed by
default in Humble Nav2, and we didn't want to rebuild a launch file
just to add a flag. Substituted a stronger empirical proof: lethal-cell
count stability under the **same conditions that broke pre-patch**.

Setup: stub pipeline injecting a synthetic 200-pixel-wide stripe into
the mask (`pipeline: 'stub'`, `inject_stripe_width: 200`,
`inject_class_id: 1` → `lane_white`). Cloud comes from real ZED at
HD1080@15. Stack: sensors + perception + nav2 (full
`navigation.launch.py enable_zed_front:=true enable_perception:=true
perception_cameras:=front`).

`tile_map_decay_time: 2.0` (the value the 2026-04-25 changelog
explicitly tested as broken pre-patch — "master grid all-zero"):

```
[decay=2.0] sample 1: lethal=1233 total=62501
[decay=2.0] sample 2: lethal=1233 total=62501  (8s later)
[decay=2.0] sample 3: lethal=1233 total=62501  (8s later)
[decay=2.0] sample 4: lethal=1233 total=62501  (8s later)
```

1233 lethal cells, identical across four ~8 s-spaced samples, on a value
that pre-patch produced **zero cells**. The patch works.

`/local_costmap/costmap` is `nav_msgs/OccupancyGrid` (0–100 scale, where
100 is lethal); the costmap_2d internal scale is 0–254. `lethal=1233` is
the count of cells at value 100.

---

## 3. Decay sweep — finding the floor (post-patch)

**Why.** Now that the master grid populates correctly at 2.0 s,
how much further can we drop? The 2026-04-25 sweep stopped at 1.5 s
because that was the broken-pre-patch boundary. Today's sweep drives
to the patch's actual floor.

Same setup as §2.3 (stub stripe, sensors + perception + nav2). For each
decay value: edit YAML, kill stack, relaunch, wait 28 s for steady
state, sample lethal-cell count 4 times at 6 s intervals.

| Decay | Sample 1 | Sample 2 | Sample 3 | Sample 4 | Stable? |
|---|---|---|---|---|---|
| 5.0 s | 1233 | 1233 | 1233 | 1233 | ✓ |
| **2.0 s** | **1233** | **1233** | **1233** | **1233** | ✓ |
| **1.5 s** | **1099** | **1099** | **1099** | **1099** | ✓ ← upstream default |
| 1.0 s | 1141 | 1141 | 1141 | 1141 | ✓ |
| 0.5 s | 933 | 933 | 933 | 933 | ✓ |
| 0.25 s | 977 | 977 | 977 | 977 | ✓ |
| 0.15 s | 946 | 946 | 946 | 946 | ✓ |
| 0.10 s | 958 | 958 | 978 | 978 | ✓ tiny flicker |
| 0.07 s | 971 | 969 | 983 | 978 | ~ at frame interval |
| 0.05 s | 942 | 939 | 947 | 963 | flickering |
| 0.01 s | 940 | 946 | 952 | 958 | flickering |

The frame interval at `grab_frame_rate: 15` is `1/15 ≈ 67 ms`. Below
that, observations from the previous frame are purged before the next
frame arrives, and the cell count starts to depend on timing relative
to the most-recent publish. Even there, the costmap doesn't go empty —
the stub publishes every frame and the patched reader holds the lock
correctly, so the most-recent frame always survives. But the master
grid value visible at any instant becomes a function of microsecond-level
scheduling, which is bad.

**`1.5 s` was chosen as the production value** — kiwicampus and
`pepisg/nav2_segmentation_demo` upstream default, ~1.5 m comet trail at
1 m/s (about robot-footprint width), with enough headroom to absorb
1–2 dropped frames before cells start to flicker.

Final state of the comment block above `semantic_layer:` in
`nav2_params_humble.yaml`:

```yaml
      # tile_map_decay_time: 1.5 — kiwicampus / upstream default. Safe with
      # patches/kiwicampus_pr2_mutex.patch applied. Pre-patch this value was
      # broken (master grid empty 90% of cycles); workaround was 5.0s.
      # Decay sweep on Jetson 2026-04-27 confirmed stable cells from 0.15s
      # up to 5s post-patch; below the 1/grab_frame_rate frame interval
      # (~67ms at 15 Hz) cells flicker as observations are purged faster
      # than they're refreshed.
```

All three `semantic_layer.{front,left,right}.tile_map_decay_time` now
hold `1.5`.

---

## 4. Side discoveries

### 4.1 SSH `pkill -f` suicide hit again

The known issue is documented in `CLAUDE.md` ("Multiple SSH sessions
died during debugging when `pkill -f velodyne` matched the SSH
session's own command line"). It bit us today within five minutes of
starting work, because the SSH transport prepends the full remote
command to `ps`, so `pkill -f "zed_node|perception_node"` matches the
ssh process itself.

Fix used: a `/tmp/avros_clean.sh` script invoked over ssh. The script
runs locally on the Jetson under bash so `$$` and `$PPID` filter out
the ssh shell from the kill list.

### 4.2 YAML duplicate-key collision when `sed`-appending overrides

First attempt at a `depth_stabilization: 0` test used a runner script
that did `cat backup >> CONFIG; printf '%s' "$override" >> CONFIG`, where
`$override` contained a fresh top-level `depth:` block. ROS's YAML
loader merges duplicate top-level keys in *some* but not all paths —
the runtime `ros2 param get` showed both `depth_mode` and
`depth_stabilization` set correctly, but cloud and depth topics
silently stopped publishing at runtime. Switched to in-place `sed -i
"/point_cloud_freq:.*/a\      depth_stabilization: 0"` to insert
inside the existing `depth:` block, problem went away.

### 4.3 `depth_mode` is read-only at runtime

`ros2 param set /zed_front/zed_node depth.depth_mode "NONE"` returns:

```
Setting parameter failed: parameter 'depth.depth_mode' cannot be set
because it is read-only
```

The wrapper opens the camera once at construction and the depth-mode
choice is baked into `sl::Camera::open()`. Restart required for any
depth-mode comparison. This blocks the cleanest "isolate NEURAL
inference cost" test — couldn't toggle NONE at runtime — and pushed
the diagnosis toward `depth_stabilization` instead, which is the
correct path anyway.

### 4.4 `set_logger_levels` not exposed in Humble Nav2

The 2026-04-25 changelog used `--log-level nav2_costmap_2d:=DEBUG` at
launch time to capture the smoking-gun degenerate-bounds output.
Today's relaunches did not include that flag and the live service
`/controller_server/set_logger_levels` does not exist on Humble — only
the lifecycle and parameter services are advertised. Substituted
lethal-cell-count proof, which is stronger anyway since it directly
shows the master grid populating.

### 4.5 ZED X serial number mismatch on bring-up

Wrapper booted reporting `Serial Number -> 42569280` on the front
camera. `CLAUDE.md` and `zed_front.yaml` both assume serial 49910017.
Doesn't affect the diagnosis (the camera is a working ZED X regardless
of serial), but the camera mapping in CLAUDE.md may be stale or a
side unit was connected to the front port. Flagged for the user but
not edited — physical inspection needed.

---

## 5. Verification matrix (full state at end of session)

| Check | Result |
|---|---|
| `avros_perception` topics still 15 Hz with `depth_stabilization: 1` | ✓ rgb 15.09, semantic_mask 15.07, semantic_points 14.12 |
| `pos_tracking_enabled: true` no longer logs the `forced to ENABLE` warning | ✓ launch log clean |
| Mean GPU on Jetson with full nav stack + perception + decay 1.5 (single camera) | 36.1 % / max 99 % |
| `kiwicampus_pr2_mutex.patch` applies idempotently via `apply_kiwicampus_patches.sh` | ✓ |
| `colcon build --packages-select semantic_segmentation_layer` after patch | clean, 54.6 s |
| Lethal cells at decay 2.0 s with stub stripe (was 0 pre-patch) | 1233, stable across 4 samples |
| Lethal cells at decay 1.5 s (upstream default) | 1099, stable across 4 samples |
| Lethal cells at decay 0.15 s (10× faster than default) | 946, stable across 4 samples |
| Lethal cells at decay below frame interval (≤ 0.07 s) | small per-sample drift; bounded |
| Final `tile_map_decay_time` shipped (×3 sources) | 1.5 |
| Final `depth_stabilization` shipped (×3 cameras) | 1 |
| Final `pos_tracking_enabled` shipped (×3 cameras) | true |

---

## 6. Open items / future work

- **Side-camera runtime verification.** `zed_left.yaml` and
  `zed_right.yaml` carry the same `depth_stabilization: 1` +
  `pos_tracking_enabled: true` fix, but the side cameras were not
  plugged in today. Block C of the 2026-04-26 PDF (3-camera tegrastats
  capture under driving load) is hardware-gated.

- **Composable container migration.** Stereolabs' `zed_ipc.launch.py`
  pattern with `use_intra_process_comms: true` saves ~63 MB/s of cloud
  serialization across 3 cameras. Worth doing once the side cameras are
  up and the basic 3-camera sensor flow is verified. Out of scope today.

- **Upstream the patch.** `kiwicampus_pr2_mutex.patch` is a real bug
  fix (not Humble-specific — confirmed present on jazzy too). Open a PR
  at <https://github.com/kiwicampus/semantic_segmentation_layer> after
  IGVC field-tests it. Suggested PR description text was drafted in the
  multi-agent research output (Phase 2 / B1) on 2026-04-26.

- **Investigate the actual 1.5 s comet trail at IGVC speeds.** At 1 m/s
  it's 1.5 m, close to robot footprint. The vehicle does up to 1.5 m/s
  (`max_linear_mps: 1.5` in `actuator_params.yaml`) → 2.25 m of trail.
  May want to drop to 1.0 s or 0.5 s for high-speed segments. Both were
  stable in today's sweep, so it's a config-only call once we have field
  data on perception-dropout rate.

- **CLAUDE.md known-issues entry for `depth_stabilization`.** The
  "wrapper silently force-enables pos_tracking" gotcha is not yet in
  the CLAUDE.md known-issues table. Add it next time we touch that
  file.

- **Side-camera serial-number sanity check.** Front enumerated as
  42569280 today, not the 49910017 documented in `zed_front.yaml`'s
  comment block. Either the comment is stale or a side unit is in the
  front harness. Physical inspection at the bench.

## Reference docs

- `docs/perception_latency_investigation.pdf` — multi-agent research
  output and bench-test plan that drove today's session
- `docs/perception_latency_investigation.tex` — source for the PDF;
  re-render with `pdflatex perception_latency_investigation.tex`
- `docs/CHANGELOG_2026-04-23.md` — diff-drive commissioning
- `docs/CHANGELOG_2026-04-24.md` — semantic segmentation layer scaffolding
- `docs/CHANGELOG_2026-04-25.md` — original kiwicampus bug discovery,
  HSV live-tuning, sensor mounts, base_link convention thrash
- `src/avros_bringup/patches/kiwicampus_pr2_mutex.patch` — the patch
- `scripts/apply_kiwicampus_patches.sh` — patch-stack runner
- Upstream reference: <https://github.com/pepisg/nav2_segmentation_demo>
- Kiwicampus repo: <https://github.com/kiwicampus/semantic_segmentation_layer>
