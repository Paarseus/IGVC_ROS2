# 2026-04-27 (afternoon) — Field session: slim camera→costmap launch, outdoor HSV calibration, overlay visibility fix

**Scope.** First field test of the perception stack on real outdoor pavement
with painted lane lines. Built an isolated launch for the camera→costmap
path (no LiDAR / no Nav2 planner / no EKF), drove the robot via WebUI from
a phone, calibrated HSV against actual sun-on-asphalt conditions, and made
the overlay actually visible in Foxglove. Several discoveries about the
kiwicampus layer's behaviour without odometry motion that are documented
here for future field sessions.

Companion to `docs/CHANGELOG_2026-04-27.md` (morning session — GPU
anomaly, kiwicampus mutex patch, decay sweep).

Files touched:

```
?? src/avros_bringup/launch/perception_test.launch.py    (new)
?? src/avros_bringup/config/perception_test_params.yaml  (new)
M  src/avros_perception/avros_perception/perception_node.py
M  src/avros_perception/config/perception.yaml
M  src/avros_perception/config/class_map.yaml
?? docs/CHANGELOG_2026-04-27_field.md                    (this file)
```

---

## 1. New slim launch for isolated camera→costmap testing

**Why.** Field testing the kiwicampus mutex patch and HSV pipeline against
real lane tape required isolating the camera→costmap path from the rest
of the stack. The full `navigation.launch.py` brings up sensors + EKF +
navsat + planner + BT + behaviors, all of which can fail or interfere
during early-stage perception tuning. Wanted a minimum-viable bring-up:
front camera + perception node + local_costmap with semantic_layer +
foxglove_bridge — and nothing else.

### 1.1 `perception_test.launch.py`

13-node stack:

- `robot_state_publisher` (URDF → TF)
- two static TFs: `map → odom` and `odom → base_link`, both identity
- `zed_camera.launch.py` for the front ZED X
- `perception.launch.py` (HSV pipeline by default)
- `controller_server` hosting only `local_costmap`
- `lifecycle_manager_perception_test`
- `foxglove_bridge` on `0.0.0.0:8765`

`enable_drive` arg (default `true`) optionally includes `webui.launch.py`
which brings up `actuator_node` + `webui_node` so the robot can be driven
from a phone joystick at `https://<jetson-ip>:8000` while watching the
costmap update in Foxglove. Set `enable_drive:=false` to suppress (e.g.
when running webui in a separate terminal or on a vehicle without motors
powered).

### 1.2 `perception_test_params.yaml`

Slim Nav2 config — only `controller_server` (with placeholder controller
plugins so it instantiates) and `local_costmap`. Costmap drops
`voxel_layer` entirely, so plugins are
`["semantic_layer", "inflation_layer"]` only. The kiwicampus
`semantic_layer` block matches the production `nav2_params_humble.yaml`
front source.

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      width: 20
      height: 20
      resolution: 0.1
      plugins: ["semantic_layer", "inflation_layer"]
      semantic_layer:
        plugin: "semantic_segmentation_layer::SemanticSegmentationLayer"
        observation_sources: front
        front:
          tile_map_decay_time: 1.5    # kiwicampus default — verified safe with patch
          ...
      inflation_layer:
        inflation_radius: 0.3
```

Usage:

```bash
ros2 launch avros_bringup perception_test.launch.py
# phone joystick: https://<jetson-ip>:8000
# foxglove:        ws://<jetson-ip>:8765
```

### 1.3 Network reachability gotcha

The Jetson has three network interfaces: WiFi (`10.110.132.84`),
vehicle subnet (`192.168.13.10`), and Tailscale (`100.93.121.3`). The
WiFi address only works for devices on the same SSID; for outdoor field
testing where there's no shared network, either:

- enable hotspot on the phone and connect the Jetson to it (Jetson then
  gets a `192.168.43.x` / `172.20.x.x` address)
- install Tailscale on the phone and use `100.93.121.3:8000`

Documented for next session — was the first thing that broke during
bring-up today.

---

## 2. Outdoor HSV calibration — what we learned

**Why.** The HSV thresholds in `perception.yaml` were tuned indoors against
artificial light and white walls. First test on real sunlit pavement had
two problems:

- white-painted concrete sidewalks, building roof reflections, and bright
  cloud edges all matched the indoor `lane_low: [0, 0, 180]` threshold,
  filling huge swaths of the costmap with phantom obstacles
- the painted lane line on asphalt — the actual target — wasn't being
  detected reliably, even though it visually looked perfectly white

The journey to figure out why is in §2.4. Skip to §2.5 if you just want
the final numbers.

### 2.1 First-pass false positives

**Sky/clouds.** Default `sky_roi_poly` cuts the top 35 % of the frame,
which on the bench (camera tilted slightly down) excluded the sky. With
the camera mounted level on the vehicle outdoors, clouds extended further
into the frame. Solution: widen ROI to top 55 %.

**Pothole class winning over lane_white.** In the original config,
`pothole_low: [0, 0, 200]` is a strict subset of `lane_low: [0, 0, 180]`
— so any bright pixel matched both classes. The HSV pipeline composites
classes in declaration order; pothole runs after lane, so pothole class
overwrites lane in the final mask. Result: the painted line was
classified as `pothole` (class 3) instead of `lane_white` (class 1). For
costmap purposes both produce lethal cells, but it broke the overlay
class-color mapping (see §3) and was confusing during diagnosis.
Tightened pothole to `[0, 0, 240]` / `[179, 25, 255]` so it only matches
near-pure white reflections with no chance of overlap with normal lane
paint.

### 2.2 Adaptive V-floor over-suppression

`adaptive_k: 3.0` (default) computes a per-frame V floor as
`mean + k·std` of the V channel. In bright outdoor scenes with sky in
the frame, this pushes the floor up to ~240 — well above what the
painted line actually measures (~180). Lane-detection silently failed.

Set `adaptive_k: 0.0` (disabled). The static `lane_low` V threshold is
plenty in this lighting; adaptive logic is more useful for indoor scenes
with mixed shadow.

### 2.3 The real bottleneck: saturation, not value

Spent multiple iterations adjusting `lane_low` V — 225, 200, 185, 160 —
trying to balance "catch the painted line" vs "don't catch concrete
pavement". Something always seemed off: V high enough to reject concrete
also rejected most of the line; V low enough to catch the line also
caught everything else.

The diagnostic break: at `V≥160` (very loose), mask was still showing
only **3 nonzero pixels** on a frame where the painted line was clearly
visible. V wasn't the problem.

Real culprit: `lane_high: [179, 30, 255]` — the saturation upper bound.
Indoor white tape has near-zero saturation (S<10). Outdoor painted lines
under direct sun pick up enough color cast (sky reflection adds slight
blue, asphalt undertone adds slight yellow) that their saturation runs
30–80. With `lane_high` S=30, the line was being **rejected by the
saturation upper bound**, regardless of how V was tuned.

Bumped `lane_high` S from 30 → 100 and the mask jumped from 3 pixels →
577 pixels, cleanly tracking the painted line.

### 2.4 Final calibration

Final HSV preset for outdoor sun (verified in field today):

| Param | Value | Notes |
|---|---|---|
| `lane_low` | `[0, 0, 160]` | V≥160 catches sun-lit white paint |
| `lane_high` | `[179, 100, 255]` | **S≤100** — was the real bottleneck (was 30) |
| `pothole_low` | `[0, 0, 240]` | tight enough to never overlap with lane_white |
| `pothole_high` | `[179, 25, 255]` | |
| `barrel_low` / `barrel_high` | unchanged | not exercised today; tighten when barrels are tested |
| `adaptive_k` | `0.0` | disabled — adaptive V-floor was over-suppressing in bright scenes |
| `blur_iters` | `3` | unchanged |
| `sky_roi_poly` | top 55 % cut | was 35 %; clouds were leaking through |

Indoor preset (`lane_low V=180`, `lane_high S=60`) works fine for bench
testing; outdoor preset is what should ship for field. Pre-tune at the
venue lighting, expect to revisit S as much as V.

---

## 3. Overlay visibility fix

**Why.** The class-color mapping in `class_map.yaml` had `lane_white`
at RGB `(255, 255, 255)`. The overlay generator in `perception_node.py`
did a 50/50 blend of the BGR image with the class color where the mask
matched. White tint blended 50/50 with white pavement = white. So the
overlay's "tinted" pixels looked exactly the same as the original
pavement — invisible.

Two-part fix:

### 3.1 Cyan tint instead of white

`class_map.yaml`: `lane_white` rgb changed `[255, 255, 255]` → `[0, 255, 255]` (cyan).
Cyan is the visual complement of white-ish surfaces and stands out
against both painted lines and pavement. `LabelInfo` is latched at
`perception_node` startup, so this change requires a relaunch of the
perception node (or the whole stack) to propagate to subscribers.

### 3.2 100 % opacity instead of 50 / 50 blend

Even with cyan, the 50/50 blend produced a pale-teal that was easy to
miss against light pavement. Changed `perception_node.py` lines 314–322:

```python
# Before:
overlay[sel] = (0.5 * overlay[sel].astype(np.float32)
                + 0.5 * np.array(color, dtype=np.float32)).astype(np.uint8)

# After:
overlay[sel] = np.array(color, dtype=np.uint8)
```

Solid cyan over the matched pixels, no blend. Visible from across the
room and unmistakable in Foxglove. Same code path is used for all class
colors, so barrel-orange and pothole-purple also become more visible.

This was the right fix even if cyan-on-white had been visible at 50/50
— for tuning sessions where the user is checking detection at a glance,
solid color is much clearer than blended.

---

## 4. Discoveries / limitations of this slim launch

These are NOT bugs, but they are non-obvious and they tripped us up
multiple times during the session. Documenting so future field sessions
don't repeat the diagnostic spiral.

### 4.1 Static identity TF means no rolling-window scrolling

The slim launch publishes `map → odom → base_link` as identity static
transforms. This means `base_link` is anchored to `(0, 0, 0)` regardless
of how the robot moves. The local costmap is `rolling_window: true` —
designed to scroll with `base_link`. When `base_link` doesn't move, the
window doesn't scroll, and cells that were painted in earlier views
**remain in the master grid forever**. Combined with the kiwicampus
layer's lack of explicit clearing (§4.2), this looks like the costmap
"isn't matching the overlay" — overlay shows current frame, costmap
shows accumulated history.

For real driving, use `navigation.launch.py` (which includes EKF + xsens
+ navsat) so `base_link` moves and stale cells fall out the back of the
rolling window. The slim launch is fine for stationary or slow walk-
test scenarios where you're verifying *that* the camera produces costmap
cells, not *whether* they decay correctly.

### 4.2 Kiwicampus layer doesn't reset its grid each cycle

Standard Nav2 layers (e.g. `nav2_costmap_2d::ObstacleLayer`) call
`resetMaps()` at the start of every `updateBounds`, so their layer-local
costmap_ array contains only current-cycle observations. The kiwicampus
`SemanticSegmentationLayer` does NOT do this — it writes 254 to layer
cells where tiles exist, but never writes 0/free anywhere. Combined
with `combination_method: 1` (Maximum, default), once a master cell is
254 it stays 254.

The mechanism that would normally clear stale 254s is the rolling
window scrolling cells out (see §4.1). Without that motion, the layer
accumulates cells for the lifetime of the launch.

A patch to add `resetMaps()` to `updateBounds` would fix this and was
proposed during the session, but not applied (would need to also adjust
combination behaviour and bound-extension to clear cells outside the
current touch region — non-trivial and at risk of regressing the mutex
patch).

### 4.3 `clear_entirely_local_costmap` permanently breaks the layer

Calling `nav2_msgs/srv/ClearEntireCostmap` on `/local_costmap/...` does
clear the master grid as documented — but it also leaves the kiwicampus
layer in a state where new observations no longer propagate to the
master. Verified: after a clear, the layer's internal `temporal_tile_map_`
went to zero entries even with active perception input, and lethal cells
stayed at 0 indefinitely. Suspect the layer's `is_current_` flag or
subscriber state gets disrupted.

So `clear_entirely_local_costmap` is NOT a viable workaround for the
accumulation problem — it makes it worse. Only "fix" available right
now is to relaunch the stack.

### 4.4 Camera auto-exposure spike at startup paints the entire FOV

When the ZED first opens, auto-exposure is hot for the first ~3–5
seconds. During that window, almost every asphalt pixel reads
V≥225, so HSV flags everything as `lane_white`. Combined with §4.1 +
§4.2, those cells stick around forever in the master grid even after
exposure stabilizes.

A rendered top-down view of the costmap during this state showed a
clean triangular wedge — exactly the camera's horizontal FOV projected
to the ground plane — fully painted lethal. Striking visual confirmation
of the diagnosis.

Workaround for clean-start testing: relaunch the stack only after the
camera has been pointed at a representative scene for ~5 s so exposure
settles, OR drive past the originally-painted area (cells will scroll
out of the rolling window once `base_link` moves — but only with the
full nav stack, not the slim launch).

### 4.5 `tile_map_decay_time: 0.3 s` is too aggressive for HSV-sparse tiles

Earlier morning session's decay sweep (`docs/CHANGELOG_2026-04-27.md` §3)
found stable propagation down to 0.15 s with the **stub pipeline** (which
publishes a dense, uniform mask). That doesn't transfer to HSV. With HSV
producing only ~10–20 tiles per frame and the cloud-stamp vs node-clock
skew of ~50–100 ms, decay=0.3 s caused the layer's `purgeOldObservations`
(which uses `node->now()` while observations are stamped with
`cloud.header.stamp`) to wipe tiles before the writer cycle could pick
them up. Master stayed at 0 lethal cells despite a healthy layer buffer.

Reverted to `tile_map_decay_time: 1.5` (kiwicampus default). Documented
in `perception_test_params.yaml`.

---

## 5. Verification matrix (end of session)

| Check | Result |
|---|---|
| Slim launch comes up cleanly, lifecycle active | ✓ 13 nodes, controller_server active |
| Foxglove WebSocket reachable | ✓ port 8765 |
| WebUI joystick reachable | ✓ port 8000 (verified via Tailscale) |
| Camera + perception + costmap rates | ✓ 15 / 14.9 / 1.94 Hz |
| HSV detects only the painted line, no false positives | ✓ 577 nonzero pixels, all on the line |
| Overlay shows clearly-visible cyan tint over detected pixels | ✓ solid cyan, 100 % opacity |
| Costmap accumulates lethal cells from camera detections | ✓ propagation working |
| Costmap **clears** stale cells when view shifts | ✗ stays accumulated (no `base_link` motion) |
| Stack survives `clear_entirely_local_costmap` | ✗ propagation breaks until relaunch |

The only ✗ items are the documented limitations of running without EKF.
For static / slow-walk perception verification, the launch is fully
functional.

---

## 6. Open items / follow-ups

- **Add `resetMaps()` patch to kiwicampus.** Standard pattern that every
  other Nav2 layer follows. Fixes the accumulation problem, the
  auto-exposure-spike persistence, and the `clear_entirely` brittleness
  in one shot. Estimated 2–3 line addition to `kiwicampus_pr2_mutex.patch`
  with a corresponding `combination_method: 0` (Overwrite) tweak to
  layer config. Not done today because of risk of regressing the mutex
  fix and no direct test scenario for it.

- **Bake the outdoor HSV preset into `perception.yaml`.** Today's runtime
  tune is in volatile params only — relaunch reverts to the indoor
  preset (V=180, S=60). Decision needed: ship two presets (indoor /
  outdoor) and switch via launch arg, or commit outdoor as default since
  IGVC is outdoor.

- **Tune barrel + pothole HSV at the venue.** Today only tested
  lane_white. Orange barrels and pothole markers haven't seen a real
  outdoor scene. Same saturation lesson likely applies.

- **Verify camera mount offset on the vehicle.** URDF has the front ZED
  at `(0.397, 0, 0.557)` relative to `base_link`. If real mount differs,
  costmap cells project to wrong world positions. Place a known marker
  at known distance, drive forward, confirm cell appears at the right
  spot.

- **Test the slim launch with EKF + xsens enabled** (manual addition or
  use `navigation.launch.py`) to confirm the rolling window actually
  scrolls and stale cells fall out as predicted. Today couldn't test
  because xsens wasn't connected during this session.

- **CLAUDE.md known-issues entries** for §4.1–4.5 should be added when
  the morning's `depth_stabilization` entry is written up.

---

## Reference docs

- `docs/CHANGELOG_2026-04-27.md` — morning session: GPU anomaly
  diagnosis, kiwicampus mutex patch verification, decay sweep
- `docs/CHANGELOG_2026-04-25.md` — original kiwicampus mutex bug
  discovery + workaround
- `docs/perception_latency_investigation.pdf` — 2026-04-26 research
  output that drove today's bench-test plan
- `src/avros_bringup/launch/perception_test.launch.py` — slim launch
- `src/avros_bringup/config/perception_test_params.yaml` — slim costmap
  config (semantic + inflation only)
- `src/avros_perception/avros_perception/perception_node.py` — overlay
  100 % opacity change is in `_on_synced`
- `src/avros_perception/config/class_map.yaml` — lane_white is now cyan
