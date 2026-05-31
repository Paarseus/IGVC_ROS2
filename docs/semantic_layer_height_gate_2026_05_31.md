# Camera Semantic Layer — Ground-Only Height Gate (design, 2026-05-31)

**Status: designed + implemented + verified, then REVERTED on 2026-05-31 pending
field calibration.** The functional change (layer C++ + YAML band) was backed
out; this doc + [`semantic_layer_height_gate_2026_05_31.patch`](semantic_layer_height_gate_2026_05_31.patch)
are kept so it can be re-applied and calibrated later. Apply with:

```bash
cd src/semantic_segmentation_layer
git apply ../../docs/semantic_layer_height_gate_2026_05_31.patch
# then re-add the YAML band (below) to the LOCAL front source and rebuild:
#   colcon build --packages-select semantic_segmentation_layer avros_bringup
```

## Problem

The camera lane pipeline (`perception_node` → kiwicampus
`semantic_segmentation_layer` → local costmap) marked **any** white pixel as a
`lane_white` LETHAL cell — including white objects **above the ground** (tents,
barrel tops, signs, people in white). Those are low-saturation, so the
`adaptive_max_sat` gate doesn't drop them, and the ROI is only an image-row cut.
A white object within 5 m gets a valid 3D point and is projected to its
ground (x,y) footprint and marked as a phantom lane.

Root cause (verified): the marking loop in `segmentation_buffer.cpp`
(`bufferSegmentation`) filters candidate points **only by spherical
distance-from-robot** (`min/max_obstacle_distance`) — **no z/height test** —
even though it has already transformed each point to the global (odom) frame
(`iter_z_global`). `min/max_obstacle_height` existed only as an unwired
(copy-paste-buggy) doc comment in `segmentation_buffer.hpp`. The LiDAR STVL
layer already height-gates (0.4–0.8 m); the camera layer simply lacked it.

## Approach (chosen)

Add a **global-frame height-band gate on the already-projected cloud, inside the
marking loop** — the exact mechanism stock nav2 `ObservationBuffer::bufferCloud`
and the STVL layer use. Wire `min_obstacle_height`/`max_obstacle_height` as real
per-source params; gate `iter_z_global` right after the finiteness check (before
`sq_dist` and before the clearing-point capture, so above-band points neither
mark nor clear).

Rejected alternatives: **IPM/homography** (needs accurate extrinsics we lack,
breaks on the ramp, discards the real ZED depth); **RANSAC ground-plane** (paint
lies *in* the plane → would delete the lanes; CPU the MPPI loop can't spare);
**grid_map elevation** (heavyweight; collapses to the same z-band).

## Implementation (in the .patch)

- `segmentation_buffer.hpp`: add members `min_obstacle_height_` / `max_obstacle_height_`
  (default ±1e30 = unbounded no-op) + setters; fix the copy-paste Doxygen.
- `segmentation_buffer.cpp`: 6-line z-gate after the finiteness `continue`.
- `semantic_segmentation_layer.cpp`: declare (`-1e6`/`1e6` sentinels), read,
  post-construction setters, and dynamic-reconfigure arms (live tuning).
- `nav2_params_humble.yaml` (NOT in the .patch — re-add manually), LOCAL front source:
  ```yaml
  min_obstacle_height: -0.5   # odom-frame meters, z=0 = ground
  max_obstacle_height:  0.5
  ```

## ⚠️ Critical caveat — `two_d_mode` and chassis pitch

Both adversarial reviewers caught this. `two_d_mode: true` on the odom EKF
forces the odom→base_link TF to report the chassis **perpetually level**, and
the camera's 15° tilt is a *static* URDF joint. So when the chassis physically
**pitches** (ramp, bump, accel nose-dip, grass), a real ground-paint point at
range `r` maps to an odom-z wrong by ~`r·sin(pitch)` — **~0.44 m at 5° @ 5 m**.
A *tight* band would silently clip real lane paint. Therefore:

- **Ship the band WIDE / fail-open** (`-0.5/+0.5`, or leave the params unset =
  no-op). It still rejects everything ≥0.6 m (barrel tops, people, tents) while
  keeping flat-ground paint and tolerating ~5.7° pitch @ 5 m.
- Robustness upgrade if pitch-clipping is seen: gate in a **robot-fixed frame
  (base_footprint)** instead of odom, or feed the real IMU pitch.
- **Local (odom) layer only** — the global/map layer has GPS-altitude jitter, so
  an odom-calibrated band is invalid there.
- Residual: the gate drops only the **elevated portion** of a white object; its
  near-ground base (0–0.5 m) still marks. Acceptable under the lanes-stay-LETHAL
  (stall-safe) design; LiDAR handles the object itself.

## Calibration procedure (do before tightening)

1. Park on flat asphalt facing a real lane line at ~2–4 m; bring up sensors +
   perception + localization.
2. Throwaway script: subscribe to synced mask + cloud, transform the cloud to
   odom, histogram `z` for points where mask == `lane_white` **and** that pass
   the real 3D slant-distance filter (camera origin, 0.3–5 m) — not a planar proxy.
3. Read cluster center + **spread vs range** (spread growing with range ⇒ the
   URDF mount *pitch* is wrong → measure it; a constant offset ⇒ mount *z* wrong).
4. Set `min = cluster_min − 0.1`, `max = cluster_max + 0.1 (+ pitch/ramp margin)`.
5. Re-measure **while the chassis is pitched** (curb lip / ramp approach), not
   only flat — that is where the gate is weakest.
6. A/B accept test (laptop Foxglove, never RViz on the Jetson): a white object at
   ~3 m must produce **no** lethal cells; a real lane line must still mark.
7. Use the dynamic-reconfigure arms to sweep the band live, then write final
   values to YAML.

## Why the adaptive pipeline was NOT reverted alongside

The height gate is *additive 3D evidence*, not a replacement for the 2D pipeline:
`process_at_full_res:true` (line visibility), `adaptive_blur:5` (ground-plane
speckle the gate must *pass*), `adaptive_max_sat:70` (colored at-band clutter),
`sky_roi_poly:0.40` all stay. See the workflow analysis (run `wt4rzjy3v`,
2026-05-31) for the full derivation.
