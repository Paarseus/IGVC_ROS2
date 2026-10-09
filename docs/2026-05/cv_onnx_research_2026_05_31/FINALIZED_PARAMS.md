# FINALIZED PARAMS — adaptive lane pipeline + ZED exposure lock

> **⚠️ CORRECTION (post-verification):** This draft was written in the Finalize phase, BEFORE the
> Verify phase. Its `adaptive_C: -11.0` recommendation (Section A) was **REFUTED** and re-measured to
> **`adaptive_C: -8.0` (KEEP current)** — at C=−8 the 537 px component is the *real* near/mid lane
> line (a 150×6 px horizontal bar at (110,154), stable across all field frames), and C=−11 erodes it
> to ~35% extent for zero precision gain (speckle is already removed by `max_sat:70`+`min_area:80`,
> not by C). **Authoritative values are in `PARAM_FINALIZE_AND_CLEANUP.md`.** The corrected block A
> below now reflects −8.0.

**Date:** 2026-05-31 · **Scope:** IGVC_ROS2 AutoNav, competition THIS WEEKEND
**Decision:** harden the deployed classical `adaptive` pipeline. NOT ONNX, NOT sooner25.
**Contract preserved:** 4-topic kiwicampus (mono8 mask + confidence + organized cloud + LabelInfo),
mask HxW == cloud HxW (224x128), single `class_id_lane=1`.
**Bias:** PRECISION — lanes are LETHAL(254) `samples_to_max_cost:1` (`nav2_params_humble.yaml:388`),
every false-positive pixel is an instant lethal cell. Do NOT create faint-line recall holes.

Verified live on cv2 4.13.0 against the real field frame
`docs/cv_adaptive_debug_2026_05_31/input_rgb_001.png` (300x480):
lane-band local-excess p90=5.2, p95=9.3, p99=36.3, p99.9=55.9, max~60;
C=-8 keeps 1375 px (incl. a 537 px speckle-merged blob), C=-11 keeps 803 px (5 clean 80-215 px line
comps), C=-14 collapses to ONE 189 px comp (line fragmented ~4x → recall hole).
Double `cv2.cvtColor(BGR2HLS)` for S confirmed BYTE-IDENTICAL → the dedup is output-identical.

---

## A. perception.yaml — adaptive block (`/**: ros__parameters:`)

```yaml
    # -------- Adaptive-threshold pipeline (pipeline:='adaptive') --------
    # Exposure-invariant local Gaussian adaptive threshold on HLS-L (cv2.adaptiveThreshold,
    # ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, C<0 => marks pixels brighter than local mean
    # by |C| = white paint). docs.opencv.org/4.x/d7/d1b. Process at FULL 480x300; the binary
    # mask is NEAREST-downsampled to the cloud 224x128 by the node. KEEP ZED AE behavior as set
    # by zed_front.yaml (now LOCKED — see Section B).
    pipeline: 'adaptive'

    adaptive_block_size: 21    # KEEP. Odd, > near/mid line width (3-17 px at 480x300) so the
                               # local mean stays asphalt-dominated. Odd-coercion guard in
                               # adaptive.py:86-88 is load-bearing (even RAISES, thresh.cpp:1909).
    adaptive_C: -8.0           # KEEP -8.0 (the -11.0 draft was REFUTED — see correction banner).
                               # T = local_mean + |C|. Re-measured live on all field frames: at C=-8
                               # the 537 px component is the REAL near/mid lane line (150x6 px bar at
                               # (110,154), present every frame). C=-11 fragments it to ~188 px (~35%
                               # extent) => LETHAL recall hole; C=-14 deletes it. Asphalt speckle is
                               # removed by max_sat:70 + min_area:80, NOT by C, so lowering C only
                               # erodes line recall for zero precision benefit. Field-tunable live.
    adaptive_channel: 'L'      # KEEP. adaptiveThreshold REQUIRES 8-bit single channel (3ch RAISES,
                               # thresh.cpp:1908). HLS-L is the lighting-stable luma proxy.
    adaptive_min_area: 80      # KEEP at 80 (NOT 120). At C=-11 the line decomposes into 80-215 px
                               # comps; min_area=120 drops the 80 px segment (verified) = recall hole
                               # on a boundary-cross-penalized LETHAL layer. Speckle already gone at
                               # C=-11, so 80 costs no precision. Field-tunable live.
    adaptive_use_open: false   # KEEP off. MORPH_OPEN erases thin lines (verified comp 114->17 px).
                               # CC min-area is the correct speckle remover. Lever retained for a
                               # rougher surface; zero cost when off.
    adaptive_blur: 9           # KEEP. Pre-blur low-passes 1-3 px asphalt-aggregate spikes before
                               # the local-mean compare; the wider line survives. Odd-coerced
                               # (adaptive.py:97-99). 5->9 drops lane-band p99.9 75->56.
    adaptive_max_sat: 70       # KEEP. White paint is low HLS-S; colored clutter (orange barrel at
                               # the SAME image height as the far line, tan pillar, grass) is high-S.
                               # ROI cannot separate the barrel from the far line — this gate can.
                               # 255 disables. THE load-bearing precision lever for this scene.

    # ROI: sky/horizon cut, applied LAST (fillPoly to 0). Masks tents/sky above the near band.
    sky_roi_poly: [0.0, 0.0,  1.0, 0.0,  1.0, 0.40,  0.0, 0.40]   # KEEP 0.40 (field-tuned).
                               # 0.40 cannot clip near/mid lanes (they sit lower, 0.46-1.0).

    class_id_lane: 1           # KEEP. Single output class -> lane_white. NEVER set to 0 (mask=0 is
                               # a no-op => silent total detection loss; guard at class_map level).

    process_at_full_res: true  # KEEP (node-level param, read in perception_node.py:354). At 224x128
                               # the INTER_AREA downscale + block 21 blurs the faint near line into
                               # asphalt BEFORE the pipeline; full-res keeps the line crisp.
```

### NOT adding the temporal vote (vote_n / vote_k)
RECOMMENDATION.md Change B proposes a K-of-N per-pixel temporal vote. It is **NOT in the deployed
code**, is a stateful BEHAVIOR change (new ring buffer, `__init__`, two new params), and the audit
explicitly scopes it OUT of the minimal-cleanup pass and flags a real motion-erosion risk
(per-pixel vote can erode a line that shifts under ego-motion at 1-5 mph on a 15° camera;
"k=1 to disable" is the escape hatch). With no labeled data and competition in <48 h, adding
stateful logic two days out — verified safe ONLY stationary — contradicts the "minimal, verified,
final" mandate. **Decision: do NOT add vote_n/vote_k.** If decay-purge flicker survives the live
A/B (see camera_costmap_flicker_analysis), the leverage fix is `tile_map_decay_time 0.3->0.6`
(nav2 side), not a new stateful CV stage.

---

## B. zed_front.yaml — EXPOSURE / WHITE-BALANCE LOCK (purely additive; no `video:` block today)

Add under `/**: ros__parameters:`. The `/**:` override wins over the wrapper base
`common_stereo.yaml:42-49` (legacy) and `zedx.yaml:12-23` (native). On GMSL ZED X set the NATIVE
µs/mDB controls (`exposure_time`, `analog_gain`, `digital_gain`) — the wrapper recommends these
over the legacy 0-100 `exposure`/`gain` (zedx.yaml:13,17,20).

```yaml
    # 2026-05-31: LOCK auto-exposure + white-balance at the source. Kills the documented AE
    # limit-cycle flicker (sooner25 swung lane px 375<->909 at the same threshold). These keys are
    # [DYNAMIC] in common_stereo.yaml:45-49 / zedx.yaml:12-23; the /**: override wins. Reversible:
    # set auto_exposure_gain:true to revert to today's known-good adaptive-with-AE behavior.
    video:
      auto_exposure_gain: false        # MASTER AE lock (sends AEC_AGC=0; model-agnostic). HIGH conf.
      auto_whitebalance: false         # MASTER WB lock; enables whitebalance_temperature below.
      # --- ZED X NATIVE manual controls (preferred on GMSL; ranges from zedx.yaml) ---
      exposure_time: 3000              # microseconds. START SHORT for motion-blur safety at 1-5 mph;
                                       # raise analog_gain to keep the asphalt well-exposed
                                       # (target HLS-L ~127, the AE-resolved point in capture_stats).
                                       # FIELD-TUNE while DRIVING. (wrapper default 16000)
      analog_gain: 4000                # mDB, range [1000-16000]. Raised above the 1255 default to
                                       # compensate the short exposure. FIELD-TUNE.
      digital_gain: 1                  # ISP factor [1-256]. Keep 1 (no ISP gain => least noise).
      whitebalance_temperature: 42     # x100, range [28,65]; only applies when auto_whitebalance:false.
                                       # 42 = wrapper default (neutral daylight). FIELD-TUNE if the
                                       # white paint tints (would shift the low-S gate).
      saturation: 4                    # = wrapper default; keep stable so adaptive_max_sat stays valid.
      sharpness: 4                     # = wrapper default.
```

### Field-tune + verify (live, laptop Foxglove — never RViz on the Jetson during nav)
```bash
ros2 param get /zed_front/zed_node video.auto_exposure_gain     # expect: false
ros2 param get /zed_front/zed_node video.exposure_time          # confirm the VALUE wrote on this GMSL unit
ros2 param set /zed_front/zed_node video.exposure_time <us>     # tune SHORT, watch /perception/front/overlay
ros2 param set /zed_front/zed_node video.analog_gain <mDB>      # raise to compensate
```
**MOTION-BLUR is the #1 field risk:** a fixed exposure clean stationary can smear the 3-inch line at
5 mph. Keep `exposure_time` as SHORT as asphalt brightness allows; verify WHILE DRIVING, then write
the locked values back here.

**CAVEAT (unmeasured premise):** the captured stationary data shows frame mean flat ~127 — AE was
NOT hunting during capture, so the AE-limit-cycle premise behind this lock is UNVERIFIED in our only
data. Ship it anyway: it is FREE + REVERSIBLE insurance. Confirm the benefit with a live A/B.

---

## C. CODE CLEANUP — adaptive.py (output-identical; standard/minimal form)

| # | Change | adaptive.py loc | Why |
|---|---|---|---|
| 1 | Compute `cv2.cvtColor(bgr, BGR2HLS)` ONCE on the `channel=='L'` path; reuse `hls[:,:,1]`=L and `hls[:,:,2]`=S for the sat gate. Keep the separate BGR2HLS for S only on the `'V'`/`'gray'` paths. | 110 + 134 | Verified byte-identical S; removes a duplicate full-frame cvtColor (~0.37 ms/frame). Output unchanged. |
| 2 | Fold `clean` + `mask` into ONE array: write `mask[labels==i]=class_id_lane` in the CC loop; drop the `clean` zeros array + the `clean>0` pass. | 148-155 | One fewer HxW allocation. Safe ONLY because output is single-class — leave a comment. |
| 3 | Hoist `_reshape_poly` + `_roi_polygon_px` into `base.Pipeline` (or `pipelines/_roi.py`); de-triplicate across hsv.py / sooner25.py / adaptive.py. | 53-74 | DRY. Behavior-identical. Wider blast radius — if strictly time-boxed before competition, SKIP and keep the triplication (cleanup-later). |
| 4 | Align in-code/declare defaults to the field-tuned YAML: `sky_roi_poly` 0.35->0.40 (adaptive.py:67 + perception_node.py:149); `adaptive_min_area` 15->80 (perception_node.py:180). | 67, 90 / node 149,180 | YAML wins at runtime; this only fixes the YAML-absent launch case + removes the 3-way default disagreement. |
| 5 | base.py docstring lists "(stub, HSV, ONNX)" — update to "(stub, HSV, sooner25, adaptive, ONNX)". | base.py:6 | Cosmetic doc accuracy. |

### KEEP exactly as-is (verified load-bearing — do NOT "optimize")
- **block_size / blur odd-coercion guards** (adaptive.py:86-88, 97-99) — even blockSize / kernel RAISE
  (thresh.cpp:1909, smooth.dispatch). Reachable via raw `ros2 param set`; one bad set stalls ALL
  perception. KEEP.
- **CC min-area Python loop** (adaptive.py:146-151) — vectorizing is SLOWER (loop short-circuits;
  `keep[labels]` materializes a full fancy-index every frame). Measured. KEEP.
- **THRESH_BINARY + C<0 polarity** (adaptive.py:120-125) — doc-exact for bright lines. KEEP.
- **adaptive_max_sat low-S gate / 255 short-circuit** (adaptive.py:133-135) — precision lever. KEEP.
- **confidence = 255*(mask>0)** (adaptive.py:163) — consumed at nav2_params_humble.yaml:367
  (`confidence_topic`); part of the 4-topic contract. KEEP.
- **sky ROI fillPoly applied LAST** (adaptive.py:159-161) — non-negotiable. KEEP.

---

## D. Dead / inert params (do NOT remove in this adaptive-focused pass)

| Param | Where | Status / why keep |
|---|---|---|
| `lane_band`, `lane_close_w`, `lane_min_area` | perception.yaml:98,100,101 | DEAD (undeclared; node never forwards them). HSV-only. Belongs to a SEPARATE HSV-retirement pass. |
| `inject_*`, `blur_iters`, `adaptive_period`, `adaptive_k`, `lane_*`, `barrel_*`, `pothole_*`, `lane_erode_iters`, `sooner25_*` | _PIPELINE_PARAM_NAMES (perception_node.py:49-68) + perception.yaml | INERT for adaptive, LIVE for stub/hsv/sooner25 (pipeline-switchable node). Removing breaks fallback pipelines. KEEP. |
| `adaptive_channel` 'V'/'gray' branches | adaptive.py:105-110 | Inert in practice (CV near-identical) but a safe one-line selector + fallback. KEEP. |
