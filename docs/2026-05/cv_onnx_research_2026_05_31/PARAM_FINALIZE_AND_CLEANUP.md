# PARAM FINALIZE + CLEANUP — adaptive lane pipeline + ZED exposure lock

**Date:** 2026-05-31 · **Project:** IGVC_ROS2, AutoNav Challenge ONLY · **Competition:** this weekend
**Decision:** harden the **deployed classical `adaptive` pipeline**. NOT ONNX, NOT sooner25.
**Contract preserved (non-negotiable):** 4-topic kiwicampus — mono8 mask + mono8 confidence + organized
PointCloud2 + latched `vision_msgs/LabelInfo`; mask HxW == cloud HxW (224×128); single `class_id_lane=1`.
**Bias:** PRECISION — lanes are **LETHAL(254)** with `samples_to_max_cost: 1`
(`src/avros_bringup/config/nav2_params_humble.yaml:387-388`), so every false-positive pixel is an instant
lethal cell — but do NOT create faint-line recall holes (boundary-cross is run-ending,
`igvc_2026_rules_fulltext.txt:342`).

> **All OpenCV claims re-verified live on `cv2 4.13.0 / numpy 2.4.4` (the system interpreter).** The
> cleaned pipeline was run against the live pipeline on **all 11 field frames × 10 param-branch combos**
> and is **byte-identical** (mask + confidence). The load-bearing `adaptive_C` decision was independently
> re-measured on `input_rgb_001/010/018.png` — see Verification Table.

---

## 1. TL;DR — finalized set + cleanup verdict

**The finalized set at one glance** (only TWO live-file edits change a value; everything else is KEEP):

| File | Param | old → new |
|---|---|---|
| `zed_front.yaml` | `video:` block (AE + WB lock, native ZED-X exposure_time/analog_gain) | **ADD** (no `video:` block today) |
| `perception.yaml` | `adaptive_C` | **KEEP −8.0** (the −11.0 draft was **REFUTED** by verification — see below) |
| `perception.yaml` | `adaptive_block_size / channel / min_area / use_open / blur / max_sat / sky_roi_poly / class_id_lane / process_at_full_res` | all **KEEP** |
| `perception_node.py` | in-code defaults `sky_roi_poly 0.35→0.40`, `adaptive_min_area 15→80` | align (runtime-neutral; YAML already wins) |
| `adaptive.py` | replace with `adaptive_cleaned.py` (single BGR2HLS, single mask array, aligned defaults) | **byte-identical output** |

**Cleanup verdict:** **SHIP the cleaned `adaptive.py`.** It is verified output-identical, removes one
redundant full-frame `cvtColor` and one HxW allocation per frame, aligns the three-way-disagreeing
in-code defaults to the field-tuned YAML, and keeps every load-bearing guard (odd-coercion, CC loop,
polarity, sat gate, ROI-last, confidence). **No behavior change. Safe to ship this weekend.** The
ROI-helper de-triplication and the temporal vote are deliberately **deferred** (see §4, §9).

**The single most important correction this report carries:** the intermediate draft
`FINALIZED_PARAMS.md` recommended `adaptive_C: -11.0`. The verification phase **REFUTED** that, and my
independent re-measurement confirms it: at `C=-8` the largest detected component (537 px, a 150×6 px
horizontal feature at row y=154) **is the real near/mid lane line, not a speckle blob**; dropping to
`C=-11` erodes mid-field line recall to ~65% (655→425 px), opening a recall hole on a LETHAL layer for
**zero precision gain** (the `max_sat=70` + `min_area=80` gates already remove all asphalt speckle).
**Finalized value: `adaptive_C = -8.0` (unchanged from live).**

---

## 2. FINALIZED PARAMETERS TABLE

### 2A. ZED exposure / white-balance lock — `src/avros_bringup/config/zed_front.yaml`

There is **no `video:` block in `zed_front.yaml` today** (the file jumps `general:` → `depth:`), so this
is **purely additive**. The `/**:` override wins over the wrapper base
(`common_stereo.yaml:41-49` legacy controls, `zedx.yaml:12-23` native controls). All keys are `[DYNAMIC]`.

| Param | old → new | key path / `ros2 param set` | Justification | Official-doc source | Conf |
|---|---|---|---|---|---|
| `auto_exposure_gain` | `true` (base default `common_stereo.yaml:45`) → **`false`** | `video.auto_exposure_gain` / `ros2 param set /zed_front/zed_node video.auto_exposure_gain false` | MASTER AE lock — sends `AEC_AGC=0` (model-agnostic). Removes the documented AE limit-cycle at the source (sooner25 swung lane px 375↔909 at one threshold; `perception.yaml:30-31`). | stereolabs InitParameters (manual exposure/gain control) | HIGH |
| `auto_whitebalance` | `true` (`common_stereo.yaml:48`) → **`false`** | `video.auto_whitebalance` | MASTER WB lock — keeps the white-paint hue/L stable across the run so the low-S gate (`adaptive_max_sat`) and HLS-L channel don't drift. Enables `whitebalance_temperature`. | `common_stereo.yaml:48-49` (`works only if auto_whitebalance is false`) | HIGH |
| `exposure_time` | (native default `16000` µs, `zedx.yaml:13`) → **field-tune SHORT, start `3000` µs** | `video.exposure_time` (µs) | GMSL ZED X native control — wrapper says "**Recommended to control manual exposure** instead of `video.exposure`" (`zedx.yaml:13`). Keep SHORT to avoid motion-blur smearing the 3-in line at 1-5 mph. | `zedx.yaml:13` | MED (value) / HIGH (key) |
| `analog_gain` | (native default `1255` mDB, `zedx.yaml:17`) → **field-tune, start `4000` mDB** | `video.analog_gain` (mDB, [1000-16000]) | Native sensor gain — "**Recommended** instead of `video.gain`" (`zedx.yaml:17`). Raise to compensate the short exposure so asphalt stays well-exposed (target HLS-L ~127, the AE-resolved point in `capture_stats.txt`). | `zedx.yaml:17` | MED (value) / HIGH (key) |
| `digital_gain` | `1` (`zedx.yaml:20`) → **keep `1`** | `video.digital_gain` ([1-256]) | Keep ISP gain at 1 → least noise; let `analog_gain` carry the brightening. | `zedx.yaml:20` | HIGH |
| `whitebalance_temperature` | `42` (`common_stereo.yaml:49`) → **`42`, field-tune** | `video.whitebalance_temperature` (x100, [28,65]) | Neutral daylight; only applies when `auto_whitebalance: false`. Re-tune if white paint tints (would shift the low-S gate). | `common_stereo.yaml:49` | MED |
| `saturation` / `sharpness` | `4` / `4` (base) → **`4` / `4`** | `video.saturation` / `video.sharpness` | Keep at base default so the `adaptive_max_sat` calibration stays valid. | `common_stereo.yaml:42-43` | HIGH |

> **CAVEAT (unmeasured premise):** the only captured field data is **stationary with already-flat
> exposure** (frame mean ~127, lane px stable 362-380, `capture_stats.txt`) — so the AE limit cycle was
> **NOT occurring during capture**. The lock is defensible as **free + reversible insurance**, but its
> benefit magnitude is **UNVERIFIED**; confirm with a live A/B. Revert with
> `ros2 param set /zed_front/zed_node video.auto_exposure_gain true`.

### 2B. Adaptive pipeline block — `src/avros_perception/config/perception.yaml`

| Param | old → new | key path / `ros2 param set` | Justification | Official-doc source | Conf |
|---|---|---|---|---|---|
| `pipeline` | `adaptive` → **`adaptive`** | `/**:.ros__parameters.pipeline` (`perception.yaml:34`) | Lock classical. No pretrained ONNX transfers zero-shot to 3-in tape on asphalt; sooner25's fixed-brightness threshold is what adaptive replaced. | RECOMMENDATION.md §2 | HIGH |
| `adaptive_block_size` | `21` → **`21`** | `adaptive_block_size` (`perception.yaml:180`); `ros2 param set /perception_node adaptive_block_size 21` | blockSize must be **odd ≥ 3** and > feature width so the local mean stays asphalt-dominated. Line perpendicular thickness ~2-3 px/column at 480×300 → 21 is correct. Odd-coercion guard (`adaptive.py:86-88`) is **load-bearing**: even blockSize RAISES at `thresh.cpp:1909` (verified). | OpenCV `adaptiveThreshold` doc | HIGH |
| **`adaptive_C`** | **`-8.0` → `-8.0` (KEEP)** | `adaptive_C` (`perception.yaml:181`); `ros2 param set /perception_node adaptive_C -8.0` | `C<0 ⇒ T = local_mean + \|C\|` (doc: C "may be negative"). **Verification REFUTED the −11.0 draft.** Re-measured on `input_rgb_001`: at C=−8 total=1380 px / largest comp 537 px = the **real near/mid line** (150×6 px at y=154); C=−11 cuts mid recall to **65%** (655→425 px), C=−14 collapses to ONE 189 px comp. The `max_sat=70`+`min_area=80` gates — NOT C — remove asphalt speckle, so lowering C erodes line recall for **zero** FP benefit. | OpenCV `adaptiveThreshold` doc; live re-measure (this report) | HIGH |
| `adaptive_channel` | `L` → **`L`** | `adaptive_channel` (`perception.yaml:182`) | adaptiveThreshold REQUIRES 8-bit single channel (3-channel RAISES at `thresh.cpp:1908`, verified). HLS-L is a lighting-stable luma proxy; else-branch fallback makes any junk string safe. (Soften the docstring: under an AE sweep gray≈L<V in stability; L is co-best, not uniquely best.) | OpenCV `adaptiveThreshold` doc; `adaptive.py:101-110` | HIGH |
| `adaptive_min_area` | `80` → **`80`** | `adaptive_min_area` (`perception.yaml:183`); `ros2 param set /perception_node adaptive_min_area 80` | CC speckle filter. At the kept C=−8, raising to 120 (the RECOMMENDATION.md draft) drops real 80-131 px line segments → recall hole. KEEP 80; speckle is already removed by max_sat. | Verified live; audit | HIGH |
| `adaptive_use_open` | `false` → **`false`** | `adaptive_use_open` (`perception.yaml:192`) | MORPH_OPEN erases thin lines (verified comp 114→17 px). CC min-area is the correct speckle remover. Lever retained (zero cost when off). | OpenCV morphology doc; `adaptive.py:137-142` | HIGH |
| `adaptive_blur` | `9` → **`9`** | `adaptive_blur` (`perception.yaml:193`); `ros2 param set /perception_node adaptive_blur 9` | Pre-blur low-passes 1-3 px asphalt-aggregate spikes before the local-mean compare; the wider line survives (5→9 drops lane-band excess p99.9 75→56). Odd-coercion guard (`adaptive.py:97-99`) load-bearing (GaussianBlur RAISES on even kernel). | OpenCV thresholding tutorial | HIGH |
| `adaptive_max_sat` | `70` → **`70`** | `adaptive_max_sat` (`perception.yaml:198`); `ros2 param set /perception_node adaptive_max_sat 70` | **THE load-bearing precision lever for this scene.** White paint is low HLS-S; the orange barrel sits at the SAME image height as the far line (`input_rgb_001/018.png`) so the ROI cannot separate them — the S>70 gate can. 255 disables (short-circuits the conversion). | `adaptive.py:127-135`; field frames; `nav2_params_humble.yaml:384` (`classes:[lane_white]`) | HIGH |
| `sky_roi_poly` | `[0,0,1,0,1,0.40,0,0.40]` → **same** | `sky_roi_poly` (`perception.yaml:144`) | Top-40% sky/horizon cut, applied LAST (`adaptive.py:159-161`). 0.40 masks tents/sky but cannot clip near/mid lanes (they sit lower). Note: in-code/declare defaults are 0.35 — **align to 0.40** (doc/consistency only; YAML wins). | `perception.yaml:128-144` | HIGH |
| `class_id_lane` | `1` → **`1`** | `class_id_lane` (`perception.yaml:112`) | Single output class → `lane_white` (`class_map.yaml:15`); the layer costs ONLY `lane_white` (`nav2_params_humble.yaml:384`). NEVER 0 (mask=0 is a no-op → silent total detection loss). | `class_map.yaml:15`; `nav2_params_humble.yaml:384` | HIGH |
| `process_at_full_res` | `true` → **`true`** | `process_at_full_res` (`perception.yaml:104`; node-level, read at `perception_node.py:354`) | At 224×128 the INTER_AREA downscale + block 21 blurs the faint near line into asphalt BEFORE the pipeline; full 480×300 keeps the line crisp, then the node NEAREST-downsamples the binary mask to cloud HxW (`perception_node.py:368-384`) — contract preserved. | `perception_node.py:354,368-384` | HIGH |

---

## 3. DEAD / REDUNDANT PARAMS in the adaptive config path

**Net for the adaptive pipeline: NOTHING is removed from `perception.yaml` or the param declarations in
this pass.** The smells below are real but are **out of scope** for an adaptive-only finalize (removing
them risks the pipeline-switchable node and the HSV/sooner25 fallbacks). Documented for the record:

| Item | Where | Status / why NOT removed here |
|---|---|---|
| `lane_band`, `lane_close_w`, `lane_min_area` | `perception.yaml:98,100,101` | **Genuinely dead w.r.t. the live node** — undeclared in `perception_node.py`, no `allow_undeclared_parameters`, so the node never forwards them; even `hsv.py` falls back to its own `.get()` defaults. They are HSV-only. Belongs to a **separate HSV-retirement pass**; removing YAML keys could surprise a re-wired HSV path. KEEP. |
| `inject_*`, `blur_iters`, `adaptive_period`, `adaptive_k`, `lane_*`, `barrel_*`, `pothole_*`, `lane_erode_iters`, `sooner25_*` | `_PIPELINE_PARAM_NAMES` (`perception_node.py:49-68`) + `perception.yaml` | **INERT for adaptive, LIVE for stub/hsv/sooner25.** The node is pipeline-agnostic (`perception.yaml:19` lists `stub\|hsv\|sooner25\|adaptive\|onnx`) and the shared `self._pipeline_params` allowlist supports live pipeline switching via `ros2 param set`. Removing them breaks the fallbacks. KEEP. |
| `adaptive_channel` `'V'`/`'gray'` branches | `adaptive.py:127-130` (cleaned) | Inert in practice (V/gray ≈ L) but a safe one-line selector + the else-branch fallback that prevents an invalid-string crash. KEEP. |

---

## 4. PIPELINE CLEANUP — `adaptive.py`

**Cleaned file:** `/home/mspacman/IGVC_ROS2/docs/cv_onnx_research_2026_05_31/adaptive_cleaned.py`
**Verification:** run against the live pipeline on **all 11 field frames × 10 param-branch combos**
(L/V/gray channels; sat on/off; open on/off; even/`<3`/even-blur/negative-blur coercion; C=−8 and
C=−11) → **byte-identical mask AND confidence in every case.** No behavior change.

### Changes made (all output-identical)

| # | Change | live `adaptive.py` loc | cleaned loc | Why it is now standard |
|---|---|---|---|---|
| 1 | **Single `BGR2HLS`** on the `channel=='L'` path — slice `hls[:,:,1]`=L and reuse `hls[:,:,2]`=S for the sat gate. The `'V'`/`'gray'` paths take their own `BGR2HLS` only if the sat gate is on. | `:110` + `:134` (two converts) | `:131-134`, `:156-159` | Removes a **redundant full-frame `cvtColor`** per default frame (verified byte-identical S). Dead-work removal. |
| 2 | **Fold `clean` + `mask` into ONE array** — write `mask[labels==i]=class_id_lane` directly in the CC loop; drop the `clean` zeros array and the `clean>0` pass. | `:148-155` | `:176-179` | One fewer HxW allocation. Safe ONLY because output is single-class (comment left in code). |
| 3 | **Align in-code defaults to the field-tuned YAML** — `adaptive_min_area` 15→80, `adaptive_blur` 3→9, `adaptive_max_sat` 255→70, `sky_roi_poly` default 0.35→0.40. | `:67,90,95,96` | `:78,97,101,102` | Removes the three-way default disagreement (code vs declare vs YAML). **Runtime-neutral** with `perception.yaml` loaded; only changes the YAML-absent launch case to match known-good field config. |
| 4 | **Tidy comments** — document the loop is intentionally faster than vectorizing, the odd-coercion is load-bearing on cv2 4.13.0, the single-convert reuse. | docstrings | `:104-114,168-173` | Doc accuracy. |

### KEEP exactly (verified load-bearing — do NOT "optimize")
- **block_size / blur odd-coercion + floor guards** (`adaptive.py:86-88,97-99`) — even blockSize/kernel
  RAISE (`thresh.cpp:1909`); reachable via raw `ros2 param set`; one bad set stalls ALL perception.
- **CC min-area Python loop** (`adaptive.py:146-151`) — vectorizing (`np.where(keep[labels])`) is
  **measured slower** (loop short-circuits on the few large comps; `keep[labels]` fancy-indexes the
  whole frame). KEEP the loop.
- **THRESH_BINARY + C<0 polarity** (`adaptive.py:120-125`) — doc-exact for bright lines.
- **`adaptive_max_sat` low-S gate + 255 short-circuit** (`adaptive.py:133-135`) — the precision lever.
- **`confidence = 255*(mask>0)`** (`adaptive.py:163`) — consumed at `nav2_params_humble.yaml:367`
  (`confidence_topic`) + `mark_confidence:1`; part of the 4-topic contract.
- **sky ROI `fillPoly` applied LAST** (`adaptive.py:159-161`) — non-negotiable horizon cut.

### Behavior changes — explicit
**NONE in the shipped cleanup.** Every change above is verified output-identical.

### Deliberately DEFERRED (NOT in the cleaned file)
- **De-triplicate `_reshape_poly`/`_roi_polygon_px` into `base.Pipeline`** — correct DRY fix but touches
  `hsv.py` + `sooner25.py` too (wider blast radius). A `NOTE` comment in the cleaned file flags it.
  **Defer to a post-competition base-class refactor.**
- **Temporal K-of-N vote (`vote_n`/`vote_k`)** — a stateful **behavior change** (ring buffer, `__init__`,
  two new params), verified safe ONLY stationary, with a real motion-erosion risk on a 15°-tilt camera at
  1-5 mph. **Do NOT add two days out.** If decay-purge flicker survives the live A/B, the leverage fix is
  on the nav2 side (`tile_map_decay_time`), not a new CV stage.
- **base.py docstring** lists "(stub, HSV, ONNX)" — should read "(stub, HSV, sooner25, adaptive, ONNX)".
  Cosmetic; optional one-line fix.

---

## 5. OFFICIAL-DOCS BASIS

**OpenCV `adaptiveThreshold` / morphology** — https://docs.opencv.org/4.x/d7/d1b/group__imgproc__misc.html
- Signature `adaptiveThreshold(src, dst, maxValue, adaptiveMethod, thresholdType, blockSize, C)`; **src
  must be 8-bit single-channel**; `thresholdType` ∈ {THRESH_BINARY, THRESH_BINARY_INV}. (3-channel src
  RAISES at `thresh.cpp:1908` — verified.)
- **blockSize**: "Size of a pixel neighborhood … 3, 5, 7, and so on" → **odd ≥ 3** (even RAISES at
  `thresh.cpp:1909`; `blockSize=1` RAISES — verified).
- **C**: "Constant subtracted from the mean or weighted mean … may be zero or **negative**" → `C<0 ⇒
  T = local_mean + |C|`, and `THRESH_BINARY` fires `maxval if src>T`, so **C<0 marks pixels brighter than
  their local neighborhood = white paint**. Correct polarity.
- **ADAPTIVE_THRESH_GAUSSIAN_C** uses a Gaussian-weighted local mean — the more noise-robust of the two
  methods, correct for textured asphalt.

**OpenCV thresholding tutorial** — https://docs.opencv.org/4.x/d7/d4d/tutorial_py_thresholding.html
- Global thresholding "might not be good … if an image has different lighting conditions in different
  areas"; adaptive "calculates the threshold for a smaller region" → "better results for images with
  varying illumination." This is the exact justification for adaptive over sooner25's fixed-brightness
  threshold under the ZED AE swing.

**ZED exposure / colorspace** — https://www.stereolabs.com/docs/api/structsl_1_1InitParameters.html
- `auto_exposure_gain:false` sends `AEC_AGC=0` (master AE lock). On GMSL ZED X the wrapper exposes native
  `exposure_time` (µs) and `analog_gain` (mDB, [1000-16000]) and **recommends them over** legacy
  `video.exposure`/`video.gain` (`zedx.yaml:13,17`). White paint is low-saturation in HLS — the basis for
  the `max_sat` gate; locking WB keeps that gate stable.

---

## 6. IGVC 2026 RULES COMPLIANCE (`docs/cv_onnx_research_2026_05_31/igvc_2026_rules_fulltext.txt`)

| Rule requirement | Cite | How the finalized params respect it | Compliance |
|---|---|---|---|
| Outer boundaries = **continuous OR dashed white lines ~3 in wide**, taped on asphalt | `:282` | Adaptive local-threshold + low-S white gate marks bright low-S paint regardless of continuous/dashed (dashes are separate CC comps, each ≥ `min_area=80`). `block_size=21` sized for the 2-3 px line; `C=−8` (kept) maximizes recall of the near+mid line. Exposure lock stabilizes contrast. | GOOD (well-painted); PARTIAL (worn — classical limit, uncharacterized in our data) |
| **Crossing internal/boundary lines** → E-Stop end of run / Careless Driving (−5 ft) / Leave the Course (−10 ft) | `:342,354-356,372` | Lanes are LETHAL(254) `samples_to_max_cost:1` (`nav2_params_humble.yaml:387-388`) → MPPI treats any detected line as a hard no-go. **`C=−8` kept precisely to avoid a recall hole** (the −11 draft cut mid-line recall to 65% → a missed line = boundary-cross risk). Precision is held by `max_sat=70`+`min_area=80`, NOT by lowering C. | GOOD when detected; recall is the honest residual on faint paint |
| **Simulated potholes = 2-ft solid white circles**, must be avoided | `:293,536,593` | A 2-ft white circle is low-S bright paint → marked LETHAL as `lane_white` → **avoided, not distinguished**. `pothole` class is neutered (`perception.yaml:124-125`) and unmarked by the layer. `max_sat=70` keeps it (white, low-S) while rejecting colored barrels. | INCIDENTALLY SAFE (avoided), not rule-distinguished |
| **Ramps up to 15% grade** | `:287` | Out of CV scope by design — Velodyne `obstacle_range` is short so it never sees the ramp deck; camera does not mark the ramp surface. No height gate added (would hit the `two_d_mode` pitch trap). | N/A to CV (by design) |
| ROI must not clip near lanes; horizon clutter at lane height | `:282` (lines) | `sky_roi_poly=0.40` masks tents/sky only; the colored horizon clutter sitting AT lane-band height is removed by `max_sat=70`, NOT by the ROI (ROI can't separate the barrel from the far line — same height; `input_rgb_001/018.png`). Near/mid lanes sit lower (rows 120-166 in our frames) and are never clipped. | GOOD |
| Speed 1-5 mph; motion at the line | `:157-164` | Actuator caps within band. **Motion blur from a too-long locked exposure is the #1 CV-side field risk** — keep `exposure_time` SHORT, raise `analog_gain`, verify WHILE DRIVING. | GOOD if exposure tuned short |
| Barrels / trees / signs (orange/white/etc.) | brief | Velodyne→STVL owns obstacles; camera marks ONLY `lane_white` (`nav2_params_humble.yaml:384`). `max_sat=70` actively rejects colored barrels from the lane mask. | By design (LiDAR owns it) |

---

## 7. VERIFICATION TABLE

| # | Claim | Verdict | Evidence / correction |
|---|---|---|---|
| V1 | **`adaptive_C` should be −11.0** (537 px comp at −8 is a "speckle-merged blob"; −11 is the precision optimum) | **REFUTED** | Re-measured live on cv2 4.13.0: at C=−8 the 537 px comp is a **150×6 px horizontal feature at x=110,y=154** = the **real near/mid lane line** (all detections lie in rows 120-166). C=−11 cuts mid-field recall to 65% (655→425 px); C=−14 → ONE 189 px comp. FP in open asphalt is **already 0** at C=−8 (the `max_sat=70`+`min_area=80` gates, not C, remove speckle). **Correction: KEEP `adaptive_C = -8.0`.** This overrides the `FINALIZED_PARAMS.md` draft. |
| V2 | `adaptive_min_area` should be 80 (not the RECOMMENDATION.md 120) | **CONFIRMED** | At kept C=−8, min_area=120 drops real 80-131 px line segments → recall hole. 80 keeps them; speckle already gone via `max_sat`. KEEP 80. |
| V3 | `adaptive_block_size=21`; odd-coercion guard load-bearing | **CONFIRMED (one sub-claim corrected)** | Even/`≤1` blockSize RAISES at `thresh.cpp:1909`; 3-channel RAISES at `thresh.cpp:1908` (verified). Guard (`adaptive.py:86-88`) rescues 20→21, 1→3, 0→3. **Correction to rationale:** the cap on raising block is **false-positive suppression** (barrel-base blob ~0 at bs≤21 vs ~188 lethal cells at bs49), **not** near-line degradation; restate line thickness as ~2-3 px perpendicular. Value 21 KEPT. |
| V4 | `adaptive_channel='L'`; 8-bit single channel required; else-branch fallback safe | **CONFIRMED** | 3-channel RAISES at `thresh.cpp:1908`; junk strings fall through to the byte-identical L path (no crash). **Soften justification:** under an AE gain sweep stability is gray≈L<V (L co-best, not uniquely best); on real frames L/V/gray differ ~9-11% px (IoU 0.886-0.963) — "similar," not "near-identical." Value L KEPT. |
| V5 | Cleaned `adaptive.py` is byte-identical to live | **CONFIRMED** | All 11 field frames × 10 param-branch combos → identical mask + confidence (this report). |
| V6 | ZED `video.*` params exist, are `[DYNAMIC]`, override-able via `/**:`; no `video:` block in `zed_front.yaml` today | **CONFIRMED** | `common_stereo.yaml:42-49` (each `# [DYNAMIC]`); `zedx.yaml:12-23` native; `zed_front.yaml` goes `general:`(18)→`depth:`(30), no `video:` key. |
| V7 | AE limit cycle is the flicker cause | **UNCERTAIN** | Captured data is stationary with flat exposure (`capture_stats.txt`) → AE was NOT hunting during capture; the lock's benefit is unmeasured. Ship as free/reversible insurance; A/B live. |
| V8 | GMSL ZED X may honor only native `exposure_time`/`analog_gain`, not legacy `exposure`/`gain` | **UNCERTAIN** | Master AE lock (`AEC_AGC=0`) is unambiguous; the locked-VALUE path on this SDK-5.2 unit could not be run here. Set the native keys; verify with `ros2 param get video.exposure_time`. |
| V9 | `confidence` plane consumed downstream (4-topic contract) | **CONFIRMED** | `nav2_params_humble.yaml:367` `confidence_topic`, `:387` `mark_confidence:1`. Removing it breaks the contract. |
| V10 | No pretrained ONNX transfers zero-shot to IGVC tape-on-asphalt | **CONFIRMED (research panel)** | All public models CULane/TuSimple/BDD/Cityscapes-trained, near-horizon windshield mount; IGVC is low-tilt single diagonal tape, no vanishing point (RECOMMENDATION.md §2). |

---

## 8. APPLY PLAN (ship order; each step reversible)

> **Build:** `cd ~/IGVC && colcon build --symlink-install --packages-select avros_perception` (and
> `avros_bringup` only if YAML isn't symlinked). `source install/setup.bash`. Never run RViz on the
> Jetson during nav — use laptop Foxglove.

**Step 1 — ZED exposure/WB lock (config-only, biggest leverage, independently shippable).**
Add the `video:` block (§2A) under `/**: ros__parameters:` in `src/avros_bringup/config/zed_front.yaml`.
Relaunch. Verify:
```bash
ros2 param get /zed_front/zed_node video.auto_exposure_gain   # expect false
ros2 param get /zed_front/zed_node video.exposure_time        # confirm the VALUE wrote on this GMSL unit
```
Field-tune live, watching `/perception/front/overlay` in Foxglove; **drive a short leg** to confirm no
motion blur, then write chosen values back. *Revert:* `video.auto_exposure_gain true`.

**Step 2 — replace `adaptive.py` with the cleaned version (output-identical).**
```bash
cp docs/cv_onnx_research_2026_05_31/adaptive_cleaned.py \
   src/avros_perception/avros_perception/pipelines/adaptive.py
```
Rebuild + relaunch. *Revert:* `git checkout src/avros_perception/avros_perception/pipelines/adaptive.py`.

**Step 3 — align in-code declare defaults (runtime-neutral) in `perception_node.py`.**
`sky_roi_poly` declare default `0.35→0.40` (`:149`); `adaptive_min_area` declare default `15→80` (`:180`).
*Revert:* git checkout.

**Step 4 — confirm `perception.yaml` adaptive block (NO value change needed).**
`adaptive_C: -8.0` (KEEP — do **not** apply the −11.0 draft), `min_area: 80`, `block_size: 21`,
`blur: 9`, `max_sat: 70`, `channel: 'L'`, `use_open: false`, `sky_roi_poly: 0.40`, `class_id_lane: 1`,
`process_at_full_res: true`. All already correct in the live file.

**Step 5 — verify the 4-topic contract intact (any break = silent frame drop):**
```bash
ros2 topic hz /perception/front/semantic_mask                          # ~5 Hz
ros2 topic echo /perception/front/label_info --once                    # latched LabelInfo present
ros2 topic echo /perception/front/semantic_points --field height --once # >1 (organized)
```
Mask H×W must equal cloud H×W (224×128). In Foxglove: overlay traces the near/mid line cleanly, no
scattered lethal speckle; slow-drive shows a stable LETHAL band (not flickering); cmd_vel holds ~13-16 Hz.

*Field fallbacks (each one `ros2 param set`):* speckle returns → raise `adaptive_min_area`; line
fragments → lower `adaptive_min_area`; over/under-exposed → re-set `video.exposure_time`/`analog_gain`,
worst case `video.auto_exposure_gain true`.

---

## 9. OPEN RISKS / NOT-VERIFIABLE

1. **AE-hunt premise unmeasured.** Captured data is stationary with flat exposure → whether the ZED AE
   limit-cycles during a moving run was never A/B-tested. Lock is free + reversible; benefit must be
   confirmed live. (V7)
2. **Motion blur from a fixed exposure** can smear the 3-in line at 1-5 mph — the highest-attention CV-side
   field risk. Keep `exposure_time` short, raise `analog_gain`, verify WHILE DRIVING.
3. **GMSL value path** — the master AE lock is unambiguous, but whether the specific `exposure_time`/
   `analog_gain` values write on this SDK-5.2 unit was not runnable here. Verify with `ros2 param get`. (V8)
4. **Worn/faint paint uncharacterized.** The only data is ONE high-contrast taped line on bright even
   asphalt — no shadow-on-line, glare, ramp deck, or worn tape. Precision-first means recall holes on
   faint paint → boundary-cross risk. No zero-training option closes this; it is the post-competition ONNX
   case (collect bags this weekend → SAM auto-label → fine-tune off-vehicle → TensorRT-EP → slot as the
   reserved `'onnx'` pipeline).
5. **5 Hz perception rate is NOT raised** by any of these changes (it's the executor/cloud-serialize
   problem). `tile_map_decay_time` is 1.5 s ≫ the ~0.47 s worst-case gap; monitor flicker in Foxglove.
6. **ROI-helper de-triplication and the temporal vote are deferred** — both correct but out of scope for a
   safe, minimal, this-weekend finalize.
