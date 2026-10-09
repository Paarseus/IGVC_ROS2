# Color-Gated Auto-Canny + IPM-Geometry Lane Pipeline — Design, Validation & Verdict

**Date:** 2026-06-01 · **Project:** IGVC_ROS2, AutoNav · **Model:** Opus 4.8
**Scope:** Add a `canny` lane pipeline to `avros_perception` that resurrects the gradient/Canny
signal the `adaptive` pipeline **deliberately rejected**, but in a form that does not re-import
asphalt texture — validate it **empirically** on the real IGVC frames and give an honest verdict.

> The `adaptive` pipeline's docstring lists, under *"Deliberately NOT included"*, a Sobel/Canny
> OR: *"measured to triple px (280→923) and push the exposure CV 0.031→0.273 by importing asphalt
> aggregate/crack texture … There is no IPM-warp + sliding-window stage downstream to reject that
> noise."* This pipeline adds **exactly that downstream geometric rejection** — and uses Canny as
> an **AND constraint, never an OR**. Unlike a paper design, this one was **run on the actual
> frames** and the shipped defaults were chosen by measurement, not by the docstring's claims.

---

## 1. TL;DR

- **Built & shipped** a fully-working `canny` pipeline behind the existing `Pipeline` abstraction —
  zero downstream changes (same 4-topic kiwicampus contract, single `class_id_lane=1` LETHAL mask,
  sky ROI zeroed last). Registered in `pipelines/__init__.py`; selector still `pipeline:'sooner25'`
  (opt-in, nothing auto-switched).
- **Candidate generator is `adaptive.py`'s exposure-invariant HLS-L local-contrast white-paint
  core** (the only recall source). Auto-Canny (median-scaled), a resolution-relative
  connected-component shape filter, and an optional cached-homography IPM bird's-eye + sliding-window
  polyfit are layered on top — every geometric stage is **gateable off**, so a bad quad or a ramp
  degrades to the proven shape-filter baseline.
- **The headline finding refuted two of the design's own defaults.** Adversarial probing measured
  that, on *this* camera/frames: (a) the hard `white AND dilate(Canny)` fusion is the **single
  worst** operator for exposure (gamma-sweep CV **0.78** with **total dropout to 0 px** at the dark
  AE end → E-stop hazard) and for faint paint (contrast ×0.7 → **364→0 px**); and (b) the
  **uncalibrated** IPM extrapolated white-obstacle vertical edges into **frame-spanning phantom
  LETHAL lanes** (barrel face **1532 px** leak with IPM on, **0** with it off). **Both were flipped
  OFF for the shipped defaults**, and the S6 shape gate was tightened (`elong≥2.5 AND fill≤0.10`)
  so it alone rejects every tested white obstacle while preserving all real tape.
- **Validation verdict: PASS as a candidate, with caveats.** Harness exits 0, 22 frames, no errors.
  Scene frames trace the real white line on both sides of an occluding barrel
  (`overlay_input_rgb_053.png`), asphalt clean, white obstacles **not** marked
  (`overlay_WHITE_OBSTACLE_probe.png`). Exposure rests on natural scene drift (CV **0.074**) + a
  synthetic gamma sweep (CV **0.20**, no dropout) — the literal `exp_frames` set is mono-exposure
  and cannot exercise the AE swing.
- **Recommended posture: do NOT make it the default.** With IPM/depth unwired it is the **S6
  shape-filter baseline** — functionally an `adaptive`-equivalent that also rejects compact white
  obstacles by shape, no more. Keep `adaptive`/`sooner25` primary; carry `canny` as an A/B candidate
  and the home for the IPM/depth geometry once the homography is calibrated and depth is wired.

---

## 2. The problem & why naive Canny was rejected before

IGVC lane lines are **~3-inch white tape** on **asphalt**. The exposure-invariant detector the
codebase converged on (`adaptive`) is a single `cv2.adaptiveThreshold(GAUSSIAN, BINARY, C<0)` on the
HLS-L channel: it fires only pixels **brighter than their local neighborhood**. This is the *only*
physically viable signal, because — measured on the real frames — **the line is only +5 HLS-L over
asphalt globally**, and **line p5 = 132 sits BELOW asphalt p95 = 168**, so no fixed / global / Otsu
absolute level can separate them. Local contrast is all there is.

A previous attempt added a Sobel/Canny **OR** into that mask to catch line edges the threshold
missed. It was rejected for cause, with numbers now baked into the `adaptive` docstring:

- **Pixel explosion:** white-core 280 → OR 923 px (and 1935 → 2562 on a scene frame). The OR pulled
  in **asphalt aggregate, cracks, and tire-scuff texture** — every one of which has a gradient edge.
- **Exposure regression:** the asphalt-texture import pushed the frame-to-frame exposure CV from
  **0.031 → 0.273** — it actively *broke* the invariance the threshold core was built to provide.
- **No downstream rejector:** unlike Udacity-style lane pipelines, there was no IPM warp +
  sliding-window fit to throw the texture back out, so the noise reached the LETHAL costmap layer.

### How THIS design fixes it

Three structural changes, each measured:

1. **Canny is an AND, never an OR (S5).** The candidate is `white AND dilate(auto-Canny edges)`. A
   crack *has* a gradient edge but is **not brighter-than-its-low-sat-neighborhood** (it is usually a
   dark trough), so it dies in the WHITE term. Measured: the white-gated AND cuts in-ROI Canny edges
   **~21,550 → ~3,400** (83% asphalt removed) while keeping **96–97%** of line edge px. This single
   inversion is the fix for the documented 280→923 explosion — *the old pipeline did color OR
   gradient, exactly backwards.*
2. **Auto-Canny, not fixed 50/150 (S4).** Thresholds are **median-derived** (`lo=(1−σ)·median`,
   `hi=(1+σ)·median`) so they ride the ZED AE 141↔167 limit cycle. Fixed 50/150 collapsed final-mask
   IoU to **0.27** under a gamma shift; median-scaled ≈doubled it to **0.58–0.65**.
3. **The downstream geometric rejector the old attempt lacked (S6–S8).** A resolution-relative
   connected-component shape filter (always on) plus an optional cached-homography IPM bird's-eye +
   per-column sliding-window 2nd-order polyfit, so that only **long, smooth, near-vertical
   ground-plane lines** survive. Cracks build no tall BEV column and admit no low-residual fit.

> **What measurement then did to the design.** The above is the *intended* fix. When run on the
> real frames, the **AND** turned out to be a net exposure *regression* vs the plain `adaptive` core
> (it fragments a solid bright ribbon — Canny fires the line's two borders, not its filled interior),
> and the **uncalibrated IPM** never fired on a real near-field line (the line sits *above* the
> eyeballed trapezoid top) yet *did* fire on obstacle edges. So the shipped pipeline keeps the
> **inverted-AND architecture available as a kill-switch** but ships with `edge_and=false` and
> `use_ipm=false`, leaning on the exposure-invariant white core + a tightened shape gate. The fix to
> the *old* failure (texture flood) is real and verified (0 false asphalt px); the *new* geometry is
> shipped dormant until calibrated. See §4 and §7.

---

## 3. The final design — every stage, its `cv2` call, and the adversary it rejects

`run(bgr, depth=None)` in `src/avros_perception/avros_perception/pipelines/canny.py`. Every
pixel-literal is derived from `(h,w)` at runtime via `sc = h/300.0` (`process_at_full_res:true` runs
this at the **published** image size, e.g. 480×300 or SVGA; the node max-pools the mask to cloud
HxW itself). Params are re-read from `self.params` **every frame** (live `ros2 param set`).

| Stage | `cv2` / numpy call | What it does & which adversary it rejects |
|---|---|---|
| **S0** Input guard + res anchor | `if bgr.ndim!=3 or bgr.shape[2]!=3: raise` ; `sc=h/300.0` | Mirrors `adaptive.py`. All kernel/length defaults scale by `sc`. `depth` is in the signature but the node calls `run(bgr)` — S9 is a guarded no-op (`if depth is not None`). |
| **S1** Colorspace | `hls=cv2.cvtColor(bgr,COLOR_BGR2HLS)` ; `chan=hls[:,:,1]` (L) ; `sat=hls[:,:,2]` (S) | One convert, slice both (byte-identical to two converts). HLS order is H,L,S — L is idx **1** (not HSV-V). Optional `createCLAHE(clip,(8,8)).apply(chan)` on L (gated `canny_use_clahe`, **off** — clip>3 washes faint paint). |
| **S2** Denoise | `cv2.GaussianBlur(chan,(bk,bk),0)`, `bk` odd-coerced | Low-passes 1–3 px asphalt-aggregate spikes that out-contrast faint paint and skew the auto-Canny median. **Odd-coercion mandatory** — an even kernel RAISES and stalls `_on_synced`, killing ALL perception. |
| **S3a** White-paint candidate **(the only recall source)** | `cv2.adaptiveThreshold(chan_blur,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,bs,C)`, `bs` odd-coerced, `C=-8` | `C<0 ⇒ T=local_mean+|C| ⇒` only pixels brighter than their neighborhood fire = white paint. **Exposure-invariant by construction** (Adversary 4 — shadows/AE). Reused verbatim from `adaptive.py`. |
| **S3b** Low-sat white gate | `white[sat>max_sat]=0` (`max_sat=70`) | **Adversary 1 (orange/brown/green barrels/cones/tents).** White paint+asphalt are near-grey (S median 11/6); orange band S median 28/p90 95, tent edge 79/p90 235 — drops ~18% orange / ~56% tent-edge, keeps **100%** of line. Their bright edges never reach the S5 AND. (Green clutter and WHITE obstacles pass by definition — that's S6/S8/S9's job.) |
| **S4** Auto-Canny edge | `v=np.median(chan_blur)`; `lo=(1−σ)v`; `hi=(1+σ)v`; `cv2.Canny(chan_blur,lo,hi,apertureSize=ak)` | Median-scaled thresholds **ride the AE swing** so Canny doesn't flicker. `apertureSize` coerced to {3,5,7} (cv2 RAISES otherwise). `chan_blur` is `CV_8UC1` as required. |
| **S5** AND-fusion **(the fix; gated `canny_edge_and`, ships FALSE)** | `ed=cv2.dilate(edges,3×3)`; `candidate=cv2.bitwise_and(white,ed)` | **Adversary 2 (cracks).** Edges are an AND *constraint*. A crack has an edge but isn't brighter-than-neighborhood-low-sat → dies in the WHITE term. The 3×3 dilate lets line-interior pixels adjacent to the gradient survive. **Ships FALSE** (`candidate=white`) — measured to wreck exposure + eat faint paint (see §4); crack rejection is covered by S6. |
| **S6** Shape filter **(ALWAYS ON — primary white-obstacle + crack-speck kill)** | `cv2.connectedComponentsWithStats(candidate,8)`; per-component `elong=max(W,H)/min(W,H)`, `fill=A/(W·H)`; keep iff `A≥min_area AND elong≥2.5 AND fill≤0.10` | **Adversary 3 (WHITE obstacles), axis 1 + Adversary 2 specks.** Real tape: elong 2.93–3.33, fill 0.035–0.056 → **ACCEPT**. Barrel face: elong 1.16–1.30 → REJECT (elong). Cone: 1.38 → REJECT. Disk/bucket: 1.10 → REJECT. Thin white pole: fill 0.13–0.14 → REJECT (fill). Color-independent, resolution-relative, **zero depth needed**. If `use_ipm=false`, `pre` *is* the mask → jump to S9. |
| **S7** IPM bird's-eye warp **(optional, gated `canny_use_ipm`, ships FALSE; M/Minv cached)** | `M=getPerspectiveTransform(src,dst)` (cached, lazy-recompute on (h,w)/param change); `cv2.warpPerspective(pre,M,(bevW,bevH),INTER_NEAREST,BORDER_CONSTANT,0)` | Warp the **BINARY** at INTER_NEAREST (NEVER raw RGB — re-imports the AE cycle, bilinear-sinks faint lines). Off-plane obstacle faces radially **smear into top-flared fans**; ground lines stay narrow columns. `dsize` is **(width,height)** not numpy `.shape`. |
| **S8** Geometric line-fit **(window default / hough alt; BEV only)** | optional `cv2.morphologyEx(bev,MORPH_CLOSE,(1,close_v))` then per-side `np.polyfit(ys,xs,2)` over sliding windows, or `cv2.HoughLinesP(...)` + near-vertical gate | **Adversary 2 + 3, geometric.** Real line peak measured **6.0× noise-floor median** (kept by `col_k=3`); a crack wanders, builds no tall column, fails the per-side `min_inliers` floor and the smooth-fit → annihilated. Per-side **independent** (single-boundary FOV safe). On a miss, `bev_fit=bev` (never blank a real line). The vertical-only `(1,N)` CLOSE bridges dash gaps **along** the line, never across cracks — safe only in BEV. |
| **S8b** Unwarp to image frame | `cv2.warpPerspective(bev_fit,Minv,(w,h),INTER_NEAREST)`; `mask=np.where(...,class_id_lane,0)` | kiwicampus contract requires the mask at **input HxW** (BEV is a different frame). Asymmetric-cost guard: if the whole IPM stack produced an empty mask while `pre` had detections, **fall back to `pre`** (catches the uncalibrated-quad case where the line sits above the trapezoid top). |
| **S9** Depth ground-plane refinement **(GUARDED — DEAD today)** | `if depth is not None and use_depth:` drop `mask[(z_ground−z)>height_tol]` ; `raised &= np.isfinite(z)` | **Adversary 3, axis 2 (the principled white-obstacle kill).** Drops a lane pixel sitting > `height_tol` (0.15 m, wide for the ramp / two_d_mode pitch trap) above expected ground. **NaN/invalid depth is KEPT.** Node calls `run(bgr)` (no depth) → **no-op on every live frame**. Real home is a node-level height test on the organized PointCloud2 (see §7). |
| **S10** Sky/horizon ROI zeroed **LAST** | `cv2.fillPoly(mask,[poly],0)` via `_roi_polygon_px` (copied verbatim from `adaptive.py`) | **Non-negotiable, AFTER all detection** — background tents/trees/people are the brightest pixels in frame (top 40% by default). Empty poly disables. |
| **S11** Output | `confidence=np.where(mask>0,255,0)`; `return PipelineResult(mask,confidence)` | Both `h×w` (== input; node max-pools to cloud HxW). Lanes stay `class_id_lane=1` == **LETHAL** downstream (repo policy — no gradient). |

**Why each adversary is rejected (stacked):**

- **Barrels/cones/tents (colored) — EASY:** killed at **S3b** by the low-S gate; anything that leaks
  is a compact high-fill blob killed at **S6**; in **S7/S8** its off-plane face smears into a BEV fan
  with no column peak. Four stacked rejections, plus Velodyne/STVL as an independent backstop.
- **Cracks/aggregate/tire-scuffs (gray, defeat the color gate) — GEOMETRIC:** (1) **S5** AND (a crack
  isn't brighter-than-neighborhood); (2) **S6** shape (short/irregular/variable-width → fails area +
  elong/fill); (3) **S8** BEV fit (no 6× column peak, large residual). S2 blur pre-suppresses 1–3 px
  spikes. Gap-bridging runs **only in BEV** where the line is straight and gaps are colinear.
- **White obstacles (barrel faces, 2-ft pothole disks — defeat the color gate by definition):**
  **Axis 1 SHAPE (always on)** — compact/high-fill → fails `elong≥2.5 AND fill≤0.10` while a thin
  ribbon passes. *Do not raise `min_area` to reject the disk — a 2-ft disk is large-area; only
  fill/elong separate it.* **Axis 2 DEPTH (S9, principled but DEAD today)** — a tape line is on the
  ground (height≈0), an obstacle face is raised. **Honest caveat:** a thin white **pole** or a white
  stripe **painted on a barrel** is elongated/low-fill/near-vertical and is the residual uncovered
  case until depth is wired — it then rests on Velodyne/STVL. The lane pipeline can only **drop** an
  obstacle from the lane mask; LiDAR/STVL owns its avoidance.

**Exposure invariance (AE kept ON — do NOT lock exposure):** invariant by construction end-to-end —
(a) S3a is a local-contrast test, (b) S4 Canny thresholds are median-derived, (c) S3b is a
relative-channel ratio, (d) S7/S8 operate on the already-BINARIZED BEV (we deliberately do NOT warp
raw grayscale and re-threshold), so geometry adds zero new absolute-brightness dependence.

**Performance (budgeted to NOT starve the 20 Hz MPPI loop on the Jetson Orin):** the node runs
**async** of MPPI at camera rate (~13–16 Hz). S1–S6 are the same O(N) family the production
`adaptive` pipeline already runs in budget, plus ~1 Canny + 1 median + 1 dilate + 1 AND (≈ a few ms).
The optional IPM block adds ~10–20 ms (warp the small BINARY at INTER_NEAREST into 300×400, one
`np.sum(axis=0)`, a 10-window slide + `polyfit`, unwarp) — acceptable **only** because it is async +
on the downscaled BEV. **Three hard rules:** (1) warp the BINARY not the RGB, (2) keep BEV
downscaled (300–512 px), (3) cache M/Minv (never `getPerspectiveTransform` per frame). Escape hatches:
`canny_use_ipm=false` → cheap S6 baseline; or push the warp to `cv2.cuda.warpPerspective` / NVIDIA
VPI VIC (~1–3 ms). **Profile on the Jetson, not a laptop.**

---

## 4. Parameter table (shipped defaults)

29 `canny_*` params + 2 shared (`class_id_lane`, `sky_roi_poly`). All live-tunable. **Shipped
defaults reflect the 2026-06-01 field re-tune** — `edge_and` and `use_ipm` ship **FALSE**, the shape
gate is tightened. Names are prefixed `canny_*` to avoid colliding with `adaptive_*`/`sooner25_*` in
the shared `_pipeline_params` dict (which is **not** cleared on pipeline switch).

| Param | Default | Stage | Purpose / why this value |
|---|---|---|---|
| `canny_block_size` | `21` | S3a | adaptiveThreshold blockSize, odd-coerced (`max(odd,3)`). Same as `adaptive_block_size` — the exposure-invariant local-contrast neighborhood. |
| `canny_C` | `-8.0` | S3a | `C<0 ⇒ T=mean+|C|`. Field-validated −8; do **not** raise (the −11/120 precision tune was refuted — erodes the real faint line). |
| `canny_blur` | `9` | S2 | GaussianBlur kernel on L, odd-coerced (`max(odd,1)`). Matches `adaptive_blur`. Odd-coercion mandatory. |
| `canny_max_sat` | `70` | S3b | HLS-S ceiling; drop `sat>this`. Identical to validated `adaptive_max_sat`. Kills colored barrels/cones/tents; keeps 100% of line. `255` disables. |
| `canny_sigma` | `0.33` | S4 | Auto-Canny median scale (`lo=0.67v`,`hi=1.33v`). The single exposure-invariance knob. **Never** replace with fixed lo/hi (fixed 50/150 → IoU 0.27). |
| `canny_aperture` | `3` | S4 | `cv2.Canny` apertureSize — **must** be in {3,5,7} or cv2 RAISES; coerced to 3 if invalid. |
| `canny_edge_and` | **`false`** | S5 | **Kill-switch, SHIPPED FALSE.** `true`: `white AND dilate(edges)` (the anti-texture rule). Measured: the AND wrecks exposure (gamma CV 0.78, dropout to 0) and faint paint (×0.7 → 0). FALSE → `candidate=white` (the proven exposure-invariant core). Crack rejection covered by S6. |
| `canny_use_clahe` | `false` | S1 | CLAHE-on-L shadow normalizer. Off (A/B only; clip>3 amplifies aggregate, washes faint paint). |
| `canny_clahe_clip` | `2.0` | S1 | createCLAHE clipLimit when enabled (tile 8×8). Keep ≤3.0 (OpenCV default 40 is far too high). |
| `canny_min_area` | `80` | S6 | connectedComponents area floor (speckle). Matches `adaptive_min_area`. **Do NOT raise to reject potholes** — a disk is large-area; shape (elong/fill) separates it. |
| `canny_elong_min` | **`2.5`** | S6 | Min `max(W,H)/min(W,H)`. **REQUIRED (AND).** Margin below the real-tape 2.93 floor; rejects barrel/cone/bucket at elong 1.1–1.4. |
| `canny_fill_max` | **`0.10`** | S6 | Max `A/(W·H)`. **REQUIRED (AND).** Real tape fill 0.035–0.056; rejects the elongated thin white pole at fill 0.13–0.14. |
| `canny_longaxis_min` | `0` | S6 | **NO-OP** (retained for param parity). The old `(elong OR longaxis≥45)` OR-escape that leaked obstacles was **removed**. |
| `canny_longaxis_min_base` | `45` | S6 | **NO-OP** (retained for param parity). Acceptance is strictly `elong AND fill`. |
| `canny_use_ipm` | **`false`** | S7/S8 | **Kill-switch, SHIPPED FALSE.** `true`: full IPM warp + line-fit. Uncalibrated `canny_ipm_src` extrapolated obstacle edges into phantom lanes (barrel face 1532 px with IPM on vs 0 off). Re-pick the quad + verify IPM-on≠off before re-enabling. |
| `canny_ipm_src` | `[0.42,0.42, 0.58,0.42, 1.0,1.0, 0.0,1.0]` | S7 | IPM source trapezoid, NORMALIZED flat (TL,TR,BR,BL), ×(w,h), float32. **UNCALIBRATED eyeballed quad — biggest correctness risk.** Re-pick per camera mount on a real straight-lane frame. M cached. |
| `canny_bev_w` | `300` | S7 | BEV width px (downscaled; protects the 20 Hz loop). `dsize=(bevW,bevH)` — width first. |
| `canny_bev_h` | `400` | S7 | BEV height px. Taller-than-wide → a line is a long near-vertical column. |
| `canny_fit_mode` | `'window'` | S8 | `'window'` = column-hist + sliding-window + 2nd-order polyfit (curve-native, the right tool for the sinusoidal course, per-side independent). `'hough'` = HoughLinesP + angle gate (guarded alt). Both BEV-only. |
| `canny_close_v` | `15` | S8 | Vertical-only `MORPH_CLOSE` SE height in BEV — bridges dashed-line gaps **along** the line. `(1,N)` thin kernel, never across cracks, safe only in BEV. `1` disables. IGVC boundaries may be dashed → ON. |
| `canny_nwindows` | `10` | S8 | Sliding-window count bottom→top (Udacity-lineage). `margin=max(8,bevW//12)`, `minpix=max(4,bevW//24)` track BEV res. |
| `canny_col_k` | `3.0` | S8 | Column-hist keep: `colsum≥k·median`. Real line peak measured **6.0× median** → k=3 cleanly separates line columns from crack/smear specks. |
| `canny_col_floor` | `8` | S8 | Absolute min column sum so a near-empty frame's tiny median doesn't admit noise. |
| `canny_min_inliers` | `150` | S8 | Min inlier px/side to accept a polyfit (prevents hallucinating across a barrel-occluded gap). Re-verify if `bev_w/h` change a lot. |
| `canny_line_draw_w` | `7` | S8 | Rasterized lane stroke width at REFERENCE res, ×`sc` (clamped ≥3), ≈ the warped 3-inch line width. |
| `canny_hough_rho` | `2` | S8 alt | HoughLinesP rho px. Coarser rho tolerates BEV warp jitter. |
| `canny_hough_thresh` | `40` | S8 alt | HoughLinesP vote threshold. `minLineLength/maxLineGap` derived from `sc` (40·sc / 100·sc). Guard `lines is None`. |
| `canny_angle_band` | `35` | S8 alt | Keep BEV segments within ±this° of vertical (rejects curb/crack/obstacle-fan lines). |
| `canny_use_depth` | `true` | S9 | Gate the ground-plane height refinement **IF** valid depth reaches `run()`. Graceful: depth is `None` on every live frame today → silent no-op. Never blocks operation. |
| `canny_height_tol` | `0.15` | S9 | Drop a lane px > this (m) above expected ground. WIDE (≥0.15) for the 15% ramp / two_d_mode pitch trap. NaN-depth KEPT. **Dead until depth is fed.** |
| `class_id_lane` | `1` | S11 | Output class (shared). Single-class, kept **LETHAL** downstream — no gradient. Must match `class_map.yaml`. |
| `sky_roi_poly` | `[0,0, 1,0, 1,0.40, 0,0.40]` | S10 | Normalized sky/horizon ROI zeroed **LAST** (top 40%). Shared. Load-bearing: brightest pixels (tents/trees/people) live above the horizon. Empty disables. |

---

## 5. Validation evidence

**Harness:** `docs/cv_canny_pipeline_2026_06_01/validate_canny.py` — loads `CannyPipeline`
**directly** from the source file via `importlib` (no colcon/ROS; stubs the `base` module), runs the
**actual** shipped pipeline on the captured IGVC frames, writes per-frame overlays, prints a JSON
summary. **Re-run confirmed for this report: exits 0, no stderr, `errors=[]`, 22 frames, cv2 4.13.0.**

```
$ python3 docs/cv_canny_pipeline_2026_06_01/validate_canny.py     # exit 0
pipeline=canny  cv2=4.13.0  frames_tested=22  errors=[]
exp7_lane_px=[0,0,0,0,0,0,0]   exposure_cv_exp7=null   exposure_cv_all_exp=null
mean_recall_proxy=0.078   mean_lane_px=151.5
```

**Per-frame lane px (measured):**

| Frame set | lane px | Note |
|---|---|---|
| `exp_frames` f00–f13 | **all 0** | Only line present is a thin 1px dashed stub at the horizon (components 12–16 px, below `min_area=80`). **The proven `adaptive(min_area=80)` baseline is ALSO 0 here** — a known short-stub-at-horizon limit, **not a canny regression**. |
| scene `input_rgb_{001,010,018,026,035,044,053,061}` | **`[418, 433, 410, 413, 342, 427, 450, 439]`** | Real white line traced on both sides of an occluding barrel; asphalt clean. Recall **UP** from the prior AND config (`[364,…,368]` → `[418,…,439]`) because `edge_and=false` no longer thins the line via the hard AND. |

**Exposure stability:**
- Scene-frame CV = **0.0741** (< 0.15 — good), from natural AE drift across distinct scenes.
- Through-the-pipeline **gamma sweep** (V 173→103, AE-limit-cycle emulation through `run()`):
  `[198,276,298,316,418,256,260,273,266]`, CV = **0.1975**, **NO dropout-to-0** — vs the
  adversary-measured CV **0.598/0.777 + E-stop dropout to 0** under the old `edge_and=true` config.
- `exposure_cv_exp7` / `exposure_cv_all_exp` are **null** (all 14 exp_frames = 0 px → mean 0 → CV
  undefined). The `adaptive` baseline is also 0 on these mono-exposure v133/v134 frames, so they
  **cannot exercise the V 141↔167 AE swing** — a data-capture limit, flagged in §8.

**White-obstacle rejection:** `max highS_leak_proxy = 0.0` on every frame. Through the **actual**
pipeline, a composited white barrel face **and** a white pole are **NOT** marked (component elong
1.1–1.4 / fill ≥0.13 fail the S6 `elong≥2.5 AND fill≤0.10` gate), the real line still traced.

**Other checks:** in-code-default (empty params) output is **byte-identical** to explicit-yaml-param
output. Param parity: 29 `canny_*` names identical across `_PIPELINE_PARAM_NAMES` ==
`declare_parameter` set == `perception.yaml`. `py_compile` OK on `canny.py` + `perception_node.py`;
`yaml.safe_load` OK.

**Overlay images** (in `docs/cv_canny_pipeline_2026_06_01/validation/`):

| Image | Shows |
|---|---|
| `overlay_input_rgb_001.png`, `overlay_input_rgb_053.png` | Green traces the real white line on **both sides of an occluding orange barrel**; asphalt clean. |
| `overlay_WHITE_OBSTACLE_probe.png` | Composited white barrel face **AND** white pole both **NOT** marked; real line still green; total 418 px. |
| `overlay_f00_v134.png` … `overlay_f13_v133.png` | The mono-exposure exp_frames — 0 lane px (short horizon stub below `min_area`, same as the `adaptive` baseline). |
| `overlay_input_rgb_{010,018,026,035,044,061}.png` | The remaining scene frames (342–450 lane px each). |

---

## 6. IGVC-rules compliance notes

| Rule element | How the pipeline handles it |
|---|---|
| **White, ~3-inch continuous lines** | S3a HLS-L local-contrast white core is built for ~3-inch white tape on asphalt; S6 `elong≥2.5 AND fill≤0.10` matches the measured real-tape signature (elong 2.93–3.33, fill 0.035–0.056). `line_draw_w` is sized to the warped 3-inch width. |
| **White dashed lines** | S8 vertical-only `MORPH_CLOSE` (`close_v=15`) + the polyfit/`maxLineGap` bridge **colinear** dash gaps **in BEV only** — IGVC outer boundaries may be dashed and a crossing = E-stop, so bridging is ON by default. *(Inert until IPM is calibrated — see §7/§8.)* |
| **White obstacles (barrels, buckets, poles)** | IGVC obstacles explicitly include **white**, defeating the color gate. Axis 1 = S6 shape (compact/high-fill rejected; verified 0 leak on the tested barrel face / cone / bucket / pole). Axis 2 = S9 depth (principled, **DEAD until wired**). The lane mask only **drops** them; **LiDAR/STVL owns avoidance**. Residual gap: a thin white pole / paint-on-a-barrel (§3 caveat, §8). |
| **Potholes (2-ft solid-white disks)** | Rejected by S6 **shape** — a disk is `elong≈1` → fails `elong≥2.5`. *Do not raise `min_area`* (a 2-ft disk is large-area). The pipeline drops it from the lane class; it must ultimately be **avoided** as an obstacle by LiDAR/STVL (and depth once wired), never mislabeled as a crossable line. |
| **Sinusoidal / curved course** | S8 default fitter is **curve-native** sliding-window 2nd-order polyfit (not straight Hough), per-side independent (single-boundary FOV). S6 keeps a long curved arc (the curve note: at cloud/SVGA res the per-frame visible arc is short → fill stays low). |
| **Ramp (15% grade)** | Static homography + S9 static ground model assume flat ground. Mitigations: WIDE depth band (≥0.15 m), **set `canny_use_ipm=false` on the ramp deck** (S6 has no flat-ground assumption). LiDAR range is already set short so STVL never marks the ramp deck (project policy). |
| **Single LETHAL lane class** | `class_id_lane=1`, kept LETHAL downstream by repo policy — no gradient, stall-safe over line-touch DQ. |

---

## 7. How to enable it + files changed

**Enable (opt-in — nothing auto-switches):**

1. Set the selector in `src/avros_perception/config/perception.yaml`:
   ```yaml
   /**:
     ros__parameters:
       pipeline: 'canny'        # was 'sooner25'
   ```
   (or override at launch: `pipeline:=canny`).
2. Build:
   ```bash
   cd ~/IGVC
   colcon build --symlink-install --packages-select avros_perception
   source install/setup.bash
   ```
3. Run (one ZED camera, front by default):
   ```bash
   ros2 launch avros_perception perception.launch.py
   # or full stack: ros2 launch avros_bringup navigation.launch.py
   ```
4. (Optional) live-tune over the running node, e.g.:
   ```bash
   ros2 param set /perception_node canny_C -8.0
   ros2 param set /perception_node canny_use_ipm false   # ships false
   ```

**What ships ON:** S1–S6 (white core + low-sat gate + shape filter) + S10 sky ROI. The pipeline is
the **S6 shape-filter baseline** out of the box — rejects barrels/cracks/compact white obstacles,
traces continuous tape.

**Before re-enabling the geometry:** `canny_use_ipm:true` and the dashed-gap BEV close are **inert**
until `canny_ipm_src` is **calibrated per camera mount** on a real straight-lane ZED frame — verify
`IPM-on ≠ IPM-off` first, or it extrapolates obstacle edges into phantom lanes.

**Files changed (all already in the tree):**

| File | Change |
|---|---|
| `src/avros_perception/avros_perception/pipelines/canny.py` | New `CannyPipeline` (S0–S11). Shipped defaults: `edge_and=false`, `use_ipm=false`, S6 `elong≥2.5 AND fill≤0.10` (the `(elong OR longaxis)` OR-escape removed; `longaxis_*` retained as NO-OP for parity). |
| `src/avros_perception/avros_perception/pipelines/__init__.py` | Registered `'canny': CannyPipeline` in `PIPELINES`; exported `CannyPipeline`. |
| `src/avros_perception/avros_perception/perception_node.py` | 29 `canny_*` params added to `_PIPELINE_PARAM_NAMES` (lines 72–83) **and** the `declare_parameter` block (lines 260–473). The pipeline is invoked at **`perception_node.py:672` → `self._pipeline.run(bgr)`** (NO depth — S9 is a no-op). |
| `src/avros_perception/config/perception.yaml` | `canny_*` config block (lines ~222–328) with matching defaults + the field-re-tune rationale in comments. Selector unchanged. |
| `docs/cv_canny_pipeline_2026_06_01/validate_canny.py` | Offline validation harness + `DEFAULT_PARAMS` (mirrors the shipped yaml). |
| `docs/cv_canny_pipeline_2026_06_01/validation/*.png` | 23 overlays (see §5). |

> **Param-registration footgun:** every new `canny_*` name **must** be in `_PIPELINE_PARAM_NAMES`
> **AND** the `declare_parameter` block **AND** `perception.yaml`, with matching defaults. A missing
> declaration silently freezes the param at the in-code default (un-settable). Verified parity: **29
> names identical across all three.**

---

## 8. Honest known limits + when to prefer adaptive / yolopv2

**Known limits (in priority order):**

1. **Depth (S9) is NOT wired.** `perception_node.py:672` calls `run(bgr)` with no depth, so the only
   **principled** white-obstacle kill is **dead code on every live frame**. White-obstacle rejection
   degrades to S6 shape + Velodyne/STVL. A thin white **pole** or a **stripe painted on a barrel**
   (elongated/low-fill/near-vertical) can still slip through as a phantom LETHAL lane. **Required
   follow-up:** a node-level height test on the already-synced organized PointCloud2 (`height>1`,
   same HxW family after the resize/pool block), OR extend `_on_synced` + `run()` to pass the
   registered cloud (verified offline to zero a pole leak 535→0). *Do not claim depth rejection works
   until this lands.*
2. **The IPM homography is uncalibrated** — `canny_ipm_src` is an eyeballed normalized quad with no
   intrinsics. On the current 480×300 frames the near-field line sits **above** the trapezoid top, so
   IPM **never fires on a real line** (IPM-on == IPM-off, byte-identical) — meaning the
   curve-native polyfit and the dashed-gap BEV close that justify "Canny over the naive attempt" are
   **inert** until the quad is re-picked per camera mount. Ships `use_ipm=false` for that reason.
3. **The exposure claim is not yet validated on real AE-swing data.** The literal `exp_frames` are
   mono-exposure (V-median 133–134, dV≈1) and yield 0 px (line in the sky ROI / below `min_area`), so
   they **cannot** exercise the V 141↔167 swing the design assumes. Current evidence is natural scene
   drift (CV 0.074) + a synthetic gamma sweep (CV 0.20, no dropout). **A field capture with a
   continuous near-field line AND a real AE swing is needed** to A/B canny vs adaptive on the actual
   invariance claim.
4. **Subtractive stack / asymmetric cost.** S5/S6/S8/S9 are all removal stages — a missed outer line
   = E-stop. Mitigated by `edge_and=false` (pure-adaptive recall), `use_ipm=false` (drop the BEV
   subtractor), window/Hough miss → keep the pre-warp mask, KEEP-on-NaN-depth. Every geometry param
   must be A/B-tested on real worn/dashed frames vs the adaptive baseline; bias every tune toward
   **detection** (the −11/120 precision tune was already refuted for eroding the real line).
5. **Dashed-gap bridging is double-edged** (could hallucinate a continuous LETHAL line across a real
   gap) — but it lives in the IPM stage, which ships off, and is length/curvature-validated by
   `min_inliers` + the smooth-fit requirement. Prefer the additive CLOSE over any `MORPH_OPEN` (open
   erases thin lines 114→17 px in this codebase — stays OFF).
6. **Not built/launched under colcon/ROS here** — validated offline via `importlib` only. Run
   `colcon build --symlink-install` + a live `perception.launch.py` smoke test on the Jetson before
   competition. The odd-coercion guards make a raw `ros2 param set` safe, but the full `_on_synced`
   path was not exercised.

**When to prefer which pipeline:**

- **Prefer `adaptive` / `sooner25` (current default) for competition** until depth + the homography
  land. With IPM/depth off, `canny` is functionally the `adaptive` white core **plus** a tightened
  shape gate that also rejects compact white obstacles — useful, but not a robustness *leap*, and the
  AND/IPM machinery is dormant. On faint/worn/dashed lines, `adaptive` is at least as good (the AND
  that `canny` would add eats faint paint).
- **Prefer `canny` (S6 baseline)** when you specifically want **compact-white-obstacle rejection by
  shape** without depth (e.g. a course with white buckets/disks near the line) — its `elong AND fill`
  gate is the cleanest classical disk/face rejector in the repo. Re-enable IPM **only** after
  calibrating `canny_ipm_src` and confirming `IPM-on ≠ IPM-off`.
- **Prefer `yolopv2`** for **continuous high-contrast tape** as a confirming vote (it cleanly traces
  continuous IGVC tape zero-shot but misses short stubs). It is complementary: `yolopv2` is strong
  exactly where `canny`'s short-stub recall is weak, and `canny` adds a classical white-obstacle
  shape gate `yolopv2` lacks. Neither is a sole source yet.

---

*Validation: `docs/cv_canny_pipeline_2026_06_01/validate_canny.py` (exit 0, 22 frames, cv2 4.13.0).
Source of record: `src/avros_perception/avros_perception/pipelines/canny.py`. Invoked at
`perception_node.py:672` (`run(bgr)`, no depth — S9 dead today).*
