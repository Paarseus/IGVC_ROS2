# CV Lane-Detection Approach — Research & Design Brief (for workflow agents)

**Date:** 2026-05-31. **Project:** IGVC_ROS2, AutoNav Challenge ONLY (not Self-Drive).
**Goal of this workflow:** find the BEST, most robust, professional, FAST-TO-DEPLOY approach
for detecting road lines (and white potholes) for the AutoNav course, deciding honestly
whether to switch to ONNX or harden the current classical CV. Score every option against
the IGVC 2026 AutoNav rules.

## DECISIVE USER CONSTRAINTS (these dominate the recommendation — do not ignore)

1. **TIMELINE = IMMINENT (THIS WEEKEND).** The 2026 rules list the competition as
   **May 29 – June 1, 2026** at Oakland University; today is **May 31, 2026**. The team is
   at / right before the competition. Prioritize approaches deployable in **hours-to-days**.
   A from-scratch trained model this weekend is essentially impossible — say so plainly if true.
2. **NO labeled data, NO training GPU.** Custom training is FUTURE work. For "now", only
   zero-training paths qualify: pretrained / off-the-shelf ONNX models, or improved classical CV.
   Investigate honestly whether ANY pretrained lane/road ONNX model transfers zero-shot to
   IGVC (3-inch white tape on asphalt, open field) — most public models are trained on highway
   automotive datasets (TuSimple/CULane/BDD100K/Cityscapes); be skeptical about domain match.
3. **SCOPE = "whatever is most standard & professional, if it works fast I'm open to things."**
   So: keep the 4-topic kiwicampus contract if it's cleanest, but you MAY recommend a more
   standard/professional pipeline (e.g. IPM/BEV occupancy grid) if it is fast to deploy and
   clearly better. Bias toward what real winning IGVC teams actually ship.

## WHAT MUST BE DETECTED (from IGVC 2026 AutoNav rules — full text:
`docs/cv_onnx_research_2026_05_31/igvc_2026_rules_fulltext.txt`)

- **Course surface: ASPHALT pavement** (~500 ft long, 120 ft × 100 ft area).
- **Lane boundaries: continuous OR dashed WHITE lines, ~3 inches wide, taped on asphalt.**
  Track width 10–20 ft; turning radius ≥ 5 ft; primarily **sinusoidal curves**.
- **Simulated potholes: 2-ft-diameter SOLID WHITE CIRCLES** (or plastic mirror) — must be avoided.
- **Barrels/drums** of various colors (white, orange, brown, green, black), plus trees, shrubs,
  light posts, signs. (Currently handled by Velodyne LiDAR→STVL, NOT the camera — see below.)
- **Ramps up to 15% grade.** Min 5 ft clearance between a line and any obstacle.
- Speed 1–5 mph; 6 min limit. Penalties for **boundary (line) crossings** and obstacle collisions
  on the main course. (Note a qualification "Lines Detection" test Q.2 says no penalty for crossing
  during that specific test — but the MAIN COURSE penalizes boundary crossings. Verify exact wording.)
- The camera's CV job for AutoNav = **robustly find the white lane lines (and white pothole
  circles) on asphalt across outdoor lighting (sun, shadow, glare, ramps).**

## CURRENT PERCEPTION PIPELINE (read these files)

- Node: `src/avros_perception/avros_perception/perception_node.py` — subscribes ZED front
  rectified RGB + organized PointCloud2, time-syncs, runs a pluggable Pipeline, publishes a
  **4-topic kiwicampus contract**: `/perception/front/semantic_mask` (mono8 class-id),
  `/semantic_confidence` (mono8), `/semantic_points` (relayed organized cloud),
  `/label_info` (vision_msgs/LabelInfo latched). All 4 share the input stamp.
- Pipelines in `src/avros_perception/avros_perception/pipelines/`:
  - `stub.py` (test), `hsv.py` (per-class HSV inRange + adaptive V-floor),
  - `sooner25.py` (Sooner Robotics 2025 winning code: blur → threshold asphalt (low-S, mid-V)
    → INVERT → everything non-asphalt = obstacle/lane). Fixed-brightness threshold.
  - `adaptive.py` = **CURRENTLY DEPLOYED** (`perception.yaml:34 pipeline: 'adaptive'`).
    Exposure-invariant LOCAL adaptive threshold (`cv2.adaptiveThreshold`, Gaussian, on HLS-L),
    + GaussianBlur de-speckle + **saturation gate** (`adaptive_max_sat:70` drops HLS-S>70 to
    reject orange barrels / tan pillar / grass) + connected-component min-area + sky ROI.
    Runs at full res (480×300) then NEAREST-downsamples mask to cloud shape (224×128).
- Config: `src/avros_perception/config/perception.yaml`, classes in `config/class_map.yaml`
  (1=lane_white, 2=barrel_orange, 3=pothole). Downstream consumer:
  `src/semantic_segmentation_layer/` (vendored kiwicampus fork) runs **in-process inside
  Nav2 controller_server** as a local-costmap layer; lanes marked **LETHAL (254)** by design.

## WHY THE CURRENT CV STRUGGLES (the actual problem to solve)

- **ZED X auto-exposure limit cycle:** AE hunts even while stationary (V_median 141↔167 every
  ~0.3–0.4 s). `sooner25`'s fixed-brightness threshold made the lane mask flicker (375↔909 px,
  2.4×). `adaptive` was created 2026-05-30 to ride this out, but per-pixel mask instability
  remains a SECOND, independent flicker source. **Fixing AE at the source (manual exposure/gain/
  white-balance lock in the ZED wrapper) may be the single highest-leverage fast fix — research it.**
- **Faint / worn / low-contrast paint vs asphalt aggregate speckle:** the code's own comments
  repeatedly conclude "faint lines remain at the classical limit — **ONNX is the robust answer
  for worn/low-contrast paint**" (`perception.yaml:179,191`, `adaptive.py`). This is the crux of
  the user's instinct to switch to ONNX. Test that instinct against the no-data/no-GPU/this-weekend
  reality.
- **Costmap flicker (separate from CV):** local semantic layer `tile_map_decay_time` was 0.3 s but
  perception worst-case inter-frame gap is ~0.47–0.48 s → marked lane tiles purged before re-marking.
  Already being raised 0.3→0.6→1.5 (recent commits). See
  `docs/camera_costmap_flicker_analysis_2026_05_31.md`.

## HARD COMPUTE CONSTRAINTS (read `docs/multicam_grayscale_analysis_2026_05_31.md`,
`docs/perception_framedrop_rca_2026_05_31.md`, `docs/cv_costmap_deep_analysis_2026_05_29.md`)

- **Jetson Orin is CPU-SATURATED.** Currently `nvpmodel` = MODE_30W (only 8 of 12 cores online,
  load 13–17). `zed_node` pegs 100% of a core (NEURAL_LIGHT depth). `controller_server`
  (MPPI + in-process kiwicampus layer) ~94%. **CPU is the binding limit; the first casualty of
  added load is the 20 Hz MPPI control loop** (documented MPPI-starvation → cmd_vel ~3 Hz → goal abort).
- **GPU has headroom** (GR3D bursts to ~96% but mean ~34%). KEY RESEARCH QUESTION: would running CV
  as an **ONNX/TensorRT model ON THE GPU actually RELIEVE the CPU bottleneck** (vs the current
  CPU-bound OpenCV)? If so, ONNX could be a *performance* win independent of accuracy. Verify
  on-Jetson onnxruntime-gpu / TensorRT-EP feasibility on JetPack 6 (L4T R36, SDK 5.2).
- ZED published image 480×300 @ 8 Hz (deliberate software cap to protect MPPI); cloud 224×128.
  Camera: ZED X, 110°(H)×80°(V) FOV, 15° down-tilt (intentional). One front camera only
  (sides exist in config but disabled — enabling them re-triggers MPPI starvation).

## WHAT ACTUALLY WINS IGVC AUTONAV (read `igvc_winners_research/igvc_autonav_winners.json`)

- **OU/Sooner Robotics won 2021, 2023, 2024, 2025 — mostly with HSV thresholding + simple
  morphology + perspective transform (IPM) → occupancy grid, NO LiDAR.** U-Net (CNN) used by
  Sooner only as a supplement to catch BLACK barrels HSV misses; lanes are HSV.
- Their pipeline (autonav_software_2023/2024/2025, public on GitHub): box blur → HSV threshold
  (with run-start auto-calibration in 2025) → "region of disinterest" mask → **perspective
  transform (IPM) → 80×80 occupancy grid → dilation**. This IPM→occupancy-grid step is the
  "standard professional" pattern the current avros pipeline does NOT have (it projects via the
  ZED cloud instead). Live HSV/PID retuning over a custom CAN bus ("CONBus").
- NMIMS 2024 (3rd): YOLOv8n + Canny, vision-only. IIT Kanpur: IPM + SLIC superpixels + random-forest.
  Manipal 2025: HSV + polynomial fit + ZED 2i.
- **Pattern: simple, reactive, classical-CV perception consistently beats heavy learned/SLAM stacks
  at IGVC's scale.** Nav2 is conspicuously absent from winners (we use it). Weigh this.

## FIELD DATA AVAILABLE FOR OFFLINE REASONING

- Captured field RGB frames: `docs/cv_adaptive_debug_2026_05_31/input_rgb_*.png` (480×300, on the
  actual IGVC practice asphalt), `exp_frames/f*.png`, plus mask/overlay outputs and
  `capture_stats.txt`. Agents MAY `Read` these PNGs to characterize the real scene (asphalt
  brightness, line contrast, clutter) — useful for judging zero-shot model domain match.
- ROS bags in `bags/` (mostly motor/odom, not camera).

## DELIVERABLE THE SYNTHESIS MUST PRODUCE

A single decision-grade report with:
1. **Honest ONNX verdict** for the user's exact situation (this weekend, no data, no GPU): is it
   viable now? Is any pretrained model deployable zero-shot? Latency/CPU-relief on Jetson? What's
   the post-competition robust ONNX path (data, model, training)?
2. **Ranked recommendation** — #1 = best fast-deploy approach for THIS WEEKEND with exact config/code
   changes (e.g. ZED exposure lock params, adaptive tuning, IPM/BEV if warranted, costmap decay),
   #2/#3 = alternatives, + the future-robust path.
3. **IGVC AutoNav 2026 rules-compliance matrix** (white lines continuous/dashed, white potholes,
   ramps, boundary-crossing penalties, speed).
4. **CPU/latency budget** showing the recommendation fits the saturated Jetson without starving MPPI.
5. **Concrete deploy + verify steps** (what to change, how to test on the Jetson headless via Foxglove,
   what success looks like), and a fallback.

Be adversarial and honest. If the answer is "don't switch to ONNX this weekend, do X classical fix
instead, and here's the ONNX plan for after," say that with evidence. Cite files as `path:line` and
web sources by URL.
