# Computer-Vision → Costmap Pipeline: Deep Analysis

**Project:** IGVC_ROS2 (AutoNav Challenge)
**Date:** 2026-05-29
**Author:** Lead reviewer (final deep-analysis report)

## Executive summary

The camera→costmap path is architecturally sound at its seams — the four-topic kiwicampus contract (mask / confidence / organized cloud / LabelInfo) is wired exactly, with matching stamps, dimensions, frame_ids, and QoS, and the layer stacking (STVL first, semantic second, inflation last, all `updateWithMax`) is textbook. But the stack carries two **competition-day blockers** that are unrelated to tuning: there is no hardware E-stop (software-only `estop` violates IGVC §I.2), and the production launch ships with **all cameras and the perception node disabled by default**, so a stock `ros2 launch ... navigation.launch.py` drives blind to lane lines and potholes. On top of that sit two **high-severity navigation defects** that compound each other: `inflation_radius` (0.3 m) is below the inscribed radius (0.914 m), so MPPI gets a 253-cost cliff instead of a smooth gradient; and lane lines are marked LETHAL (254) with `mark_confidence:1` / `samples_to_max_cost:1` on a **binary** confidence signal, so a single bright pixel becomes an instant un-samplable wall. The active `sooner25` perception pipeline is a single static V>215 threshold with no saturation gate, no temporal vote, no ground-plane check, and no shape discrimination — appropriate for the asphalt course as a base premise, but brittle to glare and lighting. The camera extrinsic remains an unverified, eyeballed placeholder whose error projects rigidly into where lane/pothole LETHAL cells land. None of these is fatal individually; together they are the difference between a robot that completes the course and one that stalls at the first barrel-in-corridor or paints a phantom wall across its own path. This report walks the pipeline, scores it against the IGVC 2026 AutoNav rules, and prioritizes the fixes.

---

## 1. How the Computer-Vision → Costmap pipeline works

### Narrative

The vision path begins at the front **ZED X** camera (the only camera enabled, and only when `enable_zed_front:=true`). The ZED ROS2 wrapper publishes a rectified RGB image and an **organized** point cloud (`/zed_front/zed_node/point_cloud/cloud_registered`, `height > 1`). At the validated SVGA + `point_cloud_res: REDUCED` config the cloud is 224×128 (≈28k points) at 8 Hz — deliberately small to keep the semantic costmap layer from starving the MPPI control loop.

`perception_node` (avros_perception) time-syncs the RGB image and cloud with an `ApproximateTimeSynchronizer` (queue 2, slop 0.02 s). It resizes the RGB to the cloud's H×W so the mask and cloud share an identical row-major index grid, runs the active **`sooner25`** pipeline on the resized frame, and publishes a **four-topic contract**:

- `/perception/front/semantic_mask` — mono8, each pixel a class ID
- `/perception/front/semantic_confidence` — mono8 (binary 0/255 from `sooner25`)
- `/perception/front/semantic_points` — the ZED organized cloud, relayed with a rewritten stamp
- `/perception/front/label_info` — `vision_msgs/LabelInfo` (class-id↔name map), latched RELIABLE + TRANSIENT_LOCAL, published once at init

All four carry the **same** stamp (`max(image.stamp, cloud.stamp)`). The `sooner25` pipeline itself is a stateless single-frame threshold: box-blur → HSV → `inRange([0,0,0],[255,255,215])` (mark "asphalt" = V≤215) → invert (so V>215 = obstacle) → assign the single class `lane_white` (id 1) → zero the top sky-ROI band.

On the consumer side, the **kiwicampus `semantic_segmentation_layer`** (vendored, branch `avros-fixes`) is a Nav2 CostmapLayer plugin running **in-process inside `controller_server`**. For the one `front` observation source it subscribes to the four topics, validates that `mask.W*H == cloud.W*H` (silent drop otherwise), and for each mask pixel `(v,u)` reads the matching organized-cloud 3D point, TF-transforms the cloud to the costmap global frame (`odom`), and projects each in-range point to a 2D tile by its `(x,y)`. Per-tile per-class observation queues decay at `tile_map_decay_time: 0.3 s`. Cost is written as `base_cost`/`max_cost` (both 254 = LETHAL for the `danger` classes). A Bresenham `raytraceFreespace` pass clears FREE_SPACE along rays to currently-seen in-FOV points. The layer merges into the master grid with `updateWithMax`.

In parallel, the **Velodyne VLP-16** feeds **STVL** (the same local costmap, listed first in the plugins) using a tight height band `[0.4, 0.8] m` and 8 m range — the 3D-obstacle (barrel) channel. The **InflationLayer** runs last and inflates the union of STVL + semantic LETHAL cells. The **MPPI controller** then reads this local costmap via its `CostCritic` (`consider_footprint: true`, `cost_weight: 6.0`, `collision_cost: 1e6`) and a 16-point circular footprint (r=0.914 m). The **global** costmap deliberately excludes the semantic layer (GPS-smear protection) and uses STVL only.

### ASCII data-flow diagram

```
 ZED X front camera (zed_wrapper, SVGA + REDUCED cloud, 8 Hz)
   │  /zed_front/.../rgb/color/rect/image (RGB, optical frame)
   │  /zed_front/.../point_cloud/cloud_registered (ORGANIZED cloud, non-optical left frame)
   ▼
 perception_node (avros_perception)
   ├─ ApproximateTimeSync(queue=2, slop=0.02s)
   ├─ resize RGB → cloud H×W (mask/cloud share row-major index)
   ├─ pipeline = sooner25:  blur → HSV → inRange(V≤215) → invert → class_id_lane(1) → zero sky-ROI
   ├─ stamp = max(img.stamp, cloud.stamp)  (written to all 4 outputs)
   ▼  FOUR-TOPIC CONTRACT
   /perception/front/semantic_mask        (mono8, class id)
   /perception/front/semantic_confidence  (mono8, BINARY 0/255)
   /perception/front/semantic_points      (ZED organized cloud, relayed)
   /perception/front/label_info           (LabelInfo, latched RELIABLE+TRANSIENT_LOCAL)
   │
   ▼
 kiwicampus semantic_segmentation_layer  (in-process in controller_server)
   ├─ size gate: mask.W*H == cloud.W*H  (else silent drop)
   ├─ TF cloud → odom (global frame); project each pixel's 3D point to (x,y) tile
   ├─ per-tile queues, decay 0.3s; mark gate: avg_conf > mark_confidence(1) && size>=samples(1)
   ├─ danger {lane_white,barrel_orange,pothole} → base/max_cost = 254 (LETHAL)
   └─ raytraceFreespace (in-FOV only)
         │                                            VLP-16 → velodyne_transform (deskew, odom)
         │                                              │  /velodyne_points
         ▼                                              ▼
   LOCAL COSTMAP plugins = [ stvl_layer , semantic_layer , inflation_layer ]   (all updateWithMax)
                              │ (3D barrels    │ (lanes/potholes   │ (gradient — but
                              │  [0.4,0.8]m)    │  LETHAL 254)       │  radius 0.3 < inscribed 0.914)
                              ▼                 ▼                    ▼
                       ───────────────  master grid  ───────────────
                                          │
                                          ▼
                       MPPI controller  (CostCritic: consider_footprint=true,
                                          cost_weight 6.0, collision_cost 1e6,
                                          footprint r=0.914m)
                                          │
                                          ▼  /cmd_vel → velocity_smoother (DANGLING) → actuator_node

 GLOBAL COSTMAP plugins = [ stvl_layer , inflation_layer ]   (semantic_layer present but enabled:false)
```

---

## 2. IGVC 2026 AutoNav rules compliance

| Requirement | How the stack addresses it | Status |
|---|---|---|
| Hardware mechanical E-stop (red ≥1″, center-rear, 2–4 ft) | None. Only software `estop` flag → serial `S` brake. | **Gap (Critical)** |
| Hardware wireless E-stop (≥100 ft, judge-held) | None. Software-only path through Jetson/ROS2/Teensy. | **Gap (Critical)** |
| Safety light: flashing in autonomous, solid on exit/E-stop | Gated on software `_estop` + `_autonomous_requested` only; would not see a hardware trip. | **Gap (Medium)** — depends on the hardware E-stop being built first |
| Max speed 5 mph (2.24 m/s), **hardware-governed**, fixed after Qual | Speeds conservative (MPPI 0.7, actuator 1.5). Teensy firmware `MAX_RPM=4600` (≈1.53 m/s) is a compile-time ceiling (backstop); `max_linear_mps` is a runtime-mutable ROS param. | **Unknown** — firmware ceiling exists but "hardware governor" is a judge call; document/lock it |
| Min speed 1 mph avg over first 44 ft / 30 s | Physically attainable (slew reaches 0.45 m/s in 1.5 s); no software monitor; early recovery `Wait`/`BackUp` could dip the average. | **Gap (Low)** — capable, not enforced |
| Lane following (white tape on asphalt) | `sooner25` camera pipeline marks white as LETHAL — **but perception is OFF by default**; single static threshold, no temporal/shape robustness. | **Gap (Critical default-off; Medium robustness)** |
| Pothole avoidance (2-ft flat white circles) | Camera-only (LiDAR blind to flush geometry). `sooner25` marks white circle LETHAL (good) but no shape class; perception OFF by default. | **Gap (Medium)** |
| Multi-color barrel avoidance (white/orange/brown/green/black) | LiDAR STVL is the color-agnostic detector; camera only catches bright (V>215) barrels. STVL `[0.4,0.8]m` band has plausible close-range gaps. | **Gap (Medium)** — verify with dark barrel at 2/5/8 m |
| 15% ramp traversal (Nominal Award gate) | No ramp state; `two_d_mode:true` flat TF can make near-horizontal LiDAR beams project the ramp surface into the `[0.4,0.8]m` odom band → false LETHAL. | **Gap (High)** |
| 5-ft (1.52 m) minimum passage | 1.83 m-diameter footprint + LETHAL lanes both sides + LETHAL barrel center + 0.3 m no-gradient inflation → near-total MPPI sample contamination at min-spec corridor. | **Gap (High)** |
| Blocking-traffic / hold-up DQ at 60 s | `movement_time_allowance: 120 s` is 2× the limit; in-controller recovery effectively disabled for the window (45 s BT Timeout aborts but does not run recovery). | **Gap (High)** |
| No prior map / mapping forbidden | No StaticLayer, no map_server, rolling-window costmaps, GPS waypoints via `/fromLL`. | **OK** |
| GPS waypoint navigation (2 m tolerance) | navsat/EKF/Navfn; goal xy tolerance 2.0 m. | **OK** |
| Tactile sensors banned | All detection non-contact (LiDAR + camera). | **OK** |

---

## 3. Findings by severity

Severity reflects the verifier-calibrated values. Where the original finding's framing was corrected by the verifier, the corrected reading is used.

### Critical

**[rules-compliance-1] No hardware E-stop (software-only `estop` violates §I.2)**
File: `src/avros_control/avros_control/actuator_node.py:331-337,474-479`; `docs/igvc_rules/IGVC_2026_rules.txt:166-175`.
The only estop is software: an `ActuatorCommand{estop:true}` sets `self._estop` → serial `S`. There is no GPIO/relay/contactor cut path independent of the Jetson/ROS2/Teensy chain. Rules require **both** a hardware mechanical mushroom button and a hardware wireless E-stop (judge-held), each bringing the vehicle to a quick complete stop. The Teensy firmware has no estop input pin. This is a Qualification blocker, not a tuning issue.
**Fix:** build a normally-closed series safety loop (mechanical button + wireless receiver) driving a relay that cuts the SparkMAX enable / motor bus, on a rail that survives the 12 V brown-out (per `ESTOP_REQUIREMENTS_2026.md §7`). Keep software estop as a secondary layer. Add a Teensy GPIO reading the loop state.

**[arch-gaps-1] Production stack runs LiDAR-only by default — perception and all cameras disabled**
File: `src/avros_bringup/launch/navigation.launch.py:142-170`.
`enable_zed_front`, `enable_zed_left`, `enable_zed_right`, and `enable_perception` all default to `false`. A bare `ros2 launch avros_bringup navigation.launch.py` brings up Nav2 with the semantic layer loaded but **zero publishers** on `/perception/*` — the layer sits idle (no error, `expected_update_rate=0.0`). The robot avoids only 3D LiDAR geometry. White tape (zero height) and flat potholes are invisible to STVL — both are camera-only. Lane following is a Qualification requirement.
**Fix:** ship a `competition.launch.py` (or flip defaults) that sets `enable_zed_front:=true enable_perception:=true`. Add a health-WARN that screams "PERCEPTION DISABLED" when Nav2+semantic_layer are up but perception is off, so LiDAR-only is never entered accidentally during a scored run.

### High

**[cv-pipeline-2] Binary confidence + `mark_confidence:1` + `samples_to_max_cost:1` → LETHAL on a single bright pixel**
File: `pipelines/sooner25.py:115`; `nav2_params_humble.yaml:384-388`.
All pipelines emit `confidence = where(mask>0, 255, 0)` (binary; `stub.py` emits all-255). The plugin gate is `size>=samples_to_max_cost && confidence_sum/size > mark_confidence` → with `(1, 1)` this is unconditionally true on the first observation carrying any pixel. There is **zero** confidence filtering: one transient bright pixel (glare flicker, ISP noise, a sun-spangle) → one cloud point → one tile → instant LETHAL (254), held for multiple MPPI cycles by decay/STVL persistence.
**Fix:** emit a **graded** confidence (distance above threshold, or connected-component area), then raise `mark_confidence` and/or `samples_to_max_cost` to 2–3 so a feature must persist across pixels/frames before marking LETHAL.

**[costmap-integration-2 / rules-compliance-5] `inflation_radius` 0.3 m < inscribed 0.914 m → 253 cliff, not a gradient**
File: `nav2_params_humble.yaml:480-481,619-620,265-266,515-516`.
Both costmaps set `inflation_radius: 0.3` with a 16-pt circle footprint r=0.914 m (inscribed ≈ 0.914 m; the YAML's "0.794" is stale). Nav2's InflationLayer needs `inflation_radius ≥ inscribed_radius` for the `exp(-cost_scaling*(d-inscribed))` decay regime to exist. At 0.3 < 0.914, every inflated cell is `INSCRIBED_INFLATED_OBSTACLE` (253) — a flat plateau — and `cost_scaling_factor: 3.0` is inert. MPPI's `CostCritic` gets near-uniform cost across samples; the centerline-pull gradient vanishes. This is the documented root of "Optimizer fail to compute path" and gives only ~0.43 s of warning at vx 0.7 m/s.
**Fix:** raise `inflation_radius` to ≥ 0.4 m, ideally 0.5–0.6 m local (the max that still fits a 3 m lane after bilateral inflation); consider a larger global radius (~0.65 m, the prior value) since the global map has no corridor constraint. Pair with making lanes traversable (below) so the corridor is not double-constrained.

**[costmap-integration-4 / rules-compliance-5] LETHAL lanes + `consider_footprint` + 0.3 m inflation → barrel-in-corridor sample starvation**
File: `nav2_params_humble.yaml:384,165,167,168,481,266`.
IGVC guarantees only a 5 ft (1.524 m) gap on one side; the robot is 1.828 m diameter. With LETHAL lanes both sides + a LETHAL barrel mid-corridor + `consider_footprint:true` + `collision_cost:1e6`, the geometry leaves a mutually-exclusive passable band — the overwhelming majority of MPPI samples sweep at least one LETHAL cell and are eliminated, producing oscillation/stall. **Note (verifier correction):** crossing an internal lane line is itself an **E-stop end of run** (rules line 342), so "just cross the lane" is *not* a cheap escape — the real fix is to make the corridor passable, not to license crossings.
**Fix:** make `lane_white` high-but-traversable (`base_cost ≈ 180–220`) while keeping barrels LETHAL — which **requires** the perception split below (today everything is `lane_white`). Then MPPI can numerically minimize a lane brush instead of finding zero valid samples. Pair with the inflation fix.

**[frame-tf-projection-1] Camera mount extrinsic is an unverified eyeballed placeholder**
File: `src/avros_bringup/urdf/avros.urdf.xacro:162-179`.
The semantic cloud is projected to costmap tiles purely by the global `(x,y)` of each point, transformed through a TF chain whose first link is the hand-set mount joint (`xyz="0.6795 0 0.4476" rpy="0 15° 0"`), still carrying `TODO: Measure exact mount position`. The offset was already changed ~28 cm once and the 15° pitch was added "by eye." Because the transform is rigid, a translation error shifts **every** lane/pothole LETHAL cell by that vector; a pitch error is range-amplified (a 5° pitch error at ~0.45 m height moves the ground intersection 0.4+ m at lane range). For a robot threading 1.52 m gaps and avoiding 0.61 m potholes, a 0.3–0.6 m systematic projection bias can place a lethal stripe across the driveable corridor with no reported error.
**Fix:** calibrate empirically — place a tape mark at a measured `(x,y)`, echo `/local_costmap/costmap`, confirm the LETHAL centroid lands within ~1 cell (0.2 m); iterate the mount xyz+pitch until it does; record it and remove the TODO. The pitch term especially needs a ground-truth check.

**[rules-compliance-4] `movement_time_allowance: 120 s` exceeds the 60 s blocking-traffic DQ**
File: `nav2_params_humble.yaml:63-66`.
The progress_checker (the layer that triggers MPPI's recovery sub-tree) tolerates no 0.3 m progress for a full 120 s — double the 60 s DQ. The 45 s BT `Timeout` aborts the action first but **does not run recovery**; it propagates FAILURE upward. So the in-controller recovery (ClearAround→Wait→BackUp→Crawl) effectively never fires inside the competition window.
**Fix:** set `movement_time_allowance` to 10–15 s so the RoundRobin recovery can run ~3 times within the 45 s BT Timeout, giving a real chance to unstick before DQ.

**[rules-compliance-6] No ramp-aware handling — `two_d_mode` flat TF false-marks the ramp deck**
File: `nav2_params_humble.yaml:304-322`; `ekf.yaml` (`two_d_mode:true` both EKFs).
Reaching one ramp waypoint in No-Man's-Land is the **Nominal Award** gate. With `two_d_mode:true` the odom→base_link TF stays flat even when the chassis pitches ~8.5° on a 15% ramp. STVL transforms the VLP-16 cloud through that flat TF, so near-horizontal beams that physically hit the rising ramp surface are interpreted at z≈0.45–0.64 m in odom — squarely inside `[0.4,0.8]m`. Confirmed beams: −5°@3.06 m→0.449 m, −3°@3.58 m→0.528 m, −1°@4.32 m→0.640 m, all within 8 m range. The recovery `ClearCostmapAroundRobot(reset_distance=3.0)` cannot clear the 3.58 m / 4.32 m marks; MPPI stays blocked, goal aborts, mission_manager skips the ramp waypoint. `mission_manager.py` has no ramp/pitch/vx_max state despite the BT header anticipating it.
**Fix:** test STVL pitched on a 15% incline. Gate STVL height band on IMU pitch (or temporarily widen/raise the band) when pitch exceeds a threshold; implement the ramp state in `mission_manager`.

**[perf-budget-4 / config-consistency-3] Side-camera YAMLs would reproduce the 3 Hz MPPI starvation if enabled**
File: `src/avros_bringup/config/zed_left.yaml`, `zed_right.yaml`.
Both omit `point_cloud_res` (inherit COMPACT ≈114k pts, 4× the front's REDUCED) and run `pub_frame_rate/point_cloud_freq: 15.0` vs front's 8.0. The front was tuned to REDUCED+8 Hz specifically because COMPACT collapsed `/cmd_vel` to ~3 Hz. Enabling left+right as-is feeds the single-core in-process semantic buffer ~8× the front-only load per side → a guaranteed return to sub-5 Hz cmd_vel and actuator timeout. (Latent — side cameras are Phase 5, not wired by default.) **Verifier correction:** the front↔right "serial collision" claim is wrong — front uses `42569280`, right uses `49910017` (a real but unverified-assignment unit), so it would open *a* camera, not none.
**Fix:** before any multi-camera enable, add `point_cloud_res:'REDUCED'` and drop `pub_frame_rate`/`point_cloud_freq` to 8.0 on both side YAMLs; verify the assigned serials; re-measure `/cmd_vel` with all cameras on. Better: move the semantic costmap to its own lifecycle node first.

**[arch-gaps-6] Sim has zero parity with the real perception/costmap stack**
File: `src/avros_sim/launch/sim_navigation.launch.py:37`; `nav2_sim_overrides.yaml`.
The Webots world has no camera and no lane/barrel/pothole geometry; sim launches no perception/ZED/semantic layer, selects a **different BT** (`navigate_route_graph_humble.xml` vs production `navigate_igvc_autonav_humble.xml`), and sets `use_collision_detection:false` with a coarse 0.5 m / 100×100 costmap vs the real 0.2 m / 50×50 local. Every vision-dependent behavior — lane following, pothole avoidance, semantic marking, the LETHAL-lane / inflation-cliff interaction — is **untestable** in sim. The highest-risk parts of the stack have no regression harness and can only be debugged in costly field tests.
**Fix:** either raise sim parity (Webots camera publishing `/zed_front/...`, white-line/barrel/pothole geometry, perception+semantic layer+same BT+real costmap resolution+collision on) or explicitly scope-document sim as GPS-routing-only and invest in a structured field-test matrix. Do not let the BT/costmap/collision divergence persist silently.

**[commits-and-docs] kiwicampus fork patches at risk of being lost on `vcs import`**
File: `avros.repos` pin / `src/semantic_segmentation_layer` (git-ignored).
The Humble-compat and behavior patches (TimePointZero TF fallback, wall-clock decay, raytrace clearing, headroom patches) live only in the vendored fork. If a teammate runs `vcs import src < avros.repos` without the pin bumped to a fork commit that contains them, they silently disappear — a competition-day reproducibility risk.
**Fix:** push all patches to `Paarseus/semantic_segmentation_layer`, bump the `avros.repos` pin to that commit, and confirm a clean `vcs import` reproduces them.

### Medium

**[cv-pipeline-1] `sooner25` has no saturation gate — any bright pixel of any color becomes LETHAL**
File: `perception.yaml:148`; `pipelines/sooner25.py`.
`sooner25_upper: [255, 255, 215]` → S-ceiling disabled; only V≤215 discriminates. After inversion, **any** V>215 pixel (glare V=255, specular highlights, light barrels, operator clothing, the robot's own bright bodywork in-frame) → LETHAL. The S=255 choice was deliberate (documented: avoid grass false-positives on a grass test course) and validated on-course, and the top-46% sky ROI mitigates sky/building pixels — but for the asphalt competition surface the residual glare/bright-object risk in the lower 54% is real.
**Fix:** restore an achromatic S ceiling (~60–95) so only near-white/gray bright pixels survive; re-validate that grass no longer false-positives and white tape still passes; realign the code default (below).

**[cv-pipeline-3 / arch-gaps-2 / rules-compliance-3] Potholes detected as `lane_white` with no shape discrimination**
File: `pipelines/sooner25.py:90-116`; `class_map.yaml:17`.
Flush 2-ft white circles are LiDAR-invisible (STVL band/range). `sooner25` *will* light them up (V>215, still LETHAL → avoidance works **when perception is on**) but collapses them into `lane_white` — class id 3 (`pothole`) is never emitted (dead ID). Avoidance is unaffected; the scored "pothole detection" capability cannot be claimed, and pothole vs lane cannot be cost-differentiated. (Verifier: the `hsv.py` pothole path is functional but neutered in YAML — switching pipeline + un-neutering would recover class 3.)
**Fix:** add a cheap `connectedComponentsWithStats` shape pass — classify compact near-circular blobs as `pothole` (3), leave elongated components as `lane_white` (1); both stay LETHAL but carry the correct class.

**[cv-pipeline-4] No shadow/glare/temporal handling — transient bright artifacts become phantom LETHAL**
File: `pipelines/sooner25.py:90-116`.
`sooner25` is stateless per-frame; the box-blur **spreads** bright blooms rather than suppressing them; binary confidence + `(1,1)` mark gate means any one-frame flood is written LETHAL immediately. Costmap decay limits persistence to roughly the next update unless the glare repeats, so a sustained stall needs a persistent glare condition — but on a sunny outdoor run these transients are the dominant false-positive source.
**Fix:** add a temporal vote (bright in M of last N frames, or `samples_to_max_cost ≥ 2` with graded confidence), a glare guard for saturated V=255 clusters, and replace box-blur with **median** blur.

**[cv-pipeline-5] No ground-plane / projection sanity check — off-ground bright pixels project as on-ground LETHAL**
File: `pipelines/sooner25.py:22-25`; `perception_node.py:373-376`; segmentation_buffer (no height gate).
Geometry is fully deferred to the organized-cloud projection, which has no z/height gate (only a 3D spherical radial gate). A bright pixel on a barrel face, building wall, tree trunk, or person's torso whose cloud z is above ground (but within 5 m) is written to the `(x,y)` tile where its ray pierces, regardless of height. The only height-ish guard is the static top-ROI rectangle (actually top **35%** per the code default, not 46%), which cannot follow a tilted horizon on the ramp.
**Fix:** add a ground-plane height gate in `perception_node` (drop mask pixels whose paired cloud z > ~0.15 m above base_link) so the camera marks only flat ground targets and STVL handles verticals; make the sky ROI horizon-aware (or widen conservatively) for the ramp.

**[cv-pipeline-6] No lower-frame / robot-body exclusion in `sooner25`**
File: `pipelines/sooner25.py:110-113`; `perception.yaml:93`.
`sooner25` applies only the top sky ROI; the entire lower 54% is live. `hsv.py`'s `lane_band [0.46,0.70]` body/hood guard was dropped. Bright bodywork/hardware at the bottom of the ZED frame → marked obstacle → projected via **near-field** cloud points directly in front of the robot, the worst place for a phantom LETHAL. Whether bodywork is actually in-frame depends on the real mount/tilt — verify physically.
**Fix:** add a configurable bottom-of-frame mask; zero the rows that show bodywork on the real vehicle. If `lane_band`-style runtime tuning is wanted, declare those params and add to `_PIPELINE_PARAM_NAMES`.

**[cv-pipeline-7] Single hand-tuned V threshold (215) is the entire detector — scene-dependent, ~10–15 count margin**
File: `perception.yaml:144-148`.
The whole detector is one scalar tuned to asphalt p95 V=204. Overcast drops tape V toward asphalt (missed lanes); full sun raises asphalt V past 215 (asphalt itself inverts to obstacle). No adaptive component in `sooner25` (hsv.py's adaptive-V is disabled and unused). ONNX is "Phase 6," not present.
**Fix:** port an adaptive upper-V floor into `sooner25` (sample a near-field road ROI percentile every N frames, set `upper = pXX + margin`) so the threshold tracks ambient brightness. Re-calibrate on the real asphalt under multiple lighting conditions.

**[costmap-integration-1] Semantic layer can never clear a stale painted cell that leaves the FOV**
File: `semantic_segmentation_layer.cpp:365-403,437-447,701-781`.
When a tile's queue decays empty, the loop `continue`s without writing FREE_SPACE — the layer's `costmap_[index]` retains its LETHAL value. `raytraceFreespace` clears only cells between the sensor and **current-frame** points (in-FOV). `updateWithMax` then prevents the master grid from being lowered. So a stripe/barrel that leaves the FOV (robot turns away, occlusion) persists LETHAL until the rolling window drops it >5 m behind. **Verifier nuance:** in-FOV clearing *is* working (covers forward driving through a lane); the gap is specifically out-of-FOV cells, and IGVC lanes usually run alongside the path — the deadlock case is a stripe directly ahead after a turn.
**Fix:** (a) lower `lane_white` cost below LETHAL (~200–220) so a stale ghost is repulsion not a wall; (b) shorten the local rolling window or have BT recovery run a periodic `ClearEntireCostmap` on the semantic layer specifically. Validate: paint a lane, turn 90°, confirm the stripe clears within ~1 s.

**[costmap-integration-6] Active `sooner25` emits only `lane_white` — lanes can't be softened without softening barrels**
File: `nav2_params_humble.yaml:382-388`; `sooner25.py`.
The single `danger` block groups `lane_white`/`barrel_orange`/`pothole` at 254. Since `sooner25` emits only id 1, **all** camera obstacles land in `lane_white` — the finding-1/finding-4 "lanes traversable, barrels LETHAL" fix is inexpressible today. (STVL independently marks 3D barrels, so this isn't a barrel blind spot in 3D.)
**Fix:** extend the pipeline to emit `barrel_orange` for orange blobs (keep non-orange bright as `lane_white`), then split the YAML into `lane` (~200) and `barrel` (254) class types; or, interim, make camera lanes traversable and rely on STVL for hard barrel stops (after verifying STVL barrel coverage).

**[rules-compliance-2] Safety light gated only on the software estop flag**
File: `actuator_node.py:487-497`.
`light = 'A1' if (autonomous_requested and not _estop) else 'A0'`. A hardware E-stop (once built) that cuts power independent of the Jetson never sets `_estop`, so the light would keep flashing after a hardware trip — non-compliant. Latent until the hardware E-stop exists.
**Fix:** feed a Teensy GPIO reading the safety-relay state into the gate: `'A1' if autonomous and not sw_estop and hw_loop_closed else 'A0'`.

**[rules-compliance-7] Multi-color barrels: camera misses dark barrels; STVL band has close-range gaps**
File: `sooner25.py:101-108`; `nav2_params_humble.yaml:321-322`.
`sooner25` (brightness-based) catches white/light barrels but a black/dark barrel may not exceed V>215. Dark barrels rely entirely on STVL `[0.4,0.8]m`; with `mark_threshold:1` (≥2 returns/voxel) a very close (1–2 m) barrel can momentarily fall out of the band or under-mark.
**Fix:** do not rely on the camera for barrels. Verify a dark barrel at 2/5/8 m produces ≥2 voxels in a column within `[0.4,0.8]m` at the real mount height; widen `max_obstacle_height` or add a near-field band if the close range drops out.

**[rules-compliance-8] Max speed software-limited, but the rule requires a hardware governor**
File: `actuator_params.yaml:34`; `nav2_params_humble.yaml:106`.
Configured speeds are conservative and compliant in magnitude. **Verifier correction:** the Teensy firmware *does* have a compile-time `MAX_RPM=4600` (≈1.53 m/s) backstop that cannot be changed at runtime — closer to a hardware governor than a software param. The residual concern is that `max_linear_mps` is a runtime-mutable `[DYNAMIC]` ROS param ("no changes after Qual").
**Fix:** present/document the Teensy `MAX_RPM` ceiling as the hardware governor for Qualification; treat host ROS caps as a sub-limit; do not allow `max_linear_mps` to exceed the firmware ceiling.

**[arch-gaps-3 / launch-wiring] Costmap declares front+left+right but only `front` is subscribed and published**
File: `nav2_params_humble.yaml:362`; launch defaults.
`observation_sources: front` only — the `left`/`right` blocks are full but inert. Even if listed, no node publishes their topics by default, and `perception_cameras` is not coupled to `enable_zed_left/right`. Dead config that reads as production.
**Fix:** commit to single-front for AutoNav and **delete** the left/right blocks (and unused zed_left/right launch entries), or fully wire multi-camera (extend `observation_sources`, couple `perception_cameras` to the enable flags, fix the side YAMLs).

**[arch-gaps-4] Single forward camera may not see both peripheral lane boundaries through switchbacks**
File: `avros.urdf.xacro:179`.
AutoNav lane-keeping needs both boundaries, especially in switchbacks. The side cameras were provisioned (URDF frames, YAMLs, dead costmap blocks) but not activated. **Verifier corrections:** the 15° down-tilt does **not** reduce horizontal FOV; both ±1.524 m boundaries are in the ZED's ±55° HFOV at any forward distance ≥1.07 m; and the side cameras are opt-in (default false), not "unwired." Medium-confidence, depends on real lens HFOV + mount.
**Fix:** empirically measure front lane-boundary coverage through an IGVC switchback before committing to single-camera; if a boundary drops out, finish wiring the side cameras (they were intended for this) or reduce down-tilt + add a bird's-eye warp.

**[arch-gaps-5] No sensor-fusion redundancy or perception/sensor failure detection**
File: `actuator_node.py:400-404`.
The only watchdog is the 0.5 s cmd_vel staleness brake. `expected_update_rate=0.0` disables the kiwicampus stale-source warning, so a dead ZED / perception node / mask stream silently stops lane/pothole marking and the robot keeps driving on LiDAR-only with no flag. No liveness monitor for `/velodyne_points`, the ZED cloud, or the semantic mask. (Obstacle avoidance is LiDAR-driven, so a vision blackout degrades lane-following but doesn't blind the robot to barrels.)
**Fix:** add a lightweight health-monitor node watching those topic rates; set `expected_update_rate` to a real value; document the degraded-mode policy (LiDAR-only OK for obstacle avoidance, NOT for lane following → stop or slow).

**[arch-gaps-8] Active pipeline is single-class — defeats per-class costing even when enabled**
File: `sooner25.py`.
Even with perception on, the camera provides one undifferentiated "bright = obstacle" channel; lane vs barrel vs pothole vs glare/concrete are indistinguishable. Defeats any per-class cost strategy and (with no S gate) lets glare/bright concrete create false LETHAL.
**Fix:** move toward multi-class output matching `class_map.yaml` (lane via elongated/boundary context, barrels LiDAR-primary + orange-secondary, potholes via compact-blob); add the S gate and temporal filter; at minimum enable per-class costing.

**[config-consistency-6 / commits-and-docs] `velocity_smoother` output is read by nobody — all its limits are inert**
File: `nav2_params_humble.yaml:626-639`; `navigation.launch.py:85,228`; `actuator_node.py:252`.
The smoother is in the lifecycle but no remaps exist, so it subscribes `cmd_vel` and publishes `cmd_vel_smoothed` into the void; the actuator reads raw `/cmd_vel`. Its `max_velocity vx=2.0` exceeds MPPI (0.7) and actuator (1.5), and `max_accel 0.5` is looser than actuator (0.3) — misleading dead config that will make a future tuner think the actuator ignores limits.
**Fix:** either wire it canonically (controller→`cmd_vel_nav`, smoother in `cmd_vel_nav` out `cmd_vel`, actuator reads `cmd_vel`) with limits aligned to the actuator, or remove it from the lifecycle.

### Low

These are confirmed but narrow; listed compactly.

- **[cv-pipeline-9 / config-consistency-9]** Code default `sooner25_upper [255,95,210]` (and `lane_high [179,60,255]`, `sync_queue 10`, others) disagree with YAML — a bare `ros2 run` (no YAML) silently runs a different, more-conservative detector. **Fix:** sync the `declare_parameter` defaults to the validated YAML values.
- **[cv-pipeline-8]** Barrel/pothole class IDs are advertised + configured as danger but never emitted by `sooner25` — dead semantic capacity (harmless for avoidance; intentional per docstring). **Fix:** document as reserved-but-unused, or re-introduce color/shape passes.
- **[cv-pipeline-10 / topic-contract-1 / topic-contract-5 / costmap-integration-5/7 / frame-tf-projection-2/3/4 / rules-compliance-9/10 / perf-budget-2/5/6/7 / config-consistency-10]** Correct/well-designed properties — see §7.
- **[topic-contract-2]** Published RGB (1.6 aspect) is anisotropically stretched to the REDUCED cloud (1.75 aspect) — ~9% geometric distortion, but **self-consistent** (mask and cloud share the 224×128 grid), subordinate to thresholding error. **Fix (optional):** match `pub_resolution` aspect to 1.75, or just document.
- **[topic-contract-3]** `process_at_full_res=true` (dormant default-false) downsample path silently skips the max-pool when the cloud is larger than the mask (guard prevents the ZeroDivision; the cost is lost thin-line fidelity, not a crash). **Fix:** log a WARN when the cloud exceeds the mask in any axis.
- **[topic-contract-4]** Confidence gate is safe today only because `base_cost==max_cost==254`; a future `mark_confidence:255` (uint8, strict `>`) would silently disable the gate. **Fix:** add a guard comment in the YAML.
- **[frame-tf-projection-2]** CLAUDE.md TF tree is wrong: the **cloud** is in `zed_front_left_camera_frame` (non-optical, x-fwd/z-up), the **image/mask** in the optical frame — they differ by the 90° optical rotation. Harmless (the plugin transforms by the cloud's own frame_id) but should be corrected to prevent a future "fix" that breaks the index contract.
- **[frame-tf-projection-5]** Cloud projection depends on TF being valid at `cloud.header.stamp` through a deep chain; ZED stamp lag + CPU starvation can drop frames (fail-safe: marks vanish, not mislocate). `transform_tolerance 0.5` only extends toward the future, not past-extrapolation. **Fix:** confirm the TimePointZero fallback is vendored/pinned; log a rate-limited dropped-frame count.
- **[perf-budget-3]** `publish_voxel_map: true` on the **local** STVL serializes an O(N_voxels) debug cloud every cycle (global correctly false) — pure CPU waste in the exact starved process. **Fix:** set it `false` for competition (one line).
- **[config-consistency-1/2/7]** The **fallback** `nav2_params.yaml` (used only on non-Humble distros, currently inactive) has drifted badly: three separate semantic-layer plugin instances (broken multi-camera anti-pattern), missing `ignored` class_type (CRITICAL ERROR log spam), RPP/1.8 m inflation/1.5 s decay/20 m raytrace. **Fix:** delete it and fail loudly on non-Humble, or re-derive from the Humble config.
- **[config-consistency-4/5/8]** Side-camera 15 Hz rate mismatch (latent); `lane_band`/`lane_close_w`/`lane_min_area` not declared as ROS params (HSV-only, inactive); MPPI `az_max` vs actuator symmetric-angular coupling to track if a separate angular decel is added.
- **[commits-and-docs / BT]** `DriveOnHeading` (recovery index 3) is unreachable with `number_of_retries=3` (one-char fix to 4); `BackUp time_allowance=25 s` blows the 45 s budget (set ~5 s); `ClearCostmapAroundRobot` with no plugins filter wipes the lane cells it was meant to preserve (add `plugins='stvl_layer'` or reduce reset_distance); actuator heading-hold P-controller fires on Nav2 straight cmd_vel (cascade-ordering violation — gate to webui/teleop only). EKF debts (`differential:true`, squared rejection threshold ≈190, inflated process noise; `decay_acceleration=2.0` now under-tuned) are localization-side, out of this report's CV→costmap scope but recorded in `commits-and-docs`.

---

## 4. Architectural problems (cross-cutting)

1. **Default-disabled perception.** The single most consequential structural issue: the production launch is LiDAR-only by default (§3 Critical / arch-gaps-1). Lane following and pothole avoidance — both camera-only, both Qualification-relevant — are off unless four flags are remembered. This is a posture problem, not a bug: the safe default for a *competition* profile should be vision-on.

2. **Dangling left/right sources.** Three places carry half-wired multi-camera config (dead `observation_sources` blocks, default-off drivers, side YAMLs that would re-trigger the 3 Hz starvation). It reads as production, invites mis-tuning, and is a latent performance + hardware-commissioning trap (arch-gaps-3/7, perf-budget-4, config-consistency-3/4).

3. **Single-camera FOV for a two-boundary task.** Lane-keeping needs both boundaries through switchbacks; the architecture provisioned side cameras for exactly this but never finished them. The down-tilt does not hurt HFOV (verifier), so the open question is empirical coverage through a real switchback (arch-gaps-4).

4. **No clearing of stale out-of-FOV semantic cost.** The layer cannot lower a LETHAL cell that has left the camera FOV (`updateWithMax` + decay-skips-write + raytrace-only-in-FOV). Combined with LETHAL lanes this is a navigation-trap class of failure (costmap-integration-1).

5. **Frame/extrinsic placeholder.** The mount xyz+pitch is eyeballed and unverified; because projection is rigid, that error becomes a systematic shift of every camera-marked LETHAL cell, range-amplified by the pitch term (frame-tf-projection-1). No empirical extrinsic check exists in `docs/`.

6. **Control-loop budget.** The semantic layer is genuinely O(N) in cloud points and runs **in-process** with the MPPI optimizer in `controller_server`, serializing on the Costmap2D mutex. The committed REDUCED+8 Hz config holds 13–16 Hz (within the 500 ms actuator timeout), but there is no headroom: the un-applied code patches (x*x not `std::pow`, filter-before-transform, dirty-flag), and ultimately moving the layer to its own lifecycle node, are the real fixes. Any added camera, RViz-on-Jetson, or CPU spike pushes the loop back toward starvation (perf-budget-1/2, arch-gaps-5).

7. **Lanes-as-LETHAL trap risk.** Marking lanes LETHAL (not high-cost-traversable), with `consider_footprint:true`, `collision_cost:1e6`, and sub-inscribed inflation, makes the min-spec 5-ft barrel-in-corridor geometry a sample-starvation stall. And because the active pipeline emits only `lane_white`, lanes cannot be softened without also softening barrels — the perception single-class collapse and the costmap cost policy are coupled and must be fixed together (costmap-integration-4/6, rules-compliance-5).

---

## 5. Prioritized recommendations

### Tier 0 — Must do before the vehicle can legally / meaningfully compete

1. **Build the hardware E-stop loop** (mechanical mushroom + wireless receiver → relay cutting motor power, brown-out-survivable) and feed its state into the safety-light gate and a Teensy GPIO. *(rules-compliance-1, -2)* — Qualification blocker.
2. **Make vision the default for the competition profile** — `competition.launch.py` with `enable_zed_front:=true enable_perception:=true`, plus a "PERCEPTION DISABLED" health WARN. *(arch-gaps-1)*

### Tier 1 — Quick config fixes (one-to-few lines, high value, low risk)

3. **`inflation_radius` → 0.5–0.6 m local / ~0.65 m global**, keep `cost_scaling_factor ~3`. Restores the MPPI gradient. *(costmap-integration-2, rules-compliance-5)*
4. **`movement_time_allowance` → 10–15 s** so recovery runs inside the 45 s BT Timeout / before the 60 s DQ. *(rules-compliance-4)*
5. **Raise the mark gate / graded confidence:** set `samples_to_max_cost` and/or `mark_confidence` to 2–3 *after* emitting graded confidence; interim, raise `samples_to_max_cost` to 2 to kill single-pixel LETHAL. *(cv-pipeline-2)*
6. **Restore the achromatic S ceiling** in `sooner25_upper` (~60–95) and **sync the code defaults** to the YAML. *(cv-pipeline-1, -9)*
7. **`publish_voxel_map: false`** on the local STVL. *(perf-budget-3)*
8. **BT one-liners:** `number_of_retries=4`; `BackUp time_allowance ≈ 5 s`; `ClearCostmapAroundRobot plugins='stvl_layer'` (or reset_distance ~1.0). *(commits-and-docs)*
9. **Document/lock the Teensy `MAX_RPM` as the hardware speed governor**; treat host caps as sub-limits. *(rules-compliance-8)*
10. **Pin and push the kiwicampus fork patches**; bump `avros.repos`. *(commits-and-docs)* — reproducibility.

### Tier 2 — Perception hardening (small code, must re-validate on real asphalt)

11. **Make lanes high-but-traversable (~200) while keeping barrels LETHAL** — requires the perception class split (12) and the YAML class-type split. *(costmap-integration-4/6, rules-compliance-5)*
12. **Add a multi-class / shape pass to `sooner25`:** `connectedComponentsWithStats` → compact circles = `pothole` (3), orange blobs = `barrel_orange` (2), elongated bright = `lane_white` (1). *(cv-pipeline-3, arch-gaps-2/8)*
13. **Add a ground-plane height gate** in `perception_node` (drop pixels with paired cloud z > ~0.15 m) and a **bottom-of-frame body mask**; make the sky ROI horizon-aware for the ramp. *(cv-pipeline-5, -6)*
14. **Temporal vote + median blur + glare guard** in `sooner25`; port an adaptive upper-V floor. *(cv-pipeline-4, -7)*
15. **Calibrate and lock the camera extrinsic** (known-target → costmap-cell check within ~1 cell); record it; remove the TODO. *(frame-tf-projection-1)*

### Tier 3 — Field validation (cannot be substituted by sim today)

16. **Ramp test:** drive a 15% incline; confirm STVL does not false-mark the deck (the `two_d_mode` flat-TF mechanism); add a pitch-gated height band and the `mission_manager` ramp state. *(rules-compliance-6)*
17. **Dark-barrel range test** (2/5/8 m, ≥2 voxels in `[0.4,0.8]m`). *(rules-compliance-7)*
18. **Switchback lane-coverage test** to decide single-vs-multi camera. *(arch-gaps-4)*
19. **Start-zone speed-floor check** (1 mph over first 44 ft / 30 s; avoid early recovery `Wait`/`BackUp`). *(rules-compliance-10)*

### Tier 4 — Larger architectural work (post-quick-fix, real headroom and testability)

20. **Move the semantic costmap to its own lifecycle node** (or apply the x*x / filter-before-transform / dirty-flag patches) so its O(N) work never shares a process/mutex with the MPPI optimizer. *(perf-budget-1)*
21. **Either delete the left/right config + fallback `nav2_params.yaml`, or fully wire/re-derive them** (REDUCED clouds, 8 Hz, correct serials, single-plugin multi-source, `ignored` class type). *(arch-gaps-3/7, config-consistency-1/2/3/4/7)*
22. **Add a sensor/perception health monitor + degraded-mode policy.** *(arch-gaps-5)*
23. **Raise sim parity (camera + lane/barrel/pothole geometry + same BT + real costmap + collision on) or scope-document sim as GPS-routing-only.** *(arch-gaps-6)*
24. **Wire `velocity_smoother` canonically or remove it.** *(config-consistency-6)*

---

## 6. Note on uncertainty

Several items depend on **physical measurements not yet taken** and should be treated as hypotheses until verified on the real vehicle/course: whether the robot's bodywork is actually in-frame at the bottom of the ZED image (cv-pipeline-6), whether a single front camera covers both lane boundaries through a real switchback (arch-gaps-4, medium-confidence), whether the STVL `[0.4,0.8]m` band has a real close-range barrel gap at the actual mount height (rules-compliance-7), and the exact ramp false-marking behavior (rules-compliance-6 — the mechanism is confirmed in config, the on-ramp magnitude is not field-measured). The camera extrinsic error (frame-tf-projection-1) is confirmed *unverified* but its actual magnitude is unknown until calibrated. The "lanes-as-LETHAL trap" geometry is mathematically confirmed at the IGVC minimum-spec corridor; how often the course presents that exact minimum-spec barrel-in-corridor is a course-design unknown.

---

## 7. What looks correct / well-designed

The stack has a solid, validated core that should be protected, not disturbed:

- **The four-topic contract is exact** (topic-contract-1, config-consistency-10): identical stamps on all four outputs, mask/cloud dimensions forced equal, LabelInfo latched RELIABLE+TRANSIENT_LOCAL matching the plugin's subscriber, every class name covered by `danger`+`ignored` (no CRITICAL ERROR spam), topic strings matching exactly. Guard it with a one-frame smoke test before competition.
- **Layer ordering and `combination_method` are textbook** (costmap-integration-5): STVL first, semantic second, inflation last, all `updateWithMax` — neither obstacle source can clobber the other's LETHAL cells. (Optionally pin `combination_method:1` explicitly on the semantic block.)
- **Global costmap correctly excludes vision** and uses shorter STVL decay (costmap-integration-7) — the right call for unaided SBAS GPS, avoiding map-frame lane smear.
- **Frame handling is self-consistent and the VIO lever-arm bug does not apply here** (frame-tf-projection-2/3/4): the plugin transforms by the cloud's own frame_id via a full rigid TF; the mask frame_id is cosmetic; the documented ZED VIO lever-arm issue is EKF-odometry-only and irrelevant to the semantic cloud (don't import a "fix" into this path).
- **The inverted-asphalt premise is correct for the surface** (cv-pipeline-10): the 2026 AutoNav course is asphalt with white tape; thresholding the stable asphalt background and inverting is more robust than forward-thresholding bright paint. The structural gaps (S gate, confidence, temporal, ground-plane) harden a fundamentally appropriate foundation.
- **The no-prior-map and GPS-waypoint constraints are satisfied** (rules-compliance-9): rolling-window reactive costmaps, no StaticLayer/map_server, GPS waypoints via `/fromLL`, 2 m goal tolerance.
- **ZED and STVL load tuning is empirically sound** (perf-budget-5/6): SVGA + REDUCED + `depth_stabilization:1` is the validated GPU/CPU config; the STVL `[0.4,0.8]m` height + 8/10 m range tightening minimizes point count and is *not* the control-loop bottleneck (the semantic layer is). Velodyne `cut_angle=2pi` full-rev + per-packet deskew are genuine root-cause fixes for the costmap flicker/scan-skew, not band-aids.
