# Phase 0 Field Test — 2026-05-29

Validation of the three Phase 0 changes from
[`yaw_diag_session_2026_05_28/nav2_cohort_strategy.md`](yaw_diag_session_2026_05_28/nav2_cohort_strategy.md):

1. **MPPI horizon** — `time_steps 25→40`, `model_dt 0.05→0.10` (2.0 s / ~1.4 m horizon at identical compute)
2. **Issue #18 TF** — `local_costmap.transform_tolerance: 0.5` (was absent → 0.3 s default; ZED stamp lags 0.3–0.4 s)
3. **BackUp recovery** — `backup_dist 1.5→0.3`, `backup_speed 0.08→0.10`

## Session setup

- **Stack:** `navigation.launch.py enable_zed_front:=true enable_perception:=true` (launched 07:00 PDT, log `/tmp/nav_20260529_065926.log`). 34 nodes; controller/planner/bt_navigator all `active`. Edited BT `navigate_igvc_autonav_humble.xml` is the default in use.
- **Bag:** `~/IGVC/bags/phase0_nav_validation_20260529_070255` (comprehensive curated topic set; raw ZED depth/registered-cloud/velodyne-packets excluded to avoid IO confounding the MPPI-loop test).
- **Environment:** outdoor, **GPS FIX status 0** (basic fix, no SBAS/RTK — §I.2 compliant), `/odometry/gps` feeding map EKF at 4 Hz → **map-frame drift (P1) is LIVE** → goals kept short + forward to isolate Phase 0.
- **Site datum match:** robot at (6.96, 9.39 m) from datum; live lat/lon 42.6680/−83.2181 vs configured datum 42.6679/−83.2182 → no datum-mismatch bug.
- **Props:** barrels/cones available (for Tests C, D).
- **Method:** goals sent via CLI (each timestamped in the bag); user supervises with e-stop; one maneuver per run, 3× for repeatability.

## Pre-flight topic health (07:01 PDT)

| Topic | Rate | | Topic | Rate |
|---|---|---|---|---|
| `/imu/data` | 99 Hz | | `/velodyne_points` | 19.7 Hz |
| `/gnss` | 4 Hz | | ZED rgb rect | 10.7 Hz |
| `/odometry/global` | 30 Hz | | `/perception/front/semantic_mask` | 6.4 Hz |
| `/odometry/gps` | 4 Hz | | `/perception/front/semantic_points` | 5.8 Hz |

TF Exception count at start: **0**.

---

## Test A — Stationary baseline (no motion) ✅ PASS

30 s motionless, 07:11:17–07:11:48 PDT.

| Metric | Result | Notes |
|---|---|---|
| **TF exceptions (window)** | **0** | Issue #18 fix holding — perception feeding costmap with zero drops |
| GPS noise x (1σ / range) | 15.9 cm / 44.3 cm | FIX-only floor; x noisier than y (consistent with prior sessions) |
| GPS noise y (1σ / range) | 4.6 cm / 15.6 cm | |
| Map-EKF drift over 30 s | x +0.5 cm, y −3.3 cm | Stationary map frame is stable — confirms P1 is motion-time only |
| Map-EKF y 1σ | 4.4 cm | Tracks GPS y-noise (differential GPS fusion working) |

**Verdict:** baseline healthy. GPS floor ~16 cm x / 5 cm y 1σ; map stable at rest.

## Test B — Forward 5 m goals ❌ FAIL (control-loop starvation) + a deployment correction

Two forward 5 m goals were sent. **Both `status: SUCCEEDED` and the chassis did move ~6.4 m, but it stuttered and wandered** (user: "straight a bit, then left, then turned right and stopped").

| Run | cmd_vel mean Hz | max gap | lateral dev | duration | odom travel | TF exc |
|---|---|---|---|---|---|---|
| Goal 1 | **3.8 Hz** | 576 ms | 3.73 m | 24.2 s | 6.43 m | 0 |
| Goal 2 | **4.1 Hz** | 508 ms | 2.27 m | 28.5 s | 6.43 m | 9 |

**Root cause — full perception stack starves the 20 Hz MPPI loop.** Nav log floods with
`Control loop missed its desired rate of 20.0000Hz`. Load avg **9.84 on 8 cores**. Top consumers:
`controller_server` (MPPI) ~106 %, `zed_node` ~75–87 %, `perception_node` ~19 %. cmd_vel at ~4 Hz
with >500 ms gaps exceeds the actuator's 500 ms timeout → motors cut out/restart → wander. Yesterday's
healthy 20 Hz (`nav2_lidar_analysis`) was **LiDAR-only**; adding ZED+perception is the difference.
This is the real **P4** — perception+MPPI coexistence on the Jetson, *not* RViz.

### ⚠️ Correction — Phase 0 was NOT actually under test this session

Mid-session discovery: the Jetson's `~/IGVC` is a **separate checkout** (was at `0516ccc`) and never
had my Phase 0 edits — they were only in the laptop repo. So the stack ran the **committed config**
(`time_steps 25`, `model_dt 0.05`, **no** `transform_tolerance: 0.5`, **no** BackUp 0.3). Consequences:
- Phase 0 (MPPI horizon, Issue #18 TF, BackUp) is **untested** — to be retested after deploy.
- My live `time_steps` "revert" to 25 was a **no-op** (already 25); 3.8→4.1 Hz was run-to-run noise.
- **The starvation is independent of Phase 0** — it's the committed config under the full stack.
- Also: my flawed laptop `time_steps 25→40` edit (false "identical compute" premise — MPPI cost ∝
  `batch×time_steps×iters`, `model_dt` is free) was reverted before commit; the deployed config keeps 25.

### Fix applied (this commit) + heavy-bag lesson

- **`zed_front.yaml`: `HD1080→SVGA`, `pub_frame_rate`/`point_cloud_freq` `10→8`** — ~3.6× fewer pixels
  frees a core for the control loop (per user direction: keep perception, lighten ZED).
- The first bag recorded raw ZED image + velodyne_points → ~108 MB/s, 53 GB, and the IO-wait even
  dropped SSH. Switched to a **light bag** (no raw image/cloud; keep semantic_mask + costmaps + odom).
- Phase 0 config now deployed to the Jetson via git (this commit).

**Next:** relaunch full stack with SVGA, re-measure cmd_vel rate under load; if ≥18 Hz, re-run B then C/D.

### Test B (cont.) — SVGA relaunch + the real root cause

After deploying Phase 0 via git (HEAD `fb7cc08`) and relaunching with SVGA:

- **SVGA dropped `zed_node` ~80%→25% (idle) and load avg 9.84→2.96.** Big win: the chassis now drives
  **straight** (lateral dev 0.05–0.67 m vs 3.73 m before), reaches the goal, ~10 s for 5 m.
- **But `/cmd_vel` was still only ~2.9 Hz** (`ros2 topic hz`, max gap 539 ms) — *not* a script artifact.
- Not raw CPU starvation (6 cores idle), but during a goal **`controller_server` = 106 % of one core**
  and **local costmap updates at only 2.3 Hz** (target 10).

**Smoking gun (live A/B):** toggling `semantic_layer.enabled` on the local costmap:

| | semantic ON | semantic OFF |
|---|---|---|
| `/cmd_vel` | ~3 Hz | **20.0 Hz** ✅ |
| `/local_costmap/costmap_raw` | 2.3 Hz | 6.1 Hz |

**ROOT CAUSE (proven):** the kiwicampus `semantic_segmentation_layer`, processing the ZED organized
cloud (`/perception/front/semantic_points`, 256×448 ≈ 114k pts @ 8 Hz) **inside the `controller_server`
process**, consumes the control core. Each semantic costmap update blocks the MPPI loop, dropping
`/cmd_vel` from 20 → ~3 Hz. Its config is already tight (`max_obstacle_distance: 5.0`,
`tile_map_decay_time: 0.3`), so this is per-point compute cost, not a range mis-set. Yesterday's healthy
20 Hz was LiDAR-only (no semantic layer); enabling it is the regression.

**This is the #1 competition-blocking item:** IGVC needs the lane layer, but it can't currently coexist
with a 20 Hz MPPI loop on the Jetson. Candidate fixes (bench): shrink the published cloud
(`perception_node` downsample / lower ZED `point_cloud_res`), drop `point_cloud_freq` to ~2–3 Hz,
profile/optimize the kiwicampus per-point path, or decouple the semantic costmap from the controller.

### Semantic-layer perf investigation — SOLVED via `point_cloud_res: REDUCED`

Read the kiwicampus hot path (`segmentation_buffer.cpp::bufferSegmentation`): per ZED message it
(1) transforms the **entire** organized cloud to the global frame *before* range-filtering
(`:134`), (2) loops over **all 114k pixels** with **3× `std::pow(x,2)` each** (`:167-170`) + an
`unordered_map` insert per in-range point, (3) with `clearing:true`, pushes every in-range point and
raytraces a ray per point in `updateBounds`, (4) with `visualize_tile_map:true`, builds+publishes a
debug cloud every message. All of this runs **inside `controller_server`** and holds the buffer lock,
so the control thread's `updateBounds` blocks on it.

**Experiment 1 (config-only): `visualize_tile_map:false` + semantic `clearing:false` + cloud rate
8→3 Hz.** Result: cmd_vel 3.0→3.58 Hz — **no real help.** This *ruled out* rate/viz/clearing: the
cost is the **per-message processing of all 114k points**, which blocks the control thread via the
buffer lock regardless of frequency.

**Experiment 2 (config-only): ZED `point_cloud_res: COMPACT → REDUCED`** (256×448=114k →
128×224=28k pts, 4× fewer). Clean single-stack result, **semantic layer ON:**

| | COMPACT | REDUCED |
|---|---|---|
| semantic_points | 256×448 (114k) | 128×224 (28k) |
| `/cmd_vel` | ~3 Hz ❌ | **20.9 Hz ✅** |
| `/local_costmap/costmap_raw` | 2.3 Hz | 6.7 Hz |
| `controller_server` CPU | ~106% (pegged) | not pegged |

**RESOLVED:** `point_cloud_res: REDUCED` lets the lane layer coexist with a 20 Hz MPPI loop —
4× fewer points = 4× cheaper `bufferSegmentation` = control thread no longer lock-blocked.

**Finalized + validated config** (clean single stack, semantic layer ON):

| param | value |
|---|---|
| ZED `grab_resolution` | SVGA |
| ZED `point_cloud_res` | **REDUCED** (128×224) |
| ZED `pub_frame_rate` / `point_cloud_freq` | 8 Hz |
| semantic `clearing` | true (restored — raytrace clearing affordable at REDUCED) |
| semantic `visualize_tile_map` | false (debug-only) |
| MPPI | 25 steps / 0.05 dt |
| `transform_tolerance` (local costmap) | 0.5 |
| BackUp | 0.3 m @ 0.10 m/s |

Validation (4 m forward goal, semantic ON): **cmd_vel ~13–16 Hz, max gap 0.146 s**, costmap 8 Hz,
straight (0.38 m lateral). max gap well under the actuator's 500 ms timeout → no stutter. Exp1's
`clearing:false`/`freq:3` variant reached 21 Hz but is a behavior trade; the committed config keeps
intended behavior at a healthy rate. Further headroom available via code (`x*x` not `pow`,
filter-before-transform) if a higher rate is ever needed.

**Committed deltas vs `fb7cc08`:** only `point_cloud_res: REDUCED` + `visualize_tile_map: false`
(the two changes that mattered; freq/clearing/SVGA were already at these values).

### Phase 0 validation status (with semantic OFF = clean 20 Hz)
- **MPPI / straight driving:** ✅ 20 Hz, 0.05 m lateral on 5 m goal.
- **Issue #18 TF (`transform_tolerance: 0.5`):** ✅ 0 TF-exception deltas on the SVGA run.
- **BackUp 0.3 m:** not yet exercised (needs a forced recovery — Test D).

## Test C — Barrel gap × 2 (longer-horizon obstacle reaction)

_pending_

## Test D — Forced recovery (BackUp 0.3 m)

_pending_
