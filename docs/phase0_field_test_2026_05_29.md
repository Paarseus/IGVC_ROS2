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

## Test C — Barrel gap × 2 (longer-horizon obstacle reaction)

_pending_

## Test D — Forced recovery (BackUp 0.3 m)

_pending_
