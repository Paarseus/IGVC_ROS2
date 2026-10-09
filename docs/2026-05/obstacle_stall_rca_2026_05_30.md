# IGVC AutoNav — Obstacle-Avoidance STALL Root-Cause Analysis (2026-05-30)

**Subject:** Robot freezes/stalls in front of a barrel instead of driving around it.
**Method:** Multi-agent investigation (6 subsystems) + adversarial verification, grounded in two offline rosbags and Nav2 Humble source. This document is the definitive synthesis.

**Bags analyzed (Jetson `~/IGVC/bags/`):**
- **PRIMARY** `goal8m_t2_20260530_103204` — recorded AFTER all 4 session fixes (cloud frame `velodyne`, inflation 1.0 local, voxel_decay 3, deskew off). Robot froze ~1.78 m short of a dead-center barrel, was nudged loose by recovery (~t=44), partially maneuvered, froze AGAIN at (5.73,−1.32), jittered ~40 s, ended (5.74,−1.47) — 3.2 m short of the (7.96,0.80) goal.
- **COMPARISON** `goal8m_20260530_101148` — recorded BEFORE the deskew-off/voxel_decay fixes (cloud frame `odom`). Drove to 1.73 m, froze dead-center, never recovered, aborted at the 45 s BT Timeout.

---

## 1. RANKED VERDICT

The stall is **NOT** a single-subsystem failure. It is a **COSTMAP↔CONTROLLER coupling**: the local costmap geometry removes MPPI's escape samples, and MPPI's footprint-aware CostCritic converts that into a zero-velocity command. The LiDAR, the global planner, and localization are faithful and exonerated as *root* causes.

| Rank | Subsystem | Role | Confidence | One-line |
|------|-----------|------|-----------|----------|
| **1** | **LOCAL COSTMAP — oversized 0.914 m circular footprint + collapsed inflation gradient** | **PRIMARY ROOT CAUSE** | **High** | 16-pt circle r=0.914 (inscribed 0.896) with inflation_radius=1.0 → a 0.086 m gradient cliff and a ~1.9 m no-go halo. Robot parks ~1.0 m short in BOTH bags. |
| **2** | **CONTROLLER (MPPI) — footprint-aware CostCritic + short horizon** | **PRIMARY PROXIMATE MECHANISM** (co-equal, coupled to #1) | **High** | `consider_footprint:true` + `collision_cost:1e6` + the near-binary 253/0 cost field → near-uniform softmax → averaged ~0 command. `time_steps=25` (1.25 s / ~0.9 m) prevents early commitment to a detour. |
| **3** | **LiDAR near-field deadzone + STVL persistence** | **Strong contributor** | **High** | `min_range=0.7 m` + height band [0.4,0.8] leaves the dead-ahead barrel un-reconfirmable once the robot is close; STVL persists the stale marks the footprint then collides with. |
| **4** | **RECOVERY TIMING (BT)** | **Contributor (prolongs, doesn't originate)** | **High** | MPPI rarely returns FAILURE, so recovery is rarely/late invoked; `movement_time_allowance=120 s` can't fire in a ~120 s run; BackUp/DriveOnHeading are collision-blocked. The only thing that breaks the robot loose is the zero-motion costmap clear. |
| **5** | **GLOBAL COSTMAP / NAVFN PLANNER** | **Exonerated as root; one real defect (global inflation cliff)** | **High** | Navfn never failed; the 12 m loop is the correct Dijkstra route around an over-marked lethal ring. Global `inflation_radius=0.65 < inscribed 0.896` → zero-gradient solid discs that box the robot, but this warps the *plan*, not the *freeze*. |
| **6** | **LOCALIZATION / ODOM / TF** | **Exonerated** | **High** | odom→base smooth (25–30 Hz, 0 teleports), 4 yaw sources agree within 5°, GPS-map drift can't reach control (all consumers in `global_frame: odom`). |

**Bottom line:** Fix the **footprint first** (right-size it), then verify the **inflation gradient** and **MPPI horizon**. That triplet is the load-bearing fix. The LiDAR deadzone is a latent hazard that the footprint fix also mitigates.

---

## 2. THE FAILURE CHAIN, FRAME BY FRAME

### 2.1 LiDAR → STVL → costmap marking (HEALTHY)

The sensing half of the chain worked correctly throughout the primary run:

- `/velodyne_points` at a steady **10.0 Hz** (dt min/mean/max 0.081/0.100/0.114 s), `frame_id='velodyne'` (deskew-off confirmed), ~17.9k valid points/cloud, **347° mean azimuth coverage** (full revolutions — the broadcast/half-wedge flicker is genuinely root-fixed). [LiDAR finding]
- The velodyne sits 0.7146 m above ground (URDF `xsens_height` 0.5556 + 0.159), so the odom-frame band [0.4,0.8] maps to a sensor-z band that the barrel mid-slice fills.
- At the first stall the barrel produced **847 in-band points** and 20–38 near-column voxels passing `mark_threshold` (max 56–67 pts/voxel) — **massively over-detected** vs the threshold of >=2 returns/voxel. The second/final stalls were extended 2.0–2.9 m structures (491–1026 in-band points). [LiDAR finding, verified]

**The marking is real, not a phantom or a flicker artifact.** Where the robot froze, STVL held a *stable* cluster of LETHAL(254) cells whose nearest edge sat at a rock-stable ~0.99–1.07 m from base_link for the entire multi-second freeze, in **both** bags despite their different clearing config — proving the freeze is geometric, not a clearing/decay/flicker issue. [Local-costmap + verifier reproduction]

### 2.2 InflationLayer → cost field (COLLAPSED GRADIENT — defect #1)

The 16-pt circular footprint has:
- inscribed_radius = 0.914 · cos(π/16) = **0.896 m**
- circumscribed_radius = **0.914 m**

With `inflation_radius=1.0` and `cost_scaling_factor=3.0`, the exponential-decay band exists only for `inscribed < d <= inflation_radius`, i.e. **1.0 − 0.896 = 0.104 m wide** (one verifier measured 0.086 m using a slightly different inscribed convention; both are sub-to-one-cell at 0.2 m resolution). The cost field is therefore a **near-binary 253/0 plateau**: cost ≈ 253 for d ≤ 0.896, dropping to 0 by d = 1.0, with only a single intermediate ring (~190) in between. Robot-centric grids confirm ~53–57 cells at 253 vs only 6–9 cells of intermediate cost within 1.5 m. `cost_scaling_factor=3.0` is effectively inert. [Local-costmap finding; verifier reproduced cost 253@0.90 → 0@1.05]

This is the **same mechanism** the prior `fresh_audit_2026_05_29.md` finding **N2** identified for the then-current `inflation_radius=0.3 < inscribed 0.37` config — but the *current* config re-introduced it in a different (and worse) way: inflation was correctly restored to 1.0, but the footprint was enlarged from the rectangle (inscribed 0.37) to the r=0.914 circle (inscribed ≈ circumscribed), so `inflation_radius − inscribed` collapsed back to ~0.1 m. **N2's prescription (raise inflation above inscribed) was satisfied numerically yet defeated by the footprint change.**

### 2.3 Planner → /plan (HEALTHY; symptom of an over-marked global costmap)

Navfn ran cleanly the entire run: **0 empty plans**, ~0.94 Hz (gated by the BT `RateController hz=1.0`, not `expected_planner_frequency=5.0`). The dramatic **11–12 m lateral loop** was the *correct* Dijkstra route around a near-closed lethal ring (the robot was boxed in 30–35 of 36 angular sectors). [Global-planner finding, verified]

The ring is amplified by **defect #2: the GLOBAL inflation cliff** — `inflation_radius=0.65 < inscribed 0.896`, so the exp-decay band is *empty* (0 gradient cells measured) and every lethal cell paints a solid 0.896 m disc, merging sparse 0.2–0.4 m-spaced returns into a continuous wall. This is a genuine costmap misconfiguration (Nav2 `inflation_layer.cpp` logs an ERROR when `inflation_radius < inscribed`), but it warps the *plan*, not the *freeze*.

**Decisive ordering evidence:** in the primary bag the robot froze at t≈10.4 s (vx 0.06, ~21 lethal cells already inside the inscribed footprint) **while the global plan was still the short straight ~6 m path**. The 12 m loop first appeared ~3 s LATER (t≈13.7). The detour is a *consequence* of the costmap inflating a ring around the already-stationary robot, not the cause of the stall. [Global-planner verifier `gp_ordering.py`]

### 2.4 Controller (MPPI) → /cmd_vel (PROXIMATE FREEZE)

MPPI was **not** starved: `/cmd_vel` ran a clean **20.00 Hz, max gap 0.057 s (primary) / 0.065 s (comparison), zero gaps >75 ms** in both bags. This rules out the 2026-05-21 RViz-on-Jetson loop-starvation mode. The freeze is a controller **decision**, and `/wheel_odom` confirms the chassis was genuinely stationary (|vx|mean 0.015 m/s). [MPPI + recovery findings]

The mechanism (Nav2 Humble `cost_critic.cpp` + `optimizer.cpp`, verified by WebFetch):
- With `consider_footprint:true`, **LETHAL(254)** on the swept footprint → `trajectory_collide=true` → that trajectory gets `collision_cost = 1e6`. **INSCRIBED_INFLATED(253)** is *not* a collision under `consider_footprint` (it returns false), but it adds a flat `critical_cost=300` per pose with **no gradient**.
- When the obstacle sits at/just inside the footprint perimeter, nearly every sampled rollout either sweeps a 254 cell (→1e6) or rides the flat 253 plateau (→equal high cost). The softmax over near-uniform high costs collapses to **uniform weights → an averaged near-zero command**, rather than throwing immediately. It only throws "Optimizer fail to compute path" after `retry_attempt_limit` — which is why the robot **jitters at vx≈0 for tens of seconds** instead of failing fast.
- `time_steps=25 × model_dt=0.05 = 1.25 s` horizon ≈ **0.9 m at vx_max 0.7** (≈0.5 m at the 0.4 m/s creep). At the 2026-05-21 *working* test the default `time_steps=56` (2.8 s, ~1.96 m) gave MPPI enough reach to score a go-around lower than straight at standoff distance. The short horizon means MPPI **cannot commit to a detour before creeping into the inflated halo**, so it creeps to footprint contact and then locks.

**On the one genuine inter-verifier disagreement (253 vs 254 as the binding cell):** the steady-state nearest *true* LETHAL(254) cell measured ~0.99–1.07 m from base_link — i.e. ~0.08–0.15 m *outside* the 0.914 m perimeter — which means the 253 inflation plateau (not a hard 254 footprint collision) is the dominant steady-state cost driver at the freeze pose. However, the obstacle is **dead-ahead** and Nav2 rasterizes the *full* polygon edges via `LineIterator`, so forward-leaning rollouts do sweep 254 cells (perimeter-sweep tests found only 0–2 of 45 rollouts 254-free, both stay-still/reverse). **Both descriptions converge on the same outcome:** essentially every trajectory that makes forward/turning progress is penalized either by a 254 collision or by the flat 253 plateau, leaving no improving direction → softmax collapse → freeze. The distinction does not change the verdict or the fix; it only refines *which* cost term binds (253 plateau dominates the standstill, 254 perimeter sweeps dominate the forward rollouts). **This is flagged as the residual modeling uncertainty.**

**Counterfactual (the load-bearing test):** re-running the perimeter-sweep on the *recorded* costmaps at the freeze poses with different footprint radii: **r=0.914 → 0 forward escapes** (boxed in); **r=0.835 (true forward circumscribed) → 12 forward escapes**; **r=0.55 → 20–22 forward escapes**. A footprint reduction flips the freeze open. This proves footprint size against the lethal field is the **binding constraint**. [Local-costmap verifier `foot_counterfactual.py`]

### 2.5 Actuator (FAITHFUL)

`/wheel_odom` mirrors `/cmd_vel`: the chassis stopped because MPPI commanded ~0, not because of any actuator/serial/heading-hold artifact. The #19/#20 phantom wheel_odom bites only under e-stop, which was not active during the drive. [Localization finding]

---

## 3. WHY THE SECOND STALL, AND WHY THE 12 m DETOUR

**The 12 m global detour** (primary bag, t≈13.7–44 s): the global costmap inflation cliff (defect #2) turned ~315 sparse LiDAR returns into ~1900 solid 253-cost cells forming a near-closed ring around the stationary robot; Navfn correctly routed Dijkstra around the only opening, producing a ~25 m / 11–12 m-lateral loop. It appeared *after* the freeze, is a faithful symptom, and is fixed by raising global inflation above inscribed + reducing upstream over-marking — not by touching the planner.

**The second stall** at (5.73,−1.32): after recovery's costmap-clear let MPPI re-plan and break loose, the robot curved into a genuinely tighter pocket — the LiDAR saw extended 2.0–2.9 m structures wrapping the front-left arc (front/left blocked, rear-right open per escape-sector analysis). With the oversized 0.914 m footprint, even a navigable ~2 m gap becomes un-samplable: turning toward the open rear-right or toward the valid +70° global plan sweeps the footprint perimeter through the front-left lethal cells → 254 → still collision. MPPI jittered (|wz| to 1.09) without committing for ~40 s until a fresh costmap clear + Navfn detour at t=105.09 fired a real escape (vx −0.41, wz −1.09). It then re-approached and froze a third time. **The second stall is part-environment (a real tight pocket) and part-footprint (the oversized circle makes the pocket un-navigable).**

---

## 4. WHAT THE SESSION FIXES CORRECTED vs WHAT REMAINS

| Session fix | Effect (verified) | Remaining gap |
|---|---|---|
| Sensor broadcast → unicast (full 360° clouds) | **Effective.** Primary bag shows steady 10 Hz, 347° coverage, no half-wedge, no flicker. | None — this regression is fixed. |
| `inflation_radius` 0.3 → 1.0 local | **Partially effective.** Restored a halo, but the *footprint* was simultaneously enlarged to r=0.914, so the gradient band stayed ~0.1 m (cliff persists). | **Footprint must be right-sized** for inflation 1.0 to produce a real gradient. |
| Deskew off (`fixed_frame` odom → empty) → clears from robot | **Effective for clearing.** Marks now anchor/clear at the robot, not the odom origin. The comparison bag (pre-fix) shows the odom-frame failure mode (Navfn could never find a detour). | The primary bag still froze — clearing was never the freeze cause. |
| `voxel_decay` 8 → 3 | **Effective.** Faster out-of-view clearing; no decay-induced obstacle loss observed. | None for this failure. |

**Net:** the 4 fixes correctly addressed *flicker, frame, and clearing*. They did **not** touch the *footprint geometry* or the *MPPI horizon* — which are the actual freeze drivers. The primary bag (post-fix) still froze, confirming the residual root cause is the footprint/inflation/horizon triplet, not sensing.

---

## 5. RECONCILIATION WITH PRIOR AUDITS

**`docs/fresh_audit_2026_05_29.md`:**
- **N2 (inflation < inscribed collapses MPPI gradient): AGREE on mechanism, SUPERSEDE on the numbers.** N2 was written for `inflation_radius=0.3 / inscribed 0.37 / rectangle footprint`. The current config restored inflation to 1.0 but enlarged the footprint to r=0.914 (inscribed 0.896), re-creating the cliff at a different operating point. The current global costmap (`inflation 0.65 < inscribed 0.896`) is *still* in the N2 regime — N2 is live for the global, stale-by-number for the local.
- **N1/N3 (model_dt rule, "identical compute" false): AGREE and now applied.** `model_dt` is back to 0.05 and `time_steps` to 25 (the 40-step bump was reverted because it dropped cmd_vel to 3.8 Hz). The legal horizon lever remains raising `time_steps` (and re-measuring Hz) or `model_dt × time_steps` within the 20 Hz budget.
- **N6/N10/N14/N15/N18 (BT recovery + progress checker): AGREE.** DriveOnHeading idx-3 unreachable with retries=3; `movement_time_allowance=120 s` can't fire in a ~120 s run; BackUp `time_allowance=25 s` too long. The recovery-timing finding confirms these from the bag (recovery only ever helped via the zero-motion clear).

**`docs/cv_costmap_deep_analysis_2026_05_29.md`:**
- Its independent prediction of the **253-cliff + barrel-in-corridor sample starvation** is **CONFIRMED** by both bags. It analyzed the hypothesized 0.3 m engulf state; the bag shows the robot actually parks at ~1.0 m (the inflation halo holds it out), but the *mechanism* (footprint-aware CostCritic on a flat high-cost field) is exactly as predicted.

**`docs/lidar_obstacle_avoidance_test_2026_05_21.md` (last known-working):**
- Used `inflation_radius=1.0` local **with the rectangle footprint (inscribed 0.37, circumscribed 0.622)** → a real ~0.6 m gradient band and a ~1.6 m halo — AND `time_steps=56`. The 2026-05-29 switch to the r=0.914 circle (to stop corner-clipping) collapsed the band to ~0.1 m and grew the halo to ~1.9 m, and the 2026-05-28 `time_steps 56→25` halved the horizon. **These two changes are the regression from the working config.** The working test fired 7 recoveries and replanned through repeated blocking; the current config freezes because the footprint makes recovery motions collision-blocked and the horizon makes MPPI creep into the trap.

---

## 6. WORKING-HYPOTHESIS SCORECARD

| Hypothesis claim | Verdict |
|---|---|
| MPPI horizon too short (25 = 1.25 s) prevents committing to a detour | **CONFIRMED** as a contributor (rank 2) — but secondary to footprint geometry. |
| Robot creeps to ~0.3 m, barrel sits inside footprint, all trajectories collide → ~0 | **PARTIALLY REFUTED on distance.** The robot does NOT reach 0.3 m — the inflation halo holds it at ~1.0 m. But the *outcome* (every progress trajectory penalized → softmax collapse → ~0) is correct. |
| 0.914 m circular footprint is oversized; min_range 0.7 m deadzone sits inside it | **CONFIRMED.** Oversized footprint is the binding constraint (counterfactual). The 0.7 m deadzone is a real latent hazard (the dead-ahead barrel becomes un-reconfirmable at close range) that did NOT fire here only because MPPI stopped the robot at >0.84 m. |
| 05-21 working test used time_steps 56 + rectangle footprint (inscribed 0.37) | **CONFIRMED** by git + the 05-21 doc. This is the regression baseline. |

**What the hypothesis missed:** (a) the inflation *cliff* (not just footprint size) flattens the cost field so MPPI has no early-standoff gradient; (b) the global inflation cliff (0.65 < inscribed) explains the 12 m loop; (c) recovery timing (MPPI rarely returns FAILURE; 120 s progress allowance) explains why the robot sits for 30–100 s instead of recovering quickly; (d) the freeze is the *local* costmap, not the global plan (the global costmap was empty at the first-stall freeze in one verifier's check).

---

## 7. PRIORITIZED FIX PLAN

Order matters: footprint first (it gates everything else), then inflation gradient, then horizon, then recovery, then global. A/B the footprint+inflation change first — it is the single highest-leverage, bench-safe edit.

### FIX 1 (PRIMARY, bench-safe) — Right-size the footprint
**File:** `nav2_params_humble.yaml:265-266` (local) and `:515-516` (global).
Replace the r=0.914 circle with a footprint reflecting the real chassis. Two options:
- **Preferred — rectangle** matching the 0.826 × 0.680 m tread envelope, e.g.
  `footprint: "[[0.55,0.36],[0.55,-0.36],[-0.28,-0.36],[-0.28,0.36]]"`
  (base_link is the IMU, offset ~0.31 m behind the chassis centroid per URDF `avros.urdf.xacro:43,50` — so the box is asymmetric front/back; tune the x-extents to the measured front/rear reach). This restores inscribed ≈ 0.36 m, a real ~0.64 m gradient band at inflation 1.0, and a ~1.35 m halo — close to the 05-21 working geometry.
- **Simpler — smaller circle** r ≈ 0.55 m (`robot_radius: 0.55` + matching 16-pt polygon). inscribed ≈ 0.54 → ~0.46 m gradient band at inflation 1.0.

**Caveat:** `consider_footprint:true` requires an explicit polygon (an empty footprint + robot_radius aborts bringup with "no robot footprint provided"). Keep the explicit polygon.
**Expected:** at the recorded 0.99 m obstacle distance, in-footprint lethal count drops to ~0; perimeter-sweep escape fraction rises from 0/45 to 27–35/45. **Needs field re-test** to confirm the smaller footprint doesn't clip corners (the original reason it was enlarged) — but the counterfactual strongly predicts the freeze opens.

### FIX 2 (bench-safe, pair with FIX 1) — Restore the inflation gradient
**File:** `nav2_params_humble.yaml:481` (local), `:620` (global).
- With FIX 1's inscribed ≈ 0.36–0.54 m, `inflation_radius: 1.0` local now yields a real multi-cell gradient — keep it.
- **Global `inflation_radius: 0.65` is BELOW even the reduced inscribed only if you pick r=0.55** (inscribed 0.54 < 0.65 → OK) — but verify it exceeds the new inscribed; if using the rectangle (inscribed 0.36), 0.65 is fine. Raise to ≥ inscribed + margin (e.g. 0.8–1.0) and confirm the launch log no longer prints the `inflation_radius < inscribed` ERROR.
- Optionally lower `cost_scaling_factor` 3.0 → ~2.0 once the band is wide, to spread the gradient over more cells (Macenski MPPI guidance).

### FIX 3 (needs field Hz re-test) — Lengthen the MPPI horizon
**File:** `nav2_params_humble.yaml:81,89`.
- Raise `time_steps: 25 → 40–56` (2.0–2.8 s, ~1.4–2.0 m reach). Keep `model_dt: 0.05` (do NOT set 0.10 — violates the "lower but not larger" rule).
- Because cost ∝ `batch_size × time_steps`, **re-verify `ros2 topic hz /cmd_vel ≥ 18`** under field load. The 2026-05-29 bump to 40 dropped it to 3.8 Hz *with the full ZED+perception stack running*; LiDAR-only has more headroom, but measure. If Hz dips, this is the trade-off point — consider `time_steps 40` with `batch_size 500`, or `time_steps 56 / batch 750`.

### FIX 4 (bench-safe) — Make recovery trip promptly
**File:** `nav2_params_humble.yaml:63-66` and `navigate_igvc_autonav_humble.xml`.
- Drop `progress_checker.movement_time_allowance: 120.0 → 8.0` s (keep `required_movement_radius ~0.2–0.3`). Then ~8 s of no progress throws "Failed to make progress", FollowPath returns FAILURE, and the recovery sub-tree fires — instead of MPPI idling at ~0 for 30–100 s. **Highest-leverage recovery fix.**
- `RecoveryNode number_of_retries 3 → 4` so DriveOnHeading/CrawlForward (RoundRobin idx 3) becomes reachable (audit N14).
- `BackUp time_allowance 25 → ~5 s` (audit N15). With the smaller footprint (FIX 1), BackUp/DriveOnHeading are no longer collision-blocked on cycle 0, so they can actually execute.

### FIX 5 (bench-safe) — Tame the upstream over-marking that builds the ring
**File:** `nav2_params_humble.yaml:557,538` (global STVL).
- Lower global `obstacle_range 10 → 8` and consider global `voxel_decay 5 → 3` (match local) so stationary distant marks evaporate and the global ring stops forming. This reduces the 12 m loops feeding MPPI a wildly deviating reference. (Coordinate with the global inflation fix in FIX 2.)

### Do NOT
- Do **not** chase the LiDAR/STVL marking params (band, mark_threshold, range) for this stall — detection is saturated and correct.
- Do **not** touch localization/odom/TF — exonerated.
- Do **not** lower lane cost or treat lanes as traversable — out of scope (LiDAR-only here) and against the documented stall-safe-over-line-touch-DQ policy.
- Do **not** revert nav to the map frame — the odom-frame control correctly isolates the 4.65 m GPS-map drift.

### A/B test sequence
1. **A/B #1:** FIX 1 + FIX 2 only (footprint + inflation), LiDAR-only, same 8 m goal. Expect the robot to drive around the barrel without freezing. This is the decisive test.
2. **A/B #2:** add FIX 3 (horizon), confirm cmd_vel Hz ≥ 18 and that the go-around commitment is earlier/smoother.
3. **A/B #3:** add FIX 4 (recovery), confirm that if it *does* stall, it recovers within ~10 s instead of 45 s.

---

## 8. EVIDENCE-CONFIDENCE LEDGER

**Proven from the bags (high confidence):**
- cmd_vel 20 Hz, not starved (both bags). Robot genuinely stationary (wheel_odom).
- LiDAR 10 Hz, full 360°, barrel over-detected; marks stable at ~1.0 m at the freeze, in both bags.
- Robot parks ~1.0 m short (NOT 0.3 m); same standoff in both bags despite different clearing config → geometric freeze.
- Navfn never failed; 12 m loop is a route around a lethal ring; freeze precedes the loop by ~3 s.
- Localization smooth; 4 yaw sources agree within 5°; GPS-map drift isolated by `global_frame: odom`.
- Footprint counterfactual: r=0.914 → 0 forward escapes; r≤0.835 → escapes reappear.

**Hypothesized / modeled (medium confidence, flagged):**
- The exact binding cost term at the standstill (253 inflation plateau vs 254 perimeter sweep). Both routes produce the same softmax collapse; the perimeter-sweep escape count uses a `LineIterator`/Bresenham approximation of `footprintCostAtPose` that may differ by ~1 cell from the live `FootprintCollisionChecker`. The 0-vs-27 escape gap dwarfs any sampling error, so the *conclusion* (footprint is binding) is robust even though the *trigger cell* is uncertain.
- The smaller footprint will not re-introduce corner-clipping — predicted by the counterfactual but **requires a field re-test** to confirm against real barrels.

---

## 9. REFERENCES

- Bags: `~/IGVC/bags/goal8m_t2_20260530_103204` (primary), `~/IGVC/bags/goal8m_20260530_101148` (comparison). Analysis scripts on Jetson `/tmp/` (lidar_analyze.py, lc_verify.py, lc_footprint2.py, foot_counterfactual.py, gp_ordering.py, mppi_analysis.py, recovery_timing.py, loc_jumps.py, etc.).
- `src/avros_bringup/config/nav2_params_humble.yaml`: `:81` time_steps 25, `:89` model_dt 0.05, `:93` batch_size 500, `:162-170` CostCritic (consider_footprint true, collision_cost 1e6, critical_cost 300, weight 6), `:265-266`/`:515-516` footprint 16-pt circle r=0.914 (inscribed 0.896), `:478-481` local inflation 1.0 / cost_scaling 3.0, `:617-620` global inflation 0.65, `:283-341` local STVL, `:535-563` global STVL.
- `src/avros_bringup/config/velodyne.yaml`: min_range 0.7, fixed_frame empty (deskew off), cut_angle 2π.
- `src/avros_bringup/urdf/avros.urdf.xacro:22,25-38,43,50`: chassis 0.743×0.680, tread env 0.826, IMU(base_link) offset 0.3143 m behind centroid.
- `src/avros_bringup/config/navigate_igvc_autonav_humble.xml`: Timeout 45 s, RecoveryNode retries=3, RoundRobin Clear/Wait/BackUp/DriveOnHeading, BackUp time_allowance 25.
- `src/avros_bringup/config/ekf.yaml`, `navsat.yaml`: dual-EKF, 5σ IMU gates, datum [lat,lon,heading_rad], `global_frame: odom` consumers.
- Nav2 Humble source: `nav2_mppi_controller/src/critics/cost_critic.cpp` (LETHAL 254 → collision_cost 1e6; INSCRIBED_INFLATED 253 → non-collision under consider_footprint + flat critical_cost; footprintCostAtPose), `optimizer.cpp` (uniform softmax → averaged ~0 when all costs near-equal; throws only after retry_attempt_limit), `nav2_costmap_2d/plugins/inflation_layer.cpp` (ERROR when inflation_radius < inscribed; 253·exp(−cost_scaling·(d−inscribed))), `nav2_behaviors/plugins/drive_on_heading.hpp` (isCollisionFree at cycle 0 = current pose), `nav2_behavior_tree/recovery_node.cpp` + `round_robin_node.cpp`.
- Nav2 docs: MPPI README (horizon = time_steps × model_dt; "model_dt lower but not larger"; CostCritic), `docs.nav2.org` inflation-layer & navfn pages.
- `spatio_temporal_voxel_layer` (SteveMacenski): mark_threshold, voxel_decay, decay_acceleration semantics.
- VLP-16 / `ros-humble-velodyne` driver: 16-beam 2° ring spacing, min_range.
- Mandow et al. 2007 IROS (skid-steer kinematics) — context for the tracked chassis ω delivery.
- In-repo: `docs/fresh_audit_2026_05_29.md` (N1–N18), `docs/cv_costmap_deep_analysis_2026_05_29.md`, `docs/lidar_obstacle_avoidance_test_2026_05_21.md` (last known-working).
