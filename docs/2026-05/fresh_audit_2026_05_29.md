# IGVC_ROS2 — Fresh Multi-Agent Audit & Superseding Multi-Phase Strategy (2026-05-29)

> Grounded against the live working tree (uncommitted Phase-0 edits included). Line numbers verified by direct read; where docs/commits quoted stale lines, the corrected value is given. This **supersedes and merges** `docs/yaw_diag_session_2026_05_28/nav2_cohort_strategy.md`.

## 1. TL;DR — Top issues

| # | Issue | Subsystem | Severity | NEW? | One-line status |
|---|-------|-----------|----------|------|-----------------|
| 1 | GPS fused with `odom0_differential: true` — contradicts official RL docs (random-walk → 4.1× path inflation) | localization | **HIGH** | known | Phase-1 fix correct; **must pair with GPS covariance floor or it snaps harder** |
| 2 | Rejection-threshold comments are unit-wrong: `13.8` is ~13.8σ Mahalanobis (gate effectively **OFF**), not "6σ²" | localization | **HIGH** | **NEW** | Entire map-EKF tuning narrative done in wrong unit |
| 3 | `model_dt: 0.10` > control period `0.05` — **violates** the Nav2 MPPI hard rule ("lower but not larger") | controller | **HIGH** | **NEW** | Phase-0 edit endorsed by strategy/debug docs without catching the rule |
| 4 | `inflation_radius: 0.3` < inscribed radius `0.37` → Nav2 ERROR + MPPI gradient **collapses to a flat plateau** | costmap | **HIGH** | **NEW** | cost_scaling_factor 3.0 is inert; cost_weight 6.0 cannot restore it |
| 5 | `transform_tolerance: 0.5` **cannot** fix Issue #18 (a *past*-extrapolation error); tolerance only extends *future* | perception/TF | **HIGH** | **NEW** | Band-aid addresses wrong failure mode |
| 6 | RoundRobin recovery: `DriveOnHeading` (CrawlForward) is **mathematically unreachable** with retries=3 | BT | MED | **NEW** | 4 children, 3 retries → idx 0,1,2 only |
| 7 | `velocity_smoother` is in the lifecycle but **DANGLING** — actuator reads raw `/cmd_vel`, smoother output read by nobody | launch | MED | **NEW** | Latent trap; every accel limit it configures is inert |
| 8 | `mission_manager` hard-`SystemExit(1)` on `/fromLL` + binds `/odometry/global` — both vanish under Phase 2 (map≡odom) | localization | **HIGH** | known | Launch-blocking prerequisite, not optional cleanup |
| 9 | map≡odom leaves **no absolute heading anchor**; the lone Xsens yaw is the very signal the 5σ gate discards on stuck-bias | localization | HIGH (uncertain) | **NEW** | Pre-existing fragility, not introduced by Phase 2; startup yaw-convergence gate + GPS-course check absent |
| 10 | `BackUp time_allowance=25` s (stale from 1.5 m backup) blows the 24 s / 45 s recovery budget | BT | MED | **NEW** | One stalled backup eats >half the outer Timeout |
| 11 | `ClearCostmapAroundRobot reset_distance=3.0` (no plugin filter) wipes the **painted-lane** cells it was chosen to preserve | BT | MED | **NEW** | Localized re-creation of the lane-wipe race |

---

## 2. Verdict on the existing cohort strategy

Per-recommendation, using `strategy_check`:

| Strategy item | Verdict | Corrected truth |
|---|---|---|
| **P1 — `odom0_differential: true → false`** (ekf.yaml:186) | **CONFIRMED** | Doc-*mandated*, not hygiene. RL integrating-GPS doc: "enabling differential integration defeats the purpose"; navsat_transform doc *also* says `odomN_differential` must be `false` for its output; canonical `dual_ekf_navsat_params.yaml` uses `false`. **Co-requisite the strategy flags but cannot yet execute: there is NO covariance-floor knob in `navsat.yaml`/`xsens.yaml`.** navsat copies raw NavSatFix covariance through unfloored; u-blox SBAS reports optimistic (<1 m) covariance vs 2–5 m true error → absolute mode snaps *harder* than differential unless you floor R (~5 m σ) at the NavSatFix source. **Do not flip without the floor.** |
| **Phase 2 — map ≡ odom (delete map EKF + navsat, identity `map→odom` @50 Hz on `/tf`)** | **CONFIRMED (with caveat)** | Architecturally valid; `/tf` not `/tf_static` is correct (static would age out of `transform_tolerance`). **Caveat the strategy under-weights:** there is then *no* absolute heading correction at all, and every odom-frame yaw source is drift-prone (Xsens 150 s warmup #15, ZED yaw-only-differential, wheel-skid). The "GPS-vs-odom heading sanity check" the doc *mentions* is the load-bearing safeguard and is **unspecified** — promote it from a guard to a **required Phase-2 deliverable**. The odom "0.04 %/24.86 m" figure is one benign session, contaminated by #17 and #16 — not a safe ceiling. |
| **Phase 0 — `local_costmap.transform_tolerance: 0.5`** | **CONFIRMED but STALE bookkeeping + INEFFECTIVE for #18** | Already applied (nav2_params_humble.yaml:250); strategy still lists it "pending." **More important: it cannot fix Issue #18** — the recorded error (`SESSION_FINAL.md:138-140`) is *past*-extrapolation (req 845.292 < earliest 845.638); tolerance only extends the lookup toward the *future* (`MessageFilter` adds tolerance to stamp; `Buffer::transform` timeout only blocks for future TF). Harmless to keep, will not stop the abort. Real fix = stamp source or `TimePointZero` fallback. |
| **Phase 0 — MPPI `model_dt 0.05→0.10`, `time_steps 25→40`** | **PARTIALLY / REFUTED rationale** | Values applied (lines 81/87/90). **Three errors in the justification:** (a) `model_dt: 0.10` > 0.05 control period **violates** the README ("set it lower but not larger") — predicted trajectory diverges from executed motion, worse on this ω-under-delivering chassis; (b) horizon is **40×0.10 = 4.0 s**, not the "2.0 s" the comment/strategy claim; (c) "IDENTICAL optimizer compute" is **false** — integrations = batch×time_steps = 500×40 = 20 000 vs 12 500 prior = **+60 %**, on a Jetson that was CPU-starved. **Revert `model_dt` to 0.05**; if a longer horizon is wanted, raise `time_steps` and re-measure `ros2 topic hz /cmd_vel ≥18` under field load. |
| **Phase 0 — BackUp `1.5→0.3 m`, speed `0.08→0.10`** | **CONFIRMED** | Applied (XML:166-167). Latent-bug claim holds. **But `time_allowance=25` (XML:168) was not reduced** — now generous for a ~3 s move and blows the budget if the chassis stalls (#17). Set to ~5 s. |
| **Phase 3 — DWB shadow fallback controller** | **PARTIALLY** | Plugin name `dwb_core::DWBLocalPlanner` correct, but: (a) XML hardcodes `controller_id="FollowPath"` inside a PipelineSequence — needs real BT surgery into a ReactiveFallback; (b) DWB's coarse DWA window on a 0 m-turn chassis in a 2–3 m lane is *unlikely* to find a path where MPPI's 500-sample batch failed. **Cheaper first:** raise MPPI `batch_size` back toward 1000 (the original starvation cause was RViz, now fixed per CLAUDE.md) before building a second controller. |
| **Phase 4 — RoboJackets `back_circle` rear-LETHAL layer** | **CONFIRMED (real source) + SEQUENCING BUG** | `back_circle_layer.cpp:32-41` + `back_circle.yaml` (offset −0.5, length 2.0, width 1.5) exist in-repo and match the strategy numbers. **But:** recovery order is `ClearCostmapAroundRobot reset_distance=3.0` (XML:151) *then* `BackUp` (XML:166) — the clear wipes the rear cells before BackUp checks them, so BackUp still backs blind on the first cycle. Mark the layer non-clearable or filter the clear. Also: **the refuted finding shows the rear is NOT actually blind** — the 360° Velodyne marks rear 3D obstacles into the local costmap; the residual gap is only flat lane *tape* (<0.2 m height filter / <0.7 m min-range). So back_circle is a *belt-and-suspenders for tape*, lower priority than the strategy implies. |
| **Refutation — "odom-frame goals while keeping map EKF" does NOT escape drift** | **CONFIRMED** | bt_navigator.global_frame=map (:21), global_costmap.global_frame=map (:491, doc's :479 stale), planner plans on map costmap; goal reprojected through drifting `map→odom` every cycle. Only full map≡odom works. |

**Items already applied but strategy lists as pending:** `transform_tolerance:0.5`, MPPI `model_dt/time_steps`, BackUp `0.3/0.10`. Update the doc's Phase-0 status.

**Strategy claims that conflict with official docs:** the `model_dt:0.10` change (violates MPPI README); the "6σ²"/"13.8" rejection framing (wrong unit, propagated from the strategy into the live YAML comments).

---

## 3. What we are still doing wrong (confirmed + most-credible uncertain)

### NEW findings first

#### Controller / costmap

**N1 — `model_dt: 0.10` violates the MPPI control-period rule.**
*Current:* `controller_frequency: 20.0` (nav2_params_humble.yaml:55) → 0.05 s period, but `model_dt: 0.10` (:87).
*Why wrong:* nav2_mppi_controller README: "if your control frequency is 20hz, this should be 0.05 … you may also set it lower but **not larger**." MPPI applies only the first control, re-optimizing every 0.05 s, but integrates the first step as if held 100 ms → predicted ≠ executed. CLAUDE.md itself: "MPPI is more sensitive to inner-loop tracking than RPP," and this chassis under-delivers ω ~13 %.
*Fix:* revert `model_dt: 0.05`. For horizon, raise `time_steps` (≤56) and verify `ros2 topic hz /cmd_vel ≥18` under field load with batch 500. The only doc-legal way to keep 0.10 is `controller_frequency: 10` — but the strategy mandates 20 Hz, so that's off the table.

**N2 — `inflation_radius 0.3 < inscribed 0.37` collapses the MPPI gradient.**
*Current:* footprint `[[0.5,0.37]...]` (:258, :500) → inscribed 0.37 m; `inflation_radius: 0.3` both costmaps (:470, :604); `cost_scaling_factor: 3.0`; CostCritic `consider_footprint: true` (:164), `cost_weight 6.0` (:162).
*Why wrong:* `inflation_layer.cpp` logs `RCLCPP_ERROR` ("configured inflation radius … smaller than … inscribed radius") on both costmaps. Every inflated cell is ≤0.37 m from the obstacle → all return `INSCRIBED_INFLATED_OBSTACLE` (253); **no cell ever reaches the `exp(-cost_scaling·(d−r_inscr))` decay regime**, so the gradient is a hard 253→0 step at 0.3 m and `cost_scaling_factor=3.0` is **inert**. CostCritic then sees only the flat 253 plateau → no smooth early avoidance → late/abrupt reactions and "optimizer fail to compute path" in tight barrel fields. The working-tree `cost_weight 3.81→6.0` bump ("strengthen centerline pull with reduced inflation") **cannot** restore a continuous field — it scales a near-binary penalty.
*Fix:* `inflation_radius ≥ 0.37` (ideally ~0.4 with a steeper `cost_scaling_factor` ~4–5 to keep the lethal halo narrow). At 0.4 m a 3 m lane leaves 2.2 m free, a worst-case 2 m lane leaves 1.2 m vs 0.74 m robot width — still passable. The CLAUDE.md "1.0 local / 0.65 global" note and `architecture_decision_global_frame.md` Q1 (40×40 m) are **stale** — global is now 100×100 m, inflation 0.3/0.3.

**N3 — Phase-0 "identical compute" comment is arithmetically false (refines N1).**
40×0.10 = **4.0 s** horizon (not 2.0 s); integrations rose **+60 %** not "identical." Correct the comment (lines 81-89). Don't trust the "free" framing on a previously-starved Jetson.

**N4 — `PathAngleCritic mode: 0` is a no-op on Humble.**
*Current:* `mode: 0` (:190); `vx_min: -0.4` enables reverse (:104); `PreferForwardCritic` weight 5.0 (:154-158).
*Why wrong:* Humble `PathAngleCritic` has **no `mode` param** (added Iron/Jazzy) — it reads only `forward_preference`. With `vx_min<0`, `reversing_allowed_=true` and `forward_preference` defaults true → it still penalizes pointing backward, fighting the reverse that `vx_min` and BackUp want. Intent is muddled.
*Fix:* delete `mode: 0`; decide intent — if reverse is wanted, set `forward_preference: false` and reduce PreferForwardCritic; if forward-only is fine for IGVC, set `vx_min: 0.0`.

#### Localization

**N5 — Rejection-threshold semantics inverted in comments; `13.8` disables the GPS gate.**
*Current:* ekf.yaml:194 `odom0_pose_rejection_threshold: 13.8`, comment "6σ² ≈ tolerate 5 m gap"; commit 0516ccc "13.8 (6σ²)".
*Why wrong:* RL `filter_base.cpp checkMahalanobisThreshold`: `threshold = n_sigmas * n_sigmas; if (squared_mahalanobis >= threshold) reject`. The parameter is **already in Mahalanobis (σ) units** — the code squares it. For the 2-DOF GPS update: value 13.8 → threshold 190 → P(reject) ≈ 4e-42 → **gate effectively OFF**. A true 6σ gate is value 6.0; "6σ²" would be √6 ≈ 2.45. The IMU gates at 5.0 (:52-53,147-148) are genuinely ~5σ — **leave those**.
*Fix:* relabel comments as Mahalanobis-distance gates; pick by chi-square on 2 DOF (99 % accept ≈ 3.0, 99.99 % ≈ 4.3). **Caveat:** the old 4.0 caused a *real* permanent lockout (rejected GPS never shrinks P → 36 m gap stayed open). Dropping to 3–6 **reintroduces that risk unless** GPS measurement covariance is sized to true 2–5 m SBAS error first. Sequence: floor covariance → flip differential → *then* tighten gate; never atomically.

**N6 — map-EKF `process_noise x,y = 0.5` (was 0.1) + 13.8 gate = GPS-following with no outlier protection.**
*Current:* ekf.yaml:209-210, raised in 0516ccc to keep the gate "alert" on the hand-move test.
*Why wrong:* This pairs a wide-open gate (N5) with fast P growth to defeat a lockout that exists *only because* GPS is differential (Issue #1). RL guidance: process noise should reflect real unmodeled dynamics, not be inflated to manipulate the gate. Together they re-introduce the per-fix snapping that differential mode was added to suppress.
*Fix:* revert in staged order — differential:false → gate ~3–6 → process noise toward 0.1 — with `recover_ekf.py` armed. Absolute mode dissolves the hand-move lockout (it was a differential-mode artifact: rejected deltas never shrink P).

**N7 (uncertain, credible) — map≡odom leaves no absolute heading anchor, and the 5σ IMU gate discards the only one we have.**
The Xsens absolute yaw (`imu0_differential: false`, ekf.yaml:41-42,140-141) is the *sole* absolute heading source in **both** the current dual-EKF design and under map≡odom — the map EKF/navsat add no heading (navsat `use_odometry_yaw: false` passes IMU yaw through; GPS contributes x/y only). So map≡odom does **not** remove a heading anchor that exists today; **the finding's "map≡odom removes the reference" framing is over-stated.** What *is* real and unmitigated: (a) `mission_manager.py:97` sends goal 0 in `__init__` with **no yaw-convergence gate** despite the documented 150 s mag warmup (#15) — standing bug today; (b) the 5σ gate *will* discard Xsens yaw exactly during a stuck-bias event (CLAUDE.md 70× gyro-bias incidents) with no redundant absolute source; (c) no GPS-course-over-ground heading cross-check exists.
*Fix:* add a startup gate blocking the first NavigateToPose until Xsens yaw converges (warmup discard leg); add a GPS-course-vs-odom-heading guard (the strategy's named-but-unspecified safeguard). These are worth doing **regardless** of map≡odom.

**N8 (low) — `navsat use_odometry_yaw: false` deviates from canonical tutorial (`true`) and anchors the GPS frame on raw unwarmed Xsens yaw.** A wrong heading here rotates the whole GPS→map mapping (the 145° class of bug). If keeping the map EKF (Phase-1 interim), test `use_odometry_yaw: true` (uses the fused, more-stable yaw). Only matters while the map EKF lives.

#### Perception / TF

**N9 — `transform_tolerance: 0.5` cannot fix Issue #18 (past-extrapolation).** Covered in §2. Keep it (harmless) but the **real fix is the cloud stamp source or a `TimePointZero` latest-transform fallback** ported from `robojackets line_layer.cpp:165-175` (`canTransform(stamp)` → on fail `lookupTransform(..., Time{0})`). Verified real in vendored source.

**N10 — Stale ZED cloud stamps: `use_pub_timestamps` never set (uncertain root cause).** `common_stereo.yaml:200` defaults `false` (camera-capture time, not ROS-now); `zed_front.yaml` doesn't override. **The genuinely valuable catch:** `segmentation_buffer.cpp:141` says the cloud "lags ~4 s" while nav2_params_humble.yaml:243 says "~0.3–0.4 s" — **the team never empirically pinned the lag.** *But* the proposed fix (`use_pub_timestamps:true`) is **direction-confused**: a 0.3–0.4 s *past* lookup against a 10 s continuously-updated buffer cannot throw a *past*-extrapolation error; pushing stamps to ROS-now risks *future*-extrapolation (worse). Also the layer consumes `perception_node`'s re-stamped relay (perception_node.py:343-375 → `max(image,cloud)` stamp), not the raw ZED cloud. *Fix:* first `ros2 topic delay` the cloud to resolve the 0.4 s vs 4 s contradiction; then prefer the structural `TimePointZero` fallback (N9), which is robust regardless of stamp source.

**N11 — `perception_node` re-stamps mask AND cloud to the OLDER input stamp, propagating staleness.** perception_node.py:343-348,375. Canonically correct for spatial accuracy *iff* the TF chain can serve it — here it guarantees the MessageFilter gets a past stamp, plus cv2/pipeline latency on top → whole-frame drop. Prefer the source/`TimePointZero` fix over a `now()`-restamp (which adds velocity×latency spatial error ~5–15 cm).

**N12 — The `segmentation_buffer.cpp` fix will be LOST on next `vcs import`.** `avros.repos:24-26` pins a fork; `git check-ignore` confirms the checkout is ignored. The existing wall-clock edit at :141 is already at risk. **Commit the `TimePointZero` + decay fixes to `Paarseus/semantic_segmentation_layer` and bump the repos pin** — otherwise the documented one-time setup silently reverts them.

**N13 (low) — HSV pipeline `adaptive_k` all-image-exclusion failure has no clamp/empty-mask warning.** hsv.py:158 `v_floor = mean + k*std` (no clamp), refresh gated at :138. If `hsv` is re-selected for night and `adaptive_k=2.5` is left set, the lane layer goes blind silently. Production pipeline is `sooner25`, but `hsv` stays selectable. Clamp `v_floor ≤ 250` + WARN on N empty frames; mark `hsv` test-only.

#### BT / recovery

**N14 — `DriveOnHeading` (CrawlForward) unreachable.** `RecoveryNode number_of_retries=3` (XML:78) + 4-child RoundRobin (XML:149-176). RoundRobin advances idx by 1 per SUCCESS; RecoveryNode ticks the recovery child at most 3 times → idx 0,1,2 only. The advertised "nudge through a borderline passage" (idx3) never fires. *Fix:* `number_of_retries=4` (budget: a 4th ~1.5 s crawl stays under 45 s) and fix the header comment (lines 131-136). One-char fix.

**N15 — `BackUp time_allowance=25` s blows the budget.** XML:168 vs a ~3 s move (0.3 m @ 0.10), the "3×8 s = 24 s" comment, and the 45 s outer Timeout. A stalled reverse (#17 fwd-43 %-after-reverse) can legally run 25 s = >half the budget. *Fix:* ~5 s; DriveOnHeading ~3–4 s.

**N16 — `ClearCostmapAroundRobot reset_distance=3.0` (no `plugins` filter) wipes painted lanes.** XML:151-154 clears *all* clearable layers including `semantic_layer` in a 3×3 m box around a robot with a 2.49 m sweep in 2–3 m lanes — the exact lane-wipe race the 2026-05-12 comment claims it fixed, just localized. *Fix:* `plugins="stvl_layer"` (clear only LiDAR) **or** `reset_distance ~1.0`. Verify `semantic_layer` clearable flag.

**N17 — Inner `ComputePathToPose` recovery does global `ClearEntireCostmap`.** XML:96-102 — exactly the global wipe the team *removed* two levels down (XML:105-110, 138-144) for being a lane-wipe hazard. A transient TF hiccup wipes global lane marks. *Fix:* `ClearCostmapAroundRobot` or `plugins="obstacle_layer"` for consistency.

**N18 — BT design premise "orchestrator fires fresh goals at 2 Hz" is false.** XML:23-24,82-83,122-124 vs `mission_manager.py:97` (single send) + cursor-advance-only re-send (:225); the 5 Hz proximity timer (:43) checks distance, doesn't re-send. Consequence: `GoalUpdated` in the ReactiveFallback almost never fires mid-waypoint → an in-flight BackUp runs to completion, contrary to design. Recovery timing was tuned against a cadence that doesn't exist. *Fix:* either make mission_manager periodically re-publish the current goal (so GPS-smear updates flow and GoalUpdated works) **or** rewrite the comments to static-goal reality and re-evaluate whether GoalUpdated belongs there.

**N19 — `mission_manager` proximity check measures distance in the drifting map frame.** :263 `math.hypot` in `/odometry/global` (map-anchored); under FIX-only GPS this drifts 5–10 cm/s → 2.0 m acceptance triggers early/late. Self-heals under Phase 2 (`/odometry/filtered`). Until then, document the contamination.

#### Control / firmware

**N20 — Heading-hold P-controller fights MPPI's closed loop.** actuator_node.py:446-453 overrides ω whenever `|w_cmd|<0.05 & |v|>0.02`, on *every* straight cmd_vel including Nav2's — a closed-loop ω correction in the driver, exactly the cascade-ordering violation CLAUDE.md/commit 401167b condemned for `yaw_rate_kp`. *Fix:* gate heading-hold OFF for the `/cmd_vel` source (enable only for webui/teleop), e.g. behind the existing `/autonomous_mode` sub (:262).

**N21 — SparkMAX velocity-PID integrator never reset across reverse→stop→forward (Issue #17 candidate #2).** During the 3 s settle, cmd_vel keeps publishing v=0 so actuator streams `L0 R0` in MODE_VELOCITY (not `S`), keeping the PID active with the negative I-term accumulated over 9 s reverse; `kIZone=600` enables but never *clears* integration. Forward duty must unwind it first → distance loss. *Fix:* on a commanded-v sign reversal, command `S` (MODE_DUTY 0) for one tick before resuming velocity mode, or add a `setIAccum(0)` Teensy command. Verify on the #17 bag.

**N22 — BURN/PERSIST cannot work as a standalone frame on FW26.1.4 (refines Issue #9).** REVLib 2025 replaced "set params then `burnFlash()`" with `configure(config, ResetMode, PersistMode.kPersistParameters)` — "parameters are now set and persisted in the same configuration call." A standalone PERSIST frame after independent PARAMETER_WRITEs likely won't capture RAM gains → matches "BURN then revert on power-cycle." *Fix:* rely on actuator_node's startup re-push (already authoritative, :200-204), remove the false "BURNed to flash" wording from actuator_params.yaml:106 + CLAUDE.md, add a param-readback verify. (Refines #9 from "wrong bytes" to "wrong persistence model.")

**N23 — `/wheel_odom` twist covariance over-confident for a skid-steer.** actuator_node.py:303 `vyaw σ²=0.0001` (σ≈0.57°/s) — but skid-derived yaw rate is the *least* trustworthy wheel quantity (Mandow exists *because* tracks violate pure rolling; 82–86 % open-loop ω delivery). This tells the EKF to trust skid yaw rate ~equally to the 100 Hz Xsens gyro, biasing the heading fed to MPPI. The pose-covariance block (:290-296) is **dead config** (EKF fuses only vx,vyaw). *Fix:* raise wheel `vyaw` covariance 1–2 orders (~0.02–0.05) so the gyro dominates; delete/annotate the unused pose block.

### Confirmed known issues (re-grounded, still open)

- **K1 — `odom0_differential: true` (Issue #1).** See §2; doc-mandated `false` + covariance floor.
- **K2 — `mission_manager` hard-`SystemExit(1)` on `/fromLL` + `/odometry/global` binding.** mission_manager.py:54,68,129-134,147; localization.launch.py:126. `__init__` calls `_convert_to_map_frame` (:69) *before* the Nav2 action wait → node dies at startup before any goal under Phase 2. navigation.launch.py:268-271 passes only `waypoints_file`, so the `/odometry/global` default is what runs. **Same-commit rewire is a launch-blocking prerequisite for Phase 2**, plus a local lat/lon→meters projection that reproduces navsat's cartesian convention (datum-typo hazard, navsat.yaml datum is Michigan `[42.667925,-83.218195,0.0]`, slot 3 = heading_rad).

---

## 4. Refuted / non-issues (do not chase)

- **BackUp is collision-blind to the rear** — REFUTED. The 360° Velodyne (STVL `horizontal_fov_angle: 6.283`, chassis-center mount) marks rear 3D obstacles into the local costmap BackUp checks. Only residual gap is flat lane *tape* (<0.2 m height / <0.7 m min-range). The proposed always-LETHAL back_circle would *disable* the recovery (isCollisionFree fails every backup).
- **"map≡odom strategy doesn't account for `/fromLL`"** — REFUTED. The strategy already lists this coupling (lines 118-130) and prescribes the same rewire. (The *finding* — hard SystemExit timing — is the valid refinement; the "strategy missed it" framing is wrong.)
- **Double accel-limiting in series (velocity_smoother + actuator)** — REFUTED. Not in series: smoother output `/cmd_vel_smoothed` is read by nobody; actuator reads raw `/cmd_vel`. The actuator slew is the *only* active limiter. (The real residual = dangling smoother, N-dead-node / §5 Phase 1.)
- **Heading-hold sign not inverted for reverse (Issue #17 cause)** — REFUTED. It's a pure *heading* regulator; `ω_body = w` independent of v, so the sign is correct in reverse. The proposed `sign(v)*kp` would *destabilize* it. The lock also releases at `|v|≤0.02` (the #17 3 s settle), so no stale-target carryover. The ~0.0078 m/s/track diversion is physically incapable of the 48-pt deficit — #17 is integrator wind-up (N21), not heading-hold.

---

## 5. Superseding multi-phase strategy

Each phase independently shippable. **[WT]** = already in working tree. **[FIELD]** = needs a field session to validate. **[DESK]** = config/code-only, bench-verifiable.

### Phase 0 — Correct the half-applied edits (DESK, no field needed)
The current Phase-0 edits are **partly wrong** and ship today:
1. **Revert `model_dt: 0.10 → 0.05`** (N1/N3). Keep `time_steps` at 40 only if `ros2 topic hz /cmd_vel ≥18` holds; else 25–32. Fix the false "identical compute / 2.0 s" comment.
2. **Raise `inflation_radius 0.3 → 0.4`** both costmaps (N2), `cost_scaling_factor ~4–5`; confirm the launch log no longer prints the inscribed-radius ERROR. Update stale CLAUDE.md / architecture_decision Q1 numbers.
3. **`BackUp time_allowance 25 → ~5`**, DriveOnHeading `→ ~4` (N15). **`RecoveryNode number_of_retries 3 → 4`** (N14). Fix BT header comments.
4. **`ClearCostmapAroundRobot plugins="stvl_layer"`** (N16); inner `ComputePathToPose` recovery → around-robot clear (N17).
5. Delete dead `PathAngleCritic mode: 0`; decide reverse intent (N4).
6. Keep `transform_tolerance:0.5` (harmless) but **stop claiming it fixes #18** (N9).
*Dependencies: none. These are corrections to changes already made — highest impact-per-effort, ship first.*

### Phase 1 — Localization config + dead-node hygiene (DESK + short FIELD validate)
*Ordering changed vs the old plan: do these in strict sequence, never atomically.*
1. **Floor `/odometry/gps` covariance** to ~5 m σ (new republisher or NavSatFix-source patch — **no knob exists today**, N5/K1 co-requisite). **Prerequisite for step 2.**
2. **`odom0_differential: true → false`** (K1) with `recover_ekf.py` armed.
3. **Relabel + retighten the rejection gate** `13.8 → ~3–6` Mahalanobis (N5); drop the "6σ²" language everywhere (YAML + commit lore).
4. **Revert map-EKF process noise `0.5 → ~0.1`** (N6) — the hand-move lockout dissolves in absolute mode.
5. **Decide `use_odometry_yaw`** (N8) — test `true` while the map EKF lives.
6. **Resolve the dangling velocity_smoother** (N7-dead-node): simplest is **drop it from the lifecycle list** (actuator already does asymmetric slew + heading-hold + Mandow the smoother can't replicate; double-smoothing adds latency that hurts MPPI). If kept, wire canonically (controller→`cmd_vel_nav`, smoother→`cmd_vel`) AND re-sync its limits to the chassis envelope (`max_velocity [0.7,0,1.5]`, `max_accel [0.4,0,1.5]`).
*Dependencies: step 1 gates 2; 2 gates 3 gates 4. Validate snap magnitude + path inflation in a short field run before tightening the gate.*

### Phase 2 — Perception/TF durability (DESK, commit upstream)
1. **Port `TimePointZero` fallback** into `segmentation_buffer.cpp` (N9, robojackets pattern) — robust regardless of stamp source.
2. **`ros2 topic delay`** the ZED cloud to resolve the 0.4 vs 4 s contradiction (N10); only then consider `use_pub_timestamps`.
3. **Commit both fixes to the `Paarseus` fork + bump `avros.repos` pin** (N12) — otherwise they vanish on re-import. **This step is mandatory or Phase 2 is not reproducible on the Jetson.**
4. Clamp HSV `v_floor` + empty-mask WARN; mark `hsv` test-only (N13).
*Dependencies: none on Phase 1. Can run in parallel.*

### Phase 3 — map ≡ odom (DESK rewire + FIELD validate) — the highest-value architectural borrow
*Converges on the proven IGVC AutoNav winner architecture (SoonerRobotics: no GPS in TF, dead-reckon between flags, GPS only sets goals).*
**Prerequisites (same commit, launch-blocking — N7/K2):**
1. Rewire `mission_manager` `odom_topic → /odometry/filtered` (+ the launch param), `/fromLL → local lat/lon→meters projection` from a per-site origin (reproduce navsat cartesian convention; datum hygiene).
2. **Add the GPS-course-vs-odom-heading guard** (promoted to required deliverable) and a **startup yaw-convergence gate** before goal 0 (N7).
3. Delete `ekf_filter_node_map` + `navsat_transform`; broadcast identity `map→odom` @50 Hz on `/tf`.
4. Fix the proximity-distance frame (N19, self-heals) and the static-goal/2 Hz comment mismatch (N18) — decide whether to re-publish goals periodically so GoalUpdated works.
*Dependencies: Phase 1 step 6 (smoother) and Phase 0 should land first so the controller is sane before changing the frame graph. **Phase 3 moots N6/N8 and the differential question** — if you commit to Phase 3 soon, Phase 1 steps 1–5 are interim-only.*

### Phase 4 — Controller robustness (FIELD)
1. **Raise MPPI `batch_size` toward 1000** (RViz starvation fixed) before any second controller — cheaper fix for "optimizer fail to compute path" than DWB.
2. If still needed, DWB shadow fallback **requires real BT surgery** (ReactiveFallback, not a plugin append) + `ros-humble-dwb-core`.
*Dependencies: Phase 0 (correct inflation/model_dt) must land first or you'll tune against a broken gradient.*

### Phase 5 — Firmware / actuator (FIELD, isolated)
1. **Gate heading-hold to teleop only** (N20) — removes the hidden ω loop fighting MPPI.
2. **Clear SparkMAX I-accumulator on v sign reversal** (N21) — fixes Issue #17 forward-delivery deficit.
3. **Raise `/wheel_odom` vyaw covariance** (N23) so the gyro dominates EKF yaw.
4. **Fix the BURN persistence model / docs** (N22, Issue #9) — rely on startup re-push, add readback.
5. **Port RoboJackets `back_circle`** (rear lane-*tape* exclusion) — but **order it after the recovery clear** or BackUp still backs blind on cycle 1. Lower priority than the strategy implied (LiDAR already covers 3D rear).
*Dependencies: independent; N20/N21 most affect MPPI tracking so pair with Phase 4 field session.*

**Net re-ordering vs old plan:** Phase 0 is now *corrections* (the old Phase-0 edits were partly wrong), not new tuning. The localization gate/differential/process-noise work is split into a strict 6-step sequence with a covariance-floor prerequisite the old plan named but couldn't execute. map≡odom drops from "Phase 2" to "Phase 3" behind controller/perception sanity, and its heading guard + startup yaw gate are promoted to required deliverables.

---

## 6. Sources

- robot_localization (cra-ros-pkg, ros2): `src/filter_base.cpp` `checkMahalanobisThreshold` (`threshold = n_sigmas * n_sigmas`); `src/navsat_transform.cpp` (no covariance floor; copies NavSatFix covariance). Integrating GPS doc: http://docs.ros.org/en/melodic/api/robot_localization/html/integrating_gps.html ("enabling differential integration defeats the purpose"); navsat_transform_node doc (`odomN_differential` must be `false`). RL issue #630 (threshold² = chi-square quantile).
- Nav2 (ros-navigation/navigation2, humble): `nav2_mppi_controller/README.md` ("set it lower but not larger"; "horizon = time_steps × model_dt"; obstacle critic weights tuned with inflation radius & scale); `nav2_costmap_2d/plugins/inflation_layer.cpp`/`.hpp` (inscribed-radius ERROR; `INSCRIBED_INFLATED_OBSTACLE`/`exp` decay); `src/critics/cost_critic.cpp`, `path_angle_critic.cpp` (no `mode` on Humble); `controller_server.cpp` (`cmd_vel` hardcoded publisher); `velocity_smoother.cpp` (`cmd_vel`→`cmd_vel_smoothed` hardcoded); `round_robin_node.cpp`, `recovery_node.cpp` (retry/idx semantics); `nav2_behaviors drive_on_heading.hpp` (collision check); `footprint.cpp` `calculateMinAndMaxDistances`.
- Nav2 docs: configuring-velocity-smoother (`cmd_vel_nav`→`cmd_vel`), configuring-navfn (holonomic), ClearCostmapAroundRobot / ClearEntireCostmap bt-plugins, configuring-bt-xml (GoalUpdated).
- Canonical tutorial: `nav2_gps_waypoint_follower_demo/dual_ekf_navsat_params.yaml` (`odom1_differential: false`, `use_odometry_yaw: true`).
- geometry2 (humble): `tf2_ros::Buffer::transform` timeout ("how long to block"), `tf2_ros::MessageFilter` time_tolerance (added to stamp, future-side).
- ZED ROS2 wrapper: `common_stereo.yaml` `use_pub_timestamps` (false = camera timestamp). https://www.stereolabs.com/docs/ros2/zed-node
- REVLib 2024→present migration: `configure(config, ResetMode, PersistMode.kPersistParameters)`. https://docs.revrobotics.com/revlib/archive/24-to-present.md
- In-repo vendored references: `robojackets/igvc_navigation/src/mapper/line_layer.cpp:165-175` (`TimePointZero` fallback), `back_circle_layer.cpp:32-41` + `back_circle.yaml`.
- Field context: SoonerRobotics 1st-place AutoNav 2024 & 2025 (`SoonerRobotics/autonav_software`, ROS2 Humble, non-Nav2) — confirms the map≡odom direction.
- Live files verified: `ekf.yaml`, `nav2_params_humble.yaml`, `navsat.yaml`, `actuator_params.yaml`, `actuator_node.py`, `mission_manager.py`, `navigation.launch.py`, `navigate_igvc_autonav_humble.xml`, `perception_node.py`, `hsv.py`, `avros.repos`.