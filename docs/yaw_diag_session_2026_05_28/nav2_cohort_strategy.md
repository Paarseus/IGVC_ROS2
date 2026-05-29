# IGVC Nav2-Cohort Analysis → Multi-Phase Strategy — 2026-05-29

Synthesized from an 11-agent workflow (P1-focused): deep code-reads of **Hosei** (`orange_ros2`)
and **TnTech** (`TnTechIGVC2026`), outdoor non-IGVC Nav2-GPS reference designs (official
`nav2_gps_waypoint_follower`, `robot_localization` upstream, ros-agriculture), a precise
characterization of our own GPS pipeline, plus lighter passes on costmap/TF, controller/compute,
and recovery. Every candidate fix was **adversarially verified against our actual config files**
(`ekf.yaml`, `navsat.yaml`, `nav2_params_humble.yaml`, the BT XML, `mission_manager.py`).

Scope constraint honored throughout: **we keep Nav2 + MPPI + NavfnPlanner + dual-EKF + kiwicampus
lane layer.** This is deliberately the "Nav2-keeping cohort" lens, not the "ditch Nav2 like Sooner"
lens of the earlier winners research.

---

## The headline finding

**This analysis independently re-derived a decision you already made and never implemented.**

`docs/architecture_decision_global_frame.md` (2026-05-26, *Status: RECOMMENDATION — not yet
implemented*) already says, for Q2:

> **Adopt TnTech `map ≡ odom` pattern + carrot-to-goal node. Decommission `navsat_transform_node`
> + `ekf_filter_node_map`.**

Starting from scratch and reading TnTech's actual code, the workflow landed on the same architecture
as the single highest-applicability fix for our #1 open problem (P1: 4.1× map-path inflation /
return-goal wiggle). Two independent derivations converging on the same answer is a strong signal.

**And it refuted the tempting shortcut.** The most attractive cheap idea — "keep the map EKF, just
send goals in the `odom` frame" — was the synthesis lead's pick, and adversarial verification
**killed it**. Reason, confirmed against our files:

- `bt_navigator.global_frame: map` (`nav2_params_humble.yaml:21`)
- `global_costmap.global_frame: map` (`:479`), planner `GridBased` plans in that frame (`:616`)

So an `odom`-frame `NavigateToPose` goal gets **re-projected into the drifting `map` frame every
planning cycle**. A current timestamp avoids the *Extrapolation Error* (CLAUDE.md:411) but does
nothing about the per-cycle re-projection — the W3 553°-rotation chase persists. **Odom-frame goals
only help if you also move `bt_navigator` + the global costmap to the `odom` frame — which *is*
`map ≡ odom`.** There is no half-measure here.

---

## Our problems vs the Nav2 cohort (comparison matrix)

| # | Our problem | How the Nav2 cohort handles it | Verdict for us |
|---|---|---|---|
| **P1** | Map-EKF inflates path 4.1× (101.87 m vs 24.87 m odom truth); return goals wiggle/circle (W3: 553° rotation, net +4.26°) because the `map→odom` TF drifts 5–10 cm/s under FIX-only GPS noise | **TnTech:** GPS never touches TF — `map ≡ odom` identity, GPS only sets goals via a carrot node. **Hosei:** odom *is* the truth frame, GPS fused into odom as occasional absolute correction, waypoints stored as odom-frame meters. **Official Nav2/agriculture:** if you *do* fuse GPS into a map EKF, use `differential: false` (absolute), never `true` | **We are the outlier.** `odom0_differential: true` (`ekf.yaml:186`) integrates noisy GPS *deltas* as a random walk — the exact Brownian signature that produces a 4.1× path. Two-stage fix below |
| **P2** | EKF/GPS Mahalanobis lockout (band-aided 2026-05-28 by raising gate 4.0→13.8 + process noise 0.1→0.5) | Cohort sidesteps it: with GPS out of the map EKF (TnTech) the gate problem can't exist; with absolute fusion (`differential:false`) the filter stays near GPS so the gate doesn't lock | Lockout is a *symptom* of forcing noisy GPS into TF. Fixing P1 dissolves P2; keep `recover_ekf.py` armed during transition |
| **P3** | Lane lines treated as lethal walls → corridor trap (mid-fix via soft-cost) | Cohort runs lane marks in the **local** costmap only (we already do; semantic layer excluded from global). RoboJackets uses a log-odds line layer (more robust than kiwicampus overwrite) | Continue the in-progress soft-cost work (`lane_following_strategy.md`). Out of P1 scope; not changed by this analysis |
| **P5** | ZED TF-stale → semantic layer drops frames (Issue #18) | RoboJackets `line_layer.cpp:165-175`: on exact-stamp `canTransform` failure, fall back to `lookupTransform(..., Time{0})` (latest) | Two-layer fix below — config tolerance now, `TimePointZero` fallback in `segmentation_buffer.cpp` for the real fix |
| **P4** | MPPI CPU-starves on Jetson → cmd_vel 3-5 Hz → actuator timeout → go/stop | TnTech: 10 Hz / batch 2000 / iter 3 on AGX+GPU. Hosei: DWB at 5 Hz | Root cause was **RViz on the Jetson**, already fixed. Keep 20 Hz. One free win: `model_dt 0.05→0.10` + `time_steps 25→40` restores 2.0 s horizon at identical compute. DWB as a *shadow fallback controller* |
| **P6** | BackUp recovery (1.5 m) crosses rear-blind lanes (no rear sensor) | RoboJackets `back_circle_layer`: paints lethal cells behind the robot so BackUp's collision check aborts. TnTech: Spin instead of BackUp | Spin is unsafe for **our** 2.49 m footprint sweep in a 2-3 m lane. Fix = shrink BackUp dist now + port `back_circle` as a Nav2 plugin later |

---

## The strategy — 5 phases, impact-ordered, each independently shippable

Phases 0/3/4 are decoupled from the P1 work and can land in parallel. Phases 1→2 are the P1 spine
and are **sequential**: hygiene first, then the structural change.

### Phase 0 — Zero/low-risk quick wins (≈1 hr, do first)

These need no field session and reduce active risk immediately.

| Fix | Change | File |
|---|---|---|
| **Issue #18 (P5) config layer** | Add `transform_tolerance: 0.5` to `local_costmap.local_costmap.ros__parameters` (currently absent → defaults to 0.3 s; ZED stamp lags 0.3-0.4 s). kiwicampus reads tolerance from the *costmap node's* namespace, not the controller's | `nav2_params_humble.yaml` local_costmap block |
| **BackUp safety (P6)** | `backup_dist: 1.5 → 0.3`, `backup_speed: 0.08 → 0.10`. The XML currently disagrees with its own header comment ("0.3 m") — the 1.5 m is a latent bug; at 0.08 m/s it also takes 18.75 s, blowing the 24 s recovery budget | `navigate_igvc_autonav_humble.xml` (BackUp node) |
| **MPPI horizon (P4)** | `model_dt: 0.05 → 0.10`, `time_steps: 25 → 40`, `batch_size: 500` unchanged. Same total optimizer work (500×40×½-cost ≈ 500×25), but restores a **2.0 s / 2.8 m** horizon vs the current 1.25 s / 0.875 m | `nav2_params_humble.yaml` FollowPath |

Verify after launch: `ros2 param get /local_costmap/local_costmap transform_tolerance`,
`grep -c "TF Exception"` over 60 s, and `ros2 topic hz /cmd_vel` ≥ 18 Hz with the new MPPI horizon.

### Phase 1 — Localization hygiene: absolute-GPS map EKF (Option 1, ≈30 min + 2 field sessions)

This is correct regardless of whether we adopt `map ≡ odom`, and it's the canonical config
(`navigation2_tutorials` and ros-agriculture both use `differential: false`). It collapses the
*math* term of the 4.1× inflation. **But it is necessary-not-sufficient** (residual per-fix snaps of
0.3-1.0 m still nudge a map-frame goal), and it must be applied **incrementally** — the verifier
flagged that the 13.8 gate + 0.5 process-noise were set *this same week* to defeat a hand-move
lockout; reverting both atomically can re-trigger it.

Staged edits to `ekf.yaml` `ekf_filter_node_map`, **one at a time, with `scripts/recover_ekf.py`
armed and the hand-move/lockout test re-run after each**:

1. `odom0_differential: true → false` (line 186) — stops Brownian delta integration. *Co-requisite:*
   floor `/odometry/gps` covariance in `navsat.yaml` (σ≈5 m) — in absolute mode the Kalman gain
   (snap magnitude) is set by GPS covariance vs P; if navsat under-reports, absolute snaps *harder*
   than differential did.
2. `odom0_pose_rejection_threshold: 13.8 → ~6` (line 194) — only after (1) is validated.
3. `process_noise_covariance` x,y `0.5 → ~0.1` (lines 209-210) — last, watch the lockout case.

Validate with a **stationary** session (map→odom no longer jumps on a sudden fix) **then** a
motion session replaying the W2/W3/W4/W5 protocol from the 2026-05-28 bag. Update CLAUDE.md's
Known-Issues entry documenting the `differential:true` rationale (now reversed — your own
`phase_1_2_results.md:163` already flagged it as suspect).

### Phase 2 — Structural fix: `map ≡ odom` + GPS-as-carrot (Option 2 = your Phase B plan, ≈1-2 days)

The fix that kills the failure class **by construction** instead of tuning around it. After this,
GPS sample noise has *no path* into the map frame, so the 4.1× inflation literally cannot recur and
the return-goal wiggle dies (a fixed map goal is genuinely fixed relative to the 0.04%-accurate odom
pose). This is `architecture_decision_global_frame.md` Q2, now validated and de-risked by the
verifier's refinements below.

Core changes:

1. **Remove** `ekf_filter_node_map` (comment out `ekf.yaml:121-223` as a dead fallback) and
   `navsat_transform_node` from `localization.launch.py` / `navigation.launch.py`. **Keep
   `ekf_filter_node_odom` exactly as-is** (this respects the dual-EKF-over-DIY-EKF prior decision —
   we keep `robot_localization`, we don't hand-roll a filter).
2. **Add `map_odom_broadcaster`** — ~50-line rclpy node publishing `map→odom` = identity on `/tf`
   (NOT `/tf_static` — costmaps need a fresh stamp) at **50 Hz** (≥2.5× our 20 Hz loop; TnTech's
   10 Hz is marginal for us).
3. **GPS → goal pipeline.** *Verifier refinement — do NOT blindly copy TnTech's full 4-node carrot
   chain.* `mission_manager.py` already sends a fixed map-frame goal via `/fromLL`. Once
   `map ≡ odom`, that fixed goal is already stable — most of the carrot re-issue machinery is
   redundant. Instead: (a) add a small `mission_global_frame` projection (lat/lon → local meters
   from a per-site origin, since `/fromLL` disappears with navsat), and (b) **rewire
   `mission_manager.py`** off the now-deleted `/odometry/global` onto `/odometry/filtered`, and off
   `/fromLL` onto the new projection. Only add a carrot/lookahead node if single-shot goals prove
   insufficient in Phase 4-style testing.
4. **Nav2 unchanged.** Keep `bt_navigator.global_frame: map` — it works because `map ≡ odom`. MPPI,
   Navfn, STVL, BT, kiwicampus all run identically.

Hard couplings the verifier caught (budget for these — they're not in a naive node count):
- `mission_manager.py` hard-exits if `/fromLL` is absent and subscribes to `/odometry/global` — both
  vanish with the map EKF. Must rewire or retire it.
- **Datum hygiene.** Live `navsat.yaml` datum is `[42.667925, -83.218195]` (Michigan), *not* the CPP
  value in CLAUDE.md. The new origin param must be set per-site and kept consistent with the
  waypoints file — this is exactly the class of bug that caused the 145° navsat heading offset on
  2026-05-27.
- Confirm no stray `static_transform_publisher` also claims `map→odom`.

Honest limitation: `map ≡ odom` means pure dead-reckoning between course flags. Our odom EKF hit
0.04% over 24.86 m in one benign session → <0.5 m over a ~100 m loop, inside the 2-2.5 m flag
tolerance. The unmodeled long-run risk is **heading drift** (we have no FAST-LIO; our odom ceiling is
wheel-skid + ZED lever-arm + Xsens warmup). Keep a cheap GPS-vs-odom heading sanity check as a
guard. If it ever proves insufficient, the fallback is to re-introduce a slow absolute GPS
re-anchor — but that's Phase 1's tuning, which we'll already have in hand.

### Phase 3 — DWB shadow fallback controller (≈3 hr, anytime)

Not for the starvation case (that was RViz). For the specific *"Optimizer fail to compute path"*
failure in tight barrel clusters where MPPI's 500-sample batch finds no collision-free trajectory.
Config block already drafted in `architecture_decision_global_frame.md:151-158`.

- `controller_plugins: ["FollowPath", "FollowPathFallback"]`; add a `FollowPathFallback` block with
  `plugin: "dwb_core::DWBLocalPlanner"` (Humble name — verify `ros-humble-dwb-core` installed),
  `sim_time: 2.0`, 10×10 samples, standard critics.
- In `navigate_igvc_autonav_humble.xml`, wrap the inner FollowPath in a `ReactiveFallback`: MPPI
  first (`controller_id="FollowPath"`), DWB second (`controller_id="FollowPathFallback"`). Fires only
  when MPPI returns FAILURE; ~zero CPU when idle.

**Do NOT add Spin recovery** — our 2.49 m footprint sweep risks crossing both lane lines in a 2-3 m
corridor (the BT comment already has this analysis; TnTech can Spin only because they're smaller).

### Phase 4 — `back_circle` rear-exclusion costmap layer (≈2-4 hr, pre-competition)

The structural version of Phase 0's BackUp-distance band-aid. Port RoboJackets `back_circle_layer`
as a Nav2 `CostmapLayer` plugin (`avros_rear_exclusion_layer`): in `updateCosts()`, paint
`LETHAL_OBSTACLE` over a half-ellipse behind base_link (offset −0.5 m, length 2.0 m, width 1.5 m,
transformed to odom). Add to the **local** costmap plugins only. Then BackUp's existing
`isCollisionFree()` aborts on its own and Phase 0's conservative distance is no longer the sole
safety margin. (§I.2 note: it's a local-frame virtual costmap layer, no GPS — analogous to inflation;
get a judge ruling only if "positioning system" is read very broadly.)

---

## Recommended execution order

1. **Phase 0** today — pure config/BT, no field session, removes active risk.
2. **Phase 1** next session — localization hygiene, staged with the recovery script armed. Ship it
   regardless of Phase 2; it's correct config and gives a fallback re-anchor.
3. **Phase 2** the main event — `map ≡ odom`. This is the decision you already recorded; the analysis
   just validated it and trimmed the carrot over-build. Budget the `mission_manager`/`/fromLL`
   rewire and datum hygiene explicitly.
4. **Phases 3 + 4** pre-competition hardening, parallelizable with the above.

## What this analysis explicitly did NOT recommend (and why)

- **Odom-frame goals while keeping the map EKF** (the "cheap Option 3") — refuted: planner plans in
  `map`, so the goal is re-projected through the drifting TF anyway. It only works as full `map ≡ odom`.
- **Spin recovery** — unsafe for our footprint in IGVC lanes.
- **Switching MPPI to 10 Hz / heavy batch** — our starvation was RViz, not MPPI; 20 Hz gives more
  timeout margin on variable Jetson load.
- **Porting Hosei's `ekf_myself.py`** — it conflates odom and GPS into one filter (the thing the
  dual-EKF tutorial deliberately separates); take their *goal-frame* insight, not their EKF.
- **Adding RTK as the primary answer** — every fix here works with unaided FIX-only GPS, per §I.2.

---

## Provenance

- Workflow: 11 agents, ~735k tokens, run `wf_edacc1fd-a9b` (2026-05-29). Raw findings + adversarial
  verdicts in the run transcript.
- Code-reads: `github.com/KBKN-Autonomous-Robotics-Lab/orange_ros2`,
  `github.com/LCAS-Lab/TnTechIGVC2026`, `docs.nav2.org` GPS tutorial, `robot_localization` upstream,
  `ros-agriculture/tractor_localization`, local `robojackets/igvc_navigation`.
- Convergent prior: `docs/architecture_decision_global_frame.md` (2026-05-26).
- Empirical anchor: `docs/yaw_diag_session_2026_05_28/nav2_lidar_analysis.md`.
- Verified against live `src/avros_bringup/config/{ekf,navsat,nav2_params_humble}.yaml`,
  `navigate_igvc_autonav_humble.xml`, `src/avros_navigation/avros_navigation/mission_manager.py`.
