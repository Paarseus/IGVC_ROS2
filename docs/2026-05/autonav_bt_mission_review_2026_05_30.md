<!-- Generated 2026-05-30 by autonav-bt-mission-review workflow (wf_5c8ce64c-31a): 5 dimension reviewers + adversarial verify + synthesis. 36 findings / 33 confirmed / 3 refuted. Verified against HEAD b798437 (dbca3f6 frame restore). -->

# IGVC AutoNav BT + mission_manager Review

Scope: `navigate_igvc_autonav_humble.xml`, `mission_manager.py`, and their Nav2 / EKF / launch context, against the IGVC 2026 AutoNav rules. All findings below were re-verified against the **current HEAD** (commit `dbca3f6`), which is newer than the `0928b9f` snapshot several input findings were checked against — this changes several conclusions (see the Frame section).

---

## 1. Verdict — is the AutoNav BT + mission_manager correct?

**Field-testing: YES. Competition: NOT YET — two one-line reverts and one config decision stand between this and ready.**

The architecture is fundamentally sound and matches the blessed upstream Nav2 pattern:

- The BT is a faithful copy of `navigate_to_pose_w_replanning_and_recovery.xml` topology: `Timeout > RecoveryNode > [PipelineSequence(RateController-gated planner ‖ free-running FollowPath), ReactiveFallback > RoundRobin(clear/wait/backup/crawl)]`. Plugin registration, action/goal-checker IDs, costmap service names, and blackboard wiring all verified clean — the tree **will load and activate** on the Jetson.
- The recovery escalation **works**: an input finding claimed `RecoveryNode.haltChild(1)` resets the `RoundRobin` index so only `ClearCostmap` ever runs — that was **refuted** by the skeptic (BT.CPP only propagates `halt()` to RUNNING children; at the SUCCESS tick the `ReactiveFallback` is not RUNNING, so the index persists). `Wait`/`BackUp`/`DriveOnHeading` are reachable. The header's escalation description is correct.
- `mission_manager.py` is a legitimate minimal orchestrator: waypoint cursor + 2.0 m proximity advance + skip-on-failure + `/autonomous_mode` safety-light latch. The cursor/result race is carefully guarded (`index != self._cursor` filter at line 203). The §I.2 safety light is correctly implemented and rules-compliant.
- **Frames are now consistent** (HEAD `dbca3f6`): goals, proximity, planner, and global costmap are all in `map`, RTK-pinned. The "map-goal / odom-planner split" that the input findings flagged as critical **no longer exists at HEAD**.

What blocks competition: two committed field-test BT values that are explicitly tagged "REVERT for competition" but still live (`Timeout=120000`, `retries=8`), an inflation radius that gives essentially no barrel standoff, and a stack-wide dependence on RTK staying FIXED with no software fallback if it degrades.

---

## 2. Critical & High findings (confirmed/partial only)

| # | Sev | Title | Location | Why it matters for IGVC | Fix |
|---|-----|-------|----------|-------------------------|-----|
| H1 | **High** | Outer `Timeout=120000 ms` shipped (competition value 45000) | `navigate_igvc_autonav_humble.xml:72` | A single wedged waypoint lets MPPI churn up to 120 s before `NavigateToPose` FAILUREs and mission_manager skip-advances (`mission_manager.py:211-218`). That is 2× the 60 s on-course-stall window (II.4 #8 Blocking Traffic = E-stop + measured, −5 ft) and burns a third of the 360 s run budget. Committed `0928b9f`, self-flagged REVERT. | Revert `msec="45000"`. Bundle with H2 — the header's recovery budget math only holds with both reverted. |
| H2 | **High** | `RecoveryNode number_of_retries=8` (competition value 3) | `navigate_igvc_autonav_humble.xml:78` | Each retry cycles the RoundRobin one action; 8 retries = ~2 full escalation cycles (~13 s of stand-still/reverse motion) + 8 replan attempts of near-zero progress, dragging the whole-run average toward the 1 mph DQ floor (II.4 #10 = Disqualified, no warning). The course is finishable at only ~0.95 mph nominal, so slack is thin. Committed `0928b9f`, self-flagged REVERT. | Revert `number_of_retries="3"`. The "8 retries = clear-only dead time" mechanism in one input finding was refuted (RoundRobin does escalate); the value is still wrong for competition. |
| H3 | **High** | `inflation_radius 0.3 m` ≈ footprint inscribed (0.2794 m) → ~2 cm avoidance gradient, ~no barrel standoff | `nav2_params_humble.yaml:481` (local), `:620` (global); footprint `:266/:516` | With `cost_scaling_factor 3.0`, inflated cost falls off over only `0.30 − 0.2794 ≈ 0.02 m` past the footprint edge — MPPI sees free space until nearly touching a lethal cell. IGVC II.2 wants 5 ft (1.5 m) standoff; II.4 penalizes Crash (−10 ft + E-stop) and Sideswipe (−5 ft). The team's own same-day commit `582b73f` restored 1.0/0.65 *because* 0.3 clipped a barrel in field test, then `0928b9f` re-dropped it. The line comment itself concedes "~no barrel standoff." | Restore local 1.0 / global 0.65 (the 05-21 working values) **but only together with a larger near-obstacle goal tolerance** (H4) — otherwise wide inflation + 0.2 m goal tol makes near-obstacle waypoints UNREACHABLE (the documented North/5002 failure). If 0.3 is kept for narrow-lane fit, make it a conscious documented go/no-go trade. |
| H4 | **High** | GPS health watchdog claimed by header is entirely unimplemented | `navigate_igvc_autonav_humble.xml:19-20` (claim) vs `mission_manager.py` (no `/gnss` sub, no dead-reckon/forward-bias) | The stack now depends on RTK FIXED: `ekf.yaml:186` fuses GPS in ABSOLUTE mode and `enable_ntrip` defaults **false** (`navigation.launch.py:135`). The map-EKF rejection gate (`ekf.yaml:194` = 13.8) is tuned for SBAS, not FLOAT/dropout (the `82f5fd3` comment admits the dropout re-tune is deferred). In No-Man's-Land (rules 144-147) there are **no lanes** to fall back on — pure GPS-waypoint guidance to the ramp. If RTK degrades mid-run, the map walks and nothing reacts. | Add a minimal watchdog: track time-since-good-fix on `/gnss`; on degraded, keep creeping forward (project a short carrot along current heading from `/odometry/global`) instead of chasing a drifting map point. Single highest-value missing feature. ~40 lines (watchdog logic) plus frame plumbing. |

Note: several input findings rated H1/H2/H3-equivalents as **critical**; the skeptics consistently downgraded to **high** because each is a one-line, self-flagged, known-TODO revert that does not break a *nominal* run — only the failure/recovery path. I follow that calibration.

---

## 3. Design-intent gap (T1) — header promises features mission_manager does not implement

The BT header (`navigate_igvc_autonav_humble.xml:14-24`) asserts a "smart orchestrator" owning four subsystems. Verified against `mission_manager.py` (the only orchestrator node in `avros_navigation`; grep across the whole package confirms absences):

| Claimed (header line) | Implemented? | Needed to finish an AutoNav run? | Risk if absent |
|---|---|---|---|
| Waypoint cursor, 2.0 m proximity advance (17) | **YES** | Required | None — works, race-guarded |
| No-Man's-Land state machine / per-waypoint metadata (18) | **NO** | **No** — plain sequential `NavigateToPose` to the GPS waypoint pairs satisfies the rule (entrance/exit + 2 ramp-guide points, rules 144-147). `waypoints.yaml` has no metadata schema and `_load_waypoints` parses only lat/lon. | Low for the rule itself; but it is the missing *substrate* the per-waypoint tolerance/vx-cap fixes would key off. |
| GPS health watchdog: dead-reckon @2 s, forward-bias @30 s (19-20) | **NO** | **Yes, genuinely needed** | **High** — see H4. The one header-claimed feature that is real safety, not gold-plating. |
| SetParameters side-effects: lane toggle, inflate goal xy_tol, cap MPPI vx_max on ramp (21-22) | **NO** | Mixed: lane toggle = unnecessary (lanes are deliberately LETHAL always-on, correct); **xy_tol inflate = the documented fix for the North/5002 near-obstacle UNREACHABLE failure (`nav2_params:70`)**; vx_max ramp cap = optional (no field evidence the 15% ramp needs it; actuator clamps top speed anyway). | Medium — the missing xy_tol inflate has a proven field consequence; the ramp cap and lane toggle do not. |

The header partially self-discloses this — line 16 says "(to be implemented)" — but lines 18-22 then describe the features in the present tense as things the orchestrator "owns." **Fix the header first** (cheap, zero-risk) so no operator assumes these safeguards run, then prioritize the GPS watchdog (H4) and per-waypoint xy_tol as the real feature work.

---

## 4. Frame & GPS correctness (T2) — RESOLVED at HEAD, not a split

**This is the most important correction to the input findings.** Every input finding alleging a "map-goal / odom-planner split → goal-recession runaway" (multiple high/critical items, dimensions mission-manager / rules-compliance / design-vs-impl / competition-readiness) was verified against commit `0928b9f`. **The current HEAD is `dbca3f6`, one commit newer, which explicitly RESTORED `global_frame: map`:**

- `nav2_params_humble.yaml:21` `bt_navigator.global_frame: map`
- `nav2_params_humble.yaml:502` `global_costmap.global_frame: map`
- `nav2_params_humble.yaml:644` `route_server.global_frame: map`
- `nav2_params_humble.yaml:243` `local_costmap.global_frame: odom` (correct — robot-centric rolling per REP-105)
- `mission_manager.py:45` `GOAL_FRAME_ID='map'`; proximity reads `/odometry/global` (= map-EKF output, `localization.launch.py:126`)

So at HEAD, **goals, the proximity check, the planner, and the global costmap are all in the same RTK-pinned `map` frame.** This is the standard Nav2 / GPS-waypoint-follower topology, is internally consistent, and matches the team's documented clean run to waypoint 5004 (2026-05-30). The `5e6f279` odom workaround (for unaided-GPS drift) was superseded by `82f5fd3` (absolute RTK fusion pins map) and finalized by `dbca3f6`.

**Conclusion on T2: sending goals in `map` is CORRECT at HEAD.** Do NOT switch `GOAL_FRAME_ID` to `odom` — that would reintroduce the 0.68 m stop-short bug (a frozen odom goal ignores the wheel-slip map→odom correction). The proposed "odom carrot" fixes in the input findings are stale and would regress the validated config.

**The genuine, narrower residual concerns (verified):**
1. **RTK-degradation robustness (real, high — H4):** the whole map-frame design is correct *while RTK is FIXED*. `ekf.yaml:186` (absolute fusion) + `enable_ntrip` default false + a SBAS-tuned rejection gate (`ekf.yaml:194`) means an RTK→FLOAT/dropout transition lets the map frame absorb meter-scale GPS noise, and a map-stamped goal tracks that wander. This is a localization-config + missing-watchdog gap (H4), **not** a goal-frame bug.
2. **FromLL caches once at startup (`mission_manager.py:128-152`):** with a FIXED navsat datum, re-projecting the same lat/lon yields identical map coords, so per-tick re-projection (proposed by one input finding) is a **no-op** — **refuted**. The cache is correct as long as the datum is fixed (it is). Caching is fine; the input "stale cache" finding does not apply under the current absolute-fusion config.
3. **`enable_ntrip` default false (`navigation.launch.py:135`)** is arguably the IGVC §I.2-compliant default (own base stations forbidden), but it is coupled to the absolute-RTK assumption — put it in the checklist as a conscious per-run decision.

---

## 5. Competition-config landmines (T3) — revert checklist

There is **no central go/no-go config document**, and the two riskiest values (goal tolerance, inflation) carry **NO `REVERT` comment**, so a pre-run `grep REVERT` pass misses them. Under 1-minute start-line prep (II.3), partial reverts are near-certain. Create `docs/competition_config_checklist.md` (or a git tag `competition-config`) with exactly:

| File:line | Param | Current (field-test) | Competition target | Has REVERT marker? |
|---|---|---|---|---|
| `navigate_igvc_autonav_humble.xml:72` | `Timeout msec` | `120000` | `45000` | ✅ yes |
| `navigate_igvc_autonav_humble.xml:78` | `RecoveryNode number_of_retries` | `8` | `3` | ✅ yes |
| `nav2_params_humble.yaml:481` | local `inflation_radius` | `0.3` | **decide** (1.0 for standoff, or keep 0.3 for narrow-lane) | ❌ **no marker** |
| `nav2_params_humble.yaml:620` | global `inflation_radius` | `0.3` | **decide** (0.65, or keep 0.3) | ❌ **no marker** |
| `nav2_params_humble.yaml:70` | `xy_goal_tolerance` | `0.2` | **decide** — must increase if inflation is raised (else near-obstacle UNREACHABLE) | ❌ **no marker** |
| `navigation.launch.py:135` | `enable_ntrip` | `false` | confirm + RTK FIXED state | n/a |
| `nav2_params_humble.yaml:21/502/644` | `global_frame` | `map` (HEAD) | confirm `map` is intended for the RTK plan | n/a (reconciled `dbca3f6`) |

Also reconcile the BT header documentation drift when reverting (see L1 below). Resolve the inflation/goal-tol coupling decision *before* the checklist is meaningful — restoring inflation without raising goal tolerance reintroduces the North/5002 UNREACHABLE failure.

---

## 6. Medium / Low & deliberate-choice / refuted notes

**Confirmed medium/low (real, lower-priority):**
- **M1 — `GoalUpdated` recovery guard is decorative** (`navigate_igvc_autonav_humble.xml:126`). mission_manager cancels+resends fresh goals (`mission_manager.py:272` then new goal); it never *preempts* a running goal, so the blackboard goal never changes mid-execution and `GoalUpdated` never fires. Harmless (recovery still runs; cancel-based interruption is prompt). Fix: drop the node or rewrite the header's "2 Hz carrot" claim (the real cadence is a 5 Hz proximity *check*, not a goal republish). Independently logged as N18 in `docs/fresh_audit_2026_05_29.md`.
- **M2 — no software 1-mph floor + start-line dead time** (`nav2_params:106` vx_max 0.7; `navigation.launch.py:261` 20 s TimerAction). Cruise = 1.57 mph leaves ~123 s of whole-run recovery slack, so "a couple recoveries → DQ" is overstated. The genuinely tight constraint is the **44 ft / 30 s start gate** (exactly 0.447 m/s): the 20 s mission_manager timer + GPS first-fix can eat the start clock. Fix: trigger the run only *after* mission_manager logs its first goal-accept. Do NOT blindly raise vx_max — 0.7 is the deliberate grass-traction value.
- **M3 — BT header advertises a 2 Hz goal loop / 45s-3retry / owned subsystems that don't match reality** (`:14-24, 27, 32, 81-83`). Pure documentation drift; the tree runs correctly. Fix with the header rewrite in §3.
- **L1 — header recovery-action arithmetic stale**: BackUp `:43` reads "@0.08 m/s (3.75 s)" but executes @0.10 (`:167`) = 3.0 s; DriveOnHeading header (`:46`) is correct. Reconcile when reverting the timeout/retries.
- **L2 — skip-on-failure rationale comment is wrong** (`mission_manager.py:213-214` says "scores by waypoint count"). AutoNav scores finishers by shortest adjusted *time*, non-finishers by longest *distance* (rules 117-119, 243-245). Behavior is fine (skip keeps accumulating down-course distance); only the comment is wrong, and skipping a No-Man's-Land *entrance* gate could make the paired *exit* goal geometrically invalid — consider retry-not-skip for tagged gates.
- **L3 — blind 0.3 m BackUp can cross a rear LANE line** (`:166-169`). Camera is front-only, LiDAR can't see flat tape; barrels behind ARE covered by the 360° Velodyne. The input finding's "no costmap cells behind base_link" is overstated — it's rear *lane* cells only. 0.3 m cap is a good guard; consider reordering so `DriveOnHeading` (forward, lane-guarded) precedes `BackUp`.
- **M4 — no ramp vx cap** (header `:22` claims it; unimplemented). `two_d_mode:true` flattening the ramp in TF is **deliberate and correct** (LiDAR range avoids the ramp by design — cleared, not a bug). The only gap is no speed reduction on the ≤15% grade; low priority, implement only if ramp testing shows over-run.

**Refuted / cleared (checked and dismissed):**
- ❌ **"RecoveryNode halts RoundRobin each retry → Wait/BackUp/DriveOnHeading are dead code."** Refuted — BT.CPP only halts RUNNING children; index persists; escalation works. The upstream canonical BT uses the identical topology.
- ❌ **"map-goal / odom-planner frame split → recession runaway."** Refuted at HEAD — `dbca3f6` restored `global_frame:map` everywhere relevant; frames are consistent. (This dissolves ~6 input findings that were verified against the older `0928b9f`.)
- ❌ **"FromLL one-shot cache goes stale → re-project per tick."** Refuted — fixed datum makes re-projection a no-op; cache is correct.
- ❌ **"RateController hz=1.0 starves replanning."** Cleared — it gates only the planner; FollowPath/MPPI run at 20 Hz independently. Canonical, correct.
- ❌ **"Pothole avoidance unverified / could be ignored as free."** Largely cleared — the active `sooner25` pipeline maps all non-asphalt (incl. white circles) to `lane_white` (LETHAL) by construction; a white pothole cannot be labeled free. Perception-test verification item, not a nav bug.

**Deliberate team choices — cleared, not flagged as bugs:**
- Lanes kept LETHAL (cost 254) — intentional (stall-safe > line-touch DQ).
- Front ZED 15° down-tilt — intentional/measured.
- `two_d_mode:true` flattening the ramp + short LiDAR `obstacle_range` so Velodyne never false-marks the ramp deck — intentional.
- `vx_max 0.7` (grass-traction firmware ceiling) — intentional; do not raise blindly.
- Plugin registration, IDs, behavior-server accel limits — verified clean.
- §I.2 safety light (solid→flash→solid, e-stop forces solid) — verified rules-compliant.

---

## 7. Prioritized action list

1. **Revert the two committed field-test BT knobs** — `navigate_igvc_autonav_humble.xml:72` `Timeout 120000→45000` and `:78` `retries 8→3`. (H1, H2)
2. **Decide and pin the competition inflation + goal-tolerance pair** — `nav2_params_humble.yaml:481/:620` inflation 0.3→(1.0/0.65 for barrel standoff) and `:70` `xy_goal_tolerance 0.2→` larger near obstacles; they MUST move together or near-obstacle waypoints go UNREACHABLE. (H3, H4-adjacent)
3. **Create the competition-config checklist** (`docs/competition_config_checklist.md` or git tag) covering all rows in §5, including the unmarked inflation/goal-tol/enable_ntrip values. (T3)
4. **Rewrite the BT header (`:14-24, 27, 32, 43, 81-83`)** to describe what mission_manager actually does and mark No-Man's-Land SM / GPS watchdog / SetParameters as NOT-YET-IMPLEMENTED; fix the BackUp 0.08→0.10 / 45s-3retry drift. (T1, M3, L1)
5. **Add the GPS-health watchdog / forward-creep fallback** to mission_manager for RTK FLOAT/dropout in No-Man's-Land — the highest-value missing feature. (H4)
6. **Implement per-waypoint xy_goal_tolerance inflation** in mission_manager (the documented fix for the North/5002 near-obstacle abort, deferred at `nav2_params:70`). (T1)
7. **Confirm frame intent + RTK plan**: `global_frame:map` (HEAD `dbca3f6`) is correct for the RTK-FIXED design — verify RTK availability with the judges in writing and that `enable_ntrip` is set appropriately for the run. (T2)
8. **Procedural**: trigger the autonomous run only after mission_manager logs its first goal-accept, so the 30 s/44 ft start gate doesn't start while the 20 s TimerAction / GPS lock is still running. (M2)
