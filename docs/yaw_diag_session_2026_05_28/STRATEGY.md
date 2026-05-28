# IGVC Readiness Strategy — 2026-05-28

Follows the 2026-05-27 yaw-diag session. Builds on `docs/yaw_diag_session_2026_05_27/TOMORROW.md` and `VERDICT.md` (refuted #13). The chassis's dominant defect — L motor 84% fwd delivery — was mechanically fixed end-of-day but **unvalidated in data**.

## TL;DR

- L-motor fix validation is the critical path. Until validated, every 2026-05-27 Nav2 number is suspect (Agent D, `VERDICT.md:23`).
- TOMORROW.md is solid but under-instrumented: gates Pass/Fail on a script with a 5× wheel-yaw bug (issue #16), has no M2 rotation, skips M0.
- Beyond TOMORROW.md, IGVC needs four phases A→D. A finishes 2026-05-28; B/C/D compound on A.
- Hidden risk: mechanical fix could *over-correct* (L > R) — invisible in forward bags but obvious in M2 CCW vs CW. **Add M2 tomorrow.**
- Don't reopen #13. Don't apply `docs/yaw_diag_patches/` — refuted by 4 motion windows (`VERDICT.md:22`).

---

## Critique of TOMORROW.md

### Good
- **Test 1 BT XML revert** is critical (120s/10-retry vs 45s/3-retry — IGVC §I.E DQ risk); one-line checkout per `TOMORROW.md:33`.
- **Test 2 setup** matches `methodology.md`.
- **Pass criteria** (`TOMORROW.md:87-93`) trace to Agent D's 84%/90% baseline.
- **Skip list** (`TOMORROW.md:236-243`) correctly defers lever-arm, GPS noise, P5.1/P5.2.

### Missing
1. **No M2 rotation.** Agent D's strongest L-motor signal was CW ~50% vs CCW ~100% (`SESSION_FINAL.md:71`). Forward distance is a *consequence*; M2 is *direct* differential-RPM evidence. Add 4 M2 variants (`docs/yaw_diag_decision_thresholds.md:71-78`) after Test 2. Costs <5 min; detects "fix overshot."
2. **Pass/Fail gated on buggy live script.** `send_rev_fwd_with_imu.py` over-reports wheel Δyaw 5× during Xsens settling (issue #16). Mitigation: record the bag *before* the script runs; derive Pass/Fail from `scripts/extract_bag.py` + `analyze_M1.py`, not console.
3. **No M0 baseline.** Can't separate "L motor still bad" from "Xsens unsettled" from "/wheel_odom cov regressed." 60s stationary against `docs/yaw_diag_decision_thresholds.md:13-21` thresholds, ~zero cost.
4. **No write-back to issue #14.** TOMORROW.md mentions it (line 230) but doesn't specify content. Post bag-extracted L_meas/L_cmd both directions + L−R asymmetry; close if `|L−R| < 2 pp`.
5. **Test 3 goal location undefined.** "10m forward" from a non-specified start blocks between-day comparison. Pick a fixed map-frame goal or record start pose into bag metadata.
6. **Test 3 pre-flight has elided commands** (`TOMORROW.md:144` ends with `... && ros2 param set` — no source line). Expand to full commands.

### Remove/revise
- **Visual flicker check (`TOMORROW.md:168`) as Pass/Fail.** Subjective single-observer; either compute lethal-cell occupancy from `/local_costmap/costmap` over a stationary obstacle, or demote to informational.
- **"16 cm/s drift" expected value (`TOMORROW.md:190`).** Site-specific to SE Michigan multipath. Replace with "drift ≤ M4 baseline × 5" per `drift_deep_analysis.md:13-15`.

---

## Multi-phase strategy

### Phase A — Post-fix chassis validation (2026-05-28, TOMORROW.md scope + M0/M2)

| Field | Value |
|---|---|
| Goal | L_meas/L_cmd ≥ 92%, BT XML reverted, datum fix holds, STVL `decay_acceleration:2.0` doesn't break costmap |
| Prereqs | Jetson up, code synced (5 commits per `TOMORROW.md:21`), webui systemd stopped |
| Tests | M0 (60s) → Test 1 → Test 2 M1b/c → **NEW M2 ×4** → Test 3 Nav2 obstacle |
| Bag | ~5 min M0/M1/M2 (~80 MB); ~6 min Nav2 (~1.2 GB) |
| Exit | L_meas/L_cmd ≥ 92% fwd AND \|L−R\| < 2 pp AND M2 CW ≥ 90% of CCW AND Nav2 goal < 0.5 m |
| If fail | Stop. Update #14, mechanical re-tune, no Nav2 |

### Phase B — Speed envelope + dead-reckon budget

| Field | Value |
|---|---|
| Goal | Find max safe `vx_max` before path-scale error blows past 1 m / 10 m |
| Prereqs | Phase A passed |
| Tests | M1d (0.7 m/s, 7s ×3) + M3 1×1 m square + same at 0.5 and 0.7 m/s |
| Bag | ~10 min (~200 MB) |
| Exit | Path-scale < 7.5% (`drift_deep_analysis.md:25`) at vx_max=0.5; M3 yaw closing < 10° (`yaw_diag_decision_thresholds.md:122`) |
| If fail | Re-measure Mandow `wheel_separation_multiplier` (operational rule #4 `drift_deep_analysis.md:53`) |

### Phase C — Perception integration

| Field | Value |
|---|---|
| Goal | Lane markings + grass borders become costmap obstacles without walkway-as-pothole false positives (`TODO.md:31`) |
| Prereqs | Phase B passed; ZED mount measured (`TODO.md:21`); HSV thresholds re-tuned |
| Tests | HSV-overlay snapshot on grass+tape; 10 m lane drive; barrel+sawhorse bench |
| Bag | ~15 min (~3 GB) |
| Exit | Lane-tape recall > 80%; walkway false-positive < 2% of frame; no perception-driven aborts |
| If fail | Tighten HSV `pothole_*`; if structural, fall back to LiDAR-only obstacle layer |

### Phase D — Competition rehearsal

| Field | Value |
|---|---|
| Goal | Full IGVC AutoNav course at competition `vx_max`, no operator interventions, no >60 s stalls (§I.E DQ) |
| Prereqs | Phases A/B/C passed; safety light 50 ft check (`TODO.md:16`); RTK FIX confirmed (`TODO.md:27`) |
| Tests | 2-3 mock-course runs at progressive `vx_max` (0.5 → 0.7 → MPPI default), each bagged |
| Bag | ~10 min × 3, ~6 GB total |
| Exit | 2 consecutive successful runs; no recoveries; no `/cmd_vel` gaps > 2 s (`drift_deep_analysis.md:30`) |
| If fail | Run 5-agent pipeline (`docs/yaw_diag_analysis_strategy.md`) on the failing bag |

---

## Prioritized TODO list

### P1 — Blocks tomorrow
- **P1.1** Revert BT XML on Jetson (`TOMORROW.md:33`, commit `681ed60`). Must run before any Nav2 command.
- **P1.2** Validate L motor per Phase A; write `docs/yaw_diag_session_2026_05_28/validation_results.md`; update issue #14.
- **P1.3** Add M2 to Test 2 (this doc, Missing #1).
- **P1.4** Switch Pass/Fail input to offline CSV (`scripts/extract_bag.py` + `analyze_M1.py`), bypassing issue #16's 5× bug.

### P2 — Blocks IGVC readiness
- **P2.1** Phase B path-scale calibration at `vx_max ≥ 0.5`. Decides whether MPPI default 0.7 is safe.
- **P2.2** Fix `live_yaw_monitor.py` + `send_rev_fwd_with_imu.py` integration (issue #16). Without this, every future live call is unreliable.
- **P2.3** ZED mount + GNSS lever-arm measurement (`TODO.md:21-22`). Tighten before Phase C.
- **P2.4** Verify RTK FIX outdoors (`TODO.md:27`); get judge ruling on NTRIP per memory `project_igvc_rtk_rule.md`.
- **P2.5** HSV `pothole_*` tightening (`TODO.md:31`) — Phase C blocker.

### P3 — Operational hardening
- **P3.1** Code-gate Xsens 150s warmup at `/cmd_vel` acceptance (issue #15). Reproducible 2 sessions (`SESSION_FINAL.md:80-82`).
- **P3.2** Confirm no brown-outs in journald post-Phase A. Hardware buck (`TODO.md:15`) is the structural fix.
- **P3.3** Cap `obstacle_layer.observation_persistence` ~4 s in `nav2_params_humble.yaml` (rule #1 `drift_deep_analysis.md:51`). Pre-Phase D.

### P4 — Deferred
- **P4.1** Jetson GitHub auth (`TODO.md:58`).
- **P4.2** Document v2→v3 yaw_diag script migration after #16 fix.
- **P4.3** Re-evaluate `twist0:/filter/twist` removal (`TODO.md:43`).

---

## Risk register

| Risk | Likelihood | Impact | Mitigation | Prior |
|---|---|---|---|---|
| L motor over-corrected (L > R now, CCW curve) | Med | High | M2 CW vs CCW; require both 90-100% | Calibration overshoots post-PID-retune 2026-05-13 |
| Xsens unsettled at t=150s on cold boot | Low | Med | M0 check; if wz σ > 0.15°/s, extend to 300s | `feedback_xsens_imu_bias_recovery`; S2/S3 R1 (`SESSION_FINAL.md:80`) |
| BT XML revert silently didn't take | Low | Critical (DQ) | `grep` verify per `TOMORROW.md:38` | First-time risk; instrumented |
| Brown-out during motor test | Med | High | Charged battery, `vx_max:0.35`, journald on | `TODO.md:15` open; M1a death-spiral 2026-05-13 |
| STVL `decay_acceleration:2.0` over-persists obstacles | Med | Med | Phase A Test 3 catches it; revert to 3.0 if fails | Untested (`SESSION_FINAL.md:76`) |
| Datum YAML typo recurrence | Low | High | `feedback_navsat_datum_yaml_semantics` memory; add comment in `navsat.yaml` above `datum:` | Self-inflicted 2026-05-27 (`VERDICT.md:24`) |
| Different-site GPS multipath | High | Low-Med | Set drift threshold relative to *this day's* M0, not historical | Site-dependent |
| MPPI `batch_size:1000` insufficient at full speed | Low | Med | Foxglove `/diagnostics`; raise to 2000 if CPU headroom | Reduced 2026-05-21 (CLAUDE.md) |
| Working-tree HSV/perception regressions | Med | Low(A)/High(C) | Phase C retests; stash + diff before re-deploy | Open files in `git status` |

---

## Tooling gaps + recommendations

1. Add to `methodology.md` §7: bag-extracted CSV is truth; live numbers advisory. Blocks issue #16 from spawning more false hypotheses.
2. Fix issue #16. Highest leverage; validate against Agent C's CSV.
3. `scripts/check_motor_balance.py` — subscribe `/avros/wheel_debug` 30s, print L/R delivery. ~80 lines, reusable.
4. Add M2 mode to the rev/fwd script so M2 is one-line operator action.
5. Foxglove L/R RPM delta panel in `docs/foxglove_layout.json` — live L motor health during Phase A.
6. Annotate `docs/yaw_diag_decision_thresholds.md` matrix: Fix 1/2/4 refuted 2026-05-27.

---

## Open questions

1. Was the L motor hand-spun on the bench before re-energizing? If not, suggest 0.1 m/s × 2 s pre-test before Test 2.
2. Next field-test date and site? Phase A assumes tomorrow; B/C/D need site allocation. SE Michigan vs CPP shifts GPS drift baselines.
3. Safety light ready for 50 ft daylight check (`TODO.md:16`)? Competition-gating per IGVC §I.2/§I.4.
4. Get judge ruling on NTRIP before Phase D, or assume §I.2 allows it? If ruling goes against, Phase D thresholds need re-derivation for unaided SBAS — `ekf.yaml` rejection thresholds become load-bearing (memory `project_no_rtk_in_codebase.md`).
5. Want a "between-day Δ" script comparing two extracted bags to flag per-source regressions?
