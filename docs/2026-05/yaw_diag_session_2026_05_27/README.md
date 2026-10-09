# 2026-05-27 Session Documentation — Read This First

This folder contains the full record of the IGVC yaw-diagnostic + obstacle-avoidance session.

## 🎯 Quick start for the next agent / next session

**Two doors:**

1. **[TOMORROW.md](TOMORROW.md)** — pick up here to continue testing. Has the exact next-session test plan with commands, pass criteria, and cleanup steps.

2. **[SESSION_FINAL.md](SESSION_FINAL.md)** — read this for the executive summary of everything that happened today, including all 5 commits, all findings, and all open items.

If you only have time to read one doc, read **TOMORROW.md** — it's the action plan.

## All documents in this folder

### Top-level (read in this order)
1. **[README.md](README.md)** — this file (you are here)
2. **[SESSION_FINAL.md](SESSION_FINAL.md)** — executive summary of today's results
3. **[TOMORROW.md](TOMORROW.md)** — next-session test plan with exact commands
4. **[VERDICT.md](VERDICT.md)** — multi-agent synthesis verdict (issue #13 refuted)
5. **[SESSION_STATE.md](SESSION_STATE.md)** — chronological state log + "TO DO ON NEXT JETSON BOOT" critical note

### Reference docs
6. **[methodology.md](methodology.md)** — exactly how every measurement was taken
7. **[bags_manifest.md](bags_manifest.md)** — bag inventory with copy/extract commands
8. **[drift_deep_analysis.md](drift_deep_analysis.md)** — answer to "would drift break obstacle avoidance?" (no, but with caveats)

### Per-test snapshots (chronological)
- [M0_snapshot.md](M0_snapshot.md) — stationary baseline
- [M1a_snapshot.md](M1a_snapshot.md) — MPPI catastrophic curve
- [M1b_M1c_snapshot.md](M1b_M1c_snapshot.md) — INVALIDATED (curb hit; flagged in doc)
- [M1b_M1c_retry_3m.md](M1b_M1c_retry_3m.md) — clean 3m retry
- [multi_source_yaw_capture.md](multi_source_yaw_capture.md) — first multi-source IMU/wheel/EKF capture
- [repeatability_failure.md](repeatability_failure.md) — variance > signal investigation
- [clean_stack_diagnosis.md](clean_stack_diagnosis.md) — after switching to yaw_diag.launch.py
- [clean_stack_run2.md](clean_stack_run2.md), [clean_stack_run3.md](clean_stack_run3.md) — repeatability confirmation

### Templates / artifacts
- [verdict_TEMPLATE.md](verdict_TEMPLATE.md) — the template VERDICT.md was built from (kept for re-use in future sessions)

## Things OUTSIDE this folder you'll need

| Where | What |
|---|---|
| `docs/yaw_diag_analysis_strategy.md` | Multi-agent analysis pipeline (5 agents) — re-runnable for any future bag |
| `docs/yaw_diag_decision_thresholds.md` | Quantitative pass/fail criteria per test (M0/M1/M2/M3) |
| `docs/yaw_diag_patches/` | Pre-staged Fix 1/2/4 patch files — **REFUTED, do not apply** |
| `docs/gps_datum_history.md` | History of GPS datums used; includes the typo-and-fix entry |
| `docs/session_2026_05_27_yaw_diag_field_plan.md` | The v2 field plan executed today |
| `scripts/` (laptop + Jetson) | All diagnostic scripts built this session (8 of them) |
| `src/avros_bringup/launch/yaw_diag.launch.py` | Clean test-mode launch file (no Nav2 contention) |
| `bags/yaw_diag_s2_20260527_144425_csv/reports/` | Agent A/C/D/E + navsat re-check reports |

## Bags collected (all preserved on Jetson `/home/dinosaur/IGVC/bags/`)

| Bag | Notes |
|---|---|
| `yaw_diag_s2_20260527_144425` (205 MB) | Multi-source rev+fwd tests — drove all multi-agent findings |
| `yaw_diag_s3_20260527_160356` (~50 MB) | Datum-fix validation |
| `yaw_diag_s3_20260527_161406` (~40 MB) | M2 rotations + M3 square (script bug — disregard M3) |
| `obstacle_avoid_20260527_164508` (9.2 GB) | Full Nav2 + LiDAR; SUCCEEDED at 0.49m off goal |

The first session's bag (`/tmp/yaw_diag_20260527_134442`) was lost on Jetson reboot. Numbers preserved in snapshot docs.

## GitHub issues

| # | Title | State |
|---|---|---|
| 12 | Phase 2 motion data | Advanced — M1/M2 captured today, motion-data goal partly met |
| 13 | EKF yaw under-weights IMU | **CLOSED** as REFUTED by multi-agent analysis |
| 14 | L SparkMAX under-delivers in forward | OPEN — fixed mechanically end-of-session, **validation pending tomorrow** |
| 15 | Xsens 150s warmup operational rule | OPEN — documented, code-gate optional |
| 16 | Live monitor wheel-integration bug | OPEN — fix anytime on laptop |

## Commits made this session (5 total, all on origin/main)

1. `f71d8dc` — navsat: SE Michigan datum (introduced 145° bug — my typo)
2. `27d6b41` — navsat: fix datum YAML (heading_rad in slot 3, not altitude)
3. `e46dc42` — yaw_diag session bundle (scripts, launch, docs, verdict)
4. `681ed60` — flag BT XML revert TODO
5. `a620200` — nav2 costmap STVL decay_acceleration 5.0 → 2.0 (anti-flicker)

## 🚨 Critical reminder for next session

**BT XML on Jetson still has unsafe test-mode values (Timeout 120s, retries 10).** Must revert before nav goals — see step 1 of TOMORROW.md.

One-liner:
```bash
ssh jetson 'cd ~/IGVC && git checkout -- src/avros_bringup/config/navigate_igvc_autonav_humble.xml'
```
