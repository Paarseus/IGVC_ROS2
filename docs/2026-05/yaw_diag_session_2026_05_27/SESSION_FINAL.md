# Session Final — 2026-05-27

Complete record of today's IGVC yaw-diagnostic + obstacle-avoidance session. **For tomorrow's continuation, see [TOMORROW.md](TOMORROW.md).**

---

## TL;DR

**Started with:** issue #13 (EKF yaw under-weighting hypothesis) and unknown navigation reliability at SE Michigan deployment.

**Ended with:**
- **Issue #13 REFUTED** by multi-agent analysis (EKF correctly tracks IMU; pre-staged Fix 1/2/4 patches NOT applied)
- **Self-found and fixed: navsat datum YAML typo** (altitude was in heading_rad slot → 145° map-frame rotation; one-line fix in commit `27d6b41`)
- **L motor under-delivery diagnosed (Agent D)** — L motor delivers 84% RPM in forward only; **user fixed mechanically end-of-session** (validation pending tomorrow)
- **Costmap STVL anti-flicker re-tuned** (decay_acceleration 5.0 → 2.0) — committed but not yet field-validated
- **Obstacle avoidance verified working** — Nav2 + LiDAR successfully reached 10m goal within 0.49m, navigated around live human obstacle
- **3 follow-up issues filed** (#14 L-motor kFF, #15 Xsens 150s warmup, #16 live-monitor integration bug)

---

## Commits made this session (chronological)

| SHA | Branch state | Description |
|---|---|---|
| `f71d8dc` | local only | navsat: SE Michigan datum (had altitude-in-heading-slot bug) |
| `27d6b41` | pushed | navsat: fix datum YAML — slot 3 is heading_rad, not altitude |
| `e46dc42` | pushed | yaw_diag session bundle (8 scripts, yaw_diag.launch.py, 16 docs, 3 patches) |
| `681ed60` | pushed | flag BT XML revert TODO for next Jetson session |
| `a620200` | pushed | nav2 costmap: STVL decay_acceleration 5.0 → 2.0 (anti-flicker) |

**All 5 commits on origin/main.** Total: 1 self-inflicted bug found+fixed in same session; 4 substantive improvements.

---

## Test sessions (chronological)

### Session 1 — yaw_diag.launch.py creation + first multi-source tests
- Bag: `/tmp/yaw_diag_20260527_134442` — **LOST** (Jetson rebooted, /tmp wiped)
- Tested: M1c reverse + M1b forward at 5m (curb contamination) and 3m (clean)
- Found: integration bug in live `send_rev_fwd_with_imu.py` over-reported wheel Δyaw 5×

### Session 2 — clean stack after creating yaw_diag.launch.py
- Bag: `/home/dinosaur/IGVC/bags/yaw_diag_s2_20260527_144425` (205 MB) — PRESERVED
- 3 multi-source rev+fwd runs on clean stack
- Run 1: showed huge IMU(quat) vs wheel disagreement (turned out to be Xsens settling time)
- Runs 2+3: clean, IMU and EKF agree within 1°
- Multi-agent analysis of this bag drove all the main findings of the day

### Session 3 — datum-fix validation + M2/M3
- Bags: `yaw_diag_s3_20260527_160356`, `yaw_diag_s3_20260527_161406` — PRESERVED
- After applying commit 27d6b41, /odometry/gps bearing offset dropped from -145° to +0.1° ✅
- M2 rotation tests revealed CW under-delivers ~50% vs CCW ~100% (confirms L motor weakness)
- M3 closing square via cmd_vel had a script bug (180° rotations instead of 90°) — not re-run

### Session 4 — Obstacle avoidance with full nav2
- Bag: `/home/dinosaur/IGVC/bags/obstacle_avoid_20260527_164508` (9.2 GB, persisted) — PRESERVED
- Launched navigation.launch.py with Velodyne ON, capped vx_max to 0.35
- 150s wait for Xsens settle
- 10m forward goal sent, **succeeded** within 0.49m of goal at t=599s
- BT recovered through 3+ retries (the new BT with 10 retries / 120s timeout was active)
- Drift analysis confirmed: drift events coincided with intentional avoidance turns; did not cause failures

---

## Findings ranked by impact

### 🔴 P1 — Critical (fixed today)

1. **Navsat datum YAML semantics** — `datum: [lat, lon, heading_rad]` NOT `[lat, lon, altitude]`. Bug introduced in commit `f71d8dc`, fixed in `27d6b41`. Caused 145° map-frame rotation of /odometry/gps. Verified eliminated post-fix.

2. **L motor delivers 84% RPM in forward only** (R 90%, both 90% in reverse). Causes forward distance under-delivery (43% of commanded), CW rotation under-delivery (~50% vs CCW 100%), and chassis curving on forward drives. **User fixed mechanically end-of-session.** Validation pending tomorrow.

### 🟡 P2 — Important (improvements pushed, untested)

3. **STVL costmap flicker** — sparse VLP-16 + STVL frustum decay produced visible "fade-and-restore" pattern. Tuned `decay_acceleration: 5.0 → 2.0` in both local + global (commit `a620200`). Field validation pending.

4. **BT XML test-mode values** still on Jetson — Timeout 120000ms + retries=10. Must revert (one-line `git checkout`) before competition (60s IGVC DQ rule). See [SESSION_STATE.md](SESSION_STATE.md) "TO DO ON NEXT JETSON BOOT".

### 🟢 P3 — Operational

5. **Xsens IMU 150s warmup rule** (issue #15) — every "Run 1" within ~150s of launch showed catastrophic chassis curve. Reproducible across 2 sessions. Document or code-gate `/cmd_vel` acceptance.

6. **GPS multipath at SE Michigan** is 10× M4 baseline (3.7m peak-to-peak vs 0.36m). Stationary map→odom drift is 9-13× M4 baseline. **Environmental, not a code issue.** Doesn't break navigation — within margins. Documented in drift_deep_analysis.md.

### 🔵 P4 — Lower priority

7. **Live monitor wheel-integration bug** (issue #16) — `live_yaw_monitor.py` and `send_rev_fwd_with_imu.py` reported wheel Δyaw 5× too large during Xsens settling windows. Caused multi-hour false hypothesis chase early in session. Fix anytime.

8. **Xsens stationary yaw jumps** — observed 36° yaw "snap" while robot was stationary (obstacle_avoid bag t≈700s). Magnetometer recovery / ZRU snap. Doesn't break anything but unexpected.

9. **Lever-arm `[0,0,0]`** in xsens.yaml — GPS antenna offset not measured. Not biting today (all straight drives) but will show in long curves.

---

## Multi-agent analysis outcomes

Four agents ran on session-2 bag CSVs (later: a 5th for navsat re-check, a 6th for drift analysis):

| Agent | Verdict | Key finding |
|---|---|---|
| A — Sensor Health | ✅ PASS | All sensors healthy; Xsens quat takes 150s to settle |
| C — EKF Behavior | ✅ **ISSUE_13_REFUTED** | r_imu > r_wheel in all 4 motion windows; rejection gates dormant |
| D — Actuator Fidelity | 🎯 L motor 6% weaker in forward only | Direction-asymmetric; recommended kFF +7% bump |
| E — Map-EKF / GPS | ⚠️ 180° claimed (later corrected to 145°) | Drift 6× M4; navsat-yaw bug found |
| Re-verify (navsat) | ⭐ ROOT CAUSE FOUND | `datum`'s 3rd slot is heading_rad, not altitude — self-inflicted typo |
| Drift Deep | ✅ Drift NOT breaking nav | Peak 55cm/s spike during turns; obstacles older than ~4s start smearing past inflation but STVL decay (5s) catches them |

All reports in `bags/yaw_diag_s2_20260527_144425_csv/reports/` + `agent_navsat_recheck.md` + `drift_deep_analysis.md` (this folder).

---

## Bags collected (all on Jetson persistent disk)

| Path on Jetson | Size | Status | Used for |
|---|---|---|---|
| ~~/tmp/yaw_diag_20260527_134442~~ | LOST | wiped on Jetson reboot | session 1 — data only in snapshot docs |
| `/home/dinosaur/IGVC/bags/yaw_diag_s2_20260527_144425` | 205 MB | preserved | session 2 — multi-agent analysis source |
| `/home/dinosaur/IGVC/bags/yaw_diag_s3_20260527_160356` | ~50 MB | preserved | session 3 first (datum-fix validation) |
| `/home/dinosaur/IGVC/bags/yaw_diag_s3_20260527_161406` | ~40 MB | preserved | session 3 M2/M3 |
| `/home/dinosaur/IGVC/bags/obstacle_avoid_20260527_164508` | 9.2 GB | preserved | session 4 — full Nav2 + LiDAR |

CSVs on laptop at `~/IGVC_ROS2/bags/*_csv/`:
- yaw_diag_s2_20260527_144425_csv (43 MB)
- yaw_diag_s3_20260527_160356_csv (~20 MB)
- obstacle_avoid_20260527_164508_csv (43 MB)

---

## Issues touched

**Closed:**
- `#13` EKF yaw under-weighting — REFUTED with multi-agent evidence

**Opened:**
- `#14` L SparkMAX under-delivers 6% RPM in forward direction only — **chassis-fixed today, validate tomorrow**
- `#15` Xsens IMU 150s settling — operational rule needed
- `#16` Live monitor wheel-integration bug — fix anytime

**Advanced (from #12):**
- Phase 2 motion data captured for M1 and M2 (M3 had a script bug)
- Phase 3 (local EKF integrity) unblocked

---

## Documentation index for this session

Read order for someone catching up:
1. **`SESSION_FINAL.md`** (this file) — high-level summary
2. **`TOMORROW.md`** — next-session test plan
3. **`VERDICT.md`** — multi-agent synthesis verdict
4. **`drift_deep_analysis.md`** — drift question answered
5. **`methodology.md`** — how every measurement was taken
6. **`SESSION_STATE.md`** — chronological state log with all checkpoints
7. **`bags_manifest.md`** — bag inventory with copy/extract commands
8. Per-test snapshots: `M0_snapshot.md`, `M1a_snapshot.md`, `M1b_M1c_retry_3m.md`, `multi_source_yaw_capture.md`, `clean_stack_diagnosis.md`, `clean_stack_run2.md`, `clean_stack_run3.md`
9. `repeatability_failure.md` — variance investigation
10. `M1b_M1c_snapshot.md` — INVALIDATED (curb contamination, flagged)

Agent reports: `bags/yaw_diag_s2_20260527_144425_csv/reports/agent_{a,c,d,e,navsat_recheck}.md`

Pre-staged fix patches (NOT applied — refuted by analysis): `docs/yaw_diag_patches/{fix1,fix2,fix4}_*.patch`
