# Yaw Diagnostic Session — 2026-05-27 — Master State Doc

**Purpose:** single source of truth for this field session. If the conversation context is lost/corrupted, this doc + the artifacts in this folder are sufficient to pick up where left off.

**Issues tracked:** [#13 EKF yaw under-weighting](https://github.com/Paarseus/IGVC_ROS2/issues/13) (primary), [#12 Phase 2 motion data](https://github.com/Paarseus/IGVC_ROS2/issues/12) (secondary).

---

## Current state (auto-updated as session progresses)

| Item | Status | Detail |
|---|---|---|
| Jetson SSH | ✅ OK | `ssh jetson` (100.93.121.3 Tailscale, user `dinosaur`) |
| Jetson git HEAD | ✅ `4cfa1bf` (will become `f71d8dc` on next pull) | Synced 2026-05-27 P0.16 |
| Laptop git HEAD | ✅ `f71d8dc` | Datum committed but NOT pushed to remote yet |
| New GPS datum | ✅ `[42.658430417, -83.241993772, 247.578]` | SE Michigan / Detroit metro, sampled 200 fixes, ~1m spread |
| webui systemd | ✅ stopped | sudo NOPASSWD verified |
| Navigation stack | ✅ running | `enable_velodyne:=false`, ZED + perception + ntrip all off |
| IMU stationary bias | ✅ 0.00346 °/s | Even better than #12 M4 baseline (0.0042 °/s) |
| Bag recording | 🟢 IN PROGRESS | `/tmp/yaw_diag_20260527_134442` on Jetson |
| M0 baseline | ✅ PASS | IMU bias +0.0143 °/s, wheel exact 0, EKF -0.000012 rad/s. [M0_snapshot.md](M0_snapshot.md) |
| M1a | ⚠️ ABORTED with curve | MPPI death-spiral; robot rotated -99° while chasing 5m goal; sensors agreed on the curve. [M1a_snapshot.md](M1a_snapshot.md) |
| M1b + M1c (5m) | ⚠️ INVALIDATED | Reverse hit a curb mid-drive; data invalid. [M1b_M1c_snapshot.md](M1b_M1c_snapshot.md) — flagged in doc |
| M1b + M1c (3m clean) | ✅ COMPLETED | Forward yaw drift -0.42 °/m, reverse +0.12 °/m. Different sign than v2 run below — surface/heading variance. [M1b_M1c_retry_3m.md](M1b_M1c_retry_3m.md) |
| M1b + M1c v2 (multi-source Run 1) | ⚠️ Single-shot, NOT REPRODUCIBLE | Initial reading suggested -3.37° IMU-wheel gap. See Run 2 below — variance is large. [multi_source_yaw_capture.md](multi_source_yaw_capture.md) |
| M1b + M1c v2 (multi-source Run 2) | ✅ COMPLETED — **variance > signal** | Same commands as Run 1; IMU-wheel gap dropped to -0.69°, wheel Δyaw fell from +4.79° to +0.81° (5× variance). **Cannot draw conclusions about issue #13 from single-shot data.** Need N=5-10 repeats OR longer-duration tests OR different motion mode. [repeatability_failure.md](repeatability_failure.md) |
| M1d | ⏳ pending | High-speed forward (0.7 m/s) — does asymmetry scale with speed? |
| **Jetson — Session 1** | 🚨 CRASHED ~14:13 UTC | Brown-out, recovered with manual power-cycle. /tmp wiped on boot → session-1 bag LOST. Snapshot docs preserve key numbers. |
| **Jetson — Session 2** | ✅ UP, clean stack 14:44 UTC | Running `yaw_diag.launch.py` (no Nav2). `/cmd_vel` publisher count = 0 (was 4 in session 1). Topic rates all healthy. |
| Bag — Session 1 | ❌ LOST | `/tmp/yaw_diag_20260527_134442` wiped on reboot |
| Bag — Session 2 | 🟢 RECORDING | `/home/dinosaur/IGVC/bags/yaw_diag_s2_20260527_144425` — on persistent disk |
| S2 M0 baseline | ✅ PASS | IMU bias 0.0073°/s (2× better than S1), wheel exact 0, EKF clean |
| S2 multi-source M1c+M1b Run 1 | ⚠️ COMPLETED — likely Xsens stuck-bias | M1b forward IMU(quat)=-13.22°. Wheel ω shows +33° false rotation reverse. [clean_stack_diagnosis.md](clean_stack_diagnosis.md) |
| S2 multi-source M1c+M1b Run 2 | ✅ COMPLETED — chassis behaves well | All sources agree within ~2°. M1b forward IMU(quat)=-0.99°. Run 1's large readings appear to be a transient Xsens incident — recovered. [clean_stack_run2.md](clean_stack_run2.md) |
| S2 multi-source M1c+M1b Run 3 | ✅ COMPLETED — cleanest run | All sources within 1°. IMU(quat) +0.18° reverse, +0.40° forward. Confirms Runs 2+3 are the normal behavior, Run 1 was outlier. **Issue #13 REFUTED. Real issue is forward distance under-delivery (43% vs 70% reverse).** [clean_stack_run3.md](clean_stack_run3.md) |
| Bag analysis (4 agents in parallel) | ✅ COMPLETED | A=PASS, C=ISSUE_13_REFUTED (r_imu > r_wheel everywhere), D=L_motor_weak_in_forward (84% vs R 90%), E=navsat_yaw_180_inverted_NEW_BUG + 6× M0 drift regression. See [VERDICT.md](VERDICT.md). |
| **Synthesis verdict** | ✅ **CLOSE #13 as REFUTED** | Multi-agent verdict: don't apply Fix 1/2/4. Real fixes: bump L SparkMAX kFF +7% and investigate navsat 180° yaw flip. 5 follow-up issues identified. |
| navsat datum bug found | ✅ Self-inflicted (commit f71d8dc): altitude 247.578 in heading slot → 145° rotation. Fixed in commit 27d6b41 pushed to origin/main. |
| Session 3 (post-fix validation) | ✅ COMPLETED | 3× rev+fwd on clean stack with fixed datum. **GPS bearing now matches raw GNSS within 0.10°** (was -145.07°). Run 1-style "first-run anomaly" REPRODUCED again (Xsens settling). Runs 2+3 clean. Forward delivery still 33-42% (L motor weakness unchanged — needs hardware fix). |
| **Xsens 150s settling pattern** | ✅ CONFIRMED across 2 sessions | Every Run 1 (within ~150s of launch) shows chassis curve that heading-hold misses. Need ≥150s warmup before accepting goals. New finding worth a CLAUDE.md update. |

---

## 🚨 TO DO ON NEXT JETSON BOOT — UNSAFE BT TIMEOUT STILL ACTIVE

The Jetson's working tree has `navigate_igvc_autonav_humble.xml` with **test-only test-mode** values from this session:

| Line | Test-mode (current on Jetson) | Original / IGVC-safe | Why |
|---|---|---|---|
| 72 | `Timeout msec="120000"` | `Timeout msec="45000"` | **IGVC §X.X auto-DQ at 60s Hold-up Traffic** — current 120s breaks rule |
| 78 | `number_of_retries="10"` | `number_of_retries="3"` | matched IGVC-safe budget per BT design |

**Laptop main + origin/main both have the safe values.** Reverting on Jetson is one command:

```bash
ssh jetson 'cd ~/IGVC && git checkout -- src/avros_bringup/config/navigate_igvc_autonav_humble.xml'
```

Or equivalent: `mv .../navigate_igvc_autonav_humble.xml.bak2 .../navigate_igvc_autonav_humble.xml`

**Verify after revert:**
```bash
ssh jetson 'grep -E "Timeout msec|number_of_retries=" ~/IGVC/src/avros_bringup/config/navigate_igvc_autonav_humble.xml | head -3'
```
Expected output: `<Timeout msec="45000">` and `number_of_retries="3"`.

**Do this BEFORE the next `ros2 launch avros_bringup navigation.launch.py`** — bt_navigator caches the XML on first activation, so once it's loaded the unsafe values stick until reboot.
| M2a/b/c/d | ⏳ pending | After M1 |
| M3 closing square | ⏳ pending | After M2 |

## Next step — RECOVERY FROM JETSON CRASH

🚨 **Jetson became unreachable at ~14:13 UTC**, after attempting to launch yaw_diag.launch.py. Most likely cause: cumulative brown-out crash from motor driving (CLAUDE.md known-issue: shared 12V rail).

### Recovery sequence

1. User physical check + power-cycle if needed
2. Wait for Tailscale to reconnect (~60s after boot)
3. SSH to `jetson` and check:
   ```bash
   ls /tmp/yaw_diag_* 2>&1   # is the bag still there?
   cat /etc/fstab | grep tmp   # is /tmp tmpfs?
   ```
4. **IF bag survived**: immediately copy to `~/IGVC/bags/` or scp to laptop
5. **IF bag lost**: data from M0/M1a/M1b/M1c is gone — only the in-conversation snapshots remain (M0_snapshot.md, M1a_snapshot.md, M1b_M1c_retry_3m.md, multi_source_yaw_capture.md, repeatability_failure.md). The numbers in those docs ARE the data; bag was for full extraction + agent analysis but the in-session reading is preserved.
6. Re-launch with new `yaw_diag.launch.py` (already pushed + built before crash)
7. Restart bag recording with a session-2 path

### Key open question pre-crash

The repeatability run showed the diagnostic signal (IMU vs wheel gap) varies 5× between identical runs. We discovered this was caused by 4 publishers on `/cmd_vel` (velocity_smoother + controller_server + behavior_server fighting with the test script). The new `yaw_diag.launch.py` removes all Nav2 nodes → clean /cmd_vel → meaningful direct-cmd_vel tests.

### Resumability checkpoint

Even if conversation context is lost, all critical data is in this folder:
- `M0_snapshot.md` — stationary baseline (PASS)
- `M1a_snapshot.md` — MPPI catastrophic curve
- `M1b_M1c_retry_3m.md` — direct cmd_vel (contaminated; under-delivery known)
- `multi_source_yaw_capture.md` — multi-source IMU/wheel/EKF capture (contaminated by smoother)
- `repeatability_failure.md` — why we can't trust the contaminated data
- `methodology.md` — how measurements were taken
- The new `yaw_diag.launch.py` is in src/avros_bringup/launch/ (committed-ready) and built on Jetson

---

## Session-specific overrides (vs the v2 field plan)

| Per field plan v2 | Actual today | Reason |
|---|---|---|
| `enable_zed_front:=true` (Test B needs ZED VIO as 3rd source) | `enable_zed_front:=false` (default, off) | User instruction — disable cameras |
| LiDAR on (sensors.launch.py default) | `enable_velodyne:=false` | User instruction — clear area, no obstacles |
| Bag includes `/imu/mag`, ZED VIO odom, ZED IMU data | Bag includes `/imu/mag` only (no ZED) | Cameras disabled |

**Impact on analysis:**
- M2 Test B loses the ZED VIO 3rd-source tiebreaker. Two-of-three becomes IMU vs wheel only. If they disagree on rotation, M3 closing-error becomes the deciding signal instead.
- M1a (MPPI to 5m goal) may behave oddly with empty local costmap (no LiDAR data → MPPI sees clear world → goes straight). For yaw diag this is acceptable.
- /imu/mag still captured → Fix 4 mag-interference hypothesis still testable.

---

## File inventory for this session

### Live documents (this folder)
- `SESSION_STATE.md` — this file
- **`methodology.md`** — what's measured, how, sources of error (read FIRST when interpreting snapshots)
- `bags_manifest.md` — list of bags collected
- `M0_snapshot.md`, `M1a_snapshot.md`, `M1b_M1c_snapshot.md` (5m, INVALIDATED), `M1b_M1c_retry_3m.md`, `multi_source_yaw_capture.md` — per-phase observations

### Strategy + plan docs (parent docs/, not duplicated here — referenced)
- [`docs/session_2026_05_27_yaw_diag_field_plan.md`](../session_2026_05_27_yaw_diag_field_plan.md) — field plan v2 (bidirectional M1, CW/CCW M2)
- [`docs/yaw_diag_decision_thresholds.md`](../yaw_diag_decision_thresholds.md) — numerical pass/fail + decision matrix
- [`docs/yaw_diag_analysis_strategy.md`](../yaw_diag_analysis_strategy.md) — multi-agent post-bag analysis plan
- [`docs/yaw_diag_session_LOG.md`](../yaw_diag_session_LOG.md) — original template (live monitor pastings + analyzer outputs)
- [`docs/gps_datum_history.md`](../gps_datum_history.md) — datum change history
- [`docs/yaw_diag_patches/`](../yaw_diag_patches/) — pre-staged fix patches (fix1, fix2, fix4)

### Scripts (laptop `scripts/`, mirrored to Jetson `~/IGVC/scripts/`)
- `preflight_check.sh` — pre-record env/topic/IMU-bias gate
- `live_yaw_monitor.py` — real-time IMU vs EKF vs wheel vs ZED Δyaw
- `extract_bag.py` — bag → per-topic CSVs (with ZED VIO handler)
- `bag_preflight.py` — post-extract CSV completeness gate
- `tf_audit.py` — TF tree + map→odom drift quantification
- `analyze_M1.py` / `analyze_M2.py` / `analyze_M3.py` — per-test verdict generators

---

## Commits made this session

| SHA | Branch | Description | Pushed? |
|---|---|---|---|
| `f71d8dc` | main (laptop) | `navsat: update datum to SE Michigan field-test location` | ❌ NOT pushed |

---

## Bags collected

| Path (Jetson) | Started | Status | Phases recorded |
|---|---|---|---|
| `/tmp/yaw_diag_20260527_134442` | 2026-05-27 13:44:42 UTC | recording | M0 in progress |

---

## How to resume this session

If conversation context is lost, a new conversation can pick up by:

1. **Read this doc** (`docs/yaw_diag_session_2026_05_27/SESSION_STATE.md`)
2. **Check Jetson stack state:**
   ```bash
   ssh jetson
   pgrep -af 'ros2 launch' | head -5       # is nav still up?
   pgrep -af 'ros2 bag record' | head -5   # is bag still recording?
   cat /tmp/bag_path.env                   # what's the current bag path?
   tail -30 /tmp/nav_launch.log            # last status of nav
   tail -10 /tmp/bag_record.log            # last status of bag
   ```
3. **Match against "Current state" table above** — what's the last green row?
4. **Continue at "Next step"** — find the next item not yet done.

### What to do if stack is down
- Re-launch: `bash /tmp/launch_nav.sh > /tmp/nav_launch.log 2>&1 &` (script preserved on Jetson)
- Re-bag: `bash /tmp/start_bag.sh > /tmp/bag_record.log 2>&1 &` (script preserved on Jetson)
- Re-run preflight: `bash ~/IGVC/scripts/preflight_check.sh`

### What to do if bag is lost
- Test data is recoverable only by re-running the field session.
- Pre-flight numbers (IMU bias = 0.00346 °/s) and configuration (datum) are preserved in this doc — re-test from M0.

---

## Decisions made this session (chronological)

| Time | Decision | Reason |
|---|---|---|
| ~13:30 | Discard Jetson local `ekf.yaml` mod, pull main 33d62ea | Local content was identical to main's committed version |
| ~13:35 | Keep `navsat.yaml` Michigan datum during git ops | Will be replaced with new sampled datum |
| ~13:38 | Sample new GPS datum from current location | User relocated; old CPP/Michigan datums stale |
| ~13:42 | Commit datum to laptop main (no push) | User asked "commit" not "push" |
| ~13:44 | Launch with `enable_velodyne:=false` only | User: disable LiDAR + cameras, clear area |
| ~13:44 | Drop ZED VIO + ZED IMU from bag | Cameras off; topics won't publish |

---

## Risks accepted

1. **No ZED VIO tiebreaker for Test B** — if IMU and wheel disagree on rotation, we'll have only 2 sources. M3 closing error becomes the tiebreaker.
2. **No LiDAR → empty local costmap** — MPPI may not properly avoid obstacles. Acceptable because user confirmed clear area.
3. **Datum committed but not pushed** — Jetson will diverge from origin/main until next push. Tracked in this doc.
4. **Conversation context > 200 turns** — if it gets compressed/truncated mid-session, this doc is the recovery path.
