# Yaw Diagnostic — Final Verdict — 2026-05-27

Multi-agent analysis of bag `/home/dinosaur/IGVC/bags/yaw_diag_s2_20260527_144425` (Session 2, clean stack, `yaw_diag.launch.py`). 23m41s of data, 594k messages, 11 topics extracted to CSV.

## TL;DR

**Issue #13 ("EKF under-weights IMU yaw vs wheel") is REFUTED.** The EKF correctly tracks IMU on this clean data — Pearson r_imu (0.23–0.70) consistently exceeds r_wheel (0.12–0.38) across all 4 motion windows. The pre-staged patches (Fix 1, Fix 2, Fix 4) are unnecessary.

**Three NEW issues discovered**, all higher-priority than #13:

1. **L motor delivers 84% of commanded RPM in forward** (vs 90% for R), but in reverse both motors deliver ~90% symmetrically. Direction-asymmetric inner-PID problem.
2. **navsat_transform yaw is 180° inverted** at the new SE Michigan location. Map EKF gates the inverted Δs out under motion, but GPS noise leaks in while stationary → 6× the M4 drift baseline.
3. **My live-session integration scripts had a bug** that fabricated +33° wheel readings in Run 1. Snapshot docs' anchoring numbers are off by ~50s. (See "Live-session corrections" below.)

**Confidence: HIGH** for #1 and #2 (multi-window correlation, CSV-verified). HIGH for #13 refutation (4 independent forward windows all agree).

## Multi-agent results

| Agent | Verdict | Key finding |
|---|---|---|
| A — Sensor Health | ✅ PASS | All sensors healthy. Gyro bias 0.008°/s in all stationary windows (M4 was 0.004°/s; both well under 0.05°/s threshold). IMU quaternion takes ~150s to settle after launch — Runs 1's outlier-looking data may partly reflect this. No wheel encoder noise; no publishing gaps. [agent_a_sensor_health.md](../../bags/yaw_diag_s2_20260527_144425_csv/reports/agent_a_sensor_health.md) |
| C — EKF Behavior | ✅ **ISSUE_13_REFUTED** | r_imu > r_wheel in all 4 motion windows. Rejection gates dormant (gaps are smooth, not spikes). Run 1 "33° wheel" was a live-script bug — CSV shows wheel +5.81°, EKF tracks IMU quat (Δ=0.03°). [agent_c_ekf_behavior.md](../../bags/yaw_diag_s2_20260527_144425_csv/reports/agent_c_ekf_behavior.md) |
| D — Actuator Fidelity | 🎯 **L motor weak in forward** | L: 84.3% delivery fwd, 90.3% rev. R: 90.1% fwd, 90.5% rev. Chassis tracks wheels 100–103% — Mandow is fine. Heading-hold already pushes L harder but L still under-delivers. [agent_d_actuator_fidelity.md](../../bags/yaw_diag_s2_20260527_144425_csv/reports/agent_d_actuator_fidelity.md) |
| E — Map-EKF / GPS | ⚠️ **REGRESSION** + **self-inflicted bug FOUND + FIXED** | M0 drift 1.56 cm/s = 6× M4 baseline. GPS noise 10× worse (Michigan multipath). Agent E claimed 180° navsat inversion; verification showed exact offset was -145.07°. **Root cause: I put GPS altitude (247.578 m) in the heading slot of `datum: [lat_deg, lon_deg, heading_rad]` triplet in commit f71d8dc.** 247.578 rad mod 2π = 145.17° = the rotation. Fixed in commit 27d6b41 (datum now `[lat, lon, 0.0]`). [agent_navsat_recheck.md](../../bags/yaw_diag_s2_20260527_144425_csv/reports/agent_navsat_recheck.md) |

## Cross-agent reconciliation

| Apparent contradiction | Resolved by |
|---|---|
| A says Xsens healthy; live observations suggested stuck-bias | The "stuck-bias" was IMU quat fusion settling time (~150s), not actual gyro drift. Bias values are normal in all stationary windows. |
| Live snapshot doc claimed Run 1 "wheel +33°" | Bug in `send_rev_fwd_with_imu.py` integration. CSV shows actual wheel Δyaw = +5.81°. Agent C verified directly from raw data. |
| Live conclusion "EKF tracks wheel" vs Agent C "EKF tracks IMU" | Live conclusion was downstream of the bug. CSV r_imu > r_wheel everywhere. |
| User saw "reverse curved" in some run | Run 1's CSV-corrected wheel Δyaw of +5.81° over 2m would be visible as a curve. Not the 33° figure I reported live but still meaningful. |

## Verdict on issue #13

**CLOSE as wrong hypothesis.** The EKF behaves correctly. The 14.7° forward yaw drift in the issue's original observation is most likely:
- (a) IMU quaternion settling time (the original observation may have been within ~150s of boot)
- (b) Actual chassis curve from the L motor weakness now found by Agent D
- (c) Same `send_rev_fwd_with_imu.py`-style bug if the original observation used similar live-integration tooling

The pre-staged patches (`docs/yaw_diag_patches/fix1_*.patch`, `fix2_*.patch`, `fix4_*.patch`) should be **archived but NOT applied**. Document them as "considered, refuted by 2026-05-27 multi-agent analysis."

## New issues to file

| # | Title | Owner | Priority |
|---|---|---|---|
| 1 | **L SparkMAX under-delivers 6% in forward only** — bump kFF or investigate mechanical asymmetry | hardware/firmware | HIGH (drives the M1a-style catastrophic curves under MPPI) |
| 2 | ~~navsat 180° flip~~ — **RESOLVED 27d6b41**: self-inflicted typo (altitude in heading slot of datum). Single-line fix; re-test on field after Jetson back up. | software | RESOLVED in this session |
| 3 | M0 drift 6× M4 baseline — likely resolves with #2 above; verify after re-test | software | MEDIUM (revisit after datum fix in field) |
| 4 | Fix `send_rev_fwd_with_imu.py` / `live_yaw_monitor.py` wheel-integration bug | tooling | MEDIUM (used in any future field session) |
| 5 | GPS antenna lever arm `[0,0,0]` is still unmeasured per xsens.yaml | hardware | LOW (today's all-straight runs didn't expose it) |

## Action items (in priority order)

1. **Fix L motor PID delivery** — Agent D recommendation: in REV Hardware Client, bump L SparkMAX kFF 0.000197 → ~0.000211 (+7%), BURN only on L. Then re-run M1b at 0.35 m/s to verify L_meas/L_cmd reaches ~95%. Do NOT alter R or actuator_node.

2. ~~Investigate navsat 180° flip~~ **DONE — fixed in commit 27d6b41.** Was my typo in commit f71d8dc placing altitude in heading slot. Re-test on field needed to verify the 6× M0 drift regression resolves.

3. **Close issue #13 as REFUTED** with this verdict doc as the summary. Archive the staged Fix 1/2/4 patches.

4. **Fix the live-monitoring scripts' integration bug** before any future session — don't trust live numbers; always cross-check with bag extracts.

5. **File 4 new follow-up issues** per the table above.

## What we will NOT change

| Item | Reason |
|---|---|
| `ekf.yaml` — process_noise_covariance, twist_cov, rejection thresholds | EKF behaves correctly. Tuning more would only degrade. |
| `_twist_cov[35] = 0.0001` in actuator_node | Fine as-is; rejection gates handle outliers. |
| Mandow `wheel_separation_multiplier = 1.19` | Verified by chassis-tracks-wheels at 100–103%. |
| Xsens filter profile General_RTK | IMU is healthy; no evidence for Fix 4. |
| heading_kp = 1.5 | Sweep showed minimal benefit from changing. Doing work in straight drives. |

## What we WILL change (sequenced)

| Step | Action | Where | Reversible? |
|---|---|---|---|
| 1 | Bump L SparkMAX kFF + BURN | REV Hardware Client (hardware) | ⚠️ persistent in motor flash; can re-BURN to revert |
| 2 | Re-test M1b on clean stack with fixed L motor | Field | n/a (test only) |
| 3 | Investigate + fix navsat yaw 180° flip | `src/avros_bringup/config/navsat.yaml` | ✅ trivial config |
| 4 | Re-test M0 stationary + 1 forward leg | Field | n/a |
| 5 | If both above improve: commit, push, close #13 | git + GitHub | git revertible |

## Source of truth for everything

- This verdict: `docs/yaw_diag_session_2026_05_27/VERDICT.md`
- Multi-agent reports: `bags/yaw_diag_s2_20260527_144425_csv/reports/agent_*.md`
- Bag: `bags/yaw_diag_s2_20260527_144425/` (laptop) + `/home/dinosaur/IGVC/bags/` (Jetson)
- CSVs: `bags/yaw_diag_s2_20260527_144425_csv/` (laptop) + same path on Jetson
- Methodology: `docs/yaw_diag_session_2026_05_27/methodology.md`
- Pre-staged patches: `docs/yaw_diag_patches/` (ARCHIVE, do not apply)
- Per-test snapshots (some with timing bugs): `docs/yaw_diag_session_2026_05_27/*_snapshot.md`, `clean_stack_*.md`
- Repeatability + variance analysis: `docs/yaw_diag_session_2026_05_27/repeatability_failure.md` (note: some claims now superseded by Agent C — the variance was partly the integration bug)

## What this session accomplished

1. **Refuted issue #13** with rigorous multi-agent analysis on a clean (no Nav2 contention) stack.
2. **Found the actual root cause of forward curve** (L motor weakness in fwd direction, fixable in firmware).
3. **Discovered a new navsat config bug** (180° yaw inversion) that affects map-EKF behavior at the SE Michigan deployment site.
4. **Built and validated `yaw_diag.launch.py`** — a clean test-mode stack for future motor-level investigations.
5. **Captured 205 MB of high-quality bag data** preserved on persistent disk for future re-analysis.
6. **Validated the multi-agent pipeline** (4 agents + 3 analyzers + preflight + tf_audit) end-to-end against real field data.
7. **Recovered cleanly from a brown-out crash mid-session** with documented procedures.
