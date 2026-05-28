# L Motor Fix — Validation Results — 2026-05-28

Bags (both preserved on Jetson `/home/dinosaur/IGVC/bags/` + laptop `bags/`):
- `l_motor_validation_20260528_060927` (49 MB) — two rev→fwd passes per the standard `send_rev_fwd_with_imu.py` script
- `m_per_rev_calib_20260528_063714` (34 MB) — clean forward-only 12 s drive on asphalt against a 3 m tape mark

Stack: `yaw_diag.launch.py`, no Nav2, no perception. 150 s Xsens warmup before the first motion.
Surface: **asphalt** (no slip).

## TL;DR

1. **L motor fix landed — issue #14 closes as RESOLVED.** Forward L delivery jumped from 84.3% → 96.2%, L−R asymmetry collapsed from −5.86 pp to ±0.3 pp on both passes. SparkMAX-side numbers from `/avros/wheel_debug`.

2. **`m_per_rev = 0.01994` is confirmed correct.** A clean forward-only 12 s drive at 0.35 m/s went 3.84 m per EKF / wheel_odom on asphalt, with tape measure confirming chassis passed the 3 m mark and ended near 3.8 m. The TODO.md L1 calibration item can close.

3. **NEW UNSOLVED BUG: forward delivery is suppressed when forward immediately follows reverse.** Same chassis, same surface, same Xsens, but the rev→fwd legs in `send_rev_fwd_with_imu.py` deliver ~45% of commanded distance while a clean forward-only drive delivers ~91%. Filed as a new issue (see "Issues" below).

4. **Xsens quaternion needs MOTION to settle, not just stationary time.** Even after 150 s of stationary warmup with healthy gyro bias (μ=+0.0085 °/s), Pass 1's forward leg showed −13.0° phantom yaw drift while Pass 2 (~90 s later, same conditions) showed only −1.7°. New memory: [[feedback-xsens-quat-needs-motion-warmup]].

## L motor delivery (the question this session was meant to answer)

Steady-state mean over each leg (1.5 s skipped at start for slew, 0.3 s at end for stop) from `/avros/wheel_debug`:

| Leg | ⟨L_cmd⟩ | ⟨R_cmd⟩ | ⟨L_meas⟩ | ⟨R_meas⟩ | L/Lc | R/Rc | L−R asym |
|---|---:|---:|---:|---:|---:|---:|---:|
| P1 M1c REV | −1012.1 | −946.5 | −977.1 | −909.0 | 96.5% | 96.0% | +0.5 pp |
| **P1 M1b FWD** | **+956.2** | **+1002.7** | **+920.0** | **+965.8** | **96.2%** | **96.3%** | **−0.1 pp** |
| P2 M1c REV | −981.1 | −977.5 | −945.3 | −939.6 | 96.4% | 96.1% | +0.2 pp |
| **P2 M1b FWD** | **+973.8** | **+985.1** | **+938.7** | **+947.1** | **96.4%** | **96.1%** | **+0.3 pp** |

Comparison vs yesterday (`docs/yaw_diag_session_2026_05_27/SESSION_FINAL.md:71`):

| Direction | Yesterday L/Lc | Today L/Lc | Yesterday L−R | Today L−R |
|---|---:|---:|---:|---:|
| Forward | 84.3% | **96.2−96.4%** | −5.86 pp | ±0.3 pp |
| Reverse | 90.3% | 96.4−96.5% | −0.22 pp | +0.2−0.5 pp |

Phase A motor-side gates from `docs/yaw_diag_session_2026_05_28/STRATEGY.md`:
- ✅ L_meas / L_cmd ≥ 92% forward (got 96.2−96.4%)
- ✅ \|L − R\| < 2 pp forward (got 0.1−0.3 pp)
- ✅ R delivery in same band (got 96.1−96.3%)

## The chassis-distance puzzle, resolved

**Two contrasting drives, same robot, same surface, same Xsens state:**

| Drive | Commanded | EKF dist | wheel_odom dist | Tape | Delivery |
|---|---:|---:|---:|---:|---:|
| P1 M1b FWD (9 s after a 9 s reverse) | 3.15 m | 1.36 m | — | — | **43%** |
| P2 M1b FWD (9 s after a 9 s reverse + settle) | 3.15 m | 1.31 m | — | — | **42%** |
| **Fwd-only 12 s (no preceding reverse)** | **4.20 m** | **3.84 m** | **3.86 m** | **>3.0 m (passed mark, ended near 3.8 m)** | **91%** |

The fwd-only run is honest: EKF, wheel_odom, and tape all agree within fine tolerance. So `m_per_rev = 0.01994` is correct, motors are honest, EKF is honest. The discrepancy on the rev→fwd legs is **NOT** a kinematics, conversion, or perception problem.

The 43-45% delivery is specific to the rev→fwd sequence. Something in the transition from reverse to forward suppresses forward velocity for the entire subsequent 9 s leg.

### Three candidates (not yet diagnosed in this session)

1. **Slew limiter state at the transition** — if `v_slewed` enters the forward leg at non-zero residual (e.g., still settling from -0.05 after the 3 s pause), the forward ramp loses time and the steady portion is shorter
2. **SparkMAX velocity-PID integrator wind-up** — 9 s of reverse winds the integrator. Forward command can't build duty until the integrator unwinds
3. **Heading-hold state contamination** — yaw reference may have shifted during reverse; on forward the heading-hold injects large asymmetric ω corrections that drain energy from forward translation

Settle by extracting time-series `v_target / v_slewed / v_after_imu / L_meas / R_meas` from the rev→fwd bag and comparing the transition seconds against the fwd-only bag. New issue filed separately (see Issues section).

## Per-leg yaw drift

| Leg | Distance | IMU(quat) Δyaw | EKF Δyaw |
|---|---:|---:|---:|
| P1 M1c REV | 2.04 m | +0.22° | +0.33° |
| **P1 M1b FWD** | **1.36 m** | **−13.0°** | **−13.2°** |
| P2 M1c REV | 2.03 m | +0.28° | +0.10° |
| **P2 M1b FWD** | **1.31 m** | **−1.72°** | **−1.57°** |
| **Fwd-only 12 s** | **3.84 m** | (clean) | **+0.10°** |

EKF tracks IMU(quat) within 0.17° on every leg — fourth independent refutation of issue #13.

The Pass 1 −13° drift was NOT caused by the chassis curving (motors were symmetric at 96%). The bag shows the actuator's heading-hold injected +0.0177 rad/s (CCW correction) for the full 9 s leg — 9.1° of commanded correction. Pass 2 injected only +0.0043 rad/s = 2.2°. The Xsens IMU(quat) reference itself was biased on Pass 1 and corrected by Pass 2. Motion-aware warmup, not stationary-time warmup, is what settles it. See [[feedback-xsens-quat-needs-motion-warmup]].

## What this session changed

- **Issue #14 (L motor under-delivery in forward):** RESOLVED — close with this doc as the proof.
- **Issue #15 (Xsens 150 s warmup):** scope expanded — 150 s stationary is necessary but not sufficient. Need a motion-warmup discard leg before any real Nav2 goal.
- **NEW issue (rev→fwd transition delivery suppression):** files separately. Not blocking the L motor question, but explains the 43-45% distance numbers that misled prior analysis (including yesterday's "grass slip" attribution and this morning's first draft of this doc).
- **TODO.md `m_per_rev` 5 m calibration item:** can close — confirmed correct via this session's tape-confirmed fwd-only drive.

## Decision for the rest of Phase A

L motor passes. Test 3 (Nav2 obstacle avoidance) is unblocked from the motor side. Add a **motion-warmup discard leg** (3 m fwd, 3 m back) before sending the real obstacle goal so Xsens quat is in its Pass-2 settled state.

The rev→fwd transition bug does not affect Nav2 directly because Nav2 emits continuous `cmd_vel` from the controller's MPPI loop, not discrete rev-then-fwd sequences. But it should be characterized before any future field-day that includes back-up recovery (which the recovery BT does include — clear-costmap + wait + back-up + crawl). If recovery back-up exits into forward travel, the same transient suppression could apply.

## Cleanup status

- Bags preserved on Jetson persistent disk + copied to laptop
- `yaw_diag.launch.py` stack still running on Jetson (ready for Test 3 with kill+launch swap, or stop)
- `avros-webui` systemd is STOPPED (was stopped at start of Phase A)
- BT XML reverted to 45 s / 3 retries ✓
- Jetson git HEAD at `32f2f7c` ✓
