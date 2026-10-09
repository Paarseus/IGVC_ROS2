# 08 — Validation Plan

Goal: show, with recorded data, that each layer is accurate (no systematic error) and precise (low, bounded random error). Each test states what it proves, how to run it, the pass criterion, and its current status.

Ground truth: RTK FIXED GNSS (~1–2 cm) once the lever arm is handled (G1), plus the IMU for heading. Always check `rtk_status == 2` for the whole run and discard runs that drop to FLOAT.

Standard recording for every moving test:
```bash
ros2 bag record /cmd_vel /avros/wheel_debug /wheel_odom /imu/data /gnss /filter/positionlla \
  /status /odometry/filtered /odometry/global /odometry/gps /tf /tf_static
```

## Status summary

| ID | Test | Layer | Status |
|---|---|---|---|
| V0 | Static 150 s capture | all | **Done** 2026-09-25 |
| V1 | Velocity-measurement lag | motor | **Done** from 2026-05 logs (185 ms) |
| V2 | Velocity step response, current gains, on ground | motor | Needed |
| V3 | Steady-state delivery under rotation load | motor / kinematics | Needed (decides D7) |
| V4 | Straight-line distance scale | odometry | Needed |
| V5 | Lever-arm verification by rotation | GNSS | Static part **done**; rotation part needed |
| V6 | IMU heading vs GNSS course | IMU / map frame | Needed |
| V7 | GNSS covariance vs scatter by RTK state | GNSS / EKF | Partly done (V0); needs a 10 min FIXED run |
| V8 | Closed-loop square with RTK truth | integration | Done 2026-05 without RTK truth; repeat |
| V9 | Heading-hold vs MPPI interaction | kinematics | Needed (decides D2) |
| V10 | Surface calibration on grass | kinematics / odometry | Needed before competition |

## Tests

### V0 — Static capture (done)
Script `evidence/scripts/capture_static.py`, analysis `analyze_static.py`.
Results: IMU bias −0.0008 °/s, yaw drift 0.05 °/min, zero IMU covariance, wheel odometry stationary, GNSS scatter 10 cm (mostly FLOAT) against a reported 1.6 cm, map pose follows GNSS noise (37 cm span), global EKF lateral velocity up to 0.08 m/s.
Repeat after each fix; the 10-minute version with RTK FIXED throughout is V7.

### V1 — Velocity-measurement lag (done)
Script `evidence/scripts/velocity_lag.py`. Result: reported velocity trails position-derived velocity by 185 ms on both tracks. Repeat after any change to the SPARK MAX measurement settings; target < 40 ms.

### V2 — Velocity step response
Proves: the SPARK MAX loop tracks commands with bounded overshoot and settling, per track, on the real surface.
Method: tracks on the ground, straight line; steps 0 → 500 → 1500 → 2500 RPM, each held 5 s (use `L/R` over the Teensy with `actuator_node` stopped, or constant `/cmd_vel`). Evaluate with **position-derived** velocity (V1 method), not reported velocity.
Pass: overshoot < 10%, settling (±5%) < 400 ms, steady-state error < 2% at 5 s, L/R mismatch < 2%.

### V3 — Steady-state delivery under rotation load
Proves or disproves that the 15% "motor delivery loss" (D7) is a steady-state deficit of the velocity loop rather than a transient.
Method: rotate in place at 0.3, 0.5, 0.8 rad/s with `wheel_separation_multiplier: 1.0`, each held **15 s**. Log track RPM (position-derived), bus voltage (`D` line), IMU yaw rate.
Analysis: delivery = actual / commanded track speed, versus time within each hold.
Decision: if delivery rises toward 100% over the hold, the integrator is too slow (raise kI or add a static-friction feedforward); if it stays at ~85% with bus voltage steady, check the current limit (`kSmartCurrentFreeLimit`, 01 §4). Then re-measure the multiplier.
Add arcs: v = 0.7 m/s with ω = 0.2 rad/s, and v = 0.4 m/s with ω = 0.5 rad/s, 15 s each. Compare IMU ω / commanded ω against the spin result. The literature predicts a smaller effective track on arcs (Wang 2015), so a multiplier correct for spins over-drives arcs. If the ratio on arcs exceeds 1.05, make the multiplier a function of turn radius or calibrate at the typical operating arc.
Run V3 after the M1 feedforward fix as well; the multiplier should then approach the chassis skid value (~1.04 on pavement).

### V4 — Straight-line distance scale
Proves: `m_per_motor_rev` (0.01994) is correct on the operating surface.
Method: RTK FIXED, drive 20 m straight at 0.5 m/s, 3 runs each direction, on pavement and on grass. Compare wheel distance (from `pos_rev`, not velocity) with the GNSS distance (lever arm corrected).
Pass: scale error < 1% (pavement) and < 3% (grass); correct `m_per_motor_rev` per surface otherwise.

### V5 — Lever-arm verification
Proves: the antenna offset is known and applied.
Static part (done): `/gnss` is 0.764 m forward of `/filter/positionlla` (`leverarm_check_2026_09_25.txt`).
Rotation part: RTK FIXED, rotate in place 360° slowly (0.2 rad/s). Before the fix, `/gnss` traces a circle of radius ~0.75 m around a stationary `base_link`. After the fix, `/odometry/gps` for `base_link` should stay within 5 cm.

### V6 — IMU heading against GNSS course
Proves: the map frame is aligned with true north and IMU yaw is correct after warm-up.
Method: RTK FIXED, drive straight 20 m in 4 directions (N/E/S/W roughly), 0.5 m/s. Course = atan2 of the GNSS displacement; compare with the mean IMU yaw over the same segment.
Pass: mean offset < 1°, spread < 1°. A consistent offset goes into `yaw_offset`; a direction-dependent one indicates magnetic or mounting error.
Also run once immediately after power-up to quantify I2 (warm-up drift).

### V7 — GNSS covariance consistency
Proves: the reported covariance matches the real scatter in each RTK state.
Method: 10 min stationary per state (FIXED, FLOAT; plain GPS by disabling NTRIP).
Pass: measured 1σ within 0.5–2× the reported σ. Current result for mostly-FLOAT data: measured ~6× reported → fails.

### V8 — Closed-loop square with RTK truth
Proves: end-to-end localization accuracy under motion.
Method: 5 × 5 m square via Nav2 goals in the map frame, 3 laps, RTK FIXED. Compare `map→base_link` with the lever-arm-corrected GNSS track, and `odom→base_link` closure without GNSS.
Pass: map pose error < 10 cm RMS; odom closure < 2% of distance (2026-05 result: 1.5% on a 1 × 1 m square).

### V9 — Heading-hold interaction
Proves or disproves D2.
Method: Nav2 run over a gently curving path (radius > 15 m). Log `/cmd_vel` ω and `heading_locked`.
Metric: fraction of time the lock is active while MPPI commands non-zero ω. More than ~10% confirms D2.

### V10 — Surface calibration (grass)
Repeat V3 (rotation delivery and multiplier) and V4 (distance scale) on grass before competition, as `docs/skid_steer_kinematics_findings_2026_05_18.md` §7 already recommends.
