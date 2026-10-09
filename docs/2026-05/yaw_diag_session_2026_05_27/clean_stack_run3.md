# Clean-Stack Run 3 — 2026-05-27 ~14:53 UTC

Third multi-source rev+fwd on `yaw_diag.launch.py`. Same script, same conditions as Run 1 + Run 2.

## Results

### M1c REVERSE 3m (distance 2.096 m)

| Source | Δyaw |
|---|---|
| IMU (quat) | +0.175° |
| IMU (∫wz) | -0.900° |
| Wheel (∫wz) | +0.652° |
| EKF | +0.185° |
| **Max gap** | **1.08°** (IMU∫wz to Wheel) |

### M1b FORWARD 3m (distance 1.277 m)

| Source | Δyaw |
|---|---|
| IMU (quat) | +0.399° |
| IMU (∫wz) | -0.039° |
| Wheel (∫wz) | +0.572° |
| EKF | +0.907° |
| **Max gap** | **0.91°** (IMU∫wz to EKF) |

## Disagreements

| Pair | Reverse | Forward |
|---|---|---|
| IMU(quat) − IMU(∫wz) | +1.08° | +0.44° |
| IMU(quat) − Wheel | -0.48° | -0.17° |
| IMU(quat) − EKF | -0.01° | -0.51° |
| Wheel − EKF | +0.47° | -0.34° |

**All sources agree within ~1° on both legs.** This is the cleanest run of the session.

## Combined 3-run pattern (clean stack only)

| | Run 1 | Run 2 | Run 3 | Median |
|---|---|---|---|---|
| M1c IMU(quat) | +0.29° | +0.19° | +0.18° | +0.19° |
| M1c Wheel | +33.33° | +0.69° | +0.65° | +0.69° |
| M1c IMU-Wheel gap | -33.04° | -0.50° | -0.48° | -0.50° |
| M1c EKF | +0.42° | +0.30° | +0.18° | +0.30° |
| M1b IMU(quat) | -13.22° | -0.99° | +0.40° | -0.99° |
| M1b Wheel | +6.59° | +1.31° | +0.57° | +1.31° |
| M1b EKF | -13.18° | -0.78° | +0.91° | -0.78° |
| M1c distance (m) | 2.09 | 2.09 | 2.10 | 2.09 |
| M1b distance (m) | 1.32 | 1.29 | 1.28 | 1.29 |

## Persistent findings (consistent across all 3 clean runs)

1. **Forward distance under-delivery is robust**: 1.28–1.32 m of 3 m commanded (43%). Not a one-off.
2. **Reverse distance ~70%**: 2.09–2.10 m of 3 m commanded.
3. **Reverse-forward distance asymmetry**: 27% more delivery in reverse than forward. **This is the most reliable signal — and it's not about yaw, it's about velocity delivery.**

## Variable findings (Run 1 was an outlier)

- Run 1 showed catastrophic ω readings (33° on reverse via wheel/gyro) with Xsens-quat clamped near 0
- Runs 2 + 3 show normal small-rotation readings on all sources
- Likely cause: transient Xsens stuck-bias episode during Run 1 that resolved by Run 2

## Conclusion: issue #13 hypothesis REFUTED

Across Runs 2 and 3:
- EKF tracks IMU(quat) within 0.51° on both legs
- Wheel signal is being correctly handled by the EKF (not making EKF diverge wildly from IMU)
- Disagreements between IMU sources are < 1° (normal Xsens filter activity)

**Pre-staged Fix 1 (yaw process noise 0.01 → 0.1) would HURT** by making the EKF trust IMU less. Currently the EKF trust ordering is correct.

**Pre-staged Fix 2 (wheel cov 0.0001 → 0.01) is neutral-to-mildly-positive** — would loosen wheel trust further but the rejection gates already do this when needed.

**Pre-staged Fix 4 (Xsens NoBaro)** would be relevant only if Xsens stuck-bias episodes (like Run 1) are mag-interference-driven. Worth investigating but not urgent given they're rare.

## What's the real issue to fix

1. **Forward velocity delivery** is 57% of reverse. Investigate:
   - Per-motor PID delivery (`/avros/wheel_debug` L_meas_rpm vs R_meas_rpm during forward)
   - Forward vs reverse friction profile on this surface
   - Motor inversion behavior under forward vs reverse current direction

2. **Catastrophic yaw events** (Run 1 + M1a) are intermittent. Need to characterize:
   - Are they correlated with Xsens warm-up state?
   - Magnetic interference (motors near IMU)?
   - Battery sag events?

## Bag window
Run 3: `~t=355..378 s` of `/home/dinosaur/IGVC/bags/yaw_diag_s2_20260527_144425`.
