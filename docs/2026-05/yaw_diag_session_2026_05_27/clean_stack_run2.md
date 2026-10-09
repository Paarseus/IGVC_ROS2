# Clean-Stack Run 2 — 2026-05-27 ~14:50 UTC

Second multi-source rev+fwd on `yaw_diag.launch.py` (no Nav2 contention). ~5 min after Run 1. Same script, same stack, same publisher count = 0.

## Results

### M1c REVERSE 3m (distance 2.087 m)

| Source | Δyaw |
|---|---|
| IMU (quat) | +0.192° |
| IMU (∫wz) | +2.344° |
| Wheel (∫wz) | +0.689° |
| EKF | +0.303° |

### M1b FORWARD 3m (distance 1.291 m)

| Source | Δyaw |
|---|---|
| IMU (quat) | -0.992° |
| IMU (∫wz) | +1.175° |
| Wheel (∫wz) | +1.307° |
| EKF | -0.779° |

## Disagreements

| Pair | M1c reverse | M1b forward |
|---|---|---|
| IMU(quat) − IMU(∫wz) | -2.15° | -2.17° |
| IMU(quat) − Wheel | -0.50° | -2.30° |
| IMU(quat) − EKF | -0.11° | -0.21° |
| Wheel − EKF | +0.39° | +2.09° |

All sources agree within ~2° on this run. **Vastly different from Run 1.**

## Comparison vs Run 1

| Metric | Run 1 | Run 2 | Change |
|---|---|---|---|
| M1c IMU(∫wz) | +34.5° | +2.34° | 15× smaller |
| M1c Wheel | +33.3° | +0.69° | 48× smaller |
| M1b IMU(quat) | -13.22° | -0.99° | 13× smaller |
| M1b EKF | -13.18° | -0.79° | 17× smaller |
| M1c IMU(quat) | +0.29° | +0.19° | similar |

The IMU quaternion (Xsens internal fusion) AND EKF dropped from huge values to small ones. Wheel/gyro raw readings dropped even more.

**This is not just sensor noise.** Drops of this magnitude (10-50×) between identical commands suggest one of:

1. **Xsens stuck-bias incident** in Run 1 that recovered by Run 2. The Xsens filter can get stuck (CLAUDE.md known issue: "Xsens MTi-680G stuck with gyro bias ~70× normal"). If Run 1 was a transient stuck-bias episode, gyro raw would show false rotation, heading-hold would over-correct via wheel commands, wheel-derived ω would track that, and EKF would integrate both → 33° false rotation.

2. **Chassis physically curved much more in Run 1** due to grass-state differences. Possible if the robot wheels were on different patches, or wheels had dug ruts that helped on Run 2.

3. **Battery sagging** changed motor delivery dynamics between runs. Less power → less slip → cleaner readings.

## Same persistent observations across both runs

- Forward distance under-delivery: 1.29 m (Run 2) vs 1.32 m (Run 1) — both ~44% of commanded
- Reverse distance: ~2.09 m both runs (~70%)
- The DIRECTION asymmetry (forward worse than reverse) is consistent — reverse drift is smaller in both runs

## User observation

> "on the way back it had a curve  ut going forward seemed fine try again"

- **Reverse curved**: matches Run 1 wheel/gyro reading (+33° rotation) BUT NOT Run 1 Xsens-quat (+0.3°) or EKF (+0.4°). If chassis truly rotated 33° during Run 1 reverse, Xsens-quat was wrong. This is consistent with hypothesis #1 (Xsens stuck-bias).
- **Forward fine**: matches Run 2 (IMU -0.99°, EKF -0.78°) but NOT Run 1 (-13.22°). The user likely observed Run 2's forward leg.

## Bag window
Run 2: `~t=235..258 s` of `/home/dinosaur/IGVC/bags/yaw_diag_s2_20260527_144425`.

Run 1: `~t=110..132 s` of same bag.

Both legs of both runs are in the same bag for offline analysis.
