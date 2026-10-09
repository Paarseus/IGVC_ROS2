# M1a Snapshot — 2026-05-27 ~13:49 UTC

**Status: ABORTED by Nav2 BT watchdog at 45 s. Robot physically curved sharply right (CW) during the drive. Did NOT reach the 5 m forward goal.**

## What was sent
- Goal: (4.977, 0.476) in map frame, orientation = start orientation (+5.47°)
- Goal type: 5 m forward in robot's current heading direction
- Controller: MPPI (default Humble config, batch=1000, vx_max=0.7, wz_max=1.9)
- Local costmap: EMPTY (no LiDAR, no cameras → no obstacle inputs)

## What happened
- Robot left start point, drove with a visible curve to the right (per physical observation)
- After 45 s, Nav2's recovery BT watchdog cancelled the goal
- BT marked the goal ABORTED
- Robot stopped (cmd_vel = 0)

## Pose evolution (EKF-reported)

| | x (m) | y (m) | yaw (°) |
|---|---|---|---|
| Start | 0.000 | 0.000 | +5.47 |
| Final | 3.776 | -2.793 | -93.48 |
| Δ | +3.776 | -2.793 | **-98.95** |

- Straight-line distance from start to end: 4.697 m
- Path heading from start to end: atan2(-2.793, 3.776) = **-36.5°** (way off from intended +5.47°)
- Yaw drift: **-99°** (almost a full 90° CW rotation while ostensibly going straight)

## Source-agreement check (immediately after stop)

| Source | Yaw reported | Δ from start |
|---|---|---|
| EKF (`/odometry/filtered`) | -97.5° | -103° from start (5.47° → -97.5°) |
| IMU (`/imu/data` quat) | -97.5° (z=-0.7520, w=0.6592) | matches EKF |
| Wheel odom (twist now) | n/a (zero — robot stopped) | n/a until extracted from bag |

**Critical:** IMU and EKF AGREE on yaw post-drive. This means the robot really did rotate ~100°. The bug is NOT (purely) IMU vs EKF disagreement as #13 hypothesized — there's a **physical curve mechanism** as well.

## Likely causes (ranked)

1. **Chassis-side friction asymmetry** (right track has ~8% higher friction per CLAUDE.md). MPPI commands forward+zero-ω; chassis delivers forward+small-CW-ω. EKF sees it correctly. MPPI's corrective ω commands don't account for this asymmetry → cascade.
2. **MPPI with empty local costmap** mis-evaluating (no LiDAR data → MPPI may behave oddly). Less likely.
3. **Goal angular alignment issue**: MPPI may have interpreted the 5m goal as needing rotation to reach.

## User's physical observation
> "the car had a curve in the drive"

Confirms: robot physically curved. Not an EKF-only artifact.

## Implication for #13 fix path

The previously-planned **Fix 1** (yaw process noise 0.01→0.1) and **Fix 2** (wheel_odom vyaw cov 0.0001→0.01) **assume the IMU and wheel disagree under motion**. But here:
- IMU + EKF agree
- Wheel state to be extracted from bag (likely agrees too — wheels actually delivered the curve)

If all 4 sources (IMU, EKF, wheel, ZED-not-recorded) report the same physical curve, **none of the candidate fixes will help** because the bug isn't sensor fusion — it's chassis dynamics. The fix becomes:
- Re-tune Mandow `wheel_separation_multiplier` on grass (currently 1.19 from indoor concrete)
- OR add per-track friction compensation in `actuator_node`
- OR fix the right-track friction asymmetry mechanically

## Bag window
Roughly t=170..230s of `/tmp/yaw_diag_20260527_134442` (bag started 13:44:42, M1a sent at 13:49:12).

Will be extracted and analyzed by `analyze_M1.py` on the full session bag.

## Next test
**M1b** — direct `/cmd_vel linear.x=0.35`, no MPPI. Will decide chassis vs MPPI as the curve source.
