# Multi-Source Yaw Capture — 2026-05-27 ~14:00 UTC

**MOST IMPORTANT DATA POINT OF THE SESSION.** First leg where all 4 yaw sources (IMU-quat, IMU-∫wz, wheel-∫wz, EKF) were simultaneously captured and integrated from a known start.

## Method
Direct /cmd_vel, no MPPI. Integrators reset at the start of each leg, then accumulated as messages arrive on /imu/data (100Hz), /wheel_odom (20Hz), and /odometry/filtered (30Hz).

## Sequence + distance anomaly

| Leg | Cmd vx | Duration | Expected dist | Actual dist | % delivered |
|---|---|---|---|---|---|
| M1c REV v2 | -0.35 m/s | 9 s | ~3.0 m | **2.02 m** | 67% |
| M1b FWD v2 | +0.35 m/s | 9 s | ~3.0 m | **1.09 m** | **36%** |

The forward leg under-delivered dramatically. Earlier rev+fwd 3m run showed 89–93% delivery, so this is a regression — likely robot hit something or surface changed.

## Multi-source Δyaw per leg

### M1c REVERSE (2.02 m traveled)

| Source | Δyaw | Per meter |
|---|---|---|
| IMU quaternion (Xsens internal fusion) | +0.56° | +0.28 °/m |
| IMU ∫wz dt (raw gyro) | +1.90° | +0.94 °/m |
| Wheel ∫wz dt (Mandow-derived) | -0.66° | -0.33 °/m |
| EKF /odometry/filtered | -0.17° | -0.08 °/m |

**Disagreements:**
- IMU(quat) − IMU(∫wz) = -1.34° → Xsens internal filter applies ~1° correction vs raw gyro (mag-aiding or ZRU)
- IMU(quat) − Wheel = **+1.22°** → IMU and wheel disagree by 1.2° on the rotation
- IMU(quat) − EKF = +0.73° → EKF lands between IMU(quat) and wheel
- Wheel − EKF = -0.49° → EKF closer to wheel than to IMU on reverse

### M1b FORWARD (1.09 m traveled — short!)

| Source | Δyaw | Per meter |
|---|---|---|
| IMU quaternion (Xsens internal fusion) | +1.42° | +1.30 °/m |
| IMU ∫wz dt (raw gyro) | +2.10° | +1.93 °/m |
| Wheel ∫wz dt (Mandow-derived) | **+4.79°** | **+4.40 °/m** |
| EKF /odometry/filtered | **+4.04°** | +3.71 °/m |

**Disagreements (forward):**
- IMU(quat) − IMU(∫wz) = -0.68° → internal-filter correction, smaller than reverse
- **IMU(quat) − Wheel = -3.37°** → wheel claims robot rotated 3× more than IMU says
- **IMU(quat) − EKF = -2.62°** → EKF tracks wheel, not IMU
- **Wheel − EKF = +0.75°** → EKF and wheel agree closely

## Headline finding — issue #13's hypothesis is supported here

In M1b forward:
- **Wheel says rotation = +4.79°**
- **EKF says rotation = +4.04°** (tracks wheel)
- **IMU says rotation = +1.42°** (Xsens quaternion-fused, drift-free over 9s)

EKF and wheel agree within 0.75°. IMU disagrees by ~3°. This is the EXACT signature issue #13 predicted: "EKF tracks wheel, IMU is under-weighted."

Per the v2 decision matrix (M1 table, row "Wheel > 3.0°, IMU clean (< 1.0°)"):
**APPLY FIX 1 first; if EKF still wheel-dominated, add Fix 2.**

## Caveats

1. **Distance under-delivery** (36% forward) is anomalous and may indicate the robot was constrained or stuck. The yaw disagreement is over only 1.09 m of actual motion. Whether the disagreement scales with distance is unclear — need a longer clean run.

2. **Sign asymmetry**: Forward yaw is +CCW (IMU says +1.4°, wheel says +4.8°). Reverse yaw is mostly -CW (wheel -0.7°, EKF -0.2°). Both directions have non-zero rotation but in opposite senses.

3. **Distance discrepancy across runs**: Earlier 3m clean run reported 2.68 m forward; this run only 1.09 m forward. Same command, same conditions(?). Suggests run-to-run variance or environmental difference. Per-leg comparison may be misleading without repeats.

4. **IMU(quat) vs IMU(∫wz) self-disagreement** of ~0.7° per 9-second leg suggests the Xsens internal filter is actively correcting the gyro (probably via magnetometer + ZRU). The Xsens-fused yaw is the more reliable reference for short windows.

## Revised diagnosis (replaces earlier "chassis-side only" conclusion)

Both effects appear present:
- **Chassis-side direction asymmetry** (the M1c reverse vs M1b forward sign + magnitude differ) — needs Mandow re-tune or per-track friction work
- **Sensor-fusion under-weighting** (EKF tracks wheel within 0.75°, IMU is the outsider by 3°) — Fix 1 + Fix 2 ARE relevant

A coherent story: chassis truly rotates by IMU's reported +1.4°. Wheel-derived ω over-estimates this as +4.8° (because Mandow forward kinematics assumes equal-friction tracks; asymmetric slip makes wheel-derived ω inflate). EKF fuses both, but with `wheel_odom.vyaw` covariance = 0.0001 (very tight trust), EKF locks onto the wheel reading.

**So both fixes are needed:**
1. Fix 1/2 to make EKF trust IMU more → EKF reports +1.4° instead of +4.0°
2. Chassis-side Mandow re-tune so wheel-derived ω matches reality → wheel reports +1.4° too, agreeing with IMU

## Next steps

1. **Check robot physical state** — why did distance under-deliver to 36%?
2. **Repeat the multi-source forward leg** in clean conditions to confirm the +3° IMU-vs-wheel gap is reproducible (not a transient from the short distance)
3. **If repeatable: apply Fix 1+2** and re-test to see if EKF starts agreeing with IMU
4. **In parallel, plan chassis-side investigation** (Mandow re-tune on grass)

## Bag windows
- INITIAL: t ≈ 818 s of `/tmp/yaw_diag_20260527_134442`
- M1c v2: t ≈ 819..829 s
- M1b v2: t ≈ 832..842 s
