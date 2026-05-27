# Clean-Stack Diagnosis — Session 2 — 2026-05-27 ~14:46 UTC

**THIS IS THE FIRST UNCONTAMINATED MULTI-SOURCE READING OF THE SESSION.** All previous direct-cmd_vel runs were contaminated by a 4-publisher race on `/cmd_vel`. With `yaw_diag.launch.py` (no Nav2 nodes), `/cmd_vel` publisher count = 0 before the test; my publishes are alone.

## Test
Same script: `send_rev_fwd_with_imu.py`. 9s × 0.35 m/s reverse, 3s settle, 9s × 0.35 m/s forward. Multi-source capture of IMU quaternion, IMU gyro integrated, wheel-derived ω integrated, EKF yaw.

## Results

### M1c REVERSE (distance 2.09 m of ~3 m commanded)

| Source | Δyaw | Per meter |
|---|---|---|
| IMU (quat) | +0.29° | +0.14 °/m |
| IMU (∫wz) | +34.50° | +16.5 °/m |
| Wheel (∫wz) | +33.33° | +16.0 °/m |
| EKF | +0.42° | +0.20 °/m |

### M1b FORWARD (distance 1.32 m of ~3 m commanded)

| Source | Δyaw | Per meter |
|---|---|---|
| IMU (quat) | **-13.22°** | **-10.0 °/m** |
| IMU (∫wz) | +7.64° | +5.78 °/m |
| Wheel (∫wz) | +6.59° | +4.99 °/m |
| EKF | **-13.18°** | **-9.98 °/m** |

### Disagreements

| Pair | M1c reverse | M1b forward |
|---|---|---|
| IMU(quat) − IMU(∫wz) | -34.21° | -20.86° |
| IMU(quat) − Wheel | -33.04° | -19.81° |
| IMU(quat) − EKF | **-0.12°** ← agree | **-0.04°** ← agree |
| Wheel − EKF | +32.91° | +19.77° |

## The diagnosis flip

**Issue #13's central claim is REFUTED on clean data:** EKF tracks IMU within 0.04–0.12° on both legs. It does NOT track wheel; in fact it rejects wheel by 33° on reverse and 20° on forward. This is the **rejection gate** (`imu0_twist_rejection_threshold: 5.0` from commit 33d62ea) doing exactly what it was designed to do — block wheel updates when they disagree wildly with IMU.

What happened in session 1: the `/cmd_vel` publisher race caused the actuator to receive a noisy/zeroed command stream → chassis behaved erratically → IMU and wheel both produced unusual data → the contention also created weird timing artifacts that made the EKF behave atypically. The "EKF tracks wheel" reading was a contamination artifact, not a real EKF behavior.

## What the data DOES show (clean signal)

1. **Chassis genuinely curves forward CW at ~10 °/m on grass.** This is a real chassis behavior visible in both IMU and EKF (which agree to within 0.04°).

2. **Wheel-derived ω is unreliable on grass.** It reported +33° during a near-stationary reverse and +7° (wrong sign) during a -13° forward rotation. Mandow forward kinematics assumes equal-friction tracks; asymmetric slip on grass invalidates this.

3. **Xsens internal filter is doing major gyro correction**: IMU(quat) − IMU(∫wz) gap is 20–34° per 9-second leg. The raw gyro is drifting significantly; the Xsens fused quaternion (mag + accel + gyro) is the trusted reference.

4. **Forward distance under-delivery persists** even on clean stack (1.32 m of 3 m = 44%). This isn't a /cmd_vel contention issue. Hypotheses:
   - Chassis loses translation energy to rotation when curving hard
   - Grass friction is severe enough to halve effective forward speed
   - Actuator slew limiter / heading-hold reduces effective velocity

5. **Reverse delivers better** (70% vs 44%). Consistent with the directional asymmetry observed throughout the session.

## Updated fix path

**Re-evaluate the pre-staged patches:**

| Patch | Original purpose (#13) | Verdict on clean data |
|---|---|---|
| Fix 1 (yaw Q 0.01→0.1) | Loosen EKF trust in IMU yaw | ❌ WRONG DIRECTION — IMU is the right source, EKF already trusts it |
| Fix 2 (wheel cov 0.0001→0.01) | Loosen EKF trust in wheel | ⚠️ correct direction but redundant — rejection gates already filter bad wheel |
| Fix 4 (Xsens NoBaro) | Different IMU filter profile | ❌ IMU is fine, no change needed |

**New direction**: investigate the chassis curve. Candidate root causes:
1. Per-track friction asymmetry on grass (CLAUDE.md notes right-track 8% higher friction — but on grass this could be different)
2. Per-motor PID delivery asymmetry (one motor delivers slightly less than other under load)
3. Mechanical: dragging cable, wedged debris, bearing issue
4. URDF / Mandow `wheel_separation_multiplier=1.19` is calibrated for indoor concrete; grass may need different value

## What we should measure next

1. **Per-wheel RPM during the drive** (`/avros/wheel_debug` L_cmd_rpm/L_meas_rpm/R_cmd_rpm/R_meas_rpm) — directly tells us if motors are delivering asymmetrically
2. **Repeat the clean test** — confirm N=2 with same behavior
3. **Manual test**: drive only the left motor, see if chassis turns CCW; drive only right, see if turns CW. Confirms which track has higher friction.
4. **In-place rotation** — does the chassis spin the commanded ω cleanly? Decouples translation from rotation.

## Caveat — N = 1

This is one reading. Single-shot variance was huge in session 1 (5× variability). Need at least N = 2 on clean stack before claiming this is the actual chassis behavior.

## Bag window
`/home/dinosaur/IGVC/bags/yaw_diag_s2_20260527_144425`, ~t=110..130 s. Persistent disk; survives reboot.
