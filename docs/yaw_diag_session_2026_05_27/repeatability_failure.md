# Repeatability Failure — 2026-05-27 ~14:05 UTC

**Critical finding: the "issue #13 signature" from the first multi-source run does NOT reproduce on a second identical run.** Variance between identical tests is larger than the diagnostic signal. **Earlier "Fix 1+2 are relevant" claim is RETRACTED pending more data.**

## Method
Same script (`/tmp/send_rev_fwd_with_imu.py`), same conditions, same robot pose start, same publish rate, same 9-second legs at ±0.35 m/s. The only difference: ~5 minutes of elapsed time between runs.

## Side-by-side per-source Δyaw

### M1c REVERSE 3m

| Source | Run 1 | Run 2 | |Δ| |
|---|---|---|---|
| Distance | 2.02 m | 2.04 m | 0.02 m |
| IMU quat | +0.56° | +0.32° | 0.24° |
| IMU ∫wz | +1.90° | -1.93° | **3.83°** sign-flip |
| Wheel ∫wz | -0.66° | -0.43° | 0.23° |
| EKF | -0.17° | +0.20° | 0.37° |
| IMU(quat) − Wheel | +1.22° | +0.75° | 0.47° |
| IMU(quat) − IMU(∫wz) | -1.34° | +2.25° | sign-flip |

Wheel and EKF are broadly consistent. IMU sources (especially gyro-integrated) **flip signs** between runs — the Xsens internal correction is changing direction between runs.

### M1b FORWARD 3m

| Source | Run 1 | Run 2 | |Δ| |
|---|---|---|---|
| Distance | 1.09 m | 1.25 m | 0.16 m |
| IMU quat | +1.42° | +0.12° | **1.30°** |
| IMU ∫wz | +2.10° | +1.16° | 0.94° |
| Wheel ∫wz | **+4.79°** | **+0.81°** | **3.98°** |
| EKF | +4.04° | +0.46° | **3.58°** |
| IMU(quat) − Wheel | **-3.37°** | **-0.69°** | **2.68°** |

**The IMU-wheel gap varied from -3.37° to -0.69° on identical commands.** That's 5× variance on the very signal that was supposed to diagnose issue #13.

## Consistent across both runs

1. **Forward distance under-delivery**: 1.09 m and 1.25 m, vs commanded 3.0 m. Always short.
2. **Reverse distance OK-ish**: 2.02 and 2.04 m of 3.0 m commanded. Closer to target but still under.
3. **EKF tracks wheel** within ~0.5° in both runs (Wheel-EKF gap: -0.49°, +0.35°). So whatever the wheel says, the EKF agrees. This is a stable EKF behavior, not session-variant.

## Variable across runs

1. The magnitude of wheel Δyaw in forward varies 5×
2. The IMU quaternion-derived Δyaw varies 12× in forward
3. IMU gyro-integrated yaw flips signs between runs

## Conclusion: variance > signal

The session's first "smoking gun" finding (Run 1's 3.4° IMU-wheel gap) is in the variance band. Cannot claim issue #13 from single shots. To extract a signal from this noise we need:

- **N=5–10 repeats** with median/IQR analysis
- OR **longer duration** legs (e.g., 30-second runs averaging over many wheel-slip events)
- OR **a different test type** (in-place rotation has a known truth: commanded ω × time)
- OR **a different surface** (concrete to characterize chassis without grass variance)

## What this means for the fix path

I should NOT claim Fix 1+2 are warranted based on single multi-source readings. The honest answer right now:

- We CAN see EKF tracking wheel reliably (`Wheel − EKF` gap stable < 0.7° both runs)
- We CAN'T characterize the IMU-wheel disagreement magnitude from these data
- We DON'T know if Fix 1 or Fix 2 would reduce or increase the noise

The bag has all 3 multi-source legs (Run 1 + Run 2 + invalidated 5m run) plus M0 plus M1a. Offline analysis with proper time windows + integration on the full data would give better numbers than these single-leg snapshots.

## Robot current state
EKF: (4.102, -1.650, -110.82°). Net displacement near zero relative to multi-source Run 1 start (3.571, -2.584). Means the robot has done 4 rev-fwd cycles total this session (M1c+M1b clean 3m, plus 2× multi-source runs), and ended up in approximately the same place. Not bad for direction-asymmetric drift — at least it's bounded.

## Bag windows
- Repeat (Run 2): t ≈ 994..1004 s reverse, 1007..1017 s forward of `/tmp/yaw_diag_20260527_134442`
