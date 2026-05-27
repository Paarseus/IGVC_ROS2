# M1b + M1c Retry at 3m — 2026-05-27 ~13:57 UTC

Curb-contaminated 5m run replaced. Clean direct-cmd_vel test at 3 m × 0.35 m/s.

## Sequence

| Phase | Command | Duration | Start pose | End pose |
|---|---|---|---|---|
| START | — | — | (3.533, -2.652, -108.24°) | — |
| M1c — REVERSE 3m | `/cmd_vel linear.x=-0.35` | 9 s | (3.533, -2.652, -108.24°) | (4.404, -0.013, -107.90°) |
| settle | (stopped) | 3 s | — | — |
| M1b — FORWARD 3m | `/cmd_vel linear.x=+0.35` | 9 s | (4.404, -0.013, -107.90°) | (3.579, -2.561, -109.03°) |

## Metrics

| | M1c REVERSE | M1b FORWARD |
|---|---|---|
| Distance | 2.779 m | 2.678 m |
| % of commanded (3 m theoretical) | 92.6% | 89.3% |
| Path bearing | +71.72° | -107.95° |
| Expected bearing | +71.76° (start_yaw + 180°) | -107.90° (= start_yaw) |
| **Path bearing error** | **-0.04°** | **-0.05°** |
| **Yaw drift over leg** | **+0.34°** | **-1.14°** |
| Yaw drift per meter | **+0.12 °/m** | **-0.42 °/m** |

## Key findings (clean data)

1. **Path translation is essentially perfect in both directions** (path bearing matches expected within 0.05°). The chassis goes where commanded.

2. **Yaw drift differs by direction and sign:**
   - Forward: -0.42 °/m (CW)
   - Reverse: +0.12 °/m (CCW — but small enough to be noise; the curb-contaminated 5m showed opposite sign)
   - Asymmetry: forward yaw drift is **3.5× the magnitude of reverse drift**

3. **"Path straight but yaw drifts"** = crabbing-type behavior. The chassis's center-of-mass translates straight, but the chassis body rotates during the translation. Consistent with skid-steer ICR offset under asymmetric track slip.

4. **Direction of forward drift is CW** (negative). If the conventional "right track has higher friction" hypothesis from CLAUDE.md were correct, you'd expect CCW drift (robot turns left because the right side grips more and goes farther). The observed CW drift contradicts this — suggests one of:
   - CLAUDE.md note has the asymmetry direction reversed
   - Asymmetric Mandow application is over/under-correcting
   - Heading-hold gain has a bias under forward motion
   - Per-track motor PID delivery is asymmetric

## Extrapolation to IGVC course distance

At -0.42 °/m forward bias:
- 10 m waypoint segment → -4.2° drift → ~0.73 m lateral by end (still within IGVC's 2 m lane)
- 50 m waypoint segment → -21° drift → **~5 m lateral deviation by end — would drive off-course**

This is the actual mechanism that catches up at competition distances. Issue #13's 14.7° at 3.15 m driven (= 4.67 °/m) is 10× worse than the per-meter rate measured today. Possible explanations:
- The original 14.7° measurement was under MPPI control (chassis bias + controller amplification, like M1a here)
- Different surface conditions on the original test
- Issue #13's robot may have been in stuck-bias IMU state at the time

## Confirmed: Fix 1/2/4 won't help

The pre-staged patches all assume sensor disagreement. With clean data, sensors agree on a small physical bias. The fix path is:
1. Investigate per-track friction asymmetry direction (which side actually grips more)
2. Re-tune Mandow `wheel_separation_multiplier` for grass with forward/reverse asymmetry
3. OR add a directional offset to the asymmetric Mandow correction
4. OR strengthen heading-hold gains so the chassis is forced straight under direct cmd_vel

## Robot current state
Position: (3.579, -2.561, -109.03°). Net displacement near zero from before the rev+fwd sequence (because rev and fwd nearly cancel).

## Bag windows
- M1c retry: t ≈ 638..648 s of `/tmp/yaw_diag_20260527_134442`
- M1b retry: t ≈ 651..661 s
