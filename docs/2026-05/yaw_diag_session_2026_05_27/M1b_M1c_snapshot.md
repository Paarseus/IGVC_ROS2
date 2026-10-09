# M1b + M1c Snapshot — 2026-05-27 ~13:53 UTC — ⚠️ DATA INVALIDATED

**⚠️ M1c REVERSE HIT A CURB and got stuck for a bit, per user observation.**

The reverse trajectory was constrained by the curb. The near-perfect 0.04° bearing error and -0.54° yaw drift are artifacts of the curb constraining the path, NOT clean direction-asymmetric-slip data. The "direction-asymmetric slip" headline finding below is **withdrawn pending re-run**.

The forward M1b data was clean per user (curb was only hit on the way back during reverse).

A 3-meter re-run was requested by user; see M1b_M1c_retry_3m.md for clean data.

---

**Direct /cmd_vel, no MPPI.** Reverse then forward, same speed, same surface.

## Sequence

| Phase | Command | Duration | Start pose | End pose |
|---|---|---|---|---|
| M1c — REVERSE | `/cmd_vel linear.x=-0.35` | 14 s | (3.781, -2.756, -97.93°) | (4.409, +1.728, -98.46°) |
| settle | (stopped) | 3 s | — | — |
| M1b — FORWARD | `/cmd_vel linear.x=+0.35` | 14 s | (4.409, +1.728, -98.46°) | (3.539, -2.627, -102.49°) |

## Metrics

| | M1c (reverse) | M1b (forward) |
|---|---|---|
| Distance traveled | **4.528 m** | **4.441 m** |
| % of expected (4.9 m) | 92.4% | 90.6% |
| Yaw drift | **-0.54°** | **-4.03°** |
| Yaw drift per meter | **0.12°/m** | **0.91°/m** |
| Path bearing | +82.03° | -101.31° |
| Expected bearing (from start yaw) | +82.07° | -98.46° |
| Path bearing error (path - expected) | **+0.04°** ≈ perfect | **-2.85°** drift left |

## Headline finding

**Direction-asymmetric slip.** Same chassis, same speed, same surface, no controller — and forward yaw drift is **~8× larger than reverse**. The chassis tracks straight in reverse but curves left when driven forward.

Linear-velocity delivery is similar in both directions (90–92% of commanded — known slip), so the asymmetry is purely angular.

## Implication for issue #13 fix path

**NONE of the pre-staged patches (Fix 1, Fix 2, Fix 4) will fix this.** All assume sensor-fusion mis-weighting, but the M1c+M1b data shows all sensors agree on the physical curve — they're correctly reporting the chassis's behavior. The bug is the chassis behaving differently than the kinematic model expects.

## Hypotheses for the asymmetry (ranked)

| # | Hypothesis | Evidence for | Evidence against |
|---|---|---|---|
| 1 | **Right-track friction higher than left** (CLAUDE.md says 8%) | Forward drift goes left → left wheel slips more → effectively turns right (CW = -yaw direction matches) | Reverse should symmetrically drift right, but reverse drifts essentially zero |
| 2 | **Mandow `wheel_separation_multiplier=1.19` is wrong for grass** | Indoor-concrete calibration | Doesn't explain forward vs reverse asymmetry — Mandow correction is direction-symmetric |
| 3 | **IMU heading-hold gains too weak** | Heading-hold engages on `|ω_cmd|<deadband` which is true for both M1b and M1c | Would need to explain why reverse is "fixed" by heading-hold but forward isn't |
| 4 | **Caster/skid pivot location differs between fwd/rev** | Skid-steer kinematics with caster: front and rear pivot points are different; under forward the ICR is behind, under reverse it's ahead; the friction asymmetry projects differently | Theoretical but plausible — Reina 2016 paper covers this |

The reverse-near-perfect finding (#1's counter-evidence) suggests it's NOT pure left-vs-right friction asymmetry. More likely **caster/skid pivot asymmetry** (#4) interacting with surface friction.

## Open questions for next session

1. Does the forward-curve scale with speed? (M1d at 0.7 m/s)
2. Does it scale with **distance** (10 m forward — is it 8° or still 4°)?
3. Does the rotation symmetry hold? (M2: CW vs CCW)
4. Does it disappear on indoor concrete? (re-calibration baseline)

## Robot current state
Position: (3.539, -2.627, -102.49°) — back near M1a end. Needs repositioning before further forward tests.

## Bag windows
- M1c: t ≈ 466..480 s of `/tmp/yaw_diag_20260527_134442`
- M1b: t ≈ 484..498 s
