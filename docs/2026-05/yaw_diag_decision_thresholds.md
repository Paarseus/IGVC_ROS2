# Yaw Diagnostic — Quantitative Decision Thresholds

Numerical pass/fail criteria for each test window. The decision matrix at the bottom maps observed data signatures to **Fix 1**, **Fix 2**, **Fix 4**, or "hold and re-test."

All angles in degrees; all rates in deg/s unless noted. Δyaw = unwrapped accumulated heading change (use `atan2(sin, cos)` on differenced yaws to avoid wrap artifacts).

---

## M0 — Stationary baseline (60 s, robot motionless)

Reference: issue #12 M4 baseline (2026-05-26), 202 s stationary.

| Metric | Healthy | Marginal | FAIL (re-test or fix env first) |
|---|---|---|---|
| `/imu/data.wz` mean (gyro bias) | \|μ\| < 0.05 °/s | 0.05–0.3 °/s | > 0.3 °/s → **Xsens stuck bias, USB power-cycle** |
| `/imu/data.wz` σ | < 0.15 °/s | 0.15–0.5 °/s | > 0.5 °/s → IMU defective or vibration |
| `/wheel_odom.wz` | 0.0 ± 1e-6 (exact) | nonzero | nonzero → wheel encoder noise or actuator publishing wrong cov |
| `/odometry/filtered.x` and `.y` drift | < 1 mm/s | 1–10 mm/s | > 10 mm/s → EKF instability |
| `map → odom` xy drift rate | < 0.5 cm/s | 0.5–2 cm/s | > 5 cm/s → GPS noise / EKF tune regression vs M4's 0.26 cm/s |
| IMU vs EKF yaw delta (both should be ~constant) | drift < 0.1° over 60 s | 0.1–1° | > 1° → EKF actively diverging from IMU **while stationary** = strong evidence the under-weighting is real even with no motion |

**Gate:** if any FAIL row triggers, halt and fix the environment before M1. Don't waste a drive on a bad baseline.

---

## M1 — Test A: Multi-condition straight drive (v2 — 4 variants)

v2 expands the original single forward-drive test into 4 variants to disambiguate the bias mechanism. Each variant is ~5 m of travel; commanded angular = 0 throughout (except M1a where MPPI may inject micro-corrections).

| Variant | Conditions | What it isolates |
|---|---|---|
| M1a | NavigateToPose (5,0) under MPPI, vx_max=0.7 | Reproduces original #13 conditions exactly |
| M1b | Direct `/cmd_vel linear.x=0.35` for 14s | Clean test (no planner), slow speed baseline |
| M1c | Direct `/cmd_vel linear.x=-0.35` for 14s | Direction asymmetry (forward vs reverse slip) |
| M1d | Direct `/cmd_vel linear.x=0.7` for 7s | Speed scaling (bias scales with speed?) |

### Per-variant decision metrics

For each variant, compute integrated |Δyaw| from each source. Under perfect straight-line, all sources should report Δyaw ≈ 0°.

| Source | Compute | Expected if source is truth |
|---|---|---|
| IMU (quaternion) | `imu_data.yaw[-1] - imu_data.yaw[0]` (wrap-safe) | \|Δyaw\| < 1.0° |
| IMU (gyro integrated) | `∫ imu_data.wz dt` | matches quaternion within 0.5° |
| Wheel odom | `∫ wheel_odom.wz dt` | \|Δyaw\| < 1.0° |
| EKF | `odometry_filtered.yaw[-1] - odometry_filtered.yaw[0]` | matches the trusted source |
| ZED VIO | `zed_*_odom.yaw[-1] - [0]` | \|Δyaw\| < 1.5° (visual drift floor) |
| ZED IMU | `zed_*_imu/data.yaw[-1] - [0]` | within 0.5° of Xsens IMU if both clean |

### Cross-variant signatures (key insights — v2 only)

| Pattern across M1a-M1d | Diagnosis |
|---|---|
| All four variants: both IMU + wheel < 1° | **Both clean** — close #13 not-reproducible |
| Wheel \|Δyaw\| similar M1b vs M1d (speed-invariant) | **Transient slip** — not systematic; one-off |
| Wheel \|Δyaw\| in M1d ≈ 2× M1b (scales with speed) | **Systematic friction asymmetry** — fix the chassis side (not just covariance) |
| Wheel \|Δyaw\| sign FLIPS M1b vs M1c (forward vs reverse) | **Direction-asymmetric slip** — common on tracked chassis on grass; consistent with right-track 8% friction asymmetry (CLAUDE.md) |
| Wheel \|Δyaw\| same sign M1b and M1c | **Constant wheel bias** — encoder or kinematics issue, not slip |
| IMU \|Δyaw\| scales with speed (M1b → M1d) | **Speed-dependent IMU error** — likely vibration or thermal; Fix 4 not Fix 1 |
| IMU \|Δyaw\| ≈ 0 in all variants, EKF tracks wheel | **EKF under-weights IMU** — Fix 1 indicated |
| M1a (MPPI) yaw drift > M1b (cmd_vel) | **MPPI's heading critic isn't tracking IMU truth** — points to controller-level fix layered on top of EKF |

### Repeatability gate

M1a and M1b are both forward at moderate speed. If their |Δyaw| values from any source differ by > 1° **between identical sources**, repeatability is poor → flag, possibly re-run.

---

## M2 — Test B: Bidirectional rotation at 2 rates (v2 — 4 variants)

v2 expands to CCW + CW × 0.5 + 0.8 rad/s. Bypasses MPPI; direct `/cmd_vel`. Ground-truth Δyaw = `commanded_omega × duration` (open-loop, 360° per variant).

| Variant | Commanded wz | Duration | Direction |
|---|---|---|---|
| M2a | +0.5 rad/s | 12.6 s | CCW |
| M2b | -0.5 rad/s | 12.6 s | CW |
| M2c | +0.8 rad/s | 7.85 s | CCW |
| M2d | -0.8 rad/s | 7.85 s | CW |

Four independent yaw sources (with ZED IMU added in v2):

| Source | Independence | Recorded? |
|---|---|---|
| Xsens IMU (quaternion) | Gyro + mag fusion, Xsens internal filter | yes |
| ZED VIO | Visual feature tracking (no gyro, no wheel) | if enable_zed_front:=true |
| ZED IMU (built-in) | Independent gyro inside ZED-X camera | if recorded (v2) |
| Wheel-derived | Encoder + Mandow kinematics | yes |

### Per-variant criteria (expected Δyaw = ±360°)

| Source | Healthy | Marginal | Biased |
|---|---|---|---|
| Best agreeing pair | within ±5° of expected | within ±10° | > ±10° |
| Odd source out | within ±5° of agreeing pair | ±5–15° | > ±15° (the dissenter) |

### Cross-variant signatures (v2 only — bidirectional + multi-rate)

| Pattern | Diagnosis |
|---|---|
| IMU error similar M2a vs M2b (CCW = CW) AND similar M2c vs M2d | **No directional asymmetry** — IMU bias (if any) is symmetric; standard mag interference |
| IMU error in M2b ≠ M2a (CW worse than CCW or vice versa) | **Heading-dependent mag interference** — magnetometer crosses a contaminated heading on one side; **Fix 4 strongly indicated** |
| IMU error scales with rate (M2c >> M2a, M2d >> M2b) | **Rate-dependent — high motor current → strong mag distortion**; classic mag interference signature → **Fix 4** |
| IMU error invariant with rate but ZED VIO drifts | **ZED is the bad source**, not Xsens; ignore ZED tiebreaker advice |
| All four IMU sources (Xsens + ZED IMU) match, wheel diverges | **Wheel-side bias confirmed** (4-vs-1 consensus); apply Fix 1+2 |
| Xsens IMU + wheel match, both ZED sources (VIO + IMU) diverge | **ZED is the bad source** (2-vs-2 split resolved by ZED-only divergence); ignore ZED for #13 verdict |

### Rate-dependence formula

For Fix 4 indication (mag interference):
- IMU_error_rate_slope = (|IMU_err(M2c)| - |IMU_err(M2a)|) / (0.8 - 0.5)
- If slope > 10°/(rad/s) AND ZED stays steady → **Fix 4** is correct
- If slope < 3°/(rad/s) → rate-independent; mag interference NOT the cause

---

## M3 — Test C: 1 × 1 m closing square

Position metric (not yaw-specific). Validates that whichever fix was applied actually improves end-to-end accuracy.

| Metric | Source | Healthy | Marginal | FAIL |
|---|---|---|---|---|
| Position closing error | `‖odometry_filtered.xy[end] - .xy[start]‖` | < 0.1 m (2.5% of 4 m perimeter) | 0.1–0.2 m (Ollman 1–5%) | > 0.2 m |
| Yaw closing error | `odometry_filtered.yaw[end] - .yaw[start]` (wrap-safe) | < 10° (Mandow §5) | 10–20° | > 20° |
| Per-leg lateral drift | each NavigateToPose leg's perpendicular deviation from the planned line | < 0.15 m | 0.15–0.4 m | > 0.4 m |

Compare to pre-fix closing error from issue #13's field test: 130% lateral drift uncorrected, 5% after MPPI. Post-fix target: < 5% AND tight yaw closing.

---

## Decision Matrix (v2 — uses cross-variant signatures)

Read top to bottom; first row that matches wins. "All M1 variants" means M1a-M1d collectively.

| If M1 collectively shows | And M2 collectively shows | Apply |
|---|---|---|
| All variants: IMU + wheel both < 1° | Any (all agree within 5°) | **Nothing** — record M3 baseline, close #13 as not-reproducible-post-33d62ea |
| Wheel biased; direction-asymmetric (M1b sign ≠ M1c sign) | 4-source consensus excludes wheel | **Fix 1** → re-test → add **Fix 2** if EKF still wheel-dominated. **Also**: file follow-up issue on per-track friction (root cause unaddressed) |
| Wheel biased; scales with speed (M1d ≈ 2× M1b) | 4-source consensus excludes wheel | **Fix 1 + Fix 2** together. Friction-slip is systematic — covariance alone won't track |
| Wheel biased; constant across all variants | 4-source consensus excludes wheel | **Fix 1 + Fix 2**. Likely encoder calibration error, not slip — file follow-up to recheck Mandow α |
| IMU biased; rate-dependent (M2c-M2a slope >10°/(rad/s)) | ZED VIO + ZED IMU + wheel match | **Fix 4** — Xsens NoBaro profile. Mag interference confirmed |
| IMU biased; direction-asymmetric (M2b ≠ M2a) | ZED + wheel agree | **Fix 4**. Heading-dependent mag interference |
| IMU biased; invariant with rate AND direction | ZED + wheel agree | **Fix 4 unlikely to help** (not mag); investigate Xsens hardware OR vibration coupling. Hold + file new issue |
| Both biased opposite sign in any variant | ZED VIO + ZED IMU are tiebreakers | Apply the fix matching the bad source. If both IMU sources agree and wheel disagrees: Fix 1+2. If wheel agrees with ZED VIO and Xsens IMU disagrees: Fix 4 |
| Both biased same sign and similar magnitude | Same | Chassis truly curved — **no yaw fix**; investigate Mandow α on grass + per-track friction balance |
| Marginal: 1°-3° on a single source, can't decide | Same | **Hold** — drive a longer test (15m straight) or trigger Agent G EKF replay |

---

## How to abort gracefully

If at any point during the session a metric goes catastrophically wrong (e.g., M0 shows EKF diverging > 0.5°/s while stationary), **stop, save the bag anyway**, and we use it as a "before" snapshot. Even a broken session bag is useful evidence.

If the chassis behaves dangerously during M1 or M2 (oscillating, sudden direction reversal), `Ctrl+C` the bag terminal AND publish e-stop:

```bash
ros2 topic pub --once /avros/actuator_command avros_msgs/msg/ActuatorCommand '{estop: true}'
```

---

**Sources:** issue #12 (M4 baseline numbers), issue #13 (original 14.7° observation, candidate fixes), `docs/skid_steer_kinematics_findings_2026_05_18.md` (Mandow §5, Ollman benchmark), `feedback_xsens_imu_bias_recovery` memory (stuck-bias threshold).
