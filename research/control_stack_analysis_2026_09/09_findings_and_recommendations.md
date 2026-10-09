# 09 — Findings and Recommendations

## 1. Verdict by layer

"Proven" means supported by recorded data in this folder or in the team's earlier test records. "Not proven" means no measurement exists yet; the corresponding test is in 08.

| Layer | Precision (repeatable, low noise) | Accuracy (no systematic error) | Limiting issue |
|---|---|---|---|
| Motor velocity loop | **Proven**: ±0.25% repeatability, L/R 0.4% | **Not proven**; steady-state deficit explained by probable feedforward unit error | M1, M2 |
| Drive kinematics | Proven: math reproduces commands exactly | **Conditional**: correct only at the calibration point (spins, pavement) | D7, D2, D6 |
| Wheel odometry | **Proven**: zero drift at rest, 1.5% closure with EKF | **Not proven**: distance scale never measured; 185 ms timing error | O1, O5 |
| IMU | **Proven**: bias < 0.001 °/s, drift 0.05 °/min | Heading reference correct by specification; offset not measured | I1, I2 |
| Local EKF (odom) | **Proven**: 6 cm / 3.6° square closure (1.5%) | Follows the IMU exactly; inherits IMU errors unchecked | E3 |
| GNSS / RTK | **Proven** when FIXED: ~1 cm | **Not accurate as integrated**: 0.76 m heading-dependent bias, measured | G1, G2 |
| Global EKF (map) | **Not proven**: stationary pose moved 37 × 34 cm tracking GNSS noise | **Not accurate**: carries the G1 bias; gates ineffective | E1, E4, G1 |

**Bottom line:** the stack is precise at the motor, odometry, IMU and local-estimation levels, with good published-comparison evidence. It is not yet shown to be accurate, and two errors are now measured: the 0.76 m antenna offset and the 185 ms feedback lag. The global pose is the weakest output.

## 2. Findings, prioritized

Severity: **Critical** = systematic error in a core signal; **High** = measured or confirmed defect affecting accuracy or safety; **Medium** = defect with bounded impact or needing data; **Low** = minor or documentation.
Status: **Confirmed** = measured or verified in source/binary; **Probable** = strong evidence, test defined; **Hypothesis** = needs data.

| # | Finding | Severity | Status | Evidence | Doc |
|---|---|---|---|---|---|
| 1 | SPARK MAX param 16 is kV (V/RPM) on FW 2026; our value supplies ~9% of intended feedforward | Critical | Probable | REV changelog/template; model matches 70% and 83% observed delivery | 01 M1 |
| 2 | GNSS antenna 0.76 m ahead of `base_link` is not corrected; map pose biased along heading | High | Confirmed | static check, driver source | 06 G1 |
| 3 | Reported wheel velocity lags real motion by 185 ms (default NEO filter) | High | Confirmed | `velocity_lag.py` on two logs, corr ≥ 0.996 | 01 M2 |
| 4 | Wheel odometry integrates the lagged velocity instead of encoder position; position is received and unused | High | Confirmed | source | 03 O1 |
| 5 | IMU covariances are zero → IMU treated as perfect; stuck-bias gates cannot detect a bias | High | Confirmed | static capture, driver and robot_localization source | 04 I1, 05 E3 |
| 6 | GNSS covariance optimistic in FLOAT (reported 1.6 cm, measured ~10 cm) → map pose follows GNSS noise | High | Confirmed | static capture | 06 G2, 05 E1 |
| 7 | Command priority between web UI and Nav2 not implemented | High | Confirmed | source | 02 D1 |
| 8 | GPS rejection threshold 13.8 is a χ² value in a σ parameter → 13.8σ gate | Medium | Confirmed | robot_localization source | 05 E4 |
| 9 | MPPI acceleration limits are not read on Humble; MPPI assumes instant velocity changes | Medium | Confirmed | binary strings, parameter query | 02 D6 |
| 10 | SPARK MAX device parameters (current limit, ramps, filter, voltage compensation) unverified; gains not re-applied after a controller reset | High | Confirmed gap | firmware source, FINDINGS.md checklist | 01 M3 |
| 11 | Heading-hold overrides MPPI for \|ω\| < 0.05 rad/s | Medium | Hypothesis | source; test V9 | 02 D2 |
| 12 | Skid multiplier compensates motor under-delivery; calibrated in spins on pavement only | Medium | Probable | 2026-05-18 decomposition; literature | 02 D7 |
| 13 | Lateral velocity unconstrained; 0.08 m/s measured at rest | Medium | Confirmed | static capture | 05 E2, 03 O3 |
| 14 | Wheel distance scale never measured | Medium | Gap | — | 03 O5 |
| 15 | Map-frame heading offset never measured; IMU heading needs motion after power-up | Medium | Gap | team records | 06 G4, 04 I2 |
| 16 | No track-speed desaturation; curvature distorts when one track saturates | Medium (manual) / Low (Nav2) | Confirmed | source | 02 D5 |
| 17 | FLOAT and FIXED weighted identically downstream | Medium | Confirmed | driver source | 06 G3 |
| 18 | Absolute IMU yaw in the odom EKF can inject heading steps into the odom frame | Low–Medium | Hypothesis | canonical but risky with this IMU | 05 E5 |
| 19 | Supply voltage and SPARK MAX fault flags not reported to ROS | Medium | Confirmed | firmware source | 01 M4 |
| 20 | Odometry stamped at publish time; E-lines have no timestamp or sequence | Medium / Low | Confirmed | source | 03 O2, 01 M6 |
| 21 | Heading-hold output bypasses angular slew; angular decel uses accel cap | Low | Confirmed | source | 02 D3, D4 |
| 22 | Documentation errors: CLAUDE.md (wheel vyaw fused, 4σ GPS gate, MPPI accel matched, `max_angular_rps` 1.0, Nav2 #5524 citation), `navsat.yaml` comments, odometry rate "50 Hz" | Low | Confirmed | this analysis | 00, 02 D8, 05 E9 |

## 3. Recommended fix order

Ordered so each step is measurable and does not mask the next. Each step names the test that proves it.

**Step 1 — Instrument and verify (no behavior change).**
- Audit both SPARK MAX parameter sets and record them (M3).
- Add a timestamp, sequence number, bus voltage and fault flags to the E-line (M4, M6).
- Proves: known device state. Test: parameter dump committed next to this folder.

**Step 2 — Motor layer.**
- Bench test M1 (V2 procedure). If confirmed, set per-track kV (~0.0022 V/RPM) and kS, re-tune kP, reduce kI.
- Shorten the hall velocity filter (M2).
- Persist parameters, and re-apply them on SPARK reset.
- Proves: V1 lag < 40 ms; V2 delivery > 98% without relying on I; V3 rotation delivery.

**Step 3 — Odometry and kinematics.**
- Integrate encoder position deltas at 50 Hz with measurement-time stamps (O1, O2).
- Publish and fuse vy = 0 with σ ≈ 0.05–0.1 m/s (O3).
- Re-measure the skid multiplier on pavement and grass, spins and arcs (D7).
- Implement command priority (D1) and restrict heading-hold to the manual path (D2).
- Add desaturation (D5).
- Proves: V3, V4, V9, V10.

**Step 4 — Estimation inputs.**
- Add the GNSS antenna frame (G1).
- Set IMU covariances from measured noise (I1).
- Inflate GNSS covariance by RTK state (G2, G3).
- Proves: V5 (rotation, < 5 cm), V7 (reported vs measured σ within 0.5–2×).

**Step 5 — Filter tuning.**
- Set map-EKF x/y `initial_estimate_covariance`, then the GPS gate to 3.72 (E4).
- Re-tune process noise from V7 data. Consider differential IMU yaw in the odom EKF (E5).
- Add a stuck-bias monitor (I3).
- Proves: V0 repeat (map pose span < 5 cm with RTK FIXED), V8 (< 10 cm RMS).

**Step 6 — Planner model.**
- Either build MPPI with acceleration constraints (PR #4352) and match them to the actuator, or remove the dead parameters and narrow the sampling std (D6).
- Correct the documentation (item 22).

## 4. What this analysis did not cover
- Motion tests: no command was sent to the robot. All moving-robot validation is specified in 08.
- The flashed Teensy binary (source matches the repository; the binary cannot be read back).
- Perception, planning and costmap behavior beyond their dependence on the pose.
