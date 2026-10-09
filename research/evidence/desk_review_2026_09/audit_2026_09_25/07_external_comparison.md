# 07 — External Comparison

How each layer compares with vendor guidance, the robot_localization and Nav2 reference designs, the skid-steer literature, commercial platforms (Clearpath), and 13 IGVC teams. Sources are in `external/01`–`04`.

Rating: **Ahead** = better than any comparable source found; **Standard** = matches recommended practice; **Behind** = below recommended practice or published peers.

## 1. Layer-by-layer

| Layer / topic | Our implementation | Reference practice | Rating | Detail |
|---|---|---|---|---|
| Velocity loop placement | SPARK MAX onboard PID over CAN, commanded by a Teensy | Onboard controller loop is the IGVC norm (Sooner 2025, U. Michigan, Oakland, TnTech); host-side loops (Toronto) are weaker | Standard | 01 |
| Feedforward | param 16 = 0.000197, set as duty/RPM | FW 2026 param 16 is kV in V/RPM; REV template 12/free speed ≈ 0.0021; REV tunes feedforward first and discourages I | **Behind** (probable unit error) | 01, M1 |
| Velocity measurement | default NEO hall filter; 185 ms measured lag | Teams set 16 ms / depth 2 (3005, 620); Oakland added external encoders because the NEO encoder was inadequate | **Behind** | 01, M2 |
| Device parameter management | 5 gains pushed; others unverified in flash | REV: persist configuration; `sparklib-py` provides audit tooling; Sooner's CONBus reads back parameters | **Behind** | 01, M3 |
| Setpoint rate limiting | Teensy setpoint ramp + host slew | REV: output ramp winds up the integrator; setpoint ramp preferred | Standard | 01, M5 |
| Live tuning | `ros2 param set` → Teensy → SPARK MAX, with ack | Rare at IGVC (Sooner CONBus is the only equivalent found) | Ahead | 01 |
| Skid correction | Measured multiplier (1.19), validated by square test | `diff_drive_controller` convention; Clearpath 1.125–1.875; no IGVC team documents one | Ahead in method; **non-standard** in asymmetric use | 02, D7 |
| Heading-hold under a planner | Actuator P-loop on IMU yaw, active under MPPI | Literature puts the slip model low and path tracking above; no inner heading loop found | Behind (for the Nav2 path) | 02, D2 |
| Planner dynamic model | MPPI on Humble; accel limits configured but not read | Acceleration constraints exist only after Nav2 PR #4352 | **Behind** (dead parameters) | 02, D6 |
| Command arbitration | shared targets, no priority | explicit mux (e.g. `twist_mux`) with priorities | **Behind** | 02, D1 |
| Wheel odometry source | integrates reported velocity | integrate position deltas (`diff_drive_controller`, WPILib) | **Behind** | 03, O1 |
| Wheel odometry in EKF | vx only; IMU owns yaw | U. Michigan identical; robot_localization docs also fuse vy = 0 | Standard; vy missing | 03, O3 |
| Distance scale calibration | nominal pulley diameter | UMBmark / Mandow straight-line calibration | Behind | 03, O5 |
| IMU covariance | zero (driver default) | realistic covariance required for absolute fusion (robot_localization docs; Autoware notes numerical problems with zero) | **Behind** | 04, I1 |
| IMU readiness | manual warm-up rule | RoboJackets waited a fixed minute; automated gate not seen | Standard | 04, I2 |
| Dual-EKF structure | odom EKF (continuous) + map EKF (+GPS absolute) | exactly the robot_localization / Nav2 reference | Standard | 05 |
| Outlier gating | IMU 5σ, GPS "13.8" | correct value √χ² (3.72 for 2 DOF, 99.9%); gates require realistic R | **Behind** (gates ineffective) | 05, E3/E4 |
| GNSS lever arm | not applied (0.76 m measured) | antenna frame in TF; `navsat_transform` removes it | **Behind** | 06, G1 |
| GNSS covariance | receiver hAcc, optimistic in FLOAT | inflate by fix state; GNSS/INS output is time-correlated (PX4 RFC) | Behind | 06, G2 |
| RTK | NTRIP, EarthScope, cm-level when FIXED | on par with Wayne State, Oakland, Cedarville, Hosei | Standard | 06 |
| Datum | hand-entered | U. Michigan: automatic median datum with accuracy gate | Behind (minor) | 06 |
| Heading source | single antenna, GNSS-derived | dual-antenna moving base (Hosei, U. Michigan) | Standard (hardware limit) | 04 |
| Validation data | square closure, delivery sweeps, static capture | best published: LTU 0.5–0.8 m RMS, VT within 6 in; most teams publish datasheet numbers only | Ahead | 08 |

## 2. Summary

**Ahead of published peers:** measured skid correction with a validation test, live parameter tuning with acknowledgement, and the volume of quantitative validation data. No IGVC team found documents comparable measurements.

**Standard:** the overall architecture (onboard velocity loop, dual EKF, absolute RTK in the map EKF, IMU-owned yaw, velocity-only wheel fusion) matches the robot_localization, Nav2 and Clearpath references.

**Behind:** almost every deficit is an integration detail at a layer boundary rather than an architectural choice:
- Units changed by a firmware update (M1).
- A vendor-default filter delay (M2).
- A driver default of zero covariance (I1).
- An antenna offset not carried into TF (G1).
- A threshold in the wrong units (E4).
- Planner parameters the installed version ignores (D6).

Each is cheap to fix. None is visible without measuring, which is why they survived the earlier tuning sessions.

**Architectural note from the IGVC survey:** the most successful teams (Sooner, three consecutive wins) used simpler estimators, and several teams (TnTech, U. Michigan) keep GPS out of the transform that the costmap uses. Our stack is more sophisticated than typical; its advantage depends on the inputs being correct, which is what this analysis addresses.
