# Research Plan: Controls and Localization

**Goal:** build an evidence base for making the robot's control and localization correct and precise before any other autonomy work. This phase is research only. Comparing it with our code is the next phase.

**Scope of each topic:** general principles and methods first (control theory, estimation, calibration, testing, lessons from other robots and vehicles), then the specifics of the hardware and software we use as one part. Research from other vehicle types is included when the principle carries over.

**Why these topics:** they follow the signal chain. Control runs from the navigation controller down to the motors; localization runs from the sensors to one position estimate. Every link in each chain gets one topic, so every question has one home.

```
CONTROL        C4 controller–vehicle interface → C3 command pipeline → C1 motor velocity control → C2 drive kinematics
LOCALIZATION   L1 wheel odometry ─┐
               L2 IMU and heading ├→ L4 sensor fusion      (L5 time sync underlies all)
               L3 GNSS / RTK ─────┘
ACROSS BOTH    X1 reference stacks · X2 test and validation methods
```

## Topics and research questions

### C1 — Motor velocity control
1. How does a brushless motor controller's velocity loop work (feedforward, P, I, D), and in what units are the gains defined?
2. How are feedforward values (static friction, speed, acceleration terms) measured (system identification)?
3. What is the recommended tuning order, and what limits the feedback gains (measurement lag, filtering)?
4. How are speed measurement filters, current limits and voltage compensation set?
5. What limits precision at low speed (friction, sensor resolution)?

### C2 — Drive kinematics
1. How do skid-steer and tracked vehicles move for given track speeds (turning centre, slip)?
2. What is the "effective track width" correction, and how does it vary with surface, speed and turn radius?
3. How are kinematic parameters calibrated?
4. Where is the rotation centre relative to the chassis, and what depends on it?

### C3 — Command pipeline
1. How do reference systems prioritise autonomy, teleop and e-stop commands, and hand control back and forth safely?
2. Where should speed and acceleration limits sit (navigation stack, smoother, motor controller)?
3. Why is an extra correction loop (e.g. heading-hold) below the navigation controller discouraged, or when is it acceptable?
4. How are stale commands, watchdogs and stopping (active braking vs coasting) handled?
5. What loop rates and delays do reference systems budget for?

### C4 — Controller–vehicle interface
1. What does a sampling-based path-tracking controller (MPPI) assume about the vehicle (response, acceleration limits)?
2. How are controller limits matched to what the chassis can deliver?
3. How does command delay or under-delivery affect tracking accuracy?
4. How do reference stacks feed measured velocity back to the controller?

### L1 — Wheel odometry
1. How is travelled distance and heading computed from encoders, and what are the main error sources?
2. How is odometry calibrated (distance scale, track width; e.g. square-path tests)?
3. Encoder position vs reported velocity; timestamping; covariance values.
4. How large are odometry errors typically for skid-steer / tracked vehicles?

### L2 — IMU and heading
1. How does a GNSS-aided IMU (e.g. Xsens MTi-680G) initialise and maintain heading without a magnetometer?
2. What happens with reverse driving, standstill, and slow driving? How is the filter reset?
3. Magnetometer-based heading: distortion, magnetic field mapping, in-run calibration, achievable accuracy.
4. Single-antenna vs dual-antenna heading options.
5. What IMU noise and covariance values are appropriate for fusion?

### L3 — GNSS / RTK
1. What determines RTK fix quality (satellites, signal strength, multipath, baseline)?
2. Antenna placement and ground plane requirements.
3. Correction delivery (NTRIP): message types, update rates, connection timeouts and reconnection.
4. FLOAT vs FIXED: accuracy, time to fix, how applications should treat each.
5. Antenna offset (lever arm) handling.

### L4 — Sensor fusion
1. Standard frame structure for mobile robots (map, odom, base_link) and why.
2. Recommended robot_localization setups (single vs dual filter, navsat_transform, absolute vs differential inputs).
3. How to set sensor noise and covariance, initial uncertainty and outlier rejection.
4. Handling GPS quality changes (FLOAT/FIXED), and delayed measurements.
5. Known failure modes and diagnostics.

### L5 — Time synchronization
1. How timestamp errors affect sensor fusion.
2. How robots keep clocks aligned (NTP, chrony, GPS/PPS), and what accuracy each achieves.
3. How sensor drivers stamp data (sensor time vs arrival time).

### X1 — Reference stacks
1. How top IGVC teams (recent design reports) structure motor control, odometry, IMU, GPS and fusion, and the accuracy they report.
2. How professional robots (Clearpath Husky/Jackal/Warthog and similar open stacks) configure control and localization.

### X2 — Test and validation methods
1. Standard tests for velocity control (step response, tracking error), odometry (UMBmark, square/loop closure), localization (static/dynamic accuracy against RTK ground truth).
2. Metrics and pass criteria used in the literature and by reference teams.

## Method

**Each topic goes through five steps** (details in `STANDARDS.md` section 5):
1. **Map:** understand the field, break the topic into subtopics so nothing is missed, and identify the foundational references (`SCOPE.md`).
2. **Research:** download foundational references first, then other reputable sources; write `README.md` with every finding cited.
3. **Gap check:** an independent researcher compares the result with the map and fills gaps.
4. **Verify:** an independent reviewer checks every finding and every source (`VERIFICATION.md`).
5. **Correct:** fix or remove flagged items; the topic is marked Verified.

**Workflows, run one at a time; each researches, verifies and corrects its topics:**

| Workflow | Topics | Status |
|---|---|---|
| W1 | C1, C2 | done — both Verified (C1: 169 findings, 27 sources; C2: 143 findings, 20 sources); general extension done — C1: 281 findings, 37 sources; C2: 231 findings, 34 sources; both Verified |
| W2 | C3, C4 | done — C3: 190 cited findings, 64 files; C4: 194 findings, 51 files; both Verified |
| W3 | L1, L2 | done — L1: 251 findings, 42 sources; L2: 253 findings, 47 sources; both Verified |
| W3b | Re-check: C1, C2 (map + gap check + two-reviewer verify), C4 (verify after targeted additions) | done — C1: 364 findings, 51 sources; C2: 320 findings, 41 sources; C4: 235 findings, 54 sources; all Verified |
| W4 | L3, L4, L5 | done — L3: 267 findings, 52 sources; L4: 428 findings, 65 sources; L5: 378 findings, 67 sources; all Verified |
| W5 | X1, X2 | running |
| W6 | Compile the master reference list, check for duplicates and contradictions between topics, write the summary | planned |

**Note on W1:** C1 and C2 started before the scope was widened. After W1 finishes, both get an extra research pass for the general side (control and estimation theory, other vehicle types), followed by the same verification. Both also get the Map and Gap-check steps (added after W1) in a later pass.

**Starting material:** 16 sources collected in an earlier attempt (REV, WPILib, ros2_control, u-blox, Xsens, Borenstein, Kozlowski, Baril) are already in the matching `sources/` folders. They get the same checks as new sources.

---

## Drivetrain architecture alternatives (D1-D5)

**Goal:** a separate question from tuning the current stack — whether a different drivetrain architecture would reach precise, repeatable motion with less custom kinematic correction than the current tracked skid-steer chassis needs, while suiting IGVC's outdoor terrain and a standard ROS 2 Nav2 stack. Research only; comparison and recommendation is a later phase. Context file: `tools/context_drivetrain_options_2026_10.md`.

### D1 — Swerve drive (independent steer + drive modules)
1. How does swerve/crab drivetrain kinematics work (independent per-wheel steering + drive, inverse/forward kinematics, singularities, wheel-coupling effects)?
2. What control and odometry techniques deliver precise motion and heading with swerve (closed-loop steering-angle control, drive-wheel velocity control, odometry fusion, known failure modes e.g. wheel skew)?
3. What off-the-shelf swerve module products exist, their specs, cost, motors/controllers/encoders, and outdoor ingress protection (or lack of it)? Starting points: REV MAXSwerve/EasySwerve, Swerve Drive Specialties MK4/MK4i/MK4n/MK4c, West Coast Products SwerveX/SwerveX2/SwerveXFlipped, ThriftyBot Thrifty Swerve, ARMABot Differential Swerve — plus any industrial/AGV independent-steer equivalents.
4. How mature is ROS 2 / Nav2 support for a holonomic swerve base (odometry plugins, ros2_control hardware interfaces, controller plugins that assume holonomic vs. differential/Ackermann kinematics)?
5. What do documented outdoor deployments (if any) report about grass, mud, dirt and water versus the flat indoor carpet swerve is normally used on?
6. What does published literature/design reports say about swerve odometry/heading-hold precision versus skid-steer and Ackermann?

### D2 — Wheeled skid-steer / differential drive (non-tracked)
1. How does wheeled skid-steer/differential-drive kinematics work, and how does its slip/scrub behavior during rotation differ physically from a tracked skid-steer (contact-patch geometry, scrub torque, effective-width correction magnitude)?
2. What published calibration methods exist for the effective-track-width/skid correction (Mandow et al. and any follow-on or alternative work), and do they report smaller or more predictable corrections for wheels than tracks?
3. What off-the-shelf wheeled skid-steer platforms/kits exist at a similar scale (Clearpath Husky/Jackal/Warthog-class designs, AndyMark/VEX wheeled skid-steer kits, independent builds), their motors/controllers/odometry approach, and cost?
4. How mature is ROS 2 / Nav2 / ros2_controllers support for wheeled differential/skid-steer bases, and how does that compare to the custom kinematic layer tracked platforms typically need?
5. What do field reports/design papers say about wheeled skid-steer traction and precision on grass, dirt and gravel versus tracks?

### D3 — Ackermann / car-like steering
1. How does Ackermann (single- or dual-axle) steering kinematics work, and what are its precision/turning-radius/odometry characteristics versus skid-steer?
2. What off-the-shelf steering/chassis platforms or kits exist for a small-to-mid autonomous ground vehicle (RC-car-based, golf-cart-based, purpose-built rovers), their actuators, and cost?
3. How mature is ROS 2/Nav2 support for Ackermann-steered bases (Regulated Pure Pursuit's original target, ackermann_msgs, ros2_controllers steering controllers, any MPPI/MPC support for Ackermann kinematics)?
4. What do IGVC and similar outdoor-autonomy design reports/retrospectives say about real-world turning radius versus typical IGVC lane widths (2-3 m) and obstacle density?
5. What does field robotics/agricultural robot/planetary rover literature say generally about Ackermann precision, dead-reckoning accuracy and outdoor terrain performance versus skid-steer?

### D4 — Omnidirectional (mecanum / omni-wheel) drivetrains
1. How does mecanum/omni-wheel kinematics work, and what mechanical/control factors limit its precision (roller geometry, lateral slip, uneven-terrain roller-contact loss)?
2. What published literature/field reports say about mecanum/omni-wheel traction and reliability on unpaved or outdoor terrain versus the smooth indoor floors they are normally used on?
3. What off-the-shelf mecanum/omni-wheel drivetrain kits/components exist for a mid-size ground robot, their cost, and typical motor/controller/odometry approach?
4. How mature is ROS 2/Nav2/ros2_controllers support for a holonomic mecanum base?
5. Net verdict for an outdoor grass/dirt competition use case.

### D5 — Tracked drive: alternatives and fixes
1. What published research (beyond Mandow et al. 2007) exists on tracked-vehicle skid-steer kinematics and correction factors, and how they vary with surface, speed and track tension — does it suggest the correction/tuning burden is fundamental to tracks or substantially reducible?
2. What other rubber-track or tracked-chassis products/kits exist at a similar scale (beyond the current AndyMark Raptor platform), their drivetrain integration approach, and reported precision/tuning experience?
3. What do other IGVC teams' and field-robotics design reports say about tracked platforms' real-world precision, maintenance burden and suitability for grass/mud/gravel versus wheels?
4. What techniques (beyond empirical multiplier calibration) do published sources recommend for improving tracked-vehicle heading/velocity precision (model-based slip estimation, sensor-fusion slip correction, track-tension monitoring)?
5. Net assessment: when do sources say tracks remain the right choice despite kinematic complexity, and when do they recommend switching away?

| Workflow | Topics | Status |
|---|---|---|
| W7 | D1, D2, D3 | done — D1: 153 findings, 43 sources; D2: 134 findings, 38 sources; D3: 134 findings, 45 sources; all Verified |
| W8 | D4, D5 | done — D4: 136 findings, 39 sources; D5: 97 findings, 27 sources; both Verified (first attempt interrupted/stalled by session events, resumed clean) |

**All five topics Verified.** Synthesis and recommendation: `docs/drivetrain_architecture_analysis_2026_10_06.md`.
