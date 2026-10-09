# L1 — Wheel odometry

| | |
|---|---|
| **Question** | How are distance and heading computed from wheel or track encoders, how large are the errors, and how are they calibrated, modelled and reported? |
| **Covers** | Dead-reckoning equations and integration, systematic and non-systematic error sources, error propagation and covariance, calibration methods (UMBmark and successors), encoder position vs velocity and filtering lag, skid-steer/tracked odometry and slip detection, test methods, and how odometry is reported in ROS (REP-105, `nav_msgs/Odometry`, `diff_drive_controller`). |
| **Not covered** | Derivation of skid-steer kinematic models (C2), fusion with IMU/GNSS (L4), and IMU heading (L2); these are used here only where they touch odometry accuracy, calibration or slip detection. |
| **Status** | Verified |
| **Last updated** | 2026-09-27 (corrections after verification) |

Page numbers in citations are the page numbers of the downloaded PDF file (not always the printed journal page). Where the two differ, the printed page is given in brackets, e.g. `p. 25 (j. 202)`.

## Summary
- Odometry integrates small wheel displacements into a pose, so its error grows without bound; heading errors are the main concern because they turn into lateral position errors that keep growing with distance [L1-S03, p. 130] [L1-S15, §1.1].
- Errors split into **systematic** errors (unequal wheel diameters, wrong average diameter, wrong effective wheelbase/track width, misalignment, finite encoder resolution and sampling rate) and **non-systematic** errors (uneven ground, objects, wheel slip and skid) [L1-S02, p. 3] [L1-S03, pp. 130–131]. Systematic errors are properties of each robot and can be reduced by vehicle-specific calibration [L1-S03, p. 139]; the robot cannot be calibrated to compensate for non-systematic errors [L1-S15, p. 5].
- For wheel odometry with random, zero-mean per-wheel errors on a straight path, and with identical noise statistics on both wheels, heading and along-track variance grow linearly with distance while cross-track variance grows with the cube of distance [L1-S04, §9.1.3, p. 25 (j. 202)] [L1-S06, p. 7].
- The UMBmark bidirectional 4×4 m square test identifies the wheel-diameter ratio and effective wheelbase; correcting just these two reduced systematic error 10- to 22-fold on a differential-drive robot [L1-S01, pp. 22–23]. Later methods calibrate from arbitrary paths (Kelly, Censi, Seegmiller) [L1-S04, §10.4] [L1-S08, p. 1] [L1-S14, pp. 18–19] or from one path driven forward and then backward (Doh's PC-method) [L1-S13, p. 6].
- Skid-steer and tracked vehicles must slip to turn, so a pure-rolling model over-predicts yaw rate [L1-S22, p. 7]. One fix is an enlarged effective track width (ICR model) identified per terrain [L1-S19, pp. 3, 5]; extended differential-drive models of this kind are among the models "commonly deployed" for skid-steer robots [L1-S20, p. 1]. Slip can also be detected from gyro, redundant encoders or motor current [L1-S21, §II] [L1-S18, p. 3].
- In ROS, the `odom` frame must be continuous and is allowed to drift; `nav_msgs/Odometry` carries pose (in `header.frame_id`) and twist (in `child_frame_id`) each with covariance [L1-S09, §odom] [L1-S11]; when wheel-odometry position, heading and velocity are all generated from the same encoders, robot_localization advises fusing only the velocities [L1-S27, odom0 example, item 1].

## Foundational references
| ID | Reference | Why it is foundational |
|---|---|---|
| L1-S01 | Borenstein & Feng, "Measurement and Correction of Systematic Odometry Errors in Mobile Robots," IEEE T-RA 12(6), 1996 | Defines E_d, E_b, UMBmark and the correction procedure every later calibration method is compared against. |
| L1-S02 | Borenstein & Feng, "UMBmark: A Benchmark Test for Measuring Odometry Errors in Mobile Robots," Proc. SPIE 2591, 1995 | Original benchmark paper; error taxonomy and extended (non-systematic) UMBmark. |
| L1-S03 | Borenstein, Everett & Feng, *"Where am I?" Sensors and Methods for Mobile Robot Positioning*, U. Michigan, 1996 | Standard survey; odometry equations (ch. 1) and odometry/dead-reckoning chapter (ch. 5), tracked vehicles. |
| L1-S04 | Kelly, "Linearized Error Propagation in Odometry," IJRR 23(2), 2004 | General linearised solution for systematic and random odometry error on any path. |
| L1-S05 | Chong & Kleeman, "Accurate Odometry and Error Modelling for a Mobile Robot," ICRA 1997 (file: Monash tech. report MECSE-1996-6) | Per-wheel noise model (variance ∝ distance) and closed-form covariance for lines, arcs and turns, with experiments. |
| L1-S06 | Kleeman, "Odometry Error Covariance Estimation for Two Wheel Robot Vehicles," Monash MECSE-95-1, 1995 | Closed-form covariance derivation behind L1-S05. |
| L1-S07 | Siegwart & Nourbakhsh, *Introduction to Autonomous Mobile Robots*, 1st ed., MIT Press 2004, ch. 5 §5.2 | Textbook odometry update and error-propagation model (k_r, k_l). |
| L1-S08 | Censi, Franchi, Marchionni & Oriolo, "Simultaneous Calibration of Odometry and Sensor Parameters," IEEE T-RO 29(2), 2013 | Closed-form maximum-likelihood calibration with no special path; reviews calibration families. |
| L1-S18 | Reina, Ojeda, Milella & Borenstein, "Wheel Slippage and Sinkage Detection for Planetary Rovers," IEEE/ASME T-Mech 11(2), 2006 | Standard set of slip indicators (encoder, gyro, motor current) for rough-terrain odometry. |
| L1-S09, L1-S10, L1-S11 | REP-105 "Coordinate Frames for Mobile Platforms", REP-103 "Standard Units of Measure and Coordinate Conventions" and `nav_msgs/Odometry` | Official ROS definitions of the `odom` frame, units and axis conventions, and the odometry message. |
| not downloaded | C. M. Wang, "Location Estimation and Uncertainty Analysis for Mobile Robots," IEEE ICRA 1988 | First derivation of the odometry pose estimator and its covariance (per SCOPE.md); no open copy. Described here only through L1-S12. |
| not downloaded | G. Antonelli, S. Chiaverini, G. Fusco, least-squares odometry calibration, IEEE T-RO 21(5), 2005 | Standard least-squares calibration method (per SCOPE.md); no open copy. Described here only through L1-S08 and L1-S14. |
| not downloaded | A. Martinelli, N. Tomatis, R. Siegwart, "Simultaneous Localization and Odometry Self Calibration for Mobile Robot," Auton. Robots 22, 2007 | Online augmented-Kalman-filter self-calibration (per SCOPE.md); no verified open copy. The related ECMR 2003 paper (L1-S12) is used instead. |
| not downloaded | R. Siegwart, I. Nourbakhsh, D. Scaramuzza, *Introduction to Autonomous Mobile Robots*, 2nd ed., MIT Press, 2011 | Standard textbook odometric error model; no licensed open copy. The 1st-edition chapter (L1-S07) is used instead. |
| not downloaded | S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics*, MIT Press, 2005, ch. 5 | Standard probabilistic odometry motion model (α1–α4); no licensed open copy. Described here through its Nav2 AMCL implementation (L1-S37, L1-S38) and L1-S36. |
| not downloaded | J. L. Martínez, A. Mandow, J. Morales, S. Pedraza, A. García-Cerezo, "Approximating Kinematics for Tracked Mobile Robots," IJRR 24(10), 2005 | Seminal ICR-based tracked-vehicle kinematics; no verified open copy. The ICR method is used via L1-S19. |

## Findings

### 1. Dead-reckoning principles and integration methods
- Encoder counts are converted to wheel travel with a factor c_m = πD_n/(nC_e) (nominal wheel diameter D_n, gear ratio n, encoder counts per revolution C_e); the centre-point displacement is the mean of the two wheel displacements and the heading change is their difference divided by the wheelbase b [L1-S03, p. 20, eqs. 1.2–1.5].
- The basic update adds the displacement along the current heading: x_i = x_{i−1} + ΔU_i cos θ_i, y_i = y_{i−1} + ΔU_i sin θ_i [L1-S03, p. 20, eqs. 1.6–1.7].
- The textbook version evaluates the displacement at the mid-point heading θ + Δθ/2 [L1-S07, §5.2.4, p. 7]; the `diff_drive_controller` source labels this mid-point form "Runge-Kutta 2nd order integration" [L1-S30, odometry.cpp lines 135–143].
- `diff_drive_controller` (Humble release 2.54.0) uses exact arc integration (x += r·(sin θ_new − sin θ_old), with r = Δs/Δθ) and falls back to the mid-point (RK2) form when |Δθ| < 1e-6 [L1-S30, odometry.cpp lines 135–160].
- Replacing straight segments by circular arcs within each sampling period brings "relatively small" benefits (Larsson et al., as summarised by Borenstein) [L1-S03, p. 138].
- Kelly describes odometry as a "forced dynamics" system: measured speeds act as inputs, and the pose is their integral, so error propagation is dynamic (integral) rather than algebraic as in triangulation [L1-S04, pp. 2–3 (j. 179–180)].
- Error propagation stops when motion stops; some systematic errors cancel on closed paths, and some can be reversed by driving back over the same path [L1-S04, p. 2 (j. 179); p. 40 (j. 217)].
- Heading errors dominate: "once incurred, orientation errors grow without bound into lateral position errors" [L1-S15, §1.1]; a move of d metres with heading error Δθ adds a lateral error of d·sin Δθ [L1-S07, §5.2.3, p. 6].
- On uneven terrain with ramps (Zoë rover), a 3D kinematic model calibrated to data logs gave "less than .25m and 2.3° error after traveling up to 201.5m", without a gyro; 2D models showed yaw-error spikes when crossing ramps [L1-S22, p. 6].

### 2. Error sources and taxonomy
- Systematic sources: unequal wheel diameters, average diameter different from nominal, wheel misalignment, uncertain effective wheelbase (non-point contact), limited encoder resolution, limited encoder sampling rate [L1-S02, p. 3] [L1-S03, pp. 130–131].
- Non-systematic sources: uneven floors, unexpected objects, and wheel slippage from slippery floors, over-acceleration, fast turning (skidding), external and internal forces, and non-point wheel contact [L1-S02, p. 3] [L1-S03, p. 131].
- On most smooth indoor surfaces systematic errors contribute much more than non-systematic errors; on rough surfaces with significant irregularities non-systematic errors dominate [L1-S03, p. 131].
- In differential-drive robots the two most "notorious" systematic sources are unequal wheel diameters (E_d = D_R/D_L) and wheelbase uncertainty (E_b = b_actual/b_nominal); wheelbase uncertainty "can be on the order of 1% in some commercially available robots" [L1-S02, p. 3] [L1-S01, p. 6].
- The scaling error E_s (average diameter vs nominal) is easy to measure with a tape measure to 0.3–0.5% of full scale and is corrected before UMBmark [L1-S02, p. 4].
- E_d makes nominally straight legs curve; E_b makes each turn too large or small; in a one-direction square test both can produce the same end error and cancel each other [L1-S02, pp. 4–5] [L1-S03, pp. 133–134].
- Siegwart groups errors geometrically into range, turn and drift errors, and states that over long periods turn and drift errors "far outweigh" range errors [L1-S07, §5.2.3, p. 6].
- A 10 mm bump under one wheel of a TRC LabMate gives an orientation error of roughly 0.6° [L1-S02, p. 6]; larger bumps gave orientation errors "on the order of 0.5° − 0.8°" in the Gyrodometry tests [L1-S15, §2, p. 4].
- Design factors: a small wheelbase makes a robot more prone to orientation error; odometry wheels should ideally be thin ("knife-edge") and non-compressible; heavily loaded castors induce slip when reversing [L1-S03, pp. 137–138].
- Kinematic-only models were accurate only up to about 0.3 m/s in a tight turn in the simulations of Boyden and Velinsky, as reported by Borenstein; limiting turning speed and acceleration reduces slip errors [L1-S03, p. 138].
- Over-constrained vehicles (more independent motors than degrees of freedom, e.g. four-wheel skid-steer with separate motors) suffer extra slip whenever wheel speeds momentarily mismatch the kinematic constraint [L1-S16, p. 6] [L1-S18, p. 2].
- Most slip and bumps cause encoders to **over-count**; skidding (a wheel partly blocked by drive-train friction or braking) produces **fewer** pulses than a tracking wheel [L1-S16, pp. 4–6] [L1-S18, p. 1].

### 3. Error modelling and covariance propagation
- Chong and Kleeman model each wheel's error as zero-mean white noise whose variance is proportional to the distance that wheel travels: σ_L² = k_L²·d_L, σ_R² = k_R²·d_R, with the two wheels uncorrelated [L1-S05, pp. 6–7, eq. 4] [L1-S06, p. 3, eq. 1].
- Instead of updating covariance in small time steps, they integrate the noise analytically over whole segments, giving closed-form covariance for a straight line, a constant-curvature arc and a turn about the axle centre; other paths are built from short arcs [L1-S06, p. 1] [L1-S05, p. 1].
- On a straight line, the standard deviations of heading and along-track error grow with the square root of distance, and the perpendicular (cross-track) standard deviation grows with distance to the power 1.5 [L1-S06, p. 7].
- Kelly's general linearised solution gives the same pattern for differential-heading (wheel) odometry on a straight path when both encoders have identical noise statistics: heading and along-track variance linear in distance, cross-track variance cubic [L1-S04, §9.1.3, p. 25 (j. 202)].
- In Kelly's analysis, biases propagate in a time-dependent way while scale errors propagate in a motion-dependent way; systematic velocity scale error cancels on closed trajectories while random velocity error keeps growing in total variance [L1-S04, p. 40 (j. 217)].
- Lateral wheel slip propagates like a scale error on angular velocity (like a gyro scale error) [L1-S04, p. 30 (j. 207)].
- Kelly found "good gyros will tend to outperform differential heading odometry" [L1-S04, p. 40 (j. 217)].
- Linearised error dynamics matched full nonlinear simulation to within 1 cm or 3% of the error magnitude on a 210 s test trajectory, and Monte Carlo variances matched the theory [L1-S04, p. 39 (j. 216)].
- Siegwart's textbook model uses the covariance of the wheel increments Σ_Δ = diag(k_r|Δs_r|, k_l|Δs_l|) and propagates Σ_p' = F_p Σ_p F_pᵀ + F_Δ Σ_Δ F_Δᵀ at each step; k_r and k_l "should be experimentally established" [L1-S07, §5.2.4, p. 8].
- In these models the uncertainty perpendicular to the direction of travel grows much faster than along it, and on circular paths the long axis of the error ellipse does not stay perpendicular to the direction of motion [L1-S07, p. 10, Figs. 5.4–5.5].
- The Chong–Kleeman first-order model stays accurate for moderate k values; at k_L = k_R = 0.05 m^1/2 over about 10 m it "performs marginally good" and beyond that "a second order model becomes necessary" [L1-S05, p. 15].
- Measured per-wheel noise on a flat parquet floor was k_L = 0.00040 m^1/2 and k_R = 0.00058 m^1/2, and these values depend on the floor material [L1-S05, p. 17].
- The model could not reproduce all six covariance elements: the small cross-track error was very sensitive to residual bias and outliers [L1-S05, p. 17]; it also cannot represent bump events, which violate the flat-floor assumption [L1-S05, p. 19].
- Error-ellipse methods can only include systematic behaviour learned from observations, because the size of non-systematic errors is unpredictable [L1-S03, p. 131].
- Martinelli and Siegwart estimate both parts online: an augmented Kalman filter holds the pose plus systematic parameters, and a second "Observable Filter" estimates the non-systematic parameters from pairs of successive pose estimates (results in simulation) [L1-S12, pp. 1–5].
- **Probabilistic odometry motion model (Thrun et al.), as implemented in Nav2 AMCL:** the code comments that it implements `sample_motion_odometry` from *Probabilistic Robotics* (p. 136); the odometry change between two updates is split into an initial rotation, a translation and a final rotation [L1-S37, lines 50–70].
- Each of the three parts is perturbed with zero-mean Gaussian noise whose variance is a weighted sum of squared motions: rotations use α1·rot² + α2·trans², the translation uses α3·trans² + α4·(rot1² + rot2²) [L1-S37, lines 86–103].
- The Nav2 documentation names these weights: α1 "rotation estimate from rotation", α2 "rotation estimate from translation", α3 "translation estimate from translation", α4 "translation estimate from rotation" (α5 is for omnidirectional models only); all default to 0.2 [L1-S38, §Parameters].
- The Nav2 implementation departs from the textbook form in two places: it treats backward motion like forward motion by measuring each rotation relative to either 0 or π, whichever is smaller (the source comment says "the standard model seems to assume forward motion"), and it sets the initial rotation to zero when the translation is below 0.01 (in-place rotation) [L1-S37, lines 55–80].
- **Non-Gaussian ("banana-shaped") uncertainty:** for a differential-drive robot driving straight with noisy wheel speeds, the cloud of possible poses is banana-shaped and is "clearly not Gaussian" in (x, y, θ); as uncertainty grows, algorithms that assume a Gaussian in these coordinates become "inconsistent" [L1-S36, p. 1, abstract and Fig. 1].
- Long et al. cite Bailey et al.: if the heading standard deviation exceeded "one or two degrees", EKF-SLAM maps ultimately failed [L1-S36, p. 1].
- The same distribution is well described by a Gaussian in exponential coordinates (the Lie-algebra coordinates of SE(2), the group of planar poses); in their simulations both representations fit about equally at small noise, and the exponential-coordinate Gaussian fits better as uncertainty grows [L1-S36, pp. 2, 4]. Closed-form mean and covariance propagation for straight segments and constant-curvature arcs is derived in these coordinates [L1-S36, abstract, §VI].

### 4. Odometry calibration methods
- **UMBmark:** drive a 4×4 m square five times clockwise and five times counter-clockwise, stopping at each corner, turning 90° on the spot and "slowly to avoid slippage"; measure the return position error each time [L1-S02, p. 7].
- The centre of gravity of each cluster (cw and ccw) is taken as the systematic error, and E_max,syst = max(r_c.g.,cw, r_c.g.,ccw) is the single figure of merit [L1-S02, pp. 5–6, eqs. 2–4].
- From the two cluster centres, the turn error α and leg-curvature error β are computed; β gives the curve radius R = (L/2)/sin(β/2) and E_d = (R + b/2)/(R − b/2); α gives b_actual = 90°/(90° − α)·b_nominal [L1-S01, pp. 17–19, eqs. 4.17–4.27].
- The diameter correction is applied as factors c_L = 2/(E_d + 1) and c_R = 2/(1/E_d + 1), which keep the average diameter unchanged [L1-S01, p. 19, eqs. 4.28–4.33].
- In eight experiments on a LabMate, E_max,syst fell from 232–423 mm to 12–35 mm (10- to 22-fold); one case needed a second pass (66 mm → 20 mm) [L1-S01, p. 22, Table I].
- A second compensation pass is likely to help only when E_max,syst > 3·SEM (standard error of the mean of the return errors); in the example SEM was 11.2 mm [L1-S01, p. 22].
- The survey summarises UMBmark calibration as a "10- to 20-fold reduction in systematic errors" that "takes about two hours" with a tape measure [L1-S03, pp. 142–143].
- Chong and Kleeman applied UMBmark to a robot with unloaded knife-edge encoder wheels: E_max,syst 135 mm → 33 mm ("4 folds"), wheelbase 0.37100 → 0.36898 m; residual systematic errors "could not be thoroughly removed despite repetitive trials" [L1-S05, p. 16, Table 8].
- **Arbitrary paths (Kelly):** because the pose error is an integral, endpoint residuals from any set of trajectories can be written as linear equations in the parameter errors and solved by pseudo-inverse [L1-S04, §10.4, pp. 33–34 (j. 210–211)].
- **Seegmiller (IPEM):** calibrating track width and both wheel radii to only the start and end poses of 28 trajectories (start and end points approximately 12 m apart) reduced systematic error "by about 75% in the worst case" [L1-S14, pp. 18–19]; IPEM needs only low-rate pose observations and is much less sensitive to timestamp errors than differentiating poses [L1-S14, pp. 3–4].
- **PC-method (Doh et al.):** drive a path forward and then backward and compare the shapes of the two odometry paths [L1-S13, p. 6]; on a differential-drive robot after about 1 km of test paths, mean endpoint error was 5.49 m (raw odometry), 2.29 m (UMBmark), 1.42 m (Kelly) and 0.82 m (PC-method) [L1-S13, p. 9, Table 3].
- **Filtering approaches:** augmented Kalman filters estimate pose and odometry parameters together during normal driving (Larsen et al., Martinelli et al.) [L1-S08, p. 1] [L1-S12, p. 1].
- **Least squares / maximum likelihood:** Antonelli et al.'s problem "is exactly linear" and is solved by linear least squares with external camera observations; Censi et al. calibrate wheel radii, wheel separation and sensor pose together in closed form, with no special trajectory and no external sensor [L1-S08, p. 1].
- Censi et al. note that UMBmark- and EKF-type methods assume known nominal values and estimate small corrections, and that EKFs can suffer from outliers caused by wheel slip [L1-S08, p. 2].
- Systematic errors "change very slowly as the result of wear or of different load distributions", and periodic re-calibration is suggested to correct for loads and tyre wear [L1-S03, p. 139] [L1-S01, p. 24].
- **Online calibration inside SLAM (Kümmerle et al.):** wheel radii, wheel separation and the laser's mounting pose are added as extra variables to a graph-based (hyper-graph) SLAM problem and estimated online from the robot's own data, needing only a rough initial guess and no prior map [L1-S42, p. 1, abstract; pp. 3, 5–6].
- Kümmerle et al. state that odometry parameters depend on the load distribution and on the surface: "when the robot carries a load the odometry will change, and similarly when it moves from carpet to concrete" [L1-S42, p. 3].
- On a PowerBot with inflated tyres carrying about 40 kg placed on its left side, the estimated radii were r_r = 0.1251 m, r_l = 0.1226 m without the load and r_r = 0.1231 m, r_l = 0.1223 m with it; using the unloaded calibration while loaded gave "a severe drift in the odometry" in a corridor run [L1-S42, pp. 11–12, Fig. 5].
- For skid-steer robots, a symmetric model can be identified from two simple tests: a pure rotation with equal and opposite track commands gives the ICR distance (effective half-track), and a measured straight-line distance gives the correction α [L1-S19, pp. 3–4].
- A full asymmetric ICR model (5 parameters) was fitted by a genetic algorithm to short segments of RTK-GPS ground truth; parameters are "optimized off-line for a specific robotic task, according to typical path motions, particular soil types, and speed ranges" [L1-S19, p. 4].
- Nav2 provides an "Odometry Calibration" behavior tree that drives a 2 m counter-clockwise square three times at 0.2 m/s, described as "a primitive experiment to measure odometric accuracy" [L1-S29].

### 5. Encoder measurement, velocity estimation and timing
- Reading a quadrature counter at a fixed rate gives a position quantisation error of up to half a count; numerical differentiation "largely amplify[ies]" this error [L1-S24, pp. 1–2].
- Fixed-time (frequency) measurement counts pulses in a fixed window; its absolute error is independent of speed, so the percentage error becomes intolerable at very low speed [L1-S25, pp. 2–3].
- Longer windows or low-pass/moving-average filters reduce quantisation noise but add lag that degrades the control loop [L1-S25, p. 3].
- Period (fixed-position) measurement times single encoder periods and is more accurate at low speed, while frequency measurement is better at high speed; mixed-mode methods combine the two [L1-S25, pp. 3–4].
- Merry et al. time-stamp encoder events and add a "skip" option that extends the observation interval over several stored events; compared with time-stamping without skip, this improved velocity estimates by 54% and acceleration estimates by 92% in their experiments [L1-S24, pp. 1, 6].
- Encoder imperfections (non-uniform slit spacing, sensor misplacement, disk eccentricity) add errors to such estimates [L1-S24, p. 3].
- `diff_drive_controller` can compute odometry from wheel **position** feedback (default `position_feedback: true`) or from wheel **velocity** feedback (then it multiplies velocity × radius × control period) or open-loop from commands (`open_loop`) [L1-S30, controller.cpp lines 147–184; parameters.yaml `position_feedback`, `open_loop`].
- In the position-feedback path, the pose is integrated from position differences, and the published velocity is a rolling mean (default window 10 samples) of increment/dt [L1-S30, odometry.cpp lines 48–99; parameters.yaml `velocity_rolling_window_size`].
- In the Humble release the odometry message and TF are stamped with the `time` argument of the controller's `update()` call [L1-S30, controller.cpp lines 211, 226].
- The later ros2_controllers master branch adds `update_from_pos(left, right, dt)` and `update_from_vel(..., dt)` that take an explicit period and convert wheel positions to angular velocities before integrating [L1-S31, lines 80–149, 236–256].
- Motor controllers filter velocity internally: the CTRE Talon SRX default is a 100 ms position-difference window plus a 64-sample rolling average (1 ms samples); the vendor recommends setting both to 1, then increasing the sampling period until the measured velocity is sufficiently granular, then increasing the rolling-average window until the signal is smooth "but still responsive enough" (FRC motor controller, used here only for comparison) [L1-S35, §"Velocity Measurement Filter"].
- Kelly observed "the existence of an optimal update rate" in an odometry-aided visual tracker [L1-S04, p. 2 (j. 179)]; Seegmiller et al. model powertrain response to commands as a time delay plus first-order lag and identify both from data [L1-S14, p. 20].

### 6. Skid-steer and tracked vehicle odometry
- Skid-steer (wheeled and tracked) vehicles intentionally rely on slip to turn; the effective contact point lies somewhere in the track footprint, so pure differential-drive dead reckoning is poor [L1-S03, p. 28]. The survey called tracked-vehicle odometry "virtually impossible" because of slip during turning [L1-S03, p. 139].
- Mandow et al. model each tread by its instantaneous centre of rotation (ICR); a steering efficiency χ (the ratio of the track-centreline distance to the distance between the tread ICRs) equals 1 with no slip [L1-S19, p. 3].
- On a Pioneer P3-AT (L = 0.4 m between tread centrelines) χ was 0.69–0.76 on asphalt and 0.71–0.75 on smooth concrete, depending on tyres; α (effective-radius factor) was 0.90–0.95 [L1-S19, p. 5, Tables I–II].
- Asphalt gave lower χ than concrete "due to greater friction", and higher tyre pressure gave α closer to 1 [L1-S19, p. 5].
- The P3-AT factory model uses an ICR distance of 0.3 m, where a non-slip model would use 0.2 m [L1-S19, p. 5].
- In validation, the asymmetric ICR model reduced mean-squared Δy error per segment from 0.00468 (factory model) to 0.00011 [L1-S19, p. 6, Table III].
- Seegmiller and Kelly state that reducing skid-steer kinematics to differential-drive kinematics "greatly overestimates yaw rate" [L1-S22, p. 7].
- Baril et al. compared ideal differential-drive, extended differential-drive (symmetric and asymmetric), radius-of-curvature and full linear models on a 590 kg skid-steer robot over more than 2 km on snow and concrete [L1-S20, p. 1].
- The two-parameter symmetric extended model (slip α and "virtual width" b̂) gave the most accurate angular prediction and was their recommended choice; all trained models were "vastly better" than the ideal differential-drive model [L1-S20, pp. 3, 6–7].
- The same robot rotated more for the same command on snow than on concrete, and the ideal differential-drive model performed better on snow than on concrete [L1-S20, pp. 6–7].
- Angular prediction error peaked when one side's commanded speed was about twice the other side's, suggesting a nonlinearity "which cannot be captured by any of the linear models tested"; translational error did not depend on the commands [L1-S20, p. 7].
- Tracked-vehicle slip ratios a_l, a_r scale each track's theoretical velocity by (1 − a); with a gyro measuring yaw rate, one more relation between a_l and a_r is enough to solve for both (Endo et al.'s SCOG method) [L1-S21, §II-B–D].
- Endo et al. found empirically that |a_l/a_r| = |v_r/v_l|^n with n = 0.4811 (P-tile), 0.6213 (plywood) and 0.5094 (artificial turf), and approximated n = 0.5 for their vehicle [L1-S21, §III-B, p. 3].
- The Mandow ICR model and the Endo slip-ratio model are experimental (per-terrain) identifications; Pentzer et al. (J. Field Robotics 2014) track each ICR online with an extended Kalman filter using position and heading measurements, validated on a 118 kg skid-steer robot (not downloaded; as described in [L1-S20, p. 2]).

### 7. Typical odometry error magnitudes by vehicle and terrain
- Indoor differential drive, uncalibrated: E_max,syst 310 mm on a 4×4 m square (16 m) for a LabMate at 0.2 m/s [L1-S02, p. 8].
- Indoor differential drive, UMBmark-calibrated: 12–35 mm on the same test [L1-S01, p. 22]; 33 mm for a low-cost robot with knife-edge encoder wheels [L1-S05, pp. 16–17].
- A 350 kg differential-drive robot with added encoder wheels had less than 200 mm error over 50 m after careful calibration (Hongo et al.) [L1-S03, p. 138].
- Differential drive, long paths: raw odometry mean endpoint error 25.7 m after 719.1 m, 4.4 m after PC-method correction (Nomad Scout) [L1-S13, p. 9].
- Gourley and Trivedi suggest limiting LabMate odometry to about 10 m before a reset [L1-S03, pp. 137–138].
- Four-wheel skid-steer (Pioneer AT) on a 12 m indoor sand track: with expert-rule encoder arbitration plus gyro checks, mean square error was 56 mm (PID) and 71 mm (cross-coupled control) [L1-S16, pp. 9, 14].
- Six-wheel rover on sand: FLEXnav (gyro + odometry) errors are "typically smaller than 1%" of distance on low-slip terrain but "may become huge" with all-wheel slip; current-based correction kept errors "well under 1%" and reduced errors by up to one order of magnitude [L1-S17, pp. 1, 12].
- Skid-steer LandTamer (2.5 m/s max), 10 m prediction horizon, no-slip kinematic model: mean position error 1.717 m (dirt), 1.465 m (grass), 1.774 m (pavement), mean yaw error 0.291, 0.277, 0.493 rad; an enhanced slip model cut these by 64–96% [L1-S22, p. 8, Tables III–IV].
- Tracked vehicle (CV-04, 25 kg, 500 mm tread) on a P-tile floor following a path with 90° and 180° turns: wheel-only odometry control ended at (22 cm, 188 cm, 2.02 rad), gyro+odometry at (13 cm, −6 cm, 3.15 rad), slip-compensated at (2 cm, −3 cm, 3.15 rad) [L1-S21, §V-C, p. 5].
- Mars Exploration Rovers: the design goal was position drift no more than 10% over a 100 m drive; IMU + wheel odometry met this on benign terrain but not on steep slopes or sand [L1-S23, p. 2]. On one 19 m uphill/cross-slope drive, wheel odometry underestimated the distance by 1.6 m and the final positions differed by nearly 5 m [L1-S23, p. 11]. Slips up to 125% were measured [L1-S23, p. 10].
- On soft sand all wheels can slip at once ("all-wheel slippage"); the earlier redundant-encoder method needed at least one wheel gripping [L1-S17, p. 1] [L1-S18, p. 2].

### 8. Slip detection and compensation
- **Gyrodometry:** use odometry most of the time, but switch to the gyro for heading during the short intervals when gyro and odometry heading rates differ by more than a threshold; in tests the result was about 18 times smaller than odometry-only and about eight times smaller than gyro-only orientation error [L1-S15, §3–4, pp. 5–6].
- Gyrodometry rests on the observation that bump-type errors act only for "a fraction of a second for each encounter" [L1-S15, abstract].
- **Fewest Pulses:** on vehicles with redundant encoders, use the encoder with the fewest pulses in each interval, because slip and bumps cause over-counting; add a +0.5-count correction when switching encoders, to cancel truncation bias [L1-S16, pp. 4–5].
- **Cross-coupled control:** coupling wheel speed loops reduces slip caused by speed mismatch on over-constrained vehicles, but the improvement on sand was "not substantial" [L1-S16, pp. 6, 12].
- **Expert rules:** if one side's encoders disagree, compute that side's displacement from the other side plus the gyro yaw rate: D_L = D_R − ω_z·T·B (or the reverse) [L1-S16, pp. 8–9, eqs. 3–4].
- **Rover slip indicators:** encoder indicator (compare encoders), gyro indicator (compare encoder yaw rate with gyro) and current indicator (motor current roughly proportional to wheel torque) [L1-S18, p. 3].
- On a sandy slope these flagged all-wheel slip correctly 31%, 56% and 91% of the time; combined with a logical OR, 94% with 1% false positives [L1-S18, p. 8]. On sand mounds the figures were 25%, 38%, 18% and 61% combined with 12% false positives [L1-S18, p. 9].
- **Current-based compensation (iComp):** a linearised slip–current model corrects encoder readings; its parameters can be tuned online with continuous GPS, sporadic GPS, or without ground truth (less accurate) [L1-S17, p. 1]; the correction works only along the direction of motion, not laterally [L1-S17, pp. 1–2].
- **Tracked slip ratios with a gyro:** SCOG estimates both track slip ratios from encoders and a gyro plus one empirically identified exponent [L1-S21, abstract, §II-D].
- **Vision:** the Mars rovers used stereo visual odometry "Slip Checks" to detect insufficient progress; wheel odometry with the IMU did not meet the accuracy goal on steep slopes or sandy terrain [L1-S23, pp. 2, 10–11].
- **Classification-based immobilisation detection (Ward & Iagnemma):** a support vector machine (a trained classifier) labels each 0.5 s window (N = 50 samples at 100 Hz) as immobilised or normal from four features: roll-rate variance, pitch-rate variance, 25–50 Hz vertical-acceleration power, and (optionally) mean wheel angular acceleration [L1-S41, pp. 2–3].
- Ward & Iagnemma's rationale for feature four: on uneven ground wheel torque varies, so wheel angular acceleration varies; this variation is minimised when the robot is immobilised [L1-S41, p. 3].
- On a four-wheel, front-wheel differential-drive DARPA LAGR robot trained on mud, mulch and grass at 0–1.0 m/s, total classification accuracy on test data (grass hill and unseen loose gravel) was 94.7%, with 98.1% of normal points and 75% of immobilised points classified correctly; with IMU features only it was 92.0% [L1-S41, p. 3].
- Comparing wheel speed with a body speed obtained by integrating accelerometer readings was not robust: at low speed "accelerometer errors dominate", the velocity estimate diverges as a random walk, and a detector built on it would be ineffective [L1-S41, p. 4].
- Fusing the classifier with a dynamic-model-based detector (Ward & Iagnemma's companion paper, not downloaded) removed false positives in their example [L1-S41, pp. 5–6].
- Immobilisation (wheels turning but vehicle stuck) is visible as very high slip: slip rates of 98.9% to 99.5% were measured by onboard visual odometry over sols 463–483 [L1-S23, p. 15].

### 9. Testing and evaluating odometry
- The unidirectional square (and the figure-8) test can hide mutually compensating E_b and E_d errors; UMBmark requires both directions [L1-S02, pp. 4–5] [L1-S03, pp. 133–134].
- UMBmark measures return errors against an L-shaped wall reference [L1-S02, p. 4] [L1-S01, p. 20] with a tape measure [L1-S03, p. 143] [L1-S01, p. 23], or with a sonar calibrator accurate to about ±2 mm and ±0.4° [L1-S01, p. 20].
- Spread within each UMBmark cluster indicates non-systematic error, but its value is limited because it depends on the floor [L1-S02, p. 6].
- **Extended UMBmark** places a ~10 mm round cable under the inside wheel 10 times during the first leg and uses the average absolute return **orientation** error, because the return position depends on where the bumps were placed [L1-S02, pp. 6–7].
- Chong and Kleeman identified random-error parameters from 60 autonomous 10 m forward / 10 m backward runs with sonar wall scans as reference [L1-S05, p. 17].
- Doh et al. and Kelly use many arbitrary paths with measured endpoints; Doh tested on 10 paths of ~100 m and used Student's t-tests to compare methods [L1-S13, pp. 9–10].
- Skid-steer model studies use RTK-GPS (under 1 cm) [L1-S19, p. 5], total station [L1-S22, p. 7], motion capture (9 mm horizontal accuracy) [L1-S21, Table I] or lidar ICP localization [L1-S20, p. 4] as ground truth.
- Baril et al. report relative translational error ε_t (%) and relative angular error ε_θ (degree/m) over evaluation windows, and found the training horizon affects which horizon a model predicts best [L1-S20, pp. 5–6].
- **Trajectory error metrics (from the odometry/SLAM benchmark literature):** the relative pose error (RPE) compares the estimated and true motion over a fixed interval Δ and "corresponds to the drift of the trajectory"; its root-mean-square over the run is reported, and Δ equal to the data rate (e.g. Δ = 30 at 30 Hz) gives drift per second [L1-S39, pp. 6–7].
- Sturm et al. call comparing only start and end point (Δ = n) "a common (but poor) choice", because it penalises rotational errors early in the trajectory more than later ones [L1-S39, p. 7]; the KITTI benchmark makes the same point about end-point error [L1-S40, p. 5].
- The absolute trajectory error (ATE) first aligns the estimated trajectory to ground truth with a least-squares rigid-body fit (Horn's method) and then takes the RMS of the remaining position differences; it measures global consistency, while RPE captures both rotational and translational drift, and the two were "strongly correlated" in their experiments [L1-S39, p. 7].
- KITTI reports translational error (%) and rotational error (deg/m) separately, averaged over all sub-sequences of given lengths, and as a function of trajectory length and speed; the best visual-odometry method in the 2012 paper had 2.2% and 0.016 deg/m (car-mounted cameras, used here only as an example of the metric) [L1-S40, pp. 5–6].
- Kelly warns that closed and symmetric paths may cancel some errors (e.g. on a symmetric figure-8 both velocity scale error and gyro bias vanish), so they "may or may not be good choices" for calibration [L1-S04, p. 40 (j. 217)].

### 10. Reporting odometry in ROS (message, frames, covariance)
- REP-105: `odom` is a world-fixed frame in which the robot pose "can drift over time, without any bounds" but "is guaranteed to be continuous… without discrete jumps"; it is "an accurate, short-term local reference" typically computed from wheel, visual or inertial odometry [L1-S09, §odom].
- The `odom → base_link` transform is broadcast by an odometry source; a localization component publishes `map → odom` instead of `map → base_link` [L1-S09, §"Frame Authorities"].
- REP-103: units are metres and radians; body frames are right-handed with x forward, y left, z up [L1-S10, §Units, §Axis Orientation].
- `nav_msgs/Odometry`: the pose is in `header.frame_id`; the twist is in `child_frame_id`; both carry covariance [L1-S11].
- robot_localization transforms odometry pose data into its `world_frame` and twist data from `child_frame_id` into `base_link_frame` [L1-S26, §"Coordinate Frames"].
- If wheel odometry provides both position and velocity, robot_localization advises fusing the velocity; when the pose, heading and velocity all come from the same encoders, "it's best to just use the velocities" [L1-S26, §Odometry item 1] [L1-S27, §odom0 example item 1].
- A reported zero lateral velocity with a non-inflated covariance is a valid measurement for a non-holonomic robot and can be fused [L1-S27, item 2].
- Covariances "matter": inflating a variance (e.g. to about 1e3) to make the filter ignore a variable is "unnecessary and even detrimental"; use the config vector instead [L1-S26, §Odometry item 3, §Common errors].
- A zero variance on a fused variable is replaced by a small epsilon (1e−6) inside robot_localization; users should set covariances properly [L1-S26, §Common errors].
- If robot_localization publishes `odom → base_link`, the robot driver's own transform broadcast must be disabled [L1-S26, §Odometry item 5].
- Nav2 requires both the `odom → base_link` transform and a `nav_msgs/Odometry` message (for velocity) [L1-S28, §"Odometry Introduction"]; it notes that "IMUs drift over time while wheel encoders drift over distance traveled" [L1-S28].

### 11. Reference implementations
- `diff_drive_controller` (ros2_controllers 2.54.0, Humble): defaults `position_feedback: true`, `open_loop: false`, `enable_odom_tf: true`, `publish_rate: 50.0` Hz, `velocity_rolling_window_size: 10`, and pose/twist covariance diagonals of all zeros, with the parameter description suggesting `[0.001, 0.001, 0.001, 0.001, 0.001, 0.01]` as a starting point [L1-S30, parameters.yaml].
- The covariance diagonals are fixed values copied into every message; they do not grow with distance [L1-S30, controller.cpp lines 437–443].
- `wheel_separation_multiplier` and `left/right_wheel_radius_multiplier` are applied to the separation and radii used both for commands and for odometry [L1-S30, controller.cpp lines 143–145, 298–302; odometry.cpp `setWheelParams`].
- With several wheels per side, the controller averages the feedback of the wheels on each side before computing odometry [L1-S30, controller.cpp lines 153–173].
- Clearpath's ROS 2 configs set `wheel_separation_multiplier: 1.875` (Husky A200, 0.555 m separation) and `1.5` (Jackal J100, 0.37559 m), use `[0.001, 0.001, 0.001, 0.001, 0.001, 0.01]` for both covariance diagonals, run at 50 Hz, and set `enable_odom_tf: false` [L1-S32, a200 and j100 control.yaml].
- The Nav2 setup guide computes linear = (v_R + v_L)/2 and angular = (v_R − v_L)/wheel_separation, with wheel velocities from changes in joint positions over time, and recommends `diff_drive_controller` [L1-S28, §"Setting Up Odometry on your Robot"].

### Product-specific: REV SPARK MAX / NEO velocity measurement
- REVLib 2026: for a quadrature encoder the velocity is Δposition/Δtime with a measurement period of 1–100 ms (default 100 ms) and an average depth of 1–64 samples (default 64) [L1-S33, `quadratureMeasurementPeriod`, `quadratureAverageDepth`].
- For the NEO's built-in hall ("UVW") sensor the measurement period is 8–64 ms (default 32 ms) and the average depth is 1, 2, 4 or 8 (default 8) [L1-S33, `uvwMeasurementPeriod`, `uvwAverageDepth`].
- Position is returned in rotations and velocity in RPM, each multiplied by a user conversion factor [L1-S33, `positionConversionFactor`, `velocityConversionFactor`].
- A REV Support statement quoted on GitHub (2022) gives the hall-sensor velocity latency with default settings as (8 − 1)/2 × 32 = 112 ms, and says REV wanted to reduce it (level D; may have changed) [L1-S34].

## Recommended practice
1. Measure and correct the average-wheel-diameter (scale) error with a tape measure before other calibration [L1-S02, p. 4].
2. Calibrate wheel-diameter ratio and effective wheelbase with a test run in **both** directions (UMBmark, 5 runs each way), slowly, stopping to turn on the spot [L1-S02, p. 7] [L1-S01, pp. 17–19].
3. Repeat the compensation if E_max,syst is still larger than 3× the standard error of the mean [L1-S01, p. 22].
4. For skid-steer/tracked platforms, identify an effective track width (ICR distance) and scale factor from a pure rotation and a straight run, per terrain and speed range, or fit a fuller ICR model to ground truth [L1-S19, pp. 3–5] [L1-S20, pp. 6–7].
5. Model random error as per-wheel variance proportional to distance (k_L, k_R) and identify k from repeated runs on the working surface [L1-S05, pp. 7, 17] [L1-S07, p. 8].
6. Re-calibrate periodically as loads and tyres change [L1-S01, p. 24] [L1-S03, p. 139].
7. Keep turning speeds and accelerations low where slip matters [L1-S03, p. 138].
8. Use a gyro to cover short, large heading disturbances (bumps, slip) that odometry cannot model [L1-S15, §3] [L1-S16, pp. 8–9].
9. Publish continuous odometry in `odom`, with frames per REP-105 and signs per REP-103, and with realistic, non-zero covariances [L1-S09] [L1-S26, §Odometry].
10. When fusing wheel odometry in robot_localization and its position, heading and velocity all come from the same encoders, fuse only its velocities; let only one node broadcast `odom → base_link` [L1-S26, §Odometry items 1, 5] [L1-S27, item 1].
11. Re-estimate wheel radii when the load or the driving surface changes, or estimate them online [L1-S42, pp. 3, 11–12].
12. Report odometry drift as relative errors over many intervals or sub-sequence lengths (RPE; % and deg/m), not only the start-to-end error of one run [L1-S39, pp. 6–7] [L1-S40, p. 5].

## Key numbers
| Quantity | Value | Conditions | Source |
|---|---|---|---|
| Effective wheelbase uncertainty | order of 1% | some commercial differential-drive robots | L1-S02, p. 3 |
| Scale error measurement accuracy | 0.3–0.5% of full scale | tape measure | L1-S02, p. 4 |
| UMBmark improvement | 10- to 22-fold (E_max,syst 232–423 mm → 12–35 mm) | LabMate, 4×4 m square, 0.2 m/s | L1-S01, p. 22 |
| UMBmark effort | about 2 h | tape measure | L1-S03, p. 143 |
| Orientation error from 10 mm bump | ≈0.6° | TRC LabMate | L1-S02, p. 6 |
| Random-error growth, straight line | σ_θ, σ_along ∝ √d; σ_cross ∝ d^1.5 | per-wheel white noise | L1-S06, p. 7 |
| Per-wheel noise constant | k_L = 0.00040, k_R = 0.00058 m^1/2 | knife-edge encoder wheels, parquet floor | L1-S05, p. 17 |
| Linearisation accuracy | ≤1 cm or 3% of error | Kelly test trajectory, 210 s | L1-S04, p. 39 |
| PC-method vs UMBmark vs Kelly | mean endpoint error 0.82 / 2.29 / 1.42 m (raw 5.49 m) | ~1 km of arbitrary paths | L1-S13, p. 9 |
| IPEM calibration | systematic error −75% worst case | 28 trajectories, start and end points ~12 m apart | L1-S14, p. 19 |
| Skid-steer steering efficiency χ | 0.69–0.76 | Pioneer P3-AT, asphalt/concrete | L1-S19, p. 5 |
| Tracked slip exponent n | 0.48–0.62 (≈0.5) | CV-04, P-tile/plywood/turf | L1-S21, p. 3 |
| Skid-steer 10 m prediction error, no-slip model | 1.47–1.77 m, 0.28–0.49 rad | LandTamer, dirt/grass/pavement | L1-S22, p. 8 |
| Error reduction with slip-enhanced model | 64–96% | same | L1-S22, p. 8 |
| 3D odometry on uneven terrain | <0.25 m and 2.3° after up to 201.5 m | Zoë rover, no gyro | L1-S22, p. 6 |
| Gyrodometry improvement | ~18× vs odometry, ~8× vs gyro | LabMate with 15 bumps, 10 cm/s | L1-S15, pp. 5–6 |
| Slip detection (combined indicators) | 94% (1% false positives) / 61% (12%) | rover on sandy slope / sand mounds | L1-S18, pp. 8–9 |
| MER drift design goal | ≤10% of 100 m drive | IMU + wheel odometry | L1-S23, p. 2 |
| `diff_drive_controller` defaults | 50 Hz, window 10, covariance 0 | 2.54.0 | L1-S30 |
| Clearpath covariance diagonals | [0.001 ×5, 0.01] | Husky, Jackal | L1-S32 |
| SPARK MAX quadrature velocity | 100 ms period, 64-sample average (defaults) | REVLib 2026 | L1-S33 |
| AMCL odometry noise weights α1–α4 | 0.2 each (default) | Nav2 AMCL, differential model | L1-S38 |
| Online radius change under load | r_r 0.1251 → 0.1231 m, r_l 0.1226 → 0.1223 m | PowerBot, inflated tyres, ~40 kg load on left side | L1-S42, p. 11 |
| Immobilisation classifier accuracy | 94.7% total (98.1% normal, 75% immobilised); 92.0% IMU-only | LAGR robot, grass hill and loose gravel, 0–1.0 m/s | L1-S41, p. 3 |
| SPARK MAX hall velocity | 32 ms period, 8-sample average (defaults); ≈112 ms latency | REVLib 2026; REV support (2022) | L1-S33; L1-S34 |

## How it is tested
| Test | What it measures | Pass criterion used in the source | Source |
|---|---|---|---|
| UMBmark (4×4 m square, 5 cw + 5 ccw) | Systematic error E_max,syst; E_d, E_b | Lower E_max,syst; second pass if E_max,syst > 3·SEM | L1-S02, pp. 5–7; L1-S01, p. 22 |
| Extended UMBmark (10 bumps of ~10 mm) | Sensitivity to non-systematic error | Average absolute return orientation error | L1-S02, pp. 6–7 |
| Nav2 odometry-calibration BT (2 m CCW square ×3, 0.2 m/s) | Odometric accuracy (primitive) | None stated | L1-S29 |
| 10 m out-and-back runs ×60 | Random-error constants k_L, k_R | Fit of 95% error ellipses | L1-S05, p. 17 |
| Forward/backward same path (PC-method) | E_d, E_b from path shape | Endpoint error on independent ~100 m paths | L1-S13, pp. 6, 9–10 |
| Arbitrary trajectories with known end poses | Wheel radii, track width | Endpoint residual reduction | L1-S04, §10.4; L1-S14, pp. 18–19 |
| Segment-wise comparison with RTK-GPS / total station | Skid-steer model prediction error | Mean squared Δx, Δy, Δφ per segment; error over 10/20 m horizons | L1-S19, p. 6; L1-S22, p. 8 |
| Relative errors over evaluation windows | ε_t (%), ε_θ (deg/m) | Median and quartiles | L1-S20, pp. 5–6 |
| Relative pose error (RPE) over interval Δ | Local drift (e.g. per frame or per second) | RMSE over all intervals; start-to-end only is called "poor" | L1-S39, pp. 6–7 |
| Absolute trajectory error (ATE) after rigid alignment | Global consistency | RMSE of position differences | L1-S39, p. 7 |
| KITTI odometry metric | Translational error (%) and rotational error (deg/m) vs length and speed | Averages over all sub-sequences of a given length or speed | L1-S40, pp. 5–6 |
| Slip-indicator comparison with ground-truth flag | Detection rate, false positives | % time correct | L1-S18, pp. 8–9 |

## Common mistakes
- Calibrating with a one-direction square (or figure-8): compensating E_b and E_d errors can give an apparently "excellent" result while both remain [L1-S02, pp. 4–5] [L1-S03, pp. 133–134].
- Correcting only the wheelbase b in software to make the square close, which hides a diameter error [L1-S02, p. 5].
- Running calibration fast: UMBmark prescribes slow runs to avoid slip [L1-S02, p. 7].
- Using the ideal differential-drive model on a skid-steer vehicle: it over-predicts yaw rate and gives much larger errors than trained models [L1-S22, p. 7] [L1-S20, p. 6].
- Averaging redundant encoders on a slipping vehicle: over-counting wheels bias the average; arbitration rules do better [L1-S16, pp. 4, 14].
- Selecting the lowest-count encoder without correcting the truncation bias, which always underestimates distance [L1-S16, p. 5].
- Inflating odometry covariances to about 1e3 to "switch off" variables, which is detrimental in robot_localization [L1-S26, §Odometry item 3].
- Leaving covariances at zero; robot_localization then substitutes 1e−6 [L1-S26, §Common errors]; `diff_drive_controller` defaults to zeros [L1-S30, parameters.yaml].
- Fusing wheel-odometry pose, heading and velocity together although they come from the same encoders, which feeds duplicate information into the filter [L1-S27, item 1].
- Two nodes broadcasting `odom → base_link` (driver and EKF) [L1-S26, §Odometry item 5].
- Using wrong signs (turning counter-clockwise must increase yaw; driving forward must increase x) [L1-S26, §Odometry item 4].
- Judging odometry only by the start-to-end error of one run: this over-weights early rotational errors; relative errors over many intervals are preferred [L1-S39, p. 7] [L1-S40, p. 5].
- Keeping one set of wheel-radius parameters when the load or surface changes, which gave severe odometry drift in Kümmerle et al.'s loaded-robot test [L1-S42, pp. 3, 11–12].
- Treating long-range odometry uncertainty as an ellipse in (x, y, θ): the true distribution becomes banana-shaped and Gaussian filters in these coordinates become inconsistent as heading uncertainty grows [L1-S36, p. 1].
- Detecting slip by comparing wheel speed with integrated accelerometer speed, which diverges at low speed [L1-S41, p. 4].
- Filtering velocity too heavily: moving-average and low-pass filters add lag to speed measurement [L1-S25, p. 3] [L1-S35, §"Recommended Procedure"].

## Disagreements between sources
- **Tracked-vehicle odometry feasibility.** The 1996 survey calls odometry on tracked vehicles "virtually impossible" and says skid steering is "generally employed only in tele-operated" applications [L1-S03, pp. 28, 139]. Later work obtains usable tracked/skid-steer odometry with ICR or slip-ratio models plus a gyro [L1-S19, p. 6] [L1-S21, §V-C] [L1-S22, p. 8]. The newer, experimentally validated results are the better guide; the survey reflects 1996 practice.
- **Square-path direction.** Nav2's odometry-calibration tree drives a counter-clockwise square only [L1-S29], while Borenstein and Feng show that single-direction squares can conceal compensating errors and require both directions [L1-S02, pp. 4–5]. The peer-reviewed source (level A) is followed; the Nav2 page itself calls its test "primitive".
- **Wheelbase proportionality wording.** The journal paper says the wheelbase is "inversely proportional" to the actual amount of rotation [L1-S01, p. 18], while the survey says "directly proportional" [L1-S03, p. 141]; both then use the same formula b_actual = 90°/(90° − α)·b_nominal [L1-S01, p. 19].
- **Which terrain is harder for skid-steer odometry.** Mandow et al. find lower steering efficiency (more skid) on higher-friction asphalt than on concrete [L1-S19, p. 5]; Baril et al. find the ideal differential-drive model does better on low-friction snow than on concrete, but the robot also rotates more per command on snow [L1-S20, pp. 6–7]. Both show that the effective-track parameters depend on the surface; they do not conflict, but the direction of change cannot be assumed across platforms.
- **Covariance form.** The error-model literature has covariance that grows with distance travelled [L1-S05, p. 7] [L1-S07, p. 8], while `diff_drive_controller` and Clearpath publish fixed covariance diagonals [L1-S30] [L1-S32]; the ROS documentation examined does not say how the fixed values relate to distance-growing covariance.

## Open questions
- No downloaded source gives measured calibrated-odometry accuracy (percent of distance, degrees per metre) for rubber-tracked skid-steer robots on **grass**; Seegmiller's LandTamer is wheeled [L1-S22, p. 7] and Endo's CV-04 was tested on P-tile, plywood and artificial turf under a fixed motion-capture camera [L1-S21, p. 3, Table I].
- How much effective track width changes with speed and turning radius for tracks: Baril shows nonlinearity at speed ratio ≈2, but no source gives a tracked-vehicle table [L1-S20, p. 7].
- Quantitative effect of motor-controller velocity filtering lag on odometry pose error during acceleration and turning was not found; sources describe the lag [L1-S25] [L1-S34] and the position-vs-velocity options [L1-S30] but not the resulting pose error.
- Timestamping at acquisition vs arrival, dropped messages, counter wrap-around and counter resets on reboot: no downloaded source treats their effect on odometry.
- Antonelli et al. (2005), Martinelli et al. (2007), Wang (1988), Thrun et al. (2005, α1–α4 odometry motion model), Martínez et al. (2005), Pentzer et al. (2014) and Ward & Iagnemma (2007) could not be read; their methods are known here only through summaries in L1-S08, L1-S12, L1-S14, L1-S19, L1-S20, L1-S36–L1-S38 and L1-S41.
- Whether the parameters for the forward (command) and inverse (odometry) skid-steer models should differ is not addressed directly in the sources read; ros2_controllers applies the same multiplier to both [L1-S30].
- When linearised (Gaussian) covariance fails for large heading error or non-Gaussian slip: Chong and Kleeman note a second-order model becomes necessary at large k [L1-S05, p. 15]; Long et al. show the banana-shaped distribution and an exponential-coordinate alternative in simulation [L1-S36], and AMCL uses sampling [L1-S37]. No source gives a heading-uncertainty threshold measured for wheel odometry itself (the one-to-two-degree figure is for EKF-SLAM, cited second-hand in L1-S36).
- How large the integration error of Euler vs mid-point (RK2) vs exact-arc integration is at typical odometry rates (20–100 Hz) and turn rates: sources describe the schemes [L1-S03, pp. 20, 138] [L1-S07, p. 7] [L1-S30] but none downloaded quantifies the difference; the one survey statement is that arcs give "relatively small" benefits [L1-S03, p. 138].
- How to project planar odometry onto slopes or into 3D (pitch effects) beyond Seegmiller and Kelly's 3D model [L1-S22]: no general treatment was found.
- How to choose AMCL-style α1–α4 or Chong–Kleeman k values for a skid-steer or tracked vehicle on grass: the sources give defaults [L1-S38] or indoor differential-drive values [L1-S05] only.
- Agricultural-vehicle odometry error figures (SCOPE 7d) were not found in an open source.
- Heading-error-based calibration paths (Jung & Chung 2011, SCOPE 4b) and bidirectional straight/arc calibration paths other than UMBmark and the PC-method are not covered: no such source was downloaded.

## Sources
Evidence levels: peer-reviewed papers and academic-press books are A; author copies of published papers are A. University technical reports from named research labs that are standard references in the field (L1-S03, L1-S05, L1-S06) are graded A consistently; L1-S05 also has a peer-reviewed ICRA 1997 version. Official ROS standards (REPs) are A; official project documentation and code are B.

| ID | Citation | Link | Accessed | File | Level |
|---|---|---|---|---|---|
| L1-S01 | J. Borenstein, L. Feng, "Measurement and Correction of Systematic Odometry Errors in Mobile Robots," *IEEE Trans. Robotics and Automation* 12(6):869–880, Dec. 1996. DOI 10.1109/70.544770 (author preprint, 26 pp.; its header wrongly says 12(5)). | https://doi.org/10.1109/70.544770 (author copy: http://www-personal.umich.edu/~johannb/Papers/paper58.pdf) | 2026-09-27 | sources/borenstein_1996_correction_systematic_odometry_errors.pdf | A |
| L1-S02 | J. Borenstein, L. Feng, "UMBmark: A Benchmark Test for Measuring Odometry Errors in Mobile Robots," *Proc. SPIE 2591, Mobile Robots X*, pp. 113–124, Philadelphia, 1995. DOI 10.1117/12.228968. | https://doi.org/10.1117/12.228968 (author copy: http://www-personal.umich.edu/~johannb/Papers/paper60.pdf) | 2026-09-27 | sources/borenstein_1995_umbmark.pdf | A |
| L1-S03 | J. Borenstein, H. R. Everett, L. Feng, *"Where am I?" Sensors and Methods for Mobile Robot Positioning*, Univ. of Michigan technical report for Oak Ridge National Laboratory / US DOE, 1996. | https://deepblue.lib.umich.edu/items/8df90133-63f8-48dc-aaea-a644d671b9d7 | 2026-09-27 | sources/borenstein_1996_where_am_i.pdf | A |
| L1-S04 | A. Kelly, "Linearized Error Propagation in Odometry," *Int. J. Robotics Research* 23(2):179–218, 2004. DOI 10.1177/0278364904041326. | https://publications.ri.cmu.edu/storage/publications/pub_files/pub4/kelly_alonzo_2004_1/kelly_alonzo_2004_1.pdf | 2026-09-27 | sources/kelly_2004_linearized_error_propagation_odometry.pdf | A |
| L1-S05 | K. S. Chong, L. Kleeman, "Accurate Odometry and Error Modelling for a Mobile Robot," Monash Univ. tech. report MECSE-1996-6 (published as ICRA 1997, pp. 2783–2788). | https://ecse.monash.edu/techrep/reports/pre-2003/MECSE-6-1996.pdf | 2026-09-27 | sources/chong_1996_accurate_odometry_error_modelling.pdf | A |
| L1-S06 | L. Kleeman, "Odometry Error Covariance Estimation for Two Wheel Robot Vehicles," Monash Univ. tech. report MECSE-95-1, 1995. | https://ecse.monash.edu/centres/irrc/LKPubs/MECSE-1995-1.pdf | 2026-09-27 | sources/kleeman_1995_odometry_error_covariance.pdf | A |
| L1-S07 | R. Siegwart, I. R. Nourbakhsh, *Introduction to Autonomous Mobile Robots*, MIT Press, 2004, ch. 5 "Mobile Robot Localization" (author-distributed chapter). | http://www.cs.cmu.edu/~rasc/Download/AMRobots5.pdf | 2026-09-27 | sources/siegwart_2004_amr_ch5_localization.pdf | A |
| L1-S08 | A. Censi, A. Franchi, L. Marchionni, G. Oriolo, "Simultaneous Calibration of Odometry and Sensor Parameters for Mobile Robots," *IEEE Trans. Robotics* 29(2):475–492, 2013 (author copy). | https://doi.org/10.1109/TRO.2012.2226380 | 2026-09-27 | sources/censi_2013_simultaneous_calibration_odometry_sensor.pdf | A |
| L1-S09 | W. Meeussen, REP-105 "Coordinate Frames for Mobile Platforms," 2010 (Active). | https://reps.openrobotics.org/rep-0105/ | 2026-09-27 | sources/rep_2010_0105_coordinate_frames.rst | A |
| L1-S10 | T. Foote, M. Purvis, REP-103 "Standard Units of Measure and Coordinate Conventions," 2010. | https://reps.openrobotics.org/rep-0103/ | 2026-09-27 | sources/rep_2010_0103_units_conventions.rst | A |
| L1-S11 | ROS 2 common_interfaces, `nav_msgs/msg/Odometry.msg`, humble branch at pinned commit 08434490 (file last changed in commit ba32be2, 2020). | https://github.com/ros2/common_interfaces/blob/08434490d3e4abd37ccc5462cdde69b88853b5b8/nav_msgs/msg/Odometry.msg | 2026-09-27 | sources/ros2_2020_nav_msgs_odometry.msg | B |
| L1-S12 | A. Martinelli, R. Siegwart, "Estimating the Odometry Error of a Mobile Robot during Navigation," *Proc. European Conf. on Mobile Robots (ECMR)*, Warsaw, 2003. | https://infoscience.epfl.ch/record/97494/files/Martinelli_ECMR03.pdf | 2026-09-27 | sources/martinelli_2003_estimating_odometry_error_navigation.pdf | A |
| L1-S13 | N. L. Doh, H. Choset, W. K. Chung, "Relative Localization Using Path Odometry Information," *Autonomous Robots* 21:143–154, 2006. DOI 10.1007/s10514-006-6474-8. | https://doi.org/10.1007/s10514-006-6474-8 | 2026-09-27 | sources/doh_2006_relative_localization_path_odometry.pdf | A |
| L1-S14 | N. Seegmiller, F. Rogers-Marcovitz, G. Miller, A. Kelly, "Vehicle Model Identification by Integrated Prediction Error Minimization," *Int. J. Robotics Research* 32(8):912–931, 2013 (author preprint). | https://www.ri.cmu.edu/pub_files/2013/7/Seegmiller_IJRR-2013_Vehicle_Model_Identification.pdf (DOI 10.1177/0278364913488635) | 2026-09-27 | sources/seegmiller_2013_vehicle_model_identification_ipem.pdf | A |
| L1-S15 | J. Borenstein, L. Feng, "Gyrodometry: A New Method for Combining Data from Gyros and Odometry in Mobile Robots," *Proc. IEEE ICRA*, pp. 423–428, 1996. | https://doi.org/10.1109/ROBOT.1996.503813 (author copy: http://www-personal.umich.edu/~johannb/Papers/paper63.pdf) | 2026-09-27 | sources/borenstein_1996_gyrodometry.pdf | A |
| L1-S16 | L. Ojeda, J. Borenstein, "Methods for the Reduction of Odometry Errors in Over-Constrained Mobile Robots," *Autonomous Robots* 16:273–286, 2004 (author copy). | https://doi.org/10.1023/B:AURO.0000025791.45313.01 | 2026-09-27 | sources/ojeda_2004_odometry_errors_over_constrained.pdf | A |
| L1-S17 | L. Ojeda, D. Cruz, G. Reina, J. Borenstein, "Current-Based Slippage Detection and Odometry Correction for Mobile Robots and Planetary Rovers," *IEEE Trans. Robotics* 22(2):366–378, 2006. DOI 10.1109/TRO.2005.862480. | https://doi.org/10.1109/TRO.2005.862480 | 2026-09-27 | sources/ojeda_2006_current_based_slippage_detection.pdf | A |
| L1-S18 | G. Reina, L. Ojeda, A. Milella, J. Borenstein, "Wheel Slippage and Sinkage Detection for Planetary Rovers," *IEEE/ASME Trans. Mechatronics* 11(2):185–195, 2006. DOI 10.1109/TMECH.2006.871095. | https://doi.org/10.1109/TMECH.2006.871095 | 2026-09-27 | sources/reina_2006_wheel_slippage_sinkage_detection.pdf | A |
| L1-S19 | A. Mandow, J. L. Martínez, J. Morales, J. L. Blanco, A. García-Cerezo, J. González, "Experimental Kinematics for Wheeled Skid-Steer Mobile Robots," *Proc. IEEE/RSJ IROS*, pp. 1222–1227, 2007. | http://babel.isa.uma.es/_utils/downloads/logdownloads.php?u=jafma&t=pdf&f=downloads/jafma/papers/mandow2007ekw.pdf (DOI 10.1109/IROS.2007.4399139) | 2026-09-27 | sources/mandow_2007_experimental_kinematics_skid_steer.pdf | A |
| L1-S20 | D. Baril et al., "Evaluation of Skid-Steering Kinematic Models for Subarctic Environments," *Proc. 17th Conf. on Computer and Robot Vision (CRV)*, pp. 198–205, 2020. DOI 10.1109/CRV50864.2020.00034 (file: open preprint arXiv:2004.05131v1). | https://doi.org/10.1109/CRV50864.2020.00034 (open copy: https://arxiv.org/pdf/2004.05131) | 2026-09-27 | sources/baril_2020_skid_steer_kinematic_models.pdf | A |
| L1-S21 | D. Endo, Y. Okada, K. Nagatani, K. Yoshida, "Path Following Control for Tracked Vehicles Based on Slip-Compensating Odometry," *Proc. IEEE/RSJ IROS*, San Diego, 2007 (page range not listed in Crossref/OpenAlex). | https://k-nagatani.org/pdf/2007-IROS-Endo-online.pdf (DOI 10.1109/IROS.2007.4399228) | 2026-09-27 | sources/endo_2007_tracked_slip_compensating_odometry.pdf | A |
| L1-S22 | N. Seegmiller, A. Kelly, "Enhanced 3D Kinematic Modeling of Wheeled Mobile Robots," *Proc. Robotics: Science and Systems X*, 2014. | https://www.roboticsproceedings.org/rss10/p20.pdf | 2026-09-27 | sources/seegmiller_2014_enhanced_3d_kinematic_modeling.pdf | A |
| L1-S23 | M. Maimone, Y. Cheng, L. Matthies, "Two Years of Visual Odometry on the Mars Exploration Rovers," *J. Field Robotics* 24(3):169–186, 2007 (author copy). | https://doi.org/10.1002/rob.20184 | 2026-09-27 | sources/maimone_2007_two_years_visual_odometry_mer.pdf | A |
| L1-S24 | R. J. E. Merry, M. J. G. van de Molengraft, M. Steinbuch, "Velocity and Acceleration Estimation for Optical Incremental Encoders," *Mechatronics* 20:20–26, 2010. | https://techunited.nl/media/files/velocity_and_acceleration_estimation_for_optical_incremental_encoders.pdf | 2026-09-27 | sources/merry_2010_encoder_velocity_estimation.pdf | A |
| L1-S25 | R. Petrella, M. Tursini, L. Peretti, M. Zigliotto, "Speed Measurement Algorithms for Low-Resolution Incremental Encoder Equipped Drives: a Comparative Analysis," *Proc. Int. Aegean Conf. on Electrical Machines and Power Electronics (ACEMP)*, pp. 780–787, 2007. | https://doi.org/10.1109/ACEMP.2007.4510607 | 2026-09-27 | sources/petrella_2007_speed_measurement_low_resolution_encoder.pdf | A |
| L1-S26 | robot_localization docs, "Preparing Your Data for Use with robot_localization," humble-devel commit 8696ee5. | https://github.com/cra-ros-pkg/robot_localization/blob/8696ee5a9e4f959fcaae37835dcf2ed12ead581b/doc/preparing_sensor_data.rst | 2026-09-27 | sources/robotlocalization_humble_8696ee5_preparing_sensor_data.rst | B |
| L1-S27 | robot_localization docs, "Configuring robot_localization," humble-devel commit 8696ee5. | https://github.com/cra-ros-pkg/robot_localization/blob/8696ee5a9e4f959fcaae37835dcf2ed12ead581b/doc/configuring_robot_localization.rst | 2026-09-27 | sources/robotlocalization_humble_8696ee5_configuring.rst | B |
| L1-S28 | Nav2 documentation, "Setting Up Odometry," docs.nav2.org commit 588d374. | https://github.com/ros-navigation/docs.nav2.org/blob/588d37415e87eb083500d6c79aaed92ee1285f52/docs/configuration_and_development/first_time_robot_setup_guide/odom/setup_odom.md | 2026-09-27 | sources/nav2docs_588d374_setup_odom.md | B |
| L1-S29 | Nav2 documentation, "Odometry Calibration" behavior tree, docs.nav2.org commit 588d374. | https://github.com/ros-navigation/docs.nav2.org/blob/588d37415e87eb083500d6c79aaed92ee1285f52/docs/getting_started/nav2_behavior_trees/trees/odometry_calibration/odometry_calibration.md | 2026-09-27 | sources/nav2docs_588d374_odometry_calibration_bt.md | B |
| L1-S30 | ros2_controllers `diff_drive_controller`, tag 2.54.0 (Humble): odometry.cpp, diff_drive_controller.cpp, diff_drive_controller_parameter.yaml, userdoc.rst. | https://github.com/ros-controls/ros2_controllers/tree/2.54.0/diff_drive_controller | 2026-09-27 | sources/ros2controllers_2_54_0_diff_drive_odometry.cpp; ros2controllers_2_54_0_diff_drive_controller.cpp; ros2controllers_2_54_0_diff_drive_parameters.yaml; ros2controllers_2_54_0_diff_drive_userdoc.rst | B |
| L1-S31 | ros2_controllers `diff_drive_controller/src/odometry.cpp`, master commit 2520ae5. | https://github.com/ros-controls/ros2_controllers/blob/2520ae5b6d18c01f44083491cd9a285b1e2cdfe0/diff_drive_controller/src/odometry.cpp | 2026-09-27 | sources/ros2controllers_master_2520ae5_diff_drive_odometry.cpp | B |
| L1-S32 | Clearpath Robotics, clearpath_common `clearpath_control/config/{a200,j100}/control.yaml`, commit 5c5ec97. | https://github.com/clearpathrobotics/clearpath_common/tree/5c5ec97ee0245d8543aeb29115e01d3c0f38900e/clearpath_control/config | 2026-09-27 | sources/clearpath_5c5ec97_a200_husky_control.yaml; sources/clearpath_5c5ec97_j100_jackal_control.yaml | C |
| L1-S33 | REV Robotics, REVLib Java 2026.0.5, `com/revrobotics/spark/config/EncoderConfig.java`. | https://maven.revrobotics.com/com/revrobotics/frc/REVLib-java/2026.0.5/REVLib-java-2026.0.5-sources.jar | 2026-09-27 | sources/rev_2026_revlib_EncoderConfig.java | B |
| L1-S34 | Piphi5 (WPILib SysId contributor), comment quoting REV Support on NEO hall-sensor velocity latency, wpilibsuite/sysid issue #258, 2022-01-12. | https://github.com/wpilibsuite/sysid/issues/258#issuecomment-1010658237 | 2026-09-27 | sources/wpilib_2022_sysid_issue258_rev_hall_latency.md | D |
| L1-S35 | CTR Electronics, Phoenix 5 documentation, "Bring Up: Talon FX/SRX Sensors — Velocity Measurement Filter," undated page on the unversioned "stable" docs (legacy framework; comparison only). | https://v5.docs.ctr-electronics.com/en/stable/ch14_MCSensor.html | 2026-09-27 | sources/ctre_2022_phoenix5_sensor_velocity.md | B |
| L1-S36 | A. W. Long, K. C. Wolfe, M. J. Mashner, G. S. Chirikjian, "The Banana Distribution is Gaussian: A Localization Study with Exponential Coordinates," *Proc. Robotics: Science and Systems VIII*, Sydney, 2012. | https://www.roboticsproceedings.org/rss08/p34.pdf | 2026-09-27 | sources/long_2012_banana_distribution_gaussian.pdf | A |
| L1-S37 | Nav2 (navigation2) `nav2_amcl/src/motion_model/differential_motion_model.cpp`, tag 1.1.18 (Humble; commit 1c68c21). | https://github.com/ros-navigation/navigation2/blob/1.1.18/nav2_amcl/src/motion_model/differential_motion_model.cpp | 2026-09-27 | sources/nav2amcl_1_1_18_motion_model_differential.cpp | B |
| L1-S38 | Nav2 documentation, "AMCL" configuration guide, docs.nav2.org commit 588d374. | https://github.com/ros-navigation/docs.nav2.org/blob/588d37415e87eb083500d6c79aaed92ee1285f52/docs/configuration_and_development/configuration_guide/others/configuring_amcl.md | 2026-09-27 | sources/nav2docs_588d374_configuring_amcl.md | B |
| L1-S39 | J. Sturm, N. Engelhard, F. Endres, W. Burgard, D. Cremers, "A Benchmark for the Evaluation of RGB-D SLAM Systems," *Proc. IEEE/RSJ IROS*, pp. 573–580, 2012. DOI 10.1109/IROS.2012.6385773. | https://cvg.cit.tum.de/_media/spezial/bib/sturm12iros.pdf | 2026-09-27 | sources/sturm_2012_tum_rgbd_benchmark.pdf | A |
| L1-S40 | A. Geiger, P. Lenz, R. Urtasun, "Are we ready for Autonomous Driving? The KITTI Vision Benchmark Suite," *Proc. IEEE CVPR*, pp. 3354–3361, 2012. DOI 10.1109/CVPR.2012.6248074. | https://www.cvlibs.net/publications/Geiger2012CVPR.pdf | 2026-09-27 | sources/geiger_2012_kitti_benchmark.pdf | A |
| L1-S41 | C. C. Ward, K. Iagnemma, "Classification-Based Wheel Slip Detection and Detector Fusion for Outdoor Mobile Robots," *Proc. IEEE ICRA*, Rome, pp. 2730–2735, 2007. DOI 10.1109/ROBOT.2007.363878 (pages and DOI confirmed via Crossref; conference-CD copy; journal extension: *Autonomous Robots* 26:33–46, 2009, DOI 10.1007/s10514-008-9105-8). | http://vigir.missouri.edu/~gdesouza/Research/Conference_CDs/IEEE_ICRA_2007/data/papers/0959.pdf | 2026-09-27 | sources/ward_2007_classification_wheel_slip_detection.pdf | A |
| L1-S42 | R. Kümmerle, G. Grisetti, W. Burgard, "Simultaneous Parameter Calibration, Localization, and Mapping," *Advanced Robotics* 26(17):2021–2041, 2012 (author's accepted manuscript). DOI 10.1080/01691864.2012.728694. | http://ais.informatik.uni-freiburg.de/publications/papers/kuemmerle12ar.pdf | 2026-09-27 | sources/kuemmerle_2012_simultaneous_parameter_calibration.pdf | A |

### References not downloaded
Foundational (per SCOPE.md) and supporting references that were not downloaded.

| Citation | Reason |
|---|---|
| C. M. Wang, "Location Estimation and Uncertainty Analysis for Mobile Robots," *IEEE ICRA*, pp. 1231–1235, 1988. | No open copy found. |
| G. Antonelli, S. Chiaverini, G. Fusco, "A Calibration Method for Odometry of Mobile Robots Based on the Least-Squares Technique," *IEEE T-RO* 21(5):994–1004, 2005. | No open copy (Semantic Scholar lists it as closed; ResearchGate returned a login page). Known here only via L1-S08 and L1-S14. |
| A. Martinelli, N. Tomatis, R. Siegwart, "Simultaneous Localization and Odometry Self Calibration for Mobile Robot," *Autonomous Robots* 22:75–85, 2007. | No verified open copy; the related ECMR 2003 paper (L1-S12) was used instead. |
| S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics*, MIT Press, 2005, ch. 5. | No licensed open copy. |
| J. L. Martínez, A. Mandow, J. Morales, S. Pedraza, A. García-Cerezo, "Approximating Kinematics for Tracked Mobile Robots," *IJRR* 24(10):867–878, 2005. | No verified open copy; the ICR method is used via L1-S19. |
| R. Siegwart, I. Nourbakhsh, D. Scaramuzza, *Introduction to Autonomous Mobile Robots*, 2nd ed., MIT Press, 2011. | No licensed open copy; 1st-edition chapter (L1-S07) used. |
| J. Pentzer, S. Brennan, K. Reichard, "Model-based Prediction of Skid-steer Robot Kinematics Using Online Estimation of Track ICRs," *J. Field Robotics* 31(3):455–476, 2014. | Only paywalled / login copies found (re-searched in the gap check: Wiley, Academia, Scribd, Penn State Pure). |
| C. C. Ward, K. Iagnemma, "Model-Based Wheel Slip Detection for Outdoor Mobile Robots," *IEEE ICRA*, pp. 2724–2729, 2007 (journal version: "A Dynamic-Model-Based Wheel Slip Detector for Mobile Robots on Outdoor Terrain," *IEEE T-RO* 24(4):821–831, 2008). | Only Scribd/ResearchGate copies; MIT DSpace returned a bot-check page. The companion classification-based ICRA 2007 paper (L1-S41) was used instead. |
| Jung & Chung, 2011, heading-error-based odometry calibration (supporting, named in SCOPE.md 4b). | Not searched to a verified citation or open copy; dropped. Not cited. |
| A. Kelly, *Mobile Robotics: Mathematics, Models, and Methods*, Cambridge Univ. Press, 2013 (supporting). | Textbook with no open copy; not cited. |
| K. Iagnemma, S. Dubowsky, *Mobile Robots in Rough Terrain*, Springer STAR vol. 12, 2004 (supporting). | No open copy; not cited. |
