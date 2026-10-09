# L1 — Wheel odometry: scope map

Mapped: 2026-09-27 (step 1 of STANDARDS.md §5). Nothing downloaded in this step.

## Overview

Wheel (or track) odometry estimates how far a vehicle has moved and how much it has turned by counting wheel or motor rotations and integrating them over time. This is called dead reckoning: each new pose is the old pose plus a small measured motion. It is cheap, fast and smooth, but its error grows without bound, so it is used as the short-term, continuous motion source (the `odom` frame in ROS REP-105) that other sensors correct. The field splits errors into **systematic** errors (repeatable: wrong wheel radius, unequal wheel diameters, wrong effective track width, misalignment), which calibration can remove, and **non-systematic** errors (random: slip, uneven ground, bumps, encoder quantisation), which must be modelled as uncertainty. The classic literature comes from 1990s indoor differential-drive research (Wang 1988; Borenstein and Feng's UMBmark and *Where am I?*; Chong and Kleeman's error model), was put on a general footing by Kelly's linearised error-propagation theory (2004), and was extended by calibration methods that use least squares or Kalman filters (Antonelli, Martinelli, Censi). Skid-steer and tracked vehicles break the pure-rolling assumption: they must skid to turn, so their odometry needs slip-aware models (instantaneous centres of rotation, ICR; Martínez, Mandow, Pentzer, Baril, Seegmiller) and slip detection (Ojeda, Reina, Ward and Iagnemma, Endo). Implementation practice (integration scheme, encoder position vs velocity, timestamps, covariance fields in `nav_msgs/Odometry`) is set by ROS documentation and reference code such as `ros2_controllers/diff_drive_controller`. The derivation of skid-steer kinematic models belongs to C2, fusion with IMU/GNSS to L4, and IMU heading to L2; this topic uses those only where they touch odometry accuracy, calibration or slip detection.

## Subtopics

| # | Subtopic | Questions | Covered already? |
|---|---|---|---|
| 1 | Dead-reckoning principles and integration methods | (a) How are incremental distance and heading change computed from left/right wheel or track travel, and from those the pose update? (b) How do Euler, second-order Runge–Kutta (mid-point heading) and exact arc integration differ, and how large is the integration error at typical sample rates and turn rates? (c) How is odometry expressed in 3D or on slopes (projection onto the ground plane, pitch effects)? (d) Why does heading error dominate position error growth (error grows with distance squared or cubed)? | Partly — `borenstein_1996_correction_systematic_odometry_errors.pdf` (basic odometry equations) |
| 2 | Error sources and taxonomy | (a) What are the systematic error sources (unequal wheel diameters, average diameter vs nominal, effective track width, misalignment, encoder resolution/sampling) and which dominate? (b) What are the non-systematic sources (slip, uneven floors, bumps, external forces, skid during turns) and which dominate outdoors? (c) How do Borenstein's E_d (diameter ratio) and E_b (wheelbase) errors produce curved and rotated paths? (d) Which errors are specific to over-constrained vehicles (more motors than degrees of freedom, e.g. skid-steer, tracks)? | Partly — `borenstein_1995_umbmark.pdf`, `borenstein_1996_correction_systematic_odometry_errors.pdf` |
| 3 | Error modelling and covariance propagation | (a) How is odometry uncertainty propagated analytically (Wang 1988; Chong–Kleeman closed-form covariance per path segment; Kelly's linearised systematic and random error dynamics)? (b) What noise models are used in probabilistic motion models (Thrun's α1–α4 rotation/translation noise; per-wheel noise proportional to travel) and how are their parameters identified? (c) How does error grow with distance and turning for straight lines, arcs and point turns? (d) When do linearised models fail (large heading error, non-Gaussian slip) and what replaces them (sampling, Gaussian mixtures)? | No |
| 4 | Odometry calibration methods | (a) How do the UMBmark bidirectional square test and its correction factors work, and what improvement factor is reported? (b) What alternative test paths exist (PC-method, bidirectional straight/arc paths, Jung & Chung heading-error methods) and how do they compare? (c) How do least-squares (Antonelli), augmented-Kalman-filter online self-calibration (Martinelli) and joint odometry-plus-sensor calibration (Censi, Kümmerle) work, and what ground truth do they need? (d) How are scale (distance per count) and effective track width calibrated for skid-steer/tracked vehicles, and do parameters change with terrain or speed? (e) How accurate does calibrated odometry get (percent of distance, degrees per metre)? | Partly — both Borenstein PDFs cover (a) |
| 5 | Encoder measurement, velocity estimation and timing | (a) Should odometry integrate encoder position (accumulated counts) or controller-reported velocity, and what are the error consequences of each? (b) How do encoder resolution and the fixed-time (count difference) vs fixed-position (period timing) velocity methods affect quantisation noise, especially at low speed? (c) How do filtering and averaging inside a motor controller add lag, and how does lag bias odometry during acceleration and turning? (d) How should samples be timestamped (at acquisition vs arrival), and what errors do jitter, dropped messages, counter wrap-around and resets cause? | No |
| 6 | Skid-steer and tracked vehicle odometry | (a) Why does pure-rolling odometry fail for skid-steer and tracked vehicles, and what correction models are used for odometry (effective track width / wheel_separation_multiplier, ICR-based models, slip-track)? (b) How are ICR or slip parameters identified offline and estimated online (Martínez 2005, Mandow 2007, Pentzer 2014 EKF, Seegmiller 2013/2014)? (c) How do these models compare in measured accuracy on different surfaces (Baril 2020 snow vs concrete)? (d) Do the corrected parameters differ between the forward model (commanding) and the inverse model (odometry)? | No (C2 folder holds several of these papers; kinematic derivation belongs to C2) |
| 7 | Typical odometry error magnitudes by vehicle and terrain | (a) What end-point errors (percent of distance travelled, heading drift per metre or per turn) are reported for differential-drive robots indoors, before and after calibration? (b) What errors are reported for skid-steer and tracked vehicles on concrete, grass, gravel, sand, snow and slopes? (c) How much larger are turning errors than straight-line errors on skid-steer platforms? (d) How do planetary-rover and agricultural results compare? | No |
| 8 | Slip detection and compensation | (a) What slip detectors exist: redundant-sensor comparison (gyro vs odometry, Gyrodometry), motor-current based (Ojeda), model-based (Ward & Iagnemma), classification-based, visual? (b) How are slip ratios estimated for tracks (Endo 2007, slip-compensated odometry) and fed back into odometry? (c) How is immobilisation (wheels turning, vehicle stuck) detected? (d) What "fewest pulses" or cross-coupled methods reduce over-count on over-constrained vehicles? | No |
| 9 | Testing and evaluating odometry | (a) Which test protocols are standard (UMBmark, closed-loop squares, out-and-back lines, circles/point turns) and what pass metrics are used (centre of gravity of return errors, E_max,syst)? (b) What ground-truth sources are used (motion capture, RTK GNSS, total station, laser scanner)? (c) How are trajectory metrics (absolute and relative trajectory error, drift per distance) computed and reported? (d) How many runs and which directions are needed to separate systematic from random error? | Partly — `borenstein_1995_umbmark.pdf` (UMBmark protocol) |
| 10 | Reporting odometry in ROS (message, frames, covariance) | (a) What do REP-103/REP-105 require of the `odom` frame (continuous, drifting) and of units/axes? (b) What are the fields of `nav_msgs/Odometry` (header.frame_id, child_frame_id, pose and twist with 6×6 covariance) and which frame each part is in? (c) How should pose vs twist covariance be set for wheel odometry, and what does robot_localization advise (fuse velocities, avoid huge covariances as a disable switch, differential mode)? (d) Should the odometry node also publish the `odom→base_link` TF when an EKF is present? | No |
| 11 | Reference implementations (last) | (a) How does `ros2_controllers/diff_drive_controller` compute odometry (position vs velocity feedback, RK2 vs exact integration, rolling-mean velocity window, `wheel_separation_multiplier`, `left/right_wheel_radius_multiplier`, fixed covariance diagonals)? (b) What do Clearpath (Husky/Jackal/Warthog) and other skid-steer ROS drivers do for odometry and covariance? (c) What pinned versions differ (Humble vs later timestep/velocity handling fixes)? | No (C2 folder holds `ros2controllers_2024_diff_drive_odometry.cpp` and Clearpath configs) |
| 12 | Probabilistic odometry motion models in localization software (added in gap check) | (a) How is the Thrun rotation–translation–rotation odometry motion model implemented in ROS 2 localization (Nav2 AMCL), and what do α1–α4 mean? (b) How does the implementation differ from the textbook (backward motion, in-place rotation)? | Added in gap check — L1-S37, L1-S38 |
| 13 | Non-Gaussian odometry uncertainty (added in gap check) | (a) Why does the pose distribution of a noisy differential-drive robot become banana-shaped? (b) When do (x, y, θ) Gaussians become inconsistent, and what alternative coordinates are proposed? | Added in gap check — L1-S36 |
| 14 | Trajectory error metrics for evaluating odometry (added in gap check; extends 9c) | (a) How are relative pose error (RPE) and absolute trajectory error (ATE) defined? (b) Why is start-to-end error a poor metric? (c) How does KITTI report % and deg/m drift? | Added in gap check — L1-S39, L1-S40 |
| 15 | Parameter change with load and surface; online calibration in SLAM (added in gap check; extends 4c–4d) | (a) How much do wheel radii change with load? (b) How can radii, wheel separation and sensor pose be estimated online inside graph SLAM (Kümmerle)? | Added in gap check — L1-S42 |
| 16 | Immobilisation detection by classification (added in gap check; extends 8a, 8c) | (a) Which IMU and wheel features detect immobilisation, and how accurate is it outdoors? (b) Why does comparing wheel speed with integrated accelerometer speed fail at low speed? | Added in gap check — L1-S41 |

## Foundational references

| Citation | Why foundational | Open copy |
|---|---|---|
| J. Borenstein, L. Feng, "Measurement and Correction of Systematic Odometry Errors in Mobile Robots," *IEEE Trans. Robotics and Automation*, 12(6):869–880, 1996. (UMBmark; conference version: SPIE Mobile Robots X, 1995.) | Defines the systematic/non-systematic error split, the E_d and E_b errors and the UMBmark bidirectional square test that the whole calibration literature benchmarks against. | Already in `sources/`: `borenstein_1996_correction_systematic_odometry_errors.pdf`, `borenstein_1995_umbmark.pdf` |
| J. Borenstein, H. R. Everett, L. Feng, *"Where am I?" Sensors and Methods for Mobile Robot Positioning*, Univ. of Michigan technical report, 1996. | Standard survey; chapter 5 on odometry and dead reckoning, error sources, tracked-vehicle and over-constrained odometry, gyro fusion. | https://deepblue.lib.umich.edu/items/8df90133-63f8-48dc-aaea-a644d671b9d7 (Deep Blue, U. Michigan) |
| C. M. Wang, "Location Estimation and Uncertainty Analysis for Mobile Robots," *IEEE ICRA*, pp. 1231–1235, 1988. | First derivation of the odometry pose estimator and its covariance matrix including slip and measurement error. | No open copy found |
| K. S. Chong, L. Kleeman, "Accurate Odometry and Error Modelling for a Mobile Robot," *IEEE ICRA*, pp. 2783–2788, 1997. (With L. Kleeman, "Odometry Error Covariance Estimation for Two Wheel Robot Vehicles," Monash tech. report MECSE-95-1.) | Closed-form covariance for straight, arc and turn-on-spot segments; the standard per-wheel noise model used for odometry covariance. | https://ecse.monash.edu/techrep/reports/pre-2003/MECSE-6-1996.pdf ; https://ecse.monash.edu/centres/irrc/LKPubs/MECSE-1995-1.pdf |
| A. Kelly, "Linearized Error Propagation in Odometry," *Int. J. Robotics Research*, 23(2):179–218, 2004. | General solution for propagation of both systematic and random odometry errors; explains error growth with path shape and distance. | https://publications.ri.cmu.edu/storage/publications/pub_files/pub4/kelly_alonzo_2004_1/kelly_alonzo_2004_1.pdf |
| G. Antonelli, S. Chiaverini, G. Fusco, "A Calibration Method for Odometry of Mobile Robots Based on the Least-Squares Technique: Theory and Experimental Validation," *IEEE Trans. Robotics*, 21(5):994–1004, 2005. | Standard least-squares calibration formulation, linear in the unknown parameters, with a measure of how informative a test path is. | No verified open copy (ResearchGate listing only) |
| A. Martinelli, N. Tomatis, R. Siegwart, "Simultaneous Localization and Odometry Self Calibration for Mobile Robot," *Autonomous Robots*, 22:75–85, 2007. | Online calibration of systematic parameters and non-systematic noise with an augmented Kalman filter during normal driving. | No verified open copy (ResearchGate/Academia listings) |
| A. Censi, A. Franchi, L. Marchionni, G. Oriolo, "Simultaneous Calibration of Odometry and Sensor Parameters for Mobile Robots," *IEEE Trans. Robotics*, 29(2):475–492, 2013. | Maximum-likelihood closed-form calibration of wheel radii, wheel separation and sensor pose, near the Cramér–Rao bound; no special path needed. | No verified open copy (CaltechAUTHORS record is metadata only) |
| R. Siegwart, I. R. Nourbakhsh, D. Scaramuzza, *Introduction to Autonomous Mobile Robots*, 2nd ed., MIT Press, 2011 (§5.2.3–5.2.4 effector noise and odometric error model). | Standard textbook treatment of odometric position estimation and its covariance propagation. | No licensed open copy (C2 folder has 1st-ed. ch. 3 only) |
| S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics*, MIT Press, 2005 (ch. 5 "Robot Motion", odometry motion model). | Standard probabilistic odometry motion model (α1–α4 noise parameters) used in AMCL and particle filters. | No licensed open copy |
| J. L. Martínez, A. Mandow, J. Morales, S. Pedraza, A. García-Cerezo, "Approximating Kinematics for Tracked Mobile Robots," *Int. J. Robotics Research*, 24(10):867–878, 2005. | Seminal ICR-based kinematics for tracked vehicles, optimised per terrain, used for tracked odometry; basis for Mandow 2007 and Pentzer 2014. | No verified open copy (Academia listing) |
| G. Reina, L. Ojeda, A. Milella, J. Borenstein, "Wheel Slippage and Sinkage Detection for Planetary Rovers," *IEEE/ASME Trans. Mechatronics*, 11(2):185–195, 2006. | Standard reference set of slip-detection measures (encoder, gyro, current, visual) for rough-terrain odometry. | No verified open copy |
| T. Foote, W. Meeussen et al., REP-105 "Coordinate Frames for Mobile Platforms" (2010), with REP-103 and the `nav_msgs/Odometry` message definition (ros2/common_interfaces). | Official ROS standard for the `odom` frame and the message that carries odometry pose/twist and covariance. | https://reps.openrobotics.org/rep-0105/ ; https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Odometry.msg |

Supporting (not foundational, but expected in step 2): Borenstein & Feng "Gyrodometry" (ICRA 1996); Ojeda & Borenstein, "Methods for the Reduction of Odometry Errors in Over-Constrained Mobile Robots," *Auton. Robots* 16, 2004; Ward & Iagnemma, "Model-Based Wheel Slip Detection for Outdoor Mobile Robots," ICRA 2007; Mandow et al., IROS 2007 (in C2 sources); Pentzer, Brennan, Reichard, *J. Field Robotics* 31, 2014; Baril et al., CRV 2020 (in C2 sources; open https://arxiv.org/pdf/2004.05131); Seegmiller & Kelly, RSS 2014 (open https://www.roboticsproceedings.org/rss10/p20.pdf); Endo et al. 2007 tracked slip-compensating odometry (in C2 sources); Doh, Choset, Chung PC-method; Jung & Chung 2011 heading-error calibration; Petrella et al. speed measurement algorithms for low-resolution encoders; Kelly, *Mobile Robotics: Mathematics, Models, and Methods*, Cambridge UP, 2013; Iagnemma & Dubowsky, *Mobile Robots in Rough Terrain*, Springer, 2004; robot_localization "Preparing Your Data" docs; `ros2_controllers` diff_drive_controller odometry.cpp at a pinned Humble tag.

## Existing files in sources/

| File | Subtopics covered |
|---|---|
| `borenstein_1995_umbmark.pdf` (SPIE 1995, 6 pp.) | 2, 4, 9 |
| `borenstein_1996_correction_systematic_odometry_errors.pdf` (IEEE TRA 1996, 6 pp.) | 1, 2, 4, 9 |

Related files already downloaded in `../C2_drive_kinematics/sources/` (may be cited by reference or copied in step 2): Mandow 2007, Baril 2020, Endo 2007, Seegmiller 2013, Helmick 2006, Kozłowski 2004, `ros2controllers_2024_diff_drive_odometry.cpp`, Clearpath Husky/Jackal control YAMLs — relevant to subtopics 6, 7, 8, 11.

## Search log

| # | Query | Useful result |
|---|---|---|
| 1 | Borenstein "Where am I" sensors and methods for mobile robot positioning pdf | Deep Blue record; 1996 report |
| 2 | Kelly "Linearized error propagation in odometry" IJRR 2004 pdf | Citation confirmed (IJRR 23(2):179–218) |
| 3 | Chong Kleeman "accurate odometry and error modelling for a mobile robot" ICRA 1997 | ICRA 1997 pp. 2783–2788 |
| 4 | Antonelli Chiaverini Fusco least squares odometry calibration IEEE TRO 2005 | TRO 21(5):994–1004 |
| 5 | Martinelli odometry calibration augmented Kalman filter simultaneous localization | Auton. Robots 2007 |
| 6 | Ojeda Borenstein over-constrained odometry / wheel slip detection tracked | Auton. Robots 16 (2004); current-based slip |
| 7 | Seegmiller Kelly enhanced 3D kinematic modeling / dynamic models slip calibration | RSS 2014 open PDF |
| 8 | Baril Kubelka Giguère Pomerleau skid-steering kinematic models evaluation | CRV 2020, arXiv 2004.05131 |
| 9 | Pentzer Brennan Reichard track ICR online estimation J. Field Robotics | JFR 31:455–476 (2014) |
| 10 | Reina Ojeda Milella Borenstein wheel slippage and sinkage detection planetary rovers | IEEE/ASME TMech 11(2), 2006 |
| 11 | encoder velocity estimation low speed quantization period counting (Petrella) | Petrella et al. comparative analysis; MT-method papers |
| 12 | Siegwart Nourbakhsh Scaramuzza odometric position estimation error model chapter 5 | §5.2.4 error model |
| 13 | Alonzo Kelly Linearized Error Propagation in Odometry ri.cmu.edu pdf | Open PDF on publications.ri.cmu.edu |
| 14 | Censi Franchi Marchionni Oriolo simultaneous calibration odometry and sensor parameters | TRO 29(2):475–492, 2013 |
| 15 | Thrun Burgard Fox Probabilistic Robotics odometry motion model α parameters | ch. 5 odometry motion model |
| 16 | Wang 1988 location estimation and uncertainty analysis for mobile robots | ICRA 1988 pp. 1231–1235 |
| 17 | Mandow et al. experimental kinematics wheeled skid-steer IROS 2007 pdf | IROS 2007 pp. 1222–1227 |
| 18 | ros2_controllers diff_drive_controller odometry.cpp integrateRungeKutta2 integrateExact covariance | Source + user docs; PR #1394 on timestep handling |
| 19 | Martínez Mandow Morales approximating kinematics tracked mobile robots IJRR 2005 | IJRR 24(10):867–878 |
| 20 | Borenstein Feng Gyrodometry 1996 ICRA | ICRA 1996 pp. 423–428 |
| 21 | odometry calibration bidirectional path Lee Chung / PC-method Doh Choset Chung | PC-method; Jung & Chung 2011 |
| 22 | robot_localization preparing your data wheel odometry covariance nav_msgs/Odometry | Official r_l docs; Nav2 odometry setup guide |
| 23 | Ward Iagnemma model-based wheel slip detection outdoor mobile robots ICRA 2007 | ICRA 2007 pp. 2724–2729; classification-based follow-up |
| 24 | Iagnemma Dubowsky Mobile Robots in Rough Terrain | Springer STAR vol. 12, 2004 |
| 25 | Kelly Mobile Robotics Mathematics Models and Methods Cambridge 2013 | Textbook, no open copy |
| 26 | Kleeman odometry error covariance estimation two wheel robot vehicles Monash | Open tech reports MECSE-95-1, MECSE-6-1996 |
| 27 | REP 105 coordinate frames mobile platforms odom frame drift | reps.openrobotics.org/rep-0105 |
| 28 | "A calibration method for odometry … least-squares technique" pdf | No open copy found |
| 29 | Borenstein Where am I deepblue pdf pos96rep | Author page (403 on fetch); Deep Blue copy |
| 30 | survey odometry wheel slip estimation skid-steer tracked terrain error percent | Endo slip-compensated odometry on slopes; terrain-adaptive odometry; 2023 off-road slippage survey (JIRS) |


## Gap check (2026-09-27)

Independent completeness pass (STANDARDS.md §5 step 3). README subtopics 1–11 were compared against every SCOPE question and the foundational list.

**Gaps found (9):**
1. SCOPE 3(b): Thrun α1–α4 odometry motion model not covered (book not downloadable). **Filled** via the Nav2 AMCL implementation at tag 1.1.18 (L1-S37) and Nav2 AMCL docs (L1-S38).
2. SCOPE 3(d): when linearised Gaussian models fail, with no source. **Filled** with Long et al., RSS 2012 "banana distribution" (L1-S36).
3. SCOPE 9(c): trajectory metrics (RPE/ATE, % and deg/m drift) missing. **Filled** with Sturm et al., IROS 2012 (L1-S39) and Geiger et al., CVPR 2012 KITTI (L1-S40).
4. SCOPE 4(c)–(d): Kümmerle joint calibration named but not sourced; no quantified parameter change with load. **Filled** with Kümmerle et al., Advanced Robotics 2012 (L1-S42).
5. SCOPE 8(a), 8(c): Ward & Iagnemma model-based detector not downloadable; classification-based immobilisation detection missing. **Partly filled** with Ward & Iagnemma, ICRA 2007 classification paper (L1-S41); the model-based ICRA 2007 / T-RO 2008 paper remains not downloaded.
6. SCOPE 1(b): integration-error size for Euler vs RK2 vs exact at typical rates. Searched; only lecture slides and forum posts found (not accepted). **Open question** added.
7. SCOPE 1(c): odometry on slopes / 3D projection. Only Seegmiller & Kelly (L1-S22) present; no general source found. **Open question** added.
8. SCOPE 7(d): agricultural-vehicle odometry errors. Not searched in depth; no open source found. **Open question** added.
9. Pentzer et al. 2014 (online ICR EKF): re-searched (Wiley, Academia, Scribd, Penn State Pure); still no open copy. **Remains not downloaded.**

**Already open and unchanged:** timestamping/jitter/counter wrap-around effects (5d), quantitative effect of controller velocity-filter lag on pose error (5c), forward vs inverse skid-steer parameters (6d), calibrated tracked-vehicle accuracy on grass, and the foundational references Wang 1988, Antonelli 2005, Martinelli 2007, Martínez 2005, Siegwart 2nd ed. (no open copies).

**Search log additions:**

| # | Query | Useful result |
|---|---|---|
| 31 | Ward Iagnemma dynamic-model-based wheel slip detector pdf | Classification-based ICRA 2007 paper open (conference CD); model-based paper paywalled |
| 32 | Kümmerle Grisetti Burgard simultaneous calibration localization mapping pdf | Advanced Robotics 2012 author manuscript (Freiburg) |
| 33 | odometry integration error Euler vs Runge-Kutta vs exact arc comparison | Only lecture slides / forum answers; nothing accepted |
| 34 | Pentzer Brennan Reichard track ICR online estimation pdf | Still paywalled only |
| 35 | (direct) RSS 2012 banana distribution; TUM RGB-D benchmark; KITTI CVPR 2012; nav2_amcl differential_motion_model.cpp @1.1.18; docs.nav2.org AMCL page @588d374 | Downloaded as L1-S36–S40 |
