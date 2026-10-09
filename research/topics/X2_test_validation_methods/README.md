# X2 — Test and validation methods

| | |
|---|---|
| **Question** | How are the velocity loop, odometry, localization and navigation of a mobile robot tested and validated against a reference, which metrics and pass criteria are used, and how should test campaigns be designed, analysed and reported? |
| **Covers** | Measurement foundations (accuracy, precision, uncertainty per GUM), velocity-loop step/frequency tests and metrics, odometry benchmark tests (UMBmark and successors), ground-truth reference systems, trajectory error metrics and alignment (ATE/RPE, KITTI drift), benchmarks and datasets, path-following and navigation metrics, standardised test methods (NIST/ASTM, ISO 18646, ISO 12188-2), robustness and simulation-vs-field testing, campaign design and statistics, and trajectory-evaluation tooling used with ROS. |
| **Not covered** | Theory of the subsystems being tested (C1–C4, L1–L5) and reference stack configurations (X1). |
| **Status** | Draft |
| **Last updated** | 2026-09-28 |

Page numbers in citations are the page numbers of the downloaded PDF file unless marked otherwise. For the Åström & Murray textbook the printed page is given first and the PDF page in brackets, e.g. `p. 151 (PDF 163)`. Standards and handbooks are cited by clause or section number. Code is cited by file and line.

## Summary
- A test result is only meaningful together with its uncertainty. GUM splits uncertainty evaluation into Type A (statistics of repeated observations) and Type B (any other information), combines them into a combined standard uncertainty, and multiplies by a coverage factor k (typically 2 to 3; k = 2 gives about 95 % for near-normal results) to report an expanded uncertainty [X2-S10, §2.3.1–2.3.6, §6.3.3]. NIST's AGV work asks for ground truth at least 10 times more accurate than the system under test [X2-S16, pp. 2, 5].
- For odometry, the UMBmark test drives a 4×4 m square five times clockwise and five times counter-clockwise, and reports the larger of the two cluster-centroid return errors (E_max,syst). Running both directions stops the two main systematic errors from cancelling [X2-S01, pp. 11–13]. Calibrating wheel-diameter ratio and effective wheelbase from it gave 10- to 22-fold smaller systematic error in most runs on a differential-drive robot (one run reached only 6.4-fold until a second calibration) [X2-S01, p. 22].
- For localization against ground truth, the standard metrics are the absolute trajectory error (ATE: align the two trajectories, then take the RMSE of the position differences) and the relative pose error (RPE: the error of motion over a fixed interval, which measures drift) [X2-S04, pp. 6–7]. KITTI reports relative error per segment length (100–800 m) as % and deg/m [X2-S05, p. 5] [X2-S06, lines 14, 87–120]. The choice of alignment (which transform, and how many poses) changed the ATE by about 150 % in one worked example, so it must be stated with the result [X2-S08, p. 8].
- For navigation and path following, standard test methods specify an apparatus, a procedure and a metric [X2-S17, PDF p. 7]. They score reliability as a number of successes in repeated trials; NIST's ASTM work uses 10–30 repetitions for "80% reliability with 80% confidence" [X2-S17, PDF p. 11], while NIST's AGV proposal lists 0 failures in 10, 1 in 20 or 3 in 30 for 80 % reliability with 85 % confidence [X2-S16, p. 5]. Path-following accuracy is reported as deviation from the commanded path (mean, maximum, standard deviation) [X2-S16, p. 3] or as cross-track error between passes, as in ISO 12188-2 [X2-S20, pp. 4–7].
- Campaign design follows standard design-of-experiments practice: randomise run order, replicate runs, block known nuisance factors, check the measuring system first, and keep all raw data [X2-S13, §5.7, §5.1.3]. Sample size follows from the chosen risks α and β and from the effect size relative to the scatter. For example, detecting a shift of one standard deviation with α = 0.05 and β = 0.10 needs about 11 runs [X2-S11, §7.2.2.2]. Success rates should get Wilson or exact binomial confidence intervals [X2-S11, §7.2.4.1].

## Foundational references
| ID | Reference | Why it is foundational |
|---|---|---|
| X2-S01 | Borenstein & Feng, "Measurement and Correction of Systematic Odometry Errors in Mobile Robots," IEEE T-RA, 1996 | Defines UMBmark (bidirectional square), E_max,syst and the Type A/B error model that later odometry tests are compared against. |
| X2-S02 | Borenstein & Feng, "UMBmark: A Benchmark Test for Measuring Odometry Errors in Mobile Robots," SPIE 1995 | Original benchmark paper; adds the extended UMBmark for non-systematic errors. |
| X2-S03 | Kelly, "Fast and Easy Systematic and Stochastic Odometry Calibration," IROS 2004 | Calibration from arbitrary paths with as little as one known ground-truth point. |
| X2-S04 | Sturm et al., "A Benchmark for the Evaluation of RGB-D SLAM Systems," IROS 2012 | Defines ATE and RPE as used across robotics; documents motion-capture ground-truth accuracy and time synchronisation. |
| X2-S05 | Geiger, Lenz & Urtasun, "Are we ready for autonomous driving? The KITTI vision benchmark suite," CVPR 2012 | Segment-length relative error metrics with RTK/INS ground truth on a road vehicle. |
| X2-S07 | Kümmerle et al., "On measuring the accuracy of SLAM algorithms," Autonomous Robots 27, 2009 | Relation-based (relative) error metric; shows why global-frame error misleads. |
| X2-S08 | Zhang & Scaramuzza, "A Tutorial on Quantitative Trajectory Evaluation for Visual(-Inertial) Odometry," IROS 2018 | Standard tutorial on choosing the alignment by unobservable degrees of freedom, and on ATE vs relative error. |
| X2-S09 | Horn, "Closed-form solution of absolute orientation using unit quaternions," JOSA A 4(4), 1987 | Closed-form least-squares alignment used before computing ATE (cited by X2-S04). |
| X2-S10 | JCGM 100:2008 (GUM) | International standard for evaluating and stating measurement uncertainty. |
| X2-S11, X2-S12, X2-S13 | NIST/SEMATECH *e-Handbook of Statistical Methods* (sections on sample size, proportions, measurement process characterisation and DOE) | Open reference for repeatability, gauge studies, design of experiments, sample sizes and confidence intervals. |
| X2-S14 | Åström & Murray, *Feedback Systems*, Princeton UP, 2008 | Standard definitions of step-response and frequency-response specifications and stability margins. |
| X2-S16 | Bostelman, Hong & Cheok, "Navigation Performance Evaluation for Automatic Guided Vehicles," IEEE TePRA 2015 | NIST ground-truth test procedure and metrics proposed to ASTM F45; open entry point to the paywalled ASTM AGV methods. |
| X2-S17 | NIST (Jacoff et al.), *Guide for Evaluating, Purchasing, and Training with Response Robots Using DHS-NIST-ASTM International Standard Test Methods*, 2014 | Open description of the ASTM E54 response-robot test method structure and repetition rules. |
| X2-S18, X2-S19 | ISO 18646-1:2016 and ISO 18646-2:2024 (publisher previews) | International performance test standards for service-robot locomotion and navigation; only the preview pages (scope, terms, test conditions, first tests) could be read. |
| not downloaded | S. Umeyama, "Least-squares estimation of transformation parameters between two point patterns," IEEE T-PAMI 13(4):376–380, 1991 | The de-facto standard SE(3)/Sim(3) alignment [X2-S08, p. 4]. No open copy found. Described here only through X2-S08 and the evo implementation (X2-S27). |
| not downloaded | ASTM F3244 (Navigation: Defined Area) and other ASTM F45 / E54.09 standards | Paywalled. Described only through X2-S16 and X2-S17. |
| not downloaded | ISO 9283:1998 (manipulator pose/path accuracy and repeatability) | Paywalled; no open preview located. Its "cluster" and "barycentre" terms are reused in ISO 18646-2 [X2-S19, §3.15–3.16]. |
| not downloaded | ISO 12188-2:2012 (GNSS auto-guidance, straight and level travel); ISO 17123-8:2015 (GNSS RTK field test); ISO 3691-4 (driverless industrial trucks) | Paywalled. ISO 12188-2 is described only through X2-S20. ISO 17123-8 and ISO 3691-4 could not be read. |

## Findings

### 1. Measurement foundations: accuracy, precision, uncertainty and traceability
- GUM defines standard uncertainty as the uncertainty of a result expressed as a standard deviation [X2-S10, §2.3.1].
- A Type A evaluation uses statistical analysis of a series of observations; a Type B evaluation uses any other means (e.g. specifications, calibration certificates, experience) [X2-S10, §2.3.2–2.3.3].
- For n independent repeated observations, the Type A standard uncertainty of the mean is the experimental standard deviation divided by √n [X2-S10, §4.2.3].
- When only bounds ±a are known and no value inside is more likely than another (a rectangular distribution), the Type B variance is a²/3 [X2-S10, §4.3.7, Eq. 7].
- The combined standard uncertainty is the square root of the sum of the input variances, each weighted by the squared sensitivity of the result to that input (the "law of propagation of uncertainty", a first-order Taylor approximation); correlated inputs add covariance terms [X2-S10, §5.1.2].
- Expanded uncertainty U = k·u_c defines an interval expected to contain a large fraction of the values that could reasonably be attributed to the measured quantity. The coverage factor k is typically 2 to 3 [X2-S10, §2.3.5–2.3.6, §6.3.1]. When the result is approximately normal and has enough degrees of freedom, k = 2 gives about 95 % and k = 3 about 99 % [X2-S10, §6.3.3].
- A result should be corrected for all recognised significant systematic effects. Enlarging the stated uncertainty instead of applying a known correction "should be avoided" [X2-S10, §3.2.4, §6.3.1 Note].
- Accuracy is "closeness of the agreement between the result of a measurement and a true value" and is a qualitative concept. Repeatability is agreement between successive measurements under the same conditions (same procedure, observer, instrument, location, over a short time). Reproducibility is agreement under changed conditions, and a statement of it must say which conditions changed [X2-S10, B.2.14–B.2.16].
- The reference base, which is the ultimate authority for a unit, is kept by national standards laboratories for fundamental units such as length and time. For comparison purposes it can also be agreed among participants, e.g. through a standard test method [X2-S12, §2.1.1.2].
- A repeatability standard deviation from a single small group of repetitions is not reliable. NIST pools repeatability standard deviations over days, runs and check standards to get more degrees of freedom [X2-S12, §2.4.4.1].
- Resolution is the smallest difference the measuring system can detect and faithfully indicate. The number of digits displayed does not show it, and "No useful information can be gained from a study on a gauge with poor resolution relative to measurement needs" [X2-S12, §2.4.5.1].
- NIST's AGV work notes that standard test methods need ground-truth equipment "with accuracy that is at least 10 times better than the system under test" [X2-S16, p. 2]. The proposed ASTM F45 method says "At least a 10X more accurate measurement system shall be used to measure ground truth" [X2-S16, p. 5].
- Before using it, NIST measured the uncertainty of its motion-capture ground truth over the test area with a metrology bar of known length. The result was a distance standard deviation of 0.26 mm and an angle standard deviation of 0.10° [X2-S16, pp. 2–3].
- The TUM benchmark validated its motion-capture system with a rod of about 2 m carrying markers at both ends. The rod's measured length had a standard deviation of 1.96 mm over the capture area. From this the authors bound the ground-truth error: below 1 mm and 0.5° frame to frame, and below 10 mm and 0.5° absolute. They state the dataset is valid for systems whose ATE/RPE errors are "significantly above these values" [X2-S04, p. 5].

### 2. Velocity-loop characterisation tests and metrics
- Standard step-response measures [X2-S14, p. 151 (PDF 163)]:
  - steady-state value y_ss: the final level of the output;
  - rise time T_r: the time to go from 10 % to 90 % of the final value;
  - overshoot M_p: the percentage by which the output first rises above the final value;
  - settling time T_s: the time after which the output stays within 2 % of its final value (1 % or 5 % are also used).
- These measures can depend on step amplitude in general, but for linear systems overshoot, rise time and settling time do not depend on the step size [X2-S14, p. 151 (PDF 163)]. Because a real drive saturates, this is a reason to state the step size used.
- The bandwidth of a system with finite zero-frequency gain is the frequency at which the gain has fallen by a factor 1/√2 from the zero-frequency gain [X2-S14, p. 155 (PDF 167)].
- A frequency response can be determined experimentally and a transfer function then fitted to the measured gain and phase curves. Åström & Murray show this for a fast piezo drive [X2-S14, p. 258 (PDF 270)].
- Gain margin is the smallest gain increase that makes the loop unstable. Phase margin is the phase lag needed to reach the stability limit at the gain-crossover frequency. Stability margin s_m is the shortest distance from the Nyquist curve to the critical point [X2-S14, pp. 278–279 (PDF 290–291)].
- "Reasonable values of the margins are phase margin φ_m = 30°–60°, gain margin g_m = 2–5 and stability margin s_m = 0.5–0.8" [X2-S14, p. 281 (PDF 293)].
- Skogestad evaluates a tuned loop with two tests, a unit setpoint step and a unit load-disturbance step at the input. For each he scores the output by the integrated absolute error, IAE = ∫|e(t)|dt, and the input effort by the total variation, TV = Σ|u_{i+1} − u_i| (a measure of smoothness) [X2-S15, p. 7 (j. 297)].
- In the same paper a peak sensitivity M_s < 1.7 guarantees gain margin > 2.43 and phase margin > 34.2° [X2-S15, p. 7 (j. 297)].
- Lower IAE usually costs input usage. In one example the IAE-optimal PI controller cut the load-disturbance IAE by a factor 3.27, but raised TV from 1.55 to 3.79 [X2-S15, p. 8 (j. 298)].
- On an agricultural robot, repeated step changes in motor voltage were used to validate a drive model. The test exposed a dead zone: the motor did not start until the control signal exceeded ±20 % of nominal [X2-S21, p. 9].
- The GEM guidelines for motion-control experiments say performance is measured as the error between the reference command and the robot's response. The measuring system "should be accurate itself": odometry is prone to wheel-slip errors, so a ceiling camera (indoors) or an accurate absolute localisation system (outdoors) is recommended [X2-S25, p. 10/25].

### 3. Odometry and kinematic calibration tests
- The uni-directional square test is unsuitable for differential-drive robots, because an unequal-wheel-diameter error and a wheelbase error can cancel each other in one direction [X2-S01, pp. 8–10].
- UMBmark procedure [X2-S01, pp. 12–13]:
  1. Measure the vehicle's absolute start position and initialise odometry to it.
  2. Drive a 4×4 m square clockwise, stopping after each 4 m leg, making four 90° turns on the spot, and driving slowly to avoid slip.
  3. Measure the absolute end position and compare it with odometry.
  4. Repeat for five runs in total, then five runs counter-clockwise.
- The UMBmark metric: compute the centre of gravity of the five return-position errors in each direction, then take E_max,syst = max(r_c.g.,cw, r_c.g.,ccw), the larger distance of the two centres from the origin. The maximum is used, not the average, because applications must plan for the largest error [X2-S01, p. 12].
- Type A errors change the total rotation in the same sense in both directions and are linked to wheelbase error E_b. Type B errors change it in opposite senses in the two directions and are linked to the wheel-diameter ratio E_d [X2-S01, pp. 13–14].
- In the UMBmark experiments, calibrating only E_b and E_d reduced E_max,syst by 10- to 22-fold in most runs; one run improved only 6.4-fold until a second compensation [X2-S01, p. 22]. In the first experiment it fell from 317 mm to 21 mm, a 15-fold improvement [X2-S01, p. 21].
- Borenstein & Feng give a rule of thumb for when to calibrate again. If E_max,syst < 3·SEM (standard error of the mean, σ/√n), a second compensation is unlikely to help. With σ ≈ 25 mm, SEM was 11.2 mm; one run with E_max,syst = 66 mm > 33.6 mm was recalibrated and reached 20 mm [X2-S01, p. 22].
- UMBmark needs only a tape measure and reference walls, yet it can isolate wheel diameters that differ by as little as 0.1 % [X2-S01, p. 23].
- The spread within each UMBmark cluster reflects non-systematic errors, but only for the floor tested. Comparing that spread between robots does not show which is more susceptible to non-systematic errors [X2-S02, p. 6].
- The extended UMBmark tests susceptibility to non-systematic errors [X2-S02, pp. 6–7]:
  - It places a round cable about 9–10 mm in diameter under the inside wheel 10 times, spread evenly along the first straight leg.
  - It scores the average absolute orientation error, not the position error, because position error depends on where the bumps occur.
- Kelly's method calibrates both systematic and stochastic odometry models from arbitrary trajectories. It uses the path-dependence of odometry to cut the needed ground truth to "as little as a single known point" [X2-S03, p. 1].
- In Kelly's validation, 28 different trajectories were driven from the same physical start point to the same physical end point [X2-S03, p. 6].

### 4. Ground-truth reference systems
- Motion capture (indoor): TUM used eight Raptor-E cameras (up to 300 Hz) tracking passive markers and recorded ground truth at 100 Hz [X2-S04, pp. 1, 4]. NIST's AGV lab used twelve wall-mounted cameras 4.3 m above the floor, with 18 reflective spheres forming a rigid model of the vehicle [X2-S16, pp. 2–3].
- Laser tracker: NIST found it unsuitable for an AGV that rotates, because it needs continuous line of sight to its target [X2-S16, p. 3].
- Vaidis et al. (field robot, forest) summarise indoor motion capture as sub-millimetre and high-rate but limited in area and struggling in direct sunlight. Outdoors the main options are GNSS, total stations and qualitative comparison with maps [X2-S22, p. 1].
- Total stations (Vaidis et al.):
  - Three total stations tracking three prisms gave full 6-DoF pose, with an average positional error of 10 mm and 0.6° [X2-S22, p. 1].
  - Each station measured range to 2 mm in nominal conditions, tracked out to 800 m, and ran at up to 2.5 Hz in prism-tracking mode [X2-S22, p. 4].
  - Polling all three over one radio channel gave about 1.4 Hz, and the data were interpolated to 20 Hz [X2-S22, p. 4].
- The total-station reference was less precise when the robot started, stopped or turned sharply; the authors attribute this to tracking during abrupt velocity changes and to non-simultaneous sampling of the three prisms [X2-S22, p. 5].
- Dynamic RTK compared with total stations, same study: in an open quarry the precision of the two was equivalent. On a forest trail, GNSS precision "drops drastically, hovering around 1.6 m", while the total-station reference averaged 7.0 mm [X2-S22, p. 7].
- Vaidis et al. estimated reference precision from known fixed distances between rigidly mounted targets (inter-prism and inter-antenna distances), in the manner of Pomerleau et al. [X2-S22, pp. 4–5].
- Ground-vehicle RTK/INS: KITTI's odometry ground truth is the output of an OXTS RT 3003 GPS/IMU unit with RTK corrections; the paper states "open sky localization errors < 5 cm" [X2-S05, pp. 1–3]. The devkit readme states "<10cm" with "RTK float/integer corrections enabled" [X2-S06, readme lines 10–11].
- Time synchronisation: TUM found the time offset between motion capture and camera by evaluating residuals for different candidate delays. The motion-capture poses were about 20 ms earlier, and this was corrected in the dataset [X2-S04, p. 6].
- Vaidis et al. put all total-station data on the robot computer's clock with a message-based synchronisation protocol, repeated periodically during experiments with a low-pass filter on the offset estimate [X2-S22, pp. 2–3].
- Extrinsic calibration between the reference markers and the sensor frame is itself validated. For example, TUM measured the checkerboard corners predicted by motion capture against those seen in the image: 3.25 mm and 4.03 mm average error for its two sensors [X2-S04, p. 5].
- NIST could not measure the AGV's navigation reference point directly. It modelled the vehicle's origin and orientation offsets and solved for them from the data. After this adjustment the errors were clearly smaller, showing the need to correct such offsets [X2-S16, pp. 2, 4].

### 5. Trajectory error metrics and alignment
- Before comparison, the estimate and the ground truth must be put in the same frame (alignment), and a summary metric must be chosen [X2-S08, p. 1].
- Horn's closed-form least-squares solution [X2-S09, p. 1 (j. 629)]:
  - The best translation is the difference between the centroid of one point set and the rotated and scaled centroid of the other.
  - The best scale is the ratio of the RMS deviations of the two sets from their centroids.
  - The best rotation, as a unit quaternion, is the eigenvector of the largest eigenvalue of a symmetric 4×4 matrix.
- ATE [X2-S04, p. 7]: find the rigid transform S that best maps the estimate onto the ground truth (Horn's method), compute F_i = Q_i⁻¹·S·P_i, and report the RMSE of the translational parts.
- RPE [X2-S04, p. 6]: E_i = (Q_i⁻¹Q_{i+Δ})⁻¹(P_i⁻¹P_{i+Δ}) measures the drift over a fixed interval Δ. It is reported as the RMSE of the translational components; some authors use the mean or the median to reduce the influence of outliers [X2-S04, p. 7].
- Choice of Δ [X2-S04, pp. 6–7]: Δ = 1 gives drift per frame, and Δ = 30 at 30 Hz gives drift per second. Setting Δ = n (start vs end point only) is "a common (but poor) choice" because it penalises early rotation errors more than late ones. For SLAM, RPE can be averaged over all Δ.
- In TUM's experiments ATE and RPE were strongly correlated, and the ranking of methods often did not change between them [X2-S04, p. 7].
- ATE is a single, easily compared number, but it is sensitive to when an error occurs: an early rotation error gives a larger ATE than the same error late in the run [X2-S08, p. 5].
- Relative error gives a set of errors over sub-trajectories, so medians and percentiles can be reported. Short sub-trajectories show local consistency and long ones show long-term accuracy [X2-S08, p. 6].
- The alignment must match the degrees of freedom the estimator cannot observe [X2-S08, pp. 3–5, Table I]:
  - monocular vision: similarity transform (Sim(3), rotation, translation and scale);
  - stereo vision: rigid body (SE(3));
  - visual-inertial: yaw-only rotation plus translation (4 DoF), because gravity makes roll and pitch observable.
- There is "no gold standard" for which poses to use for alignment. Using all poses tends to give a lower ATE; using only the first pose(s) shows error growing over time [X2-S08, p. 4].
- In one example, the translation ATE differed by about 150 % between aligning on the first pose and aligning on all poses. The authors conclude that the alignment states must be the same across compared algorithms and must "always be presented together with the evaluation results" [X2-S08, p. 8].
- Kümmerle et al. show the flaw of global-frame error with a 1-D example: an error e in the first motion is counted in every later pose, giving a total error of T·e, whereas shifting the whole map gives an error of only e [X2-S07, p. 6].
- Kümmerle et al. instead sum squared translational and rotational errors of selected relative displacements δ_ij, and suggest evaluating the two parts separately [X2-S07, p. 7]. Relations between nearby poses stress local consistency; relations between far-apart poses (e.g. from an external measurement device) stress global accuracy [X2-S07, pp. 7–8].
- KITTI builds on Kümmerle's relative metric in two ways: it treats rotation and translation separately, and it reports errors as a function of trajectory length and speed [X2-S05, p. 5].
- The KITTI evaluation code [X2-S06, lines 14, 87, 108–120]:
  - uses segment lengths 100, 200, …, 800 m and a start frame every 10 frames ("every second");
  - computes the relative pose error over each segment;
  - divides the translation and rotation errors by the segment length, giving % and deg/m.
- Estimator consistency is tested with NEES and NIS (normalized estimation error squared and normalized innovation squared) [X2-S26, p. 6]:
  - NEES = e_xᵀP⁻¹e_x, where e_x is the true estimation error and P the filter's covariance. It needs ground truth.
  - NIS = e_zᵀS⁻¹e_z, where e_z is the innovation. It needs only the measurements.
  - For a correctly tuned filter they are χ²-distributed with n_x and n_z degrees of freedom, so their means should be about n_x and n_z.
- NEES and NIS are χ²-distributed only for a correctly tuned filter, so a χ² test alone "is not sufficient to ensure that an estimator has been correctly tuned" [X2-S26, p. 1].
- The GEM SLAM guidelines list consistency as a separate criterion ("how realistic / optimistic / pessimistic" the error estimate is), measurable by NEES given ground truth [X2-S25, p. 7/25].
- Kümmerle et al. note that a global NEES suffers from the same problem as global error, and that not every algorithm provides a covariance [X2-S07, pp. 6–7].

### 6. Benchmarks and datasets
- TUM RGB-D benchmark: 39 sequences in an office (6×6 m²) and an industrial hall (10×12 m²), with motion-capture ground truth, including sequences from a camera mounted on a wheeled Pioneer robot [X2-S04, pp. 1, 3–4]. It provides evaluation scripts for RPE and ATE [X2-S04, p. 6].
- KITTI odometry benchmark: 22 stereo sequences totalling 39.2 km, with RTK/INS ground truth [X2-S05, pp. 2, 4]. Ground truth is released only for sequences 00–10; sequences 11–21 are held back for evaluation [X2-S06, readme lines 12–13].
- TUM also evaluates part of its benchmark only on the benchmark website "to avoid over-fitting" [X2-S04, p. 2].
- BARN (ground navigation): 300 simulated obstacle environments ordered by difficulty metrics, which "can also be easily instantiated in the physical world" [X2-S23, p. 1].
- TUM warns that "a good trajectory does not necessarily imply a good map": a small map error can still stop a robot, e.g. an obstacle wrongly placed in a doorway [X2-S04, p. 6].
- The GEM SLAM guidelines call for evaluation on public real datasets with ground truth, and note that many available datasets lack ground truth or cover only a few sensors [X2-S25, p. 6/25].

### 7. Path-following and navigation performance metrics
- NIST's AGV tests commanded a straight line (5 m), 3 m circles and 3 m squares, each traversed 10 times, with each experiment repeated three times [X2-S16, pp. 2–3]. Performance was reported as the mean, maximum and standard deviation of the errors in x, y, distance and angle between ground truth and the vehicle [X2-S16, p. 3].
- At 0.25 m/s the AGV tracked the circle and square paths with a standard deviation of about 3 mm to 5 mm [X2-S16, p. 6]. The maximum deviation from the commanded straight line was about ±25 mm [X2-S16, p. 3].
- The main metric proposed to ASTM F45 is "path traversal accuracy" over a specified number of continuous repetitions. Secondary metrics are elapsed time and average tasks per minute, and human intervention or recharging during a test counts as a fault [X2-S16, p. 5].
- In ISO 12188-2-based testing (agricultural GNSS auto-steer), the tractor is tested at a representative vehicle point (RVP) along the guidance line (the "XTE test"). The measured quantity is the relative cross-track error (XTE): the spacing error between adjacent parallel passes [X2-S20, pp. 3–4, 7].
- In the Eminoğlu et al. field version, the pass-to-pass distance was measured every 5 m and compared with the steering system's log by an independent-samples t-test; no significant difference was found (p > 0.05) [X2-S20, p. 7]. The authors state that overlap or skip is expected to be within 2–3 cm for automatic steering systems [X2-S20, p. 3].
- Moreno et al. (agricultural robot) measured path offset as the distance between desired and followed path using dynamic time warping, a method that pairs up points of two curves with different timing [X2-S21, p. 17].
- To compare vehicles of different size, Moreno et al. normalised as relative path offset = maximum observed path offset / vehicle width [X2-S21, p. 17]. In one test they saw a transient deviation of 5 cm and a steady-state error of 2.5 cm on a robot with an 81.28 cm axle track [X2-S21, p. 14].
- Obstacle navigation benchmark (BARN) [X2-S23]:
  - Difficulty is described by path metrics such as distance to closest obstacle (averaged along the path), average visibility, dispersion, characteristic dimension and tortuosity [X2-S23, pp. 3, 5].
  - Performance is traversal time normalised by path length (s/m), from five trials per planner per environment, with a 30-second penalty for a failed trial [X2-S23, p. 4].
- Success weighted by path length (SPL) is one navigation metric used alongside success rate [X2-S24, p. 2]. In Kadian et al., an episode counted as successful only if the robot stopped within 0.2 m of the goal, within 200 steps and with at most 40 collisions [X2-S24, p. 4].
- The NIST response-robot mobility tests report average rate of advance (m/min) on a figure-8 path of at least 150 m. The path runs over increasingly complex terrains: sand, gravel, flat line following, continuous ramps, crossing ramps and stepfields [X2-S17, PDF p. 13].
- The GEM obstacle-avoidance guidelines recommend storing sensor data and odometry so that an experiment can be re-run off-line and measurements recomputed [X2-S25, p. 14/25].

### 8. Standardised test methods (NIST/ASTM/ISO)
- A standard test method contains an apparatus (a repeatable, reproducible representation of a task), a procedure (a script for administrator and operator) and a metric (a quantitative measure, possibly with thresholds of acceptability). It places no design constraints on the robot [X2-S17, PDF p. 7].
- Each robot configuration must be clearly described and put through all applicable test methods. NIST's AGV proposal adds that any variation in configuration requires retesting across all applicable methods [X2-S17, PDF p. 7] [X2-S16, p. 5].
- Repetition rules for response robots [X2-S17]:
  - Trials use 10–30 repetitions for "80% reliability with 80% confidence", and up to three interactions (resets or minor repairs) are allowed within a 30-repetition trial [PDF p. 11].
  - A trial can stop after 10 successes with no failures. Otherwise no more than 1 failure in 20 or 3 in 30 is allowed [PDF p. 17].
- NIST's AGV navigation proposal lets the test sponsor set reliability R and confidence C. For 80 % reliability with 85 % confidence it allows at most 0 failures in 10, 1 in 20, 3 in 30, 4 in 40, 6 in 50 or 8 in 60 repetitions [X2-S16, p. 5].
- The same proposal requires recording pre-test information (date, facility, vehicle, light, temperature, humidity) and environmental settings on a standardised test form [X2-S16, pp. 5–6].
- ISO 18646-2:2024 covers the following navigation characteristics [X2-S19, Contents, §1]:
  - pose accuracy and pose repeatability;
  - obstacle detection and obstacle avoidance;
  - path deviation;
  - narrow passage;
  - mapping accuracy.
- Clauses 8–10 (path deviation, narrow passage, mapping accuracy) are new in the 2024 edition [X2-S19, Foreword].
- ISO 18646-2 applies to mobile platforms in contact with the travel surface and to indoor environments; an informative Annex A covers outdoor use. It is not for verifying safety requirements [X2-S19, §1].
- ISO 18646-2 test conditions [X2-S19, §4.1–4.5]:
  - ambient temperature 10–30 °C, humidity 0–80 %, illumination 100–1 000 lux;
  - a hard, even, horizontal surface with friction coefficient 0.6–1.0;
  - the robot at rated speed and rated load;
  - test paths scaled by a length unit LU = ⌈w/500 mm⌉ × 500 mm, where w is the robot width;
  - landmarks and preparations recorded in the test report.
- ISO 18646-2 reuses the ISO 9283 terms "cluster" (the set of measured points used to calculate accuracy and repeatability) and "barycentre" (the mean point of a cluster) [X2-S19, §3.15–3.16].
- ISO 18646-1:2016 (wheeled locomotion) tests rated speed, stopping characteristics, maximum slope angle, maximum speed on a slope, mobility over a sill and turning width [X2-S18, Contents].
- ISO 18646-1 rated-speed test [X2-S18, §5.2–5.3]:
  - The measurement area must be at least 1 000 mm long, with space to accelerate and decelerate.
  - A trial fails if the robot deviates from the designated direction by more than 10 % of that length.
  - Rated speed is the minimum speed of three consecutive successful trials.
- ISO 18646-1 defines stopping distance as the maximum distance travelled by the platform origin between initiation of the stop and full stop [X2-S18, §3.10].

### 9. Robustness, stress and failure-mode testing
- The standard test methods vary apparatus settings, e.g. terrain type, slope and lighting. NIST's AGV proposal lists floor surface type, wetness and friction, temperature and humidity, and lighting (down to ≤ 0.1 lux for dark tests) as test conditions to record [X2-S16, p. 5].
- In the response-robot methods, a robot that passes a setting to statistical significance moves on to "more aggressive apparatus settings to determine the limit of the robot's capabilities" [X2-S17, PDF p. 17].
- The extended UMBmark is a disturbance-injection test: controlled bumps under one wheel expose sensitivity to non-systematic odometry errors [X2-S02, pp. 6–7].
- Borenstein & Feng note that real applications must determine and test the largest possible disturbance, because scatter on a smooth floor says nothing about the error after a large bump [X2-S02, p. 6].
- NIST lists "Make a Process Robust (i.e., the process gets the "right" results even though there are uncontrollable "noise" factors)" among the uses of design of experiments. The same page describes reducing variation by experimenting with the hard-to-control factors [X2-S13, §5.1.2].
- NIST treats a process as a black box with controllable input factors, measured responses, and uncontrolled factors, both discrete (e.g. machines, operators) and continuous (e.g. ambient temperature), that the experiment has to account for [X2-S13, §5.1.1].
- Hardware-in-the-loop (HIL) emulation, where real controller hardware runs against a simulated plant, was used to test path-tracking strategies before field tests of an agricultural robot. The authors add that "it is almost impossible to reproduce in emulation external conditions" of the real environment [X2-S21, p. 17].
- Kadian et al. measure how well simulation predicts reality with the Sim2Real Correlation Coefficient (SRCC): the sample Pearson correlation between each method's score in simulation and on the real robot [X2-S24, p. 5].
- For one widely used simulator setting, SRCC was 0.18 for success rate, with 9 rank reversals between simulation and reality. Tuning simulator parameters raised it to 0.844 [X2-S24, pp. 1–2].
- In BARN, 50 physical trials in five new environments confirmed that the difficulty predicted from simulation matched real performance; the fitted line had a slope of 0.96 [X2-S23, p. 5].
- The GEM motion-control guidelines ask for "a set of representative trajectories" rather than a single one, including cases where the controller "might not behave so well" [X2-S25, p. 10/25].

### 10. Field campaign design, statistics and reporting
- Randomisation schedules runs so that conditions in one run neither depend on nor predict the next; NIST states it "is necessary for conclusions drawn from the experiment to be correct, unambiguous and defensible" [X2-S13, §5.7].
- Replication means running the same treatment combination more than once. It gives an estimate of random error separate from lack-of-fit error [X2-S13, §5.7].
- Blocking concentrates a known nuisance effect (e.g. a change of operator or machine) into the blocking variable by restricting randomisation [X2-S13, §5.7].
- NIST's DOE checklist includes [X2-S13, §5.1.3]:
  - check the performance of gauges/measurement devices first;
  - watch for process drifts and shifts during the run;
  - avoid unplanned changes;
  - "Preserve all the raw data--do not keep only summary averages!";
  - "Record everything that happens".
- The sample size needed to detect a shift δ in a mean is N = (z_{1−α/2} + z_{1−β})²(σ/δ)² for a two-sided test. Here α is the risk of rejecting a true hypothesis and β the risk of missing a real shift. When σ must be estimated, the t-distribution version is used and iterated [X2-S11, §7.2.2.2].
- NIST's worked example: to detect a one-standard-deviation increase with α = 0.05 and β = 0.10 (one-sided) gives N ≈ 9 with the normal approximation, and N ≈ 11 after one iteration with t critical values [X2-S11, §7.2.2.2].
- NIST's table for two-sided tests with α = 0.05 and β = 0.10 gives 43, 11 and 5 samples for shifts of 0.5σ, 1.0σ and 1.5σ [X2-S11, §7.2.2.2].
- For a proportion such as a success rate, NIST gives the Wilson interval, recommended by Agresti and Coull "for virtually all combinations of n and p". Its lower limit cannot be negative, unlike the common p̂ ± z√(p̂(1−p̂)/n) formula. For very small numbers of failures or small samples, NIST gives an exact binomial interval [X2-S11, §7.2.4.1].
- Vehicle field tests in the sources state their run structure explicitly. Eminoğlu et al. ran each test three times on two consecutive days, randomly assigned start times, spaced runs more than one hour apart to vary GNSS satellite geometry, and let more than 24 h pass between the first and last test [X2-S20, pp. 4–5].
- Borenstein & Feng used five runs per direction with the standard error of the mean to judge whether a further calibration is justified [X2-S01, p. 22].
- The GEM general guidelines ask whether [X2-S25, pp. 3–4/25]:
  - the hypotheses and system limits are clear;
  - evaluation criteria and "success" are stated;
  - the measurements match the criteria;
  - there is enough information (methods, parameter settings, benchmark variations) to reproduce the work;
  - uncontrolled variations are eliminated, grouped or handled statistically;
  - conclusions are consistent with the statistics.
- Outliers "may not be eliminated from analysis without justification and discussion" [X2-S25, p. 4/25].
- GEM notes that a common weakness of motion-control papers is "the lack of statistical relevance", results often being a single trajectory [X2-S25, p. 10/25].
- GUM asks a report to err toward too much information [X2-S10, §7.1.4]:
  - describe the calculation methods;
  - list all uncertainty components and how they were evaluated;
  - make each analysis step followable and repeatable;
  - give all corrections and constants and their sources.

### 11. Tooling: trajectory evaluation and benchmarking scripts used with ROS
- evo (third-party open-source tool) [X2-S27]:
  - Provides `evo_ape` (absolute pose error, "useful to test the global consistency of a trajectory") and `evo_rpe` (relative pose error, "insights about the local accuracy, i.e. the drift") [wiki, sections "evo_ape", "evo_rpe"].
  - Reads ROS bag files, including TF topics [wiki, introduction].
- evo alignment options [X2-S27, wiki "Alignment"]:
  - `--align` gives SE(3) Umeyama alignment;
  - `--align --correct_scale` gives Sim(3);
  - `--correct_scale` gives scale only;
  - `--align_origin` aligns the origins only, which "can be useful for drift/loop closure evaluation".
- In evo, RPE can be computed per distance, e.g. `--delta 1 --delta_unit m` for drift per metre. Pose pairs are consecutive unless `--all_pairs` is given [X2-S27, wiki "evo_rpe"; metrics.py lines 201–260].
- evo reports RMSE, mean, median, standard deviation, min, max and SSE [X2-S27, metrics.py lines 53–60, 143–158]. Its RPE follows the TUM notation (E_i from reference and estimate pose pairs) [X2-S27, metrics.py lines 262–282].
- evo associates two trajectories by timestamp with a default maximum difference of 0.01 s and an optional time offset [X2-S27, sync.py lines 42–64, 71–77].
- rpg_trajectory_evaluation (the toolbox released with X2-S08) [X2-S28, README lines 7, 85–93, 131, 146, 249–276]:
  - offers alignment types `sim3`, `se3`, `posyaw` (translation plus yaw) and `none`, with `align_num_frames` (−1 = all poses);
  - defaults to `sim3` using all poses if no configuration file is given;
  - computes relative error "in the same way as in KITTI";
  - supports multiple trials, summarised as median/mean/min/max and RMSE boxplots.
- Nav2's planner benchmarking scripts [X2-S29]:
  - run a set of planners over randomly generated maps and goals [README];
  - compute planning time, path length, and average and maximum costmap cost along each path [process_data.py lines 37–42, 58–70, 88–129].

## Recommended practice
What the sources recommend, in order of a typical campaign.
1. State the question, the hypotheses, the system limits, the performance criteria and what counts as success before testing [X2-S25, p. 3/25] [X2-S13, §5.1.3].
2. Choose a ground-truth system at least 10× more accurate than the system under test, and measure its own uncertainty over the test area, e.g. with a metrology bar or rod of known length [X2-S16, pp. 2–3, 5] [X2-S04, p. 5].
3. Calibrate and validate time offsets and extrinsics between reference and robot, and correct known offsets rather than inflating the uncertainty [X2-S04, pp. 5–6] [X2-S16, p. 4] [X2-S10, §6.3.1 Note].
4. Describe the exact robot configuration and retest after any change [X2-S17, PDF p. 7] [X2-S16, p. 5].
5. Plan repetitions from the risks and effect size (sample-size formula), randomise run order, block known nuisance factors, and record environmental conditions [X2-S11, §7.2.2.2] [X2-S13, §5.7] [X2-S16, pp. 5–6].
6. For odometry, run the test in both directions (UMBmark), slowly and with on-the-spot turns, five runs each; use E_max,syst and the SEM rule to decide on recalibration [X2-S01, pp. 12–13, 22].
7. For velocity loops, test both a setpoint step and a load-disturbance step and report output error (e.g. IAE, overshoot, settling time) together with input effort (TV) [X2-S15, p. 7] [X2-S14, p. 151 (PDF 163)].
8. For trajectories, pick the alignment by the estimator's unobservable degrees of freedom, keep it the same across compared methods, and report it with the result. Report both ATE and relative error [X2-S08, pp. 4–6, 8].
9. For pass/fail reliability, use a repetition-and-failure table for a stated reliability and confidence, and report success rates with Wilson or exact binomial intervals [X2-S16, p. 5] [X2-S11, §7.2.4.1].
10. Keep all raw data and logs, do not drop outliers without justification, and report methods, parameters, uncertainty components and corrections in enough detail for someone else to repeat the work [X2-S13, §5.1.3] [X2-S25, pp. 3–4/25] [X2-S10, §7.1.4].

## Key numbers
| Quantity | Value | Conditions | Source |
|---|---|---|---|
| Ground-truth accuracy ratio | ≥ 10× better than system under test | NIST AGV / ASTM F45 proposal | X2-S16, pp. 2, 5 |
| Coverage factor k | typically 2 to 3; k = 2 ≈ 95 %, k = 3 ≈ 99 % | near-normal result, adequate degrees of freedom | X2-S10, §2.3.6, §6.3.3 |
| Rectangular-distribution variance | a²/3 | bounds ±a, no other knowledge | X2-S10, §4.3.7 |
| Rise time / settling band | 10 %→90 %; within 2 % (1 % or 5 % also used) | step response definitions | X2-S14, p. 151 (PDF 163) |
| Bandwidth | gain down by 1/√2 from zero-frequency gain | finite zero-frequency gain | X2-S14, p. 155 (PDF 167) |
| Reasonable margins | PM 30°–60°, GM 2–5, s_m 0.5–0.8 | general loop design | X2-S14, p. 281 (PDF 293) |
| UMBmark path and runs | 4×4 m square, 5 runs cw + 5 runs ccw | differential drive, slow, on-the-spot turns | X2-S01, pp. 12–13 |
| UMBmark improvement after calibration | 10- to 22-fold in most runs (e.g. 317 mm → 21 mm) | LabMate, smooth concrete floor | X2-S01, pp. 21–22 |
| Extended UMBmark bump | cable ≈ 9–10 mm dia., 10 bumps on first leg | non-systematic error test | X2-S02, p. 6 |
| Motion-capture GT error (TUM) | < 1 mm and 0.5° frame-to-frame; < 10 mm and 0.5° absolute | 8 cameras, 100 Hz GT | X2-S04, pp. 1, 5 |
| Motion-capture uncertainty (NIST AGV lab) | σ 0.26 mm distance, σ 0.10° angle | 12 cameras, metrology bar | X2-S16, p. 3 |
| Total-station GT | average 10 mm and 0.6°; 7.0 mm average on forest trail | 3 Trimble S7, ~1.4 Hz polled, 20 Hz interpolated | X2-S22, pp. 1, 4, 7 |
| RTK under forest canopy | precision around 1.6 m | same trail as above | X2-S22, p. 7 |
| KITTI GT | < 5 cm open sky (paper); < 10 cm (devkit readme) | OXTS RT 3003 with RTK | X2-S05, p. 2; X2-S06, readme |
| KITTI segment lengths | 100–800 m in 100 m steps; start every 10 frames | odometry benchmark | X2-S06, lines 14, 87 |
| Motion-capture to camera time offset | ≈ 20 ms | TUM benchmark | X2-S04, p. 6 |
| ATE change from alignment choice | ≈ 150 % | first pose vs all poses, VINS-Mono on EuRoC MH01 | X2-S08, p. 8 |
| AGV path-tracking scatter | σ ≈ 3–5 mm | circle and square paths at 0.25 m/s | X2-S16, p. 6 |
| Reliability repetitions (AGV) | 0/10, 1/20, 3/30, 4/40, 6/50, 8/60 failures | 80 % reliability, 85 % confidence | X2-S16, p. 5 |
| Reliability repetitions (response robots) | 10–30 reps; 0/10, ≤1/20, ≤3/30 failures | 80 % reliability, 80 % confidence | X2-S17, PDF pp. 11, 17 |
| Sample size (mean shift) | 43 / 11 / 5 | δ = 0.5σ / 1.0σ / 1.5σ, two-sided, α = 0.05, β = 0.10 | X2-S11, §7.2.2.2 |
| ISO 18646-2 test conditions | 10–30 °C, 0–80 % RH, 100–1 000 lux, friction 0.6–1.0 | indoor navigation tests | X2-S19, §4.2–4.3 |
| ISO 18646-1 rated speed | ≥ 1 000 mm measuring area; min of 3 successful trials; fail if deviation > 10 % of length | wheeled locomotion | X2-S18, §5.2–5.3 |
| Evo time association default | max 0.01 s difference | `associate_trajectories` | X2-S27, sync.py line 76 |

## How it is tested
| Test | What it measures | Pass criterion used in the source | Source |
|---|---|---|---|
| Setpoint step and load-disturbance step | IAE (output), TV (input effort), rise/settling time, overshoot | no universal pass value; compare with IAE-optimal controller and with robustness (M_s, margins) | X2-S15, pp. 7–8; X2-S14, p. 151 (PDF 163) |
| Frequency response (measured, then fitted) | gain/phase vs frequency, bandwidth, margins | margins in "reasonable" ranges PM 30°–60°, GM 2–5 | X2-S14, pp. 258, 281 (PDF 270, 293) |
| UMBmark (bidirectional 4×4 m square) | systematic odometry error E_max,syst; Type A/B errors | recalibrate unless E_max,syst < 3·SEM | X2-S01, pp. 12–13, 22 |
| Extended UMBmark (10 cable bumps) | susceptibility to non-systematic errors (average absolute orientation error) | comparison between robots; no fixed threshold | X2-S02, pp. 6–7 |
| Arbitrary-path calibration | systematic and stochastic odometry parameters | residuals reduced after calibration; single known end point | X2-S03, pp. 1, 6 |
| ATE after alignment | global consistency of a trajectory | valid only if errors are well above ground-truth error | X2-S04, pp. 5, 7 |
| RPE / KITTI segment error | drift per time or per distance (% and deg/m) | ranking against other methods | X2-S04, p. 6; X2-S05, p. 5; X2-S06 |
| NEES / NIS | estimator consistency | mean ≈ state / measurement dimension (χ²); χ² test alone not sufficient | X2-S26, pp. 1, 6 |
| AGV path navigation (line, circle, square × 10, 3 experiments) | deviation from commanded path vs ground truth | reliability table (e.g. 0 failures in 10); path traversal accuracy recorded | X2-S16, pp. 3, 5 |
| Pass-to-pass XTE (ISO 12188-2 style) | relative cross-track error between adjacent passes | agreement with steering log (t-test p > 0.05); overlap/skip expected within 2–3 cm | X2-S20, pp. 3, 7 |
| Response-robot terrain/mobility (figure-8 ≥ 150 m) | rate of advance, completion reliability | ≤ 1 failure in 20 or ≤ 3 in 30 for 80 %/80 % | X2-S17, PDF pp. 13, 17 |
| ISO 18646-1 rated speed | rated speed on horizontal surface | min of 3 consecutive successful trials; deviation ≤ 10 % of test length | X2-S18, §5.3 |
| BARN obstacle navigation | traversal time normalised by path length (s/m) | 30 s penalty on failure; comparison between planners | X2-S23, p. 4 |
| Sim-vs-real correlation (SRCC) | whether simulation ranks methods as reality does | high SRCC (close to 1) = predictive simulator | X2-S24, pp. 2, 5 |

## Common mistakes
- Testing odometry in one direction only: wheelbase and wheel-diameter errors can cancel, hiding both [X2-S01, pp. 9–11].
- Using start-to-end-point error (RPE with Δ = n) as the drift metric: early rotation errors are penalised more than late ones [X2-S04, p. 7] [X2-S05, p. 5].
- Comparing algorithms with different alignment settings, or not reporting them: the ATE can change by about 150 % [X2-S08, p. 8].
- Using an alignment that does not match the estimator's unobservable degrees of freedom (e.g. Sim(3) for a system with known scale): the error is then not the "distance" between equivalent solutions [X2-S08, pp. 3–4].
- Using the robot's own odometry as the reference for motion-control accuracy: it is prone to slip errors [X2-S25, p. 10/25].
- Using a ground truth that is not clearly more accurate than the system tested: results are only valid for errors "significantly above" the ground-truth error [X2-S04, p. 5] [X2-S16, p. 5].
- Treating a low optimisation residual (χ² error) as proof of accuracy: "low χ² errors do not guarantee a good map or an accurate estimate of the trajectory" [X2-S04, p. 2].
- Reporting a single run or trajectory: this gives no statistical relevance [X2-S25, p. 10/25].
- Keeping only summary averages, or dropping outliers without justification [X2-S13, §5.1.3] [X2-S25, p. 4/25].
- Using the normal-approximation interval p̂ ± z√(p̂(1−p̂)/n) for success rates: it can give an impossible negative lower limit [X2-S11, §7.2.4.1].
- Enlarging the uncertainty instead of applying a known correction for a systematic effect [X2-S10, §6.3.1 Note].
- Not correcting the offset between the vehicle's navigation reference point and the tracked point: path errors were clearly larger before this adjustment [X2-S16, p. 4].

## Disagreements between sources
- **Confidence level for the same repetition table.** The NIST response-robot guide states that 10–30 repetitions (0/10, ≤1/20, ≤3/30 failures) give "80% reliability with 80% confidence" [X2-S17, PDF pp. 11, 17]. NIST's AGV proposal attributes a similar table (0/10, 1/20, 3/30, …) to 80 % reliability with 85 % confidence [X2-S16, p. 5]. Both are NIST sources; neither shows the calculation.
- **KITTI ground-truth accuracy.** The CVPR paper states "open sky localization errors < 5 cm" [X2-S05, p. 2]; the official devkit readme states "<10cm" [X2-S06, readme line 10].
- **Travel-surface friction in ISO 18646.** Part 1 (locomotion, 2016) requires a coefficient of friction of 0.75–1.0 [X2-S18, §4.3]; Part 2 (navigation, 2024) requires 0.6–1.0 [X2-S19, §4.3]. They are different parts and editions, so this may be intentional.
- **Squared vs absolute error.** Kümmerle et al. prefer squared relative errors (an "energy" interpretation) but also report absolute values [X2-S07, p. 7]. TUM notes that some researchers prefer the mean or median over RMSE to reduce the effect of outliers [X2-S04, pp. 6–7].
- **Citation detail for X2-S01.** The downloaded author copy is headed "IEEE Transactions on Robotics and Automation, Vol 12, No 5, October 1996" [X2-S01, p. 1]. Other bibliographic records (and topic L1) list issue 6, December 1996, pp. 869–880. The issue number is not confirmed here.

## Open questions
- No open copy was found of the ASTM F3244 navigation test method or of other F45/E54 standards. Their final scoring (as opposed to NIST's proposals in X2-S16 and X2-S17) could not be confirmed.
- The body of ISO 18646-2 (path-deviation, pose-accuracy and obstacle-avoidance procedures, number of repetitions) and the ISO 9283 formulas are behind a paywall; only the preview pages were read.
- ISO 12188-2 and ISO 17123-8 (RTK field precision test) could not be read. How RTK fix/float integrity is tested in a standard way remains open.
- No general source was found giving pass criteria for closed-loop velocity bandwidth, or a step-by-step frequency-sweep (chirp/PRBS) test procedure for a vehicle drive (same gap as topic C1).
- No peer-reviewed source here gives odometry accuracy as "% of distance travelled" typical for good wheeled or tracked robots, or reports UMBmark-style results for skid-steer or tracked vehicles (see L1 for theory).
- Deliberate injection of sensor faults (GNSS outage, IMU bias, wheel slip) and how to score them was not covered by any downloaded source.
- Regression testing (re-running a fixed test after every software change) and CI-based simulation tests for navigation stacks were not covered by any source found.
- Use of ROS 2 diagnostics (topic rates, robot_localization diagnostics) as pass/fail signals was not researched with a source.
- EuRoC (Burri et al., IJRR 2016, laser-tracker and motion-capture ground truth), Pomerleau et al. (IJRR 2012, theodolite ground truth) and Furgale & Barfoot (JFR 2010, visual teach and repeat field path tracking) were identified, but no open copy could be downloaded (publisher pages or bot checks), so their numbers are not cited.

## Sources
| ID | Citation | Link | Accessed | File | Level |
|---|---|---|---|---|---|
| X2-S01 | J. Borenstein, L. Feng, "Measurement and Correction of Systematic Odometry Errors in Mobile Robots," IEEE Trans. Robotics and Automation, vol. 12, pp. 869–880, 1996 (author copy). | https://johnloomis.org/ece445/topics/odometry/borenstein/paper58.pdf | 2026-09-28 | sources/borenstein_1996_correction_systematic_odometry_errors.pdf | A |
| X2-S02 | J. Borenstein, L. Feng, "UMBmark: A Benchmark Test for Measuring Odometry Errors in Mobile Robots," Proc. SPIE Conf. Mobile Robots, Philadelphia, 1995 (author copy). | https://johnloomis.org/ece445/topics/odometry/borenstein/paper60.pdf | 2026-09-28 | sources/borenstein_1995_umbmark.pdf | A |
| X2-S03 | A. Kelly, "Fast and Easy Systematic and Stochastic Odometry Calibration," Proc. IEEE/RSJ IROS 2004, Sendai (author copy of submitted version). | https://www.cs.cmu.edu/~alonzo/pubs/papers/iros04.pdf | 2026-09-28 | sources/kelly_2004_fast_easy_odometry_calibration.pdf | A |
| X2-S04 | J. Sturm, N. Engelhard, F. Endres, W. Burgard, D. Cremers, "A Benchmark for the Evaluation of RGB-D SLAM Systems," Proc. IEEE/RSJ IROS 2012, pp. 573–580, DOI 10.1109/IROS.2012.6385773. | https://cvg.cit.tum.de/_media/spezial/bib/sturm12iros.pdf | 2026-09-28 | sources/sturm_2012_rgbd_slam_benchmark.pdf | A |
| X2-S05 | A. Geiger, P. Lenz, R. Urtasun, "Are we ready for autonomous driving? The KITTI vision benchmark suite," Proc. IEEE CVPR 2012, pp. 3354–3361, DOI 10.1109/CVPR.2012.6248074. | https://www.cvlibs.net/publications/Geiger2012CVPR.pdf | 2026-09-28 | sources/geiger_2012_kitti_benchmark.pdf | A |
| X2-S06 | A. Geiger, P. Lenz, R. Urtasun, KITTI Visual Odometry/SLAM benchmark development kit (readme and `cpp/evaluate_odometry.cpp`), KIT/TTIC (official benchmark archive `devkit_odometry.zip`, unversioned). | https://s3.eu-central-1.amazonaws.com/avg-kitti/devkit_odometry.zip | 2026-09-28 | sources/kitti_devkit_odometry_readme.txt; sources/kitti_devkit_odometry_evaluate_odometry.cpp | B |
| X2-S07 | R. Kümmerle, B. Steder, C. Dornhege, M. Ruhnke, G. Grisetti, C. Stachniss, A. Kleiner, "On measuring the accuracy of SLAM algorithms," Autonomous Robots 27:387–407, 2009, DOI 10.1007/s10514-009-9155-6 (author copy). | http://ais.informatik.uni-freiburg.de/publications/papers/kuemmerle09auro.pdf | 2026-09-28 | sources/kuemmerle_2009_measuring_slam_accuracy.pdf | A |
| X2-S08 | Z. Zhang, D. Scaramuzza, "A Tutorial on Quantitative Trajectory Evaluation for Visual(-Inertial) Odometry," Proc. IEEE/RSJ IROS 2018, pp. 7244–7251. | https://rpg.ifi.uzh.ch/docs/IROS18_Zhang.pdf | 2026-09-28 | sources/zhang_2018_trajectory_evaluation_tutorial.pdf | A |
| X2-S09 | B. K. P. Horn, "Closed-form solution of absolute orientation using unit quaternions," J. Opt. Soc. Am. A 4(4):629–642, 1987. | https://people.csail.mit.edu/bkph/papers/Absolute_Orientation.pdf | 2026-09-28 | sources/horn_1987_absolute_orientation.pdf | A |
| X2-S10 | JCGM 100:2008, *Evaluation of measurement data — Guide to the expression of uncertainty in measurement* (GUM), BIPM, DOI 10.59161/JCGM100-2008E. | https://www.bipm.org/documents/20126/2071204/JCGM_100_2008_E.pdf | 2026-09-28 | sources/jcgm_2008_gum.pdf | A |
| X2-S11 | NIST/SEMATECH, *e-Handbook of Statistical Methods*, §7.2.2.2 "Sample sizes required" and §7.2.4.1 "Confidence intervals" (proportions), NIST. | https://www.itl.nist.gov/div898/handbook/prc/section2/prc222.htm ; .../prc241.htm | 2026-09-28 | sources/nist_sematech_handbook_7_2_2_2_sample_size_mean.md; sources/nist_sematech_handbook_7_2_4_1_proportion_ci.md | A |
| X2-S12 | NIST/SEMATECH, *e-Handbook of Statistical Methods*, §2.1.1.2 "Reference base", §2.4.4.1 "Analysis of repeatability", §2.4.5.1 "Resolution", NIST. | https://www.itl.nist.gov/div898/handbook/mpc/section1/mpc112.htm ; .../mpc/section4/mpc441.htm ; .../mpc451.htm | 2026-09-28 | sources/nist_sematech_handbook_2_1_1_2_repeatability_terms.md; sources/nist_sematech_handbook_2_4_4_1_repeatability.md; sources/nist_sematech_handbook_2_4_5_1_resolution.md | A |
| X2-S13 | NIST/SEMATECH, *e-Handbook of Statistical Methods*, §5.1.1 "What is experimental design?", §5.1.2 "What are the uses of DOE?", §5.1.3 "What are the steps of DOE?", §5.7 "A Glossary of DOE Terminology", NIST. | https://www.itl.nist.gov/div898/handbook/pri/section1/pri11.htm ; .../pri12.htm ; .../pri13.htm ; .../pri/section7/pri7.htm | 2026-09-28 | sources/nist_sematech_handbook_5_1_1_doe_what.md; sources/nist_sematech_handbook_5_1_2_doe_uses.md; sources/nist_sematech_handbook_5_1_3_doe_steps.md; sources/nist_sematech_handbook_5_7_doe_glossary.md | A |
| X2-S14 | K. J. Åström, R. M. Murray, *Feedback Systems: An Introduction for Scientists and Engineers*, Princeton University Press, 2008 (author electronic edition). | https://www.cds.caltech.edu/~murray/amwiki | 2026-09-28 | sources/astrom_murray_2008_feedback_systems.pdf | A |
| X2-S15 | S. Skogestad, "Simple analytic rules for model reduction and PID controller tuning," J. Process Control 13:291–309, 2003. | (copied from topic C1, C1 sources) | 2026-09-28 | sources/skogestad_2003_simc_pid_tuning.pdf | A |
| X2-S16 | R. Bostelman, T. Hong, G. Cheok, "Navigation Performance Evaluation for Automatic Guided Vehicles," Proc. IEEE TePRA 2015, Boston, NIST. | https://tsapps.nist.gov/publication/get_pdf.cfm?pub_id=918241 | 2026-09-28 | sources/bostelman_2015_agv_navigation_performance.pdf | A |
| X2-S17 | A. Jacoff (test director), E. Messina, H.-M. Huang et al., *Guide for Evaluating, Purchasing, and Training with Response Robots Using DHS-NIST-ASTM International Standard Test Methods*, NIST Intelligent Systems Division, 2014 (image-compressed copy of the 35 MB original; text and page numbering unchanged). | https://www.nist.gov/system/files/documents/el/isd/ks/DHS_NIST_ASTM_Robot_Test_Methods-2.pdf | 2026-09-28 | sources/nist_2014_response_robot_test_methods_usage_guide.pdf | B |
| X2-S18 | ISO 18646-1:2016, *Robotics — Performance criteria and related test methods for service robots — Part 1: Locomotion for wheeled robots*, ISO (publisher preview: contents, clauses 1–6.1). | https://cdn.standards.iteh.ai/samples/63127/0c8198cd39524679ab704fa982e9dc95/ISO-18646-1-2016.pdf | 2026-09-28 | sources/iso_2016_18646-1_preview.pdf | A |
| X2-S19 | ISO 18646-2:2024, *Robotics — Performance criteria and related test methods for service robots — Part 2: Navigation*, ISO (publisher preview: contents, clauses 1–4.5). | https://cdn.standards.iteh.ai/samples/82643/d7c0eb63bf4e4a6984bd529e4957706b/ISO-18646-2-2024.pdf | 2026-09-28 | sources/iso_2024_18646-2_preview.pdf | A |
| X2-S20 | M. B. Eminoğlu, U. Yegül, U. Türker, "Developing A Field Test Method for Automatic Steering Systems," J. Tekirdag Agricultural Faculty 22(3), 2025, DOI 10.33462/jotaf.1623406. | https://dergipark.org.tr/en/download/article-file/4536172 | 2026-09-28 | sources/eminoglu_2025_autosteer_pass_to_pass_test.pdf | A |
| X2-S21 | I. J. Moreno, D. Ouardani, D. Chaparro-Arce, A. Cardenas, "Real-Time Hardware-in-the-Loop Emulation of Path Tracking in Low-Cost Agricultural Robots," Vehicles 5:894–913, 2023, DOI 10.3390/vehicles5030049. | https://doi.org/10.3390/vehicles5030049 (copied from topic C1 sources) | 2026-09-28 | sources/moreno_2023_agricultural_robot_speed_control.pdf | A |
| X2-S22 | M. Vaidis, P. Giguère, F. Pomerleau, V. Kubelka, "Accurate outdoor ground truth based on total stations," Proc. 18th Conf. on Robots and Vision (CRV), 2021 (open copy: arXiv 2104.14396). | https://norlab.ulaval.ca/pdf/Vaidis2021.pdf | 2026-09-28 | sources/vaidis_2021_total_station_ground_truth.pdf | A |
| X2-S23 | D. Perille, A. Truong, X. Xiao, P. Stone, "Benchmarking Metric Ground Navigation," Proc. IEEE SSRR 2020 (open copy: arXiv 2008.13315v3). | https://arxiv.org/pdf/2008.13315 | 2026-09-28 | sources/perille_2020_benchmarking_metric_ground_navigation.pdf | A |
| X2-S24 | A. Kadian, J. Truong, A. Gokaslan, A. Clegg, E. Wijmans, S. Lee, M. Savva, S. Chernova, D. Batra, "Sim2Real Predictivity: Does Evaluation in Simulation Predict Real-World Performance?," IEEE Robotics and Automation Letters, 2020 (accepted-version preprint, arXiv 1912.06321). | https://arxiv.org/pdf/1912.06321 | 2026-09-28 | sources/kadian_2020_sim2real_predictivity.pdf | A |
| X2-S25 | F. Bonsignorio, J. Hallam, A. P. del Pobil (eds.), *GEM Guidelines*, EURON Special Interest Group on Good Experimental Methodology, version 0.9, 2008 (archived copy). | http://web.archive.org/web/2016id_/http://www.heronrobots.com/EuronGEMSig/downloads/GemSigGuidelinesBeta.pdf | 2026-09-28 | sources/bonsignorio_2008_gem_guidelines.pdf | C |
| X2-S26 | Z. Chen, H. Biggie, N. Ahmed, S. Julier, C. Heckman, "Kalman Filter Auto-tuning through Enforcing Chi-Squared Normalized Error Distributions with Bayesian Optimization," arXiv 2306.07225v1, 2023 (no peer-reviewed version confirmed). | https://arxiv.org/pdf/2306.07225 | 2026-09-28 | sources/chen_2023_kf_tuning_nees_nis.pdf | C |
| X2-S27 | M. Grupp, evo — Python package for the evaluation of odometry and SLAM: `evo/core/metrics.py`, `evo/core/geometry.py`, `evo/core/sync.py` at commit bc52497d7f403bf2afb25be3f760f3d6be1432c1; wiki page "Metrics" at wiki commit 8cb784f3546eece74639b1d3dc09e78677159b72. | https://github.com/MichaelGrupp/evo | 2026-09-28 | sources/evo_bc52497_metrics.py; sources/evo_bc52497_geometry.py; sources/evo_bc52497_sync.py; sources/evo_wiki_8cb784f_metrics.md | C |
| X2-S28 | Robotics and Perception Group, Univ. Zurich, rpg_trajectory_evaluation, `README.md` at commit 8c8ceec55c5c5094a6494208cfc5f54afe0bbc4d (toolbox of X2-S08). | https://github.com/uzh-rpg/rpg_trajectory_evaluation | 2026-09-28 | sources/rpg_8c8ceec_trajectory_evaluation_README.md | B |
| X2-S29 | Open Navigation / Nav2 contributors, navigation2 `tools/planner_benchmarking/README.md` and `process_data.py` at commit 7b9bcb4c2d6851414f0c8db3c1ef5a595221bd74. | https://github.com/ros-navigation/navigation2/tree/7b9bcb4c2d6851414f0c8db3c1ef5a595221bd74/tools/planner_benchmarking | 2026-09-28 | sources/nav2_7b9bcb4_planner_benchmarking_README.md; sources/nav2_7b9bcb4_planner_benchmarking_process_data.py | B |
| — | S. Umeyama, "Least-squares estimation of transformation parameters between two point patterns," IEEE T-PAMI 13(4):376–380, 1991, DOI 10.1109/34.88573. | https://doi.org/10.1109/34.88573 | 2026-09-28 | not downloaded (no open copy) | A |
| — | ASTM F3244, *Standard Test Method for Navigation: Defined Area*, ASTM International. | https://www.astm.org/f3244-21.html | 2026-09-28 | not downloaded (paywalled) | A |
| — | ISO 9283:1998; ISO 12188-2:2012; ISO 17123-8:2015; ISO 3691-4. | https://www.iso.org/standard/22244.html ; https://www.iso.org/standard/54926.html | 2026-09-28 | not downloaded (paywalled) | A |
| — | M. Burri et al., "The EuRoC micro aerial vehicle datasets," IJRR 35(10):1157–1163, 2016; F. Pomerleau, M. Liu, F. Colas, R. Siegwart, "Challenging data sets for point cloud registration algorithms," IJRR 31(14):1705–1711, 2012; P. Furgale, T. Barfoot, "Visual teach and repeat for long-range rover autonomy," JFR 27(5):534–560, 2010. | https://doi.org/10.1177/0278364915620033 ; https://doi.org/10.1177/0278364912458814 ; https://doi.org/10.1002/rob.20342 | 2026-09-28 | not downloaded (no open copy reachable) | A |
