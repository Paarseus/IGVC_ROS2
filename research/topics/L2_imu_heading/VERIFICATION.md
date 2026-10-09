# L2 — IMU and heading: claim verification

| | |
|---|---|
| **Topic** | L2 — IMU and heading |
| **Date** | 2026-09-27 |
| **Reviewer** | independent — claims |
| **Scope** | Every cited item in `README.md` (Summary, Foundational references, Findings, Recommended practice, Key numbers, How it is tested, Common mistakes, Disagreements, cited Open questions), checked against the saved files in `sources/` (PDFs via `pdftotext -layout`, page = PDF page index; text files by line number). Sources themselves are not graded here. |

## Counts

| Status | Count |
|---|---|
| Verified | 237 |
| Partly supported | 19 |
| Not supported | 0 |
| **Total checked** | **256** |

## Items needing correction

- #1 (Summary) Yaw not observable from gravity; needs separate reference (mag, GNSS, Earth rotation, vision or map), else gyro drift — Partly supported: Drop 'vision or map' or cite a source for it (S15 p41 lists map matching/visual aiding as aiding sources, not as yaw references).
- #2 (Summary) Single-antenna GNSS/INS: dynamic alignment; heading unobservable at standstill/low dynamics and during GNSS outages — Partly supported: Add S23 'Heading Determination' page, GNSS/INS limitations, for the outage part.
- #46 (F§2) Xsens Mtx 12 h @100 Hz: BI 36–43 °/h, ARW 4.6–4.8 °/√h — Partly supported: Change to 32–43 °/h.
- #75 (F§4) Complementary filter LP abs angle + HP gyro; related to KF; computationally cheaper — Partly supported: Say 'computationally cheaper than smoothing/optimisation (abstract, p1)' or drop.
- #80 (F§4) 'Low-cost orientation filters commonly neglect Earth rotation': Kok assume negligible (7.29e−5 rad/s) — Partly supported: Drop 'commonly'; state Kok et al.'s assumption only (or cite another source for the generalisation).
- #100 (F§6) NHC + wheel speed: heading/position always unobservable; forward velocity unobservable on straight paths — Partly supported: Rephrase: 'with constraints alone, forward velocity is unobservable on straight paths without pitching or yawing, which is why wheel speed is added'.
- #115 (F§7) MTi-680(G) CZRU automatically starts bias estimation whenever motionless — Partly supported: Add 'when enabled'.
- #124 (F§8) Vasconcelos: all LTI distortions, MLE, no attitude ref; calibration = ellipsoid; alignment = Procrustes — Partly supported: Cite PDF p. 1–2 or reword to the abstract's wording.
- #130 (F§8) Calibration must be repeated whenever removed/remounted or geometry changes; more accurate for smaller disturbances — Partly supported: Use 'Xsens advises repeating ... if the geometry is significantly altered'.
- #131 (F§8) ICC runs continuously to refine hard/soft iron; car example; MFM still recommended — Partly supported: Add 'when enabled (disabled by default)'.
- #163 (F§11) Test: MFM norm ≈1, low std/max error, Gaussian residuals — Partly supported: Cite PDF p. 16 and 18.
- #166 (F§11) Consistency test vs 68/95/99; >68% = conservative; heading below Gaussian values (63.5/93.9/99.2) — Partly supported: Say 'only heading fell below the Gaussian values at 1σ and 2σ (63.5%, 93.9%)'.
- #192 (Rec) 8. Calibrate mounted in homogeneous field, full circles; repeat after any mounting or geometry change — Partly supported: 'repeat after remounting or a significant geometry change'.
- #207 (Key#) Mtx BI 36–43 °/h / ARW 4.6–4.8 °/√h — Partly supported: Change BI to 32–43 °/h.
- #228 (Test) Static-dynamic-static: pass = heading after stop matches gyro-tracked heading — Partly supported: Label as inferred, or write 'no criterion given; source shows the failure (drift to magnetic heading)'.
- #229 (Test) MFM report: norm ≈1, small std/max, Gaussian residuals — Partly supported: Cite p. 16, 18.
- #234 (Test) Error envelope: pass ≥68/95/99% inside 1σ/2σ/3σ — Partly supported: Label as inferred or state 'compared with 68/95/99%; >68% = conservative'.
- #249 (Dis) VectorNav: horizontal accel of any kind, small speed fluctuations suffice vs Xsens 7 m/s; 'not a direct contradiction' — Partly supported: Quote the speed condition; label the 'not a contradiction' sentence as inference.
- #250 (Dis) Wu: global vs linearized verdicts opposite (Table I); earlier linearization claims 'theoretically incorrect' — Partly supported: 'says specific earlier linearization-based claims (its refs 7, 24, 25) are theoretically incorrect'.

## Uncited or unlabelled statements

- Foundational references, L2-S05: "Most-cited Allan-variance application" — evaluative, no source given.
- Foundational references, L2-S09/S14: "Seminal" — evaluative, no source given (the factual parts are supported).
- Disagreements #1: "These are product-specific statements, not a direct contradiction" — reviewer inference, not labelled.
- Disagreements #4: "The difference follows from gyro grade relative to Earth rate" — inference (partly grounded in L2-S15 p. 168), not labelled.
- F§5: "(≈5.7°)" — arithmetic conversion of 100 mrad; correct, no citation needed.
- Open questions without citations are statements about gaps in the research, not factual claims, and were not graded.

## All items

| # | Section | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |
|---|---|---|---|---|---|---|
| 1 | Summary | Yaw not observable from gravity; needs separate reference (mag, GNSS, Earth rotation, vision or map), else gyro drift | S11 §4.5.1 p48; S38 Intro; S23 Heading Determination | Partly supported | S11 p48: 'heading can only be estimated using the gyroscope ... will therefore drift'; S38: gravity 'not possible for the Yaw/Heading axis'; S23 Heading page lists magnetometer, GNSS/INS, GNSS compass, gyrocompassing only. 'Vision or map' is in none of the cited locations (S18 p7 actually says VIO global yaw is unobservable). | Drop 'vision or map' or cite a source for it (S15 p41 lists map matching/visual aiding as aiding sources, not as yaw references). |
| 2 | Summary | Single-antenna GNSS/INS: dynamic alignment; heading unobservable at standstill/low dynamics and during GNSS outages | S23 GNSS-aided INS page, 'Dynamic alignment', 'Static or low-dynamic' | Partly supported | gnss_ins.md l.54, l.103 support alignment + low-dynamic loss. Heading loss 'caused by GNSS outages' is stated on the Heading Determination page l.51, not in the cited GNSS-INS sections (its outage section l.121 says the system 'defaults to an INS'). | Add S23 'Heading Determination' page, GNSS/INS limitations, for the outage part. |
| 3 | Summary | Constant bias -> linear θ=ε·t; white noise -> ARW ∝ √t; bias instability -> 2nd-order RW | S06 p11–13, Table 2 | Verified | p11 'θ(t)=ǫ·t'; eq.5 '∝ square root of time'; Table 2 p13 'A second-order random walk'. | — |
| 4 | Summary | Allan slopes (−1/2 at τ=1 s; flat; +1/2 at τ=3 s); static noise optimistic, ×10+ for lowest-cost | S07 p105–109; S08 | Verified | Hou p105 'reading the slope line at T = 1', p108 flat region, p109 'slope of +1/2 ... at T = 3'; Kalibr l.142–144 'optimistic ... factor of 10x or more may be necessary'. | — |
| 5 | Summary | Mag heading limited by vehicle-fixed (calibratable) vs time-varying/external (not) distortion; 0.76° mean lab to 3.6° σ vs nav-grade INS | S09 p23; S12 p15; S34 p7–8 | Verified | S34 p7–8 two kinds of distortion; S12 p15 'mean error ... 0.76°'; S09 p23 'standard deviation of 3.6° and a mean of 1.2°'. Note: range mixes a mean and a σ. | Optional: state the two statistics separately rather than as a 'range'. |
| 6 | Summary | MTi-670/680: only GeneralMag uses mag; others yaw 0° until GNSS fix + motion (min 7 m/s std GNSS, lower RTK); SetInitialHeading | S32 p26; S33 p27; S35 p27–28 | Verified | S32 p26 'magnetometer data is only actively used in the GeneralMag'; S33 p27 'Yaw will initialize at 0 degrees'; S35 p27 '(min. 7 m/s with standard GNSS, lower with RTK enabled)'; p28 SetInitialHeading. | — |
| 7 | Foundational | Groves — not downloaded, not cited for findings | S01 | Verified | Not cited anywhere in Findings (checked). | — |
| 8 | Foundational | Titterton & Weston — not downloaded, not cited for findings | S02 | Verified | Not cited in Findings. | — |
| 9 | Foundational | Farrell — not downloaded, not cited for findings | S03 | Verified | Not cited in Findings. | — |
| 10 | Foundational | IEEE 952 definitions used only as reproduced in S07 | S04 | Verified | Hou figures 4.2–4.5 marked '(after IEEE 952 1997)' p103–109. | — |
| 11 | Foundational | El-Sheimy et al.; Hou (co-author) thesis used instead | S05 | Verified | Hou is listed co-author in S05 citation; 'most-cited' is an uncited evaluative claim (see uncited list). | — |
| 12 | Foundational | Woodman intro to MEMS gyro errors, Allan variance, drift | S06 | Verified | Sections 3.2, 5.1, 7 cover these (p11–13, 18–19, 28–29). | — |
| 13 | Foundational | Hou thesis: derivation of IEEE 952 Allan terms and estimation accuracy | S07 | Verified | §4.3 p101–114, §4.5 p115–116. | — |
| 14 | Foundational | Gebre-Egziabher: field-domain calibration replacing compass swinging | S09 | Verified | p1 abstract 'In contrast to the traditional method of compass swinging ... estimates magnetometer output errors directly'. | — |
| 15 | Foundational | Vasconcelos: ML calibration for all linear time-invariant distortions | S10 | Verified | p1 abstract 'compensates for the combined effect of all linear time-invariant distortions ... Maximum Likelihood Estimator'. | — |
| 16 | Foundational | Kok et al.: tutorial covering EKF, complementary, optimisation | S11 | Verified | p1 abstract lists smoothing/filtering, EKF and complementary filter; §4.4 p43. | — |
| 17 | Foundational | Dissanayake: NHC aiding with observability analysis | S14 | Verified | p1 abstract; p2 'An observability analysis is also presented'. | — |
| 18 | Foundational | REP-103/REP-145 frames, yaw zero, IMU covariance | S27/S28 | Verified | REP-103 l.169–175; REP-145 Frame Conventions, l.110. | — |
| 19 | Foundational | Xsens primary docs: heading behaviour, profiles, MFM/ICC, specs | S31–S34 | Verified | Datasheet Table 3/6/18; FRM §4.4; UM p27; MagCal §3–4. | — |
| 20 | F§1 | Heading = rotation about vertical axis of gravity-aligned inertial frame | S23 Heading page ¶1 | Verified | heading_determination.md l.6 'rotation of a system about the vertical axis of the inertial reference frame (aligned to gravity)'. | — |
| 21 | F§1 | Xsens heading def; ENU yaw from East, right-hand rule | S32 p20 | Verified | p20 'angle between East (X) and the horizontal projection of the sensor roll axis (x), positive about the local vertical axis'. | — |
| 22 | F§1 | ENU yaw 90° north / 0° east; NWU/NED 0° north | S32 p21 Table 7 | Verified | Table 7 rows as stated. | — |
| 23 | F§1 | REP-103 yaw CCW, 0 at east; differs from compass; drivers convert | S27 l.170–175 | Verified | l.169–175 'yaw is zero when pointing east ... Hardware drivers should make the appropriate transformations'. | — |
| 24 | F§1 | REP-103 prefers quaternion/rotation matrix; Euler discouraged, 24 conventions | S27 l.145–167 | Verified | l.149–167 'No singularities'; 'Euler angles are generally discouraged due to having 24 valid conventions'. | — |
| 25 | F§1 | Xsens Euler gimbal lock at ±90° pitch; absent in quaternion/DCM | S32 p20 fn 6 | Verified | fn 6 'singularity is in no way present in the quaternion or rotation matrix output'. | — |
| 26 | F§1 | REP-145 ENU world frame rel. magnetic north; no mag -> yaw arbitrary | S28 Frame Conventions | Verified | l.35 'x-east, y-north, z-up, relative to magnetic north'; l.37 'orientation around the z axis can be arbitrary'. | — |
| 27 | F§1 | robot_localization assumes ENU, not NED | S29 l.38 | Verified | l.38 'assumes an ENU frame for all IMU data, and does not work with NED'. | — |
| 28 | F§1 | navsat: 0 yaw at east; yaw_offset π/2 for north-zero IMU; declination if mag-north | S30 l.19–25 | Verified | l.21 declination 'needed if your IMU provides its orientation with respect to the magnetic north'; l.25 'pi/2'. | — |
| 29 | F§1 | Declination varies, from WMM; Xsens applies it when position set; GNSS/INS set position automatically | S32 p21; S34 p6 | Verified | S32 p21 'yaw/heading will then be corrected for the declination ... GNSS/INS products set automatically the current position'; S34 p6 note. | — |
| 30 | F§1 | Dynamic-alignment heading ≠ course over ground | S23 GNSS-INS 'Dynamic alignment'; Heading page | Verified | gnss_ins.md l.60 'not the same as assuming heading is in the same direction as the velocity vector'; heading page l.45. | — |
| 31 | F§1 | Sideslip β = γ − ψ; GPS gives velocity; two-antenna also heading | S45 p1–2, eq.3 | Verified | p2 eq.(3) 'β = γ − ψ'; 'two-antenna GPS receiver provides both velocity and attitude'. | — |
| 32 | F§1 | WMM long-wavelength only; crust/external absent; 3–4° anomalies not uncommon (small extent); some >10° | S46 l.93–95 | Verified | l.95 'Declination anomalies of the order of 3 or 4 degrees are not uncommon but are usually of small spatial extent'; 'can exceed 10 degrees'. | — |
| 33 | F§1 | WMM2025 declination 1σ √(0.26²+(5417/H)²)°; lower mid/low latitudes | S46 l.238–265 | Verified | l.238 'one standard deviation'; l.239 'lower at mid- to low-latitudes, and larger near the magnetic poles'; l.265 formula. | — |
| 34 | F§1 | Blackout H<2000 nT; caution 2000≤H<6000 nT | S46 l.227–228 | Verified | l.227–228 as stated. | — |
| 35 | F§1 | REP-145 frame_id = sensor frame; mounting via TF; drivers don't transform; transforming needs body+world | S28 l.41–56, 86–89 | Verified | l.41–43, l.56 'modifications ... delegated to a downstream consumer'; l.89 'requires transforming both the body and the world frames'. | — |
| 36 | F§1 | robot_localization corrects rotated IMU given static TF base_link→IMU frame | S29 l.40 | Verified | l.40 as stated. | — |
| 37 | F§2 | Constant bias -> θ=ε·t; estimate by averaging while not rotating | S06 p11 §3.2.1 | Verified | p11 'long term average of the gyro's output whilst it is not undergoing any rotation'. | — |
| 38 | F§2 | White noise -> ARW ∝√t; GG5300 0.2°/√h (0.28° after 2 h) | S06 p11 §3.2.2 eqs 5–6 | Verified | p11 'Honeywell GG5300 has an ARW measurement of 0.2°/√h ... after 2 hours ... 0.28'. | — |
| 39 | F§2 | Bias instability 1σ over ~100 s, constant temp; RW model; 2nd-order RW; bounded in reality | S06 p12 §3.2.3 | Verified | p12 'typically around 100 seconds, in fixed conditions'; 'second-order random walk'; 'only a good approximation ... for short periods'. | — |
| 40 | F§2 | Temperature bias not in bias stability; linear growth; often highly nonlinear | S06 p12 §3.2.4 | Verified | p12 as stated. | — |
| 41 | F§2 | Calibration errors drift only while turning, ∝ rate and duration | S06 p12 §3.2.5 | Verified | p12 'only observed whilst the device is turning ... proportional to the rate and duration'. | — |
| 42 | F§2 | For MEMS, ARW and uncorrected bias usually most important | S06 p12 §3.2.6 | Verified | p12 as stated. | — |
| 43 | F§2 | Allan procedure: bins ≥9, AVAR = ½ mean of squared successive differences, log-log | S06 p18 §5.1 | Verified | p18 steps 1–3, eq.19 1/(2(n−1))Σ. | — |
| 44 | F§2 | ARW slope −1/2 at T=1; BI flat → 0.664B; RRW +1/2 at T=3; quantization −1 | S07 p101–109 §4.3.1–4.3.4 | Verified | p103 'slope of –1'; p105 'T = 1'; p107 '0.664B'; p109 'T = 3'. | — |
| 45 | F§2 | Allan-deviation % error 1/√(2(N/n−1)); 20k pts: 5k clusters ≈40%, 100-pt ≈5% | S07 p115–116 §4.5 | Verified | p116 eq.4.66; '20,000 data points ... 5,000 points ... approximately 40% ... 100 points ... about 5%'. | — |
| 46 | F§2 | Xsens Mtx 12 h @100 Hz: BI 36–43 °/h, ARW 4.6–4.8 °/√h | S06 p19 Table 4 | Partly supported | Table 4: X 36 °/h, Y 32 °/h, Z 43 °/h; ARW 4.6/4.8/4.8 °/√h. The bias-instability range is 32–43, not 36–43. | Change to 32–43 °/h. |
| 47 | F§2 | FOG/RLG/HRG ~10⁻⁴ °/hr; several MEMS <1 °/hr | S19 p1 | Verified | p1 '(with 10−4 °/hr bias instability)'; 'several groups have reported silicon MEMS gyroscopes with less than 1 °/hr'. | — |
| 48 | F§2 | Uncompensated temperature sensitivity ~500 (°/hr)/°C typical for MEMS | S19 p3 | Verified | p3 'on the order of 500 (°/hr)/°C is typical for MEMS'. | — |
| 49 | F§2 | Smartphone 55 min: bias first/last minute values; recalibrates when stationary | S11 p13 Ex.2.3 | Verified | p13 values exactly as stated. | — |
| 50 | F§2 | Gyro bias easier than accel bias (table not level) | S11 p13 Ex.2.3 | Verified | p13 'difficult to distinguish between a bias and a table that is not completely flat'. | — |
| 51 | F§2 | Allan variance stationary only; sampling, saturation, temperature sensitivity excluded; 'never just rely' | S11 p14 | Verified | p14 quote verbatim. | — |
| 52 | F§3 | Model ω̃=ω+b+n; σg rad/s/√Hz; σbg rad/s²/√Hz | S08 Noise Model, Bias, Table 1 | Verified | l.7, l.43–50, Table 1 l.85–87. | — |
| 53 | F§3 | Discrete σg/√Δt, σbg√Δt; ideal LPF assumption, not for subsampling | S08 White Noise, Bias | Verified | l.31, l.35, l.60. | — |
| 54 | F§3 | Datasheet ARW/noise density = σg; bias RW rarely given; in-run bias stability used | S08 From the Datasheet | Verified | l.100–105 (condition: assuming white noise + random walk dominate). | — |
| 55 | F§3 | Woodman σ=RW/√δt and σ=√(δt/t)·BS | S06 p28–29 eqs 50–51 | Verified | p28 eq.50; p29 eq.51. | — |
| 56 | F§3 | Static-test noise optimistic; ×10 or more for lowest cost | S08 In Practice | Verified | l.142–144. | — |
| 57 | F§3 | Simulation with Allan WN+BI drifted slower than real device | S06 p29 §7.2 | Verified | p29 'drift grows significantly faster when using the real device ... does not fully model all of the error sources'. | — |
| 58 | F§3 | Kalibr recommends 15–24 h stationary record | S08 In Practice | Verified | l.132 '~15-24 hour dataset recording of the IMU being stationary'. | — |
| 59 | F§3 | REP-145 unknown cov = 0; unreported → element 0 = −1; *_stddev params override | S28 Topics, Common Parameters | Verified | l.110; l.126–140 'Overrides any values reported by the sensor'. | — |
| 60 | F§3 | RL: covariances matter; don't inflate, disable; 0 variance → 1e−6 | S29 l.64, 79, 102–103 | Verified | l.64, l.79, l.102–103. | — |
| 61 | F§3 | RL: two orientation sources, one under-reports → fuse orientation from better; other angular vel or _differential | S29 l.60 | Verified | l.60 as stated. | — |
| 62 | F§3 | Random constant, random walk (var q·t, non-stationary), GM; xₖ₊₁=a·xₖ+wₖ | S15 p85–86, 90 | Verified | p85 eqs 3.25–3.26; p86 eq 3.27; p90 eq 3.30, Table 3.1. | — |
| 63 | F§3 | GM: ẋ=−x/T+w, q=2σ²/T, a=e^(−Δt/T), qₖ=σ²(1−e^(−2Δt/T)); T→∞ constant, T→0 white | S15 p87–89 | Verified | p87 eqs 3.28a–b; p88–89 limits. | — |
| 64 | F§3 | GM bounded uncertainty; decays in outage; used for biases/scale factors | S15 p87–88 | Verified | p88 'the state will be forgotten ... uncertainty will ... converge to its designed value'. | — |
| 65 | F§3 | 200 T of data → ~10% accuracy; 4 h T → 800 h; chosen empirically | S15 p89–90 | Verified | p89–90 as stated. | — |
| 66 | F§3 | Add small process noise to constant states for long runs | S15 p86–87 | Verified | p86 'preferable to add noise intentionally to prevent the state covariance from becoming non-positive definite'. | — |
| 67 | F§3 | Tuning: ARW 3.5, VRW 0.6, GM σ=100 deg/h T=1 h, SF 1000 PPM T=4 h; 200–1000 deg/h removed by averaging | S15 p167–168 Table 5.1 | Verified | p168 Table 5.1 and text. | — |
| 68 | F§3 | Heading 63.5/93.9/99.2% vs 68/95/99; conservative except heading | S15 p168–169 Table 5.2 | Verified | p168 'consevative Kalman filter except for the heading'; Table 5.2. | — |
| 69 | F§4 | Accelerometers give inclination; no info about rotation around gravity → mags for heading | S11 p26 §3.4.3; S12 p2 | Verified | S11 p26 'orientation around the gravity vector which can not be determined from the accelerometer'; S12 p2 'small or zero acceleration ... dominated by gravity'. | — |
| 70 | F§4 | Without mag, heading only from gyro, drifts; roll/pitch accurate | S11 p48 Ex.4.2 | Verified | p48 'heading angle drifts significantly ... similar to the drift from dead-reckoning'. | — |
| 71 | F§4 | With mag, heading less accurate: lower SNR, horizontal component only; dip 71° Linköping | S11 p48 Ex.4.1 | Verified | p48 as stated. | — |
| 72 | F§4 | Mag heading everywhere except magnetic poles | S11 p26 | Verified | p26 'except on the magnetic poles, where the local magnetic field ... is vertical'. | — |
| 73 | F§4 | AHRS assumes accel = gravity; sustained accel corrupts pitch/roll; velocity allows compensation | S23 AHRS 'Sustained acceleration' | Verified | ahrs.md l.24, l.68, l.76, l.80. | — |
| 74 | F§4 | Xsens assumes mean movement acceleration zero; long accelerations degrade; GNSS aiding offered | S32 p25–26 §4.4.2 | Verified | p25–26 as stated; NOTE lists GNSS/INS products. | — |
| 75 | F§4 | Complementary filter LP abs angle + HP gyro; related to KF; computationally cheaper | S11 p43 §4.4 | Partly supported | p43 supports LP/HP structure and 'strong relationship between complementary and Kalman filtering for linear models'. 'Computationally cheaper' is not in §4.4; the abstract (p1) calls EKF and complementary filters 'computationally cheaper' than smoothing/optimisation, not than Kalman filters. | Say 'computationally cheaper than smoothing/optimisation (abstract, p1)' or drop. |
| 76 | F§4 | EKF/MEKF workhorse; MEKF 3-component error + normalized quaternion | S13 p5, 12 | Verified | p5 'EKF ... is the workhorse of real-time spacecraft attitude'; p12 'three-vector φ while the correctly normalized four-component q̂ provides a globally nonsingular attitude'. | — |
| 77 | F§4 | Madgwick: cheaper alternative; gradient descent; single parameter β | S42 p1, 4, eq.30, §E | Verified | p1 abstract 'computationally efficient'; p4 eq.30; §E '1 adjustable parameter, β'. | — |
| 78 | F§4 | Madgwick Table I: ψ RMS 1.073/1.110 vs 1.150/1.344; IMU N/A; β 0.033/0.041 | S42 p5 Table I | Verified | Table I and text p5. | — |
| 79 | F§4 | Madgwick mag compensation: disturbances affect only heading; no predefined reference | S42 p4 §D eqs 31–32 | Verified | p4 'magnetic disturbances are limited to only affect the estimated heading component'. | — |
| 80 | F§4 | 'Low-cost orientation filters commonly neglect Earth rotation': Kok assume negligible (7.29e−5 rad/s) | S11 p11, 24, 30, eqs 3.45, 3.70 | Partly supported | p11 '7.29·10−5 rad/s'; p24 and p30 'assume that the magnitude of the earth rotation and of the Coriolis acceleration are negligible'. Kok states its own modelling choice; the generalisation 'commonly' is not in the source. | Drop 'commonly'; state Kok et al.'s assumption only (or cite another source for the generalisation). |
| 81 | F§4 | Shin: Earth rate ≈15 °/h negligible vs 200–1000 deg/h biases; stationary output = initial bias | S15 p167–168 | Verified | p167–168 as stated. | — |
| 82 | F§4 | MER LN-200 (3 °/h max drift) subtract Mars rotation before integrating | S44 p6 | Verified | p6 'subtract out the Mars rotation'; 'maximum drift in the LN-200 IMU specification is 3º/hour'. | — |
| 83 | F§4 | Xsens x/y gyro bias from gravity; z only with mag profile in homogeneous field or roll/pitch >30° for >10 s | S32 p25 §4.4.1 | Verified | p25 as stated. | — |
| 84 | F§5 | Alignment table: static coarse/fine; in-motion GPS velocity or LHU EKF | S15 p69 Table 2.1 | Verified | Table 2.1 p69. | — |
| 85 | F§5 | Low-cost gyros: no stationary gyrocompassing; heading from multi-antenna GPS or compass | S15 p68 | Verified | p68 as stated. | — |
| 86 | F§5 | ψ=atan(vE/vN) if forward axis ∥ velocity; roll 0 within ±5° | S15 p68 eqs 2.73 | Verified | p68 as stated. | — |
| 87 | F§5 | ±180° when backward; LHU until few degrees, then SHU | S15 p77 §3.1.4 | Verified | p77 as stated. | — |
| 88 | F§5 | 40° errors at 11.5 km/h: UKF roll/pitch 10 s, heading 50 s; EKF ~200 s; 60° heading also UKF faster | S15 p172, 174 | Verified | p172 and p174 as stated. | — |
| 89 | F§5 | ωh = 15.041067 °/hr·cos(lat); 12.5 °/hr at 33.7° N; 4.8 at 71.4° N | S19 p2 eq.1 | Verified | p2 as stated. | — |
| 90 | F§5 | 1 °/hr → ~100 mrad at 45°; 0.03 °/hr for 4 mrad at 60° S–60° N | S19 p1, 4 | Verified | p1 '1 °/hr leads to a 100 mrad azimuth uncertainty (at a 45° latitude)'; p4 Fig.4 caption and text. | — |
| 91 | F§5 | Gyrocompassing limited to FOG/RLG; prohibitive SWaP-C | S23 Heading page 'Gyrocompassing' | Verified | heading_determination.md l.87. | — |
| 92 | F§5 | Static alignment with unknown constant biases unobservable; estimator converges per settings; practitioners assume zero biases | S43 p4–5 Thm 1, Remark 2 | Verified | p5 Remark 2 'converge to one of the unobservable states depending on the estimator settings'; 'simply assuming zero inertial sensor biases ... standard deviations ... impose a limit'. | — |
| 93 | F§5 | Rotation about two axes → completely observable; single axis → ≤2 unobservable | S43 p1 abstract | Verified | p1 abstract as stated. | — |
| 94 | F§5 | Xsens output may need time to stabilise (gyro bias); bias changes with temperature/impact | S32 p27 §4.4.5 | Verified | p27 as stated. | — |
| 95 | F§6 | Dynamic alignment compares accel with GNSS; most modern need only horizontal accel | S23 GNSS-INS 'Dynamic alignment' | Verified | gnss_ins.md l.54, l.58. | — |
| 96 | F§6 | Static/low-dynamic → heading lost; also during GNSS outages | S23 Heading page GNSS/INS limitations | Verified | heading_determination.md l.51. | — |
| 97 | F§6 | Short low-dynamic: degrading heading ~1 min industrial grade; fall back on magnetometer | S23 GNSS-INS 'Static or low-dynamic' | Verified | gnss_ins.md l.103. | — |
| 98 | F§6 | MTi-G-710 General: 'the more movement ... better yaw' | S36 MTi-G-710 | Verified | filter_profiles.md l.25. | — |
| 99 | F§6 | Xsens min 7 m/s; 'the more acceleration and movement the better' | S37 Minimum Speed | Verified | automotive.md l.68. | — |
| 100 | F§6 | NHC + wheel speed: heading/position always unobservable; forward velocity unobservable on straight paths | S14 p2 §I | Partly supported | p2 'heading and position of the vehicle are always unobservable' (supported). But forward-velocity unobservability is stated for the case without a speed sensor: 'In such situations the speed of the vehicle needs to be measured, typically ... wheel encoder'. With wheel speed included, as the claim's framing says, forward velocity is measured. | Rephrase: 'with constraints alone, forward velocity is unobservable on straight paths without pitching or yawing, which is why wheel speed is added'. |
| 101 | F§6 | Low-accuracy IMU: heading error grows via z-gyro; also at constant speed | S15 p77–78 §3.1.4 | Verified | p77–78 'This situation can also happen when the vehicle is driven with a constant speed due to the poor observability of the heading'. | — |
| 102 | F§6 | MTi-680G lever arm corrects position/velocity; essential for cm-level PVA | S31 p22; S37 Antenna Placement | Verified | S31 p22 'correct its position and velocity measurements'; S37 l.65 'essential parameter ... reliable cm-level position, velocity and orientation'. | — |
| 103 | F§7 | ZUPTs apply zero-velocity pseudo-measurement when detector says stationary | S16 p2 eq.1 | Verified | p2 eq.(1b) and text. | — |
| 104 | F§7 | ZUPTs: all observable except position, yaw, gyro bias along gravity | S16 p3 §III-A | Verified | p3 as stated. | — |
| 105 | F§7 | ZARU/ZIHR only when rates ≪ gyro bias; detect via gyro variance, not norm detectors | S16 p3 §III-A | Verified | p3 'significantly smaller than the gyroscope bias'; 'temporal variance of the gyroscope measurements'. | — |
| 106 | F§7 | 2-min ZUPT: heading drift >3°; ZIHR held heading | S15 p175 §5.1.3, Fig 5.8 | Verified | p175 'heading drifts over 3 degrees during the ZUPT period'. | — |
| 107 | F§7 | ZIHR useful on parked wheeled vehicle | S15 p66 | Verified | p66 'useful on a wheeled vehicle such as a van ... while parked'. | — |
| 108 | F§7 | NHC: zero lateral and normal velocity; violated by slip/vibration; noisy pseudo-measurements | S14 p3–4 §II-B | Verified | p3 'no side slip ... no motion normal'; 'violated due to ... side slip during cornering and vibration'; p4 'Gaussian white noise'. | — |
| 109 | F§7 | Constraint validity varies (lateral larger in turns); AI-IMU adapts covariance (KITTI) | S17 p1, 4 | Verified | p1 'lateral slip is larger in bends'; p4 'much larger in turns than in straight lines'. | — |
| 110 | F§7 | DGPS course: pitch/heading errors ≤3°/6° at 10–55 km/h; <1° above | S15 p171–172 | Verified | p172 as stated. | — |
| 111 | F§7 | GPS-velocity heading ±180° backward | S15 p77 | Verified | p77. | — |
| 112 | F§7 | MTi-G-710 Automotive assumes yaw = COG; not for side slip (racing, tracked, articulated, rough terrain) | S36; S37 | Verified | filter_profiles.md l.31; automotive.md l.23. | — |
| 113 | F§7 | HighPerformanceEDR estimates bias when motionless; vibration/slow motion may affect | S36 | Verified | filter_profiles.md l.33. | — |
| 114 | F§7 | MGBE: device will not rotate for set period (default 6 s); rejected if motion; AGV at each stop | S38 Performing a MGBE | Verified | mgbe.md l.16, l.30, l.32, l.36. | — |
| 115 | F§7 | MTi-680(G) CZRU automatically starts bias estimation whenever motionless | S33 p26 | Partly supported | p26 'If enabled, the Continuous Zero Rotation Update will ... automatically initiate a gyroscope bias estimation sequence whenever the Motion Tracker is motionless'. The condition 'if enabled' is dropped. | Add 'when enabled'. |
| 116 | F§7 | Large heading error → LHU model, switch to small model at few degrees | S15 p77 | Verified | p77. | — |
| 117 | F§7 | Shin prefers Scherzinger LHU (sin ψz, cos ψz−1), switch both directions | S15 p77–78 eq.3.12 | Verified | p78 'the error model switch can be done in both directions and, therefore, the latter approach will be more appropriate'. | — |
| 118 | F§7 | ZUPT: roll/pitch controlled but heading grows; motivates ZIHR | S15 p101–102 | Verified | p101–102 as stated. | — |
| 119 | F§8 | Hard iron constant/slow; soft iron induced, varies with orientation; model incl. SF, misalignment, noise | S09 p4 §2 eq.2 | Verified | p4 as stated. | — |
| 120 | F§8 | Circle → ellipse (SF, soft iron rotates) → shifted; 3-D ellipsoid | S09 p8–11 | Verified | p8–11 as stated. | — |
| 121 | F§8 | Compass swinging location-dependent, needs heading ref and level; field-domain direct, works with triad | S09 p1, 8 | Verified | p1 abstract; p8. | — |
| 122 | F§8 | Batch estimator diverged often with 10° strip + 10 mG; unsuitable unless large portion; 360° turn for 2-axis | S09 p20–21 | Verified | p20 'diverges just about as frequently as it converges'; p21 'not suitable unless a large portion of the ellipsoid'; '360° turn on a level surface'. | — |
| 123 | F§8 | Calibrated low-cost mags: σ 3.6°, mean 1.2° vs nav-grade INS; <3° RMS one-minute trace | S09 p23 | Verified | p23 as stated. | — |
| 124 | F§8 | Vasconcelos: all LTI distortions, MLE, no attitude ref; calibration = ellipsoid; alignment = Procrustes | S10 p1 abstract | Partly supported | Abstract p1 supports LTI/MLE/no reference/Procrustes, but says calibration is 'equivalent to the estimation of a rotation, scaling and translation'; the ellipsoid statement is on PDF p2 ('calibration parameters describe an ellipsoid surface'). | Cite PDF p. 1–2 or reword to the abstract's wording. |
| 125 | F§8 | Mag-only calibration leaves rotation unknown; Kok & Schön joint; 0.76° mean (max 2.48°) | S12 p3, 15 | Verified | p3 'the rotation of this sphere remains unknown'; p15 'mean error ... 0.76° ... maximum error ... 2.48°'. | — |
| 126 | F§8 | Temporal/spatial (not compensable) vs static (calibratable) distortions | S34 p7–8 §2.1 | Verified | p7–8 as stated. | — |
| 127 | F§8 | Currents (several A), magnets, ferromagnetics; disturbance >10–30 s → converges to disturbed north | S32 p26, 34; S39 | Verified | S32 p34 'strong currents (several amperes)'; p26/p34 '>10 to 30 s'; S39 l.16. | — |
| 128 | F§8 | 2-D: ≥360°, <15 km/h, homogeneous, ≥3 m from ferromagnetics; accurate only within envelope | S34 p11 §3.2 | Verified | p11 as stated. | — |
| 129 | F§8 | Wheeled robot 2-D mapping after two full circles; drone residual noise from motors | S34 p23 | Verified | p23 Fig.18/19 captions. | — |
| 130 | F§8 | Calibration must be repeated whenever removed/remounted or geometry changes; more accurate for smaller disturbances | S34 p10 §3.1 | Partly supported | p10 'it is advised to repeat the calibration' when removed, and when geometry is 'significantly altered'. 'Must' and 'changes' overstate 'advised' and 'significantly altered'. | Use 'Xsens advises repeating ... if the geometry is significantly altered'. |
| 131 | F§8 | ICC runs continuously to refine hard/soft iron; car example; MFM still recommended | S34 p32–33 §4.4.3 | Partly supported | p32 'continuously running in the background'; p32–33 car example; 'MFM ... still recommended over or in addition to the ICC'. Omitted condition: p33 'ICC is disabled by default'. | Add 'when enabled (disabled by default)'. |
| 132 | F§8 | Ideal mag 1–2° over extended periods; field can shift up to 2° per day | S23 Heading page, Magnetometer limitations | Verified | heading_determination.md l.24–33. | — |
| 133 | F§8 | ArduPilot current loop field ∝ area, current; falls with cube of distance; mast, short twisted wires, higher V | S47 interference l.27–118 | Verified | l.31–39, l.83–107. | — |
| 134 | F§8 | CompassMot uses current monitor ('linear with current'); <30% ok, 31–60% grey, >60% move | S47 compass setup l.106–148 | Verified | l.111–115, l.140–147. | — |
| 135 | F§8 | External sources list; not objectively tested | S47 interference l.120–134 | Verified | l.123–134. | — |
| 136 | F§8 | Static-dynamic-static: gyro tracks turn, then heading drifts to wrong mag heading | S23 AHRS 'Drift in the drift-free' | Verified | ahrs.md l.102. | — |
| 137 | F§9 | GNSS compass: moving-baseline RTK; instantaneous differencing; no motion needed | S23 GNSS Compass page | Verified | gnss_compass l.128 'does not require motion'. | — |
| 138 | F§9 | θerr = Perr/L; longer baselines longer lock time | S23 GNSS Compass eq.1 | Verified | l.141–164. | — |
| 139 | F§9 | Clear sky, ≥6 common satellites, multipath-sensitive; ground planes | S23 GNSS Compass 'Challenges' | Verified | l.186. | — |
| 140 | F§9 | u-blox: 2 antennas x-axis → heading+roll; 3 → full; automotive 1–3 m, drones 20–30 cm | S21 p4, 8 | Verified | p4 notes; p8 table. | — |
| 141 | F§9 | ZED-F9H: 0.4 deg heading, <10 s convergence multi-GNSS, 0.3 deg dynamic (50% at 30 m/s); identical antennas | S22 p4–5 Tables 1, 4 | Verified | p4 Table 1 + fn 2; p5 Table 4; p5 identical antennas/orientation. | — |
| 142 | F§9 | Baseline mismatch → wrong ambiguity fix, wrong heading | S21 p20 | Verified | p20 as stated. | — |
| 143 | F§9 | Two-antenna car: 10 Hz vel <3 cm/s, 5 Hz att <0.2 deg; KF with 0.2 deg/s gyro, bias at 100 Hz | S45 p1, 3–4 | Verified | p3 §5 as stated; p4 '100 Hz updates'; p3 'eliminate errors arising from gyro bias'. | — |
| 144 | F§9 | Time alignment via time tags + sync pulse; half-sample latency; offset → significant errors | S45 p3–4 | Verified | p3–4 as stated. | — |
| 145 | F§9 | Single-antenna velocity heading: ±180° reverse; ≤6° at 10–55 km/h | S15 p77, 171–172 | Verified | p77; p172. | — |
| 146 | F§10 | QMG 0.2 °/hr; carouseling 4 mrad; maytagging similar but needs temperature calibration | S19 p1 abstract | Verified | p1 abstract. | — |
| 147 | F§10 | Virtual maytagging 0.204° in 5 min at 28.2°; BI 0.0078 °/h over one day | S20 p1 abstract | Verified | p1 abstract. | — |
| 148 | F§10 | Monocular VINS: 4 unobservable directions (global yaw, position) | S18 p7 | Verified | p7 as stated. | — |
| 149 | F§10 | Dissanayake abstract (attitude observable) vs intro (heading unobservable) | S14 p1–2 | Verified | p1 abstract; p2 intro. | — |
| 150 | F§10 | Dual-antenna heading used for farming, heavy machinery, ships, cars | S21 p5–7 | Verified | p5 applications list; p6–7 figures. | — |
| 151 | F§10 | MER sun imaging (16° FOV): Sunfind heading from sun + tilt; Sungaze QUEST | S44 p1, 3–4 | Verified | p1; p4. | — |
| 152 | F§10 | Sun heading fails near local noon | S44 p4 | Verified | p4 'At local noon ... the heading cannot be determined'. | — |
| 153 | F§10 | Between sun updates gyro propagation; Articulate accel tilt; yaw only via gyro | S44 p5–6 | Verified | p6 'Yaw knowledge is not changed by this operation, although it is by the gyro integration'. | — |
| 154 | F§10 | IMU off when stationary: saves power, avoids gyro drift | S44 p1 | Verified | p1 'IMU is only turned on when the rover's attitude is expected to be changing. This also prevents ... gyro drift errors'. | — |
| 155 | F§10 | Requirement ≤1.5° (3σ) from HGA 2° (3σ) | S44 p1 | Verified | p1 as stated. | — |
| 156 | F§10 | Re-acquire after ~10,000 s IMU time (~20 sols), by HGA signal strength | S44 p7–8 | Verified | p7–8 as stated. | — |
| 157 | F§10 | Wheel odometry + gyro heading; VO where slip high (≥100% at Gusev) | S44 p6–7 | Verified | p7 'sometimes as much as 100% or more'. | — |
| 158 | F§10 | Gyrocompassing would have been 'comforting' | S44 p8 | Verified | p8 as stated. | — |
| 159 | F§11 | Sustained disturbance → converge to wrong north | S32 p34 | Verified | p34 'converge to a new solution using the new local magnetic north'. | — |
| 160 | F§11 | Magnetization makes calibration useless; mild can be recalibrated | S34 p7 | Verified | p7 NOTE. | — |
| 161 | F§11 | Failed MGBE no extra error except constant-rate rotation | S38 Best practices | Verified | mgbe.md l.43. | — |
| 162 | F§11 | Test: static-dynamic-static drive | S23 AHRS page | Verified | ahrs.md l.92–102. | — |
| 163 | F§11 | Test: MFM norm ≈1, low std/max error, Gaussian residuals | S34 p16 | Partly supported | p16 lists std of norm, average of norm close to 1, maximum error. Gaussian residuals are in Advanced Results, PDF p18 (Fig.12). | Cite PDF p. 16 and 18. |
| 164 | F§11 | Test: 24 orientations 90° apart | S12 p13–15 | Verified | p13 '24 orientations that differ from each other by 90 degrees'. | — |
| 165 | F§11 | Test: nav-grade INS as truth | S09 p23 | Verified | p23 Honeywell YG1851. | — |
| 166 | F§11 | Consistency test vs 68/95/99; >68% = conservative; heading below Gaussian values (63.5/93.9/99.2) | S15 p168–169 Table 5.2 | Partly supported | p168 definitions verified. But heading 3σ = 99.2% is above the Gaussian 99%, so heading is below only at 1σ and 2σ. p168 says 'conservative ... except for the heading', not that only heading was below all three values. | Say 'only heading fell below the Gaussian values at 1σ and 2σ (63.5%, 93.9%)'. |
| 167 | F§11 | MER HGA signal strength; ~10,000 s interval | S44 p7–8 | Verified | p7–8. | — |
| 168 | F§11 | Wrong convergence in static alignment chosen by initial settings | S43 p5 Remark 2 | Verified | p5 Remark 2 'depending on the estimator settings, e.g., the selection of initial value'. | — |
| 169 | F§11 | Allan collection 12 h@100 Hz; 15–24 h; warm-up ≥5, pref 10 min | S06 p19; S08; S38 | Verified | S06 p19; S08 l.132; S38 l.43. | — |
| 170 | F§12 | MTi-600 gyro 8 °/h, 0.007 °/s/√Hz, 520 Hz, 0.001 °/s/g, SF 0.5%/1.5%; accel 60 µg/√Hz | S31 p13 Tables 6–7 | Verified | p13 tables. | — |
| 171 | F§12 | MTi-680G roll/pitch 0.2°/0.5°, yaw 1° dynamic RMS typical | S31 p12 Table 3 | Verified | p12 Table 3 + 'RMS values based on typical application scenarios'. | — |
| 172 | F§12 | Profiles General/NoBaro/Mag (_RTK) with sensors | S31 p24 Table 18; S33 p27 | Verified | Tables as stated. | — |
| 173 | F§12 | GNSS/INS: mag only in GeneralMag; accel vs GNSS acceleration as example | S32 p26 §4.4.3 | Verified | p26 as stated. | — |
| 174 | F§12 | Only GeneralMag north-referenced at power-up; others 0° then converge (7 m/s std, lower RTK); SetInitialHeading highly recommended | S33 p27; S35 p27–28; S37 | Verified | S33 p27; S35 p27; S37 l.48. | — |
| 175 | F§12 | AHS not for GNSS/INS; discouraged | S32 p28 Table 11 | Verified | p28 as stated. | — |
| 176 | F§12 | GNSS outage >45 s stops PV output | S31 p22 | Verified | p22 as stated. | — |
| 177 | F§12 | Bias estimates not stored, not removed from rate output | S38 Best practices | Verified | mgbe.md l.45–47. | — |
| 178 | F§12 | Automotive MFM while driving ≥3 circles | S37 Magnetic Calibration | Verified | automotive.md l.38. | — |
| 179 | F§12 | Driver MTi-680(G) profile index 0/1/2 when enable_filter_config; MGBE off, [15, 5] | S40 yaml l.93–109, 244–255 | Verified | yaml l.99–109; l.249 'enable_manual_gyro_bias: false'; l.255 '[15, 5]'. | — |
| 180 | F§12 | Driver *_stddev default 0; non-Sirius/Avior orientation cov from orientation_stddev squared; −1 if unavailable | S40 imupublisher.h l.50–69, 118–176; yaml l.268–276 | Verified | h l.53–60 defaults {0,0,0}; l.67 variance_from_stddev_param; l.143–148; l.152 '-1'. The squaring itself is in a helper not in the cited file (implied by name). | Optional: note squaring is inferred from the helper name. |
| 181 | F§12 | RotSensor S→O; RotLocal L→L′; inclination/heading/alignment resets | S41 five methods | Verified | resets.md l.13, l.19–23. | — |
| 182 | F§12 | Housing aligned; non-orthogonality '<0.05' (unit lost) | S32 p18 | Verified | p18 'The non-orthogonality between the axes of Sxyz is <0.05'. | — |
| 183 | F§12 | No alignment reset during init; wait ≥5 min | S41 Important remarks | Verified | resets.md l.77. | — |
| 184 | F§12 | Reset volatile unless Store | S41 Orientation resets | Verified | resets.md l.66. | — |
| 185 | Rec | 1. ENU yaw east-zero CCW, or yaw_offset/declination | S27; S29; S30 | Verified | See F§1 rows. | — |
| 186 | Rec | 2. Long stationary thermally stable Allan record; ARW τ=1 s, RRW τ=3 s | S07 p105–109; S08 | Verified | Hou p105, p109; Kalibr l.111–113, 132, 142. | — |
| 187 | Rec | 3. Inflate static-test noise (×10+ lowest-cost) | S08 | Verified | l.142–144. | — |
| 188 | Rec | 4. Real covariances; no huge variances; avoid zero on fused variables | S29 | Verified | l.64, 79, 102–103. | — |
| 189 | Rec | 5. Warm up ≥5/≥10 min before bias estimation; no-rotation/ZIHR at standstill | S38; S15 p175 | Verified | S38 l.36, 43; S15 p175. | — |
| 190 | Rec | 6. Zero-rate detection from gyro variance, not norm | S16 p3 | Verified | p3. | — |
| 191 | Rec | 7. Provide horizontal acceleration; supply initial heading | S23; S33 p27 | Verified | gnss_ins.md l.58; S33 p27. | — |
| 192 | Rec | 8. Calibrate mounted in homogeneous field, full circles; repeat after any mounting or geometry change | S34 p10–11; S37 | Partly supported | p11 and S37 l.38 support procedure. p10 advises repeat when removed or geometry 'significantly altered' — 'any ... geometry change' overstates. | 'repeat after remounting or a significant geometry change'. |
| 193 | Rec | 9. Identical, identically oriented antennas, ground planes; validate with baseline | S22 p5; S23; S21 p20 | Verified | S22 p5; gnss_compass l.126, 186; S21 p20. | — |
| 194 | Rec | 10. IMU mounting as static TF, not in-driver changes | S28 l.41–56; S29 l.40 | Verified | See F§1. | — |
| 195 | Rec | 11. Alignment resets after stabilised (≥5 min) and store | S41 | Verified | l.66, 77. | — |
| 196 | Rec | 12. Time-align GNSS with IMU via tags/sync pulse | S45 p3–4 | Verified | p3–4. | — |
| 197 | Rec | 13. Mag away from DC wiring; short twisted; compensate with measured current | S47 | Verified | interference l.31–39; compass setup l.111–115. | — |
| 198 | Key# | Earth rate 15.041067 °/hr | S19 p2 | Verified | p2. | — |
| 199 | Key# | 0.03 °/hr for 4 mrad, 60° S–60° N | S19 p4 | Verified | p4. | — |
| 200 | Key# | ≈100 mrad from 1 °/hr at 45° | S19 p1 | Verified | p1. | — |
| 201 | Key# | MTi-600 bias stability 8 °/h | S31 p13 | Verified | Table 6. | — |
| 202 | Key# | Noise density 0.007 °/s/√Hz | S31 p13 | Verified | Table 6. | — |
| 203 | Key# | MTi-680G yaw 1° dynamic RMS | S31 p12 | Verified | Table 3. | — |
| 204 | Key# | Xsens min speed 7 m/s; lower RTK | S37; S35 p27 | Verified | S37 l.68; S35 p27. | — |
| 205 | Key# | GNSS outage 45 s | S31 p22 | Verified | p22. | — |
| 206 | Key# | Mag disturbance >10–30 s | S32 p26 | Verified | p26. | — |
| 207 | Key# | Mtx BI 36–43 °/h / ARW 4.6–4.8 °/√h | S06 p19 | Partly supported | Table 4: BI 36, 32, 43 °/h. | Change BI to 32–43 °/h. |
| 208 | Key# | Calibrated mag σ 3.6°, mean 1.2° | S09 p23 | Verified | p23. | — |
| 209 | Key# | Mag ideal 1–2° | S23 | Verified | heading_determination.md l.27–33. | — |
| 210 | Key# | ZED-F9H 0.4 deg Table 4 | S22 p5 | Verified | p5. | — |
| 211 | Key# | 2-min ZUPT >3° | S15 p175 | Verified | p175. | — |
| 212 | Key# | GPS-velocity heading ≤6° 10–55 km/h | S15 p172 | Verified | p172. | — |
| 213 | Key# | Allan error ≈40%/≈5% | S07 p116 | Verified | p116. | — |
| 214 | Key# | ~500 (°/hr)/°C | S19 p3 | Verified | p3. | — |
| 215 | Key# | WMM2025 D formula | S46 l.265 | Verified | l.265. | — |
| 216 | Key# | Anomalies 3–4° not uncommon; >10° | S46 l.95 | Verified | l.95. | — |
| 217 | Key# | Blackout/caution zones | S46 l.227–228 | Verified | l.227–228. | — |
| 218 | Key# | GM σ=100 deg/h T=1 h; ARW 3.5 | S15 p168 | Verified | Table 5.1. | — |
| 219 | Key# | 200×T for ~10%; 800 h for T=4 h | S15 p89–90 | Verified | p89–90. | — |
| 220 | Key# | Madgwick ψ RMS 1.073/1.110 | S42 p5 | Verified | Table I. | — |
| 221 | Key# | β 0.033/0.041 optimal | S42 p5 | Verified | p5 'found these values to provide optimal performance'. | — |
| 222 | Key# | Two-antenna attitude <0.2 deg 1σ at 5 Hz, NovAtel | S45 p3 | Verified | p3. | — |
| 223 | Key# | Wait ≥5 min before alignment reset | S41 | Verified | l.77. | — |
| 224 | Key# | MER ≤1.5° (3σ) from 2° (3σ) | S44 p1 | Verified | p1. | — |
| 225 | Key# | MER ~10,000 s, ~20 sols | S44 p8 | Verified | p8. | — |
| 226 | Key# | CompassMot thresholds | S47 l.140–146 | Verified | l.140–147. | — |
| 227 | Test | Allan: slopes −1/2, 0, +1/2; ≥9 bins | S06 p18–19; S07 | Verified | S06 p18 '≥9 bins', p19 slopes; Hou p109 +1/2. | — |
| 228 | Test | Static-dynamic-static: pass = heading after stop matches gyro-tracked heading | S23 AHRS | Partly supported | ahrs.md l.92–102 describes the test and the drift to the magnetometer heading; it states no pass criterion. The criterion is an inference. | Label as inferred, or write 'no criterion given; source shows the failure (drift to magnetic heading)'. |
| 229 | Test | MFM report: norm ≈1, small std/max, Gaussian residuals | S34 p16 | Partly supported | Norm criteria p16; Gaussian residuals PDF p18. | Cite p. 16, 18. |
| 230 | Test | 24-orientation test; mean 0.76° after ML | S12 p15 | Verified | p15. | — |
| 231 | Test | Nav-grade INS comparison σ 3.6°, mean 1.2° | S09 p23 | Verified | p23. | — |
| 232 | Test | MGBE status 0 success vs 2/3 failed | S38 | Verified | mgbe.md l.32. | — |
| 233 | Test | Baseline length check | S21 p20 | Verified | p20. | — |
| 234 | Test | Error envelope: pass ≥68/95/99% inside 1σ/2σ/3σ | S15 p168–169 | Partly supported | p168 defines Gaussian 68/95/99% and calls >68% 'conservative'; it gives no pass/fail criterion. '≥' as a pass criterion is an inference (an over-conservative filter also satisfies it). | Label as inferred or state 'compared with 68/95/99%; >68% = conservative'. |
| 235 | Test | Optical reference; static if rate <5°/s | S42 p5 | Verified | p5 '< 5°/s'. | — |
| 236 | Test | CompassMot <30% | S47 l.106–148 | Verified | l.140–142. | — |
| 237 | CM | COG as heading on side-slip vehicles or reverse (180°) | S36; S15 p77 | Verified | S36 l.31; S15 p77. | — |
| 238 | CM | NED/north-zero with ENU consumers w/o offset | S29 l.38; S30 l.23–25 | Verified | As F§1. | — |
| 239 | CM | Static noise values → over-trust IMU | S08 | Verified | l.142 'will tend to trust your IMU measurements too much'. | — |
| 240 | CM | Inflating covariances / zero covariances | S29 l.64, 102–103 | Verified | As F§3. | — |
| 241 | CM | Planar calibration then outside envelope | S34 p11; p32–33 | Verified | p11 note; p32 ICC aim. | — |
| 242 | CM | Bias estimation before warm-up or during constant slow rotation | S38 | Verified | l.43. | — |
| 243 | CM | ZUPT alone leaves yaw unobservable | S16 p3; S15 p175 | Verified | S16 p3; S15 p175 'ZUPTs are not enough for low-cost INSs'. | — |
| 244 | CM | Alignment reset before stabilised / not stored | S41 | Verified | l.66, 77. | — |
| 245 | CM | GM parameters from short records | S15 p89–90 | Verified | p89–90. | — |
| 246 | CM | Large high-current loops near mag | S47 l.83–118 | Verified | l.83–118. | — |
| 247 | CM | GPS/INS without time-offset correction | S45 p3–4 | Verified | p4 'any time offset ... may result in significant estimation errors'. | — |
| 248 | CM | WMM declination assumed exact; local anomalies | S46 l.95 | Verified | l.95. | — |
| 249 | Dis | VectorNav: horizontal accel of any kind, small speed fluctuations suffice vs Xsens 7 m/s; 'not a direct contradiction' | S23 Dynamic alignment; S37; S35 p27 | Partly supported | gnss_ins.md l.58: 'most smaller vehicles simply need to get up to a decent speed ... the small fluctuations of a car at highway speed ... is enough'. The README drops 'decent speed'/'at highway speed', which understates VectorNav's speed condition. 'Not a direct contradiction' is an unlabelled inference. | Quote the speed condition; label the 'not a contradiction' sentence as inference. |
| 250 | Dis | Wu: global vs linearized verdicts opposite (Table I); earlier linearization claims 'theoretically incorrect' | S43 p8 Table I | Partly supported | Table I verified (vertical/N-S unobservable globally, observable linearized; E-W reverse). 'Theoretically incorrect' refers to specific prior claims in refs [7], [24], [25], not to earlier linearization-based claims in general. | 'says specific earlier linearization-based claims (its refs 7, 24, 25) are theoretically incorrect'. |
| 251 | Dis | Soft iron: Gebre-Egziabher (induced, constant coefficients) vs Madgwick (earth-frame interference) | S09 p4–6; S42 p4 | Verified | S09 p4, p6 'effective soft iron coefficients ... constants'; S42 p4 'Sources of interference in the earth frame, termed soft iron errors'. | — |
| 252 | Dis | Earth rotation negligible (Kok, Shin) vs gyrocompassing / MER subtract; difference from gyro grade | S11 p24, 30; S15 p167–168; S19 p2; S44 p6 | Verified | All four positions verified. Closing sentence is an inference, partly grounded in S15 p168 ('negligibly small against the large biases'). | Optionally label last sentence as inference. |
| 253 | Dis | Gebre-Egziabher abstract 1–2° vs 3.6° σ experiment; VectorNav 1–2° best case | S09 p1, 23; S23 | Verified | S09 p1 'heading errors on the order of 1 to 2 degrees'; p23 3.6°. | — |
| 254 | OQ | Current-induced mag errors: only drone-oriented docs (S47) found | S47 | Verified | S47 compass setup marked [site wiki='copter']; interference page drone-oriented. | — |
| 255 | OQ | Heading needs horizontal acceleration rests on S23, S37, S15 | S23; S37; S15 | Verified | gnss_ins.md l.58, 103; automotive l.68; S15 p77–78. | — |
| 256 | OQ | COG error vs speed: only measured values in S15 | S15 | Verified | S15 p172 gives measured values only. | — |

## Corrections applied (2026-09-27)

Applied by the topic editor from this review and `SOURCE_AUDIT.md`. Each source was re-checked at the cited location before rewording. After corrections: 0 Not supported, 0 Partly supported, 0 failing sources. README Status set to **Verified**.

### Claims (all 19 Partly supported items rewritten)
- #1 Summary: "vision or map" removed from the list of yaw references (now "for example a magnetometer, GNSS or Earth rotation"); Heading Determination citation re-pointed to L2-S26.
- #2 Summary: added [L2-S26, "Heading Determination" page, "GNSS/INS" limitations] for the GNSS-outage part (line 51 of that file).
- #46 / #207 (F§2, Key numbers): Mtx bias instability changed 36–43 → 32–43 °/h, with per-axis values (36/32/43 °/h) in F§2 (S06 PDF p. 19, Table 4).
- #75 (F§4): "computationally cheaper" split into a separate sentence citing S11 PDF p. 1 abstract (EKF and complementary filters cheaper than optimisation-based smoothing/filtering).
- #80 (F§4): generalisation "Low-cost orientation filters commonly neglect Earth rotation" removed; only Kok et al.'s assumption is stated.
- #100 (F§6): reworded to the S14 wording: with constraints, forward velocity is unobservable on straight paths without pitching/yawing, which is why wheel speed is added; heading and position always unobservable.
- #115 (F§7): "if enabled" added to CZRU (S33 PDF p. 26).
- #124 (F§8): reworded to the S10 abstract wording ("combined effect of all linear time-invariant distortions"; rotation/scaling/translation; alignment = orthogonal Procrustes solution); citation now PDF p. 1–2, abstract and section I.
- #130 (F§8): now "Xsens advises repeating the calibration every time the sensor is temporarily removed ... and if the geometry is significantly altered" (S34 PDF p. 10).
- #131 (F§8): "when enabled (it is disabled by default)" added; citation extended to sections 4.4.3–4.4.4.
- #163 / #229 (F§11, How it is tested): citation extended to S34 PDF p. 16 and 18.
- #166 (F§11): now "only heading fell below the Gaussian values at 1σ and 2σ (63.5% and 93.9%; 99.2% at 3σ)".
- #192 (Recommended practice 8): "repeat after remounting or a significant geometry change".
- #228 (How it is tested): static-dynamic-static pass criterion replaced by "no criterion given; the source shows the failure case".
- #234 (How it is tested): error-envelope criterion now "fractions compared with 68/95/99%; more than 68% inside 1σ = conservative".
- #249 (Disagreements): VectorNav speed condition quoted ("get up to a decent speed", "small fluctuations of a car at highway speed"); "not a direct contradiction" labelled as inference.
- #250 (Disagreements): now "specific earlier linearization-based claims (its references 7, 24 and 25) are theoretically incorrect" (S43 PDF p. 8).
- Optional note on #5 (Summary): the magnetometer error statistics are now stated separately instead of as a range.

### Uncited or unlabelled statements
- Foundational references: "Most-cited" (S05) and "Seminal" (S09, S14) removed; rows now state factual reasons. "Widely cited" (S06) removed.
- Disagreements #4: "difference follows from gyro grade" labelled as inference and linked to S15 PDF p. 167–168.

### Sources
- ID gap L2-S24–S26 closed by splitting the grouped VectorNav primer (formerly all L2-S23) into one ID per page: L2-S23 AHRS (1.6), L2-S24 GNSS-Aided INS (1.7), L2-S25 GNSS Compass/INS (1.8), L2-S26 Heading Determination (1.9), each with its own page URL. All former L2-S23 citations were re-pointed to the page they cite; the three bare [L2-S23] citations (Recommended practice 7 and 9, Key numbers "Magnetometer heading, ideal environment", Disagreements #5) now cite the specific page and section. No other IDs changed, so all other item IDs in the table above remain valid.
- Evidence levels: L2-S06 (un-refereed technical report) A → B; L2-S28 (Draft REP) A → B. Theses S07/S15 stay A (examined thesis).
- Citation details added: DOIs for S12 (10.1109/JSEN.2016.2569160), S19 (10.1109/JMEMS.2013.2282936), S42 (10.1109/ICORR.2011.5975346), S43 (10.1109/TAES.2012.6129622, pp. 78–102), S44 (10.1109/ICSMC.2005.1571116, vol. 1, pp. 20–27) — confirmed via Crossref; S17 arXiv version (v1); dates for S21 (14 Sep 2023) and S22 (21 Mar 2024); "branch humble-devel" removed from S29 (commit pin kept).
- S31 link changed from the Farnell mirror to the official Xsens URL (HTTP 200, application/pdf, same byte size as the saved file).
- Foundational references: S31/S32 currency note added (2020 revisions; S33 is the 2023 export of current docs); REP-145 row notes Draft status.
- Files renamed to publication year: `xsens_2026_kb_automotive_best_practices.md` → `xsens_2022_…`, `xsens_2026_kb_manual_gyro_bias_estimation.md` → `xsens_2024_…`, `xsens_2026_kb_yaw_magnetically_disturbed.md` → `xsens_2022_…`.
- Stray `sources/.playwright-cli/` folder (browser logs, not a source) deleted.
- SCOPE.md: REP-145 author corrected to P. Bovbel.
- No source failed the audit; no source deleted. No text copy had an open-PDF replacement (the optional NOAA WMM technical report is a different document and was not added).

### Final mechanical check
- `file` on all 45 files in `sources/`: every `.pdf` is a PDF, every text file is text.
- Every file appears in the Sources table; every cited ID exists in the table; every table row is cited; IDs L2-S01–L2-S47 have no gaps or duplicates (47 rows).
