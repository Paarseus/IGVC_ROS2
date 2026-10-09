# L2 — IMU and heading

| | |
|---|---|
| **Question** | How do inertial systems obtain, keep and lose heading (yaw relative to north), and what sensor error models and noise values describe that process? |
| **Covers** | Heading definitions and ROS conventions; gyro error sources and Allan variance; turning sensor specs into filter noise and covariance; attitude/AHRS filters; initial alignment; yaw observability in GNSS/INS; standstill, slow and reverse driving (ZUPT/ZARU/ZIHR, motion constraints); magnetometer heading and calibration; single- and dual-antenna GNSS heading; gyrocompassing and visual heading; failure modes and tests; Xsens MTi-600 series and its ROS 2 driver. |
| **Not covered** | General sensor-fusion configuration (L4), GNSS positioning and RTK (L3), wheel odometry (L1). |
| **Status** | Verified |
| **Last updated** | 2026-09-27 |

**Citation note.** Page numbers are the page index of the downloaded PDF file ("PDF p."), which can differ from the number printed on the page. Text sources are cited by section heading or line number of the downloaded file.

## Summary
- Roll and pitch can be observed from gravity, but yaw cannot; it needs a separate reference (for example a magnetometer, GNSS or Earth rotation), otherwise it is carried forward only by integrating the gyro and drifts. [L2-S11, section 4.5.1 (PDF p. 48)] [L2-S38, "Introduction"] [L2-S26, "Heading Determination" page]
- A single-antenna GNSS/INS finds heading by comparing GNSS-observed acceleration with accelerometer-sensed acceleration ("dynamic alignment"); heading becomes unobservable at standstill or in low dynamics and during GNSS outages. [L2-S24, "GNSS-aided INS" page, "Dynamic alignment" and "Static or low-dynamic situations"] [L2-S26, "Heading Determination" page, "GNSS/INS" limitations]
- A constant gyro bias ε produces a heading error that grows linearly (θ = ε·t); white rate noise produces an angle random walk that grows with √t; bias instability produces a second-order random walk. [L2-S06, PDF p. 11–13, Table 2]
- Allan variance separates these noise terms by slope on a log-log plot (−1/2 angle random walk read at τ = 1 s, flat region = bias instability, +1/2 rate random walk read at τ = 3 s); noise values from a static, constant-temperature test are optimistic and may need to be increased (by a factor of 10 or more for the lowest-cost sensors) before use in a filter. [L2-S07, PDF p. 105–109] [L2-S08, "From the Allan standard deviation" and "Kalibr IMU Noise Parameters in Practice"]
- Magnetometer heading is limited by vehicle-fixed (hard/soft iron) distortion, which can be calibrated by ellipsoid fitting, and by time-varying or external distortion, which cannot; after calibration, reported heading errors include a 0.76° mean deviation in a laboratory rotation test, and a 3.6° standard deviation (1.2° mean) in a field comparison with a navigation-grade INS. [L2-S09, PDF p. 23] [L2-S12, PDF p. 15] [L2-S34, PDF p. 7–8]
- For the Xsens MTi-670/680(G) GNSS/INS, only the GeneralMag profiles use the magnetometer; in the other profiles yaw starts at 0° and converges to north only once there is a GNSS fix and sufficient motion (Xsens states a minimum of 7 m/s with standard GNSS, lower with RTK), and an initial heading can be supplied with SetInitialHeading. [L2-S32, PDF p. 26] [L2-S33, PDF p. 27] [L2-S35, PDF p. 27–28]

## Foundational references
| ID | Reference | Why it is foundational |
|---|---|---|
| L2-S01 | Groves, *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*, 2nd ed. (not downloaded) | Standard integrated-navigation textbook (inertial errors, alignment, magnetic heading, INS/GNSS observability). No open copy; not cited for findings. |
| L2-S02 | Titterton & Weston, *Strapdown Inertial Navigation Technology*, 2nd ed. (not downloaded) | Classic strapdown INS reference (alignment, gyrocompassing, error propagation). No open copy; not cited for findings. |
| L2-S03 | Farrell, *Aided Navigation: GPS with High Rate Sensors* (not downloaded) | Aided-navigation text with AHRS and GPS-aided INS case studies. No open copy; not cited for findings. |
| L2-S04 | IEEE Std 952-1997, Allan-variance annex (not downloaded) | Defines ARW, bias instability, rate random walk. Paywalled; its definitions are used here only as reproduced in L2-S07. |
| L2-S05 | El-Sheimy, Hou, Niu, "Analysis and Modeling of Inertial Sensors Using Allan Variance," IEEE TIM 2008 (not downloaded) | Peer-reviewed journal paper applying IEEE 952 Allan-variance analysis to inertial sensors, listed as foundational in SCOPE.md. No reachable open copy; the thesis L2-S07 by its co-author Hou, which contains the same method, is used instead. |
| L2-S06 | Woodman, "An introduction to inertial navigation," Cambridge tech. report, 2007 | Open introduction to MEMS gyro errors, Allan variance and drift propagation (not peer-reviewed; level B). |
| L2-S07 | Hou, *Modeling Inertial Sensors Errors Using Allan Variance*, MSc thesis, Univ. of Calgary, 2004 | Full derivation of the IEEE 952 Allan-variance noise terms and estimation accuracy; stands in for L2-S05. |
| L2-S09 | Gebre-Egziabher et al., "Calibration of Strapdown Magnetometers in Magnetic Field Domain," ASCE J. Aerosp. Eng., 2006 | Field-domain (ellipsoid) magnetometer calibration as an alternative to compass swinging. |
| L2-S10 | Vasconcelos et al., "Geometric Approach to Strapdown Magnetometer Calibration in Sensor Frame," IEEE TAES, 2011 | Standard maximum-likelihood calibration for all linear time-invariant magnetometer distortions. |
| L2-S11 | Kok, Hol, Schön, "Using Inertial Sensors for Position and Orientation Estimation," Found. Trends Signal Process., 2017 | Tutorial/survey of inertial sensor models and orientation estimation (EKF, complementary filter, optimisation). |
| L2-S14 | Dissanayake et al., "The aiding of a low-cost strapdown IMU using vehicle model constraints for land vehicle applications," IEEE TRA, 2001 | Non-holonomic-constraint aiding of a low-cost IMU for land vehicles, with observability analysis. |
| L2-S27, L2-S28 | REP-103 and REP-145 | Official ROS conventions for frames, yaw sign/zero and IMU covariance reporting (REP-145 is a Draft REP). |
| L2-S31–S34 | Xsens MTi 600-series datasheet, MTi Family Reference Manual, MTi 600-series User Manual, Magnetic Calibration Manual | Primary manufacturer specification of MTi heading behaviour, filter profiles, MFM/ICC and sensor specs. L2-S31 and L2-S32 are 2020 revisions; current documentation is maintained at mtidocs.movella.com, of which L2-S33 is a 2023 export. |

## Findings

### 1. Heading fundamentals: definitions, frames and conventions
- Heading (also called yaw or azimuth) is the rotation of a system about the vertical axis of the inertial reference frame, which is aligned to gravity. [L2-S26, "Heading Determination" page, first paragraph]
- Xsens defines heading as the angle between north and the horizontal projection of the roll axis; with the default East-North-Up (ENU) output frame, Xsens yaw is the angle between East and the horizontal projection of the sensor x-axis, positive about the vertical axis by the right-hand rule. [L2-S32, PDF p. 20, "Interpretation of yaw as heading"]
- In ENU the Xsens yaw is 90° when the vehicle points north and 0° when it points east; in NWU or NED it is 0° when pointing north. [L2-S32, PDF p. 21, Table 7]
- ROS REP-103 states that yaw increases as the child frame rotates counter-clockwise and that, for geographic poses, yaw is zero when pointing east; this differs from a compass bearing, which is zero at north and increases clockwise, and drivers should convert before publishing standard ROS messages. [L2-S27, "Rotation Representation", lines 170–175]
- REP-103 prefers quaternions or rotation matrices (no singularities) and discourages Euler angles because there are 24 valid conventions. [L2-S27, "Rotation Representation", lines 145–167]
- Xsens notes that its Euler output has a gimbal-lock singularity when pitch approaches ±90°, which is absent in the quaternion and rotation-matrix outputs. [L2-S32, PDF p. 20, footnote 6]
- REP-145: for ENU-type IMUs the world frame is x-east, y-north, z-up relative to magnetic north; a device without an absolute yaw reference (magnetometer) is aligned only in roll and pitch, and its yaw can be arbitrary. [L2-S28, "Frame Conventions"]
- robot_localization assumes ENU for all IMU data and does not work with NED data. [L2-S29, line 38]
- `navsat_transform_node` expects the IMU to read 0 yaw when facing east; for an IMU that reads 0 facing north, `yaw_offset` should be π/2; `magnetic_declination_radians` is needed if the IMU reports orientation relative to magnetic north. [L2-S30, lines 19–25]
- Magnetic declination (the angle between magnetic and true north) varies with location and can be obtained from the World Magnetic Model; Xsens applies it internally when the position is set, and GNSS/INS devices set the position automatically from a GNSS fix. [L2-S32, PDF p. 21, "True North vs. Magnetic North"] [L2-S34, PDF p. 6, note]
- A GNSS/INS heading from dynamic alignment is not the same as assuming heading equals course over ground (the direction of the velocity vector). [L2-S24, "GNSS-aided INS" page, "Dynamic alignment"] [L2-S26, "Heading Determination" page]
- On a vehicle, the sideslip angle is the difference between the direction of the velocity γ and the vehicle heading ψ, β = γ − ψ; a GPS receiver provides the absolute velocity (and so γ), and a two-antenna receiver also provides heading, so sideslip can be calculated directly (car application). [L2-S45, PDF p. 1–2, eq. 3]
- The World Magnetic Model (WMM) describes only the long-wavelength field from Earth's core; crustal and external fields are not included, local or regional declination anomalies of 3–4° are "not uncommon" (usually of small spatial extent) and some exceed 10°. [L2-S46, lines 93–95]
- The WMM2025 error model gives a one-standard-deviation declination uncertainty of √(0.26² + (5417/H)²)°, with H the horizontal field intensity in nT (superscripts lost in the saved text); the error is lower at mid- to low latitudes and larger near the magnetic poles. [L2-S46, lines 238–265]
- Where H < 2000 nT ("Blackout Zones" around the magnetic poles) compasses are unreliable, and for 2000 ≤ H < 6000 nT ("Caution Zone") they should be used with caution. [L2-S46, lines 227–228]

#### Sensor mounting alignment (IMU frame versus vehicle frame)
- REP-145: the `frame_id` of all IMU messages is the sensor frame (default `imu_link`), and the transform from other frames such as `base_link` to that frame represents the IMU's mounting position and orientation; drivers should not transform data themselves, and transforming IMU data requires changing both the body and the world frame. [L2-S28, lines 41–56 and 86–89]
- robot_localization corrects for an IMU mounted on its side or rotated, provided a static transform from the `base_link_frame` to the IMU message's `frame_id` exists. [L2-S29, line 40]

### 2. IMU error sources and stochastic characterisation
- A constant gyro bias ε integrates to an angle error that grows linearly with time, θ(t) = ε·t; it can be estimated by averaging the output while the gyro is not rotating. [L2-S06, PDF p. 11, section 3.2.1]
- White (thermo-mechanical) rate noise integrates to an angle random walk (ARW) whose standard deviation grows with √t; ARW is quoted in °/√h, e.g. 0.2 °/√h for the Honeywell GG5300 (0.2° after 1 h, 0.28° after 2 h). [L2-S06, PDF p. 11, section 3.2.2, eqs. 5–6]
- Bias instability (flicker noise) is usually specified as a 1σ value over a period of about 100 s at constant temperature and modelled as a bias random walk; integrated, it gives a second-order random walk in angle; the random-walk model holds only for short periods because the real bias stays within a range. [L2-S06, PDF p. 12, section 3.2.3]
- Temperature-induced bias changes are not included in bias-stability figures (measured at fixed conditions), and any residual temperature bias produces an angle error that grows linearly with time; the bias–temperature relationship of MEMS is often highly nonlinear. [L2-S06, PDF p. 12, section 3.2.4]
- Calibration errors (scale factor, misalignment, nonlinearity) produce drift only while the device turns, proportional to the rate and duration of the motion. [L2-S06, PDF p. 12, section 3.2.5]
- For MEMS gyros, angle random walk and uncorrected bias (from temperature or from an error in the initial bias estimate) are usually the most important error sources. [L2-S06, PDF p. 12, section 3.2.6]
- Allan variance: divide a long record into bins of length t (at least 9 bins), average each bin, and compute AVAR(t) = ½·mean of squared differences of successive bin averages; the Allan deviation is plotted against t on log-log axes. [L2-S06, PDF p. 18, section 5.1]
- Reading an Allan deviation plot: angle random walk is a slope of −1/2 and its coefficient is read at T = 1; bias instability is the flat region, where σ tends to 0.664·B; rate random walk is a slope of +1/2 read at T = 3; quantization noise is a slope of −1. [L2-S07, PDF p. 101–109, sections 4.3.1–4.3.4]
- The percentage error of an Allan-deviation estimate is 1/√(2(N/n − 1)) for N data points and clusters of n points; with 20,000 points and 5,000-point clusters it is about 40%, with 100-point clusters about 5%. [L2-S07, PDF p. 115–116, section 4.5]
- Example (MEMS, human-motion IMU): a 12-hour stationary log at 100 Hz from an Xsens Mtx gave gyro bias instability of 32–43 °/h (36, 32 and 43 °/h on the x, y and z axes) and ARW of 4.6–4.8 °/√h. [L2-S06, PDF p. 19, Table 4]
- Grade comparison from the gyrocompassing literature: fiber-optic, ring-laser and hemispherical resonator gyros reach about 10⁻⁴ °/hr bias instability, while several silicon MEMS gyros have been reported below 1 °/hr. [L2-S19, PDF p. 1]
- Uncompensated temperature sensitivity on the order of 500 (°/hr)/°C is described as typical for MEMS. [L2-S19, PDF p. 3]
- Gyro bias is slowly time-varying: for a stationary smartphone over 55 minutes, the gyro bias in the first minute was (35.67, 56.22, 0.30)·10⁻⁴ rad/s and in the last minute (37.01, 53.17, −1.57)·10⁻⁴ rad/s; this is why the phone recalibrates its gyro during stationary periods. [L2-S11, PDF p. 13, Example 2.3]
- Gyro bias is easier to determine than accelerometer bias: leaving the sensor stationary is enough for the gyro, while an accelerometer bias cannot be separated from a table that is not perfectly level. [L2-S11, PDF p. 13, Example 2.3]
- Allan variance describes only stationary conditions in a stable climate; sampling effects, saturation and temperature-dependent sensitivity are not included, so "we should therefore never just rely on the Allan variance" when choosing a sensor. [L2-S11, PDF p. 14]

### 3. Turning sensor specifications into filter noise and covariance
- A common IMU error model is ω̃ = ω + b + n: white noise n of strength σg (noise density, rad/s/√Hz) plus a bias b driven as a random walk of strength σbg (rad/s²/√Hz). [L2-S08, "The IMU Noise Model", "Bias", Table 1]
- Discrete-time conversion: the per-sample white-noise standard deviation is σg/√Δt and the per-sample bias-random-walk step is σbg·√Δt, where Δt is the sample time; the white-noise scaling assumes ideal low-pass filtering to the Nyquist band and does not hold for plain subsampling. [L2-S08, "Additive White Noise" and "Bias"]
- Datasheet "angle random walk" or "rate noise density" corresponds to σg; bias random walk is rarely given directly, and the in-run bias (in)stability, the minimum of the Allan deviation, is often used instead to choose σbg. [L2-S08, "From the Datasheet of the IMU"]
- Woodman converts Allan-variance values to simulation parameters with σ = RW/√δt for white noise and σ = √(δt/t)·BS for the bias random walk (t = averaging time of the bias-stability value). [L2-S06, PDF p. 28–29, eqs. 50–51]
- Noise parameters from a static, constant-temperature record ignore scale-factor errors and temperature-driven bias changes, so they are optimistic; for the lowest-cost sensors, increasing them by 10× or more may be necessary. [L2-S08, "Kalibr IMU Noise Parameters in Practice"]
- A simulation using only the Allan-variance-derived white noise and bias instability drifted more slowly than the real device, indicating that those two terms did not model all error sources. [L2-S06, PDF p. 29, section 7.2]
- Kalibr's wiki recommends a 15–24 hour stationary recording for Allan-deviation noise identification. [L2-S08, "Kalibr IMU Noise Parameters in Practice"]
- REP-145: if a covariance is unknown, all elements should be 0 (unless overridden by a parameter); if a field is not reported, element 0 of its covariance should be −1; drivers should accept `~orientation_stddev`, `~angular_velocity_stddev` and `~linear_acceleration_stddev` parameters that override sensor-reported values. [L2-S28, "Topics" and "Common Parameters"]
- robot_localization: covariances "matter"; do not inflate variances to make the filter ignore a variable (disable it in the configuration instead); a variance of 0 on a fused variable is replaced by a small epsilon (1e−6), and setting covariances properly is preferred. [L2-S29, lines 64, 79, 102–103]
- robot_localization: if two orientation sources exist and one under-reports its covariance, fuse absolute orientation only from the more accurate one and use the other's angular velocity or `_differential` mode. [L2-S29, line 60]

#### Bias models: random constant, random walk, first-order Gauss-Markov
- Three standard stochastic models for slowly varying sensor errors: a random constant (ẋ = 0, no driving noise), a random walk (ẋ = w, integrated white noise whose variance grows as q·t, so it is non-stationary), and a first-order Gauss-Markov process; all can be written as xₖ₊₁ = a·xₖ + wₖ. [L2-S15, PDF p. 85–86 and 90, eqs. 3.25–3.27, Table 3.1]
- A first-order Gauss-Markov process with correlation time T and variance σ² is ẋ = −x/T + w with q = 2σ²/T; in discrete time a = e^(−Δt/T) and qₖ = σ²(1 − e^(−2Δt/T)); as T → ∞ it becomes a random constant, as T → 0 it approaches white noise. [L2-S15, PDF p. 87–89, eqs. 3.28a–c]
- Its main property is bounded uncertainty: during a measurement outage the estimate decays to zero and its uncertainty grows back to the designed σ, which is why it is used for biases and scale factors in INS filters. [L2-S15, PDF p. 87–88]
- Estimating Gauss-Markov parameters from the autocorrelation needs very long records: 200 correlation times of data give only about 10% accuracy, so a 4-hour correlation time would need 800 hours; in practice the parameters are chosen empirically. [L2-S15, PDF p. 89–90]
- Even when a state is truly constant, adding small process noise for long runs is preferable, to keep the state covariance from becoming non-positive-definite. [L2-S15, PDF p. 86–87]
- Example tuning for a low-cost land-vehicle IMU: ARW 3.5 deg/√h, VRW 0.6 m/s/√h, gyro bias Gauss-Markov σ = 100 deg/h with T = 1 hour, gyro scale factor σ = 1000 PPM with T = 4 hour; σ = 100 deg/h was chosen because the actual 200–1000 deg/h gyro biases were first removed by averaging while stationary. [L2-S15, PDF p. 167–168, Table 5.1]
- With these values, the fraction of true errors inside the filter's 1σ/2σ/3σ bounds was 63.5/93.9/99.2% for heading, compared with 68/95/99% expected for a Gaussian; the filter was "conservative" (bounds too large) for all states except heading. [L2-S15, PDF p. 168–169, Table 5.2]

### 4. Attitude / AHRS estimation algorithms
- Accelerometers give inclination (roll and pitch) because, at small acceleration, they measure mainly gravity; they carry no information about rotation around the gravity vector, which is why magnetometers are typically added for heading. [L2-S11, PDF p. 26, section 3.4.3] [L2-S12, PDF p. 2, section 1]
- Without magnetometer data, heading can only be estimated from the gyroscope and drifts like pure gyro dead reckoning, while roll and pitch stay accurate. [L2-S11, PDF p. 48, section 4.5.1, Example 4.2]
- Even with a magnetometer, heading is estimated less accurately than roll and pitch because the magnetometer signal-to-noise ratio is lower and only the horizontal field component carries heading information (small at a large dip angle, e.g. 71° in Linköping). [L2-S11, PDF p. 48, Example 4.1]
- Magnetometers provide heading information everywhere except at the magnetic poles, where the field is vertical. [L2-S11, PDF p. 26, section 3.4.3]
- An AHRS typically assumes the accelerometer measures gravity alone; sustained accelerations (turns, starting and stopping) violate this and corrupt pitch and roll, and real-time velocity measurements allow the sustained acceleration to be estimated and compensated. [L2-S23, "AHRS" page, "Sustained acceleration"]
- Xsens' algorithm assumes that on average the acceleration due to movement is zero; long-lasting accelerations (e.g. an accelerating car) can degrade orientation until the motion again matches the assumption, and GNSS aiding is offered for such applications. [L2-S32, PDF p. 25–26, section 4.4.2]
- Complementary filters combine a low-pass-filtered absolute angle (e.g. from a magnetometer) with high-pass-filtered integrated gyro data; they are closely related to Kalman filters for linear models. [L2-S11, PDF p. 43, section 4.4] Kok et al. describe EKF and complementary-filter implementations as computationally cheaper than optimisation-based smoothing and filtering. [L2-S11, PDF p. 1, abstract]
- The extended Kalman filter, particularly the multiplicative EKF (MEKF), is described as the workhorse of real-time spacecraft attitude estimation; the MEKF estimates a small three-component attitude error while keeping a normalized quaternion as the global, non-singular attitude. [L2-S13, PDF p. 5 and 12]
- Madgwick's filter (designed as a computationally cheaper alternative to Kalman-based orientation filters) integrates the gyro quaternion rate and removes the gyro measurement error along a direction computed by gradient descent from accelerometer and magnetometer data; it has a single tuning parameter β, the gyro measurement error expressed as a quaternion-derivative magnitude. [L2-S42, PDF p. 1 and 4, eq. 30, section E]
- In a hand-rotation test against optical motion capture, Madgwick's filter matched the Kalman-based filter of a commercial sensor: heading RMS error 1.073° static and 1.110° dynamic for the magnetometer version, versus 1.150° and 1.344° for the Kalman-based filter; the accelerometer+gyro-only version gives no heading ("N/A"). β was 0.033 (with magnetometer) and 0.041 (without). [L2-S42, PDF p. 5, Table I]
- Madgwick's magnetic distortion compensation uses the accelerometer's attitude to remove the inclination of the measured field, so that magnetic disturbances affect only the heading component of orientation and no reference field direction has to be predefined. [L2-S42, PDF p. 4, section D, eqs. 31–32]
- Kok et al. assume that Earth rotation (≈7.29·10⁻⁵ rad/s) and Coriolis acceleration are negligible compared with the measurements, and model the gyro as body rate plus bias plus noise. [L2-S11, PDF p. 11, 24 and 30, eqs. 3.45 and 3.70]
- For land-vehicle INS with low-cost gyros, Shin also treats the Earth rotation rate (≈15 °/h) as negligible against gyro biases of 200–1000 deg/h, so stationary gyro output can be used as the initial bias. [L2-S15, PDF p. 167–168]
- The Mars Exploration Rovers, with a Litton LN-200 IMU (3 °/hour maximum drift specified), subtract the planet's rotation from the measured gyro rates before integrating, so that only rotation relative to the surface propagates attitude. [L2-S44, PDF p. 6]
- In Xsens' description, gyro bias on the x and y axes is estimated from gravity; the z-axis (yaw) bias is estimated only with a magnetometer-using profile in a homogeneous field, or when roll/pitch motion exceeds 30° for more than 10 s. [L2-S32, PDF p. 25, section 4.4.1]

### 5. Initial alignment and heading initialisation
- Static alignment has coarse methods (levelling/gyrocompassing, analytic) and a fine stage (EKF with a small-heading-uncertainty model); in-motion coarse alignment uses GPS velocity or an EKF with a large-heading-uncertainty (LHU) model. [L2-S15, PDF p. 69, Table 2.1]
- With low-cost gyroscopes, gyrocompassing and analytical coarse alignment cannot be applied in stationary mode; only roll and pitch can be found from the accelerometers, and heading must come from other sensors (multi-antenna GPS or a magnetic compass). [L2-S15, PDF p. 68]
- For land vehicles, heading can be initialised from GPS velocity as ψ = atan(vE/vN) when the forward axis is parallel to the velocity vector; roll can be initialised as zero within about ±5°. [L2-S15, PDF p. 68, eqs. 2.73a–b]
- Heading from GPS velocity has a ±180° error when the vehicle moves backward; LHU error models are used until the heading error is small (typically a few degrees), then the filter switches to a small-heading-uncertainty model. [L2-S15, PDF p. 77, section 3.1.4]
- Test result (van, low-cost IMU): with 40° initial attitude errors at about 11.5 km/h, a UKF converged in roll/pitch within 10 s and heading within 50 s, while an EKF needed about 200 s; with a 60° initial heading error the UKF again converged faster. [L2-S15, PDF p. 172 and 174]
- Gyrocompassing measures the horizontal component of Earth rate, ωh = 15.041067 °/hr · cos(latitude); at 33.7° N this is 12.5 °/hr and at 71.4° N only 4.8 °/hr. [L2-S19, PDF p. 2, eq. 1]
- A gyro bias error of 1 °/hr gives about 100 mrad (≈5.7°) azimuth uncertainty at 45° latitude; achieving 4 mrad between 60° S and 60° N requires about 0.03 °/hr bias error. [L2-S19, PDF p. 1 and 4]
- Gyrocompassing is limited to high-performance gyros such as FOGs and RLGs, whose size, weight, power and cost are prohibitive for most applications. [L2-S26, "Heading Determination" page, "Gyrocompassing"]
- Nonlinear observability analysis of static alignment: with unknown constant gyro and accelerometer biases, a stationary INS cannot determine its initial attitude (infinitely many attitude/bias combinations fit the data); an estimator then converges to one of these solutions depending on its settings, and practitioners simply assume zero biases, whose standard deviations then limit alignment accuracy. [L2-S43, PDF p. 4–5, Theorem 1 and Remark 2]
- The same analysis shows that alignment with unknown constant biases becomes completely observable if the INS is rotated successively about two different axes, and nearly observable (at most two unobservable states) if it is rotated about a single axis. [L2-S43, PDF p. 1, abstract]
- Xsens orientation output may need time to stabilise after entering measurement mode, mainly to correct small gyro-bias errors; gyro bias changes with temperature and impact. [L2-S32, PDF p. 27, section 4.4.5]

### 6. Yaw observability in GNSS/INS integration
- In dynamic alignment, the INS compares accelerometer-measured acceleration with GNSS-derived position/velocity change to determine heading; most modern systems need only horizontal acceleration of any type (e.g. driving around the block). [L2-S24, "GNSS-aided INS" page, "Dynamic alignment"]
- In static or low-dynamic situations the horizontal acceleration is near zero and heading observability is lost; heading also becomes unobservable during GNSS outages. [L2-S26, "Heading Determination" page, "GNSS/INS" limitations]
- During short low-dynamic periods the INS keeps an accurate but degrading heading, "on the order of 1 min for industrial grade"; most GNSS/INS systems then fall back on a magnetometer. [L2-S24, "GNSS-aided INS" page, "Static or low-dynamic situations"]
- For the MTi-G-710 General profile, Xsens says yaw is referenced by comparing GNSS acceleration with the accelerometers, "so the more movement ... will result in a better yaw". [L2-S36, "Filter Profiles for MTi-G-710"]
- Xsens recommends a minimum velocity of 7 m/s for GNSS/accelerometer yaw estimation, "and the more acceleration and movement the better". [L2-S37, "Minimum Speed to Estimate Yaw in GNSS/INS Devices"]
- For a land-vehicle IMU aided by non-holonomic constraints, forward velocity is unobservable on a straight path without pitching or yawing, which is why wheel speed is added; heading and position are always unobservable, so external information such as GPS is required for navigation over long periods. [L2-S14, PDF p. 2, section I]
- With a low-accuracy IMU and no aiding, heading error can grow quickly because z-gyro uncertainty drives it; Shin notes this can also happen while the vehicle is driven at constant speed, because heading is then poorly observable. [L2-S15, PDF p. 77–78, section 3.1.4]
- The GNSS lever arm (antenna position relative to the IMU) is used by the MTi-680G to correct position and velocity and is described as essential for reliable cm-level position, velocity and orientation. [L2-S31, PDF p. 22, "Lever Arm Correction"] [L2-S37, "Antenna Placement and antenna offset"]

### 7. Standstill, slow and reverse driving; motion constraints and resets
- Zero-velocity updates (ZUPTs) apply a zero-velocity pseudo-measurement whenever a detector decides the sensor is stationary. [L2-S16, PDF p. 2, eq. 1] (pedestrian foot-mounted INS)
- With ZUPTs, all navigation states and sensor biases become observable except position, yaw and the gyro bias along gravity, so position and yaw errors still grow. [L2-S16, PDF p. 3, section III-A] (pedestrian)
- Zero-angular-rate updates (ZARU) or zero-integrated-heading-rate (ZIHR) updates should be applied only when angular rates are much smaller than the gyro bias, and these events should be detected from the temporal variance of the gyro signal (unaffected by bias), not from norm-based zero-velocity detectors. [L2-S16, PDF p. 3, section III-A]
- On a land vehicle, a 2-minute ZUPT let the heading drift over 3°; ZIHR measurements applied together with ZUPTs held heading fixed. [L2-S15, PDF p. 175, section 5.1.3 and Fig. 5.8]
- ZIHR is useful on a wheeled vehicle parked with unchanged heading. [L2-S15, PDF p. 66]
- Non-holonomic constraints for a vehicle on a surface are zero lateral velocity (no side slip) and zero velocity normal to the surface; in practice they are violated by side slip in cornering and by vibration, and can be modelled as noisy (Gaussian white noise) pseudo-measurements. [L2-S14, PDF p. 3–4, section II-B]
- The validity of zero-lateral and zero-vertical velocity assumptions varies with the manoeuvre: lateral velocity is much larger in turns than on straight lines; AI-IMU adapts the pseudo-measurement covariance with a neural network (car, KITTI dataset). [L2-S17, PDF p. 1 and 4]
- Course from DGPS position-derived velocity had pitch and heading errors up to 3° and 6° at 10–55 km/h, and below 1° above that speed range. [L2-S15, PDF p. 171–172]
- GPS-velocity heading has a ±180° error when driving backward. [L2-S15, PDF p. 77]
- Xsens' Automotive profile (MTi-G-710) assumes yaw equals GNSS course over ground; Xsens states this does not hold for vehicles with side slip, including racing cars, tracked vehicles, some articulated vehicles and vehicles on rough terrain. [L2-S36, "Filter Profiles for MTi-G-710"] [L2-S37, "MTi-G-710"]
- Xsens' HighPerformanceEDR profile (MTi-G-710) estimates gyro bias automatically when the MTi is motionless; vibrations and very slow movements may affect the bias estimate. [L2-S36, "Filter Profiles for MTi-G-710"]
- Xsens Manual Gyro Bias Estimation (formerly No Rotation Update) tells the filter the device will not rotate for a set period (default 6 s); if motion is detected the result is rejected; Xsens suggests an autonomous ground vehicle could send the command each time it stops. [L2-S38, "Performing a Manual Gyro Bias Estimation"]
- The MTi-680(G) Continuous Zero Rotation Update, if enabled, automatically starts a gyro-bias estimation whenever the device is motionless. [L2-S33, PDF p. 26]
- When heading error is large, the filter error model must change (LHU model), then switch to the small-error model once heading error is a few degrees. [L2-S15, PDF p. 77]
- Because heading error can grow large again during unaided periods or constant-speed driving, Shin prefers an LHU formulation (Scherzinger's extended misalignment vector, with sin ψz and cos ψz − 1 as states) in which the model switch can be made in both directions. [L2-S15, PDF p. 77–78, eq. 3.12]
- When a ZUPT holds the vehicle's velocity at zero, roll and pitch errors stay controlled but the heading error of a low-cost INS can still grow rapidly because heading is poorly observable; this is the motivation for ZIHR measurements. [L2-S15, PDF p. 101–102]

### 8. Magnetometer heading and calibration
- Hard-iron errors come from constant or slowly varying fields of nearby ferromagnetic material and add a bias; soft-iron errors come from material magnetised by the applied field and vary with vehicle orientation; the full model also includes scale factor, misalignment and wide-band noise. [L2-S09, PDF p. 4, section 2, eq. 2]
- Error-free two-axis measurements lie on a circle; scale-factor and soft-iron errors turn it into an ellipse (soft iron also rotates it), and hard iron shifts its centre; in 3-D the sphere becomes a displaced ellipsoid. [L2-S09, PDF p. 8–11]
- Compass swinging is location-dependent and needs an independent heading reference and a level vehicle; field-domain calibration estimates sensor errors directly and works with a triad. [L2-S09, PDF p. 1 and 8]
- Gebre-Egziabher et al. found that their batch estimator diverged often with only a 10° strip of the ellipsoid and 10 mG noise, and concluded it is not suitable for noisy low-cost magnetometers unless a large portion of the ellipsoid is available; a 360° turn on a level surface suffices for the two-axis case. [L2-S09, PDF p. 20–21]
- Experimentally, calibrated low-cost magnetometers gave heading error with 3.6° standard deviation and 1.2° mean against a navigation-grade INS (less than 3° RMS over a one-minute trace). [L2-S09, PDF p. 23]
- The Vasconcelos method compensates for the combined effect of all linear time-invariant distortions (soft iron, hard iron, non-orthogonality, bias) with a maximum-likelihood estimator, without external attitude references; the readings lie on an ellipsoid, calibration is equivalent to estimating a rotation, scaling and translation, and the sensor alignment is given by the solution of the orthogonal Procrustes problem, separate from calibration. [L2-S10, PDF p. 1–2, abstract and section I]
- Magnetometer-only calibration maps the ellipsoid to a sphere but leaves the rotation between magnetometer and inertial axes unknown; Kok & Schön estimate that misalignment jointly with inertial data; after calibration, heading differences between 90° rotations deviated by a mean of 0.76° (maximum 2.48°). [L2-S12, PDF p. 3 and 15]
- Distortions are either temporal/spatial (from objects moving independently of the sensor), which cannot be fully compensated, or static (from the platform the sensor is mounted on), which can be calibrated. [L2-S34, PDF p. 7–8, section 2.1]
- Strong currents (several amperes), permanent magnets and ferromagnetic materials alter the local field; if a disturbance lasts more than about 10–30 s (profile-dependent), Xsens heading slowly converges to the new, disturbed north. [L2-S32, PDF p. 26 and 34] [L2-S39]
- For a 2-D (planar) calibration the vehicle must turn through at least a full 360° circle, preferably at constant low speed (<15 km/h), in a homogeneous field at least 3 m from large ferromagnetic objects; heading is only accurate within the orientation envelope captured. [L2-S34, PDF p. 11, section 3.2]
- Xsens shows a successful 2-D mapping of a wheeled robot with a metal structure, batteries and motors after driving two full circles, and a drone calibration where residual noise was possibly caused by motors and electronics. [L2-S34, PDF p. 23]
- Xsens advises repeating the calibration every time the sensor is temporarily removed from the object, and if the object's geometry is significantly altered (e.g. components added or removed); the calibration is more accurate for smaller disturbances. [L2-S34, PDF p. 10, section 3.1]
- In-Run Compass Calibration (ICC), when enabled (it is disabled by default), runs continuously in the filter to refine hard- and soft-iron parameters, e.g. when a car later reaches roll/pitch outside the MFM envelope; Xsens still recommends MFM over or in addition to ICC. [L2-S34, PDF p. 32–33, sections 4.4.3–4.4.4]
- Even in ideal magnetic environments a magnetometer gives heading accuracy of 1° to 2° over extended periods; Earth's field can shift by up to 2° from one day to the next. [L2-S26, "Heading Determination" page, "Magnetometer" limitations]
- Current-induced interference (ArduPilot, mainly multicopters): the field from a current loop grows with the enclosed loop area and with current and falls off with the cube of distance far from the loop; recommended hardware fixes are an external compass on a mast away from power wiring, short and twisted DC power wires, and higher voltage/lower current for the same power. [L2-S47, `common-magnetic-interference.rst` lines 27–118]
- ArduPilot's CompassMot (Copter) compensates motor/power-wire interference using a battery current monitor, "because the magnetic interference is linear with current drawn"; interference below 30% is acceptable, 31–60% is a "grey zone", and above 60% the compass should be moved or replaced by an external one. [L2-S47, `common-compass-setup-advanced.rst` lines 106–148]
- ArduPilot lists steel-framed buildings, reinforced concrete, iron pipes and culverts, high-power electric lines, vehicles, electric motors and computers as external sources that distort the local field, and notes this list has not been objectively tested for its effect on accuracy. [L2-S47, `common-magnetic-interference.rst` lines 120–134]
- Static-dynamic-static test (vehicle, uncalibrated magnetometer): the gyro tracks a turn correctly, but after the vehicle stops the AHRS heading drifts to the new, still-wrong magnetic heading. [L2-S23, "AHRS" page, "Drift in the drift-free solution"]

### 9. GNSS-derived heading: single and dual antenna
- A GNSS compass uses moving-baseline RTK between two rigidly mounted antennas; heading comes from differencing the two receivers at one instant and does not need motion. [L2-S25, "GNSS Compass/INS" page]
- Heading error is inversely proportional to the antenna baseline L (θerr = Perr/L), since the relative-position error stays nearly constant; longer baselines increase the time needed to lock on to heading. [L2-S25, "GNSS Compass/INS" page, eq. 1]
- A GNSS compass needs both antennas with a clear sky view and at least six common satellites, and is more sensitive to multipath than a single-antenna system; ground planes under the antennas reduce ground reflections. [L2-S25, "GNSS Compass/INS" page, "Challenges"]
- u-blox moving-base: two antennas on the vehicle x-axis give heading and roll; three antennas give full attitude; typical baselines are 1–3 m for automotive heading and 20–30 cm for drones. [L2-S21, PDF p. 4 and 8]
- The ZED-F9H datasheet gives 0.4 deg moving-base heading accuracy, convergence under 10 s (multi-constellation) and 0.3 deg dynamic heading accuracy (50% at 30 m/s); identical, identically oriented antennas are recommended. [L2-S22, PDF p. 4–5, Tables 1 and 4]
- A mismatch between the RTK-computed antenna distance and the known baseline indicates wrongly fixed ambiguities and a wrong heading. [L2-S21, PDF p. 20]
- Car test with a two-antenna GPS (laterally placed antennas): 10 Hz velocity with noise below 3 cm/s and 5 Hz attitude with noise below 0.2 deg (1σ); a Kalman filter fused the GPS yaw with a yaw-rate gyro (0.2 deg/s noise) to estimate heading and gyro bias at 100 Hz, removing the drift of plain gyro integration. [L2-S45, PDF p. 1 and 3–4, sections 4–5]
- In the same work, GPS measurements had to be time-aligned with INS data using the receiver's time tags and synchronisation pulse, because the receiver adds half a sample period of latency and any time offset "may result in significant estimation errors". [L2-S45, PDF p. 3–4]
- Single-antenna heading from velocity: see section 7 (±180° in reverse; errors up to 6° at 10–55 km/h). [L2-S15, PDF p. 77 and 171–172]

### 10. Other heading sources and how other vehicle types do it
- MEMS gyrocompassing (lab): a quadruple-mass MEMS gyro with 0.2 °/hr bias instability reached 4 mrad azimuth uncertainty with continuous rotation ("carouseling"); ±180° turning ("maytagging") gave similar uncertainty but needs temperature calibration. [L2-S19, PDF p. 1, abstract]
- A MEMS north-finder with electronically rotated modes ("virtual maytagging") reached 0.204° azimuth accuracy in 5 min at 28.2° latitude, with 0.0078 °/h bias instability over one day. [L2-S20, PDF p. 1, abstract]
- Monocular visual-inertial navigation has four unobservable directions: global yaw and global position. [L2-S18, PDF p. 7]
- Dissanayake et al. state that non-holonomic constraints plus wheel speed guarantee observability of the velocity and "the attitude of the inertial unit" (abstract), while their introduction states that heading and position are always unobservable without external information such as GPS. [L2-S14, PDF p. 1–2]
- Dual-antenna GNSS heading is used for farming vehicles, heavy machinery, ships and cars. [L2-S21, PDF p. 5–7]

#### Planetary rovers: sun sensing (Mars Exploration Rovers)
- The Mars Exploration Rovers acquire attitude by imaging the sun with an articulated camera (16 degree field of view): "Sunfind" gets heading from the sun direction plus the current tilt, and "Sungaze" watches the sun move and solves full attitude with the QUEST algorithm. [L2-S44, PDF p. 1 and 3–4]
- Sun-based heading fails near local noon, when the sun is close to the zenith. [L2-S44, PDF p. 4]
- Between sun updates, attitude is propagated by gyro integration while driving, and in "Articulate" mode (e.g. arm use) accelerometer averages refresh tilt; yaw is changed only by the gyro integration. [L2-S44, PDF p. 5–6]
- The IMU is switched off when the rover is stationary, which saves power and also avoids gyro-drift errors while the rover is known to be still. [L2-S44, PDF p. 1]
- Requirement: attitude error no more than 1.5 degrees (3σ), derived from pointing the high-gain antenna within 2 degrees (3σ). [L2-S44, PDF p. 1]
- In operation, the rovers needed to reacquire attitude after about 10,000 seconds of accumulated IMU integration time (roughly every 20 sols), judged by high-gain-antenna signal strength. [L2-S44, PDF p. 7–8]
- Position was propagated with wheel odometry plus gyro heading; visual odometry was added where wheel slip was high (sometimes 100% or more in Gusev crater). [L2-S44, PDF p. 6–7]
- Lesson noted by the team: a gyrocompassing capability (finding the planet's rotation vector with the rover stationary) would have been "comforting", although it was not needed. [L2-S44, PDF p. 8]

### 11. Failure modes, integrity and testing of heading
- Magnetic heading failure: a sustained disturbance makes heading converge to a wrong magnetic north. [L2-S32, PDF p. 34]
- Magnetization of the sensor by strong fields (magnets, speakers, motors) makes its magnetometer calibration useless, typically seen as a large heading deviation; mild magnetization can be re-calibrated. [L2-S34, PDF p. 7]
- Xsens states that a failed Manual Gyro Bias Estimation (rejected because of movement) causes no additional error, except when the device rotates at a very constant angular velocity for the whole update period. [L2-S38, "Best practices"]
- Test: static-dynamic-static drive to reveal heading drift after a manoeuvre. [L2-S23, "AHRS" page]
- Test: magnetometer calibration quality is judged by the norm of the calibrated field (mean close to 1, low standard deviation and maximum error) and Gaussian residuals. [L2-S34, PDF p. 16 and 18]
- Test: rotating a calibrated unit through 24 orientations 90° apart and checking heading changes against 90°. [L2-S12, PDF p. 13–15]
- Test: comparison with a navigation-grade INS as heading truth. [L2-S09, PDF p. 23]
- Filter consistency test: the percentage of true errors (against a reference) inside the filter's 1σ, 2σ and 3σ bounds is compared with the Gaussian 68/95/99%; more than 68% inside 1σ means a "conservative" filter. In Shin's land-vehicle test only heading fell below the Gaussian values at 1σ and 2σ (63.5% and 93.9%; 99.2% at 3σ). [L2-S15, PDF p. 168–169, Table 5.2]
- Operational check without ground truth: the Mars rovers judged heading/attitude degradation from high-gain-antenna signal strength and set a re-acquisition interval of about 10,000 s of IMU integration. [L2-S44, PDF p. 7–8]
- Wrong-convergence failure: in static alignment with unknown biases, an estimator converges to one of infinitely many consistent solutions, chosen by its initial settings rather than by the data. [L2-S43, PDF p. 5, Remark 2]
- Allan-variance data collection: 12 h at 100 Hz stationary (Woodman), 15–24 h stationary (Kalibr); gyro biases change during warm-up, so Xsens advises at least 5 min, preferably 10 min, of warm-up before bias estimation. [L2-S06, PDF p. 19] [L2-S08] [L2-S38, "Best practices"]

### 12. Product-specific: Xsens MTi-600 series and ROS 2 driver
- MTi-600 gyro: in-run bias stability 8 °/h, noise density 0.007 °/s/√Hz, bandwidth 520 Hz, g-sensitivity 0.001 °/s/g, scale-factor variation 0.5% typical (1.5% over life); accelerometer noise density 60 µg/√Hz. [L2-S31, PDF p. 13, Tables 6–7]
- MTi-680G orientation specification: roll/pitch 0.2° static and 0.5° dynamic, yaw 1° dynamic (RMS, typical scenarios). [L2-S31, PDF p. 12, Table 3]
- MTi-670/680(G) filter profiles: General/General_RTK (GNSS + barometer), GeneralNoBaro(_RTK) (GNSS only) and GeneralMag(_RTK) (GNSS + barometer + magnetometer). [L2-S31, PDF p. 24, Table 18] [L2-S33, PDF p. 27]
- For GNSS/INS devices the magnetometer is used only in GeneralMag; the other profiles are completely independent of the magnetic field; Xsens gives comparing accelerometer data with GNSS acceleration as an example of how heading can be estimated.  [L2-S32, PDF p. 26, section 4.4.3]
- Only GeneralMag gives north-referenced yaw at power-up; in the other profiles yaw initialises at 0° and converges to north once there is a GNSS fix and the MTi moves at sufficient velocity ("min. 7 m/s with standard GNSS, lower with RTK enabled"); if heading is approximately known, SetInitialHeading is "highly recommended". [L2-S33, PDF p. 27] [L2-S35, PDF p. 27–28] [L2-S37, "Initial Heading"]
- Xsens Active Heading Stabilization (AHS) is not tuned for or intended for GNSS/INS devices, and Xsens discourages its use there. [L2-S32, PDF p. 28, Table 11]
- If a GNSS outage lasts more than 45 s, the MTi-670/680G stops outputting position and velocity until GNSS is acceptable again. [L2-S31, PDF p. 22]
- Xsens internal bias estimates are not written to the sensor memory and are not removed from the published rate-of-turn output; users of raw gyro data must remove biases themselves. [L2-S38, "Best practices"]
- Magnetic calibration for automotive use: Xsens recommends running the Magnetic Field Mapper while driving at least 3 circles. [L2-S37, "Magnetic Calibration"]
- ROS 2 driver: filter profile index for MTi-680(G) is 0 = General_RTK, 1 = GeneralNoBaro_RTK, 2 = GeneralMag_RTK, set only when `enable_filter_config` is true; periodic Manual Gyro Bias Estimation is off by default (`enable_manual_gyro_bias: false`, `manual_gyro_bias_param: [15, 5]`). [L2-S40, `xsens_mti_node.yaml` lines 93–109, 244–255]
- ROS 2 driver covariance: the `*_stddev` parameters default to 0, and for non-Sirius/Avior devices the orientation covariance diagonal is filled from `orientation_stddev` (squared) rather than from a sensor-reported uncertainty; unavailable fields get −1 in element 0 (REP-145 convention). [L2-S40, `imupublisher.h` lines 50–69 and 118–176] [L2-S40, `xsens_mti_node.yaml` lines 268–276]
- The MTi can rotate its output inside the device: RotSensor rotates the sensor frame S into an object frame O, and RotLocal rotates the earth-fixed frame L into L′; an "inclination reset" makes roll and pitch 0°, a "heading reset" ("bore sighting") makes yaw 0° while keeping z vertical, and an "alignment reset" does both. [L2-S41, "Answer", list of five methods]
- The housing of the MTi 600-series is aligned with the output frame during factory calibration, with non-orthogonality between the sensor axes "<0.05" (the unit symbol does not render in the PDF text). [L2-S32, PDF p. 18]
- Xsens does not recommend performing an alignment (orientation) reset during the filter's initialisation phase: the new "north" is only meaningful once the estimated heading has stabilised, and Xsens recommends waiting at least 5 minutes. [L2-S41, "Important remarks"]
- An Xsens orientation reset is volatile unless a "Store" action writes the new RotSensor/RotLocal to non-volatile memory; only then is it used at the next power-up. [L2-S41, "Orientation resets"]

## Recommended practice
1. Ensure IMU orientation is published in ENU with yaw zero at east and counter-clockwise positive, or set `yaw_offset`/declination accordingly. [L2-S27, lines 170–175] [L2-S29, line 38] [L2-S30, lines 19–25]
2. Characterise gyro noise with a long stationary, thermally stable Allan-variance record and read ARW at τ = 1 s and rate random walk at τ = 3 s. [L2-S07, PDF p. 105–109] [L2-S08]
3. Inflate noise parameters derived from static tests before using them in a filter (10× or more for lowest-cost sensors). [L2-S08]
4. Report real covariances; do not use huge variances to ignore data, and avoid zero variances on fused variables. [L2-S29, lines 64, 79, 102–103]
5. Warm the IMU up (≥5 min, preferably ≥10 min) before gyro-bias estimation, and apply no-rotation/ZIHR updates during standstill. [L2-S38] [L2-S15, PDF p. 175]
6. Detect zero-rate periods from gyro variance, not from signal norm. [L2-S16, PDF p. 3]
7. For single-antenna GNSS/INS, provide horizontal acceleration (manoeuvres, speed changes) for heading convergence, and supply an initial heading when known. [L2-S24, "Dynamic alignment"] [L2-S33, PDF p. 27]
8. If a magnetometer is used, calibrate it mounted on the vehicle in a homogeneous field, turning through full circles, and repeat after remounting or a significant geometry change. [L2-S34, PDF p. 10–11] [L2-S37]
9. For dual-antenna heading, use identical, identically oriented antennas with ground planes and validate fixes against the known baseline length. [L2-S22, PDF p. 5] [L2-S25, "Challenges"] [L2-S21, PDF p. 20]
10. Express the IMU mounting as a static transform from the robot base frame to the IMU `frame_id` rather than altering data in the driver. [L2-S28, lines 41–56] [L2-S29, line 40]
11. Perform alignment resets only after the heading estimate has stabilised (Xsens: at least 5 minutes), and store them if they must survive power cycles. [L2-S41]
12. Time-align GNSS measurements with IMU data using receiver time tags or a synchronisation pulse. [L2-S45, PDF p. 3–4]
13. Keep a magnetometer away from high-current DC wiring, keep power wiring short and twisted, and, where interference scales with current, compensate using a measured current. [L2-S47, `common-magnetic-interference.rst` lines 27–118; `common-compass-setup-advanced.rst` lines 106–116]

## Key numbers
| Quantity | Value | Conditions | Source |
|---|---|---|---|
| Earth rotation rate | 15.041067 °/hr | horizontal part = × cos(latitude) | L2-S19, PDF p. 2 |
| Gyro bias for 4 mrad gyrocompass azimuth | 0.03 °/hr | latitudes 60° S–60° N | L2-S19, PDF p. 4 |
| Azimuth error from 1 °/hr gyro bias | ≈100 mrad | gyrocompassing at 45° latitude | L2-S19, PDF p. 1 |
| MTi-600 gyro in-run bias stability | 8 °/h | datasheet | L2-S31, PDF p. 13 |
| MTi-600 gyro noise density | 0.007 °/s/√Hz | datasheet | L2-S31, PDF p. 13 |
| MTi-680G yaw accuracy | 1° | dynamic, RMS, typical | L2-S31, PDF p. 12 |
| Xsens minimum speed for GNSS yaw | 7 m/s | standard GNSS; lower with RTK | L2-S37; L2-S35, PDF p. 27 |
| Xsens GNSS outage before PV output stops | 45 s | MTi-670/680G | L2-S31, PDF p. 22 |
| Xsens magnetic disturbance before heading re-converges | >10–30 s | filter-profile dependent | L2-S32, PDF p. 26 |
| Low-cost MEMS (Xsens Mtx) gyro bias instability / ARW | 32–43 °/h / 4.6–4.8 °/√h | 12 h static log | L2-S06, PDF p. 19 |
| Heading error, calibrated low-cost magnetometer | σ 3.6°, mean 1.2° | vs navigation-grade INS | L2-S09, PDF p. 23 |
| Magnetometer heading, ideal environment | 1–2° | extended periods | L2-S26, "Magnetometer" limitations |
| ZED-F9H moving-base heading accuracy | 0.4 deg | datasheet Table 4 | L2-S22, PDF p. 5 |
| Heading drift during 2-min ZUPT without ZIHR | >3° | low-cost INS, van | L2-S15, PDF p. 175 |
| GPS-velocity heading error | up to 6° | 10–55 km/h, DGPS | L2-S15, PDF p. 172 |
| Allan-deviation estimate error | ≈40% / ≈5% | 20,000 points; 5,000- / 100-point clusters | L2-S07, PDF p. 116 |
| Typical MEMS temperature sensitivity | ~500 (°/hr)/°C | uncompensated | L2-S19, PDF p. 3 |
| WMM2025 declination uncertainty (1σ) | √(0.26² + (5417/H)²)° | H = horizontal field in nT | L2-S46, line 265 |
| Local declination anomalies | 3–4° not uncommon; some >10° | not modelled by WMM | L2-S46, line 95 |
| Compass blackout / caution zone | H < 2000 nT / 2000–6000 nT | near magnetic poles | L2-S46, lines 227–228 |
| Gauss-Markov bias tuning example | σ = 100 deg/h, T = 1 h (gyro bias); ARW 3.5 deg/√h | low-cost land-vehicle IMU after stationary bias averaging | L2-S15, PDF p. 168 |
| Data needed for Gauss-Markov autocorrelation | 200 × T for ~10% accuracy | e.g. 800 h for T = 4 h | L2-S15, PDF p. 89–90 |
| Madgwick filter heading RMS error | 1.073° static / 1.110° dynamic | MARG version, hand rotations vs optical reference | L2-S42, PDF p. 5 |
| Madgwick β | 0.033 (MARG) / 0.041 (IMU) | values found optimal in that test | L2-S42, PDF p. 5 |
| Two-antenna GPS attitude noise | <0.2 deg (1σ) at 5 Hz | car test, NovAtel receivers | L2-S45, PDF p. 3 |
| Xsens wait before alignment reset | ≥5 min | heading must have stabilised | L2-S41 |
| MER attitude requirement | ≤1.5° (3σ) | from 2° (3σ) antenna pointing | L2-S44, PDF p. 1 |
| MER attitude re-acquisition interval | ~10,000 s of IMU integration | LN-200 IMU, ~every 20 sols | L2-S44, PDF p. 8 |
| CompassMot interference thresholds | <30% ok; 31–60% grey zone; >60% relocate | ArduPilot Copter | L2-S47, compass setup lines 140–146 |

## How it is tested
| Test | What it measures | Pass criterion used in the source | Source |
|---|---|---|---|
| Stationary Allan-variance record | ARW, bias instability, rate random walk | slopes −1/2, 0, +1/2 identified; ≥9 bins per averaging time | L2-S06, PDF p. 18–19; L2-S07 |
| Static-dynamic-static drive | heading drift after a turn | no criterion given; the source shows the failure case (after the stop, heading drifts to the still-wrong magnetic heading) | L2-S23, "AHRS" page |
| MFM report | magnetometer calibration quality | norm ≈1, small std/max error, Gaussian residuals | L2-S34, PDF p. 16 and 18 |
| 90° rotation sequence (24 orientations) | calibrated heading accuracy | deviation from 90° (mean 0.76° after ML) | L2-S12, PDF p. 15 |
| Comparison with navigation-grade INS | absolute heading error | residual statistics (σ 3.6°, mean 1.2°) | L2-S09, PDF p. 23 |
| Manual Gyro Bias Estimation status | whether bias update succeeded | status returns 0 (success) vs stuck at 2/3 (failed) | L2-S38 |
| Baseline length check | dual-antenna fix validity | computed distance matches known baseline | L2-S21, PDF p. 20 |
| Error-envelope (consistency) test | whether filter σ-bounds match true errors | fractions inside 1σ/2σ/3σ compared with 68/95/99%; more than 68% inside 1σ = "conservative" | L2-S15, PDF p. 168–169 |
| Optical motion-capture reference | static/dynamic Euler-angle RMS error | RMS vs optical system; static if rate < 5°/s | L2-S42, PDF p. 5 |
| CompassMot run | compass interference vs current | % interference < 30% | L2-S47, compass setup lines 106–148 |

## Common mistakes
- Treating GNSS course over ground as vehicle heading on vehicles with side slip (tracked, articulated, rough terrain), or during reverse driving, where it is off by 180°. [L2-S36] [L2-S15, PDF p. 77]
- Mixing NED/north-zero IMU data with ENU/east-zero consumers without a yaw offset. [L2-S29, line 38] [L2-S30, lines 23–25]
- Using static-test noise values directly, making the filter over-trust the IMU. [L2-S08]
- Inflating covariances to "switch off" a variable, or leaving zero covariances. [L2-S29, lines 64, 102–103]
- Calibrating a magnetometer only in the plane, then operating outside that orientation envelope. [L2-S34, PDF p. 11] [L2-S34, PDF p. 32–33]
- Estimating gyro bias before warm-up, or while the device is rotating slowly at constant rate. [L2-S38]
- Relying on ZUPT alone at standstill, which leaves yaw unobservable. [L2-S16, PDF p. 3] [L2-S15, PDF p. 175]
- Performing an Xsens alignment reset before the heading estimate has stabilised, or not storing it, so it is lost at power-up. [L2-S41]
- Estimating Gauss-Markov bias parameters from short records; the autocorrelation estimate is unreliable unless the record is hundreds of correlation times long. [L2-S15, PDF p. 89–90]
- Routing high-current DC wiring in large loops near a magnetometer; the disturbance grows with loop area and current. [L2-S47, `common-magnetic-interference.rst` lines 83–118]
- Fusing GPS data with INS data without correcting the receiver's time offset. [L2-S45, PDF p. 3–4]
- Assuming the WMM declination is exact at a site; local anomalies of several degrees occur. [L2-S46, line 95]

## Disagreements between sources
- Required motion for GNSS/INS heading: VectorNav says most modern systems need only horizontal acceleration of any type, and that "most smaller vehicles simply need to get up to a decent speed", where "the small fluctuations of a car at highway speed" are enough [L2-S24, "Dynamic alignment"], while Xsens recommends a minimum of 7 m/s with standard GNSS for its devices [L2-S37] [L2-S35, PDF p. 27]. (Inference: these are statements about different products, not a direct contradiction.)
- Observability of alignment under rotation: Wu et al. show that global (nonlinear) and linearization-based analyses give opposite verdicts for rotation about a single axis — vertical and north-south rotation are unobservable globally but observable in linearized analyses, east-west the reverse — and say that specific earlier linearization-based claims (its references 7, 24 and 25) are "theoretically incorrect". [L2-S43, PDF p. 8, Table I]
- Meaning of "soft iron": Gebre-Egziabher et al. use it for vehicle material magnetised by the externally applied field, modelled as a matrix of constant soft-iron coefficients in the calibration [L2-S09, PDF p. 4–6], while Madgwick et al. use it for interference fixed in the earth frame, removable only with another orientation reference [L2-S42, PDF p. 4].
- Earth rotation in gyro models: Kok et al. and Shin treat it as negligible for low-cost sensors [L2-S11, PDF p. 24 and 30] [L2-S15, PDF p. 167–168], whereas gyrocompassing uses it as the heading signal [L2-S19, PDF p. 2] and the Mars rovers' LN-200 attitude propagation subtracts planetary rotation [L2-S44, PDF p. 6]. (Inference: the difference follows from gyro grade relative to Earth rate; Shin compares Earth rate with gyro bias explicitly [L2-S15, PDF p. 167–168].)
- Magnetometer heading accuracy: Gebre-Egziabher et al. report 1–2° in the abstract but 3.6° σ in their experiment [L2-S09, PDF p. 1 and 23]; VectorNav gives 1–2° as the best case over extended periods [L2-S26, "Magnetometer" limitations].

## Open questions
- How fast MTi-680G yaw drifts at standstill or at low speed without magnetometer (no Xsens figure found in the sources read).
- How the MTi GNSS/INS profiles behave during reverse driving or on skid-steer/tracked vehicles (Xsens only states that the MTi-G-710 Automotive profile excludes tracked vehicles).
- Quantitative guidance on reset/re-initialisation of heading filters after divergence (beyond LHU/SHU switching).
- Innovation/Mahalanobis gating and cross-checks between heading sources (belongs mainly to L4; not covered by the sources read here).
- Current-induced magnetometer errors (motor/battery currents) quantified for ground robots — only drone-oriented project documentation (L2-S47) was found; Silic & Rogers 2019 was not obtained.
- Formal GNSS/INS yaw-observability analyses for land vehicles (Rhee et al., IEEE TAES 2004; Tang et al., IEEE TVT 2009; Jiang et al., Sensors 2016) could not be downloaded (ION/IEEE paywall; MDPI and PMC blocked automated download), so the statement that heading needs horizontal acceleration rests on manufacturer documentation (L2-S24, L2-S26, L2-S37) and Shin's thesis (L2-S15).
- Mahony et al., "Nonlinear Complementary Filters on the Special Orthogonal Group," IEEE TAC 2008 (the standard complementary attitude filter) could not be downloaded (HAL copy behind a bot-check); invariant EKF methods for heading were not covered.
- An open, peer-reviewed statement of how single-antenna course-over-ground error scales with speed (e.g. σψ ≈ σv/v) was not found; only the measured values in L2-S15 are available.
- How angle wrap-around (±180°) of yaw innovations is handled in fusion filters was not covered by the sources read.
- Groves, Titterton & Weston, Farrell, IEEE Std 952 and El-Sheimy et al. 2008 could not be read (no open copy).

## Sources
| ID | Citation | Link | Accessed | File | Level |
|---|---|---|---|---|---|
| L2-S01 | P. D. Groves, *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*, 2nd ed., Artech House, 2013. | https://us.artechhouse.com/Principles-of-GNSS-Inertial-and-Multisensor-Integrated-Navigation-Systems-Second-Edition-P1636.aspx | 2026-09-27 | not downloaded | A |
| L2-S02 | D. H. Titterton, J. L. Weston, *Strapdown Inertial Navigation Technology*, 2nd ed., IET, 2004. doi:10.1049/PBRA017E | https://doi.org/10.1049/PBRA017E | 2026-09-27 | not downloaded | A |
| L2-S03 | J. A. Farrell, *Aided Navigation: GPS with High Rate Sensors*, McGraw-Hill, 2008. | https://intra.ece.ucr.edu/~farrell/?page=content/PubSupp.html | 2026-09-27 | not downloaded | A |
| L2-S04 | IEEE Std 952-1997 (R2008), *IEEE Standard Specification Format Guide and Test Procedure for Single-Axis Interferometric Fiber Optic Gyros*. | https://ieeexplore.ieee.org/document/660628 | 2026-09-27 | not downloaded | A |
| L2-S05 | N. El-Sheimy, H. Hou, X. Niu, "Analysis and Modeling of Inertial Sensors Using Allan Variance," IEEE Trans. Instrum. Meas. 57(1), 140–149, 2008. doi:10.1109/TIM.2007.908635 | https://doi.org/10.1109/TIM.2007.908635 | 2026-09-27 | not downloaded | A |
| L2-S06 | O. J. Woodman, "An introduction to inertial navigation," Univ. of Cambridge Computer Laboratory, Tech. Rep. UCAM-CL-TR-696, 2007. | https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf | 2026-09-27 | sources/woodman_2007_intro_inertial_navigation.pdf | B (university technical report, not peer-reviewed) |
| L2-S07 | H. Hou, *Modeling Inertial Sensors Errors Using Allan Variance*, MSc thesis, UCGE Report 20201, Univ. of Calgary, 2004. | https://www.ucalgary.ca/engo_webdocs/NES/04.20201.HaiyingHou.pdf | 2026-09-27 | sources/hou_2004_allan_variance_thesis.pdf | A (examined thesis) |
| L2-S08 | ETH Zurich ASL, Kalibr wiki "IMU Noise Model", wiki commit 73a2ba7. | https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model | 2026-09-27 | sources/ethzasl_kalibr_wiki_imu_noise_model.md | B |
| L2-S09 | D. Gebre-Egziabher, G. H. Elkaim, J. D. Powell, B. W. Parkinson, "Calibration of Strapdown Magnetometers in Magnetic Field Domain," ASCE J. Aerospace Engineering 19(2), 87–102, 2006 (author copy). | https://users.soe.ucsc.edu/~elkaim/Documents/magcal.pdf | 2026-09-27 | sources/gebreegziabher_2006_magnetometer_calibration.pdf | A |
| L2-S10 | J. F. Vasconcelos, G. Elkaim, C. Silvestre, P. Oliveira, B. Cardeira, "Geometric Approach to Strapdown Magnetometer Calibration in Sensor Frame," IEEE Trans. Aerosp. Electron. Syst. 47(2), 1293–1306, 2011 (author preprint). | https://users.soe.ucsc.edu/~elkaim/Documents/ReimmanTAES08.pdf | 2026-09-27 | sources/vasconcelos_2011_geometric_magnetometer_calibration.pdf | A |
| L2-S11 | M. Kok, J. D. Hol, T. B. Schön, "Using Inertial Sensors for Position and Orientation Estimation," Foundations and Trends in Signal Processing 11(1–2), 1–153, 2017. doi:10.1561/2000000094 (arXiv 1704.06053v2) | https://arxiv.org/abs/1704.06053 | 2026-09-27 | sources/kok_2017_inertial_position_orientation.pdf | A |
| L2-S12 | M. Kok, T. B. Schön, "Magnetometer Calibration Using Inertial Sensors," IEEE Sensors J. 16(14), 5679–5689, 2016. doi:10.1109/JSEN.2016.2569160 (open copy: arXiv 1601.05257v3) | https://arxiv.org/abs/1601.05257 | 2026-09-27 | sources/kok_2016_magnetometer_calibration_inertial.pdf | A |
| L2-S13 | J. L. Crassidis, F. L. Markley, Y. Cheng, "Survey of Nonlinear Attitude Estimation Methods," J. Guidance, Control, and Dynamics 30(1), 12–28, 2007 (author copy). | https://ancs.eng.buffalo.edu/pdf/ancs_papers/2007/att_survey07.pdf | 2026-09-27 | sources/crassidis_2007_nonlinear_attitude_survey.pdf | A |
| L2-S14 | G. Dissanayake, S. Sukkarieh, E. Nebot, H. Durrant-Whyte, "The aiding of a low-cost strapdown inertial measurement unit using vehicle model constraints for land vehicle applications," IEEE Trans. Robotics and Automation 17(5), 731–747, 2001. | http://www-personal.acfr.usyd.edu.au/nebot/publications/gps_ins_constraints.pdf | 2026-09-27 | sources/dissanayake_2001_vehicle_model_constraints.pdf | A |
| L2-S15 | E.-H. Shin, *Estimation Techniques for Low-Cost Inertial Navigation*, PhD thesis, UCGE Report 20219, Univ. of Calgary, 2005. | https://www.ucalgary.ca/engo_webdocs/NES/05.20219.EHShin.pdf | 2026-09-27 | sources/shin_2005_lowcost_ins_thesis.pdf | A (examined thesis) |
| L2-S16 | J. Wahlström, I. Skog, "Fifteen Years of Progress at Zero Velocity: A Review," IEEE Sensors J. 21(2), 1139–1151, 2021 (arXiv 2008.09208v1). | https://arxiv.org/abs/2008.09208 | 2026-09-27 | sources/wahlstrom_2021_zero_velocity_review.pdf | A |
| L2-S17 | M. Brossard, A. Barrau, S. Bonnabel, "AI-IMU Dead-Reckoning," IEEE Trans. Intelligent Vehicles 5(4), 585–595, 2020 (open copy: arXiv 1904.06064v1). | https://arxiv.org/abs/1904.06064 | 2026-09-27 | sources/brossard_2020_ai_imu_dead_reckoning.pdf | A |
| L2-S18 | G. Huang, "Visual-Inertial Navigation: A Concise Review," IEEE ICRA 2019, 9572–9582 (arXiv 1906.02650v1). | https://arxiv.org/abs/1906.02650 | 2026-09-27 | sources/huang_2019_vins_concise_review.pdf | A |
| L2-S19 | I. P. Prikhodko, S. A. Zotov, A. A. Trusov, A. M. Shkel, "What is MEMS Gyrocompassing? Comparative Analysis of Maytagging and Carouseling," J. Microelectromechanical Systems 22(6), 1257–1266, 2013. doi:10.1109/JMEMS.2013.2282936 | https://escholarship.org/uc/item/7m76w201 | 2026-09-27 | sources/prikhodko_2013_mems_gyrocompassing.pdf | A |
| L2-S20 | T. Miao et al., "Removal of the rate table: MEMS gyrocompass with virtual maytagging," Microsystems & Nanoengineering 9, 138, 2023. doi:10.1038/s41378-023-00610-3 | https://www.nature.com/articles/s41378-023-00610-3 | 2026-09-27 | sources/miao_2023_mems_gyrocompass_virtual_maytagging.pdf | A |
| L2-S21 | u-blox, "ZED-F9P Moving base applications," Application note UBX-19009093 R03, 14 Sep 2023. | https://content.u-blox.com/sites/default/files/ZED-F9P-MovingBase_AppNote_UBX-19009093.pdf | 2026-09-27 | sources/ublox_2023_zedf9p_moving_base_appnote.pdf | B |
| L2-S22 | u-blox, "ZED-F9H-01B Data sheet," UBX-21025012 R05, 21 Mar 2024. | https://www.u-blox.com/en/product/zed-f9h-module | 2026-09-27 | sources/ublox_2024_zedf9h_datasheet.pdf | A |
| L2-S23 | VectorNav, *Inertial Navigation Primer*, section 1.6 "Attitude & Heading Reference System (AHRS)" (web page, undated). | https://www.vectornav.com/resources/inertial-navigation-primer/theory-of-operation/theory-ahrs | 2026-09-27 | sources/vectornav_2026_primer_ahrs.md | B |
| L2-S24 | VectorNav, *Inertial Navigation Primer*, section 1.7 "GNSS-Aided Inertial Navigation System (GNSS/INS)" (web page, undated). | https://www.vectornav.com/resources/inertial-navigation-primer/theory-of-operation/theory-gpsins | 2026-09-27 | sources/vectornav_2026_primer_gnss_ins.md | B |
| L2-S25 | VectorNav, *Inertial Navigation Primer*, section 1.8 "GNSS Compass/INS (Dual GNSS/INS)" (web page, undated). | https://www.vectornav.com/resources/inertial-navigation-primer/theory-of-operation/theory-gnsscompass | 2026-09-27 | sources/vectornav_2026_primer_gnss_compass.md | B |
| L2-S26 | VectorNav, *Inertial Navigation Primer*, section 1.9 "Heading Determination" (web page, undated). | https://www.vectornav.com/resources/inertial-navigation-primer/theory-of-operation/theory-heading | 2026-09-27 | sources/vectornav_2026_primer_heading_determination.md | B |
| L2-S27 | T. Foote, M. Purvis, REP-103 "Standard Units of Measure and Coordinate Conventions," ros-infrastructure/rep commit 11ca24a. | https://www.ros.org/reps/rep-0103.html | 2026-09-27 | sources/ros_rep103_units_coordinates.rst | A |
| L2-S28 | P. Bovbel, REP-145 "Conventions for IMU Sensor Drivers" (Draft), ros-infrastructure/rep commit 11ca24a. | https://reps.openrobotics.org/rep-0145/ | 2026-09-27 | sources/ros_rep145_imu_driver_conventions.rst | B (draft REP, not adopted) |
| L2-S29 | robot_localization documentation, "Preparing Your Data for Use with robot_localization," commit 8696ee5. | https://github.com/cra-ros-pkg/robot_localization/blob/8696ee5a9e4f959fcaae37835dcf2ed12ead581b/doc/preparing_sensor_data.rst | 2026-09-27 | sources/robotlocalization_preparing_sensor_data.rst | B |
| L2-S30 | robot_localization documentation, "navsat_transform_node," commit 8696ee5. | https://github.com/cra-ros-pkg/robot_localization/blob/8696ee5a9e4f959fcaae37835dcf2ed12ead581b/doc/navsat_transform_node.rst | 2026-09-27 | sources/robotlocalization_navsat_transform_node.rst | B |
| L2-S31 | Xsens, *MTi 600-series Datasheet*, MT1603P rev. 2020.B, June 2020. | https://www.xsens.com/hubfs/Downloads/Leaflets/MTi%20600-series%20Datasheet.pdf | 2026-09-27 | sources/xsens_2020_mti600_datasheet.pdf | A |
| L2-S32 | Xsens, *MTi Family Reference Manual*, MT1600P rev. 2020.A, June 2020. | https://www.xsens.com/hubfs/Downloads/Manuals/MTi_familyreference_manual.pdf | 2026-09-27 | sources/xsens_2020_mti_family_reference_manual.pdf | A |
| L2-S33 | Xsens, *MTi 600-series User Manual* (export created 2023-10-31). | https://mtidocs.movella.com | 2026-09-27 | sources/xsens_2023_mti600_user_manual.pdf | A |
| L2-S34 | Xsens, *Magnetic Calibration Manual*, MT0202P rev. O, Nov 2019. | https://www.xsens.com/hubfs/Downloads/Manuals/MT_Magnetic_Calibration_Manual.pdf | 2026-09-27 | sources/xsens_2019_magnetic_calibration_manual.pdf | A |
| L2-S35 | Movella, "Xsens GNSS/INS: Supercharge Your GNSS Receiver," application note MTAN001 rev. A, 28 Jun 2022. | https://www.movella.com/hubfs/Automation%20and%20Mobility%20-%20Application%20Notes/Supercharge%20your%20GNSS-INS%20Receiver/Xsens%20GNSS-INS%20-%20Supercharge%20Your%20GNSS%20Receiver%20-%20MTAN001-A.pdf | 2026-09-27 | sources/movella_2022_gnss_ins_supercharge_appnote.pdf | B |
| L2-S36 | Xsens BASE knowledge base, "MTi Filter Profiles" (last published 2026-05-27). | https://base.xsens.com/s/article/MTi-Filter-Profiles-1605869708823 | 2026-09-27 | sources/xsens_2026_kb_mti_filter_profiles.md | B |
| L2-S37 | Xsens BASE, "Best Practices for Automotive Applications" (last published 2022-06-30). | https://base.xsens.com/s/article/Best-Practices-for-Automotive-Applications?language=en_US | 2026-09-27 | sources/xsens_2022_kb_automotive_best_practices.md | B |
| L2-S38 | Xsens BASE, "Manual Gyro Bias Estimation (MGBE)" (last published 2024-06-21). | https://base.xsens.com/s/article/Manual-Gyro-Bias-Estimation?language=en_US | 2026-09-27 | sources/xsens_2024_kb_manual_gyro_bias_estimation.md | B |
| L2-S39 | Xsens BASE, "Estimating Yaw in magnetically disturbed environments" (last published 2022-09-30). | https://base.xsens.com/s/article/Estimating-Yaw-in-magnetically-disturbed-environments?language=en_US | 2026-09-27 | sources/xsens_2022_kb_yaw_magnetically_disturbed.md | B |
| L2-S40 | Movella/Xsens, Xsens MTi ROS 2 driver (`README.txt`, `param/xsens_mti_node.yaml`, `src/messagepublishers/imupublisher.h`), repo xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client, branch ros2, commit e145fb5. | https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client/tree/e145fb5051447374925a656d7fd637ff07085efe/src/xsens_mti_ros2_driver | 2026-09-27 | sources/xsens_ros2driver_README.txt; sources/xsens_ros2driver_xsens_mti_node.yaml; sources/xsens_ros2driver_imupublisher.h | B |
| L2-S41 | Xsens BASE, "Changing or Resetting the MTi reference co-ordinate systems" (last published 2022-01-11). | https://base.xsens.com/s/article/Changing-or-Resetting-the-MTi-reference-co-ordinate-systems-1605869706643?language=en_US | 2026-09-27 | sources/xsens_2022_kb_reference_frames_resets.md | B |
| L2-S42 | S. O. H. Madgwick, A. J. L. Harrison, R. Vaidyanathan, "Estimation of IMU and MARG orientation using a gradient descent algorithm," Proc. IEEE Int. Conf. Rehabilitation Robotics (ICORR), Zurich, 2011, pp. 179–185. doi:10.1109/ICORR.2011.5975346 | http://vigir.missouri.edu/~gdesouza/Research/Conference_CDs/RehabWeekZ%C3%BCrich/icorr/papers/Madgwick_Estimation%20of%20IMU%20and%20MARG%20orientation%20using%20a%20gradient%20descent%20algorithm_ICORR2011.pdf | 2026-09-27 | sources/madgwick_2011_imu_marg_gradient_descent.pdf | A |
| L2-S43 | Y. Wu, H. Zhang, M. Wu, X. Hu, D. Hu, "Observability of Strapdown INS Alignment: A Global Perspective," IEEE Trans. Aerosp. Electron. Syst. 48(1), 78–102, 2012. doi:10.1109/TAES.2012.6129622 (open copy: arXiv 1112.5282, accepted version) | https://arxiv.org/abs/1112.5282 | 2026-09-27 | sources/wu_2012_ins_alignment_global_observability.pdf | A |
| L2-S44 | K. S. Ali, C. A. Vanelli, J. J. Biesiadecki, M. W. Maimone, Y. Cheng, A. M. San Martin, J. W. Alexander, "Attitude and Position Estimation on the Mars Exploration Rovers," Proc. IEEE Int. Conf. Systems, Man and Cybernetics, 2005, vol. 1, pp. 20–27. doi:10.1109/ICSMC.2005.1571116 (JPL copy) | https://www-robotics.jpl.nasa.gov/media/documents/ali_sapp05.pdf | 2026-09-27 | sources/ali_2005_mer_attitude_estimation.pdf | A |
| L2-S45 | J. Ryu, E. J. Rossetter, J. C. Gerdes, "Vehicle Sideslip and Roll Parameter Estimation using GPS," Proc. 6th Int. Symposium on Advanced Vehicle Control (AVEC), Hiroshima, 2002 (Stanford copy). | http://www-cdr.stanford.edu/dynamic/estimationGPS/avec2002ryu.pdf | 2026-09-27 | sources/ryu_2002_sideslip_gps_estimation.pdf | A |
| L2-S46 | NOAA NCEI, "World Magnetic Model Accuracy, Limitations, and Error Model" (WMM2025) web page. | https://www.ncei.noaa.gov/products/world-magnetic-model/accuracy-limitations-error-model | 2026-09-27 | sources/noaa_2026_wmm_accuracy_error_model.md | B |
| L2-S47 | ArduPilot documentation, "Magnetic Interference" and "Advanced Compass Setup" (`common/source/docs/common-magnetic-interference.rst`, `common-compass-setup-advanced.rst`), ArduPilot/ardupilot_wiki commit 5365bb6. | https://github.com/ArduPilot/ardupilot_wiki/tree/5365bb696d91a16e6ffb5db2523f0bee0d13c94d/common/source/docs | 2026-09-27 | sources/ardupilot_wiki_magnetic_interference.rst; sources/ardupilot_wiki_compass_setup_advanced.rst | B (drone/autopilot project docs) |

