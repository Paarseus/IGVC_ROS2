# L4 — Sensor fusion

| | |
|---|---|
| **Question** | How do mobile robots combine wheel odometry, IMU and GNSS into one pose-and-velocity estimate with an honest uncertainty, and how is such a filter configured, checked and debugged? |
| **Covers** | ROS frame conventions (REP-103/105), Kalman filter theory (KF, EKF, UKF, error-state, invariant, factor graphs), motion and measurement models, covariances (R, Q, P0), outlier gating, GNSS integration with `navsat_transform_node` and dual EKF, delayed data, consistency tests (NEES/NIS), field failure modes, reference configurations, and robot_localization in ROS 2 Humble. |
| **Not covered** | GNSS receiver behaviour (L3), IMU internals (L2), wheel odometry derivation (L1), time synchronisation (L5). |
| **Status** | Verified |
| **Last updated** | 2026-09-28 |

**Citation locations.** For PDFs, "PDF p. N" is the page number of the downloaded file (it can differ from the printed journal page). For source code, "line N" refers to the pinned file in `sources/`. For documentation files, the section heading or parameter name is given.

## Summary

- ROS fixes where each estimate lives: `odom` is continuous but drifts without bound, `map` does not drift but may jump; because a TF frame can have only one parent, the tree is `map → odom → base_link` and the localization component publishes `map → odom` rather than `map → base_link` [L4-S09, sections "odom", "map", "Relationship between Frames", "Frame Authorities"].
- The robot_localization maintainers suggest two filter instances for GNSS: one fusing only continuous data (wheel odometry, IMU) in `odom`, used for local planning, and one fusing everything including GNSS in `map`; they also say this is "just a suggestion" [L4-S18, "Notes on Fusing GPS Data"].
- When the model is linear and the noises are white and Gaussian, the Kalman filter is "the best filter of any conceivable form"; without the Gaussian assumption it is still the best (minimum error variance) linear unbiased filter [L4-S36, PDF p. 10]; the EKF linearises about the current estimate and is "an ad hoc state estimator" [L4-S35, PDF p. 8].
- Covariances drive everything: robot_localization replaces a zero variance on a fused variable with a small value and raises a WARN diagnostic [L4-S25, lines 2501–2509; L4-S16, "Common errors"], and the docs say inflating covariances to hide a variable is "unnecessary and even detrimental" — the `_config` vectors should be used instead [L4-S16, "Odometry" item 3 and "Common errors"].
- Outlier gates in robot_localization are Mahalanobis distances whose defaults are `numeric_limits<double>::max()` (no gating); the maintainer states the example values are "arbitrary" [L4-S15, "~[sensor]_threshold"; L4-S53].
- Filter consistency is judged with NEES (needs ground truth) and NIS (needs only sensor data); both should follow chi-square distributions with the state or measurement dimension as degrees of freedom when the filter is consistent [L4-S43, PDF p. 2; L4-S14, PDF p. 11]; more generally, "a filter is optimal if" its innovations are zero-mean and white [L4-S54, PDF p. 101], and for Gaussian measurements a one-step predictor is optimal if and only if its residuals are zero-mean and white [L4-S54, PDF p. 133, Theorem 6.1]; a mismatch between the design and actual innovation statistics is the "prime indicator" of divergence [L4-S54, PDF p. 143].

## Foundational references

| ID | Reference | Why it is foundational |
|---|---|---|
| L4-S01 | Kalman 1960, "A New Approach to Linear Filtering and Prediction Problems" | Introduced the recursive Kalman filter. |
| L4-S02 | Julier & Uhlmann 1997, "A New Extension of the Kalman Filter to Nonlinear Systems" | Early description of the unscented filter; it cites an earlier 1995 ACC paper by Julier, Uhlmann & Durrant-Whyte on the same approach. |
| L4-S03 | Julier & Uhlmann 2004, "Unscented Filtering and Nonlinear Estimation" | Standard journal treatment of the UKF and the limits of linearisation. |
| L4-S04 | Smith & Cheeseman 1986, "On the Representation and Estimation of Spatial Uncertainty" | Formal treatment of compounding and merging uncertain transforms with first-order covariance propagation. |
| L4-S05 | Bar-Shalom, Li & Kirubarajan 2001, *Estimation with Applications to Tracking and Navigation* | Standard reference for NEES/NIS and gating. **Not downloaded** (no open copy); cited only through sources that summarise it. |
| L4-S06 | Thrun, Burgard & Fox 2005, *Probabilistic Robotics* | Standard robotics textbook on Gaussian filters and EKF localization. **Not downloaded** (no authorised open copy); not cited for findings. |
| L4-S07 | Moore & Stouch 2014/2016, "A Generalized Extended Kalman Filter Implementation for the Robot Operating System" | The paper that introduced robot_localization. |
| L4-S08, L4-S09 | REP-103 and REP-105 | Official ROS conventions for units, axes and the `map`/`odom`/`base_link` frames. |
| L4-S15 – L4-S27 | robot_localization 3.5.4 documentation, example configs and source | Primary specification of the implementation. |
| L4-S10 | Moore, ROSCon 2015 talk slides | The maintainer's own guidance. |
| L4-S11 | Mehra 1970, "On the Identification of Variances and Adaptive Kalman Filtering" | Root of innovation-based noise estimation. **Not downloaded** (no open copy). |
| L4-S12 | Bar-Shalom 2002, "Update with Out-of-Sequence Measurements in Tracking: Exact Solution" | Standard exact OOSM update. **Not downloaded** (no open copy). |
| L4-S13 | Larsen, Andersen, Ravn & Poulsen 1998, "Incorporation of Time Delayed Measurements in a Discrete-time Kalman Filter" | Compares ways to fuse time-delayed measurements in a discrete-time Kalman filter and derives an extrapolation method with an optimal gain. |
| L4-S14 | Huang, Mourikis & Roumeliotis 2010, "Observability-based Rules for Designing Consistent EKF SLAM Estimators" | Explains a basic source of EKF inconsistency and its fixes. |
| L4-S54 | Anderson & Moore 1979, *Optimal Filtering* (Prentice-Hall; author-hosted copy) | Standard textbook: innovations whiteness as the optimality test, filter divergence and its remedies, fixed-lag smoothing by state augmentation. Added in the gap check. |
| L4-S63 | Groves 2013, *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*, 2nd ed. | Standard text on GNSS/INS integration (loose and tight coupling). **Not downloaded** (no open copy); not cited for findings. |
| L4-S64 | Julier & Uhlmann 1997, "A Non-divergent Estimation Algorithm in the Presence of Unknown Correlations" (ACC) | Original Covariance Intersection paper. **Not downloaded** (no open copy); CI is cited through L4-S56. |
| L4-S65 | Fitzgerald 1971, "Divergence of the Kalman Filter" | Classic analysis of Kalman filter divergence. **Not downloaded** (no open copy); divergence is cited through L4-S54, which cites it. |

## Findings

### 1. Frame conventions and why they are arranged that way

- REP-103: all frames are right-handed; body axes are x forward, y left, z up; short-range geographic positions use ENU (x east, y north, z up); units are SI (metres, radians) [L4-S08, sections "Units", "Chirality", "Axis Orientation"].
- REP-103: yaw increases counter-clockwise and, for geographic poses, is zero when pointing east — unlike a compass bearing (zero north, clockwise); "Hardware drivers should make the appropriate transformations before publishing" [L4-S08, "Rotation Representation"].
- REP-103: an NED frame may be provided as a secondary frame with the `_ned` suffix; REP-103 also recommends a nearby origin (e.g. the start position) to avoid float32 precision problems [L4-S08, "Suffix Frames", "Axis Orientation"].
- REP-105: `base_link` is rigidly attached to the robot base, at any position the platform chooses [L4-S09, "base_link"].
- REP-105: the pose in `odom` "can drift over time, without any bounds" but "is guaranteed to be continuous"; it is "useful as an accurate, short-term local reference" [L4-S09, "odom"].
- REP-105: the pose in `map` "should not significantly drift over time" but "can change in discrete jumps at any time", making `map` "a poor reference frame for local sensing and acting" [L4-S09, "map"].
- REP-105: for maps referenced to the earth, the default is x east, y north, z up at the origin [L4-S09, "Map Conventions"].
- REP-105: "Although intuition would say that both `map` and `odom` should be attached to `base_link`, this is not allowed because each frame can only have one parent" [L4-S09, "Relationship between Frames"].
- REP-105: `odom → base_link` is broadcast by an odometry source; the localization component computes `map → base_link` but broadcasts `map → odom`, using the received `odom → base_link` [L4-S09, "Frame Authorities"].
- REP-105 notes that odom drift rates differ by platform: a vehicle with redundant high-resolution encoders drifts much less than "a skid steer robot which only has open loop feedback on turning" [L4-S09, "odom Frame Consistency"].
- REP-105: at long distances from the `odom` origin, float precision degrades; for centimetre accuracy the maximum distance is "approximately 83km" [L4-S09, "odom Frame Consistency"].
- robot_localization produces pose in `map` or `odom` and velocity in `base_link`; odometry pose is transformed from `header.frame_id` into `world_frame`, and twist from `child_frame_id` into `base_link_frame` [L4-S16, "Coordinate Frames and Transforming Sensor Data"].
- An IMU mounted rotated relative to the robot is handled by a static `base_link → imu` transform; the filter corrects the data automatically [L4-S16, "Coordinate Frames and Transforming Sensor Data"].
- For twist input, robot_localization adds the lever-arm term: linear velocity is rotated into `base_link` and the cross product of the sensor offset with the state's angular velocity is added [L4-S25, lines 3176–3180 and 3226–3230].
- For linear acceleration, the lever arm is not handled: a code comment says the transform "needs to take into account offsets from the origin" and currently only rotates the vector [L4-S25, lines 2657–2665].
- In `_differential` mode, the code zeroes the translation of the sensor-to-target transform before rotating the pose delta (line 3016) and then tags the resulting velocity as being in `base_link` (line 3056) [L4-S25, lines 3016, 3056].
- `navsat_transform_node` removes the GNSS antenna offset using the `base_link → gps` transform. When it computes the robot's Cartesian origin pose and that transform is missing, it logs an error and assumes the device is at the robot origin [L4-S26, lines 518–559]; when it computes the robot's world-frame origin pose and a transform is missing, it logs "Will not remove offset of navsat device from robot's origin" and leaves that pose at identity [L4-S26, lines 566–603].
- Nav2 expects `map → odom → base_link → [sensor frames]`; `map → odom` normally comes from a localization package and every sensor frame needs a transform back to `base_link`, usually via URDF and robot_state_publisher [L4-S30, "Transforms in Nav2"; L4-S32, "2. Setup GPS Localization system"].
- The Nav2 GPS tutorial uses `base_footprint` as the filter's `base_link_frame`, with robot_state_publisher providing `base_footprint → base_link` [L4-S32, "Local Odometry"; L4-S33, `base_link_frame`].
- Smith & Cheeseman define two operations on uncertain transforms: compounding (chaining transforms and their covariances, to first order) and merging (combining parallel estimates with the Kalman gain) [L4-S04, PDF pp. 2–6].
- Their method assumes small errors (first-order model) and sensor errors independent of location error [L4-S04, PDF p. 2].
- Autoware (automotive stack, ROS 2) uses a different tree: `earth → map → base_link → sensor frames`, with the localization module publishing `map → base_link` directly; `odom` or `base_footprint` may be added "as long as the tf structure above is maintained" [L4-S59, "TF tree"; L4-S60, "Output", "Kinematics Fusion Filter" and "TF tree"].
- In Autoware, `base_link` is the centre of the rear axle projected onto the ground, `map` axes are East, North, Up, and global positions are projected to a plane with UTM or MGRS [L4-S59, "TF tree"; L4-S60, "TF tree"].
- Autoware names per-sensor estimates `x_by_y` (frame x estimated by source y), e.g. `base_link_by_gnss_ins` is obtained from the GNSS/INS pose plus the static `gnss_ins → base_link` transform [L4-S59, "Estimating the `base_link` frame by using the other sensors"].
- Autoware's `gnss_poser` shifts the fix from the antenna frame (`NavSatFix.header.frame_id`) to `base_link` via TF; if that transform is unavailable it outputs the antenna position untransformed — the same fallback as `navsat_transform_node` [L4-S62, "Design"; L4-S26, lines 518–559].

### 2. Kalman filter theory: KF, EKF, UKF and alternatives

- Kalman showed that, for Gaussian processes, the optimal estimate is the orthogonal projection (conditional expectation); without Gaussianity, the orthogonal projection is optimal among linear estimates for squared-error loss [L4-S01, PDF p. 4, Theorem 2 and remark (e)].
- The KF equations: predict `x̂⁻ = A x̂ + B u`, `P⁻ = A P Aᵀ + Q`; update with gain `K = P⁻Hᵀ(HP⁻Hᵀ + R)⁻¹`, `x̂ = x̂⁻ + K(z − Hx̂⁻)`, `P = (I − KH)P⁻` [L4-S35, PDF p. 6, Figure 1-2].
- The KF assumes process and measurement noises independent, white and normally distributed [L4-S35, PDF p. 2].
- Under a linear model with white Gaussian noise, the mean, mode and median of the conditional density coincide; if the Gaussian assumption is removed, the KF is still "the best (minimum error variance) filter out of the class of linear unbiased filters" [L4-S36, PDF p. 10].
- White noise is used because it makes the mathematics tractable and, within the system's bandwidth, is indistinguishable from real wideband noise [L4-S36, PDF p. 11].
- The EKF linearises about the current mean and covariance using Jacobians [L4-S35, PDF pp. 7–8].
- "A fundamental flaw of the EKF is that the distributions ... are no longer normal after undergoing their respective nonlinear transformations. The EKF is simply an ad hoc state estimator" [L4-S35, PDF p. 8].
- Julier & Uhlmann report a consensus that the EKF is "difficult to implement, difficult to tune, and only reliable for systems which are almost linear on the time scale of the update intervals" [L4-S02, PDF p. 1; L4-S03, PDF p. 1].
- Linearisation can give a biased and inconsistent (variance-underestimating) result; a common fix is to pad the covariance [L4-S03, PDF pp. 3–4].
- The unscented transform propagates a set of sigma points through the nonlinear function; its mean and covariance are correct to second order, no Jacobians are needed, and its cost is "the same order of magnitude as the EKF" [L4-S03, PDF p. 6].
- robot_localization offers both an EKF and a UKF using the same motion model; the docs say the UKF "eliminates the use of Jacobian matrices and makes the filter more stable" but is "more computationally taxing" [L4-S15, "ukf_localization_node"].
- The UKF parameters `alpha` (default 0.001), `kappa` (default 0) and `beta` (default 2, Gaussian) should normally be left at default [L4-S15, "ukf_localization_node" parameters].
- Error-state (indirect) KF: the large-signal "nominal" state is integrated nonlinearly while the filter estimates a small error state; the error state is minimal (no redundant attitude parameters), stays near the origin (away from singularities such as gimbal lock), its second-order products are negligible, and its dynamics are slow, so corrections can run at a lower rate than predictions [L4-S42, §5.1, p. 52–53].
- The invariant EKF (IEKF) makes the estimation error autonomous for a broad class of systems on Lie groups and is locally stable around any trajectory under the standard linear-case conditions; in simulations the EKF can diverge where the IEKF "with identical tuning keeps converging" [L4-S41, PDF p. 1, abstract].
- The EKF's gain is computed assuming a small error; when the estimate is far from the truth the linearisation is invalid and the gain "may amplify the error", which can lead to divergence [L4-S41, PDF pp. 1–2].
- Factor graphs solve the fusion problem as nonlinear least squares over many states; the primary state-estimation node of the ROS 2 `fuse` package is a fixed-lag smoother [L4-S39, PDF pp. 17–18]. The fuse README describes it as a plugin-based nonlinear least-squares framework and warns that its ROS 2 port is "a work in progress" that is "**not** expected to work" until done [L4-S40, note at top and "Overview"].
- Maintainers list the advantages of fuse over robot_localization: configurable state, motion and sensor models; better support for relative pose measurements; linearisation errors reduced at every iteration — at higher compute cost [L4-S39, PDF p. 18].
- In their test on a 541 m route through a commercial environment, fuse's end estimate was 1.44 m closer to ground truth than robot_localization's, both below 1 % of distance, while fuse used about 3.7× the CPU [L4-S39, PDF pp. 18–19].
- Factor-graph smoothing "allows the easy incorporation of asynchronous and delayed measurements", which the authors call one of its main advantages over filtering [L4-S49, PDF p. 1].
- robot_localization wraps angle innovations (roll, pitch, yaw) with `angles::normalize_angle` before the update [L4-S24, lines 176–183].
- The short covariance update `P = (I − KH)P⁻` holds only for the optimal gain; the Joseph form `P = (I − KH)P⁻(I − KH)ᵀ + KRKᵀ` is valid for any K and keeps P symmetric, while the short form's subtraction can make P non-symmetric through floating-point error, which "usually leads to the Kalman filter diverging" [L4-S37, "Stable Compution of the Posterior Covariance"].
- robot_localization's EKF correction uses the Joseph form [L4-S24, lines 194–202].
- Covariance Intersection (CI) fuses two estimates whose error cross-correlation is unknown: the fused bound is `(ω Π_A⁻¹ + (1 − ω) Π_B⁻¹)⁻¹`, with ω in [0, 1] chosen to minimise a measure such as the determinant or trace; it "considers all admissible cross-correlations" and gives "the best guaranteed quality" [L4-S56, PDF pp. 1–3, Eqs. 13–15].
- Combining CI with a subsequent Kalman filter recursively does not give the optimal bound after later prediction and update steps; the optimal fusion under unknown correlation "cannot be obtained recursively" [L4-S56, PDF pp. 1, 6].
- REP-103 prefers quaternions (compact, no singularities) and discourages Euler angles because of 24 valid conventions [L4-S08, "Rotation Representation"]; robot_localization's state nevertheless holds orientation as Euler angles [L4-S07, PDF p. 2].

### 3. Motion (process) models for ground robots

- Land vehicles obey two nonholonomic constraints (no sideways and no vertical velocity when not slipping or jumping); these can be applied as "virtual observations" whose noise reflects expected violations from side slip and vibration [L4-S46, PDF pp. 3–4].
- With these constraints plus vehicle speed, velocity and attitude of a low-cost IMU become observable and errors are bounded; position stays unobservable without GPS [L4-S46, PDF p. 1 abstract, PDF p. 12].
- Process noise discretisation: for a continuous white-noise acceleration model, `Q = ∫₀^Δt F(t) Q_c F(t)ᵀ dt`; the spectral density is usually unknown and becomes "an 'engineering' factor" tuned experimentally [L4-S37, ch. 7 "Continuous White Noise Model"].
- Setting Q to zero except the highest-order term is "while not correct, ... often a useful approximation" [L4-S37, ch. 7 "Simplification of Q"].
- In a scalar example the predicted variance grows linearly with the time since the last measurement: `σ²(t₃⁻) = σ²(t₂) + σ_w²(t₃ − t₂)` [L4-S36, PDF p. 17].
- robot_localization's current state has 15 variables: `x, y, z, roll, pitch, yaw`, their velocities, and linear accelerations; it uses an omnidirectional 3-D kinematic model [L4-S39, PDF p. 17, Eq. 8; L4-S15, "ekf_localization_node"].
- The original 2014 paper described a 12-variable state (pose and velocities, no accelerations) [L4-S07, PDF p. 2].
- The package supports no other kinematic model; a unicycle model is obtained by clamping the 3-D variables plus `ẏ` and `ÿ` to zero [L4-S39, PDF p. 17].
- `two_d_mode` "will fuse 0 values for all 3D variables (Z, roll, pitch, and their respective velocities and accelerations)", keeping their covariances from exploding and holding the estimate to the X-Y plane [L4-S15, "~two_d_mode"].
- In code, `forceTwoD` sets those variables' measurement variances to `1e-6` and marks them as measured [L4-S25, lines 339–385].
- The docs frame `two_d_mode` as appropriate "if your robot is operating in a planar environment and you're comfortable with ignoring the subtle variations in the ground (as reported by an IMU)" [L4-S15, "~two_d_mode"].
- The Nav2 GPS tutorial runs both EKFs in 2-D mode "because nav2's costmap environment representation is 2-Dimensional, and several layers rely on the `base_link` frame being on the same plane as their global frame" [L4-S32, "Local Odometry"].
- A kinematic constraint can be fused as a measurement: odometry reporting zero `ẏ` (with a sensible variance) is "a perfectly valid measurement" for a robot that cannot move sideways [L4-S17, "Fusing Unmeasured Variables" item 2; L4-S07, PDF p. 5].
- The maintainers recommend fusing the zero `ẏ` from wheel odometry on differential-drive robots, to stop the filter "artificially generating non-zero ẏ in the state as the robot goes around turns" [L4-S39, PDF pp. 17–18].
- Without an acceleration reference, the filter predicts constant velocity; the fused velocity is then a weighted average of old and new values, which can cause "sluggish" convergence, especially visible in LIDAR data during rotations [L4-S20, comment above `use_control`].
- `use_control` turns `cmd_vel` into an acceleration term in the prediction, limited by `acceleration_limits`/`deceleration_limits`; a measured linear acceleration overrides it [L4-S15, "~use_control"; L4-S20, `use_control` block].
- robot_localization's EKF adds `delta_sec × Q` to the predicted covariance, i.e. Q is treated as a rate per second [L4-S24, lines 427–431].

### 4. Measurement modelling: what each sensor contributes and how

- GNSS/INS integration is usually either loosely coupled (LC: the GNSS position/velocity solution is fused with the INS, "easier to implement") or tightly coupled (TC: one centralised filter fuses raw GNSS measurements with the INS, "more complicated but can still be valid without enough satellites") (land-vehicle study) [L4-S58, PDF p. 2].
- In that study, tightly coupled fusion with only three GPS satellites limited 3-D position error to within 10 m over one minute, and horizontal drift to a few metres after several minutes [L4-S58, PDF p. 1, abstract].
- Nonholonomic (velocity) constraints are "the most common and effective constraint for land vehicle navigation using MEMS-based IMUs" [L4-S58, PDF p. 2].
- Output of a commercial GNSS receiver is itself filtered and time-correlated, so feeding it into another KF as if it were white measurement noise "violated the mathematical requirements of the filter"; in Labbe's simulation re-filtering produced smoother output that "diverges from the track" [L4-S38, "Exercise: Can you Filter GPS outputs?"].
- Each input's `_config` vector selects which of the 15 variables are fused, in the order `X, Y, Z, roll, pitch, yaw, Ẋ, Ẏ, Ż, roll̇, pitcḣ, yaẇ, Ẍ, Ÿ, Z̈`, and it is specified in the sensor's `frame_id`, not the world or base frame [L4-S15, "~[sensor]_config"; L4-S17, "Sensor Configuration"].
- The measurement matrix H is an identity-like selector: when measuring m variables, H is m × 12 (in the 2014 design) with ones in the measured columns, which allows partial updates [L4-S07, PDF p. 2].
- Odometry best practice: if it provides position and velocity, fuse the velocity; if it provides orientation and angular velocity, fuse the orientation [L4-S16, "Odometry" item 1].
- Wheel-encoder pose, heading and velocity usually come from the same source, so fusing all of them feeds "duplicate information"; the docs recommend fusing only the velocities [L4-S17, "Fusing Unmeasured Variables" item 1].
- An IMU's `Ÿ` acceleration is left out in the docs' example because noisy nonzero values "can cause your estimate to drift rapidly" [L4-S17, "Fusing Unmeasured Variables" item 3].
- Every linear and rotational dimension must be referenced by some input (pose or velocity); the maintainers advise one pose source when a dimension has a single input, and one pose plus many velocity sources when it has several [L4-S39, PDF p. 17].
- Linear acceleration alone is "generally insufficient" because double integration without a velocity or pose reference grows without bound [L4-S39, PDF p. 17].
- The maintainers advise starting with the minimum set of inputs that references every dimension and adding inputs one at a time [L4-S39, PDF p. 17].
- robot_localization raises a diagnostic ERROR when "Neither [a pose variable] nor its velocity is being measured", warning of "unbounded error growth and erratic filter behavior" [L4-S25, lines 1726–1752].
- `_differential`: a pose measurement at time t has the one at t−1 subtracted and is converted to velocity; this avoids oscillation when two absolute sources disagree, but orientation variance then "will grow without bound" unless another absolute source exists [L4-S15, "~[sensor]_differential"].
- Rule of thumb: with one orientation source, `_differential` should be false; with N sources, set it true for N−1 of them, "or simply ensure that the covariance values are large enough to eliminate oscillations" [L4-S17, "The differential and relative Parameters"].
- An example of why: 1.5 m of position error is acceptable, but 1.5 rad of yaw error makes position error "explode" when the robot next drives [L4-S17, "The differential and relative Parameters"].
- If two IMUs each report yaw variance 0.1 but differ by more than 0.1, the output "will oscillate back and forth"; users "should make sure that the noise distributions around each measurement overlap" [L4-S17, "The differential and relative Parameters"].
- With more than one absolute input for a pose variable, the node warns "This may result in oscillations" [L4-S25, lines 1713–1724].
- `_relative`: measurements are fused relative to the sensor's first measurement, without conversion to velocity [L4-S15, "~[sensor]_relative"]; if both `_relative` and `_differential` are true, differential wins [L4-S25, lines 1119–1124].
- GPS data from `navsat_transform_node` must be fused with `_differential` false: "enabling differential integration defeats the purpose of using it" [L4-S18, "Configuring robot_localization"; L4-S19, note under introduction].
- robot_localization assumes ENU IMU data and "does not work with NED frame data" [L4-S16, "Coordinate Frames and Transforming Sensor Data"].
- Expected IMU accelerometer signs: +9.81 m/s² on Z when flat, +9.81 on Y when rolled +90°, −9.81 on X when pitched +90° [L4-S16, "IMU" item 3].
- REP-145: IMU `frame_id` is the sensor frame; ENU world frames are x-east, y-north, z-up "relative to magnetic north"; without a magnetometer only roll and pitch are referenced and yaw "can be arbitrary" [L4-S28, "Frame Conventions"].
- Moore & Stouch's text says they fused roll, pitch, yaw and their rates from each of two IMUs and x and yaw velocity from the wheel encoders [L4-S07, PDF p. 2]; their sensor-configuration table marks x, y and z velocity plus yaw velocity for odometry and x, y, z for each GPS [L4-S07, PDF p. 3, Table I]. Their Pioneer 3 loop-closure error fell from 69.65 m, 160.33 m (odometry only) to 1.21 m, 0.26 m (odometry + two IMUs + one GPS) [L4-S07, PDF p. 3, Table II].

### 5. Covariances: sensor noise R, process noise Q, initial P0

- R is "usually measured prior to operation" from offline sample measurements; Q "is generally more difficult" because the process cannot be observed directly [L4-S35, PDF p. 6].
- With constant Q and R, P and K "stabilize quickly and then remain constant" [L4-S35, PDF p. 6].
- As R → 0 the gain trusts the measurement more; as P⁻ → 0 it trusts the prediction more [L4-S35, PDF pp. 3–4].
- "Covariance values **matter** to robot_localization" [L4-S16, "Odometry" item 3].
- Inflating a variance (e.g. to about 1e3) to make the filter ignore a variable, as older `robot_pose_ekf` drivers did, is "both unnecessary and even detrimental"; use `_config` instead [L4-S16, "Odometry" item 3 and "Common errors"].
- A zero variance on a fused variable is replaced with a small value and a diagnostic warning says it "should be corrected at the message origin" [L4-S25, lines 2502–2509; L4-S16, "Common errors"].
- In the EKF update, any measurement variance below `1e-9` is raised to `1e-9` so the gain computation does not "blow up"; negative variances are replaced by their absolute value [L4-S24, lines 120–147].
- REP-145: if an IMU driver does not know its covariance, all elements should be 0; if a field is not reported, the first element should be −1 [L4-S28, "Topics"].
- `NavSatFix.position_covariance` is in m², ENU, row-major; if only DOP is available, the driver should "estimate an approximate covariance from that"; `position_covariance_type` is UNKNOWN, APPROXIMATED, DIAGONAL_KNOWN or KNOWN [L4-S29, NavSatFix.msg].
- Overconfident inputs make the filter's own uncertainty wrong: in Moore & Stouch's runs, the estimated x/y standard deviations were "much smaller than the true position estimation errors", partly because odometry and IMU data were "noisier than [their] covariance values reported" and Q was not tuned [L4-S07, PDF p. 5].
- Q in robot_localization is exposed because it "can be difficult to tune"; the larger Q is relative to a measurement's variance, the faster the filter converges to that measurement [L4-S15, "~process_noise_covariance"; L4-S07, PDF p. 2].
- The example config suggests raising a variable's Q diagonal if it is slow to converge [L4-S20, comment above `process_noise_covariance`].
- Default Q diagonal (x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az): 0.05, 0.05, 0.06, 0.03, 0.03, 0.06, 0.025, 0.025, 0.04, 0.01, 0.01, 0.02, 0.01, 0.01, 0.015 [L4-S23, lines 110–124].
- `dynamic_process_noise_covariance` scales Q by the norm of the velocity so that covariance "stop[s] growing when the robot is stationary" [L4-S15, "~dynamic_process_noise_covariance"; L4-S23, lines 129–152].
- P0 (`initial_estimate_covariance`) controls how fast measurements are trusted at start: a tiny value (e.g. 1e-12) with a high-variance measurement makes the filter "very slow to 'trust'" it [L4-S15, "~initial_estimate_covariance"].
- When fusing only velocities, users "will likely *not* want" large P0 on pose variables, since those errors grow without bound anyway [L4-S15, "~initial_estimate_covariance"; L4-S20, comment above `initial_estimate_covariance`].
- The default P0 is `1e-9` on every diagonal element [L4-S20, `initial_estimate_covariance`; L4-S23, line 91].
- On the first measurement, the filter copies the measured variables' values and covariance into the state and P, so P0 only persists for variables that first measurement did not contain [L4-S23, lines 227–247].
- When no measurement arrives within `sensor_timeout` (default `1/frequency`), the filter predicts without correcting, so covariance keeps growing [L4-S15, "~sensor_timeout"; L4-S25, lines 699–708 and 877].
- Without absolute yaw or position, dead-reckoning covariance grew so fast that "the condition number of the covariance matrix grew rapidly, indicating filter instability" [L4-S07, PDF p. 5].
- Q may also be adapted online, e.g. reduced when motion is slow and increased when dynamics change [L4-S35, PDF p. 7].
- Systematic tuning: Q and R can be tuned by minimising a cost built from the NEES (with ground truth) or NIS (sensor data only) chi-square statistics, with Bayesian optimisation [L4-S43, PDF pp. 1, 5].
- Tuning against chi-square tests "is most often done manually", by repeated guessing and checking over Monte Carlo runs [L4-S43, PDF p. 3].
- Adaptive filters based on the innovation need reliable observations and are "significantly affected by outliers" [L4-S44, PDF p. 1].
- Methods for estimating unknown Q and R fall into four groups: Bayesian inference, maximum likelihood, covariance matching and correlation methods; Mehra (1970) introduced the first innovation-correlation method, and Odelson et al. (2006) developed autocovariance least squares (ALS) from it [L4-S55, PDF pp. 2–3].
- Covariance matching raises Q when the sample innovation covariance is much larger than its theoretical value; "The convergence has never been proved for this method" [L4-S55, PDF p. 3].
- Not every Q and R can be identified from data: identifiability depends on the rank of a matrix built from innovation auto- and cross-covariances, and Odelson gave an observable and controllable system whose full Q was not estimable [L4-S55, PDF pp. 6, 9–12].
- Anderson & Moore list remedies for model error: increasing the design input (process) noise, adapting it online from the measured innovations, overweighting recent data (finite memory or exponential weighting), or putting a lower bound on the gain; they call raising the noise variance and exponential weighting the "easiest" [L4-S54, PDF p. 144].
- Autoware's EKF tuning guide sets the continuous process-noise standard deviations from physical limits: `proc_stddev_vx_c` to the maximum linear acceleration and `proc_stddev_wz_c` to the maximum angular acceleration [L4-S61, "2. Tune process model parameters"].

### 6. Outlier rejection and robust fusion

- A KF itself "provides no way to detect and reject a bad measurement"; one far-off measurement pulls the estimate strongly toward it [L4-S38, "Detecting and Rejecting Bad Measurement"].
- The squared Mahalanobis distance of the innovation, `γ = ẑᵀ S⁻¹ ẑ` with `S = HPHᵀ + R`, follows a chi-square distribution with m (measurement dimension) degrees of freedom when there are no outliers; a threshold is taken from the chi-square table for a chosen significance level α [L4-S45, PDF p. 7, Eqs. 32–36; L4-S44, PDF p. 5].
- Gao et al. used 12.592, the chi-square threshold at 95 % confidence with 6 degrees of freedom (vehicular INS/GNSS) [L4-S45, PDF p. 10].
- Instead of rejecting, the measurement covariance can be inflated by a scale factor until the gate is satisfied, reducing the gain for abnormal observations (vehicular INS/GNSS) [L4-S45, PDF pp. 1, 8].
- robot_localization's `*_rejection_threshold` parameters are Mahalanobis distances limiting "how far away from the current vehicle state a sensor measurement is permitted to be"; each "defaults to `numeric_limits<double>::max()`" [L4-S15, "~[sensor]_threshold"; L4-S25, lines 1128–1137].
- Thresholds apply separately to the pose and twist parts of a message (and to linear acceleration for IMUs), not per variable [L4-S20, comment above `odom0_pose_rejection_threshold`; L4-S15, "~[sensor]_threshold"].
- The code compares the squared distance with the threshold squared (`n_sigmas * n_sigmas`) over all fused variables of that part together [L4-S23, lines 431–450; L4-S24, lines 185–189].
- The code does not scale the threshold with the number of fused variables (it uses `n_sigmas²` directly) [L4-S23, lines 435–439]; for a chi-square gate the threshold depends on the degrees of freedom [L4-S45, PDF p. 7].
- The maintainer describes the threshold in 1-D as "effectively just an unsigned Z-score" and says the template values "are just examples ... the values are arbitrary" [L4-S53].
- The example config says it is "strongly recommended that these parameters be removed if not required" [L4-S20, comment above `odom0_pose_rejection_threshold`].
- A rejected measurement is simply not applied (no state or covariance update) [L4-S24, lines 185–211].
- The maintainer points users to these thresholds for multipath-type outliers; the issue author notes the risk that "measurements are discarded for some time, the covariance of the prediction grows, enabling faulty measurements to be fused" [L4-S52].
- Labbe: a 3-sigma gate "is likely to discard some good measurements" because real data are not purely Gaussian [L4-S38, "Detecting and Rejecting Bad Measurement"]; "Theory says 3 std, but practice says otherwise. You will need to experiment" [L4-S38, "Gating and Data Association Strategies"].
- Rectangular gates are cheap but pass more spurious measurements than ellipsoidal (Mahalanobis) gates [L4-S38, "Gating and Data Association Strategies"].
- Robust optimisation alternative (automotive GNSS): "switch variables" attached to each pseudorange factor let the optimiser turn off multipath outliers; about 21 % of observations were declared outliers in an urban dataset, and switch values ended close to 0 or 1 [L4-S48, PDF pp. 3–5].
- Least squares is "not robust against such outliers", and "even a single outlier can have catastrophic effects" [L4-S48, PDF p. 3].
- Autoware's EKF gates pose and twist separately, assuming the Mahalanobis distance follows chi-square with 3 degrees of freedom for pose and 2 for twist; its table gives thresholds such as 11.3 (3 DOF) and 9.21 (2 DOF) at significance 10⁻², up to 49.5 and 46.1 at 10⁻¹⁰ [L4-S61, "3. Tune gate parameters"].
- Because "the accuracy of covariance estimation itself is not very good", Autoware recommends a very small significance level (a wide gate) to reduce false rejections [L4-S61, "3. Tune gate parameters"].
- Innovation-based fault detection and exclusion in an EKF performs well for dynamic platforms when the models are right, but it is model-dependent: with unmodelled errors or unexpected dynamics it is "prone to high false alarm" (urban GNSS/INS) [L4-S57, PDF p. 13].

### 7. GNSS integration: navsat_transform, dual EKF and quality changes

- GPS data are "subject to discrete discontinuities ('jumps')", so an estimate including GPS "will likely be unfit for use by navigation modules" [L4-S18, "Notes on Fusing GPS Data"].
- Suggested setup: filter 1 fuses only continuous data with `world_frame = odom` (local plans run here); filter 2 fuses everything including GPS with `world_frame = map` [L4-S18, "Notes on Fusing GPS Data"].
- When `world_frame = map`, "something else" must generate `odom → base_link`; if that is another robot_localization instance, "that instance should *not* fuse the global data" [L4-S15, "~[frame]" item 3].
- The robot_localization dual-EKF example fuses wheel velocities (vx, vy, vz, vyaw) and IMU roll, pitch and all rates and accelerations in both filters, and adds `odometry/gps` x and y only in the `map` filter [L4-S21, `ekf_filter_node_odom`, `ekf_filter_node_map`].
- In that example the `map` filter's Q for x and y is 1.0 versus 1e-3 in the `odom` filter, and its P0 for x and y is 1.0 versus 1e-9 [L4-S21, `process_noise_covariance`, `initial_estimate_covariance`].
- The Nav2 GPS tutorial uses the same dual-EKF layout and describes it as "a common setup on robot_localization when using GPS data" [L4-S32, "2. Setup GPS Localization system"].
- `navsat_transform_node` needs three inputs: a NavSatFix, an IMU with earth-referenced heading, and the filter's odometry output (for the robot's current pose when the first fix arrives) [L4-S18, "Required Inputs"; L4-S19, "Subscribed Topics"].
- It converts the fix to UTM (or a local Cartesian frame with `use_local_cartesian`) and builds a transform from the first fix, the IMU heading and the current odometry pose; it then uses this transform for all later fixes [L4-S10, slide 14; L4-S26, lines 102, 247–330].
- The transform is computed once (`transform_good_ = true`) and is not recomputed from later fixes; the node stops using the IMU once it has the transform (unless `use_odometry_yaw` or manual datum) [L4-S26, lines 247–330 and 660–664]. It is reset (and recomputed) only when the `set_datum` service is called or `magnetic_declination_radians` is changed at runtime [L4-S26, lines 365 and 921].
- The heading used is `imu_yaw + magnetic_declination + yaw_offset + meridian_convergence`; meridian convergence is 0 with `use_local_cartesian` [L4-S26, lines 270–299].
- IMU heading must be zero facing east; an IMU reading zero at north needs `yaw_offset` = π/2 [L4-S19, "~yaw_offset"; L4-S18, "IMU Data"].
- `magnetic_declination_radians` is "needed if your IMU provides its orientation with respect to the magnetic north" [L4-S19, "~magnetic_declination_radians"].
- `use_odometry_yaw` takes heading from the odometry input, which must be earth-referenced (at least one absolute orientation source with `_differential` and `_relative` false) [L4-S19, "~use_odometry_yaw"; L4-S22, comment above `use_odometry_yaw`].
- `datum` fixes the origin: `[latitude°, longitude°, heading rad]`, heading 0 = east; it requires `wait_for_datum: true` [L4-S18, "Required Inputs" item 2; L4-S22, `datum`].
- With automatic datum, the origin is set to the first valid fix; a fixed datum makes the same coordinates always map to the same Cartesian point [L4-S32, "Navsat Transform"].
- `delay` waits before computing the transform, "especially important if you have `use_odometry_yaw` set to true" [L4-S22, `delay`].
- Heading is critical: the Nav2 tutorial says "measuring absolute orientation is mandatory" and lists why commercial IMUs often give poor heading: no magnetometer, hard to calibrate on large robots, motor and current-induced magnetic disturbances [L4-S32, "GPS Localization Overview"].
- Without a good initial heading the robot "may need to move around for a bit in an 'initialization dance'"; dual-GPS or map overlays give a good initial heading [L4-S32, same section].
- The Nav2 tutorial states standalone GPS accuracy of "1-2 meters under excellent conditions and up to 10 meters", with "frequent jumps"; RTK can bring it "down to 1cm" [L4-S32, "GPS Localization Overview"].
- The node discards a fix only if status is `STATUS_NO_FIX` or latitude, longitude or altitude is NaN; otherwise it copies the fix's covariance, whatever the fix status, and later rotates it into the world frame for the output [L4-S26, lines 619–652 and 803–820].
- NavSatStatus has only NO_FIX (−1), FIX (0), SBAS_FIX (1) and GBAS_FIX (2) [L4-S29, NavSatStatus.msg].
- As a result the ROS 2 NMEA driver maps both RTK fixed (GGA quality 4) and RTK float (5) to `STATUS_GBAS_FIX`; their default position errors differ (0.02 vs 4.0) and are multiplied by HDOP to give the variance, unless the receiver sends GST sentences, whose error estimates then replace the defaults [L4-S51, lines 58–64, 75–117, 179–191].
- Default NMEA-driver position errors: SPS 4.0, DGPS 0.1, RTK fixed 0.02, RTK float 4.0, WAAS 3.0, invalid 1000000 (standard deviations before multiplication by HDOP) [L4-S51, lines 59–64].
- In Moore & Stouch's test with GPS only every 120 s, each fix caused "noticeable instantaneous position changes", pulling the state part-way toward the GPS while x/y variances "decrease considerably"; loop-closure error was (12.06, 0.52) m [L4-S07, PDF p. 5].
- A vineyard robot study (agricultural) reports RTK GPS that "does not return to an accurate fix in (semi-) occluded areas", odometry slip on turns, and IMU drift from vibration; the authors found robot_localization "did not perform well" there and replaced it with a rule-based selection of the most accurate sensor [L4-S47, PDF p. 1 abstract, PDF pp. 3–4].
- Autoware's EKF has a "smooth update": because a single update from a low-rate measurement "can cause large changes in the estimated value", the measurement is split into several pieces applied over successive steps; `pose_smoothing_steps` trades smoothness against estimation performance [L4-S61, "Features" and "1. Tune sensor parameters"].
- Autoware's design doc states GNSS-only accuracy of "~10m", improved to "~10cm" with RTK, and lists blocked signals (tunnels, buildings) as the destabilising situation [L4-S60, "GNSS"].
- Aviation-derived GNSS integrity defines an Alert Limit (largest allowable error), a Protection Level computed by the user (alert when PL > AL), a Time to Alert and an Integrity Risk; aviation budgets the risk of hazardously misleading information at 10⁻⁷ to 10⁻⁹, while no such value had been set for urban applications [L4-S57, PDF pp. 4–5].
- Classic receiver integrity monitoring (RAIM) mainly targets large errors and single faults and needs redundant satellites; the review found RAIM-based methods perform well in open sky but not in urban canyons [L4-S57, PDF pp. 6, 13].

### 8. Delayed, out-of-order and asynchronous measurements

- Fusing a measurement that arrives late (e.g. slow vision processing) is "not a trivial problem"; the designer trades optimality against computation [L4-S13, PDF p. 2].
- Options compared by Larsen et al.: recalculating the filter from the measurement time, Alexander's method, a modified Alexander method, and extrapolating the delayed measurement to the present with an optimal gain [L4-S13, PDF pp. 3–6].
- Extrapolation is optimal when no other measurements were fused during the delay; otherwise it is not strictly optimal [L4-S13, PDF p. 5].
- Recalculation "can only be used if the delay N is small or if the computation time is uncritical" [L4-S13, PDF p. 6].
- robot_localization's `smooth_lagged_data`: on a measurement older than the last update, the filter reverts to the last saved state before it and re-processes all measurements up to now; `history_length` (seconds) must cover the lag [L4-S15, "~smooth_lagged_data", "~history_length"; L4-S25, lines 612–660].
- If the history is too short, the revert fails and the old measurement is processed without reverting [L4-S25, lines 634–650].
- With `smooth_lagged_data` false, a measurement older than the last filter update is not predicted to its own time: `processMeasurement` skips the predict step when the time delta is negative and applies the correction to the current state [L4-S23, lines 205–226].
- A message older than the previous message on the same topic is dropped with a diagnostic warning that it "may indicate a bad timestamp" [L4-S25, lines 204–262; same check at lines 1953 and 2379].
- Messages stamped at or before the last `set_pose` reset are ignored [L4-S25, lines 1822–1830].
- `permit_corrected_publication` re-publishes a corrected state with an old stamp when a late measurement arrives (default false) [L4-S15, "~permit_corrected_publication"].
- `predict_to_current_time` also predicts to the current time rather than only to the latest measurement [L4-S15, "~predict_to_current_time"].
- `transform_timeout` defaults to 0, meaning the latest available transform is used; a nonzero value makes the filter wait for transforms and it will then often miss its output rate [L4-S15, "~transform_timeout"].
- Queue sizes (`*_queue_size`) should be raised when sensors run much faster than `frequency`, so intermediate measurements are not lost [L4-S15, "~[sensor]_queue_size"].
- With unknown delays, ignoring the delay "produces high systematic errors"; explicitly estimating each delay inside a factor graph removed them in simulation of odometry + GPS fusion [L4-S49, PDF pp. 2, 6].
- Fixed-lag smoothing can be derived by running a Kalman filter on an augmented signal model that stacks delayed copies of the state [L4-S54, PDF pp. 186–188, Sec. 7.3].
- Autoware's EKF handles measurement delay with such an augmented state (citing Anderson & Moore Sec. 7.3) and says the computational cost "does not significantly change" because of the augmented structure; it raises a WARN when a message's timestamp is beyond the delay-compensation range [L4-S61, "time delay model", "Diagnostics"].
- Autoware's tuning guide asks first to check that message stamps are sensor time, with `pose_additional_delay`/`twist_additional_delay` to correct known offsets, and that the pose derivative matches the twist (unit or bias errors cause "large estimation errors") [L4-S61, "0. Preliminaries"].

### 9. Filter consistency and evaluation

- NEES: `ε_x = eᵀ P⁻¹ e` with the true error e; NIS: `ε_z = ẑᵀ S⁻¹ ẑ` with innovation ẑ. For a consistent filter they are chi-square with n_x and n_z degrees of freedom [L4-S43, PDF p. 2, Eqs. 18–19].
- NEES needs ground truth, usually from Monte Carlo "truth model" simulations; NIS can be checked online on real data logs [L4-S43, PDF pp. 1–2].
- For a consistent filter the average NEES should be close to the state dimension (e.g. about 3 for a 2-D robot pose); larger deviations mean worse inconsistency [L4-S14, PDF p. 11].
- For the same error, NEES gets larger as P gets smaller; a P that is small relative to the actual error indicates worse performance, and the average NEES should be below the state dimension [L4-S38, "Normalized Estimated Error Squared (NEES)"].
- Labbe calls an overconfident filter "smug": its P keeps shrinking while its residuals leave the 3σ bounds, because "P is only reporting the theoretical performance ... assuming all of the inputs are correct" [L4-S38, "Evaluating Filter Order"].
- Plotting residuals against their 3σ bounds is a practical check during design [L4-S38, "Evaluating Filter Order" and "Normalized Estimated Error Squared"].
- Standard EKF-SLAM is inconsistent because linearising at the latest estimate makes the unobservable global orientation appear observable, so covariance shrinks "in directions of the state space where no information is available" [L4-S14, PDF pp. 1, 7].
- Fixes: First-Estimates Jacobian (FEJ) EKF and Observability-Constrained (OC) EKF choose linearisation points so the model keeps the right unobservable directions; both beat the standard EKF in consistency and accuracy [L4-S14, PDF pp. 1–2, 9–10].
- In Julier & Uhlmann's polar-to-Cartesian example the linearised estimate is biased and inconsistent; they add that Lerro showed, for radar tracking, that the transformations can become inconsistent when the bearing standard deviation is below one degree [L4-S03, PDF p. 3].
- Ground-truth metrics: relative pose error (RPE) measures local drift; absolute trajectory error (ATE) first aligns the estimated and true trajectories and then compares absolute poses (from visual SLAM benchmarking) [L4-S50, PDF p. 2].
- Timestamps between the estimator and ground truth must be synchronised; the TUM benchmark measured and removed the delay between its motion-capture system and camera [L4-S50, PDF pp. 4, 6].
- Moore & Stouch evaluated by loop-closure error (driving back to the start) and compared the filter's final standard deviations with the actual error [L4-S07, PDF p. 3].
- Macenski et al. used an amcl-based trajectory as ground truth on a 541 m route that ended within 10 cm of its start [L4-S39, PDF p. 18].
- Maybeck frames performance evaluation as one of four core questions of stochastic estimation and control [L4-S36, PDF pp. 5–6].
- The innovations of the optimal filter are white; "a filter is optimal if that quantity which should be the innovations sequence is zero mean and white" [L4-S54, PDF pp. 100–101].
- Anderson & Moore prove the converse (Theorem 6.1, for Gaussian measurements and a causally invertible predictor): a one-step predictor is the optimal one if and only if its residual sequence is zero-mean and white; checking whiteness in general needs correlations at all lags, but for finite-dimensional time-invariant models a finite number of lags suffices (Theorem 6.2) [L4-S54, PDF pp. 132–134].
- These tests assume stationary, ergodic processes so that time averages can stand in for ensemble averages [L4-S54, PDF p. 132].
- If the gain is not optimal, the innovation sequence is correlated; Mehra's method uses sample innovation autocorrelations at several lags, which vanish for all nonzero lags at the optimal gain [L4-S55, PDF pp. 12–13].

### 10. Failure modes of EKF localization on field robots and their diagnosis

- Divergence from a wrong model or overconfident inputs, with P still shrinking ("smug" filter) [L4-S38, "Evaluating Filter Order"].
- EKF divergence when the estimate is far from truth, because the linearisation-based gain amplifies the error [L4-S41, PDF pp. 1–2].
- Oscillation between two absolute sources of the same variable whose covariances do not overlap [L4-S17, "The differential and relative Parameters"; L4-S20, comment above `odom0_differential`].
- Unbounded yaw covariance when the only yaw source is fused differentially [L4-S15, "~[sensor]_differential"; L4-S17, same section].
- Unmeasured variables (neither pose nor velocity) causing "unbounded error growth and erratic filter behavior" [L4-S25, lines 1726–1752].
- Covariance growth and instability in dead reckoning without absolute yaw or position (condition number of P grew rapidly) [L4-S07, PDF p. 5].
- Heading errors from magnetometers near electromagnetic interference: in Moore & Stouch's parking-lot tests, a second IMU barely helped because it suffered the same interference and because it stopped reporting data halfway through the collection [L4-S07, PDF p. 5].
- Wheel slip on turns (especially skid-steer), IMU drift from vibration, and loss of RTK fix near buildings in agricultural field robots [L4-S47, PDF pp. 1, 3–4].
- Sign and frame errors: data not following REP-103 (angles not increasing in the right direction), wrong `frame_id`s, inflated or missing covariances are the documented "Common errors" [L4-S16, "Common errors"].
- Two publishers of `odom → base_link` (e.g. a robot driver and the EKF) — the driver's broadcast must be disabled if robot_localization is to own the transform [L4-S16, "Odometry" item 5].
- `map` jumps when absolute (GPS) data are fused, by design of REP-105 [L4-S09, "map"; L4-S18, "Notes on Fusing GPS Data"].
- Gating lock-out risk: while measurements are rejected, covariance grows and may later admit faulty ones [L4-S52].
- Bad timestamps: out-of-order messages on a topic are dropped with a warning [L4-S25, lines 204–262; same check at lines 1953 and 2379].
- Missing `base_link → gps` or `world → base_link` transforms make `navsat_transform_node` skip the antenna-offset correction with an error log [L4-S26, lines 551–603].
- Diagnosis tools: `print_diagnostics` publishes to `/diagnostics` [L4-S15, "~print_diagnostics"]; the example config suggests echoing `/diagnostics_agg` "if you're having trouble" [L4-S20, `print_diagnostics`]; `debug` writes "a massive amount of data" to a file [L4-S15, "~debug"].
- A rejected measurement is logged in debug output with its squared Mahalanobis distance, threshold, innovation and innovation covariance [L4-S23, lines 440–446]; the issue asking for the Mahalanobis distance to be published stayed closed, and the maintainer said he would reopen it "if someone would be willing to PR the fix" [L4-S53].
- `reset_on_time_jump` resets the filter when sim time jumps backwards (bag replay) [L4-S15, "~reset_on_time_jump"].
- Robust estimation (switch variables) can identify and remove GNSS multipath outliers in urban driving [L4-S48, PDF pp. 1, 5].
- Divergence (textbook definition): the filter's own error covariance stays bounded while the actual error becomes very large relative to it, or unbounded; it is "typically, but not always" linked to low or zero process noise, signal models that are not asymptotically stable, and bias errors, and "seems to arise more from modeling error than computational errors" [L4-S54, PDF pp. 142–143].
- A second warning sign is a gain (or design covariance) tending to zero: old data "saturate" the filter, and it may converge to a wrong value — it has "learned the wrong state" [L4-S54, PDF p. 143].
- Autoware's EKF diagnostics raise WARN or ERROR when too many consecutive updates are missing (`*_no_update_count_threshold_*`), when a measurement exceeds the Mahalanobis gate or the delay range, and when the covariance ellipse's long axis or lateral axis exceeds a size threshold [L4-S61, "Diagnostics"].
- A planar (3-DoF: x, y, yaw) filter on slopes: Autoware notes the vehicle then appears "buried in the ground" when going uphill and corrects z from pitch [L4-S61, "Features"].
- Autoware's EKF estimates one yaw bias (sensor mounting error); with several pose sources, each with its own yaw bias, the single bias state "would not make any sense" (listed known issue) [L4-S61, "Kalman Filter Model" and "Known issues"].
- Automotive design docs list situations that destabilise each input: wheel speed on slippery or bumpy roads, IMU bias that depends on temperature, magnetometers near steel structures, GNSS blocked by buildings or tunnels [L4-S60, sections "Wheel speed sensor", "IMU", "Geomagnetic sensor", "GNSS"].

### 11. Reference configurations and how other robots do it

- robot_localization template (`ekf.yaml`): 30 Hz, `sensor_timeout` 0.1 s, `two_d_mode` false, `world_frame: odom`, `publish_tf` true, example inputs with rejection thresholds, `use_control` true with `acceleration_limits` [1.3, 0, 0, 0, 0, 3.4] [L4-S20].
- Dual-EKF navsat example: both filters 30 Hz, `two_d_mode` false; navsat `delay` 3.0 s, `magnetic_declination_radians` 0.0429351 ("For lat/long 55.944831, -3.186998"), `yaw_offset` π/2 ("IMU reads 0 facing magnetic north"), `broadcast_utm_transform` true [L4-S21, `navsat_transform`].
- Nav2 GPS demo: both EKFs in 2-D mode with `base_footprint`; wheel odometry velocities (vx, vy, vz, vyaw) in both; IMU yaw only (absolute) in both; GPS x, y in the map filter; navsat `zero_altitude` true, `use_odometry_yaw` true [L4-S33].
- The Nav2 demo comments that on a real robot `imu0_differential` might be set true "since usually absolute measurements from real imu's are not very accurate" [L4-S33, `imu0_differential`].
- Nav2's "Smoothing Odometry" guide fuses wheel odometry and IMU in one EKF that publishes `odom → base_link` [L4-S31, introduction and "Configuring Robot Localization"].
- Clearpath's A200 (Husky) default localization config runs one EKF in `odom` at 50 Hz in 2-D mode, fusing only wheel odometry x, y, yaw, vx, vy and vyaw [L4-S34].
- Moore & Stouch's Pioneer 3: odometry + two IMUs + one GPS gave loop-closure error 1.21 m, 0.26 m after about 777 s over a route reaching about 110 m from the origin [L4-S07, PDF p. 3].
- Vineyard robot (Rovitis 4.0, agricultural): custom sensor-selection fusion of wheel odometry, IMU and RTK GPS reported mean error 0.005 m ± 0.220 m in position and 0.6° ± 3.5° in orientation against a drone-based ground truth [L4-S47, PDF p. 1].
- Maintainers' comparison (541 m route through a commercial environment, wheel encoders + IMU): robot_localization at 30 Hz, fuse at 20 Hz; fuse about 3.7× the CPU [L4-S39, PDF pp. 18–19].
- Autoware (automotive): a pose estimator (matching external sensor data to the map) and a twist/accel estimator feed a kinematics fusion filter that publishes `map → base_link`, and a separate localization-diagnostics module monitors reliability [L4-S60, "Recommended Architecture"]; required outputs are pose, twist and acceleration with covariance at "50Hz~" [L4-S60, "Required Architecture" → "Output"]; the EKF uses a 2-D vehicle model with a yaw-bias state [L4-S61, "Overview" and "Kalman Filter Model"].

### 12. Product-specific: robot_localization in ROS 2 Humble and GNSS/INS inputs

- The pinned release is robot_localization 3.5.4 (2025-08-29, commit 8696ee5); its changelog includes "Fixing angle clamping for humble (#854)" (3.5.3) and "Utm using geographiclib humble branch (#834)" (3.5.2) [L4-S27, entries 3.5.2–3.5.4].
- 3.5.4 changes include "Fixing off-diagonal covariance values in measurement (#942)" and a runtime parameter callback for `magnetic_declination_radians` (#920) [L4-S27, 3.5.4].
- 3.5.2 made navsat_transform wait for an odometry message before setting a manual datum (#835) [L4-S27, 3.5.2].
- In ROS 2, `broadcast_utm_transform` is deprecated in favour of `broadcast_cartesian_transform`, and `use_local_cartesian` switches from UTM to a local ENU frame [L4-S26, lines 102, 109–134, 337].
- In the 3.5.4 code, the deprecated parent-frame parameter is declared with a trailing underscore (`"broadcast_utm_transform_as_parent_frame_"`), unlike the documented name [L4-S26, line 123; L4-S19, "~broadcast_utm_transform_as_parent_frame"].
- `publish_filtered_gps` defaults to true in the 3.5.4 code [L4-S26, line 99].
- `print_diagnostics` defaults to false in the 3.5.4 code [L4-S25, line 762].
- `sensor_timeout` defaults to `1/frequency` [L4-S25, line 877; L4-S20, comment above `sensor_timeout`].
- The docs still describe ROS 1 details (XML `<rosparam>`, `tcpNoDelay`, `ros::Time::isSimTime()`), and the Nav2 guide links ROS 1 (melodic, jade, noetic) API pages for parameters [L4-S15, "~[sensor]_nodelay", "~reset_on_time_jump"; L4-S31, "Configuring Robot Localization"; L4-S32, "GPS Localization Overview"].
- `navsat_transform_node` development was done with a Garmin 18x receiver, so "there may be intricacies of the data generated by other units" [L4-S18, "GPS Data"].
- An INS that runs its own internal filter produces time-correlated outputs; feeding them into another KF as independent white measurements breaks the KF's assumptions (general KF principle, shown for a GNSS receiver) [L4-S38, "Exercise: Can you Filter GPS outputs?"; L4-S35, PDF p. 2].

## Recommended practice

1. Make sensor data follow REP-103/105: ENU, right-handed, SI units, yaw counter-clockwise from east; check signs by moving the robot [L4-S16, "Adherence to ROS Standards" and "Odometry" item 4; L4-S08].
2. Publish a static transform from `base_link` to every sensor frame (URDF + robot_state_publisher) [L4-S30, "Transforms in Nav2"; L4-S16, "Coordinate Frames and Transforming Sensor Data"].
3. Fill covariances honestly at the driver; do not inflate them to hide variables — use `_config` [L4-S16, "Common errors"].
4. From wheel odometry fuse velocities (including the zero lateral velocity) [L4-S17, "Fusing Unmeasured Variables"; L4-S39, PDF pp. 17–18]; with two orientation sources, fuse both only if both report accurate covariances — if one under-reports, fuse orientation from the more accurate one and angular velocity from the other [L4-S16, "Odometry" item 1 note].
5. Give every dimension a pose or velocity reference, start with the minimum set, add inputs one at a time [L4-S39, PDF p. 17].
6. For GNSS, run one filter in `odom` without GPS for local control and one in `map` with GPS; fuse `odometry/gps` x, y with `_differential` false [L4-S18, "Notes on Fusing GPS Data" and "Configuring robot_localization"].
7. Provide an earth-referenced heading in ENU (declination, `yaw_offset`) before the navsat transform is computed; use `delay` or a fixed `datum` as needed [L4-S19; L4-S22; L4-S32, "Navsat Transform"].
8. Use `two_d_mode` only in planar environments [L4-S15, "~two_d_mode"; L4-S39, PDF p. 17].
9. Add rejection thresholds only where outliers occur, and choose them from the chi-square distribution for the fused dimension [L4-S20, comment above `odom0_pose_rejection_threshold`; L4-S45, PDF p. 7].
10. Enable `smooth_lagged_data` with a sufficient `history_length` for lagged inputs [L4-S15, "~smooth_lagged_data"].
11. Tune Q and P0 deliberately; raise Q for a variable that converges too slowly [L4-S20, comments above `process_noise_covariance` and `initial_estimate_covariance`].
12. Check consistency with NIS on logs and NEES or ATE/RPE when ground truth exists [L4-S43, PDF p. 2; L4-S50, PDF p. 2].
13. Turn on `print_diagnostics` and read `/diagnostics` when something is wrong [L4-S20, `print_diagnostics`].
14. Compute the covariance update in Joseph form (or otherwise keep P symmetric) [L4-S37, "Stable Compution of the Posterior Covariance"].
15. Check the innovations of an operating filter for zero mean and whiteness; correlated innovations mean the gain, and hence Q or R, is not right [L4-S54, PDF pp. 101, 133–134; L4-S55, PDF pp. 12–13].
16. If the filter diverges from modelling error, raise the process noise or adapt it from the innovations [L4-S54, PDF p. 144].
17. When the cross-correlation between two estimates is unknown (e.g. outputs of another filter), fuse them with a method that bounds all possible correlations, such as Covariance Intersection, rather than as independent data [L4-S56, PDF p. 1; L4-S38, "Exercise: Can you Filter GPS outputs?"].
18. Monitor the size of the covariance ellipse and the count of consecutive missing updates, and raise warnings on thresholds (automotive practice) [L4-S61, "Diagnostics"].

## Key numbers

| Quantity | Value | Conditions | Source |
|---|---|---|---|
| State size, robot_localization | 15 | current package | L4-S39, PDF p. 17 |
| State size, original paper | 12 | 2014 design | L4-S07, PDF p. 2 |
| Default Q diagonal | 0.05, 0.05, 0.06, 0.03, 0.03, 0.06, 0.025, 0.025, 0.04, 0.01, 0.01, 0.02, 0.01, 0.01, 0.015 | if parameter unset | L4-S23, lines 110–124 |
| Default P0 diagonal | 1e-9 | all 15 variables | L4-S20; L4-S23, line 91 |
| Minimum measurement variance in EKF update | 1e-9 | floor applied at update | L4-S24, lines 140–146 |
| Variance used for `two_d_mode` pseudo-measurements | 1e-6 | z, roll, pitch, their rates, az | L4-S25, lines 371–377 |
| Default rejection threshold | `numeric_limits<double>::max()` | all inputs | L4-S15; L4-S25, lines 1131–1137 |
| Default `sensor_timeout` | 1/frequency | ROS 2 code | L4-S25, line 877 |
| Chi-square gate example | 12.592 | 95 %, 6 DOF | L4-S45, PDF p. 10 |
| Q per prediction | `delta_sec × Q` | EKF predict | L4-S24, line 431 |
| NMEA default position error (std, ×HDOP) | SPS 4.0; DGPS 0.1; RTK fixed 0.02; RTK float 4.0; WAAS 3.0 | nmea_navsat_driver 2.0.1 | L4-S51, lines 59–64 |
| Standalone GPS accuracy (tutorial statement) | 1–2 m excellent, up to 10 m | standalone receiver | L4-S32, "GPS Localization Overview" |
| RTK accuracy (tutorial statement) | down to 1 cm | with RTK corrections | L4-S32, "GPS Localization Overview" |
| Max `odom` distance for cm float precision | ≈ 83 km | REP-105 | L4-S09, "odom Frame Consistency" |
| Dual-EKF example Q (x, y) | odom 1e-3; map 1.0 | robot_localization example | L4-S21 |
| Loop-closure error, odom + 2 IMU + 1 GPS | 1.21 m, 0.26 m | Pioneer 3, ~777 s | L4-S07, PDF p. 3 |
| Loop-closure error, odometry only | 69.65 m, 160.33 m | same run | L4-S07, PDF p. 3 |
| fuse vs robot_localization CPU | ≈ 3.7× | 541 m route, commercial environment | L4-S39, PDF p. 18 |
| Chi-square gate, Autoware EKF | 11.3 (3 DOF) / 9.21 (2 DOF) at 10⁻²; 49.5 / 46.1 at 10⁻¹⁰ | pose 3 DOF, twist 2 DOF | L4-S61, "3. Tune gate parameters" |
| GNSS accuracy (Autoware design doc) | ~10 m; ~10 cm with RTK | open environment | L4-S60, "GNSS" |
| Aviation hazardously-misleading-information risk budget | 10⁻⁷ to 10⁻⁹ | aviation; not set for urban use | L4-S57, PDF p. 5 |
| Autoware localization output rate | 50 Hz or more | pose/twist/accel | L4-S60, "Required Architecture" |
| Tightly coupled PPP/INS, 3 GPS satellites | 3-D error within 10 m in one minute | low-cost MEMS IMU, land vehicle | L4-S58, PDF p. 1 |

## How it is tested

| Test | What it measures | Pass criterion used in the source | Source |
|---|---|---|---|
| NEES chi-square test (Monte Carlo) | Whether P matches actual error | Average NEES near state dimension / within chi-square bounds | L4-S43, PDF pp. 2–3; L4-S14, PDF p. 11 |
| NIS chi-square test | Whether innovations match S | Within chi-square bounds with n_z DOF; usable on real logs | L4-S43, PDF p. 2 |
| Residual vs 3σ plot | Divergence, overconfidence | Residuals stay inside ±3σ | L4-S38, "Evaluating Filter Order" |
| Loop-closure (return to start) | Accumulated drift | Final estimate near origin; compare with filter σ | L4-S07, PDF p. 3 |
| ATE / RPE vs ground truth | Global error / local drift | Errors well above ground-truth accuracy to be meaningful | L4-S50, PDF pp. 2, 5 |
| Infrequent-GPS replay | Behaviour when absolute data are sparse | Covariance stays stable; estimate pulled toward fixes | L4-S07, PDF p. 5 |
| Bag replay with `reset_on_time_jump` | Repeatable offline tuning | — (tooling) | L4-S15, "~reset_on_time_jump" |
| Innovation whiteness (autocorrelation at nonzero lags) | Whether the gain is optimal (Q, R right) | Zero mean; autocorrelations ≈ 0 for all lags ≠ 0 (finite set of lags suffices for finite-dimensional models) | L4-S54, PDF pp. 101, 133–134; L4-S55, PDF pp. 12–13 |
| Design vs actual innovation statistics | Divergence | Actual innovation mean, whiteness and covariance match the design values | L4-S54, PDF p. 143 |
| Covariance-ellipse and missing-update diagnostics | Loss of absolute updates, growing uncertainty | Below configured `warn_/error_ellipse_size` and no-update counts | L4-S61, "Diagnostics" |
| Integrity check (Protection Level vs Alert Limit) | Whether the position can be trusted | PL < AL; Stanford diagram needs true error | L4-S57, PDF pp. 4–5 |

## Common mistakes

- Inflating covariances to ignore variables instead of using `_config`, which hurts robot_localization [L4-S16, "Odometry" item 3].
- Zero covariances on fused variables (replaced with a tiny value, with a WARN diagnostic, so the data are over-trusted) [L4-S16, "Common errors"; L4-S25, lines 2501–2509; L4-S24, lines 140–146].
- Fusing wheel-odometry pose, heading and velocity together, which feeds duplicate information [L4-S17, "Fusing Unmeasured Variables" item 1].
- Two absolute yaw sources with non-overlapping covariances, which makes the output oscillate [L4-S17, "The differential and relative Parameters"].
- Fusing the only yaw source differentially, so yaw variance grows without bound [L4-S17, same section].
- Fusing GPS into the `odom` filter, which breaks `odom` continuity [L4-S15, "~[frame]" item 3; L4-S18, "Notes on Fusing GPS Data"].
- Setting `_differential` true on the GPS input [L4-S18, "Configuring robot_localization"].
- NED IMU data or a heading that is zero at north without `yaw_offset` [L4-S16, "Coordinate Frames and Transforming Sensor Data"; L4-S19, "~yaw_offset"].
- Leaving a driver's `odom → base_link` broadcast on while the EKF also publishes it [L4-S16, "Odometry" item 5].
- Fusing IMU `Ÿ` acceleration on a vehicle that cannot accelerate sideways, which makes the estimate drift [L4-S17, "Fusing Unmeasured Variables" item 3].
- Large P0 on pose variables that are only fused through velocities [L4-S15, "~initial_estimate_covariance"].
- Treating a filtered device output (receiver or INS) as white measurement noise [L4-S38, "Exercise: Can you Filter GPS outputs?"].
- Using the short covariance update `(I − KH)P⁻` with a gain that is not exactly optimal, or letting P lose symmetry numerically, which can make the filter diverge [L4-S37, "Stable Compution of the Posterior Covariance"].
- Designing with very low or zero process noise, which is "typically, but not always" associated with divergence [L4-S54, PDF pp. 142–143]; a gain tending to zero is a separate warning sign that the filter may have "learned the wrong state" [L4-S54, PDF p. 143].
- Stamping messages with receive time instead of sensor time, which corrupts delay compensation [L4-S61, "0. Preliminaries"].
- A narrow chi-square gate when covariances are not well known, which rejects good data [L4-S61, "3. Tune gate parameters"].

## Disagreements between sources

- **Zero-variance replacement value.** The docs say zero variances are replaced by `1e-6` [L4-S16, "Common errors"]; the 3.5.4 EKF code floors variances at `1e-9` [L4-S24, lines 140–146]. The code (level B, current) is the implementation actually run.
- **UKF cost.** robot_localization docs say the UKF is "more computationally taxing" than the EKF [L4-S15]; Julier & Uhlmann say the UT's cost is "the same order of magnitude as the EKF" [L4-S03, PDF p. 6]. The two statements are compatible (same order, still more), but emphasise differently.
- **`publish_filtered_gps` default.** The example YAML says it "Defaults to false" [L4-S22] (the docs section gives no default [L4-S19, "~publish_filtered_gps"]); the 3.5.4 code declares it with default true [L4-S26, line 99].
- **Initial covariance comment vs value.** A code comment says the covariance should start with "large values" so measurements are accepted rapidly, but the code sets `1e-9` [L4-S23, lines 86–91]; the example config comment says large values give "rapid convergence" [L4-S20].
- **Fusing wheel-odometry pose.** robot_localization docs recommend fusing only velocities from wheel odometry [L4-S17, "Fusing Unmeasured Variables"]; Clearpath's A200 config fuses odometry x, y and yaw as well [L4-S34] (level B vs level C).
- **IMU yaw in the dual-EKF.** robot_localization's example fuses IMU roll and pitch but not absolute yaw [L4-S21]; the Nav2 GPS demo fuses absolute IMU yaw only and suggests differential mode on real robots [L4-S33].
- **Choice of gate value.** Gao et al. derive the gate from the chi-square quantile for the measurement dimension [L4-S45, PDF pp. 7, 10]; Labbe says theory's 3σ fails in practice and the value must be found experimentally [L4-S38]; the robot_localization maintainer calls the template values arbitrary [L4-S53].
- **Suitability of robot_localization in the field.** The maintainers call it "sufficient for most mobile robotics applications" [L4-S39, PDF p. 17]; the Rovitis study found it performed poorly with slipping odometry, vibrating IMU and occluded RTK [L4-S47, PDF pp. 1, 3].
- **State vector size.** 12 in the 2014 paper [L4-S07, PDF p. 2] versus 15, including linear accelerations, in the current package [L4-S39, PDF p. 17].
- **Is an `odom` frame needed?** REP-105 requires `map → odom → base_link`, with the localizer publishing `map → odom` [L4-S09, "Relationship between Frames", "Frame Authorities"] (level A); Autoware's localizer publishes `map → base_link` directly and makes `odom` optional [L4-S59, "TF tree"; L4-S60, "Output", "Kinematics Fusion Filter" and "TF tree"] (level B). Both are current; they reflect different stack designs, not a factual conflict.
- **Where `base_link` sits.** REP-105 lets the platform choose any position [L4-S09, "base_link"]; Autoware fixes it at the rear-axle centre projected to the ground [L4-S59, "TF tree"].
- **Low-rate absolute updates.** robot_localization applies each measurement in one update, so sparse GPS causes "noticeable instantaneous position changes" [L4-S07, PDF p. 5]; Autoware splits a low-rate measurement over several steps ("smooth update") to avoid large jumps [L4-S61, "Features"].
- **Gate width.** Autoware recommends a very small significance level (wide gate) because its covariances are inaccurate [L4-S61]; Labbe warns a 3σ gate discards good data [L4-S38]; the robot_localization example config recommends removing gates unless needed [L4-S20] — the sources agree gates should be loose but give no common value.

## Open questions

- Bar-Shalom, Li & Kirubarajan (L4-S05) was not available; the exact chi-square bounds for single-run and multi-run NEES/NIS tests and the numeric confidence bounds of the innovation autocorrelation test are not stated here from the primary source (the whiteness principle itself is covered by L4-S54 and L4-S55).
- Mehra 1970 (L4-S11) and Odelson et al. 2006 (ALS) were not read in the original; they are described here only through the survey in L4-S55. No study applying ALS or Mehra's method to a robot_localization-type EKF was found.
- The exact OOSM update of Bar-Shalom 2002 (L4-S12) was not read.
- Autoware's frame layout is now covered (L4-S59, L4-S60); no source was found for PAL robots or planetary rovers.
- No source here quantifies errors from `two_d_mode` on slopes or ramps; Autoware only describes the effect qualitatively (L4-S61).
- Automotive smoothing of low-rate updates (L4-S61) and GNSS integrity concepts (L4-S57) are now covered; no source here describes how agricultural systems handle a change between RTK fixed, float and standalone inside a loosely coupled EKF, or tested fallback modes for field robots.
- No source here gives a validated method to set NavSatFix covariance for RTK float vs fixed beyond driver defaults, or to gate on fix type inside robot_localization (`navsat_transform_node` only rejects NO_FIX or NaN fixes [L4-S26, lines 619–622]).
- No source here gives a tested recipe for fusing a GNSS/INS unit that outputs fused orientation and position into robot_localization without double-counting; the general principle (L4-S38) and a general method for unknown correlations (Covariance Intersection, L4-S56) are covered, and the original CI paper (Julier & Uhlmann, ACC 1997, L4-S64) was not downloaded (no open copy found).
- No NEES/NIS evaluation of robot_localization itself was found.
- Probabilistic Robotics (L4-S06) was not read.
- Groves 2013 (L4-S63), the standard GNSS/INS integration text, and Fitzgerald 1971 (L4-S65) have no open copy and were not read; divergence is covered through L4-S54, which cites Fitzgerald.

## Sources

| ID | Citation | Link | Accessed | File | Level |
|---|---|---|---|---|---|
| L4-S01 | R. E. Kalman, "A New Approach to Linear Filtering and Prediction Problems," *Trans. ASME J. Basic Engineering* 82(D), 35–45, 1960. doi:10.1115/1.3662552 (the file is a retyped reproduction, so PDF pages do not match printed pp. 35–45; cited by PDF page) | https://web.archive.org/web/2024id_/https://www.cs.unc.edu/~welch/kalman/media/pdf/Kalman1960.pdf (original UNC host now returns 404) | 2026-09-28 | sources/kalman_1960_new_approach_linear_filtering.pdf | A |
| L4-S02 | S. J. Julier, J. K. Uhlmann, "A New Extension of the Kalman Filter to Nonlinear Systems," *Proc. SPIE* 3068, 182–193, 1997. doi:10.1117/12.280797 | https://people.eecs.berkeley.edu/~pabbeel/cs287-fa19/optreadings/JulierUhlmann-UKF.pdf | 2026-09-28 | sources/julier_1997_new_extension_kalman_filter.pdf | A |
| L4-S03 | S. J. Julier, J. K. Uhlmann, "Unscented Filtering and Nonlinear Estimation," *Proc. IEEE* 92(3), 401–422, 2004. doi:10.1109/JPROC.2003.823141 | https://www.cs.ubc.ca/~murphyk/Papers/Julier_Uhlmann_mar04.pdf | 2026-09-28 | sources/julier_2004_unscented_filtering.pdf | A |
| L4-S04 | R. C. Smith, P. Cheeseman, "On the Representation and Estimation of Spatial Uncertainty," *Int. J. Robotics Research* 5(4), 56–68, 1986. doi:10.1177/027836498600500404 | https://people.csail.mit.edu/brooks/idocs/Smith_Cheeseman.pdf | 2026-09-28 | sources/smith_1986_spatial_uncertainty.pdf | A |
| L4-S05 | Y. Bar-Shalom, X. R. Li, T. Kirubarajan, *Estimation with Applications to Tracking and Navigation*, Wiley, 2001. doi:10.1002/0471221279 | https://doi.org/10.1002/0471221279 | 2026-09-28 | not downloaded (no open copy) | A |
| L4-S06 | S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics*, MIT Press, 2005. | https://mitpress.mit.edu/9780262201629/probabilistic-robotics/ | 2026-09-28 | not downloaded (no authorised open copy) | A |
| L4-S07 | T. Moore, D. Stouch, "A Generalized Extended Kalman Filter Implementation for the Robot Operating System," in *Intelligent Autonomous Systems 13*, AISC 302, Springer, 335–348, 2016 (IAS-13, 2014). doi:10.1007/978-3-319-08338-4_25. Revised author copy. | https://docs.ros.org/en/lunar/api/robot_localization/html/_downloads/robot_localization_ias13_revised.pdf | 2026-09-28 | sources/moore_2014_generalized_ekf_ros.pdf | A |
| L4-S08 | T. Foote, M. Purvis, "REP-103: Standard Units of Measure and Coordinate Conventions," ROS Enhancement Proposals (rep repo commit 11ca24a). | https://github.com/ros-infrastructure/rep/blob/11ca24a41f31480dfb9562ba99f2a5b93d3ebda5/rep-0103.rst | 2026-09-28 | sources/ros_rep103_units_coordinates.rst | A |
| L4-S09 | W. Meeussen, "REP-105: Coordinate Frames for Mobile Platforms," ROS Enhancement Proposals (commit 11ca24a). | https://github.com/ros-infrastructure/rep/blob/11ca24a41f31480dfb9562ba99f2a5b93d3ebda5/rep-0105.rst | 2026-09-28 | sources/ros_rep105_coordinate_frames.rst | A |
| L4-S10 | T. Moore, "Working with the robot_localization Package," ROSCon 2015, Hamburg (slides). | https://roscon.ros.org/2015/presentations/robot_localization.pdf | 2026-09-28 | sources/moore_2015_roscon_robot_localization.pdf | B |
| L4-S11 | R. K. Mehra, "On the Identification of Variances and Adaptive Kalman Filtering," *IEEE Trans. Automatic Control* 15(2), 175–184, 1970. doi:10.1109/TAC.1970.1099422 | https://doi.org/10.1109/TAC.1970.1099422 | 2026-09-28 | not downloaded (no open copy) | A |
| L4-S12 | Y. Bar-Shalom, "Update with Out-of-Sequence Measurements in Tracking: Exact Solution," *IEEE Trans. Aerospace and Electronic Systems* 38(3), 769–778, 2002. doi:10.1109/TAES.2002.1039398 | https://doi.org/10.1109/TAES.2002.1039398 | 2026-09-28 | not downloaded (no open copy) | A |
| L4-S13 | T. D. Larsen, N. A. Andersen, O. Ravn, N. K. Poulsen, "Incorporation of Time Delayed Measurements in a Discrete-time Kalman Filter," *Proc. 37th IEEE CDC*, 3972–3977, 1998. doi:10.1109/CDC.1998.761918 | https://orbit.dtu.dk/en/publications/incorporation-of-time-delayed-measurements-in-a-discrete-time-kal/ | 2026-09-28 | sources/larsen_1998_time_delayed_measurements.pdf | A |
| L4-S14 | G. P. Huang, A. I. Mourikis, S. I. Roumeliotis, "Observability-based Rules for Designing Consistent EKF SLAM Estimators," *Int. J. Robotics Research* 29(5), 502–528, 2010. doi:10.1177/0278364909353640 (author copy) | https://www-users.cse.umn.edu/~stergios/papers/IJRR-Observability-based-rules-consistency-2010.pdf | 2026-09-28 | sources/huang_2010_observability_consistent_ekf.pdf | A |
| L4-S15 | robot_localization, "State Estimation Nodes" (doc/state_estimation_nodes.rst), tag 3.5.4, commit 8696ee5. | https://github.com/cra-ros-pkg/robot_localization/blob/3.5.4/doc/state_estimation_nodes.rst | 2026-09-28 | sources/robotlocalization_3.5.4_state_estimation_nodes.rst | B |
| L4-S16 | robot_localization, "Preparing Your Data for Use with robot_localization" (doc/preparing_sensor_data.rst), tag 3.5.4. | https://github.com/cra-ros-pkg/robot_localization/blob/3.5.4/doc/preparing_sensor_data.rst | 2026-09-28 | sources/robotlocalization_3.5.4_preparing_sensor_data.rst | B |
| L4-S17 | robot_localization, "Configuring robot_localization" (doc/configuring_robot_localization.rst), tag 3.5.4. | https://github.com/cra-ros-pkg/robot_localization/blob/3.5.4/doc/configuring_robot_localization.rst | 2026-09-28 | sources/robotlocalization_3.5.4_configuring_robot_localization.rst | B |
| L4-S18 | robot_localization, "Integrating GPS Data" (doc/integrating_gps.rst), tag 3.5.4. | https://github.com/cra-ros-pkg/robot_localization/blob/3.5.4/doc/integrating_gps.rst | 2026-09-28 | sources/robotlocalization_3.5.4_integrating_gps.rst | B |
| L4-S19 | robot_localization, "navsat_transform_node" (doc/navsat_transform_node.rst), tag 3.5.4. | https://github.com/cra-ros-pkg/robot_localization/blob/3.5.4/doc/navsat_transform_node.rst | 2026-09-28 | sources/robotlocalization_3.5.4_navsat_transform_node.rst | B |
| L4-S20 | robot_localization, example config params/ekf.yaml, tag 3.5.4. | https://github.com/cra-ros-pkg/robot_localization/blob/3.5.4/params/ekf.yaml | 2026-09-28 | sources/robotlocalization_3.5.4_params_ekf.yaml | B |
| L4-S21 | robot_localization, example config params/dual_ekf_navsat_example.yaml, tag 3.5.4. | https://github.com/cra-ros-pkg/robot_localization/blob/3.5.4/params/dual_ekf_navsat_example.yaml | 2026-09-28 | sources/robotlocalization_3.5.4_params_dual_ekf_navsat_example.yaml | B |
| L4-S22 | robot_localization, example config params/navsat_transform.yaml, tag 3.5.4. | https://github.com/cra-ros-pkg/robot_localization/blob/3.5.4/params/navsat_transform.yaml | 2026-09-28 | sources/robotlocalization_3.5.4_params_navsat_transform.yaml | B |
| L4-S23 | robot_localization source src/filter_base.cpp, tag 3.5.4. | https://github.com/cra-ros-pkg/robot_localization/blob/3.5.4/src/filter_base.cpp | 2026-09-28 | sources/robotlocalization_3.5.4_filter_base.cpp | B |
| L4-S24 | robot_localization source src/ekf.cpp, tag 3.5.4. | https://github.com/cra-ros-pkg/robot_localization/blob/3.5.4/src/ekf.cpp | 2026-09-28 | sources/robotlocalization_3.5.4_ekf.cpp | B |
| L4-S25 | robot_localization source src/ros_filter.cpp, tag 3.5.4. | https://github.com/cra-ros-pkg/robot_localization/blob/3.5.4/src/ros_filter.cpp | 2026-09-28 | sources/robotlocalization_3.5.4_ros_filter.cpp | B |
| L4-S26 | robot_localization source src/navsat_transform.cpp, tag 3.5.4. | https://github.com/cra-ros-pkg/robot_localization/blob/3.5.4/src/navsat_transform.cpp | 2026-09-28 | sources/robotlocalization_3.5.4_navsat_transform.cpp | B |
| L4-S27 | robot_localization CHANGELOG.rst, tag 3.5.4. | https://github.com/cra-ros-pkg/robot_localization/blob/3.5.4/CHANGELOG.rst | 2026-09-28 | sources/robotlocalization_3.5.4_CHANGELOG.rst | B |
| L4-S28 | P. Bovbel, "REP-145: Conventions for IMU Sensor Drivers," ROS Enhancement Proposals, status **Draft** (not an accepted standard) (commit 11ca24a). | https://github.com/ros-infrastructure/rep/blob/11ca24a41f31480dfb9562ba99f2a5b93d3ebda5/rep-0145.rst | 2026-09-28 | sources/ros_rep145_imu_driver_conventions.rst | A |
| L4-S29 | ROS 2 common_interfaces, sensor_msgs/NavSatFix.msg and NavSatStatus.msg, tag 4.2.4. | https://github.com/ros2/common_interfaces/tree/4.2.4/sensor_msgs/msg | 2026-09-28 | sources/ros2_common_interfaces_4.2.4_NavSatFix.msg; sources/ros2_common_interfaces_4.2.4_NavSatStatus.msg | B |
| L4-S30 | Nav2 documentation, "Setting Up Transformations" (docs.nav2.org commit 588d374). | https://github.com/ros-navigation/docs.nav2.org/blob/588d37415e87eb083500d6c79aaed92ee1285f52/docs/configuration_and_development/first_time_robot_setup_guide/transformation/setup_transforms.md | 2026-09-28 | sources/nav2_docs_setup_transforms.md | B |
| L4-S31 | Nav2 documentation, "Smoothing Odometry using Robot Localization" (commit 588d374). | https://github.com/ros-navigation/docs.nav2.org/blob/588d37415e87eb083500d6c79aaed92ee1285f52/docs/configuration_and_development/first_time_robot_setup_guide/odom/setup_robot_localization.md | 2026-09-28 | sources/nav2_docs_setup_robot_localization.md | B |
| L4-S32 | P. Gonzalez (Kiwibot), "Navigating using GPS Localization," Nav2 documentation (commit 588d374). | https://github.com/ros-navigation/docs.nav2.org/blob/588d37415e87eb083500d6c79aaed92ee1285f52/docs/tutorials/general_tutorials/navigation2_with_gps/navigation2_with_gps.md | 2026-09-28 | sources/nav2_docs_navigation2_with_gps.md | B |
| L4-S33 | navigation2_tutorials, nav2_gps_waypoint_follower_demo/config/dual_ekf_navsat_params.yaml (commit 9f58746). | https://github.com/ros-navigation/navigation2_tutorials/blob/9f587464617d2939e80d65f2849203207a9e328e/nav2_gps_waypoint_follower_demo/config/dual_ekf_navsat_params.yaml | 2026-09-28 | sources/nav2tutorials_gps_demo_dual_ekf_navsat_params.yaml | B |
| L4-S34 | Clearpath Robotics, clearpath_common, clearpath_control/config/a200/localization.yaml, tag 1.3.9. | https://github.com/clearpathrobotics/clearpath_common/blob/1.3.9/clearpath_control/config/a200/localization.yaml | 2026-09-28 | sources/clearpath_common_a200_localization.yaml | C |
| L4-S35 | G. Welch, G. Bishop, "An Introduction to the Kalman Filter," Tech. Rep. TR 95-041, UNC Chapel Hill, updated 24 July 2006. | https://web.archive.org/web/2024id_/https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf (original UNC host now returns 404) | 2026-09-28 | sources/welch_2006_intro_kalman_filter.pdf | C |
| L4-S36 | P. S. Maybeck, *Stochastic Models, Estimation, and Control*, vol. 1, ch. 1 "Introduction," Academic Press, 1979 (reproduced by permission). | https://web.archive.org/web/2024id_/https://www.cs.unc.edu/~welch/kalman/media/pdf/maybeck_ch1.pdf (original UNC host now returns 404) | 2026-09-28 | sources/maybeck_1979_stochastic_models_ch1.pdf | A |
| L4-S37 | R. R. Labbe, *Kalman and Bayesian Filters in Python*, ch. 7 "Kalman Filter Math" (commit 04b2bea; markdown and code cells extracted). | https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/04b2bea802321086effbd99402fc13c893d11110/07-Kalman-Filter-Math.ipynb | 2026-09-28 | sources/labbe_kbfp_07_kalman_filter_math.md | C |
| L4-S38 | R. R. Labbe, *Kalman and Bayesian Filters in Python*, ch. 8 "Designing Kalman Filters" (commit 04b2bea; markdown and code cells extracted). | https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/04b2bea802321086effbd99402fc13c893d11110/08-Designing-Kalman-Filters.ipynb | 2026-09-28 | sources/labbe_kbfp_08_designing_kalman_filters.md | C |
| L4-S39 | S. Macenski, T. Moore, D. V. Lu, A. Merzlyakov, M. Ferguson, "From the Desks of ROS Maintainers: A Survey of Modern & Capable Mobile Robotics Algorithms in the Robot Operating System 2," *Robotics and Autonomous Systems* 168, 104493, 2023. doi:10.1016/j.robot.2023.104493 (open arXiv copy 2307.15236v2) | https://arxiv.org/abs/2307.15236 | 2026-09-28 | sources/macenski_2023_desks_of_ros_maintainers.pdf | A |
| L4-S40 | Locus Robotics, fuse README.md, release tag 1.3.4 (commit 40e31ec, also the head of the `rolling` branch); the README states the ROS 2 port is a work in progress and "not expected to work" yet. | https://github.com/locusrobotics/fuse/blob/40e31ec80dba2295f9c12b51313c83704295b919/README.md | 2026-09-28 | sources/locusrobotics_fuse_README.md | B |
| L4-S41 | A. Barrau, S. Bonnabel, "The Invariant Extended Kalman Filter as a Stable Observer," *IEEE Trans. Automatic Control* 62(4), 1797–1812, 2017. doi:10.1109/TAC.2016.2594085 (open arXiv copy 1410.1465v4) | https://arxiv.org/abs/1410.1465 | 2026-09-28 | sources/barrau_2017_invariant_ekf_stable_observer.pdf | A |
| L4-S42 | J. Solà, "Quaternion kinematics for the error-state Kalman filter," arXiv:1711.02508, 2017 (no peer-reviewed version). | https://arxiv.org/abs/1711.02508 | 2026-09-28 | sources/sola_2017_quaternion_kinematics_eskf.pdf | C |
| L4-S43 | Z. Chen, C. Heckman, S. Julier, N. Ahmed, "Weak in the NEES?: Auto-tuning Kalman Filters with Bayesian Optimization," *Proc. 21st Int. Conf. Information Fusion (FUSION)*, 2018. doi:10.23919/ICIF.2018.8455782 (open arXiv copy 1807.08855v1) | https://arxiv.org/abs/1807.08855 | 2026-09-28 | sources/chen_2018_weak_in_the_nees.pdf | A |
| L4-S44 | C. Jiang, S.-B. Zhang, "A Novel Adaptively-Robust Strategy Based on the Mahalanobis Distance for GPS/INS Integrated Navigation Systems," *Sensors* 18(3), 695, 2018. doi:10.3390/s18030695 | https://www.mdpi.com/1424-8220/18/3/695 | 2026-09-28 | sources/jiang_2018_adaptively_robust_mahalanobis_gps_ins.pdf | A |
| L4-S45 | B. Gao, G. Hu, X. Zhu, Y. Zhong, "A Robust Cubature Kalman Filter with Abnormal Observations Identification Using the Mahalanobis Distance Criterion for Vehicular INS/GNSS Integration," *Sensors* 19(23), 5149, 2019. doi:10.3390/s19235149 | https://www.mdpi.com/1424-8220/19/23/5149 | 2026-09-28 | sources/gao_2019_robust_ckf_mahalanobis_ins_gnss.pdf | A |
| L4-S46 | G. Dissanayake, S. Sukkarieh, E. Nebot, H. Durrant-Whyte, "The Aiding of a Low-Cost Strapdown Inertial Measurement Unit Using Vehicle Model Constraints for Land Vehicle Applications," *IEEE Trans. Robotics and Automation* 17(5), 731–747, 2001. doi:10.1109/70.964672 | https://doi.org/10.1109/70.964672 | 2026-09-28 | sources/dissanayake_2001_vehicle_model_constraints.pdf | A |
| L4-S47 | J. Rakun, M. Pantano, P. Lepej, M. Lakota, "Sensor fusion-based approach for the field robot localization on Rovitis 4.0 vineyard robot," *Int. J. Agric. & Biol. Eng.* 15(6), 91–95, 2022. doi:10.25165/j.ijabe.20221506.6415 | https://doi.org/10.25165/j.ijabe.20221506.6415 | 2026-09-28 | sources/rakun_2022_rovitis_vineyard_fusion.pdf | A |
| L4-S48 | N. Sünderhauf, M. Obst, G. Wanielik, P. Protzel, "Multipath Mitigation in GNSS-based Localization using Robust Optimization," *Proc. IEEE Intelligent Vehicles Symposium*, 2012. doi:10.1109/IVS.2012.6232299 (author copy) | https://nikosuenderhauf.github.io/assets/papers/IV12-multipathMitigation.pdf | 2026-09-28 | sources/sunderhauf_2012_gnss_multipath_robust_optimization.pdf | A |
| L4-S49 | N. Sünderhauf, S. Lange, P. Protzel, "Incremental Sensor Fusion in Factor Graphs with Unknown Delays," *Proc. ESA Symposium on Advanced Space Technologies in Robotics and Automation (ASTRA)*, 2013 (author copy; ESA symposium with abstract-based selection, so not graded as peer-reviewed). | https://nikosuenderhauf.github.io/assets/papers/suenderhauf13delays.pdf | 2026-09-28 | sources/sunderhauf_2013_factor_graph_unknown_delays.pdf | C |
| L4-S50 | J. Sturm, N. Engelhard, F. Endres, W. Burgard, D. Cremers, "A Benchmark for the Evaluation of RGB-D SLAM Systems," *Proc. IEEE/RSJ IROS*, 2012. doi:10.1109/IROS.2012.6385773 (author copy) | https://cvg.cit.tum.de/_media/spezial/bib/sturm12iros.pdf | 2026-09-28 | sources/sturm_2012_tum_rgbd_benchmark.pdf | A |
| L4-S51 | ros-drivers/nmea_navsat_driver, src/libnmea_navsat_driver/driver.py, tag 2.0.1 (commit 861323c). | https://github.com/ros-drivers/nmea_navsat_driver/blob/2.0.1/src/libnmea_navsat_driver/driver.py | 2026-09-28 | sources/nmea_navsat_driver_2.0.1_driver.py | C |
| L4-S52 | cra-ros-pkg/robot_localization issue #417, "Feature request: discard erroneous measurements," with maintainer (T. Moore) reply, 2018. | https://github.com/cra-ros-pkg/robot_localization/issues/417 | 2026-09-28 | sources/rl_issue_417.md | D |
| L4-S53 | cra-ros-pkg/robot_localization issue #630, "pose_rejection_threshold documention unclear," with maintainer (T. Moore) reply, 2021–2022. | https://github.com/cra-ros-pkg/robot_localization/issues/630 | 2026-09-28 | sources/rl_issue_630.md | D |
| L4-S54 | B. D. O. Anderson, J. B. Moore, *Optimal Filtering*, Prentice-Hall, 1979 (scanned copy hosted on the authors' ANU page; 367 PDF pages). | https://users.cecs.anu.edu.au/~john/papers/BOOK/B02.PDF | 2026-09-28 | sources/anderson_1979_optimal_filtering.pdf | A |
| L4-S55 | L. Zhang, D. Sidoti, A. Bienkowski, K. R. Pattipati, Y. Bar-Shalom, D. L. Kleinman, "On the Identification of Noise Covariances and Adaptive Kalman Filtering: A New Look at a 50 Year-Old Problem," *IEEE Access* 8, 59362–59388, 2020. doi:10.1109/ACCESS.2020.2982407 (open-access author manuscript NIHMS1724120 from PMC, PMC8638515, CC BY, 75 PDF pages; cited by PDF page) | https://pmc-oa-opendata.s3.amazonaws.com/PMC8638515.1/PMC8638515.1.pdf | 2026-09-28 | sources/zhang_2020_noise_covariance_identification.pdf | A |
| L4-S56 | J. Ajgl, M. Šimandl, M. Reinhardt, B. Noack, U. D. Hanebeck, "Covariance Intersection in State Estimation of Dynamical Systems," *Proc. 17th Int. Conf. Information Fusion (FUSION)*, 2014 (author copy, KIT ISAS). | https://isas.iar.kit.edu/pdf/Fusion14_Ajgl.pdf | 2026-09-28 | sources/ajgl_2014_covariance_intersection_dynamical_systems.pdf | A |
| L4-S57 | N. Zhu, J. Marais, D. Bétaille, M. Berbineau, "GNSS Position Integrity in Urban Environments: A Review of Literature," *IEEE Trans. Intelligent Transportation Systems* 19(9), 2762–2778, 2018. doi:10.1109/TITS.2017.2766768 (HAL author copy hal-01709519; PDF p. 1 is the HAL cover, so PDF p. N = printed p. N−1) | https://hal.science/hal-01709519 | 2026-09-28 | sources/zhu_2018_gnss_integrity_urban_review.pdf | A |
| L4-S58 | Y. Liu, F. Liu, Y. Gao, L. Zhao, "Implementation and Analysis of Tightly Coupled Global Navigation Satellite System Precise Point Positioning/Inertial Navigation System (GNSS PPP/INS) with Insufficient Satellites for Land Vehicle Navigation," *Sensors* 18(12), 4305, 2018. doi:10.3390/s18124305 | https://doi.org/10.3390/s18124305 | 2026-09-28 | sources/liu_2018_tightly_coupled_ppp_ins_land_vehicle.pdf | A |
| L4-S59 | Autoware Foundation, "Coordinate system," Autoware Documentation (autoware-documentation commit f43b960). | https://github.com/autowarefoundation/autoware-documentation/blob/f43b9606771ec6badf51d03131d73c0f7b708049/docs/contributing/coding-guidelines/ros-nodes/coordinate-system.md | 2026-09-28 | sources/autoware_docs_coordinate_system.md | B |
| L4-S60 | Autoware Foundation, "Localization component design doc," Autoware Documentation, *architecture v1* (legacy design, superseded by the current Autoware architecture) (commit f43b960). | https://github.com/autowarefoundation/autoware-documentation/blob/f43b9606771ec6badf51d03131d73c0f7b708049/docs/design/autoware-architecture-v1/components/localization/index.md | 2026-09-28 | sources/autoware_docs_localization_design.md | B |
| L4-S61 | Autoware Foundation, autoware_core, `localization/autoware_ekf_localizer/README.md`, tag 1.9.0 (commit f25f83c). | https://github.com/autowarefoundation/autoware_core/blob/1.9.0/localization/autoware_ekf_localizer/README.md | 2026-09-28 | sources/autoware_core_1.9.0_ekf_localizer_README.md | B |
| L4-S62 | Autoware Foundation, autoware_core, `sensing/autoware_gnss_poser/README.md`, tag 1.9.0 (commit f25f83c). | https://github.com/autowarefoundation/autoware_core/blob/1.9.0/sensing/autoware_gnss_poser/README.md | 2026-09-28 | sources/autoware_core_1.9.0_gnss_poser_README.md | B |
| L4-S63 | P. D. Groves, *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*, 2nd ed., Artech House, 2013. | https://us.artechhouse.com/Principles-of-GNSS-Inertial-and-Multisensor-Integrated-Navigation-Systems-Second-Edition-P1574.aspx | 2026-09-28 | not downloaded (no open copy) | A |
| L4-S64 | S. J. Julier, J. K. Uhlmann, "A Non-divergent Estimation Algorithm in the Presence of Unknown Correlations," *Proc. American Control Conference*, 1997. doi:10.1109/ACC.1997.609105 | https://doi.org/10.1109/ACC.1997.609105 | 2026-09-28 | not downloaded (no open copy) | A |
| L4-S65 | R. J. Fitzgerald, "Divergence of the Kalman Filter," *IEEE Trans. Automatic Control* 16(6), 736–747, 1971. doi:10.1109/TAC.1971.1099836 | https://doi.org/10.1109/TAC.1971.1099836 | 2026-09-28 | not downloaded (no open copy) | A |
