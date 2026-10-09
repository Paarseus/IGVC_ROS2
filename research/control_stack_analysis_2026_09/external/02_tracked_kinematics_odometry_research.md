# External Research — Tracked / Skid-Steer Kinematics, Odometry and Rate Limiting

Collected 2026-09-25. Items the research could not confirm are marked UNVERIFIED.

## 1. Skid-steer and tracked kinematics (ICR models)

**Mandow et al. 2007, IROS, "Experimental kinematics for wheeled skid-steer mobile robots"** ([PDF](http://babel.isa.uma.es/_utils/downloads/logdownloads.php?u=jafma&t=pdf&f=downloads/jafma/papers/mandow2007ekw.pdf)).
- Model: tread ICR lateral positions x_ICRl and x_ICRr, vehicle ICR longitudinal offset y_ICRv, per-side speed factors α_l and α_r. The five parameters are fitted by a genetic algorithm against RTK-DGPS ground truth.
- Steering efficiency χ = L / (x_ICRr − x_ICRl), 0 < χ ≤ 1 (χ = 1 is an ideal differential drive). In `diff_drive_controller` terms the multiplier ≈ 1/χ. Eccentricity e = (x_ICRr + x_ICRl)/(x_ICRr − x_ICRl).
- Quick symmetric calibration (their eqs. 11, 12): spin in place, x_ICR ≈ (∫V_r dt − ∫V_l dt)/(2φ); drive straight, α ≈ 2d/(∫V_r dt + ∫V_l dt).
- Pioneer P3-AT (L = 0.4 m, 23.6 kg):

  | Surface | χ | Multiplier ≈ 1/χ | α |
  |---|---|---|---|
  | Asphalt | 0.695–0.761 | 1.31–1.44 | 0.905–0.946 |
  | Smooth concrete | 0.705–0.746 | 1.34–1.42 | 0.905–0.946 |

  y_ICRv ≈ 1 cm behind the origin in all runs (depends on centre of mass); e ≈ 0. "The loss of thrust power while turning is greater in asphalt than in concrete due to greater friction… observed as a lower steering efficiency value χ."
- The same ICR set is used for dead-reckoning and for the inverse (their eqs. 16, 17): symmetric use.

**Martínez et al. 2005, IJRR 24(10):867–878, "Approximating kinematics for tracked mobile robots"** ([journal](https://journals.sagepub.com/doi/10.1177/0278364905058239)). Tread ICRs "are dynamics-dependent, but they lie within a bounded area"; constant ICRs optimized per terrain give an approximate kinematic model, validated for online odometry and low-level motion control on the tracked Auriga-α. Numeric values UNVERIFIED (full text not accessed).

**Yu, Chuy, Collins, Hollis** (skid-steer dynamics and power) ([ResearchGate](https://www.researchgate.net/publication/221918595)). Expansion factor grows with rolling resistance; P3-AT α ≈ 1.5 on vinyl, > 2 on concrete. Attribution of these numbers UNVERIFIED (search summary).

**Wang et al. 2015, P3-AT with laser scanner** ([PMC4481911](https://pmc.ncbi.nlm.nih.gov/articles/PMC4481911/)). Fitted χ(λ) = 1 + 0.4728/(1 + 0.0538·|λ|^½), λ = (V_l+V_r)/(V_l−V_r). χ ≥ 1 in their convention; mean 1.41–1.47. The effective track shrinks as the turning radius grows; it is largest when spinning in place.

**Pentzer, Brennan, Reichard 2014, JFR** ([Wiley](https://onlinelibrary.wiley.com/doi/abs/10.1002/rob.21509)). EKF estimates track ICRs online; ICR odometry on a tracked robot had −0.42 m mean error over 40.5 m.

**Wang et al. 2018, Complexity** ([Wiley](https://onlinelibrary.wiley.com/doi/10.1155/2018/4816712)). Terrain-adaptive EKF re-estimates ICRs; terrain classified from accelerometer vibration, process noise switched per terrain.

**Baril et al. 2020, CRV** ([arXiv](https://arxiv.org/abs/2004.05131), [project](https://norlab.ulaval.ca/publications/subartic_kinematic_modelling/)). 590 kg skid-steer, > 2 km on snow and concrete; compared ideal differential drive, extended differential drive, radius-of-curvature and full-linear models per surface. Extended differential drive with 5 parameters performed best (search summary; parameter values UNVERIFIED).

Variation: higher friction or rolling resistance → lower χ (larger multiplier); tighter turns → larger effective track; speed and load (centre of mass, pressure) shift ICRs. No measured grass-vs-asphalt ICR values for tracked robots were found (UNVERIFIED); the friction trend predicts a larger multiplier on grass than on smooth concrete.

## 2. `diff_drive_controller` applies the multiplier to both paths

Humble source ([diff_drive_controller.cpp](https://github.com/ros-controls/ros2_controllers/blob/humble/diff_drive_controller/src/diff_drive_controller.cpp)):
```cpp
// odometry (update(), on_configure)
const double wheel_separation = params_.wheel_separation_multiplier * params_.wheel_separation;
odometry_.setWheelParams(wheel_separation, left_wheel_radius, right_wheel_radius);
// command
const double velocity_left  = (linear_command - angular_command * wheel_separation / 2.0) / left_wheel_radius;
const double velocity_right = (linear_command + angular_command * wheel_separation / 2.0) / right_wheel_radius;
```
ROS 1 `ros_controllers` (noetic-devel) does the same ([source](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/diff_drive_controller/src/diff_drive_controller.cpp)). Parameter description: "Correction factor when the actual wheel separation differs from the nominal value" ([parameter yaml](https://github.com/ros-controls/ros2_controllers/blob/humble/diff_drive_controller/src/diff_drive_controller_parameter.yaml)). There is no separate command and odometry multiplier.

Our command-only use has no precedent in the sources found. It is internally consistent only if 1.19 is read as ω feedforward for motor under-delivery (which the encoders already measure) on top of a small chassis skid (α ≈ 0.96, so an odometry-side value of ~1.04 rather than 1.0).

## 3. Odometry error and EKF covariance

- **UMBmark** (Borenstein and Feng, 1995–96; [paper 60](https://johnloomis.org/ece445/topics/odometry/borenstein/paper60.pdf), [paper 58](https://cs.au.dk/~ocaprani/legolab/DigitalControl.dir/NXT/Lesson10.dir/paper58.pdf)): square path driven clockwise and counter-clockwise; separates systematic errors (Ed wheel-diameter ratio, Eb wheelbase) from non-systematic ones; reports at least an order-of-magnitude reduction in systematic error. For skid-steer, Eb ≈ the ICR expansion factor; slip on tracked robots is largely non-systematic and surface-specific.
- **robot_localization** ([configuring_robot_localization.rst](https://github.com/cra-ros-pkg/robot_localization/blob/ros2/doc/configuring_robot_localization.rst)): if pose and velocity all come from encoders, "it's best to just use the velocities"; fuse the zero ẏ that nonholonomic odometry reports unless its covariance is inflated; with N orientation sources, make N − 1 differential.
- **Yaw-rate covariance from a square test:** convert integrated odometry yaw error into a variance for angular velocity ([ROS Answers 311304](https://answers.ros.org/question/311304/); Tom Moore attribution UNVERIFIED).
- **Inflating yaw-rate variance during turns** is physically justified by ICR data; no standard ROS implementation found (UNVERIFIED). Wang 2018's terrain-adaptive process noise is the closest analogue.

## 4. Where heading/yaw control belongs

- Martínez 2005 and Pentzer, Brennan, Reichard 2014 IROS ([paper](https://pure.psu.edu/en/publications/the-use-of-unicycle-robot-control-strategies-for-skid-steer-robot/)) put the slip model in the low-level kinematics and run unicycle path-tracking controllers above it. Huskić et al. 2019 IJRR ([paper](https://journals.sagepub.com/doi/abs/10.1177/0278364919859634)) does high-speed skid-steer path following. None adds an IMU heading-hold inside the actuator (weak evidence; consensus UNVERIFIED).
- **Nav2 issue #5524** ([issue](https://github.com/ros-navigation/navigation2/issues/5524)) is about OPEN_LOOP vs CLOSED_LOOP feedback in the velocity smoother and controller server. It does not say closed-loop ω correction belongs at the EKF or controller rather than the actuator. What it does say: DWB and MPPI "consider the dynamic limitations from the current speed when calculating the next trajectory"; real robots usually publish odometry at ≥ 100 Hz.

## 5. Reference robot parameters

| Robot / source | Separation (m) | Multiplier | Twist covariance diag | EKF inputs |
|---|---|---|---|---|
| Husky ROS 1 ([control.yaml](https://github.com/husky/husky/blob/melodic-devel/husky_control/config/control.yaml)) | URDF | 1.875 | [0.001×5, 0.03] | wheel odometry via robot_localization |
| Husky ROS 2 humble-devel ([control.yaml](https://github.com/husky/husky/blob/humble-devel/husky_control/config/control.yaml), [localization.yaml](https://github.com/husky/husky/blob/humble-devel/husky_control/config/localization.yaml)) | 0.512 | 1.0 | [0.001×5, 0.01] | odom vx, vy, vz, vyaw; IMU roll/pitch/yaw + rates, `imu0_differential: true` |
| Clearpath A200 ([control.yaml](https://github.com/clearpathrobotics/clearpath_common/blob/humble/clearpath_control/config/a200/control.yaml), [localization.yaml](https://github.com/clearpathrobotics/clearpath_common/blob/humble/clearpath_control/config/a200/localization.yaml)) | 0.555 | 1.875 | [0.001×5, 0.01] | odom x, y, yaw, vx, vy, vyaw (IMU added by generator, UNVERIFIED) |
| Clearpath J100 Jackal ([control.yaml](https://github.com/clearpathrobotics/clearpath_common/blob/humble/clearpath_control/config/j100/control.yaml); ROS 1 [robot_localization.yaml](https://github.com/jackal/jackal/blob/melodic-devel/jackal_control/config/robot_localization.yaml)) | 0.3756 | 1.5 | [0.001×5, 0.01] | odom vx, vy, vz, vyaw; IMU orientation + rates, absolute |
| Clearpath W200 Warthog ([control.yaml](https://github.com/clearpathrobotics/clearpath_common/blob/humble/clearpath_control/config/w200/control.yaml)) | 1.5 | 1.125 | [0.001×5, 0.01] | — |

Clearpath ROS 2 uses `preserve_turning_radius: true`. Acceleration limits: Husky ROS 1 3.0 m/s², 6.0 rad/s²; A200 1.0 m/s², 1.0 rad/s². All use `diff_drive_controller` (symmetric multiplier). No open-source tracked robot with published multiplier and covariance values was found (UNVERIFIED).

FRC: WPILib `DifferentialDriveOdometry` uses the gyro angle plus left/right distances ([docs](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-odometry.html)); characterization measures an effective track width with a gyro ([docs](https://docs.wpilib.org/en/2021/docs/software/wpilib-tools/robot-characterization/introduction.html)).

## 6. Slew and acceleration limiting

- Nav2 `velocity_smoother` supports separate `max_accel` and `max_decel`; OPEN_LOOP is the default and is "a good assumption" when limits are set appropriately; CLOSED_LOOP needs high-rate, low-latency odometry ([docs](https://github.com/ros-navigation/docs.nav2.org/blob/rolling/docs/configuration_and_development/configuration_guide/core_servers/configuring_velocity_smoother.md)).
- `diff_drive_controller` also limits acceleration and jerk inside the base controller.
- **MPPI on Humble does not read `ax_max`, `ax_min` or `az_max`.** Humble `optimizer.cpp::getParams()` reads only velocity limits and sampling std ([source](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/optimizer.cpp)); acceleration constraints came in [PR #4352](https://github.com/ros-navigation/navigation2/pull/4352) (merged to main 2024-06-04). Confirmed on our Jetson binary: `evidence/live_measurements/mppi_accel_params_check_2026_09_25.txt`.
- Rolling MPPI documentation notes: `clamp_raw_controls` "may cause issues if `ax_max` && `ax_min` are very asymmetric"; odometry should publish at least as fast as the control frequency; `model_dt` should equal the control period ([docs](https://github.com/ros-navigation/docs.nav2.org/blob/rolling/docs/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic.md)).
