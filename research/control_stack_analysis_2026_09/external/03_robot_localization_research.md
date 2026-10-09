# External Research — robot_localization Dual EKF, navsat_transform, INS Integration

Collected 2026-09-25. Primary sources: robot_localization `humble-devel` source and docs, the Nav2 GPS tutorial, the Xsens MTi Family Reference Manual, and the vendored `xsens_mti_ros2_driver` source. docs.ros.org and answers.ros.org were blocked; the ROSCon 2015 slides timed out. Items not read directly are marked UNVERIFIED.

## 1. Canonical dual-EKF + navsat_transform pattern

- `doc/integrating_gps.rst` ([source](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/integrating_gps.rst)): a GPS-fused pose "will likely be unfit for use by navigation modules, owing to … discrete discontinuities ('jumps')". Recommended: one instance fusing only continuous data in `odom`, where local plans execute; one fusing everything including GPS in `map`.
- GPS fusion: "You should have no need to modify the `_differential` setting … The GPS is an absolute position sensor, and enabling differential integration defeats the purpose of using it." `navsat_transform_node.rst` also says to keep `odomN_differential` false.
- Reference configs: robot_localization [`params/dual_ekf_navsat_example.yaml`](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/params/dual_ekf_navsat_example.yaml) and the [Nav2 tutorial yaml](https://github.com/ros-navigation/navigation2_tutorials/blob/rolling/nav2_gps_waypoint_follower_demo/config/dual_ekf_navsat_params.yaml):

  | | Odom EKF | Map EKF |
  |---|---|---|
  | Frequency | 30 Hz | 30 Hz |
  | Wheel odometry | vx, vy, vz, vyaw | same |
  | IMU (Nav2 yaml) | yaw only, absolute | yaw only, absolute |
  | GPS | — | x, y only, absolute |
  | Q x/y | 1e-3 | 1.0 |
  | Q yaw | 0.01 | 0.01 |
  | Q vx/vy | 0.5 | 0.5 |

  Absolute IMU yaw in both EKFs is the canonical pattern; GPS belongs only in the map EKF. The Nav2 yaml comment on `imu0_differential`: "If using a real robot you might want to set this to true, since usually absolute measurements from real imu's are not very accurate."
- Nav2 tutorial page ([docs](https://docs.nav2.org/jazzy/tutorials/general_tutorials/navigation2_with_gps/navigation2_with_gps/)) assumes an IMU with "zero yaw when facing east" and warns that "commercial grade IMU's … will often not produce accurate absolute heading".
- Multiple orientation sources (`doc/preparing_sensor_data.rst`): fuse orientation from both only if both report accurate covariance; otherwise fuse only the better one or make the other differential. `doc/configuring_robot_localization.rst`: with N orientation sources, N − 1 may be differential; a differential orientation's variance "will grow without bound". Runtime warning "N absolute pose inputs detected … may result in oscillations" ([ros_filter.cpp](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/src/ros_filter.cpp) ~L1713).

## 2. Differential vs absolute; the differential covariance formula

- Differential GPS turns the map EKF into dead reckoning (unbounded x/y covariance, no earth anchoring).
- Differential measurements use variance `(cov_t + cov_{t−1}) · dt` (`ros_filter.cpp` ~L3075) instead of the dimensionally correct `/dt²`. Known open issue [#356](https://github.com/cra-ros-pkg/robot_localization/issues/356); the maintainer kept it because the correct form converged too slowly. Differentially fused pose sources (our ZED yaw) are weighted far more heavily than their noise justifies.
- Absolute IMU yaw in the odom EKF passes INS heading re-convergence steps into `odom→base_link` (REP-105 continuity). Shared IMU error affects both frames identically.

## 3. Process noise and initial covariance

- Q is a rate: prediction does `P += Δt · Q` ([ekf.cpp](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/src/ekf.cpp) ~L431).
- Default initial P = 1e-9·I ([filter_base.cpp](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/src/filter_base.cpp) L90–91). The first measurement of each sensor initializes only that sensor's fused variables.
- `dynamic_process_noise_covariance` scales pose Q by ‖twist‖² (filter_base.cpp L142–153): pose Q = 0 at standstill.
- Validation: NIS (`νᵀS⁻¹ν` should be χ² with m DOF, mean/m ≈ 1; > 1 = overconfident) ([reference](https://kalman-filter.com/normalized-innovation-squared/)); NEES against RTK FIXED truth; automated χ²-consistent tuning ([arXiv 2306.07225](https://arxiv.org/abs/2306.07225)). robot_localization has no NIS topic; use `debug: true` logging or recompute from bags. FusionCore ships `nis_from_bag.py` ([repo](https://github.com/manankharwar/fusioncore)).

## 4. Mahalanobis rejection thresholds

- **Units: Mahalanobis distance (σ), not squared.** `checkMahalanobisThreshold`: `squared_mahalanobis = innovation.dot(innovation_covariance * innovation); threshold = n_sigmas * n_sigmas; if (squared_mahalanobis >= threshold) reject` ([filter_base.cpp](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/src/filter_base.cpp)). Default is no gate; the template says to remove the parameters if not required.
- The gate applies jointly to the fused subset; on failure the whole correction is skipped. Odometry messages have separate pose and twist gates; Imu messages have orientation, angular-velocity and acceleration gates.
- Correct value = √(χ² quantile). 2 DOF at 99.9%: χ² = 13.82 → **3.72**. FusionCore's robot_localization baseline uses exactly this ([config](https://github.com/manankharwar/fusioncore/blob/main/benchmarks/comparison/rl_ekf_outdoor.yaml)). The reverse unit confusion exists in Autoware's `ekf_localizer` ([issue 1464](https://github.com/autowarefoundation/autoware_core/issues/1464)).
- Lock-out mechanism: a rejected state is readmitted only as P grows by Q·Δt. Tiny Q, dynamic process noise at rest, or overconfident R make recovery slow or impossible. Classic case is start-up with P = 1e-9 and position initialized away from the first GPS fix. Mitigations: large `initial_estimate_covariance` on x/y; Q_pose > 0; relax the gate after a genuine gap; adaptive R; or no gate with upstream outlier filtering.

## 5. navsat_transform

- Heading must be ENU (0 = east, CCW positive); `imu_yaw += magnetic_declination + yaw_offset + utm_meridian_convergence` ([navsat_transform.cpp](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/src/navsat_transform.cpp) ~L291).
- **Xsens:** "the yaw output is 0º when the vehicle … is pointing East"; heading is corrected for declination and "referenced to 'local' True North" once a GNSS fix is available; "the magnetometer data is only actively used in the GeneralMag filter profile" ([MTi Family Reference Manual](https://www.xsens.com/hubfs/Downloads/Manuals/MTi_familyreference_manual.pdf)). With General_RTK, `magnetic_declination_radians: 0` and `yaw_offset: 0` are correct. That General_RTK yaw starts at 0 and converges only with motion is UNVERIFIED (search summary) but matches the team's observations.
- **Datum:** exactly three elements `[lat_deg, lon_deg, heading_rad]` in ROS 2 (L148–171); any other length silently becomes 0,0,0.
- **With `wait_for_datum: true` the IMU subscription is never created** (L186). The transform heading comes from the datum heading (plus declination, offset, meridian convergence). Datum heading 0 means map +x = east, which is correct because the EKFs fuse true-ENU IMU yaw.
- `broadcast_cartesian_transform` publishes `world_frame → utm/local_enu`, not `map→odom`, so it cannot create a TF loop.
- **Antenna offset is removed only via TF:** `getRobotOriginWorldPose()` uses `base_link → <NavSatFix.header.frame_id>` (L564–603).

## 6. INS output as EKF input

- GNSS/INS output is time-correlated; a Kalman filter treating correlated samples as independent becomes overconfident ("9–12× at 10 Hz"); proposed fix scales R by correlation time ([PX4 RFC 28837](https://github.com/PX4/PX4-Autopilot/issues/28837)). robot_localization "assumes independence between the measurements" (issue #356).
- Common patterns: fuse INS orientation and gyro plus raw GNSS position (ours), or use the INS pose directly ([Autoware discussion 3095](https://github.com/orgs/autowarefoundation/discussions/3095)). Zero INS orientation covariance causes numerical problems ([Autoware discussion 4667](https://github.com/orgs/autowarefoundation/discussions/4667)).

## 7. Xsens driver covariances (vendored source)

- `imupublisher.h`: `orientation_stddev`, `angular_velocity_stddev`, `linear_acceleration_stddev` default to `[0,0,0]`; the template yaml ships zeros. Device-reported orientation std is used only for Avior/Sirius devices with `pub_euler_stddev` configured during device configuration. Confirmed on our robot: all-zero covariances (`evidence/live_measurements/static_2026_09_25/SUMMARY.txt`).
- robot_localization clamps variances below 1e-9 to 1e-9 (ekf.cpp L140–146) and emits a "was zero … replaced with a small value" diagnostic.
- `/gnss` covariance is `hAcc²`/`vAcc²` from the u-blox PVT (`gnsspublisher.h` L67–74).

## 8. Published configurations (IMU + wheel + GPS)

| Project | Key settings |
|---|---|
| robot_localization example / Nav2 tutorial | 30 Hz dual; wheel velocities + IMU; GPS x,y absolute in map EKF; no gates; Q x,y 1e-3 / 1.0 |
| [nickcharron/waypoint_nav](https://github.com/nickcharron/waypoint_nav/tree/master/outdoor_waypoint_nav/params) (Husky, Novatel) | 30 Hz dual; IMU roll/pitch + rates + accel (no absolute yaw); GPS absolute; `yaw_offset −π/2`, declination 0.0842 |
| [RoboticsClubatUCF/AGV](https://github.com/RoboticsClubatUCF/AGV/blob/master/ugv_nav/config/robot_localization/dual_ekf_navsat.yaml) (IGVC) | ZED odom pose + 2 VectorNav IMUs, all absolute; IMU gates 0.8; GPS absolute, no gate; `yaw_offset π/2` |
| [KTH-SML/svea](https://github.com/KTH-SML/svea/blob/main/src/svea_localization/params/robot_localization/global_ekf.yaml) | 10 Hz; GPS x,y,z absolute, gate 5; IMU orientation differential |
| [swisscheese38/rtklawnmower](https://github.com/swisscheese38/rtklawnmower/blob/main/catkin_ws/src/rtklm_localization/config/ekf_map.yaml) (RTK) | 10 Hz; IMU absolute, gates 0.8; RTK GPS absolute, no gate; Q x,y 1.0, initial P x,y 1.0 |
| [ros-agriculture/tractor_localization](https://github.com/ros-agriculture/tractor_localization/blob/master/params/gps_imu_localization.yaml) | 30 Hz single EKF; GPS gate 5; IMU relative, gates 0.8 |
| [uos/pluto_robot](https://github.com/uos/pluto_robot/blob/master/pluto_bringup/config/ekf_navsat.yaml) | 80 Hz dual 3D; wheel + laser odometry; 3 IMUs relative |
| [vigneshrajap agri-fields](https://github.com/vigneshrajap/vision-based-navigation-agri-fields/blob/master/gnss_waypoint_navigation/config/dual_ekf_navsat_3D.yaml) | odom EKF: IMU rates only; map EKF: IMU absolute + GPS absolute |
| [FusionCore baseline](https://github.com/manankharwar/fusioncore/blob/main/benchmarks/comparison/rl_ekf_outdoor.yaml) | wheel vx + vyaw gate 3.72; GPS gate 3.72; IMU without absolute yaw |
| [clearpath_common a200](https://github.com/clearpathrobotics/clearpath_common/blob/humble/clearpath_control/config/a200/localization.yaml) | 50 Hz odom EKF only; no GPS shipped |
