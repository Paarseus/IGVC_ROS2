# X1 — Reference stacks

| | |
|---|---|
| **Question** | How do complete, working robot and vehicle stacks (IGVC AutoNav teams, commercial research platforms, field/agricultural, planetary and automotive systems) connect motor control, odometry, IMU, GNSS/RTK, state estimation, navigation and safety, and what do they report about accuracy, rates and failures? |
| **Covers** | Layering and interfaces, shipped vendor defaults (ros2_control, robot_localization, Nav2), IGVC design-report stacks 2018–2025, field, planetary, Formula Student and Autoware stacks, safety chains, recurring patterns, documented failures and how stacks are validated. |
| **Not covered** | The theory of each subsystem (C1–C4, L1–L5) and test procedures (X2). |
| **Status** | Draft |
| **Last updated** | 2026-09-28 |

**Page conventions.** Journal papers (Stanley, Boss, Kato 2015) are cited by printed journal page. MDPI (Thorvald) by its "n of 16" page. arXiv/author copies (Macenski 2020/2022/2023, Kabzan, Bradford, Maimone JFR, Moore, Gat, CLARAty) and all IGVC design reports are cited by PDF page ("PDF p. n") and, where useful, section. Code and configuration files are cited by file and parameter or line.

## Summary
- The stacks reviewed here split the work into three levels: a fast feedback loop next to the actuators, a middle layer that sequences behaviours and estimates state, and a slow planner. Gat names these controller, sequencer and deliberator [X1-S01, PDF p. 4]. Stanley [X1-S08, p. 666], Boss [X1-S09, p. 425], Autoware [X1-S37, architecture overview] and Nav2 [X1-S03, PDF p. 3] all use this layered shape.
- In every stack reviewed here that describes it, the wheel-velocity loop is closed below the ROS computer: in vendor MCUs and motor drivers (Clearpath, AgileX), in smart motor controllers (SPARK MAX, Roboteq, RoboClaw) or in team microcontrollers (Teensy, STM32). The host sends body or wheel velocities. [X1-S32, "System Architecture"; X1-S33, scout_messenger.hpp L185–200; X1-S23, PDF p. 7 §5.3; X1-S27, PDF p. 10; X1-S39, roboclaw_wrapper.py L317–338]
- The Nav2 GPS tutorial calls two EKFs plus navsat_transform "a common setup" for GNSS in ROS. The local (odom) filter fuses wheel twist and IMU. The global (map) filter adds GNSS. [X1-S36, "Global Odometry"; X1-S34, robot_localization_world.launch; X1-S27, PDF p. 14; X1-S28, PDF p. 9 §6.2.2] Recent IGVC winners (Sooner 2023–2025) use a particle filter over encoders and GPS instead [X1-S21, PDF p. 12; X1-S22, PDF p. 11; X1-S23, PDF p. 9].
- Shipped vendor defaults are modest: diff_drive_controller at 20–50 Hz, odometry at 50 Hz, a 0.25–0.5 s `cmd_vel` timeout, and an explicit `wheel_separation_multiplier` for skid-steer bases (1.875 Husky, 1.5 Jackal, 1.125 Warthog) [X1-S30, a200/j100/w200 diff_4wd.yaml].
- The failures documented most often are about timing and data integrity, not the core algorithms. Examples: stalled or badly time-stamped sensor streams (Stanley's 300–1,100 ms laser stalls [X1-S08, p. 688]), GNSS jumps and loss (Boss [X1-S09, pp. 439–441, 461]), IMU filter convergence (RoboJackets, about one minute [X1-S29, imu_issue.md]), end-to-end latency (0.4 s Bender [X1-S26, PDF p. 13]; about 300 ms AMZ [X1-S18, PDF p. 33]) and wheel slip or traction loss [X1-S12, PDF p. 1; X1-S25, PDF p. 15].
- Safety chains are layered. A hardware e-stop cuts motor power through a relay or contactor. On top of that sit software stops and command timeouts, and the higher-level stacks add a command gate or monitor. [X1-S16, PDF p. 5; X1-S32, "Emergency Stop Buttons"; X1-S23, PDF p. 8 §5.5; X1-S38, vehicle_cmd_gate README]

## Foundational references
| ID | Reference | Why it is foundational |
|---|---|---|
| X1-S01 | Gat 1998, "On Three-Layer Architectures" | Standard statement of the controller / sequencer / deliberator layering; cited by Stanley as its architectural model. |
| X1-S02 | Kortenkamp & Simmons 2008, "Robotic Systems Architectures and Programming" (Springer Handbook) | Handbook survey of robot architectures. **Not downloaded** (no open copy); not cited for any finding. |
| X1-S03 | Macenski et al. 2020, "The Marathon 2: A Navigation System" (IROS) | Defines Nav2's behaviour-tree / server / lifecycle design used by ROS 2 reference robots. |
| X1-S04 | Macenski et al. 2022, "Robot Operating System 2" (Science Robotics) | Primary description of the ROS 2 middleware all ROS 2 stacks share. |
| X1-S05 | Macenski et al. 2023, "From the desks of ROS maintainers" (RAS) | Maintainers' description of the modern ROS 2 mobile stack (controllers, robot_localization, fuse, GPS). |
| X1-S06 | Chitta et al. 2017, "ros_control" (JOSS) | Origin of the controller-manager / hardware-interface pattern used by vendor bases. |
| X1-S07 | Moore & Stouch 2014/2016, robot_localization (IAS-13) | The state estimator used by almost every ROS reference stack here. |
| X1-S08 | Thrun et al. 2006, "Stanley" (J. Field Robotics) | Seminal complete outdoor stack with measured estimation and failure data. |
| X1-S09 | Urmson et al. 2008, "Boss and the Urban Challenge" (J. Field Robotics) | Seminal layered automotive stack with a documented GNSS/INS localization layer. |
| X1-S10 | Kato et al. 2015, "An Open Approach to Autonomous Vehicles" (IEEE Micro) | First publication of Autoware. |
| X1-S11 | Kato et al. 2018, "Autoware on Board" (ICCPS) | Embedded Autoware profile. **Not downloaded** (no open copy); not cited. |
| X1-S12 | Maimone, Cheng & Matthies 2007, "Two Years of Visual Odometry on the MER" (J. Field Robotics) | Primary field report of a GNSS-free rover stack (IMU + wheel + visual odometry). |
| X1-S13 | Maimone et al. 2007, "Overview of the MER Autonomous Mobility and Vision Capabilities" (ICRA 2007 workshop) | Companion overview of MER driving modes and position estimation. |
| X1-S14 | Grimstad & From 2017, "The Thorvald II Agricultural Robotic System" (Robotics) | Reference modular ROS agricultural robot. |
| X1-S15 | Ruckelshausen et al. 2009, "BoniRob" (Precision Agriculture '09) | First BoniRob platform paper. |
| X1-S16 | IGVC 2026 Official Competition Details, Rules and Format | Primary specification every IGVC stack is designed against. |

## Findings

### 1. Reference architectures and layering (theory and survey)
- Gat describes three components that run as separate computational processes: a reactive feedback controller, a sequencer that selects which primitive behaviour the controller runs, and a deliberator for time-consuming computations [X1-S01, PDF p. 4 §3].
- Gat sorts algorithms by internal state. Stateless sensor-based algorithms go in the controller, algorithms with memory of the past go in the sequencer, and algorithms that predict the future go in the deliberator [X1-S01, PDF p. 3 §2].
- Gat sets rules for the controller layer. Each iteration should take constant-bounded time and space, with a constant small enough to give the bandwidth needed for stable closed-loop control. Controller algorithms should "fail cognizantly", meaning they detect their own failures so that higher layers can recover. Filtering algorithms are the named exception allowed to keep internal state in the controller. [X1-S01, PDF p. 4 §3.1]
- JPL's CLARAty architecture turns the three-level design into two layers. A Decision Layer couples planner and executive tightly, and it interacts with a Functional Layer that is "an interface to all system hardware", at all levels of granularity [X1-S17, PDF p. 2, Executive Summary].
- Nav2 has a behaviour-tree navigator that calls separate planner, controller and recovery servers through ROS 2 actions. All servers are managed (lifecycle) nodes. Algorithms are plugins loaded at run time. [X1-S03, PDF pp. 2–3 §III]
- The ROS maintainers' survey describes the standard REP 105 split. `odom→base_link` comes from wheel encoders, usually fused with continuous sources such as IMU angular velocity, and is used for local control. `map→base_link` is drift-free but "subject to discontinuities, making it a poor choice for control algorithms". [X1-S05, PDF p. 16]
- ros_control separates hardware (the `RobotHW` hardware interface with typed state/position/velocity/effort interfaces) from controllers, which the controller_manager loads and starts at run time [X1-S06, PDF p. 2].
- Autoware's control component outputs generic commands: target steering angle, steering torque, speed and acceleration. Vehicle-specific values such as pedal positions and individual wheel speeds are excluded. An adapter module converts between these generic commands and each vehicle's drive-by-wire interface. [X1-S37, control design, "Autoware Control Design", "Vehicle interface adapter"]
- Autoware's architecture document lists seven stacks: sensing, map, localization, perception, planning, control and vehicle interface. It says the initial design left fail-safe, real-time processing, redundancy and state monitoring as future work. [X1-S37, architecture overview]
- Loop rates stated by the sources:
  - Stanley polled sensors at up to 100 Hz and controlled steering, throttle and brake at up to 20 Hz [X1-S08, p. 665].
  - Nav2's marathon behaviour tree replanned the global path at 1 Hz [X1-S03, PDF p. 4].
  - The survey gives the BT tick rate as 100 Hz and recommends MPPI at 1000 samples at 50 Hz or 2000 samples at 30 Hz [X1-S05, PDF pp. 8, 16].
  - Autoware requires localization output at "50Hz~" [X1-S37, localization design, "Output"].

### 2. IGVC AutoNav team stacks, 2018–2025
General pattern first, then team by team.
- **Low-level control:** every report reviewed puts the wheel-speed PID on a microcontroller or motor controller, not on the main computer. [X1-S20, PDF p. 7 §4.3.3; X1-S21, PDF p. 9 §5.3.3; X1-S22, PDF p. 8; X1-S23, PDF p. 7 §5.3; X1-S26, PDF p. 7 §4.5; X1-S27, PDF p. 10; X1-S28, PDF p. 8 §5.5]
- **Sensors and fusion:** the choices vary widely. There are dual robot_localization EKFs (IIT Kharagpur 2018, MANAS 2025), a custom EKF (Sooner 2021), particle filters (Sooner 2023–2025), and GPS-plus-compass direction logic without a filter (Ville 2025). [X1-S27, PDF p. 14; X1-S28, PDF p. 9; X1-S20, PDF p. 11; X1-S23, PDF p. 9; X1-S25, PDF p. 13]
- **Sooner Competitive Robotics, 2021 ("Aluminum Whale"):**
  - An STM32F103 receives each wheel's speed over CAN and runs a software PID using wheel encoders [X1-S20, PDF p. 7 §4.3.3].
  - Localization is a custom EKF with state [x, ẋ, y, ẏ, φ, φ̇, v_l, v_r]. It uses a measurement vector of wheel angular velocities and yaw, and a constant-velocity model. [X1-S20, PDF p. 11 §5.3]
  - The team resets the EKF each time a new path is planned [X1-S20, PDF p. 12].
  - Paths are followed with pure pursuit [X1-S20, PDF p. 12 §5.4.2].
- **Sooner 2023 ("Weeb Wagon"):**
  - Motors are driven by microcontroller PWM with quadrature encoders and a PID holding the velocity commanded over CAN [X1-S21, PDF p. 9 §5.3.3].
  - A particle filter compares GPS with encoder motion. The report says it gets a precise location and heading "within the first few seconds", and that heading has no other source "due to the lack of a tested IMU". [X1-S21, PDF p. 12 §6.6]
- **Sooner 2024 ("Danger Zone"):**
  - Two NEO motors through 10.7:1 gearboxes drive a six-wheel drop-centre skid-steer [X1-S22, PDF p. 5].
  - A custom "motor manager" commands the SPARK MAX controllers by PWM and runs its own PID on output-shaft encoders. Gains are tunable live over CAN, and the manager accepts linear and angular velocity. [X1-S22, PDF p. 8 "Motor Control"]
  - A particle filter fuses encoders and GPS [X1-S22, PDF p. 11].
  - Pure pursuit runs "approximately 20 times a second" [X1-S22, PDF p. 12].
- **Sooner 2025 ("Twistopher"):**
  - Eight SPARK MAX controllers sit on a CAN bus separate from the main bus. Four run onboard position control for steering and four run onboard velocity control for drive. The team says offloading the PID to the controllers simplifies the electronics and minimizes latency. [X1-S23, PDF p. 7 §5.3]
  - A particle filter combines swerve encoder odometry with GPS [X1-S23, PDF p. 9 §6.4].
  - The sensor list includes a VN-200 IMU/GPS [X1-S23, PDF p. 4 §2.5].
  - Reported measured navigation accuracy is "<1 meters" and obstacle reaction time is 47 ms [X1-S23, PDF p. 14 §8.4].
- **Cedarville "Delta Bee" (2022, 1st):**
  - The team uses a behavioural master state machine "instead of using the ROS navigation stack" [X1-S24, PDF p. 11].
  - A Teensy handles the shaft encoders [X1-S24, PDF p. 4].
  - The stack mixes ROS Noetic and Galactic through a bridge [X1-S24, PDF pp. 4, 9].
- **Ville Robotics "ALiEN 6.0" (2025, 2nd):**
  - An industrial PLC drives SEW Movimot motors with built-in controllers over Profinet [X1-S25, PDF p. 7].
  - A Teensy reads a NEO-6M GPS and a QMC5883L compass and sends the PLC a 3-wire "octal" direction code, with no path planning [X1-S25, PDF p. 13].
  - Reported GPS error is about 2 feet, waypoint arrival is within 1–2 feet, and reaction time is under 10 ms [X1-S25, PDF p. 14].
- **Boise State "Bender" (2019, 1st):**
  - A Teensy 3.6 runs a speed PID and a steering-angle PID for each wheel group. The laptop sends commands over serial USB. [X1-S26, PDF p. 7 §4.5]
  - A Pixhawk 4 with GPS serves as the compass [X1-S26, PDF p. 6].
  - Mapping uses a 5 Hz RPLiDAR [X1-S26, PDF p. 6].
- **IIT Kharagpur "Eklavya 6.0" (2018, 2nd):**
  - A Roboteq MDC2230 runs PID speed control using two front-wheel encoders, tuned from an identified motor model [X1-S27, PDF p. 10].
  - Sensors include a VectorNav VN-200 GPS-aided INS [X1-S27, PDF p. 8].
  - Localization uses two EKF nodes: an odom-frame one and a map-frame one that adds filtered GPS [X1-S27, PDF p. 14].
  - Planning uses navfn (A*) with the TEB local planner [X1-S27, PDF pp. 15–16].
  - The team reports about 10 cm closed-loop error in a loop-closure test [X1-S27, PDF p. 14].
- **Project MANAS "STEVE" (2025, 3rd):**
  - A local EKF fuses wheel encoders and IMU. A global EKF fuses GPS and Direct LiDAR Odometry. AMCL is used when a saved map exists. [X1-S28, PDF p. 9 §6.2]
  - move_base with NavFn A* and TEB runs through a ROS 1–ROS 2 bridge [X1-S28, PDF p. 10 §6.5].
  - A drive-controller MCU runs an adaptive PID per motor from `cmd_vel` [X1-S28, PDF p. 8 §5.5].
- **Georgia Tech RoboJackets `igvc-software` (ROS 1, 2022 snapshot):**
  - A single robot_localization EKF runs at 30 Hz with `world_frame: odom`. It fuses wheel-odometry velocities (vx, vy, vyaw), GPS position from `/odometry/gps` (x, y, z), IMU roll/pitch, yaw rate and acceleration, and magnetometer yaw. [X1-S29, ekf_localization_node_params.yaml]
  - navsat_transform runs at 30 Hz with the magnetometer remapped as its IMU input [X1-S29, localization.launch].
  - The motor-controller node connects to the motor board at an IP address and port, runs at `frequency: 50.0` and sets `watchdog_delay: 3.0`. Its PID gain parameters are all 0.0. [X1-S29, motor_controller.launch]

### 3. Commercial research platforms: low-level control and kinematics
- **Clearpath (ROS 2, clearpath_common 2.9.17):**
  - Husky A200, Jackal J100 and Warthog W200 all ship ros2_control's `diff_drive_controller`, with one command signal per side (`wheels_per_side: 1 # actually 2`) [X1-S30, a200/j100/w200 diff_4wd.yaml].
  - Controller-manager rates are 20 Hz on A200 and 50 Hz on J100 and W200. Odometry publishes at 50 Hz. [X1-S30, diff_4wd.yaml `update_rate`, `publish_rate`]
  - Skid-steer correction is set explicitly: `wheel_separation_multiplier` 1.875 (A200, `wheel_separation` 0.555 m), 1.5 (J100, 0.37559 m) and 1.125 (W200, 1.5 m) [X1-S30, diff_4wd.yaml].
  - `cmd_vel_timeout` is 0.5 s (A200, J100) and 0.25 s (W200) [X1-S30, diff_4wd.yaml].
  - `enable_odom_tf: false`, so the controller does not publish `odom→base_link` [X1-S30, diff_4wd.yaml]. The EKF node publishes it instead (`publish_tf: true`, `world_frame: odom`) [X1-S30, generic/localization.yaml].
  - Odometry covariance diagonals are 0.001 for x, y, z, roll and pitch, and 0.01 for yaw, for both pose and twist [X1-S30, diff_4wd.yaml].
  - Shipped limits:

    | Platform | Linear velocity | Linear acceleration | Linear deceleration | Angular velocity |
    |---|---|---|---|---|
    | A200 | 1.0 m/s | 1.0 m/s² | not set | 1.0 rad/s |
    | J100 | 2.0 m/s | 2.0 m/s² | 4.0 m/s² | 4.0 rad/s |
    | W200 | 5.0 m/s | 2.5 m/s² | 10.0 m/s² | 4.0 rad/s |

    [X1-S30, diff_4wd.yaml]
  - W200 sets `position_feedback: false` because its "motor controllers only report velocity" [X1-S30, w200 diff_4wd.yaml].
- **Husky A300 manual:**
  - The computer is paired with a 32-bit MCU over Ethernet that handles "low-level hardware functions such as power supply control, light control, screen control, and hardware monitoring" [X1-S32, "System Architecture"].
  - "Husky's control loops can accurately maintain velocities as low as 0.1 m/s" [X1-S32, "General Warnings"].
  - The manual gives an effective track W of 0.562 m for its wheel-to-platform velocity relation [X1-S32, "Robot Equations"].
- **Configuration model (Clearpath):** a single `robot.yaml` feeds generators that write `setup.bash`, the URDF, launch files and parameter files into `/etc/clearpath/`. Users change `robot.yaml`, not the generated files. [X1-S31, generators.mdx] Parameters of platform nodes can be overridden with `extras: ros_parameters` [X1-S31, platform overview.mdx].
- **AgileX (Scout, Bunker tracked, Hunter; ugv_sdk + *_ros2):**
  - The base talks CAN (the SDK set-up brings `can0` up at `bitrate 500000`) using protocol V1 or V2 [X1-S33, ugv_sdk README].
  - The ROS 2 node passes `linear.x` and `angular.z` straight to `SetMotionCommand`, so wheel-level kinematics sit in the base firmware [X1-S33, scout_messenger.hpp L199–200; bunker_messenger.hpp L186].
  - The host integrates the base-reported linear and angular velocity into a pose. It publishes `odom` and the `odom→base_link` TF in a 50 Hz loop. [X1-S33, scout_messenger.hpp L251–302; scout_base_ros.cpp L151–158]
  - The published odometry message fills pose and twist but no covariance fields [X1-S33, scout_messenger.hpp L288–302].
- **Robotnik Summit XL (summit_xl_common, `humble` branch, files in ROS 1 launch format):**
  - `robotnik_base_control` sets `cmd_watchdog_duration: 0.5`, `imu_watchdog_duration: 0.1` and `odom_publish_frequency: 100`.
  - Limits are 1.5 m/s linear, 0.6 m/s² linear acceleration, 3 rad/s angular and 1.5 rad/s² angular acceleration.

  [X1-S34, robot_control.yaml]
- **JPL Open Source Rover (ROS 2, RoboClaw):**
  - Drive commands go to RoboClaw controllers with `SpeedAccelM1/M2`, which closes the velocity loop on the RoboClaw, or `DutyAccel` in duty mode [X1-S39, roboclaw_wrapper.py L317–338].
  - The shipped default is `duty_mode: true`. It is commented as not to be "used for autonomous navigation as velocity commands aren't interpreted correctly" [X1-S39, roboclaw_params.yaml].
  - After `velocity_timeout: 2.0` s without a command, the node first ramps velocity to zero and then sends a full stop [X1-S39, roboclaw_params.yaml; roboclaw_wrapper.py L167–180].
- **ros2_controllers defaults:** `diff_drive_controller` defaults are `wheel_separation_multiplier` 1.0, `cmd_vel_timeout` 0.5 s, `publish_rate` 50 Hz, `open_loop` false and `enable_odom_tf` true [X1-S35, diff_drive_controller_parameter.yaml].
- **linorobot2 (open ROS 2 reference robot kit; microcontroller firmware plus host packages):**
  - The microcontroller runs micro-ROS (`rclc`) and subscribes to `cmd_vel` directly. A timer runs the control callback every 20 ms ("50 Hz"). [X1-S44, firmware.ino L185–197]
  - Each motor has its own PID from required RPM to PWM, using the encoder-measured RPM. The same loop turns the measured RPMs back into body velocities and integrates odometry on the microcontroller. [X1-S44, firmware.ino L82–85, L259–294]
  - If no `cmd_vel` has arrived for 200 ms, the firmware sets the commanded velocity to zero [X1-S44, firmware.ino L247–256].
  - The default base type `DIFFERENTIAL_DRIVE` is commented "2WD and Tracked robot w/ 2 motors". The default gains are K_P 0.6, K_I 0.8 and K_D 0.5, and the allowed RPM is capped at `MAX_RPM_RATIO` 0.85 of the motor's maximum RPM. [X1-S44, lino_base_config.h L21, L37–39, L50–51]
  - On the host, one robot_localization EKF runs at 50 Hz in `two_d_mode` with `world_frame: odom`. It fuses wheel vx, vy and vyaw, plus IMU yaw rate only. [X1-S44, ekf.yaml]
- **iRobot Create 3 (base of the TurtleBot 4 ROS 2 reference robot; indoor):**
  - The base fuses IMU, optical-mouse and wheel-encoder data on board into a dead-reckoning `odom` topic at 20 Hz. Raw wheel topics publish at 62.5 Hz, and `imu` at 100 Hz from firmware G.4/H.1 (62.5 Hz before). [X1-S45, create3_odometry.md]
  - It also publishes a boolean `slip_status` estimate at 20 Hz. The documentation recommends that users who build their own estimator from wheel encoders "inflate the differential motion covariance matrix" to allow for slip, and notes that the optical mouse is not affected by slippage. [X1-S45, create3_odometry.md "The slip_status topic"]

### 4. Commercial research platforms: localization and navigation defaults
- **Clearpath generic EKF:**
  - The EKF runs at 50 Hz with `two_d_mode: true`. It fuses wheel odometry x, y, yaw, vx, vy and vyaw (`odom0_differential: false`). [X1-S30, generic/localization.yaml]
  - The generator adds each IMU with only yaw rate and x-acceleration fused (`imu_config`), plus IMUs built into GPS receivers [X1-S30, generator param/platform.py L686–743].
  - `enable_ekf` defaults to true, and the output is remapped to `platform/odom/filtered` [X1-S30, localization.launch.py].
- **Robotnik dual EKF:**
  - Local EKF: 45 Hz, `two_d_mode`, `sensor_timeout 0.1`. It fuses wheel odometry x, y with `odom0_differential: true`, and IMU yaw rate plus x-acceleration. [X1-S34, robot_localization_odom.launch]
  - A comment warns against fusing IMU yaw "coming from an attitude estimator which is more likely to use a (slow-response) compass" [X1-S34, robot_localization_odom.launch].
  - Global EKF: 30 Hz. It fuses the local output differentially plus `odometry/gps` x, y [X1-S34, robot_localization_world.launch].
  - navsat_transform takes yaw from the IMU (`use_odometry_yaw: false`) and sets `zero_altitude: true` [X1-S34, navsat_transform_node.launch].
- **Nav2 GPS tutorial (Kiwibot):**
  - Two EKFs are used. `ekf_filter_node_odom` fuses wheel velocities and IMU heading. `ekf_filter_node_map` adds `odometry/gps` x, y. navsat_transform supplies the GPS odometry. The tutorial calls this "a common setup on robot_localization when using GPS data". [X1-S36, tutorial §2 "Setup GPS Localization system"; dual_ekf_navsat_params.yaml]
  - Both EKFs run in 2D mode because Nav2's costmaps are 2D [X1-S36, "Local Odometry"].
  - Absolute orientation is "mandatory" when using robot_localization with GPS [X1-S36, "GPS Localization Overview"].
  - The datum is taken from the first valid fix unless fixed. Fixing it is needed when a static map or stored cartesian waypoints are used. [X1-S36, "Navsat Transform"]
  - The demo parameters use:
    - `frequency: 30.0` in both EKFs;
    - process noise of 1e-3 on x/y in the odom EKF and 1.0 in the map EKF;
    - navsat `delay: 3.0` and `use_odometry_yaw: true`.

    [X1-S36, dual_ekf_navsat_params.yaml]
- **Vendor Nav2 controller settings:** survey-level guidance says MPPI is expected to replace DWB as the default controller, and "1000 samples at 50 Hz or 2000 samples at 30 Hz yield good results" [X1-S05, PDF pp. 7–8]. Controller tuning itself is C4.
- **Clearpath Nav2 demos (`clearpath_nav2_demos`, `humble` branch):**
  - The Husky A200 file is a simulation demo (`use_sim_time: True`) [X1-S42, a200 nav2.yaml L3, L71].
  - It runs the controller server at 20 Hz with the DWB local planner: `max_vel_x` 1.0 m/s, `acc_lim_x` 0.5 m/s², `sim_time` 1.7 s [X1-S42, a200 nav2.yaml L72, L99, L103, L111, L120].
  - The planner is Navfn [X1-S42, L224].
  - The local costmap is a 5 m × 5 m rolling window in the `odom` frame, updated at 5 Hz, with `inflation_radius` 0.8 m [X1-S42, L143–157].
  - The velocity smoother runs at 20 Hz in `OPEN_LOOP` mode, with accelerations of 0.5 and a 1.0 s `velocity_timeout` [X1-S42, L279–292].
  - The Warthog W200 file differs only in footprint and a 1.0 m inflation radius [X1-S42, diff of a200 and w200 nav2.yaml].
- **Clearpath OutdoorNav (the vendor's own outdoor GNSS navigation product, documented but not open source):**
  - It is described as "GPS-based localization with sensor fusion of camera, IMU, LiDAR and platform odometry" and path following "through a network of paths", for Husky, Jackal, Warthog and third-party UGVs [X1-S43, introduction "Key Features", "Compatible Platforms"].
  - Its stated performance "with standard sensors" is location accuracy under 5 cm and under 2° (position and heading) and path-tracking accuracy of about 10 cm. The documentation says performance is "highly dependent" on the base UGV, the sensors, the integration and the environment. [X1-S43, operating conditions "Performance"]
- **robot_localization practice from the maintainers:**
  - Use `two_d_mode` when planar.
  - Fuse the zero ẏ from differential-drive wheel odometry.
  - Give every dimension a reference.
  - Prefer one pose source and many velocity sources in a dimension.

  [X1-S05, PDF p. 17]

### 5. Field-robot and agricultural stacks
- **Thorvald II (NMBU):**
  - The main battery enclosure holds the robot computer and is the connection point for the CAN bus used to talk to motor controllers [X1-S14, p. 4 of 16 §2.1.2].
  - Each steering module has a two-channel motor controller on the robot's CAN that runs its own motor and the attached drive module [X1-S14, p. 5 §2.1.4].
  - Hardware parameters are loaded on the ROS parameter server. Reconfiguring, for example from 4WD/4WS to differential drive, needs only width, length and drive-type parameters before the robot "is ready to receive commands and publish odometry". [X1-S14, pp. 6–7 §2.3]
  - A phenotyping configuration navigates predefined waypoints with an RTK-GNSS receiver and an IMU. A polytunnel UV robot uses IMU, encoder odometry and LiDAR with a map. [X1-S14, p. 8 §2.4]
  - The software paper describes the command path. A `twist_mux` multiplexer forwards whichever velocity topic (teleoperation or autonomous) has the highest priority and is currently publishing. A `base_driver` node turns the resulting Twist into joint commands over CAN and publishes velocity estimates from motor feedback as `nav_msgs/Odometry`. [X1-S48, p. 161 §2.2, §3.1]
  - Each motor sits on a motor controller "with a control loop for reaching target wheel speeds and steering positions" [X1-S48, p. 162 §3.2].
  - The authors state that encoder odometry alone is "not sufficient" for absolute pose, because wheel slip varies across surfaces and tyres differ slightly in diameter and pressure. It is still used for SLAM input and for "keeping track of the robot between low-frequency GPS measurements". [X1-S48, p. 162 §3.3.2]
- **Agricultural navigation surveys:**
  - Bonadies & Gadsden state that GPS and GIS are "the most commonly used means" of guiding vehicles through farm fields without an operator. They cite an RTK-DGPS weeding robot (Bakker et al.) with location accuracy of 1–2 cm. [X1-S47, p. 24]
  - The same review says low-cost GPS gives accuracy "within meters" and dead reckoning loses accuracy over time through slip. In row crops, GPS alone "may not provide the accuracy needed to avoid damage to crops", which motivates camera-based row following. [X1-S47, p. 26]
  - It groups the control methods used as PID, fuzzy logic, neural-network/genetic-algorithm control and Kalman filtering. It concludes that crop-row strategies "typically make use of machine vision and PID and fuzzy control methods". [X1-S47, pp. 26, 31]
- **Cornell PPBv2 vineyard robot (ROS 2, preprint, level C):**
  - A Raspberry Pi 5 runs the navigation stack. It sends (v, ω) commands over USB serial to a Feather M4 CAN microcontroller, which transmits them periodically on CAN to a four-wheel skid-steer farm-ng Amiga. The authors say this boundary "decouples ROS callback timing from periodic chassis-bus transmission". [X1-S46, PDF p. 2 §III.A]
  - Two interchangeable localization front ends feed one odometry interface. One is RTK position plus a separate IMU's yaw; the other is a dual-antenna RTK receiver that also gives heading. There is deliberately no fusion filter: the stack "stops when either required stream is invalid". [X1-S46, PDF pp. 2–4 §§II, III.B, IV.D]
  - All controller modes (pure pursuit, cross-track PID, NMPC and a hybrid) run at a nominal 10 Hz and publish the same Twist type [X1-S46, PDF p. 2 §III.B].
  - The datum is the first accepted RTK-fixed position and is then immutable. The authors note that a restart creates a new datum, and that applications needing identical coordinates across days should use a surveyed, stored datum. [X1-S46, PDF p. 3 §IV.B]
- **BoniRob (Bosch / Osnabrück):**
  - Speed and steering control are a separate system that commands the motors and hydraulics. Internal communication uses Ethernet, with CAN "on lower levels". A timestamp concept synchronises data. [X1-S15, PDF p. 3 "System architecture"]
  - An RTK-GPS receiver is the main localization sensor. Odometry and inertial data are fused in a Kalman filter to keep sub-decimetre accuracy when RTK is temporarily degraded. The open copy is partly garbled at this sentence. [X1-S15, PDF p. 4 "Navigation"]
  - Navigation-control components can be deployed under a real-time OS, and less time-critical components elsewhere [X1-S15, PDF p. 3].
- **Formula Student Driverless (AMZ, ETH Zürich):**
  - Velocity is estimated by an EKF that fuses six sensors (ground-speed sensor, GNSS, IMU, wheel resolvers, and more) with a tire-slip model [X1-S18, PDF pp. 4, 7, 18–19 §4.1].
  - Outliers are rejected with a chi-square test, and drift is detected from variance across sensors [X1-S18, PDF pp. 19–20 §4.1.2].
  - The FastSLAM localizer integrates velocities to give a pose update at 200 Hz [X1-S18, PDF p. 20 §4.2].
- **F1TENTH (1/10-scale autonomous racing research platform, U. Pennsylvania):**
  - The base chassis has a brushless motor with a VESC electronic speed controller (open-source design) that converts speed input into motor RPM, plus a steering servo. "Odometry is provided by the VESC." [X1-S49, p. 81 §4]
  - The default computer is a Jetson TX2, with support for Xavier and Nano. A power management board gives the computer a stable supply "since the battery voltage varies as the vehicle is operated". [X1-S49, p. 81 §4]
  - To narrow the simulation-to-reality gap, the simulator uses vehicle parameters identified on the real car and test surface [X1-S49, p. 80 §3.2].
- **Formula Student (QUT, ROS 2):** the team replaced its custom modules with robot_localization, slam_toolbox, Nav2's Smac Hybrid-A* planner and Regulated Pure Pursuit [X1-S19, PDF pp. 3–4]. The paper notes that FS Driverless teams commonly build on ROS or ROS 2 and that their architectures often follow a standard perception-to-control pipeline [X1-S19, PDF p. 3 §2.4].

### 6. Planetary rover and early field-autonomy stacks
- **MER:**
  - Without visual odometry, the rovers estimate position from wheel encoders and IMU gyros. On level ground Spirit's estimate was reported off by 3% after more than 2 km. [X1-S13, PDF p. 4]
  - Heading drift from integrating gyros "for thousands of seconds" is removed by locating the Sun with a camera and using accelerometer gravity [X1-S13, PDF p. 2].
  - Visual odometry was needed on slopes and sand. The rovers saw slips of 100%, 99.9% and 125%. [X1-S13, PDF p. 4]
  - VO converged 97% (Spirit) and 95% (Opportunity) of the time [X1-S12, PDF p. 1].
  - When VO fails, the wheel-odometry-plus-IMU estimate is kept [X1-S12, PDF p. 2].
  - A single VO tracking step could take "up to three minutes" on the 20 MHz CPU [X1-S12, PDF p. 9].
- **Stanley (DARPA 2005):**
  - The software has six layers: sensor interface, perception, control, vehicle interface, user interface and global services. It has no central master process, and data flows one way through a pipeline. [X1-S08, p. 666 §3.1–3.2]
  - A UKF estimates 15 state variables at 100 Hz from GPS, GPS compass, IMU and wheel encoders [X1-S08, p. 668 §4].
  - During GPS outages the UKF switches to a vehicle model that only allows motion in the pointing direction. The health monitor caps speed at 10 mph. [X1-S08, p. 668]
  - With wheel motion integrated, error after 1.3 km without GPS was 1.7 m [X1-S08, p. 669].
  - The steering controller outputs commands at 20 Hz [X1-S08, p. 684 §9.2].
  - The implemented velocity is the minimum of the path planner, health monitor and velocity recommender outputs [X1-S08, p. 683].
- **Boss (DARPA 2007):**
  - An Applanix POS-LV fuses GPS, inertial and wheel-encoder data into a 100 Hz position estimate [X1-S09, p. 439].
  - Boss then corrects that estimate against lane markings to get a smooth, road-registered pose [X1-S09, pp. 439–440].
  - Planning is split into mission, behavioural and motion layers [X1-S09, p. 425 abstract].

### 7. Automotive stack: Autoware localization → control → vehicle interface
- Kato et al. describe the original chain as localization (3D NDT scan matching against a 3D map, "at the order of centimeters") → detection/tracking → mission and motion planning → path following (pure pursuit). The output velocity and angle are sent to the vehicle controller. [X1-S10, pp. 62, 64, Fig. 2]
- The test vehicle's gateway (ZMP Robocar) lets a plug-in computer send "pedal strokes and steering angles" over the vehicle CAN [X1-S10, p. 61].
- Kato et al. warn that if actuation "is not aligned with the velocity and angle output of the Pure Pursuit algorithm because of some noise, the vehicle could temporarily get off the path" [X1-S10, p. 64].
- Current Autoware control has two modules [X1-S37, control design]:
  - `trajectory_follower` computes the command;
  - `vehicle_cmd_gate` filters abnormal values and switches between sources, including the MRM (minimal risk manoeuvre) module.
- The trajectory follower runs a lateral controller (`mpc` or `pure_pursuit`) and a longitudinal PID. It publishes a combined `Control` only if both commands are no older than `timeout_thr_sec`. [X1-S38, trajectory_follower_node README "Parameter"]
- Autoware localization must output pose, twist and acceleration with covariance at "50Hz~", plus diagnostics and `map→base_link` TF [X1-S37, localization design "Output"].
- Autoware's localization design rates each sensor:
  - GNSS alone: about 10 m; with RTK: about 10 cm [X1-S37, localization design "GNSS"].
  - IMU bias depends on temperature and can cause drift [X1-S37, "IMU"].
  - Wheel speed is unreliable on slippery or bumpy roads [X1-S37, "Wheel speed sensor"].
- Delay compensation accepts only fixed delays, set separately for longitudinal and lateral. The control design assumes the vehicle has ABS and ESC. [X1-S37, control design "Control Feature Design"]

### 8. Safety chain and supervision across stacks
- **IGVC rules:**
  - Both the mechanical and the wireless e-stop "must be hardware based and not controlled through software". The wireless e-stop must work to at least 100 ft. [X1-S16, PDF p. 5]
  - Maximum speed is 5 mph and must be "hardware governed". Minimum average speed is 1 mph. [X1-S16, PDF p. 5]
  - No base stations are allowed "for positioning accuracy" [X1-S16, PDF p. 5].
- **Clearpath:**
  - Motor-driver power on Husky A300 goes through a normally-open relay in series with the e-stop [X1-S32, "Emergency Stop Buttons"].
  - Commands received during e-stop are not buffered [X1-S32, "Emergency Stop Buttons"].
  - The wireless e-stop asserts itself automatically on loss of communication [X1-S32, "Wireless Emergency Stop (Optional Kit)"].
  - In software, `twist_mux` gives `platform/emergency_stop` a lock at priority 255, `safety_stop` at 254 and external `cmd_vel` priority 1, with 0.5 s input timeouts [X1-S30, twist_mux.yaml].
- **Sooner 2025:**
  - Motor power flows only when the remote, physical and software e-stops are all inactive. A solid-state relay is driven directly by the e-stop line. [X1-S23, PDF pp. 3, 8 §2.2, §5.5]
  - A buffer on the hub stops any board other than the e-stop board from driving that line [X1-S23, PDF p. 8].
  - The e-stop receiver stops the robot when its heartbeat is missed [X1-S23, PDF p. 13].
- **Sooner 2021:** three layers stop the motors: a contactor, a software zero-speed on the e-stop signal, and CAN emergency and mobility-stop messages [X1-S20, PDF p. 9].
- **Thorvald II:** releasing a pressed e-stop does not restore motor power. The robot must receive a command to re-engage the contactor. [X1-S14, p. 4 §2.1.2]
- **BoniRob:** each wheel system has an emergency shutdown button, software malfunctions cause a shutdown, and there is an external shutdown over WLAN [X1-S15, PDF p. 5 "Safety concept"].
- **Stanley and Boss:** Stanley had a health monitor that restarts or power-cycles components and a DARPA wireless E-Stop [X1-S08, pp. 665–666, 668]. Boss had a safety radio to engage, pause or disable autonomy [X1-S09, p. 428].
- **Autoware:**
  - `vehicle_cmd_gate` applies a "final guard" filter. It is explicitly "not designed to enhance ride comfort", and if it is often active the control module needs tuning. The gate also checks heartbeats. [X1-S38, vehicle_cmd_gate README "Filter function", "Assumptions"]
  - `control_validator` flags commands whose predicted trajectory deviates from the reference by more than `max_distance_deviation` (default 1.0 m), among other checks [X1-S38, control_validator README].
- **AMZ:** an Emergency Braking System can be triggered from a Remote Emergency System or from the onboard computer [X1-S18, PDF p. 7].

### 9. Common patterns and differences across stacks
**Patterns**
- **Wheel-velocity loop on dedicated hardware; host sends Twist or wheel speeds.** Seen in Clearpath, AgileX, OSR, SPARK MAX (Sooner 2025), Roboteq (IIT Kharagpur) and team MCUs. [X1-S30; X1-S33; X1-S39; X1-S23, PDF p. 7; X1-S27, PDF p. 10; X1-S20, PDF p. 7]
- **Continuous local estimate separate from a jumpy global one.** REP 105 odom/map split in ROS [X1-S05, PDF p. 16; X1-S36]; Boss's smooth road-registered frame [X1-S09, p. 439].
- **Command timeouts of about 0.25–0.5 s at the base.** diff_drive_controller defaults and Clearpath settings [X1-S35; X1-S30]; Robotnik `cmd_watchdog_duration` [X1-S34]. Exception: OSR uses 2.0 s [X1-S39].
- **Heading from IMU yaw rate rather than absolute IMU yaw in vendor local filters.** Clearpath's generator [X1-S30, platform.py L688–692] and Robotnik [X1-S34] fuse vyaw only.

**Differences**
- **Global estimator:**
  - robot_localization dual EKF [X1-S36; X1-S34; X1-S27; X1-S28];
  - single EKF fusing GPS with `world_frame: odom` [X1-S29];
  - particle filter [X1-S21; X1-S22; X1-S23];
  - UKF in one filter [X1-S08];
  - commercial INS plus map correction [X1-S09].
- **Wheel odometry pose vs velocity:** Clearpath's generic EKF fuses wheel x, y, yaw absolutely [X1-S30, generic/localization.yaml]. The Nav2 GPS demo and RoboJackets fuse only wheel velocities [X1-S36; X1-S29].
- **Navigation:**
  - Nav2 / move_base with TEB [X1-S27; X1-S28];
  - custom A* plus pure pursuit [X1-S22];
  - ray-cast heuristics without a map [X1-S23, PDF p. 9];
  - behavioural state machines [X1-S24; X1-S25].

### 10. Documented problems and lessons learned
- **Stanley:**
  - Laser data "repeatedly stalled for durations of 300 to 1,100 ms", 17 times. The resulting inaccurate time stamps put phantom obstacles into the map and caused four significant swerves. [X1-S08, pp. 688–689]
  - Pose errors smaller than 0.5° caused massive terrain-classification errors [X1-S08, p. 670 Fig. 9].
- **Boss:**
  - The POS-LV "will occasionally generate position jumps". Boss rejects position changes inconsistent with wheel speed and heading (ζ = 0.05, ε = 0.02, τ = cos 30°) and accumulates the residual. [X1-S09, p. 440]
  - Omnistar HP corrections were "frequently disrupted by even small amounts of overhead vegetation", and reacquisition took about half an hour [X1-S09, p. 439].
  - Just before the final event its GPS receivers were not receiving signals, most likely because of jamming. The Jumbotron near the start area was shut down and the receivers were restarted. [X1-S09, p. 461]
- **RoboJackets:** the IMU "starts from an incorrect value and takes time to converge", close to a minute. Motion during gyro calibration "ruins the process". [X1-S29, imu_issue.md]
- **robot_localization paper:** magnetometer headings were corrupted by electromagnetic interference, and a second IMU failed midway. Adding GPS removed the heading problem. [X1-S07, PDF pp. 3–5]
- **Boise State 2019:** latency from sensors to wheels was about 0.4 s. The team mitigated it by driving near the 1 mph minimum. Laptop CPU throttling on battery caused "cascading lag throughout the ROS network". [X1-S26, PDF p. 13 §7.1–7.2]
- **Sooner 2025 failure table** [X1-S23, PDF p. 13 §8.2]:
  - unstable encoder feedback is absorbed by the particle filter;
  - loss of sensor data makes nodes stop;
  - loss of the wireless e-stop heartbeat stops the robot.
- **Ville Robotics 2025:** high motor torque made the wheels "lose traction and spin", with traction loss above 3 mph [X1-S25, PDF p. 15].
- **AMZ:** about 300 ms from cone detection to control command was "one of the biggest bottlenecks", handled by limiting top speed and wheel torques. Redundant sensing was justified because "many failures where [sic] observed" in testing. [X1-S18, PDF pp. 7, 33]
- **MANAS 2025:** "One key lesson ... was the importance of choosing the right software early", after moving from ROS 1 to ROS 2 [X1-S28, PDF p. 15 §10.1].
- **Nav2 maintainer (level D):** "odom for real hardware robots is typically at about 100hz or higher" [X1-S40, SteveMacenski comment].

### 11. How the reference stacks are validated
- Nav2 was validated by two robots running 37.4 miles in 22.8 h with 168 recoveries, 0 collisions and 0 emergency stops, at an average speed of 0.37 m/s [X1-S03, PDF p. 6 Table III].
- The robot_localization paper replayed one 777 s rosbag through five sensor configurations and reported loop-closure error [X1-S07, PDF p. 3 Table II]:

  | Configuration | Loop-closure error (x, y) |
  |---|---|
  | Odometry only | 69.65 m, 160.33 m |
  | Odometry + two IMUs + one GPS | 1.21 m, 0.26 m |

- The maintainers' survey compared robot_localization and fuse on a 541 m route, using AMCL as ground truth [X1-S05, PDF pp. 18–19 Table V]:

  | Estimator | Update rate | Error | CPU |
  |---|---|---|---|
  | robot_localization | 30 Hz | 4.25 m (0.78%) | 1.38 ± 1.17% |
  | fuse | 20 Hz | 2.81 m (0.52%) | 5.19 ± 1.37% |

- Stanley reported GPS-denied drift [X1-S08, p. 669]. Boss reported staying in lane over 5.7 km of GPS-denied driving, while the raw POS-LV error reached 2.5 m [X1-S09, p. 441].
- IGVC reports mostly give requirement-vs-measured tables (speed, e-stop range, reaction time, "navigation accuracy") and simulator runs, with no ground-truth method described [X1-S23, PDF p. 14; X1-S22, PDF pp. 15–16; X1-S25, PDF p. 14]. The exception is IIT Kharagpur, which reports a closed-loop test with about 10 cm error [X1-S27, PDF p. 14].

### 12. Product-specific: stacks with similar components
- **SPARK MAX / NEO:**
  - Sooner 2025 commands SPARK MAX controllers over a dedicated CAN bus in onboard velocity and position modes [X1-S23, PDF p. 7 §5.3].
  - Sooner 2024 commands them by PWM and closes the loop on its own microcontroller with output-shaft encoders [X1-S22, PDF p. 8].
- **Tracked and skid-steer bases:**
  - AgileX Bunker (tracked) takes (v, ω) and reports body velocities over CAN, so no track kinematics are configured on the host [X1-S33, bunker_messenger.hpp L186, L199–230].
  - Clearpath skid-steer bases configure `wheel_separation_multiplier` in `diff_drive_controller` [X1-S30].
  - Sooner 2024's six-wheel skid-steer exposes linear and angular velocity at its motor manager [X1-S22, PDF p. 8].
- **GNSS/INS units with an internal filter:**
  - IIT Kharagpur used a VectorNav VN-200 INS inside a dual EKF [X1-S27, PDF pp. 8, 14].
  - Sooner 2025 lists a VN-200 [X1-S23, PDF p. 4].
  - Stanley and Boss used commercial or custom GPS/IMU estimators [X1-S08; X1-S09].
  - No source was found for an Xsens MTi with robot_localization.
- **Nav2 MPPI on a Jetson outdoors:** no reference stack was found. See Open questions.

## Recommended practice
1. Keep the fast velocity loop in hardware close to the motors, and have the host send body velocities with a timeout. [X1-S01, PDF p. 4; X1-S35; X1-S30]
2. Use a continuous odom-frame estimate for control and a separate global estimate for goals. [X1-S05, PDF p. 16; X1-S36]
3. In robot_localization, run `two_d_mode` when planar, fuse wheel ẏ = 0, and prefer one pose source and several velocity sources per dimension. [X1-S05, PDF p. 17]
4. Provide absolute heading when fusing GPS, or use a differential IMU input and let the filter derive heading from motion. [X1-S36, "GPS Localization Overview" and "Localization Testing" tip 1]
5. Guard against GNSS jumps and outages with odometry-consistency checks and speed limits during outages. [X1-S09, p. 440; X1-S08, p. 668]
6. Layer safety: a hardware e-stop that cuts motor power, then command timeouts and priority locks, then a supervisory gate. [X1-S16, PDF p. 5; X1-S30, twist_mux.yaml; X1-S38]
7. Log full runs and replay them, and test in simulation with randomised noise before field tests. [X1-S23, PDF pp. 11, 14; X1-S18, PDF p. 33; X1-S07, PDF p. 3]

## Key numbers
| Quantity | Value | Conditions | Source |
|---|---|---|---|
| diff_drive_controller update rate | 20 Hz / 50 Hz / 50 Hz | Husky A200 / Jackal / Warthog | X1-S30 |
| Odometry publish rate | 50 Hz | Clearpath, diff_drive default | X1-S30; X1-S35 |
| `cmd_vel_timeout` | 0.5 s (default), 0.25 s (W200) | ros2_controllers / Clearpath | X1-S35; X1-S30 |
| `wheel_separation_multiplier` | 1.875 / 1.5 / 1.125 | A200 / J100 / W200 | X1-S30 |
| Vendor local EKF rate | 50 Hz (Clearpath), 45 Hz (Robotnik), 30 Hz (Nav2 demo) | shipped configs | X1-S30; X1-S34; X1-S36 |
| AgileX base state loop | 50 Hz | scout_ros2 | X1-S33 |
| Robotnik odom publish | 100 Hz | robot_control.yaml | X1-S34 |
| Stanley UKF / steering | 100 Hz / 20 Hz | DARPA 2005 | X1-S08, pp. 668, 684 |
| Boss POS-LV output | 100 Hz; 0.3 m with GPS, 0.88 m after 1 km without | Applanix spec | X1-S09, p. 439 |
| Autoware localization output | "50Hz~" | design requirement | X1-S37 |
| MPPI samples / rate | 1000 @ 50 Hz or 2000 @ 30 Hz | maintainers' guidance | X1-S05, PDF p. 8 |
| RTK accuracy (claimed) | ~1 cm / 2 cm or better / ~10 cm | Nav2 tutorial / survey / Autoware | X1-S36; X1-S05, PDF p. 20; X1-S37 |
| Standalone GNSS | 1–2 m, up to 10 m / ~10 m | Nav2 tutorial / Autoware | X1-S36; X1-S37 |
| End-to-end latency | ~0.4 s / ~300 ms | Bender 2019 / AMZ | X1-S26, PDF p. 13; X1-S18, PDF p. 33 |
| IMU convergence after driver start | close to 1 min | RoboJackets Yost IMU | X1-S29 |
| IGVC speed limits | ≤5 mph (hardware governed), ≥1 mph average | 2026 rules | X1-S16, PDF p. 5 |

## How it is tested
| Test | What it measures | Pass criterion used in the source | Source |
|---|---|---|---|
| Long-duration navigation | Collisions, recoveries, distance | 0 collisions, 0 e-stops over 37.4 miles | X1-S03, PDF p. 6 |
| Return-to-start (loop closure) on replayed bag | Final position error per sensor set | None stated; errors compared | X1-S07, PDF p. 3 |
| Route vs AMCL ground truth | Final error, % of distance, CPU | None stated; both "well below" 1% | X1-S05, PDF p. 18 |
| GNSS-denied driving | Drift after distance | Stay in lane over 5.7 km | X1-S09, p. 441 |
| Closed-loop drive | Loop-closure error | ~10 cm reported | X1-S27, PDF p. 14 |
| IGVC requirement table | Speed, reaction time, e-stop range, nav accuracy | e.g. reaction <250 ms, accuracy <1 m | X1-S23, PDF p. 14 |

## Common mistakes
- Fusing the jumpy map-frame pose into control. The survey calls `map→base_link` "a poor choice for control algorithms". [X1-S05, PDF p. 16]
- Fusing compass-based IMU yaw in the local filter. Robotnik warns it is "slow-response", and the Nav2 demo comments that real IMU absolute yaw is often inaccurate. [X1-S34; X1-S36, dual_ekf_navsat_params.yaml `imu0_differential` comment]
- Running the OSR default duty mode for autonomy, which its own config says is unsuitable [X1-S39, roboclaw_params.yaml].
- Moving the robot during IMU gyro calibration, which "ruins the process" [X1-S29].
- Ignoring sensor time-stamp stalls, which created phantom obstacles in Stanley [X1-S08, pp. 688–689].

## Disagreements between sources
- **RTK accuracy.** The Nav2 tutorial says "down to 1cm" [X1-S36]. The maintainers' survey says "2 cm or better" [X1-S05, PDF p. 20]. Autoware says "~10cm" [X1-S37]. The first two are level B and A; Autoware's figure is a system-level design value.
- **Standalone GNSS accuracy.** 1–2 m, up to 10 m [X1-S36] and ~10 m [X1-S37], against a team claim of ~2 ft with a NEO-6M [X1-S25, PDF p. 14, level C].
- **Wheel odometry as pose or velocity.** Clearpath fuses wheel x, y, yaw absolutely [X1-S30]. The Nav2 demo and RoboJackets fuse velocities only [X1-S36; X1-S29]. Robotnik fuses wheel x, y differentially [X1-S34].
- **IMU absolute yaw in the filter.** The Nav2 demo fuses absolute IMU yaw in both EKFs [X1-S36]. Clearpath and Robotnik fuse only yaw rate [X1-S30; X1-S34].
- **Odometry rate.** A Nav2 maintainer says real robots typically publish odom at about 100 Hz or higher [X1-S40, level D]. Clearpath ships 50 Hz odometry and a 20 Hz controller loop on Husky A200 [X1-S30, level C].

## Open questions
- No reference stack was found that runs Nav2 MPPI on a Jetson-class computer outdoors and reports loop rates.
- No published stack was found that fuses an Xsens MTi GNSS/INS (internal filter) with robot_localization.
- No published stack was found that drives a tracked chassis through ros2_control with a measured skid correction. The vendor multipliers found are for wheeled skid-steer bases.
- Clearpath's outdoor navigation demos (GPS configuration) were not downloaded, so their shipped GNSS fusion is not covered.
- IGVC reports rarely describe how "navigation accuracy" was measured.
- Kortenkamp & Simmons (X1-S02) and Kato 2018 (X1-S11) have no open copies and were not read.
- Perseverance autonomy (Verma et al. 2023) and a Thorvald or BoniRob path-tracking accuracy paper were not obtained.

## Sources
| ID | Citation | Link | Accessed | File | Level |
|---|---|---|---|---|---|
| X1-S01 | E. Gat, "On Three-Layer Architectures," in D. Kortenkamp, R. P. Bonasso, R. Murphy (eds.), *Artificial Intelligence and Mobile Robots*, AAAI Press, 1998 (author text). | https://u.cs.biu.ac.il/~kaminkg/teach/current/intsys/readings/on-three-layer-arch-tla-1998.pdf | 2026-09-28 | sources/gat_1998_three_layer_architectures.pdf | A |
| X1-S02 | D. Kortenkamp, R. Simmons, "Robotic Systems Architectures and Programming," *Springer Handbook of Robotics*, 2008, pp. 187–206. doi:10.1007/978-3-540-30301-5_9 | https://doi.org/10.1007/978-3-540-30301-5_9 | 2026-09-28 | not downloaded | A |
| X1-S03 | S. Macenski, F. Martín, R. White, J. Ginés Clavero, "The Marathon 2: A Navigation System," IEEE/RSJ IROS 2020, pp. 2718–2725 (arXiv 2003.00368). | https://arxiv.org/abs/2003.00368 | 2026-09-28 | sources/macenski_2020_marathon2_nav2.pdf | A |
| X1-S04 | S. Macenski, T. Foote, B. Gerkey, C. Lalancette, W. Woodall, "Robot Operating System 2: Design, architecture, and uses in the wild," *Science Robotics* 7(66), 2022 (arXiv 2211.07752). | https://arxiv.org/abs/2211.07752 | 2026-09-28 | sources/macenski_2022_ros2_design_architecture.pdf | A |
| X1-S05 | S. Macenski, T. Moore, D. V. Lu, A. Merzlyakov, M. Ferguson, "From the desks of ROS maintainers," *Robotics and Autonomous Systems* 168, 104493, 2023 (arXiv 2307.15236). | https://arxiv.org/abs/2307.15236 | 2026-09-28 | sources/macenski_2023_ros_maintainers_survey.pdf | A |
| X1-S06 | S. Chitta et al., "ros_control: A generic and simple control framework for ROS," *JOSS* 2(20), 456, 2017. | https://doi.org/10.21105/joss.00456 | 2026-09-28 | sources/chitta_2017_ros_control_joss.pdf | A |
| X1-S07 | T. Moore, D. Stouch, "A Generalized Extended Kalman Filter Implementation for the Robot Operating System," IAS-13, AISC 302, Springer, 2016, pp. 335–348. | https://docs.ros.org/en/lunar/api/robot_localization/html/_downloads/robot_localization_ias13_revised.pdf | 2026-09-28 | sources/moore_2014_generalized_ekf_ros.pdf | A |
| X1-S08 | S. Thrun et al., "Stanley: The robot that won the DARPA Grand Challenge," *J. Field Robotics* 23(9), 661–692, 2006. | http://ai.stanford.edu/~gabeh/papers/thrun.stanley05.pdf | 2026-09-28 | sources/thrun_2006_stanley_darpa.pdf | A |
| X1-S09 | C. Urmson et al., "Autonomous driving in urban environments: Boss and the Urban Challenge," *J. Field Robotics* 25(8), 425–466, 2008. | https://www.ri.cmu.edu/pub_files/pub4/urmson_christopher_2008_1/urmson_christopher_2008_1.pdf | 2026-09-28 | sources/urmson_2008_boss_urban_challenge.pdf | A |
| X1-S10 | S. Kato, E. Takeuchi, Y. Ishiguro, Y. Ninomiya, K. Takeda, T. Hamada, "An Open Approach to Autonomous Vehicles," *IEEE Micro* 35(6), 60–68, 2015 (course-hosted copy via Internet Archive). | https://web.archive.org/web/2020/http://cs.furman.edu/~tallen/csc271/source/openAppr.pdf | 2026-09-28 | sources/kato_2015_open_approach_autonomous_vehicles.pdf | A |
| X1-S11 | S. Kato et al., "Autoware on Board," ACM/IEEE ICCPS 2018, pp. 287–296. doi:10.1109/ICCPS.2018.00035 | https://doi.org/10.1109/ICCPS.2018.00035 | 2026-09-28 | not downloaded | A |
| X1-S12 | M. Maimone, Y. Cheng, L. Matthies, "Two Years of Visual Odometry on the Mars Exploration Rovers," *J. Field Robotics* 24(3), 169–186, 2007 (JPL author copy). | https://www-robotics.jpl.nasa.gov/media/documents/rob-06-0081.R4.pdf | 2026-09-28 | sources/maimone_2007_two_years_visual_odometry_mer.pdf | A |
| X1-S13 | M. Maimone, J. Biesiadecki et al., "Overview of the Mars Exploration Rovers' Autonomous Mobility and Vision Capabilities," ICRA 2007 Space Robotics Workshop (JPL copy). | https://www-robotics.jpl.nasa.gov/media/documents/mer_autonomy_icra_2007.pdf | 2026-09-28 | sources/maimone_2007_mer_autonomy_icra.pdf | B |
| X1-S14 | L. Grimstad, P. J. From, "The Thorvald II Agricultural Robotic System," *Robotics* 6(4), 24, 2017. doi:10.3390/robotics6040024 | https://www.mdpi.com/2218-6581/6/4/24 | 2026-09-28 | sources/grimstad_2017_thorvald_ii.pdf | A |
| X1-S15 | A. Ruckelshausen et al., "BoniRob – an autonomous field robot platform for individual plant phenotyping," *Precision Agriculture '09*, Wageningen Academic, 2009, pp. 841–847. | https://www.hs-osnabrueck.de/fileadmin/HSOS/Homepages/COALA/Veroeffentlichungen/2009-JIAC-BoniRob.pdf | 2026-09-28 | sources/ruckelshausen_2009_bonirob.pdf | A |
| X1-S16 | IGVC, "Official Competition Details, Rules and Format," 34th IGVC, Oakland University, 2026. | http://www.igvc.org/2026rules.pdf | 2026-09-28 | sources/igvc_2026_official_rules.pdf | A |
| X1-S17 | R. Volpe, I. Nesnas, T. Estlin, D. Mutz, R. Petras, H. Das, "CLARAty: Coupled Layer Architecture for Robotic Autonomy," JPL technical report, Dec. 2000. | https://www-robotics.jpl.nasa.gov/media/documents/CLARAty.pdf | 2026-09-28 | sources/volpe_2001_claraty.pdf | B |
| X1-S18 | J. Kabzan et al., "AMZ Driverless: The Full Autonomous Racing System," *J. Field Robotics* 37(7), 2020 (arXiv 1905.05150). | https://arxiv.org/abs/1905.05150 | 2026-09-28 | sources/kabzan_2020_amz_driverless.pdf | A |
| X1-S19 | A. Bradford, G. van Breda, T. Fischer, "Racing With ROS 2: A Navigation System for an Autonomous Formula Student Race Car," arXiv 2311.14276, 2023 (peer-reviewed venue not confirmed). | https://arxiv.org/abs/2311.14276 | 2026-09-28 | sources/bradford_2023_racing_with_ros2_formula_student.pdf | C |
| X1-S20 | Sooner Competitive Robotics (Univ. of Oklahoma), "Aluminum Whale," IGVC 2021 Design Report. | http://www.igvc.org/design/2021/7.pdf | 2026-09-28 | sources/sooner_2021_igvc_design_report.pdf | C |
| X1-S21 | Sooner Competitive Robotics, "Weeb Wagon," IGVC 2023 Design Report. | http://www.igvc.org/design/2023/4.pdf | 2026-09-28 | sources/sooner_2023_igvc_design_report.pdf | C |
| X1-S22 | Sooner Competitive Robotics, "Danger Zone," IGVC 2024 Design Report. | http://www.igvc.org/design/2024/17.pdf | 2026-09-28 | sources/sooner_2024_igvc_design_report.pdf | C |
| X1-S23 | Sooner Competitive Robotics, "Twistopher," IGVC 2025 Design Report (submitted 15 May 2025). | http://www.igvc.org/design/2025/7.pdf | 2026-09-28 | sources/sooner_2025_igvc_design_report.pdf | C |
| X1-S24 | Cedarville University AutoNav, "Delta Bee," IGVC 2022 Design Report. | http://www.igvc.org/design/2022/10.pdf | 2026-09-28 | sources/cedarville_2022_igvc_design_report.pdf | C |
| X1-S25 | Ville Robotics (Cedarville University), "ALiEN 6.0," IGVC 2025 Design Report. | http://www.igvc.org/design/2025/13.pdf | 2026-09-28 | sources/villerobotics_2025_igvc_design_report.pdf | C |
| X1-S26 | Boise State University VIP, "Bender," IGVC 2019 Design Report. | http://www.igvc.org/design/2019/12.pdf | 2026-09-28 | sources/boisestate_2019_igvc_design_report.pdf | C |
| X1-S27 | AGV IIT Kharagpur, "Eklavya 6.0," IGVC 2018 Design Report. | http://www.igvc.org/design/2018/8.pdf | 2026-09-28 | sources/iitkgp_2018_igvc_design_report.pdf | C |
| X1-S28 | Project MANAS (Manipal), "STEVE," IGVC 2025 Design Report. | http://www.igvc.org/design/2025/2.pdf | 2026-09-28 | sources/manas_2025_igvc_design_report.pdf | C |
| X1-S29 | Georgia Tech RoboJackets, `igvc-software`, commit bd65158 (2022-07-30): `igvc_navigation/config/ekf_localization_node_params.yaml`, `igvc_navigation/launch/localization.launch`, `igvc_platform/launch/motor_controller.launch`, `documents/research/imu_issue/imu_issue.md` (T. Gemicioglu). | https://github.com/RoboJackets/igvc-software/tree/bd65158fb92d75cae4c6fea9cd330291e12701fc | 2026-09-28 | sources/robojackets_bd65158_ekf_localization_node_params.yaml; robojackets_bd65158_localization.launch; robojackets_bd65158_motor_controller.launch; robojackets_bd65158_imu_issue.md | C |
| X1-S30 | Clearpath Robotics, `clearpath_common` tag 2.9.17 (commit 726d7fb): `clearpath_control/config/{a200,j100,w200}/control/diff_4wd.yaml`, `config/generic/localization.yaml`, `config/twist_mux.yaml`, `launch/localization.launch.py`, `clearpath_generator_common/.../param/platform.py`. | https://github.com/clearpathrobotics/clearpath_common/tree/726d7fba367f18e53899d3ac03f0b276380ae847 | 2026-09-28 | sources/clearpath_2.9.17_a200_diff_4wd.yaml; clearpath_2.9.17_j100_diff_4wd.yaml; clearpath_2.9.17_w200_diff_4wd.yaml; clearpath_2.9.17_generic_localization.yaml; clearpath_2.9.17_twist_mux.yaml; clearpath_2.9.17_localization.launch.py; clearpath_2.9.17_generator_param_platform.py | C |
| X1-S31 | Clearpath Robotics documentation (ROS 2 Humble), "Generators", "Robot YAML Overview", "Platform Overview" (cpr-documentation commit e364165). | https://github.com/clearpathrobotics/cpr-documentation/tree/e364165a48035f9e7a60fe558f2ccfc6c99f3301/docs_versioned_docs/version-ros2humble/ros/config | 2026-09-28 | sources/clearpathdocs_e364165_humble_generators.mdx; clearpathdocs_e364165_humble_yaml_overview.mdx; clearpathdocs_e364165_humble_platform_overview.mdx | B |
| X1-S32 | Clearpath Robotics, "Husky A300 User Manual" (web documentation). | https://docs.clearpathrobotics.com/docs_robots/outdoor_robots/husky/a300/user_manual_husky/ | 2026-09-27 | sources/clearpath_2026_husky_a300_user_manual.md | A |
| X1-S33 | AgileX Robotics / Weston Robot: `ugv_sdk` README (commit f2704ea); `scout_ros2` humble (commit bf0110f) `scout_messenger.hpp`, `scout_base_ros.cpp`; `bunker_ros2` humble (commit c4737f2) `bunker_messenger.hpp`. | https://github.com/agilexrobotics/ugv_sdk/tree/f2704eacdc90357078cd93ec60aae08bb4baab35 ; https://github.com/agilexrobotics/scout_ros2/tree/bf0110ff44cc70cdda2931d69f0e2669a3e26b9d ; https://github.com/agilexrobotics/bunker_ros2/tree/c4737f249129e88c8e9e0bfeb3af81b498a0ebbe | 2026-09-28 | sources/agilex_f2704ea_ugv_sdk_README.md; agilex_bf0110f_scout_messenger.hpp; agilex_bf0110f_scout_base_ros.cpp; agilex_c4737f2_bunker_messenger.hpp | C |
| X1-S34 | Robotnik Automation, `summit_xl_common`, branch `humble` (commit 77a7282; files in ROS 1 launch format): `robot_localization_odom.launch`, `robot_localization_world.launch`, `navsat_transform_node.launch`, `summit_xl_control/config/robot_control.yaml`. | https://github.com/RobotnikAutomation/summit_xl_common/tree/77a728201069cf74c1be04730785086d3f1aa493 | 2026-09-28 | sources/robotnik_77a7282_summit_xl_robot_localization_odom.launch; robotnik_77a7282_summit_xl_robot_localization_world.launch; robotnik_77a7282_summit_xl_navsat_transform_node.launch; robotnik_77a7282_summit_xl_robot_control.yaml | C |
| X1-S35 | ros-controls, `ros2_controllers` tag 2.54.0 (Humble), `diff_drive_controller` parameter file and user doc. | https://github.com/ros-controls/ros2_controllers/tree/2.54.0/diff_drive_controller | 2026-09-27 | sources/ros2controllers_2_54_0_diff_drive_parameters.yaml; ros2controllers_2_54_0_diff_drive_userdoc.rst | B |
| X1-S36 | P. Gonzalez (Kiwibot), "Navigating using GPS Localization," Nav2 docs (docs.nav2.org commit 588d374); `navigation2_tutorials` `nav2_gps_waypoint_follower_demo/config/dual_ekf_navsat_params.yaml` (commit 9f58746). | https://github.com/ros-navigation/docs.nav2.org/blob/588d37415e87eb083500d6c79aaed92ee1285f52/docs/tutorials/general_tutorials/navigation2_with_gps/navigation2_with_gps.md ; https://github.com/ros-navigation/navigation2_tutorials/blob/9f587464617d2939e80d65f2849203207a9e328e/nav2_gps_waypoint_follower_demo/config/dual_ekf_navsat_params.yaml | 2026-09-28 | sources/nav2docs_588d374_navigation2_with_gps.md; nav2tutorials_9f58746_gps_dual_ekf_navsat_params.yaml | B |
| X1-S37 | Autoware Foundation, Autoware Documentation, architecture v1: "Architecture overview", "Control component design", "Localization component design" (commit f43b960). | https://github.com/autowarefoundation/autoware-documentation/tree/f43b9606771ec6badf51d03131d73c0f7b708049/docs/design/autoware-architecture-v1 | 2026-09-28 | sources/autoware_docs_f43b960_arch_v1_index.md; autoware_docs_f43b960_arch_v1_components_control_index.md; autoware_docs_f43b960_arch_v1_components_localization_index.md | B |
| X1-S38 | Autoware Foundation, `autoware_universe` tag 0.52.1 (commit 02a5892): READMEs of `autoware_trajectory_follower_node`, `autoware_vehicle_cmd_gate`, `autoware_mrm_handler`, `autoware_control_validator`. | https://github.com/autowarefoundation/autoware_universe/tree/02a589200c1af644ca4b4cb3ed98695b4b62118b | 2026-09-28 | sources/autoware_universe_0.52.1_trajectory_follower_node_README.md; autoware_universe_0.52.1_vehicle_cmd_gate_README.md; autoware_universe_0.52.1_mrm_handler_README.md; autoware_universe_0.52.1_control_validator_README.md | B |
| X1-S39 | NASA JPL, Open Source Rover `osr-rover-code` (commit 6b17c22): `roboclaw_wrapper.py`, `rover.py`, `roboclaw_params.yaml`, `osr_params.yaml`. | https://github.com/nasa-jpl/osr-rover-code/tree/6b17c22a900182f5339c23ba1f86739b5c9e8c1c/ROS | 2026-09-28 | sources/jpl_osr_6b17c22_roboclaw_wrapper.py; jpl_osr_6b17c22_rover.py; jpl_osr_6b17c22_roboclaw_params.yaml; jpl_osr_6b17c22_osr_params.yaml | B |
| X1-S40 | ros-navigation/navigation2 issue #5524, "Open Loop vs Closed Loop in Controller Server, Velocity Smoother," comments by S. Macenski (maintainer), 2025. | https://github.com/ros-navigation/navigation2/issues/5524 | 2026-09-27 | sources/nav2_gh5524_open_closed_loop.md | D |
