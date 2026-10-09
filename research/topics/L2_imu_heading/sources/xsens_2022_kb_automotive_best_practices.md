# Xsens BASE: Best Practices for Automotive Applications
Source: https://base.xsens.com/s/article/Best-Practices-for-Automotive-Applications?language=en_US (rendered page text, accessed 2026-09-27)


Title
Best Practices for Automotive Applications
Last Published Date
6/30/2022, 9:59 AM
URL Name
Best-Practices-for-Automotive-Applications
Answer

Xsens MTi AHRS and GNSS/INS products are used in a large variety of automotive applications, such as autonomous cars, trucks, shuttles and trains. The Xsens MTi GNSS/INS are designed to use fused IMU and GNSS data to improve orientation and position estimates of a vehicle in motion. This also helps compensate for transient accelerations. However, the estimates of position, velocity, and heading may be challenged in automotive applications when GNSS reception is limited, when maneuvering at slow speeds or in static conditions. This article summarizes our recommendations for the use of AHRS and GNSS/INS products in automotive applications, including aspects such as mechanical installation and software settings, such that your MTi product can reach the highest performance.

The primary documentation for configuring and getting the most out of your MTi device can be found in the product's Datasheets and Manuals.

Here are some additional general tips for getting the most out of an AHRS or GNSS/INS device for automotive applications:
 

Filter Profiles – Selecting the right Filter Profile for an application will improve the MTi’s performance in that automotive environment by tuning it specifically for the expected dynamics and external disturbances. See below for recommended filter profiles in automotive applications:

MTi-G-710:
The Automotive filter profile assumes that the yaw of the MTi-G-710 is also the GNSS course over ground (holonomic constraints). This assumption holds for most automotive/ground vehicles, except for those who experience side slip, such as racing cars, tracked vehicles, some articulated vehicles (depending on where the MTi-G-710 is mounted) and vehicles driving on rough terrain. The Automotive filter profile thus primarily uses GNSS to determine the yaw. Note that it is essential to mount MTi exactly in the direction of movement in order to prevent an offset. When GNSS is lost, yaw will be determined by the velocity estimation algorithm for 45 seconds, before yaw is determined by gyroscopes integration only. Should GNSS outages recur regularly or if you have bad GNSS-availability (e.g. in urban canyons), consider using HighPerformanceEDR.
The GeneralMag filter profile bases its yaw mainly on magnetic heading, together with comparison of GNSS acceleration and the accelerometers. Although this combination makes the yaw more robust than magnetic field alone, a homogeneous or calibrated-for magnetic field is essential for good-performance yaw. Other parameters are tuned the same as in the General filter profile.
The HighPerformanceEDR filter profile is specially designed for ground-based navigation applications where deteriorated GNSS conditions and GNSS outages (0- 600s) are a regular feature. Note that the accuracy of position, velocity and orientation estimates will still deteriorate during GNSS outages. This filter profile does not use the holonomic constraints and thereby removes the need for mounting considerations. Target applications: slow moving ground vehicles and locomotive navigation. The filter profile HighPerformanceEDR automatically estimates the gyro bias when the MTi is not moving. The sensor fusion algorithm automatically detects when the MTi is motionless. Vibrations and very slow movements may influence the accuracy of the gyro bias estimation.
MTi-300:
The General filter profile is the default setting. It assumes moderate dynamics and a homogeneous magnetic field. External magnetic distortions are considered relatively short (up to ~20 seconds). A homogeneous magnetic environment and a proper magnetic calibration are essential for a good performance.
The low_mag_dep filter profile assumes that the dynamics are relatively low and that there are long-lasting external magnetic distortions. Also, use this filter profile when it is difficult to do a very good magnetic calibration. The use of the low_mag_dep filter profile can be useful to limit drift in heading whilst not being in a homogeneous magnetic field.
MTi-680(G), MTI-670(G), MTi-7 and MTi-8:
The General/General_RTK filter profile is the default setting. The yaw output of the MTi is initialized at 0 degrees at power-up and it will converge to a North referenced yaw when the MTi starts moving and GNSS data is available.
The GeneralMag/GeneralMag_RTK filter profile bases its yaw estimate mainly on magnetic heading and GNSS measurements. A homogeneous magnetic environment and a proper magnetic calibration are essential for a good performance. This filter profile produces a North referenced yaw output directly after powering up the MTi.
MTi-630(R):
The MTi-630(R) offers a tiered approach allowing the user to select a base filter profile and a heading behavior separately:
The Robust base filter profile is suitable for most of the applications, and the recommended profile for automotive applications. Compared to the other base filter profiles it has a more robust tuning. When the MTi is static, an automatic gyro bias estimation is performed in the background.
The NorthReference heading behavior assumes a homogeneous magnetic environment that can be used to estimate a stable North referenced heading. A proper magnetic calibration is essential when using this heading behavior. If this is not possible, considering using the FixedMagRef heading behavior instead.
 

Magnetic Calibration -- Magnetic calibration is important for preventing ferromagnetic materials both in the application and in the environment from distorting the magnetic field data. A proper magnetic calibration is essential for a good performance if you have selected a filter profile that uses the magnetometers to estimate heading. More information on mitigating these magnetic distortions can be found below. For automotive applications, the easiest way to perform a magnetic calibration is by running the Magnetic Field Mapper tool while driving several (at least 3) circles. 

Magnetic Calibration Manual
Magnetic immunity of the MTi AHRS
 

Alignment -- When installing the MTi in an automotive application, it is important to understand the alignment of the vehicle’s reference frame with the sensor’s reference frame. By default, the MTi uses the East-North-Up (ENU) reference frame for its orientation and velocity estimates. Orientation is given as the differences between this reference frame and the MTi’s sensor frame (X-Y-Z axes). Sensor data (acceleration, rate of turn and magnetic field) are provided in the MTi's sensor frame. In order to change the reference frames of the MTi, see this article on Changing or Resetting the MTi reference co-ordinate systems.

GNSS Configuration -- MTi GNSS/INS devices with external GNSS receiver support (MTi-7, MTi-8, MTi-670, MTi-680) can be configured for supported GNSS receiver options. MTi-7, MTi-8, MTi-670, MTi-680 officially support the u-blox MAX family of GNSS receivers and any receiver using NMEA string data types. Additionally, beta firmware releases have been released to support the u-blox ZED and u-blox NEO GNSS receiver families. These can be configured using the SetGnssReceiver command in MT Manager or via XDA commands. For more details on low-level commands, we refer to the Low Level Communication Protocol Documentation.

Initial Heading -- When powered on and not using the GeneralMag filter profile, the Xsens MTi GNSS/INS will output an arbitrary heading of 0 degrees until it starts moving with a GNSS fix and gains the correct North-referenced heading. To compensate for this, the initial heading of the MTi can be configured with an estimated initial heading until there is a good GNSS fix using the SetInitialHeading function. By applying an initial estimate of the device’s heading, the MTi will maintain an improved level of accuracy until the GNSS data allows the device to estimate a proper heading estimate. For more details on low-level commands, we refer to the Low Level Communication Protocol Documentation.
 

NVIDIA Drive -- The Xsens MTi-G-710 is officially supported with the NVIDIA Drive AGX platform and is the featured inertial measurement unit of the NVIDIA Drive Hyperion 7 Developer Kit. The MTi-G-710 provides the NVIDIA Drive AGX with high-rate 9-axis IMU data, orientation estimation, and GNSS position and velocity to allow NVIDIA’s deep learning algorithms to react quickly and accurately to the environment, leading to a safer and more reliable autonomous vehicle. For more information refer to: https://developer.nvidia.com/drive

Vibration Handling – The MTi samples IMU signals at 10kHz per channel, processing them using a strapdown integration algorithm with coning/sculling compensation, mitigating errors when the device is under vibration. In general, it is still best practice to mechanically isolate the MTi as much as possible, particularly under the following conditions:

The magnitude of the vibration is larger than the measurement range of the accelerometer. This will cause the accelerometer to saturate, which may be observed as a “drift” in the zero-level of the accelerometer. This will show up as an erroneous roll/pitch.
The frequency of the vibration is higher than the bandwidth of the accelerometer. In theory, such vibrations are rejected, but in practice they can still give rise to aliasing, especially if close to the bandwidth limit. This can be observed as a low frequency oscillation. Further, high frequency vibrations often tend to have large acceleration amplitudes (see item 1).

If the MTi is encountering significant vibration, it is recommended to mechanically dampen the MTi with vibration dampeners to remove the metal-to-metal connection that transduces vibrations to the MTi. Tested with vibrations up to 1200 Hz, the recommended dampeners for the MTi-G-710 are manufactured by Norelem with part number 26102-00800855.

The recommended placement of the MTi in an automotive application is underneath the driver’s seat in order to minimize vibrations. Other locations that have been used include mounted in the trunk, aligned with the front or rear axle, or fixed to the floor of the passenger seat. Regardless of the location, it is still recommended to mechanically dampen the vibration as well.
 

Antenna Placement and antenna offset (lever arm) – The recommended placement of the GNSS antenna is mounted on the roof with a clear view of the sky. Other locations include the hood of the vehicle or on top of the trunk.

With the release of the MTi-680(G) RTK GNSS/INS, Xsens has also introduced a new configuration parameter called the GNSS lever arm. The GNSS lever arm is an essential parameter in order to achieve reliable cm-level position, velocity and orientation data. The article The GNSS lever arm (antenna offset) and its role in the GNSS/INS sensor fusion algorithm provides a more in-depth analysis of the GNSS lever arm.
 

Minimum Speed to Estimate Yaw in GNSS/INS Devices –When using GNSS combined with the MTi’s accelerometers to estimate the heading, it is recommended to maintain a minimum velocity of 7 m/s and the more acceleration and movement the better for yaw estimation performance.

 

Inertial Sensor Modules
Algorithms, Data & Theory
