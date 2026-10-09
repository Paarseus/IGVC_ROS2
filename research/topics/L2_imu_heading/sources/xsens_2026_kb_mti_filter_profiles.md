# Xsens BASE: MTi Filter Profiles
Source: https://base.xsens.com/s/article/MTi-Filter-Profiles-1605869708823 (rendered page text, accessed 2026-09-27)

Title
MTi Filter Profiles
Last Published Date
5/27/2026, 7:26 AM
URL Name
MTi-Filter-Profiles-1605869708823
Answer

The Xsens sensor fusion algorithm optimally estimates the orientation with respect to an Earth fixed frame utilizing the 3D inertial sensor data (orientation and velocity increments), 3D magnetometer, and GNSS data (if a GNSS/INS device).

The user can set the sensor fusion algorithm with different filter profiles in order to get the best performance based on the application scenario. These filter profiles contain predefined filter parameter settings suitable for different user application scenarios. Every application is different and although example applications are listed, results may vary from setup to setup. It is recommended to reprocess recorded data with different filter profiles in MT Manager to determine the best results in your specific application.

Below are the available filter profiles for the latest stable firmware release for the Xsens range of the industrial sensor products (MTi, Sirius, Avior).

 

GNSS/INS Devices
Filter Profiles for MTi-G-710

 

The General filter profile is the default setting. It makes few assumptions about movements. Yaw is referenced by comparing GNSS acceleration with the on-board accelerometers, so the more movement (when GNSS is available) will result in a better yaw. Altitude (height) is determined by combining static pressure, GNSS altitude and accelerometers. The barometric baseline is referenced by GNSS, so during GNSS outages, accurate height measurements are maintained because this barometric baseline is monitored.

The GeneralNoBaro filter profile is very similar to the general filter profile. However, it does not use the barometer for height estimation (it thus uses GNSS and accelerometers only). Since airflows near the venting holes in the MTi-G will lower the barometric pressure (and thus make height estimations inaccurate), you can use this filter profile when the MTi-G is mounted in such airflow.

The GeneralMag filter profile bases its yaw mainly on magnetic heading, together with comparison of GNSS acceleration and the accelerometers. Although this combination makes the yaw more robust than magnetic field alone, a homogeneous or calibrated-for magnetic field is essential for good-performance yaw. Other parameters are tuned the same as in the General filter profile.

The Automotive filter profile assumes that the yaw of the MTi-G-710 is also the GNSS course over ground (holonomic constraints). This assumption holds for most automotive/ground vehicles, except for those who experience side slip, such as racing cars, tracked vehicles, some articulated vehicles (depending on where the MTi-G-710 is mounted) and vehicles driving on rough terrain. The Automotive filter profile thus uses GNSS to determine the yaw. Note that it is essential to mount MTi-G exactly in the direction of movement in order to prevent an offset. Please refer to 5.2.3 for proper mounting. When GNSS is lost, yaw will be determined by the velocity estimation algorithm for 45 seconds, before yaw is determined by gyroscopes integration only. Should GNSS outages occur regularly or if you have bad GNSS-availability (e.g. in urban canyons), consider using HighPerformanceEDR.

The HighPerformanceEDR filter profile replaces the previously available AutomotiveUrbanCanyon filter profile. This filter profile is specially designed for ground-based navigation applications where deteriorated GNSS conditions and GNSS outages (0- 600s) are a regular feature. Note that the accuracy of position, velocity and orientation estimates may still deteriorate during GNSS outages. This filter profile does not use the holonomic constraints and thereby removes the need for mounting considerations. Target applications: slow moving ground vehicles and locomotive navigation. The filter profile HighPerformanceEDR automatically estimates the gyro bias when the MTi is not moving. The sensor fusion algorithm detects when the MTi is motionless. Vibrations and very slow movements may influence the accuracy of the gyro bias estimation.

Every application is different and although example applications are listed above, results may vary from setup to setup. It is recommended to reprocess recorded data with different filter profiles in MT Manager to determine the best results in your specific application.
 

Filter Profiles for MTi-680(G), MTi-670(G), MTi-7 and MTi-8

 

* External Aiding Sensor for MTi-7

** RTK filter profiles apply to MTi-8 and MTi-680(G)

*** This filter profile can be used even when the barometer is not part of the design for the MTi-7/8.

 

The General/General_RTK filter profile is the default setting. Yaw is North referenced (when GNSS is available). Altitude (height) is determined by combining static pressure, GNSS altitude and accelerometers. The barometric baseline is referenced by GNSS, so during GNSS outages, accuracy of height measurements is maintained.

The GeneralNoBaro/GeneralNoBaro_RTK filter profile is very similar to the general filter profile except for the use of barometer.

The GeneralMag/GeneralMag_RTK filter profile bases its yaw estimate mainly on magnetic heading and GNSS measurements. A homogeneous or magnetic field calibration is essential for good performance.



Filter Profiles for Sirius GNSS/INS and RTK GNSS/INS
Name	Product	Typical Applications
General-INS	
Sirius GNSS/INS
Sirius RTK GNSS/INS
	
Default and currently only filter profile

 

The General-INS filter profile is currently the only filter profile available for Sirius GNSS/INS and RTK GNSS/INS devices. This filter profile is the default setting. Heading behaviour is determined by the heading profile. Altitude (height) is determined by GNSS altitude and accelerometers. When the sensor is static, an automatic gyro bias estimation is performed in the background.  



Heading Behaviour* for Sirius GNSS/INS and RTK GNSS/INS

Name	Product	Typical Applications
NoMag	
Sirius GNSS/INS
Sirius RTK GNSS/INS
	

Environments where the magnetic field cannot be trusted (e.g. stabilized antenna platforms or pipeline inspection tools). 


MagAided	
Sirius GNSS/INS
Sirius RTK GNSS/INS
	

All applications that are used in environments where different magnetic fields are present (e.g. mixed indoor/outdoor applications).

 

* The user can configure different algorithm behaviours by selecting a “base” filter profile and, next to it, a heading behaviour. The “base” filter profile selection affects the general behaviour of the device, mainly based on the nature of the typical expected dynamics of the application. The heading behaviour, as the name suggests, affects the heading/yaw output of the sensor, and determines how the magnetometer measurements are interpreted. This tiered approach gives more freedom to select the desired behaviour for different user application scenarios.

 

The NoMag heading behaviour starts with an heading initialized at 0° (similar to non GNSS/INS VRU heading behaviour) when the MTi is powered up. Once a GNSS fix is obtained, the yaw will converge to a North-referenced heading. The magnetic field is not used to estimate the yaw.

The MagAided heading behaviour is mainly  based on magnetic heading, together with comparison of GNSS acceleration and the accelerometers. Although this combination makes the yaw more robust than magnetic field alone, a homogeneous or calibrated-for magnetic field is essential for good-performance yaw.

 

AHRS and VRU/AHT Devices
Filter Profiles for MTi-200 VRU/AHT and MTi-300 AHRS

 

The General filter profile is the default setting. It assumes moderate dynamics and a homogeneous magnetic field. External magnetic distortions are considered relatively short (up to ~20 seconds). Typical applications include camera tracking (e.g. TV camera’s), remotely operated robotic arms on ROV’s etc.

The High_mag_dep filter profile assumes homogeneous magnetic field and an excellent Magnetic Field Mapping. This filter profile heavily relies on the magnetometer for heading. Dynamics of the motion are relatively slow. Typical applications are navigation of ROV’s or the control of small unmanned helicopters.

The Dynamic filter profile assumes jerky motions. However, the assumption is also made that there is no GNSS available and/or that the velocity is not very high. In these conditions a 100-series MTi may be a better choice. The dynamic filter profile uses the magnetometer for stabilization of the heading, and assumes very short magnetic distortions. Typical applications are where the MTi is mounted on persons or hand-held (e.g. HMD, sports attributes etc.).

The Low_mag_dep filter profile assumes that the dynamics is relatively low and that there are long-lasting external magnetic distortions. Also use this filter profile when it is difficult to do a very good Magnetic Field Mapping (MFM). The use of the low_mag_dep filter profile can be useful to limit drift in heading whilst not being in a homogeneous magnetic field. Typical applications are large vessels and unmanned ground vehicles in buildings.

The VRU_general filter profile assumes moderate dynamics in a field where the magnetic field cannot be trusted at all and benefits from the Active Heading Stabilization feature. It is also possible to use this filter profile in situations where an alternative source of yaw is available. Yaw from the VRU/AHT is unreferenced; note however, that because of the working principle of the VRU/AHT, the drift in yaw will be much lower than when gyroscope signals would be integrated. Typical applications are stabilized antenna platforms mounted on cars of ships and pipeline inspection tools. This filter profile is the only one available for the MTi-20/200 VRU/AHT.

 

Filter Profiles for MTi-620, Sirius VRU and Avior VRU, and MTi-630(R), Sirius AHRS and Avior AHRS
Name	Product	Typical Applications
Responsive	
MTi-620(R)
MTi-630(R)
Sirius VRU
Sirius AHRS
Avior VRU
Avior AHRS
	
Outdoor/Indoor handling objects
Indoor ground vehicles
Outdoor/Indoor head tracker
Indoor mapping, outdoor mapping if handheld (e.g. tripods with camera, backpack)
Industrial robotic arm

Robust	
MTi-620(R)
MTi-630(R)
Sirius VRU
Sirius AHRS
Avior VRU
Avior AHRS
	
Ships/vessels
Automotive
Ground vehicles outdoor
Outdoor mapping with vehicles

General	
MTi-620(R)
MTi-630(R)
Sirius VRU
Sirius AHRS
Avior VRU
Avior AHRS
	
Automotive
Ground vehicles outdoor
Outdoor mapping with vehicles

 

The Responsive filter profile is designed for indoor applications as well as applications that experience high dynamics and jerky movements. With the Avior and Sirius, when the sensor is static, an automatic gyro bias estimation is performed in the background. With the MTi-630(R), this only happens with the FixedMagRef or NorthRef heading profiles (see below).

The Robust filter profile is suitable for most of the applications. Compared to the other filter profiles it has a more robust tuning. With the Avior and Sirius, when the sensor is static, an automatic gyro bias estimation is performed in the background. With the MTi-620(R) and MTi-630(R), this only happens with the FixedMagRef or NorthRef heading profiles (see below).

The General filter profile is only recommended for users who are looking for similar behaviour as the previous generation Xsens products in the typical applications suggested in the table. Using the General filter profile is not recommended for new designed applications. This filter profile behaves like the General filter profile implemented for the previous generation Xsens Products (e.g. MTi-30). It is more sensitive to the magnetic field changes. It does not perform an automatic gyro bias estimation in background. This filter profile cannot be combined with the FixedMagRef heading behaviour.

 

Heading behaviour* for MTi-620, Sirius VRU and Avior VRU, and MTi-630(R), Sirius AHRS and Avior AHRS

Name	Product	Typical Applications
NorthReference	
MTi-630(R)
Sirius AHRS
Avior AHRS
	

All applications that require a North referenced heading and are used in a homogeneous magnetic field.


FixedMagRef	
MTi-630(R)
Sirius AHRS
Avior AHRS
	

All applications that are used in environments where different magnetic fields are present (e.g. mixed indoor/outdoor applications).


VRU	
MTi-620(R)
MTi-630(R)
Sirius VRU
Sirius AHRS
Avior VRU
Avior AHRS
	

Applications where only roll and pitch is of interest and/or applications that are used in environments where the magnetic field cannot be trusted (e.g. stabilized antenna platforms or pipeline inspection tools).




VRUAHS	
MTi-620(R)
MTi-630(R)
Sirius VRU
Sirius AHRS
Avior VRU
Avior AHRS
	Scenarios where the magnetic field cannot be trusted completely, but a stable yaw is needed.

 

* With the MTi-620, MTi-630(R), Sirius VRU/AHRS and Avior VRU/AHRS, the user can configure different algorithm behaviours by selecting a “base” filter profile and, next to it, a heading behaviour. The “base” filter profile selection affects the general behaviour of the device, mainly based on the nature of the typical expected dynamics of the application. The heading behaviour, as the name suggests, affects the heading/yaw output of the sensor, and determines how the magnetometer measurements are interpreted. This tiered approach gives more freedom to select the desired behaviour for different user application scenarios.

The NorthReference heading behaviour assumes a homogeneous magnetic environment that can be used to estimate a stable North referenced heading.

The FixedMagRef heading behaviour is based on the idea that the heading is not necessarily referenced to the local magnetic North. Instead, it maintains a fixed heading reference frame based on what is defined when the sensor is powered up (based on the initially observed magnetic field). This means that there is no drift with respect to the starting frame when the local magnetic field changes. For example, when moving from room A to room B, where room B has a different local magnetic field direction than room A, the heading output of the sensor does not change. This is in contrast to the NorthReference heading behaviour, which forces the sensor to estimate the heading based on the local magnetic field.

For the VRU heading behaviour, the yaw is unreferenced. This means that it is initialized at 0° when the sensor is powered up and the yaw will be computed relative to this initial orientation. The magnetic field is not used to estimate the yaw. Because of small inaccuracies that originate when integrating gyroscope data, the Yaw output will contain an error that builds up over time, also known as “drift”. Note however, that because of the working principle of the sensor fusion algorithm, the drift in yaw will be much lower than when gyroscope signals would be simply integrated.

The VRUAHS heading behaviour activates the Active Heading Stabilization (AHS) on top of the above described VRU behaviour. AHS is a software component within the sensor fusion engine designed to give a low-drift unreferenced heading solution, even in a disturbed magnetic environment. The yaw remains unreferenced, but the drift is limited. For more information on the capabilities of AHS, refer to the BASE article: Active Heading Stabilization (AHS) - Note that in the previous Xsens products, AHS was activated by means of a separate setting.

Filter Profiles for MTi-2 VRU/AHT and MTi-3 AHRS

 

