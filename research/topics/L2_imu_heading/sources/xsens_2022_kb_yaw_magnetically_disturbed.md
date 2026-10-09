# Xsens BASE: Estimating Yaw in magnetically disturbed environments
Source: https://base.xsens.com/s/article/Estimating-Yaw-in-magnetically-disturbed-environments?language=en_US (rendered page text, accessed 2026-09-27)


Title
Estimating Yaw in magnetically disturbed environments
Last Published Date
9/30/2022, 5:28 PM
URL Name
Estimating-Yaw-in-magnetically-disturbed-environments
Answer

By default, the MTi-3/30/630(R)/300 AHRS and the MTi-7/MTi-8/670(G)/680G/MTi-G-710 GNSS/INS (when using the GeneralMag filter profile) stabilize Yaw using the local Earth's magnetic field. In other words, the measured magnetic field is used as a compass. In addition, the gyroscope biases are continuously estimated by the MTi's on-board filter. For the rate of turn around the x-axis and the y-axis (roll and pitch axes), the gyroscope bias is estimated using gravity (i.e. by using the accelerometers). In a homogeneous magnetic field, the gyroscope bias around the z-axis can be successfully estimated as well by monitoring the direction of the magnetic field. 
 

The magnetic field can be distorted by the presence of ferromagnetic materials, permanent magnets or power lines with strong currents (several amperes) in the vicinity of the device. The distance to the object and the amount of ferromagnetic material determines the magnitude of disturbance introduced. If the local Earth magnetic field is temporarily disturbed, the on-board filters will initially track this disturbance instead of incorrectly assuming that the device has rotated. However, in case of continuous magnetic disturbances (>10 to 30 s, depending on the filter settings) the computed heading will slowly converge to a new solution using the 'new' local magnetic north. Note that the magnetic field has no direct effect on the inclination estimate.
 

In the special case that the MTi is rigidly strapped to an object containing ferromagnetic materials, constant magnetic disturbances will be present. Using a so-called 'magnetic field mapping' (MFM, i.e. a 3D calibration for soft and hard iron effects), these magnetic disturbances can be completely calibrated for, allowing the MTi to be used as if it would not be secured to the object containing ferromagnetic materials. In order to tackle this type of distortions, refer to:

Magnetic Field Mapping (MFM), or 
In-Run Compass Calibration (ICC), in case using the MFM is not possible.

For more information on both calibration tools, refer to the Magnetic Calibration Manual .

If the magnetic distortions are temporal or spatial, they cannot be calibrated for by magnetic field mapping tools. Instead, there are other ways to mitigate these effects. They are described in the following articles and documentation.

Active Heading Stabilization (AHS)
Manual Gyro Bias Estimation
 

Also consider reading this article for a list of possible causes of incorrect or drifting heading estimates. 

Inertial Sensor Modules
Algorithms, Data & Theory
