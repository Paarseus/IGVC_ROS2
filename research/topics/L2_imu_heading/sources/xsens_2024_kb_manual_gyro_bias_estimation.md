# Xsens BASE: Manual Gyro Bias Estimation (MGBE)
Source: https://base.xsens.com/s/article/Manual-Gyro-Bias-Estimation?language=en_US (rendered page text, accessed 2026-09-27)


Title
Manual Gyro Bias Estimation (MGBE)
Last Published Date
6/21/2024, 1:54 AM
URL Name
Manual-Gyro-Bias-Estimation
Answer
Introduction

Manual Gyro Bias Estimation (MGBE, formerly know as the No Rotation Update) is a feature that can be used to improve the MTi's internal estimate of biases on the gyroscope data. For a general introduction to sensor biases and their influence, we recommend first reading this article. Manual Gyro Bias Estimation is not supported by the IMU models (MTi-1/10/100/610).

For the Roll and Pitch axes, the gravitational acceleration can be used to determine inertial sensor bias. This is however not possible for the Yaw/Heading axis. If the gyroscope (rate of turn) sensor data bias for the yaw axis is not determined properly by the MTi's on-board filters, then this may result in a drift of the Yaw/Heading estimate. Although the onboard filters continuously estimate sensor biases in the background, a Manual Gyro Bias Estimation can very quickly (in a matter of seconds) help the online filters to determine sensor biases on all axes. It does this by informing the filters that the device will not move at all during a specified period of time. The MTi will use the data recorded during that interval to establish a high confidence gyro bias estimation within the filter.

Alternative Watch at Bilibili

 

Performing a Manual Gyro Bias Estimation

A Manual Gyro Bias Estimation can be performed in various ways.

The most simple case is when using MT Manager. Click the Gyro Bias Estimation icon (). A window will open, that will guide you through the process: 

Figure 1: The Manual Gyro Bias Estimation window in MT Manager.

You can simply set the period of the Manual Gyro Bias Estimation, hold the sensor motionless, and start the procedure by clicking 'Estimate Now'. The default period of six seconds is recommended and sufficient for a decent estimate.

While a Manual Gyro Bias Estimation is active, its corresponding status bit will be high, as shown in Figure 2. At the end of the estimation, there are two possibilities; either the status bit goes back to 0, indicating a successful update, or the status bit stops at 2/3, indicating a failed update. In the latter case, the filter has detected that the device has moved during the update and its result will therefore be neglected.


Figure 2: The status bits in MT Manager showing a successful Manual Gyro Bias Estimation (left) and a failed Manual Gyro Bias Estimation (right).                                                                                                          
The Manual Gyro Bias Estimation can also be performed using the SetNoRotation Low Level Communication command (MID 0x22). An example use case could be an autonomous ground vehicle; every time the control software of the robot decides that the robot should stop moving, it might as well send out a SetNoRotation command to the MTi. Note that in order to do this, the device has to be in Measurement mode, as live gyroscope data is required to estimate sensor bias. For more information on the use of this command, refer to the Low Level Communication Protocol Documentation. 
Finally, the Manual Gyro Bias Estimation can be performed by using the Xsens Device API. We recommend reading this BASE post for an example C++ snippet. Some processors (e.g. ARM) cannot use the XDA library. This BASE post shows an example C++ snippet for such cases. 

 

Best practices

Note that gyro biases may change during the warm up period, as they depend on temperature. It is therefore important to properly warm up the MTi before performing a Manual Gyro Bias Estimation (at least 5 minutes, preferably 10 minutes or more). In addition, one could perform an update directly after turning on the MTi as well as after 10 minutes. Note that a failed MGBE (as a result of movement during the update period) should not cause any additional errors - the MGBE result will simply be rejected in that case. An exception is the case where the device is rotated at a very constant angular velocity during the full update period.

It is not possible to write sensor bias estimates to the MTi's internal memory (not even by clicking 'Write to MT'). The MTi will always initialize using the sensor bias estimates as determined by the factory calibration.

Finally, you may also notice that the inertial sensor data plots will not show a change after the MGBE. This is expected behavior, as the gyro bias estimation only affects the internal states in the filter. The inertial data with biases removed is used in the internal sensor fusion filter to calculate an accurate orientation estimate. However if you would like to use the sensor's accelerometer or gyroscope inertial data outputs, then you will still need to remove the biases from those. We do not remove the biases on the outputs with our internal bias estimation values, because we do not want the data to have unknown offsets that may change suddenly without the customer knowing what is going on (which would be very undesirable for many control systems).

 

Implement In the Code

 

1) Xbus Low Level:

It is recommended to implement right after power on, send a sequence of commands as below:

GoToConfig:  FA FF 30 00 D1

GoToMeasurement:  FA FF 10 00 F1

SetNoRotation, 6 second as an example:  FA FF 22 02 00 06 D7
Click the Device Data View and Status Data button;
Click Goto Config button
Click Goto Measurement button
Paste the FA FF 22 02 00 06 D7 into the Message input box, click "Send".

2) Periodical Manual Gyro Bias Estimation with ROS Driver(MT SDK):

Example code:

bool XdaInterface::manualGyroBiasEstimation(uint16_t duration)
{
    // Check if duration is less than 2; if so, set it to 2
    if (duration < 2)
    {
        ROS_INFO("Duration is less than 2 seconds, setting it to 2 seconds.");
        duration = 2;
    }

    XsMessage snd(XMID_SetNoRotation, sizeof(uint16_t));
    XsMessage rcv;
    snd.setDataShort(duration);
    if (!m_device->sendCustomMessage(snd, true, rcv, 1000))
        return false;

    //ROS_INFO("Manual Gyro Bias Estimation sent at %d seconds.", duration);
    return true;
}

 

 

By default, the periodical MGBE is not enabled in the ROS Driver.

User could change the parameters below to enable it, then a separate threading would send the setNoRotation with the defined duration and gap.

enable_manual_gyro_bias: true

# Parameters for manual gyro bias estimation:
# - 'event_interval': Time in seconds between two consecutive invocations.
# - 'duration': Time in seconds for which the MGBE process executes each time.
#   The minimum value for 'event_interval' is 10 seconds, and the minimum value  for 'duration' is 2 seconds.
manual_gyro_bias_param: [10, 3] # [event_interval, duration]

Monitoring MGBE Success:
To verify if MGBE was successful, use 'rostopic echo /status'.
If the 'no_rotation_update_status' changes from 3 to 0, MGBE was successful.

 

 

 

Inertial Sensor Modules
Algorithms, Data & Theory
