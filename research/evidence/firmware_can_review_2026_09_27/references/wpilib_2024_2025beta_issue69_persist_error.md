<!-- Source: https://github.com/wpilibsuite/2025Beta/issues/69 (issue + all comments via GitHub API, fetched 2026-09-27). Comments by jfabellera are from a REV Robotics software engineer (REVLib contributor). -->
# REVLib HAL Error: Unknown error status Persist Parameters

Opened 2024-12-10T21:43:18Z by thenetworkgrinch, state closed

**Describe the bug**
While using YAGSL 2025, the drive motors throw the message `"ERROR ﻿﻿ 4 ﻿﻿ [CAN SPARK] IDs: 22, 24, 26, WPILib or External HAL Error: Unknown error status Persist Parameters"` as a driver station error without crashing the program. As a result of calling `motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);`. 

**To Reproduce**
Steps to reproduce the behavior:
1. Create a `SparkMax` object
2. Configure the config with the following.
```java
 SparkMaxConfig            cfg                    = new SparkMaxConfig();
cfg.signals
        .absoluteEncoderPositionAlwaysOn(false)
        .absoluteEncoderVelocityAlwaysOn(false)
        .analogPositionAlwaysOn(false)
        .analogVelocityAlwaysOn(false)
        .analogVoltageAlwaysOn(false)
        .externalOrAltEncoderPositionAlwaysOn(false)
        .externalOrAltEncoderVelocityAlwaysOn(false)
        .primaryEncoderPositionAlwaysOn(false)
        .primaryEncoderVelocityAlwaysOn(false)
        .iAccumulationAlwaysOn(false)
        .appliedOutputPeriodMs(10)
        .faultsPeriodMs(20);

      cfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
      cfg.encoder
          .positionConversionFactor(12.8) 
          .velocityConversionFactor(12.8/ 60);
    
      cfg.encoder
          .quadratureMeasurementPeriod(10)
          .quadratureAverageDepth(2);

      
      cfg.signals
          .primaryEncoderVelocityAlwaysOn(true) // Changes to `true` for drive motors, `false` for angle/azimuth motors
          .primaryEncoderPositionAlwaysOn(true)
          .primaryEncoderPositionPeriodMs(20);
 motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters)
```

 - Link to code (if applicable):
https://github.com/BroncBotz3481/YAGSL-Example/blob/dev/src%2Fmain%2Fjava%2Fswervelib%2Fmotors%2FSparkMaxSwerve.java#L290C5-L327C47

**Expected behavior**
Expected to not throw the HAL error.

**Desktop (please complete the following information if applicable):**
 - OS: Win 10
 - OS Language: English
 - Project Information: 
WPILib Information:
Project Version: 2025.1.1-beta-2
VS Code Version: 1.95.3
WPILib Extension Version: 2025.1.1-beta-2
C++ Extension Version: 1.22.9
Java Extension Version: 1.36.2024092708
Java Debug Extension Version: 0.58.2024090204
Java Dependencies Extension Version 0.24.0
Java Version: 17
Java Location: C:\Users\Public\wpilib\2025\jdk
Vendor Libraries:
maplesim (0.2.4)
PathplannerLib (2025.0.0-beta-5)
CTRE-Phoenix (v5) (5.34.0-beta-3)
CTRE-Phoenix (v6) (25.0.0-beta-3)
photonlib (v2025.0.0-beta-6)
ReduxLib (2025.0.0-beta0)
REVLib (2025.0.0-beta-3)
Studica (2025.1.1-beta-3)
WPILib-New-Commands (1.0.0)
YAGSL (2025.1.0)

**roboRIO (please complete the following information if applicable):**
 - roboRIO 1
 - Image version: `FRC_roboRIO_2025_v1.1.zip`

**Additional context**
Is there a way to get more info on this HAL error? This may not be related but this is how the velocity PID control is reacting.
https://drive.google.com/file/d/1D1L4oIacTZSiN7mFE15kobOfwW1COfw9/view?usp=sharing

The team code that is causing this is here
https://github.com/FRC3546/YAGSL-2025-beta/tree/main



---
**thenetworkgrinch** 2024-12-10T21:44:02Z:

CD thread is here https://www.chiefdelphi.com/t/trouble-with-yagsl-2025-beta/476325/30


---
**thenetworkgrinch** 2024-12-10T22:22:12Z:

The next release of YAGSL has the configuration redundancy check added back to it and this may go away after that. I thought the internal redundancy check would catch this?


---
**jfabellera** 2024-12-10T22:32:41Z:

Ran this on my own setup with just two SPARK MAXes and I am not seeing that error.

An error associated with a command like that one usually means that sending the command failed, a status response for that command was not received, or the status response was received but has an unknown error code. In this specific case, I believe the status being returned is non-zero (error) and unknown which is particularly odd.

Also errors are grouped up and flushed periodically to the driver station which explains why ID 20 is in it's own error message.

> The next release of YAGSL has the configuration redundancy check added back to it and this may go away after that. I thought the internal redundancy check would catch this?

What is the redundancy check in YAGSL for? We have checks for certain configurations against the detected device where possible.

Also unrelated, I notice that they are setting the quadrature encoder average depth and measurement period, but if it is a SPARK MAX, then that only affects an encoder connected to the front port while in brushed mode and I assume they are using a NEO.



---
**thenetworkgrinch** 2024-12-10T22:42:24Z:

I am about to head to the shop and will answer your questions later but can you try this in a high CAN traffic env? 

Say like 12 sparkmaxes getting configured all at the same time then setReference called every cycle just for good measure, and random reconfigs?


---
**jfabellera** 2024-12-12T17:47:08Z:

I haven't had the chance to try it yet. Did you get any more information or a chance to try and reproduce it?


---
**thenetworkgrinch** 2024-12-12T18:38:40Z:

Absolutely! Sorry for the late reply!

> What is the redundancy check in YAGSL for? We have checks for certain configurations against the detected device where possible.

The redundancy check is a fail-safe that ensures a `SparkMax` or `SparkFlex` retries configuring when it fails for whatever reason given a set delay. The configuration takes place before a `setReference` call and has a small delay whenever it has been changed after initial configuration of the swerve module.
Code for the retry it here: 
https://github.com/BroncBotz3481/YAGSL-Example/blob/dev/src/main/java/swervelib/motors/SparkMaxSwerve.java#L112-L128

Code for configuring is here:
https://github.com/BroncBotz3481/YAGSL-Example/blob/dev/src/main/java/swervelib/motors/SparkMaxSwerve.java#L430-L433

Code calling config is here:
https://github.com/BroncBotz3481/YAGSL-Example/blob/dev/src/main/java/swervelib/motors/SparkMaxSwerve.java#L460-L461

The reason for the delay is to give the SparkMax time to apply its new settings. I would love to be able to remove the 100ms delay after configuration apply if that is reliable on a HIGH can traffic robot.

The order of ops in YAGSL for configuring a SparkMax/SparkFlex is 
1. Swerve Module Constructed; drive and azimuth spark max configured 
2. save drive; no delay but redundant config check with 10ms delay on failure
3. save azimuth/angle, no delay but redundant config check with 10ms delay on each failure
4. On `setReference` call, check if config needs to updated, if so call redundant error checking with 10ms delay between each failure and wait 100ms for changes to apply since this could affect current SparkMax/SparkFlex running PID/readings/etc.


---
**thenetworkgrinch** 2024-12-13T14:54:32Z:

I reduced my delays and the error still happens according to the team.


---
**thenetworkgrinch** 2024-12-13T15:17:54Z:

As some further info i just realized that i had been fetching data from a pigeon2 on the same bus over 10ms.


---
**jfabellera** 2024-12-16T22:25:17Z:

Sorry, haven't had the time to look at this further.

> The redundancy check is a fail-safe that ensures a SparkMax or SparkFlex retries configuring when it fails for whatever reason given a set delay.

REVLib already retries whenever it fails to send a command/receive a response for any of the CAN commands in the `configure()` process. That includes resetting parameters, setting parameters, and persisting parameters. The amount of retries is configurable with `setCANMaxRetries()` and has a default value of 5.

However, if a specific error is returned by the firmware (which seems to be happening here for whatever reason), it will not do a retry because almost every time, doing the same action will result in the same error. Essentially, the built in retry mechanism is only for failures in communication.

That being said, each command is blocking until a response is received or it times out (the timeout is configurable with `setCANTimeout()` and has a default value of 20ms). Persisting parameters takes more time and has a decent timeout separate from the others.


---
**jfabellera** 2024-12-17T05:42:24Z:

So I was able to sort of reproduce this, but I'm not sure if it's the same issue as I was using Flex to test.

I put 20x SPARK Flex on a bus, and I used the same configuration that you provided and applied it to each controller. I repeatedly restarted the robot code to re-configure the controllers and to get the error to occur, and I would say the chance of it happening was relatively low, but it did happen on some occasions. The CAN utilization was around 80%, but I don't think this plays a part. Having more controllers on the bus meant more chance for one to fail.

I dug a bit deeper and found that communication errors weren't happening and that the controller was in fact returning an error when persisting parameters. I looked at the devices that failed and noticed a green-orange LED blink code, and I saw EEPROM faults when connecting to the Hardware Client.

Just to make sure I'm not going down a rabbit hole of a separate issue:

1. Do you know if anyone has experienced this issue on Flex?
2. When this issue occurs, what is the LED doing? Is it blinking green and orange?
3. How often would you say the issue occurs?


---
**jfabellera** 2024-12-17T05:45:52Z:

I will try again with MAX if I get time this week.

I know another team reported an issue with the green and orange LEDs on Flex. Wondering if it's related to what you're seeing.


---
**thenetworkgrinch** 2024-12-17T19:10:46Z:

> Do you know if anyone has experienced this issue on Flex?

I do not, unfortunately. I have only seen this on SparkMAX's
> When this issue occurs, what is the LED doing? Is it blinking green and orange?

I will ask, I do not know at the moment.
> How often would you say the issue occurs?

Out of the limited beta teams using YAGSL only one has reported this. So I would say a small amount, but given that it was caught early I wouldn't say its very small.

As an update the second team I thought had experienced this was able to give me a screenshot of their DS log which showed that they didnt update their SparkMAX's. 


---
**jfabellera** 2024-12-17T22:34:21Z:

Okay I'm starting to think this is a separate issue. Another issue has been opened #77 to track what I described, and when tracking down that bug, it became clearer this may be exclusive to Flex, it just so happened to occur with persisting parameters as well.


---
**thenetworkgrinch** 2024-12-18T18:25:57Z:

An update to the light status: I just heard back from the team that the lights are not green and orange. 

I do have a new theory. The conversion factor doesnt seem to be applying for them and they're going to attempt compensate by using PID. 

In YAGSL for SparkMAX's i apply the unit conversion in the conversion factor so my output units would be degrees or meters. Should i change this functionality to make SparkMAX conversion factor in rotations then convert to degrees or meters in YAGSL when reading the encoder values?


---
**jfabellera** 2024-12-18T18:37:02Z:

So we finally got a chance to fully reproduce this by pulling down the code the team was using, and we would consistently see the issue when hitting enable which is already an obvious code smell.

The error is a response from the device saying that the action (persist parameters) failed because the robot is enabled. This check is in place since the SPARK MAX will basically shut down entirely while it is burning its flash, and if the motor is enabled (and possibly running) this could cause undefined behavior and be potentially unsafe.

As a rule of thumb, most configuration should take place during robot initialization, and this includes persisting parameters. Any configuration outside of initialization and during operation is generally not recommended, but some use cases may necessitate it (like change idle mode). However, given the details above, persisting parameters must only be done when disabled.

While this is not necessarily a bug in REVLib or in the SPARK firmware, we will improve the error message to make this clearer and also include this information in our documentation.

Let me know if that helps.


---
**thenetworkgrinch** 2024-12-18T18:39:31Z:

Awesome! That helps alot and I can reorient around that paradigm. Thank you so much! 

Would you mind keeping this open until i push an update and we know from the team that it works?


---
**thenetworkgrinch** 2024-12-18T18:53:36Z:

As an FYI the setting that was changed mid execution was break mode to coast. So the drive motors would be in coast for teleop but break for auto.


---
**thenetworkgrinch** 2024-12-25T22:00:48Z:

Heard back from several people that it works! Thank you!

