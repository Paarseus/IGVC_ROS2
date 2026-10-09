<!-- Source: https://github.com/REVrobotics/REV-Software-Binaries/releases (GitHub Releases API, tags sm-*, sparkmax-*, revlib-*; fetched 2026-09-27). Release bodies copied verbatim. -->

## sparkmax-25.0.2  (2025-01-21T22:27:24Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sparkmax-25.0.2

* Fixes follower mode bug where motor will continue to spin after receiving command to stop following


## sparkmax-25.0.1  (2025-01-06T19:46:42Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sparkmax-25.0.1

### Note for updating

If you have any SPARK MAXes on your bus that are running firmware version 25.0.0, turn the robot power off and update those SPARK MAXes to version 25.0.1 individually. Once all devices on the bus are running either version 25.0.1 or version 24.0.1 or earlier, you can turn the robot back on and update the remaining devices in bulk.

### Fixes

 * Fixes a bug causing high CAN utilization when devices with 24.0.x firmware are present and Hardware Client is opened

## sparkmax-25.0.0  (2025-01-04T01:23:27Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sparkmax-25.0.0

**NOTE: We recommend performing a Factory Reset and then Persisting Parameters on every SPARK updated to firmware 25.0.0 and later for the first time.**

### Fixes

### New Features

* Adds support for zero-centered mode for duty cycle sensor
* Adds MAXMotion
* Only sends periodic CAN frames if they are needed (except for frames 0 and 1, which are enabled by default)
* Optimizes parameter operations for lower CAN traffic
* Reworks the CAN frames for better performance and reliability
* Supports standard roboRIO Universal Heartbeat
* Reorganizes faults to differentiate between warnings (which will not stop the motor) and faults (which will stop the motor)
* Slows down status frames when other devices are updating firmware


## sm-26.1.5  (2026-03-12T20:00:17Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sm-26.1.5

* Fixes issue where Current Control would only spin in one direction
* Fixes USB compatibility for Mac


## sm-26.1.4  (2026-02-18T20:12:06Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sm-26.1.4

* Improves performance of USB bridging
* Removes Legacy Status Frame 0, which is unused by REV Hardware Client 2 and newer releases of REV Hardware Client 1
* Fixes issue where soft-limits only updated when the controller was enabled


## sm-26.1.3  (2026-02-06T22:34:47Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sm-26.1.3

* Add support for CAN enabled REV encoders as feedback sensors
* Fixes an issue where enabling an enabled periodic frame would reset the time to send out the frame.


## sm-26.1.2  (2026-01-31T01:04:07Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sm-26.1.2

* Fixes issue in initializing status 8 (frame that indicates setpoint, pid slot, etc.)


## sm-26.1.1  (2026-01-23T00:14:47Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sm-26.1.1

* Fixes follower mode on REV Hardware Client


## sm-26.1.0  (2026-01-10T00:51:29Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sm-26.1.0

* Adds ability to configure limit switch homing positions
* Fixes the I PID component not limiting in the negative direction when an iMaxAccum value is set
* Adds ability to set a PID tolerance outside MAXMotion
* Adds expanded feedforward support to all PID modes
* Changes Current PID to use Phase current instead of Bus current
* Reworks MAXMotion for smoother and more consistent motions
  * All new and improved profile generation and following
  * Reduces noise and inconsistency
  * Adds a new status frame with the internal position and velocity setpoints for improved tuning experience
* Adds new status frame with MAXMotion status and additional tuning tools
* Adds new status frame with closed loop control status
* Adds the specific SPARK Model to status 0
* Removes SmartMotion and SmartVelocity in favor of MAXMotion
* Fixes bug that can prevent force enable parameters from resetting correctly
* Switches the USB CAN bridging to use SLCan
  * Note: Devices that use SLCan are NOT compatible with REV Hardware Client when used over USB. Devices running v26.0.0-prerelease.2+ can only be used via RHC2, or downstream of a REV CAN device running v26.0.0-prerelease.1 or lower with RHC.
* Changes model value to 2 for MAX in Status 0 frame


## sm-25.0.4  (2025-02-13T22:28:20Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sm-25.0.4

* Fixes issue where SPARK MAX crashes after receiving a new set position value for primary encoder while in brushed mode

## sm-25.0.3  (2025-01-25T00:35:10Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sm-25.0.3

* Fixes issue where connecting a SPARK MAX to a computer for the first time with previous v25.+ causes all SPARK MAXes to be unidentifiable by the Hardware Client
  * See release notes for Hardware Client 1.7.1

## revlib-2027.0.0-alpha-6  (2026-07-28T22:17:02Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2027.0.0-alpha-6

REVLib should be available in the vendor dependencies of WPILib VS Code, but you can also use this JSON URL directly:

```txt
https://software-metadata.revrobotics.com/REVLib-2027.json
```

[Offline Installer](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2027.0.0-alpha-6/REVLib-offline-v2027.0.0-alpha-6.zip)

Refer to [WPILib Docs](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html) about installing 3rd party libraries.

## Changelog:

- [A301] Java: Fixes setCurrent()


## revlib-2027.0.0-alpha-5  (2026-07-27T22:24:17Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2027.0.0-alpha-5

REVLib should be available in the vendor dependencies of WPILib VS Code, but you can also use this JSON URL directly:

```txt
https://software-metadata.revrobotics.com/REVLib-2027.json
```

[Offline Installer](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2027.0.0-alpha-5/REVLib-offline-v2027.0.0-alpha-5.zip)

Refer to [WPILib Docs](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html) about installing 3rd party libraries.

## Changelog:

- [REVLib] Java: Fixes crash in getPeriodicStatus8()
- [A301] Adds IdleMode getter and setter
- [A301] Fixes inconsistencies when inverted
- [A301] Adds a busId-only constructor to C++
- [A301/SPARK] Adds ability to modify the absolute position range about zero -- (-1.0, 0] to [0, 1.0)


## revlib-2027.0.0-alpha-4  (2026-07-02T17:40:58Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2027.0.0-alpha-4

REVLib should be available in the vendor dependencies of WPILib VS Code, but you can also use this JSON URL directly:

```txt
https://software-metadata.revrobotics.com/REVLib-2027.json
```

[Offline Installer](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2027.0.0-alpha-4/REVLib-offline-v2027.0.0-alpha-4.zip)

Refer to [WPILib Docs](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html) about installing 3rd party libraries.

## Changelog:

- [REVLib] Removes deprecated functions
- [REVLib] Fixes links in documentation
- [REVLib] Fixes Sim classes to include CAN Bus ID in name
- [A301] - Adds new A301-specific CAN specifications
- [A301] - Removed isContinuous argument from setAbsolutePosition(); replaced with (En/Dis)ableAbsolutePositionContinuousInput()
- [A301] - Adds a setRelativePositionWithSpeed() and setAbsolutePositionWithSpeed() to run position closed loop control with a speed constraint
- [A301] - Updates A301(int busId) constructor to autodetect the device ID instead of requiring the device to be the factory default value of 3


## revlib-2027.0.0-alpha-3  (2026-05-29T21:45:10Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2027.0.0-alpha-3

## Changelog:
- [A301] Adds setRelativeEncoderPosition()
- [A301] Fixes setInverted() and getInverted()
- [ServoHub] Fixes internal crash when calling Status getters
- [A301] Renames getOutputCurrent() to getMotorCurrent()


## revlib-2026.0.5  (2026-03-12T18:23:23Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2026.0.5

You can install the Java/C++ version of this library using this JSON URL in VSCode:

https://software-metadata.revrobotics.com/REVLib-2026.json

This release does not include LabVIEW. v2026.0.1 for LabVIEW is available to download [here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2026.0.1/revlib_2026.0.1-0_windows_all.nipkg).

## Changelog:

- [SPARK] Adds warning for SPARK devices not on 2026 firmware

## revlib-2026.0.4  (2026-03-04T21:42:15Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2026.0.4

You can install the Java/C++ version of this library using this JSON URL in VSCode:

https://software-metadata.revrobotics.com/REVLib-2026.json

This release does not include LabVIEW. v2026.0.1 for LabVIEW is available to download [here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2026.0.1/revlib_2026.0.1-0_windows_all.nipkg).

## Changelog:

- [REVLib] Fixes issue where the SplineEncoder was not fetching its status periods at creation
- [REVLib] Updates SplineEncoder documentation

## revlib-2026.0.3  (2026-02-18T20:25:27Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2026.0.3

You can install the Java/C++ version of this library using this JSON URL in VSCode:

https://software-metadata.revrobotics.com/REVLib-2026.json

This release does not include LabVIEW. v2026.0.1 for LabVIEW is available to download [here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2026.0.1/revlib_2026.0.1-0_windows_all.nipkg).

## Changelog:

- [REVLib] Java/C++ - Fixes MAXSpline Encoder issues/crashes


## revlib-2026.0.2  (2026-02-13T18:18:06Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2026.0.2

You can install the Java/C++ version of this library using this JSON URL in VSCode:

```
https://software-metadata.revrobotics.com/REVLib-2026.json
```

This release does not include LabVIEW. v2026.0.1 for LabVIEW is available to download [here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2026.0.1/revlib_2026.0.1-0_windows_all.nipkg).

## Changelog:

- [REVLib] Adds SignalsAccessor for MAXSpline Encoders
- [REVLib] Fixes the issue where DetachedEncoderSignals were not applied
- [REVLib] Fetches status periods from device on object creation
- [REVLib] Updates SPARK parameter descriptions
- [REVLib] Adds new MAXSpline Encoder configuration parameters (start/end pulse and absolute period) and accessors


## revlib-2026.0.1  (2026-01-13T20:50:02Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2026.0.1

You can install the Java/C++ version of this library using this JSON URL in VSCode:

```
https://software-metadata.revrobotics.com/REVLib-2026.json
```

REVLib v2026.0.1 for LabVIEW is available to download [here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2026.0.1/revlib_2026.0.1-0_windows_all.nipkg).

## Changelog

- [REVLib] Java/C++: Fixes simulation crash on MacOS
- [REVLib] C++: Fixes cpp check warnings
- [REVLib] LabVIEW: Updates for 2026 FRC LabVIEW


## revlib-2026.0.0  (2026-01-10T00:27:50Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2026.0.0

You can install the Java/C++ version of this library using this JSON URL in VSCode:

```
https://software-metadata.revrobotics.com/REVLib-2026.json
```

REVLib v2026.0.0-1 for LabVIEW is available to download [here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2026.0.0/revlib_2026.0.0-1_windows_all.nipkg).

## What's new for 2026

### Major Changes

- [MAXSpline Encoder] Java/C++: Adds initial support for MAXSpline Encoders with `DetachedEncoder` class
- [REVLib] Java/C++: Adds `StatusLogger`, the Official REV-Compatible Logger adapted from [URCL](https://github.com/Mechanical-Advantage/URCL)
  - Requires the [revlog-converter](https://www.npmjs.com/package/@rev-robotics/revlog-converter) cli tool to convert revlogs to wpilogs or you can import them directly into AdvantageScope
- [SPARK] Adds support for new feedforward parameters: `kV` (formerly `kF`), `kA`, `kS`, `kG`, `kCos`, and `kCosRatio`
- [SPARK] Renames MAXMotion parameters to be more descriptive (`kMaxVelocity` -> `kCruiseVelocity`, `kAllowedClosedLoopError` -> `kAllowedProfileError`)
- [SPARK] Java/C++: Adds simulation support and parity for new MAXMotion and feedforward features
- [SPARK] Adds support for new MAXMotion status signals: `MAXMotionSetpointPosition` and `MAXMotionSetpointVelocity`
- [SPARK] Adds support for closed loop status signals: `isAtSetpoint`, `setpoint`, and `selectedClosedLoopSlot`
- [SPARK] Adds the ability to set allowed closed loop error when using regular PID control
- [SPARK] Java/C++: Adds `SparkSoftLimit` class for getting soft limit status
- [SPARK] Java/C++: Adds `getControlType()` to get the selected control type (last used in calling `setReference()`/`setSetpoint()`)
- [SPARK] Java/C++: Adds configuration presets for various motors
- [SPARK] Java/C++: Adds configuration presets for REV through-bore encoders (V1 and v2) and MAXSpline Encoder (when used via the 6-pin JST) for primary encoder, external/alternate encoder, and absolute encoders
- [SPARK] Java/C++: Removes automatic clear faults call when creating a SparkFlex/SparkMax object. You can still call clearFaults manually if you wish.
- [Servo Hub] Java/C++: Removes automatic clear faults call when creating a ServoHub object. You can still call clearFaults manually if you wish.
- [REVLib] Java/C++: Refactors `ResetMode` and `PersistMode` to be a single common enum instead of device specific
- [SPARK] Java/C++: Deprecates `setReference()` in favor of `setSetpoint()`
- [SPARK] Removes SmartMotion in favor of MAXMotion

### Other Changes

- [REVLib] Java/C++: Fixes potential dangling reference in configure async calls
- [REVLib] Java/C++: Fixes memory leaks in daemons
- [SPARK] C++: Fixes crash when setting three or more signals within the same periodic status
- [SPARK] Java/C++: Fixes memory leaks in simulation when certain devices are never created
- [ServoHub] Java: Fixes `getChannelDisableBehavior()` always returning `kDoNotSupplyPower` in Java
- [SPARK] Java/C++: Fixes bug causing stale parameter reads
- [REVLib] Java/C++: Fixes issue with booleans in simulation
- [REVLib] Java: Prevents segmentation fault in multithreaded Java code that may call close
- [SPARK] Java: Adds getters for Periodic Status Frames for SPARKs
- [SPARK] Java/C++: Improves SPARK model detection
- [Servo Hub] Java: Makes `ServoHub` class AutoCloseable
- [Servo Hub] Java: Return null for periodic status getters if reading the frame failed

## revlib-2025.0.3  (2025-03-03T22:10:40Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2025.0.3

### Changes for Java and C++

- [SPARK] Improves documentation concerning Relative Encoders and Position and Velocity Conversion Factors
- [SPARK] Removes `setPositionConversionFactor()` and `setVelocityConversionFactor()` methods from the sim classes
  - Instead, use the appropriate Config objects and `positionConversionFactor()` and `velocityConversionFactor()` methods
- [SPARK] Fixes crash when calling `Spark[Flex, Max].configureAsync()` in simulation

### Changes for LabVIEW

- Fixes issue where an error would be incorrectly generated when configuring follower mode

## revlib-2025.0.2  (2025-01-21T19:27:16Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2025.0.2

## Changelog:

- [SPARK] Improves SPARK error messages by adding the invalid value which caused the error
- [SPARK] Improves SPARK error messages by displaying the parameter name as well as its ID
- [Servo Hub] Fixes uninitialized variables
- [REVLib] Fixes issue where REVLib doesn't clear a previous error (as viewed through `GetLastError`)
- [REVLib] Fixes threading issues encountered while running Googletest unit tests



## revlib-2025.0.1  (2025-01-13T21:35:35Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2025.0.1

An offline installer is available [here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2025.0.1/REVLib-offline-v2025.0.1.zip).

This release does not include LabVIEW. REVLib v2025.0.0 for LabVIEW is available to download [here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2025.0.0/revlib_2025.0.0-0_windows_all.nipkg).

C++ docs: [https://codedocs.revrobotics.com/cpp/](https://codedocs.revrobotics.com/cpp/)
Java docs: [https://codedocs.revrobotics.com/java/](https://codedocs.revrobotics.com/java)

## Changes for Java and C++

- [SPARK] Fixes issue where enabling a limit switch in Java simulation would cause it to always return that it was pressed
- [SPARK] Fixes issue causing REVLib to have higher than normal CPU usage when retrieving SPARK status frames
- [SPARK] Fixes issue where a status frame timeout would cause a segmentation fault

## revlib-2025.0.0  (2025-01-04T02:40:07Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2025.0.0

You can install the C++/Java version of this library using this JSON URL in VSCode:
```
https://software-metadata.revrobotics.com/REVLib-2025.json
```

An offline installer is available [here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2025.0.0/REVLib-offline-v2025.0.0.zip).

REVLib for LabVIEW is available to download [here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2025.0.0/revlib_2025.0.0-0_windows_all.nipkg).

C++ docs: [https://codedocs.revrobotics.com/cpp/](https://codedocs.revrobotics.com/cpp/)
Java docs: [https://codedocs.revrobotics.com/java/](https://codedocs.revrobotics.com/java)

## Major Changes

- [REVLib] Requires non-prerelease versions of SPARK and Servo Hub firmware v25.0.0 or higher
- [SPARK] Java/C++: Moves to a more declarative approach for configuring devices
  - Adds `SparkFlexConfig`, `SparkMaxConfig` which includes settings for different aspects of each device
  - Adds `configure()` method to apply a config object's settings to one or more devices of the correct type
  - Adds `configureAsync()` to configure a device without blocking the program
  - Adds a `configAccessor` field to device classes for reading configuration parameters directly from the device
- [SPARK] Java/C++: Adds better support for simulation
  - Moves away from `REVPhysicsSim` to offer better support for WPILib physics simulation instead
  - Revamps simulation GUI data, including brand new fields for auxiliary devices
  - Adds Sim classes for each auxiliary device, allowing for more thorough simulation in the WPILib injection style
  - Adds `SparkSim.iterate()` method which features simulated current limits, closed-loop control, and more
  - Adds `SparkSimFaultManager` for throwing simulated faults
- [SPARK] Adds support for MAXMotion
  - Adds control types `MAXMotionPositionControl` and `MAXMotionVelocityControl`
  - Adds `MAXMotionConfig`. Only trapezoidal profile is available at this time.
  - MAXMotion is not a drop-in replacement for Smart Motion, as you will need to retune PID gains.
- [SPARK] Improves experience with managing status signals from SPARK devices
  - Adds `SignalsConfig` to adjust signal periods and always on setting
  - Automatically enables relevant status frames if a signal is requested by the user
- [Servo Hub] Java/C++: Adds initial support for Servo Hub
  - Follows the same paradigms used for SPARK
  - Includes basic simulation support for Servo Hub

## Breaking Changes

- [SPARK] Renames `CANSparkFlex` and `CANSparkMax` to `SparkFlex` and `SparkMax` respectively
- [SPARK] Renames `SparkPIDController` to `SparkClosedLoopController`
- [SPARK] Removes configuration parameter setter/getter methods. Use `SparkBase.configure()` and `SparkBase.configAccessor` instead.
- [SPARK] Removes `burnFlash()` and `restoreFactoryDefaults()`. Use the `ResetMode` and `PersistMode` options in `SparkBase.configure()` instead.
- [SPARK] Removes `REVPhysicsSim` in favor of new simulation system
- [SPARK] Removes async mechanism for setting parameters by setting CAN timeout to 0 in favor of `configureAsync()`
- [SPARK] Moves all SPARK related classes into a `spark` package in Java and namespace in C++
- [SPARK] LabVIEW: Reworks entire VI palette
  - Improves organization of VI palette by separating VIs by configuration, device status, and utility
  - Moves towards increased usage of polymorphic VIs for easier navigation of the palette

## Other Changes

- [SPARK] Fixes issue where multiple setpoint commands would be sent when switching control types on a SPARK, resulting in the motor oscillating between the different setpoints
- [SPARK] Deprecates `kSmartMotion` and `kSmartVelocity` control types in favor of `kMAXMotionPositionControl` and `kMAXMotionVelocityControl` respectively.
- [SPARK] Deprecates `SparkBase.setInverted()` and `SparkBase.getInverted()` in favor of using the new configuration system
- [SPARK] Updates `ClosedLoopController.setReference()` to use the `ClosedLoopSlot` enum instead of an int
- [SPARK] Improves error description when attempting to persist parameters while the robot is enabled
- [SPARK] Improves getting faults/warnings by returning a `Faults` or `Warnings` object
  - The raw bits of faults and warnings are available as a field in the respective struct
- [SPARK] Adds `hasActiveFault()`, `hasStickyFault()`, `hasActiveWarning()`, and `hasStickyWarning()` to check if there is a fault/warning present at all on the SPARK device
- [SPARK] Adds `pauseFollowerMode()` and `resumeFollowerMode()`
- [SPARK] Adds ability for follower mode to work even if the follower is not referenced in user code
- [SPARK] Adds support for specifying an absolute encoder's duty cycle start and end pulse widths in `AbsoluteEncoderConfig`
- [SPARK] Adds configuration option for setting whether the absolute encoder is zero-centered
- [REVLib] Fixes potential memory leaks in string handling
- [SPARK] LabVIEW: Improves reliability of CAN transactions by adding a retry mechanism

## sparkmax-25.0.0-prerelease.9  (2024-12-19T00:12:22Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sparkmax-25.0.0-prerelease.9

### Fixes

* (Prerelease Regression) Fixed issue where SPARK MAX would not respond to input PWM signal
* Fixes issue where MAXMotion Velocity doesn't decelerate properly when a new setpoint value is less than the previous setpoint
* Updates MAXMotion velocity mode to ignore the MAXMotion max velocity parameter
  - This parameter is used for MAXMotion position mode only

## sparkmax-25.0.0-prerelease.7  (2024-11-25T23:52:54Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sparkmax-25.0.0-prerelease.7

### Fixes

* Fixes range of duty cycle zero-centered mode to be [-0.5, 0.5) instead of [-1, 1)


## sparkmax-25.0.0-prerelease.6  (2024-11-09T00:17:30Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sparkmax-25.0.0-prerelease.6

### Fixes

* (Prerelease Regression) Fixes case where changing the CAN ID of a SPARK MAX connected directly via USB would affect other SPARK devices on the CAN bus with the same CAN ID
* (Prerelease Regression) Fixes case where Hardware Client could compete with a roboRIO for control of a SPARK MAX
* (Prerelease Regression) Fixes issue causing Hardware Client to show an error when setting the CAN ID

## sparkmax-25.0.0-prerelease.4  (2024-11-05T00:00:30Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sparkmax-25.0.0-prerelease.4

### New Features
- Adds MAXMotion
- Only sends periodic CAN frames if they are needed (except for frames 0 and 1, which are enabled by default)
- Optimizes parameter operations for lower CAN traffic
- Reworks the CAN frames for better performance and reliability
- Supports standard roboRIO Universal Heartbeat
- Reorganizes faults to differentiate between warnings (which will not stop the motor) and faults (which will stop the motor)
- Slows down status frames when other devices are updating firmware

## sm-26.0.0-prerelease.2  (2025-11-21T22:50:16Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sm-26.0.0-prerelease.2

* Removes SmartMotion and SmartVelocity in favor of MAXMotion
* Fixes bug that can prevent force enable parameters from resetting correctly
* Switches the USB CAN bridging to use SLCan
    * Note: Devices that use SLCan are NOT compatible with REV Hardware Client when used over USB. Devices running v26.0.0-prerelease.2+ can only be used via RHC2, or downstream of a REV CAN device running v26.0.0-prerelease.1 or lower with RHC.
* Changes model value to 2 for MAX in Status 0 frame


## sm-26.0.0-prerelease.1  (2025-08-08T18:47:27Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sm-26.0.0-prerelease.1

You can install this version of firmware by pasting the following code into REV Hardware Client > Downloads > Subscribe to Software Channels:

```
frcAlpha2026
```

## Changelog

* Adds ability to configure limit switch homing positions
* Fixes the I PID component not limiting in the negative direction when an iMaxAccum value is set
* Adds ability to set a PID tolerance outside MAXMotion
* Adds expanded feedforward support to all PID modes
* Changes Current PID to use Phase current instead of Bus current
* Reworks MAXMotion for smoother and more consistent motions
  * All new and improved profile generation and following
  * Reduces noise and inconsistency
  * Adds a new status frame with the internal position and velocity setpoints for improved tuning experience
* Adds new status frame with MAXMotion status and additional tuning tools
* Adds new status frame with closed loop control status
* Adds the specific SPARK Model to status 0


## revlib-2027.0.0-alpha-7  (2026-09-11T22:12:04Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2027.0.0-alpha-7

## Changelog:

- [REVLib] Updates to WPILib 2027 Alpha 7
- [REVLib] Device constructors now take the WPILib CANPort enum instead of a raw int for the CAN bus ID
- [REVLib] Renames getBusId() to getCanPort()
- [A301] Updates CAN specs and fixes inversion. Requires latest A301 firmware.
- [SPARK] Deprecates AbsoluteEncoderConfig.zeroCentered(). Use rangeOffset() instead.
- [SPARK] Removes hall sensor velocity averaging configurations in favor of new firmware filtering system
- [SPARK] Adjusts default encoder average depth to 8 and sample delta to 20
- [REVLib] Removes Conversion Factors in SPARKs and MAXSplineEncoder
- [SPARK] Removes ClosedLoopConfig.positionWrappingMinInput, ClosedLoopConfig.positionWrappingMaxInput, and ClosedLoopConfig.positionWrappingInputRange
- [REVLib] C++ - Replace usage of fmt library with std

## revlib-2027.0.0-alpha-2  (2026-05-22T20:42:11Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2027.0.0-alpha-2

## Changelog:
- [A301] Adds support for A301
- [REVLib] Java/C++: Creates a Signal wrapper type for signals, allowing a user to know if a value is outdated. This is a breaking change and will require user code to call `.get()` / `.get(default)` in Java or `Get()` in C++. User code can query `.isValid()` in Java or `IsValid()` in C++ to know whether the value is recent.

## revlib-2027.0.0-alpha-1  (2025-06-24T17:17:56Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2027.0.0-alpha-1

REVLib should be available in the vendor dependencies of WPILib VS Code, but you can also use this JSON URL directly:

```txt
https://software-metadata.revrobotics.com/REVLib-2027.json
```

[Offline Installer](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2027.0.0-alpha-2/REVLib-offline-v2027.0.0-alpha-2.zip)

Refer to [WPILib Docs](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html) about installing 3rd party libraries.

## Changelog:

- [REVLib] Adds support for Systemcore and the WPILib 2027 alpha

## revlib-2026.0.0-beta-1  (2025-11-22T02:06:42Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2026.0.0-beta-1

> [!NOTE]
> WPILib 2026 beta is required to use this version of REVLib

See our [2026.0.0-alpha-1 release notes](https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2026.0.0-alpha-1) to see what's new for 2026.

You can install the Java/C++ version of this library using this JSON URL in VSCode:

```
https://software-metadata.revrobotics.com/REVLib-2026.json
```

REVLib v2026.0.0-beta-1 for LabVIEW is available to download [here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2026.0.0-beta-1/revlib_2026.0.0-1_windows_all.nipkg).

## Changelog

- [Spark] Removes SmartMotion
- [SPARK] Java: Adds getters for Periodic Status Frames for SPARKs
- [SPARK] Java: Adds common RevDevice interface for REV devices
- [SPARK] Fixes bug causing stale parameter reads
- [SPARK] Improves SPARK model detection
- [SPARK, Servo Hub] Prevents SigSegv in multithreaded Java code that may call close
- [Servo Hub] Makes ServoHub's Java class AutoCloseable
- [ServoHub] Java: Return null for periodic status getters if reading the frame failed
- [REVLib] Fixes issue with booleans in simulation

## revlib-2026.0.0-alpha-1  (2025-08-08T18:34:54Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2026.0.0-alpha-1

> [!NOTE]
> Since WPILib 2026 is not yet available, this version of REVLib 2026 can be used with WPILib 2025.

> [!NOTE]
> To take full advantage of this release, please update your devices to 2026+ firmware.

You can install the Java/C++ version of this library using this JSON URL in VSCode:

```
https://software-metadata.revrobotics.com/REVLib-2026.json
```

REVLib v2026.0.0-alpha-1 for LabVIEW is available to download [here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2026.0.0-alpha-1/revlib_2026.0.0-1_windows_all.nipkg).

## Changelog

### Major Changes

- [REVLib] Java/C++: Adds `StatusLogger`, the Official REV-Compatible Logger adapted from [URCL](https://github.com/Mechanical-Advantage/URCL)
  - Requires the [revlog-converter](https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlog-converter-0.1) cli tool to convert revlogs to wpilogs
- [SPARK] Adds support for new feedforward parameters: `kV` (formerly `kF`), `kA`, `kS`, `kG`, `kCos`, and `kCosRatio`
- [SPARK] Renames MAXMotion parameters to be more descriptive (`kMaxVelocity` -> `kCruiseVelocity`, `kAllowedClosedLoopError` -> `kAllowedProfileError`)
- [SPARK] Java/C++: Adds simulation support and parity for new MAXMotion and feedforward features
- [SPARK] Adds support for new MAXMotion status signals: `MAXMotionSetpointPosition` and `MAXMotionSetpointVelocity`
- [SPARK] Adds support for closed loop status signals: `isAtSetpoint`, `setpoint`, and `selectedClosedLoopSlot`
- [SPARK] Adds the ability to set allowed closed loop error when using regular PID control
- [SPARK] Java/C++: Adds `SparkSoftLimit` class for getting soft limit status
- [SPARK] Java/C++: Adds `getControlType()` to get the selected control type (last used in calling `setReference()`/`setSetpoint()`)
- [SPARK] Java/C++: Deprecates `setReference()` in favor of `setSetpoint()`

### Fixes

- [REVLib] Java/C++: Fixes potential dangling reference in configure async calls
- [REVLib] Java/C++: Fixes memory leaks in daemons
- [SPARK] C++: Fixes crash when setting three or more signals within the same periodic status
- [SPARK] Java/C++: Fixes memory leaks in simulation when certain devices are never created
- [ServoHub] Java: Fixes `getChannelDisableBehavior()` always returning `kDoNotSupplyPower` in Java

## revlib-2025.0.0-beta-4  (2024-12-19T00:54:23Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2025.0.0-beta-4

### Changes for Java and C++

- Adds support for Servo Hub
- Adds missing config accessor for absolute encoder zero offset for SPARK
- Fixes memory leaks in simulation for SPARK
- Fixes potential memory leaks in string handling
- Fixes issue where default control mode for SPARK was not being set correctly in simulation, causing errors in console with `iterate()`
- Fixes issue where the SPARK primary encoder simulation device does not get freed properly

### Known Issues

- Setting configurations on Servo Hub in simulation will fail and result in an error to the console


## revlib-2025.0.0-beta-3  (2024-11-25T23:18:07Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2025.0.0-beta-3

### Improvements for Java and C++

- Adds fields to `Faults` and `Warnings` structs for the raw bits representation of faults and warnings
- Adds configuration option for setting whether the absolute encoder is zero-centered
- Corrects simulation Conversion Factor default values to 1.0
  - Fixes issue where sensor `.iterate` methods would not set the position correctly
- Fixes documentation for `SparkMax.configure()`
- Fixes issue where multiple setpoint commands would be sent when switching control types on a SPARK, resulting in the motor oscillating between the different setpoints

## revlib-2025.0.0-beta-2  (2024-11-08T23:26:57Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2025.0.0-beta-2

This is only for Java and C++. An update for LabVIEW will come at a later time.

This release requires [WPILib 2025.1.1-beta-1](https://github.com/wpilibsuite/allwpilib/releases/tag/v2025.1.1-beta-1) and SPARK MAX/Flex firmware v25.0.0 which can be downloaded and installed via the REV Hardware Client v1.6.7 Beta.

You can install the Java/C++ version of this library using this JSON URL in VSCode:

```
https://software-metadata.revrobotics.com/REVLib-2025.json
```

An offline install for Java/C++ can be found [here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2025.0.0-beta-2/REVLib-offline-v2025.0.0-beta-2.zip).

API documentation:

- [Javadocs](https://codedocs.revrobotics.com/java/)
- [C++ docs](https://codedocs.revrobotics.com/cpp/)

### Fixes for Java and C++

- Fixes issue where setting certain parameters (follower mode, MAXMotion, and status signals) in simulation erroneously throws an exception
- Fixes issue where retrieving auxiliary objects from a SPARK MAX (alternate encoder, absolute encoder, and limit switches) before setting the data port config with `configure()` can erroneously throw an exception

## revlib-2025.0.0-beta-1  (2024-11-05T00:00:56Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2025.0.0-beta-1

This is the first beta release of REVLib for the 2025 FRC season. This initial release only includes changes for Java and C++. An update for LabVIEW will come at a later time.

This release requires [WPILib 2025.1.1-beta-1](https://github.com/wpilibsuite/allwpilib/releases/tag/v2025.1.1-beta-1) and SPARK MAX/Flex firmware v25.0.0 which can be downloaded and installed via the REV Hardware Client v1.6.7 Beta.

An offline installer for Java/C++ can be found [here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2025.0.0-beta-1/REVLib-offline-v2025.0.0-beta-1.zip).

API documentation:

- [Javadocs](https://codedocs.revrobotics.com/java/)
- [C++ docs](https://codedocs.revrobotics.com/cpp/)

## Major Changes

- Moves to a more declarative approach for configuring SPARK devices
  - Adds `SparkFlexConfig` and `SparkMaxConfig` which includes settings for different aspects of the SPARK
  - Adds `SparkBase.configure()` method to apply a SPARK config object's settings to one or more SPARK devices
- Adds support for MAXMotion
  - Adds control types `MAXMotionPositionControl` and `MAXMotionVelocityControl`
  - Adds `MAXMotionConfig`. Only trapezoidal profile is available at this time.
  - MAXMotion is not a drop-in replacement for Smart Motion, as you will need to retune PID gains.
- Adds better support for simulation
  - Moves away from `REVPhysicsSim` to offer better support for WPILib physics simulation instead
  - Revamps simulation GUI data, including brand new fields for auxiliary devices
  - Adds Sim classes for each auxiliary device, allowing for more thorough simulation in the WPILib injection style
  - Adds `SparkSim.iterate()` method which features simulated current limits, closed-loop control, and more
  - Adds `SparkSimFaultManager` for throwing simulated faults
- Improves experience with managing status signals from SPARK devices
  - Adds `SignalsConfig`
  - Automatically enables relevant status frames if a signal is requested by the user
- Adds a `configAccessor` field to SPARK classes for reading configuration parameters directly from the device

## Breaking Changes

- Renames `CANSparkFlex` and `CANSparkMax` to `SparkFlex` and `SparkMax` respectively
- Renames `SparkPIDController` to `SparkClosedLoopController`
- Removes configuration parameter setter/getter methods. Use `SparkBase.configure()` and `SparkBase.configAccessor` instead.
- Removes `burnFlash()` and `restoreFactoryDefaults()`. Use the `ResetMode` and `PersistMode` options in `SparkBase.configure()` instead.
- Removes `REVPhysicsSim` in favor of new simulation system
- Moves all SPARK related classes into a `spark` package in Java and namespace in C++

## Other Changes

- Deprecates `kSmartMotion` and `SmartVelocity` control types in favor of `MAXMotionPositionControl` and `MAXMotionVelocityControl` respectively.
- Adds `pauseFollowerMode()` and `resumeFollowerMode()`
- Allows follower mode to work even if the follower is not referenced in user code
- Improves getting faults/warnings by returning a `Faults` or `Warnings` object
- Adds `hasActiveFault()`, `hasStickyFault()`, `hasActiveWarning()`, and `hasStickyWarning()` to check if there is a fault/warning present at all on the SPARK device
- Adds support for specifying an absolute encoder's duty cycle start and end pulse widths in `AbsoluteEncoderConfig`


## sparkmax-24.0.1  (2024-01-11T22:14:26Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sparkmax-24.0.1

* Improves filtering of invalid PWM signals
    * Previously, noise on the signal wires could be occasionally be erroneously interpreted as a PWM signal, causing the motor to spin unexpectedly

## sparkmax-24.0.0  (2024-01-06T10:59:04Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sparkmax-24.0.0

### Breaking Changes
* Moves the IAccum value to periodic status frame 7
    * Periodic status frame 7 is new to this release, and by default is sent every 250ms.

### Enhancements
* Allows changing the CAN ID of a SPARK MAX connected directly via USB without affecting other SPARK devices on the CAN bus with the same CAN ID
* Makes changes towards improving the reliability of saving and persisting parameters

### Bug fixes
* Fixes alternate encoder position accuracy
* Fixes the main quadrature encoder position jumping in brushed mode

## sparkmax-1.6.3  (2023-01-30T17:54:33Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sparkmax-1.6.3

## Version 1.6.3
* Fixes issue where changing the inversion mode of the duty cycle absolute encoder with a zero offset specified would cause the physical zero position to change

## sparkmax-1.6.2  (2023-01-13T20:01:04Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sparkmax-1.6.2

## Version 1.6.2
* Fixes critical issue where new parameters introduced in 1.6.0 were not being burned to flash correctly
* Fixes issue where new parameters were not being read back correctly despite being set correctly

## sparkmax-1.6.1  (2023-01-06T19:37:32Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sparkmax-1.6.1

## Version 1.6.1
* Fixes duty cycle offset to match the inverted setting
* Fixes parameters being NaN after updating to 1.6.0
* Fixes burn flash command response

## Version 1.6.0
* Adds new parameters for configuring hall sensor velocity measurement
* Adds support for duty cycle absolute encoders
* Adds new parameters to enable and configure position PID rollover


## revlib-2024.2.4  (2024-03-18T20:55:36Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2024.2.4

You can install the C++/Java version of this library using this JSON URL in VSCode:
```
https://software-metadata.revrobotics.com/REVLib-2024.json
```

This release does not include LabVIEW. REVLib v2024.2.0 for LabVIEW is available to download [here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2024.2.0/REVLib-labVIEW-2024.2.0-0_windows_all.nipkg).

C++ docs: [https://codedocs.revrobotics.com/cpp/](https://codedocs.revrobotics.com/cpp/)
Java docs: [https://codedocs.revrobotics.com/java/](https://codedocs.revrobotics.com/java)

### Changes to C++ and Java

- Increases the default timeout to wait for a periodic status from 2*framePeriodMs to 500ms.
  - Reduces possibility for large, inaccurate jumps in data to occur when retrieving from status frames.
  - Reduces amount of "timed out while waiting for periodic status X" errors in driver station.
  - Adds `setPeriodicFrameTimeout()` to configure the CAN timeout for periodic status frames. See code docs for more information.
- Improves reliability of RTR CAN frames like setting parameters and other commands that expect a response from the device.
  - Adds mechanism to retry requests if sending the request or receiving the response failed. The default value for maximum number of retries is 5.
  - Adds `setCANMaxRetries()` to configure the value for maximum number of retries. See code docs for more information.
- Fixes undefined behavior when SPARK motor controller information cannot be retrieved during initialization.


## revlib-2024.2.3  (2024-02-28T02:08:24Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2024.2.3

You can install the C++/Java version of this library using this JSON URL in VSCode:
```
https://software-metadata.revrobotics.com/REVLib-2024.json
```

This release does not include LabVIEW. REVLib v2024.2.0 for LabVIEW is available to download [here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2024.2.0/REVLib-labVIEW-2024.2.0-0_windows_all.nipkg).

C++ docs: [https://codedocs.revrobotics.com/cpp/](https://codedocs.revrobotics.com/cpp/)
Java docs: [https://codedocs.revrobotics.com/java/](https://codedocs.revrobotics.com/java)

### Changes to Java

* Fixes issue introduced in v2024.2.2 where calling getEncoder() multiple times can cause a fatal exception in certain circumstances.

### Changes to C++ and Java

* Removes dynamic check for SPARK model when calling getEncoder(), causing unnecessary CAN traffic.
* Moves zero argument CANSparkBase.getEncoder() to CANSparkMax and CANSparkFlex subclasses to determine default encoder values.

## revlib-2024.2.2  (2024-02-24T02:38:00Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2024.2.2

You can install the C++/Java version of this library using this JSON URL in VSCode:
```
https://software-metadata.revrobotics.com/REVLib-2024.json
```

This release does not include LabVIEW. REVLib v2024.2.0 for LabVIEW is available to download [here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2024.2.0/REVLib-labVIEW-2024.2.0-0_windows_all.nipkg).

C++ docs: [https://codedocs.revrobotics.com/cpp/](https://codedocs.revrobotics.com/cpp/)
Java docs: [https://codedocs.revrobotics.com/java/](https://codedocs.revrobotics.com/java)

### Changes to C++ and Java

* Fixes issue where configuring the velocity filter for the default relative encoder of a SPARK Flex would not set the correct parameters.

### Changes to Java

* Improves memory allocation performance.

## revlib-2024.2.1  (2024-02-09T23:53:25Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2024.2.1

You can install the C++/Java version of this library using this JSON URL in VSCode:
```
https://software-metadata.revrobotics.com/REVLib-2024.json
```

This release does not include LabVIEW. REVLib v2024.2.0 for LabVIEW is available to download [here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2024.2.0/REVLib-labVIEW-2024.2.0-0_windows_all.nipkg).

C++ docs: [https://codedocs.revrobotics.com/cpp/](https://codedocs.revrobotics.com/cpp/)
Java docs: [https://codedocs.revrobotics.com/java/](https://codedocs.revrobotics.com/java)

## Changelog
- C++/Java: Changes behavior of SPARK Flex and MAX initialization errors to throw exceptions rather than terminating the robot program.
- C++/Java: Fixes issue where initializing a SPARK Flex or MAX in brushed mode while the device is disconnected from the CAN bus causes the robot program to terminate.
- C++/Java: Fixes issue where initializing a SPARK Flex or MAX in brushed mode causes robot simulation to terminate.
- C++/Java: Fixes warning about using the wrong class for a SPARK Flex or MAX during robot simulation.
- C++: Fixes ambiguous overload error when no parameters are supplied when calling `GetAnalogSensor()`.
- C++: Fixes ambiguous overload error when no parameters are supplied when calling `GetEncoder()`.

## revlib-2024.2.0  (2024-01-06T11:37:33Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2024.2.0

**Official 2024 FRC kickoff release for REVLib, with full support for SPARK Flex. Requires WPILib 2024 and SPARK Flex/SPARK MAX firmware 24.x.x.**

You can install the C++/Java version of this library using this JSON URL in VSCode:
```
https://software-metadata.revrobotics.com/REVLib-2024.json
```

The REVLib LabVIEW package is available to [download here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2024.2.0/REVLib-labVIEW-2024.2.0-0_windows_all.nipkg).

C++ docs: [https://codedocs.revrobotics.com/cpp/](https://codedocs.revrobotics.com/cpp/)
Java docs: [https://codedocs.revrobotics.com/java/](https://codedocs.revrobotics.com/java)

### Changes to C++, Java, and LabVIEW
* Throws an error if firmware version is less than 24.0.0
* Throws an error if the motor type is set to Brushed on a SPARK Flex while a SPARK Flex Dock is not connected
* Gets main encoder position with enhanced precision

### Changes to C++ and Java
* Sends a warning to the Driver Station if the wrong class is used for the type of SPARK that is connected
* Adds `CanSparkBase` class that exposes functionality that is common to both the SPARK MAX and the SPARK Flex
* Adds `CanSparkFlex` class that exposes all functionality of the SPARK Flex
    * `CanSparkFlex` has a `getExternalEncoder()` method that returns a `SparkFlexExternalEncoder` instead of a `getAlternateEncoder()` method that returns a `SparkMaxAlternateEncoder`.
    * This is because Alternate Encoder Mode is not necessary for SPARK Flex, and has been replaced by the External Encoder Data Port feature:
        * Can be used simultaneously with the internal encoders in NEO class motors
        * Can be used simultaneously with an absolute encoder and limit switches
        * Virtually no RPM limit
        * No special configuration
* The following items have been deprecated in favor of new equivalents:
    * Instead of `CANSparkMaxLowLevel`, use `CANSparkLowLevel`
    * Instead of `SparkMaxAbsoluteEncoder`, use `SparkAbsoluteEncoder`
    * Instead of `SparkMaxAnalogSensor`, use `SparkAnalogSensor`
    * Instead of `SparkMaxLimitSwitch`, use `SparkLimitSwitch`
    * Instead of `SparkMaxPIDController`, use `SparkPIDController`
    * Instead of `SparkMaxRelativeEncoder`, use `SparkRelativeEncoder`
    * Instead of `ExternalFollower.kFollowerSparkMax`, use `ExternalFollower.kFollowerSpark`
    	* The `ExternalFollower` enum can be accessed at `CANSparkMax.ExternalFollower`, `CANSparkFlex.ExternalFollower`, or `CANSparkBase.ExternalFollower`
* Adds a `CANSparkBase.getSparkModel()` method that returns a `SparkModel` enum

### Changes to LabVIEW 
* Deprecates old VIs that are prefixed with "Spark MAX" and replaces them with VIs prefixed with "SPARK"
  * Deprecated icons are "grayed out"
  * Help context (documentation) for deprecated VIs point the user to the equivalent new VI
  * New icons say "SPARK" instead of "REV MAX"
* Adds `SPARK Get Model.vi`
* Fixes `SPARK Get Analog Sensor Voltage.vi` when used with a SPARK Flex
* Updates `SPARK Get I Accum.vi` to get I Accum from status 7 instead of status 2
* Updates "Alternate Encoder" VIs to be "Alternate or External Encoder"
  * Only throw the data port config warnings when the device is a SPARK MAX

## revlib-2023.1.3  (2023-02-01T23:31:20Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2023.1.3

**This version of REVLib requires SPARK MAX Firmware v1.6.3. Please update your SPARK MAX through the REV Hardware Client to use with REVLib 2023.1.3.**

You can install the C++/Java version of this library using this JSON URL in VSCode:
```
https://software-metadata.revrobotics.com/REVLib-2023.json
```

The REVLib LabVIEW package is available to [download here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2023.1.3/REVLib-labVIEW-2023.1.3-2_windows_all.nipkg).

C++ docs: [https://codedocs.revrobotics.com/cpp/](https://codedocs.revrobotics.com/cpp/)
Java docs: [https://codedocs.revrobotics.com/java/](https://codedocs.revrobotics.com/java)

## Changelog:
* Improves documentation for the setZeroOffset() and getZeroOffset() methods on Absolute Encoder objects
* Fixes issue where reading an absolute encoder’s zero offset could return an incorrect value in certain conditions

## revlib-2023.1.2  (2023-01-13T20:03:08Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2023.1.2

**This version of REVLib requires SPARK MAX Firmware v1.6.2. Please update your SPARK MAX through the REV Hardware Client to use with REVLib 2023.1.2.**

You can install the C++/Java version of this library using this JSON URL in VSCode:
```
https://software-metadata.revrobotics.com/REVLib-2023.json
```

The REVLib LabVIEW package is available to [download here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2023.1.2/REVLib-labVIEW-2023.1.2-0_windows_all.nipkg).

C++ docs: [https://codedocs.revrobotics.com/cpp/](https://codedocs.revrobotics.com/cpp/)
Java docs: [https://codedocs.revrobotics.com/java/](https://codedocs.revrobotics.com/java)

## Changelog:
* Adds support to configure the hall sensor's velocity measurement
  * C++/Java: Updates `SetMeasurementPeriod()` and `SetAverageDepth()` in the `SparkMaxRelativeEncoder` class to be used when the relative encoder is configured to be of type `kHallSensor`.
  * LabVIEW: Adds `SPARK MAX Configure Hall Sensor.vi` and `SPARK MAX Get Hall Sensor Config.vi` to set and get the hall sensor's measurement period and average depth.

## revlib-2023.1.1  (2023-01-06T18:50:54Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2023.1.1

Official 2023 FRC kickoff release for REVLib. This release mainly adds capabilities for the SPARK MAX designed for use with swerve modules, such as the [REV 3in MAXSwerve Module](https://www.revrobotics.com/rev-21-3005/).

You can install the C++/Java version of this library using this JSON URL in VSCode:
```
https://software-metadata.revrobotics.com/REVLib-2023.json
```

The REVLib LabVIEW package is available to [download here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2023.1.1/REVLib-labVIEW-2023.1.1-0_windows_all.nipkg).

C++ docs: [https://codedocs.revrobotics.com/cpp/](https://codedocs.revrobotics.com/cpp/)
Java docs: [https://codedocs.revrobotics.com/java/](https://codedocs.revrobotics.com/java)

## Changelog:
* Adds support for using a duty cycle absolute encoder as a feedback device for the SPARK MAX.
  * C++/Java: Adds `SparkMaxAbsoluteEncoder` class.
  * LabVIEW: Adds VIs for configuring and getting the values from a duty cycle absolute encoder.
* Adds Position PID Wrapping to allow continuous input for the SPARK MAX PID controller.
  * C++/Java: Adds `PositionPIDWrapping` methods to the `SparkMaxPIDController` class.
  * LabVIEW: Adds VIs for setting and getting the Position PID Wrapping configuration.
* Allows configuring the periodic frame rates for status frames 4-6.

## Known issues:
* SparkMaxPIDController.setIAccum() only works while the control mode is active

## revlib-2022.1.2  (2022-03-07T23:41:41Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2022.1.2

## Breaking Changes

* LabVIEW: The version of NI Package Manager bundled with the FRC LabVIEW offline installer will no longer work when installing the REVLib package. NIPM must be updated to the latest version or installed from the FRC LabVIEW online installer to be able to install this package of REVLib for LabVIEW

## Enhancements

* LabVIEW: Adds `Spark MAX Set Inverted.vi` and `Spark MAX Get Inverted.vi`

## Known issues

* SparkMaxPIDController.setIAccum() only works while the control mode is active
* LabVIEW: VIs to get the SPARK MAX controller parameters do not work


## revlib-2022.1.1  (2022-01-11T00:50:50Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2022.1.1

## Fixes
* Adds Linux aarch64 (64-bit ARM) build
* C++: Adds missing `GetAlternateEncoder(int countsPerRev)` method

## Known issues
* SparkMaxPIDController.setIAccum() only works while the control mode is active


## revlib-2022.1.0  (2022-01-07T23:32:28Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2022.1.0

## Enhancements
* Java: Adds initial WPILib simulation support
  * Supports `ControlType.kVelocity` and `ControlType.kVoltage`
  * To use, make the following modifications to your Robot class (adjust parameters as necessary):
    * Call `RevPhysicsSim.getInstance().addSparkMax(sparkMax, DCMotor.getNEO(1))` from `simulationInit()`
    * Call `RevPhysicsSim.GetInstance.run()` from `simulationPeriodic()`
    * These changes will keep the simulated position value up-to-date.
  * Limitations
    * When in simulation mode, calling `setReference()` will only update the velocity of the primary encoder, even if `SparkMaxPIDController.setFeedbackDevice()` was called with a different feedback sensor

## Fixes
* C++: Fixes move semantics for supported classes

## Known issues
* SparkMaxPIDController.setIAccum() only works while the control mode is active

## revlib-2024.1.1  (2024-01-02T23:55:21Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2024.1.1

Adds support for SPARK Flex. This release is not intended for competition use in the 2024 FRC season. It is compatible with SPARK Flex firmware 23.x.x and SPARK MAX firmware 1.6.x, and requires WPILib 2024.

You can install the C++/Java version of this library using this JSON URL in VSCode:
```
https://software-metadata.revrobotics.com/REVLib-2024.json
```

This update is not yet available for LabVIEW. You can continue to use the [2024.0.0 beta version](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2024.0.0/REVLib-labVIEW-2024.0.0-0_windows_all.nipkg). Even though the VIs are named for the SPARK MAX, they will work with the SPARK Flex as well.

C++ docs: [https://codedocs.revrobotics.com/cpp/](https://codedocs.revrobotics.com/cpp/)
Java docs: [https://codedocs.revrobotics.com/java/](https://codedocs.revrobotics.com/java)

## Changelog

* Compatible with SPARK Flex firmware 23.x.x and SPARK MAX firmware 1.6.x
* Adds `CanSparkBase` class that exposes functionality that is common to both the SPARK MAX and the SPARK Flex
* Adds `CanSparkFlex` class that exposes all functionality of the SPARK Flex
    * `CanSparkFlex` has a `getExternalEncoder()` method that returns a `SparkFlexExternalEncoder` instead of a `getAlternateEncoder()` method that returns a `SparkMaxAlternateEncoder`.
    * This is because Alternate Encoder Mode is not necessary for SPARK Flex, and has been replaced by the External Encoder Data Port feature:
        * Can be used simultaneously with the internal encoders in NEO class motors
        * Can be used simultaneously with an absolute encoder and limit switches
        * Virtually no RPM limit
        * No special configuration
* The following items have been deprecated in favor of new equivalents:
    * Instead of `CANSparkMaxLowLevel`, use `CANSparkLowLevel`
    * Instead of `SparkMaxAbsoluteEncoder`, use `SparkAbsoluteEncoder`
    * Instead of `SparkMaxAnalogSensor`, use `SparkAnalogSensor`
    * Instead of `SparkMaxLimitSwitch`, use `SparkLimitSwitch`
    * Instead of `SparkMaxPIDController`, use `SparkPIDController`
    * Instead of `SparkMaxRelativeEncoder`, use `SparkRelativeEncoder`
    * Instead of `ExternalFollower.kFollowerSparkMax`, use `ExternalFollower.kFollowerSpark`
    	* The `ExternalFollower` enum can be accessed at `CANSparkMax.ExternalFollower`, `CANSparkFlex.ExternalFollower`, or `CANSparkBase.ExternalFollower`
* Adds a `CANSparkBase.getSparkModel()` method that returns a `SparkModel` enum



## revlib-2024.0.0  (2023-10-19T21:01:08Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2024.0.0

2024 beta release of REVLib. This requires the WPILib 2024 beta.


You can install the C++/Java version of this library using this JSON URL in VSCode:
```
https://software-metadata.revrobotics.com/REVLib-2024.json
```

The REVLib LabVIEW package is available to [download here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2024.0.0/REVLib-labVIEW-2024.0.0-0_windows_all.nipkg).

C++ docs: [https://codedocs.revrobotics.com/cpp/](https://codedocs.revrobotics.com/cpp/)
Java docs: [https://codedocs.revrobotics.com/java/](https://codedocs.revrobotics.com/java)

## Known issues:
* SparkMaxPIDController.setIAccum() only works while the control mode is active

## revlib-2023.0.1  (2022-12-16T17:34:33Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2023.0.1

You can install the C++/Java version of this library using this JSON URL in VSCode:
```
https://software-metadata.revrobotics.com/REVLib-2023.json
```

C++ docs: [https://codedocs.revrobotics.com/cpp/](https://codedocs.revrobotics.com/cpp/)
Java docs: [https://codedocs.revrobotics.com/java/](https://codedocs.revrobotics.com/java)

## Changelog:
* Adds support for osxuniversal

## Known issues:
* SparkMaxPIDController.setIAccum() only works while the control mode is active

## revlib-2023.0.0  (2022-11-03T20:49:57Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2023.0.0

2023 beta release of REVLib. This requires the WPILib 2023 beta.

You can install the C++/Java version of this library using this JSON URL in VSCode:
```
https://software-metadata.revrobotics.com/REVLib-2023.json
```

C++ docs: [https://codedocs.revrobotics.com/cpp/](https://codedocs.revrobotics.com/cpp/)
Java docs: [https://codedocs.revrobotics.com/java/](https://codedocs.revrobotics.com/java)

## Known issues:
* SparkMaxPIDController.setIAccum() only works while the control mode is active

## revlib-2022.0.0  (2021-11-19T18:16:59Z)  https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2022.0.0

First release of REVLib, which replaces the SPARK MAX API and the REV Color Sensor V3 API.

You can install the C++/Java version of this library using this JSON URL in VSCode:
`https://software-metadata.revrobotics.com/REVLib.json`

**Alternate URL for use with WPILib 2022 beta 3**
The version of VS Code in WPILib beta 3 erroneously rejects the certificate used at that URL. As a workaround, you can use the following URL instead:
`https://rev-robotics-software-metadata.netlify.app/REVLib.json`

You can install the LabVIEW version of this library by [installing this package](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2022.0.0/REVLib-labVIEW-2022.0.0-0_windows_all.nipkg).

C++ docs: [https://codedocs.revrobotics.com/cpp/](https://codedocs.revrobotics.com/cpp/)
Java docs: [https://codedocs.revrobotics.com/java/](https://codedocs.revrobotics.com/java)

## Breaking changes
* C++/Java: `CANError` has been renamed to `REVLibError`.
* Java: `ColorMatch.makeColor()` and the `ColorShim` class have been removed. Use the WPILib `Color` class instead.
* C++/Java: Deleted deprecated constructors, methods, and types
  * Replace deprecated constructors with `CANSparkMax.getX()` functions.
  * Replace `CANEncoder.getCPR()` with `getCountsPerRevolution()`.
  * Remove all usages of `CANDigitalInput.LimitSwitch`.
  * Replace `CANSparkMax.getAlternateEncoder()` with `CANSparkMax.getAlternateEncoder(int countsPerRev)`.
  * Remove all usages of `CANSparkMax.setMotorType()`. You can only set the motor type in the constructor now.
  * Replace `SparkMax` with `PWMSparkMax`, which is built into WPILib.
* Java: `CANSparkMax.get()` now returns the velocity setpoint set by `set(double speed)` rather than the actual velocity, in accordance with the WPILib `MotorController` API contract.
* C++/Java: `CANPIDController.getSmartMotionAccelStrategy()` now returns `SparkMaxPIDController.AccelStrategy`.
* C++/Java: Trying to do the following things will now throw an exception:
  * Creating a `CANSparkMax` object for a device that already has one
  * Specifying an incorrect `countsPerRev` value for a NEO hall sensor
  * Java: Calling a `CANSparkMax.getX()` method using different settings than were used previously in the program
  * Java: Trying to use a `CANSparkMax` (or another object retrieved from it) after `close()` has been called
  * C++: Calling a `CANSparkMax.getX()` method more than once for a single device
* C++/Java: Deprecated classes in favor of renamed versions
  * C++ users will get `cannot declare field to be of abstract type` errors until they replace their object declarations with ones for the new classes. Java users will be able to continue to use the old classes through the 2022 season.
  * `AlternateEncoderType` is replaced by `SparkMaxAlternateEncoder.Type`.
  * `CANAnalog` is replaced by `SparkMaxAnalogSensor`.
  * `CANDigitalInput` is replaced by `SparkMaxLimitSwitch`.
  * Java: `CANEncoder` is replaced by `RelativeEncoder`.
  * C++: `CANEncoder is replaced by `SparkMaxRelativeEncoder` and `SparkMaxAlternateEncoder`.
  * `CANPIDController` is replaced by `SparkMaxPIDController`.
  * `CANSensor` is replaced by `MotorFeedbackSensor`.
  * `ControlType` is replaced by `CANSparkMax.ControlType`.
  * `EncoderType` is replaced by `SparkMaxRelativeEncoder.Type`.

## Enhancements:
* C++/Java: Added the ability to set the rate of periodic frame 3

## Fixes:
* C++/Java: `CANSparkMax.getMotorType()` no longer uses the Get Parameter API, which means that it is safe to call frequently
* Java: The `CANSparkMax.getX()` methods no longer create a new object on every call

## Known issues:
* `SparkMaxPIDController.setIAccum()` only works while the control mode is active

