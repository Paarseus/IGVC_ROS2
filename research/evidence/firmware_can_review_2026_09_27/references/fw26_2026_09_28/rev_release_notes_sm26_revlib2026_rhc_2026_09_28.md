<!-- Source: https://api.github.com/repos/REVrobotics/REV-Software-Binaries/releases (fetched 2026-09-28). Bodies verbatim. -->

## sm-26.1.5 (2026-03-12T20:00:17Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sm-26.1.5

* Fixes issue where Current Control would only spin in one direction
* Fixes USB compatibility for Mac


## sm-26.1.4 (2026-02-18T20:12:06Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sm-26.1.4

* Improves performance of USB bridging
* Removes Legacy Status Frame 0, which is unused by REV Hardware Client 2 and newer releases of REV Hardware Client 1
* Fixes issue where soft-limits only updated when the controller was enabled


## sm-26.1.3 (2026-02-06T22:34:47Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sm-26.1.3

* Add support for CAN enabled REV encoders as feedback sensors
* Fixes an issue where enabling an enabled periodic frame would reset the time to send out the frame.


## sm-26.1.2 (2026-01-31T01:04:07Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sm-26.1.2

* Fixes issue in initializing status 8 (frame that indicates setpoint, pid slot, etc.)


## sm-26.1.1 (2026-01-23T00:14:47Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sm-26.1.1

* Fixes follower mode on REV Hardware Client


## sm-26.1.0 (2026-01-10T00:51:29Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sm-26.1.0

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


## rhc-1.7.7 (2026-09-12T01:12:48Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/rhc-1.7.7

## Changelog

- Update banner for EOL

## rhc-1.7.6 (2026-03-11T23:07:19Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/rhc-1.7.6

## Changelog

- Adds banner about RHC2

## revlib-2027.0.0-alpha-6 (2026-07-28T22:17:02Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2027.0.0-alpha-6

REVLib should be available in the vendor dependencies of WPILib VS Code, but you can also use this JSON URL directly:

```txt
https://software-metadata.revrobotics.com/REVLib-2027.json
```

[Offline Installer](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2027.0.0-alpha-6/REVLib-offline-v2027.0.0-alpha-6.zip)

Refer to [WPILib Docs](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html) about installing 3rd party libraries.

## Changelog:

- [A301] Java: Fixes setCurrent()


## revlib-2027.0.0-alpha-5 (2026-07-27T22:24:17Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2027.0.0-alpha-5

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


## revlib-2027.0.0-alpha-4 (2026-07-02T17:40:58Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2027.0.0-alpha-4

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


## revlib-2027.0.0-alpha-3 (2026-05-29T21:45:10Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2027.0.0-alpha-3

## Changelog:
- [A301] Adds setRelativeEncoderPosition()
- [A301] Fixes setInverted() and getInverted()
- [ServoHub] Fixes internal crash when calling Status getters
- [A301] Renames getOutputCurrent() to getMotorCurrent()


## revlib-2026.0.5 (2026-03-12T18:23:23Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2026.0.5

You can install the Java/C++ version of this library using this JSON URL in VSCode:

https://software-metadata.revrobotics.com/REVLib-2026.json

This release does not include LabVIEW. v2026.0.1 for LabVIEW is available to download [here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2026.0.1/revlib_2026.0.1-0_windows_all.nipkg).

## Changelog:

- [SPARK] Adds warning for SPARK devices not on 2026 firmware

## revlib-2026.0.4 (2026-03-04T21:42:15Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2026.0.4

You can install the Java/C++ version of this library using this JSON URL in VSCode:

https://software-metadata.revrobotics.com/REVLib-2026.json

This release does not include LabVIEW. v2026.0.1 for LabVIEW is available to download [here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2026.0.1/revlib_2026.0.1-0_windows_all.nipkg).

## Changelog:

- [REVLib] Fixes issue where the SplineEncoder was not fetching its status periods at creation
- [REVLib] Updates SplineEncoder documentation

## revlib-2026.0.3 (2026-02-18T20:25:27Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2026.0.3

You can install the Java/C++ version of this library using this JSON URL in VSCode:

https://software-metadata.revrobotics.com/REVLib-2026.json

This release does not include LabVIEW. v2026.0.1 for LabVIEW is available to download [here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2026.0.1/revlib_2026.0.1-0_windows_all.nipkg).

## Changelog:

- [REVLib] Java/C++ - Fixes MAXSpline Encoder issues/crashes


## revlib-2026.0.2 (2026-02-13T18:18:06Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2026.0.2

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


## revlib-2026.0.1 (2026-01-13T20:50:02Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2026.0.1

You can install the Java/C++ version of this library using this JSON URL in VSCode:

```
https://software-metadata.revrobotics.com/REVLib-2026.json
```

REVLib v2026.0.1 for LabVIEW is available to download [here](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2026.0.1/revlib_2026.0.1-0_windows_all.nipkg).

## Changelog

- [REVLib] Java/C++: Fixes simulation crash on MacOS
- [REVLib] C++: Fixes cpp check warnings
- [REVLib] LabVIEW: Updates for 2026 FRC LabVIEW


## revlib-2026.0.0 (2026-01-10T00:27:50Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2026.0.0

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

## sm-26.0.0-prerelease.2 (2025-11-21T22:50:16Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sm-26.0.0-prerelease.2

* Removes SmartMotion and SmartVelocity in favor of MAXMotion
* Fixes bug that can prevent force enable parameters from resetting correctly
* Switches the USB CAN bridging to use SLCan
    * Note: Devices that use SLCan are NOT compatible with REV Hardware Client when used over USB. Devices running v26.0.0-prerelease.2+ can only be used via RHC2, or downstream of a REV CAN device running v26.0.0-prerelease.1 or lower with RHC.
* Changes model value to 2 for MAX in Status 0 frame


## sm-26.0.0-prerelease.1 (2025-08-08T18:47:27Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/sm-26.0.0-prerelease.1

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


## revlib-2027.0.0-alpha-7 (2026-09-11T22:12:04Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2027.0.0-alpha-7

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

## revlib-2027.0.0-alpha-2 (2026-05-22T20:42:11Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2027.0.0-alpha-2

## Changelog:
- [A301] Adds support for A301
- [REVLib] Java/C++: Creates a Signal wrapper type for signals, allowing a user to know if a value is outdated. This is a breaking change and will require user code to call `.get()` / `.get(default)` in Java or `Get()` in C++. User code can query `.isValid()` in Java or `IsValid()` in C++ to know whether the value is recent.

## revlib-2027.0.0-alpha-1 (2025-06-24T17:17:56Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2027.0.0-alpha-1

REVLib should be available in the vendor dependencies of WPILib VS Code, but you can also use this JSON URL directly:

```txt
https://software-metadata.revrobotics.com/REVLib-2027.json
```

[Offline Installer](https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-2027.0.0-alpha-2/REVLib-offline-v2027.0.0-alpha-2.zip)

Refer to [WPILib Docs](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html) about installing 3rd party libraries.

## Changelog:

- [REVLib] Adds support for Systemcore and the WPILib 2027 alpha

## revlib-2026.0.0-beta-1 (2025-11-22T02:06:42Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2026.0.0-beta-1

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

## revlib-2026.0.0-alpha-1 (2025-08-08T18:34:54Z) https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2026.0.0-alpha-1

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

