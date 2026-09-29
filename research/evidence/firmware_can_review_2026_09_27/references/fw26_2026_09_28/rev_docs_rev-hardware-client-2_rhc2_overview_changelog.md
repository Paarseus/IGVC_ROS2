<!-- Source: https://docs.revrobotics.com/rev-hardware-client-2/rhc2/overview/changelog.md (fetched 2026-09-28) -->
> For the complete documentation index, see [llms.txt](https://docs.revrobotics.com/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://docs.revrobotics.com/rev-hardware-client-2/rhc2/overview/changelog.md).

# Changelog

{% hint style="warning" %}
For the most up-to-date changelog, please refer to the [About Tab](/rev-hardware-client-2/rhc2/navigation.md#about-tab) within the REV Hardware Client 2
{% endhint %}

### Version 1.4.2

* Adds a download location prompt for download actions from Control Hub's Robot Controller Console
* Improves recovery of wireless Control Hub connections after a Wi-Fi drop

### Version 1.4.1

* Adds a prompt on startup when an update is available, with the option to update now, on exit, or ignore it
* Adds a prompt on startup showing the changelog after the app has been updated
* Adds visualizer for absolute encoder utility
* Adds inversion configuration to A301
* Adds support for opening the update manager directly from a device's update tab to enable updating multiple devices of the same type
* Uses Control Hub's friendly name in device list
* Improves general experience with the FTC Log Viewer
* Updates the supported devices list on the About page to reflect all currently supported devices
* Fixes UI crash on Systemcore
* Fixes Robot Controller Console not working as expected for Control Hubs connected over USB
* Fixes Expansion Hub firmware version not showing up for Control Hub devices
* Fixes Control Hub menu labels being illegible in light mode
* Fixes false positive error toast when attempting to run an A301

### Version 1.4.0

* Adds controller support for running motors
* Adds auto-fetching latest releases on startup
* Adds toast to show when an action failed due to being in read-only mode
* Adds setpoint presets to A301/SPARK run pages
* Adds FTC Log Viewer
* Fixes crash when adding closed-loop control telemetry for a SPARK
* Fixes reset safe parameters showing the incorrect number of changes made
* Fixes run multiple checkbox not working with A301
* Fixes moving app to applications folder on macOS
* Fixes selecting an A301 opening a SPARK with the same CAN ID on the same bus

### Version 1.3.1

* Fixes crash on run tab with A301 on versions before prerelease 16

### Version 1.3.0

* Adds support for connecting to a remote RHC2 instance (like Systemcore) from a desktop client
* Adds support for installing/updating a Systemcore's RHC2 from the desktop client
* Adds ability for leader to release control and read only-clients to claim control
* Adds a prompt for Windows users to install DFU drivers if none are found
* Adds support for A301's absolute range offset parameter
* Adds tooltip and animation for identify button
* Improves robustness of leader/reader mechanism when clients disconnect
* Updates robot program override to only be controlled by the leader
* Fixes Systemcore serving a stale, cached UI after an update. You will need to empty cache and refresh the UI for this to take effect.
* Fixes frontend crash when downloads database file gets corrupted, particularly on Systemcore
* Fixes sorting of Motioncore buses in telemetry tab

### Version 1.2.1

* Adds support for new A301 CAN specification introduced in firmware version 27.0.0-prerelease.15
* Adds the ability to specify a speed for position control in A301's run utility
* Adds a toggle on the Motioncore card to override the robot program to run motors without a driver station
* Adds gearbox and motor health status cards for A301
* Fixes devices not showing up on the Hardware page in Safari

### Version 1.2.0

* Adds support for REV FTC devices (Control Hub, Expansion Hub, Driver Hub, Driver Station Phone)
* Adds support for CAN Encoder Adapter
* Adds search bar for signals on Telemetry page
* Adds search bar for SPARK configuration tab
* Adds relative encoder utility to MAXSpline Encoder
* Fixes sidebar on telemetry expanding when a device has an alert in either run tab
* Fixes issue reporting
* Fixes issue where signals would freeze after closing a device
* Fixes issue with A301 CAN ID change not persisting
* Fixes issue where firmware versions sometimes do not populate in the update page

#### Known Issues

* When connecting to a Control Hub via Wi-Fi, no other computer will be able to see the Control Hub until it is power-cycled. The same laptop can continue to see and use the Control Hub even if it reconnects or RHC2 is reopened.

### Version 1.1.1

* Adds missing applied output signal for A301
* Adds indicator to navigation bar to show whether you are in read-only mode
* Improves startup time
* Improves CPU and memory utilization during runtime
* Improves leader/reader behavior
* Improves download page
* Improves update progress status display
* Fixes update progress status when updating multiple devices across separate buses

### Version 1.1.0

* Adds support for A301
* Adds support for Linux Arm64
* Adds support for multiple separate CAN buses
* Adds support for updating the bootloader of supported devices
* Adds support for devices connected via Motioncore when running on Systemcore
* Adds relative encoder utility for resetting relative position
* Improves SPARK setpoint handling to minimize interfering with a connected robot's setpoint commands
* Fixes issue where SPARKs with pre-2025 firmware are not detected on the CAN bus

#### Known Issues

* Linux Arm64 does not currently support devices in DFU recovery mode

### Version 1.0.7

* Adds support for configuring new MAXSpline Encoder parameters
* Fixes start/stop button moving when running a SPARK

### Version 1.0.6

* Fixes motors not stopping immediately when running multiple motors simultaneously
* Fixes a bug causing devices to appear as remaining in bootloader mode after updating

### Version 1.0.5

* Adds button to install DFU drivers on Windows
* Adds warning when trying to run a motor without 12V
* Adds ability to reset SPARK slider to zero by double-clicking on slider handle
* Fixes splash screen not appearing in the foreground
* Fixes taskbar icon not appearing immediately on Windows
* Fixes SPARK slider setpoint resetting to zero after interacting with the minimum and maximum setpoint input boxes
* Fixes detecting MAXSpline Encoders in bootloader mode
* Fixes device drawer not defaulting to update page for devices in bootloader mode
* Fixes firmware version select taking a long time to load for devices in bootloader mode

### Version 1.0.4

* Adds warning about SPARK devices with an unconfigured CAN ID
* Adds button to clear telemetry signals
* Adds dialog to move the application into the Applications folder on macOS for improved experience:
  * Fixes devices in recovery mode not appearing on macOS
  * Fixes application not updating after clicking the Update button on macOS
* Adds AdvantageScope logo
* Fixes AdvantageScope sometimes not loading layout correctly on startup or when loaded from a file
* Fixes AdvantageScope not resuming when computer wakes from sleep
* Fixes regression preventing telemetry to resume when reconnecting a device

### Version 1.0.3

* Adds warning when trying to run a motor with the roboRIO present on the bus
* Adds error toast for when a SPARK parameter value is rejected
* Adds help dialog when loading devices takes a long time, especially for brand new SPARK MAXes on unsupported factory firmware
* Adds privacy policy
* Improves CAN utilization when running multiple devices on telemetry page
* Prevents multiple instances of the app from running at once
* Fixes bug causing SPARKs in brushed mode to get set to brushless upon connection
* Fixes issue with parameters not refreshing when switching devices on telemetry run tab
* Fixes spike in CAN utilization when opening a SPARK device by switching to lazy loading parameters
* Fixes issue with SPARK run slider sometimes continuing to drag with mouse after releasing
* Fixes device list sometimes not updating when unplugging USB
* Disables highlighting across general areas of the application
* Updates troubleshooting steps on about page

### Version 1.0.2

* Adds ability to start and stop multiple selected motors simultaneously
* Fixes high CAN utilization when switching between many SPARK device tabs
* Fixes an issue causing some SPARKs to continue spinning when disabling multiple at once
* Fixes an issue causing SPARK devices to resume spinning after leaving the bus and returning

### Version 1.0.1

* Fixes crash when opening on macOS
* Fixes detection of devices on Linux
* Fixes app icon

### Version 1.0.0

Initial stable release of REV Hardware Client 2
