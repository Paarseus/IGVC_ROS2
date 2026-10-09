<!-- source: https://github.com/UTNuclearRobotics/FRCCan/blob/8e2d039/README.md (commit 8e2d039, fetched 2026-09-28) -->
# proprietary_template
This provides a template for new NRG repos. It includes files that NRG repos generally all need.

## Things you'll need to fill in or update:
 - Copyright year in LICENSE.txt
 - The CHANGELOG.md
 - This README!
 - Add the new repository to the [NRG Repository List](https://wikis.utexas.edu/display/NRG/NRG+Repository+List)

## Adding a new catkin package
If you want to create a new catkin package, use `catkin_create_pkg` and move the files:
```sh
cd <INTO YOUR NEW REPOSITORY>
catkin_create_pkg <pkg_name> <depend1> <depend2> <...>
mv <pkg_name>/* .
rm -r <pkg_name>/
```

## CAN Message ID structure
```sh
    /* WPILib/FRC 29-bit CAN ID structure that prefixes all 8-byte data frames.
    * Meant to be an abstracted structure of the signal received below
    */
    /*  typedef struct [[gnu::packed]] FrcMsgID{
            uint32_t deviceType : 5; // [28-24]  motor controller, sensor, etc.
            uint32_t manufacturer : 8; // [23-16] ThriftyBot, REV, CTRE, etc.
            uint32_t apiClass : 6; // [15-10] manufacturer-defined api classes
            uint32_t apiIndex : 4; // [9-6] manufacturer-defined api indexes within classes
            uint32_t deviceID : 6; // [5-0] identity of the device that has been saved to the device and is used to call on specific device on CANbus(ex: CAN ID 6)
        } FrcMsgID_t; */
        
```
## A full FRC CAN Heartbeat in protocol
```sh
/* Universal CAN heartbeat that any device on bus can listen/react to, sent every 20ms, 8B packet
    * The heartbeat has a full CAN ID of 0x01011840 (which is the NI Manufacturer ID, RobotController type, Device ID 0 and API ID 0x061
    * The heartbeat encodes the robot state and mode into the signal below
    struct [[gnu::packed]] RobotState {
    uint64_t matchTimeSeconds : 8; [63-56]
    uint64_t matchNumber : 10; [55-46]
    uint64_t replayNumber : 6; [45-40]
    uint64_t redAlliance : 1; [39]
    uint64_t enabled : 1; [38]
    uint64_t autonomous : 1; [37]
    uint64_t testMode : 1; [36]             The only two bits that matter are testMode and systemWatchdog. 3rd party main controller can only send
    uint64_t systemWatchdog : 1; [35]       commands to SparkFlex when testMode enabled. Other bits don't affect motor behavior.
    uint64_t tournamentType : 3; [34-32]
    uint64_t timeOfDay_yr : 6; [31-26]
    uint64_t timeOfDay_month : 4; [25-22]
    uint64_t timeOfDay_day : 5; [21-17]
    uint64_t timeOfDay_sec : 6; [16-11]
    uint64_t timeOfDay_min : 6; [10-5]
    uint64_t timeOfDay_hr : 5; [4-0]
    };  */ 
    
```
    
## Broadcast message signal options
```sh
    enum class BroadcastMsg : uint8_t{
        disable = 0,        //  Instantly commands all devices on the bus to transition into a safe, non-functional, or un-powered output state.
        systemHalt = 1,     // Pauses execution or stops all motor movements globally, typically freezing operations immediately without fully cutting device power.
        systemReset = 2,    // Forces a full software or hardware reboot on every connected node simultaneously.
        deviceAssign = 3,   // Dynamically assigns unique node IDs or configurations to newly discovered devices during network setup.
        deviceQuery = 4,    // Polls the bus to ask all connected devices to report their current status, presence, or ID tracking.
        heartbeat = 5,      // A regular, timed signal sent to keep devices alive; if missed, nodes automatically enter a safe disabled state.
        sync = 6,           // Synchronizes internal clocks, sensors, or scheduled operations across multiple devices to ensure they execute simultaneously.
        update = 7,         // Signals nodes to apply a new configuration or prepare to receive a data packet update.
        firmwareVersion = 8, // Commands every device to broadcast its current firmware version back to the master controller.
        enumerate = 10,     // Initiates a discovery scan where all nodes on the bus must announce themselves to map out the network topology.
        systemResume = 11   // Re-enables operations and un-freezes devices that were previously paused by a systemHalt command.
    };
    
```


## NRG Sample ROS Code
See [nrg_ros_samples](https://github.com/UTNuclearRobotics/nrg_ros_samples) for examples of good ROS practices, CMakeLists help, and more.
