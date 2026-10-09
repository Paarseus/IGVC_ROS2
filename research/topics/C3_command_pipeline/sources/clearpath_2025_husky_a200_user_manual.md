Source: https://docs.clearpathrobotics.com/docs_robots/outdoor_robots/husky/a200/user_manual_husky/
Retrieved: 2026-09-27


## Introduction ​
Husky is a rugged and easy-to-use unmanned ground robot for rapid prototyping and research applications.
Mobile robots are inherently hazardous. It is the responsibility of the deployer to ensure that
all personnel that may come into contact with the system are trained properly. It is also the
responsibility of the deployer to understand the hazards presented and to ensure sufficient
safeguards or other mitigations are in place. Refer to the Safety section of
this manual for further details.

### Shipment Contents ​
Your Husky shipment contains the following:
- Clearpath Robotics Husky
- 24 V Sealed Lead-Acid Battery Pack
- Battery Door Cover
- Battery Charger
- Lockout Keys (2X)
- Power Connectors (3X)
If you purchased standard payload modules or custom integration services with Husky, then additional equipment will be included per your specific configuration, plus further documentation as required.

### Hardware Overview ​

#### System Architecture ​
Husky is built around a computer running Ubuntu (Intel-based computer or Jetson developer kit), paired with a 32-bit microcontroller MCU.
The MCU handles power supply monitoring and motor control.
The communication channel between the MCU and computer is a serial connection.

#### Exterior Features ​
The Husky exterior is shown below and includes:
- Extruded aluminum mounting rail
- User payload area, including user power panel
- Front and rear bumper bars
- Battery bay
- Status panel
- Power button
- Motion Stop button and lockout

#### Status Panel ​
The status panel is a display of LED indicators located on the rear of the chassis which provide information about the current status of Husky.
The indicators are described in the table below.
| Icon | Description |
|  | Battery Status The four LED segments provide an approximate indication of the relative lifetime remaining in the battery. |
|  | Communication Status When green, Husky is receiving a stream of correctly-formatted motion commands, and is ready to drive. When yellow, Husky is receiving commands, but will not drive due to emergency stop or another error. When red, serial communications are currently timed-out. |
|  | General Error Status Illuminates red when Husky will not drive due to an error state. Such states include emergency stop, insufficient battery power, or an unspecified software error. |
|  | Emergency Stop Status Illuminates red when Husky will not drive due to the emergency stop being activated, either onboard or wireless (if available). |
|  | Charge Indicator Illuminates red when Husky user power is being supplied externally. |

#### Buttons and Lockouts ​
The silver button at the top of the Status Panel is used to power the Husky on and off. From the "off" state, pressing the button
will cause the Husky to power on and a blue ring at the edge of the button will be illuminated. From the "on" state, pressing
the button will cause the Husky to power off and the blue ring will no longer be illuminated.
See Stop button and Lockout for details of the Stop button and Lockout.

#### Payloads ​
Additional payloads such IMUs, lidars, cameras, GPS, manipulators and many more can be integrated with Husky, either as part of your Husky purchase or installed after receiving your Husky.
You can find a list of supported packages and related documentation for payloads in this documentation portal, under the Computers , Sensors , Manipulators , and Accessories sections.
In addition, refer to Husky Integration for details on how to integrate your own payload or how to integrate a payload kit from Clearpath Robotics.
Each payload kit that ships from Clearpath Robotics comes with a custom mounting bracket and cabling for easy attachment to Husky.

#### Orientation References ​
The reference frame used by all Clearpath Robotics ground robots is based on ISO 8855, and is shown in the figure below.
Husky's front is shown.
When commanded with a positive translational velocity (forward), wheels travel in the positive X-direction.
The direction of the axes differs from those used for roll, pitch, and yaw in aircraft, and care should be taken to ensure that data is interpreted correctly.

#### Robot Equations ​
As a starting point, Clearpath Robotics recommends using the following relationship between wheel velocity and platform velocity:
v v v represents the instantaneous translational speed of the platform and ω \omega ω the instantaneous rotational speed. v { r } v_\{r\} v { ​ r } and v { l } v_\{l\} v { ​ l } are the right and left wheel velocities, respectively. W W W is the effective track of the robot, 0.555 m.

### System Specifications ​
| Specification | Measurement |
| Dimension, Length | 990 mm |
| Dimension, Width | 670 mm |
| Dimension, Height | 390 mm |
| Track Width | 550 mm |
| Wheelbase Length | 512 mm |
| Ground Clearance | 130 mm |
| Mass | 50 kg |
| Maximum Payload¹ | 75 kg |
| All-terrain Payload² | 20 kg |
| Speed, Maximum | 1 m/s |
| Climb Grade | 45 ° |
| Traversal Grade / Sideslope | 30 ° |
| Operating Temperature, Minimum | -10 °C |
| Operating Temperature, Maximum | 40 °C (not in direct sunlight) |
| Operating Time, Average | 3 hours |
| Operating Time, Standby | 8 hours |
| Battery | 20 Ah @ 24 V, sealed lead-acid |
| Battery Charger | Short-circuit, over-current, over-voltage and reverse voltage protection |
| Charge Time | 10 hours |
| User Power, 2021 and older | 5 V, 12 V, V-battery, each fused at 5 A |
| User Power, 2022 and newer | 12 V @ 7.5 A, 24 V @ 3 A, VBAT @ 5 A |
| Communication | RS-232, 115200 baud |
| Wheel Encoders | 78,000 ticks/m |
| Internal Sensing | Battery Status, Wheel Odometry, Motor Currents |
| Environmental | IP44 (base Husky); IP30 (with computer); IP55 (with weatherproofing kit) |
¹Continuous operation on relatively flat terrain with wide turns. ²robot climbing 30° grade with high-mounted payload, or turning in place in high-friction conditions.

## Getting Started ​
You are ready to go! This section details how to get Husky into action.
To begin, place Husky "up on blocks" as described in the Safety section, making sure that the wheels are clear of the ground.
For most Husky setups, there will be an Onboard Computer which is directly connected to the Husky and an Offboard Computer which is used to control Husky and gather data.
These two computers must be set up to communicate with each other.

### Onboard Computer Setup ​
If you purchased an Onboard Computer from Clearpath Robotics with Husky, it is already installed, connected, and powered.
If not, or to set up your own Onboard Computer, see the Husky Tutorials .
Note that the Onboard Computer must be connected to Husky's serial port, which is located in the user power panel accessible from the User Bay.
The connector is a female DE-9, suitable for connection directly to a USB-serial adaptor or to a computer serial port via a straight-through modem cable (not a null modem cable).
How you connect the Onboard Computer to Husky depends on whether the Onboard Computer has a dedicated serial port.
Husky is delivered with a USB-serial adaptor.

### Powering Up ​
Husky comes with the battery fully charged and installed, but disconnected for safety during shipping.
To reconnect the battery:
- Ensure Husky's main power button is in the "off" position and that stop mode is activated.
- Using curled index fingers to grip the battery door latches, press with thumbs on the latch buttons
and lift the battery door free of the chassis.
- Carefully connect the battery plug to the mating connector in the fuse panel.
Ensure that it is firmly seated and clicks into place.
It will be difficult to connect it in the incorrect polarity.
- Replace the battery door cover, ensuring that it clicks into place and is flush with the rear panel.
To power on Husky, firmly press the power button located above the status panel.
It will illuminate blue, and the status panel lights will show a brief test pattern.
The COMM status light will turn red, as the computer is not yet communicating with Husky.
The STOP status light will be red if the Stop button is pressed.
If the Stop button is not pressed, press it and verify that the STOP light illuminates.
Leave the platform in stop mode until it is successfully receiving commands, indicated by the COMM status light turning green or yellow.
If a computer was purchased with Husky, it will be setup to interface automatically upon startup; 60-90 seconds after booting up, COMM status light will change from red to green (or yellow, depending on the stop mode status).
This indicates that ROS is up on the computer and has established communications with the Husky base.

### Network Configuration ​
To get Husky connected to your local Wi-Fi network, you must first access the Onboard Computer using a wired connection to the robot.
See the Husky Tutorials for details.

### Connecting and Using the Controller ​
To use your controller, it must be paired with your Husky and powered on.
If your purchased the Onboard Computer from Clearpath Robotics, your controller is already paired with your Husky.
If your controller has become unpaired or if you wish to pair a new controller, following the instructions in the Husky Tutorials .
To teleoperate Husky, first release the motion stop. Then, referring to the following images, press the enable button (either slow or fast mode) and use the left thumbstick to drive Husky.

### Using ROS ​
Robot Operating System (ROS) is an extensible framework for controlling and working with robotic systems.
Clearpath Robotics recommends using ROS with your robot. If this is your first time using ROS, it is strongly
recommended to run through our series of ROS tutorials to learn
the basics of ROS.

### Battery Charging ​
The battery shipped with Husky A200 may be charged outside the robot or while still inside of Husky.
If charging while installed, you will need to open the access door to the battery area, and disconnect the battery from the MCU board.
From there:
- Connect the DC output cable from the charger to the battery terminal connector.
- Plug the charger power cord into the charger and then into a wall receptacle.
- The LED light on the charger will illuminate red.
- When battery is fully charged, the LED light will turn green.
- Unplug the charger from the wall and then disconnect it from the battery.
The battery charger shipped with Clearpath Robotics's lead-acid battery-powered products uses a two-state charge cycle.
- A fast, high current initial charge, until the battery voltage reaches 27.2 V.
- A topping charge, during which the battery charges at 27.2 V constant voltage.
Some older Husky chargers use a three-state charge cycle. For those older chargers, the LED light turns off
(rather than green) when charging is complete.

## Safety Considerations ​
Clearpath Robotics is committed to high standards of safety.
Husky contains several features to protect the safety of users and the integrity of the robot.

### General Warnings ​
Husky is a rugged and high-performance robot.
For the safety of yourself and others, always conduct initial experiments and software development with
the robot raised off the ground. Place a wooden crate, a set of sawhorses, a sturdy storage tub, or any
other solid flat structure having a height greater than 150 mm under Husky to keep the wheels clear of the
ground ("up on blocks").
When starting out, favour slower wheel speeds. Husky's control loops can accurately maintain velocities as low as 0.1 m/s.
Operating at such speeds will give you more time to react if things don't go quite as you expect.
When Husky is operating, keep clear of the wheels, paying particular attention to the pinch hazard which exists between each wheel and the end of the corresponding bumper bar.

### Stop Button and Lockout ​
Ensure the Stop button is accessible at all times.
Avoid mounting payloads that extend over the rear of Husky and would occlude the Stop button.
The motion-stop function allows a person in the physical vicinity of the robot to halt its motion and prevent re-initiation of motion.
Power supply to Husky's motor drivers is enabled by a normally-open relay, which is closed in series with the stop switch.
This motion-stop system is intended for use in emergency situations.
The motion stop system should not be used as the means of controlling robot motion during regular operation.
When in stop mode, the status panel stop light will illuminate red and Husky will not drive.
The commands received during stop mode are not buffered; Husky will always act on the latest commands received.
This means that if the commands are stopped before the stop is released, the Husky will not move.
If the commands are continued, Husky will move at the speed commanded once the stop is released.
This "Stop" function can be asserted by pressing the red stop actuator present at the rear of the Husky, opposite the status panel, as shown in the following figure.
The motion-stop function can be reset by twisting the depressed actuator until it returns to an extended position.
There is no second "start" action required to reset the stop function.
The robot may begin moving again immediately after the stop actuator has returned to its extended position.
The lockout provides a way to secure Husky from performing any type of motion.
When in lockout mode, the robot will still power on, but the motors will not drive.

### Electrical System ​
Husky is powered by a single 24 V sealed lead-acid battery, similar to the type found in electric wheelchairs, golf carts, and other devices.
Husky's battery is capable of delivering 1800 W, similar to wall mains.
This gives Husky's motors their great performance, however, it is also enough power to cause severe bodily harm.
Please observe the following precautions:
- Do not tamper with the plug attached to the battery.
- Do not tamper with the fuse panel, except to check and change the fuses, and to connect and disconnect the battery plug.
- Do not operate Husky without the battery door in place.
The battery is not restrained without the door, and will come loose, damaging the fuse panel.
- Charge the battery only with the charger provided by Clearpath Robotics.
- Please dispose of the batteries properly.

### Lifting and Transport ​
For the safety of users and to maximize the lifetime of Husky, please observe the following when manually transporting the robot:
- Husky should be lifted by two persons, firmly gripping the front and rear bumper-bars.
The battery can be removed to lighten the load of a manual transport.
- Ensure that Husky is in Stop mode when transporting short distances and powered off when transporting longer distances.
- It is not advised to push Husky, as this can cause damage to the motors.

### Performance Considerations ​
Included in Husky are native software checks and limits to protect the robot.
However, it is recommended to monitor the system's status during usage with the /status and /diagnostics topics.
These topics provide useful information regarding voltages, currents, temperatures and general health of the system.
The total current draw does not include the motors drivers; it is the current consumed by the MCU and user power ports.
Husky's motors are rated to draw 10.7 A continuous, but they will spike to several times higher than this, particularly when traversing rough terrain and when turning on the spot.
To reduce current draw, consider commanding wider-radius turns from your control software.
The temperature is measured in the motor drivers and on the motor casings; the coils inside the motor casings cannot be measured.
Therefore, it is important to note that the temperature measured on the motor casings is a lagging indicator of the temperature of the coils inside the casing.
Be aware of the delay in heat propagation on the motors during heavy use.
The thermal limit of the system is 50 °C, and the system will shut down if this limit is reached.
Monitoring these fields over longer periods of operation will allow you to ensure that you are not putting excessive wear on Husky's motors.

### Recommended Safe Work Procedures ​
Clearpath Robotics recommends the following procedures be followed to reduce the likelihood of harm.
- Maintain a safe distance of at least 10 m from the robot whenever possible.
- Never assume that the robot can be reliably stopped on command.
- Assert a stop using the remote stop device (if present) before approaching the robot.
- Never place the remote stop device (if present) on or inside the robot.
- Always be vigilant around the robot, even when a stop is asserted.
- Perform no maintenance or troubleshooting on the robot without training and familiarization.
- Follow electrical safety best practices when working with electrical systems and components.
- Disconnect battery using main disconnect of the robot before working inside robot.
- Wear personal protective equipment including high-visibility clothing, head and eye protection while
operating, observing or working on the robot.
- Never climb on or ride the robot.
- Never lie on the ground under or near the robot.
- Avoid operating the robot on slopes and grades.
- Never stand downhill of the robot.
- Operate the robot at the lowest speed possible.
- Never attempt to defeat a safety function for any reason.
- Chock the robot's wheels when the robot will be stationary for more than a few seconds.
- Never work on the robot while it is on a slope.
- Replace fuses only with identical product from specified manufacturer.
- Discontinue use and contact Clearpath Robotics support at the first sign of strange robot behaviour.
- Keep tools and loose parts away from electrical components and out of the robot's bays.
- Check the functionality of stop systems on a regular basis, including before and after each use.

## Support ​
Clearpath is committed to your success.
Please get in touch with us and we will do our best to get you rolling again quickly: support@clearpathrobotics.com .
To get in touch with a salesperson regarding Clearpath Robotics products, please email research-sales@clearpathrobotics.com .
If you have an issue that is specifically about ROS and is something which may be of interest to the broader community,
consider asking it on Robotics Stack Exchange .
If you do not get a satisfactory response, please ping us and include a link to your question as posted there.
If appropriate, we will answer on Robotics Stack Exchange for the benefit of the community.
