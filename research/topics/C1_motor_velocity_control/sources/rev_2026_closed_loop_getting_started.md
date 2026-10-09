<!-- Source: https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started (fetched 2026-09-27, official Markdown export) -->

> For the complete documentation index, see [llms.txt](https://docs.revrobotics.com/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started.md).

# Closed Loop Control Getting Started

## Setting up Closed-Loop Control

Closed-loop control in REVLib is accessed through the SPARK's closed loop controller object. This object is specific to each motor and contains all the methods needed to control your motor with closed-loop control. It can be accessed as shown below:

{% tabs %}
{% tab title="Java" %}

```java
// Initialize the motor (Flex/MAX are setup the same way)
SparkFlex m_motor = new SparkFlex(deviceID, MotorType.kBrushless);

// Initialize the closed loop controller
SparkClosedLoopController m_controller = m_motor.getClosedLoopController();
```

API Docs: [SparkFlex](https://codedocs.revrobotics.com/java/com/revrobotics/spark/sparkflex), [SparkClosedLoopController](https://codedocs.revrobotics.com/java/com/revrobotics/spark/sparkclosedloopcontroller)
{% endtab %}

{% tab title="C++" %}

```cpp
using namespace rev::spark;

// Initialize the motor (Flex/MAX are setup the same way)
SparkMax m_motor{deviceID, SparkMax::MotorType::kBrushless};

// Initialize the closed loop controller
SparkClosedLoopController m_controller = m_motor.GetClosedLoopController();
```

API Docs: [SparkMax](https://codedocs.revrobotics.com/cpp/classrev_1_1spark_1_1_spark_max.html), [SparkClosedLoopController](https://codedocs.revrobotics.com/cpp/classrev_1_1spark_1_1_spark_closed_loop_controller.html)
{% endtab %}
{% endtabs %}

To drive your motor in a closed-loop control mode, address the closed loop controller object and give it a set point (a target in whatever units are required by your control mode: position[^1], velocity[^2], or current[^3]) and a control mode as shown below:

{% hint style="info" %}
This will run your motor in the provided mode, but it won't move until you've configured the [PID constants.](#pid-constants-and-configuration)
{% endhint %}

{% tabs %}
{% tab title="Java" %}

```java
// Set the setpoint of the PID controller in raw position mode
m_controller.setSetpoint(setPoint, ControlType.kPosition);
```

API Docs: [setSetpoint](https://codedocs.revrobotics.com/java/com/revrobotics/spark/sparkclosedloopcontroller#setSetpoint\(double,com.revrobotics.spark.SparkBase.ControlType\)), [ControlType](https://codedocs.revrobotics.com/java/com/revrobotics/spark/sparkbase.controltype)
{% endtab %}

{% tab title="C++" %}

```cpp
// Set the setpoint of the PID controller in raw position mode
m_controller.SetSetpoint(setPoint, SparkBase::ControlType::kPosition);
```

API Docs: [SetSetpoint](https://codedocs.revrobotics.com/cpp/classrev_1_1spark_1_1_spark_closed_loop_controller.html#aefb8aa2d8ea8533a8e726f58c58facc9), [ControlType](https://codedocs.revrobotics.com/cpp/classrev_1_1spark_1_1_spark_low_level.html#afd05d7bbe5176b6b9ed55f854af116d0)
{% endtab %}
{% endtabs %}

The provided example above runs the motor in position control mode, which is just a conventional PID loop reading the motor's current position from the configured encoder and taking a setpoint in rotations.

{% hint style="danger" %}
Use caution when running motors in closed-loop modes, as they may move very **quickly** and **unexpectedly** if improperly tuned.
{% endhint %}

## PID Constants and Configuration

To run a PID loop, several constants are required. More advanced controllers require additional parameters to be set and tuned.

{% hint style="info" %}
To read more about configuration, see [this page on general configuration](/revlib/configuring-devices.md). For more information about SPARK specific configuration, see [this page](/revlib/spark/configuring-a-spark.md).
{% endhint %}

### PID Parameters

A PID controller has 3 core parameters or gains. For more information on these gains and how to tune them, see [Getting Started with PID Tuning](/revlib/spark/closed-loop/getting-started-with-pid-tuning.md).

These gains can be configured on the with the `closedLoop` member of a `SparkFlexConfig`or `SparkMaxConfig` object as seen below:

{% tabs %}
{% tab title="Java" %}

```java
SparkFlexConfig config = new SparkFlexConfig();

// Set PID gains
config.closedLoop
    .p(kP)
    .i(kI)
    .d(kD)
    .outputRange(kMinOutput, kMaxOutput);
```

API Docs: [ClosedLoopConfig](https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/closedloopconfig)
{% endtab %}

{% tab title="C++" %}

```cpp
using namespace rev::spark;

SparkFlexConfig config;

// Set PID gains
config.closedLoop
    .P(kP)
    .I(kI)
    .D(kD)
    .OutputRange(kMinOutput, kMaxOutput);
```

API Docs: [ClosedLoopConfig](https://codedocs.revrobotics.com/cpp/classrev_1_1spark_1_1_closed_loop_config.html)
{% endtab %}
{% endtabs %}

### Feedforward Parameters

There are several Feedforward parameters that can be used to model your system and help support the PID controller, resulting in more precise and consistent motions. These are explained on the [Feed Forward Control page](/revlib/spark/closed-loop/feed-forward-control.md).

{% tabs %}
{% tab title="Java" %}

```java
SparkFlexConfig config = new SparkFlexConfig();

// Set PID gains
config.closedLoop.feedForward
    .kS(s)
    .kV(v)
    .kA(a)
    .kG(g) // kG is a linear gravity feedforward, for an elevator
    .kCos(g) // kCos is a cosine gravity feedforward, for an arm
    .kCosRatio(cosRatio); // kCosRatio relates the encoder position to absolute position
```

API Docs: [ClosedLoopConfig](https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/closedloopconfig)
{% endtab %}

{% tab title="C++" %}

```cpp
using namespace rev::spark;

SparkFlexConfig config;

// Set PID gains
config.closedLoop.feedForward
    .kS(s)
    .kV(v)
    .kA(a)
    .kG(g) // kG is a linear gravity feedforward, for an elevator
    .kCos(g) // kCos is a cosine gravity feedforward, for an arm
    .kCosRatio(cosRatio); // kCosRatio relates the encoder position to absolute position
```

API Docs: [ClosedLoopConfig](https://codedocs.revrobotics.com/cpp/classrev_1_1spark_1_1_closed_loop_config.html)
{% endtab %}
{% endtabs %}

### MAXMotion Parameters

MAXMotion has parameters that allow you to configure and tune the motion profiles generated by MAXMotion. The parameters can be set through the `maxMotion` member of the `closedLoop` config.

{% hint style="warning" %}
The MAXMotion Cruise Velocity parameter only applies to MAXMotion Position Control Mode, while MAXMotion Velocity Control Mode does not honor it in order to ensure any setpoint is reachable. This means any top-speed clamping you want to do must be done *before* you send the setpoint to the Motor Controller.
{% endhint %}

{% tabs %}
{% tab title="Java" %}

```java
SparkMaxConfig config = new SparkMaxConfig();

// Set MAXMotion parameters
config.closedloop.maxMotion
    .cruiseVelocity(cruiseVel)
    .maxAcceleration(maxAccel)
    .allowedProfileError(allowedErr);
```

API Docs: [MAXMotionConfig](https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/maxmotionconfig)
{% endtab %}

{% tab title="C++" %}

```cpp
using namespace rev::spark;

SparkMaxConfig config;

// Set MAXMotion parameters
config.closedloop.maxMotion
    .CruiseVelocity(cruiseVel)
    .MaxAcceleration(maxAccel)
    .AllowedProfileError(allowedErr);
```

API Docs: [MAXMotionConfig](https://codedocs.revrobotics.com/cpp/classrev_1_1spark_1_1_m_a_x_motion_config.html)
{% endtab %}
{% endtabs %}

{% hint style="info" %}
Cruise Velocity is in units of Revolutions per Minute (RPM) by default

Maximum Acceleration is in units of RPM per Second (RPM/s) by default
{% endhint %}

### Slots

The SPARK MAX and SPARK Flex each have 4 closed-loop slots, each with their own set of constants. These slots are numbered 0-3. You can pass the desired as an argument to each of the applicable configurations.

{% tabs %}
{% tab title="Java" %}

```java
SparkFlexConfig config = new SparkFlexConfig();

config.closedLoop
    // Set PID gains for position control in slot 0.
    // We don't have to pass a slot number since the default is slot 0.
    .p(kP)
    .i(kI)
    .d(kD)
    .outputRange(kMinOutput, kMaxOutput)
    // Set PID gains for velocity control in slot 1
    .p(kP1, ClosedLoopSlot.kSlot1)
    .i(kI1, ClosedLoopSlot.kSlot1)
    .p(kD1, ClosedLoopSlot.kSlot1);
```

API Docs: [ClosedLoopConfig](https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/closedloopconfig)
{% endtab %}

{% tab title="C++" %}

```cpp
using namespace rev::spark;

SparkFlexConfig config;

config.closedLoop
    // Set PID gains for position control in slot 0.
    // We don't have to pass a slot number since the default is slot 0.
    .P(kP)
    .I(kI)
    .D(kD)
    .OutputRange(kMinOutput, kMaxOutput)
    // Set PID gains for velocity control in slot 1
    .P(kP1, ClosedLoopSlot::kSlot1)
    .I(kI1, ClosedLoopSlot::kSlot1)
    .D(kD1, ClosedLoopSlot::kSlot1);
```

API Docs: [ClosedLoopConfig](https://codedocs.revrobotics.com/cpp/classrev_1_1spark_1_1_closed_loop_config.html)
{% endtab %}
{% endtabs %}

When applying the setpoint, pass the slot number and the motor controller will switch to the appropriate config.

{% tabs %}
{% tab title="Java" %}

```java
// Use the PID gains in slot 0 for position control
m_controller.setSetpoint(setPoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);

// Use the PID gains in slot 1 for velocity control
m_controller.setSetpoint(setPoint, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
```

API Docs: [setSetpoint](https://codedocs.revrobotics.com/java/com/revrobotics/spark/sparkclosedloopcontroller#setSetpoint\(double,com.revrobotics.spark.SparkBase.ControlType\)), [ControlType](https://codedocs.revrobotics.com/java/com/revrobotics/spark/sparkbase.controltype)
{% endtab %}

{% tab title="C++" %}

```cpp
using namespace rev::spark;

// Use the PID gains in slot 0 for position control
m_controller.SetSetpoint(setPoint, SparkBase::ControlType::kPosition, ClosedLoopSlot::kSlot0);

// Use the PID gains in slot 1 for velocity control
m_controller.SetSetpoint(setPoint, SparkBase::ControlType::kVelocity, ClosedLoopSlot::kSlot1);
```

API Docs: [SetSetpoint](https://codedocs.revrobotics.com/cpp/classrev_1_1spark_1_1_spark_closed_loop_controller.html#aefb8aa2d8ea8533a8e726f58c58facc9), [ControlType](https://codedocs.revrobotics.com/cpp/classrev_1_1spark_1_1_spark_low_level.html#afd05d7bbe5176b6b9ed55f854af116d0)
{% endtab %}
{% endtabs %}

[^1]: All positions are counted in rotations

[^2]: All velocities are in rotations per minute

[^3]: Current is counted in Amps
