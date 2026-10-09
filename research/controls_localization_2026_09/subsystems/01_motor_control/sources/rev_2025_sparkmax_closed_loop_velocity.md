

<!-- Source: https://docs.revrobotics.com/revlib/spark/closed-loop (fetched 2026-09-27, official Markdown export) -->

> For the complete documentation index, see [llms.txt](https://docs.revrobotics.com/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://docs.revrobotics.com/revlib/spark/closed-loop.md).

# Closed Loop Control

## Closed-Loop Control Basics

A Closed-Loop Control System in its most basic form is a process that uses feedback to improve the accuracy of its outputs. Closed-Loop Control Systems, sometimes referred to as Feedback Controllers, are frequently used when maintaining or reaching a steady output is important or if the system may have outside influences that could affect the system's output.

<figure><img src="https://content.gitbook.com/content/0OKYENVWAIgVP2TmkWl3/blobs/Q4L9DuAiEsaWFumzxafo/Closed-Loop-Control.drawio.png" alt=""><figcaption></figcaption></figure>

A simple example using this type of Control is an automatic coffee maker. In its Closed-Loop Control System, the output is hot coffee and the process we are getting feedback on is the heating of the water. If the coffee maker receives feedback that the water is cold, it will start to heat the pot. When the water is almost hot enough to brew the coffee, the control algorithm will continue to heat the water until the correct goal temperature has been reached. Once the water reaches it's goal temperature, or if it gets too hot, the system will stop heating the water and wait until it receives feedback that the heater needs to begin again.&#x20;

<figure><img src="https://content.gitbook.com/content/0OKYENVWAIgVP2TmkWl3/blobs/W9Z4tv9tzeHDGStqHAW1/Coffee-Closed-Loop.drawio.png" alt=""><figcaption></figcaption></figure>

## Closed-Loop Control with SPARK Motor Controllers

Closed-Loop Control is a staple of complex FRC mechanism programming. WPILib offers [several libraries](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/index.html) to allow teams to run PID loops on the roboRIO, but they require manual setup in your team's code, need additional configuration to run at high frequencies, and may require specifically-configured feedback devices for fast responses.

With a PID loop onboard a SPARK Motor Controller, the setup is simple, doesn't clutter your code, and the loop is updated every 1ms, increasing the responsiveness and precision of the controller. Even when using a more complex control algorithm on the roboRIO, it's still recommended to put as much processing on the motor controller as possible. The PID controller onboard the SPARK can also be configured and tuned with the REV Hardware Client, allowing for a much faster tuning process that doesn't rely on your other subsystems.

Configuring SPARK PID with REVLib can be done in a couple of lines and fits right into the configuration of the motor controller.

```java
SparkFlexConfig config = new SparkFlexConfig()
    .closedLoop.pid(0.01, 0, 0.001);
spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
```

Setting a setpoint for the PID is just as easy, whether you want to set a position or velocity or even use a motion profile.

```java
SparkClosedLoopController closedLoopController = spark.getClosedLoopController();
closedLoopController.setSetpoint(10, ControlType.kVelocity); // 10 RPM
```

Both the SPARK MAX and SPARK Flex can operate in several closed-loop control modes, using sensor input to tightly control the motor velocity, position, or current. The internal control loop follows a standard PID algorithm and incorporates [several feedforward terms](/revlib/spark/closed-loop/feed-forward-control.md) to account for known system dynamics. This allows the motor to follow precise and repeatable motions, useful for complex mechanisms.

Additionally, an arbitrary feedforward signal is added to the output of the control loop after *all* calculations are done. The units for this signal can be selected as either *voltage* or *duty cycle.* This feature allows more advanced feedforward calculations to be performed by the controller. This can be useful for systems with more complex dynamics than can be represented by the SPARK feedforward.

***


<!-- Source: https://docs.revrobotics.com/revlib/spark/closed-loop/getting-started-with-pid-tuning (fetched 2026-09-27, official Markdown export) -->

> For the complete documentation index, see [llms.txt](https://docs.revrobotics.com/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://docs.revrobotics.com/revlib/spark/closed-loop/getting-started-with-pid-tuning.md).

# Getting Started with PID Tuning

For a detailed technical and mathematical description of each term and its effect, the [WPILib docs page on PID](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html) is a good resource.

## FRC Usage

In FRC, PID loops are used in many types of mechanisms, from flywheel shooters to vertical arms. These need to be tuned to different constants, depending on the units they use and the physical design of the mechanism, however the process to find these constants is roughly the same.

Most teams find success using controllers tuned primarily with P[^1] and D[^2], using a Feedforward[^3] to account for [steady-state error](#user-content-fn-4)[^4].

## The Constants

### P - Proportional Gain

P, the proportional gain, is the primary factor of the control loop. This is multiplied by the error and that gain is added to the output. This does the heavy lifting of the motion, pushing the motor in the direction it needs to go.

### I - Integral Gain

I, the integral gain, is not often recommended in FRC. It is useful for eliminating steady-state error, or error that the other gains leave behind and cannot address. It accumulates the error over time and multiplies it by the I gain, gradually increasing the power it supplies until that has evened out. If it is needed, it's recommended to use a limited to prevent [I windup](#user-content-fn-5)[^5]. For FRC purposes, Feedforward gains are recommended to eliminate steady-state error instead.

### D - Derivative Gain

The derivative gain, D, is used to tune out oscillation and dampen the motion. It resists motion, decreasing power when the mechanism is moving. A good balance of P and D is needed to make a smooth motion with no oscillation.

## Tuning

Several guides for PID tuning are available, such as [this technical one on the WPILib docs](https://docs.wpilib.org/en/2020/docs/software/advanced-control/introduction/tuning-pid-controller.html). It may be useful to consult multiple, especially those available that reference your specific mechanism.

Any method for PID tuning will start with the same concept, however, regardless of mechanism. Before you can tune your mechanism, you should setup a graph of the setpoint and that measured value, either through the [REV Hardware Client](https://docs.revrobotics.com/rev-hardware-client/ion/telemetry) or a similar utility. This will allow you to analyze each test and properly evaluate the changes to make.

To then tune a basic PID loop, follow the steps below:

1. Set all constants (P, I, D, etc) to 0
2. Ensure the mechanism is safe to actuate. This process will spin the motor, potentially at unexpected speeds and in unexpected directions
3. Check the direction of the motor, and invert it if needed so that positive output is in the desired direction
4. [Setup and tune any relevant feedforwards](/revlib/spark/closed-loop/feed-forward-control.md)
5. Set P to a very small number, relative to the units you are working in
6. Set a target for the motor to move to. Ensure this is within the range of your mechanism.
7. Gradually increase P until you see movement, by small increments
8. Once you see motion, increase P by small increments until it reaches the target at the desired speed
9. If you see oscillation, decrease P or begin to increment D by a small amount. A precisely tuned P gain is better than a D gain, but a D gain may be needed to counteract the dynamics of the system
10. Continue to adjust these parameters until the motion is quick, precise, and repeatable

[^1]: the proportional term

[^2]: the derivative term

[^3]: a physics-based mechanism-specific function to find the voltage needed to maintain a position

[^4]: an error from the target that the controller is not able to account for on its own

[^5]: when the integral gain increases to a point where supplies excessive power when it's not needed or expected, often rapidly


<!-- Source: https://docs.revrobotics.com/revlib/spark/closed-loop/units (fetched 2026-09-27, official Markdown export) -->

> For the complete documentation index, see [llms.txt](https://docs.revrobotics.com/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://docs.revrobotics.com/revlib/spark/closed-loop/units.md).

# Units

## Default Units

| Quantity                        | Default Units                 | Affected by                |
| ------------------------------- | ----------------------------- | -------------------------- |
| Setpoint                        | Rotations                     | Position Conversion Factor |
| Encoder Position                | Rotations                     | Position Conversion Factor |
| Encoder Velocity                | RPM                           | Velocity Conversion Factor |
| Applied Output                  | Duty Cycle                    |                            |
| kP                              | Duty cycle per rotation       | Position Conversion Factor |
| kI                              | Duty cycle per (rotation\*ms) | Position Conversion Factor |
| kD                              | (Duty cycle\*ms) per rotation | Position Conversion Factor |
| kS                              | Volts                         |                            |
| kV                              | Volts per RPM                 | Velocity Conversion Factor |
| kA                              | Volts per RPM/s               | Velocity Conversion Factor |
| kG                              | Volts                         |                            |
| kCos                            | Volts per Rotation            |                            |
| MAXMotion Cruise Velocity       | RPM                           | Velocity Conversion Factor |
| MAXMotion Maximum Acceleration  | RPM/s                         | Velocity Conversion Factor |
| MAXMotion Allowed Profile Error | Rotations                     | Position Conversion Factor |

## Conversion Factors

There are two configurable Conversion Factors on each Encoder type that can be used to account for gear ratios and unit conversions in the motor control logic. These are applied independently and **the velocity factor does not rely on the position factor, so different units can be used for each**.

### Position Conversion Factor

Positions read from the feedback encoder are multiplied by the Position Conversion Factor before being processed by the closed-loop controller.

#### Common Position Conversion Factors

<table><thead><tr><th width="374">Description</th><th>Factor</th></tr></thead><tbody><tr><td>Default (Revolutions)</td><td>1</td></tr><tr><td>Degrees</td><td>360</td></tr><tr><td>Radians</td><td>2π (6.28318530718)</td></tr><tr><td>10:1 Gearbox, Rotations at output</td><td>1/10 (0.1)</td></tr><tr><td>Distance in inches traveled with a 6in diameter wheel</td><td>6π (18.8495559215)</td></tr></tbody></table>

### Velocity Conversion Factor

Velocities read from the feedback encoder are multiplied by the Velocity Conversion Factor before being processed by the closed-loop controller.

The velocity conversion factor is **completely independent** of the position conversion factor, so **both need to be set** to change both units.

All accelerations on the SPARK controllers are in terms of velocity per second, where the velocity is in units specified by the Velocity Conversion Factor.

#### Common Velocity Conversion Factors

<table><thead><tr><th width="374">Description</th><th>Factor</th></tr></thead><tbody><tr><td>Default (RPM)</td><td>1</td></tr><tr><td>Revolutions per Second</td><td>1/60 (0.01666666666)</td></tr><tr><td>Degrees per Minute</td><td>360</td></tr><tr><td>Degrees per Second</td><td>360/60 (6)</td></tr><tr><td>Radians per Minute</td><td>2π (6.28318530718)</td></tr><tr><td>Radians per Second</td><td>2π/60 (0.10471975512)</td></tr></tbody></table>


<!-- Source: https://docs.revrobotics.com/revlib/spark/closed-loop/velocity-control-mode (fetched 2026-09-27, official Markdown export) -->

> For the complete documentation index, see [llms.txt](https://docs.revrobotics.com/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://docs.revrobotics.com/revlib/spark/closed-loop/velocity-control-mode.md).

# Velocity Control Mode

Velocity Control uses the PID controller to run the motor at a set speed in RPM (or configured conversion factor units).

{% hint style="success" %}
Want to control the acceleration of your velocity controller? See [MAXMotion Velocity Control](/revlib/spark/closed-loop/maxmotion-velocity-control.md) for an improved version of Velocity Control with more features and control.
{% endhint %}

It is called in the same way as Position Control:

{% tabs %}
{% tab title="Java" %}

```java
m_controller.setSetpoint(setPoint, ControlType.kVelocity);
```

API Docs: [setSetpoint](https://codedocs.revrobotics.com/java/com/revrobotics/spark/sparkclosedloopcontroller#setSetpoint\(double,com.revrobotics.spark.SparkBase.ControlType\))
{% endtab %}

{% tab title="C++" %}

```cpp
using namespace rev::spark;

m_controller.SetSetpoint(setPoint, SparkBase::ControlType::kVelocity);
```

API Reference: [SetSetpoint](https://codedocs.revrobotics.com/cpp/classrev_1_1spark_1_1_spark_closed_loop_controller#aefb8aa2d8ea8533a8e726f58c58facc9)
{% endtab %}
{% endtabs %}

{% hint style="danger" %}
Velocity Control mode will turn your motor continuously. Be sure your mechanism does not have any hard limits for rotation.
{% endhint %}

{% hint style="info" %}
Velocity Loop constants are often of a very low magnitude, so if your mechanism isn't behaving as expected, try *decreasing* your gains.
{% endhint %}

<figure><img src="https://content.gitbook.com/content/0OKYENVWAIgVP2TmkWl3/blobs/5VPoujaJT8c5XBGpUZGb/Velocity%20Control.png" alt=""><figcaption></figcaption></figure>


<!-- Source: https://docs.revrobotics.com/revlib/spark/closed-loop/maxmotion-velocity-control (fetched 2026-09-27, official Markdown export) -->

> For the complete documentation index, see [llms.txt](https://docs.revrobotics.com/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://docs.revrobotics.com/revlib/spark/closed-loop/maxmotion-velocity-control.md).

# MAXMotion Velocity Control

MAXMotion Velocity Control utilizes the [MAXMotion parameters](/revlib/spark/closed-loop/closed-loop-control-getting-started.md#maxmotion-parameters) to improve upon velocity control. Honoring the maximum acceleration, MAXMotion Velocity Control will speed up your flywheel or rotary mechanism in a controlled way, reducing power draw and increasing consistency.

MAXMotion Velocity Control utilizes an internal velocity closed-loop controller, so transitioning from Velocity Control mode to MAXMotion Velocity Control is as simple as setting a maximum acceleration and changing the setSetpoint call.

It is called as seen below:

{% tabs %}
{% tab title="Java" %}

```java
m_controller.setSetpoint(setPoint, ControlType.kMAXMotionVelocityControl);
```

API Docs: [setSetpoint](https://codedocs.revrobotics.com/java/com/revrobotics/spark/sparkclosedloopcontroller#setSetpoint\(double,com.revrobotics.spark.SparkBase.ControlType\))
{% endtab %}

{% tab title="C++" %}

```cpp
using namespace rev::spark;

m_controller.SetSetpoint(setPoint, SparkBase::ControlType::kMAXMotionVelocityControl);
```

API Reference: [SetSetpoint](https://codedocs.revrobotics.com/cpp/classrev_1_1spark_1_1_spark_closed_loop_controller#aefb8aa2d8ea8533a8e726f58c58facc9)
{% endtab %}
{% endtabs %}

{% hint style="danger" %}
MAXMotion Velocity Control will turn your motor continuously. Be sure your mechanism does not have any hard limits for rotation.
{% endhint %}

## Tips for Smooth Motions

* The Static, Velocity, and Acceleration [feed forward](/revlib/spark/closed-loop/feed-forward-control.md) constants are super helpful in making your motion smooth and consistent. You should be able to get decent performance with only kV/kA and no PID at all
* If your motion seems jittery, try reducing your PID constants, especially P. If the underlying velocity PID outruns the acceleration target, the motion may seem jittery and the velocity will not increase smoothly.
* Make sure your units are correct: maximum velocity is set in RPM by default and maximum acceleration is set in RPM per second by default.
* At low speeds, the acceleration may seem wobbly or inconsistent if the loop has been tuned for higher speeds or vice versa. If both are needed, try tuning separate PIDs and switching between slots when needed. This may be easier than finding those perfect constants that work beautifully across the board.
