

<!-- Source: https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control (fetched 2026-09-27, official Markdown export) -->

> For the complete documentation index, see [llms.txt](https://docs.revrobotics.com/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control.md).

# Feed Forward Control

Closed loop PID control and MAXMotion motion profiled control are excellent tools for precisely and reactively controlling mechanisms on your robot, but the effectiveness of these tools can be increased further with the introduction of Feed Forward terms. A feed forward (or feedforward) controller is an additional calculation that helps factor system dynamics like gravity and resistance into your closed loop movements, which can be especially helpful on heavy systems.

[WPILib offers classes for Feedforward control](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html) that behave similarly to the SPARK motor controllers internal calculations and also have an [explanation of the math behind DC motor feedforward control.](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html) The SPARK feed forward system is designed to drop-in to many of the use cases of these utilities, so much of the information on them is transferable, though you may need to watch your units.

The SPARK feed forward system has the added benefits of directly integrating with MAXMotion, being able to use high feedback frequencies without increased CAN bus traffic or additional configuration, being easy to setup and use, and conserving processing resources on your robot controller.

## Feed Forward Constant Quick Reference

For more information on these terms, see their descriptions below

| Term      | Units                | Usage Notes                                                                                                        |
| --------- | -------------------- | ------------------------------------------------------------------------------------------------------------------ |
| kS        | Volts                |                                                                                                                    |
| kV        | Volts per velocity   | Volts per motor RPM by default                                                                                     |
| kA        | Volts per velocity/s | Volts per motor RPM/s by default                                                                                   |
| kG        | Volts                | Elevator/linear mechanism gravity feedforward                                                                      |
| kCos      | Volts                | <p>Arm/rotary mechanism gravity feedforward.</p><p></p><p>Feedback sensor must be configured to 0 = horizontal</p> |
| kCosRatio | Ratio                | Converts feedback sensor readings to mechanism rotations                                                           |

## Feed Forward Constant Terms

The SPARK Feed Forward system includes 5 terms and one additional constant, each of which apply to some control modes but not others. The compatibility of these is listed in the chart below:

<table><thead><tr><th>Term</th><th data-type="checkbox">MAXMotion Position Control Mode</th><th data-type="checkbox">MAXMotion Velocity Control Mode</th><th data-type="checkbox">Position Control Mode</th><th data-type="checkbox">Velocity Control Mode</th></tr></thead><tbody><tr><td>kS</td><td>true</td><td>true</td><td>true</td><td>true</td></tr><tr><td>kV</td><td>true</td><td>true</td><td>false</td><td>true</td></tr><tr><td>kA</td><td>true</td><td>true</td><td>false</td><td>false</td></tr><tr><td>kG*</td><td>true</td><td>false</td><td>true</td><td>false</td></tr><tr><td>kCos*</td><td>true</td><td>false</td><td>true</td><td>false</td></tr><tr><td>kCosRatio</td><td>true</td><td>false</td><td>true</td><td>false</td></tr></tbody></table>

{% hint style="warning" %}
\*kG and kCos are both gravity feedforwards, and only one can be used at a time. Many calculators refer to both as "kG", but arms will need to use kCos instead.
{% endhint %}

Each term can be set per closed loop slot in the config, as seen below.

{% tabs %}
{% tab title="Java" %}

```java
SparkFlexConfig config = new SparkFlexConfig();

// Set PID gains
config
    .closedLoop
        .pid(0, 0, 0) // slot 0
        .pid(0, 0, 0, ClosedLoopSlot.kSlot1) // slot 1
        .feedForward
            .kS(s) // slot 0 by default
            .kV(v, ClosedLoopSlot.kSlot0) // slot 0 explicitly
            .kA(a)
            .kG(g) // Only use one of kG and kCos
            .kCos(g)
            .kCosRatio(cosRatio)
            
            .sva(s, v, a, ClosedLoopSlot.kSlot1); // slot 1
```

API Docs: [ClosedLoopConfig](https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/closedloopconfig)
{% endtab %}

{% tab title="C++" %}

```cpp
using namespace rev::spark;

SparkFlexConfig config;

// Set PID gains
config
    .closedLoop
        .pid(0, 0, 0) // slot 0
        .pid(0, 0, 0, ClosedLoopSlot::kSlot1) // slot 1
        .feedForward
            .kS(s) // slot 0 by default
            .kV(v, ClosedLoopSlot::kSlot0) // slot 0 explicitly
            .kA(a)
            .kG(g) // Only use one of kG and kCos
            .kCos(g)
            .kCosRatio(cosRatio)
            
            .sva(s, v, a, ClosedLoopSlot::kSlot1); // slot 1
```

API Docs: [ClosedLoopConfig](https://codedocs.revrobotics.com/cpp/classrev_1_1spark_1_1_closed_loop_config.html)
{% endtab %}
{% endtabs %}

### kS - Static Gain

The Static Gain is used to counteract any resistance in your motor or mechanism, and is applied in the direction of desired velocity.

To find this value, find the smallest output that causes the mechanism to move slightly, then decrease it slightly so that it doesn't move on it's own, but has no resistance in that direction. See [kG](#kg-static-elevator-gravity-gain) for how to experimentally find this value for an elevator or [kCos](#kcos-cosine-arm-gravity-gain) for the equivalent on an arm. Note that kS is input in Volts.

This should allow the motor/mechanism to move as soon as any other output is applied, eliminating any "dead zone" of output because of resistance. This can be measured using SysID.

### kV - Velocity Gain

The Velocity Gain is used to help your motor and mechanism maintain the desired velocity, and is multiplied by the velocity setpoint. The units are Volts per velocity as measured by the feedback sensor, after the conversion factor. By default, the units are Volts per RPM, prior to any gear ratio.

Many calculators will estimate this in terms of the mechanism's movement, so be sure to account for gear ratios or velocity unit conversions. This can be estimated with a tool like [ReCalc](https://www.reca.lc/) (note the units) or measured with SysID.

### kA - Acceleration Gain

The Acceleration Gain is used to accelerate your motor to the desired acceleration, and is multiplied by the acceleration setpoint. The units are Volts per velocity unit per second, with the same caveats on the velocity units as the velocity gain. By default, the units are Volts per RPM per second, prior to any gear ratio.

This can be estimated with a tool like [ReCalc](https://www.reca.lc/) (note the units) or measured with SysID.

### kG - Static (Elevator) Gravity Gain

The Static Gravity Gain, for elevators and mass moving straight up and down, is simply added to the output and serves to hold the mechanism's position against gravity. The units are Volts.

<details>

<summary>Manually finding kG and kS for an elevator</summary>

1. Ensure your elevator is free to move up and down and note any physical limits
2. Set up a Voltage output to the motors driving the elevator, via REVLib or REV Hardware Client
3. Increase the output slowly until the elevator begins to rise
4. Decrease the output slowly until the elevator stops and stays where it is
5. Increase the output slightly until any more makes the elevator rise
6. Note the Voltage output as V1
7. Decrease the output slowly until the elevator begins to fall
8. Increase the output slowly until any less makes the elevator fall
9. Note the Voltage output as V2

You now have two Voltage values, V1 and V2, that define the edges of the region of output where the elevator holds its position. Any more than V1 and the elevator will rise, and any less than V2 and the elevator will fall.

Use the equations below to find kS and kG:

* kS = (V1 - V2) / 2
* kG = V2 + kS

kG is right in the middle of this region, where it will keep the elevator right where it is.

kS is the distance to the edges of this region, where kG + kS is the maximum output without upward movement and kG - kS is the minimum output without downward movement. This allows the PID controller to overcome resistance in either direction.

</details>

kG can be can be estimated with a tool like [ReCalc](https://www.reca.lc/), or it can be measured with SysID.&#x20;

As kG and kCos are both different types of gravity feedforward gains, they shouldn't be used together. If your mechanism is an elevator, use kG. If your mechanism is an arm, use kCos.

### kCos - Cosine (Arm) Gravity Gain

The Cosine Gravity Gain, for arms and mechanisms that fight gravity in a rotary way, is the most complicated but also the most useful of the feed forward gains. It is multiplied by the cosine of the absolute position of your mechanism, which means it pushes the most when the mechanism is horizontal and the least when it's vertical.

{% hint style="success" %}
To use this gain properly, the motor on your arm needs to be configured such that when the arm (the radius to the center of mass of the arm) is perfectly horizontal the selected sensor's position is zero.
{% endhint %}

This can be easily accomplished by setting up an absolute encoder or limit switch to reset the position of the arm and then using an initialization or homing sequence to zero the position correctly. Once the zero position is set, make sure to also set up the kCosRatio constant to ensure the calculations are done correctly.

The Units are Volts and kCosRatio needs to be set to convert position to absolute mechanism rotations.

This gain can be estimated with a tool like [ReCalc](https://www.reca.lc/) or measured with SysID, but is referred to as kG in these systems and may need unit conversions.

<details>

<summary>Manually finding kCos and kS for an arm</summary>

1. Ensure your arm is free to move up and down and note any physical limits
2. Set up a Voltage output to the motors driving the arm, via REVLib or REV Hardware Client
3. Set a current limit to avoid damaging the motors
4. Hold the arm horizontally
5. Increase the output slowly until the arm begins to rise
6. Decrease the output slowly until the arm stops and stays perfectly horizontal under its own power
7. Don't let the arm hang under its own power horizontally longer than it needs to, or you risk damaging the motor as it heats up
8. Increase the output slightly until any more makes the arm rise
9. Note the Voltage output as V1
10. Pause, disable, and power off the motor for a few minutes
11. Hold the arm horizontally again, set the voltage at or just below V1
12. Decrease the output slowly until the arm begins to fall
13. Increase the output slowly until any less makes the arm fall but the arm stays perfectly horizontal under its own power
14. Note the Voltage output as V2

You now have two Voltage values, V1 and V2, that define the edges of the region of output where the arm holds its position horizontally. Any more than V1 and the arm will rise, and any less than V2 and the arm will fall.

Use the equations below to find kS and kG:

* kS = (V1 - V2) / 2
* kG = V2 + kS

kG is right in the middle of this region, where it will keep the arm perfectly horizontal.

kS is the distance to the edges of this region, where kG + kS is the maximum output without upward movement and kG - kS is the minimum output without downward movement. This allows the PID controller to overcome resistance in either direction.

</details>

As kCos and kG are both different types of gravity feedforward gains, they shouldn't be used together. If your mechanism is an arm, use kCos. If your mechanism is an elevator, use kG.

### kCosRatio - Ratio Constant for use with kCos

Once your arm is zeroed correctly as explained above in the kCos section, the kCosRatio also needs to be configured so that your mechanism's absolute position can be calculated correctly. This ratio should convert from the units of your setpoint (selected feedback sensor's conversion factor) to **absolute rotations** of your mechanism, and is multiplied by the selected sensor's read position (in units set by your position conversion factor).

This must convert your motor's selected feedback sensor's position into Rotations of the mechanism for the calculation to work.

If your conversion factor is 1 (default), this should simply be any gear reduction between your motor and the actual motion of the arm. If your conversion factor is set, it'll need to be factored into this ratio to properly determine the absolute position of your arm.

## Arbitrary Feed Forward

For more complex feedforward models, there is also a means of applying an arbitrary voltage which can be calculated in your team code and passed to the API.

{% hint style="success" %}
WPILib offers [several basic feed forward calculation classes](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html#the-wpilib-feedforward-classes) that work great with arbFF
{% endhint %}

It can be applied with the setpoint as seen below:

{% tabs %}
{% tab title="Java" %}

```java
// Set the setpoint of the controller in raw position mode, with a feedforward
m_controller.setSetpoint(
    setPoint, 
    ControlType.kPosition,
    0, // setpoint position
    arbFeedForward
);
```

API Docs: [SparkClosedLoopController](https://codedocs.revrobotics.com/java/com/revrobotics/spark/sparkclosedloopcontroller), [setSetpoint](https://codedocs.revrobotics.com/java/com/revrobotics/spark/sparkclosedloopcontroller#setSetpoint\(double,com.revrobotics.spark.SparkBase.ControlType\))
{% endtab %}

{% tab title="C++" %}

```cpp
using namespace rev::spark;

// Set the setpoint of the controller in raw position mode, with a feedforward
m_controller.SetSetpoint(
    setPoint, 
    SparkBase::ControlType::kPosition,
    0, // setpoint position
    feedForward
);
```

API Docs: [SparkClosedLoopController](https://codedocs.revrobotics.com/cpp/classrev_1_1spark_1_1_spark_closed_loop_controller), [SetSetpoint](https://codedocs.revrobotics.com/cpp/classrev_1_1spark_1_1_spark_closed_loop_controller.html#aefb8aa2d8ea8533a8e726f58c58facc9)
{% endtab %}
{% endtabs %}
