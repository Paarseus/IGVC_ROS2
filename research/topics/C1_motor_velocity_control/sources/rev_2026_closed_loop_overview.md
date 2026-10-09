

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

