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
