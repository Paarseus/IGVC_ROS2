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
