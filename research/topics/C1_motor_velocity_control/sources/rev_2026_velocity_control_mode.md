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
