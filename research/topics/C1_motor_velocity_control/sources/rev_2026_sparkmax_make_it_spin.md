<!-- Source: https://docs.revrobotics.com/brushless/spark-max/gs/make-it-spin (fetched 2026-09-27, official Markdown export) -->

> For the complete documentation index, see [llms.txt](https://docs.revrobotics.com/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://docs.revrobotics.com/brushless/spark-max/gs/make-it-spin.md).

# Make it Spin!

{% hint style="success" %}
[Make it Spin!](https://docs.revrobotics.com/rev-hardware-client-2/home/run-motor) For REV Hardware Client 2 is available.
{% endhint %}

## Power On

Now that the device is wired, and the connections carefully checked, power on the robot. You should see the SPARK MAX slowly blinking its for a new device the color will be Magenta. If the LED is dark, or you see a different blink pattern, refer to the [Status LED](/brushless/spark-max/status-led.md) guide for troubleshooting.&#x20;

{% hint style="info" %}
If you are using a brushed motor, you may see a sensor error. This is expected until you configure the device to accept a brushed motor in the following steps.
{% endhint %}

## Connect to the SPARK MAX

Plug in the USB cable and start the REV Hardware Client. Select the SPARK MAX from the Connected Hardware.

![](https://content.gitbook.com/content/e0CWwhMSoCEH7NLVoLhF/blobs/VUKgytfFIHY6EbLxjDMo/SPARK%20MAX%20-%20Single%20Device.svg)

{% hint style="info" %}
If you can not see the SPARK MAX, make sure that the SPARK MAX is not being used by another application. Then unplug the SPARK MAX from the computer and plug it back in.
{% endhint %}

## Basic Setup and Configuration

Before any parameters can be changed, you **must** first assign a unique CAN ID to the device. This can be any number between 1 and 63. After setting a unique CAN ID, the user interface will refresh and allow you to change other parameters.

![](https://4148826207-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2Fe0CWwhMSoCEH7NLVoLhF%2Fuploads%2FDIMAT7gV1sIRtcvkhsB1%2FBasic%20Setup%20and%20Configuration.png?alt=media\&token=2a516ad0-d19b-4489-b976-20c9ee0f33c4)

{% hint style="info" %}
Eventually you may set up a CAN network on your test bench or robot. Be sure each device on the network has a unique CAN ID. It is helpful to label each device with its ID number to aid in troubleshooting.
{% endhint %}

### Set the Motor Type

If you are using a NEO or NEO 550, verify that the motor type is set to **REV NEO Brushless**, Sensor Type is **Hall Effect**, and the LED is blinking Magenta or Cyan.

![](https://4148826207-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2Fe0CWwhMSoCEH7NLVoLhF%2Fuploads%2F3yxAiBLt5SVzECkrewgh%2FSet%20Motor%20Typ.png?alt=media\&token=185071fc-ba88-4e2d-bb05-67baacfe0f63)

{% hint style="info" %}
If you see a *Sensor Fault* blink code, make sure the encoder cable is plugged in completely.
{% endhint %}

If you are running brushed motor, set the motor type to **Brushed** and the sensor type will change to **Quadrature**, and verify that the LED is blinking Yellow or Blue.

![](https://4148826207-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2Fe0CWwhMSoCEH7NLVoLhF%2Fuploads%2FRFWTPHAbmdGTvOJCv2VC%2FBrushed%20Motor%20and%20Sensor.png?alt=media\&token=b6f30713-1123-4736-99ff-9f874dd82d36)

### Limiting Current

There are two ways to protect your robot’s motors from electrical damage in high-current situations: Circuit Breakers and the SPARK MAX’s Smart Current Limit Setting. To protect your motors from currents that are too high, it is a best practice to limit your current both with the SPARK MAX’s Smart Current Limit **and** an appropriately rated circuit breaker.

Circuit breakers, while an extremely important part of a robot's wiring and safety, are only designed to trip at a specific temperature, after a set amount of time, to protect the electrical system from fire or other electrical hazards. Due to this, we recommend setting a Smart Current Limit to protect your motors from damage due to high currents.

The SPARK MAX Motor Controller includes a Smart Current Limit feature that can adjust the applied output to the motor to maintain a constant phase current.&#x20;

Out of the box, the SPARK MAX's Smart Current Limit default setting is 80A for any motor that you use. We recommend utilizing our locked-rotor testing data or the table below to decide what to set your Smart Current Limit to for your robot: Locked-Rotor Testing for the [NEO (REV-21-1650) ](https://www.revrobotics.com/neo-brushless-motor-locked-rotor-testing/)and [NEO 550 (REV-21-1651)](https://www.revrobotics.com/neo-550-brushless-motor-locked-rotor-testing/).

{% hint style="danger" %}
Remember that some settings, like Smart Current Limit, must be burned to flash via code or the Hardware Client in order to be retained through a power cycle of the SPARK MAX.
{% endhint %}

#### Suggested Current Limits

Your ideal current limit may vary based on your specific application, but these values can be used as a starting point to reduce the chance of an overload on your motor as you begin tuning your specific mechanism's Smart Current Limit.

| Motor Type                                                        | Current Limit Range |
| ----------------------------------------------------------------- | ------------------- |
| NEO ([REV-21-1650](https://www.revrobotics.com/rev-21-1650/))     | 40A - 60A           |
| NEO 550 ([REV-21-1651](https://www.revrobotics.com/rev-21-1651/)) | 20A - 40A           |

{% hint style="warning" %}
Warning: Setting current limits outside of the suggested ranges listed above may cause unintended overload and severe damage to components that are not covered by warranty.
{% endhint %}

![](https://4148826207-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2Fe0CWwhMSoCEH7NLVoLhF%2Fuploads%2Fj3DRBFwMNbs7HYfRRKQ1%2FSmart%20Current%20Limit.png?alt=media\&token=77310993-e7db-4e91-b3a2-e7ef73fc87dc)

## Save the Settings

The settings must be saved for the SPARK MAX to remember its new configuration through a power cycle. To do this, press the *Burn Flash* button at the bottom of the page. It will take a few seconds to save, indicated by the loading symbol on the button.

{% hint style="warning" %}
As of REV Hardware Client version 1.7.0, "Burn Flash" has been renamed to "Persist Perimeters"!
{% endhint %}

![](https://4148826207-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2Fe0CWwhMSoCEH7NLVoLhF%2Fuploads%2FwYpIgzzmHMYWJUTFlUcN%2FPersist%20Parameters.png?alt=media\&token=91fb192f-c5fc-411b-aff6-58db208754b3)

Any settings saved this way will be remembered when the device is powered back on. You can always restore the factory defaults if you need to reset the device.

## Spin the Motor

{% hint style="danger" %}
Before running any motor, make sure all components are in a safe state, that the motor is secured, and that anyone nearby is aware. FRC motors are very powerful and can quickly cause damage to people and property.&#x20;
{% endhint %}

{% hint style="info" %}
Keep the CAN cable disconnected throughout the test. For safety reasons, the REV Hardware Client will not run the motor if the roboRIO is connected. If the roboRIO was connected, power cycle the SPARK MAX.
{% endhint %}

To spin the motor, go to the Run tab, keep all of the default settings and press *Run* *Motor.* The *setpoint* is 0 by default, meaning that the motor is being commanded to **idle** (0% power). When you press *Run* you should see the LED go from slow blinking to solid, indicating that the motor is idling.

![](https://content.gitbook.com/content/e0CWwhMSoCEH7NLVoLhF/blobs/gmXngz13M2El0XjHHGh4/SPARK%20MAX%20-%20Run%20Single%20Device.svg)

**Slowly** ramp the setpoint slider up. The motor should start to spin and you should see a green blink pattern proportional to the speed you have set to the motor. Slowly ramp the slider down. The motor should spin in reverse, and you should see a red blink pattern proportional to the speed you have set to the motor.

If you are unable to spin the motor, visit our [troubleshooting guide](/brushless/spark-max/troubleshooting.md).
