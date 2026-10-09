> For the complete documentation index, see [llms.txt](https://docs.revrobotics.com/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://docs.revrobotics.com/brushless/spark-max/operating-modes.md).

# SPARK MAX Operating Modes

## Brushed/Brushless Mode - Motor Type

Brushed and brushless DC motors require different motor control schemes based on the differences in their technology. It is possible to damage the SPARK MAX, the motor, or both if the appropriate motor type isn't configured properly.&#x20;

Brushed or brushless motor types can be configured using the Mode Button, CAN, and USB interfaces.

### Mode Button Configuration

Follow the steps below to switch motor types with the Mode Button. It is recommended that the motor be left disconnected until the correct mode is selected.

{% hint style="info" %}
Use a small screwdriver, straightened paper clip, pen, or other small implement to press the button. Do not use any type of pencil as the pencil lead can break off inside the SPARK MAX.
{% endhint %}

1. Connect the SPARK MAX to the main power, not just USB Power.
2. The Status LED will indicate which motor type is configured by blinking yellow or blue for Brushed Mode or blinking magenta or cyan for Brushless Mode.
3. Press and hold the Mode Button for approximately 3 seconds.
4. After the button has been held for enough time, the Status LED will change and indicate the different motor configuration.
5. Release the mode button.

{% hint style="info" %}
Please see the [Status LED Patterns](/brushless/spark-max/status-led.md) guide for information on how to identify the Motor Type configuration by the color of the Status LED!
{% endhint %}

### USB Configuration

Follow the steps below to switch motor types with the USB and the REV Hardware Client application. Be sure to [download and install the REV Hardware Client](/rev-hardware-client/gs/install.md) application before continuing.

1. Connect the SPARK MAX to your computer using a USB-C cable.
2. Open the REV Hardware Client and verify that the application is connected to your SPARK MAX.
3. On the **Basic** tab, select the appropriate motor type under the **Select Motor Type** menu.
4. Click **Burn Flash** and confirm the change.

### CAN Configuration

Please see the [API Information](/revlib/install.md) for information on how to configure the SPARK MAX using the CAN interface.&#x20;

## Brake/Coast Mode - Idle Behavior

When the SPARK MAX is receiving a neutral command the idle behavior of the motor can be handled in two different ways: **Braking** or **Coasting**.&#x20;

When in **Brake Mode**, the SPARK MAX will effectively short all motor wires together. This quickly dissipates any electrical energy within the motor and brings it to a quick stop.

When in **Coast Mode**, the SPARK MAX will effectively disconnect all motor wires. This allows the motor to spin down at its own rate.

The Idle Mode can be configured using the Mode Button, CAN, and USB interfaces.

### Mode Button Configuration&#x20;

Follow the steps below to switch the Idle Mode between Brake and Coast with the Mode Button.

{% hint style="info" %}
Use a small screwdriver, straightened paper clip, pen, or other small implement to press the button. Do not use any type of pencil as the pencil lead can break off inside the SPARK MAX.
{% endhint %}

1. Connect the SPARK MAX to main power, not just USB Power.
2. The Status LED will indicate which Idle Mode is currently configured by blinking blue or cyan for Brake and yellow or magenta for Coast depending on the motor type.
3. Press and release the Mode Button
4. You should see the Status LED change to indicate the selected Idle Mode.

{% hint style="info" %}
Please see the [Status LED Patterns](/brushless/spark-max/status-led.md) guide for information on how to identify the Idle Behavior configuration by the color of the Status LED!
{% endhint %}

### USB Configuration

Follow the steps below to switch the Idle Mode between Brake and Coast with the USB and the REV Hardware Client application. Be sure to [download and install the REV Hardware Client](/rev-hardware-client/home/rev-hardware-client-overview.md) application before continuing.

1. Connect the SPARK MAX to your computer using a USB-C cable.
2. Open the REV Hardware Client application and verify that the application is connected to your SPARK MAX.
3. On the **Basic** tab, select the desired mode with the **Idle Mode** switch.
4. Click **Burn Flash** and confirm the change.

### CAN Configuration

Please see the [API Information](/revlib/install.md) for information on how to configure the SPARK MAX using the CAN interface.&#x20;
