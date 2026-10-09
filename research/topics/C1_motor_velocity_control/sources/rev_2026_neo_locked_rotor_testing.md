<!-- Source: https://docs.revrobotics.com/brushless/neo/locked-rotor-testing (fetched 2026-09-27, official Markdown export) -->

> For the complete documentation index, see [llms.txt](https://docs.revrobotics.com/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://docs.revrobotics.com/brushless/neo/locked-rotor-testing.md).

# Locked Rotor Testing

## What is Locked Rotor Testing?

When stalled, both brushed and brushless motors draw a lot of current and generate a large amount of heat. This heat can permanently damage a motor in a fraction of a second if not managed correctly. Locked-rotor stall data can be a useful tool when designing robot mechanisms that need to hold a mechanical load for a particular amount of time. It is useful to know the approximate time to failure depending on the applied load so that mechanisms can be successfully designed around these limitations.

Brushless motors, like the NEO family of Brushless Motors, offer higher efficiency and higher power density than brushed motors. However, they require a more complex control scheme to operate due to the fundamentally different motor technology. Sensors are built into each version of NEO Brushless Motors to enable proper operation with a SPARK MAX or SPARK Flex Motor Controller, and are required for proper operation.

### Constant Current vs. Constant Voltage

Most Brushed DC motors used in FRC have locked-rotor stall data available showing Torque vs. Time at a particular constant applied voltage. This data shows how long a particular voltage can be applied before the motor fails. Because of differences in motor construction, especially winding resistance and torque constants, voltage data cannot be used alone to compare the survivability of different motors.

While applied voltage is straightforward and sometimes the only variable that you can control (e.g., with the original SPARK or Victor SPX), it isn’t as useful when you need to maintain a constant torque to hold a constant load. Over time, when a constant voltage is applied to a brushless motor, the motor's current and torque output will change as the motor's windings heat up.&#x20;

Holding a constant torque requires the current applied to the motor windings (or motor phases) to remain constant, despite these changes due to heat. Therefore, it is useful to know the time to failure of a motor at different currents rather than different voltages.

### Locked-rotor Testing with SPARK MAX Smart Current Limit

The SPARK MAX Motor Controller includes a Smart Current Limit feature that can adjust the applied output to the motor to maintain a constant phase current. Below you will find data at various current levels being maintained in the NEO 550 Brushless Motor.

Please take the following into consideration when interpreting the data below:&#x20;

* Average motor phase current (or winding current) is different than the average input current to the motor controller.
* Average Input Current = Average Phase Current x Duty Cycle
* Motor torque is proportional to phase current, not the input current. Therefore, it is important to control the phase current and not the input current.
* The torque values in the graphs are approximate and are measured using a digital torque wrench which includes an error up to \~5%. **The intent is to show the point of failure and not the torque/current relationship.** Please see each motor's documentation page for its torque and current specifications.
* **At higher current limits, the time-to-failure depends on many different factors. It is best practice to design mechanisms with a considerable safety margin.**
* Locked-rotor test setup:
  * The motor is mounted to an aluminum motor bracket at the face plate with the output shaft locked in place through a digital torque wrench.
  * SPARK MAX is controlling the motor and its Smart Current Limit is configured to the desired limit for the test. It is then commanded to go full-power while letting the Smart Current Limit adjust the applied output duty cycle as necessary to keep the phase current at the limit.
  * Temperature measurements are read from the motor's internal temperature sensor. Temperature measurements lag behind the actual motor coil temperature due to the physical location on the motor. This data can also be used to approximate where limiting can be useful in user code based on the temperature measured.
  * Power is provided by the following:
    * NEO V1 & NEO V1.1 Locked Rotor Testing - 12V nominal, 18Ah, lead-acid battery through an FRC Power Distribution Panel with a 40A breaker. The breaker did not trip during any of the tests. Bus voltage is graphed to show the drop in battery voltage throughout the tests.
    * NEO 550 Locked Rotor Testing - 200A 12V DC power supply.

## Locked Rotor Testing Data

{% tabs %}
{% tab title="NEO V1 & NEO V1.1" %}

<figure><img src="https://4148826207-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2Fe0CWwhMSoCEH7NLVoLhF%2Fuploads%2FJ4Rwiz29Uk3tL5pglzj4%2FNEO%20Stall%20Torque%20with%20SPARK%20MAX%20Smart%20Current%20Limit.png?alt=media&amp;token=08f4652d-c177-49ff-bbfc-67e9f385227a" alt=""><figcaption></figcaption></figure>

{% hint style="danger" %}
The intent of this graph and the graphs below is to show an approximate time-to-failure at various current limits. Various factors can affect these times and, as always, mechanisms should be designed with a considerable margin.&#x20;

These stall times are not guaranteed.
{% endhint %}

#### Download the raw data in CSV format: [NEO Locked-rotor Testing Raw Data](https://www.revrobotics.com/content/docs/NEOLocked-rotorTestingRawData.zip)

<figure><img src="https://4148826207-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2Fe0CWwhMSoCEH7NLVoLhF%2Fuploads%2FqLylDbkH3irGOyPYxdvz%2FNEO%20Stall%20Data%20at%2040A%20Limit.png?alt=media&amp;token=965a714a-60f2-4a62-844a-1b781db2249a" alt=""><figcaption></figcaption></figure>

<figure><img src="https://4148826207-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2Fe0CWwhMSoCEH7NLVoLhF%2Fuploads%2FRZkpFy1JfQbaXq7w9z4G%2FNEO%20Stall%20Data%20at%2050A%20Limit.png?alt=media&amp;token=3f475cae-bf69-4e9d-b41d-a02591b3d694" alt=""><figcaption></figcaption></figure>

<figure><img src="https://4148826207-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2Fe0CWwhMSoCEH7NLVoLhF%2Fuploads%2FSOeoCCVOODfSDPPeEAFk%2FNEO%20Stall%20Data%20at%2060A%20Limit.png?alt=media&amp;token=7c90b6f8-b881-483f-8f82-26788d253b41" alt=""><figcaption></figcaption></figure>

<figure><img src="https://4148826207-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2Fe0CWwhMSoCEH7NLVoLhF%2Fuploads%2FFiFc0S1U3LXgCQqJbTT6%2FNEO%20Stall%20Data%20at%2080A%20Limit.png?alt=media&amp;token=b4c13332-a798-4764-abd1-c30323eb1260" alt=""><figcaption></figcaption></figure>
{% endtab %}

{% tab title="NEO 550" %}

<figure><img src="https://4148826207-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2Fe0CWwhMSoCEH7NLVoLhF%2Fuploads%2FOkak5vtCPWa1NCUfsPyD%2FNEO%20550%20Stall%20Torque%20with%20SPARK%20MAX%20Smart%20Current%20Limit.png?alt=media&amp;token=7cda3a55-06c4-4b3c-b418-3fe3c24d8c16" alt=""><figcaption></figcaption></figure>

{% hint style="danger" %}
The intent of this graph and the graphs below is to show an approximate time-to-failure at various current limits. Various factors can affect these times and, as always, mechanisms should be designed with a considerable margin.&#x20;

These stall times are not guaranteed.
{% endhint %}

#### Time to Failure Summary

* 20A Limit - Motor survived full 220s test.
* 40A Limit - Motor failure at approximately 27s.
* 60A Limit - Motor failure at approximately 5.5s
* 80A Limit\* - Motor failure at approximately 2.0s

\*80A is the default Smart Current Limit in the SPARK MAX. It is highly recommended to adjust the Smart Current Limit when driving the NEO 550.

<figure><img src="https://4148826207-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2Fe0CWwhMSoCEH7NLVoLhF%2Fuploads%2FzfRnYIoVZ708GDZVg1fI%2FNEO%20550%20Stall%20Data%20at%2020A%20Limit.png?alt=media&amp;token=07da31e3-1afb-4d2d-974f-29a22abb801f" alt=""><figcaption></figcaption></figure>

<figure><img src="https://4148826207-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2Fe0CWwhMSoCEH7NLVoLhF%2Fuploads%2F7mfmnEnN9DC5gH6TyBTs%2FNEO%20550%20Stall%20Data%20at%2040A%20Limit.png?alt=media&amp;token=64dbd323-08de-4a45-a85d-b57aad5502ca" alt=""><figcaption></figcaption></figure>

<figure><img src="https://4148826207-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2Fe0CWwhMSoCEH7NLVoLhF%2Fuploads%2F0Dico9sYVwh5oZ8hZq6i%2FNEO%20550%20Stall%20Data%20at%2060A%20Limit.png?alt=media&amp;token=0395a2b1-0a61-497e-be7d-ff387f926b47" alt=""><figcaption></figcaption></figure>

<figure><img src="https://4148826207-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2Fe0CWwhMSoCEH7NLVoLhF%2Fuploads%2FlYhjNev6F2UuFeWNokmE%2FNEO%20550%20Stall%20Data%20at%2080A%20Limit.png?alt=media&amp;token=10ffc422-51a4-4e28-ba5c-fda64906ca9e" alt=""><figcaption></figcaption></figure>
{% endtab %}
{% endtabs %}
