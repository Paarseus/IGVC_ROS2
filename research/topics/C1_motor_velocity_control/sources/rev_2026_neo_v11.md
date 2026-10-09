<!-- Source: https://docs.revrobotics.com/brushless/neo/v1.1 (fetched 2026-09-27, official Markdown export) -->

> For the complete documentation index, see [llms.txt](https://docs.revrobotics.com/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://docs.revrobotics.com/brushless/neo/v1.1.md).

# NEO V1.1

## NEO V1.1 Overview

The REV [NEO Brushless Motor V1.1 (REV-21-1650)](https://www.revrobotics.com/rev-21-1650/) is the initial update on the first brushless motor designed to meet the unique demands of the *FIRST* Robotics Competition community. NEO V1.1 offers an incredible power density due to its compact size and reduced weight, and it's designed to be a drop-in replacement for CIM-style motors, as well as an easy install with many mounting options. \
\
The built-in hall-effect encoder guarantees low-speed torque performance while enabling smart control without additional hardware. NEO V1.1 has been optimized to work with the [SPARK MAX Motor Controller (REV-21-2158)](https://www.revrobotics.com/rev-11-2158/) to deliver incredible performance and feedback.

<div><figure><img src="https://4148826207-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2Fe0CWwhMSoCEH7NLVoLhF%2Fuploads%2F6wpxQheRLladQGXC4QlC%2FREV-21-1650-NEO1.1-Hero-FINAL.png?alt=media&amp;token=d090a1bc-038d-4484-902e-532fb2e03b7f" alt=""><figcaption></figcaption></figure> <figure><img src="https://4148826207-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2Fe0CWwhMSoCEH7NLVoLhF%2Fuploads%2FRpSNUwcvD7uhsTbnU5EG%2FREV-21-1650-NEO1.1-Side-FINAL.png?alt=media&amp;token=c90a5e6c-761e-4cc1-85bd-88f89e9d51bc" alt=""><figcaption></figcaption></figure></div>

### Features

* Drop-in replacement for CIM-style motors
* Shielded out-runner construction
* Front and rear ball bearings
* High-temperature neodymium magnets
* High-flex silicone motor wires
* Integrated motor sensor
  * 3-phase hall sensors
  * Motor temperature sensor

#### New to NEO V1.1

* A tapped #10-32 hole on the end of the shaft, allowing teams to retain pinions on the shaft without using external retaining rings&#x20;
* A tapped #10-32 hole on the back housing of the motor, making it no longer necessary to remove the motor housing to press pinions&#x20;
* Additional holes on the front face of the motor for added mounting flexibility

## Wiring Connections

Connecting the NEO V1.1 Brushless motor is fairly straightforward. Follow the guide at[ Wiring the Spark Max](/brushless/spark-max/gs/wiring.md), and don't forget to connect your sensor wire; the motor will not spin without it!

{% hint style="danger" %}
CAUTION: Improperly wiring the connectors can cause severe motor damage and is not covered by the warranty. <mark style="color:red;">DO NOT</mark> connect the motor directly to the battery.&#x20;
{% endhint %}

## Motor Specifications

| Parameter                      | Value and Units    |
| ------------------------------ | ------------------ |
| Nominal Operating Voltage      | 12 V               |
| Motor Kv                       | 473 Kv             |
| Free Speed\`                   | 5676 RPM           |
| Free Running Current           | 1.8 A              |
| Stall Current                  | 105 A              |
| Stall Torque                   | 2.6 Nm             |
| Peak Output Power              | 406 W              |
| Typical Output Power at 40 A   | 380 W              |
| Hall-Sensor Encoder Resolution | 42 counts per rev. |

### Mechanical Specifications

<table><thead><tr><th width="299">Parameter</th><th>Value and Units</th></tr></thead><tbody><tr><td>Output Shaft Diameter</td><td>8mm (keyed)</td></tr><tr><td>Output Shaft Length</td><td>35mm (1.38in)</td></tr><tr><td>Output Pilot</td><td>19.05mm (0.75in)</td></tr><tr><td>Body Length</td><td>58.25mm (2.3in)</td></tr><tr><td>Body Diameter</td><td>60mm (2.36in)</td></tr><tr><td>Weigh<strong>t</strong></td><td>0.938 lbs (0.425 kg)</td></tr><tr><td>Phase Wire Length</td><td>5.91in (150mm)</td></tr><tr><td>Phase Wire Gauge</td><td>12AWG</td></tr><tr><td>Sensor Cable Length</td><td>11.81in (300mm)</td></tr><tr><td>Sensor Cable Gauge</td><td>24AWG</td></tr></tbody></table>

## NEO V1.1 Motor Curve

<figure><img src="https://4148826207-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2Fe0CWwhMSoCEH7NLVoLhF%2Fuploads%2F8wPft4VUDDZHVrdXM7LP%2FREV%20NEO%20v1.1%20Motor%20Curve.svg?alt=media&amp;token=48650c32-dc84-4731-b732-e86572c8474b" alt=""><figcaption><p>NEO v1.1 Motor Curve</p></figcaption></figure>

{% hint style="info" %}
Please read our [Locked Rotor Testing Documentation](/brushless/neo/locked-rotor-testing.md) and ensure you understand how to set an appropriate [Smart Current Limit](/brushless/spark-max/gs/make-it-spin.md#limiting-current) before using your NEO Brushless Motor.
{% endhint %}
