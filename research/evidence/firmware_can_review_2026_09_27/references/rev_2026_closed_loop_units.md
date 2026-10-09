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
