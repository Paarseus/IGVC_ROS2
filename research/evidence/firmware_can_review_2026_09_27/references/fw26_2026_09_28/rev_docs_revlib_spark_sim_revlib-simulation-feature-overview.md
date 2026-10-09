<!-- Source: https://docs.revrobotics.com/revlib/spark/sim/revlib-simulation-feature-overview.md (fetched 2026-09-28) -->
> For the complete documentation index, see [llms.txt](https://docs.revrobotics.com/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://docs.revrobotics.com/revlib/spark/sim/revlib-simulation-feature-overview.md).

# REVLib Simulation Feature Overview

## SparkSim Features

### Automatic GUI Generation

As your simulation runs, GUI elements will be added to the Devices tab as they are called, with specific dialogues for each sensor and tool.

<figure><img src="https://4253857238-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F0OKYENVWAIgVP2TmkWl3%2Fuploads%2FhdTD2ZCcx2mCLg6JpXRr%2Fimage.png?alt=media&amp;token=f25bd3b3-2da4-48b0-b1dd-4cd4ef5f7bcb" alt="" width="186"><figcaption></figcaption></figure>

### WPILib Physics Model Integration

Every device simulation object includes a .iterate method designed for easy integration with WPILib's Physics models and tools.

### Control Over Native Spark Object

Nearly every attribute of the Spark object is directly addressable via the SparkSim object, allowing you to tailor your simulations to any scenario.

### Simulated Fault Manager

By creating a SimFaultManager object, you are given the ability to throw each possible fault individually, either through the GUI or programmatically with the object.

<figure><img src="https://4253857238-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F0OKYENVWAIgVP2TmkWl3%2Fuploads%2Fwa2EgmjX1m9wy0roULel%2Fimage.png?alt=media&amp;token=2f14f681-1c8e-452f-b2c9-df2772d08fe3" alt="" width="244"><figcaption></figcaption></figure>

## Algorithms and Features

### Closed Loop Control

Position, velocity, current, MAXMotion Position Control, and MAXMotion Velocity Control algorithms have been translated into the simulation. All feedforward terms are fully supported.

### MAXMotion Simulation

Both MAXMotion Position Control and MAXMotion Velocity Control are able to be fully simulated.

### Voltage Compensation Algorithm

The Voltage Compensation algorithm from the Sparks has been ported to the simulation.

### Current Limiting Algorithm

The Smart Current Limiting algorithm from the Sparks has been ported to the simulation.

### Encoder, Sensor, and Limit Switch Simulation

All auxiliary devices are able to be fully controlled, through their individual simulation objects. Selected sensors will automatically be updated by the SparkSim.iterate() method. For more details on how to set these device simulations up, see [Simulating Additional Sensors and Auxiliary Devices](/revlib/spark/sim/simulating-additional-sensors-and-auxiliary-devices.md).
