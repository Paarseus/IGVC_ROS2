> For the complete documentation index, see [llms.txt](https://yagsl.yassrobotics.com/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://yagsl.yassrobotics.com/explanation/chassis-control.md).

# Chassis Control

Chassis-level speed limiting, discretization, and lock pose

Beyond individual module behavior, `SwerveDrive` and `SwerveDriveConfig` control a few chassis-wide concerns: how fast the robot is allowed to go, how commanded velocities get corrected for a subtle simulation-of-motion error, and how to make the robot maximally hard to push.

## Max Speed

YAGSL/YAMS store a maximum chassis linear and angular speed and use it for:

* `SwerveDriveKinematics.desaturateWheelSpeeds` — scaling down all module speeds proportionally if any one module would need to exceed the physical maximum.
* Telemetry (`swerve/maxSpeed`).
* Deriving a default drive feedforward from the motor's free speed, when `pidfproperties.json` doesn't set one explicitly.

The maximum speed represents the physical maximum speed of the robot — it is not a "slow mode" throttle (use `SwerveInputStream.withScaleTranslation`/`withScaleRotation`, or scale your controller input directly, for that).

It's set once via `SwerveDriveConfig.withMaximumChassisSpeed(LinearVelocity, AngularVelocity)`:

```java
var cfg = new SwerveDriveConfig()
    .withMaximumChassisSpeed(MetersPerSecond.of(4.5), DegreesPerSecond.of(360));
```

{% hint style="info" %}
This is a constructor-time setting on `SwerveDriveConfig`, not a JSON field — set it in the code that builds your `SwerveDriveConfig` before passing it to `SwerveParser.createSwerveDrive(...)`.
{% endhint %}

## Chassis Speed Discretization

When you command a `ChassisSpeeds` with both translation and rotation, WPILib's `SwerveDriveKinematics` computes module states assuming that velocity is held constant for the whole next timestep. In reality your loop runs at a discrete rate (typically 20ms), and the true path curves slightly within that window — this shows up as unwanted skew/drift, especially at higher rotation rates.

`ChassisSpeeds.discretize(speeds, dt)` corrects for this by working out what constant-velocity command over `dt` would actually produce the desired end pose. YAMS applies this automatically once you set a discretization time:

```java
var cfg = new SwerveDriveConfig()
    .withDiscretizationTime(Seconds.of(0.02))
    .withSimDiscretizationTime(Seconds.of(0.02)); // optionally different for simulation
```

If you don't set a discretization time, no discretization correction is applied. `0.02` (matching the default robot loop period) is a good starting point for most teams.

## Lock Pose

Lock Pose is a special stance where all the wheels point inward into an X formation, making the robot extremely difficult to push. It's most useful for holding position during defense, or at the end of a match.

```java
drive.lockPose();
```

{% hint style="warning" %}
Lock Pose should only be used when no other drive input is given, or you may get undefined/fighting behavior between the lock and your normal drive command.
{% endhint %}

A typical binding calls it repeatedly while a button is held:

```java
driverXbox.x().whileTrue(Commands.runOnce(drive::lockPose, driveSubsystem).repeatedly());
```


---

# Agent Instructions
This documentation is published with GitBook. GitBook is the documentation platform designed so that both humans and AI agents can read, navigate, and reason over technical content effectively. Learn more at gitbook.com.

## Querying This Documentation
If you need additional information that is not directly available in this page, you can query the documentation dynamically by asking a question.

Perform an HTTP GET request on the current page URL with the `ask` query parameter, and the optional `goal` query parameter:

```
GET https://yagsl.yassrobotics.com/explanation/chassis-control.md?ask=<question>&goal=<endgoal>
```

`ask` is the immediate question: it should be specific, self-contained, and written in natural language.
`goal` is optional and describes the broader end goal you are ultimately trying to accomplish on behalf of the user. GitBook uses it to tailor the answer towards what is most useful for that goal.

The response will contain a direct answer to the question and relevant excerpts and sources from the documentation.

Use this mechanism when the answer is not explicitly present in the current page, you need clarification or additional context, or you want to retrieve related documentation sections.
