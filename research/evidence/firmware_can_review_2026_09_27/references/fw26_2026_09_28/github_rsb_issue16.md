<!-- Source: https://github.com/REVrobotics/REV-Software-Binaries/issues/16 (fetched 2026-09-28 via GitHub API) -->
# No way to set PID tolerance without using maxmotion 
 mathewdunne 2025-01-09T02:52:26Z closed 

 My team is looking to use the new SparkMaxConfig APIs for setting up PID controllers this year, but there doesn't seem to be a way to set a tolerance on the PID unless you use the maxmotion config. We're having an issue with our swerve modules "clicking" as they oscillate tiny amounts around the setpoint, I think due to backlash in the module gears. Setting a tolerance of 1-2 degrees on the Spark Max's closed loop controller would be an easy fix, but there doesn't seem to be a way to do this.

From what I understand, `SparkMaxConfig.closedLoop.maxMotion.allowedClosedLoopError` will do what I want, but that requires using maxmotion's trapezoidal PID controller, which we don't want to do.

This is our motor config, which is copied from Advantagekit's spark swerve example:
```
var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .encoder
        .positionConversionFactor(turnEncoderPositionFactor)
        .velocityConversionFactor(turnEncoderVelocityFactor)
        .uvwAverageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .pidf(turnKp, 0.0, turnKd, 0.0);
    turnConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
```

Is there a way to set a tolerance on the closedLoop position controller? Or alternatively, is there a way to use maxmotion just as a normal PID without providing velocity and acceleration constraints?

## Comments
--- jfabellera MEMBER 2026-01-19T18:05:20Z 
 This has been added in the [2026 version of REVLib](https://github.com/REVrobotics/REV-Software-Binaries/releases/tag/revlib-2026.0.0). See the [javadocs](https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/closedloopconfig#allowedClosedLoopError(double,com.revrobotics.spark.ClosedLoopSlot))
--- mathewdunne NONE 2026-02-07T02:44:00Z 
 Thanks!
