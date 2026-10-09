<!-- Source: https://github.com/REVrobotics/REV-Software-Binaries/issues/29 (fetched 2026-09-28 via GitHub API) -->
# Gravity "cosine" voltage bug in Spark simulation 
 mpaulonis 2026-06-02T13:19:30Z open 

 In doing some summer simulation experiments, I'm finding that gravity cosine voltage (kCos) is not varying with arm position in Java Spark simulation for MAXMotion position with feedforward on a WPILib SingleJointedArmSim.  I've configured the simulation so everything except gravity cosine feedforward is working properly.  I know the controller is getting encoder feedback since velocity feedforward is working fine.  The kCos feedforward voltage component remains constant at the configured (0 arm degree) value regardless of the simulated arm angle.   The results are the same for a simulated Spark Flex and Spark MAX from REVLib-java:2026.0.5. I'm assuming that the problem only exists in simulation, since it would likely have been reported long ago if it existed in hardware.  I don't have access to hardware right now to test that environment.  You may already have simulation code you can test with, or you might want to create some.  If you would like me to provide a GitHub repo with code to reproduce this problem, just let me know.

## Comments
--- amsam0 NONE 2026-09-28T01:17:06Z 
 The kV term also doesn't seem to do anything. We had to add this to our sim code:

```java
        // REVLib doesn't include kV when simulating. This is a bug with REVLib.
        // To work around the bug, just calculate the kV term ourselves.
        // See also https://github.com/REVrobotics/REV-Software-Binaries/issues/29
        double ffkV = setpointRadPerSec * spark.configAccessor.closedLoop.feedForward.getkV(ClosedLoopSlot.kSlot1);
```
