<!-- Source: https://github.com/REVrobotics/REV-Software-Binaries/issues/25 (fetched 2026-09-28 via GitHub API) -->
# Rev Client 2.0 doesn't run motor 
 Chris7898 2026-01-17T20:27:49Z open 

 While attempting to run a spark max / Neo combo in the new rev client, no matter what type and how much power, it doesn't run the motor. I tried while it was connected to CAN and without CAN. Additionally while updating spark maxes, it would occasionally freeze up and get to a point where I would need to restart rev client and power cycle the robot.

## Comments
--- jfabellera MEMBER 2026-01-18T03:28:51Z 
 Please click the report issue button at the bottom right of the application after attempting to run the motor. This will send us some logs so we can look at it further.

When this happens, what is the LED doing on the motor controller? Does reflashing the firmware help?
