<!-- Source: https://github.com/REVrobotics/REV-Software-Binaries/issues/12 (fetched 2026-09-28 via GitHub API) -->
# Using voltage compensation on sparkmax causes any commanded value to not move motor.  
 eohara0920 2024-12-28T23:39:06Z closed 

 Using configuration with 2025 API to enable voltage compensation (in my testing a value of 11.0) causes the motor controller to no longer output any power to the motor when using .set() and closed loop methods. 

## Comments
--- doleksy NONE 2025-01-27T20:52:00Z 
 I'm sorry for not getting back to you sooner. Are you still experiencing this issue? I am not able to reproduce this on my end.

What is your setpoint? With a very slow-moving motor (setpoint < 0.05), it appears that the voltage compensation doesn't work.

Do you have an example I could take a look at?
--- jfabellera MEMBER 2025-02-10T17:01:40Z 
 Closing since we can't reproduce this. Feel free to re-open if you are still having issues.
