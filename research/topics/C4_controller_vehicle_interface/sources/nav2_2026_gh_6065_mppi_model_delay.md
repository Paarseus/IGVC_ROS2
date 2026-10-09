# #6065 [MPPI] Add model delay to represent physical latency
Author: BriceRenaudeau  Created: 2026-04-05T14:22:56Z  State: closed
URL: https://github.com/ros-navigation/navigation2/issues/6065

## Feature request

#### Feature description
On every real robot, there is a small latency between the cmd_vel command and the real motion.
It's often small enough to be omitted, and a small acceleration can hide it.
But on heavy or long robots, this can create overshoot and zigzag.

By adding this `model_delay` parameter, we can make MPPi aware of this behavior.

#### Implementation considerations
I did a small implementation by shifting the command vector values.
The behavior in long and narrow corridors was clearly improved.

But there are several points that can be discussed:
- The delay is not applied to the visualisation so the trajectory is wrong
- We can discuss what to do with the first points (copy or interpolation)


---
**SteveMacenski** (MEMBER, 2026-04-07T00:13:14Z):
Can you check out https://github.com/ros-navigation/navigation2/tree/low_accel which does some pretty big changes to MPPI for handling low accelerations? I wonder if what you're proposing is still needed in this case if you're only seeing a problem with low accelerations. There are alot of problems today with what happens with low accelerations, so I wouldn't be surprised if this was another way to mask the real issue which I solve in that branch.

That branch is largely completely in correctly handling low accelerations - though due to changes in how the controls are combined, the angular velocity has a little more noise in it (I don't have the characterized % yet, but the jitter is still in the < 2 deg/s range at max, so totally usable even while I work on reducing it further). That is my next thing I'm working on there to patch that up as well.

Still open to merging your PR (and plan to anyway since its a good feature to have even if not needed for this reason anymore) but please give that a try  
