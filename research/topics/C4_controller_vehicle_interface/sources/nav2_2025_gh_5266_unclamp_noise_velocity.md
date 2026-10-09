# navigation2 #5266: Unclamp noise velocity.
URL: https://github.com/ros-navigation/navigation2/pull/5266
Author: chanhhoang99  Created: 2025-06-13T08:30:22Z  State: closed  PR: true

<!-- Please fill out the following pull request template for non-trivial changes to help us process your PR faster and more efficiently.-->

---

## Basic Info

| Info | Please fill out this column |
| ------ | ----------- |
| Ticket(s) this addresses   | (add tickets here #5214) |
| Primary OS tested on | (Ubuntu) |
| Robotic platform tested on | (Real robot with video and bags recorded) |
| Does this PR contain AI generated software? | (No) |

---

## Description of contribution in a few bullet points
Verify acceleration constrain applied to noise velocity sample.

## Description of documentation updates required from your changes

<!--
* Added new parameter, so need to add that to default configs and documentation page
* I added some capabilities, need to document them
-->

## Description of how this change was tested
Tested with robot base which has acceleration clamp in driver controller and verify if robot MPPI controller can predict that acceleration clamp or not.(Tested with low acceleration e.g az = 0.1 rad/s^2, ax max = 0.5 m/s^2, ax min = -0.5m/s^2)

<!--
* I wrote unit tests that cover 90%+ of changes and extensively tested on my physical robot platform in production for 1 week
* I wrote unit tests and tested in simulation for 10 minutes
* Performed linting validation using pre-commit run --all or colcon test
-->

---

## Future work that may be required in bullet points

<!--
* I think there might be some optimizations to be made from STL vector
* I see a lot of redundancy in this package, we might want to add a function `bool XYZ()` to reduce clutter
* I tested on a differential drive robot, but there might be issues turning near corners on an omnidirectional platform
-->

#### For Maintainers: <!-- DO NOT EDIT OR REMOVE -->
- [ ] Check that any new parameters added are updated in docs.nav2.org
- [ ] Check that any significant change is added to the migration guide
- [ ] Check that any new features **OR** changes to existing behaviors are reflected in the tuning guide
- [ ] Check that any new functions have Doxygen added
- [ ] Check that any new features have test coverage
- [ ] Check that any new plugins is added to the plugins page
- [ ] If BT Node, Additionally: add to BT's XML index of nodes for groot, BT package's readme table, and BT library lists



---
## Comment by mergify[bot] (CONTRIBUTOR) at 2025-06-13T09:16:37Z

@chanhhoang99, your PR has failed to build. Please check CI outputs and resolve issues.
You may need to rebase or pull in `main` due to API changes (or your contribution genuinely fails).

---
## Comment by SteveMacenski (MEMBER) at 2025-06-16T20:31:42Z

See my comment in https://github.com/huynhduc9905/navigation2/pull/6#discussion_r2150823818

---
## Comment by codecov[bot] (NONE) at 2025-06-17T03:56:23Z

## [Codecov](https://app.codecov.io/gh/ros-navigation/navigation2/pull/5266?dropdown=coverage&src=pr&el=h1&utm_medium=referral&utm_source=github&utm_content=comment&utm_campaign=pr+comments&utm_term=ros-navigation) Report
All modified and coverable lines are covered by tests :white_check_mark:

| [Files with missing lines](https://app.codecov.io/gh/ros-navigation/navigation2/pull/5266?dropdown=coverage&src=pr&el=tree&utm_medium=referral&utm_source=github&utm_content=comment&utm_campaign=pr+comments&utm_term=ros-navigation) | Coverage Δ | |
|---|---|---|
| [...ler/include/nav2\_mppi\_controller/motion\_models.hpp](https://app.codecov.io/gh/ros-navigation/navigation2/pull/5266?src=pr&el=tree&filepath=nav2_mppi_controller%2Finclude%2Fnav2_mppi_controller%2Fmotion_models.hpp&utm_medium=referral&utm_source=github&utm_content=comment&utm_campaign=pr+comments&utm_term=ros-navigation#diff-bmF2Ml9tcHBpX2NvbnRyb2xsZXIvaW5jbHVkZS9uYXYyX21wcGlfY29udHJvbGxlci9tb3Rpb25fbW9kZWxzLmhwcA==) | `100.00% <100.00%> (ø)` | |

... and [7 files with indirect coverage changes](https://app.codecov.io/gh/ros-navigation/navigation2/pull/5266/indirect-changes?src=pr&el=tree-more&utm_medium=referral&utm_source=github&utm_content=comment&utm_campaign=pr+comments&utm_term=ros-navigation)

<details><summary> :rocket: New features to boost your workflow: </summary>

- :snowflake: [Test Analytics](https://docs.codecov.com/docs/test-analytics): Detect flaky tests, report on failures, and find test suite problems.
</details>

---
## Comment by SteveMacenski (MEMBER) at 2025-06-17T05:24:43Z

Your compiler warnings are odd - I just opened a PR that should address it https://github.com/ros-navigation/navigation2/pull/5277. I'll merge once CI passes in the morning and then you should rebase on it to get the Build Against Released Distributions" jobs to pass 

---
## Comment by SteveMacenski (MEMBER) at 2025-06-17T06:27:11Z

Merged - please rebase!

I’m not sure what’s going on with mypy linting - feel free to ignore that for the scope of this PR if we get everything else passing.  

---
## Comment by SteveMacenski (MEMBER) at 2025-06-17T17:46:42Z

@chanhhoang99 please sign off with DCO :-) That's the only thing blocking for merge other than asking others to validate since this is a major change (even though its like 3 changes :laughing: ) 

---
## Comment by SteveMacenski (MEMBER) at 2025-06-20T20:26:27Z

Just waiting on testing BTW! 

---
## Comment by SteveMacenski (MEMBER) at 2025-07-02T17:32:38Z

I'm having a hard time finding beta testers with similar situations that aren't too underwater to test this. I'm going to go ahead and merge this after some preliminary testing done be 2-3 people and didn't notice an issue. I'll just keep a look out on the issue tracker if there are any regressions reported. 

Thanks for this contribution @chanhhoang99 ! 

---
## Comment by SteveMacenski (MEMBER) at 2025-07-02T17:33:08Z

@chanhhoang99 should we keep chatting about the lag compensation piece? 

---
## Comment by chanhhoang99 (CONTRIBUTOR) at 2025-07-03T02:32:00Z

@SteveMacenski , I have not yet make progress on that. I will create another issue when there is improvement.

---
## Comment by SteveMacenski (MEMBER) at 2025-07-03T02:37:55Z

Great thanks for the update! Let me know if I can be of help! I really appreciate you digging into MPPI, not many folks have given me critical feedback on the internal functioning to date (only that they like the outputs) 

---
## Comment by tonynajjar (CONTRIBUTOR) at 2025-08-19T12:55:35Z

For me these changes are breaking; the acceleration limits are no longer respected:

before this PR

<img width="311" height="265" alt="image" src="https://github.com/user-attachments/assets/bdfd739b-6d39-4210-b9a9-a9fac5446952" />

after this PR

<img width="207" height="251" alt="image" src="https://github.com/user-attachments/assets/c58f0771-5f94-476c-98ec-e566216350eb" />


I'm not too familiar with this part to debug, can you guys see an issue with it? I created https://github.com/ros-navigation/navigation2/issues/5464 to track

---
## Comment by SteveMacenski (MEMBER) at 2026-04-01T19:03:28Z

@chanhhoang99 note that I'm currently thinking about this again on a related topic and I as before think that this is correct (though had to revert since it broke people). 

On some analysis, I think the issue with the wobbling is due to the control sequence being based on the unsmoothed `cvx/cwz` vs the previously acceleration bounded versions. That makes the output trajectory more unstable. 

The reason that Open Loop odometry seemed to work is since any error / lag in execution is going to amplify the issues, but I don't think that actually solves the core issue at play. 

Thus I think the answer is to spend some time on improving the pipeline so that this is more stable with different sampling distributions/spaces and/or post-processing. I have ideas but so far nothing tested to give you concrete ideas. 

---
## Comment by SteveMacenski (MEMBER) at 2026-04-02T23:06:06Z

@chanhhoang99 @adivardi

Check out my notes / progress here: https://github.com/ros-navigation/navigation2/blob/low_accel/low_accel_notes.md

I'm curious on your thoughts, but I think that the open-loop solution is actually a hack that really shouldn't work, but it makes the problem less amplified. I've found the actual no-kidding solution. The next step I need to do is improve MPPI's smoothness to make sure that this is all a net-positive for all users (though it probably already is).

I do not see the wobble issue that you report, but it is **very likely** a manifestation of some other issues I had to resolve to get the open-loop / closed-loop to work properly with acceleration handling at `t=0`. I suspect if you try my solution that may just go away. There's still the matter of reducing the jitter in the velocities with better smoothing / sampling distribution / etc, but that's a different topic while I'll start exploring next week.

Please take a look and let me know. Sorry about the delay, I can't always personally jump in at the time when users file issues :( I am doing so now and really digging into it. 

---
## Comment by chanhhoang99 (CONTRIBUTOR) at 2026-04-03T08:00:00Z

@SteveMacenski Do you still have emails that I sent you which have video recordings/yaml file when I was testing with various accel, model_dt, ... changes ? I think they could help understand situation a little bit.
(Those are for the inital PR unclamp the cxv not the open_loop PR.)

---
## Comment by SteveMacenski (MEMBER) at 2026-04-03T19:29:06Z

I can't seem to find it, can you forward it back to me again? If there's any color you can give me with the context of my current work (things to check/test) that would be appreciated

---
## Comment by chanhhoang99 (CONTRIBUTOR) at 2026-04-06T04:05:10Z

Fowarded

---
## Comment by adivardi (CONTRIBUTOR) at 2026-04-07T12:53:30Z

I think this sums up well a few things I also noticed. In particular:
- Reading the original MPPI paper I also concluded that #5266 is theoretically correct
- use `open_loop` mode violates acceleration limits between iterations:  Also noticed that, though it is not causing much issues as our drive motor & steering are really slow, so they "smooth it out"  (good to fix though)
- `Add acceleration constraints in applyControlSequenceConstraints() relative to the starting speed to ensure that the trajectory is feasible to execute from the current state as well as within the optimal trajectory`  :   I added these to our branch for sanity a while ago, though I didn't see a noticeable change.
- `Essentially the shifting logic 'skips' the first control sample vx(0) and applies vx(1)`  :  I still don't get why we do that like we discussed in another PR, but changing it didn't make a noticeable change :sweat_smile: 
- At the time before the open_loop was added (but with the unclamped controls), I also tried playing with gamma & temperature a bit and didn't get any improvement

I also tested it a bit in sim, though it is still WIP, right?

- When using `open_loop: false`  with and without your changes: robot is doing big snake motion around the straight path. sometimes it becomes unstable and fully veers off away from the path, potentially getting stuck circling forever. This is also what I had last year without the open loop mode or without PR 5266   (I think with the delay and slow steering, the odometry is just lagging so much behind that MPPI cannot be "dynamic" enough to follow a path)

- With `open_loop: true` and your fix, I am still getting the "wobble" that I had with #5266  - very noisy wz command causing the optimal trajectory to jump around a lot
<img width="1735" height="1101" alt="steve_fix_open_loop" src="https://github.com/user-attachments/assets/32c60963-2e39-4ef6-89ba-cce5629d7db2" />

For comparison, without your changes:
<img width="1644" height="1105" alt="open_loop" src="https://github.com/user-attachments/assets/f1aad439-edeb-4551-948b-65f47420aac5" />

For refernce, an extract of my config

```
controller_frequency: 20.0
time_steps: 100
model_dt: 0.05
batch_size: 2000

      ax_max: 1.7   # acceleration, regardless of direction
      ax_min: -1.7  # deceleration, regardless of direction
      ay_max: 0.0
      ay_min: 0.0
      az_max: 0.6061
      vx_max: 0.8
      vx_min: -0.4
      vy_max: 0.0
      wz_max: 0.853
```



---
## Comment by SteveMacenski (MEMBER) at 2026-04-07T17:56:22Z

Thanks for the feedback - what about pulling in the changes of https://github.com/ros-navigation/navigation2/pull/6066 to that branch, does that solve your "snaking" issue? I think that's probably due to a delay in the system which this could model. Using your plots, find the dt between the odom and the cmd_vel, then put that in the model delay parameter. Try again with open vs closed loop :-) 

I personally hope that open loop is just totally unnecessary to get good behavior. I would leave it in place (some rare applications have no odometry at all), but my aim would be to have something that uses your real data to make sure you're physically grounded. 

You're only seeing that behavior with low accelerations, correct? 

---
## Comment by chanhhoang99 (CONTRIBUTOR) at 2026-04-08T03:58:30Z

@adivardi @SteveMacenski  , As I remember correcly, if applying this PR, user have to make there base controller (motor driver controller) to have control cmd clamp(which sent by nav2 stack) with exact acceleration clamp in the MPPI in the driver. 
For example if I set ax_max = 0.5m/s^2 for MPPI, I have to do exact clamp in the base controller. Else it could not work correctly.

Read the "Description of how this change was tested" in this PR could tell.

---
## Comment by SteveMacenski (MEMBER) at 2026-04-08T06:15:23Z

The version in my branch is different; the acceleration limits are applied in the refinement of the optimal trajectory 🙂

---
## Comment by adivardi (CONTRIBUTOR) at 2026-04-09T07:44:19Z

> Thanks for the feedback - what about pulling in the changes of #6066 to that branch, does that solve your "snaking" issue? I think that's probably due to a delay in the system which this could model. Using your plots, find the dt between the odom and the cmd_vel, then put that in the model delay parameter. Try again with open vs closed loop :-)
> 
> I personally hope that open loop is just totally unnecessary to get good behavior. I would leave it in place (some rare applications have no odometry at all), but my aim would be to have something that uses your real data to make sure you're physically grounded.

oh nice, I wasn't aware of it. We have also been working on something similar since our robot has a really high delay.

I tried it out. It improves the oscillations but still oscillates a lot more than with the open_loop. That being said, it also improves the performance with open_loop, so a good addition regardless of the open_loop choice.

> You're only seeing that behavior with low accelerations, correct?
the oscillations? yes, together with delay. When I had perfect sim, MPPI was working well. But I needed open_loop to make it work with the actual robot since it has high actuation delay + slow steering speed (=> low jerk => low accel). once I added actuation delay to the sim, I started getting the same results


I haven't tried again with your low_acceleration branch, since I see you are still working on it.

---
## Comment by SteveMacenski (MEMBER) at 2026-04-09T18:27:37Z

The low acceleration branch is now done and ready for testing! 

---
## Comment by adivardi (CONTRIBUTOR) at 2026-04-13T11:40:36Z

So I tested again in simulation and I am still getting the very noisy/wobbly command signal.

- Adding the delay compensation PR: Always helpful, whatever the other settings are :heavy_check_mark: 
- closed_loop: Together with the delay compensation, I have no oscillations anymore, but it gets stuck before the u-turn for some reason

- noisy command: I tried reducing wz_std. Originally it was 0.4rad/s2  (default). I reduced all the way to 0.1rad/s2, but at this value the path tracking is worse. At 0.2rad/s2, path tracking is OK. Anyway, neither change helps with the noisy signal.

Here is a comparison:
Without the low_accel branch:
 
<img width="1974" height="971" alt="mppi_nolowaccel_openloop_delay0-4_wzstd0-4_public" src="https://github.com/user-attachments/assets/1cdd0d77-ea12-4997-ad76-cb9e8c6e35d3" />

With the branch:
<img width="1974" height="971" alt="mppi_lowaccel_openloop_delay0-4_wzstd0-4_public" src="https://github.com/user-attachments/assets/d9c45f1f-9f91-4726-8824-5dd89875ff8d" />

you can see how the blue command from mppi is really noisy in the 2nd image. Also the slow down before the u-turn is much more jumpy (maybe can be improved with tuning ax a bit, but still)

---
## Comment by SteveMacenski (MEMBER) at 2026-04-13T19:53:52Z

And with open-loop these are removed in either/both cases? It sounds like in your application its worth sticking with that one then. I'm curious if you have any ideas why that is or something I can do to improve it? It seems to me maybe its due to noisy odometry -- else I"m not sure what else is the difference between open and closed loop. The two contributors that come to my mind are (1) delay (which we've modeled) and (2) noise (which we haven't). 

---
## Comment by adivardi (CONTRIBUTOR) at 2026-04-14T07:15:47Z

> And with open-loop these are removed in either/both cases? It sounds like in your application its worth sticking with that one then. I'm curious if you have any ideas why that is or something I can do to improve it? It seems to me maybe its due to noisy odometry -- else I"m not sure what else is the difference between open and closed loop. The two contributors that come to my mind are (1) delay (which we've modeled) and (2) noise (which we haven't).

No, it happens once I add the `low_accel` branch, whatever else I do, even in open loop. It is the same issue I had when adding this PR (5266).
It also happens when I remove the delay from sim. I also using the Gazebo ground truth for odometry.

I tried keeping all your other changes, but clamping the raw controls (`state.cvx/cwz`) in `motion_models` (so just undo your changes tho this 1 file, see diff below) - this solves the issue. So it really is caused by unclamping the raw controls, but I am not sure why.

```
--- a/nav2_mppi_controller/include/nav2_mppi_controller/motion_models.hpp
+++ b/nav2_mppi_controller/include/nav2_mppi_controller/motion_models.hpp
@@ -92,13 +92,15 @@ public:
         0).select(
         state.vx.col(i - 1) + max_delta_vx,
         state.vx.col(i - 1) - min_delta_vx);
-      state.vx.col(i) = state.cvx.col(i - 1)
-        .cwiseMax(lower_bound_vx)
-        .cwiseMin(upper_bound_vx);
+      state.cvx.col(i - 1) = state.cvx.col(i - 1)
+                                 .cwiseMax(lower_bound_vx)
+                                 .cwiseMin(upper_bound_vx);
+      state.vx.col(i) = state.cvx.col(i - 1);
 
-      state.wz.col(i) = state.cwz.col(i - 1)
-        .cwiseMax(state.wz.col(i - 1) - max_delta_wz)
-        .cwiseMin(state.wz.col(i - 1) + max_delta_wz);
+      state.cwz.col(i - 1) = state.cwz.col(i - 1)
+                                 .cwiseMax(state.wz.col(i - 1) - max_delta_wz)
+                                 .cwiseMin(state.wz.col(i - 1) + max_delta_wz);
+      state.wz.col(i) = state.cwz.col(i - 1);
 
       if (is_holo) {
         auto lower_bound_vy = (state.vy.col(i - 1) >
@@ -109,9 +111,10 @@ public:
           0).select(
           state.vy.col(i - 1) + max_delta_vy,
           state.vy.col(i - 1) - min_delta_vy);
-        state.vy.col(i) = state.cvy.col(i - 1)
-          .cwiseMax(lower_bound_vy)
-          .cwiseMin(upper_bound_vy);
+        state.cvy.col(i - 1) = state.cvy.col(i - 1)
+                                   .cwiseMax(lower_bound_vy)
+                                   .cwiseMin(upper_bound_vy);
+        state.vy.col(i) = state.cvy.col(i - 1);
       }
     }
```

---
## Comment by SteveMacenski (MEMBER) at 2026-04-15T19:43:14Z

I have no earthly idea either, I can't reproduce it. Do you you see it running in the turtlebot sim in nav2_bringup? If not, then maybe you can highlight some of the changes you have in your simulator (use the diff drive plugin, modeled delay or anything else?) which may be able to narrow down to some way I can reproduce and debug. 

Push comes to shove, I can parameterize whether to clamp the controls so you can still get that behavior, but I would like to be able to reproduce and understand why (and hopefully fix) rather than introducing yet another variable and not having very clear documentation about who should use it when/why.

---
## Comment by SteveMacenski (MEMBER) at 2026-04-21T21:05:32Z

@chanhhoang99 I just pushed one more update which should handle some of the low acceleration issues - can you check if you see a difference now? I don't know if it'll help, but its one of my last honest tries :-) 

---
## Comment by chanhhoang99 (CONTRIBUTOR) at 2026-04-22T03:17:52Z

@SteveMacenski , I think you mentioned @adivardi , I really want to test your change but currently I don't have correct setup to test it.

---
## Comment by SteveMacenski (MEMBER) at 2026-04-22T20:58:27Z

You are correct, my apologies :upside_down_face: 

---
## Comment by adivardi (CONTRIBUTOR) at 2026-04-28T09:42:25Z

@SteveMacenski 
I tried your branch again, this time with the turtlebot simulation `ros2 launch nav2_bringup tb4_simulation_launch.py headless:=False`

For each test I sent to goals from rviz: each roughly straight ahead with yaw at ~180degrees (so it goes straight then kinda turns on the spot) 

I set the kinematic params to my robot's params, but left the other params as default.
```
     ax_max: 1.7  # 3.0
     ax_min: -1.7  # -3.0
     ay_max: 0.0  # 3.0
     ay_min: 0.0  # -3.0
     az_max: 0.6061  # 3.5
     vx_std: 0.2  # see comparison table!
     vy_std: 0.2
     wz_std: 0.4
     vx_max: 0.5
     vx_min: -0.35
     vy_max: 0.0  # 0.5
     wz_max: 0.8534  # 1.9
```

I can recreate the issue when I increase the `timesteps` to 100 and the `vx_std` to 0.5.
The `vx_std` change only ampilfies it though, it is still noisy with 0.2 though not as crazy (and it doesn't even reach the max speed of 0.5m/s in that case!)

I used `prune_distance` of 10m, just to make sure it doesn't cut the horizon when the steps are 100, but I didn't see any change when reducing it back to 2m.


slow kinematics + steps 56 + prune 10 + vxstd 0.5

<img width="948" height="990" alt="slow kinematics + steps 56 + prune 10 + vxstd 0 5" src="https://github.com/user-attachments/assets/2d44e505-fd02-4195-990b-f7d1173ce864" />

slow kinematics + steps 100 + prune 10 + vxstd 0.5

<img width="948" height="990" alt="slow kinematics + steps 100 + prune 10 + vxstd 0 5" src="https://github.com/user-attachments/assets/6c9ad5ad-04a9-454a-b8b5-e588566c4bc1" />

slow kinematics + steps 56 + prune 10 + vxstd 0.2

<img width="948" height="990" alt="slow kinematics + steps 56 + prune 10 + vxstd 0 2" src="https://github.com/user-attachments/assets/cf77dd88-9b59-4ada-98c2-33780541d487" />

slow kinematics + steps 100 + prune 10 + vxstd 0.2

<img width="948" height="990" alt="slow kinematics + steps 100 + prune 10 + vxstd 0 2" src="https://github.com/user-attachments/assets/092bc4e7-8701-4ed1-a313-3ada0d92c8e6" />


---
## Comment by SteveMacenski (MEMBER) at 2026-04-30T16:30:35Z

OK https://github.com/ros-navigation/navigation2/pull/6072 has been tested and in its final form. I made a few tweaks that actually fixed some issues.

I guess the follow up here is: (1) is this PR still an issue for you or are you satisfied with it, and then relatedly (2) should I parameterize the unclamping of the control velocities if you need that option so that I don't break you? 

For (2): if its necessary to keep, can you give me a short write up about your intuition and/or confirmed understanding about why this is necessary in your case? I will need to describe the parameter, so knowing when people should consider using it would be good. 

---
## Comment by adivardi (CONTRIBUTOR) at 2026-05-04T06:52:35Z

Have you seen my last comment with the plots?
Have you been able to replicate my issues with the turtlebot when increasing `time_steps`?
Or am I the only one seeing the noisy command signal issue on both my robot and turtlebot?


The last commit included in my test is `67f45aedb81f9dcc6ed845dcab8f105db57d40b7`.  I can try to test the newest branch with turtlebot sometimes this week.

---
## Comment by SteveMacenski (MEMBER) at 2026-05-05T19:36:21Z

>  Have you been able to replicate my issues with the turtlebot when increasing time_steps?

What's your model_dt and controller_frequency? If you make them the same amount (i.e. 10hz = 0.1s or 20hz = 0.05s) do you see the same? If you have that already, doing 10hz @ 0.05s for a 2:1 would be useful. I'm hypothesizing the control shift enabled may be playing a roll. 

The newest commit though does actually address an important underlying bug so that may resolve your issue if you try that out. 



---
## Comment by SteveMacenski (MEMBER) at 2026-05-07T00:52:52Z

BTW - I'm looking to merge my branch shortly. If I merge it and your tests are still having problems, please file a ticket and I'll add to my queue to allow for a parameter to clamp the controls. This would be helpful in this case though:

> can you give me a short write up about your intuition and/or confirmed understanding about why this is necessary in your case? I will need to describe the parameter, so knowing when people should consider using it would be good.

I really look forward to hearing how to it goes for you though, I think it should be better now :smile: 

---
## Comment by adivardi (CONTRIBUTOR) at 2026-05-11T13:22:23Z

Sorry for the slow reply, I was sick last week. I will try to test it more this week and next week on our robot

---
## Comment by adivardi (CONTRIBUTOR) at 2026-05-26T13:45:55Z

I tested it last week with turtlebot & our robot. I unfortunately still see the incresed "noise" when the time_steps are set high.
I opened a new issue [here](https://github.com/ros-navigation/navigation2/issues/6171)
