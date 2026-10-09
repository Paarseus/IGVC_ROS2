Source: https://github.com/ros-navigation/navigation2/issues/5524
Retrieved: 2026-09-27 via gh issue view --comments

title:	Open Loop vs Closed Loop in Controller Server, Velocity Smoother
state:	CLOSED
author:	mini-1235 (Maurice Alexander Purnawan)
labels:	question
comments:	13
assignees:	
projects:	
milestone:	
issue-type:	
parent:	
sub-issues:	
sub-issues-completed:	
blocked-by:	
blocking:	
number:	5524
--
I was reading about DWPP https://github.com/Decwest/nav2_dynamic_window_pure_pursuit_controller, which shows that it has better performance than RPP. I noticed that it has an option to choose open loop/closed loop in the plugin, https://github.com/Decwest/nav2_dynamic_window_pure_pursuit_controller/blob/bf281cd92d74d748794aea386f767f9c8d67f78f/src/dynamic_window_pure_pursuit_controller.cpp#L257-L264

Currently in the codebase, we have only closed loop mode in controller server, except for rotation shim, which was introduced in https://github.com/ros-navigation/navigation2/pull/4880. For velocity smoother, we have closed loop / open loop(default), which means by default, we will have:

- Controller Server (closed loop)
- Velocity Smoother (open loop)


From the description of velocity smoother's parameters:
> Type of feedback to use for the current state of the robot’s velocity. In OPEN_LOOP, it will use the last commanded velocity as the next iteration’s current velocity. When acceleration limits are set appropriately, this is a good assumption. In CLOSED_LOOP, it will use the odometry from the odom topic to estimate the robot’s current speed. In closed loop mode, it is important that the odometry is high rate and low latency, relative to the smoothing frequency.

I am actually curious about what happen if we set it to open loop + open loop, for example when we have set the acceleration limits appropriately, or when we do not have high rate odometry. Will the result be better theoretically?

I have actually tried closed loop + closed loop (I have a high rate odometry) a few weeks ago, but the result is worse than closed loop + open loop, I guess I didn't set the limits appropriately




=== COMMENTS ===
author:	SteveMacenski
association:	member
edited:	false
status:	none
--
This is the first I'm seeing of this (to be fair, the github was created last week, so I think the first most are seeing of it 😆 ). I think it might be nice to pull @Decwest into this discussion. 

So the open / closed loop in the Velocity Smoother comes from my research and consolidation of features across Velocity Smoothers from ROS 1. I was aware of a half dozen or so Velocity Smoothers that each worked slightly differently, with slightly different behavior, for slightly different applications - but broadly interchangeable. I put together a feature set by reviewing all implementations I could find and developing a package for Nav2 that incorporated all of the implementations' features in one concise package so that there wasn't a need to create additional ones in the future with the goal of eliminating the need of additional Velocity Smoothers. Closed / Open Loop were one of those features and useful in the context of highly delayed or particularly low-quality odometry. 

> Will the result be better theoretically?

If the acceleration limits are accurately set from empirical data and your robot has a professional quality response time, then it could theoretically be better only if the odometry from the robot is delayed and/or inaccurate (but the robot did actually execute the command accurately and timely). Otherwise, it could largely be the same. You could test this out relatively easily (if not messy) by removing your odometry and lightly modifying the controller server to publish the odom topic and odom TF transformations based on the current velocity command. Or, just passing in the speed to use as the previous output command - but just take care that no TF transformations with odometry or other uses of `OdomSmoother` are being used in the code to estimate speed. 

Certain algorithms like RPP and Graceful really don't use the input velocity so its moot. Those are more for DWB and MPPI which consider the dynamic limitations from the current speed when calculating the next trajectory. The `speed` in RPP is only used to find the lookahead carrot distance when using adaptive lookahead distances and for rotating to heading - but not the actual path following. 

PS @Decwest: Any reason this is a standalone repository? I'd be happy to have this integrated into the Nav2's RPP controller implementation if you were interested. I took a quick glance and I don't see any reason why it would be terribly difficult and would give your work more exposure / use! I'd just want to sync up on what it does and how you see that improving some element of robot behavior (which it sounds like it does well). Can you send me a PDF of your paper? 
--
author:	mini-1235
association:	collaborator
edited:	false
status:	none
--
> So the open / closed loop in the Velocity Smoother comes from my research and consolidation of features across Velocity Smoothers from ROS 1. I was aware of a half dozen or so Velocity Smoothers that each worked slightly differently, with slightly different behavior, for slightly different applications - but broadly interchangeable. I put together a feature set by reviewing all implementations I could find and developing a package for Nav2 that incorporated all of the implementations' features in one concise package so that there wasn't a need to create additional ones in the future with the goal of eliminating the need of additional Velocity Smoothers. Closed / Open Loop were one of those features and useful in the context of highly delayed or particularly low-quality odometry.

I tested the closed loop mode of velocity smoother on tb3/tb4, and noticed that the robots occasionally get stuck on obstacles. I assume the default configuration already applies the proper limits, and /odom runs at 30 Hz compared to the velocity smoother’s 20 Hz frequency. Do you have any thoughts on why closed loop mode velocity smoother might be problematic for tb3/tb4? Also, do we have any examples of velocity smoother configured in closed loop mode so that I can take a look?

> Certain algorithms like RPP and Graceful really don't use the input velocity so its moot. Those are more for DWB and MPPI which consider the dynamic limitations from the current speed when calculating the next trajectory. The speed in RPP is only used to find the lookahead carrot distance when using adaptive lookahead distances and for rotating to heading - but not the actual path following.

I am actually interested in the rotate to heading part as #4880 introduces open loop mode to the rotation shim controller, RPP has the exact same logic for calculating wz. Is that PR intended to solve cases where the rotation shim is used with low-rate odometry, such that the current speed estimate is not reliable?
--
author:	SteveMacenski
association:	member
edited:	true
status:	none
--
>  and /odom runs at 30 Hz compared to the velocity smoother’s 20 Hz frequency. Do you have any thoughts on why closed loop mode velocity smoother might be problematic for tb3/tb4?

What does 'stuck on obstacles' mean? Also, odom for real hardware robots is typically at about 100hz or higher, so try increasing that in gazebo. I'm not sure if there's any simulator downside to that or not. 30hz was just taken from Robotis / Clearpath on their simulators. 

> Is that PR intended to solve cases where the rotation shim is used with low-rate odometry, such that the current speed estimate is not reliable?

See https://github.com/ros-navigation/docs.nav2.org/pull/639/files. Maybe adding that option to another places wouldn't be a bad idea. 
--
author:	mini-1235
association:	collaborator
edited:	false
status:	none
--
I tried to do some experiments on tb4 today to see how different odometry rates and feedback types affect the controller. From visualizing the robot in RViz and looking at the /cmd_vel plots, I think it is hard to clearly show my result here. Are there any tools I can use to more clearly demonstrate the performance actually improves after a change? I noticed that we have a benchmark tool in MPPI, is that something I can use here? 
--
author:	SteveMacenski
association:	member
edited:	false
status:	none
--
It depends on what metric you're trying to capture. Something you could do possibly is compare the simulated 'actual' speed with the provided speed to the controller algorithm to see the latency. You could also plot that via RQT to visually see the delay. 

Note the use of the odom smoother in the controller server https://github.com/ros-navigation/navigation2/blob/main/nav2_controller/src/controller_server.cpp#L226 you may also consider looking at those parameters as well. 
--
author:	decwest
association:	contributor
edited:	false
status:	none
--
Sorry for my late reply, and thank you very much for mentioning me.

**Regarding the integration of DWPP** @SteveMacenski

I strongly wish for this integration. In fact, I had been planning to reach out myself, so I am truly delighted and honored that you brought this up.
I have sent the PDF of the paper to the email address listed on your GitHub profile. I would be grateful if you could kindly take a look.

The advantages of introducing DWPP for robot behavior are as follows:
By computing the velocity that follows the path optimally under velocity and acceleration constraints, the tracking error is reduced. This effect is especially significant on paths with large curvature, where the method generates smoother behaviors with less overshoot, by appropriately reducing linear velocity while turning.

**Regarding OPEN_LOOP / CLOSED_LOOP**
Since there is inevitably a delay between when the velocity command is published and when the robot’s joints rotate to reflect that velocity, I think that in CLOSED_LOOP mode there may be cases where proper acceleration and deceleration cannot be achieved.
For proper acceleration and deceleration, it is necessary that the previous velocity command has been realized and reflected in the /odom when the next command is computed. Therefore, I suspect that the higher the publish frequency of the velocity command, the more difficult it may become to achieve appropriate acceleration and deceleration.
--
author:	SteveMacenski
association:	member
edited:	false
status:	none
--
I'm looking over your paper and let me know if I'm summarizing this correctly - which might be wrong: 

It looks at the window of translational and angular velocities that meet acceleration constraints. It does not simply threshold the velocities down to something attainable but will actually slow a robot's translational speed proactively when approaching a steep turn so that it can more accurately track the path with the limits on the rotational velocity? 

It does not deviate from the theoretical basis of PP techniques in that we still have:

```
angular_vel = linear_vel * curvature;
```

Just the selection of the translational velocity is selected such that acceleration limits are respected - even if that means proactively slowing down before a large turn. I'm not sure this is what I get from skimming the paper, but that is what I would intuit should be done.

---

Would you be open to opening a draft PR implementing this feature into RPP controller? It should be parameterized to enable or disable just like we have for all the other PP widgets (which makes it easy to benchmark PP, APP, RPP, etc since you can toggle some switches and get all of them).
--
author:	decwest
association:	contributor
edited:	false
status:	none
--
Thank you for reading my paper. Your understanding is perfectly correct.
Regarding proactively slowing down before a sharp turn, I described a concrete example where DWPP slows down before the sharp turn while conventional PPs do not:
https://github.com/Decwest/nav2_dynamic_window_pure_pursuit_controller/blob/main/algorithm.md#:~:text=Difference%20of%20actual%20velocity%20realized%20by%20conventional%20PP%20variants%20and%20DWPP

Of course, let me implement it and create a PR!

To start, should I begin with Humble support by branching off from the [humble](https://github.com/ros-navigation/navigation2/tree/humble) branch and then create a PR targeting the humble branch? (Support for other versions can be added easily.)
--
author:	SteveMacenski
association:	member
edited:	false
status:	none
--
Usually we start with `main` and then backport as needed. Jazzy/kilted should be an easy auto-backport and Humble might require a little bit of manual effort due to the changes in the software over time. But if we start with `main` then you should get 3x distros pretty easily. 

P.S. I'm not committed to it, but I would be open to considering a rename of the controller in this case. Maybe something more generic since we now support so many PP variants within it (and more improvements in RPP than were listed in the original paper). 
--
author:	decwest
association:	contributor
edited:	false
status:	none
--
Thank you for your guidance. Then I will start with the main branch.

Improving the name sounds good. Indeed, since it includes many methods, I agree with giving it a more general name. Unified (or Enhanced) Pure Pursuit…? There is room for discussion:)
--
author:	decwest
association:	contributor
edited:	false
status:	none
--
@SteveMacenski I have implemented DWPP and created a pull request to the main branch. I would appreciate it if you could review it when you have time. Thank you in advance.
--
author:	SteveMacenski
association:	member
edited:	false
status:	none
--
@mini-1235 how do you feel about this ticket ?
--
author:	mini-1235
association:	collaborator
edited:	false
status:	none
--
This is something I want to test together with https://github.com/ros-navigation/navigation2/issues/5248#issuecomment-3326729096, but I haven’t had much time recently. Feel free to close this one, I will file a new ticket with more details or open a PR if I find anything
--
