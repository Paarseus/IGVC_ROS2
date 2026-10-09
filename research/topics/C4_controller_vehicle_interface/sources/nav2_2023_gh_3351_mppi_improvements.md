# navigation2 #3351: Potential MPPI Controller Improvements
URL: https://github.com/ros-navigation/navigation2/issues/3351
Author: SteveMacenski  Created: 2023-01-06T00:21:53Z  State: open  PR: false

General improvements
- [x] Add in some hand-crafted trajectories into the mix via https://arxiv.org/pdf/2401.09241 (i.e. drive to goal, stop, slow, fast, even other algorithms perhaps, etc with samples)

Improved smoothness options
- [x] ~Test using a Log-Normal sampling distribution as established in Log-MPPI paper. Handles less noise for similar exploration of space, reducing sampling noise needing to process.~
- [x] ~Add in the derivative SMPPI term (mutually exclusive with 1% sampling). Attempts to smooth using differential changes in control~
- [x] Mostly fine, just needs minorly better path post processing - increase the SG window size or order, recursively call it, or consider other options
- [x] ~MPPI: Low-frequency distribution sampling to create smoother trajectories https://github.com/ros-navigation/navigation2/issues/5973~ No improvement https://github.com/ros-navigation/navigation2/pull/6151
- [x] ~Adding a low-pass filter on each iteration update~ https://github.com/ros-navigation/navigation2/pull/6149
- [ ] Use the described Tsallis VI variation described below to weight only the best samples in averaging

Improvements in speed potentially
- [ ] GPU for trajectory rollouts or critic analysis
- [ ] Precomputation and otherwise optimization in a specialized collision checker object


New critics
- [ ] Rate of change of wz to penalize noise `|wz(i) - wz(i-1)|`. Probably could be applied to translational velocities too in another way to incentivize commitment to the current action. Maybe not just 2 timesteps but looking at the next in the future and making sure it continues to accelerate / keep rough velocity and not slow down for jerky behavior. Probably actually look at derivatives then. 
- [ ] Add acceleration constraints to the constraint critic
- [ ] Track path with time information to keep up with a trajectory https://github.com/ros-navigation/navigation2/issues/4155
- [ ] Track discrete waypoints rather than a path for either stand-alone sparse routes or viapoints of particular interest to achieve accurately https://github.com/ros-navigation/navigation2/issues/5502 
- [x] Get around lager obstacles blocking the route https://github.com/ros-navigation/navigation2/pull/6144

---

(supporting std change over prediction horizon)

![Image](https://github.com/user-attachments/assets/abee7488-2d47-437f-a07a-0dc5c5db471b)


---
## Comment by oswinso (NONE) at 2023-03-04T08:45:24Z

You may want to consider [Tsallis VI-MPC](https://arxiv.org/pdf/2104.00241.pdf), an MPPI variant which uses the Tsallis divergence instead of the KL divergence. As noted in https://github.com/artofnothingness/mppic/pull/99, tuning lambda can be difficult when the costs varies across a large range in different scenarios. While [adaptive temperature](https://arxiv.org/pdf/2210.03512.pdf) methods can be used to resolve this issue, an easy-to-implement fix is to switch to using the Tsallis divergence instead of the KL divergence used in the original work.

The Tsallis divergence is a generalization of the KL divergence that allows it to "interpolate" between the MPPI weighting scheme (i.e., exponential) and the CEM weighting scheme (i.e., maintaining a fraction of elite samples). 

<p align="center">
<img src="https://user-images.githubusercontent.com/4125463/222878521-329c6abe-a099-43fd-ad99-eaf0191e43df.png" height="200">
</p>

and can be implemented by replacing the weight $\exp(-\frac{1}{\lambda} S(V))$ with

$$\begin{equation}
\begin{cases}\exp\left(\frac{1}{r-1} \log(1 - \frac{S(V)}{ \tilde{S} }) \right) & S(V) \leq \tilde{S} \\ 
0 & S(V) > \tilde{S}\end{cases}
\end{equation}$$

where $\tilde{S}$ is defined as some quantile of the cost of all rollouts.

Intuitively, compared to MPPI, this allows you to ignore a fraction of the worst samples _without_ over-emphasizing the best performing samples, which was previously not possible. In experiments, we found this added flexibility resulted in smaller variances compared to MPPI in difficult situations.

---
## Comment by SteveMacenski (MEMBER) at 2023-03-06T20:45:12Z

Hi, thanks for the note, I have not run into that yet! I have a couple of follow-up questions on the systems side (since that's really the world I come from, lesser the optimal control world :smile: )
- I'm taking from your description that it allows us to ignore a portion of the worst samples as scored by our critic functions. Does $\tilde{S}$ correspond to something physical or mathematical someone can intuit about, or is it just of a hyper-parameter not rooted in much?
- If we ignore some subset of the samples, what is that buying us, specifically? I intuitively understand that if we ignore even more so poorly performing trajectory samples that we're likely to have a better overall outcome, but from your experiments, what does "better" mean? I'd like to understand what problem this addresses so I can have it binned in my mind properly (e.g. just more optimal, more smooth, less jitter, etc). That way I can best understand what I'm evaluating and have this option described to users that if you have problem X consider doing Y. 
- What are some of the reasonable values of `r` and $\tilde{S}$ ? `r` derives back from lambda, so I'd probably just keep the `lambda` parameter for simplicity (unless that's not a good idea?)



---
## Comment by oswinso (NONE) at 2023-03-07T16:11:15Z

### 1:
Although $\tilde{S}$ can have a physical meaning, I think it's best to think of it as a hyper-parameter in the same way that $\lambda$ in MPPI has a physical meaning but for practically speaking it's just a hyperparameter. One way of understanding the meaning of the parameter is that it limits the contribution of small probabilities.

### 2:
One of the benefits we found from VI-MPC was that the variance of the cost of the optimal control was smaller. The tradeoff in MPPI for $\lambda$ is typically that:

1. Lower temperatures basically result in picking the rollout with the lowest cost at the cost of a high variance solutions (i.e., since no averaging is done, the output trajectory can look very noisy and jump all over the place from timestep-to-timestep). Moreover, the solution may become more susceptible to model errors if the (singular) lowest cost trajectory exploits a region of the state-space that has incorrect dynamics. The SV filter can be seen as a hack to try and reduce the variance from having a small temperature.
2. Higher temperatures results in more averaging of rollouts. However, this limits performance, since samples that have low cost have similar weights to samples that have high cost.
    
Ideally, we would be able to pick (multiple) rollouts that have a low cost *without* also averaging over too many of them in a way that hinders performance (e.g., not being able to achieve full speed). Ignoring the worst $k$ samples provides a way for the algorithm to avoid "averaging over too many bad trajectories" but still be able to "average over enough of the good trajectories" to be robust to noise and model errors.

### 3:
A robust way to pick $\tilde{S}$ is to specify it as some quantile of all the costs. i.e., only some fraction of the "elite" samples will have non-zero weight. This probably depends on how many samples are used. In the planar navigation (2D double integrator ) example in the paper, the elite fraction was chosen to be $0.07$ with a sample size of $256$, i.e., only the best $18$ trajectories have non-zero weight. In the humanoid control example, the elite fraction was $0.02$ for $1024$ samples (i.e., best $20$ trajectories have non-zero weight).

I agree that $r$ plays the same role as $\lambda$ and so $\lambda$ can be kept. A reasonable range would be something like
$$\frac{1}{r-1} \in [\frac{1}{0.8}, \frac{1}{5}]$$

---
## Comment by SteveMacenski (MEMBER) at 2023-03-07T18:12:00Z

You've made a good case for this, I will update my checklist above to investigate this solution for MPPI. While its not so large it prevented me from releasing the controller and feeling that its ready for use, there is some jitter in the output path that could be reduced or removed if we can either (a) reduce the noise characteristics which still results in exploration of the control space or (b) combine results in a way to better average smoothly. (a) can be done using Log-MPPI (or another similar formulation of changing sampling mechanics) and (b) what you propose sounds very good as well. 

I don't suppose this is something you'd be interested in contributing? I know there are definitely some users out there that would be more than happy to test it for us to let us know the results on actual robot hardware deployed. If not, its something I'd probably start looking into next month after I finish my current project set. 



---
## Comment by oswinso (NONE) at 2023-03-07T18:20:09Z

Id actually be happy to contribute, but I’m not sure how to run MPPI and benchmark improvements vs the current version. I tried running on the turtlebot sim using the default bringup config but using MPPI for the FollowPath controller, but I got errors regarding the global path having an empty frame. Is there some standard setup for testing MPPI (in sim at least)?

---
## Comment by SteveMacenski (MEMBER) at 2023-03-07T18:27:00Z

On reasonable values - we do typically in the thousands of trajectory samples (between 1000-2000). 

A piece of confusion that I'm sure the paper would address but since you're here, its probably better to simply ask: 

For the piecewise function, I assume we apply the `exp(...)` or `0` based on a trajectory-by-trajectory basis' value of `S(V)`? I haven't carefully read over the manuscript yet, but wouldn't that make $\tilde{S}$ dependent on the actual values of candidate trajectory costs in a given cycle - it doesn't appear to me to be a percentage ratio. E.g. $\tilde{S}$ is a normalization factor in the `log()` and the inequality is talking about actual cost values, not relative ratios. It seems to me that 2-7% needs to be applied relative to `<something>` to create $\tilde{S}$. The lowest trajectory cost, perhaps? But then that would consider trajectories only within 2-7% of that trajectory's cost -- not considering 2-7% of all possible trajectories. 

> I tried running on the turtlebot sim using the default bringup config but using MPPI for the FollowPath controller, but I got errors regarding the global path having an empty frame. Is there some standard setup for testing MPPI (in sim at least)?

What errors? I use the `ros2 launch nav2_bringup tb3_simulation_launch.py` almost daily. I've often tested using the exact configurations under the `FollowPath` namespace from the documentation https://navigation.ros.org/configuration/packages/configuring-mppic.html#example.




---
## Comment by oswinso (NONE) at 2023-03-08T02:30:37Z

> Wouldn't that make $\tilde{S}$ dependent on the actual values of candidate trajectory costs in a given cycle

Yes, you are correct that $\tilde{S}$ varies each optimization iteration. The "fraction of elite samples" is the tuned hyperparameter, while $\tilde{S}$ is the threshold cost that is recomputed each cycle using all the trajectory costs.

If we want to find $\tilde{S}$ such that only the $q\%$ best samples have non-zero weights, we simply need to compute the $q$% quantile of the costs. For example, if we want only $10\%$ best samples, then we compute the $10\%$ quantile of all the costs (e.g., via [xt::quantile](https://xtensor.readthedocs.io/en/latest/api/xsort.html#_CPPv4I000EN2xt8quantileEDaRR1ERK1PNSt9ptrdiff_tE1T1T)).

> What errors? I use the `ros2 launch nav2_bringup tb3_simulation_launch.py` almost daily.

Interesting. Do you have a config file you use with `tb3_simulation_launch.py`? The `PathHandler::transformPose` gives errors when called from

https://github.com/ros-planning/navigation2/blob/34f18d9222e33a2fddb2e45653cd5c0ef7b3bb11/nav2_mppi_controller/src/path_handler.cpp#L75-L78

Print debugging shows that `global_plan_pose->poses[0].header.frame_id` is empty, which is what I think causes the error, but I'm not sure why the `frame_id` would be empty...

**EDIT**: I'm not sure what the contract for the `Planner` and the `Controller` are w.r.t. the `frame_id` for individual poses in the `nav_msgs::Path`, but it seems like the `NavfnPlanner` doesn't set the `frame_id` of the individual poses of a plan

https://github.com/ros-planning/navigation2/blob/34f18d9222e33a2fddb2e45653cd5c0ef7b3bb11/nav2_navfn_planner/src/navfn_planner.cpp#L422-L430

even though `MPPIController` does use the `frame_id` of individual poses in `TransformPose`  

https://github.com/ros-planning/navigation2/blob/34f18d9222e33a2fddb2e45653cd5c0ef7b3bb11/nav2_mppi_controller/src/path_handler.cpp#L127-L146

---
## Comment by SteveMacenski (MEMBER) at 2023-03-08T20:09:32Z

> If we want to find $\tilde{S}$ such that only the $q\%$ best samples have non-zero weights

I think an important question is why should we care about including the top `N` samples (10, 20, 100, etc) instead of including any number of samples within the general vicinity of the best scored sample (e.g. `best_score * 1.05` for all within 5%)? Is there some basis in theory about why it is we want to create a static number set of trajectories to apply the function to instead of using the "great" ones of any number? It seems like that might have better performance when there are many closely grouped. Though the counter situation is where there's 1 great outlier we don't average with any others. So, I'm not arguing necessarily for doing that - but it would certainly be much more computationally simple (e.g. faster since we don't need to both sort and find the quartiles) and the parameter would be more easy to tune and understand the exact consequences of from an end-user's perspective - so its worth asking the question. 

> Do you have a config file you use with tb3_simulation_launch.py? 

The default bringup is what I dogfood so I see what users see. It loads the config file from `nav2_bringup/params/nav2_params.yaml` which I modify by replacing the content under `FollowPath` with those in https://navigation.ros.org/configuration/packages/configuring-mppic.html#example. I also modify the frequency to `30hz` from `20hz`, but that's a performance optimization thing, shouldn't effect whether errors appear or not. 

> Print debugging shows that global_plan_pose->poses[0].header.frame_id is empty

That's not overly surprising that a `nav_msgs/Path` from a planner doesn't contain the header information in each individual pose marker since the path message itself has that at a higher level (that's really more there in case the path represents a trajectory). 

Edit:  I just grabbed `main`, modified the `nav2_params.yaml` file as I described above for MPPI, and ran the system and its broken you're correct (!) 

@doisyg it looks like an issue was introduced in https://github.com/ros-planning/navigation2/pull/3425 - how did your testing not pick that up? Edit2: Oh I know how, I actually implement the header frames in the Smac Planners that you're using. Got it. Anyhow, easy enough fix. 

```
[component_container_isolated-5] [ERROR] [1678306743.643081546] [controller_server]: Exception in transformPose: Invalid argument "" passed to lookupTransform argument source_frame - in tf2 frame_ids cannot be empty
```

I patched it here: https://github.com/ros-planning/navigation2/pull/3458 will backport to Humble as well once merged. Will be merged within an hour and you should be able to move on from there.

---
## Comment by doisyg (CONTRIBUTOR) at 2023-03-08T20:56:40Z

> @doisyg it looks like an issue was introduced in #3425 - how did your testing not pick that up? Edit2: Oh I know how, I actually implement the header frames in the Smac Planners that you're using. Got it. Anyhow, easy enough fix.
> 
> ```
> [component_container_isolated-5] [ERROR] [1678306743.643081546] [controller_server]: Exception in transformPose: Invalid argument "" passed to lookupTransform argument source_frame - in tf2 frame_ids cannot be empty
> ```
> 
> I patched it here: #3458 will backport to Humble as well once merged. Will be merged within an hour and you should be able to move on from there.

My bad, sorry, yes tested with Smac planners but not NavFn.

---
## Comment by SteveMacenski (MEMBER) at 2023-03-08T21:04:30Z

No worries, simple fix :smile: 

---
## Comment by oswinso (NONE) at 2023-03-09T08:49:57Z

@SteveMacenski Just curious, is there a way to see what the "jitter in the output path" you mentioned looks like? In some sense, a control trajectory that is more "optimal" may actually enhance the "jitter" that you see in the output path if the jitter comes from the cost function (and hence the optimal control) jumping from timestep to timestep, and you may actually want the optimizer to be "less optimal" for a "smoother path".

---
## Comment by SteveMacenski (MEMBER) at 2023-03-13T21:03:28Z

That is a very good point. I'm thinking this jitter is the result of the sampling mechanics. I think this because when I reduce the noise's `std`, I see smoother actions, so I figured the optimal trajectory had some of those noises in there. But I think that could also be from the cost functions as you suggest and is worth some analysis.

I can say from experience charting the controls while tuning the cost functions, the cost functions _can_ cause jitter (particularly the obstacle critic) but largely modifying them did not change the noisy characteristics of the trajectory (again, with the exception of the obstacle critic, which is somewhat unsmooth). I think it is worth some analysis on my side to make sure that I isolate the system away from the critics and see if the reason is more to do with the objective functions than the sampling or divergence used. I don't have defensible metrics or experiments off hand right now, but the impression left upon me after tuning the critics was that the remaining noise wasn't largely due to critics being added/removed, but to the system processes at hand. That impression could be wrong though since I wasn't thinking in those terms, so I'm not saying that with a high degree of confidence. 

Though, if changing to the Tsallis divergence as you propose makes things smoother, that would be a marker that at least some of the lack of smoothness is due to KL and/or the necessarily low `lambda` needed to work well with it. I chart using rqt plot the `nav_cmd_vel` coming out of the controller and you can see the jitter in the linear / angular velocity plotted in real-time. I added some additional analytics on variance and such but you can get a visual confirmation via `rqt` pretty quickly if it helps significantly or not (and high level tuning of the hyper parameters). 

Did you get things working after I fixed the path handler issue? I don't suppose you tried with Tsallis?


---
## Comment by mikeferguson (CONTRIBUTOR) at 2023-11-10T00:17:35Z

Another suggestion for improvement here: in the documentation - For each of the critic sections, add a one sentence summary of what the critic is supposed to do. I realize most are self explanatory (e.g. "Goal Angle Critic"), but from just the docs I'm not entirely clear on what "Prefer Forward Critic" actually does (it prefers forward motion? it prefers a forward heading?).

---
## Comment by SteveMacenski (MEMBER) at 2023-11-10T04:25:21Z

@mikeferguson I'll do you one better: https://github.com/ros-planning/navigation2/issues/3951

---
## Comment by RBT22 (CONTRIBUTOR) at 2023-11-30T12:31:23Z

Do you have a specific approach in mind for incorporating the acceleration constraints? Any assistance or guidance on this would be greatly appreciated. @SteveMacenski 

---
## Comment by SteveMacenski (MEMBER) at 2023-11-30T19:46:16Z

There are a few places: 
- Add penalties in the ConstraintCritic for violating acceleration limits
- Add the acceleration thresholding of the final trajectory in motion model's `applyConstraints` used in `applyControlSequenceConstraints`

These get started, but are probably not sufficient for more "hard" constraints like acceleration. Think of these steps as incentivizing the right thing. We should also:
-  Attempt adding the acceleration constraints in the motion model `predict` function. 
 
I'll note that previous attempts to take the randomly sampled trajectories and threshold them up front before going into the critic functions caused some issues. We used to threshold max velocities and resulted in us never being able to achieve the maximum velocity since samples near the limit were being cut off and in the averaging of the softmax function the final trajectory never got up to the final limit. That's why the Constraint critic was added and we removed the velocity constraint clipping to the final trajectory's output.

However, the predict function is supposed to according to the MPPI theory be used to use a dynamics and/or AI model to take the controls and convert them into vehicle velocities - so I think for acceleration constraints specifically that is perfectly A-OK. But I did want to note that we should do some testing to make sure we can actually achieve the max acceleration limits in that case, or close enough to them that its OK due to this prior experience. 

@RBT22 is this something you're open to contributing? I don't think its actually all that difficult, but some careful testing and validation will take a few hours

---
## Comment by RBT22 (CONTRIBUTOR) at 2023-12-06T09:32:01Z

I tried to do some experiments following your notes, but encountered a few challenges. I'd appreciate your input on the following observations:

- Adding thresholding to the applyConstraints might not ensure that cmd_vel values won't jump. This is because the first value of the trajectory comes from the previous one with added noise.

- When adding constraints to the predict function to prevent jumps in velocities starting from the first value (odometry), it renders the critic useless and it doesn't model the real behavior.

I also attempted modifying the predict function to allow a jump from the odometry value but not after, modeling the final applyConstraints. In this case the critic could be useful, but I could not get this to work.

---
## Comment by SteveMacenski (MEMBER) at 2023-12-06T20:08:26Z

> Adding thresholding to the applyConstraints might not ensure that cmd_vel values won't jump.

Think of the critics function of Constraints as weighting optimal samples away from using ones that invalidate acceleration limits -- as a **soft constraint**, the carrot before the stick. So once we apply the softmax function, we should be approximately already following the acceleration limits, assuming that the critic's weight it sufficiently high. 

That's a soft constraint that might still have some error outside of the acceleration bounds when combined with other critics incentivizing their own behaviors. That's where the **hard constraint** of `applyConstraints` comes in -- the stick after the carrot. We apply the acceleration constraint to the whole trajectory to guarantee that it is fully dynamically feasible -- just like with max velocity:

```

void Optimizer::applyControlSequenceConstraints()
{
  auto & s = settings_;

  if (isHolonomic()) {
    control_sequence_.vy = xt::clip(control_sequence_.vy, -s.constraints.vy, s.constraints.vy);
  }

  control_sequence_.vx = xt::clip(control_sequence_.vx, s.constraints.vx_min, s.constraints.vx_max);
  control_sequence_.wz = xt::clip(control_sequence_.wz, -s.constraints.wz, s.constraints.wz);

  motion_model_->applyConstraints(control_sequence_);
}
```

So we send the first new velocity, which we know is valid from the current robot's state including velocity we initialized with and onward we go round-n-round. I could be wrong here on some subtle detail, but I did this exact thing with the velocities and thought very closely about it, so I'm reasonably confident this should work fine. 

Now, if we also add in the acceleration constraints into the prediction function (which is kind of what its there for TBH), we even further reduce the impact the soft constraints in the Critic Constraint objective function has -- to the point that its probably not even necessary. But I don't want to say _its not necessary_ until we test and my hypothesis is true. 

Its still possible to have the infeasibility introduced from the softmax function since its combining multiple trajectories together, but that should be really limited and the `applyConstraints` will project that into the feasible space easily. It technically modifies the optimal trajectory _slightly_ but empirically its works very well & promises then that the next cycle starts with something totally feasible to keep the problem stable. This is what is already happening today with max velocities and I've spoken at length with academics in this area and referenced the appropriate papers, so trust me that this is expected and normal. 

 > When adding constraints to the predict function to prevent jumps in velocities starting from the first value (odometry), it renders the critic useless and it doesn't model the real behavior. I also attempted modifying the predict function to allow a jump from the odometry value but not after, modeling the final applyConstraints. In this case the critic could be useful, but I could not get this to work.

I suppose I don't understand this comment. I'm representing this logic as a loop because I haven't thought about the vectorized version for a quick remark:

```
for every batch in batch_size
  my_batch = batch[batch_idx]
  curr_sample = my_batch[0]
  for every sample in my_batch
    next_sample = my_batch[i++]
    // Check if transition from curr_sample -> next_sample meets dynamic limits 
    // If not, threshold it -- ie next_sample = clip(next_sample)
    curr_sample = next_sample
```

The first value is already set to the[ robot's state which is used to project from](https://github.com/ros-planning/navigation2/blob/main/nav2_mppi_controller/src/optimizer.cpp#L254-L259). So when [you then predict](https://github.com/ros-planning/navigation2/blob/main/nav2_mppi_controller/src/optimizer.cpp#L262-L266), you're populating `vx` `vy` `wz` based on the control (`c*`) [generated from the last trajectory + noises](https://github.com/ros-planning/navigation2/blob/main/nav2_mppi_controller/include/nav2_mppi_controller/motion_models.hpp#L56-L57).

The current implementation is just a pass through, we set cv* to v* and move on without applying a dynamics model. This is exactly the place for the dynamics model to translate control requests into the vehicle's actual responses to those requests -- starting from `state.vx(0)` which is the robot's current speed. You should be able to accomplish this comparing the control velocities sequentially and thresholding changes to within acceleration limits given a constant `model_dt`. In fact, if you simply multiply the acceleration / deceleration constraints by `model_dt`, then you'll have the exact velocity bounds and it is literally just clipping the `i+1`th based on `i`'s valid. and so on.

---
## Comment by RBT22 (CONTRIBUTOR) at 2023-12-11T11:07:27Z

Thanks a lot for the detailed reply!

I'm still grappling with some aspects, as I've noticed issues with the first control value. While addressing velocity constraints, it was feasible to clamp it in the applyConstraint without requiring knowledge of the previous state, in this scenario, it seems more challenging for me.

> So we send the first new velocity, which we know is valid from the current robot's state

How I see, the first value in the control_sequence might not be guaranteed to be valid, considering that odometry only extends to the state's `v*` and not to the `c*`. The control values are derived from the [previous commands with a noise](https://github.com/ros-planning/navigation2/blob/113564965f54009686d521902ff3fcc9d101c5b5/nav2_mppi_controller/src/noise_generator.cpp#L65). 

Could you help me understand if there's something crucial I might be overlooking? Appreciate your guidance!

---
## Comment by SteveMacenski (MEMBER) at 2023-12-11T19:38:22Z

`v*` are set as the robot's actual state: https://github.com/ros-planning/navigation2/blob/main/nav2_mppi_controller/src/optimizer.cpp#L254-L259. The `v*` for the non-first bin are set by the `predict` function, given a set of controls (`c*`) what is the outcome of the robot. Those `v*` are then used for projecting the velocities into trajectories with poses & then scoring the critics. 

[`v*` is also used for the actual velocity sent to the robot base](https://github.com/ros-planning/navigation2/blob/main/nav2_mppi_controller/src/optimizer.cpp#L389-L403). I say these things to illustrate to you with specific evidence (rather than in generalities) that `c*` are essentially unused after we predict `v*` from `c*` using the dynamics model in `predict(...)`. Currently that's a pass through, but adding the acceleration constraints or other dynamic constraints is the precise purpose of this location. 

The only place [that isn't true is here](https://github.com/ros-planning/navigation2/blob/main/nav2_mppi_controller/src/optimizer.cpp#L380-L384), which I'm suspicious of now that we're talking about it that this shouldn't actually be `c*`, but worked fine until now since `predict` is just a pass through model -- but I can look into that later. I think `c*` may be correct there when we're talking about situations that you're commanding your robot to do Operation X which is difficult to model where `cx` creates `vx` of interest. For example, skidding on a dirt road or drifting - its not necessarily linear to ask the robot to do X in order to create Y behavior. If we're applying operations that are **setting limits** like what we are doing here, when it should be `vx` based on the thresholded constrained values - we're asking the robot to do X to get exactly X behavior. We're not (currently) applying this to crazy high speed applications with sliding, so I think that's a fair assumption for the purposes of this stack. 

I think from those links you should be able to prove to yourself that  all `v*` 0th values are the actual robot state, and then all other subsequent ones are w.r.t. that one and when applying dynamic constraints in `predict` are feasible. 

> How I see, the first value in the control_sequence might not be guaranteed to be valid, considering that odometry only extends to the state's v* and not to the c*. The control values are derived from the [previous commands with a noise](https://github.com/ros-planning/navigation2/blob/113564965f54009686d521902ff3fcc9d101c5b5/nav2_mppi_controller/src/noise_generator.cpp#L65).

That happens before the dynamics models is applied in `predict`. That's just generating the noised controls, that are then applied with the dynamics model, then scored. Maybe looking at the order of operations will clear that up :smile: 

---
## Comment by SteveMacenski (MEMBER) at 2024-01-04T02:05:41Z

@RBT22 does that make sense?

---
## Comment by RBT22 (CONTRIBUTOR) at 2024-01-08T13:01:06Z

@SteveMacenski I'm really sorry for not being really active on this topic. Towards the end of last year, I had to address some pressing issues, and honestly I was a bit even more confused after I read your message.

> [v* is also used for the actual velocity sent to the robot base](https://github.com/ros-planning/navigation2/blob/main/nav2_mppi_controller/src/optimizer.cpp#L389-L403). I say these things to illustrate to you with specific evidence (rather than in generalities) that c* are essentially unused after we predict v* from c* using the dynamics model in predict(...). Currently that's a pass through, but adding the acceleration constraints or other dynamic constraints is the precise purpose of this location.

Contrary to this, I think the opposite of this is true actually. After using the state's `v*` values in the critics, they are no longer needed. The link you provided is using the `control_sequence_.v*` not the state's `v*` which are coming from the state's `c*` values.

The sequence of operations follows these major steps:
- The predict function provides the `c*` values. 
- `integrateStateVelocities` rollouts `v*` to poses. 
- `critic_manager_.evalTrajectoriesScores` determines the `costs_` from the `v*` values. 
- `updateControlSequence` calculates the `control_sequence_.v*` from its previous value, the `costs_` and the state's `c*`. 
- Starting from here the `control_sequence_.v*` values are used in `applyControlSequenceConstraints`, `savitskyGolayFilter` and `getControlFromSequenceAsTwist`.

---
## Comment by SteveMacenski (MEMBER) at 2024-01-08T20:00:47Z

No need for apologies, just following up :-) I know things come up

> The predict function provides the c* values.

Nope, lets look at a snippet from the predict function

```
    xt::noalias(xt::view(state.vx, xt::all(), xt::range(1, _))) =
      xt::view(state.cvx, xt::all(), xt::range(0, -1));
```

We're setting the value of `xv` for all batches (first `xt::all()`) from ranges in timesteps from `1-N` (skipping `0` which is set previously as the actual robot's state). We're setting it to the  value of the `cvx`, or the control generated by the noise trajectories from all until the last - 1. 

You can also prove this to yourself looking at the noise generator:

```
  xt::noalias(state.cvx) = control_sequence.vx + noises_vx_;
```

We're setting the `cvx` to the noise values. `vx` appears nowhere in the object. The `c`s are the controls. The `predict` model is what converts the controls into the body's actual velocity outputs if those controls were applied - e.g. `v`s. 

> integrateStateVelocities rollouts v* to poses.
> critic_manager_.evalTrajectoriesScores determines the costs_ from the v* values.


Correct, because these methods is called after `predict` . So we're rolling out the body's velocities into poses for use in scoring. 

> updateControlSequence calculates the control_sequence_.v* from its previous value, the costs_ and the state's c*.

I mentioned before, but I know this is all complicated, but this is actually an area I want to explore more later. I'm not totally convinced that we did the 100% right thing here for our applications. For now, regardless, its fine since we just have the pass-through `predict` dynamics. I suppose its correct to use `cvx` assuming the `predict` dynamics are actually accurate that applying `cvx` will create `vx` if applied. That's true if it models that actual robot behavior doing wacky stuff like skidding on dirt roads. For our application that we want to artificially limit the dynamic model into bounds of behavior we want to see happen, I think that should be swapped to `vx` to represent the limited values. The intent of the predict function is a little different in this case. 








---
## Comment by RBT22 (CONTRIBUTOR) at 2024-01-12T14:22:51Z

> The predict function provides the c* values.

Sorry about this, the `c*` was an unfortunate typo here, I wanted to write `v*`. Your explanation is perfectly clear to me. 

I find your suggestion of using `v*` values as `c*` values a bit unconventional, as it could potentially complicate the integration of real models in the future. I'm not an expert on this, but with a predictive controller, it seems crucial to me to avoid being constrained by a pass-through model in the long term.

Wouldn't it be more prudent to guarantee the validity of the predict function for the specific `c*` values it operates on by imposing restrictions on these generated values?

---
## Comment by SteveMacenski (MEMBER) at 2024-01-12T19:23:51Z

>  it seems crucial to me to avoid being constrained by a pass-through model in the long term. 

Of course, that's what we're discussing right now. Replacing the pass-through model with an actual dynamics model. The MPPI research uses this slot to do things like adding DNNs trained on robot data to convert `c` to `v` using trained dynamics of hard to model situations. That's a fancy way of doing what we're talking about here that's applicable to odd situations like skidding and very high speeds. For low-speed (sub-10 m/s) that we see in AMRs, I assert traditional dynamics models like what we're discussing are good enough. But, the plan would be to make the motion model a plugin if folks wanted to build on it in the future. 


> Wouldn't it be more prudent to guarantee the validity of the predict function for the specific c* values it operates on by imposing restrictions on these generated values?

I believe that breaks the information theoretic constraints. But, lets game this out and tell me if you find a gap:
- Given the explanation avoid, we can agree that `v`s are used everywhere except for the final step of computing the new control sequence from the scored values - where `c` are used, currently
- Right now, `c = v` (shifted by one timestep, but thats an unimportant detail for the sake of argument I think) as a pass through model
- So if we limit the output of `v` by `predict` & use `v` in the compute the new control sequence that should be identical to limiting `c` and continuing to use `c` in compute new control sequence. I imagine you're going to limit `c` by dynamic limits, then apply a control to velocity, so `v` also reflects the changes in `c`

Thus, I think its probably just another way of thinking about the same thing. 

---

I think as I've explained before, but in different phrasing: my concern with using `c` in computing the new control sequence is that `c` is supposed to represent the controls you apply to generate some body motion of `v`. You give robot control `c` and it generates output `v`. Its a real model of what the robot really does. 

For the case of wanting to artificially limit the dynamics computed in the controller is that if you don't constrain <whatever is being used to compute the new control sequence> by the acceleration limits you'd like to enforce, then there's no enforcement of it at the controller level. The outputs are unconstrained. However, you could have something like the `velocity_smoother` below the MPPI controller that could/would. So if paired with the velocity smoother, you could continue to use `c`'s without acceleration limits, knowing `v`s have it applied so the scoring functions used the acceleration-limited `v`s to compute the best `c` (which are then re-squashed by the acceleration limits).  

That seems pretty roundabout given that both the velocity smoother and the controller has access to the same data. And it also wouldn't then actually result in any acceleration limits applied to the output of the controller server if **not** paired with a velocity smoother. If the velocity smoother is going to squash it anyway, why not use the `v`s in computing the new control sequence and just be done with it, with valid outputs directly from the server. 

So, I agree `c` should be used for computing new control sequences when the dynamics represent "I send `c` and I get `v` back, because that's how my robot physically reacts". When we want to limit our robot's acceleration characteristics and not model how it actually reacts, using `v` seems to make sense to me. However, there's an argument to be made that the velocity smoother downstream **creates** the "that's how my robot physically reacts" when in use, but that seems fragile as a general purpose statement to assume everyone, all the time is using the velocity smoother. But, if we can make that assumption, then leaving `c` as is is fine, but I think that leaves easy misunderstandings for users ("MPPI doesn't _actually_ respect my dynamics" since it requires analysis after vel smoothing ) and opens opportunities for more subtle problems with dynamic limits being slightly broken if new odometry information is received by the velocity smoother after the controller server publishes its output. 

^ So... I think that could be perhaps a parameter? But honestly I think for the AMR case, `v` is the right answer. If folks are applying this to more complex dynamics situations, then I agree `c` should be used.  

---
## Comment by boilerbots (NONE) at 2024-01-22T01:04:02Z

I would love to see the acceleration constraints added for MPPI. Also it would be nice to incorporate some velocity saturation controls. A differential robot may not be able to simultaneously achieve max linear when angular velocities are introduced.

---
## Comment by RBT22 (CONTRIBUTOR) at 2024-02-23T14:26:01Z

@SteveMacenski  I read your comment on another issue (https://github.com/ros-planning/navigation2/issues/4126#issuecomment-1954840052) and please don't wait for me. Unfortunately, my current schedule doesn't allow me the time to dedicate to this task. I made some attempts, but none of them were successful. If you see that this can be completed easily, please proceed.

When I initially said I would work on this, it seemed like no one besides my team really cared about this topic, and I had more free time. I genuinely hope I can contribute to something else in the future.

---
## Comment by Ayush1285 (CONTRIBUTOR) at 2024-05-04T22:50:58Z


> There are a few places:
> 
> * Add penalties in the ConstraintCritic for violating acceleration limits
> * Add the acceleration thresholding of the final trajectory in motion model's `applyConstraints` used in `applyControlSequenceConstraints`
> 
> These get started, but are probably not sufficient for more "hard" constraints like acceleration. Think of these steps as incentivizing the right thing. We should also:
> 
> * Attempt adding the acceleration constraints in the motion model `predict` function.
> 
> I'll note that previous attempts to take the randomly sampled trajectories and threshold them up front before going into the critic functions caused some issues. We used to threshold max velocities and resulted in us never being able to achieve the maximum velocity since samples near the limit were being cut off and in the averaging of the softmax function the final trajectory never got up to the final limit. That's why the Constraint critic was added and we removed the velocity constraint clipping to the final trajectory's output.
> 
> However, the predict function is supposed to according to the MPPI theory be used to use a dynamics and/or AI model to take the controls and convert them into vehicle velocities - so I think for acceleration constraints specifically that is perfectly A-OK. But I did want to note that we should do some testing to make sure we can actually achieve the max acceleration limits in that case, or close enough to them that its OK due to this prior experience.
> 
> @RBT22 is this something you're open to contributing? I don't think its actually all that difficult, but some careful testing and validation will take a few hours

@SteveMacenski  Is anyone currently working on this? If not, then I can take up this work.
I just have a doubt about the 2nd point of applying hard constraints on control_sequence.

Since we will already be covering the acceleration constraints in simulating the future velocities using the motion model(in `MotionModel::predict()`), then what's the logic of again applying constraints on control sequences? 
As per my understanding, if we are applying constraints while simulating the dynamics, then it would result in dynamically feasible velocity values and that would result in feasible control values. Therefore, there should be no need to again apply constraints on control values.
It would be great to know your insights on this.

---
## Comment by SteveMacenski (MEMBER) at 2024-05-06T22:25:35Z

> Is anyone currently working on this? If not, then I can take up this work.

Not currently! I'd love the help :-) 

> then what's the logic of again applying constraints on control sequences?

The softmax operation can go beyond those bounds when taking account the scoring. You don't have to take my word for that either, you can print the output trajectories and you should see that happen yourself with slightly infeasible trajectories in some cycles. It should be very small (if at all) if they're being limited on generation, but I believe it will be technically possible to occur. 

---
## Comment by Ayush1285 (CONTRIBUTOR) at 2024-05-10T18:53:02Z

> > Is anyone currently working on this? If not, then I can take up this work.
> 
> Not currently! I'd love the help :-)
> 
> > then what's the logic of again applying constraints on control sequences?
> 
> The softmax operation can go beyond those bounds when taking account the scoring. You don't have to take my word for that either, you can print the output trajectories and you should see that happen yourself with slightly infeasible trajectories in some cycles. It should be very small (if at all) if they're being limited on generation, but I believe it will be technically possible to occur.

I wasn't able to build docker image with main branch due to missing rosdeps, and source installation of all the missing dependencies would be too painful. So I've started with iron branch for now and will get back after few days.



---
## Comment by Ayush1285 (CONTRIBUTOR) at 2024-05-17T19:05:41Z

> > Is anyone currently working on this? If not, then I can take up this work.
> 
> Not currently! I'd love the help :-)
> 
> > then what's the logic of again applying constraints on control sequences?
> 
> The softmax operation can go beyond those bounds when taking account the scoring. You don't have to take my word for that either, you can print the output trajectories and you should see that happen yourself with slightly infeasible trajectories in some cycles. It should be very small (if at all) if they're being limited on generation, but I believe it will be technically possible to occur.

I've tried accounting constraints in the predict function, which works fine in most cycles. But as you said, in a few cycles acceleration violates the constraints. So either we need to penalize the critic cost to reduce this possibility further or to apply hard constraints on the final output velocities. I've created a PR with the initial commit #4352 .

---
## Comment by SteveMacenski (MEMBER) at 2024-05-17T23:39:52Z

Reviewed! Lets follow up in the PR :-) 

