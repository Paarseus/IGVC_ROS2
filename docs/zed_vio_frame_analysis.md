# ZED VIO frame analysis — why we fuse yaw only

**Date:** 2026-05-12
**Author:** investigation triggered by T3 field-test finding #4
([`T3_field_report_2026-05-12.md`](#) — kept in the operator's brain vault, not
checked in)

## TL;DR

The T3 field test logged that the ZED VIO trajectory was the right magnitude
but the wrong **direction** relative to `/odometry/filtered` (the
IMU-driven local EKF). On the surface this looked like a frame-orientation
bug. After source-code review of both `zed-ros2-wrapper` and
`robot_localization`, plus a live capture on the Jetson, the root cause is:

1. The ZED wrapper hardcodes its odom message's `child_frame_id` to
   `<camera_name>_camera_link`.
2. `robot_localization`'s differential-pose path does **not** correct for
   the lever arm (ω × r) between that child_frame and `base_link`.

With our camera mounted 0.68 m forward of `base_link`, every yaw rotation
makes the camera sweep an arc that the EKF reads as base_link translating
sideways. At a typical IGVC obstacle-dodge yaw rate of 0.5 rad/s, that's
~0.34 m/s of phantom lateral velocity continuously fed to Nav2's controller.

**Fix applied:** drop `x, y` from `odom1_config` in `ekf.yaml`; fuse yaw
only. Angular rate is invariant under rigid translation, so the lever-arm
bug doesn't apply to yaw fusion. We keep the only thing ZED was actually
helping with — bias-stable yaw during stationary pauses, where the Xsens
drifted ~6°/min on grass and ZED drifted 0.26°.

No relay node, no extra moving part, no new dependency. One config change.

## Source-code evidence

### ZED wrapper hardcodes child_frame_id

`src/zed-ros2-wrapper/zed_components/src/zed_camera/src/zed_camera_component_main.cpp:1593-1594`:

```cpp
mBaseFrameId = mCameraName;
mBaseFrameId += "_camera_link";
```

…and at `:6354`:

```cpp
odomMsg->child_frame_id = mBaseFrameId;   // camera_frame
```

There is no YAML parameter that changes `mBaseFrameId`. The wrapper publishes
the pose of the camera_link origin in the `odom` frame, with twist expressed
in the camera_link frame.

### robot_localization differential-pose path has no lever-arm correction

`robot_localization/src/ros_filter.cpp` (humble-devel branch). The
differential-pose code path, summarized:

```cpp
// ~3004: compute delta in world frame
pose_tmp.setData(prev_measurement.inverseTimes(pose_tmp));
// ~3013: zero the translation of the target frame transform,
//        apply rotation only
target_frame_trans.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
pose_tmp.mult(target_frame_trans, pose_tmp);
// ~3027: turn delta-position into velocity
double xVel = pose_tmp.getOrigin().getX() / dt;
// ~3056: stamp the synthesized twist as base_link, no further transform
twist_ptr->header.frame_id = base_link_frame_id_;
twist_ptr->twist.twist.linear.x = xVel;
```

Two consequences:

1. The pose message's `child_frame_id` is **ignored** on the pose path —
   only `header.frame_id` (the world frame) is consulted.
2. The synthesized "twist in base_link" is the delta of the camera_link
   origin, not the delta of base_link. No `ω × r` correction is applied
   anywhere in this path.

(For the twist path, `prepareTwist` *does* look up the
`child_frame_id → base_link` transform and applies a rotation — but again,
no lever-arm correction. Same problem, different path.)

## Live verification on Jetson (2026-05-12)

20-second stationary capture from `/zed_front/zed_node/odom` on the live
Jetson stack (29 samples at ~10 Hz):

```text
header.frame_id  : odom
child_frame_id   : zed_front_camera_link        ← confirmed hardcoded value
first pose       : (0.0033, -0.0051, 0.014) m, q ≈ identity
pose covariance  : ~5e-9 to 5e-8 mag           ← very tight, the danger
```

Static TF check (`tf2_echo base_link zed_front_camera_link`):

```text
Translation : [0.679, 0.000, 0.448] m
RPY (deg)   : [0.000, 15.000, 0.000]
```

Matches the URDF mount joint at `avros.urdf.xacro:178`:

```xml
<origin xyz="0.6795 0.0 ${xsens_height - 0.108}"
        rpy="0 ${radians(15)} 0"/>
```

## Quantifying the phantom lateral bias

For a rigid body, the velocity of any point P offset from the base origin
by vector r is:

```
v_P = v_base + ω × r
```

ZED reports v_P (the camera_link velocity), and the EKF interprets it as
v_base. The discrepancy is `ω × r`:

| Yaw rate ω | Lever arm (XY) | Phantom lateral velocity |
|---|---|---|
| 0.1 rad/s (gentle drift) | 0.68 m | **6.8 cm/s** |
| 0.3 rad/s (lane following) | 0.68 m | **20 cm/s** |
| 0.5 rad/s (obstacle dodge) | 0.68 m | **34 cm/s** |
| 1.0 rad/s (sharp turn) | 0.68 m | **68 cm/s** |

For a 0.5 m/s commanded vehicle, a 0.34 m/s phantom lateral bias points
the EKF's velocity vector ~34° off-axis during turns. That matches the
T3.3 cross-track comparison: ZED reported (+1.18, −0.30) m while
`/odometry/filtered` reported (+1.36, −1.29) m over the same window. Same
~1.2 m magnitude, ~30° rotation — exactly the lever-arm signature.

## Why "fuse yaw only" is the right answer

Angular velocity is a property of the entire rigid body. For any two points
P and Q rigidly attached to the same body, `ω_P = ω_Q`. There is no
`ω × r` analog for angular rate.

This means:

- `ω_cam = ω_base` *exactly*, no transform needed.
- `Δyaw_cam = Δyaw_base` over any time interval.
- robot_localization's differential mode on the yaw component is therefore
  exact, regardless of where the sensor is mounted.

By contrast, position deltas of an off-axis point are *not* equal to
position deltas of the base origin during rotation — the lever arm matters
exactly there.

So we fuse yaw, drop x and y, and the math is clean.

## What ZED yaw fusion actually buys us (T3 evidence)

From [T3 field report](#) finding #3:

> T3.2 showed +6° yaw drift end-vs-start in EKF (driven by IMU) while
> ZED VIO only saw +0.26° over the same window.

The Xsens MTi-680G is excellent for orientation **during motion** (GNSS-
aided yaw alignment via course-over-ground), but its yaw can wander a few
degrees during stationary pauses when COG is unavailable. ZED VIO,
running purely on visual features, doesn't have that failure mode at low
motion — it's a useful complementary source there.

This is also the only ZED VIO signal we should trust regardless: position
will accumulate drift faster than the IMU+GPS solution, and twist
covariance is published as zero by the wrapper (uninformative for the
EKF). Yaw is what's left, and it's exactly what helps.

## The alternative we rejected: lever-arm-corrected relay

An earlier worktree iteration added a `zed_odom_base_relay` node that
composed `T_odom_base = T_odom_cam · T_cam_base` and republished with
`child_frame_id = base_link`. It works mathematically and was tested
locally. We rejected it because:

1. It adds a single-point-of-failure node before competition.
2. It does not give us anything `yaw-only` doesn't, since ZED's x/y from
   a forward-mounted camera adds noise faster than signal once the
   xsens+GNSS is already running.
3. One more dependency (`tf2_ros`, `numpy`) on `avros_control`, which
   currently has none beyond `pyserial`.

`Yaw only` is strictly smaller, strictly simpler, and matches the
production-robotics convention of "trust your good IMU+GNSS for state,
trust VIO for orientation rate where it complements." If we later mount a
second VIO sensor at base_link with no lever arm, we'd revisit. Until
then, the one-line change in `ekf.yaml` is the canonical answer.

## Related

- T3 field-test report (operator's brain vault, not committed).
- Memory: `feedback_zed_leverarm_in_rl.md`.
- Affected files:
  - `src/avros_bringup/config/ekf.yaml` (`odom1_config` block + comment).
  - `CLAUDE.md` Known Issues & Fixes table.
