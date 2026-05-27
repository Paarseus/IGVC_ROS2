# Measurement Methodology — 2026-05-27 Session

What every snapshot in this folder is measuring, how it's derived, and what could be wrong. Every per-leg analysis here should be read in context of this doc.

---

## 1. Coordinate frames in use

| Frame | Meaning | Where it comes from |
|---|---|---|
| `map` | Global frame, +X = east, +Y = north (ENU). Origin = navsat datum [42.658430417, -83.241993772, 247.578]. | `ekf_filter_node_map` outputs `map → odom` transform |
| `odom` | Continuous local frame, drifts slowly. | `ekf_filter_node_odom` outputs `odom → base_link` transform |
| `base_link` | Robot body. Yaw=0 in `base_link` = robot's forward direction (per URDF). | URDF + EKF |

When this session reports a robot pose like `(3.5, -2.6, -110°)`, those are **in the `map` frame** because we're reading `/odometry/filtered.pose.pose` which is in `header.frame_id = "map"` per ekf.yaml (`map_frame: map`).

Robot heading "+5.47°" at start means the robot's `+X axis (forward)` was 5.47° CCW from `map` `+X` (east).

---

## 2. Commands sent during tests

### Direct `/cmd_vel` mode (M1b, M1c, M1d, M2 — bypassing MPPI)

```python
twist = geometry_msgs/Twist
twist.linear.x = +0.35   # forward, m/s   (or -0.35 reverse, +0.7 high speed)
twist.angular.z = 0.0    # no commanded rotation
# Published at 20 Hz for the leg duration
# Followed by 2× zero-twist to stop
```

The publisher rate is 20 Hz (50 ms period) so each cycle the actuator gets a fresh command. The actuator's slew-rate limiter then processes the input.

### NavigateToPose mode (M1a)

```yaml
goal:
  pose:
    header.frame_id: "map"
    pose.position.x: start_x + 5.0 * cos(start_yaw)
    pose.position.y: start_y + 5.0 * sin(start_yaw)
    pose.position.z: 0.0
    pose.orientation: <copied from start orientation>
```

This is a position+orientation goal 5 m forward from the robot's current heading, in `map` frame.

---

## 3. Yaw sources (4 sources, each with its own derivation)

### Source A: IMU quaternion (Xsens internal fusion)

**Topic:** `/imu/data` (sensor_msgs/Imu), 100 Hz
**Field:** `orientation` (quaternion)
**Derivation:**
```python
def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)   # radians
```

**What it represents:** Xsens MTi-680G's General_RTK filter combines gyro + magnetometer + accelerometer (and GNSS heading when locked, but we have no RTK) to produce an absolute orientation. The quaternion is the fused output. Drift-free over short windows (10s) but susceptible to magnetometer interference and (per #13 / `feedback_xsens_imu_bias_recovery`) stuck-bias incidents.

**Best for:** short-window absolute heading reference.

### Source B: IMU integrated angular velocity (∫wz dt)

**Topic:** `/imu/data`, 100 Hz
**Field:** `angular_velocity.z` (rad/s about base_link's z-axis, which is up)
**Derivation:** trapezoidal integration over time since the integrator was last reset:
```python
def on_imu(msg):
    t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
    if t_prev is not None and 0 < (t - t_prev) < 1.0:
        imu_wz_int += msg.angular_velocity.z * (t - t_prev)
    t_prev = t
```

**What it represents:** raw gyro yaw rate, integrated. Includes any gyro bias as accumulated drift. Should match the quaternion-derived value within 0.5° per few seconds *if* the bias is well-cancelled by the Xsens internal filter and the gyro is healthy.

**Best for:** detecting whether the Xsens filter is actively correcting the gyro (gap between A and B = correction magnitude).

### Source C: Wheel-derived yaw (∫wheel_odom.wz dt)

**Topic:** `/wheel_odom` (nav_msgs/Odometry), 20 Hz
**Field:** `twist.twist.angular.z`
**Derivation:** same trapezoidal integration as Source B but on the wheel_odom topic.

**What it represents:** `actuator_node` derives this from the per-wheel encoder velocities via the Mandow-corrected skid-steer inverse kinematics:
```
wz = (R_mps - L_mps) / (track_width × wheel_separation_multiplier)
```
where `wheel_separation_multiplier=1.19` per CLAUDE.md. This assumes both wheels have equal traction and the chassis behaves like an idealized skid-steer. Any deviation from that assumption (slip, friction asymmetry, surface roughness) shows up as error in this derived ω.

**Best for:** what the wheels "think" happened. Disagrees with IMU when chassis really rotates without commensurate differential wheel motion (slip), or vice versa.

### Source D: EKF fused yaw

**Topic:** `/odometry/filtered` (nav_msgs/Odometry), 30 Hz
**Field:** `pose.pose.orientation` (quaternion) → `yaw_from_quat()`
**Derivation:** robot_localization's local EKF fuses (per ekf.yaml):
- `/imu/data` orientation (yaw[5]=true) + angular velocity (vyaw[11]=true), with covariances configured in ekf.yaml
- `/wheel_odom` vx[6]=true + vyaw[11]=true with `_twist_cov[35] = 0.0001`
- (ZED VIO yaw[5]=true differential — but ZED is OFF this session)

Per issue #13 (now somewhat re-supported by today's data), the relative covariance weights make the EKF track wheel-derived vyaw more strongly than IMU.

**Best for:** "what does the system as a whole think happened" — but this is the suspect signal we're investigating, not a ground truth.

---

## 4. Δyaw computation (wrap-safe)

For any source pair (start, end):
```python
def wrap(angle_rad):
    return atan2(sin(angle_rad), cos(angle_rad))   # in (-π, π]

dyaw = wrap(end_yaw - start_yaw)   # avoids ±π wrap artifacts
```

Convention: positive = CCW (counter-clockwise viewed from above). Negative = CW.

For integrators (sources B and C), the integrator IS the Δyaw since the last reset — no subtraction needed.

---

## 5. Position + bearing computation

```python
dx = end_x - start_x      # in map frame
dy = end_y - start_y
dist = sqrt(dx² + dy²)
path_bearing = atan2(dy, dx)   # direction of travel in map frame, radians
```

Distance is straight-line start-to-end, not path-length (we don't integrate the trajectory).

---

## 6. Things that could be wrong with the measurements

### Pose source for distance: EKF (Source D)
The "distance traveled" and "path bearing" come from `/odometry/filtered` x/y. If the EKF position is wrong (e.g., because of bad wheel-odom integration in motion), so is our distance/bearing. The clean 3m run showed 89-93% distance delivery; the multi-source run showed 36-67% — wildly different. This could be:
- Robot actually moved less (constrained physically)
- EKF reporting less than reality (under-integration)
- /odometry/filtered timing artifacts in this multi-source script

### Integrator startup gap
The IMU/wheel integrators only accumulate when 2 consecutive messages have arrived. The first message after reset just sets `t_prev`; integration starts on the second. Lost ~10–50 ms at the start of each leg.

### "End" reading is whatever-last-arrived
After commanding stop, we wait 0.5s then read the last cached message of each source. If a topic isn't being received (e.g., transient drop), we read a stale value.

### Wheel-derived ω is Mandow-modeled
`/wheel_odom.wz` is **not** a sensor measurement — it's a kinematic computation from encoder velocity differences. The model assumes Mandow's skid-steer correction is accurate (it's calibrated for symmetric, equal-friction tracks). Any modelling error shows up as wheel-vs-IMU disagreement.

### IMU yaw uses Xsens internal filter
The Xsens General_RTK profile fuses gyro + mag + accel. Magnetometer interference (from motor currents under the chassis) could bias this signal — specifically under motion when motor currents are high. This is the Fix 4 hypothesis.

### No ZED VIO tiebreaker
Cameras off this session. When IMU and wheel disagree, we have no independent visual reference to break the tie. The 4-source consensus from the v2 field plan is reduced to 3 sources.

---

## 7. What's recorded in the bag (for post-session analysis)

Bag path: `/tmp/yaw_diag_20260527_134442` on Jetson.

13 topics (full list in `bags_manifest.md`). The bag has **everything** these in-session snapshots compute, plus a lot more (TF, /cmd_vel, /avros/* full message contents). All in-session numbers should be cross-checked by post-session `extract_bag.py` + `analyze_M1.py` against the bag for confirmation.

---

## 8. What each snapshot doc captures

For each test (M0, M1a, M1b/c retry 3m, multi-source v2, M2/M3 if run):

1. **Sequence**: exact command sent + duration
2. **Per-source raw readings** at start and end of each leg
3. **Computed Δyaw** for each source
4. **Distance + bearing** in map frame
5. **Disagreements** between sources (the diagnostic signal)
6. **Hypotheses** consistent with the data
7. **Bag time-window** so the same data can be re-extracted offline

If a snapshot doc deviates from this methodology (e.g., the curb-contaminated M1b_M1c_snapshot.md), it's flagged at the top.
