# 02 — Drive Kinematics Layer (`actuator_node`)

File: `src/avros_control/avros_control/actuator_node.py` (identical in the repository and on the Jetson).
Parameters: `evidence/deployed_snapshot/src/avros_bringup/config/actuator_params.yaml`.

## 1. What the node does, in order, every 20 ms

| Step | Implementation | Deployed value |
|---|---|---|
| Command selection | Uses `_target_v/_target_w` if either input is fresher than `cmd_timeout_s` | 0.5 s |
| E-stop | Latched by an `ActuatorCommand` with `estop=true`; cleared only by the next non-estop `ActuatorCommand` | — |
| Slew limit, linear | +accel when speeding up, decel otherwise | 0.3 / 1.3 m/s² |
| Slew limit, angular | same cap both directions | 1.2 rad/s² |
| Heading-hold | If \|ω_slewed\| < deadband and \|v\| > 0.02 m/s: ω = kp × (locked yaw − IMU yaw), clamped to ±0.5 × max ω | deadband 0.05 rad/s, kp 1.5, clamp ±0.75 rad/s |
| Inverse kinematics | `v ∓ ω × (track × multiplier) / 2` | 0.7366 m × 1.19 = 0.877 m |
| Units | m/s → motor RPM | 0.01994 m/rev |
| Output | `L<rpm> R<rpm>`, or `S` when both inputs are stale and the slewed command is zero | — |
| Speed clamps | \|v\| ≤ max_linear, \|ω\| ≤ max_angular (applied on input) | 1.5 m/s, 1.5 rad/s |

## 2. Assessment

### 2.1 Correct and consistent with standard practice
- **Inverse kinematics.** The math reproduces `v_slewed/w_slewed` exactly (verified in `docs/motor_sync_logs/REPORT.md`). The `wheel_separation_multiplier` convention matches `ros2_controllers/diff_drive_controller`.
- **Stop behavior.** The `S` command is gated on the slewed values, not the post-heading-hold ω (fix from commit 30d6eb0). Brake idle engages on stale commands.
- **Safety layering.** Host-side timeout (0.5 s), Teensy watchdog (300 ms), SPARK MAX heartbeat timeout (100 ms). Three independent stops.
- **Asymmetric slew.** Decel > accel is the right choice for stopping distance (1.3 m/s² → 0.19 m from 0.7 m/s).

### 2.2 Findings

**D1 — Command priority is not enforced (correctness, High).**
`_on_cmd_vel` and `_on_actuator_cmd` both write the same `_target_v/_target_w`. Freshness only decides whether to drive, not which source wins. When the phone joystick and Nav2 are both active, the target alternates between the two sources message by message (both publish at 20 Hz). The docstring and CLAUDE.md state "actuator_command > cmd_vel", which the code does not implement. E-stop is unaffected, because it is latched separately.
Fix: store each source's target separately and select in `_control_loop` by freshness and priority.

**D2 — Heading-hold overrides small Nav2 turn commands (behavior, Medium, needs data).**
Heading-hold engages for any \|ω\| < 0.05 rad/s, including commands from MPPI. At the 0.7 m/s cruise cap this covers every path with a turn radius above 14 m, and every small heading correction. In that band, MPPI's ω is discarded and replaced by a P-loop on the yaw captured when the lock engaged. MPPI then has to command \|ω\| ≥ 0.05 to be obeyed, which produces a discontinuity at the deadband edge. The actuator's own docstring states that path tracking is MPPI's job. The heading-hold is appropriate for the manual joystick path only.
Evidence needed: bag `/cmd_vel` and `/avros/wheel_debug` (`heading_locked`) during a Nav2 run and count the fraction of time the lock overrides MPPI.
Fix: apply heading-hold only to the `ActuatorCommand` path, or disable it while `/cmd_vel` is the active source.

**D3 — Heading-hold output bypasses the angular slew limit (Low).**
The slew limiter runs before heading-hold, so a yaw-error step (for example an IMU yaw correction) can produce up to 0.75 rad/s immediately. The Teensy ramp (100 RPM per 20 ms ≈ 1.66 m/s² per track) is the only limit left. The IMU is also a single point of failure here: a yaw jump in the Xsens output steers the robot directly (see 04).

**D4 — Angular deceleration uses the acceleration cap (Low).**
`max_angular_accel_rps2` limits both speeding up and slowing down rotation, so rotation stops take the full 1.25 s. This is already noted in CLAUDE.md as a planned change.

**D5 — No desaturation of track speeds (Medium at manual speeds, Low for Nav2).**
Each track is clamped independently at 4600 RPM (Teensy). If v and ω together exceed the limit, one track is clipped and the other is not, so the executed curvature differs from the commanded curvature.
- Nav2 worst case: 0.7 m/s + 1.5 rad/s × 0.877/2 = 1.36 m/s = 4090 RPM. Below the clamp.
- Manual worst case: 1.5 + 0.66 = 2.16 m/s = 6500 RPM. Clipped.
Separately, bus-voltage sag lowers the real ceiling to ~3000 RPM under sustained load (`firmware/teensy_diff_drive/FINDINGS.md`), which the host cannot see. Standard practice (WPILib `DifferentialDrive.desaturateWheelSpeeds`) scales both tracks by the same factor so curvature is preserved.

**D6 — MPPI has no acceleration model on Humble; the configured limits are ignored (Medium, confirmed).**
`nav2_params_igvc_autonav.yaml` sets `ax_max 0.4`, `ax_min −1.5`, `az_max 1.5`, and CLAUDE.md states they are matched to the actuator. On the installed Humble package they are not read at all:
- `libmppi_controller.so` (ros-humble-nav2-mppi-controller 1.1.20) contains no `ax_max`/`ax_min`/`az_max` strings, and `ros2 param get /controller_server FollowPath.ax_max` returns "Parameter not set" (`evidence/live_measurements/mppi_accel_params_check_2026_09_25.txt`).
- Acceleration constraints were added to MPPI in Nav2 PR #4352 (merged to `main` 2024-06-04), after Humble (`external/02`).

MPPI therefore starts each rollout from the odometry speed and assumes velocity can change instantly. The only acceleration limit in force is the actuator slew (0.3 / 1.3 m/s², 1.2 rad/s²), and the `velocity_smoother` is not in the command path. Effects:
- From rest, MPPI's predicted trajectory runs ahead of the robot (0.3 m/s² → 2.3 s to reach 0.7 m/s), so early collision checks and path-progress estimates are optimistic.
- Braking is harder in reality than MPPI can predict. That direction is safe.
- The residual limits that are active: `vx_max` 0.7 m/s (tighter than the actuator's 1.5) and `wz_max` 1.9 rad/s (looser than the actuator's 1.5, intentional per the config comment).

Options: build `nav2_mppi_controller` from a branch that includes PR #4352 and set limits equal to the actuator's; or keep Humble, remove the dead parameters so the config does not mislead, and reduce `vx_std`/`wz_std` so sampled controls stay closer to what the slew limit allows. The CLAUDE.md statement should be corrected either way.

**D7 — The skid multiplier compensates for a velocity-loop deficit, not only for skid (Architecture, Medium).**
`docs/skid_steer_kinematics_findings_2026_05_18.md` decomposes the 1.19 multiplier as ground skid α ≈ 0.96 (negligible) times motor under-delivery ≈ 0.85 under rotation load. Using a kinematic constant to cancel a tracking error means:
- The correction is only right at the load and speed where it was calibrated. Under-delivery depends on torque, which varies with surface, turn rate and battery voltage.
- A velocity loop with a working integrator should reach ~100% at steady state. A persistent 15% deficit suggests the integrator is too slow (kI 2.5e-7) or the feedforward lacks a static-friction term. See 01 §3.
- The multiplier was calibrated in pure rotation. The skid-steer literature shows the effective track is largest when spinning in place and shrinks as the turn radius grows (Wang et al. 2015; `external/02`). At MPPI's typical gentle arcs, 1.19 likely over-drives ω.
- Applying the multiplier to commands but not to odometry is unusual. `diff_drive_controller` (ROS 1 and ROS 2), Mandow 2007 and Clearpath's robots all use one effective separation for both paths. The asymmetry is defensible here only because the multiplier is acting as ω feedforward for motor under-delivery, which the encoders already see. That also means our chassis skid value (α ≈ 0.96) is far closer to ideal than any published skid-steer value (Pioneer P3-AT χ 0.70–0.76; Clearpath multipliers 1.125–1.875), and should be treated with suspicion off smooth pavement.

The multiplier produced a measured 10× improvement in the closed-loop square test, so it is effective as calibrated. The recommendation is to fix delivery at the motor layer first (01, M1), then re-measure the multiplier with encoder-integral spins and arcs on each surface (08, V3 and V10); it should drop toward the true skid value.

**D8 — Design rationale in the code cites a source that does not support it (Documentation, Low).**
The `actuator_node` docstring and CLAUDE.md attribute to Nav2 issue #5524 (Macenski) the guidance that closed-loop ω correction belongs at the EKF or controller rather than the actuator. Issue #5524 is about open-loop vs closed-loop feedback in the velocity smoother and controller server; it does not say this (`external/02`). The design conclusion is reasonable and consistent with the tracked-vehicle literature (Martínez 2005 and Pentzer 2014 put the slip model at the low level and path tracking above it). The citation should be corrected so future readers are not misled.

## 3. Items verified as not problems
- Python timer jitter: the Teensy watchdog is 300 ms and host commands are resent every 20 ms, so a single missed cycle cannot stop the robot or cause a runaway.
- Serial contention: one lock serializes writes; E-lines are parsed in a separate thread.
- Integer RPM formatting (`%.0f`): 1 RPM = 0.33 mm/s at the track. Negligible.
