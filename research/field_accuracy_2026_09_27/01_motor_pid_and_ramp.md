# 01 — Motor velocity loop, PID tuning and the Teensy setpoint ramp

Scope: `actuator_node` → Teensy (`L<rpm> R<rpm>`, 50 Hz; `control_rate_hz: 50`, not 20) → CAN → 2× SPARK MAX FW 26.1.4 velocity PID slot 0 → NEO → track.
Inputs: deployed snapshot (`deployed_snapshot/firmware/teensy_diff_drive/teensy_diff_drive.ino`, `.../actuator_node.py`, `.../actuator_params.yaml`), the 2026-09 audit (`../control_stack_analysis_2026_09/01, 02, 08, 09, external/01`), the 2026-04-23 bench CSVs in `firmware/teensy_diff_drive/data/`, `docs/motor_sync_logs/`, and REV documentation (sources at the end). No command was sent to the robot.

## Summary

1. **The kFF units problem is confirmed by the team's own bench data** (not only by REV documentation). In the 2026-04-23 bench run, feedforward alone (kFF 0.000197, kP = kI = 0, target 1500 RPM) produced **8 RPM on the left track and 0 on the right**. If the value were duty/RPM, it would have produced about 1540 RPM. Twenty P-only points also fit an effective feedforward of **1.4–1.9e-5 duty/RPM**, which matches 0.000197 V/RPM ÷ 12 V = 1.64e-5 and is 12× below the duty reading. So param 16 is kV in V/RPM, and today it supplies about **9%** of the feedforward needed.
2. As a result, P and I carry about 90% of the motor output. In the bench model, P alone delivers about 82% of the command, and the integrator closes the rest with a time constant of **about 3.5 s**. At the low speeds used to thread gaps, every speed change is under-delivered for seconds. The "93–96% delivery", the overshoot when a slew ends, and the 15% rotation loss that the 1.19 multiplier hides all come from this.
3. kP 0.0007 sits at about 88% of the gain that already oscillated on the bench (kP 0.0008: velocity std 190–330 RPM). The 185 ms velocity-filter lag is what limits it.
4. **New firmware risk (PR #23):** `S` and the watchdog now hold VELOCITY mode at 0 RPM. The node sends `S` every tick while idle and on e-stop, so:
   - An e-stop from 0.4 m/s or more gives about 0.84 to 1.0 **reverse duty** (the P term saturates), not the passive Brake-idle stop used before.
   - Brake idle probably never engages while `actuator_node` is running.
   - The 185 ms lag can drive a short reverse kick after the robot stops. This is not yet measured; test T4 checks it.
5. **Ramps:** the actuator slew (18 / 78 / 32 RPM per tick for accel / decel / angular) is below the Teensy ramp (M = 100 RPM per tick), so the Teensy ramp only acts on heading-hold steps and combined decel-plus-turn. A normal stop through `L0 R0` is not held back by the Teensy ramp. E-stop and watchdog stops skip it completely.
6. **Recommended order:** kV first (about 0.0021 V/RPM, raised in stages), then drop kI to about 1e-7 or 0 and kIZone to about 200, then re-check kP, then **re-measure `wheel_separation_multiplier`**. The multiplier must fall toward about 1.04 in the same session, or rotation will overshoot by about 15%. Update the yaml before BURN, because `actuator_node` overwrites the SPARK RAM gains from the yaml on every start.

## Findings

| # | Finding | Sev. | Status | Evidence | New? |
|---|---|---|---|---|---|
| F1 | **Param 16 is kV (V/RPM).** 0.000197 gives about 9% of the needed feedforward. The right value is about 12.06/5632 = **0.00214 (L)** and 12.06/5294 = **0.00228 (R)**, from the bench duty sweep. | Critical | **Confirmed** | Feedforward-only row in `data/phase6b_pid_fine_20260423_182202.csv`: 8/0 RPM at 1500 target. Fit of 20 kP-only rows in `phase6_pid_tune_*.csv` and `phase6b_*.csv` gives f_eff = 1.4–1.9e-5 duty/RPM (fit script inline in this session, plant from `phase4_duty_sweep_*.csv`). REV feedforward page: "kV: Volts per motor RPM". | Audit M1 said *Probable*; **now Confirmed with existing data** |
| F2 | Delivery depends on the integrator. P alone gives about 82% on the bench (58–68% on the ground at kP 0.0004, `docs/motor_sync_logs/csv`). Closed-loop integral time constant ≈ (1+kP·a)/(kI·a) ≈ **3.5 s** (a = 5632 RPM/duty; kI assumed per ms). kI 1e-6 was tried and gave 10–14% L/R asymmetry (skid doc §"rejected"). | High | Probable (τ from model; measure with T2) | as cited | Extends M1 and D7 |
| F3 | **kP stability margin is small.** Bench P-only: kP 0.0008 oscillated (std 192–327 RPM, flagged `oscillating=True`). The 0.0004 validation rows at ≥3000 RPM also limit-cycled (std about 300 RPM). Current kP 0.0007. The 185 ms lag (M2) is the cause. | Medium | Confirmed (bench) | `phase6_pid_tune_*.csv`, `phase6b_*.csv` | New |
| F4 | **E-stop and watchdog now command 0 RPM in velocity mode.** At 1200 RPM (0.4 m/s) that is about 0.84 reverse duty from P alone. At 2100 RPM or more it saturates at −1. Previously `S` meant duty 0 plus Brake idle. The reverse torque continues while the lagged velocity estimate still reads motion (about 185 ms), so a small backward kick is possible. It also means current spikes on the shared 12 V rail, which has a history of Jetson brownouts. | High | Probable (source); magnitude is a Hypothesis | `.ino:326-338`, `.ino:514-521`; `actuator_node.py` e-stop → `S` each tick | New (PR #23) |
| F5 | **Idle hold is an active 0-RPM velocity loop, not Brake idle.** `actuator_node` sends `S` every 20 ms while idle. REV applies Brake only on a "neutral signal". It is not verified that FW 26 treats a velocity-0 setpoint as neutral. With kI active, possible effects are creep, buzz or a slow limit cycle, and weaker passive holding than shorted windings. Code comments (`actuator_node.py` near line 470: "duty-0 → brake-idle") and CLAUDE.md ("S switches to MODE_DUTY=0") are now wrong. | Medium | Hypothesis (behavior); Confirmed (doc drift) | `.ino:326-338`; REV control-interfaces page | New |
| F6 | **Three limiters; the actuator slew dominates.** Per 20 ms tick at the track:<br>• accel 0.3 m/s² = 18 RPM<br>• decel 1.3 m/s² = 78 RPM<br>• angular 1.2 rad/s² × 0.4385 m = 32 RPM<br>• Teensy M = 100 RPM (1.66 m/s²).<br>The Teensy ramp binds only when (a) decel and angular slew add up (110 > 100), or (b) heading-hold steps of up to ±990 RPM, which it spreads over about 200 ms. SPARK closed-loop ramp (param 114) is unknown and has never been read back. | Low | Confirmed (source); param 114 unknown | `.ino:529-532`, `actuator_node.py` slew block, yaml | Refines M5 |
| F7 | **A normal stop is not delayed by the Teensy ramp.** The node decelerates at 1.3 m/s² through `L…R…` lines and sends `S` only once slewed v and ω are both 0. So the Teensy ramp never lengthens a normal stop, and `S` skipping the ramp only matters for e-stop and stale-command cases. Stopping distance from 0.4 m/s ≈ 0.06 m plus loop lag. | Info | Confirmed (source) | as above | New |
| F8 | **Low-speed precision limits.** (a) Hall encoder: 42 counts/rev, so one count per 32 ms window = 44.6 RPM, which is ±15% of 300 RPM (0.1 m/s) before averaging; the 8-deep averaging then adds the lag. (b) Stiction: breakaway 0.03 duty (L) / **0.06 duty (R)**; running-friction offset about 0.27 V (L) / 0.49 V (R) on the bench, higher on grass and much higher when pivoting. (c) kS is not supported: firmware `K` accepts only P, I, D, F, Z. (d) `tuneBoth()` writes **one value to both** controllers, so per-track kV or kS is impossible without a firmware change. Expected result: stick-slip on slow pivots, since kP 0.0007 needs about 200 RPM of error to produce 0.15 duty. | High (for tight gaps) | Probable | `phase3_stiction_*.csv`, `phase4_*.csv`, `.ino:235-238, 414-430` | Extends M2 |
| F9 | **SPARK device parameters are still unaudited:** current limit, param 114, voltage compensation (74/75), hall filter (136/137), allowed-error (97), IMaxAccum (96). The FINDINGS.md "Rank 3" statement that the 20 A free limit "always" applies is wrong: REVLib's smart current limit interpolates between the stall limit at 0 RPM and the free limit at limitRPM. With defaults, the limit is about 74 A at 1000 RPM, per motor, on a shared 12 V buck. | High | Confirmed gap | FINDINGS.md; REVLib `smartCurrentLimit` doc | Extends M3 |
| F10 | **Gain persistence and ownership.** `actuator_node` pushes the yaml gains on every start (`actuator_node.py:198-208`), so flash only matters after a SPARK brownout or reset mid-run. At that point the SPARK silently reverts to flash values until the node restarts. `M` is not in ROS and resets to 100 whenever the Teensy reboots. The firmware still carries a "BURN returns 255" comment (`.ino:371-374`), which contradicts the PR #23 claim. `burn=-1/-1` in the live DIAG only means no BURN has been attempted since boot. Whether BURN now succeeds is **not verified**. | Medium | Confirmed (source) | as cited | Extends M3 |
| F11 | **Ack log flood.** The firmware acks every `L…R…` and `S` (`OK L=…`, `OK S`) at 50 Hz. `OK_RE` in `actuator_node.py` (about line 609) matches `L=` and `S`, so the node logs about 50 INFO lines/s to /rosout and disk. That costs CPU on the Jetson (known MPPI starvation history). Every ack also costs serial bandwidth, and only E-lines have a non-blocking guard. | Low–Med | Probable (source; runtime not observed) | `.ino:337, 447`; `actuator_node.py` `OK_RE` | New |
| F12 | Voltage compensation: P and I are in duty units, so loop gain scales with bus voltage (8.5/12 = 71% at sag). kV in volts is *plausibly* converted using measured bus voltage (not documented by REV). At ≤ 1200 RPM the loop needs ≤ 0.25 duty at 12 V (0.35 at 8.5 V), so sag does not cost headroom for fine control. Priority: verify the setting is "disabled" and leave it. | Low | Hypothesis | REV docs silent; FINDINGS.md | Refines M4 |

## Field tests (outdoors today)

**Common setup for T1–T5 (through ROS; `actuator_node` owns the serial port):**
- Launch **only** `sensors.launch.py` (for the IMU) and `actuator.launch.py`. Do **not** run Nav2, webui or teleop: they would fight over `/cmd_vel` (audit D1).
- Every shell: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$HOME/IGVC_ROS2/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml`
- Disable heading-hold so the tracks see pure commands: `ros2 param set /actuator_node heading_hold_deadband 0.0`. Restore 0.05 afterwards.
- Record:
  - `python3 motor_step_log.py record tN.csv` (this folder; read-only)
  - `ros2 bag record /avros/wheel_debug /imu/data /cmd_vel /avros/actuator_command /wheel_odom /rosout`
- Analyze: `python3 motor_step_log.py analyze tN.csv`. It measures velocity from **encoder position**, never from the lagged reported RPM.
- Command sequences use this snippet. It **publishes**, so the operator runs it deliberately:
  ```bash
  python3 - <<'EOF'   # steps = [(v, w, seconds), ...]; always ends with 3 s of zero
  import rclpy, time; from geometry_msgs.msg import Twist
  steps=[(0.1,0,6),(0.2,0,6),(0.3,0,6),(0.4,0,6),(0.2,0,6),(0,0,3)]   # edit per test
  rclpy.init(); n=rclpy.create_node('step_pub'); p=n.create_publisher(Twist,'/cmd_vel',10)
  try:
      for v,w,d in steps:
          t=time.time()
          while time.time()-t<d: m=Twist(); m.linear.x=v; m.angular.z=w; p.publish(m); time.sleep(0.05)
  finally:
      for _ in range(10): p.publish(Twist()); time.sleep(0.05)
  EOF
  ```
- Safety for every test:
  - One person holds the e-stop: `ros2 topic pub --once /avros/actuator_command avros_msgs/msg/ActuatorCommand '{estop: true}'`, or the hardware stop.
  - Clear lane of at least 15 m.
  - Ctrl-C stops publishing; the robot then decelerates within 0.5 s timeout plus 1.3 m/s².

### T1 — Feedforward-only delivery (decides F1 on the ground; about 10 min)
- **Purpose:** show that kV, not P/I, can carry the load, and find the ground kV.
- **Procedure:**
  1. Record a baseline: T2 with current gains.
  2. `ros2 param set /actuator_node kI 0.0` then `kP 0.0001`. A small P keeps it safe; with P = 0 the robot will not move. Confirm each `<- Teensy ack: OK K…` line.
  3. For kFF in `0.0005 → 0.0010 → 0.0016 → 0.0021`: run steps `[(0.2,0,6),(0.4,0,6),(0,0,3)]`.
- **Safety:**
  - Start at kFF 0.0005. If param 16 were duty/RPM (it is not, per F1), that step alone would triple the speed, and you would see it at 0.2 m/s. **Abort if speed exceeds 0.3 m/s at the 0.2 step.**
  - Restore gains at the end (see the tuning procedure).
- **Log:** `dEnd` per track from `analyze`.
- **Expected and pass:**
  - Delivery rises roughly in proportion to kFF: about 25–35% at 0.0005, 85–100% at 0.0021 (friction takes the rest).
  - **Pass (F1 confirmed on ground):** delivery at 0.0021 is ≥ 3× delivery at 0.0005.
  - Choose kFF_ground = the value where the *left* track reaches 97–100% at 0.4 m/s. The right track will be about 5% short, which I closes later.

### T2 — Low-speed step and tracking (baseline vs tuned; about 5 min per gain set)
- **Purpose:** measure what MPPI actually gets at 0.1–0.4 m/s: delivery vs time, overshoot and settling.
- **Procedure:**
  - Steps `[(0.1,0,6),(0.2,0,6),(0.3,0,6),(0.4,0,6),(0.2,0,6),(0,0,3)]` forward, then the same sequence with negative v.
  - Optional second pass with `max_linear_accel_mps2 2.0` so the Teensy ramp (M = 100) is the only limiter, which gives step-like inputs. Restore 0.3 after.
- **Log:** `analyze` columns d0.5, d1, d2, dEnd, ovs%, t95 and CoV% per segment.
- **Pass:**
  - Delivery ≥ 95% within 1.0 s after the slew ends, and ≥ 98% at segment end.
  - Overshoot < 8%, steady-state CoV < 3%, t95 < 0.6 s.
  - Baseline expectation (F2): d1 about 80–90%, still rising at dEnd.

### T3 — Slow pivot and gentle arcs (stick-slip; about 5 min)
- **Purpose:** check fine turning, which is what threading barrels needs.
- **Procedure:**
  - Pivots: `[(0,0.1,8),(0,0.2,8),(0,0.3,8),(0,0,3)]`, both directions.
  - Arcs: `[(0.2,0.1,8),(0.3,0.2,8),(0,0,3)]`.
  - Keep the multiplier at 1.19 for the baseline. After tuning, repeat once with 1.0 and once with 1.19.
- **Log:** `imu_w/w_cmd` column; per-track CoV%.
- **Pass:**
  - Motion starts within 0.5 s of the command.
  - Yaw-rate CoV < 10% (no stick-slip; you can also hear and see it).
  - IMU ω / commanded ω between 0.95 and 1.05 at every step.
  - If ω delivery rises during the 8 s hold, the integrator is doing the work (F2).

### T4 — Stop, e-stop and idle hold (F4, F5; about 10 min)
- **(a) Normal stop.** Steps `[(0.2,0,5),(0,0,8)]` and `[(0.4,0,5),(0,0,8)]`.
  - **Pass:** stop distance ≤ v²/(2·1.3) + v·0.1 (0.06 m at 0.4 m/s; up to about 0.10 m accepted); reverse travel < 5 mm; drift in the next 5 s < 2 mm per track.
- **(b) E-stop.** Hold 0.3 m/s via the snippet, then send the e-stop line while it is publishing. Clear the e-stop by publishing any non-estop `ActuatorCommand` (`'{estop: false}'`, once).
  - **Pass:** reverse travel < 10 mm; no Jetson reboot (check `uptime`).
  - **Watch for:** a visible backward lurch, which is F4.
  - Repeat at 0.5 m/s only if 0.3 m/s is clean.
- **(c) Idle hold.** Robot idle 60 s after a stop, then an operator pushes it firmly forward and backward for 2 s each.
  - **Log:** `L_pos_rev`, `R_pos_rev`.
  - **Pass:** no audible buzz or hunting; drift < 0.05 rev while unpushed; the robot resists pushing and does not keep rolling after release.
  - How to read the push: a velocity loop at 0 with I active behaves like a soft position spring (it resists, then pulls back after release). Brake idle is drag proportional to speed, with no pull-back. Stopping `actuator_node` does not give a Brake-idle comparison, because the Teensy watchdog also holds velocity 0. A true duty-0 comparison needs `UL0 UR0` over serial (T6 setup).

### T5 — L/R matching straight line (about 5 min)
- **Procedure:** 0.3 m/s for 12 s on the flattest surface available, heading-hold off, 2 runs each direction.
- **Log:** `L/R mism%`; IMU yaw change over the run.
- **Pass:** track delivery mismatch < 2%; yaw drift < 3° per 10 m.
- **Note:** after the kV change, the right track (8% more friction) will lag more until I or a per-track kV fixes it. That is expected, and it is why kI must not go all the way to 0 unless per-track kV or kS is added.

### T6 — Teensy-side checks (requires stopping `actuator_node`; optional, about 10 min)
- **Setup:** kill `actuator_node` (per the serial-collision memory, **never** open the port while the node runs). Then `python3 -m serial.tools.miniterm /dev/serial/by-id/usb-Teensyduino_USB_Serial_20383890-if00 115200`. Tracks off the ground, or chassis on blocks.
- **Procedure:** poll `D` while sending `L600 R600`, then `L1200 R1200` (repeat every < 300 ms, or the watchdog stops the motors), then `S`.
- **Log:** bus voltage `V=` at each step; `mode=VEL`; ramp value (second number in `L=meas/ramp/cmd`).
- **Pass:** `ramp` reaches `cmd` in cmd/100 × 20 ms; V stays ≥ 11.5 V at 1200 RPM.
- **BURN check** (only after tuning, see below): send `BURN`. **Pass:** `OK BURN result L=0 R=0`. Anything else (255, −1) means flash was not written.

## Tuning procedure (after T1 and T2 baselines)

Order: feedforward, then I down, then P, then kinematics. Change one thing at a time. Run T2 plus T3 after every step, and never tune on reported RPM (it lags 185 ms).

1. **kV:** `ros2 param set /actuator_node kFF <kFF_ground from T1>`. Start value 0.0021. Keep kP 0.0007 and kI 2.5e-7 for this first check.
   - Expect delivery to jump and possibly overshoot, because I no longer has to supply 90% of the output.
   - If T2 overshoot is > 10%, go straight to step 2.
2. **kI down, kIZone down:** `kI 1e-7`, `kIZone 200`.
   - Judge: T2 dEnd ≥ 98% on both tracks and no overshoot at the end of the slew.
   - If the right track settles at < 97%, raise kI in steps of 5e-8. Do **not** exceed 2.5e-7: at 1e-6 the team measured 10–14% L/R asymmetry.
3. **kP:** try 0.0005, then 0.0004.
   - Keep the lowest kP that still passes T2 (t95 < 0.6 s) and T3 (no stick-slip). Lower kP means more margin from the bench oscillation onset at 0.0008 and less noise-driven buzz at hold.
   - Do not raise kP above 0.0007 until the hall filter is shortened (audit M2; not possible today without a firmware `P<id>` write or the Hardware Client).
4. **Skid multiplier (mandatory, same session):** repeat the 2026-05-18 spin test (`docs/skid_steer_kinematics_findings_2026_05_18.md` §procedure) with `wheel_separation_multiplier 1.0`. Set multiplier = 1 / (IMU ω / commanded ω). Expect about 1.02–1.08 on pavement.
   - Leaving it at 1.19 after fixing kV **over-drives every turn by about 15%**.
   - Re-check arcs (T3) at the new value; grass will differ.
5. **Teensy M:** leave it at 100. It is a backstop for heading-hold steps and raw serial tests. Raising it to 120 removes the only binding case (decel plus turn) but does not help fine control. Lowering it below 78 would lengthen normal stops.
6. **Make it stick:**
   - (a) Edit `src/avros_bringup/config/actuator_params.yaml` on the Jetson (`kFF`, `kP`, `kI`, `kIZone`, `wheel_separation_multiplier`). **`ros2 param set` values are lost at node restart, and the node re-pushes the yaml on every start.** Fix the stale comments: kFF is **V/RPM**, and the "BURNed via Hardware Client" note.
   - (b) With `--symlink-install`, confirm the installed yaml is a symlink (`ls -l install/avros_bringup/share/avros_bringup/config/actuator_params.yaml`); otherwise rebuild `avros_bringup`.
   - (c) Restart `actuator_node` and confirm the `SparkMAX gains set:` log line.
   - (d) BURN via T6 so a mid-run SPARK brownout falls back to the same gains (F10).
   - Also update the `declare_parameter` defaults in `actuator_node.py` (kP 0.0004, kI 0, kIZone 200), which silently apply if the yaml is not loaded.
7. **Revert criteria:** if any T4 or T2 run shows oscillation (CoV > 5%) or a lurch, go back to the baseline set: `kFF 0.000197 kP 0.0007 kI 2.5e-7 kIZone 600`, multiplier 1.19.

Follow-ups that need code or firmware (not today):
- A generic parameter write and read-back (`P<id> <val>` / `G<id>`) to set kS (param 204, UNVERIFIED id), per-track kV, hall filter (136/137), current limit (40 A) and param 114, and to audit 74/75/96/97.
- Stop mode: e-stop should use duty 0 (Brake), or a decel ramp, rather than a saturated 0-RPM command (F4).
- Stop logging `OK L=` / `OK S` acks, or drop those acks in firmware (F11).
- Expose `M` as a ROS param.

## Script

`motor_step_log.py` (this folder; read-only, never publishes).
- `record` subscribes to `/avros/wheel_debug` (50 Hz, 16 fields including L/R cmd, meas, pos_rev, targets, `heading_locked`, `estop`) and `/imu/data`. It writes CSV with receive time and IMU ω.
- `analyze` computes:
  - Per constant-command segment: position-derived RPM (±50 ms centered difference), delivery at 0.5/1/2 s and at the end, overshoot, t95, steady-state CoV, reported/position ratio, L/R mismatch, IMU ω / w_target.
  - Per stop (v_target drops to 0, or e-stop rises): stopping distance, reverse travel and drift in the following 5 s, per track.
- Checked against `docs/motor_sync_logs/csv` (P-only era). It reproduces 58–59% (L) and 49–51% (R) delivery at about 1000–1200 RPM on the ground.
- Limitation: timestamps are host receive times (E-lines carry no Teensy time; audit M6), so timing metrics are ±20 ms.

## Sources
- REV Feed Forward Control (kS V, kV "Volts per motor RPM", mode applicability): https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control
- REVLib changelog ("kV (formerly kF)"): https://docs.revrobotics.com/revlib/install/changelog
- REV Closed Loop Control (1 ms loop, PID + feedforward): https://docs.revrobotics.com/revlib/spark/closed-loop
- REV SPARK MAX parameters (IZone, param list): https://docs.revrobotics.com/brushless/spark-max/parameters
- REV control interfaces / idle mode ("brake … on neutral signal"): https://docs.revrobotics.com/brushless/spark-max/control-interfaces
- REV MAXSwerve template, kV = 12/freeSpeed: https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/Configs.java
- REV-Specs frames and parameters (param 114, 74/75, 136/137, 204): https://github.com/REVrobotics/REV-Specs
- REVLib smart current limit (stall → free interpolation): https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/sparkbaseconfig
- NEO specs (42 cpr, 5676 RPM): https://www.revrobotics.com/rev-21-1650/
- NEO velocity filter lag PSA (Veness): https://www.chiefdelphi.com/t/psa-default-neo-sparkmax-velocity-readings-are-still-bad-for-flywheels/454453
- WPILib SysId gain analysis (filter delay d = T(n−1)/2): https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/analyzing-gains.html
- RobotPy EncoderConfig (32 ms / depth 8 defaults): https://robotpy.readthedocs.io/projects/rev/en/stable/rev/EncoderConfig.html
- Team data: `firmware/teensy_diff_drive/data/phase3,4,6,6b_*.csv`, `FINDINGS.md`; `docs/motor_sync_logs/csv*/avros_wheel_debug.csv`; `docs/skid_steer_kinematics_findings_2026_05_18.md`.
