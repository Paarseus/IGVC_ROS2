# External Research — SPARK MAX / NEO Velocity Control

Collected 2026-09-25. Every claim links to its source; items the research could not confirm are marked UNVERIFIED.

## 1. SPARK MAX velocity PID internals

- **Loop rate: 1 kHz.** "the loop is updated every 1ms" ([REV closed-loop docs](https://docs.revrobotics.com/revlib/spark/closed-loop)). A REV engineer confirmed "The PID loop still runs at 1ms"; a "10 ms" figure in the docs was a typo ([Chief Delphi](https://www.chiefdelphi.com/t/why-is-rev-pid-significantly-slower-now/483639)).
- **Gain units** (REV "Units" table, [REV docs full text](https://docs.revrobotics.com/llms-full.txt); linked from [Feed Forward Control](https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control)):
  - Applied output is duty cycle.
  - kP: duty per unit error (duty/RPM in velocity mode).
  - kI: duty per (unit·ms); the integrator accumulates every 1 ms. The velocity form is inferred from the position form.
  - kD: (duty·ms) per unit.
  - **kS: volts. kV: volts per RPM. kA: volts per RPM/s.**
- **Firmware 26 changed feedforward from duty to volts.**
  - REVLib 2026.0.0 changelog: "Adds support for new feedforward parameters: `kV` (formerly `kF`), `kA`, `kS`, `kG`, `kCos`…" ([changelog](https://docs.revrobotics.com/revlib/install/changelog)).
  - The 2026 `SparkParameters.h` header keeps `kV_0 = 16`, the same ID as the old F slot 0, and adds `kS_0 = 204`, `kA_0 = 205` ([REVLib 2026.0.2 header copy](https://github.com/l5vel/sparklib-py/tree/HEAD/reference/revlib-2026.0.2)).
  - REV's MAXSwerve template changed `velocityFF(1/freeSpeed)` to `feedForward.kV(12.0/freeSpeed)` and left `pid(0.04,0,0)` unchanged, commit b5b9d364af ([MAXSwerve-Java-Template Configs.java](https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/Configs.java)). This factor-of-12 change is the key evidence.
- **Where each feedforward term applies:** kS in all closed-loop modes; kV in Velocity and MAXMotion; kA only in MAXMotion ([Feed Forward Control](https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control)). How volts become duty (for example, division by measured bus voltage) is UNVERIFIED.
- **Arbitrary feedforward** is added "after *all* calculations are done", in volts or duty ([REV closed-loop docs](https://docs.revrobotics.com/revlib/spark/closed-loop)). In the VELOCITY_SETPOINT frame (cls 0, idx 0) it occupies bits 32–47 (int16 × 0.0009765923, ±32); bit 50 selects units (0 = V, 1 = duty); bits 48–49 are the PID slot ([REV-Specs spark-frames-2.1.0](https://github.com/REVrobotics/REV-Specs)). The Teensy can therefore send a host-side feedforward with every setpoint.
- **IZone** (param 17): "The PIDF loop integrator will only accumulate while the setpoint is within IZone of the target" ([REV parameters](https://docs.revrobotics.com/brushless/spark-max/parameters)). Whether the accumulator resets or holds outside IZone is UNVERIFIED.
- **IMaxAccum** (param 96, default 0) constrains the I accumulator ([ClosedLoopConfig javadoc](https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/closedloopconfig)). Whether 0 means unlimited is UNVERIFIED.
- **Integrator visibility:** SET_I_ACCUMULATION (cls 10, idx 2); STATUS_7 reports the accumulator and STATUS_8 the setpoint and at-setpoint flag; both 20 ms and disabled by default ([REV-Specs](https://github.com/REVrobotics/REV-Specs)).
- **Output range:** params 19/20, default −1..1 duty.
- **Ramp rates:** Open Loop Ramp Rate param 56. Closed Loop Ramp Rate param 114, "in DC per second", default 0 (off); it limits the output, not the setpoint ([REV-Specs parameters](https://github.com/REVrobotics/REV-Specs/blob/main/parameters/SparkParameters-v0.1.2.md); [CANSparkMax javadoc](https://team2168.org/javadoc/com/revrobotics/CANSparkMax.html)).
- **Smart Velocity vs Velocity.** SMARTVELOCITY is a legacy mode using the SmartMotion parameters (76–95); the REVLib page now returns 404 and exact behavior is UNVERIFIED. The FW 25+ replacement is MAXMotion Velocity (Max Accel param 167, RPM/s; supports kA). REV notes "kV/kA and no PID at all" can perform well ([MAXMotion Velocity](https://docs.revrobotics.com/revlib/spark/closed-loop/maxmotion-velocity-control)). Plain Velocity mode = PID + kS/kV on the raw setpoint, no profile.

## 2. NEO hall-encoder velocity measurement

- **Resolution:** 42 counts/rev. Kv 473 RPM/V, free speed 5676 RPM at 12 V, stall 105 A ([NEO product page](https://www.revrobotics.com/rev-21-1650/)).
- **Default filter:** measurement period **32 ms** (range 8–64 ms), average depth **8** (1, 2, 4 or 8) ([RobotPy EncoderConfig](https://robotpy.readthedocs.io/projects/rev/en/stable/rev/EncoderConfig.html)). Raw parameters: 136 "Uvw Sensor Sample Rate" (default 0.03125), 137 "Uvw Sensor Average Depth" (default 3) ([REV-Specs parameters](https://github.com/REVrobotics/REV-Specs/blob/main/parameters/SparkParameters-v0.1.2.md)). Encoding (seconds; log2 depth) is UNVERIFIED.
- **Configurable** since REVLib 2023.1.2 / FW 1.6.2: "Adds support to configure the hall sensor's velocity measurement" ([changelog](https://docs.revrobotics.com/revlib/install/changelog)). Older javadoc stating it has no effect in brushless mode ([2168 javadoc](https://team2168.org/javadoc/com/revrobotics/SparkMaxRelativeEncoder.html)) is out of date.
- **Lag:**
  - "8-tap moving average filter applied with 32 ms between samples. This adds (8 − 1)/2 × 32 ms = 112 ms of delay" (Tyler Veness, [Chief Delphi PSA](https://www.chiefdelphi.com/t/psa-default-neo-sparkmax-velocity-readings-are-still-bad-for-flywheels/454453)).
  - Another estimate is ~82 ms, with the warning that "the max stable feedback gain shrinks exponentially as your signal delay grows past the system time constant" ([Chief Delphi](https://www.chiefdelphi.com/t/inconsistent-sparkmax-velocity-control/456497)).
  - WPILib formula: d = T(n − 1)/2 ([SysId analyzing gains](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/analyzing-gains.html)).
  - That the onboard PID uses the same filtered signal is widely assumed but UNVERIFIED by REV.
- **Settings other teams use:** Team 3005 uses 16 ms / depth 2 on swerve drive motors (same PSA thread). Team 620: "16 ms period and depth 2 … The defaults add about 80 ms of lag" ([T620 PR #17](https://github.com/FRC-Team-620/T620-SPF-NIDMOT-SW/pull/17)).
- **Quantization** (arithmetic): one hall count per 32 ms window ≈ 44.6 RPM before averaging; ≈ 89 RPM at 16 ms.
- **Odometry:** position is an unfiltered integer count, so position deltas are the clean odometry source.

## 3. CAN status frames (FW 25+/26, [REV-Specs spark-frames-2.1.0](https://github.com/REVrobotics/REV-Specs))

| Frame | Content | Default period | Enabled by default |
|---|---|---|---|
| STATUS_0 (46,0) | applied output, voltage, current, temperature, limits, heartbeat lock | 10 ms | yes |
| STATUS_1 (46,1) | faults/warnings incl. BROWNOUT_WARNING, HAS_RESET_WARNING, STALL | 250 ms | yes |
| STATUS_2 (46,2) | primary encoder velocity (float RPM) **and** position (float rotations) | 20 ms | no |
| STATUS_7 / STATUS_8 | I accumulator / setpoint, at-setpoint | 20 ms | no |

- Pre-2025 documentation placing velocity in Status 1 is obsolete ([old control-interfaces page](https://docs.revrobotics.com/brushless/spark-max/control-interfaces)). FW 25+ sends frames other than 0/1 only when enabled ([REVLib migration](https://docs.revrobotics.com/revlib/archive/24-to-present)).
- Period: params 158–165 ("Status N Period"; unit labelled μs in the spec but defaults are clearly ms, UNVERIFIED). Enable: SET_STATUSES_ENABLED (cls 1, idx 0) or "Force Enable Status N" params 186–193 (Status 2 = 188).

## 4. Tuning practice

- Theoretical feedforward: old duty units 1/5676 = 1.76e-4 duty/RPM; **FW 26 volt units 12/5676 = 2.11e-3 V/RPM** (REV template pattern `nominalVoltage / freeSpeed`, [MAXSwerve Configs](https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/Configs.java)).
- REV tuning order: all gains 0, tune feedforward, then small P, D only if needed. I "is not often recommended … Feedforward gains are recommended to eliminate steady-state error instead" ([REV docs full text](https://docs.revrobotics.com/llms-full.txt)). kS = largest output that does not move the mechanism, or from SysId ([Feed Forward Control](https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control)).
- SysId has presets for smart-controller velocity filter delay and a 1 kHz controller period; custom filter settings require recalculating the delay ([WPILib SysId](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/analyzing-gains.html)).
- Host-side feedforward was common because onboard kF was duty-based: "The lack of feedforward is a big reason why REV's profiling hasn't worked very well" ([Chief Delphi](https://www.chiefdelphi.com/t/rev-feedforwardconfig/506313)).
- Reference gains: REV MAXSwerve drive kP 0.04 duty/(m/s) with kV = 12/free speed (≈ 1.3e-5 duty/RPM in our units, ~50× smaller than our 0.0007, because feedforward carries the load). Legacy REV velocity example: kP 6e-5, kI 0, kFF 1.5e-5 ([SPARK-MAX-Examples](https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java)). No authoritative NEO drivetrain gain set was found (UNVERIFIED).

## 5. Pitfalls

- **Heartbeat:** universal heartbeat `0x01011840`, 8 bytes, every 20 ms ([WPILib CAN spec](https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html)). After seeing it, a SPARK sets PRIMARY_HEARTBEAT_LOCK and "will ignore the Secondary Heartbeat until it is power cycled" ([REV-Specs](https://github.com/REVrobotics/REV-Specs)). The disable timeout (often quoted as 100 ms) is UNVERIFIED.
- **Supply:** 6–24 V operating, 4.5 V minimum before full brownout; 60 A continuous, 100 A for 2 s ([REV docs](https://docs.revrobotics.com/llms-full.txt)). Non-persisted parameters revert after a power cycle or brownout ([Configuring a SPARK](https://docs.revrobotics.com/revlib/spark/configuring-a-spark)).
- **Voltage compensation:** param 74 (mode) and 75 (nominal voltage) ([REV-Specs](https://github.com/REVrobotics/REV-Specs/blob/main/parameters/SparkParameters-v0.1.2.md)). Duty-based feedforward was found "reliant on Battery Voltage" and voltage compensation fixed RPM droop ([Chief Delphi](https://www.chiefdelphi.com/t/inconsistent-sparkmax-velocity-control/456497)). Volt-based FW 26 feedforward is plausibly sag-independent (UNVERIFIED). Compensation cannot add headroom during a sag.
- **Current limits:** default 80 A stall / 20 A free, config RPM 10000 (params 59–61). REV recommends 40–60 A for NEO, and for drive motors starting at 20 A and increasing until traction breaks ([REV docs](https://docs.revrobotics.com/llms-full.txt)).
- **2025/2026 changes:** FW 25 new status layout with frames off by default; FW/REVLib 2026 kF → kV (volts), added kS/kA/kG, allowed closed-loop error (param 97) ([changelog](https://docs.revrobotics.com/revlib/install/changelog)). MAXMotion max velocity no longer clamps plain velocity setpoints ([Chief Delphi](https://www.chiefdelphi.com/t/why-is-rev-pid-significantly-slower-now/483639)).

## 6. Open-source non-roboRIO SPARK drivers

| Project | Platform | Relevance |
|---|---|---|
| [REVrobotics/REV-Specs](https://github.com/REVrobotics/REV-Specs) | JSON frame and parameter spec | Ground truth for the Teensy firmware |
| [willGuimont/CanControl](https://github.com/willGuimont/CanControl) | Arduino + MCP2515, generated from REV-Specs | 20 ms heartbeat, rate-limited scheduler; frame-packing reference |
| [l5vel/sparklib-py](https://github.com/l5vel/sparklib-py) | Python, SocketCAN, FW 26.1.x aware | REVLib 2026 headers, 44-item failure-mode catalogue, parameter audit tools |
| [grayson-arendt/sparkcan](https://github.com/grayson-arendt/sparkcan) | C++ SocketCAN | FW 24 only; obsolete |
| [turhans23/SparkMaxDriver](https://github.com/turhans23/SparkMaxDriver) | STM32 HAL | Duty-only, pre-25 layout |

No maintained ros2_control interface for FW 25+ SPARKs was found (UNVERIFIED that none exists).

## Implications stated by the research (to be verified; see 01 and 08)

1. Param 16 on FW 26.1.4 is kV in V/RPM, so kFF 0.000197 supplies ~9% of the intended feedforward (0.000197 vs 12/5676 = 0.00211). A pure-P model reproduces the observed delivery: kP 0.0004 → 72% predicted vs ~70% observed; kP 0.0008 → 84% predicted vs 83% observed. The duty interpretation predicts 104% for kP 0.0004.
2. Retune feedforward first (per-track kV from measured free speeds, kS in param 204), then reduce P and I.
3. Shorten the hall velocity filter (param 136 = 16 ms, 137 = depth 2 or 4) and read back to confirm the encoding.
4. Build odometry from STATUS_2 position; optionally raise STATUS_2 to 10 ms (param 160) and force-enable it (param 188).
5. A host-side kS/kV/kA can be sent in the VELOCITY_SETPOINT arbitrary-feedforward field.
6. Keep the Teensy setpoint ramp; Closed Loop Ramp Rate limits output and can wind up the integrator.
7. Check and persist the current limit (params 59/60); ~40 A also reduces the inrush that sags the shared 12 V rail.
8. Decode STATUS_1 BROWNOUT_WARNING / HAS_RESET_WARNING on the Teensy.
