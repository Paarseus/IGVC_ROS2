# Teensy SPARK MAX CAN firmware vs REV's CAN specification (review, 2026-09-27)

**Status:** Verified: an independent review checked 83 claims (73 verified, 10 partly supported, 0 not supported), and all 10 corrections are applied below. See `VERIFICATION.md`.

Reviewed: `firmware_snapshot/teensy_diff_drive.ino` (deployed commit f242825, cited as `ino:<line>`), its `firmware_snapshot/CLAUDE.md`, `firmware/teensy_diff_drive/FINDINGS.md`, `src/avros_control/avros_control/actuator_node.py`, `src/avros_bringup/config/actuator_params.yaml`.
Target: 2 × SPARK MAX on firmware 26.1.4, NEO hall sensor, no roboRIO, 1 Mbit/s. References are in `references/` (section 2). This is a review only; no code was changed.

Terms: **arbitration ID** = the 29-bit CAN frame ID, packing device type, manufacturer, API class, API index and device number. **DLC** = number of payload bytes. **Persist/BURN** = copy the parameters in RAM to flash. **Status frame** = a frame the SPARK sends on a timer. **Enabled** = the SPARK drives the motor only while a heartbeat says the robot is enabled. **RTR (remote frame)** = a CAN request frame with no data.

## 1. Summary (most serious first)

1. **Our "kFF" goes to parameter 16. On firmware 26 that parameter is kV, in volts per RPM, not duty cycle per RPM.**
   - REVLib 2026 names ID 16 `kV_0(16, FLOAT)` (`rev_2026_revlib_java_2026.0.5_SparkParameters.java`) and documents it as "Volts per velocity" (`…FeedForwardConfig.java`, kV javadoc).
   - REV's units table says "kV: Volts per RPM" (`rev_2026_closed_loop_units.md`, "Default Units").
   - REV's own example changed from `velocityFF(1.0/5767)` (`rev_2025_revlib_example_closed_loop_Robot.java:72`) to `kV(12.0/5767)` with the comment "kV is now in Volts, so we multiply by the nominal voltage (12V)" (`rev_2026_…Robot.java:74-75`).
   - We send 0.000197 (= 1/5072), sized as duty per RPM. Read as V/RPM, feedforward at 4514 RPM is 0.89 V (~7 % duty at 12 V) instead of ~0.89 duty. That is about 12× too weak, so P and I are doing almost all of the work.
   - Circumstantial support (inference, not a measurement): with kP 0.0004, kI 0, a 12 V bus and the Phase-4 left-wheel plant (5667 RPM/duty, −135 intercept), the volts reading predicts 1042 RPM (69.5 %) at a 1500 RPM command, and the duty reading predicts 1512 RPM (100.8 %). The root CLAUDE.md records ~70 % delivery for issue #6; FINDINGS.md:39 records 82 % at kP 0.0004. Both are well below the ~101 % the duty reading predicts.
   - Fix direction: kV ≈ 12 × 0.000197 ≈ 0.0024.
   - **Not confirmed on hardware.** Firmware sm-26.1.0 notes say only "Adds expanded feedforward support to all PID modes". Run bench test T1 first.
2. **BURN = 255 most likely means "refused because the SPARK is enabled".**
   - A REV software engineer states it directly (wpilibsuite/2025Beta#69, 2024-12-18): persist "failed because the robot is enabled… persisting parameters must only be done when disabled".
   - We send the roboRIO heartbeat with Enabled + System-watchdog every 20 ms (`ino:185`, `ino:524`).
   - REVLib has `kCannotPersistParametersWhileEnabled = 26` (`rev_2026_revlib_cpp_2026.0.5_REVLibError.h`; `…REVLibError.java:58`).
   - REVLib 2025.0.0 "Improves error description when attempting to persist parameters while the robot is enabled".
   - The spec defines only "0 on success" (`rev_2026_spark_frames_2.1.0.json:1089`). **Only the raw code 255 is unconfirmed.**
3. **Every response frame is ignored, so no write is ever verified.**
   - PARAMETER_WRITE_RESPONSE (class 14 index 1, DLC 7) returns the current value and a result code: 0 ok, 1 invalid ID, 2 mismatched type, 3 access mode, 4 invalid, 5 not implemented (`…2.1.0.json:5032`).
   - The firmware prints `OK K<x>=` before any CAN reply (`ino:427-428`), and actuator_node logs that as "end-to-end confirmation".
4. **Speed-reading lag: about 112 ms of the ~185 ms comes from the SPARK's hall-velocity filter, which we never configure.**
   - REV support: sampled every 32 ms, averaged over 8 samples, lag ≈ (8−1)/2 × 32 = 112 ms (`rev_2022_hall_velocity_latency_via_wpilib_issue258.md`). This is a 2022 statement, from before firmware 25, relayed in a community GitHub issue. The 32 ms / 8-sample defaults are confirmed in REVLib 2026 (`…EncoderConfig.java`), but the 112 ms figure is **not confirmed for firmware 26.1.4**.
   - The ~185 ms total is our own earlier measurement, not from any reference.
   - Writable on firmware 26: parameter 136 `kUvwSensorSampleRate` (FLOAT seconds, 8–64 ms) and 137 `kUvwSensorAverageDepth` (UINT32 index 0–3 = 1/2/4/8 samples) (`…EncoderConfig.java`).
   - Whether the on-board PID uses the same filtered value is **not confirmed**.
5. **SET_STATUSES_ENABLED is sent with DLC 8; the spec says 4** (`ino:209-213` vs `…2.1.0.json:1025`). The mask and enable fields are correct. STATUS_2 arrives, so it is tolerated in practice. The response frame (class 1 index 1) is ignored.
6. **`setParam()` always writes a float32** (`ino:218-223`). The value field's type "depends on the Parameter Type" (`…2.1.0.json:5006`).
   - These parameters are UINT32 or BOOL: 6, 45, 59–61, 74, 137, 158–165.
   - FINDINGS.md "Rank 3" ("identical … float32" for 60/74/76/77) is wrong. 76/77 (SmartMotion) were removed in sm-26.1.0.
7. **Correct as-is:** arbitration-ID layout, universal heartbeat, velocity and duty setpoints, STATUS_0 voltage, STATUS_2, PARAMETER_WRITE layout for 13–17, and the persist frame, magic number and response frame.
8. **Future-firmware risk:** REVLib 2027.0.0-alpha-7 "Removes hall sensor velocity averaging configurations…" and "Removes Conversion Factors" (IDs 112/113/136/137 are absent from `…2027.0.0-alpha-7_CANSparkParameters.h`). Do not update the SPARKs past 26.x without re-review.
9. **Side finding:** the root CLAUDE.md says serial `S` switches to MODE_DUTY=0, but the deployed firmware's `S` sets velocity mode at 0 RPM (`ino:326-338`).

## 2. References (level A = REV official spec/source, B = REV/WPILib docs or REV staff, C = community)

| File in references/ | What / exact source | Level |
|---|---|---|
| rev_2026_spark_frames_2.1.0.json | REV-Specs `can-frames/spark-frames-2.1.0`. Newest; has `versionImplemented` 26.0.0 frames. github.com/REVrobotics/REV-Specs commit 1e90305632317f97e922257ad4b22d7a324f3afd (2026-01-02) | A |
| rev_2025_spark_frames_2.0.0-dev.11.json | Previous spec; same repo, added in commit 58eb8ff31bd2c2297df4797351677cb95ade867e (2025-05-12) | A |
| rev_2025_spark_parameters_v0.1.2.md, rev_2026_rev_specs_README.md | REV-Specs parameter table and README @ 1e90305 | A |
| rev_2026_revlib_driver_2026.0.5_{CANSparkFrames,CANSparkDriver,CANSparkParameters}.h | maven.revrobotics.com com/revrobotics/frc/REVLib-driver/2026.0.5/…-headers.zip (`kMinFirmwareVersion = v26.0.0`) | A |
| rev_2026_revlib_driver_2027.0.0-alpha-7_{CANSparkFrames,CANSparkParameters}.h | …/REVLib-driver/2027.0.0-alpha-7/…-headers.zip (the version the firmware comments cite) | A |
| rev_2026_revlib_cpp_2026.0.5_REVLibError.h | …/REVLib-cpp/2026.0.5/…-headers.zip | A |
| rev_2026_revlib_java_2026.0.5_{SparkParameters,ClosedLoopConfig,FeedForwardConfig,EncoderConfig,SignalsConfig,SparkBaseConfig,MAXMotionConfig,FeedbackSensor,REVLibError}.java | …/REVLib-java/2026.0.5/REVLib-java-2026.0.5-sources.jar | A |
| rev_2026_node_revlog_converter_spark.public.dbc | REV's public DBC; github.com/REVrobotics/node-revlog-converter commit 382d7922e8099c2449ce6baaa0edf0aab8718cd0 | A |
| rev_2025/2026_revlib_example_closed_loop_Robot.java | REV examples (from C1 sources) | A |
| rev_2026_firmware_revlib_release_notes.md, rev_2026_closed_loop_units.md, rev_2026_feedforward_control.md, rev_2026_sparkmax_parameters_legacy_docs_page.md | REV release notes and docs (C1 sources) | B |
| rev_2022_hall_velocity_latency_via_wpilib_issue258.md | REV Support statement quoted in wpilibsuite/sysid#258 (2022-01-12) | B |
| wpilib_2024_2025beta_issue69_persist_error.md | wpilibsuite/2025Beta#69 plus comments; jfabellera is a REV software engineer per his GitHub profile | B |
| wpilib_2026_frc_can_device_spec_can_addressing.rst | frc-docs `can-devices/can-addressing.rst` commit 1897febd8ae910a6e0de78388f62e7ccd6d8a085 (2026-09-19) | B |
| grayson-arendt_2026_sparkcan_{SparkBase.hpp,.cpp,README.md} | sparkcan commit 11d2d13787fc32e508690926efc3c9749e58770e. 12 stars, pushed 2026-07-15, active. README: "only work with firmware 24.0.X", i.e. the pre-25 legacy protocol | C |
| l5vel_2026_sparklib_{PROTOCOL,SPARK-MAX-REFERENCE}.md | sparklib-py commit 88a188affe5068209d324620cd01363f66d243c1. 1 star, active. Follows spec 2.1.0; hardware-measured on Flex 26.1.6 and MAX 24.0.1 (no MAX on 25+) | C |

Spec changes from 2.0.0-dev.11 to 2.1.0: added BOOTLOADER_0, STATUS_0 `SPARK_MODEL` (bits 54–57), STATUS_8 and STATUS_9; removed the SmartVelocity and SmartMotion setpoints. None of the frames we use changed.

## 3. Frame-by-frame

Arbitration ID = 2<<24 | 5<<16 | class<<10 | index<<6 | device (wpilib rst "Addressing"; JSON:4). Our `sparkId()` at `ino:162-168` matches.

| Frame | We send/decode | Reference says | Match? | Consequence | Reference |
|---|---|---|---|---|---|
| Universal heartbeat 0x01011840 | `78 01 00 12 59 04 00 60` every 20 ms | 8-byte packed little-endian struct: bit 25 enabled, bit 28 system watchdog → byte 3 = 0x12; roboRIO sends every 20 ms; devices disable 100 ms after the last one | Yes | Keeps the SPARKs permanently enabled (see §6) | wpilib rst:188-249 |
| SECONDARY_HEARTBEAT 0x2052C80 | 8×0xFF, 20 ms | 64-bit enable bitfield; ignored once the SPARK has locked onto the universal heartbeat (STATUS_0 bit 53: "until it is power cycled") | Yes, but redundant | Harmless | JSON:1525, :81 |
| VELOCITY_SETPOINT 0x2050000 | DLC 8, float RPM, bytes 4–7 = 0 | float setpoint; arb FF int16 bits 32–47 ×0.0009766; slot bits 48–49; FF units bit 50 (0 = V, 1 = duty) | Yes (slot 0, no arb FF) | Units depend on param 113, never read back over CAN | JSON:716 |
| DUTY_CYCLE_SETPOINT 0x2050080 | DLC 8 float, ±0.30 | same layout | Yes | – | JSON:764 |
| SET_STATUSES_ENABLED 0x2050400 | DLC 8, mask 0x0004 in bytes 0–1, enable in bytes 2–3, every 1 s | DLC 4 (uint16 mask, uint16 enable); response 0x2050440 DLC 5 | Partial | Tolerated in practice; not proven that this frame, rather than param 188, is what enables STATUS_2 | JSON:1025, :1040; DBC |
| STATUS_0 decode | voltage = bits 16–27 × 0.007326 | VOLTAGE uint12 ×0.0073260073; also applied output (int16 ×3.0824e-5), current (bits 28–39 ×0.03663 A), temperature, inverted, heartbeat lock | Yes | Applied output, current and temperature unused | JSON:81 |
| STATUS_2 decode | float velocity + float position | same; default 20 ms, off by default | Yes | – | JSON:419 |
| PARAMETER_WRITE 0x2053800 | DLC 5: ID + float32 | DLC 5: ID uint8 + 32-bit value typed per parameter; no type byte (unlike FW 24's class-48 path, sparkcan SparkBase.cpp:278) | Yes for 13–17 | Wrong for UINT32/BOOL parameters | JSON:5006 |
| PARAMETER_WRITE_RESPONSE 0x2053840 | not decoded | ID, type (1 int/2 uint/3 float/4 bool), current value, result 0–5 | Missing | Failed writes are invisible | JSON:5032 |
| PERSIST_PARAMETERS 0x205FFC0 | DLC 2, A3 3A | DLC 2, magic 15011 LE; "may take up to a second" | Yes | – | JSON:14882; 2026.0.5 header `…LENGTH (2u)` |
| PERSIST_PARAMETERS_RESPONSE 0x2050500 | decoded | DLC 1, "0 on success", no other codes | Yes | We get 255 | JSON:1089; DBC |
| RX device filter | decodes any extended frame without checking device type/manufacturer (`ino:256-258`) | – | Minor | Possible misdecode if another vendor's device is added | – |
| BURN wait | waits up to ~1.25 s (`ino:379-383`), exiting early once both replies arrive; no heartbeats, setpoints or E-lines meanwhile | 100 ms heartbeat timeout | – | If the wait exceeds 100 ms the SPARKs disable during BURN; the persist itself is sent while still enabled | wpilib rst:249 |

## 4. Parameters

| ID | FW 26 name | Type | Units | We send | Correct? | Reference |
|---|---|---|---|---|---|---|
| 13 | kP_0 | FLOAT | duty per error (REV's table gives position units; per RPM for velocity by analogy, **not confirmed**) | 0.0007 | ID/type yes | SparkParameters.java; closed_loop_units.md |
| 14 | kI_0 | FLOAT | duty per (error·ms) | 2.5e-7 (`KI2.5e-07`, `atof` parses it) | yes | same |
| 15 | kD_0 | FLOAT | (duty·ms) per error | 0 | yes | same |
| 16 | **kV_0** (kF_0 up to FW 25) | FLOAT | **V per RPM** | 0.000197 as if duty/RPM | **Probably wrong (~12×); test T1** | SparkParameters.java; FeedForwardConfig.java; closed_loop_units.md; examples 2025:72 / 2026:74-75 |
| 17 | kIZone_0 | FLOAT | velocity units (RPM), not stated explicitly | 600 | yes | ClosedLoopConfig.iZone |
| 6, 45, 59–61, 74, 137, 158–165, 186–193 (future) | IdleMode, Inverted, SmartCurrent, VoltageCompMode, UvwDepth, status periods, force-enable | UINT32/BOOL | – | not sent | would need non-float encoding | SparkParameters.java |

## 5. Feature coverage

| Feature | Do we support it? | Matters for velocity control? | Handled today by |
|---|---|---|---|
| Parameter READ: classes 15–22, RTR, DLC 8 (JSON:5386) | No | High | Hardware Client. Spec note: "SPARK MAX does not currently support this in v25.0.0-prerelease.4"; sparklib (C) got answers on Flex 26.1.6 only as RTR DLC 8; MAX 26.x not confirmed |
| Write-response decode (acts as a read-back) | No | High | – |
| kS (204) / kV (16) / kA (205) | kV only, with the wrong unit assumption | kS high (right track has 2× stiction), kV high, kA only in MAXMotion | – |
| PID slot selection (bits 48–49) | slot 0 only | low–medium | – |
| Arbitrary feedforward in the setpoint frame | always 0 | medium (host-side kS/kA without parameter writes) | – |
| kIZone (17) | yes | medium | pushed at startup |
| kIMaxAccum (96) | no | medium (anti-windup; negative side fixed in 26.1.0) | unknown |
| kDFilter (18) | no | low | – |
| Output min/max (19/20) | no | medium (must be ±1) | Hardware Client, checked once |
| Closed-loop ramp (114; REVLib writes 1/seconds) / open-loop ramp (56) | no | medium / low | Hardware Client (not verified) |
| Smart current limit (59/60/61, UINT32) | no | high (free-speed limit defaults to 20 A) | not recorded |
| Secondary current limit (11/12) | no | low–medium | – |
| Voltage compensation (74 UINT32 mode 2, 75 FLOAT) | no | medium–high (rail sags 8.5–12 V; kV is in volts) | Hardware Client "0" |
| Idle mode (6, brake = 1) / Inverted (45) | no | medium | Hardware Client |
| Hall velocity period/depth (136/137) | no | high (~112 ms lag) | defaults 32 ms / 8 samples |
| Quadrature average/delta (70/71) | no | none (affects only a brushed front-port encoder, per the REV engineer in issue 69) | – |
| Status periods (158–165, 199, 224; ms) | no | high (STATUS_2 period adds up to 20 ms) | defaults |
| Force-enable status (186–193) | no | medium | 1 s re-enable loop |
| CAN timeout | n/a | – | not a device parameter; the device side is the 100 ms heartbeat timeout |
| Factory / safe reset (class 1 index 7 magic 29741 / index 5 magic 36292) | no | low (REV recommends factory reset + persist after the first FW 25+ update) | – |
| CLEAR_FAULTS (class 6 index 14, DLC 0) | no | low | – |
| STATUS_1 faults and warnings (brownout, overcurrent, EEPROM, reset) | no | medium–high | – |
| Current and temperature (STATUS_0; GET_TEMPERATURES class 12 index 0) | no | medium | – |
| STATUS_7 (I accumulator) / STATUS_8 (setpoint, at-setpoint) | no | medium (windup diagnosis) | – |
| MAXMotion velocity (class 0 index 9, param 167 in RPM/s, uses kA) | no | medium | Teensy M-slew + actuator_node slew |
| GET_FIRMWARE_VERSION (class 9 index 8, RTR) | no | medium | Hardware Client |
| Closed-loop sensor (9, must be 1) / conversion factors (112/113) | no | high | Hardware Client |

## 6. BURN = 255

Our persist frame matches the spec exactly. 255 is documented nowhere the reviewer found; the spec's only code is "0 on success" (JSON:1089, DBC).

Candidate causes, most likely first:
1. **The SPARK is enabled.**
   - The heartbeat marks it enabled every 20 ms.
   - REVLib has error 26 `kCannotPersistParametersWhileEnabled`.
   - REVLib 2025.0.0 "Improves error description when attempting to persist parameters while the robot is enabled". That implies the device refuses, and that the earlier message was "Unknown error status Persist Parameters", as seen in issue 69.
   - Which raw code the firmware returns is **not confirmed**.
   - The same REV engineer states in issue 69 (2024-12-18) that persisting "must only be done when disabled".
2. **Malformed payload.** Unlikely. On the FW 24 legacy burn, sparklib (C) measured 0xFF for a bad magic or length; our payload matches the FW 25+ spec.
3. **EEPROM fault.** Unlikely. In issue 69 a REV engineer first suggested EEPROM faults ("green-orange LED blink"), then treated it as a separate, likely SPARK Flex-only issue; the reporting team said their LEDs were not green/orange.

Note: the root CLAUDE.md says the 2026-05-18 gains were burned via REV Hardware Client.

## 7. Disagreements between references

- **ID 16:** spec md v0.1.2 and the legacy docs call it "F 0 / kF_0". REVLib 2026 calls it `kV_0` in volts. REVLib 2026 wins for FW 26; the firmware notes are silent, hence T1.
- **Status period units:** the md says "in μs". JSON `defaultPeriodMs` and REVLib `SignalsConfig(periodMs)` say milliseconds. The μs is a typo.
- **UVW defaults:** md "0.03125, depth 3" vs REVLib "32 ms, 8 samples". Consistent: 0.03125 s ≈ 32 ms, and 3 is the index for 8 samples.
- **Header version labels:** the 2026.0.5 header says "2.0.0-dev.11" but contains STATUS_8/9 and BOOTLOADER_0. The 2027 header says "2.1.0" but has extra position-setpoint fields. Labels are unreliable, but none of the frames we use differ.
- **Parameter reads on a MAX:** the spec says unsupported (prerelease.4); sparklib shows reads working on Flex 26.1.6. Open for MAX 26.1.4.
- **Type codes:** firmware CLAUDE.md says "PTYPE_FLOAT = 2"; spec 2.1.0 says float = 3, and it appears only in response/type frames. Harmless today.
- **sparkcan (FW 24: class 48 + type byte, velocity class 1 index 2, burn class 63 index 2)** vs REV 2.1.0. REV wins; sparkcan is not valid for FW 26.
- **Firmware comment "universal heartbeat required on FW 25+"** vs release notes "Supports". Not confirmed as required.
- **Firmware CLAUDE.md "STATUS_2 on by default"** (line 95) vs spec `enabledByDefault: false`. The same file says "disabled" at line 127, so line 95 is stale. Spec wins.
- **FINDINGS Rank 3 "float32 for 60/74/76/77"** vs 60/74 being UINT32 and 76/77 removed. Spec wins.

## 8. Open questions and safe bench tests

Tracks off the ground, actuator_node stopped, one writer on the serial port.

- **T1: kV units.** Uses the current firmware over serial only.
  - Send five **separate lines** (the firmware parses one K command per line): `KP0`, `KI0`, `KD0`, `KZ0`, `KF0.000197`. Then `L1000 R1000` for 3 s (re-send within 300 ms), then `S`.
  - ~1000 RPM means duty/RPM. Near 0 means V/RPM (predicted duty ≈ 0.016, below the 0.03–0.06 stiction).
  - **Only if step 1 gave near 0 RPM,** try `KF0.00236` and expect ~1000 RPM. **Safety:** if step 1 gave ~1000 RPM, KF 0.00236 means 2.36 duty, so the output saturates at 100 % and, with KP = 0, the tracks run to ~5000–5500 RPM. `MAX_RPM` clamps only the setpoint, not the output.
  - Restore the yaml gains afterwards (RAM only).
- **T2: BURN cause.** Needs a temporary sketch or USB-CAN adapter; do not commit it.
  - (a) Listen to STATUS_1 (0x205B840|dev) and decode the EEPROM fault/warning bits 6/18/19/30/42/43.
  - (b) Stop the universal heartbeat, or send it with byte 3 = 0x00, for ≥300 ms (the secondary is ignored because of the lock). Confirm applied output is 0.
  - Before persisting: power-cycle or re-push the yaml gains and read them back, because PERSIST saves whatever is in RAM, including test values from T1, T4 or T6. Also confirm the secondary heartbeat is not re-enabling the device: check STATUS_0 bit 53 (heartbeat lock) or stop the secondary heartbeat, and confirm applied output is 0.
  - (c) Send PERSIST and read the response. 0 means "enabled" was the cause. Confirm with a power cycle and a T3 read-back.
- **T3: read-back (read-only).**
  - Send remote frames with DLC 8 to 0x2053E00|dev (parameters 16/17) and 0x2054A00|dev (112/113).
  - Send GET_FIRMWARE_VERSION as a remote frame to 0x2052600|dev (DLC 0 and 8). Expect 26.1.4; the build field is big-endian (JSON:1303).
- **T4: write check.** Re-send the same kP and capture 0x2053840|dev. Expect RESULT = 0, TYPE = 3, VALUE = 0.0007.
- **T5: SET_STATUSES_ENABLED length.** Capture the 0x2050440|dev response after DLC 8, then after DLC 4, and compare RESULT and the bitfield.
- **T6: hall-filter lag.** Needs a temporary sketch: the current firmware's K command handles only P/I/D/F/Z and always sends floats. In RAM only, with no BURN afterwards, write 136 = 0.016 (float) and 137 = 1 (uint32). Compare step-to-E-line lag and RPM noise before and after. Expected filter lag drops from ~112 ms to ~8 ms (formula not confirmed on FW 26).
- **T7: hidden caps (read-only via T3).** Read 9, 6, 19/20, 59–61, 74/75, 96, 114, 160.
- **Safety summary:** T3, T5 and T7 are read-only. T4 rewrites the same kP in RAM. None of them commands motion or touches the persist or reset frames. T1 commands motion (tracks off the ground). T2(c) writes flash.
- **Not confirmed, no safe test found:** whether the on-board PID uses the filtered hall velocity, and how kV volts become duty when voltage compensation is off.
