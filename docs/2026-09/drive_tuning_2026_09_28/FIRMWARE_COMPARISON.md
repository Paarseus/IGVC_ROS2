> **Status: historical (firmware 25 / Teensy v2).** The live setup is SPARK firmware 26.1.5 + Teensy v2d; see `README.md` and `firmware/teensy_diff_drive_v2/PROTOCOL.md`. The comparison with sparkcan still applies.

# Firmware Comparison: Our Teensy Bridge vs sparkcan vs REV's Specification

**Question:** is our firmware doing everything correctly, and as close as possible to a mature SPARK MAX library?

**Short answer:**
- **sparkcan cannot be copied.** Its README says it "will only work with firmware 24.0.X, it will not work with the new 25.0.X releases". Our controllers run **26.1.4**, which uses REV's new CAN protocol: different message IDs and parameter-write format, and no type byte. Sending sparkcan's messages would be sending wrong messages.
- The correct reference for the message **format** is REV's own specification (REV-Specs `spark-frames-2.1.0` + REVLib 2026.0.5 driver headers). sparkcan is used as the reference for **which features** a complete library offers.
- **v1** (deployed): message format correct, but missing the features needed for precise tuning (write confirmation, read-back, typed parameters, per-side gains, faults, voltage mode, safe BURN).
- **v2** (`firmware/teensy_diff_drive_v2/`): REV-Specs format for firmware 26 plus the sparkcan feature set that matters for tuning. It is backward compatible with actuator_node. Independent review in progress (`firmware/teensy_diff_drive_v2/REVIEW.md`).

Sources:
- sparkcan: github.com/grayson-arendt/sparkcan, commit 11d2d13 (2026-07-14), `include/SparkBase.hpp`.
- REV-Specs and REVLib: `research/evidence/firmware_can_review_2026_09_27/references/`.
- v1 review: `research/evidence/firmware_can_review_2026_09_27/REPORT.md` (verified).

## 1. Protocol format: why sparkcan's messages are wrong for firmware 26

| Message | sparkcan (FW 24) | REV spec for FW 25+/26 | v1 | v2 |
|---|---|---|---|---|
| Velocity setpoint | class 1, index 2 | class 0, index 0 | class 0, index 0 ✔ | ✔ |
| Duty-cycle setpoint | class 0, index 2 | class 0, index 2 | ✔ | ✔ |
| Voltage setpoint | class 4, index 2 | class 0, index 5 | — | ✔ |
| Parameter write | class 48 + type byte | class 14, index 0; 5 bytes, value typed per parameter, no type byte | ✔ for floats only | ✔ all types |
| Save to flash (BURN) | class 63, index 2 | class 63, index 15, magic 0x3AA3, 2 bytes | ✔ frame; sent while enabled (rejected) | ✔ frame; sent **disabled** |
| Status frames | "Period 0–4" (FW 24 layouts) | STATUS_0–9 (class 46) with new layouts | STATUS_0 voltage, STATUS_2 | STATUS_0 (all), STATUS_1 (faults), STATUS_2 |
| Heartbeat | class 11, index 2 (non-roboRIO) | roboRIO universal heartbeat (big-endian bitfield: SystemWatchdog = `data[4]` bit 4) + secondary heartbeat | ✔ both (enabled frame works by coincidence of the year byte) | ✔ both; **disabled = all zeros** (the v1-style partial clear left the controllers enabled); `HB0` bench test |

## 2. Feature coverage (sparkcan's feature list, implemented in REV's FW 26 format)

| Feature | sparkcan | v1 | v2 | Matters for tuning? |
|---|---|---|---|---|
| Velocity / duty setpoints | ✔ | ✔ | ✔ | yes |
| Voltage setpoint (identify feedforward directly in volts) | ✔ | — | ✔ `UVL/UVR` | **yes**: kV is in volts on FW 26 |
| Write any parameter with the correct type | ✔ (typed setters) | floats only | ✔ `PW` + typed table | **yes**: current limits, idle mode, filter depth and status periods are integers or booleans |
| Confirmation of each write | — (fire and forget) | — | ✔ `PWR` (controller's value + result code) | **yes**: proves the controller has the value |
| Parameter read-back | ✔ (getters) | — | ✔ `PR` (may be unanswered on MAX 26.1.4; the spec leaves this open) | yes |
| Per-side gains | per device | both sides only | ✔ `K<x><L/R>` | **yes**: the right track needs about 2× the static friction |
| kS / kA feedforward | — (FW 24 has only kF) | — | ✔ | **yes** |
| Hall velocity filter (period, depth) | ✔ | — | ✔ (params 136/137) | **yes**: about 112 ms speed-reading lag at default |
| Status frame periods | ✔ | — | ✔ (params 158–160) | yes |
| Current limit, idle mode, voltage compensation, ramps, output limits, I max accumulator | ✔ | — | ✔ | yes |
| Faults, sticky faults, warnings | ✔ (Period 0) | — | ✔ STATUS_1 decoded by name, `CF` to clear | yes: brownouts, overcurrent |
| Applied output, current, temperature | ✔ | — | ✔ | yes (identification, heating) |
| Firmware version | ✔ | — | ✔ `FV` | yes (confirm 26.1.4) |
| Identify (LED blink) | ✔ | — | ✔ | convenience |
| Factory reset / defaults | ✔ | — | — (left out on purpose) | no (risky in the field) |
| Position, SmartMotion, limit switches, analog, alternate encoder, follower | ✔ | — | — | no (not used on this robot) |
| Timestamped telemetry for lag measurement | — | — | ✔ `X` line (Teensy µs arrival times) | **yes** |
| Safe BURN (refuse while moving, disable, persist, report) | — | — | ✔ | yes |

## 2b. Newer community libraries (survey 2026-09-28)
See `research/evidence/firmware_can_review_2026_09_27/COMMUNITY_LIBRARIES_2026_09_28.md`. There is no maintained FW 25/26 library to adopt wholesale. The useful ones are:
- **REV node-can-bridge:** REV's own code; disables with zeros.
- **MacRover/spark_mmrt:** C++, tested on FW 25.0.4.
- **DiazPaz/movemaster:** Python; hardware log shows heartbeat lock with the universal heartbeat.
- **willGuimont/CanControl:** generated from spark-frames 2.1.0.
- **crumboe/rev_system_identification:** SysId over USB, community level.

The survey found the heartbeat byte-order bug, fixed in v2.

## 3. What remains open (needs the bench test)
1. **kV units on the real controller:** the firmware can't settle this. The first bench test confirms whether parameter 16 behaves as volts per RPM (REV docs) before any tuning.
2. **Whether SPARK MAX 26.1.4 answers parameter reads:** if not, the write confirmation (`PWR`) still returns the controller's current value for every write.
3. **Whether the controller accepts BURN once disabled** (the likely cause of the old result code 255).
