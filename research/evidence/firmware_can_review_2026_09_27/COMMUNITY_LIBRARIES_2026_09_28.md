# Community SPARK MAX/Flex CAN libraries (non-roboRIO): survey, 2026-09-28

Scope: open-source code that drives REV SPARK MAX or SPARK Flex over CAN without a roboRIO, or that documents the SPARK CAN protocol, focusing on firmware 25.x/26.x. Our controllers are SPARK MAX on 26.1.4 with NEO motors, driven from a Teensy 4.1 (FlexCAN). The v2 bridge is `firmware/teensy_diff_drive_v2/` (see its PROTOCOL.md).

Evidence copies are pinned to a commit in `references/community_2026_09_28/`, one file per source, named `<owner>_<repo>_<file>`. Line 1 of each copy is a header with the source URL and commit, so **line numbers below are for the saved copy, which is the upstream line + 1**. The one exception is `DiazPaz_movemaster_run203044_status_0_head.csv`, which holds only the first 40 data rows.

Evidence levels:
- **C**: established open-source code that I opened and read.
- **D**: community posts, README claims, or code comments that nobody has confirmed on hardware.

---

## 1. Best finds and why they matter to us

1. **The universal heartbeat byte order: our v2 "disabled" heartbeat very likely does not disable the SPARK. This is the most important result (C).** On the wire the WPILib RobotState is laid out **big-endian**: match time is in `data[7]` and System watchdog is `data[4] & 0x10`. Four independent sources agree:
   - The WPILib byte table puts "Match time" in byte 8 and "System watchdog" in byte 5, counting from 1 (`references/wpilib_2026_frc_can_device_spec_can_addressing.rst`, the table above the struct; still identical on wpilib-docs `main` @132f3ff).
   - FRC team 195's LED controller reads a real roboRIO heartbeat as `robotEnabled = (receivedData[4] & 0x10)` (`frcteam195_CKCANM4LEDController_main.cpp:139`).
   - A roboRIO heartbeat listener, which comes with a video of it following enable, uses `rx[4]` bit 4 (`sikaxn_FRC-Custom-CAN-Sensor_canHeartbeat.ino:100-101`). Its Python decoder reads the payload as a big-endian bit string: `match_time` at bit 56, `watchdog` at 35 (`sikaxn_FRC-Custom-CAN-Sensor_heartbeatreader.py:23,30`).
   - UTNuclearRobotics/FRCCan runs on Teensy with FlexCAN_T4. It enables a SPARK Flex by sending `buf[4] = 0x18`, test mode plus System watchdog (`UTNuclearRobotics_FRCCan_frcCan.cpp:136`, README `:46`).

   Our frame `78 01 00 12 59 04 00 60` turns out to be the **little-endian** packing of willGuimont/CanControl's `default_heartbeat()`: match time 120, match 1, enabled, watchdog, year 25, month 1, day 1, 12 h (`willGuimont_CanControl_heartbeat.h:58-70`, LE packing at `willGuimont_CanControl_frc_can.h:369`). Because the year is 25 (0x19), `data[4]` comes out as 0x59. Read big-endian, 0x59 has watchdog (0x10), test (0x08) and red (0x01) set. So the frame "works" under **both** byte orders, and it cannot tell us which one the SPARK uses.

   v2's disabled frame zeroes only `data[3]` (`teensy_diff_drive_v2.ino:458-459`). That leaves `data[4] = 0x59`, so a big-endian reader still sees System watchdog = 1. It is plausible that this is also why v1's BURN got result 255. **Suggested fix:** send 8 × `0x00` as the disabled heartbeat, which is disabled under either byte order, or stop sending the universal heartbeat entirely. Then confirm on the bench with STATUS_0 `hblock` and a velocity setpoint that must not move the motor.

2. **REVrobotics/node-can-bridge @7984f1d (C, official REV, the REV Hardware Client back end).** This is how REV itself enables a SPARK from a PC without a roboRIO:
   - SECONDARY_HEARTBEAT `0x2052C80` with an 8-byte enable bitfield, every 20 ms.
   - A "REV common heartbeat" at `0x00502C0` with a 1-byte payload: `{1}` enabled, `{0}` disabled.
   - To disable, it **keeps sending all-zero frames** instead of going silent (`REVrobotics_node-can-bridge_canWrapper.cc:25-31, 864, 933-944`; the watchdog pushes disabled frames at `:808-822`).

   v2 never sends `0x00502C0`. It also stops the secondary heartbeat during BURN instead of sending zeros. Both are harmless for a SPARK that is locked to the universal heartbeat, but REV's own disable pattern is "send zeros".

3. **DiazPaz/movemaster @c8de457 (C, with real hardware logs).** Python on a Raspberry Pi 5 and SocketCAN, frames from spark-frames 2.1.0 ("firmware 25+", README `:28`). The heartbeat is the universal `0x01011840` with 8 × `0xFF` only, and no secondary heartbeat (`DiazPaz_movemaster_ensayo_neo_can.py:56-57`). The committed run `ensayo_20260918_203044` logged **6600 STATUS_0 frames, all with PRIMARY_HEARTBEAT_LOCK = 1**, and drove a NEO from 0 to 100 % duty (`DiazPaz_movemaster_run203044_status_0_head.csv`, `DiazPaz_movemaster_run203044_resumen.csv`). This is direct evidence that a FW 25+ SPARK MAX locks to the universal heartbeat from a non-roboRIO host. Its persist handler treats RESULT_CODE 255 as "not yet, keep waiting" rather than as failure (`DiazPaz_movemaster_teach_pendant_backend.py:507-509`); that part is only tested against an emulator (README `:288`), so it is D.

4. **MacRover/spark_mmrt @7fc5e4f (C, 2026-09-24, MIT).** C++ with SocketCAN. It claims "firmware versions 25.0.X … tested and confirmed to work on 25.0.4" (`MacRover_spark_mmrt_README.md:3`). It covers:
   - typed PARAMETER_WRITE with response decode, and PERSIST with the response result;
   - STATUS 0-9 decode;
   - both the secondary heartbeat (`11/2`, FF × 8) and the universal heartbeat (`0x01011840`, FF × 8) (`MacRover_spark_mmrt_SparkFrames.hpp:47,60`, `MacRover_spark_mmrt_SparkFrames.cpp:111`, `MacRover_spark_mmrt_roboRIO.cpp:18`).

   Its parameter read does **not** use the READ_PARAMETER pair frames. It sends the **pre-25 legacy form**: a zero-length DATA frame to api `0x300 | id`, i.e. class 48 (`MacRover_spark_mmrt_SparkMax.cpp:193-195`), and its monitor example depends on that read. If that works on 25.0.4 it is a second read path to try on 26.1.4. It is unconfirmed: the repo has no log showing the reply.

5. **crumboe/rev_system_identification @3fb78ad (C code; its protocol notes are D).** A Flutter SysId tool (quasistatic and dynamic tests, kS/kV/kA/kG by OLS, WPILib JSON export) that talks to a SPARK over the **USB** CDC port, not CAN.
   - It writes kV to parameter 16 (`crumboe_rev_system_identification_spark_protocol.dart:284`), taken straight from a fit of volts against RPM.
   - Its reverse-engineered FW 26 USB notes claim a parameter read at class 7 / index 1 with a type tag (`crumboe_rev_system_identification_sparkmax_fw26_protocol.txt:4`).
   - Before persisting it stops the heartbeat and waits 1 s, with the comment "the device returns 0xFF (error) if other traffic is present on the bus" (`crumboe_rev_system_identification_parameter_api.dart:423-432`).
   - It mirrors the RHC2 heartbeat pattern (secondary heartbeat plus `0x000502C0`, 25 ms; `…spark_protocol.dart:69`, `…heartbeat.dart:35,121`).

   The notes also contain claims that are clearly wrong, such as "manufacturer changed from 0x15 to 0x05 in fw26" (`…fw26_protocol.txt:345`), so treat the whole document as D.

Also worth knowing: **willGuimont/CanControl** (PlatformIO registry, v1.2.1, MIT per library.json but the repo LICENSE is GPL-3.0) generates its SPARK frames from REV-Specs spark-frames-2.1.0. It targets MCP2515 and AVR, not FlexCAN.

---

## 2. Comparison table

| Repo | Lang / platform | FW supported (quoted) | Last commit | ★ / forks | License | Key features | Level |
|---|---|---|---|---|---|---|---|
| [REVrobotics/node-can-bridge](https://github.com/REVrobotics/node-can-bridge) | C++ / Node N-API, CANdle USB-CAN, Windows/Linux | Not stated. It is the RHC back end and uses the FW 25+ SECONDARY_HEARTBEAT ID | 2025-03-18 (7984f1d) | 1 / 1 | "NOASSERTION" (custom) | secondary heartbeat plus REV common heartbeat at 20 ms, disabled = zeros, ack watchdog (1 s) | C |
| [MacRover/spark_mmrt](https://github.com/MacRover/spark_mmrt) | C++17 / Linux SocketCAN | "supported on firmware versions 25.0.X … tested … on 25.0.4" | 2026-09-24 (7fc5e4f) | 0 / 0 | MIT | typed-ish param write plus response; legacy class-48 read; persist plus result; STATUS 0-9; secondary and universal heartbeats; duty/vel/pos/volt/current/MAXMotion; TUI control panel | C |
| [DiazPaz/movemaster](https://github.com/DiazPaz/movemaster) | Python / RPi 5, python-can SocketCAN | "Las tramas modernas … se introdujeron en firmware 25" | 2026-09-24 (c8de457) | 0 / 0 | none | JSON-driven codec for spark-frames 2.1.0; universal heartbeat FF×8 (**HB lock = 1 logged on hardware**); param write with read-back check; persist 0/255 handling; SET_STATUSES_ENABLED check; logged STATUS_0/2 datasets | C (logs) / D (persist) |
| [crumboe/rev_system_identification](https://github.com/crumboe/rev_system_identification) | Dart/Flutter / desktop, **USB** CDC (not CAN) | "SPARK MAX Firmware 26.x — USB Write API Reference" | 2026-04-23 (3fb78ad) | 0 / 1 | none | **SysId** (kS/kV/kA/kG, PID derivation); param read/write with type tag; persist; RHC2-style heartbeat; STATUS 0-9 | C code / D protocol notes |
| [willGuimont/CanControl](https://github.com/willGuimont/CanControl) | C++ / Arduino MCP2515, PlatformIO registry | "generated from … spark-frames-2.1.0" (README:320) | 2026-09-25 (d9724b2) | 7 / 1 | GPL-3.0 repo / MIT in library.json | every spec frame generated; RobotState packer (LE); queued controller | C |
| [UTNuclearRobotics/FRCCan](https://github.com/UTNuclearRobotics/FRCCan) | C++ / **Teensy FlexCAN_T4** | none stated; claims SPARK Flex works | 2026-07-24 (8e2d039) | 0 / 0 | NOASSERTION | generic FRC ID codec; universal heartbeat with `buf[4]=0x18` | C (code) / D (claim) |
| [grayson-arendt/sparkcan](https://github.com/grayson-arendt/sparkcan) (already had) | C++ / SocketCAN | "only work with firmware 24.0.X" | 2026-07-14 (**11d2d13, unchanged**) | 12 / 5 | MIT | FW 24 only | C |
| [vedAnts256/sparkcan_pybindings](https://github.com/vedAnts256/sparkcan_pybindings) | C++ + pybind11 / SocketCAN | "This works with firmware 25.0.X" | 2026-04-03 (239ad73) | 0 / 2 | MIT | sparkcan fork with a V25 API table; some IDs wrong (ClearFaults 7/14 vs spec 6/14) | C (low quality) |
| [l5vel/sparklib-py](https://github.com/l5vel/sparklib-py) (already had) | Python / SocketCAN | spec 2.1.0, Flex 26.1.6; MAX rig is 24.0.1 | 2026-09-11 (88a188a) | 1 / 0 | Apache-2.0 | most complete; no universal heartbeat | C |
| [turhans23/SparkMaxDriver](https://github.com/turhans23/SparkMaxDriver) | C / STM32 HAL | none; mixes FW 25 secondary heartbeat with legacy status IDs (`0x02051840`) | 2026-09-16 | 1 / 0 | MIT | duty plus legacy status | C (low) |
| [nolanpeterson07/esp-sparkmax](https://github.com/nolanpeterson07/esp-sparkmax) | C / ESP32 TWAI | none; legacy (FW ≤24) IDs | 2026-03-31 | 0 / 0 | none | duty/vel/pos, heartbeat 10 ms | C (low) |
| [npwtub/sparkmax-arduino-control](https://github.com/npwtub/sparkmax-arduino-control) | C++ / Arduino MCP2515 | none | 2026-05-02 | 0 / 0 | MIT | stub; protocol notes are TODO | D |
| [PolarRobotics/MCP2515-SPARK-CAN](https://github.com/PolarRobotics/MCP2515-SPARK-CAN) | C / Arduino MCP2515 | none | 2026-03-23 | 0 / 0 | MIT | basic | C (low) |
| [frcteam195/CKCANM4LEDController](https://github.com/frcteam195/CKCANM4LEDController) | C++ / Feather M4 CAN | n/a (heartbeat **consumer** on a real roboRIO) | f63fc34 | – | – | reads `data[4] & 0x10` as robot enabled | C |
| [sikaxn/FRC-Custom-CAN-Sensor](https://github.com/sikaxn/FRC-Custom-CAN-Sensor) | Arduino/ESP32 + Python | n/a (heartbeat decoder) | 2026-09-03 (0047f66) | 12 / 2 | GPL-3.0 | big-endian RobotState decoder, roboRIO video | C / D |

---

## 3. Answers to the open questions

**(a) Does SPARK MAX FW 26.x answer parameter-read (remote-frame) requests?** Still **not confirmed by anyone for a MAX on 26.x.**
- l5vel confirms that a *Flex* on 26.1.6 answers a remote frame with DLC 8 (already in our references).
- The only MAX 25+ implementation, MacRover, uses the **legacy** read instead: a zero-length data frame to `0x02050000 | ((0x300|id) << 6) | dev` (`MacRover_spark_mmrt_SparkMax.cpp:193-195`, D for "it works on 25.0.4").
- crumboe claims that FW 26 answers a class-7 / index-1 read over **USB** (`crumboe_rev_system_identification_sparkmax_fw26_protocol.txt:4`; the read ID is at `…spark_protocol.dart:52`), D.

Action: keep v2's RTR DLC-8 attempt. As a fallback, add the legacy class-48 zero-DLC data read, and log which form answers.

**(b) What disables a SPARK for persist?**
- REV's own tool disables by **sending zero frames**: secondary heartbeat `00×8`, REV common heartbeat `0x00502C0` `{0}` (`REVrobotics_node-can-bridge_canWrapper.cc:30-31, 933-944`).
- For the universal heartbeat, the bit the SPARK checks is System watchdog (RST: "If the System watchdog flag is set, motor controllers are enabled"). On the wire that bit is **`data[4]` bit 4 (0x10), not `data[3]`** (see §1 item 1: `frcteam195_CKCANM4LEDController_main.cpp:139`, `sikaxn_…canHeartbeat.ino:100-101`, `UTNuclearRobotics_FRCCan_frcCan.cpp:136`).
- crumboe stops all heartbeats for 1 s before persisting and reports 0xFF otherwise (`crumboe_rev_system_identification_parameter_api.dart:423-432`, D).
- movemaster persists only while "disarmed", meaning no heartbeat, and waits past 255 (`DiazPaz_movemaster_teach_pendant_backend.py:507-509`, D).

**(c) kV units on FW 26.** There is no new first-party evidence here, so REPORT §1.1 still stands.
- The one community SysId tool fits V = kS + kV·ω with ω in RPM when the conversion factor is 1, and writes kV into parameter 16 / `feedForward.kV` unchanged (`crumboe_rev_system_identification_spark_protocol.dart:284`; the exporter is in the repo at `lib/data/code_snippet_exporter.dart:300-302`). That is consistent with **volts per RPM**, at level D for FW 26.
- MacRover still calls parameter 16 "F" and treats it as a plain float.
- Nobody tested this on a bench.

**(d) Hall velocity filter on FW 26.**
- No community code writes parameters 136/137 on 26.x.
- The only new item comes from REV through WPILib: REVLib 2027.0.0-alpha-7 "Removes hall sensor velocity averaging configurations in favor of new firmware filtering system" and "Adjusts default encoder average depth to 8 and sample delta to 20" (`wpilibsuite_SystemCoreTesting_REV.md:32-33`). That points to firmware after 26 replacing the hall filter. It says nothing about how 26.1.4 behaves, so the question stays open and bench test T-hall is still needed.

**(e) Quirks with the universal heartbeat from a non-roboRIO host.**
1. **Byte order.** The spec's C struct suggests little-endian, but real roboRIO traffic is big-endian: System watchdog is `data[4] & 0x10` (§1 item 1).
2. **8 × `0xFF` works and locks.** movemaster logged PRIMARY_HEARTBEAT_LOCK = 1 on 6600/6600 frames (`DiazPaz_movemaster_run203044_status_0_head.csv`). sparkcan and MacRover also use FF×8.
3. **FW 24.0.1 ignores `0x01011840` entirely** (l5vel, already in references). The universal heartbeat is FW 25+ only.
4. **REV's own host tool sends the secondary heartbeat plus `0x00502C0` rather than the universal heartbeat.** It also stops heartbeating when its GUI stops acknowledging for 1 s (`REVrobotics_node-can-bridge_canWrapper.cc:796-822`).
5. **Only one heartbeat owner.** movemaster warns that a second heartbeat sender defeats disabling (`DiazPaz_movemaster_README.md`, "un único propietario del heartbeat"). RHC 1.7.0 locks control to one instance per bus.

---

## 4. Where v2 differs from newer FW 25/26 implementations

| Item | v2 (`teensy_diff_drive_v2.ino`) | Newer implementations | Risk |
|---|---|---|---|
| Universal heartbeat, disabled | `78 01 00 00 59 04 00 60` (only `data[3]` cleared; ino:459) | System watchdog is `data[4] & 0x10` (team 195, sikaxn, FRCCan, RST byte table) | **High:** BURN likely still sees an enabled SPARK, which fits the 255 result. Send `00×8` or stop sending. |
| Universal heartbeat, enabled | `78 01 00 12 59 04 00 60` | FF×8 (sparkcan, MacRover, movemaster), `buf[4]=0x18` (FRCCan) | Low: ours is enabled under both byte orders, but only because year 25 → `0x59`. The comment at ino:453-457 and :205-207 describes the wrong byte. |
| Secondary heartbeat while disabled | stopped | REV sends `00×8` every 20 ms (node-can-bridge:814-816) | Low. Matching REV costs nothing. |
| REV common heartbeat `0x00502C0` | not sent | RHC sends `{1}`/`{0}`, 1 byte, 20 ms | Unknown. Only RHC uses it, and no SPARK spec lists it. Not needed while the universal heartbeat lock holds. |
| Param read | RTR, DLC 8, pair frame | Flex 26.1.6: same (l5vel). MAX 25.0.4: legacy class-48 zero-DLC data frame (MacRover) | Medium: add the legacy form as a fallback |
| PARAMETER_WRITE | DLC 5, typed | MacRover DLC 5 (`MacRover_spark_mmrt_SparkFrames.cpp:297`), CanControl generated from 2.1.0 | Match |
| PERSIST | `A3 3A`, DLC 2, `0x205FFC0` | same in MacRover (`…SparkFrames.cpp:316-320`), vedAnts256, crumboe, movemaster | Match. movemaster treats 255 as "pending", which v2 could emulate by waiting for a later 0 until its 1.5 s timeout. |
| SET_STATUSES_ENABLED | DLC 4 | movemaster uses the JSON length (4) and checks the response | Match |
| Setpoint frames | DLC 8, arbFF 0, slot 0 | MacRover DLC 8, slot in byte 6 | Match |

---

## 5. Searched and not found

- **Maintained ROS 2 / ros2_control SPARK MAX FW 25+ driver:** none. Moonrockers/2026-2027-Rover has only an open issue (#8), and educationmoment, rat-trak and TennesseeLunabotics vendor FW 24 sparkcan copies.
- **Rust:** no SPARK crate beyond `arfur`, which wraps REVLib on a roboRIO; nothing found for SocketCAN or FW 25+.
- **REV GitHub org:** no Linux, SocketCAN, USB or CLI tool for SPARKs. Only node-can-bridge (Windows CANdle driver, RHC back end), CANBridge (netcomm emulation, pushed 2026-04), REV-Specs 2.1.0 (unchanged since 2026-01-02) and node-revlog-converter. No REV Hardware Client CLI. REV-Software-Binaries issue #4 "Use REVLib without roboRIO" has no answer.
- **WPILib 2027 / SystemCore:** REVLib takes `CANPort`, SystemCore has its own CAN buses, and hall averaging is removed in REVLib 2027 alpha-7 (above). The heartbeat doc only added an FTC Motor Override bit (bit 23, wpilib-docs a8c3210). No SPARK frame changes were found. Alternative path: `ThadHouse/HALSIM_SocketCAN` (2019), used by vikings204/reefscaperos to run REVLib on Linux over SocketCAN, but it is unmaintained and was never tested with FW 25+.
- **Any community statement on kV units, the persist 255 meaning, or which heartbeat bit FW 26 checks:** not found beyond the above. No Chief Delphi thread with a FW 25+ non-roboRIO repo turned up (D-level search only).
- **Arduino, Teensy, PlatformIO or ESP32 libraries with FW 25+ parameter read/write and persist:** only CanControl (MCP2515) and FRCCan (Teensy, heartbeat only).
- **Code searches with no hits on FW 25+ SPARK code outside the repos above:** `0x2053800`, `0x205B880`, `0x205ffc0`, `PERSIST_PARAMETERS 15011`, `SPARK_SECONDARY_HEARTBEAT`, `PRIMARY_HEARTBEAT_LOCK`, `READ_PARAMETER_0_AND_1`, "spark max socketcan", "sparkflex socketcan", "spark max ros2", "spark max rust", "spark max teensy", "spark max esp32".
