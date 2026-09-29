# Verification of REPORT.md (Teensy SPARK MAX CAN firmware vs REV spec)

- **Date:** 2026-09-27
- **Reviewer:** independent (checked only; no new research, no edits to REPORT.md or code)
- **Method:** every claim opened at its cited location in `references/` and `firmware_snapshot/`. Arbitration IDs recomputed as `2<<24 | 5<<16 | class<<10 | index<<6 | dev` and compared with the `arbId` fields parsed from both JSON specs. The snapshot `.ino` is byte-identical to `git show f242825:firmware/teensy_diff_drive/teensy_diff_drive.ino`. Note that it is **not** identical to the file on `main` (`firmware/teensy_diff_drive/teensy_diff_drive.ino`, last touched in bccb3b5), which lacks the burn-response decode and the `M` slew.

## Counts

| Status | Count |
|---|---|
| Verified | 73 |
| Partly supported | 10 |
| Not supported | 0 |
| **Total claims checked** | **83** |

Recomputed IDs (all match the spec `arbId`): heartbeat 0x01011840 · SECONDARY_HEARTBEAT 11/2 = 0x2052C80 · VELOCITY 0/0 = 0x2050000 · DUTY 0/2 = 0x2050080 · SET_STATUSES_ENABLED 1/0 = 0x2050400 · its response 1/1 = 0x2050440 · PERSIST_RESPONSE 1/4 = 0x2050500 · PARAMETER_WRITE 14/0 = 0x2053800 · PARAMETER_WRITE_RESPONSE 14/1 = 0x2053840 · PERSIST 63/15 = 0x205FFC0 · STATUS_0 46/0 = 0x205B800 · STATUS_1 46/1 = 0x205B840 · STATUS_2 46/2 = 0x205B880 · GET_FIRMWARE_VERSION 9/8 = 0x2052600 · READ_PARAMETER_16_AND_17 15/8 = 0x2053E00 · READ_PARAMETER_112_AND_113 18/8 = 0x2054A00.

## Claims

| # | Claim (short) | Citation | Status | Evidence | Correction needed |
|---|---|---|---|---|---|
| **Summary §1** |||||
| 1 | ID 16 is `kV_0(16, FLOAT)` in REVLib 2026 | SparkParameters.java | Verified | line 51 `kV_0(16, Type.FLOAT)`; driver header `c_Spark_kV_0 = 16` | – |
| 2 | kV documented as "Volts per velocity" | FeedForwardConfig.java | Verified | lines 80, 180 "@param kV The kV gain in Volts per velocity"; line 187 writes it to param 16 as a float | – |
| 3 | Units table: "kV: Volts per RPM" | closed_loop_units.md | Verified | "\| kV \| Volts per RPM \| Velocity Conversion Factor \|"; feedforward_control.md:102 agrees | – |
| 4 | Example changed from `velocityFF(1.0/5767)` to `kV(12.0/5767)`, "kV is now in Volts…" | 2025 example:72; 2026 example:74-75 | Verified | exact text at those lines | Minor: the "2026" example was fetched from `main`, not a pinned 2026 tag |
| 5 | We send 0.000197 = 1/5072. Read as V/RPM, 4514 RPM gives 0.89 V, about 7 % duty at 12 V, about 12× too weak | FINDINGS.md; arithmetic | Verified | 1/5072 = 1.9716e-4; 4514 × 1.97e-4 = 0.889; 0.889/12 = 7.4 %; 12× | – |
| 6 | Inference: kP 0.0004 with the Phase-4 plant predicts about 69 % (volts) vs about 101 % (duty) at 1500 RPM; issue #6 measured about 70 % | FINDINGS.md:26, :78 | Partly supported | Arithmetic is correct: duty reading gives y = 1512 RPM (100.8 %); volts reading at 12 V gives y = 1042 RPM (69.5 %) | The ~70 % is from the root CLAUDE.md issue-#6 row, not FINDINGS. FINDINGS:39 records **82 %** at kP = 0.0004, which fits neither prediction. State the assumptions: left-wheel plant only, kI = 0, 12 V bus. |
| 7 | Fix direction kV ≈ 12 × 0.000197 ≈ 0.0024 | arithmetic | Verified | 0.002364 | – |
| 8 | sm-26.1.0 notes say only "Adds expanded feedforward support to all PID modes" | release notes | Verified | line 70 (and prerelease line 453) | – |
| 9 | Heartbeat Enabled + System watchdog sent every 20 ms | ino:185, ino:524 | Verified | `{0x78,0x01,0x00,0x12,...}`; `sendHeartbeats()` in the 20 ms tick | – |
| 10 | `kCannotPersistParametersWhileEnabled = 26` | REVLibError.h; REVLibError.java:58 | Verified | .h:60 and .java:58 | – |
| 11 | REVLib 2025.0.0 "Improves error description when attempting to persist parameters while the robot is enabled" | release notes | Verified | line 385, inside the revlib-2025.0.0 section (from line 327) | – |
| 12 | Spec defines only "0 on success" for the persist response | JSON:1089 | Verified | JSON:1089–1106; DBC:151 | – |
| 13 | BURN 255 = enabled is "most likely", **not confirmed**; EEPROM fault is the second candidate | issue 69 | Partly supported | Issue 69, jfabellera (REV) 2024-12-18: "The error is a response from the device saying that the action (persist parameters) failed because the robot is enabled… persisting parameters must only be done when disabled." | The evidence for the refusal mechanism is **stronger** than the report says: a REV engineer states it directly. Only the raw code 255 is unconfirmed. The EEPROM lead was withdrawn in the same thread (see #52). |
| 14 | PARAMETER_WRITE_RESPONSE is class 14 index 1, DLC 7: ID, type, current value, result 0–5 | JSON:5032 | Verified | arbId 0x2053840, lengthBytes 7; ID bits 0–7, type bits 8–15 ("0 Unused, 1 Int, 2 Uint, 3 Float, 4 Boolean"), value bits 16–47 ("current value… will not match… if the write failed"), result bits 48–55 ("0 Success, 1 Invalid ID, 2 Mismatched Type, 3 Access Mode, 4 Invalid, 5 Not Implemented") | – |
| 15 | Firmware prints `OK K<x>=` before any CAN reply | ino:427-428 | Verified | `tuneBoth(p,val); Serial.printf("OK K%c=...")`, with no response decode | – |
| 16 | actuator_node logs that as "end-to-end confirmation" | actuator_node.py | Verified | lines 603–605 | – |
| 17 | About 112 ms of the ~185 ms lag comes from the hall filter: sampled every 32 ms, 8 samples, (8−1)/2 × 32 = 112 | issue258 | Partly supported | The quote and formula are verbatim | The source is a **2022** REV Support statement (pre-FW25 firmware) relayed by a community user (Piphi5), and it says REV "is working on lowering this latency". Applicability to FW 26.1.4 is unconfirmed. The "~185 ms" total is not in any reference. |
| 18 | 136 `kUvwSensorSampleRate` FLOAT seconds, 8–64 ms; 137 `kUvwSensorAverageDepth` UINT32 index 0–3 = 1/2/4/8 | EncoderConfig.java | Verified | lines 153–187: depth maps 1→0, 2→1, 4→2, else→3; period ms/1000 as a float, "range [8, 64]"; SparkParameters:127–128 | – |
| 19 | SET_STATUSES_ENABLED sent with DLC 8; the spec says 4; mask and enable are correct | ino:209-213; JSON:1025 | Verified | `canSend(...,d,8)`; spec lengthBytes 4, MASK bits 0–15, ENABLED bits 16–31; DBC:135 `: 4`; driver `SET_STATUSES_ENABLED_LENGTH (4u)` | – |
| 20 | Its response (class 1 index 1) is ignored | ino:249-284 | Verified | no 1/1 decode | – |
| 21 | `setParam()` always writes float32 | ino:218-223 | Verified | `fromFloat(value, d+1)` | – |
| 22 | Value type "depends on the Parameter Type" | JSON:5006 | Verified | VALUE description at JSON:5016 | – |
| 23 | 6, 45, 59–61, 74, 137, 158–165 are UINT32/BOOL | SparkParameters.java | Verified | 6 U32, 45 BOOL, 59/60/61 U32, 74 U32, 137 U32, 158–165 U32 | – |
| 24 | FINDINGS Rank 3 "float32 for 60/74/76/77" is wrong; 76/77 removed in sm-26.1.0 | FINDINGS:191; release notes | Verified | FINDINGS:191 "[param_id_u8, float32_LE]"; notes line 79 "Removes SmartMotion"; 76/77 absent from SparkParameters.java 2026.0.5 | – |
| 25 | Correct as-is: ID layout, heartbeat, setpoints, STATUS_0 voltage, STATUS_2, PARAMETER_WRITE for 13–17, persist frame and response | see §3 rows | Verified | see #29–41 | – |
| 26 | 2027.0.0-alpha-7 "Removes hall sensor velocity averaging…" and "Removes Conversion Factors"; 112/113/136/137 absent from the 2027 header | release notes; 2027 Parameters.h | Verified | notes lines 473, 475; header diff shows 112/113/136/137 removed | – |
| 27 | Root CLAUDE.md says `S` = MODE_DUTY=0, but firmware `S` sets velocity 0 RPM | ino:326-338 | Verified | `ctrl_mode = MODE_VELOCITY`; root CLAUDE.md line 232 | – |
| 28 | Reviewed file is deployed commit f242825 | – | Verified (commit only) | snapshot == `git show f242825:…` | "Deployed" is not verifiable here; `main`'s copy differs |
| **Frames §3** |||||
| 29 | `sparkId()` matches the FRC addressing | ino:162-168; rst; JSON:4 | Verified | type<<24, mfg<<16, cls<<10, idx<<6, dev; JSON:4 type 2, mfg 5 | – |
| 30 | Heartbeat 0x01011840: bit 25 enabled, bit 28 watchdog, so byte 3 = 0x12; 20 ms; 100 ms disable | rst:188-249 | Verified | packed struct: matchTime 8 + matchNumber 10 + replay 5 + ftc 1 + red 1 → enabled = bit 25, watchdog = bit 28, giving byte 3 = 0x02 \| 0x10 = 0x12; rst:190, :249 | – |
| 31 | SECONDARY_HEARTBEAT 0x2052C80, 64-bit field, ignored once locked | JSON:1525, :81 | Verified | JSON:1527 "only gets respected when the SPARK is not locked to the Universal Heartbeat"; STATUS_0 bit 53 "until it is power cycled" | Minor: bit 53 is at JSON:189, not :81 (the STATUS_0 block starts at 81) |
| 32 | VELOCITY_SETPOINT: float; arb FF int16 bits 32–47 × 0.0009766; slot 48–49; FF units bit 50 (0 = V, 1 = duty) | JSON:716 | Verified | scale 0.0009765923; units "0: Voltage, 1: Duty Cycle" | – |
| 33 | We send DLC 8 with bytes 4–7 = 0 | ino:192-198 | Verified | `uint8_t d[8] = {0}` | – |
| 34 | DUTY_CYCLE_SETPOINT 0x2050080, same layout, ±0.30 | JSON:764; ino:89, :200-206 | Verified | – | – |
| 35 | SET_STATUSES_ENABLED response 0x2050440 DLC 5 | JSON:1040; DBC | Verified | lengthBytes 5; DBC:140 | – |
| 36 | STATUS_0 voltage bits 16–27 × 0.007326; current bits 28–39 × 0.03663; applied output int16 × 3.0824e-5 | JSON:81; ino:263-264 | Verified | 0.0073260073, 0.0366300366, 3.0823695e-5 | – |
| 37 | STATUS_2: float velocity and position, default 20 ms, off by default | JSON:419 | Verified | parsed: defaultPeriodMs 20, enabledByDefault **false** (both spec versions) | – |
| 38 | PARAMETER_WRITE 0x2053800 DLC 5 = ID uint8 + 32-bit value, no type byte; FW 24 sparkcan uses class 48 + type byte | JSON:5006; sparkcan cpp:278 | Verified | JSON lengthBytes 5; sparkcan cpp:182 class 48, cpp:277 `data[4] = parameterType`, cpp:278 send | – |
| 39 | PERSIST 0x205FFC0 DLC 2, magic 15011 LE (A3 3A), "may take up to a second" | JSON:14882; 2026.0.5 header | Verified | 15011 = 0x3AA3; driver `SPARK_PERSIST_PARAMETERS_LENGTH (2u)`; DBC:2795 | – |
| 40 | PERSIST_RESPONSE 0x2050500 DLC 1, "0 on success" | JSON:1089; DBC | Verified | DBC:149-151 | – |
| 41 | RX decodes any extended frame without checking type or manufacturer | ino:256-258 | Verified | only `msg.flags.extended` is checked (ino:253) | – |
| 42 | BURN blocks 1.2 s with no heartbeats, setpoints or E-lines, so the SPARKs disable during BURN | ino:379-383 | Partly supported | the loop waits `< 1200 ms` **or until both results arrive**, plus a 50 ms delay between frames | Say "up to ~1.25 s". If both replies (e.g. 255) arrive quickly the gap can be shorter than 100 ms and the SPARKs may not disable. The persist is still sent while enabled, which is the point. |
| **Spec diff §2** |||||
| 43 | 2.0.0-dev.11 → 2.1.0 added BOOTLOADER_0, STATUS_0 SPARK_MODEL (bits 54–57), STATUS_8/9; removed SmartVelocity/SmartMotion; frames we use unchanged | both JSON | Verified | parsed diff; all used frames are equal, and STATUS_0 differs only by SPARK_MODEL (bit 54, 4 bits) | – |
| **Parameters §4** |||||
| 44 | 13 kP_0 FLOAT, duty per error (velocity by analogy, not confirmed) | SparkParameters; units.md | Partly supported | the units table gives only "Duty cycle per rotation" (position) | None; the report already flags it |
| 45 | 14 kI_0 FLOAT; `atof("2.5e-07")` parses | same; ino:417 | Verified | – | – |
| 46 | 15 kD_0 FLOAT | same | Verified | – | – |
| 47 | 16 kV_0 (kF_0 up to FW 25), V/RPM | see #1–4 | Verified (docs) | hardware behaviour is unconfirmed, as the report says | – |
| 48 | 17 kIZone_0 FLOAT, RPM units not stated | ClosedLoopConfig.iZone | Partly supported | javadoc: "The integral zone value" only | None; the report already flags it |
| 49 | 186–193 force-enable BOOL | SparkParameters:173-180 | Verified | – | – |
| **Features §5** |||||
| 50 | Parameter READ classes 15–22, RTR, DLC 8; spec note "SPARK MAX does not currently support this in v25.0.0-prerelease.4"; sparklib reads on Flex 26.1.6 as RTR DLC 8 | JSON:5386; sparklib | Verified | 128 read frames, classes 15–22, rtr true; PROTOCOL.md:69-72; REFERENCE.md:65 | – |
| 51 | kS 204 / kA 205 FLOAT; kA only in MAXMotion; right track has 2× stiction | SparkParameters:191-192; feedforward doc; FINDINGS:19-20 | Verified | compatibility table: kA false for Velocity mode | – |
| 52 | kIMaxAccum 96, negative-side fix in 26.1.0 | notes:68 | Verified | – | – |
| 53 | 18 kDFilter, 19/20 output min/max, 114 closed-loop ramp (REVLib writes 1/s), 56 open-loop ramp | SparkParameters; SparkBaseConfig:363-383 | Verified | `rate = 1.0 / rate` | – |
| 54 | 59/60/61 UINT32; free limit defaults to 20 A | SparkParameters; params md:127 | Verified | – | – |
| 55 | 11/12 secondary current limit | SparkParameters:46-47 | Verified | kCurrentChop / kCurrentChopCycles | – |
| 56 | Voltage compensation 74 UINT32 mode 2, 75 FLOAT; rail sags 8.5–12 V | SparkBaseConfig:395-396; FINDINGS | Verified | – | – |
| 57 | Idle mode 6 (brake = 1), Inverted 45 | SparkBaseConfig:43-45 | Verified | – | – |
| 58 | Hall defaults 32 ms / 8 samples | EncoderConfig; md:203-204 | Verified | – | – |
| 59 | 70/71 affect only a brushed front-port encoder (REV engineer) | issue 69:116 | Verified | – | – |
| 60 | Status periods 158–165, 199, 224 in ms | SparkParameters; SignalsConfig | Verified | – | – |
| 61 | Factory reset 1/7 magic 29741; safe reset 1/5 magic 36292; REV recommends factory reset + persist on the first FW 25+ update | JSON:~1114-1190; notes:20 | Verified | – | – |
| 62 | CLEAR_FAULTS 6/14 DLC 0; GET_TEMPERATURES 12/0; STATUS_7 I-accum; STATUS_8 setpoint/at-setpoint | JSON | Verified | – | – |
| 63 | MAXMotion velocity 0/9; param 167 RPM/s; uses kA | JSON; SparkParameters:154; MAXMotionConfig:143 | Verified | – | – |
| 64 | GET_FIRMWARE_VERSION 9/8 RTR; closed-loop sensor 9 must be 1 | JSON:1303; FeedbackSensor.java | Verified | `kPrimaryEncoder(1)` | – |
| **BURN §6** |||||
| 65 | Our persist frame matches the spec exactly | ino:230-233 | Verified | – | – |
| 66 | Candidate 1: enabled; "implies… earlier message was 'Unknown error status Persist Parameters'" | issue 69 | Verified (understated) | issue title plus the 2024-12-18 REV explanation | Cite the 2024-12-18 comment as direct evidence rather than an implication |
| 67 | Candidate 2: EEPROM fault — REV engineer found persist errors with green-orange blink and EEPROM faults (2024-12-17) | issue 69 | Partly supported | quote exists at :193, **but** on 2024-12-17 22:34 jfabellera calls it "a separate issue… may be exclusive to Flex" (#77), and the team reported the LEDs were **not** green/orange | Demote: the EEPROM lead is a Flex-specific side issue that REV withdrew. It is weak for a MAX, but still checkable via STATUS_1. |
| 68 | Candidate 3: sparklib measured 0xFF for a bad magic/length on the FW 24 burn | sparklib REFERENCE.md:270-272 | Verified | "0x00 accepted, 0xFF refused"; refused = "wrong value, big-endian, a single byte" | Minor: that is the pre-25 api 0x072 frame, a different frame |
| 69 | Candidate 4: persisting too soon after writes; "community-only" | none cited | Partly supported | no reference in references/ states this | Add a citation or drop it |
| 70 | Root CLAUDE.md: 2026-05-18 gains burned via REV Hardware Client | root CLAUDE.md | Verified | – | – |
| **Disagreements §7** |||||
| 71 | Spec md v0.1.2 and legacy docs say "F 0 / kF_0" | md:83; legacy page | Verified | – | – |
| 72 | Status-period "μs" in the md is a typo | md:225-232; JSON; SignalsConfig:92 | Verified | – | – |
| 73 | UVW defaults 0.03125 / depth 3 = 32 ms / 8 samples | md:203-204 | Verified | – | – |
| 74 | Header labels unreliable (2026.0.5 says dev.11 but has STATUS_8/9 and BOOTLOADER_0; 2027 says 2.1.0 with extra position-setpoint fields) | headers | Verified | 2026.0.5:29; 34 STATUS_8/9/BOOTLOADER_0 hits; 2027 adds POSITION_SETPOINT_RESPONSIVENESS/TYPE | – |
| 75 | Firmware CLAUDE.md `PTYPE_FLOAT = 2` vs spec float = 3 | snapshot CLAUDE.md:92; JSON:5040 | Verified | – | – |
| 76 | sparkcan: FW 24 only; velocity class 1 index 2; burn class 63 index 2 | sparkcan README:7; hpp:41,46 | Verified | `(63<<4)\|2`, `(1<<4)\|2` | – |
| 77 | Firmware comment "required on FW 25+" vs notes "Supports" | ino:184; notes:31 | Verified | – | – |
| 78 | Firmware CLAUDE.md says "STATUS_2 on by default"; spec wins | snapshot CLAUDE.md | Partly supported | CLAUDE.md:95 says on by default, **but** CLAUDE.md:127 already says "disabled by default, must be enabled explicitly" | Say the CLAUDE.md contradicts itself; line 95 is stale |
| **Bench tests §8** |||||
| 79 | T1: send `KP0 KI0 KD0 KZ0 KF0.000197`; the V/RPM reading predicts duty ≈ 0.016 at 1000 RPM, below the 0.03–0.06 stiction; then `KF0.00236` | ino:414-430; FINDINGS:19-20 | Partly supported | 1000 × 1.97e-4 / 12 = 0.0164 is correct; stiction 0.030 / 0.060 is correct | (a) The parser handles **one K per line**: on one line only KP=0 is applied (`atof("0 KI0…") = 0`), so kI 2.5e-7 stays live. Send 5 separate lines. (b) Make step 2 explicitly **conditional** on step 1 giving ~0 RPM (see Safety). |
| 80 | T2: STATUS_1 = 0x205B840|dev; EEPROM bits 6/18/19/30/42/43 | JSON:207-411 | Verified | ESC_EEPROM_FAULT 6, ESC_EEPROM_WARNING 18, EXT_EEPROM_WARNING 19, sticky fault 30, sticky warnings 42/43 | – |
| 81 | T3: RTR DLC 8 to 0x2053E00|dev (16/17) and 0x2054A00|dev (112/113); GET_FIRMWARE_VERSION 0x2052600 RTR; build big-endian | JSON:5386, :1303 | Verified | READ_PARAMETER_16_AND_17 15/8, _112_AND_113 18/8; BUILD `isBigEndian: true` | – |
| 82 | T4: capture 0x2053840|dev, expect RESULT 0, TYPE 3, VALUE 0.0007; T5: capture 0x2050440|dev | JSON:5032, :1040 | Verified | – | – |
| 83 | T6: write 136 = 0.016 float and 137 = 1 uint32; lag ~112 → ~8 ms | EncoderConfig | Verified (arithmetic) | 16 ms is within [8, 64]; index 1 = 2 samples; (2−1)/2 × 16 = 8 ms | The current firmware cannot send this (K maps only P/I/D/F/Z, and `setParam` is float-only), so a temporary sketch or adapter is needed. The formula is unconfirmed on FW 26, as stated. |

## References check

`file` output is summarised. Content was compared with the report's description. Commit hashes and repository URLs cited in REPORT §2 are **not embedded** in most files, so provenance could not be verified offline. That applies to both JSON specs, the params md, the specs README, the DBC, all REVLib headers and Java sources, sparkcan, sparklib and the rst. Their content is consistent with the claimed identity.

| File | `file` type | Report says | Check |
|---|---|---|---|
| rev_2026_spark_frames_2.1.0.json | JSON | REV-Specs 2.1.0, A | OK: `framesVersion "2.1.0"`, has STATUS_8/9 and BOOTLOADER_0 |
| rev_2025_spark_frames_2.0.0-dev.11.json | JSON | previous spec, A | OK: `framesVersion "2.0.0-dev.11"` |
| rev_2025_spark_parameters_v0.1.2.md | UTF-8 text | REV-Specs params @1e90305, A | Official but **stale for FW 26**: no kS/kA (204/205), 16 = "F 0". The file carries no version string, and the "rev_2025" prefix vs "@1e90305 (2026)" is inconsistent. Used correctly as the "old naming" side. |
| rev_2026_rev_specs_README.md | ASCII | README, A | OK |
| rev_2026_revlib_driver_2026.0.5_*.h (3) | C source | REVLib-driver 2026.0.5, A | OK: `kMinFirmwareVersion = 0x1a000000 // v26.0.0` |
| rev_2026_revlib_driver_2027.0.0-alpha-7_*.h (2) | C source | 2027 alpha-7, A | OK: header says spec 2.1.0; 112/113/136/137 absent. Note: **alpha** software, not a release |
| rev_2026_revlib_cpp_2026.0.5_REVLibError.h | C++ | REVLib-cpp 2026.0.5, A | OK |
| rev_2026_revlib_java_2026.0.5_*.java (9) | Java/ASCII | REVLib-java 2026.0.5, A | OK: REV copyright headers |
| rev_2026_node_revlog_converter_spark.public.dbc | ASCII | REV public DBC, A | OK: content matches the JSON |
| rev_2025_revlib_example_closed_loop_Robot.java | Java | REV example, A | OK: pinned commit 814041d in the header |
| rev_2026_revlib_example_closed_loop_Robot.java | Java | REV example, A | Fetched from **`main`** (unpinned); content is 2026-API |
| rev_2026_firmware_revlib_release_notes.md | "SGML" (HTML comment + md) | REV release notes, B | OK: verbatim release bodies with tags and URLs |
| rev_2026_closed_loop_units.md, rev_2026_feedforward_control.md, rev_2026_sparkmax_parameters_legacy_docs_page.md | "HTML document" (md with an HTML comment) | REV docs, B | OK: official docs.revrobotics.com exports, not error pages. The "legacy" page is the current SPARK MAX parameters page using legacy (kF_0) naming. |
| rev_2022_hall_velocity_latency_via_wpilib_issue258.md | SGML/ASCII | REV Support quoted in sysid#258, B | Community user (Piphi5) quoting REV Support, **2022, pre-FW25**. The `rev_` filename prefix overstates it. Level B is fair, but not current-firmware evidence. |
| wpilib_2024_2025beta_issue69_persist_error.md | UTF-8 text | issue 69, B | OK. It contains the decisive 2024-12-18 REV statement ("failed because the robot is enabled") that REPORT under-uses |
| wpilib_2026_frc_can_device_spec_can_addressing.rst | UTF-8 text | frc-docs, B | OK: lines 188–249 match |
| grayson-arendt_2026_sparkcan_{README,hpp,cpp} | text / C / C++ | community, C, FW 24 only | OK: README:7 "only work with firmware 24.0.X" |
| l5vel_2026_sparklib_{PROTOCOL,SPARK-MAX-REFERENCE}.md | ASCII | community, C | OK: MAX measured only on 24.0.1, Flex on 26.1.6 |

None of the reference files are error pages, and no community source is presented as official. The only mislabel risk is the `rev_2022_…` filename (community-relayed, old firmware).

## Safety of bench tests

- **T1, step 2 (`KF0.00236`) can command full output.** If step 1 shows ~1000 RPM (the duty/RPM hypothesis), then KF 0.00236 × 1000 = 2.36 duty, which saturates at 100 % duty. With KP = 0 there is no correction, so the tracks run to about 5 000–5 500 RPM. `MAX_RPM` clamps only the setpoint, not the output. Run step 2 **only if** step 1 gave ~0 RPM, and keep the tracks off the ground with the e-stop in reach.
- **T1 command format:** one K command per serial line. As written, only KP is zeroed and kI (2.5e-7, iZone 600) stays active, which can confound the result.
- **T2(c) writes flash.** PERSIST saves **whatever is in RAM**, including T1/T4/T6 values such as KP = 0, KF = 0.00236 or 136 = 0.016. Before T2(c), power-cycle or re-push the production yaml gains, and confirm them with T3/T4. Otherwise the test can silently burn test gains. T2(b) also relies on the heartbeat lock to ignore the 0xFF secondary heartbeat. Either confirm STATUS_0 bit 53 is set or stop sending the secondary heartbeat too, then check applied output is 0 before persisting.
- **T6** changes the velocity filter in RAM only. It needs a temporary sketch and must not be followed by a BURN unless intended.
- **T3, T5 and T7** are read-only (RTR reads, the status-enable response). **T4** re-writes the same kP (RAM). None of these command motion. None of the listed frame IDs collides with PERSIST (0x205FFC0) or the reset frames (0x2050540 / 0x20505C0).

## Corrections applied (2026-09-27)
All 10 partly supported items were applied to REPORT.md:
- ~69 % inference: assumptions stated (left-wheel plant, kI 0, 12 V). Sources corrected: ~70 % is from the root CLAUDE.md; FINDINGS.md:39 records 82 %.
- BURN 255: the REV engineer's direct statement (issue 69, 2024-12-18) added. EEPROM fault demoted to least likely. "Persisting too soon" dropped (unsourced).
- Hall-filter 112 ms: marked as a 2022, pre-FW25 statement, not confirmed for 26.1.4. The ~185 ms is labelled as our own measurement.
- BURN wait: up to ~1.25 s with early exit; disabling happens only if the gap exceeds 100 ms.
- Firmware CLAUDE.md STATUS_2 contradiction noted (line 95 stale; line 127 correct).
- T1 now sends K commands one per line. T1 step 2 is gated on step 1 giving ~0 RPM, with a saturation warning.
- T2: re-push and read back gains before any persist; check the heartbeat lock and zero output.
- T6: needs a temporary sketch; RAM only, no BURN.
- A safety summary for all bench tests was added.
