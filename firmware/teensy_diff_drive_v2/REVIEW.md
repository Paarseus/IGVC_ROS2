# Independent review: teensy_diff_drive_v2 (2026-09-28)

Reviewed: `teensy_diff_drive_v2.ino` and `PROTOCOL.md` in this folder, against v1 (`research/evidence/firmware_can_review_2026_09_27/firmware_snapshot/teensy_diff_drive.ino`), the references in `research/evidence/firmware_can_review_2026_09_27/references/`, the installed FlexCAN_T4 library and Teensy 4 core (Teensyduino 1.59.0), and the host `src/avros_control/avros_control/actuator_node.py`.

Abbreviations are the ones PROTOCOL.md uses: JSON = `rev_2026_spark_frames_2.1.0.json`, HDR = `rev_2026_revlib_driver_2026.0.5_CANSparkFrames.h`, PARAM = `rev_2026_revlib_java_2026.0.5_SparkParameters.java`, PHDR = `rev_2026_revlib_driver_2026.0.5_CANSparkParameters.h`, RST = `wpilib_2026_frc_can_device_spec_can_addressing.rst`, DBC = `rev_2026_node_revlog_converter_spark.public.dbc`. "ino:N" means a line of the reviewed `.ino` **after** the review fixes.

## 1. Summary

- **Every CAN frame is correct.** I recomputed each arbitration ID from `2<<24 | 5<<16 | class<<10 | index<<6 | dev`. The ID, DLC, byte layout, scaling, signedness, endianness and RTR use all match JSON and HDR. A script confirmed that JSON and HDR agree on all 328 frame IDs and lengths, and that all 128 READ_PARAMETER frames follow `class = 15 + (id/2)/16`, `index = (id/2)%16`.
- **The parameter table is correct.** All 31 ids and types match PARAM and PHDR. The 32-bit encoding per type is correct.
- **The host keeps working.** Every line that `actuator_node` sends or parses behaves byte-for-byte as in v1. That covers the E line, `OK K<x>=`, `OK S\r\n`, `OK L=`, `OK A<n>\r\n`, `OK BURN result`, the 300 ms watchdog, S, MAX_RPM 4600 and the M slew default of 100.
- **Timing is sound.** Nothing after `setup()` blocks the 20 ms heartbeat:
  - there is no `delay()`;
  - all CAN request/response exchanges are state machines;
  - serial input is bounded per loop pass;
  - output never waits (I checked `out()` against `usb_serial_write`/`usb_serial_write_buffer_free` in the core).
  - The CAN RX callback runs from `can.events()` in loop context, so it cannot race the main loop.
- **No blockers found.** I made four minimal safety fixes, all small (section 4):
  - NaN setpoints got past every clamp;
  - `MDnan` disabled the duty and voltage cap;
  - a NaN or inf K gain could be written to the SPARK;
  - a stale velocity ramp was sent as a step after a duty or voltage run.
- **Result:** after these fixes the firmware compiles with `--warnings all` and no warnings (FLASH code 62784 B, RAM1 22592 B variables).
- **Verdict:** safe to flash for a bench test **with the tracks off the ground**, following the checklist in section 5. Three should-fix items remain (section 3). None of them affects the host's normal L/R/S/A/K traffic.

## 2. Item-by-item check

| Item | Spec reference | Firmware line | Correct? | Note |
|---|---|---|---|---|
| Arbitration ID packing | RST "Addressing"; JSON:4 (type 2, mfg 5) | ino:395-401 | Yes | Masks cls to 6 bits, idx to 4, dev to 6 |
| Universal heartbeat, enabled `78 01 00 12 59 04 00 60` | RST:188-249 (struct: enabled = bit 25, systemWatchdog = bit 28) | ino:458 | Yes | Byte 3 = 0x02 \| 0x10 = 0x12. Same bytes as v1. |
| Universal heartbeat, disabled (byte 3 = 0x00) | RST:245 ("if System watchdog is set, motor controllers are enabled") | ino:459 | Yes | Clears Enabled, SystemWatchdog and redAlliance. Byte 2 bit 7 (ftcMotorOverride, bit 23) stays 0. Clearing both bits is the conservative choice. |
| SECONDARY_HEARTBEAT 11/2, dev 0 = 0x2052C80, DLC 8, all 0xFF | JSON:1525; HDR:79/409 | ino:464-467 | Yes | Only sent while enabled, which is right: an unlocked SPARK must not be re-enabled by it during BURN |
| VELOCITY_SETPOINT 0/0 = 0x2050000, DLC 8 | JSON:716 (float LE bits 0-31, arbFF int16 32-47, slot 48-49, units bit 50) | ino:470-485 | Yes | Bytes 4-7 = 0, so slot 0 and no arbitrary FF |
| DUTY_CYCLE_SETPOINT 0/2 = 0x2050080, DLC 8 | JSON:764 | ino:470, 499 | Yes | |
| VOLTAGE_SETPOINT 0/5 = 0x2050140, DLC 8, float V | JSON:851 (versionImplemented 25.0.0); HDR:51/381 | ino:470, 500 | Yes | Cap semantics: see finding M1 |
| SET_STATUSES_ENABLED 1/0 = 0x2050400, DLC 4 | JSON:1025 (MASK u16 bits 0-15, ENABLED u16 bits 16-31); HDR:55/385 | ino:501-506 | Yes | `07 00 07 00`. Fixes v1's DLC 8. |
| SSE bit n = STATUS_n | Not stated in the spec; DRV:96-103 enum; v1's 0x0004 enabled STATUS_2 in practice | ino:151 | Reasonable | Judgement call 5 |
| SET_STATUSES_ENABLED_RESPONSE 1/1 = 0x2050440, DLC 5 | JSON:1040 (RESULT u8 bits 0-7, MASK u16 bits 8-23, BITFIELD u16 bits 24-39) | ino:777-784 | Yes | buf[0]; buf[1..2] LE; buf[3..4] LE |
| PERSIST_PARAMETERS 63/15 = 0x205FFC0, DLC 2, `A3 3A` | JSON:14882, magic 15011 at :14898; HDR:375/705 | ino:508-511 | Yes | 15011 = 0x3AA3, sent LE |
| PERSIST_PARAMETERS_RESPONSE 1/4 = 0x2050500, DLC 1 | JSON:1089 ("0 on success") | ino:785-788 | Yes | |
| CLEAR_FAULTS 6/14 = 0x2051B80, DLC 0 | JSON:1215; HDR:63/393 | ino:513 | Yes | |
| IDENTIFY 7/7 = 0x2051DC0, DLC 0 | JSON:1228; HDR:65/395 | ino:514 | Yes | |
| GET_FIRMWARE_VERSION 9/8 = 0x2052600, RTR, DLC 8 | JSON:1303 (rtr true, len 8); HDR:70/400 | ino:557-559, 812-825 | Yes | Reply: MAJOR buf0, MINOR buf1, BUILD = buf2<<8 \| buf3. That is BE, per JSON:1312 and DBC `BUILD : 23\|16@0+` (Motorola, MSB in byte 2). DEBUG buf4, HW buf5. |
| FV DLC-0 fallback | Community (l5vel PROTOCOL:76-77) | ino:575-581 | Reasonable | Actually retried **twice**, not "once" as PROTOCOL §5.2 says. A late DLC-8 reply is labelled `dlc_req=0`. Minor. |
| PARAMETER_WRITE 14/0 = 0x2053800, DLC 5 | JSON:5006 (ID u8 bits 0-7, VALUE u32 bits 8-39, typed per parameter at :5018) | ino:547-552 | Yes | No type byte, which is correct for FW 25+ |
| PARAMETER_WRITE_RESPONSE 14/1 = 0x2053840, DLC 7 | JSON:5032 (ID bits 0-7, TYPE bits 8-15 with 1 int / 2 uint / 3 float / 4 bool at :5044, VALUE bits 16-47, RESULT bits 48-55 with codes 0-5 at :5068) | ino:795-810 | Yes | Decoded with the type the SPARK reports. Matched on device (by arbitration ID) and parameter id. |
| READ_PARAMETER_2k_2k+1, 15..22/0..15, RTR, DLC 8 | JSON:5082.. (FIRST u32 bits 0-31, SECOND bits 32-63); HDR:102-229 / 432-559 | ino:539-542, 554-556, 827-851 | Yes | All 128 checked by script. Whether a MAX answers is unknown (JSON:5388). |
| RTR TX in FlexCAN_T4 | i.MX RT FlexCAN: after a remote frame is sent, the MB becomes RX_EMPTY | FlexCAN_T4.tpp:432, 1221-1260 | OK | The library's TX interrupt returns the MB to TX_INACTIVE, so no TX mailbox leaks. With MRP=0 the reply lands in the FIFO. |
| STATUS_0 46/0 = 0x205B800 | JSON:81: APPLIED int16 ×3.0824e-5; V u12 bits 16-27 ×0.007326; I u12 bits 28-39 ×0.03663; temp u8 bits 40-47; flags 48-53; MODEL u4 bits 54-57 | ino:754-764 | Yes | Signed cast on applied output is correct. s0_flags b4 = INVERTED (52), b5 = HB_LOCK (53). |
| STATUS_1 46/1 = 0x205B840 | JSON:207 (faults 0-7, warnings 16-23, sticky faults 24-31, sticky warnings 40-47, IS_FOLLOWER 48) | ino:675-706, 765-771 | Yes | Mask 0x0001FF00FFFF00FF matches. Name order matches JSON bit order. |
| STATUS_2 46/2 = 0x205B880 | JSON:419 (float vel bits 0-31, float pos 32-63, off by default at :451) | ino:747-753 | Yes | |
| RX hardware filter | FlexCAN_T4 `setFIFOUserFilter` (Table A, RTR bit compared through mask bit 31) | ino:1263-1266 | Yes | Only extended data frames with bits 28..16 = 0x0205 are accepted. Self-reception is disabled by the library (tpp:164). |
| RX software filter | – | ino:734-743 | Yes | Rejects remote frames and any device other than 1 or 2 |
| RX timestamp | FlexCAN timer = bit clock (i.MX RT RM) | ino:725-732 | Reasonable | Judgement call 7. A frame queued longer than 65.5 ms would alias, but that can't happen in normal loop timing. |
| Parameter table: 31 ids and types | PARAM:42-192; PHDR:52-202 | ino:225-257 | Yes | All ids and types match. Note: 61 is `kSmartCurrentConfig`; REVLib writes the RPM limit there (SparkBaseConfig.java:280), so the name "smartLimitRpm" fits. |
| 32-bit encoding | JSON:5018/5044 | ino:909-923 | Yes | float: IEEE bits. uint: strtoul. int: two's complement. bool: 0/1. All LE through putU32. |
| Type codes | JSON:5044 (1 int, 2 uint, 3 float, 4 bool) | ino:219 | Yes | Not REVLib's Java `Type` ordinal. That ordinal is never sent, which is correct. |
| E line | v1 `E L%.0f %.4f R%.0f %.4f\n`; host E_RE | ino:1346-1352 | Yes | Identical, including the ≥64 `availableForWrite` guard |
| `OK K<x>=%.8f\n` (both sides) | v1; host OK_RE `K[PIDF Z]` | ino:970 | Yes | Per-side `OK KPL=` also matches OK_RE. `OK KS=` and `OK KA=` don't, and are ignored. |
| `OK S\r\n`, `OK L=%.0f R=%.0f\n`, `OK UL=`, `OK A<n>\r\n`, `OK M=`, `ERR ...` | v1 | ino:1068-1078, 1105, 1122, 1147, 1220 | Yes | Byte-identical |
| `OK BURN result L=%d R=%d\n` | v1 | ino:670 | Yes | Now asynchronous. The host never sends BURN. |
| 300 ms watchdog | v1 | ino:1295-1304 | Yes | Zeroes velocity (and its ramp), duty and voltage. Fed only by L/R, S, U and UV. A, K, PW and D don't feed it. |
| S behaviour | v1: velocity mode, 0 RPM, ramp bypassed | ino:1068-1078 | Yes | Also zeroes the voltage command. (The root CLAUDE.md claim "S = duty 0" is still wrong, as it was in v1.) |
| MAX_RPM 4600, M default 100/tick | v1 | ino:97, 365 | Yes | |
| DIAG prefix | v1 | ino:1020-1044 | Yes | v1 fields first. Worst case is about 420 chars, under the 512-byte `out()` buffer. |

## 3. Safety findings

### Blockers
None.

### Should-fix (before this firmware is used beyond the bench)

- **S1. `PW` writes any id, including dangerous ones.**
  - The dangerous ids include 0 `kCANID`, 1 `kInputMode`, 2 `kMotorType`, 10 `kPolePairs`, 57/58 legacy follower and 194/195 follower (PARAM:37-46, 88-89, 181-182).
  - A typo such as `PW L 0 u 5` re-addresses a SPARK. The universal heartbeat still enables it, and it keeps running its last setpoint. After that, neither the Teensy watchdog nor `S` can reach it; only the power switch stops it.
  - Writing `kMotorType` = brushed to a NEO can damage hardware.
  - Fix: add a denylist that requires an explicit force token.
- **S2. BURN sends the persist "anyway" after 300 ms if applied output is not ~0 (ino:655-664).**
  - A nonzero applied output after the disabled heartbeat means the SPARK is still enabled, or its STATUS_0 is stale.
  - Persisting while enabled is exactly what the REV engineer says is unsafe: the SPARK "will basically shut down entirely while it is burning its flash" (issue69:247-249).
  - Fix: abort. Restore the heartbeat, print `ERR BURN aborted: output not zero`, and do not send PERSIST.
- **S3. Commanded state survives BURN.**
  - A `UL`/`UVL` sent during BURN (for example by a streaming host) is applied at full value the instant the heartbeat is restored. Velocity is ramped from 0, but duty and voltage are not.
  - Fix: zero `cmd_duty`/`cmd_volt`/`cmd_rpm` in `burnStart()`. Keep the host (actuator_node) stopped during BURN; the checklist below does.

### Minor

- **M1. Voltage cap and bus sag.** The voltage cap is 12 V × duty cap. On a sagging rail (8.5-12 V has been observed) the effective duty exceeds the duty cap: 3.6 V at 8.5 V is 0.42 duty, and 7.2 V at the 0.6 ceiling is 0.85 duty. Consider `duty_cap × min(bus V of both SPARKs, 12)`. Voltage mode is bench-only, so this is minor (judgement call 9).
- **M2. Overlong serial lines run truncated.** A line longer than 127 chars is cut, then executed (ino:1226-1237). v1 had the same behaviour at 95 chars. Better to discard the line and print `ERR`.
- **M3. Numeric parsing quirks in `PW` and `PR`.**
  - `parseParamId` uses `strtol(...,0)`, so `010` is read as octal 8.
  - `encodeValue` does not detect uint/int overflow, because `strtoul` saturates without an error.
  - `PW ... f nan` is accepted. I left this on purpose: PW is an expert tool.
- **M4. `M` edge values.** `M0` freezes the velocity ramp: L0 then no longer slows the wheel, and only S or the watchdog stops it. `Mnan` disables the slew. Both behave as in v1. Clamp M to a small positive minimum.
- **M5. Unknown lines can act as velocity commands.** Any unrecognised line containing `L` or `R` is parsed as a velocity command and feeds the watchdog (for example `HELLO` sets the left wheel to 0 RPM). v1 behaves the same.
- **M6. Callback can run in ISR context before the first `can.events()`.** Until the first call, FlexCAN_T4 has `isEventsUsed = 0`, so a frame arriving between `enableFIFOInterrupt()` in `setup()` and the first `loop()` runs `onCanRx`, and possibly `out()`, from the ISR. The window is under 1 ms. Fix: call `can.events()` once before `enableFIFOInterrupt()`.
- **M7. FlexCAN_T4 TX-queue bug.** `events()` (tpp:1089-1096) peeks one queued frame and writes it into **every** idle mailbox, popping once per mailbox. That sends the head frame several times and drops the others. It only matters when the software TX queue is in use (no ACK or bus-off; watch `txq=` in DIAG). v1 had the same exposure.
- **M8. Response-matching edge cases.**
  - A late reply to a retried write can confirm a later queued write of the **same** id. It is then reported as a readback mismatch, which is harmless.
  - A late PERSIST response from a previous timed-out BURN, if it arrives during the next BURN's disable phase, counts as that BURN's result.
- **M9. SET_STATUSES_ENABLED keeps going out during BURN.** It is sent every 1 s even during PERSISTING. This is probably harmless, but suppress it during BURN for a cleaner flash write.
- **M10. NaN velocity passes the BURN check.** If a SPARK ever reported NaN velocity, `fabsf(NaN) > 30` is false, so the BURN "moving" check passes.
- **M11. Carried over from REPORT §1.1: kV units.** Host `KF 0.000197` is written as kV. On FW 26 that is volts per RPM, probably about 12× too small. This is out of scope for the firmware; run bench test T1.

### Judgement calls (PROTOCOL.md §5)

| # | Call | Verdict |
|---|---|---|
| 1 | Parameter READ as RTR, DLC 8; silence is reported as TIMEOUT | Reasonable. This is the spec form, and it is read-only and harmless if the MAX ignores it. |
| 2 | FV: DLC 8 first, then DLC 0 | Reasonable. Correct the doc: DLC 0 is tried twice. |
| 3 | Disabled heartbeat clears both Enabled and SystemWatchdog | Reasonable and conservative. RST:245 names SystemWatchdog; clearing both covers either reading. |
| 4 | Persist result codes other than 0 printed raw | Correct; the spec defines only 0 |
| 5 | SSE bit n = STATUS_n | Reasonable. REVLib's enum order supports it, and v1's 0x0004 produced STATUS_2 in practice. Setting bits 0/1 re-asserts defaults that are already on, which is harmless. |
| 6 | UNSUPPORTED only for replies shorter than 8 bytes | Reasonable |
| 7 | FlexCAN timer ticks once per bit | Reasonable (i.MX RT FlexCAN design). The >20 ms guard limits damage if it is wrong. It only affects `X` telemetry, never control. |
| 8 | BURN thresholds (30 RPM, 200 ms, 120/300 ms, 1.5 s) | Reasonable. The 120 ms dwell exceeds the 100 ms heartbeat timeout. **But see S2:** the "persist anyway" branch at 300 ms should abort instead. |
| 9 | Voltage cap = 12 V × duty cap | Acceptable for the bench; see M1 |
| 10 | `OK K<x>=` printed on queue, confirmation via `PWR` | Correct for compatibility; the host only logs it |

## 4. Changes made by this review

All are marked `// REVIEW fix` in the `.ino`. After the changes the firmware recompiled cleanly with `--warnings all`, and `build_check/` was deleted.

1. **ino:479 `setVelocity`:** NaN → 0 RPM. NaN fails every `>`/`<` compare, so `Lnan` (which Python produces from a NaN float with `f'{x:.0f}'`) reached the SPARK as a NaN setpoint. It also poisoned `ramp_rpm` until S or the watchdog cleared it.
2. **ino:485 `clampDuty` / ino:493 `clampVolt`:** NaN → 0, for the same reason.
3. **ino:1211 velocity parse clamp:** NaN → 0, so it never enters `cmd_rpm`/`ramp_rpm`.
4. **ino:1134 `MD`:** `if (!(d >= 0))` replaces `if (d < 0)`. Before, `MDnan` set `duty_cap = NaN`, which **disabled both the duty and the voltage clamp**, so `UL1` went through at full duty.
5. **ino:965 `cmdK`:** a non-finite value is refused with `ERR K?\r\n`. Before, `KPnan` wrote NaN into the SPARK's PID. The host check `val < 0.0` does not reject NaN, so `ros2 param set /actuator_node kP nan` could reach this path.
6. **ino:1314-1321, duty/voltage branches of the 20 ms tick:** `ramp_rpm` is held at 0 while in duty or voltage mode.
   - Before: `L1000` (ramp reaches 1000), then `UL0` (wheel stops), then `L0` sent a first velocity frame of 900 RPM to a stopped wheel. The ramp resumed from its stale value.
   - Now velocity always ramps up from 0 after a duty or voltage session, the same as after S, the watchdog and BURN.
   - The host never uses U/UV, so its behaviour is unchanged.

Nothing else was changed. PROTOCOL.md was not edited; fix 6 is a behaviour note for §1.1 (`L/R` after a `U`/`UV` session ramp from 0).

## 5. First power-up bench checklist

Preconditions:
- chassis on blocks, **both tracks clear of the ground and of hands and cables**;
- a person at the main power / E-stop;
- the v1 `.hex` on hand for rollback.

The serial port must have exactly one writer. Stop everything that opens `/dev/ttyACM0` first:

```bash
pkill -f "[a]ctuator_node"; pkill -f "[w]ebui_node"; fuser /dev/ttyACM0   # expect no output
```

Interactive terminal (commands below are typed into it):

```bash
python3 -m serial.tools.miniterm --eol LF /dev/ttyACM0 115200
```

Because of the 300 ms watchdog, motion needs a stream. Use this helper in a second shell. It exits cleanly and the watchdog stops the wheels when it ends:

```bash
# stream.sh "<line>" <seconds>   e.g.  ./stream.sh "L200 R200" 3
python3 - "$1" "$2" <<'EOF'
import serial, sys, time
s = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
line, dur = sys.argv[1], float(sys.argv[2]); t0 = time.time()
while time.time() - t0 < dur:
    s.write((line + '\n').encode()); time.sleep(0.05)
    sys.stdout.write(s.read(4096).decode(errors='replace'))
s.write(b'S\n'); time.sleep(0.1); print(s.read(4096).decode(errors='replace'))
EOF
```

(Close miniterm while the stream runs. Two writers cause false stepping; see memory `feedback_serial_port_collision`.)

**A. Flash with the motor rail OFF (Teensy on USB only).**
1. Flash as in `project_safety_light_wiring_flash` (arduino-cli compile, then `teensy_loader_cli -s`).
2. Expect the banner `# avros diff-drive bridge v2 ready` and the proto line. The safety light should be solid amber.
3. `D`: `hb=1`, `rx=0`. `tx` rising; `txq`/`txfail` may rise because with no SPARK nothing ACKs, which is expected here. `# WDT host-timeout stop` appears once.
4. `PT`: 31 rows.
5. `A1`: `OK A1` and the light flashes. `A0`: `OK A0`. Stop sending A1 and the light returns to solid within 750 ms.

**B. Motor rail ON, no motion.**
1. The SPARK LEDs must **not** blink magenta (that would mean no heartbeat).
2. `D`:
   - `rx` rising;
   - `sse=0:0x0007/0:0x0007`;
   - `hblock=1/1`;
   - `V≈12`;
   - `T` = ambient;
   - `app=0.000/0.000`;
   - `f=0x00/0x00`;
   - `txq` no longer rising;
   - `foreign=0`.
3. `SSE L res=0 ...` and `SSE R res=0 ...` lines appear once. `F L ...` and `F R ...` lines appear once. HAS_RESET sticky warning is normal after power-up; `CF` clears sticky faults.
4. `ID L`, then `ID R`: the correct physical SPARK blinks. **This confirms L = CAN 1 = left track.**
5. `FV`: expect `FV L 26.1.5 ...` and `FV R 26.1.5 ...` (SPARKs updated 2026-09-28; the list originally said 26.1.4). Record `dlc_req`. A TIMEOUT is not a failure. On v2d, also run `CHK` and require `CHK OK` before any motion test.
6. Read test: `PR B kP`, `PR B 136`. Expect `PRD` lines or TIMEOUT. A TIMEOUT answers judgement call 1; it is not a failure.
7. Write test, same values only (RAM):
   - `PW B kP 0.0007` → `PWR L id=13 type=3 val=0.0007 res=0 kP ok` and the same for R, with no `readback` warning;
   - `PW B idleMode 1` → `type=2 ... res=0`;
   - `PW L inverted` with the value the Hardware Client shows → `type=4 res=0`. (Rewrite only the current value.)
   - Optionally, as a type-check probe: `PW L 6 f 1` should return `res=2 mismatched_type`, which confirms that the SPARK enforces types. Then re-send `PW L idleMode 1`.
8. v1 gain path: send `KF0.000197`, `KP0.0007`, `KI2.5e-07`, `KD0.0`, `KZ600` (the actuator_params.yaml values). Expect each `OK K<x>=...` plus two `PWR ... res=0` lines.

**C. Motion (tracks off the ground).**
1. `X1` in miniterm, then close miniterm.
2. `./stream.sh "L200 R200" 3`:
   - both tracks turn forward;
   - E lines read about 200/200 after the ramp (100 RPM per tick, so 2 ticks);
   - `OK S` at the end, and the tracks stop.
3. Watchdog: `./stream.sh "L300 R300" 2`, then press Ctrl-C mid-run (no S is sent). The tracks must stop within about 0.3 s and `# WDT host-timeout stop` must print.
4. Clamp: `./stream.sh "L9000 R9000" 0.3` → `OK L=4600 R=4600`. Stop immediately; don't let it reach speed. The point is to check the ack text.
5. Duty:
   - `./stream.sh "UL0.05 UR0.05" 2` → slow rotation;
   - `./stream.sh "UL0.9 UR0.9" 0.2` → ack `OK UL=0.300 UR=0.300`.
6. Voltage:
   - `./stream.sh "UVL0.6 UVR0.6" 2` → slow rotation, `mode=VOLT` in `D`;
   - `UVL20` → ack `UVL=3.600`.
7. Fix-6 regression: `./stream.sh "L500 R500" 2`, then immediately `./stream.sh "UL0 UR0" 1`, then `./stream.sh "L0 R0" 1`. There must be **no kick** at the switch back to L.
8. NaN guard: `./stream.sh "Lnan Rnan" 1` → `OK L=0 R=0` and no motion. `MDnan` → `OK MD=0.000 VCAP=0.00`, then `MD0.3`.
9. Heartbeat under load: with `X1` on, stream `L300 R300` while a second loop spams `D` at 50 Hz. The SPARK LEDs must never blink magenta, `sdrop` may rise, and E lines must keep arriving.
10. USB pull: while streaming `L200 R200`, unplug USB. The tracks stop within about 0.3 s. The heartbeat continues on Teensy power if it is externally powered.

**D. BURN (optional; do it last).** PERSIST saves **every** RAM parameter.
1. Power-cycle the SPARKs, or re-send exactly the yaml gains from B.8, so no test value is in RAM.
2. `./stream.sh "L300 R300" 1` and, while it is still turning, `BURN` → expect `ERR BURN refused: moving ...`.
3. With the wheels still (`E L0 ... R0 ...`) and no stream running:
   - send `BURN`;
   - expect `# BURN disabling (hb_lock L=1 R=1)`;
   - the SPARK LEDs show disabled for about 1-2 s;
   - then `OK BURN result L=0 R=0`.
   A `# BURN applied output not ~0` line means S2 applied; stop and investigate. **Result 0 would confirm REPORT §6 (the old 255 = "enabled").**
4. Power-cycle the SPARKs and re-check the burned values: run `PW B kP 0.0007` (the reply shows the current value), `PR B kP`, or use the REV Hardware Client.

**E. Host.** Start `ros2 launch avros_bringup actuator.launch.py` (tracks still off the ground). Expect:
- the `<- Teensy ack: OK KF=...`, `OK KP=...` logs;
- `ros2 topic pub /cmd_vel ... {linear: {x: 0.3}}` turns both tracks forward;
- Ctrl-C of the publisher brings the tracks to a stop through the host's S path;
- `/wheel_odom` updating.

## Should-fix items applied after review (2026-09-28)
Recompiled with `--warnings all`: 0 warnings. FLASH code 63,040 B; RAM1 variables 22,592 B.
- **S1 (applied):** `PW` refuses protected parameter IDs unless the line ends in `FORCE`. Protected IDs, from SparkParameters.java: CAN ID 0, input mode 1, motor type 2, commutation 3, pole pairs 10, current chop 11–12, limit switches and hard/soft limits 50–55, 115–116 and 201–203, legacy follower 57–58, 62, motor Kv 63, encoder counts 69/128, compatibility port 127, follower 194–195.
- **S2 (applied):** BURN aborts, and nothing is persisted, if applied output is not about 0 within 300 ms of disabling. It prints `ERR BURN aborted: ...`, restores the enabled heartbeat and zeroes all commands.
- **S3 (applied):** `zeroAllCommands()` (velocity mode, 0 RPM, ramp reset, duty and voltage 0) runs at BURN start and at BURN end. No duty, voltage or velocity command from before or during BURN resumes when the heartbeat returns.
- **Minor, documented, not changed:** the voltage cap assumes a 12 V bus, so on a sagging rail the real duty for a given voltage command is higher than 12 × cap would suggest. Treat `MD` as a voltage cap, not a duty cap.

## Heartbeat byte-order fix (2026-09-28, from the community-library survey)
- The WPILib RST table is big-endian on the wire (match time = byte 8 = `data[7]`), so **SystemWatchdog = `data[4]` bit 4**. The draft's disabled frame cleared only `data[3]`, which left `data[4] = 0x59` with the watchdog bit set. Under the big-endian reading the SPARKs would have stayed enabled during BURN, which is also the likely cause of v1's BURN 255.
- **Fix:** the disabled universal heartbeat is now 8 × `0x00`, and the secondary heartbeat is 8 × `0x00`, not stopped. The enabled frame is unchanged from v1; it enables under both byte orders.
- **Bench proof added:** `HB0`/`HB1`. With the tracks off the ground, stream `UL0.05 UR0.05`, then send `HB0`. The tracks must stop within about 100 ms and `D` must show `app=0.000/0.000`. After 5 s, or on `HB1`, streaming resumes the motion. **Do not run BURN until this passes.**

## Final check of post-review changes (2026-09-28)
This was an independent re-check of the S1/S2/S3 items and the heartbeat/HB0 changes. It fixed clear bugs only and did not redesign anything. Recompiled with `arduino-cli compile --fqbn teensy:avr:teensy41 --warnings all`: **0 warnings**. FLASH code 63,296 B; RAM1 variables 23,616 B.

1. **Heartbeat byte order: PASS.** RobotState bit positions, LSB first: matchTime 0–7, matchNumber 8–17, replay 18–22, ftc 23, red 24, **Enabled 25**, auto 26, test 27, **SystemWatchdog 28**, tournament 29–31, then time of day 32–63.
   - *Big-endian*: wire byte k (1..8) holds bits 64−8k..71−8k. This matches every row of the RST table, e.g. byte 5 = red/enabled/auto/test/watchdog/tournament and byte 8 = match time. So Enabled = `data[4]` 0x02 and SystemWatchdog = `data[4]` 0x10.
   - *Little-endian*: Enabled = `data[3]` 0x02, SystemWatchdog = `data[3]` 0x10.
   - Enabled frame `78 01 00 12 59 04 00 60`:
     - BE: `data[4]=0x59` gives SystemWatchdog=1, **Enabled=0**, test=1, red=1.
     - LE: `data[3]=0x12` gives Enabled=1, SystemWatchdog=1.
     - SystemWatchdog is set under both readings. RST: "If the System watchdog flag is set, motor controllers are enabled."
   - All-zero disabled frame: every bit is clear under both readings.
   - Community code agrees with big-endian:
     - team 195 `receivedData[4] & 0x10`;
     - FRCCan `buf[4]=0x18` (test+watchdog, other bytes 0; this enables only under BE);
     - sikaxn decoder: MSB-first bit string, watchdog at index 35 = `data[4]` 0x10;
     - willGuimont SHIFT 25/28;
     - movemaster and MacRover send `FF×8`.
   - Residual uncertainty:
     - REV does not document which bit(s) SPARK FW 26.1.4 gates on. The v1 frame has Enabled=0 under BE and has driven the motors, which suggests SystemWatchdog (or "either") is what matters.
     - Only the bench HB0 test proves that the all-zero frame actually disables.
2. **Secondary heartbeat: PASS.** `FF×8` when enabled and `00×8` when disabled. This matches REV node-can-bridge `disabledSparkHeartbeat = {0,…,0}` and the crumboe/HC2 pattern ("always send; zeros when disabled"). The field is a 64-bit little-endian bitfield (JSON:1525). Per crumboe, device n = byte n/8 bit n%8, so device 1 = `data[0]` 0x02 and device 2 = `data[0]` 0x04. All-ones covers both.
3. **S1 PW_DENY: PASS.**
   - All 27 IDs match SparkParameters.java: kCANID 0, kInputMode 1, kMotorType 2, kCommutationAdvance 3, kPolePairs 10, kCurrentChop 11/12, limit polarity/enable 50–55, legacy follower 57/58, reserved 62, kMotorKv 63, encoder counts 69, soft limits 115/116, compat port 127, alt-encoder counts 128, follower 194/195, limit-switch position 201–203.
   - No tuning parameter is blocked: nothing in the PARAMS table, K-command IDs 13–17/204/205, or 96/114/56 etc.
   - `FORCE` is accepted only as the last token when n ≥ 4, and it is case-insensitive.
   - A value spelled "FORCE" can never be taken as FORCE with n = 3. It also fails every `encodeValue` type.
   - Lines with more than 6 tokens are truncated, so the last token is not FORCE and the line is rejected with a usage error.
   - Note: `parseParamId` uses `strtol(...,0)`, so `010` = 8 (octal). The denylist check is on the parsed ID, so this is not a bypass.
4. **S2/S3: PASS.**
   - Persist is sent only when dt ≥ 120 ms AND both STATUS_0 frames after `burn_t0` show |applied| < 0.01. Otherwise at 300 ms the BURN aborts. No path persists without the dwell.
   - Both exits (abort; done/timeout) run `zeroAllCommands()` and set `hb_enabled = true`. There are no other exits.
   - `zeroAllCommands()` also runs at burnStart.
   - Limitation (not a bug): during BURN the firmware sends 0 RPM and the wheels are already stopped, so "applied ≈ 0" cannot tell *disabled* from *enabled at zero*. HB0 is the real proof.
5. **HB0/HB1: PASS after two fixes.**
   - Parsing: `case 'H'` requires `B`/`b` and then `0`/`1`, and anything else gives `ERR HB?`. The host never sends H, and the velocity parser is only the `default` branch.
   - Timer: `hb_test_until = now+5000` with 0→1 as the sentinel, and a wrap-safe `(int32_t)(now-until) >= 0` check.
   - BURN interaction: HB during BURN is refused. The auto-restore is skipped while BURN is active.
   - HB0 does not refresh the host watchdog.
   - Scenario "stream UL0.05, HB0, stream stops, auto-restore at 5 s": the 300 ms watchdog zeroes `cmd_duty` and `ctrl_mode` stays DUTY, so duty 0 is sent at restore. **No restart.**
   - **Fix A:** in velocity mode, `ramp_rpm` kept slewing toward `cmd_rpm` during HB0, so HB1 or auto-restore while streaming `L…` stepped the SPARK straight to full `cmd_rpm`. The ramp is now held at 0 while `!hb_enabled`. At most one step (`max_rpm_step`) is sent while disabled, so the test is still meaningful, and the ramp restarts on re-enable.
   - **Fix B:** a BURN started during HB0 left `hb_test_until` armed, which printed a spurious "HB test ended" later. burnStart now clears it. BURN's own exit restores the heartbeat with commands zeroed.
6. **Compile: PASS.** 0 warnings.
7. **Host compatibility: PASS, unchanged.**
   - actuator_node sends `KF/KP/KI/KD/KZ<float>`, `L<n> R<n>`, `S` and `A0/A1`, and parses `E L<int> <f> R<int> <f>`, `OK (K..|A..|S|UL=|BURN|L=)` and `ERR`.
   - The parsers, replies (`OK K%c=%.8f`, `OK S`, `OK L=..`, `OK A%c`) and the E-line format are byte-identical.
   - The new `H` case cannot be reached by host traffic.

**Doc fixes:**
- PROTOCOL.md §3 steps 2–3 still described the old draft (byte 3 cleared, secondary heartbeat stopped, "persist sent anyway with a # warning"). They now describe the all-zero frames and the S2 abort.
- The changelog row and the .ino header/burnStart comments were aligned the same way.

**Go/no-go: GO for a bench test with the tracks off the ground**, in this order:
1. Checklist A–C.
2. The HB0 proof: stream `UL0.05 UR0.05`, then `HB0`. The tracks must stop within about 100 ms and `D` must show `app=0.000/0.000`.
3. Only if step 2 passes, BURN.

If HB0 does not stop the tracks, do not BURN. That result would mean the SPARK gates on something other than the documented bits.

## v2d update for SPARK FW 26.1.5 (2026-09-28)

The changes are listed in PROTOCOL.md under "v2d changes". Review notes:

- **Frames.** Only STATUS_7 and STATUS_8 are new.
  - IDs 0x205B9C0 / 0x205BA00 and DLC 8 are checked against JSON:633/649 and HDR:371-372/701-702.
  - Signal positions come from JSON:641 and :657-659, all little-endian, and agree with HDR `spark_status_7_t` / `spark_status_8_t`.
  - The SSE mask 0x0187 follows REVLib's `1 << statusIndex`.
  - No other frame layout changed.
- **Interlock placement.** The substitution happens inside `sendSetpoint()`, the one function every setpoint path goes through: velocity, duty, voltage, BURN zeroing and the watchdog stop. No mode can bypass it.
  - The substitute is duty 0, which puts the SPARK in its idle mode, the same as the stop-to-idle path.
  - Heartbeats are untouched, so parameter writes still work and the motor type can be repaired with `PW B 2 1 FORCE`.
- **Interlock sources.** READ replies of pair 2/3, and PARAMETER_WRITE_RESPONSE with result 0 for id 2 (its VALUE is the current value, JSON:5032). This includes unsolicited responses, such as RHC2 writing param 2.
  - A failed write (res ≠ 0) is ignored. The follow-up read decides.
- **Remaining risk.**
  - Until the first read after boot, motion is allowed by design (as v1). The boot CHK closes that window about 2 s after boot, and the 5 s re-read closes it within 5 s if the boot CHK timed out.
  - If a SPARK never answers reads, the interlock never engages. DIAG shows `mt=-1`.
- **Judgement calls.**
  - CHK treats kV < 5e-4, param 204 ≠ 0 and an L/R idle mismatch as FAIL reasons. The task said "warn", but they are listed so `CHK OK` means "safe to tune".
  - Feedback sensor, voltage compensation, kA and the hall filter are only printed.
  - `KA` still writes 205, so the host keeps its byte-identical `OK KA=`. The "only via PW" rule is read as "nothing writes 205 automatically".
- **Compile.** `arduino-cli compile --fqbn teensy:avr:teensy41 --warnings all`: 0 warnings, the same as v2c. Flash 67.2 kB code (v2c 63.8 kB).
- **Not flashed, not committed, not run on the robot.** Bench order on 26.1.5:
  1. `CHK` (expect a FAIL on motorType/kV/idle until they are fixed);
  2. `PW B 2 1 FORCE`, then `CHK`;
  3. B10 (`HB0`);
  4. then B1 before any kV change.

## v2d independent review (2026-09-28)

Scope: diff v2c → v2d (478 diff lines), checked against `rev_2026_spark_frames_2.1.0.json`, REVLib-driver 2026.0.5 `CANSparkFrames.h` / `CANSparkParameters.h` / `CANSparkDriver.h`, REVLib-java `SparkParameters.java` / `FeedbackSensor.java`, FW26_CHANGES §8, and the host parsers (`actuator_node.py:607-608`, `drive_tuner.py:108-111`, `bench/tio.py:24-27`). Not flashed, not committed, no robot contact.

1. **STATUS_7/8 + SSE mask: PASS.**
   - STATUS_7: class 46 idx 7 → 0x205B9C0 = JSON arbId 33929664; I_ACCUMULATION float LE bits 0-31 (JSON:641). STATUS_8: 0x205BA00; SETPOINT float bits 0-31, IS_AT_SETPOINT bit 32, SELECTED_PID_SLOT uint bits 33-36 (JSON:657-659), scale 1 / offset 0 everywhere. Code `(v>>32)&1`, `(v>>33)&0xF` on the LE 64-bit word is correct. `msg.len >= 8` guarded.
   - SSE: MASK bits 0-15, ENABLED_BITFIELD bits 16-31 LE (JSON:1033-1034); 0x0187 = bits 0,1,2,7,8 = STATUS_0,1,2,7,8, consistent with `c_Spark_kStatus0..7` = 0..7 and REVLib's `1 << statusIndex`. Bytes `87 01 87 01`.
2. **Motor-type interlock: PASS (no path found that sends a nonzero setpoint to a SPARK known ≠ 1).**
   - Every setpoint frame (class 0, idx 0/2/5) is built in exactly one place, `sendSetpoint()` (grep: the only `CLS_SETPOINT` use). velocity (incl. arbFF kS), duty, voltage, the ramp, the watchdog stop, BURN zeroing and HB0 all go through it; the check is per frame, so it overrides all modes and also forces arbFF = 0.
   - Mid-tick replies: `onCanRx` runs from `can.events()` in `loop()` (FlexCAN_T4 switches to queued dispatch on the first `events()` call), i.e. never concurrently with `sendSetpoint()`. A reply that sets the block takes effect on the next 20 ms frame; at most one frame already queued in a mailbox can precede it. No data race on the new fields (single context; 32-bit stores anyway).
   - Wrong clearing: not possible from another param or device. The block changes only via `cacheStore(w, 2, …)`: (a) a READ reply whose pair matches the in-flight READ job (pair 1 = ids 2/3, id_a = 2), or (b) a PARAMETER_WRITE_RESPONSE with `buf[0] == 2` and result 0 (its VALUE is the current value, JSON:5032). Device is taken from the arbitration ID, so the other SPARK's reply updates only its own Wheel. A late reply to a timed-out read arriving while a WRITE is in flight is ignored (kind mismatch). A float-typed `PW … 2 f 1` echo would read 0x3F800000 ≠ 1 → stays blocked (fail-safe).
   - "Allow until first read": **acceptable for the bench**, matches the documented design; the window is ≤ ~2 s after a Teensy reset (boot CHK) or ≤ 5 s if that times out. Recommendation (not a bug, not changed): fire the first quiet param-2 read on the first loop pass (`t_mt_check = millis() - MT_CHECK_MS` in setup) to shrink the window after an in-motion Teensy reset.
   - 5 s re-read: keeps running (timer advances every period; skipped only during BURN/CHK; `readQueued` prevents duplicates; quiet failure on a full queue). It cannot starve BURN: a healthy read is in flight for ~1 ms, an offline SPARK ~120 ms (2 × 60 ms) per 5 s, so `ERR BURN refused: parameter requests pending` is rare and a retry succeeds. It cannot starve other jobs (one job per 5 s, FIFO).
   - Recommendation (design, not changed): the block is per side, so with one side blocked and the other driving, the chassis would pivot on the ground. Fine for tracks-off bench; before ground use consider blocking both sides when either is blocked.
3. **CHK: PASS.** IDs/types match `SparkParameters.java`: 2 kMotorType u32, 6 kIdleMode u32, 9 kClosedLoopControlSensor u32 (1 = primary encoder), 16 kV_0 f, 74 kVoltageCompensationMode u32 (2 = nominal comp), 75 f, 112/113 conversion f, 136 kUvwSensorSampleRate f, 137 u32, 204 kS_0 f, 205 kA_0 f. 9 quiet jobs per SPARK (FV + 8 pairs) through the existing queue; completion is detected in `loop()` via `chk_outstanding`, every job terminates (write/read retries, FV DLC-8 → DLC-0 fallback), so CHK cannot hang. FAIL reasons and line formats match PROTOCOL.md "v2d changes" §4. Note: kV = 0 also FAILs (`kV<5e-4`) with a "looks like duty/RPM" text; the FAIL is useful (no FF after a FW reset) though the wording is slightly off.
4. **Backward compatibility: PASS, byte-identical.** E line (`E L%.0f %.4f R%.0f %.4f\n`) untouched; `OK K%c=%.8f\n` / `OK K%c%c=`, `OK S\r\n`, `OK L=%.0f R=%.0f\n`, `OK A%c\r\n`, `OK BURN result L= R=` unchanged; host regex `OK (K[PIDF Z]|A[01]|S|UL=|BURN|L=)` still matches. X line: fields 0-23 identical format; `I7 a b SP8 c d e f` appended as fields 24-31; drive_tuner reads `parts[1..23]`, tio requires `len(p) >= 24` and floats only up to 22 — both unaffected (the `nan` tokens are beyond their slices).
5. **kS single source: PASS.** `KS…` is handled before `kLetterToParam`, which no longer maps any letter to 204; 204 is reachable only by explicit `PW` (with a `#` warning when nonzero) and CHK FAILs if 204 ≠ 0. The v2c arbFF path is unchanged. `KA` writes 205 as before and prints `# KA: …` after the unchanged `OK KA=` line; `PW … 205` warns too.
6. **Timing / buffers / races: PASS after fix 1.** No new `delay()`/blocking loops; all new I/O goes through `out()` (drop-on-full) and the job queue; CHK report is ~22 formatted lines in one pass (~1 ms), heartbeat is millis-scheduled so not missed. **Bug found:** the v2d DIAG fields push the DIAG line to ~495 chars typical and ~590 worst case (long uptime counters, blocked-frame counter growing at 50/s), over `out()`'s 512-byte buffer, so the tail (`blkn=`, `chk=`) would be silently truncated.
7. **Compile: PASS**, `--warnings all`, 0 warnings, before and after the fix. FLASH code 67200 B, RAM1 vars 25664 B.

**Fixes made**
1. `out()` line buffer 512 → 768 bytes (`teensy_diff_drive_v2.ino`, `out()`), so the full DIAG line is emitted. Truncation logic unchanged; Teensy 4 `availableForWrite()` reports free 2048-byte USB buffers, so the larger line is never blocked by the guard.

**Go/no-go: GO** for flashing and bench-testing with the tracks off the ground, in the documented order (`CHK` → expect FAIL on motorType/kV/idle → `PW B 2 1 FORCE` → `CHK` → B10 `HB0` → B1). **No-go for ground driving** until the interlock has been bench-proven (read param 2 = 0 → `!!` line, `blk=1`, `app=0.000` while streaming `UL0.05`), CHK passes on both sides, and the two recommendations above (early first read, block-both-sides) have been decided.

### Review recommendations applied (2026-09-28)
- The interlock blocks **both** tracks when either SPARK reports motor type ≠ 1 (`motionBlocked()` checks `left.mt_blocked || right.mt_blocked`), so one track can never drive alone.
- The first motor-type read happens on the first loop pass (`t_mt_check = millis() - MT_CHECK_MS`), not after 5 s.
- Recompiled: 0 warnings, code 67,264 B.
