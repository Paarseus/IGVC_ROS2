# teensy_diff_drive_v2 — serial protocol, CAN frames, changelog

Teensy 4.1 USB-serial to CAN bridge for 2 × REV SPARK MAX on firmware **26.1.5** (CAN ID 1 = left, 2 = right, NEO hall sensor, 1 Mbit/s, no roboRIO). Replaces `firmware/teensy_diff_drive/` (v1, deployed commit f242825). Current revision: **v2d** (see "v2d changes" at the end).

**Status: FINAL, flashed and ground-verified 2026-10-08.** v2/v2c were flashed and bench-tested on FW 25.0.4 (superseded). v2d had been compiled and desk-tested only until tonight; deliberately reflashed on the Jetson from this exact source (`firmware/teensy_diff_drive_v2/teensy_diff_drive_v2.ino`, byte-identical to commit `3a3aecc` on the orphaned `drive/fw26-motor-control` branch — never merged to `main`) to remove any doubt about source/flash correspondence. Boot banner read directly off the serial port immediately after the flash: `# avros diff-drive bridge v2d ready (SPARK MAX FW 26.1.5)`. `CHK OK` on both SPARKs post-flash; reference gains (kV 0.00211, kP 0.0004, kS 0.40/0.39) re-pushed and read back via `PR` matching exactly; a same-day alternating-order ground retest (0.05/0.1 m/s) matched the pre-reflash numbers within the same pass bands. Full session: `docs/drive_tuning_2026_09_28/ROOT_CAUSE_ANALYSIS_2026_10_08.md` §7.

```bash
arduino-cli compile --fqbn teensy:avr:teensy41 --warnings all firmware/teensy_diff_drive_v2
```

Reference abbreviations (all files are in `research/evidence/firmware_can_review_2026_09_27/references/`):

| Tag | File |
|---|---|
| JSON | `rev_2026_spark_frames_2.1.0.json` (REV-Specs 2.1.0) |
| HDR | `rev_2026_revlib_driver_2026.0.5_CANSparkFrames.h` (REVLib-driver 2026.0.5, minimum firmware v26.0.0) |
| DRV | `rev_2026_revlib_driver_2026.0.5_CANSparkDriver.h` |
| PARAM | `rev_2026_revlib_java_2026.0.5_SparkParameters.java` |
| RST | `wpilib_2026_frc_can_device_spec_can_addressing.rst` |
| ISSUE69 | `wpilib_2024_2025beta_issue69_persist_error.md` |

JSON and HDR agree on every frame ID and DLC in both files. I checked this with a script over all `*_FRAME_ID` and `*_LENGTH` defines, so no frame needed a tie-break between them.

---

## 1. Serial protocol

The port is USB CDC; the nominal 115200 baud doesn't matter. Lines end in `\n` (a `\r` is also accepted). Commands are case-insensitive. The parser reads at most 256 bytes per loop pass, and every reply goes through a writer that drops the line instead of blocking (see section 4). The drop counter is `sdrop=` in DIAG.

### 1.1 v1 commands (the host `actuator_node` sees no change)

| Command | Effect | Reply (same bytes as v1) |
|---|---|---|
| `L<rpm> R<rpm>`, `L<rpm>`, `R<rpm>` | Velocity mode. Clamped to ±4600 RPM, slewed by `M` per 20 ms tick, sent to the SPARK velocity PID. | `OK L=%.0f R=%.0f\n` |
| `UL<d> UR<d>` (either one alone is fine) | Duty mode, clamped to ±duty cap (default 0.30) | `OK UL=%.3f UR=%.3f\n` |
| `S` | Stop: velocity mode at 0 RPM, ramp bypassed. Also zeroes the duty and voltage commands. | `OK S\r\n` |
| `D` | One DIAG line. The v1 fields come first with v1 formats; v2 fields follow after ` \| `. | `DIAG tx=.. rx=.. wdt=.. mode=VEL\|DUTY\|VOLT L=meas/ramp/cmd R=.. duty L= R= V= M= burn=L/R \| ...` |
| `K<P\|I\|D\|F\|Z><val>` | Writes kP (13), kI (14), kD (15), kV (16) or kIZone (17) to **both** SPARKs as FLOAT (PARAM:48-52). `F` keeps its v1 letter and reply (`OK KF=`), but **param 16 = kV, V/RPM on FW 26 (was kF duty/RPM on FW 25)**. Returns once the writes are queued. The SPARK confirmations arrive later as `PWR` lines. | `OK K<x>=%.8f\n` |
| `M<val>` | Velocity slew, RPM per 20 ms tick (default 100) | `OK M=%.2f\n` |
| `BURN` (any line starting with `B`) | Safe persist (section 3). Finishes asynchronously. Aborts, without persisting, if output is not about 0 after disabling (`ERR BURN aborted`). All commands are zeroed at start and end. | `OK BURN result L=<c> R=<c>\n` (`-1` = no response), or `ERR BURN refused: ...` |
| `A1` / `A0` | IGVC §I.2 safety light: flash / solid. Replies only when the state changes. Does not feed the motor watchdog. | `OK A1\r\n` / `OK A0\r\n` |
| anything else | – | `ERR unknown\r\n` |

The error strings are unchanged: `ERR U?`, `ERR A?`, `ERR M?`, `ERR K?`. The only other v1 difference is the boot banner, which now says `v2`; the host does not parse it.

Host compatibility was checked against `src/avros_control/avros_control/actuator_node.py:608-610`:
- `E_RE = E L(-?\d+) (-?[\d.]+) R(-?\d+) (-?[\d.]+)`: the E line is unchanged.
- `OK_RE = OK (K[PIDF Z]|A[01]|S|UL=|BURN|L=).*`: every v1 ack keeps its text. The new per-side acks `OK KPL=` and friends also match. `OK KS=` and `OK KA=` do not match, so the host ignores them.
- No new line type starts with `E L`, `OK ` in a way that would be misread, or `ERR ` except real errors.

### 1.2 New commands

| Command | Effect | Reply |
|---|---|---|
| `K<P\|I\|D\|F\|V\|Z\|A><L\|R><val>` | Writes one side only. `F` and `V` both mean kV (16, V/RPM on FW 26), `A` = kA (205), all FLOAT (PARAM:51,192). `KA<val>` without a side writes both. `KS` is **not** a parameter write: see v2c (Teensy-side arbFF kS). | `OK K<x><L\|R>=%.8f` / `OK K<x>=%.8f`, followed by `PWR` lines. `KA` also prints `# KA: param 205 only affects MAXMotion on FW 26`. |
| `PW <L\|R\|B> <id\|name> [f\|u\|i\|b] <value> [FORCE]` | Typed PARAMETER_WRITE. Protected IDs (CAN ID, motor or sensor type, follower, limits; see REVIEW.md) need a trailing `FORCE`. The type can be left out when the id is in the table (section 1.4). An explicit type overrides the table and prints a `#` note; use this to provoke a "mismatched type" reply on purpose. `u` and `i` accept `0x` hex. `b` accepts `0/1/true/false`. | `OK PW <side> id= type= val= raw=0x........`, then one `PWR` per side |
| `PR <L\|R\|B> <id\|name>` | Parameter READ: a remote frame with DLC 8 on the pair frame that contains `id` | `OK PR ..`, then `PRD` per side |
| `PT` | Prints the built-in parameter table | `PT <id> <f\|u\|i\|b> <name>` per row |
| `FV` | GET_FIRMWARE_VERSION from both SPARKs | `OK FV`, then `FV L ..` / `FV R ..` |
| `CF [L\|R\|B]` | CLEAR_FAULTS (default both) | `OK CF <side>` |
| `ID [L\|R\|B]` | IDENTIFY (LED blink pattern) | `OK ID <side>` |
| `X1` / `X0` | High-rate telemetry line on / off (off at boot) | `OK X1` / `OK X0` |
| `HB0` / `HB1` | Bench test of the **disabled** heartbeat that BURN uses. HB0 sends all-zero universal and secondary heartbeats for up to 5 s (then restores itself). HB1 restores now. While HB0 is active a streamed `UL0.05` must not move the tracks. | `OK HB0 ...` / `OK HB1` / `# HB test ended ...` |
| `UVL<V> UVR<V>` (either one alone is fine) | Voltage mode (VOLTAGE_SETPOINT). Clamped to ±12 V × duty cap (default 3.6 V). Covered by the 300 ms watchdog. | `OK UVL=%.3f UVR=%.3f` |
| `CHK` | v2d configuration check of both SPARKs (also runs automatically about 2 s after boot). Reads FV and params 2, 6, 9, 16, 74/75, 112/113, 136/137, 204/205 through the job queue (non-blocking, no PRD lines). See "v2d changes". | `OK CHK requests=<n>` (boot: `OK CHK (boot) requests=<n>`), then `CHK <L\|R\|B> ...` lines and `CHK OK` / `CHK FAIL <reasons>` |
| `MD<d>` | Sets the duty cap at run time, limited to [0, 0.6]. The voltage cap is 12 × d, so 7.2 V at most. Commands already active are re-clamped. Resets to 0.30 on reboot. | `OK MD=%.3f VCAP=%.2f` |

### 1.3 Teensy → host lines

| Line | When |
|---|---|
| `E L<rpm> <pos> R<rpm> <pos>` | 50 Hz. Format identical to v1 (`%.0f %.4f`), no fields added. |
| `PWR <L\|R> id=<n> type=<1..4> val=<v> res=<0..5> <name> <ok\|invalid_id\|mismatched_type\|access_mode\|invalid\|not_implemented> raw=0x........ [unsolicited]` | Every PARAMETER_WRITE_RESPONSE from SPARK 1 or 2. `val` is the controller's **current** value, decoded with the type it reports (JSON:5032, :5044, :5068). `unsolicited` means no matching write was in flight, for example REV Hardware Client or a late reply to a retried write. |
| `# PWR <side> id=.. readback 0x.. != sent 0x..` | The reply said success (`res=0`) but the value differs from what was sent |
| `PWR <L\|R> id=<n> TIMEOUT` | No reply after 3 tries × 60 ms |
| `PRD <L\|R> id=<n> type=<c> val=<v> raw=0x.. <name> \| pair id=<m> val=<v>` | Read reply. Both values of the pair are decoded with the table type, or shown as hex if the id is not in the table. |
| `PRD <L\|R> id=<n> UNSUPPORTED dlc=<k>` / `... TIMEOUT` | The reply was shorter than 8 bytes / no reply after 2 × 60 ms |
| `FV <L\|R> <maj>.<min>.<build> debug=<d> hw=<h> dlc_req=<8\|0> [(expected 26.1.5)]` / `FV <L\|R> TIMEOUT` | Firmware version. BUILD is decoded big-endian (JSON:1312). |
| `F <L\|R> f=0x.. w=0x.. sf=0x.. sw=0x.. follower=<0\|1> <names…\|none>` | The first STATUS_1 received, and every later change. Names are prefixed `F:` fault, `W:` warning, `SF:` sticky fault, `SW:` sticky warning. |
| `SSE <L\|R> res=<n> mask=0x.... en=0x....` | SET_STATUSES_ENABLED_RESPONSE, printed only when the result or the enabled bitfield changes. The frame is re-sent every 1 s. |
| `# PERSIST response <side> res=<n> (unsolicited)` | A persist reply that arrives outside `BURN` |
| `X <teensy_us> L <vel_rpm> <pos_rot> <s2_rx_us> <applied> <current_A> <bus_V> <temp_C> R <same 7 fields> SP <sp_L> <sp_R> <sp_tx_us_L> <sp_tx_us_R> <V\|D\|U> I7 <L_iaccum> <R_iaccum> SP8 <L_setpoint> <R_setpoint> <L_atsp> <R_atsp>` | v2d appended fields 24-31 (whitespace-split index; 0-23 unchanged): STATUS_7 I accumulator and STATUS_8 closed-loop setpoint (RPM in velocity mode) / at-setpoint flag. `nan` / `-1` until the first frame. | With `X1`: one line per loop pass in which a new STATUS_2 arrived. `s2_rx_us` is when that STATUS_2 arrived, in Teensy `micros()` (section 4). `SP` gives the last setpoint put on the bus and the `micros()` it was sent, so the host can measure lag from setpoint to response and compute velocity from position differences. The last field is the mode: V velocity, D duty, U voltage. |
| `!! MOTOR TYPE <n> on <L\|R>: motion blocked (NEO needs brushless=1)` / `# MOTOR TYPE 1 on <L\|R>: motion unblocked` | v2d motor-type interlock, printed once per change |
| `DIAG ...` (appended fields) | `volt` = voltage commands; `cap`/`vcap`; `txq` = frames put in the library's software TX queue; `txfail`; `foreign` = frames rejected by the software filter; `sdrop`; `hb` = heartbeat enabled; `burnst`; `hblock` = STATUS_0 bit 53 per side; `inv` = bit 52; `model` = SPARK_MODEL; `app`, `I`, `T` = applied output, current, temperature; `sse=res:bitfield` per side; `f`/`sf` = active/sticky fault bytes; `jq` = requests queued per side; `x`; `rxage` = largest CAN-timestamp correction seen, in µs; v2c `ks`, `arb`; v2d `iacc` (STATUS_7), `sp8`, `atsp`, `slot` (STATUS_8), `mt` = last read motor type (-1 unknown), `blk` = interlock active, `blkn` = setpoints replaced by duty 0, `chk` = CHK running |

### 1.4 Built-in parameter table (`PT`)

Types come from PARAM, and the line numbers below are PARAM lines. Values are **raw wire units**: REVLib converts some of them before writing, as the Notes column says.

| id | name | type | PARAM line | Notes |
|---|---|---|---|---|
| 2 | motorType | u | 39 | 0 brushed, 1 brushless. **Must be 1 for a NEO** (v2d interlock). Protected: `PW` needs `FORCE` |
| 6 | idleMode | u | 42 | 0 coast, 1 brake |
| 9 | feedbackSensor | u | 44 | 1 = primary encoder |
| 13 / 14 / 15 | kP / kI / kD | f | 48-50 | |
| 16 | kV | f | 51 | **V/RPM on FW 26 (was kF duty/RPM on FW 25)**; not rescaled by the update, so 0.000197 is ~12× too weak (≈0.00236 equivalent). Confirm with FW26_CHANGES bench B1 before raising it. |
| 17 | kIZone | f | 52 | |
| 18 | kDFilter | f | 53 | |
| 19 / 20 | outputMin / outputMax | f | 54-55 | |
| 45 | inverted | b | 80 | |
| 56 | openLoopRamp | f | 87 | Raw = 1 / (seconds from 0 to full), 0 = off (SparkBaseConfig.java:361-366) |
| 59 / 60 / 61 | smartStallA / smartFreeA / smartLimitRpm | u | 90-92 | |
| 74 | voltCompMode | u | 99 | 0 off, 2 on (SparkBaseConfig.java:396,406) |
| 75 | nominalVoltage | f | 100 | |
| 96 | kIMaxAccum | f | 101 | |
| 97 | allowedClErr | f | 102 | PID tolerance, new in 26.1.0: inside it the output is FF only. Keep 0 |
| 112 / 113 | posConvFactor / velConvFactor | f | 109-110 | Must stay 1.0 because MAX_RPM and the E line assume RPM and rotations |
| 114 | closedLoopRamp | f | 111 | Raw = 1/s, same as 56 |
| 136 | hallSamplePeriod | f | 127 | Seconds, 0.008-0.064 (EncoderConfig.java:182-185) |
| 137 | hallAvgDepth | u | 128 | Index 0..3 = 1/2/4/8 samples |
| 158 / 159 / 160 | status0Period / status1Period / status2Period | u | 145-147 | ms |
| 165 / 199 | status7Period / status8Period | u | 152 / 186 | ms |
| 186 / 187 / 188 | forceEnStatus0/1/2 | b | 173-175 | |
| 193 / 200 | forceEnStatus7 / forceEnStatus8 | b | 180 / 187 | |
| 204 / 205 | kS / kA | f | 191-192 | kS in volts: **keep 0** (kS comes from the Teensy `KS` arbFF only; CHK fails otherwise). kA in V/(RPM/s): MAXMotion only on FW 26 |

Other ids can be written with `PW` by giving an explicit type.

---

## 2. CAN frames used

Arbitration ID = `2<<24 | 5<<16 | class<<10 | index<<6 | device` (RST "Addressing"; JSON:4 has device type 2 and manufacturer 5). All frames are 29-bit.

| Frame | Class/idx | ID (dev 0) | DLC | Direction | JSON | HDR (ID / len) | Use |
|---|---|---|---|---|---|---|---|
| Universal heartbeat | – | 0x01011840 | 8 | TX every 20 ms | – (RST:188-249) | – | Enabled: `78 01 00 12 59 04 00 60`, same as v1. Disabled (BURN and `HB0` only): **all 8 bytes 0x00**. The RST byte table is big-endian on the wire, so SystemWatchdog is `data[4]` bit 4. The enabled frame's `data[4] = 0x59` sets it, and clearing only `data[3]` would have left the controllers enabled (see COMMUNITY_LIBRARIES_2026_09_28.md). |
| SECONDARY_HEARTBEAT | 11/2 | 0x2052C80 | 8 | TX every 20 ms: `FF×8` enabled, `00×8` disabled (as REV node-can-bridge does) | 1525 | 79 / 409 | `FF×8` |
| VELOCITY_SETPOINT | 0/0 | 0x2050000 | 8 | TX 20 ms | 716 | 48 / 378 | float RPM, arbitrary FF 0, slot 0 |
| DUTY_CYCLE_SETPOINT | 0/2 | 0x2050080 | 8 | TX 20 ms | 764 | 49 / 379 | float duty |
| VOLTAGE_SETPOINT | 0/5 | 0x2050140 | 8 | TX 20 ms | 851 (versionImplemented 25.0.0) | 51 / 381 | float V |
| SET_STATUSES_ENABLED | 1/0 | 0x2050400 | **4** | TX every 1 s | 1025 | 55 / 385 | mask 0x0187, enable 0x0187 (STATUS_0,1,2,7,8; bit n = STATUS_n) |
| SET_STATUSES_ENABLED_RESPONSE | 1/1 | 0x2050440 | 5 | RX | 1040 | 56 / 386 | result, mask, bitfield |
| PERSIST_PARAMETERS_RESPONSE | 1/4 | 0x2050500 | 1 | RX | 1089 | 57 / 387 | result (the spec defines only "0 on success") |
| CLEAR_FAULTS | 6/14 | 0x2051B80 | 0 | TX on `CF` | 1215 | 63 / 393 | |
| IDENTIFY | 7/7 | 0x2051DC0 | 0 | TX on `ID` | 1228 | 65 / 395 | |
| GET_FIRMWARE_VERSION | 9/8 | 0x2052600 | 8, RTR | TX RTR / RX data | 1303 (BUILD big-endian at 1312) | 70 / 400 | |
| PARAMETER_WRITE | 14/0 | 0x2053800 | 5 | TX | 5006 (value type at 5018) | 100 / 430 | id u8 + 32-bit value typed per parameter |
| PARAMETER_WRITE_RESPONSE | 14/1 | 0x2053840 | 7 | RX | 5032 (type at 5044, result at 5068) | 101 / 431 | id, type, current value, result |
| READ_PARAMETER_2k_AND_2k+1 | 15..22 / 0..15 | 0x2053C00..0x2055BC0 | 8, RTR | TX RTR / RX data | 5082.. (e.g. 16/17 at 5386) | 102..229 / 432..559 | pair k = id/2, class 15 + k/16, index k%16 |
| STATUS_0 | 46/0 | 0x205B800 | 8 | RX (10 ms, on by default) | 81 (signals 89-199; default at 204-205) | 364 / 694 | applied output, V, A, °C, limits, inverted, heartbeat lock, model |
| STATUS_1 | 46/1 | 0x205B840 | 8 | RX (250 ms, on by default) | 207 (bits 215-411; default at 416-417) | 365 / 695 | faults, warnings, sticky versions, follower |
| STATUS_2 | 46/2 | 0x205B880 | 8 | RX (20 ms, **off** by default) | 419 (default at 450-451) | 366 / 696 | float velocity, float position |
| STATUS_7 | 46/7 | 0x205B9C0 | 8 | RX (20 ms, **off** by default) | 633 (signal 641; default 647) | 371 / 701 | float I accumulator, bits 0-31 |
| STATUS_8 | 46/8 | 0x205BA00 | 8 | RX (20 ms, **off** by default) | 649 (signals 657-660; default 665) | 372 / 702 | float setpoint bits 0-31, at-setpoint bit 32, PID slot bits 33-36 |
| PERSIST_PARAMETERS | 63/15 | 0x205FFC0 | 2 | TX on `BURN` | 14882 (magic 15011 at 14898) | 375 / 705 | `A3 3A` |

### RX filtering

- **Hardware:** the FlexCAN FIFO filter 0 accepts only extended **data** frames with `(id & 0x1FFF0000) == 0x02050000`, that is device type 2 and manufacturer 5 (`setFIFOFilter(REJECT_ALL)` + `setFIFOUserFilter(0, 0x02050000, 0x1FFF0000, EXT)`).
- **Software:** `onCanRx()` checks again and also rejects remote frames and any device number other than 1 or 2 (counted in `foreign=`). Rejecting remote frames matters because a read request and its reply share one ID, and FlexCAN self-reception is not disabled.
- Setting `USE_HW_RX_FILTER 0` reverts the hardware filter to v1's ACCEPT_ALL; the software check stays on.

---

## 3. BURN sequence

1. **Refusal checks.** BURN is refused (`ERR BURN refused: ...`) if:
   - another BURN is running;
   - any parameter request is queued or in flight;
   - either STATUS_2 is older than 200 ms;
   - either |measured RPM| is above 30.
2. **Disable.** The universal heartbeat changes to all 8 bytes `0x00`, clearing the Enabled and SystemWatchdog bits under either byte order, and keeps going out every 20 ms. The secondary heartbeat keeps going out as all 8 bytes `0x00`. Velocity setpoints of 0 are sent, and host setpoints are ignored for the rest of the sequence. The log line reports STATUS_0 bit 53 (heartbeat lock) per side.
3. **Wait for zero output.** The persist is sent after at least 120 ms, once **both** SPARKs have sent a STATUS_0 since step 2 with |applied output| < 0.01. The 120 ms is longer than the SPARK's 100 ms heartbeat timeout. If that has not happened by 300 ms, BURN **aborts** (`ERR BURN aborted: ...`): nothing is persisted, all commands are zeroed and the enabled heartbeat returns.
4. **Persist.** PERSIST_PARAMETERS (`A3 3A`, DLC 2) goes to both SPARKs. The firmware then waits up to 1.5 s for both PERSIST_PARAMETERS_RESPONSE frames without blocking; the spec says "may take up to a second" (JSON:14884).
5. **Restore.** The enabled heartbeat comes back and the firmware prints `OK BURN result L=<code> R=<code>` (`-1` = no reply). The velocity ramp restarts from 0.

Why the robot must be disabled: a REV engineer says persisting is refused while the robot is enabled, because the SPARK "will basically shut down entirely while it is burning its flash" (ISSUE69:247-249). PERSIST saves **every** RAM parameter, including test values.

---

## 4. Timing and robustness

- **Heartbeat.** The 20 ms tick sends the heartbeats and setpoints. After `setup()` nothing in `loop()` blocks:
  - no `delay()`;
  - the CAN request/response exchanges (writes, reads, FV, BURN) are state machines;
  - serial input is limited to 256 bytes per pass;
  - output is dropped when the USB TX buffers are full. Teensy's CDC write can otherwise wait up to 120 ms (`cores/teensy4/usb_serial.c:281`).
- **CAN receive.** `can.events()` structure is kept from v1. Each loop pass calls it up to 32 times to drain the receive queue, since it dispatches one frame per call.
- **NeoPixel.** Handling is identical to v1: `show()` runs only on mode changes and flash edges, and not within 2 ms of a heartbeat tick.
- **Receive timestamps (`s2_rx_us`).** `micros()` at dispatch, minus the age of the frame. The age is taken from the FlexCAN free-running timer (`FLEXCAN1_TIMER`) minus the frame's hardware timestamp, at one tick per bit time (1 µs at 1 Mbit/s). If the age comes out above 20 ms, dispatch time is used instead. The hardware stamps the frame near its start, so `s2_rx_us` is about 0.1 ms earlier than the end of the frame on the wire. `rxage=` in DIAG shows the largest correction applied.
- **Write handling.** One request is in flight per SPARK and each SPARK has its own 16-entry queue. Writes time out after 60 ms and are tried up to 3 times; REVLib uses 20 ms and 5 retries (ISSUE69:179-183). Reads and FV time out after 60 ms and are tried twice.

---

## 5. Decisions the spec does not settle (unconfirmed)

1. **Do parameter reads work on a SPARK MAX?** (Settled: they worked on 25.0.4 on the bench and are observed working on 26.1.5, FW26_CHANGES §5.) Every read frame's description says "SPARK MAX does not currently support this in v25.0.0-prerelease.4" (e.g. JSON:5388). The firmware sends the spec form: a remote frame, DLC 8. A community driver found that a Flex on 26.1.6 answers only that form (`l5vel_2026_sparklib_PROTOCOL.md:69-72`). A MAX that stays silent shows up as `PRD .. TIMEOUT`. A PARAMETER_WRITE reply already carries the current value, so re-writing a known value also works as a read-back.
2. **Which DLC does GET_FIRMWARE_VERSION need?** JSON and HDR say an RTR frame with DLC 8, and that is sent first. The same community source reports the device answering a DLC-0 remote frame, so after two DLC-8 timeouts the firmware tries DLC 0 once. `dlc_req=` shows which one got the answer.
3. **What does "disabled" mean in the heartbeat?** RST:249 says devices are enabled while SystemWatchdog is set, and the struct also has an Enabled bit. REV does not say which bit the SPARK checks, so BURN clears both.
4. **What does the persist result 255 mean?** Only "0 on success" is defined, so any other code is printed raw.
5. **Which bit of SET_STATUSES_ENABLED is which frame?** Settled: bit n = STATUS_n. REVLib 2026.0.5 builds the bitfield as `|= 1 << statusIndex` (SparkFrameManager.cpp:80-134, FW26_CHANGES §2), consistent with DRV:96-103, and v1's 0x0004 did turn STATUS_2 on.
6. **What does "UNSUPPORTED" mean for a read?** The spec defines no negative reply, so the firmware reports UNSUPPORTED only when a reply is shorter than 8 bytes. Silence is reported as TIMEOUT.
7. **Receive-timestamp tick rate.** The FlexCAN timer is assumed to tick once per bit time. That comes from the i.MX RT FlexCAN design and is not checked on hardware. There is a plausibility fallback.
8. **BURN thresholds.** 30 RPM, 200 ms freshness, 120/300 ms dwell and 1.5 s response timeout are engineering choices, not spec values.
9. **Voltage cap.** 12 V × duty cap assumes a 12 V nominal bus. The SPARK may scale by the actual bus voltage.
10. **When to print `OK K<x>=`.** The line is printed when the write is queued, as in v1 (the host only logs it). The confirmation is the `PWR` line.

---

## 6. Changelog vs v1 (f242825), mapped to REPORT.md / VERIFICATION.md

| Review item | v2 |
|---|---|
| §1.3 every response ignored, no write verified | PARAMETER_WRITE_RESPONSE decoded per SPARK (`PWR`), with retries and TIMEOUT; SET_STATUSES_ENABLED, PERSIST and FV responses decoded as well |
| §1.5 SET_STATUSES_ENABLED sent with DLC 8 | DLC 4 per JSON:1025 / HDR:385. Mask/enable 0x0007 turns on STATUS_0-2. Response decoded. |
| §1.6 `setParam()` always float | Typed encoding (f/u/i/b). The K commands use the table; `PW` covers any id. |
| §1.2 BURN 255, probably "enabled" | Safe BURN (section 3): refused while moving, all-zero universal + secondary heartbeat, waits for zero output (aborts if not seen), non-blocking wait for the reply, then restore |
| §3 BURN blocked for up to ~1.25 s with no heartbeat | State machine; the 20 ms heartbeat keeps going |
| §3 RX decoded any extended frame | Hardware and software filter on device type 2 / manufacturer 5, device 1/2, data frames only |
| §5 no read-back | `PR` (spec form RTR DLC 8), `FV` |
| §5 STATUS_1 faults, current, temperature unused | STATUS_0 and STATUS_1 fully decoded, `F` line on change, `CF`, DIAG fields |
| §5 kS / per-side gains missing | `KS`/`KA` plus per-side `K<x><L\|R>` |
| §5 hall filter 136/137, status periods, current limits, voltage compensation, ramps, idle mode | In the table, writable with the right type through `PW` |
| §1.4 hall-filter lag unmeasured | `X` telemetry with hardware-corrected STATUS_2 arrival time and setpoint TX time |
| §5 voltage setpoint | `UVL`/`UVR` (VOLTAGE_SETPOINT 0/5) with watchdog and cap; `MD` raises the cap |
| `tuneBoth()` used `delay(5)` | Removed. There is no `delay()` after `setup()`, and the boot delays are removed as well. |
| v1 header / firmware CLAUDE.md stale claims (STATUS_2 "on by default", "universal HB required", PTYPE_FLOAT = 2) | Header describes actual behaviour: STATUS_2 is off by default, float = 3 |
| Serial writes could block up to 120 ms | Non-blocking `out()` for every line; the E line keeps v1's `availableForWrite()` guard |

Unchanged on purpose:
- MAX_RPM 4600, the 300 ms watchdog, the M slew (default 100), duty cap default 0.30;
- E line format, S semantics, K/A/BURN reply text;
- NeoPixel code;
- heartbeat bytes while enabled;
- slot 0 with no arbitrary feedforward in the setpoint frames.

### Not fixed or out of scope
- **kV units (REPORT §1.1).** v2 still writes whatever value the host sends. `KF` = parameter 16 = kV, which is volts per RPM on FW 26. The host yaml value 0.000197 is probably about 12× too small. That is a host and config change; confirm it with bench test T1 first.
- **Future firmware.** REVLib 2027 alpha removes parameters 112/113/136/137 (REPORT §1.8). Re-review before updating the SPARKs past 26.x.
- **FlexCAN_T4 TX queue.** When every TX mailbox is busy (for example with no ACK on the bus), `write()` falls back to the library's software queue. v1 behaved the same way, and v2 only counts these frames (`txq=`).
- **Not implemented:**
  - factory and safe resets (1/7, 1/5), left out on purpose as destructive;
  - MAXMotion;
  - PID slot selection and arbitrary feedforward in the setpoint frames;
  - GET_PARAMETER_TYPES (class 13) for looking up types of ids outside the table;
  - SET_PRIMARY_ENCODER_POSITION, left out because the host integrates E-line position and a reset would jump its odometry.

## v2c additions (2026-09-28)
- **`KS[L|R]<volts>`** sets the per-side **static-friction feedforward**, stored on the Teensy. It is sent in every VELOCITY_SETPOINT's `ARBITRARY_FEEDFORWARD` field (JSON:716, frame versionImplemented 25.0.0): int16 at bits 32–47, scale 0.0009765923, units bit 50 = 0 (volts). Value = `kS × sign(ramped setpoint)`, and 0 when |setpoint| ≤ 1 RPM, so a zero command stays zero with no chatter at standstill. Clamped to 0–2 V, default 0. Reply: `OK KS=… V (arbFF)` / `OK KSL=…`.
- **Why here:** FW 25.0.4 has no kS parameter (a write to param 204 returns `invalid_id`). This is the same field REVLib's `setReference(…, arbFeedforward, units)` uses.
- DIAG appends `ks=<L>/<R> arb=<L>/<R>` (configured kS and the feedforward in the last velocity setpoint, in volts).
- Duty and voltage setpoints still send 0 feedforward.
- **Also in v2b/v2c:** `S` and the 300 ms watchdog stop to idle (duty 0 → the SPARK idle mode) instead of a velocity-0 setpoint (`STOP_TO_IDLE`).

## v2d changes (2026-09-28, SPARK MAX FW 26.1.5)

Source: `research/evidence/firmware_can_review_2026_09_27/FW26_CHANGES_2026_09_28.md` §8 (table rows cited as C<n>) and §9 (bench tests B<n>). Host compatibility is unchanged: the E line, `OK K*` (including `OK KF=`), `OK S\r\n`, `OK L=`, `OK A*` and `OK BURN` are byte-identical, and the X line only gains fields after the mode field.

1. **STATUS_7 / STATUS_8 (C4).** The status-enable mask/enable is now **0x0187** (STATUS_0,1,2,7,8; bit n = STATUS_n, from REVLib's `1 << statusIndex`). Decoding follows JSON:633-665 / HDR:371-372, 701-702:
   - STATUS_7: I accumulator, float, bits 0-31 (JSON:641).
   - STATUS_8: closed-loop setpoint, float, bits 0-31 (JSON:657); at-setpoint bit 32 (JSON:658); PID slot bits 33-36 (JSON:659).
   - The X line appends `I7 <L_iaccum> <R_iaccum> SP8 <L_setpoint> <R_setpoint> <L_atsp> <R_atsp>`, which are whitespace-split fields 24-31. DIAG appends `iacc= sp8= atsp= slot=`.
   - This adds about 200 frames/s to the bus (~2.6 % of 1 Mbit/s).
2. **kS from one source only (C2).** The v2c `KS[L|R]` arbitrary feedforward (volts, 0 at |setpoint| ≤ 1 RPM) stays the only kS.
   - Native param 204 adds to it and applies +kS even at a 0.0 setpoint (SIM, §1).
   - The `K` command can no longer reach 204.
   - `PW … 204 <nonzero>` prints a `#` warning.
   - CHK fails if 204 ≠ 0.
3. **kA (C3).** Param 205 is ignored in plain velocity mode on FW 26 (it is MAXMotion only; docs + SIM; bench B4 is open).
   - `KA` still writes 205, as before, and now also prints `# KA: param 205 only affects MAXMotion on FW 26`.
   - `PW … 205` prints the same line.
   - Nothing writes 205 automatically.
4. **`CHK` configuration check (C6).** It runs on the `CHK` command and automatically about 2 s after boot. It uses the existing non-blocking job queue: 9 quiet requests per SPARK, and no PRD/FV/TIMEOUT lines.
   - Output, one line per item and side:
     ```
     CHK <L|R> fw <maj>.<min>.<build> ok|(expected 26.1.5)
     CHK <L|R> motorType(2) <n> brushless ok | BRUSHED: BAD, NEO needs brushless=1
     CHK <L|R> feedbackSensor(9) <n> (primary encoder)|(note: 1 = primary encoder)
     CHK <L|R> idleMode(6) <n> brake|coast
     CHK <L|R> kV(16) <v> V/RPM raw=0x........ ok | WARN kV looks like a duty/RPM value; FW 26 uses V/RPM
     CHK <L|R> voltComp(74/75) mode=<n> (off|on) nominal=<v> V
     CHK <L|R> kS(204) <v> V ok|WARN ... (Teensy arbFF KS=<v> V)
     CHK <L|R> kA(205) <v> V/(RPM/s) [(note: param 205 only affects MAXMotion on FW 26)]
     CHK <L|R> hall(136/137) period=<s> s raw=0x........ depth=<idx> (<n> samples)
     CHK <L|R> conv(112/113) pos=<v> vel=<v> ok|BAD: must be 1.0
     CHK B idleMode L=<n> R=<n> ok|WARN L/R differ ...
     CHK OK | CHK FAIL <reason> <reason> ...
     ```
     A missing reply prints `NOREAD` for that item.
   - FAIL reasons: `<L|R>:fw?`, `:fw<26`, `:motorType=<n>`, `:kV<5e-4` (only when FW ≥ 26), `:kS204!=0`, `:conv!=1`, `:noread(<id>)`, `idleMode_L!=R`, `queue_full`.
   - Printed only, never a FAIL: feedback sensor, voltage compensation, kA, and the hall filter (136 is shown as float **and** raw hex, per B8).
5. **Motor-type interlock (new).** Compile-time `EXPECT_BRUSHLESS = true`.
   - **Trigger.** Any read-back of param 2 blocks a SPARK when the value is not 1 (brushless). Read-backs come from a read reply, or from a successful write reply, whose VALUE is the controller's current value.
   - **While blocked,** every setpoint to that SPARK is replaced by **duty 0** in every mode: velocity, duty, voltage, BURN, stop. Its velocity ramp is held at 0, so unblocking cannot step. The firmware prints `!! MOTOR TYPE <n> on <L|R>: motion blocked (NEO needs brushless=1)` once per change.
   - **Unblock.** A later read of 1 clears the block and prints `# MOTOR TYPE 1 on <side>: motion unblocked`.
   - **Before the first successful read after boot,** motion is allowed, as in v1.
   - **Re-checks.** Param 2 is re-read quietly every 5 s (skipped during BURN/CHK, never queued twice) and after every `PW` to param 2.
   - **Background.** The FW 26.1.5 update left param 2 = 0 on both NEO controllers, and an 8 % duty command drew about 30 A into stalled motors.
   - **Repair.** `PW B 2 1 FORCE`, check that CHK passes, then `BURN`.
6. **Naming/docs (C10).**
   - Header comments and the `FV` "expected" note now say 26.1.5.
   - The boot banner says `v2d`.
   - Param 16 is documented as kV, V/RPM on FW 26 (was kF duty/RPM on FW 25). The `KF` letter and reply are unchanged.
   - The table gains 2, 97, 165, 193, 199 and 200.
7. **Unchanged:** heartbeat bytes and period, stop-to-idle (`S`/watchdog), BURN sequence, `HB0`, the `PW` FORCE denylist, E-line format, MAX_RPM, slew, duty cap.

**Side effect to know about:** BURN is still refused while any parameter request is pending. The 5 s quiet param-2 read and the boot CHK can therefore occasionally cause `ERR BURN refused: parameter requests pending`. Retry after a moment.

**Not done (still open per FW26_CHANGES §8):**
- the kV value itself: C1 is a host/yaml change, gated on bench B1;
- idle-mode write + BURN (C7);
- the param-97 check (use `PR B 97`);
- force-enable params (C5);
- MAXMotion (C9).

**Desk test:** a host-side harness compiled the unmodified sketch against mock FlexCAN/Serial/NeoPixel with two simulated SPARKs (34/34 checks). It covered:
- the SSE bytes `87 01 87 01`;
- boot CHK FAIL on brushed/kV/idle mismatch;
- one `!!` line per change;
- duty-0 substitution in velocity, duty and voltage mode;
- unblock via `PW L 2 1 FORCE`, and via a periodic re-read;
- the `KA` warning and byte-exact `OK KA=`/`OK KF=`/`OK S`;
- CHK OK after the fixes;
- STATUS_7/8 decode, X-line field indices 24-31, and the DIAG fields;
- CHK with silent SPARKs (NOREAD, FAIL, no TIMEOUT spam).
