# Teensy Diff-Drive Firmware

Thin USB-Serial ↔ CAN bridge for the IGVC differential drive. The Jetson's ROS2 `actuator_node` owns the diff-drive kinematics and streams per-wheel RPM setpoints; this firmware forwards them to each SparkMAX's built-in velocity PID and echoes wheel feedback.

Derived from `Paarseus/AVL-IGVC2026/firmware/teensy_sparkmax_arduino/teensy_sparkmax_fw26_synced/` with several fixes and simplifications (see "Delta from upstream" below). This folder is local staging — the canonical upstream is the IGVC firmware repo. Nothing here is wired into the ROS2 build; it's flashed independently.

## Tuned PID gains (burned to SparkMAX flash on both controllers, 2026-04-23)

| Gain | Value | Rationale |
|---|---|---|
| **kFF** | **0.000197** | ≈ 1 / Phase-4 slower-wheel max RPM (5072) — feedforward won't saturate |
| **kP**  | **0.0004**   | Highest stable P before oscillation at kP=0.0008 |
| **kI**  | **9e-07**    | Smallest kI that eliminates steady-state error, no wind-up osc |
| **kD**  | **0**        | Skipped — velocity PID on NEO usually doesn't need D |

Verified via Phase 6c: 99% tracking at 500-3000 RPM, 0.24% trial-to-trial precision, 0.83% L/R sync delta.

These are persisted in flash via PERSIST_PARAMETERS (cls=63 idx=15). The actuator_node yaml also pushes them on startup as redundant override.

## Hardware

- Teensy 4.1 — CAN1 on pins CTX1=22, CRX1=23
- SN65HVD230 or TJA1051T/3 CAN transceiver, 120 Ω termination at each bus end
- 2× REV SparkMAX **FW 26.1.4** — CAN ID 1 = left wheel, CAN ID 2 = right wheel
- 2× REV NEO brushless motors (free speed 5676 RPM)
- CAN bus: 1 Mbit/s, extended (29-bit) frames only

## Build & flash

```bash
# From the Jetson (preferred — enables remote iteration):
arduino-cli compile --fqbn teensy:avr:teensy41 firmware/teensy_diff_drive
arduino-cli upload  --fqbn teensy:avr:teensy41 -p /dev/ttyACM0 firmware/teensy_diff_drive

# Interactive serial monitor:
screen /dev/ttyACM0 115200
# or:
python3 -m serial.tools.miniterm /dev/ttyACM0 115200
```

## Serial protocol (115200 baud, newline-terminated, case-insensitive)

**Host → Teensy**

| Command | Effect |
|---|---|
| `L<rpm> R<rpm>` | Set both wheel setpoints (signed RPM) |
| `L<rpm>` / `R<rpm>` | Set one wheel, other unchanged |
| `S` | Stop both wheels (zero RPM, resets watchdog) |
| `D` | Print one `DIAG` line |
| `KP<v>` / `KI<v>` / `KD<v>` / `KF<v>` | Tune SparkMAX PID slot 0, both motors |
| `BURN` | Persist current PID gains to SparkMAX flash |

**Teensy → Host**

| Line prefix | Meaning |
|---|---|
| `E L<rpm> <pos> R<rpm> <pos>` | 50 Hz wheel feedback (motor RPM + cumulative rotations) |
| `OK …` | Command ack |
| `ERR …` | Parse failure |
| `DIAG …` | Response to `D` |
| `# …` | Log / info line (watchdog trips, boot banner, etc.) |

**Safety**: 300 ms host watchdog — no `L/R/S` in that window → both wheels forced to 0 RPM and one `# WDT host-timeout stop` line emitted. `MAX_RPM = 3000` clamp in `setVelocity()`.

## SparkMAX prerequisites (configure once via REV Hardware Client)

These cannot be reliably configured over CAN and must be set via USB-C from REV Hardware Client on a Windows host before first bring-up:

- **CAN ID** = 1 (left) or 2 (right)
- **Motor Type** = Brushless
- **Feedback Sensor** = Hall Sensor (NEO internal encoder)
- **Velocity Conversion Factor** = 1.0 — otherwise `MAX_RPM = 3000` is in the wrong units
- **Position Conversion Factor** = 1.0 — otherwise the `E` line position scale is wrong

## Research status — CAN protocol verification (2026-04-19)

Four parallel research agents cross-checked the protocol against REVLib-2024, REVLib-2026 driver headers, REV SPARK-MAX-Server proto, and WPILib DriverStation source. Summary:

**Firmly confirmed for FW 26.1.4**
- Duty cycle setpoint: `cls=0 idx=2`, float LE in bytes [0:3]
- Universal Heartbeat CAN ID `0x01011840`, byte 3 = `0x12` (enabled=bit1, systemWatchdog=bit4)
- Secondary Heartbeat `cls=11 idx=2 dev=0`, 0xFF×8 (ignored after first Universal HB, harmless)
- STATUS_2 at `cls=46 idx=2`, velocity@[0:3] + position@[4:7] as `float32 LE` (FW 25 reworked status frames from class 6 to class 46)
- SparkMAX heartbeat timeout: 100 ms
- PID slot 0 IDs: `kP_0=13, kI_0=14, kD_0=15` (from REV `SPARK-MAX-Types.proto`)
- `kF_0 = 16`, `kIZone_0 = 17` (slot stride = 8, so slot 1: kP=21, kI=22, kD=23, kF=24)
- `PTYPE_FLOAT = 2`
- BURN magic bytes `0xA3 0x3A`
- Native units: motor RPM (pre-gearbox), cumulative motor rotations, when both conversion factors are 1.0
- STATUS_2 is **on by default at 20 ms** on a factory SparkMAX — our `SET_STATUSES_ENABLED` call is redundant

**Real bug fixed from upstream `_synced.ino`**
- **kFF parameter ID was 17 in `_synced.ino`. Correct value is 16.** Parameter 17 is `kIZone_0` (integrator dead zone), not feedforward. The upstream has been silently disabling the integrator on every `KF<val>` command instead of tuning feedforward for who knows how long. Fixed here in `teensy_diff_drive.ino` around line 66.

**Verified (2026-04-23 bench session) — see `FINDINGS.md`**
- **Velocity setpoint**: upstream `cls=1 idx=2` legacy path is **dropped silently on FW 26.1.4**. REV-Specs 2.1.0 authoritative frame is **`cls=0 idx=0`** (firmware now uses this). When the legacy path was active, commanded RPM produced zero motion — confirming FW 25+ removed the legacy.
- **Parameter SET layout**: **`cls=14 idx=0`, DLC=5, `[param_id@0, float32_LE@1:4]`** per REV-Specs 2.1.0. The old cls=48 / 8-byte-payload frame silently dropped on FW 26.1.4. Verified by writing sentinel kP=0.00042069 via firmware and reading back in Hardware Client on both controllers.
- **BURN frame**: `cls=63 idx=15`, magic `0x3AA3` LE (bytes `A3 3A`). Phase 7 power-cycle readback pending.

## Delta from upstream `_synced.ino`

| Area | `_synced.ino` | This firmware |
|---|---|---|
| kFF parameter ID | 17 (writes to kIZone, wrong) | 16 (correct, per REV proto) |
| `configurePID()` in `setup()` | Yes — clobbers BURNed values every boot | Removed; flash is authoritative |
| `V<rpm> W<rpm>` serial mode | Yes (confusing RPM sum/diff hack) | Removed |
| `directMode` flag / branching | Yes | Removed (everything is direct) |
| Host watchdog | None | 300 ms → zero both wheels |
| Feedback print | 500 ms human-readable | 50 Hz machine-readable `E` line |
| Serial grammar | Ad-hoc | Structured `OK / ERR / DIAG / # / E` |
| Non-blocking TX guard | No | `Serial.availableForWrite() >= 64` |
| Single-motor variants | Leftover from single-wheel sketches | Removed (dual-only) |

CAN wire format is otherwise **byte-for-byte identical** to `_synced.ino` except for the kFF parameter ID.

## TODOs

- [x] Run phases 2-6 of `BRING_UP.md` — done 2026-04-23, see `FINDINGS.md`
- [x] Verify cls=14 PARAMETER_WRITE landed (sentinel readback via Hardware Client)
- [x] Confirm cls=0 idx=0 velocity setpoint works on FW 26.1.4 (it does)
- [x] Confirm kFF=16 is the correct feedforward parameter ID (it is)
- [x] STATUS_2 enable frame is NOT redundant on FW 26.1.4 — disabled by default, must be enabled explicitly (firmware keepalives it every 1 s)
- [ ] Phase 1 hand-spin resistance check (manual, user-facing)
- [ ] Phase 7 BURN persistence across power cycle (needs manual 12V toggle)
- [ ] Upstream the kFF=16 fix to `Paarseus/AVL-IGVC2026` as a PR against `_synced.ino`
- [ ] Consider adding a generic `GET <id>` command for in-field parameter readback (low priority — Hardware Client handles verification for now)
- [ ] Decode STATUS_1 fault flags in DIAG (over-current, brownout, sensor fault) — nice-to-have diagnostic
