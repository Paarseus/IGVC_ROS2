# firmware/ — Review

## Summary

The `firmware/` tree is the lowest-level safety boundary in the IGVC_ROS2 workspace and the single highest-risk place for a runaway-vehicle defect. It contains:

- `teensy_diff_drive/teensy_diff_drive.ino` — 392-line production sketch that bridges Jetson USB-CDC serial to two REV SparkMAXes over CAN1. Owns the heartbeat, the host-to-firmware watchdog, the per-wheel velocity/duty setpoint dispatch, and PID parameter persistence (`BURN`).
- `teensy_diff_drive/teensy_bridge.py` — shared Python helper for the bring-up scripts. Connection, E-line parsing, CSV logging, signal handlers.
- `teensy_diff_drive/test.py` — standalone CLI for ad-hoc bench testing.
- `teensy_diff_drive/phase{1..7}_*.py` — 9 one-shot bring-up scripts (Phase 1 through Phase 7, with 6b and 6c). Already run successfully on 2026-04-23 per FINDINGS.md.
- `teensy_diff_drive/{CLAUDE.md, BRING_UP.md, FINDINGS.md}` — three docs that together form a model bring-up record.
- `teensy_diag/teensy_diag.ino` — 216-line CAN sniffer used to verify the protocol pre-Phase-2; vestigial post-bring-up.

**Headline verdict.** The firmware is closer to competition-ready than the rest of the workspace would suggest. The author is clearly fluent in Arduino/Teensy idioms — `volatile`, FIFO+ISR receive, fixed-rate scheduler with rollover-safe subtraction, no `String`, no `malloc`, no `delay()` in the 50 Hz tick path, MAX_RPM/MAX_DUTY clamps as defense-in-depth. Multiple subtle SparkMAX FW 26.1.4 protocol bugs from the upstream `_synced.ino` were caught and fixed here (kFF parameter ID 16 vs 17, velocity setpoint cls=0 idx=0 vs cls=1 idx=2, PARAMETER_WRITE cls=14 vs cls=48). The PID tune is competition-grade (99% tracking, 0.83% L/R sync delta).

**The five things that prevent shipping today.** (1) **The vehicle has no hardware e-stop wired into the firmware** — IGVC inspection failure on day 0. (2) **The watchdog trip leaves `ctrl_mode = MODE_VELOCITY`**, which is the exact failure mode the `S` command was rewritten to avoid (no Brake-idle engagement); a one-line fix. (3) **No hardware MCU watchdog (RTWDOG)** as backstop against firmware lockup. (4) **`BURN` fires unconditionally** with a 50 ms `delay()` inside it — can both glitch the heartbeat mid-motion *and* persist a pathological gain (e.g. atof("garbage")=0) to flash. (5) **The watchdog auto-rearms on the next valid command** rather than requiring an explicit reset.

The rest is P1 anti-patterns (volatile gaps, silent buffer overflow, `atof` zero-on-bad) and P2 cleanup (duplicate `Teensy` classes, magic-number hoisting, dead diagnostic sketch). The bring-up phase scripts are the right tool for what they did but should be promoted to a regression harness — currently they only print, they don't assert. The CSVs they produced are a competition-prep gold mine.

**Length.** ~390 LOC of safety-critical Arduino C++; ~1,400 LOC of Python bring-up scripts; ~700 lines of well-organized docs. Total firmware footprint is small; review is correspondingly tractable.

## Per-file findings

### teensy_diff_drive.ino

The single highest-risk file in the workspace. 392 lines, written carefully — most of the safety contract is correctly stated in comments, and the critical bugs that the bring-up uncovered (kFF parameter ID, cls=0 idx=0 velocity setpoint, cls=14 PARAMETER_WRITE) are fixed. There are still several embedded anti-patterns and one P0-shaped concern around the `S` (stop) command's interaction with the host watchdog.

Findings, by concern.

**Watchdog discipline (lines 47, 98–100, 252–253, 293–294, 355–362).** The host-to-firmware command-loss timeout is implemented and works. `WATCHDOG_MS = 300` is in the upper end of the recommended 50–500 ms window — at 1.5 m/s the vehicle travels 45 cm in 300 ms, which is acceptable for a chassis at IGVC speeds but worth flagging. The trip path zeroes both `cmd_rpm` *and* `cmd_duty`, leaves the current `ctrl_mode` alone, and then the next tick of the 50 Hz control loop sends `setVelocity(_, 0)` or `setDuty(_, 0)` to both SparkMAXes. Heartbeats keep flowing during the trip (`sendHeartbeats()` runs unconditionally on line 364), so the SparkMAX stays enabled and its onboard PID drives the wheel to zero — the right thing for a controlled stop.

The `wdt_tripped` flag is correct in spirit (single `# WDT host-timeout stop` log line per trip) but has a latching gap. Once tripped, any new valid `L`/`R`/`S`/`UL`/`UR`/`K`/`B` line silently re-arms the watchdog (see line 252 `wdt_tripped = false` in `U`, line 224 in `S`, line 294 in default/velocity, line 277 implicit because no clear). The CLAUDE.md and standards docs both say "refuses to re-arm without an explicit reset" is best practice; this firmware re-arms on the next setpoint. For a tracked vehicle with no operator-near-the-wheels guarantee, that means a host that crashes-and-resumes-the-publisher will see motion resume mid-stride. P1 — minimum, force the host to send an explicit `S` to clear `wdt_tripped` before any non-zero setpoint is honored.

**CAN heartbeat (lines 125–132, 364).** The 0x01011840 Universal Heartbeat is sent from the 50 Hz control tick (`CTRL_DT_MS = 20`) — fixed-rate scheduler driven by `millis() - t_ctrl >= CTRL_DT_MS` (line 352, the *correct* subtraction idiom, rollover-safe). Cadence is 50 Hz, comfortably below SparkMAX's 100 ms timeout even with one missed beat. Heartbeats also fire during a watchdog trip (line 364 is inside the `if (now - t_ctrl >= CTRL_DT_MS)` but outside the watchdog `if`), which is the correct controlled-stop behavior — losing heartbeat would coast the wheels (Coast idle) or shudder the bridge (Brake idle); neither is what you want for a clean ramp to zero. Good.

The Secondary Heartbeat (line 130–131, all-`0xFF`) is *also* sent every 20 ms — the FINDINGS doc says it is "ignored after first Universal HB, harmless." Fine, but it doubles the bus utilization for no benefit; if you ever drop frames under a CAN burst this is the first thing to remove.

The heartbeat byte 3 = `0x12` is documented in-line ("byte 3 = 0x12 (enabled + sysWdt)") with the correct bit interpretation. Magic numbers in `uni[8]` (`0x78, 0x01, 0x00, 0x12, 0x59, 0x04, 0x00, 0x60`) are otherwise opaque — the comment names byte 3 only. Worth a sentence on what each byte is, even if "REV opaque" for most of them.

**CAN ISR / mailbox (lines 84, 89–91, 181–206, 321–327).** The receive path uses `FlexCAN_T4` FIFO + `onReceive()`; per the standards doc, this callback runs in ISR context (PJRC and FlexCAN_T4 README both say so), which the code clearly understands — `meas_rpm`, `meas_pos`, and `got_enc` are declared `volatile` (lines 89–91), and `bus_voltage` is `volatile` (line 101). Good.

But the ISR–`loop()` synchronization is incomplete:
- `meas_rpm` and `meas_pos` are 32-bit `float`s. On Cortex-M7 (Teensy 4.1) a single 32-bit aligned read is atomic at the load-instruction level, but the *pair* (rpm, pos) is read in three separate places (DIAG line 232–234, E-line line 387–389) without `noInterrupts()`/`interrupts()` guarding them. A frame arriving mid-`Serial.printf` can update one and not the other for the duration of that printf, so the host can see one wheel's RPM from cycle N and its position from cycle N+1 in the same E-line. Not a safety bug but it does desynchronize wheel odometry slightly. The standards doc rule 4 explicitly calls this out: "multi-byte ISR-shared reads guarded by `noInterrupts()`/`interrupts()`."
- `tx_count` and `rx_count` are non-`volatile` `uint32_t` (line 99). `tx_count` is incremented from `loop()` only (line 118 — fine), but `rx_count` is incremented from the ISR (line 183) and read from `loop()` (line 229 in DIAG). Missing `volatile` — the compiler may cache it in a register inside the printf and miss recent ISR increments. P1 anti-pattern, listed verbatim in the standards anti-pattern list.
- `bus_voltage` is `volatile` and decoded inside `onCanRx` from a single STATUS_0 frame — the read in DIAG (line 235) is a single 32-bit aligned float, atomic on M7. Fine.

`RX_SIZE_256` / `TX_SIZE_16` queue sizes (line 84) are reasonable. There's no dropped-frame counter exposed; FlexCAN_T4 has overrun reporting available but the code doesn't surface it. Worth adding to DIAG.

**Loop discipline.** `delay()` appears twice in command handlers (not the 50 Hz tick): `delay(5)` in `tuneBoth()` (line 176) between L/R PARAMETER_WRITEs, and `delay(50)` in BURN (lines 258–259). The 50 ms BURN delay is the bigger concern — close to SparkMAX's 100 ms heartbeat tolerance; if BURN fires mid-motion, one missed heartbeat → magenta-LED disable. Standards anti-pattern list calls out `delay()` on control paths. P1.

`String` is not used (good — `char buf[96]` + `Serial.printf`). DIAG line at line 229 is not gated by `availableForWrite()` — blocks if host TX-stalled. P2.

`millis()` rollover-safe subtraction idiom used everywhere (lines 352, 355, 377, 384). Good.

**Serial protocol.** Line-oriented ASCII, 115200 baud, defensible given USB-CDC link-layer CRC. Parser is 96-byte bounded with `\n`/`\r` termination. Two parser bugs:
- **No NACK on too-long line.** Lines >96 chars have characters silently dropped, then the leading fragment parses on next `\n`. Silent corruption. P1.
- **`atof(lp + 1)` on `L\n` (no number) returns 0** — line truncated by SSH stall produces a half-stop, not error. P1.
- **`KP<garbage>` parses garbage as 0** via `atof` and writes 0 to flash. Combined with unconditional BURN, can persist 0 to slot 0. P1.

Bidirectional handshaking otherwise correct: every command path emits `OK ...` or `ERR ...`.

**Mode state machine (lines 95–96, 215–226, 248–254, 290–296).** Three modes are implicit in the code: VELOCITY, DUTY, and "stopped." `MODE_STOP` from CLAUDE.md is *not* a separate enum value — `S` flips into MODE_DUTY with `cmd_duty = 0.0`. The comment at line 217–219 explains why: keeping MODE_VELOCITY with `cmd_rpm = 0` runs the velocity PID against zero, which the SparkMAX's Brake-idle setting doesn't engage, causing the chatter that the FINDINGS document called out. So `S` switches to MODE_DUTY=0 to engage Brake idle. Subtle but correct. Worth a one-line comment in CLAUDE.md too — anyone reading the source first will be confused.

The bigger issue: there is no transition from "velocity command at speed" to MODE_DUTY=0 *during* a watchdog trip. Lines 360–361 zero both `cmd_rpm` and `cmd_duty` but leave `ctrl_mode` unchanged. So if the host had been sending `L100 R100` (MODE_VELOCITY) and crashes, the watchdog trip continues sending `setVelocity(_, 0)` — the velocity PID is still running against zero, which (per the FINDINGS comment for `S`) is the failure mode `S` was specifically written to avoid. The fix is one line: `if (wdt_tripped) ctrl_mode = MODE_DUTY;` or equivalent at line 360–361. As written, the watchdog stop is *not* the same stop as `S` — it's the version `S` was rewritten to fix. P0-adjacent. The wheels will stop (PID drives to zero) but the SparkMAX won't engage Brake idle, so on a slope the chassis will roll. For a chassis at walking speed this is recoverable but it's a real bug.

Malformed `K` (line 263–278): if the second character isn't `P/I/D/F`, returns `ERR K?`. If a number can't be parsed, `atof` returns 0 — and the firmware *writes 0* to the parameter and reports `OK Kp=0.0`. Writing kP=0 silently is mostly harmless (kP=0 disables proportional) but writing kFF=0 will halt motion under any velocity setpoint. The host test scripts work around this by sending well-formed `K` lines, but if a stray UART glitch turned `KP0.0004` into `KPO.0004`, atof returns 0 and kP gets clobbered. P1 — require `atof` to consume non-empty input or report `ERR K-value`.

**PID / BURN safety.** `BURN` (lines 257–261) takes no confirmation, no precondition. Possible to BURN mid-motion: 50 ms `delay()` (heartbeat-gap risk), persists whatever is in slot 0 (may be a pathological mid-tune value), velocity setpoints continue during write. P1 — gate on wheels-at-rest, or require `ARM_BURN` then `BURN`.

`KP<garbage>` writes 0 silently (atof returns 0). `KP<garbage>` then `BURN` persists 0 to flash; field-broken until USB-C recovery. Add `isfinite(val)` check at line 266.

No PARAMETER_READ — firmware writes, host trusts. CLAUDE.md TODO is `GET <param_id>`.

**DIAG (line 229–235).** Adequate: tx/rx/wdt/mode/per-wheel meas/cmd/duty/bus voltage. Missing: uptime, STATUS_2 last-seen-per-wheel (encoder-stuck detection), STATUS_1 fault-decode (TODO). DIAG is poll-only (response to `D`); 50 Hz E-line has wheel state but counters/wdt require poll.

**E-stop path (none).** Firmware has no GPIO input wired to a hardware e-stop circuit. IGVC rules: "Vehicle E-Stops must be hardware based and not controlled through software." Software-only e-stop fails inspection. Firmware needs `pinMode(ESTOP_PIN, INPUT_PULLUP)` + latched-fault state — not currently present. P0 for IGVC.

**Magic numbers / function length / RTWDOG.** Most constants named at lines 41–82. Unnamed: heartbeat bytes (127, 130), STATUS_2 enable mask, `>= 64` USB-CDC packet size, `delay(5)`/`delay(50)`. P2. All functions ≤ 60 lines except `handleLine()` at ~90 with inline lambdas — borderline NASA P10 rule 4. RTWDOG not enabled — standards rule 9. P1.

No `-Wall -Wextra -Wpedantic` evidence — P2. No recursion/goto — good.

### teensy_bridge.py

238-line shared helper. Solid design — one `Teensy` class, plain functions for `set_gains`/`run_duty`/`run_velocity`, CSV `open_log` context manager, `install_stop_handler` for SIGINT/SIGTERM. Type hints throughout, Python 3.10+ syntax, f-strings.

Issues:
- **`Teensy.stop()` swallows `Exception` silently** (line 57). If serial closed mid-stop, SIGINT exits clean while motors might still be commanded. P1 — at least `print(e, file=sys.stderr)` first.
- **No `OK S` readback verification.** `time.sleep(0.1)` and trust. Fine for bench, not for competition launcher.
- **E-line regex (line 32) and DIAG regex (lines 33–37) tightly coupled** to firmware printf format. A `%.0f` → `%.1f` change silently drops frames. P2 — make tolerant or add a contract test.
- **`drain_for` has no host-side watchdog.** If firmware goes silent, refresh keeps firing forever. P2 — abort on no-E-line in N seconds.
- **`set_gains` uses `gap=0.20` (200 ms)** because "50 ms gaps wedged USB" (comment line 134). Likely interaction with the firmware's own `delay(5)` between left/right param writes. Worth investigating.
- **`run_duty` and `run_velocity` near-identical** (lines 159–206). Merge.
- **`settle_frac=0.4`** is fixed-fraction; for a 15s stability run that throws away 6s. Use fixed-time settle.
- **`M_PER_REV = 0.01994` not declared here** — Phase 4 declares; Phase 5 inlines. Hoist.

### test.py

239-line standalone CLI. Re-implements `Teensy` (lines 40–81) instead of importing from `teensy_bridge.py` — same logic, same `stop()`-swallows-`Exception` bug. P2 — pick one.

Three concrete issues:
- **Wrong duty clamp comment** (line 19): "Duty clamped to |d| <= 0.6 (matches firmware MAX_DUTY)" — firmware is 0.30. The clamp at lines 99–100 is also 0.6. Firmware re-clamps so not unsafe, but the comment lies.
- **`./test.py raw "BURN"` not gated** (lines 163–174). A stray paste writes to flash. Refuse `raw` matching `BURN` or `K*` without `--allow-flash`. P1.
- **No host-side watchdog** in the refresh-during-duration loop (lines 106–111, 126–131). If firmware goes silent, the script keeps sending forever.

`--max-time 10` cap (line 202), SIGINT/SIGTERM handler that calls `t.stop()` (line 217), `try/finally` ensuring `t.close()` (line 234) — all good. Argparse subparsers clean.

### phase{1-7}_*.py (cluster)

Nine scripts, ~1,400 LOC, ran successfully on 2026-04-23. The right tool for one-shot bring-up per standards § 8 ("Hardware bring-up is an iterative, exploratory phase"). Phase 4 linear fits, Phase 5 stability analysis, Phase 6/6b/6c PID tune progression, Phase 7 BURN verification — textbook. CSV archives in `data/`; FINDINGS.md captures headlines; BRING_UP.md is replayable.

Issues:
- **Duplicate 6-line boilerplate at top of every script** (sys.path, Teensy(), install_stop_handler, t.stop, sleep). Wrap in a `bench.session()` context manager. P2.
- **`hold_and_log` / `run_and_capture` / `run_velocity` are 3 versions of the same loop** (Phase 5, Phase 6c, bridge). Consolidate. P2.
- **Phase 7 BURN sentinel overwrites tuned kP without restoring** (line 52: writes 0.00013579, BURNs, never restores). Operator must re-tune afterward. P1 — read-write-burn-restore-burn pattern.
- **Phase 7 has no machine verification** — relies on operator typing y/n after Hardware Client lookup (no firmware GET command yet). Procedure, not test.
- **Phase 6 superseded by 6b.** Operator might run 6 and miss the kP+kI+oscillation-detection improvements. Delete Phase 6 with a tag, or rename. P2.
- **Threshold values scattered** across scripts (`STEP=0.01`, `DUTY_MAX=0.15`, `RPM_THRESHOLD=20`, `M_PER_REV=0.01994`, etc.). Hoist common values to `teensy_bridge.py`. P2.
- **No phase loads a baseline CSV and asserts against it** — they only print. Standards § 8 calls this "calibration drift detection." Small lift to add `regression_check.py` that wraps Phase 4/5/6c. P2.
- **No `--dry-run`/`--no-motors` flag** for smoke-testing logic. P2.
- **No unit tests** on pure helpers in `teensy_bridge.py`. P2.

Common safety pattern is consistent across phases: every motor-spinning script imports `install_stop_handler`, calls `t.stop()` + `t.close()` at end, uses `run_velocity`/`run_duty` (both end with `t.stop()`), and has a docstring "Safety:" block. Good consistency.

### CLAUDE.md / BRING_UP.md / FINDINGS.md

Three docs forming one of the best-documented firmware bring-up trails in the workspace. Pattern (CLAUDE.md = stable spec, BRING_UP.md = test plan, FINDINGS.md = post-session journal) is reusable.

- **CLAUDE.md (128 lines)** — hardware inventory, build/flash, serial protocol, SparkMAX prereqs, "research status" pre/post bench, "Delta from upstream" table, honest TODOs. Tuned PID gains table is the canonical source.
- **BRING_UP.md (409 lines)** — phase-by-phase test plan with prerequisites, risk, goal, procedure, pass criteria, fail modes per phase. Phase 4's "Pre-flight safety check" is a tick-list; "Safety policy" reads like industrial machinery doc. Stale `~/AVROS/install/setup.bash` at line 348 is the only mistake. P2.
- **FINDINGS.md (153 lines)** — bench session post-mortem. The "Power rail ceiling identified" section is the clearest statement in the workspace of why sustained ground speed is 1.0 m/s. Preserves wrong-then-right reasoning trail (initial `kOutputMax_0` hypothesis → re-interpreted as PID-saturation under Brake idle).

**Cross-doc consistency.** PID gains match across CLAUDE.md, FINDINGS.md, and `phase6c_pid_verify.py` constants. CAN frame formats (PARAMETER_WRITE cls=14 DLC=5, BURN cls=63 idx=15 magic 0x3AA3) documented identically in CLAUDE.md and the firmware comments. New operator could re-derive the protocol from CLAUDE.md alone.

**Bring-up replayable.** ~2-hour budget per BRING_UP.md, with detailed commands and expected values per phase. Better than typical embedded-firmware docs.

### teensy_diag/teensy_diag.ino

216-line CAN sniffer used to verify the protocol pre-Phase-2. Same idioms as production sketch. Header correctly states "DOES NOT send velocity setpoints. Safe to run with NEO motors connected."

Issues:
- **Same `volatile` discipline gap as production sketch.** `rx_count`, `n_seen`, `verbose`, `seen[].count` all touched in ISR + loop without `volatile`. P1.
- **TX command parser fragile** (lines 147–164). The outer `while (*p == ' ' && len < 8)` only iterates if current char is a space — depending on exact whitespace formatting, may parse 0, 1, or all bytes. P2.
- **TX command can issue any frame including velocity setpoints** — undermines the "no setpoints" guarantee in the header. Refuse to send to known setpoint IDs, or warn. P2.
- **`printSummary()` not gated by `availableForWrite()`** (line 124) — with 32 unique frames blasts ~2.4 kB through a 64-byte USB buffer, blocks-and-drains. P2.

**Status.** Vestigial — bring-up done, sketch unused. Standards § 8: "promote to a fixture, or delete with a tag." Currently "left in tree, has bugs, nobody runs it." Pick one path. P2.

## Safety analysis (dedicated)

**Failure-mode table** (rows already covered in detail in per-file findings; this is the cross-reference).

| Failure | Detected? | Response | Verdict |
|---|---|---|---|
| Host crash / ROS node dead | Yes — watchdog at 300 ms | Wheels zeroed; `ctrl_mode` stays VELOCITY (no Brake-idle) | P0-adjacent |
| USB unplugged | Yes — same path | Same | OK (intentional, comment line 346) |
| Firmware lockup | No — RTWDOG not enabled | Heartbeat stops; SparkMAX disables at 100 ms | P1 |
| CAN frame drops | Counted but not surfaced | Silent | P2 |
| Malformed `K<garbage>` | No — atof returns 0 silently | Writes 0 to RAM; BURN persists | P1 |
| Serial line >96 chars | No — silent drop after byte 96 | Parses fragment | P1 |
| `BURN` mid-motion | No — fires unconditionally | 50 ms `delay()`, heartbeat at edge of tolerance | P1 |
| Mushroom e-stop | NOT IN FIRMWARE | No GPIO input wired | **P0 for IGVC** |
| Wireless e-stop | NOT IN FIRMWARE | Same | **P0 for IGVC** |
| Brown-out (12V sag) | Logged; no auto-action | Jetson may reboot; documented | P0 hardware fix in progress |
| Stale encoder (no STATUS_2) | `got_enc` flag exists but unchecked | Last value stays in DIAG/E-line | P2 |
| MAX_RPM exceeded | Yes — clamp at 3000 in setVelocity | Clamped | OK (defense in depth) |
| Motor inversion misconfig | No — relies on SparkMAX param | If wrong, PID runs away | Documented; SparkMAX-side fix |

**Worst-case scenarios.**

1. **Host crashes mid-turn-at-speed.** 300 ms later watchdog trips, wheels driven to 0 by velocity PID (no Brake-idle), settle time ~340 ms (Phase 6c). Total stopping distance ~64 cm at 1 m/s. Flat surface OK; slope rolls. One-line fix.
2. **`K<noise>` + `BURN`.** `KP0.O004` (letter O) → atof=0 → kP=0 in RAM → BURN persists. Field-broken until USB-C recovery. Strict numeric parse + refuse-BURN-on-zero-gain.
3. **IGVC inspection.** Software-only e-stop fails inspection day 0. Hardware mushroom + wireless + contactor required. Firmware needs GPIO input + latched-fault.
4. **BURN during motion.** 50 ms heartbeat gap right at SparkMAX 100 ms tolerance; one missed beat → magenta-LED disable → lurch. Gate BURN on wheels-at-rest.

**Layered-defense status.** Standards § 5 prescribes 5 layers; firmware currently provides only layer 5 (`S` command + watchdog). Layers 1–4 (mushroom button, wireless e-stop, contactor, GPIO input → latched-fault) are absent. The firmware can implement layer 4 with a few hours of work; layers 1–3 are hardware-team scope.

**Safety verdict.** Two killer issues: missing hardware e-stop integration and the watchdog `ctrl_mode = VELOCITY` bug. Both are addressable in firmware in a short session. Everything else is P1/P2.

## Cross-cutting issues

- **Volatile/atomicity gap.** Both sketches: `rx_count`, `n_seen` (diag), `verbose` (diag) ISR-shared without `volatile`. `meas_rpm`/`meas_pos` pair read without `noInterrupts()` brackets — torn pairs in E-lines. P1.
- **Code duplication across host scripts.** `Teensy` class in 2 files; "send refresh, parse E-line" loop in 4 places (bridge, test, phase5, phase6c). P2.
- **`M_PER_REV = 0.01994`** declared in Phase 4, inlined in Phase 5. Hoist. P2.
- **Tuned PID gains in 3 places** (CLAUDE.md, phase6c, actuator_params.yaml). Drift risk. P2.
- **No `-Wall -Wextra -Wpedantic`.** P2.
- **No unit tests** for pure helpers (`stats`, regex). P2.
- **No CI for `arduino-cli compile`.** P2.
- **Stale `~/AVROS` path** in BRING_UP.md line 348. P2.
- **No firmware version stamp** on boot banner — `__DATE__ __TIME__` is one line. P2.
- **RTWDOG not enabled.** P1.
- **Bring-up CSVs may not be in `.gitignore`** — verify. P2.
- **`setup()` does not push initial PID gains** — flash is authoritative; replaced SparkMAX silently uses factory defaults. CLAUDE.md says actuator_node pushes on startup, but firmware itself does not. Trade-off documented; consider logging current gains on boot once GET is implemented. P2.

## Punch list

### P0 (IGVC blockers / runaway risk)

1. **Hardware e-stop not wired into firmware.** Per IGVC AutoNav rules cited in the standards doc, "Vehicle E-Stops must be hardware based and not controlled through software." The firmware has no GPIO input for an e-stop signal, no latched-fault state machine, and no contactor-feedback path. The current implementation is a software-only `S` command + a 300 ms watchdog → both software paths. **Inspection failure on day 0 of the competition.** Fix: add `pinMode(ESTOP_PIN, INPUT_PULLUP)` in `setup()`, poll on every control tick, on rising edge enter a latched-fault state that zeroes setpoints AND sets `ctrl_mode = MODE_DUTY` (so Brake-idle engages), refuses to re-arm without a power-cycle or explicit reset command. In parallel, the hardware team needs the mushroom button + wireless e-stop wired to a contactor breaking motor power.

2. **Watchdog trip leaves `ctrl_mode = MODE_VELOCITY`.** `teensy_diff_drive.ino` lines 360–361 zero `cmd_rpm` and `cmd_duty` but don't change `ctrl_mode`. Means a host-loss event running velocity commands continues sending `setVelocity(_, 0)`, which is the exact failure mode the `S` command was rewritten to avoid (per the in-line comment at line 217–219 — "keeps the velocity PID running, which prevents the controller from entering idle state"). Wheels will stop via PID, but SparkMAX Brake-idle won't engage. On a slope, vehicle rolls. Fix is one line: `if (wdt_tripped) ctrl_mode = MODE_DUTY;` at line 360 or equivalent.

3. **No hardware MCU watchdog (RTWDOG).** If the firmware loop hangs (e.g. unbounded `while` somewhere), heartbeats stop, SparkMAX disables, but the firmware itself never reboots. RTWDOG with a 1–2 s timeout would catch this. Without it, a frozen firmware stays frozen until power cycle. Standards rule 9.

### P1 (embedded anti-patterns)

1. **Watchdog trip auto-rearms on next valid command** (teensy_diff_drive.ino lines 224, 252, 294). Once tripped, any `L<rpm>`/`R<rpm>`/`UL`/`UR`/`S` clears `wdt_tripped` and accepts new motion. Standards best-practice is "refuses to re-arm without explicit reset." If host crashes-and-resumes, motion resumes mid-stride. Fix: require explicit `S` to clear, or a separate `RESET` command.

2. **Missing `volatile` on ISR-shared `rx_count`** (teensy_diff_drive.ino line 99, teensy_diag.ino line 55). Compiler may cache the value in a register inside DIAG/summary printf, miss recent ISR increments. Standards rule 3 — listed in the anti-pattern top-10.

3. **Multi-byte ISR-shared reads not bracketed.** `meas_rpm` and `meas_pos` are `volatile float` (atomic at load level on M7) but read in pairs without `noInterrupts()`/`interrupts()` — DIAG and E-line can show a torn pair. Standards rule 4.

4. **`delay(5)` and `delay(50)` in hot paths** (teensy_diff_drive.ino lines 176, 258, 259). `tuneBoth()` sleeps 5 ms between left/right param writes; `BURN` sleeps 50 ms. Standards anti-pattern: "delay() anywhere on a control path. Blocks the loop; misses serial bytes, CAN frames, watchdog feeds." Replace with non-blocking state machine or only invoke when wheels at zero.

5. **Serial buffer overflow silent** (teensy_diff_drive.ino line 308; teensy_diag.ino line 177). Lines longer than 96 (or 128) bytes have characters silently dropped, then the leading fragment is parsed on next `\n`. Add `ERR overflow` and resync on next newline.

6. **`atof` on malformed `K` value writes 0** (teensy_diff_drive.ino lines 263–278). `KP<garbage>` silently sets kP=0 and replies `OK Kp=0.0`. Combined with #11 below (BURN unconditionally), can persist 0 to flash. Strict numeric parse + range check.

7. **`L` or `R` alone (no number) sets the wheel to 0** (teensy_diff_drive.ino lines 280–296). `atof("")` returns 0. A line truncated by SSH stall produces a half-stop, not an error. Reject empty-numeric.

8. **BURN issued unconditionally** (teensy_diff_drive.ino lines 257–261). No precondition (wheels at rest, gains-not-NaN), no two-step arm-fire, no debounce. Wears flash; can persist pathological state. Gate on `wheels_at_rest_for >= 1s` or require `ARM_BURN` then `BURN`.

9. **No range-check on PID gain values** before write (teensy_diff_drive.ino line 266 → 275). `KF99999` writes 99999 to flash. P1 — clamp to plausible range (e.g. kFF ∈ [0, 0.001], kP ∈ [0, 0.01]).

10. **No DIAG availableForWrite gating** (teensy_diff_drive.ino line 229; teensy_diag.ino line 124). The 50 Hz E-line is gated; DIAG and summary are not. If host stalls on TX, DIAG blocks. Add `if (Serial.availableForWrite() >= 256)` guard.

11. **`teensy_diag.ino` TX raw command parser fragile** (teensy_diag.ino lines 147–164). The outer `while (*p == ' ' && len < 8)` only iterates if current char is a space — the parsing of multi-byte payloads depends on exact whitespace. Easy to misuse. Rewrite to consume bytes until end-of-line or `len == 8`.

12. **`test.py raw "BURN"` / `test.py raw "KP<bad>"` not gated.** P1 — refuse `raw` commands matching `BURN` or `K*` without an explicit `--allow-flash` flag.

13. **`teensy_bridge.Teensy.stop()` swallows all exceptions** (line 57). If serial port is closed mid-stop, the SIGINT handler exits clean while motors potentially still commanded. P1 — at least `print(e, file=sys.stderr)` before swallowing.

14. **Phase 7 BURN sentinel overwrites tuned kP** (phase7_burn_verify.py line 52). Writes 0.00013579 to kP, BURNs it. No restore step. Operator must re-tune (or re-BURN the real kP) afterward. P1 — read current kP, write sentinel, BURN, prompt power-cycle, then restore kP and BURN again.

15. **No host-side watchdog** in `teensy_bridge.drain_for`. If the firmware goes silent (frozen, USB-CDC stuck), the host script keeps re-sending refresh commands forever. P1 — abort if no E-line received in N seconds.

### P2 (style)

- Heartbeat magic bytes unnamed (.ino lines 127, 130; diag lines 77, 79). Name `UNIVERSAL_HB_PAYLOAD[8]` with per-byte comments.
- STATUS_2 enable mask `0x04` deserves `STATUS_2_ENABLE_BIT = 0x0004`.
- `availableForWrite() >= 64` — 64 is USB-CDC packet size; name `CDC_PKT_SIZE`.
- `test.py` line 19 comment ("|d| <= 0.6 matches firmware MAX_DUTY") is wrong; MAX_DUTY is 0.30.
- Duplicate `Teensy` class (`teensy_bridge.py`, `test.py`); duplicate "send refresh, parse E-line" loop in 4 places.
- `run_duty` and `run_velocity` are near-identical (bridge lines 159–206); merge.
- `settle_frac=0.4` is fixed-fraction; use fixed-time settle (`settle_seconds=0.5`) for long runs.
- `M_PER_REV = 0.01994` in 2 scripts inlined in a 3rd. Hoist.
- Tuned PID gains in 3 places (CLAUDE.md, phase6c, actuator_params.yaml). Make yaml the source.
- No `__DATE__ __TIME__` in firmware boot banner.
- No `-Wall -Wextra -Wpedantic` on Arduino build (standards rule 10).
- No CI for `arduino-cli compile`.
- Phase 1 verdict thresholds undocumented.
- Phase 6 superseded by 6b; delete or rename.
- `teensy_diag.ino` is dead-post-bring-up; promote or delete.
- Stale `~/AVROS` path in BRING_UP.md line 348.
- No regression harness loading baseline CSVs.
- No `--dry-run` flag on phase scripts.
- Bring-up CSVs may not be in `.gitignore`.
- Phase 7 has no machine verification (needs firmware GET).
- Secondary heartbeat redundant per FINDINGS.md (TODO in BRING_UP.md line 396).
- STATUS_2 enable re-sent every 1 s — likely too often.
- `handleLine` borderline 90 lines with lambdas; extract `parseUduty`/`parseLR`/`parseK`.
- `E_LINE_RE` regex tightly coupled to firmware printf format — a `%.0f` change silently breaks all phase scripts.

## Positives

The firmware tree is one of the better-engineered components in the workspace.

- **Host-to-firmware watchdog implemented + tested.** 300 ms in standards window. Phase 6 dedicated test.
- **CAN heartbeat at 50 Hz, rollover-safe scheduling.** Standards rules 2, 5.
- **`volatile` discipline mostly correct** (meas_rpm, meas_pos, bus_voltage, got_enc). Gaps are real (rx_count, paired reads) but the awareness is there.
- **No `String`, no `malloc`/`new` after setup, no recursion, no goto.** Standards rules 13, 14; NASA P10 rule 1.
- **CAN via FIFO + ISR + queue, not polled.** Standards rule 15.
- **All functions ≤ 60 lines** (handleLine borderline at 90 with lambdas, extractable). P10 rule 4.
- **MAX_RPM and MAX_DUTY clamps in firmware** as defense-in-depth.
- **Brake idle mode used** (REV Hardware Client side). Standards § 9.
- **Motor inversion at SparkMAX, not in firmware.** Standards anti-pattern explicitly avoided.
- **Multiple upstream protocol bugs caught and fixed.** kFF=16 vs 17, velocity setpoint cls=0 idx=0 vs cls=1 idx=2, PARAMETER_WRITE cls=14 vs cls=48. All verified in bring-up.
- **PID tune is competition-grade.** Phase 6c: 99–100% tracking 500–3000 RPM, 0.24% trial-to-trial precision, 0.83% L/R sync delta.
- **`sendHeartbeats()` runs during watchdog trip** (line 364, outside the watchdog `if`). Correct controlled-stop semantics.
- **No `!Serial` guard** in hot loop — avoids the magenta-LED disable bug (commented at line 346 with reasoning).
- **Documented protocol with rationale.** Bug provenance preserved in CLAUDE.md + in-code comments.
- **Full bring-up trail captured** (CLAUDE.md + BRING_UP.md + FINDINGS.md + 9 phase scripts + timestamped CSVs). Reproducible; drift-detection ready.
- **Brown-out / shared-rail issue documented** (FINDINGS.md "Power rail ceiling identified"). Actionable.
- **Persistent journald enabled** on Jetson per CLAUDE.md — post-crash forensics possible.

The firmware is closer to ready for IGVC than the rest of the stack. The killer remaining work — hardware e-stop integration + the watchdog `ctrl_mode = MODE_VELOCITY` fix — is hours of firmware effort plus the wiring. Everything else is P1/P2 cleanup.
