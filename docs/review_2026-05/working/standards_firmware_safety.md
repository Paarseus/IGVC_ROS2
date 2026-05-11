# Reference Standards: Embedded Motor-Control Firmware & Real-Time Chassis Safety

*Working reference for the May 2026 IGVC_ROS2 firmware/control review. This document captures
what professional / upstream practice looks like for each topic — it does **not** review the
actual project source. Phase 2 of the review compares the real code against this baseline.*

Scope: Teensy 4.1 (ATmega-style Arduino C++) bridging USB-CDC serial to a CAN bus of REV
SparkMAX motor controllers, plus the ROS 2 actuator node that talks to it. The robot is a
diff-drive chassis at human-walking speeds (≤ 1.5 m/s) entered in IGVC AutoNav.

---

## 1. Arduino / Teensy C++ idioms vs anti-idioms

**`setup()` / `loop()` discipline.** Arduino's reference defines exactly two roles: `setup()`
runs once at boot for pin modes, peripheral init, and any allocation; `loop()` runs forever and
must be non-blocking so the runtime can service serial buffers, timers, and interrupts between
iterations
([Arduino docs — `loop()`](https://docs.arduino.cc/language-reference/en/structure/sketch/loop/),
[`setup()`](https://docs.arduino.cc/language-reference/en/structure/sketch/setup/)).

- Allocate buffers, open `Serial`/`CAN` peripherals, configure GPIO, push initial parameters in
  `setup()` only — never inside `loop()`.
- `loop()` should be a fast pass over a state machine; expensive one-shot work belongs in
  `setup()`.

**Where to put state — globals are pragmatic, not "wrong" in Arduino.** NASA's Power of 10 says
"declare data objects at the smallest possible level of scope"
([NASA/JPL — Power of 10](https://en.wikipedia.org/wiki/The_Power_of_10:_Rules_for_Developing_Safety-Critical_Code)).
On Arduino that translates to: `static` file-scope variables inside the `.ino`, not extern
globals across translation units. Avoid `extern` and avoid mutable globals that two ISRs could
both touch without synchronization.

**`String` is discouraged for fixed-size buffers.** `String` lives on the heap and reallocates
on `+=` / concatenation; in long-running embedded code it fragments the heap and eventually
fails to allocate even when free RAM exists
([cpp4arduino — Heap fragmentation](https://cpp4arduino.com/2018/11/06/what-is-heap-fragmentation.html),
[Majenko — The Evils of Arduino Strings](https://hackingmajenkoblog.wordpress.com/2016/02/04/the-evils-of-arduino-strings/),
[Arjen Stens — Arduino String alternative](https://arjenstens.com/arduino-string-class-alternative/)).
The accepted alternatives are:

- `char buf[N]` + `snprintf(buf, sizeof(buf), "...")` for outbound formatting.
- `strtok_r` / `strtol` / manual tokenizing for parsing.
- A fixed-capacity ring or linear buffer for incoming serial bytes, drained one full line at a
  time.

**`millis()` rollover safety.** `millis()` is `unsigned long` (32-bit) and wraps at ~49.7 days.
The correct comparison is *subtraction*, not absolute time:

```c
if (millis() - last_tick_ms >= PERIOD_MS) { ... last_tick_ms = millis(); }
```

Unsigned subtraction underflows cleanly across the wrap boundary, so the condition stays
correct
([Tech Explorations — millis rollover](https://techexplorations.com/guides/arduino/programming/millis-rollover/),
[Norwegian Creations — millis overflow](https://www.norwegiancreations.com/2018/10/arduino-tutorial-avoiding-the-overflow-issue-when-using-millis-and-micros/),
[Bald Engineer — millis() addition](https://www.baldengineer.com/arduino-millis-plus-addition-does-not-add-up.html)).
Storing `last + PERIOD` as an absolute deadline is the bug pattern — additions can desynchronize
across the rollover.

**`volatile` for ISR-shared state.** Any variable touched in an ISR and read from `loop()` (or
vice versa) must be `volatile` so the compiler doesn't cache it in a register. For multi-byte
types (`uint32_t`, `float`, structs) `volatile` is necessary but not sufficient — atomic access
also requires either disabling interrupts or using a lock-free pattern (e.g. double-buffer with
`__atomic_*` builtins). PJRC's official guidance is explicit: "Variables usually need to be
'volatile' types … temporarily disable interrupts, copy the shared variable, then re-enable
interrupts before using the copy"
([PJRC TimerOne reference](https://www.pjrc.com/teensy/td_libs_TimerOne.html)).

**`delay()` is a smell in control loops.** `delay()` is purely blocking — the CPU does nothing
else for the duration, so serial bytes can be lost, CAN frames can be missed, and the watchdog
can't be fed
([Arduino Forum — non-blocking delays](https://forum.arduino.cc/t/non-blocking-delays-and-timer-free/635670),
[Tech Explorations — Timer interrupts](https://techexplorations.com/blog/arduino/timer-interrupts-for-non-blocking-code-execution-the-arduino/)).
The standard non-blocking pattern is the BlinkWithoutDelay / `millis()` idiom above. `delay()`
is acceptable in `setup()` (e.g. waiting for a peripheral to power up) but should never appear
inside `loop()` on a control path.

**FlexCAN_T4 patterns** (the de-facto Teensy 4.x CAN library
[tonton81/FlexCAN_T4](https://github.com/tonton81/FlexCAN_T4)):

- Configure mailboxes explicitly in `setup()` (`setMB(MBn, RX, EXT)` for 29-bit IDs that
  SparkMAX uses; `TX` mailboxes for outbound).
- Enable per-mailbox interrupts (`enableMBInterrupt(MBn)`) and register a callback with
  `onReceive(MBn, cb)` — **the callback runs in ISR context**, so it must do the minimum
  (timestamp, copy frame into a queue, set a flag) and return.
- Call `myCan.events()` once per `loop()` iteration to dispatch queued frames to user
  callbacks.
- Use `setFIFOFilter(REJECT_ALL)` + per-ID accept filters so the MCU isn't woken for every
  frame on a busy bus.

---

## 2. Real-time control loop discipline

**Fixed-rate scheduler patterns.** The dominant pattern for safety-critical embedded loops is
Time-Triggered Cooperative (TTC) scheduling: each task has a fixed period; the super-loop checks
deadlines via `millis()` / `micros()` and runs the next due task
([Nahas et al. — Highly-predictable TTC](https://drive.uqu.edu.sa/_/mmnahas/files/Publications/Journals/11.pdf),
[Harvie — Latency and jitter in time-critical firmware](https://medium.com/@lanceharvieruntime/addressing-latency-and-jitter-in-time-critical-firmware-applications-b1a03172981a)).
The contract:

- Heartbeat task at 50 Hz (20 ms) — emit SparkMAX universal heartbeat.
- Setpoint refresh at the host's command rate or higher.
- Telemetry / DIAG at a slower rate (10 Hz typical) so it never preempts control.

**Acceptable jitter for a chassis at walking speed.** At 1.5 m/s, 10 ms of control-loop jitter
moves the vehicle 1.5 cm — well below sensor resolution and below any reasonable goal tolerance.
Targets:

- Heartbeat jitter: ≤ 2 ms (SparkMAX disables on > 100 ms gap, but > 50 ms gaps risk the LED
  going magenta intermittently).
- Setpoint command-to-CAN latency: ≤ 5 ms typical, ≤ 20 ms worst-case.
- Loop period nominal: 1–2 ms (Teensy 4.1 at 600 MHz can easily sustain a 1 kHz super-loop).

The relevant principle is "*the scheduler must be predictable, not fast*" — TTC schedulers
sacrifice average throughput for low jitter, which is the right trade for closed-loop motor
control
([ResearchGate — TTC implementation](https://www.researchgate.net/publication/322978779_Implementation_of_highly-predictable_time-triggered_cooperative_scheduler_using_simple_super_loop)).

**No `malloc` in steady state.** NASA Rule 3: "Do not use dynamic memory allocation after
initialization"
([Power of 10 — Wikipedia](https://en.wikipedia.org/wiki/The_Power_of_10:_Rules_for_Developing_Safety-Critical_Code),
[Holzmann — original P10 paper](https://spinroot.com/gerard/pdf/P10.pdf)). On Arduino this means
no `String`, no `new`, no `malloc`, no `std::vector::push_back` past `setup()`. Allocate
fixed-size buffers up front; reject input that doesn't fit rather than growing.

**ISR-driven I/O with mailboxes / queues is the right pattern.** For CAN, the receive path
should never be polled in a hot loop — the FlexCAN ISR copies the incoming frame into a small
ring buffer, sets a flag, and returns; the foreground drains the buffer at its scheduled tick.
This decouples CAN bus rate from loop rate and bounds worst-case latency
([FlexCAN_T4 README](https://github.com/tonton81/FlexCAN_T4)).

---

## 3. Serial protocol design

**Line-oriented ASCII vs binary framed.** Both are defensible — the trade-off is robustness vs
debuggability:

| | ASCII line-oriented (`L100 R100\n`) | Binary framed (COBS / SLIP / fixed-length) |
|---|---|---|
| Debug | trivial — `cat /dev/ttyACM0` | needs decoder / wireshark |
| Bandwidth | ~3-5x more bytes per command | tight |
| Robustness to single-byte error | partial line gets dropped | depends on framing + checksum |
| Resync | trivial (next `\n`) | trivial only with COBS-style delimiter |

For a 115200 baud control link carrying ≤ 50 commands/s, ASCII is fine and the debuggability is
worth the overhead. For ≥ 1 kHz telemetry or sensor fusion buses, binary framed wins.

**When COBS / SLIP framing is justified** ([Cheshire & Baker — COBS
paper](https://stuartcheshire.org/papers/COBSforToN.pdf), [Wikipedia
— Consistent Overhead Byte
Stuffing](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing),
[Pikokosan/PacketSerial Arduino library](https://github.com/Pikokosan/PacketSerial)):

- Binary payload that can contain the delimiter byte (e.g. raw IMU `float`s containing `0x0A`).
- Need guaranteed resync to the next packet on any single-byte glitch — COBS adds at most 1
  byte per 254, average 0.23 % overhead, and `0x00` is *guaranteed* not to appear inside a
  packet so the receiver can hard-resync on `0x00`.
- Fixed-overhead framing where bandwidth matters.

**Checksums.** Three tiers:
- **No checksum** — only acceptable if you fully trust the link (USB-CDC over a 30 cm cable in
  a clean enclosure). USB itself has CRC at the link layer.
- **XOR-8 / sum-8** — better than nothing, catches single-bit flips, fails on swapped bytes.
- **CRC-16 (CCITT, MODBUS)** — the embedded standard. MODBUS RTU's CRC-16/0xA001 is canonical
  for industrial serial; CRC-16-CCITT (0x1021, init 0xFFFF) is the alternative
  ([Wikipedia — CRC](https://en.wikipedia.org/wiki/Cyclic_redundancy_check),
  [SRecord — CRC-16-CCITT](https://srecord.sourceforge.net/crc16-ccitt.html)).

For an over-USB CDC link to a Teensy, a CRC is overkill *unless* you've seen corruption in
practice. For RS-485 / long cables / EMI environments (motor drivers nearby), CRC-16 is
mandatory.

**Bidirectional handshaking.** Every host command should produce *some* firmware response
within a bounded time, even if it's just `OK` or an echo of the parsed command. Silently
dropping a command is the worst failure mode — the host can't distinguish "command accepted" from
"command lost in noise" from "firmware crashed". Recommended pattern:

- Host sends `L100 R100\n`; firmware replies `OK\n` or `ERR <reason>\n` within < 10 ms.
- If host sees no reply for > 50 ms, retry once; if still no reply, declare the link dead and
  enter a safe state on the host side too.
- Periodic `D` / `DIAG` lines from firmware → host carry tx/rx counters, watchdog state,
  measured wheel speeds, faults. Host watches the counters move.

**Heartbeat from host to firmware.** Recommended: even with no new setpoint, the host sends a
"keep-alive" line every 50–100 ms. Absence of keep-alive *or* setpoint within the watchdog
window triggers a controlled stop on the firmware side (see § 4)
([memfault — firmware watchdog best
practices](https://interrupt.memfault.com/blog/firmware-watchdog-best-practices)).

---

## 4. Watchdog timers

Watchdog timers are layered: hardware watchdog → software watchdog → command-loss timeout.
Each protects against a different failure mode
([Ganssle — Great
Watchdogs](https://www.ganssle.com/watchdogs.htm),
[memfault — firmware watchdog best
practices](https://interrupt.memfault.com/blog/firmware-watchdog-best-practices),
[In Compliance — Robust watchdog
timers](https://incompliancemag.com/implementing-robust-watchdog-timers-for-embedded-systems/)).

**Host-to-firmware command-loss timeout.** This is the most important one for a chassis. If the
Jetson dies, the SSH session drops, the ROS 2 node crashes, or USB unplugs, the firmware *must*
notice and stop the motors. Recommended timeout window:

- **50 ms — tight, for high-rate (≥ 50 Hz) command streams.** Matches the diff-drive controller
  command rate.
- **100–200 ms — typical for ROS 2 / mid-rate command streams.** ros2_control's
  `diff_drive_controller` defaults `cmd_vel_timeout` to **0.5 s** as the upper bound for "stale"
  ([ROS 2 Control —
  diff_drive_controller](https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)).
- **> 500 ms is a smell** for a moving vehicle — the robot can travel half a meter at 1 m/s
  before stopping.

**Firmware-to-actuator heartbeat.** This is the *other* direction: the firmware must keep
sending the SparkMAX universal heartbeat or the motor controller will disable output on its own.
SparkMAX disables on a > 100 ms gap and shows magenta LED on its dedicated PWM input timeout
([REV — SPARK MAX Control
Interfaces](https://docs.revrobotics.com/brushless/spark-max/control-interfaces)). Standard
practice is 50 Hz (20 ms) so even with one missed beat the bus stays alive.

**What should happen on watchdog trip.**
- *Controlled deceleration* is preferred when the failure mode is "host went away" — keep
  sending heartbeat to the SparkMAX, ramp setpoint to zero with the slew limiter, then idle
  Brake (or Coast) once stopped. This avoids slamming the chassis to a halt with infinite jerk.
- *Immediate stop* (set duty 0, send `S`) is correct when an actual fault is detected — bus
  voltage out of range, encoder fault, parameter mismatch. Jack Ganssle's principle: "force all
  control outputs to safe states"
  ([Ganssle — Great Watchdogs](https://www.ganssle.com/watchdogs.htm)).

**Independent hardware watchdog as backstop.** Best practice from Ganssle and memfault: a
separate watchdog peripheral (or external supervisor IC) that resets the whole MCU if the
software-level watchdog never fires. Teensy's hardware WDT (RTWDOG / IWDG-equivalent on iMXRT)
should be enabled with a timeout of a few seconds — long enough that normal jitter never trips
it but short enough that a firmware lockup is detected
([Ganssle — Great
Watchdogs](https://www.ganssle.com/watchdogs.htm)).

---

## 5. E-stop / safety interlock standards

**Software e-stop is not enough.** The principle is bedrock in machinery safety: the e-stop
must physically remove power from the actuators, regardless of what the controller is doing
([ABB — How to implement an emergency stop, Cat
0](https://library.e.abb.com/public/19bcb95da6599951c1257dc8003d36b2/17359_TD-ESTOP_0_CONTACTOR_EN_3AUA0000172867_RevA_lowres.pdf),
[Industrial Monitor Direct — Safety relay e-stop
design](https://industrialmonitordirect.com/blogs/knowledgebase/safety-relay-output-wiring-proper-e-stop-circuit-design),
[Machinery Safety 101 — Stop
categories](https://machinerysafety101.com/2010/09/27/emergency-stop-categories/)).
"Best practice is to make E-stop as independent as possible … the worst design would have the
E-stop signal go into the controlling microcontroller, while the best design is where E-stop
removes power to the motors."

**Stop categories** ([IEC 60204-1 / ISO 13850](https://us.idec.com/RD/safety/law/iso-iec/iso13850),
[ISO 13849](https://en.wikipedia.org/wiki/ISO_13849)):
- **Category 0** — uncontrolled stop by immediate removal of power. Coast to halt.
- **Category 1** — controlled stop with power retained until standstill, then power removed.
- **Category 2** — controlled stop with power retained.

For a battery-powered ground vehicle, Category 0 with a dual-channel mechanical e-stop driving a
contactor between battery and motor drivers is the canonical design.

**Layered defense.**
1. Big red mushroom button on the rear of the vehicle (mandatory for IGVC, see below).
2. Wireless e-stop receiver (RC / 2.4 GHz / dedicated safety radio).
3. Either of (1) / (2) drives a safety relay (or pair, for redundancy) that opens a contactor
   between battery and SparkMAX power.
4. The same e-stop signal is also wired to a digital input on the firmware → firmware enters
   soft-stop / latched-fault state, refuses to re-arm without an explicit reset.
5. ROS 2 publishes a software e-stop topic that the firmware also honors — but this is
   *layered on top of* (1)–(4), not a replacement.

ISO 13849-1 specifies Performance Levels (PLa–PLe). For a generic e-stop the typical risk
assessment lands on **PLr = c (or SIL 1) minimum**, which means dual-channel monitored
contactors and force-guided contacts
([ISO 13849-1 overview](https://www.iso.org/standard/73481.html),
[Pilz — Performance Level basis](https://www.pilz.com/en-US/support/law-standards-norms/functional-safety/en-iso-13849-1)).

**IGVC AutoNav rules — exact requirements.** The IGVC competition rules are explicit
([IGVC 2024 rules PDF — search
result](https://www.google.com/search?q=igvc+2024+rules+e-stop)):

- **"Vehicle E-Stops must be hardware based and not controlled through software."** Software-only
  e-stop fails inspection.
- **Mechanical e-stop must be located on the center rear of the vehicle, between 2 ft and 4 ft
  high.**
- **A wireless e-stop is also required**, range checked at a minimum of **50 ft**
  (older revisions required 100 ft). During AutoNav and SD challenges the wireless e-stop is
  held by the judges.

The competition checks both mechanical placement and wireless range during inspection — a
software-only or out-of-spec mechanical e-stop is a disqualifier.

---

## 6. Slew rate / acceleration limits

**Why limit `du/dt` on (v, ω).** Step changes in commanded velocity ask the motor to draw
peak current immediately. That:
- Brown-outs the shared rail (see § 7).
- Snaps drivetrain backlash (chains, gearboxes, belts) and accelerates wear.
- Saturates the SparkMAX's velocity PID, causing overshoot and oscillation.
- Spikes IMU readings, confusing localization.

**The trapezoidal velocity profile pattern** ([MathWorks — Trapezoidal velocity
profile](https://www.mathworks.com/help/robotics/ug/design-a-trajectory-with-velocity-limits-using-a-trapezoidal-velocity-profile.html),
[CTRL ALT FTC — Motion
profiling](https://www.ctrlaltftc.com/advanced/motion-profiling),
[WPILib — Trapezoidal motion
profiles](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html),
[PMD Corp — Mathematics of motion
profiles](https://www.pmdcorp.com/resources/type/articles/get/mathematics-of-motion-control-profiles-article)):
three phases — accelerate at constant `a_max`, cruise at `v_max`, decelerate at constant
`-a_max`. The `du/dt` on velocity is bounded by `a_max`. Implementation in firmware / ROS:

```python
v_target = clamp(v_command, -v_max, v_max)
dv_max = a_max * dt
v_setpoint += clamp(v_target - v_setpoint, -dv_max, dv_max)
```

**Separate accel from decel limits.** Robotics convention treats braking as
asymmetric — a vehicle should be able to decelerate harder than it accelerates because
deceleration is the safety direction
([ROS 2 Control —
diff_drive_controller](https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
exposes `max_acceleration` *and* `max_deceleration` as independent parameters per axis).
Typical ratio: `decel_max ≈ 1.5–2 × accel_max`.

**Integrating with closed-loop velocity controllers.** The slew limiter must run *before* the
inner velocity PID on the SparkMAX, not after. The host sends a smoothed, slew-limited
setpoint; the SparkMAX PID then tracks that smooth ramp. If the host sends a step and lets the
PID smooth it, the PID's integrator winds up and overshoots.

**Caveat — trapezoidal has infinite jerk at corners.** Acceptable for chassis at walking speed;
high-precision systems use S-curves (jerk-limited) instead
([MDPI — Trapezoidal velocity profile
analysis](https://www.mdpi.com/1996-1073/12/7/1222)).

---

## 7. Brown-out / shared power rail issues

**The well-known problem.** Sharing a 12 V (or 24 V, or 48 V) rail between compute (Jetson,
router, sensors) and high-current motor drivers is the classical embedded-vehicle mistake. NEO
brushless motors can pull 100–200 A peak during stall or rapid acceleration; a buck converter
sized for the *average* compute load sags below the Jetson's brown-out threshold during the
transient, and the Jetson reboots
([Embedded — Brown-out
reset](https://www.embedded.com/brown-out-reset/),
[Silicon Labs AN0018.1 — Supply voltage
monitoring](https://www.silabs.com/documents/public/application-notes/an0018.1-efr32-efm32-series-1-supply-voltage-monitoring.pdf),
[Microchip — AVR Brown-out
detection](https://developerhelp.microchip.com/xwiki/bin/view/products/mcu-mpu/8-bit-avr/structure/bod/)).

**Why dedicated buses are standard.** Industry practice is two completely separate DC-DC
converters fed off the battery — one for compute, one for actuators. The compute converter is
sized for the steady-state load with comfortable headroom; the actuator converter passes through
to the motor drivers without touching anything else. Cross-talk is bounded to common-mode noise
on chassis ground.

**Capacitor sizing rules of thumb** ([Silicon Labs
AN0018.1](https://www.silabs.com/documents/public/application-notes/an0018.1-efr32-efm32-series-1-supply-voltage-monitoring.pdf)):
- **Bulk cap on motor rail:** ≥ 1000 µF per amp of average draw, electrolytic, low-ESR. For a
  pair of NEO motors: 4700–10000 µF.
- **Decoupling on Jetson rail:** 100 µF bulk + 10 µF ceramic + 100 nF ceramic at the regulator
  output, plus per-IC decoupling at the consumer.
- **TVS / snubber across motor leads:** clamps regen voltage spikes.

**Detecting brown-outs post-hoc.**
- `journalctl -k -b -1` shows the previous-boot kernel log; look for last messages before the
  reset.
- `last reboot` shows reboot history.
- `dmesg | grep -i 'reset\|brown\|undervolt\|throttl'` on the Jetson (Linux exposes
  `nvpmodel` / `tegrastats` for SoC-level undervoltage on Tegra).
- Persistent `journald` (`Storage=persistent` in `/etc/systemd/journald.conf`) is the
  prerequisite — the default volatile journal is wiped on reboot.
- For the firmware side, log bus voltage in DIAG telemetry; a voltage sag right before
  `last_command_age > timeout` is the smoking gun.

---

## 8. Bring-up / characterization scripts

**When one-shot Python scripts are appropriate.** Hardware bring-up is an iterative,
exploratory phase; one-shot scripts (e.g. `phase1_handspin.py` … `phase7_burn_verify.py`) are
the right tool to:

- Characterize an unknown motor / drivetrain (max RPM, friction asymmetry, kFF).
- Tune a PID one parameter at a time with the operator watching for instability.
- Persist parameters once and verify the persistence survives a power cycle.
- Generate a one-time calibration table.

**When they should graduate.** Once the hardware is characterized and the gains are persisted,
those scripts become *regression tests* — they answer "is the hardware still behaving the way
we measured at bring-up?" Two paths:

- **Promote to a test fixture.** Wrap the script in `pytest` or a `colcon test` runner; assert
  measured RPM is within ± 5 % of the recorded baseline; fail loudly on drift.
- **Delete (with a tag).** If the script can never be re-run safely (e.g. it intentionally
  stalls a wheel to characterize friction), delete it and reference the git tag in the
  characterization document. Don't leave dangerous scripts around for someone to run by accident.

**Calibration drift detection** is the high-value use case. At competition prep, run the same
phase scripts against the live vehicle and compare to the bring-up baseline. A 20 % drop in max
RPM is a chain wear / belt slip / SparkMAX gain change — easier to diagnose now than during a
run. The principle is "*calibration is data, not code*"; the bring-up scripts produce the data.

**Anti-pattern.** Scripts that mutate persistent SparkMAX flash state (PERSIST_PARAMETERS, CAN
ID changes) without clear naming and a confirmation prompt — a `phase6_tune_pid.py` that runs
`BURN` silently is a foot-gun.

---

## 9. SparkMAX-specific gotchas (FW 26.x)

- **Brake vs Coast idle mode** — selected via parameter `kIdleMode` (id 6); 0 = Coast, 1 =
  Brake. State is the half-bridge behavior when commanded duty is 0 or the controller is
  disabled. **For diff-drive chassis that need to hold position on a slope or stop quickly,
  Brake is the correct setting.** Coast is correct for inertia-coast applications
  (e.g. flywheels). Idle mode persists across firmware updates
  ([REV — SPARK MAX
  Parameters](https://docs.revrobotics.com/brushless/spark-max/parameters)).

- **"Motor Inverted" affects both output AND encoder sign.** Setting the inversion bit reverses
  PWM polarity *and* flips the encoder reading sign so the velocity PID stays in a consistent
  reference frame. Inverting in firmware (negating L_mps before sending) is *wrong* — the
  encoder feedback is then inverted relative to the command and the PID runs away. Always set
  inversion via REV Hardware Client on whichever motor is mirror-mounted
  ([REV — SPARK MAX
  Parameters](https://docs.revrobotics.com/brushless/spark-max/parameters)).

- **FW 26.x is not API-compatible with FW 25.x.** REV migrated the parameter ID layout, the
  status frame format, and several control APIs between major firmware versions. Mixing a 26.x
  controller with 25.x driver code (or vice versa) silently corrupts setpoints and parameter
  reads. Pin firmware versions across both motor controllers and document them in the project
  README
  ([REV — Device firmware
  changelogs](https://docs.revrobotics.com/brushless/revlib/device-firmware-changelogs)).

- **PERSIST_PARAMETERS (cls=63 idx=15) is required to survive power cycle.** Parameter writes
  go to RAM only; without an explicit `BURN` (PERSIST_PARAMETERS), the gains revert at the next
  power-up. The flash has finite write endurance, so don't `BURN` on every command — only after
  characterization
  ([REV — SPARK MAX
  Parameters](https://docs.revrobotics.com/brushless/spark-max/parameters)).

- **Universal heartbeat is mandatory** — without periodic 0x01011840 frames, FW 26.x disables
  output after ~100 ms (matches the PWM-input 60 ms timeout for parity)
  ([REV — Control
  interfaces](https://docs.revrobotics.com/brushless/spark-max/control-interfaces)).

---

## 10. Anti-patterns to flag in review

These are the top-10 list a reviewer should grep for first. Each maps to a concrete failure
mode the project has either seen or is one bug away from seeing.

- **`delay()` anywhere on a control path.** Blocks the loop; misses serial bytes, CAN frames,
  watchdog feeds
  ([Tech Explorations — Timer interrupts for non-blocking
  code](https://techexplorations.com/blog/arduino/timer-interrupts-for-non-blocking-code-execution-the-arduino/)).
- **No command-loss timeout.** Host crash → motors keep running. The single highest-severity
  defect class
  ([memfault — firmware watchdog best
  practices](https://interrupt.memfault.com/blog/firmware-watchdog-best-practices)).
- **Hard-coded magic numbers without comments.** `if (rpm > 2450)` with no clue what 2450
  represents — fails MISRA's maintainability rules
  ([Perforce — MISRA
  overview](https://www.perforce.com/resources/qac/misra-c-cpp)).
- **No e-stop path.** Software-only stop, or e-stop wired only to firmware GPIO with no
  contactor. IGVC inspection failure
  ([Industrial Monitor Direct — Safety relay e-stop
  design](https://industrialmonitordirect.com/blogs/knowledgebase/safety-relay-output-wiring-proper-e-stop-circuit-design)).
- **`String` concatenation in hot paths.** Heap fragmentation; eventually `String s = a + b;`
  returns empty
  ([cpp4arduino — Heap
  fragmentation](https://cpp4arduino.com/2018/11/06/what-is-heap-fragmentation.html)).
- **Missing `volatile` on ISR-shared variables.** Compiler caches the value in a register; the
  ISR's writes are invisible to `loop()`
  ([PJRC — TimerOne
  reference](https://www.pjrc.com/teensy/td_libs_TimerOne.html)).
- **No DIAG / heartbeat output from firmware.** Silent failure mode; host can't tell if a
  command was accepted or dropped.
- **Step-function setpoint changes (no slew limit).** Stresses the drivetrain, brown-outs the
  rail, saturates the PID
  ([CTRL ALT FTC — Motion
  profiling](https://www.ctrlaltftc.com/advanced/motion-profiling)).
- **Shared compute / motor power rail with no current monitoring.** Brown-out on every hard
  acceleration; reboots the Jetson mid-run
  ([Embedded — Brown-out
  reset](https://www.embedded.com/brown-out-reset/)).
- **Writing SparkMAX flash (`BURN`) on every parameter update.** Wears out flash; risks
  corrupting params if power cuts mid-write
  ([REV — SPARK MAX
  Parameters](https://docs.revrobotics.com/brushless/spark-max/parameters)).
- **`millis() + period` arithmetic for next-deadline tracking.** Breaks at the 49.7-day rollover
  ([Bald Engineer — millis() addition
  pitfall](https://www.baldengineer.com/arduino-millis-plus-addition-does-not-add-up.html)).
- **Recursion or `goto` in firmware.** Violates NASA Power of 10 rule 1 — unbounded stack
  depth, unanalyzable control flow
  ([Power of 10 — Wikipedia](https://en.wikipedia.org/wiki/The_Power_of_10:_Rules_for_Developing_Safety-Critical_Code)).
- **Motor inversion done in firmware code (negating setpoint) instead of in SparkMAX
  parameter.** Encoder sign and command sign disagree → PID runs away
  ([REV — SPARK MAX
  Parameters](https://docs.revrobotics.com/brushless/spark-max/parameters)).
- **`malloc` / `new` after `setup()`.** Heap fragmentation, non-deterministic latency
  ([Holzmann — P10 paper](https://spinroot.com/gerard/pdf/P10.pdf)).

---

## Quick checklist for reviewers

A boolean checklist for the firmware + actuator-node review. Each item is one yes/no
question; "yes" is the desired answer.

1. ☐ Does `loop()` complete in bounded time (no `delay()`, no blocking I/O)?
2. ☐ Are time-elapsed checks written as `millis() - start >= period` (subtraction, not
   addition)?
3. ☐ Are all variables shared between ISRs and `loop()` declared `volatile`?
4. ☐ Are multi-byte ISR-shared reads guarded by `noInterrupts()` / `interrupts()` (or
   equivalent)?
5. ☐ Is there a fixed-rate (≥ 20 Hz) heartbeat from firmware to SparkMAX universal heartbeat
   ID 0x01011840?
6. ☐ Is there a host-to-firmware command-loss timeout in the 50–500 ms range that triggers a
   controlled stop?
7. ☐ Does the firmware emit a periodic DIAG / status line so the host can detect silent
   firmware faults?
8. ☐ Does every host command produce a bounded-time firmware response (`OK` / `ERR <reason>`)?
9. ☐ Is the hardware MCU watchdog (RTWDOG / iMXRT) enabled with a multi-second timeout as a
   backstop?
10. ☐ Is the e-stop hardware-based (mushroom button → contactor → motor power), with a
    parallel firmware GPIO path?
11. ☐ Does the e-stop hardware path satisfy IGVC inspection (centered rear, 2–4 ft high) and
    is there a wireless e-stop with ≥ 50 ft range?
12. ☐ Are slew-rate (acceleration) limits applied to commanded velocity *before* the inner
    SparkMAX velocity PID, with separate accel and decel caps?
13. ☐ Is `String` absent from hot paths (use `char[]` + `snprintf` instead)?
14. ☐ Is dynamic allocation (`malloc` / `new` / `String +=`) absent after `setup()`?
15. ☐ Are CAN frames received via FlexCAN_T4 mailbox + ISR + queue, not polled in `loop()`?
16. ☐ Are SparkMAX idle mode (Brake), motor inversion, and PID gains persisted via
    PERSIST_PARAMETERS (cls=63 idx=15) and is the firmware version pinned to a known one?
17. ☐ Is the motor-rail power separated from compute-rail power (or has the brown-out been
    proven absent under worst-case load)?
18. ☐ Is `journald` configured with `Storage=persistent` so post-crash forensics are possible?
19. ☐ Are bring-up / characterization scripts either gated as regression tests or deleted (no
    half-promoted dead Python in `firmware/`)?
20. ☐ Are magic numbers in firmware named / commented (max RPM, gear ratio, track gauge)?
21. ☐ Is there at least one assertion / range-check per non-trivial function (NASA P10 rule
    5)?
22. ☐ Are functions ≤ 60 lines (NASA P10 rule 4) and free of `goto` / unbounded recursion
    (rule 1)?
23. ☐ Is the code compiled with `-Wall -Wextra -Wpedantic` and clean of warnings (NASA P10
    rule 10)?
24. ☐ Is the host-side stack also defensive — does the actuator node clamp setpoints, time
    out on stale `cmd_vel`, and handle serial reconnect gracefully?
25. ☐ Is the e-stop button physically reachable from outside the vehicle while it is in
    motion (i.e. a person walking alongside can hit it without leaning over)?
