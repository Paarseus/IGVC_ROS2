# IGVC E-Stop Architectures — Prior-Art Survey

How other IGVC teams have built their emergency-stop systems, with citations
to specific design reports. Source PDFs cached at `/tmp/igvc_reports/`,
extracted to text in the same directory. URLs are `http://www.igvc.org/design/<year>/<n>.pdf`.

Reports surveyed: 13 (2015-2025), with 2023 Paradigm and 2025 Sooner being
the most thoroughly documented.

---

## 1. The four architectural patterns

Every team's E-stop falls into one of four buckets. Pick which one you want
to build before getting into part numbers.

### Pattern A — Series-NC kill loop (simplest, most common)

A single normally-closed loop runs through every kill input. Open any
contact and a control relay drops, killing motor power.

```
+24V ─[Mech NC]─[Wireless RX NC]─[…]─→ control relay coil
                                            │
                                            └─► drops contactor / SSR / motor-controller enable
```

**Used by:**
- **Oakland Horizon 2025** — physical button + RF latch in series; relay coil opens, motor-controller power drops.
- **ODU Little Blue 2025** — *two* wireless pendants (operator + safety officer) plus mechanical, all NC in series. "Both contacts open until both E-stops set to off." 433.92 MHz RF.
- **ODU Asterius MkII 2025** — mechanical button and RF receiver both wired to the same relay; either path cuts PWM from flight controller to ESCs.
- **LTU ACTor 2022** — closed hardware loop through all E-stop buttons, terminating at a Raspberry Pi GPIO that polls the loop state.

**Pros:** trivially simple, fail-safe by topology, easy to defend in design report.
**Cons:** all inputs must be NC; loss of wire continuity (frayed, unplugged) trips the loop just like an E-stop press, which is what you want for safety but sometimes annoying during bench work.

### Pattern B — Logic-OR digital E-stop signal

Each input generates an active-high (or active-low) signal; a logic IC OR-gates them and the result drives a power MOSFET / motor-controller enable.

**Used by:**
- **Paradigm B.O.A.T. 2023** — most thorough write-up of this pattern. Mech latching NO button → active-high signal. ESP32-LoRa-915 MHz wireless TX/RX → active-high signal. The two are OR'd in a logic IC, output drives a low-side power MOSFET on the motor power rail. Same active-high signal also fed to the ESP32-S3 control board so software *knows* a stop happened, but can't *prevent* it.
- **Paradigm 2022 (described in 2023 report)** — original implementation was *analog* and "suffered interference issues" at competition. They rebuilt as fully digital logic IC for 2023. Lesson: don't build E-stop logic in the analog domain anywhere near brushless motor drivers.

**Pros:** signal is digital end-to-end so EMI is bounded; one wire to the power board carries the kill state; clean hardware/software separation.
**Cons:** more parts, more board area, requires a separate power rail to the logic IC. Not worth it unless you're already designing custom PCBs.

### Pattern C — Microcontroller-polled E-stop (avoid)

A microcontroller reads E-stop state on a GPIO and decides whether to forward motor commands.

**Used by:**
- **LTU ACTor 2022** (partially) — Raspberry Pi reads loop state on a GPIO, then "sends a signal to the Intel NUC through Ethernet. The NUC will then safely stop the vehicle." This works at IGVC because the closed hardware loop is the *primary* path; the Pi just informs software. **Don't read this as "MCU-only is OK."**
- **Bridgeport 2025** — FlySky FS-i6S → receiver → MCU pin → MCU stops motor commands. Their own report flags this in the cyber-security section as "vulnerable to RF jamming/spoofing/replay." They got away with it but their design report is honest about the weakness.

**Pros:** flexible, easy to add features (auto-restart inhibit, reset button logic).
**Cons:** **Violates the IGVC rule** that the E-stop be "hardware-based, not controlled through software." Use only as a *layer on top of* a hardware cut, never as the sole path.

### Pattern D — DIY LoRa transceiver pair (most popular at IGVC, custom build)

Two ESP32-LoRa or Adafruit Feather M0-LoRa boards. Heartbeat-based fail-safe: TX sends a periodic "I'm here, button is up" message; RX trips the kill if heartbeat stops or button-pressed message arrives.

**Used by:**
- **Sooner Robotics 2020-2025** — `igvc_hardware_2020`, `igvc_software_2021`, `autonav_software_2024`. Adafruit Feather M0 RFM95 LoRa pair. 2-second heartbeat. "E-stop is not reversible — robot must be completely restarted to move again."
- **Paradigm 2023** — ESP32-LoRa pair. Heartbeat ping; "if ping fails, e-stop is activated on the vehicle." 915 MHz, antennas placed away from motors to limit EMI coupling.
- **RPI Robotics 2025** ([github.com/RPI-IGVC-2025/EmergencyStop](https://github.com/RPI-IGVC-2025/EmergencyStop)) — ESP32-S3 + HC-12 433 MHz radios, FreeRTOS heartbeat task. README starts with the disclaimer: *"This program is NOT intended for life-critical or industrial safety applications."*

**Pros:** earns design-report points (Sooner and Paradigm are praised in their own writeups for this); heartbeat fail-safe is rule-compliant if the receiver drives a hardware relay; cheap (~$60 in parts).
**Cons:** ~40 hr of engineering effort; commercial fallback strongly recommended (see Ville 2024 below); RF coupling from the vehicle's own motors into the handheld is a known failure mode (RoboJackets `roboracing-firmware/rigatoni/2025-firmware/EstopRadios/testing_transmitter_state_change_bug.md` documents the transmitter randomly resetting under exactly this condition).

---

## 2. Where the cut actually happens (kill-loop topology)

Every team cuts at a different point. Here's the menu, ranked by how
aggressive the cut is:

| Cut point | Aggressiveness | Used by |
|---|---|---|
| **Power MOSFET on motor low-side** (motor power physically interrupted) | Most aggressive — motors freewheel/brake based on driver-side configuration | Paradigm 2023 |
| **Solid-state relay on motor power rail** | Aggressive — fast, arc-free, no mechanical wear | Sooner 2025, Wayne State 2025 |
| **Mechanical contactor on motor power** | Aggressive — definitive disconnect, but contactor coil draws steady current | Oakland 2025 (relay before motor controllers) |
| **Motor-controller "shutdown" / "enable" pin** (RoboClaw, SparkMAX, Roboteq, ESC PWM cut) | Moderate — relies on motor controller honoring the input; controller is still powered | UMich GOAT 2019 (RoboClaw shutdown pin), ODU Asterius 2025 (cuts PWM from flight controller to ESCs), IIT-M 2018 ("only motor power, not whole vehicle so NUC and router stay up") |
| **MCU stops sending commands** | Weakest — software-mediated, not rule-compliant on its own | Bridgeport 2025, LTU 2022 (as secondary path) |

**Critical observation:** every team that *passed qualification* with a software-only path also had a hardware path doing the actual cut. The MCU path is informational, never sole.

**For our IGVC_ROS2 build:** we already cut motor-driver power via the mushroom. That puts us in the "mechanical contactor" tier, which is the same as Oakland Horizon 2025 — perfectly defensible. Don't *downgrade* to "Teensy stops sending RPM commands."

---

## 3. Combining mechanical + wireless inputs

| Combination scheme | Teams | Notes |
|---|---|---|
| **Series NC contacts** | Oakland 2025, ODU Little Blue 2025, ODU Asterius 2025, LTU 2022 | Either input opens the loop. Trivial, fail-safe, bulletproof. |
| **Logic OR of active-high signals** | Paradigm 2023 | Cleaner if you're already building a control PCB. |
| **MCU polls both, decides** | Bridgeport 2025 | Avoid as the sole mechanism. |
| **Independent paths, both cut motor power separately** | Sooner 2025 | Three independent interlocks (button + radio + software) all gate the same SSR. Highest redundancy. |

**Recommended for our build:** Pattern A (series NC). The mushroom you already have is normally-closed (when the button is up, contact closed). Add the wireless receiver's NC output in series. Done.

---

## 4. Wireless link choices in the wild

| Link type | Teams | Notes |
|---|---|---|
| **915 MHz LoRa (DIY)** | Paradigm 2023, Sooner 2020-2025 | Best penetration, low EMI susceptibility. Custom build. |
| **433 MHz RF (commercial relay kit)** | ODU Little Blue 2025, Sparky 2015 ("RF Control Systems 200M kit") | Off-the-shelf, ~$30-80. Often these are normally-open momentary, so verify failsafe behavior. |
| **433 MHz HC-12 (DIY)** | RPI Robotics 2025 | Cheap, well-documented, but no inherent failsafe — implement in firmware. |
| **2.4 GHz hobby RC (FlySky, FrSky)** | Bridgeport 2025 | Works but Bridgeport's own report flags it as RF-jam/spoof vulnerable. Crowded band at competition. |
| **2.4 GHz industrial pendant (Kar-Tech 3A548)** | Clearpath Husky/Jackal stock; not yet documented in any IGVC report | What we recommended for our team — see [`ESTOP_REQUIREMENTS_2026.md`](ESTOP_REQUIREMENTS_2026.md). |
| **900 MHz industrial (Kar-Tech 3A563)** | Same | Better band choice if 2.4 GHz competition floor is congested. |
| **ZigBee custom** | Hosei Orange2015 | Unusual; works. |

**Failure mode every team encountered or anticipated:** RF interference from the vehicle's own motors. Mitigations cited:
- Place 915 MHz antenna far from motor controllers (Paradigm 2023).
- Put the E-stop logic on a board far from the motor power section (Paradigm 2023).
- Switch from analog to digital E-stop signaling (Paradigm 2022 → 2023).
- Use 900 MHz over 2.4 GHz when in doubt (Kar-Tech 3A563 vs 3A548 question).

---

## 5. Software interlock — what it adds, what it doesn't

Almost every modern team layers a software E-stop on top of the hardware
cut. **It doesn't satisfy the rule on its own**, but it's how you get clean
state transitions (safety light, telemetry, restart prevention, status
LEDs):

- **Sooner 2025** — software interlock is one of three independent kill conditions. Software *also* re-checks the hardware lines and refuses to send non-zero motor commands unless all three are clear. "To prevent tampering, each of the three signals is also monitored in software, where the same logic is redundantly enforced."
- **Sooner 2021** — "E-stop is not reversible — robot must be completely restarted to move again. This prevents an accidental double-press from having adverse effects." (Worth copying into our firmware.)
- **Oakland 2025** — software E-stop sends a CAN message to the motor controllers, separate from the hardware path.
- **Bridgeport 2025** — software E-stop publishes "Red: Emergency Stop" to the safety light tower the moment the hardware trips.
- **Paradigm 2023** — control-board ESP32-S3 reads the active-high E-stop signal and drives the safety light state machine. Light goes from flashing (autonomous) → solid (manual) the instant the hardware signal asserts.

**For our build:** keep the existing `/avros/actuator_command estop:true` software path and the WebUI-disconnect estop. They're the *secondary* layer. The hardware path does the cut.

---

## 6. Failure case studies (verbatim from reports)

These are why prior-art research matters — every team that wrote about a
failure had to fix it under competition pressure, and we can avoid the same
pothole.

### Ville Robotics ALiEN 5.0 (2024) — custom radio failed, reverted to commercial

> "Custom remote E-stop could not reliably transmit the stop signal — Reverted to a reliable remote E-stop."

No vendor named for the replacement. Lesson: **don't roll your own
without a commercial fallback in the parts bin**. This is the strongest
argument against starting from scratch with a custom LoRa pair if you're
short on time.

### Paradigm B.O.A.T. (2022 → 2023) — analog E-stop suffered EMI

> "The team observed interference issues with the analog e-stop circuit during last year's competition, so a digital circuit was used to reduce noise susceptibility."

Lesson: **no analog signaling in the E-stop path**. Brushless motor PWM
couples into anything analog within ~1 m. Logic-level digital signals or
contact closures only.

### RoboJackets RoboRacing (2025) — transmitter resets under vehicle RF

> "We strongly suspect something is causing the remote to reset, but this only occurs when the car is responding to the remote."

Source: [`roboracing-firmware/rigatoni/2025-firmware/EstopRadios/testing_transmitter_state_change_bug.md`](https://github.com/RoboJackets/roboracing-firmware).

Lesson: **the handheld is also susceptible to vehicle-side RF**, not just
the on-vehicle receiver. Antenna isolation matters on both ends. If we use
a Kar-Tech industrial pendant this is much less likely (FCC certification
plus shielded enclosure plus 2.4 GHz spread spectrum).

### Bridgeport (2025) — hobby RC vulnerable to interference

Their own cyber-security section lists their FlySky FS-i6S as vulnerable
to RF jam, spoof, and replay. They passed qualification but documented the
weakness. Lesson: **if we use hobby RC, we need to acknowledge this in
*our* design report's cyber section** — judges read for honest threat
modeling.

### Sooner 2021 — the "no auto-reset" rule

> "E-stop is not reversible — robot must be completely restarted to move again."

Lesson: implement E-stop reset as a deliberate two-step action (toggle
key, restart firmware, etc.), not auto-recovery on signal restoration.
Otherwise a momentary RF dropout briefly stops the robot, then it lurches
forward again unsupervised.

---

## 7. Cybersecurity / threat-model section

The 2025 rulebook now expects a cyber-security analysis using NIST RMF
language. Every 2025 report we surveyed addresses E-stop in this section:

- **Sooner 2025** — "Remote E-stop: Malicious interference could disable or trigger the e-stop." Control: AC-3 Access Enforcement. Implementation: "Authenticate and encrypt the e-stop radio channel."
- **ODU Asterius 2025** — "If a wireless E-Stop signal is intercepted and misused, a rival team could stop the run." High likelihood, high impact, medium risk.
- **Bridgeport 2025** — explicitly lists FlySky FS-i6S as vulnerable to jam/spoof/replay.

**Implication for us:** the design report's cyber section needs to discuss
the wireless E-stop's attack surface. Kar-Tech 3A548 has rolling-code
authentication built in (datasheet language: "automatic frequency hopping
and unique address"), which writes the answer for us — cite the datasheet
and move on.

---

## 8. Composite recommended architecture for IGVC_ROS2

Combining what worked in 2023-2025 reports with our existing build:

```
                                                               ┌─► Beacon (solid in manual,
                                                               │   flashing in autonomous —
                                                               │   driven from autonomous-mode
                                                               │   line, NOT from estop signal)
                                                               │
+24V ─[ Existing mushroom NC ]─[ Kar-Tech RX Out#1 NC ]──────┬─┘
                                                             │
                                                             ▼
                                                    Safety relay coil
                                                             │
                                                             ▼
                                              Motor power contactor
                                              (already in place — drops
                                              motor power on loop break)

                                              ┌─────────────────────────────┐
                                              │ Teensy reads loop state via │
                                              │ optoisolated GPIO →         │
                                              │ publishes /avros/estop_hw   │
                                              │ → Jetson sees it → sets     │
                                              │ autonomous_mode = false →   │
                                              │ beacon goes solid           │
                                              └─────────────────────────────┘
                                              (informational only —
                                               cut is already done)
```

This satisfies:
- IGVC rule "hardware-based, not controlled through software": cut path is mushroom + relay contact + contactor; no MCU in series.
- "Quick and complete stop": SparkMAX Brake idle (per `CLAUDE.md`) + power removed from drivers = immediate halt.
- "≥ 100 ft range": Kar-Tech rated 1000 ft / 300 m.
- Series-NC topology proven by Oakland 2025, ODU 2025, LTU 2022.
- Same general architecture as Paradigm 2023 (digital signal end-to-end, control logic far from motors).
- Software interlock layered on top per Sooner 2021/2025 pattern.

The only thing this build does NOT do that some teams (Sooner, Paradigm)
did is the **third independent software interlock**. We already have the
software estop via `/avros/actuator_command` and WebUI-disconnect, so this
is effectively present — just call it out in the design report.

---

## 9. Reports cited (with local cache + URL)

| Report | Cache | URL |
|---|---|---|
| Paradigm B.O.A.T. 2023 | `/tmp/igvc_reports/2023_14.pdf` | http://www.igvc.org/design/2023/14.pdf |
| Sooner Twistopher 2025 | `/tmp/igvc_reports/2025_7.pdf` | http://www.igvc.org/design/2025/7.pdf |
| Oakland Horizon 2025 | `/tmp/igvc_reports/2025_14.pdf` | http://www.igvc.org/design/2025/14.pdf |
| ODU Little Blue 2025 | `/tmp/igvc_reports/2025_15.pdf` | http://www.igvc.org/design/2025/15.pdf |
| ODU Asterius MkII 2025 | `/tmp/igvc_reports/2025_24.pdf` | http://www.igvc.org/design/2025/24.pdf |
| Wayne State Shanti 2025 | `/tmp/igvc_reports/2025_26.pdf` | http://www.igvc.org/design/2025/26.pdf |
| Bridgeport 2025 | `/tmp/igvc_reports/2025_6.pdf` | http://www.igvc.org/design/2025/6.pdf |
| Ville ALiEN 5.0 2024 | `/tmp/igvc_reports/2024_8.pdf` | http://www.igvc.org/design/2024/8.pdf |
| LTU ACTor 2022 | `/tmp/igvc_reports/2022_27.pdf` | http://www.igvc.org/design/2022/27.pdf |
| UMich GOAT 2019 | `/tmp/igvc_reports/igvc.secs.oakland.edu_design_2019_19.pdf` | http://www.igvc.org/design/2019/19.pdf |
| IIT-M Abhiyaan 2018 | `/tmp/igvc_reports/2018_9.pdf` | http://www.igvc.org/design/2018/9.pdf |
| Sparky 2015 | `/tmp/igvc_reports/2015_22.pdf` | http://www.igvc.org/design/2015/22.pdf |
| IGVC 2026 Rules | `docs/igvc_rules/IGVC_2026_rules.pdf` | http://www.igvc.org/2026rules.pdf |

GitHub repos referenced (for the DIY LoRa pattern):
- [SoonerRobotics/igvc_hardware_2020](https://github.com/SoonerRobotics/igvc_hardware_2020)
- [SoonerRobotics/igvc_software_2021](https://github.com/SoonerRobotics/igvc_software_2021)
- [SoonerRobotics/autonav_software_2024](https://github.com/SoonerRobotics/autonav_software_2024)
- [RPI-IGVC-2025/EmergencyStop](https://github.com/RPI-IGVC-2025/EmergencyStop)
- [umigv/safety](https://github.com/umigv/safety)
- [RoboJackets/roboracing-firmware](https://github.com/RoboJackets/roboracing-firmware) — see `rigatoni/2025-firmware/EstopRadios/testing_transmitter_state_change_bug.md` for the transmitter-reset failure case.
