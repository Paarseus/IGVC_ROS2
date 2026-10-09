# IGVC 2026 Emergency Stop Requirements — Deep-Read Notes

Source: `IGVC_2026_rules.pdf` (61 pp, dated 2025-06-30, downloaded from
http://www.igvc.org/2026rules.pdf on 2026-05-02). Local extract:
`docs/igvc_rules/IGVC_2026_rules.txt`. All quoted lines reference that text file.

The rulebook does **not** put E-Stop in a single dedicated chapter — the
requirements are spread across the vehicle-specifications list (§I.2),
qualification check (§I.4 / Table 1), the run-procedure rules (§II.3), the
traffic-violation table (§II.4), and the design-report rubric (§V Safety).
This doc consolidates everything in one place.

---

## 1. Two independent E-Stops are required

The vehicle must have **both** a mechanical (push-button) E-Stop and a wireless
E-Stop. Both must be **hardware-based and not controlled through software**, and
**both must bring the vehicle to a "quick and complete stop"** when activated
(rules ll. 166-175).

> "Vehicle E-stops must be hardware based and not controlled through software.
> Activating the E-Stop must bring the vehicle to a quick and complete stop."
> — repeated for the mechanical E-stop (l. 169-171) **and** the wireless E-stop
> (l. 173-174).

Implication for our stack: cutting `/cmd_vel`, sending an `ActuatorCommand`
with `estop: true`, or any pure-software path **does not satisfy the rule by
itself**. The motor power (or motor-controller enable) must drop on a hardware
signal that does not pass through the Jetson, ROS2, or the Teensy firmware
logic. Software E-stop can still exist (and we already have it via the WebUI
disconnect path), but it is layered *on top* of the hardware cutoff, not in
place of it.

## 2. Mechanical E-Stop — physical & electrical specs

From rules ll. 166-171 and the Qualification checklist ll. 202-204, 472-475:

| Requirement | Spec |
|---|---|
| Type | Push-to-stop button (latching) |
| Color | **Red** |
| Diameter | **≥ 1 inch** |
| Location | **Center rear** of the vehicle |
| Mounting height | **≥ 2 ft and ≤ 4 ft** above ground |
| Identification | "Easy to identify and activate safely, even if the vehicle is moving" |
| Implementation | **Hardware-based, not software-controlled** |
| Effect | "Quick and complete stop" |

The judges physically inspect the button position with a tape measure during
Qualification — the 2 ft / 4 ft band is checked, not waived.

"Easy to activate safely even if moving" is a real ergonomic requirement.
Mushroom-head 22 mm or 40 mm panel-mount buttons (e.g. IDEC HW1B-V4F01R,
Schneider XB4BS842, or any large red NC mushroom) on a rear-facing plate at
roughly hip height meet this cleanly.

## 3. Wireless E-Stop — performance spec

From rules ll. 172-175, 205-207, 476-477:

| Requirement | Spec |
|---|---|
| Range | **≥ 100 feet** (≈ 30 m) — verified at Qualification |
| Implementation | **Hardware-based, not software-controlled** |
| Effect | "Quick and complete stop" |
| Antenna | Allowed to exceed the 6 ft height limit ("excluding emergency stop antenna" — l. 153, l. 471) |
| Custody during runs | **A judge holds the wireless E-Stop transmitter during AutoNav and Self-Drive runs** (l. 175, l. 206) |

Practical consequence of the "judge holds the remote" rule: **whatever
transmitter you hand the judge needs to be self-explanatory and reliable**.
A single big red button on a fob, or a clear-cased industrial pendant, is the
norm. Anything that requires a startup sequence, charging, or pairing on the
day will get you E-stopped or DQ'd at Qualification.

Hardware-based means the receiver's output drives a relay/contactor directly;
the Jetson is not in the loop. Common implementations in past IGVC entries:
- A licence-free ISM-band industrial e-stop pendant (e.g. Ahouse / Yueqing /
  Telecrane / Autonics 2.4 GHz or 433 MHz pendant) wired to a normally-energized
  safety relay — pendant loses signal or button pressed → relay drops →
  motor-controller enable line drops.
- A radio-controlled relay receiver (e.g. Linx, Inovonics) on a dedicated
  channel, again driving the motor enable.

## 4. What "quick and complete stop" means in practice

The rulebook does not give a numeric stopping-distance bound, but the run-rule
section (§II.4 Traffic Violations) makes the consequences clear:
- Judge's-Choice E-Stop and Student's-Choice E-Stop are recorded in the
  scoring table — overall distance is measured "from the starting line to the
  front of the vehicle … if stopped, crossed the boundary outside edge"
  (ll. 375-376).
- Any boundary crossing, crash, payload loss, or one-minute idle is itself
  enforced via E-Stop.

So the E-stop both protects bystanders and is the formal mechanism by which
runs end. From a system standpoint:
- The cut must be deterministic and immediate (don't pass through a brake-ramp
  or a slew-rate limiter on the safety path).
- The vehicle should not coast — IGVC interprets "quick and complete stop" as
  immediate halt, which on our diff-drive means dropping into Brake idle on
  the SparkMAXes (we already use Brake idle, see CLAUDE.md "Diff-Drive
  Parameters: Idle mode: Brake").

## 5. Safety light is part of the E-Stop story

Not the E-Stop itself, but bundled with it in the rules (ll. 176-179, 208-209,
478-481):

- Solid indicator light **whenever the vehicle is powered on**.
- Light goes **solid → flashing** as soon as the vehicle enters autonomous mode.
- Light goes **flashing → solid** as soon as the vehicle exits autonomous mode
  (including after an E-stop).

Judges verify this transition during Qualification. So the autonomous-mode
flag the safety light reads from must transition to "off" when *either* E-Stop
is triggered.

## 6. Design Report tie-in

The Design Report rubric (§V "5. Safety", ll. 1576-1583) explicitly asks the
team to:
- Describe safety aspects while transported, parked, charging, **and operating**.
- Specifically describe both **mechanical and wireless ESTOP systems**.
- List safety **requirements**, then compare each requirement's target value
  to the measured actual value.

The Safety section is worth **100 points** in three different scoring tables:
- Design Report scoring (l. 1671) — 100 pts
- Oral presentation scoring (l. 1716) — 100 pts
- Vehicle examination scoring (l. 1814) — 100 pts (sharp edges, exposed
  belts/chains, loose connections — physical inspection)

So writing this up well isn't optional — it's three 100-point line items.

## 7. Gap analysis vs. our current build

Pulling against `CLAUDE.md`:

| Requirement | Current state | Gap |
|---|---|---|
| Mechanical E-Stop button, red, ≥ 1", center-rear, 2-4 ft | Not described in CLAUDE.md / repo | **Need to confirm physical install** — button + bracket, photo for design report |
| Mechanical E-Stop hardware-cuts motors (not software) | Not described | **Likely missing** — current `actuator_node` slew-limits a software estop. Need contactor / SparkMAX enable line wired in series with the button NC contact |
| Wireless E-Stop ≥ 100 ft, hardware-based | Not described | **Likely missing** — need an industrial pendant or RF receiver wired to the same enable line as the mechanical button |
| Safety light, solid → flashing on autonomous, flashing → solid on exit | Not described | Need to add — drive a beacon from a GPIO or relay tied to an `autonomous_mode` line that the actuator node controls, **and** that drops when the E-stop drops |
| Software E-Stop (`/avros/actuator_command estop:true`, WebUI disconnect) | Implemented (CLAUDE.md "Web UI" section: "WebSocket disconnect → e-stop published automatically") | Keep as a **secondary** layer — does not satisfy the rule on its own |

The cleanest design that meets the rules:

```
   [Mechanical E-Stop NC]──┐
                           ├──(series)── 24V safety loop ──► safety relay coil
   [Wireless E-Stop NC]────┘                                  │
                                                              ▼
                              SparkMAX motor-controller enable / contactor on motor bus
                                                              │
                                                              ├──► Safety beacon (drops to solid via "not autonomous" logic)
                                                              └──► (optional) status line back into Teensy GPIO so software knows it tripped
```

Both buttons in a single normally-closed series loop is the standard pattern:
either button (or loss of radio) opens the loop, the safety relay de-energizes,
and motor power / motor-controller enable drops. The Jetson can monitor the
loop via a Teensy digital input but must **not** be in the cut path.

## 8. Things the rulebook is silent on (judgment calls)

These are not specified, so they're ours to decide and justify in the report:
- **Stopping distance / deceleration limit** — no number given. We should pick
  a target (e.g. ≤ 1 m at 5 mph) and demonstrate it.
- **Brake mode after cut** — Brake vs. Coast on the SparkMAX. We use Brake
  (CLAUDE.md), which is the right call for "quick stop".
- **Dual-channel / SIL-rated safety** — not required at IGVC. A single
  series-NC loop with a control relay is accepted; you don't need a Pilz or
  Sick safety relay.
- **Estop reset behavior** — the rules don't dictate latching vs. momentary,
  but a latching mushroom + key-twist reset is conventional. The wireless
  pendant should require an explicit re-arm action so a dropped pendant can't
  silently re-enable the vehicle.
- **Power source for the safety loop** — typically a separate small battery
  or an always-on rail so the loop survives even if the main 12V rail browns
  out (relevant for us: see CLAUDE.md "Jetson crashes randomly during motor
  testing" — the safety loop should not share that rail).

## 9. Recommended next steps for our team

1. **Hardware design (electrical):** spec the safety loop — pick a 22 mm red
   mushroom button, a wireless pendant kit (≥ 100 ft, FCC Part 15 unlicensed),
   a control relay with rated coil voltage, and decide whether to drop the
   SparkMAX enable line or kill 48 V at a contactor. Document on the wiring
   diagram alongside the existing 48V→12V buck.
2. **Mechanical mounting:** rear panel plate at hip height (~ 36 in) with the
   button, beacon, and pendant antenna. Center on the vehicle's longitudinal
   axis.
3. **Software interlock:** add a Teensy GPIO that reads the safety-loop
   status, publishes it as a topic, and forces the actuator node to publish
   `autonomous_mode = false` (so the safety light goes solid). The software
   path is *informational* — the cut is already done by the relay.
4. **Field test:** measure 100 ft range with the wireless pendant outdoors,
   measure stopping distance from 5 mph on Brake idle, photograph the install
   for the design report.
5. **Design Report write-up:** populate §5 Safety with: requirements list
   (mechanical button geometry, wireless range, "quick stop" target distance,
   safety-light state machine), target vs. measured table, photos, schematic.
   Reuse the requirement-vs-measured table format the rubric explicitly asks
   for (l. 1573).
