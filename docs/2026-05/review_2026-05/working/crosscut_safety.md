# Cross-cutting: Safety Chain End-to-End — Review

## Summary

**Verdict: the IGVC_ROS2 vehicle is not safe to operate autonomously today, and would fail IGVC inspection on day zero.** The competition is one month away; every P0 below is fixable in that window. Headline issues:

- **No hardware e-stop chain.** IGVC rules require "Vehicle E-Stops must be hardware based and not controlled through software." No e-stop GPIO in firmware, no contactor on motor power, no wireless RF receiver. The webui's red button is a software fourth-layer convenience. **Inspection failure on day zero.**
- **Multiple software safety paths broken.** `_estop` auto-clears on next non-estop command (`actuator_node.py:217`). No serial-link liveness — USB unplug = control loop writes into dead handle. IMU "fresh" flag set once, never reset → Xsens failure leaves heading-hold using frozen yaw. Teensy watchdog leaves `ctrl_mode = MODE_VELOCITY` so brake-idle never engages — chassis rolls on slopes. All afternoon-scale fixes; collectively a Swiss-cheese pattern.
- **No `nav2_collision_monitor`, and no LiDAR on the Humble local costmap.** Active plugins are `["semantic_layer", "inflation_layer"]` — Velodyne data is wasted, only camera classifications produce obstacles, no high-rate collision halt. Camera misclassification at 1.5 m/s = collision.
- **Power-rail brown-out documented but not fixed.** Jetson and SparkMAXes share a 12 V buck; motor inrush has rebooted the Jetson during testing. CLAUDE.md flags this as "hardware fix in progress."

Total effort to close every P0: ~ 1 week of dedicated team time. Hardware integrations (mushroom + contactor + RF + dedicated compute buck) in parallel with the software P0s (latch e-stop, serial liveness, IMU staleness, the one-line firmware watchdog fix, collision_monitor, voxel_layer to local costmap).

---

## IGVC AutoNav e-stop requirements (verbatim from rules)

`standards_firmware_safety.md` § 5 quotes the IGVC rules:

> **"Vehicle E-Stops must be hardware based and not controlled through software."**
>
> **"Mechanical e-stop must be located on the center rear of the vehicle, between 2 ft and 4 ft high."**
>
> **"A wireless e-stop is also required, range checked at a minimum of 50 ft (older revisions required 100 ft). During AutoNav and SD challenges the wireless e-stop is held by the judges."**

The engineering rationale (ABB, ISO 13850, IEC 60204-1): an e-stop is a Category 0 stop — uncontrolled removal of power — via a mechanical contactor between battery and motor drivers, driven by force-guided contacts. Software-only stops, GPIO-without-contactor, or ROS-topic estop *do not satisfy the rule*. Inspection checks both mechanical placement and wireless range; failing either means the vehicle does not run.

The team must therefore have, at competition arrival:

1. Mushroom button at center-rear, 2–4 ft high, latching, NC contacts.
2. Wireless e-stop receiver with verified ≥ 50 ft range, latching, NC contacts.
3. Contactor (or redundant pair) on battery-to-motor-driver power path, normally open, energized only when both (1) and (2) are clear.
4. The same e-stop signal fed to a Teensy GPIO so the firmware enters latched-fault state. This is layered on top of items 1–3, not a replacement.

`package_firmware.md` and CLAUDE.md show no evidence any of items 1–4 currently exist. The inspection-day checklist below verifies them on the physical vehicle.

---

## Safety chain trace (operator → motors)

### Link 1 — Physical mushroom button (HARDWARE)

**Should:** operator presses center-rear mushroom button → latches open, breaks safety circuit, drops contactor coil → motor power removed in ms. SparkMAXes lose 12V, wheels coast (Cat-0). Compute rail stays up.

**Today:** no evidence the circuit exists. `teensy_diff_drive.ino` has no `pinMode(ESTOP_PIN, ...)` and no GPIO read in `loop()`. `package_firmware.md` confirms: "Firmware has no GPIO input wired to a hardware e-stop circuit. Software-only e-stop fails inspection."

**Action:** 22 mm latching mushroom button (Allen-Bradley 800FM-MT44 or equivalent, 2-pole NC) + contactor (TE EV200 or equivalent, 12 V coil, 250 A) physically wired before any autonomous testing.

**Severity: P0 — HARDWARE.** Inspection-day item.

### Link 2 — Wireless e-stop receiver (HARDWARE)

**Should:** judge with RF transmitter at ≥ 50 ft presses kill → receiver drops relay → opens same safety circuit as link 1 → contactor opens. Frequency must be on a dedicated safety radio (Telecrane, Hetronic, ePlex) so WiFi/Tailscale failures don't disable it.

**Today:** nothing in the workspace handles RF reception. The webui WS disconnect path (`webui_node.py:164`) publishes e-stop but depends on a software stack — the IGVC rule deliberately requires a hardware-only path.

**Action:** off-the-shelf wireless e-stop kit wired in series with the mushroom button so either trigger opens the contactor. Range-test with a tape measure.

**Severity: P0 — HARDWARE.** Inspection-day item.

### Link 3 — Motor-power contactor (HARDWARE)

**Should:** a contactor (or redundant pair per ISO 13849-1 PLr c) between battery and SparkMAX 12 V. Coil is energized only while the safety circuit is intact. Any e-stop trigger drops coil → contactor opens within 10–30 ms (EV-rated DC contactor). Compute rail unaffected.

**Today:** the only "stop" is SparkMAX brake-idle, which is a half-bridge short — *not* removal of power. If SparkMAX firmware locks up or CAN faults, brake-idle doesn't engage. CLAUDE.md confirms shared 12 V rail with no contactor on the motor leg.

**Severity: P0 — HARDWARE.** Without this, links 1 and 2 are decorative.

### Link 4 — WebUI e-stop button (SOFTWARE)

**Should:** operator hits red button → JS sends `{type:'estop', value:true}` → webui_node publishes `ActuatorCommand{estop=True}` → actuator_node sets `_estop`. Fourth-layer convenience; works only while autonomy stack + WS + phone are all alive.

**Today (correct):** HTML button is full-width red, JS sends estop on click and on disconnect, `webui_node.py:145-150` + `finally:` work as designed.

**Gap 1 (P0):** no auth on WS. `host='0.0.0.0'` (`webui_node.py:200`), no token, no Origin. Anyone on venue WiFi or Tailscale can open `wss://<jetson>:8000/ws` and drive. Single-controller mutex loses to reconnect-loop. **Spectator can drive the chassis.**

**Gap 2 (P0):** software-only e-stop doesn't satisfy IGVC hardware rule.

**Gap 3-5 (P1):** no server-side joystick watchdog (`receive_json()` blocks indefinitely on half-open TCP); NaN propagates through `float(data.get('x', 0))` to Teensy; `max_throttle: 1.0` in YAML overrides safer 0.55 default.

**Severity: P0 — SOFTWARE.**

### Link 5 — Software e-stop in ActuatorCommand (SOFTWARE)

**Should:** any node publishes `ActuatorCommand{estop=True}` → actuator_node *latches* `_estop`, decelerates with bounded jerk (separate `estop_decel_mps2`), engages brake-idle at zero, *refuses to re-arm* without explicit clear. Canonical latched-fault pattern.

**Today (`actuator_node.py:217, 258-259`):**
```python
self._estop = False        # AUTO-CLEAR on next non-estop message
...
self._slew_v = 0.0         # instant zero step — *infinite* deceleration
```

**Gap 1 (P0):** phone WiFi blip → JS reconnect → publishes `'control'` frame with `estop=false` → motion resumes mid-stride. `package_avros_control.md` F6.

**Gap 2 (P0):** infinite-jerk decel saturates SparkMAX PID, regen-spikes 12 V rail (the documented brown-out cause), snaps drivetrain backlash. Standards § 6: emergency stops should still respect a decel cap.

**Gap 3 (P1):** no Nav2-callable software stop. Only `_on_actuator_cmd` can set `_estop`; Nav2 publishes only `cmd_vel`. A Nav2 BT abort produces a 500 ms watchdog timeout, not an immediate stop.

**Gap 4 (P1):** fragile message contract — bare `bool estop`, `string mode` not `uint8` enum, `header.stamp` unused. `package_avros_msgs.md` P0.

**End-to-end trace (intact path):** ~ 32 ms typical, ~ 50 ms worst-case → ~ 75 mm extra travel at 1.5 m/s. Fast, but only meaningful with gaps 1-3 fixed.

**Severity: P0 — SOFTWARE.** Each fix < 1 hour.

### Link 6 — cmd_vel / ActuatorCommand staleness watchdog (SOFTWARE)

**Should:** if no `/cmd_vel` or `/avros/actuator_command` arrives within `cmd_timeout_s` (0.5 s), zero the requested velocity, ramp slew toward zero at `max_linear_decel_mps2` (1.5 m/s²), once at rest send `S` to engage brake-idle.

**Today: this link works correctly.** `actuator_node.py:231-250` checks both timestamps; when both are stale, `v_req=w_req=0`. The slew limiter at lines 261-269 ramps the active setpoint to zero using the decel cap. Line 298 sends `S` once the slew is below 1e-6.

**Scenario — Nav2 bt_navigator hangs:** stop time ~ 1.5 s (0.5 s timeout + 1.0 s decel ramp), stopping distance ~ 0.75 m at top speed. Acceptable for static-obstacle arena; insufficient if a person walks in front at 1.5 m distance.

**Gap 1 (P1):** 500 ms timeout is conservative; tighten to 0.3 s reduces coast travel from 0.75 m to 0.45 m.

**Gap 2 (P1):** `msg.watchdog_active = False` hard-coded (`actuator_node.py:327`); operator has no UI signal.

**Gap 3 (P2):** slew uses parameter `dt`, not measured `dt`.

**Severity: OK with P1 polish.**

### Link 7 — Serial-link liveness watchdog (SOFTWARE — broken)

**Should:** timestamp last successful write and last E-line; if either > 200 ms set `_serial_dead`, force `_estop`, re-open with backoff, throttle-log only on state change.

**Today: none of this exists.** Per `package_avros_control.md` F3+F4 (P0): `actuator_node.py` opens serial in `__init__` with no try/except (`:107-110`); writes catch all exceptions and log at 50 Hz (`:369-374`); reader same shape (`:376-398`); E-line match never timestamps freshness (`:387-395`).

**Failure modes:**
1. **USB unplug mid-drive** — control loop logs at 50 Hz, wheels keep moving until Teensy's own 300 ms watchdog trips → ~ 0.6 s stop + slope risk (link 8 bug). Operator-blind.
2. **Teensy lockup** — kernel queues writes successfully, Teensy doesn't see them. Heartbeat stops → SparkMAX coasts at 100 ms.

**Severity: P0 — SOFTWARE.** ~ 1 afternoon.

### Link 8 — Teensy host-loss watchdog (SOFTWARE — buggy)

**Should:** on host timeout (300 ms): (a) zero setpoints, (b) switch `ctrl_mode = MODE_DUTY` so SparkMAX brake-idle engages, (c) keep sending heartbeat (heartbeat loss → SparkMAX coasts, not brakes), (d) latch trip; require explicit `S` to clear.

**Today (`teensy_diff_drive.ino:355-362`):**
```c
if (now - t_last_host > WATCHDOG_MS) {
    if (!wdt_tripped) { Serial.println("# WDT ..."); wdt_tripped = true; }
    left.cmd_rpm = right.cmd_rpm = 0.0f;
    left.cmd_duty = right.cmd_duty = 0.0f;
}
```

**Missing line:** `ctrl_mode = MODE_DUTY;`. The control tick at line 365 then takes the velocity branch and sends `setVelocity(_, 0)` — the velocity PID drives wheels to zero RPM but does NOT engage brake-idle. This is the exact failure mode the `S` command was rewritten to avoid (in-firmware comment at lines 217-219). On a slope, chassis rolls. `package_firmware.md` P0 #2.

**Fix is one line:** add `ctrl_mode = MODE_DUTY;` after line 361.

**Other gaps (P1):** watchdog auto-rearms on next valid command (lines 224, 252, 294) — host crash-and-respawn resumes motion mid-stride. DIAG state never reaches ROS (host parses E-lines only). No bounded-time host response check.

**Severity: P0 — SOFTWARE.** One-line fix.

### Link 9 — Teensy hardware MCU watchdog / RTWDOG (SOFTWARE — disabled)

**Should:** RTWDOG enabled with ~ 2 s timeout, fed from `loop()`. Firmware lockup → MCU reset → re-runs `setup()`, re-establishes heartbeat. SparkMAX coasts during the reset gap (heartbeat lost > 100 ms).

**Today: not enabled.** No `WDOG` / `RTWDOG` / `Watchdog_t4` reference in `teensy_diff_drive.ino`. `package_firmware.md` P0 #3: "frozen firmware stays frozen until power cycle."

**Why P1, not P0:** SparkMAX heartbeat-loss does coast wheels safely — runaway is impossible. But firmware lockup ends the run, no recovery without power-cycle.

**Severity: P1 — SOFTWARE.** Recoverability backstop. ~ 10 lines using `Watchdog_t4` library.

### Link 10 — CAN bus liveness / SparkMAX heartbeat (BOTH)

**Should:** Teensy sends `0x01011840` Universal Heartbeat at 50 Hz, byte 3 = `0x12`. SparkMAX disables output on > 100 ms heartbeat gap (REV doc).

**Today: works correctly.** `teensy_diff_drive.ino:364` calls `sendHeartbeats()` unconditionally each control tick, including during watchdog trips (deliberate — keeps SparkMAX enabled long enough for controlled stop). 50 Hz cadence, FlexCAN_T4 FIFO + ISR pattern is standards-correct.

**Gap (P1): asymmetric CAN failure.** If one CAN cable becomes intermittent or one SparkMAX powers off, that wheel coasts at heartbeat timeout while the other drives — chassis pivots uncontrollably. The host can't detect this (only parses E-lines, not per-wheel timestamps). Mitigation: per-wheel STATUS_2 timestamps + soft stop on stale.

**Severity: link OK; P1 for asymmetric-fault detection.**

### Link 11 — SparkMAX brake-idle behavior (HARDWARE config)

**Should:** SparkMAX with duty 0 + no setpoint enters idle mode. `kIdleMode = 1` = Brake (half-bridge short for dynamic braking + slope hold).

**Today:** Brake-idle is persisted in flash on both SparkMAXes (CLAUDE.md). **But:** brake-idle only engages when SparkMAX sees MODE_DUTY=0. The link 8 watchdog bug leaves MODE_VELOCITY and runs the PID against zero — wheels stop, brake-idle does NOT engage, chassis rolls on a slope. Heartbeat-loss path also coasts (SparkMAX shutdown ≠ Brake idle). Both fix when host commands MODE_DUTY=0 before timing out.

**Severity: HW CONFIG OK** (Brake is set). Operationally tied to link 8 fix. Slope test on inspection day.

### Link 12 — IMU loss / staleness (SOFTWARE — broken)

**Should:** if `/imu/data` is silent > 200 ms, mark stale, disable heading-hold and gyro-rate branches, fall back to open-loop ω from cmd_vel. On resume, re-capture `_heading_target`.

**Today: no staleness check.** `actuator_node.py:228`:
```python
self._imu_fresh = True   # set on first message, never reset
```
Heading-hold at `:274-289` runs as long as `_imu_fresh` is true. No timestamp.

**Failure modes:**

1. **Xsens disconnect mid-drive.** `_current_yaw` frozen → heading-hold applies `heading_kp * (target - frozen_yaw)` indefinitely → chassis curves off course while operator sees no signal.
2. **Xsens GNSS heading lock loss → yaw jump.** Heading-hold computes huge error → swerve up to `_max_w * 0.5 = 0.5 rad/s`. Not runaway but not controlled.
3. **Quaternion all-zero or NaN.** `yaw_from_quaternion` returns 0 silently; heading-hold targets yaw 0.

**`package_avros_control.md` F5 (P0).** 30-minute fix: timestamp + 200 ms gate + throttled WARN.

**Severity: P0 — SOFTWARE.** "Off-course = collision risk" angle, not literal runaway.

### Link 13 — GNSS / RTK degradation (SOFTWARE — unchecked)

**Should:** RTK FIXED → continue; RTK FLOAT → degraded warning; SPS → disable global localization, fall back to local dead-reckoning. Publish `/avros/rtk_status` for operator visibility.

**Today:** navsat_transform_node consumes `/gnss` and weights it by `position_covariance` only — no discrete RTK status check, no UI indicator. EKF down-weights bad fixes via covariance but has no Mahalanobis outlier rejection (no `pose0_rejection_threshold` in `ekf.yaml`). RTK FIXED → SPS jump of several meters propagates into the EKF.

**Failure mode (P1):** NTRIP drop mid-run → Xsens degrades to FLOAT/SPS → position error grows from 2 cm to 5 m. At 1.5 m/s, chassis can be 5 m off the planned path within 3 s. No operator warning. Vehicle drives in the wrong place but doesn't run away.

**Severity: P1 — SOFTWARE.** Off-course navigation, not runaway. Add status node + EKF outlier gate + verify `magnetic_declination_radians`.

### Link 14 — Collision monitor / costmap inflation (SOFTWARE — missing)

**Should:** `nav2_collision_monitor` runs separately at LiDAR scan rate (10-20 Hz) with SlowDown polygon (~ 1.5 m, scale to 30 %) and Stop polygon (~ 0.8 m, immediate). Bypasses planner. Standards § 8.

**Today: not configured** (`grep collision_monitor` returns nothing in either nav2 params file). The team relies on RPP regulated-velocity scaling reading a 5 Hz local costmap. Worst-case reaction latency ~ 200 ms + RPP horizon.

**Worse: Humble local costmap has no LiDAR layer** (`nav2_params_humble.yaml:142` lists only `["semantic_layer", "inflation_layer"]`). The `voxel_layer` block at :144-176 is YAML-loaded but not in the active list — Velodyne data wasted. Only camera-classified hazards appear; camera misclassification = invisible obstacle.

Also: `inflation_radius: 0.5` < robot_radius 0.8 → planner plans into hard collision.

**Failure modes:** person walks into path 1 m ahead (no LiDAR, no monitor → collision); camera misclassifies obstacle (LiDAR sees it but isn't on costmap → collision); ZED disconnects (local costmap loses its only obstacle source).

**Severity: P0 — SOFTWARE.** voxel_layer → plugins (30 min); collision_monitor config + launch (2 hours).

### Link 15 — Power-rail brown-out (HARDWARE)

**Should:** Jetson and SparkMAXes powered from separate DC-DC converters off the battery. Compute buck sized for ~ 30 W steady + headroom; motor buck sized for NEO peak inrush (100-200 A). Brown-out events on motor rail don't affect compute. Standards doc § 7.

**Today: shared 12 V rail.** CLAUDE.md known-issues table:
> "Jetson crashes randomly during motor testing | Shared 12 V rail — Jetson and SparkMAXes both fed from the 48V→12V buck. Motor inrush (~200A transient) sags the rail below Jetson brown-out threshold. Fix: dedicated 48V→19V buck for Jetson, separate from motor rail."

Documented as "P0 hardware fix in progress" but not finished.

**Failure scenario — every hard accel:** rail sags → Jetson reboots → actuator_node dies → Teensy 300 ms watchdog (with link 8 bug, no brake-idle) → wheels stop via PID → 30-60 s blind during ROS restart → if slope, chassis rolls. Empirically observed.

**Related (P1):** bus voltage is in Teensy DIAG but actuator_node only parses E-lines, so it's invisible to ROS. No soft-stop trigger on bus < 11 V.

**Severity: P0 — HARDWARE.** Dedicated compute buck (~$50, half-day install) + bus-voltage host monitor (P1).

## Failure mode → response matrix

| # | Failure | Should stop? | Today | Time / outcome | Severity |
|---|---|---|---|---|---|
| F1 | Mushroom button pressed | YES Cat-0 | NO — not wired | ∞ | **P0 HW** |
| F2 | Wireless e-stop pressed | YES Cat-0 | NO — not wired | ∞ | **P0 HW** |
| F3 | WebUI e-stop pressed | YES assist | YES but auto-clears + infinite-jerk decel | ~50 ms, re-arm risk | **P0 SW** |
| F4 | WebUI WS disconnect | YES assist | YES — `finally:` publishes estop | ~50 ms | OK |
| F5 | Hostile WiFi user connects to webui | Reject | NO auth — full control granted | n/a | **P0 SW** |
| F6 | Nav2 stops publishing cmd_vel | YES controlled | YES — 500 ms + 1.0 s decel | ~1.5 s, ~0.75 m | OK (P1) |
| F7 | USB to Teensy unplugged | YES immediate | No host detect; firmware WDT stops PID-zero (no brake) | ~0.6 s + slope risk | **P0 SW** |
| F8 | Teensy firmware lockup | YES | Heartbeat ceases → SparkMAX coast | ~100 ms | OK (P1 RTWDOG) |
| F9 | Host watchdog trip | YES Cat-1 brake | YES PID-zero but no brake-idle | slope risk | **P0-adj** (1-line fix) |
| F10 | One CAN cable disconnect | YES symmetric | NO — chassis pivots uncontrolled | ∞ asymmetric | **P1 SW** |
| F11 | Both SparkMAX lose heartbeat | YES | YES — both coast | ~100 ms + coast | OK |
| F12 | IMU disconnect / freeze | Degrade | NO — heading-hold uses frozen yaw | curves off course | **P0 SW** |
| F13 | NaN quaternion | Ignore | Silently treated as yaw=0 | drift | **P2 SW** |
| F14 | RTK FIXED → SPS | Warn | NO warning | off-course | **P1 SW** |
| F15 | Person 1 m ahead at 1.5 m/s | YES | NO collision_monitor; no LiDAR on local costmap | collision likely | **P0 SW** |
| F16 | Camera misclassifies obstacle | YES LiDAR fallback | NO — LiDAR not on Humble local costmap | collision likely | **P0 SW** |
| F17 | 12 V rail sag during hard accel | Warn | Jetson reboots, firmware WDT stops | 30-60 s blind | **P0 HW** |
| F18 | Bus voltage < 11 V | Soft stop | NOT DETECTED — DIAG not parsed | n/a | **P1 SW** |
| F19 | Op presses Ctrl-C twice | Graceful | Mostly OK | ~100 ms | OK (P2) |
| F20 | Nav2 wants software estop | YES | No path — Nav2 publishes only cmd_vel | n/a | **P1 SW** |
| F21 | webui_node crashes mid-drive | YES (cmd_vel takeover) | YES — 500 ms watchdog | ~1.5 s | OK |

**Tally: 6 P0 HW + 6 P0 SW.** HW is off-the-shelf parts + a few days of integration; SW is afternoon-scale code changes.

**Most urgent combined risk:** F12 (IMU loss → curves off course) × F15/F16 (no collision monitor + camera-only costmap). Chassis drifts off course and doesn't see what's in the new direction. Realistic worst case for a 1.5 m/s vehicle around spectators.

## Inspection-day checklist

Verify on the *physical vehicle* before competition. Print this; sign each item off as a team.

### Mechanical / electrical

- [ ] Mushroom button on the rear of the vehicle, between 2 ft and 4 ft (tape measure).
- [ ] Mushroom is latching (twist-and-pull to release) and uses NC contacts (multimeter: continuity present in idle, broken when pressed).
- [ ] Wireless e-stop receiver mounted with clear line-of-sight antenna; transmitter has fresh batteries; latching switch.
- [ ] **Wireless range test at 50 ft** in open space; verify motors lose power on press.
- [ ] Both mushroom and wireless feed the *same* contactor (in series).
- [ ] Contactor is between battery and SparkMAX 12 V — not just signaling firmware. Press mushroom: SparkMAX LEDs must go off.
- [ ] After release, vehicle does NOT auto-resume — explicit re-arm required.
- [ ] **Slope test:** chassis on 5° ramp, command stop, confirm it holds (not rolls). If rolls, brake-idle isn't engaging.
- [ ] Compute rail separate from motor rail (multimeter Jetson supply during hard accel; should stay above 18 V).
- [ ] Battery cutoff disconnects everything.
- [ ] All power cables have strain relief.

### Software pre-flight (every test day)

- [ ] `colcon build --symlink-install` clean.
- [ ] `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` + `CYCLONEDDS_URI=...` exported in the shell.
- [ ] `ros2 topic hz /imu/data` ≈ 100 Hz, `/velodyne_points` ≈ 70 Hz, `/gnss` reports RTK FIXED.
- [ ] On jacks: `ros2 topic pub /cmd_vel ...` confirms both wheels drive at same speed.
- [ ] Watchdog test: stop publishing cmd_vel, confirm wheels stop in < 1.5 s.
- [ ] E-stop test (webui + hardware): both stop wheels within their respective windows.

### Bring-up regression

- [ ] Rerun `phase6c_pid_verify.py`; assert max RPM within 5% of 2026-04-23 baseline (5532 L / 5072 R). > 20 % drop = chain wear / belt slip.
- [ ] CAN continuity: 60 Ω across CAN_H/CAN_L (two 120 Ω terminators in parallel).
- [ ] Both SparkMAX FW 26.1.4 (REV Hardware Client).
- [ ] Both SparkMAX idle mode = Brake (Hardware Client, persisted in flash).
- [ ] Motor inversion configured at the SparkMAX, not the firmware.

### What to bring

This checklist (empty boxes), IGVC rules PDF (current year), multimeter, tape measure, plywood for slope test, wireless e-stop transmitter (fresh batteries), spare mushroom button + spare contactor, SSH access + operator laptop.

## Consolidated safety punch list

### P0 — UNSAFE TO DRIVE in current state

Fix all before any non-jacked field test.

**Hardware:**

1. **Mushroom-button e-stop circuit** (link 1). Center-rear, 2-4 ft, NC contacts, feeds contactor coil. **IGVC inspection failure without this.** ~ 1 day work.
2. **Wireless e-stop receiver** (link 2). 868 MHz or 2.4 GHz safety RF kit, range-tested at ≥ 50 ft, in series with mushroom. **IGVC inspection failure without this.** ~ 0.5 day.
3. **Motor-power contactor** (link 3). EV200 or equivalent between battery and SparkMAXes, driven by the safety circuit. **IGVC inspection failure without this.** ~ 0.5 day.
4. **Dedicated compute-rail buck** (link 15). 48 V → 19 V for Jetson, separate from motor rail. ~ 0.5 day + parts.

**Software:**

5. **Latch `_estop` and bound decel** (link 5). `actuator_node.py:217, 256-259`. Don't auto-clear on next non-estop msg; require `clear_estop` field or service. Replace `_slew_v=0` with bounded `estop_decel_mps2`. ~ 1 hour.
6. **Serial liveness detection** (link 7). `actuator_node.py:107-110, 369-398`. Timestamp last write and last E-line; > 200 ms → `_estop=True`, re-open with backoff. ~ 1 afternoon.
7. **IMU staleness detection** (link 12). `actuator_node.py:225-228`. Timestamp last IMU; > 200 ms → reset `_imu_fresh=False`, skip heading-hold. ~ 30 min.
8. **Teensy watchdog `ctrl_mode` fix** (link 8). `teensy_diff_drive.ino:355-362`. Add `ctrl_mode = MODE_DUTY;` after wheel zeroing. **One-line fix.**
9. **WebSocket auth** (link 4). `webui_node.py:200`. Bind localhost or Tailscale-only; add token + Origin check. ~ 1 hour.
10. **`nav2_collision_monitor`** (link 14). Add config to `nav2_params_humble.yaml` (SlowDown 1.5 m, Stop 0.8 m), launch from `navigation.launch.py`. ~ 2 hours.
11. **`voxel_layer` to Humble local costmap** (link 14). `nav2_params_humble.yaml:142`, add to plugins list. ~ 30 min.

**Total P0: ~ 1 week of dedicated team effort** (4 days HW + 1.5 days SW).

---

### P1 — Must-fix-before-competition

12. **Enable Teensy RTWDOG** (link 9). 2 s timeout via `Watchdog_t4`. ~ 1 hour.
13. **Tighten `cmd_timeout_s` 0.5 → 0.3** (link 6). `actuator_params.yaml:29`.
14. **Wire `msg.watchdog_active`** (link 6). `actuator_node.py:327`. ~ 5 lines.
15. **Bus-voltage monitor host-side** (link 15). Parse Teensy DIAG, soft stop on < 11 V. ~ 2 hours.
16. **Stop firmware watchdog auto-rearm** (link 8). Require explicit `S` to clear. ~ 30 min.
17. **Server-side joystick watchdog** (link 4). `asyncio.wait_for(receive_json, 0.5)`. ~ 5 lines.
18. **Lower webui `max_throttle` 1.0 → 0.5** (link 4). 1 line.
19. **NaN check on joystick floats** (link 4). `webui_node.py:138-139`. ~ 5 lines.
20. **RTK status monitoring** (link 13). Subscribe `/gnss` → `NavSatStatus`, publish state, EKF outlier gate. ~ 4 hours.
21. **Verify LiDAR contributes to local costmap** after P0 #11 (link 14). Bench test.
22. **Nav2-callable software e-stop topic** (link 5). New `/avros/software_estop` for BT-driven halts. ~ 1 hour.
23. **Set `RMW_IMPLEMENTATION` + `CYCLONEDDS_URI` in every launch file.** Phase 2 P0 #2 in bringup. ~ 30 min.
24. **`clear_estop` semantics** (link 5). Add field or enum to disambiguate. ~ 30 min.
25. **Asymmetric CAN-fault detection** (link 10). Per-wheel STATUS_2 timestamps, soft stop on stale. ~ 2 hours.
26. **Verify `magnetic_declination_radians`** (link 13). Field test against known east-pointing path. ~ 2 hours.
27. **Firmware GPIO e-stop + latch** (link 1 follow-up). After hardware is wired. ~ 2 hours.
28. **Unit tests for kinematics + slew** in `avros_control`. ~ 2 hours.
29. **Vendor nipplejs** in `webui/static/`. Avoids offline-Jetson risk. ~ 30 min.
30. **Inflation_radius 0.5 → 0.8** in Humble nav2_params (matches robot_radius). 2 lines.

### P2 — Improve before next year

31. `diagnostic_msgs/DiagnosticArray` output from all nodes; rqt_diagnostics for unified view.
32. Replace `string mode` with `uint8` enum in ActuatorCommand/State.
33. Slew-saturation telemetry on `/avros/diagnostics/slew_lag`.
34. Unit tests: `wrap_angle`, `yaw_from_quaternion`, diff-drive inverse, slew limiter, midpoint integrator.
35. Regression test harness around Phase 6c (assert max RPM within 5% of baseline).
36. Always-on rosbag recording of safety-critical topics to removable SSD.
37. `SetParametersCallback` runtime parameter validation.
38. Documented test plan covering bring-up + inspection-day verification.
39. Remote Tailscale kill topic for chase-vehicle safety officer.
40. Fail-loud serial port detection on missing `/dev/ttyACM0`.
41. Udev rule + symlink for Teensy.
42. Dual-channel safety relay (PLr c) — future industry-grade.
43. Wiring diagram of the full safety chain in `docs/`.
44. Safety-state UI panel in webui (auth, serial, IMU, RTK, bus voltage, watchdog).
45. Firmware compiled with `-Wall -Wextra -Wpedantic`.
46. Bring-up CSVs in artifacts directory + `.gitignore`.
47. Reduce nav2_params (Jazzy) ↔ nav2_params_humble drift.
48. Bandwidth-throttled costmap publishing (turn off `always_send_full_costmap` for remote viz).
49. Bus-voltage histogram in DIAG output for sub-brown-out detection.

## Positives

The workspace shows real engineering maturity in places worth preserving while the P0s are fixed.

- **Control architecture is right.** Last-wins between `cmd_vel` and `ActuatorCommand`; both feed the same slew limiter, IMU corrections, and freshness watchdog. Webui and Nav2 produce identical behavior.
- **Slew limiting with separate accel/decel caps** (`actuator_node.py:263-269`) matches standards § 6.
- **Midpoint pose integration** in `_publish_odom` is 2nd-order accurate; replaces a buggy forward-Euler.
- **Heading-hold gating** (`|w| < deadband AND |v| > 0.02`) correctly avoids stationary engagement.
- **Firmware is the most engineered part of the workspace** — `volatile` discipline, rollover-safe `millis()` subtraction, no String/malloc/delay() in the 50 Hz tick, MAX_RPM/MAX_DUTY clamps, multiple SparkMAX FW 26.1.4 protocol bugs caught and fixed.
- **CAN heartbeat at 50 Hz, sent during watchdog trips** — keeps SparkMAX enabled long enough for controlled stop.
- **Webui disconnect → e-stop** via `finally:` (`webui_node.py:130-165`) fires on every exit path.
- **Brake-idle persisted in SparkMAX flash**; persistent journald enabled on Jetson (the brown-out diagnosis used it).
- **Team documents what they learn.** CLAUDE.md known-issues + `firmware/.../FINDINGS.md` capture the trail.
- **Reproducible bring-up.** Phase 1–7 scripts + timestamped CSVs + BRING_UP.md form a model record.
- **CycloneDDS config correct** (shared memory off, 10 MB socket buffer).
- **Dual-EKF localization correct** — local fuses IMU + wheel odom, global adds GPS; `broadcast_cartesian_transform: false` prevents TF loop.
- **`actuator_params.yaml` is exemplary** — every constant has a unit and a derivation comment.

The fix list is finite and nothing is inventive. The team has demonstrated through the firmware bring-up that they can execute on careful, instrumented embedded work. **Execute the P0 list, run the inspection-day checklist on the actual vehicle, and the chassis will pass inspection.**
