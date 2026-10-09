# Session log — 2026-04-23: Diff-drive commissioning

Full bench bring-up of the new AndyMark Raptor Track Drive chassis and
rewrite of the actuator control stack for differential-drive kinematics.
Firmware, ROS node, protocol, and tuning were all changed — this is a
breaking change vs the pre-existing Ackermann/UDP pipeline.

## Hardware

| Item | Spec |
|---|---|
| Chassis | AndyMark Raptor Track Drive, 29" track gauge |
| Drivetrain | 2× NEO brushless motor → 12.75:1 ToughBox Mini → 22T:22T #35 chain → 20T × 0.5" pitch pulley → timing belt track |
| Effective ratio | 0.01994 m ground per motor revolution |
| Motor controllers | 2× REV SparkMAX FW 26.1.4 (CAN ID 1 = left, 2 = right) |
| Bus bridge | Teensy 4.1 on CAN1 (pins 22/23), USB-serial to Jetson at 115200 |
| Power | 48 V battery → buck → 12 V shared bus (feeds Jetson + SparkMAXes — **see known issues**) |

## Firmware — `firmware/teensy_diff_drive/teensy_diff_drive.ino`

New firmware written from scratch. Replaces the old UDP-JSON Teensy firmware.

Serial protocol (115200 baud, line-oriented):

| Cmd | Effect |
|---|---|
| `L<rpm> R<rpm>` | velocity mode setpoint per wheel, SparkMAX PID handles loop |
| `UL<d> UR<d>` | duty-cycle mode setpoint (bypasses PID), for bench/open-loop |
| `S` | stop — switches to MODE_DUTY with 0 duty to trigger SparkMAX Brake idle |
| `D` | DIAG line: tx, rx, wdt, mode, L/R meas/cmd, duty, bus voltage |
| `K[PIDF]<val>` | write PID slot-0 gain to both SparkMAXes via PARAMETER_WRITE |
| `BURN` | PERSIST_PARAMETERS — writes RAM PID to SparkMAX flash |

CAN protocol aligned with REV-Specs 2.1.0 `spark-frames-2.1.0`:

| Frame | cls | idx | Notes |
|---|---|---|---|
| Universal heartbeat | CAN ID 0x01011840 | — | FRC roboRIO heartbeat, byte 3=0x12 (enabled+sysWdt) |
| Secondary heartbeat | 11 | 2 | broadcast 0xFF×8 fallback |
| Velocity setpoint | **0** | **0** | was cls=1 idx=2 in legacy — FW 25+ moved it |
| Duty-cycle setpoint | 0 | 2 | proven working on FW 26.1.4 |
| STATUS_0 (decode) | 46 | 0 | voltage @ bits[27:16] × 0.007326 V |
| STATUS_2 (decode) | 46 | 2 | velocity float32 LE @ [0:3], position @ [4:7] — disabled by default, needs SET_STATUSES_ENABLED |
| SET_STATUSES_ENABLED | 1 | 0 | mask=0x0004, enable=0x0004 → turns on STATUS_2 |
| PARAMETER_WRITE | **14** | **0** | was cls=48 legacy — byte 0 = param_id, bytes 1-4 = float32 LE value, DLC = 5 |
| PERSIST_PARAMETERS (BURN) | **63** | **15** | magic 0xA3 0x3A (0x3AA3 LE) |

Safety: 300 ms host watchdog → zero cmd_rpm/cmd_duty on loss. No `!Serial`
guard (it flickered under CDC traffic and caused blinking-magenta).

## Bench test infrastructure — `firmware/teensy_diff_drive/`

Seven phase scripts that go through the serial protocol (no firmware
changes needed to run any of them):

| Script | Purpose |
|---|---|
| `phase1_handspin.py` | Manual hand-spin resistance check (no motor) |
| `phase2_baseline.py` | DIAG + voltage + CAN rate health check |
| `phase3_stiction.py` | Per-wheel minimum duty to start motion |
| `phase4_duty_sweep.py` | Duty vs steady-state RPM characterization |
| `phase5_stability.py` | Long-hold RPM stability + periodic-dip detection |
| `phase6_pid_tune.py` | Auto-tune kFF and kP from empirical Phase 4 data |
| `phase7_burn_verify.py` | Write sentinel, BURN, manual power-cycle, verify |
| `test.py` | General-purpose interactive tool (`u` duty, `v` velocity, `live`, `raw`) |
| `teensy_bridge.py` | Shared helper: Teensy class, run_duty/run_velocity with stats, CSV logger, SIGINT→stop |
| `FINDINGS.md` | Empirical results from phases 2-6 |
| `data/*.csv` | Raw time-series and summary data |

All scripts import `teensy_bridge.py`. Phase 1 requires human; all others
fully automated.

## ROS actuator_node — `src/avros_control/avros_control/actuator_node.py`

Complete rewrite. Replaces UDP/JSON/Ackermann with serial/diff-drive.

| Feature | Implementation |
|---|---|
| Serial I/O | pyserial to `/dev/ttyACM0`, background reader thread parses `E` lines |
| Command input | Subscribes `/cmd_vel` (Twist) and `/avros/actuator_command` (webui). Both map to unified (v, ω) target |
| Kinematics | Diff-drive inverse: `L_mps = v − ω·W/2`, `R_mps = v + ω·W/2`, convert to motor RPM |
| Heading-hold | When `|ω_cmd| < 0.05 rad/s` AND `|v| > 0.02 m/s`, lock IMU yaw target and apply P correction via `heading_kp` |
| Gyro-stabilized turns | When turning, use IMU yaw-rate as feedback via `yaw_rate_kp` |
| Slew-rate limiter | Ramps target (v, ω) toward requested values. Protects 12V rail + passengers. Asymmetric accel/decel caps |
| Startup | Pushes Phase-6 PID gains (kFF/kP/kI/kD) to both SparkMAXes via `K` commands |
| Publishes | `/wheel_odom` (nav_msgs/Odometry, integrated from E-line positions) for EKF fusion; `/avros/actuator_state` for webui display |
| E-stop | Immediate stop, bypasses slew cap |

Config: `src/avros_bringup/config/actuator_params.yaml` — all tunables are
ROS params (serial port, track width, speed limits, accel/decel, gains).

## WebUI changes

`src/avros_webui/webui_node.py` unchanged. Still publishes `ActuatorCommand`.
The new `actuator_node` interprets `throttle/brake/steer` as unified (v, ω)
target, so heading-hold + slew-rate apply to webui commands too.

`webui_params.yaml`:
- `max_throttle` raised `0.55 → 1.0` (phone reaches full 1.5 m/s range)

## Verified working

1. Both tracks spin via duty-cycle mode (5% left wheel isolated → ~170 RPM)
2. Parameter-write (cls=14) confirmed: sentinel kP=0.00042069 read back in REV Hardware Client on both controllers
3. Velocity mode (cls=0 idx=0) drives both wheels (PID gains `kFF=0.000197, kP=0.0004`)
4. Brake mode stop: 9.47 s coast → **0.58 s active brake** (16× faster)
5. `S` command uses MODE_DUTY path so SparkMAX idle state triggers regen brake
6. WebUI on Jetson, phone joystick controls the robot with smooth accel/decel
7. `/wheel_odom` publishing for EKF

## Known hardware issues (unresolved)

### Shared 12V power rail — **blocking field testing**

Jetson and both SparkMAXes are all fed from the same 48V→12V buck converter
output. Motor inrush currents (NEO peak ~105A, two NEOs ~200A transient)
cause voltage sag on the 12V rail that crosses the Jetson's brown-out
threshold. Confirmed by 3 crashes during a single afternoon of motor
testing, correlated with high-duty transitions.

Fix: add a dedicated 48V→19V buck for the Jetson, separate from the motor
rail. Meanwell SD-50C-24 or DROK equivalent, ~$25. Standard FRC/URC
architecture — never share motor and logic power rails.

Persistent journald now enabled on Jetson so the next crash leaves
forensic logs, but the real fix is hardware.

### Right-track mechanical asymmetry — not a blocker, cosmetic

Phase 3 stiction sweep: Left = 0.030 duty to start, Right = 0.060 duty.
2× stiction asymmetry. Phase 4 duty sweep: Left extrapolated max 5532 RPM
(97% of free speed), Right 5072 RPM (89%). Right track has uniform higher
running friction.

No periodic RPM dips (Phase 5 CoV = 1.0% on both) → not a bad tooth or
flat bearing race. Uniform drag — most likely over-tensioned belt.

Fix if desired: loosen right track tension to AndyMark spec (~1-2 mm
deflection under firm finger pressure). Not required — per-wheel SparkMAX
PID absorbs asymmetry at normal setpoints, and Phase 5 showed both wheels
sync to within 5% under velocity mode.

### Coast-mode slow stop (mitigated)

Original SparkMAX idle mode was Coast → 170 RPM coasted for 9.47 seconds
through 23 rotations before stopping. Set idle mode to Brake via REV
Hardware Client → near-instant regen stop. Documented because this is
the first thing someone runs into when commissioning SparkMAXes for a
heavy drivetrain.

### Motor inversion — fixed

Mirror-mounted motors naturally spin in opposite ground directions under
positive duty. Inverted one SparkMAX via REV Hardware Client "Motor
Inverted" checkbox. Now `L+ R+` produces synchronized forward ground
motion on both tracks.

### Output range — had to verify

REV Hardware Client → PID Slot 0 → Output Range needs `-1.0` to `+1.0`
on both controllers. Was briefly suspected as the cause of a 2450 RPM
velocity-mode ceiling, but was actually full-range — the ceiling was
velocity-PID + Brake-idle interaction under high kFF.

## Critical findings documented elsewhere

- `firmware/teensy_diff_drive/CLAUDE.md` — firmware architecture, CAN protocol byte layouts
- `firmware/teensy_diff_drive/BRING_UP.md` — original 7-phase bench plan (still applies, status updated)
- `firmware/teensy_diff_drive/FINDINGS.md` — empirical phase 2-6 data summary

## Next steps

1. **Separate Jetson power** from motor rail (blocking field test)
2. Launch `sensors.launch.py` alongside `webui.launch.py` so `/imu/data` is publishing → heading-hold activates
3. Ground-drive test with heading-hold, tune `heading_kp` if drift or wobble
4. Phase 7 BURN persistence verification (manual power-cycle test)
5. Test full Nav2 stack (`navigation.launch.py`) once power is stable
6. Optional: loosen right track to recover the 8% asymmetric friction

---

## Update — PID fine-tune + burn + verification (same session, later)

### New scripts

- `firmware/teensy_diff_drive/phase6b_pid_fine_tune.py` — closed-loop tuner that fixes kFF from Phase 4 then sweeps kP (doubling until oscillation or ratio ≥ 0.95), then sweeps kI (tripling until ratio ≥ 0.99 or oscillation). Auto-validates across 500 / 1000 / 1500 / 2000 / 3000 RPM targets.
- `firmware/teensy_diff_drive/phase6c_pid_verify.py` — four rigorous precision tests: repeatability (5 trials at 1500), step response (overshoot + rise + settle), max-RPM tracking (4514 / 1.5 m/s target), sync precision (L vs R delta over a 4-s hold).

### Converged gains (burned to SparkMAX flash via cls=63 idx=15 PERSIST_PARAMETERS)

```
kFF = 0.000197
kP  = 0.0004
kI  = 9e-07
kD  = 0
```

These now persist across SparkMAX power cycles. The actuator_node yaml still pushes them on startup as belt-and-suspenders.

### Accuracy / precision results (Phase 6c)

| Test | Result | Verdict |
|---|---|---|
| Repeatability at 1500 RPM (5 trials) | L = 1479 ± 3.5 RPM (0.24%), R = 1477 ± 3.6 RPM (0.25%) | Excellent precision |
| Steady-state accuracy at 1500 RPM | L = −20.6 RPM (1.4%), R = −22.9 RPM (1.5%) offset | Minor — kI still converging over the 3 s window |
| Step response (0 → 1500) | 10.7% overshoot, ±5% settle at 338 ms | Acceptable |
| Sync at 1500 RPM | L-R mean delta = 12.4 RPM (0.83% of setpoint), max 40 RPM | Synced |
| Max-RPM tracking (cmd 4514) | L = 2992 (66%), R = 2996 (66%) | **Power-rail-limited** (see below) |
| Validation 500 / 1000 / 1500 / 2000 / 3000 RPM | 99 – 100% tracking on both | Excellent |

### Power rail confirmed as 1.0 m/s ceiling

Earlier, a sustained 4514 RPM command produced only 3000 RPM on both wheels. A diagnostic that sampled `STATUS_0` bus voltage every 0.3 s while commanding 4514 revealed:

```
t=0.00s  V=12.12  L=0     R=0       (idle)
t=0.30s  V=10.88  L=3309  R=3120    (sag starts immediately)
t=2.60s  V=8.51   L=617   R=1720    (voltage collapse)
t=3.00s  V=11.88  L=4326  R=3966    (brief recovery — near target!)
t=6.20s  V=9.48   L=2400  R=2343    (sag again)
```

The 48 V → 12 V buck cannot sustain the current required for two NEOs at ~90% duty. Mean RPM plateaus at ~3000 because half the time the motor is running on reduced voltage. Verified not PID: when voltage briefly recovered to 11.88 V, motors jumped to 4326 RPM — the PID is doing its job, the supply isn't.

**Practical effect**: max achievable ground speed on the current power architecture is **~1.0 m/s sustained** (3010 motor RPM), even though the firmware cap is 1.5 m/s and the motor+drivetrain math says 1.6+ m/s is possible. Fixing the shared power rail unlocks the remaining 50%.

### 15-second stability at 1.0 m/s — clean

`L3010 R3010` for 15 s continuous, 679 samples in steady state (t ≥ 1.5 s):

- Left  mean: 99.6% of commanded, **0 dips** below 90% of setpoint for >40 ms
- Right mean: 99.6% of commanded, **0 dips**
- V_bus held at 11.99–12.01 V throughout

The earlier brief R-track dip at t=3.03 s during a 5-second test (collapsed to 62% for ~1 s) did NOT reproduce in 15 seconds of sustained run — classified as a one-off transient (warmup, minor debris, or thermal), not a chronic binding fault. If it returns during actual field use, we'd do a hand-spin isolation test (chain vs belt vs bearing).

### Why there's residual ~1% error at 1500 RPM

P-only feedback cannot eliminate steady-state error. Our tuned kI = 9e-7 is intentionally conservative — higher kI caused oscillation during the sweep. The 1% offset at low targets is absorbed by Nav2's outer loop (RegulatedPurePursuit reads `/wheel_odom` and compensates). Not a priority to tighten further.

### Bench-test suite is now frozen at

```
phase1_handspin.py        manual, no-motor
phase2_baseline.py        automated — CAN + voltage health
phase3_stiction.py        automated — per-wheel minimum duty to start
phase4_duty_sweep.py      automated — duty vs steady-state RPM
phase5_stability.py       automated — periodic-dip + CoV at fixed cmd
phase6_pid_tune.py        automated — initial kP tuning (superseded by 6b)
phase6b_pid_fine_tune.py  automated — kP + kI closed-loop fine-tune (NEW)
phase6c_pid_verify.py     automated — repeatability + step + max + sync (NEW)
phase7_burn_verify.py     manual power-cycle step — BURN persistence
```

All scripts import the shared `teensy_bridge.py` helper. No firmware changes needed to run any phase.
