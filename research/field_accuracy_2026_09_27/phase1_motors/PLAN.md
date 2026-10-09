# Phase 1: Motors and PID Plan

**Why this matters for autonomy:**
- The navigation controller (MPPI) plans 2.8 s ahead, assuming the tracks deliver exactly the speed and turn rate it commands.
- Any gap between commanded and delivered motion becomes position error the controller only sees after it has happened.
- In a 1.524 m gap the robot has 0.35 m of room per side. The motors must do what they're told, quickly, repeatably, and the same on both tracks.

**What went wrong before:**
- Gains were adjusted until overshoot looked acceptable. There was no target, no model of the motors, and no test run the same way each time.
- The main error (feedforward about 11× too small, from a unit mismatch; report 01) was hidden, because the integrator made up for it slowly.
- This plan fixes the order: **measure the motors first, set feedforward from the measurements, then add only as much feedback as needed**, all checked against fixed targets.

## 1. Targets (derived from autonomy needs)

| # | Target | Value | Why (autonomy link) |
|---|---|---|---|
| T1 | Steady speed accuracy, each track, 0.1–1.0 m/s | within **±2 %** | 2 % speed error over MPPI's 2.8 s plan at 0.7 m/s = 4 cm |
| T2 | Accuracy **without** the integrator (feedforward + P only) | within **±5 %** | Integrator-driven accuracy is slow (seconds); feedforward is instant |
| T3 | Response to a speed step (after the host ramp) | 95 % in **≤ 0.4 s** | MPPI replans every 50 ms; slow response = constant lag |
| T4 | Overshoot | **≤ 5 %** | Overshoot near an obstacle closes clearance |
| T5 | Left/right mismatch at the same command | **≤ 1 %** | 1 % mismatch = about 0.7° heading drift per metre on straight lines |
| T6 | Turn-rate delivery (in place and arcs), on grass | **0.97–1.03** of commanded | Wrong turn rate = wrong heading entering gaps |
| T7 | Slowest controllable speed | **0.05 m/s** straight, **0.1 rad/s** turning, no stick-slip | Fine positioning at gaps and waypoints |
| T8 | Command-to-motion delay | **≤ 100 ms** | Every 100 ms of delay at 0.7 m/s = 7 cm of unplanned travel |
| T9 | Stopping from 0.7 m/s | **≤ 0.25 m**, no reverse kick > 5 mm | Predictable stopping distance for obstacle margins |
| T10 | Repeatability (same test, 3 runs) | spread **≤ 1 %** | Tuning is meaningless if results vary |
| T11 | Battery sensitivity (12.2 V vs about 11.5 V) | speed change **≤ 2 %** | Behaviour must not change as the battery drains |

## 2. Steps

### Step A: Instrument and check (15 min)
- Confirm logging of wheel positions at 50 Hz (`/avros/wheel_debug`), IMU, and RTK position.
- Velocity is always computed from **encoder position**, never from the SparkMAX's reported speed, which lags about 185 ms.
- **Measure the speed-reading lag directly** (no settings change): during the Step B power steps, log both the SparkMAX's reported speed and the speed computed from encoder position. The time shift between the two curves is the real measurement lag. Compare it with the documented 112 ms (default filter: 8 readings, 32 ms apart; see `research/topics/C1_motor_velocity_control/`). The difference from the 185 ms seen end-to-end is the CAN and Teensy reporting delay.
- Record battery voltage at the start and end of every test (Teensy status line).
- Robot on its tracks, on the competition-like surface, heading-hold **off**.

### Step B: Measure the motors (open loop, 20 min)
No PID. Command fixed motor power (duty) and measure what happens:
1. **Breakaway power (kS):** slowly raise power on each track until it moves. Earlier bench data: right track needs about twice the left (0.06 vs 0.03).
2. **Speed per volt (kV):** hold 4–5 power levels, measure steady speed on each track. The slope gives kV per track (expected about 0.0021 V/RPM from report 01).
3. **Time constant:** step the power, measure how fast each track reaches steady speed.
4. **Turning load:** repeat 2 while turning in place. Tracks scrape sideways, so turning needs more power.

Result: a simple model of each track (breakaway, speed per volt, response time). All gains are then calculated from it, not guessed.

### Step C: Feedforward (15 min)
- Set kFF from the measured kV (in the units the SparkMAX expects, volts per RPM).
- Check T2 with P and I at zero: speed within ±5 % across 0.1–1.0 m/s.
- The Teensy firmware sends one kFF to both motors. If the tracks differ by more than 5 %, note it: per-track values need a small firmware change later.

### Step D: Feedback (20 min)
- **P:** raise from low until T3 is met, stopping before overshoot exceeds T4. The 185 ms speed-measurement lag limits how high P can go (kP 0.0008 already oscillated before).
- **I:** add only enough to meet T1, with a small integrator zone, so it trims the last few percent instead of doing the work.
- **Optional:** shorten the SparkMAX speed-measurement filter to reduce the 185 ms lag. That allows a higher P and a faster response. It's a SparkMAX setting; try only if T3 isn't met.

### Step E: Full check against targets (30 min)
Run the fixed test set 3 times each, forward and reverse:
- speed steps 0.1 / 0.3 / 0.5 / 0.7 / 1.0 m/s (T1, T3, T4, T10)
- 10 m straight lines (T5)
- turns in place at 0.1 / 0.3 / 0.6 rad/s, and arcs (T6, T7)
- crawl at 0.05 m/s (T7)
- stops from 0.4 and 0.7 m/s, plus E-STOP (T9)
- command-to-motion delay from the step logs (T8)
- repeat the 0.5 m/s step at the end of the session, at a lower battery voltage (T11)

### Step F: Lock in (10 min)
- Write the final gains to `actuator_params.yaml`. `actuator_node` sends these to the SparkMAXes at every start, so this file is the real source of truth.
- Save the gains to SparkMAX flash (`BURN`) as a backup.
- Re-measure the skid multiplier (currently 1.19). It was partly compensating for the weak feedforward and should drop once feedforward is right.
- Write the results table: target, measured, pass/fail.

## 3. Keeping future autonomy in mind
- **Give the navigation controller the real numbers.** The measured response time, acceleration and turn delivery become MPPI's settings (speed limits, sampling spread, turn rates), so it plans with what the chassis actually does.
- **One repeatable test.** The Step E test set becomes a script. Run it after any hardware change (battery, tracks, motors, firmware) and before competition.
- **Surface-specific values.** Grass and asphalt behave differently (skid multiplier, breakaway power). Measure both; the competition is on asphalt.
- **Known limits to fix later, in code:**
  - per-track feedforward (needs firmware)
  - command priority between the phone and navigation
  - heading-hold overriding small turns from the navigation controller
  - stop behaviour: the SparkMAX now actively holds 0 RPM instead of braking

## 4. What we need
- A clear, flat area of about 15 × 15 m, grass (and asphalt if possible).
- One person on the E-STOP at all times.
- About 2 hours including setup.
