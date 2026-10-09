> **Status: superseded, firmware 25.0.4 results.** Repeated on firmware 26 in `FW26_BENCH_2026_09_28.md`.

# Isolated Bench Scopes for MPPI Readiness (2026-09-28)

These are the bench-now tests from `MPPI_READINESS_TEST_PLAN.md` §4, run on the robot with the tracks **off the ground**:
- Teensy firmware **v2b** (S and watchdog stop to idle).
- SPARK MAX FW 25.0.4.
- All setting changes are RAM-only, and the originals were restored after each scope (confirmed writes).

Scripts: `tools/bench/scopes.py` (serial scopes S0–S5) and `tools/bench/pipeline_scopes.py` + `run_pipeline_scopes.sh` (through actuator_node). Raw JSON on the Jetson: `~/bench_scopes_2026_09_28/`.

"Standard setup" in these tests means P 0.0003, I 0, IZone 0, kF 0.000197, Brake idle, 50/50 A, hall depth 3, unless stated.

**Measurement floor:** position-derived speed has about ±25 RPM of quantization (1 hall count = 1/42 rev; about 24 RPM at 1000 RPM in a 60 ms window). Ripple values below about ±30 RPM are at the floor.

## Summary against the MPPI readiness targets

| Scope | What was measured | Result | Target (plan §1.3) | Verdict |
|---|---|---|---|---|
| S0 clock map | Teensy µs vs Jetson arrival time | drift 91 ppm; arrival jitter ±2.8 ms | residual p99 ≤ 2 ms | **Marginal.** Use the Teensy timestamps (µs, from the CAN RX time), not host arrival |
| S1 timing | STATUS_2 interval, setpoint TX interval, CAN rates, serial RTT, watchdog | STATUS_2 20.2 ms (about 1 % at 40 ms); setpoint TX 20.0 ms (rare 40 ms skipped tick); CAN 203 tx/s and 214 rx/s, 0 queue or fail; RTT 5.2 ms (p99 5.9); watchdog 260 ms after the last host write | 50 Hz, no gaps > 100 ms, watchdog ≈ 300 ms | **Pass.** Note: `sdrop` = Teensy drops X telemetry lines when USB backs up (logging only) |
| S2 baseline | all 31 table parameters + FW version | recorded in `s2_manifest_baseline.json` (Coast, 80/20 A, P 0.0007, I 2.5e-7, IZone 600, kF 0.000197) | — | Baseline recorded |
| S3.1 feedforward only (P = I = 0) | delivered ÷ commanded, both directions | 300 RPM: **0.80**; 1000: 1.02–1.03; 2000: 1.06–1.08; 3500: 1.06–1.08. Unloaded, the effective duty per RPM is 0.000181–0.000191 plus a friction offset of about 0.015 duty | FF within ±5 % (T2) | **Off-ground only:** high by 6–8 % above 1500 RPM and low by 20 % at 300. Refit on the ground (S3.2) |
| S3 speed loop, P 0.0003 | steady error and ripple, 300–3500 RPM, both directions, plus differential commands | 1000–3500 RPM: +1.3 to +3.5 %; **300 RPM: −5 to −7 %**; ripple ±30–100 (at the floor to 3–4× it) | ±2 % (T1) | **Not met yet** off the ground (the FF is too high unloaded, the low end is friction). Recheck on the ground |
| S3 speed loop, P 0.0002 | same | 1000–3500: +0.3 to +4.2 %; 300: −9 to −11 % | ±2 % | Worse at the low end |
| S3 instant steps (Teensy ramp off) | overshoot of 0→1000, 1000→2000, reversals | **85–135 % overshoot** with instant setpoint steps; the motor reaches the new speed in 20–40 ms unloaded | ≤ 5 % (T4) | **Instant steps are unusable.** Setpoints must be ramped (actuator slew or Teensy M); the earlier ramped tests gave 9–21 % |
| S3.7 hall filter depth × P | lag, ripple, stability | lag **152 ms** (depth 3), **72–84 ms** (depth 2), **32–52 ms** (depth 1). Stable combinations: depth 3 with P ≤ 0.0003; depth 2 with P ≤ 0.0003; depth 1 with P ≤ 0.0002 | measurement lag ≤ 50 ms | **Depth 1 + P 0.0002 meets it** (32 ms, stable). Depth 2 + P 0.0002 is the conservative option (72 ms) |
| S3.8 low speed (P 0.0003) | mean speed and stick fraction, 50–300 RPM | 50 RPM: stuck 27–57 % of the time; **100: −30 %**; **150 (0.05 m/s): −18 %**; 300: −7 % | T7: 0.05 m/s with no stick-slip | **Fail.** Needs static-friction feedforward (the FW 25 option is arbitrary FF in the setpoint frame) or a small bounded I |
| S3.9 saturation | 4600 RPM command | delivered 4725 (+2.7 %), duty 0.88 (12 % headroom) | reachable with headroom | Pass. **But a Brake stop from 4600 RPM set a DRV sticky fault on R** |
| S5 hop latency | host → Teensy ack; host → first CAN setpoint; setpoint → motion; cmd_vel → setpoint | 5.2 ms; 5–15 ms (20 ms tick); 1–21 ms (one status period); **cmd_vel → actuator setpoint 4–18 ms (p50 8)** | command-to-motion ≤ 100 ms (T8) | **Pass**, about 10–55 ms end to end at the motor layer |
| S4b ω flips (pipeline) | +0.8 → −0.8 rad/s | no oscillation (0 sign changes), **1.1 s** to reach the new direction, set by the 1.2 rad/s² actuator slew | 0 reversals | Pass on oscillation. The slew time is an MPPI model-mismatch item (plan finding) |
| S9 odometry (pipeline) | /wheel_odom rate, gaps, standstill noise, twist lag | **20 Hz** (header says 50), gap max 52 ms, standstill noise 0; **twist lags position by 130–170 ms**, rms error 0.18 m/s during speed changes | ≥ 20 Hz and **lag ≤ 50 ms** | **Fail on lag** (see below) |

## Key findings

1. **The odometry speed that MPPI sees is 130–170 ms old.** actuator_node publishes `/wheel_odom` from its 20 Hz state timer and computes the twist from the SPARK's *filtered* speed report (hall depth 3, about 150 ms), not from wheel position (`actuator_node.py` `_publish_state` → `_publish_odom`). EKF v comes only from this, so MPPI's rollouts start from a stale speed.
   - Remedy options (code change, not applied):
     - (a) compute v from E-line position differences at 50 Hz in actuator_node and publish at 50 Hz;
     - (b) hall depth 1 with P ≤ 0.0002 (32 ms measured);
     - both.
2. **Low speed is under-delivered** (−18 % at 0.05 m/s, sticking at 50 RPM). FW 25 has no kS parameter; the standard remedy is static-friction feedforward. It can be sent as the setpoint frame's arbitrary feedforward (spec: bits 32–47, ×0.0009766, volts or duty), which needs a small firmware + host change, or a small I with IZone and max-accumulator bounds.
3. **Setpoints must be ramped.** Instant steps overshoot 85–135 % because P acts on a lagged measurement. actuator_node's slew does this in normal operation. Test tools must use `M100` or the actuator path, never `M10000`, for step metrics.
4. **Stopping from high speed straight into Brake can trip DRV** (R, from 4600 RPM). In normal operation actuator_node decelerates first. A watchdog stop at full speed would brake hard, so consider it for the watchdog path (e.g. keep Coast for the watchdog, or ramp).
5. **Timing and latency layers are healthy.** The 50 Hz loop holds, CAN has no errors, and command-to-motion is well under 100 ms. The delay MPPI can't see comes from the actuator slew caps (plan finding) and the odometry lag, not from the firmware or CAN.

## Follow-up: static-friction feedforward (firmware v2c), bench proof
FW 25 has no kS parameter, but its velocity setpoint frame carries an **arbitrary feedforward** field (REV spec, frame implemented since 25.0.0). v2c sends `kS × sign(target)` in it, per side (`KSL/KSR`, volts). Slow-speed scope, P 0.0003, off the ground:

| Command (RPM) | kS = 0 (L / R) | **kS = 0.18 V** (L / R) | kS = 0.30 V (L / R) |
|---|---|---|---|
| 50 | 15 / 17, **stuck 19–24 %** | **49 / 53, never stuck** | 69 / 72 (+40 %) |
| 100 | 66 / 69 (−32 %) | **97 / 100** | 120 / 122 (+21 %) |
| 150 (0.05 m/s) | 121 / 123 (−19 %) | **152 / 153** | 172 / 174 (+15 %) |
| 300 | 277 / 279 (−7 %) | **307 / 308** (+2.5 %) | 326 / 330 |
| −100 | −68 / −70 | **−100 / −104** | −121 / −125 |

**The mechanism works:** 0.18 V removes the sticking and brings 50–150 RPM within about 3 %. Too much kS (0.30 V) overshoots, so **kS must be measured on the ground, per side** (Session 1 step B fits it). The off-ground value is not the field value.

## Still to do
- **Decide and persist the configuration (S2 final):** Brake + 50 A, P, I, hall depth, then BURN, power-cycle and diff.
- **Code changes to evaluate:**
  - position-derived 50 Hz wheel odometry;
  - kS via arbitrary feedforward;
  - the watchdog stop mode from high speed.
- **Ground scopes:** S0b/c, S3.2 ground FF identification, S4 stopping distance, S6 electrical soak, S7 delivery (asphalt, then grass), S8 model identification and horizon replay, S9 on the ground, S10 end-to-end with heading-hold A/B.
