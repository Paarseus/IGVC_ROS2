# 01 — Motor Control Layer (SPARK MAX, NEO, Teensy)

Sources: firmware `evidence/deployed_snapshot/firmware/teensy_diff_drive/teensy_diff_drive.ino`, tuning history in `firmware/teensy_diff_drive/FINDINGS.md`, `docs/motor_sync_logs/REPORT.md`, `docs/skid_steer_kinematics_findings_2026_05_18.md`, external research `external/01_sparkmax_neo_research.md`.

## 1. Configuration

| Item | Value | Where set |
|---|---|---|
| Controller | SPARK MAX FW 26.1.4, onboard velocity PID, 1 kHz loop | device |
| Gains, slot 0 | P 0.0007, I 2.5e-7, D 0, param 16 = 0.000197, IZone 600 RPM | pushed by `actuator_node` at start-up; also in flash |
| Encoder | NEO hall, 42 counts/rev, default velocity filter (32 ms period, 8-sample average) | device default, never changed |
| Status | STATUS_2 (velocity + position) re-enabled every 1 s by the Teensy, default 20 ms period | firmware |
| Heartbeat | universal 0x01011840 at 50 Hz + REV secondary | firmware |
| Setpoint ramp | 100 RPM per 20 ms tick | firmware |
| Clamp | ±4600 RPM (1.53 m/s) | firmware |
| Idle mode | Brake | device (REV Hardware Client) |
| Current limit, ramp rate, voltage compensation, output range | **not verified** | device (never read back) |
| Supply | shared 48 V → 12 V buck with the Jetson | hardware |

## 2. Measured behavior

| Quantity | Value | Evidence |
|---|---|---|
| Command → real wheel motion delay | 15–45 ms | `evidence/live_measurements/velocity_lag_2026_05_logs/RESULT.txt` |
| Reported velocity lag behind real motion | 185 ms | same |
| Steady-state delivery, current gains | 93–96% translation, ~85% rotation | 2026-05-18 findings |
| Overshoot, current gains | 0–9% linear, 8–14% rotation | CLAUDE.md, 2026-05-18 |
| L/R match, wheels up | 0.43% over 73 s | motor sync report |
| Repeatability at 1500 RPM | ±0.25% (5 trials) | FINDINGS.md phase 6c |
| Sustained ceiling on current power system | ~3000 RPM (≈1.0 m/s) under load; bus sags 12 → 8.5 V | FINDINGS.md |

## 3. Findings

**M1 — The feedforward parameter is probably in the wrong units (Accuracy, Critical, needs a bench test).**
On SPARK MAX firmware 2026, parameter 16 is `kV` in **volts per RPM**; it was previously `kF` in duty per RPM (REVLib 2026 changelog; REV's own template changed `1/freeSpeed` to `12/freeSpeed`; see `external/01`). Our value, 0.000197, was chosen as a duty/RPM value (≈ 1/5076). Read as V/RPM it supplies **9%** of the feedforward needed (theoretical 12/5676 = 0.00211 V/RPM).

The team's own delivery data fits the volts reading and not the duty reading. Steady-state model with proportional control only, unloaded motor, 12 V: speed ≈ duty × N with N = 5676 RPM, so delivery = (f·N + kP·N) / (1 + kP·N), where f = 0.000197/12 (volts reading) or 0.000197 (duty reading):

| kP | Predicted delivery, param 16 as V/RPM | Predicted delivery, as duty/RPM | Observed |
|---|---|---|---|
| 0.0004 | 72% | 104% | ~70% (issue #6), 64% (motor sync, unloaded) |
| 0.0008 | 84% | 102% | 83% (2026-05-13 session) |
| 0.0007 (current) | 82% before the integrator acts | 102% | 93–96% with kI after settling |

Consequences if confirmed:
- Proportional and integral action currently carry almost the whole load. That explains the chronic under-delivery (issue #6), the need for a relatively large kP, and the overshoot at speed transitions: the integrator winds up through the measurement lag (M2) and releases when the setpoint stops changing.
- The 15% rotation "motor delivery loss" that the 1.19 skid multiplier compensates (02, D7) is most likely this same deficit under higher load.

Test (V2 in 08): kP = kI = 0, param 16 = 0.000197, command 2000 RPM → expect ~9% delivery. Then param 16 = 0.0021 → expect 90–100%.
Fix: per-track kV from measured free speed (≈ 12/5532 = 0.00217 left, 12/5072 = 0.00237 right) or from a SysId fit, add kS (param 204) for static friction, then re-tune P and reduce I. Update the unit labels in CLAUDE.md and `actuator_params.yaml`.

**M2 — Velocity feedback is 185 ms late (Control and odometry, High).**
Measured on both tracks (V1). The default NEO hall filter (32 ms period, 8-sample average) accounts for ~112 ms (Veness; WPILib formula d = T(n−1)/2). The STATUS_2 period and E-line sampling account for part of the rest. This delay affects:
- The onboard PID, if it uses the same filtered signal (widely assumed, not confirmed by REV). The stable kP is limited and integral action overshoots.
- Wheel odometry, which integrates this velocity (03, O1).
Teams using NEOs for velocity control commonly set a 16 ms period with depth 2 (Team 3005, Team 620). Fix: shorten the filter (params 136/137), then read back to confirm the encoding and re-run V1 (target < 40 ms).

**M3 — Device parameters are not verified or managed in code (Robustness, High).**
The firmware writes only P, I, D, param 16 and IZone. Current limits (default 80 A stall / 20 A free), closed-loop ramp rate, voltage compensation, output range and encoder filter depend on whatever is in each controller's flash. The 2026-05-12 follow-up checklist to verify them (`FINDINGS.md`, Rank 3) is unchecked. Two further gaps:
- The firmware notes that `BURN` currently returns result 255 (rejected). CLAUDE.md says the gains were burned via REV Hardware Client instead. The flash contents have not been read back since.
- After a SPARK MAX brownout or reset, non-persisted parameters revert to flash values (REV documentation). `actuator_node` pushes the gains only at start-up, so a mid-run brownout silently changes the gains until the node restarts.
Fix: audit both controllers (REV Hardware Client, or `sparklib-py` over SocketCAN), record the values, and have the Teensy write the full parameter set on every SPARK reset (detect via STATUS_1 HAS_RESET_WARNING).

**M4 — Supply voltage limits speed and is not monitored in ROS (Robustness, Medium).**
The motor rail is shared with the Jetson and sags from 12 V to 8.5 V under sustained load; at that point both tracks plateau near 3000 RPM. Bus voltage is decoded on the Teensy but only reported on demand (`D`), and STATUS_1 brownout and reset flags are not decoded. REV recommends a 40–60 A current limit for NEO drives, which also reduces inrush on the shared rail.
Fix: add bus voltage and fault flags to the E-line and to `/avros/actuator_state`, and set and persist a current limit.

**M5 — Setpoint ramp and safety logic (Correct).**
The Teensy ramps setpoints (100 RPM per tick ≈ 1.66 m/s² at the track), which is the recommended place for rate limiting; the output ramp (param 114) winds up the integrator instead. The 300 ms watchdog bypasses the ramp for an immediate stop. The heartbeat matches the WPILib 20 ms period. STATUS_2 is re-enabled every second, which survives controller resets.

**M6 — E-lines carry no time or sequence information (Timing, Low).**
Feedback lines have no Teensy timestamp or counter, so the host cannot measure feedback age or detect dropped lines. The `Serial.availableForWrite() >= 64` guard can silently drop lines under USB back-pressure. Fix: append `millis()` and a sequence number.

## 4. Assessment
The motor layer is precise: L/R tracking within 0.4%, trial-to-trial repeatability within 0.25%, a clean watchdog and heartbeat design. It is not yet accurate by design. Delivery depends on P and I rather than feedforward, which is most likely caused by the firmware-2026 unit change (M1), and feedback is 185 ms late (M2). Fixing M1 and M2 is the highest-leverage change in the stack, because the skid multiplier, the overshoot and the odometry timing all trace back to them.
