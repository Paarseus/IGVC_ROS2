# Tuning Test Plan: Accurate Tracks Before MPPI (from 2026-10-02)

Continues `TUNING_LOG.md` (baseline, experiments E0–E4a). This plan fixes **what will be tested, how many times, what counts as a pass, and when to stop**, before any further run.

## 1. Why more work, and what "accurate" means here
Tight passes leave about 35 cm per side; the drive stack may use at most about 10 cm of that (`GROUND_TEST_PLAN.md` §1.3). From the measured errors:

| Source | Effect over MPPI's 2.8 s look-ahead |
|---|---|
| left/right speed mismatch going straight | 1 cm per 1 % at 0.7 m/s (worst measured today: 2.2 %, about 2 cm) |
| turn-rate error | about 13 cm per 10 % |
| speed error | along the path (3 % at 0.7 m/s = 6 cm) |
| slow maneuvering near obstacles | −5 to −20 % at 0.05–0.1 m/s forward today |

Targets (fixed now, before testing):

| ID | Condition | Target |
|---|---|---|
| A1 | straight, 0.3–0.7 m/s, each track, each direction | within **±2 %** |
| A2 | straight, 0.1–0.2 m/s | within **±4 %** |
| A3 | straight, 0.05 m/s | within **±10 %**, never stuck |
| A4 | left/right match at 0.1–0.7 m/s | within **1.5 %** |
| A5 | forward minus reverse difference (mean of tracks) | within **1.5 %** |
| A6 | spins (0.3 and 0.6 rad/s) | within **±10 %** (the multiplier absorbs the rest); 0.1 rad/s reported only |
| A7 | speed noise | no more than 1.5× the E0 baseline; no oscillation |
| A8 | stops | rollback at most 5 mm, no reversal |
| A9 | electrical | bus at least 9.5 V, current at most 45 A, no controller fault |
| A10 | ground truth | distance on the ground matches the encoder distance within 1 % (tape) |
| A11 | repeatability | the same result in a fresh session and after an e-stop power cycle |

## 2. What the research and the data say (and what changes)
| Finding | Source | Consequence for the plan |
|---|---|---|
| Tune the feedforward first; "systems that seem to require integral control probably have an inaccurate feedforward model"; REV/WPILib avoid I | C1-S16, S17, S04 | **Refine the feedforward model first; integral action is a secondary, bounded experiment** |
| Friction compensation belongs in feedforward; identify it in both directions and at several speeds | C1 practices 6–7; C1-S32 (Coulomb friction differs by direction) | Fine speed grid, both directions, each track on its own |
| Static friction at zero speed can only be removed by integral action; integral action causes limit cycles at low speed and at reversals | C1-S31, S26, S40 | I only with a narrow zone and a small cap; test reversals and stops explicitly |
| Windup mitigation: lower kI, I-zone, cap on the accumulator | C1-S17, S08, S10 | the E4a mistake was a 700 RPM zone; the new tests use 80 RPM |
| 3 repeats resolve only large effects (43 / 11 / 5 runs for 0.5σ / 1σ / 1.5σ shifts) | X2-S11 | 3 repeats to screen, **5 to decide** |
| Today's data: run-to-run noise σ about 0.3 % at speed, 3–6 % at crawl; forward-minus-reverse gap is a constant in RPM; the forward crawl is non-linear (E1 straight-line fit predicted −5 %, E2 measured −30 %) | E0–E3a | A straight-line fit is not enough at low speed: a **speed-dependent correction** is needed |

## 3. Principles
1. **One factor at a time, from a named reference.** Everything else is read back and logged.
2. **Bracketing.** The reference set is measured before, between and after the candidates (A–B–A), so drift (battery, temperature, surface) is visible. A candidate is compared with the adjacent reference, not with a number from an hour ago.
3. **Pre-registered decisions.** A change is adopted only if it improves its target metric by more than twice the pooled 95 % band, worsens nothing else by more than 1 %, and all guards stay clean.
4. **Fit and validate on different runs** (repeats 1–3 fit, 4–5 judge).
5. **Verify what the controllers actually hold.** The harness has so far checked the Teensy's reply, not the SPARK's own value; the plan reads the parameters back from the SPARKs (`PR`) at the start and end of every set and after any power cycle.
6. **Never trust an automatic retry after a human action.** After an e-stop the set aborts and waits.

## 4. Tools to build and test first (software only, robot not needed)
| # | Tool | Why |
|---|---|---|
| T1 | **Live in-run guard** in `drive_tuner.py`: stop the run within one tick on bus < 9.0 V, current > 48 A, or speed oscillation (velocity reversal against the command, or duty sign flips) | E4a oscillated for 11 s before the post-run guard could act |
| T2 | **SPARK read-back** (`PR B kP/kI/kV/kIZone/kIMaxAccum`) at set start, set end and after e-stop; mismatch aborts | proves the SPARKs hold the intended values |
| T3 | Log the **integrator accumulator and setpoint** (STATUS_7/8, already decoded by the firmware) in `teensy.csv` | makes windup visible |
| T4 | Analysis: paired comparison against the bracketing reference (difference, band, decision), windup / reversal / break-away metrics, per-track and per-direction summaries | the decision rule in code, not by eye |
| T5 | **Correction-table fit and validation** (`analyze.py maptrim`): from delivered-vs-commanded data to per-track, per-direction setpoint multipliers with held-out error | Phase C |
| T6 | Synthetic tests for each of the above (known truth), as for the existing analyses | no tool is trusted before it recovers a planted answer |

## 5. Phases

### Phase 0: session start (every session, 10 min)
Resting bus voltage ≥ 12.0 V; `CHK OK`; SPARK read-back equals the reference set; IMU/temperature logged; same surface and start mark; **180° check**: one forward and one reverse cruise run in each of two headings (robot turned 180°): if the forward–reverse gap flips with the heading it is slope; if it stays with the robot it is the drivetrain (decides whether A5 can be fixed at all).

### Right-track check (add to Phase 0, off the ground, 10 min)
The right track has been the weaker one in every record (about 8–10 % more friction since May; forward delivery −1.4 to −3.1 % against the left's −0.5 to −1.0 %; noisier speed; stuck more often in spins; the only controller fault flag today was on the right; a right-motor bearing failed in the 2026-05-13 high-P test). Before more tuning: power off and turn each track by hand (feel, noise, belt tension and alignment); then, tracks lifted, drive both at the same voltage (2, 4, 6 V ramp) and compare speed and current. If the right draws more current for the same speed off the ground, the cause is internal friction (bearing, gearbox, belt) and should be fixed mechanically; if not, it is load on the ground. Record the result in `TUNING_LOG.md`.

### Phase B: friction and delivery map (identification, no new control terms, 25 min)
Reference set R0: kV 0.00211, kS 0.40 / 0.39, kP 0.0004, kI 0.
| Test | Runs | Metric |
|---|---|---|
| Fine speed grid forward and reverse: 0.03, 0.05, 0.075, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7 m/s | 5 repeats, runs split to stay within about 4 m | delivered speed per track vs command (the map) |
| Break-away: ramped start 0 → 0.05 and 0 → 0.1 m/s | 5 each direction | time to first motion, overshoot |
| Slow-speed hold, 0.05 m/s, 10 s | 3 each direction | stuck share, ripple (stick-slip) |
Output: error map per track and direction with 95 % bands.

### Phase C: feedforward correction (the main fix, 40 min)
1. Fit the correction from repeats 1–3: a per-track, per-direction multiplier on the **setpoint** as a function of speed (interpolated table, about 8 points), plus a break-away boost below 0.1 m/s if the data show a step.
2. Implement it **in `actuator_node`** (host side; the firmware is not changed), configured in yaml, off by default, with a unit test.
3. Validate on repeats 4–5 **and** a fresh 5-repeat set through the real path (`/cmd_vel` → `actuator_node` slews, heading-hold off).
Pass: targets A1–A5 and A7–A9. Fail: fall back to Phase D, or accept the measured limits.

### Phase D: bounded integral term (secondary; only if C leaves A2/A3/A5 open; 30 min)
First validate the integrator itself **off the ground** (tracks lifted): confirm the accumulator grows at `error × kI × dt`, resets when the error exceeds the zone, and stops at the cap (using a deliberately wrong kV to create a known error). Then on the ground, one factor at a time, ascending, abort at the first bad run:
| Set | kI | zone | cap | reps |
|---|---|---|---|---|
| D1 | 3e-5 | 80 RPM | 0.03 | 3 |
| D2 | 1e-4 | 80 | 0.03 | 3 |
| D3 | best of D1/D2 | 40, 80, 150 (one at a time) | 0.03 | 3 |
| D4 | best | best | 0.015, 0.03, 0.06 | 3 |
Extra tests for any I > 0: windup after a long sticky hold then stop; reversal +0.3 → −0.3 m/s; break-away; 60 s hold on lifted tracks.
Adopt only if it beats Phase C on its target metrics **and** passes A7/A8 with margin.

### Phase E: confirmation (45 min)
The final set, from a **cold start through the deployed path**: power-cycle the drivers; start `actuator_node` with the yaml (so the values come from the repository, not the harness); then
1. 5-repeat full protocol (straight, spins, stops) bracketed by the R0 reference;
2. **ground truth**: 5 m runs at 0.3 and 0.7 m/s forward and reverse measured with the tape (start and end marks, along and sideways), 5 repeats: encoder distance vs ground distance (A10);
3. reversal (+0.5 → −0.5 m/s ramped), restart after an e-stop (gains re-pushed, behaviour unchanged), stop behaviour after 60 s of mixed driving;
4. electrical summary (bus, current, motor temperature) over the whole session.

### Phase F: adoption
Update `actuator_params.yaml` and `CLAUDE.md`; save the controller values (BURN) **only with your approval**; reboot test (values come back); results file `results/GROUND_<date>.md`; log entries for every experiment.

## 6. Statistics and decision rules
- Run-to-run σ today: about 0.3 % at ≥ 0.3 m/s (band ±0.1–0.6 % with n = 2–3), 3–6 % at crawl. **n = 5 gives a 95 % band of about ±0.3 % at speed and ±3–5 % at crawl**: enough to certify ±2 % and ±10 %.
- Paired comparison: candidate vs the adjacent reference, per track, direction and speed. Adopt only if the improvement exceeds twice the pooled band.
- No outlier is dropped without a written reason (e.g. e-stop: run void, set aborts).
- Report mean ± band, worst run and n for every cell.

## 7. What could fool us (and the control for each)
| Confounder | Control |
|---|---|
| battery state and temperature | bracketing, bus and temperature logged every run |
| sidewalk slope | the 180° check; forward and reverse always paired; alternating directions |
| surface (concrete now; asphalt and grass later) | every number is labelled with its surface; Phase E repeats on asphalt later |
| gains reverting after a power cycle | SPARK read-back (T2) and the counter-reset guard |
| the harness vs the deployed path | Phase E runs from the yaml through `actuator_node` |
| encoder measures motor, not ground | tape ground truth (A10) |
| operator intervention | any e-stop voids the run and aborts the set |

## 8. Safety protocol
Operator at the e-stop, about 5 m clear for straight runs, nobody in the spin area. Guards: live in-run (T1) and post-run (bus 9.5 V, current 45 A, ripple 60 RPM, fault flags, counter reset). Any abort restores the reference gains and waits for a human. No firmware change; the emergency stop and watchdog are untouched.

## 9. Schedule (robot time)
Tools T1–T6: about 1 h, no robot. Phase 0: 10 min. B: 25 min. C: 40 min. D (if needed): 30 min. E: 45 min. Total about 2.5 h on the ground, in two sessions, with one battery swap or charge in between.

## 10. Decision points
1. After B: is the error repeatable enough (bands) for a correction table to work? If not, stop and report the limits.
2. After C: do A1–A9 pass? If yes, skip D.
3. After D (if run): integral adopted or rejected.
4. After E: ready for the turn calibration, or back to C.
