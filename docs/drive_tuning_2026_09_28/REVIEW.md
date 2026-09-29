> **Status: historical.** Independent review of the first version of STRATEGY.md and the tools (2026-09-28, firmware 25). Its fixes are included in the current tools.

# Review: drive-tuning strategy and field tools (2026-09-28)

**Scope:** `STRATEGY.md`, `tools/drive_tuner.py`, `tools/analyze.py`, checked against `firmware/teensy_diff_drive_v2/PROTOCOL.md` (and the `.ino` printf and parser), the research topics C1–C4, L1–L3 and X2, the firmware review `REPORT.md`, `actuator_params.yaml`, `xsens.yaml`, the URDF and the Xsens driver source in `src/xsens_mti/`.
**Method:** I read each item against the cited Recommended-practice or finding and wrote a synthetic generator (`sim.py`, in the session scratchpad, not committed). It produces firmware-shaped X lines: per-side STATUS_2 at 20 ms with independent phase, 1/42-rev hall position quantisation, a 32 ms × 8 hall filter and a kS/kV/kA motor. It also produces RTK fixes (antenna 0.74 m ahead, 1 cm noise, 100 ms latency) and a biased gyro. Every analysis function was run on this data and compared with the known truth. I also drove `drive_tuner.py` against a fake Teensy on a pty, including a mid-run disconnect. `python3 -m py_compile` passes on both tools.

## 1. Summary

- **Strategy:** sound. The order motor → distance → turning is correct and needed: in the synthetic turn test, a 2.7 % m_per_rev error became a 2.7 % multiplier error, because width = Δv/ω scales 1:1 with m_per_rev. Open-loop identification before feedback, per-side and per-direction fits, on-ground grass tests, independent references and read-back before BURN all match the references.
- **Strategy gaps.** I fixed the clear errors in place: the r² criterion, the MPPI accel-limit claim, T3 through the slewed pipeline, and the missing gyro-bias and RTK gates. The rest are listed below.
- **Tools: one blocker in `analyze.py`, now fixed.**
  - The firmware prints an X line whenever *either* side's STATUS_2 arrives, so each side's timestamp repeats on alternate rows.
  - `np.gradient` on the repeated times gave NaN for every sample, so **`analyze.py ff` printed no fit at all** on real-format data.
- **Two biased formulas in `analyze.py`, also fixed:**
  - **kS/kA regression form.** Regressing V on a noisy, twice-differentiated acceleration gave kA 21–50 % low and kS up to 20 % high.
  - **Step overshoot noise floor.** Hall quantisation read as a spurious +4 to +5.5 % overshoot on a response that has none (T4 limit 5 %).
- **Three smaller analysis issues, fixed:**
  - the ENU radius (0.2 % scale bias, against the 1 % K1 budget);
  - the lateral-drift metric (biased +2.5 σ of RTK noise, about +3 cm against the K2 5 cm);
  - no gyro-bias removal.
- **`drive_tuner.py`:** protocol use is correct (X-line indices, UVL/UVR, MD, S, keepalive).
  - A serial error could escape the teardown and leave the terminal in cbreak mode. Fixed.
  - Added: a field-time check that ROS data is arriving, and an MD-reply check that stops a v1 firmware from silently accepting `MD` as `M0` (slew = 0).
- **Readiness:** ready for a field session **after** the firmware bench check. The plan is about 195 min with no buffer, so it will not fit in 3 h (section 5).

## 2. Item-by-item

| Item | Reference (topic + finding/practice #) | Verdict | Note |
|---|---|---|---|
| P1 signal-chain order motor → distance → turning | C3 practice 2 (inner first); C3 §2; C2 practice 3 | Correct | Turn width scales 1:1 with m_per_rev (shown on synthetic data) |
| P2 open-loop ID first; closed-loop data biases naive fits | C1 practice 1, 9, 15, 18; C1-S44 | Correct | |
| P3 independent reference per quantity | C2 practice 4; L1 practice 8 | Correct | L1 practice 8 (gyro for short heading disturbances) is only loosely relevant; C2 practice 4 carries it |
| P4 surface, weight, per side, both directions | C1 practice 6, 15; C2 practice 8, 18; L1 practice 11 | Correct | |
| P5 3 repeats | L1 practice 3, 5; X2 practice 5, 6 | Needs fix (minor) | UMBmark uses 5 per direction (L1 practice 2, X2 practice 6). With n = 3 the SEM rule (L1 practice 3) is weak. X2 practice 5 also asks for a randomised run order; not mentioned |
| P6 verify every write; BURN disabled | REPORT §1 items 2–3, §6; PROTOCOL §3 | Correct | |
| P7 log raw data, graph setpoint and measurement | C1 practice 21; X2 practice 10 | Correct | Tools log raw lines + CSV; no plotting tool yet |
| Targets T1–T11, K1–K3 internally consistent | – | Needs fix | T3 vs actuator slew (fixed in G.1). T9 ≤ 0.25 m vs decel 1.3 m/s² + T8 0.1 s gives 0.188 + 0.07 = 0.26 m, infeasible by construction. K2 "lateral drift" undefined (sagitta or end offset from the start heading differ by 4×). T11 "battery about 11.5 V": CLAUDE.md says the SPARKs are fed from a 48→12 V buck, so check that bus voltage actually tracks the battery |
| §3 kV in V/RPM, 0.000197 ~12× too weak | REPORT §1 item 1; C1 practice 12, C1-S11 | Correct | |
| §3 hall filter 32 ms × 8 ≈ 112 ms | C1-S20/S21; REPORT §1 item 4 | Correct | Formula (N−1)·T/2 is right |
| §3 accel caps "then match MPPI (C4)" | C4 §11 "Nav2 MPPI — Humble" (no `ax_max`/`az_max`); C4 practice 3 | Needs fix → **fixed** | Humble MPPI ignores accel params; row reworded |
| §3 heading-hold off during tuning | C3 §2 ("fighting" between layers) | Correct | |
| A2 read-back, CF, faults | REPORT §1 item 3; PROTOCOL §1.2 | Correct | Reads may TIMEOUT on a MAX (PROTOCOL §5.1); the tool says so |
| A3–A5 direction, free-run, breakaway in the air | C1 practice 14; C2 practice 10; C1-S03/S16 (kS by breakaway) | Correct | For A5 use a low cap, e.g. `ramp --max 2 --rate 0.2 --cap 0.2` |
| A3 add: inverted side's `applied` sign | – | Missing | One SPARK is inverted. The ff fit assumes sign(applied) = sign(velocity) on both sides; check it in A3, or a side's kV comes out negative |
| B1 quasistatic 0 → 7 V at 0.5 V/s ×3 fwd/rev | C1-S18 (1 V/s default); C1-S32 (ramp slope must be small) | Correct | 0.5 V/s is more quasistatic than SysId's default, which is good. About 5–6 m travel, as claimed. The tool's `cap=0.6` → MD0.6 = 7.2 V cap ≥ 7 V, as intended |
| B2 dynamic steps 3/5/7 V | C1-S18 (7 V default step); C1-S20 (≥ 2 steady events + 1 accel event) | Correct | Each 3-step run travels about 5 m: alternate fwd/rev runs to stay in 15 × 15 m |
| B3 spin ramp | C1-S20 ("DrivetrainAngular") | Correct | |
| B4 fit "r² ≥ 0.9" | C1 practice 16; C1-S18 | Needs fix → **fixed** | SysId's 0.9 is the *simulated-velocity* r². The voltage-residual r² that was printed is near 1 on any ramp. The tool now prints sim-vel r² and acceleration r² |
| B4 regression form | C1-S20 `FeedforwardAnalysis.cpp` (OLS on acceleration) | Needs fix → **fixed** | See maths M1 |
| Validation data | C1 practice 9; C4 practice 1 | Missing → **added to B4** | Fit on 2 repeats, validate on the 3rd |
| B5 lag by cross-correlation; filter trade-off | C1 practice 17, 19 | Correct | To set 16 ms × 2: `PW B hallSamplePeriod 0.016` and `PW B hallAvgDepth 1` (136 is in **seconds**, 137 is an **index**: 1 = 2 samples; PROTOCOL §1.4). Predicted lag (2−1)/2·16 = 8 ms, but 1/42 rev in 16 ms is about 90 RPM of quantisation, so P will chatter |
| C1 kS/kV per side, current limit 40–60 A, brake | C1 practice 12, 13 | Correct | |
| C2 T2 check with P = I = 0 | C1-S16/S17 (FF first; I hides a bad FF) | Correct | |
| D2 P sweep, back off; delay limits P | C1 practice 17, 18 | Correct | |
| D3 I only if needed, bounded | C1 practice 5, 18 | Correct | |
| D4 Teensy slew M from the accel fit | C2 practice 10 (limit accel) | Correct | |
| D add: load-disturbance step, input effort (TV) | X2 practice 7 | Missing (minor) | |
| E1 straight 10–15 m, RTK FIXED | C2 practice 3, 4; L1 practice 4 | Correct; RTK gate made explicit (**fixed**) | 15 m runs + stopping in a 15 m area is tight: use 10–12 m |
| E2 m_per_rev = RTK ÷ revs | L1 practice 1 (scale first); C2 practice 3 (α from straight run) | Correct | RTK endpoint noise about 1.4 cm over 10 m = 0.14 %, about 7× better than K1 1 %. X2 practice 2 asks for 10×, so average the 3 repeats per condition |
| E3 left/right scale from drift + gyro, both directions | L1 practice 2 (UMBmark both directions) | Correct in intent; tool now supports it | The encoder L/R ratio is 1.000 by construction under closed-loop equal RPM. The scale information is the gyro Δψ, which flips sign fwd vs rev for a diameter-ratio error (UMBmark type B) and keeps its sign for a bias |
| Lever arm on E | L3 §10, L3 practice 10; xsens.yaml `GNSS_LeverArm [0.74,0,0]` | Correct (negligible) | `/gnss` is raw u-blox PVT = antenna (driver `gnsspublisher.h`). On a straight leg the antenna translates with base_link. For a steady curvature the traces are concentric, so the chord error is ~L²/2R² ≈ 1e-5. Wobble only adds noise |
| F gyro-bias gate before turning | L2 practice 5; L2 §12 / L2-S38 (Xsens bias **not** removed from published rate); CLAUDE.md stuck-bias issue | Missing → **added** | The stuck-bias case (−2.86 °/s = 0.05 rad/s) is a 17 % error at 0.3 rad/s. The tool now subtracts a stationary bias and flags > 0.1 °/s |
| F3 width = Δv/ω_gyro; multiplier = width/0.7366 | C2 practice 3, 13; C2-S01 eq. 11 | Correct | Matches the Mandow/diff_drive_controller convention in actuator_params.yaml |
| F4 radius-dependent width if spins ≠ arcs | C2 practice 9, 7 | Correct | |
| Gyro axis and sign | L2 practice 1, 10; URDF `imu_joint rpy="0 0 0"`; REP-103 | Correct | gz = `angular_velocity.z` in imu_link = base_link yaw rate, CCW positive (assumes the physical mount matches the URDF; the first CCW spin must give gz > 0) |
| G2 5 × 5 m square CW/CCW ×3 | L1 practice 2, 12; X2 practice 6 | Needs fix (minor) | UMBmark is 4 × 4 m, 5 runs per direction, slow, on-the-spot turns. No tool drives the square (teleop? Nav2?): specify one |
| G accel caps vs chassis | C4 practice 3; C3 practice 12 | Correct | |
| H BURN disabled, read back | REPORT §6; PROTOCOL §3 | Correct | |
| Per-terrain calibration | C2 practice 8, 18; L1 practice 11 | Correct | Grass only; re-do on the IGVC surface |
| GUM / statistics | X2 §1, §10; X2 practice 5, 10 | Missing (minor) | Report mean ± s/√n and the uncertainty components (RTK noise, gyro scale 0.5 %, m_per_rev) with each K result |
| §5 "ROS bag" | – | Needs fix (minor) | The tool logs CSV (`imu.csv`, `gnss.csv`, `gga.csv`), not a bag, and not `/filter/*`. A parallel `ros2 bag record` is optional |
| §6 MD never above 7.2 V | PROTOCOL §1.2 (hard ceiling 0.6 → 7.2 V) | Correct | Enforced in firmware |

## 3. Maths findings (`analyze.py`)

Each was tested on synthetic data with known truth (numbers below are from those runs).

- **M0 (blocker, fixed): repeated timestamps.** Rows alternate L-new / R-new, so each side's `s2_rx_us` repeats. `np.gradient` divides by zero → NaN everywhere → `ff` printed nothing. Now all per-side maths runs on that side's unique STATUS_2 samples (`np.unique`), and `pos_velocity` maps results back to rows so the segment indexing in `steps` and `turn` still works.
- **M1 (fixed): feedforward regression form and units.**
  - **Units:** volts = `applied × bus_V` per side (STATUS_0 duty × SPARK bus voltage), RPM and RPM/s. This is correct and better than the command voltage, because it includes current-limit action.
  - **Form:** the original regressed V on (sgn v, v, a) with a = gradient of position-differentiated v. Noise in the regressor a attenuates kA and biases kS. On the synthetic data (truth kS 1.0, kV 0.0022, kA 0.0006) it gave kA 0.00030–0.00047 and kS up to 1.20. SysId's form (a on v, V, sgn v; kS = −γ/β, kV = −α/β, kA = 1/β) gave kS 0.965, kV 0.00221, kA 0.00066. The fwd/rev split by sign of measured v is correct.
  - **Segments:** the fit now uses only the `ramp …` / `step …` test segments. The `S`/`stop` gaps are velocity mode (closed-loop, plus brake idle), which is not the open-loop model. `--all-segments` restores the old behaviour.
  - **Checks:** SysId's simulated-velocity r² (exact first-order discretisation, static-friction hold) and acceleration r² are printed.
- **M2 (correct; window hardened): velocity from position.**
  - Central difference over ±win/2 on STATUS_2 arrival times is zero-phase and correct.
  - `lag` used win = 0.04, i.e. ±20 ms, which just includes or excludes the ±20 ms neighbours depending on jitter. Now 0.06 (always ±1 sample).
- **M3 (correct): lag direction.** err(k) compares a(t) with b(t + k), so k > 0 means the reported velocity lags. Both signals come from the same STATUS_2 frame, so CAN and serial delays cancel. On the synthetic 32 ms × 8 filter it returned 140–144 ms, which is the simulated filter's true delay (112 ms average plus the 32 ms per-sample difference).
- **M4 (fixed): step metrics.**
  - The t95 and overshoot definitions and the sign handling for negative steps are correct: for span < 0 it uses the minimum and (target − min)/|span|.
  - The bias was noise. 1/42 rev over a 60 ms window is ±24 RPM, and the peak of noisy samples read as +4 to +5.5 % overshoot on a true 0 % response.
  - The default window for `steps` is now 0.1 s: floor about 2 %, and a real 9.4 % overshoot still read 9.0–9.7 %.
  - Remaining caveat: on steps ≤ 900 RPM (0.3 m/s) the overshoot floor is about 2 %, so judge T4 on 0.3 → 0.7 m/s (1200 RPM) or larger steps.
  - t95 is from the host command change, so it includes serial, Teensy slew and CAN.
  - L/R steady mismatch (T5) is now printed; the docstring promised it.
- **M5 (fixed): ENU.** A single 6 378 137 m radius for north overstates distances by 0.22 % at 42.7° N. Now uses WGS-84 meridional/prime-vertical radii.
- **M6 (correct): distance time alignment.** Encoder revolutions are interpolated at the host arrival times of the first and last fix. A constant GNSS latency δ shifts both ends equally, so at steady speed (after the 1 s trim) it cancels: the synthetic 100 ms latency gave m/rev 0.02052 against a truth of 0.0205.
- **M7 (correct): chord vs path.** Chord is right: it is not inflated by RTK noise, and chord/arc = 1 − Δψ²/24 ≈ 0.03 % at 5°.
- **M8 (fixed): lateral drift.** max |offset from chord| over about 70 fixes with 1 cm noise gave 0.078 m against a true 0.049 m. Now a least-squares parabola sagitta (gave 0.049 m) plus the gyro Δψ and the end offset from the start heading (L·Δψ/2 = 4 × sagitta). K2 must say which one.
- **M9 (correct): lever arm on straight runs.** Negligible for distance and for steady-curvature sagitta (see table). Relevant only in turns. `/gnss` quantisation is 1e-7° (about 1.1 cm N / 0.8 cm E), which is inside the noise budget.
- **M10 (correct; bias added): turn calibration.**
  - width = (v_R − v_L)/ω is right, and positive for both CCW and CW with REP-103 signs; the synthetic truth 0.877 m was recovered as 0.8769–0.8776.
  - The width scales 1:1 with m_per_rev (0.01994 instead of 0.0205 gave 0.853).
  - The gz axis and sign are right given `imu_joint rpy = 0`.
  - The missing piece was the gyro bias (L2-S38). It is now estimated from the stationary samples before the first nonzero setpoint (`vel --pre 3` default) and subtracted, with a flag above 0.1 °/s.
- **M11 (fixed): `load_csv`.** It crashed on empty GGA fields (no fix, empty HDOP); these now load as NaN.

## 4. Safety and tool findings (`drive_tuner.py`)

Protocol use checked against `PROTOCOL.md` and the `.ino`:
- **X-line indices:** `[1] + [3:10] + [11:18] + [19:23] + [23]` = 20 fields, matching `out("X %lu L … SP … %c")` exactly. The smoke test against the fake Teensy logged the correct columns.
- **Command syntax:** `UVL%.3f UVR%.3f` parses (`strchr` for L/R after "UV"). `MD%.3f` gets `OK MD=`, and `S` gets `OK S`.
- **Voltage cap:** ramp/steps default `cap=0.6` → 7.2 V, the firmware hard ceiling, ≥ the 7 V plan.
- **Diff-drive inverse:** `vw_to_rpm` is the standard inverse, v_L = v − ωb/2 and v_R = v + ωb/2, then × 60/m_per_rev.
- **Spin signs:** `ccw` gives L = −V, R = +V, which is CCW and correct.
- **Ramp:** 0.05 V per 50 ms = 0.5 V/s; measured about 2.5 % slower from sleep overhead, which is harmless because the fit uses measured volts. `int()` became `round()` so float error cannot drop the last step.

**Blocker:** none in `drive_tuner.py`.

**Should-fix (fixed):**
1. **Serial loss.** `write()` raised `SerialException` into `stop()`/`end_session()`, which skipped `key.restore()`: terminal left in cbreak, log and ROS not closed. Now `write()` catches the error and marks the link down, teardown is in `try/finally`, and the reader thread is let go before the files close. Tested by killing the fake Teensy mid-ramp: clean exit, log written. Motors stop via the Teensy 300 ms watchdog. The keepalive at 50 ms gives 6× margin.
2. **ROS data silently absent.** `--ros` now waits up to 4 s for `/imu/data` and `/gnss` and prints message counts and the last GGA quality. It warns if `RMW_IMPLEMENTATION` is not CycloneDDS (CLAUDE.md "DDS Config": the CLI shell does not inherit the launch's env). QoS: the Xsens publishers are default-reliable (`imupublisher.h`, `gnsspublisher.h`, `nmeapublisher.h`); the sensor-data (best-effort) and depth-50 reliable subscriptions are both compatible. `Sentence.sentence` and GGA fields 6/7/8 = quality/nsat/HDOP are correct, and the driver emits quality 4/5 from the StatusWord RTK bits (L3 §12).
3. **`MD` without v2 firmware.** v1 parses `MD0.6` as `M` with `atof("D0.6") = 0`, i.e. velocity slew 0. The tool now exits if `OK MD=` does not come back, and resets `MD0.30` at the end of a session.
4. **Gyro bias data.** `vel` now holds `L0 R0` for `--pre` 3 s before the sequence.

**Minor:**
- `fuser` missing now warns instead of passing silently.
- A non-TTY stdin warns that SPACE-stop is off (use `ssh -t`); Ctrl-C still stops.
- Keep `pkill -f` away from these scripts' names over SSH (memory note: it self-matches the shell).

## 5. Can the plan run safely in about 3 h, and what is missing

**Time:** the steps add up to 15 + 40 + 15 + 30 + 25 + 30 + 30 + 10 = **195 min, with no allowance** for:
- setup;
- Xsens warm-up (≥ 5–10 min, L2 practice 5);
- RTK convergence;
- turnarounds between the roughly 60 runs;
- battery swaps;
- the firmware v2 first flash (not yet bench-tested per PROTOCOL).

Realistic: split it. Session 1 is A–C plus E (motor model and distance scale). Session 2 is D, F and G. If only one session is possible, drop G.2 and T11.

**Missing to run it:**
1. Firmware v2 bench check first. PROTOCOL says "compiled only, not flashed and not bench-tested".
2. Sensors-only stack: `ros2 launch avros_bringup sensors.launch.py` (no actuator_node in it; `enable_zed_front` defaults false, NTRIP on; the Jetson needs internet for MDOT CORS). Check that nothing else holds the Teensy: webui is launched manually now (memory), so make sure no webui/actuator launch is up.
3. The tuner shell: `source /opt/ros/humble/setup.bash && source ~/IGVC_ROS2/install/setup.bash` (for `nmea_msgs`), plus `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://…/cyclonedds.xml`. Run over `ssh -t` for SPACE-stop. The Xsens is on `/dev/ttyUSB*` and the Teensy on `/dev/ttyACM*`/by-id, so there is no port conflict.
4. A way to drive the G.2 square and a plotting script (P7). Neither exists.
5. The target decisions flagged above: K2 definition, T9 vs decel cap, T11 vs the 12 V buck.

## 6. Changes made

**`tools/analyze.py`**
1. Per-side maths on unique STATUS_2 samples (`uniq`, `central_velocity`, `pos_velocity` maps back to rows) — fixes M0.
2. `ff`:
   - SysId acceleration-form OLS;
   - fits only `ramp`/`step` segments by default (`--all-segments` to override);
   - prints simulated-velocity r², acceleration r² and the validation advice.
3. `lag`: unique samples; window 0.04 → 0.06 s.
4. `steps`:
   - `--win` (default 0.1 s);
   - L/R steady mismatch line;
   - noise-floor note.
5. `enu`: WGS-84 radii.
6. `distance`:
   - parabola-fit sagitta replaces max |offset|;
   - gyro Δψ (bias-corrected) and end offset from the start heading;
   - requires ≥ 5 fixes;
   - lever-arm note.
7. `turn`: stationary gyro-bias estimate, subtraction and a > 0.1 °/s flag; frame/sign note.
8. `load_csv`: empty fields → NaN.

**`tools/drive_tuner.py`**
1. `write()` catches serial errors.
2. Reader tolerates `OSError`; log writes are guarded after close; `close()` lets the reader exit and tolerates a dead port.
3. `end_session` uses `try/finally` for the terminal restore, and resets `MD0.30` if the session set a cap.
4. `session` checks for the `OK MD=` reply (exits on v1 firmware) and warns about the RMW env.
5. `RosLogger`:
   - message counts;
   - a 4 s first-data check;
   - GGA quality print;
   - safe spin and close.
6. `vel --pre` (3 s `L0 R0` hold for the gyro bias).
7. `fuser`-missing warning; non-TTY warning; `round()` in the ramp step count.

**`STRATEGY.md`**
1. §3 accel-cap row: Humble MPPI has no accel params (C4 §11).
2. B4: SysId acceleration-form OLS, simulated-velocity r² > 0.9 and accel r² > 0.2, validation on a held-out repeat (C1 practice 9, C1-S18/S20).
3. E1: explicit 100 % RTK-FIXED gate via GGA quality; trim in seconds, as the tool does.
4. F preamble: gyro-bias gate (< 0.1 °/s), Xsens warm-up, `--pre` hold (L2 practice 5, L2-S38).
5. G1: T3 is a motor-layer target; through actuator_node, measure lag behind the slewed setpoint.
6. §7 tools table: actual tools and their status.

Not changed (decisions for the author): P5 repeat count, K2 definition, T9 feasibility, T11 supply question, G2 square procedure, time budget.
