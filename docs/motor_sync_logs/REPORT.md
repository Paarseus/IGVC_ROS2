# Motor Sync Investigation — 2026-05-05

## TL;DR

**The wheels are NOT out of sync.** Wheels-up testing across 73 seconds shows L and R integrate to within **0.43% of each other** (L=+725 rev, R=+722 rev). The on-ground "motors not synced" symptom is **mechanical** (R-side friction) plus **command saturation**, not a controller bug.

The real problems we found:

1. **Both wheels saturate at ~2050 RPM unloaded**, regardless of command. Commands above 2050 RPM cannot be tracked.
2. **In the linear region (cmd < 1800 RPM), both wheels under-track to ~64% of commanded.** SparkMAX velocity-PID doesn't reach the setpoint even with no load. Suggests kFF too low or kP too low.
3. The actuator_node currently maps full joystick → 1.5 m/s → ~4516 RPM commanded — well above the saturation ceiling. Most of the joystick travel is wasted.

## Test setup

Two bags recorded with `actuator_node` patched to publish a 50 Hz `/avros/wheel_debug` (`std_msgs/Float32MultiArray`, 16 floats) carrying commanded L/R RPM, measured L/R RPM, cumulative L/R motor revs, and the (v,w) targets through every stage of the controller.

| Bag | duration | env | what we wanted to learn |
|---|---|---|---|
| `motor_sync_bag/` | 27 s | wheels on ground | initial diagnostic |
| `motor_sync_bag_wheels_up/` | 73 s | wheels off ground | isolate controller from mechanics |

## Key numbers

### Wheels-up (the controlled experiment)

| metric | L | R | comment |
|---|---|---|---|
| net rotation, 73 s | +725.34 rev | +722.25 rev | **0.43% match** |
| follow-ratio in linear region (cmd 200-1800 RPM) | 0.64 | 0.63 | both under-track equally |
| sign(meas) matches sign(cmd) | 99.2% | 99.1% | wiring fine |
| measured ceiling at high cmd | ±2074 RPM | ±2017 RPM | hard saturation |

### On-ground (for context)

| metric | L | R | comment |
|---|---|---|---|
| net rotation, 27 s | -33 rev | +91 rev | opposite signs — but explained by left-biased joystick + mixed brake input, not a sync bug |
| follow-ratio under load (active ticks) | 0.59 | 0.50 | R's 8% friction disadvantage shows |
| follow-ratio std-dev | 0.57 | 1.25 | R is much choppier under load |

## What this rules out

- **Encoder/wiring inversion** — wheels-up sign correlation 99.1-99.2%.
- **Diff-drive inverse kinematics bug** — math reproduces v_slewed/w_slewed exactly.
- **Heading-hold over-correcting** — IMU was inactive in both runs (`v_after_imu == v_slewed`).
- **R-side controller asymmetry** — wheels-up, L and R behave nearly identically.

## Real root causes

### 1. Hard saturation at ~2050 RPM (unloaded)

Even with no load, neither wheel reached above ±2080 RPM no matter how high the command. Possible causes (need follow-up):

- **Battery sag.** NEO free-spin at 12 V is 5676 RPM; saturation at 2050 RPM implies the rail is at ~4.3 V, OR the SparkMAX is current-limited, OR the kFF is so low that the PID never asks for more duty.
- **kFF too low for actual reachable max.** kFF=0.000197 = 1/5076 RPM. If the system can only physically reach 2050 RPM, the feedforward duty for cmd=2050 is 0.404 (40%), and the PID has to do all the rest. With kP=0.0004 and a 3000 RPM error, that's only 1.2 duty correction → clamped at 1.0, but if the rail is sagged or current-limited the duty doesn't translate to RPM.
- **SparkMAX kOutputMax** parameter — was a known issue in early Phase 4. Should re-verify it's at 1.0 in REV Hardware Client.

### 2. Both wheels under-track in linear region

In the 200-1800 RPM band where there's no saturation, L and R both deliver ~64% of commanded. This is purely a control-gain issue (not a hardware ceiling). Either kFF needs to be doubled or kP needs significant raising. The fact that both wheels under-track by the same amount confirms it's a global gain issue, not a per-wheel calibration problem.

### 3. The on-ground "motors not synced" symptom

Combination of:
- Both wheels saturating ~2000 RPM (under load it's lower) → SparkMAX velocity-PID cannot close the loop → output is railed
- In the railed condition, mechanical asymmetry dominates → R's documented 8% extra friction makes it slower than L
- Robot pulls right (or veers, depending on which wheel slips more)

## Recommended fixes (in order of impact)

1. **Drop `max_linear_mps` to ~0.66 m/s** in `actuator_params.yaml` so commanded RPM stays inside the linear region. Current 1.5 m/s = 4516 RPM commanded; achievable max is ~2050 RPM = 0.68 m/s ground. The webui joystick will then map full-throttle to the achievable max instead of wasting 60% of joystick travel in saturation.

2. **Investigate why max RPM is 2050 unloaded.** Steps in priority order:
   - Check battery voltage at idle and during full-throttle command. Sag below 11 V → battery weak or shared rail issue.
   - Check `kOutputMax_0` in REV Hardware Client on both SparkMAXes — must be 1.0.
   - Check SparkMAX current limits — if set to ~30 A, the motor will saturate well below free-spin.

3. **Re-tune kFF for measured loaded max RPM.** Once the saturation ceiling is understood, set `kFF = 1 / actual_loaded_max_RPM`. Currently kFF=0.000197 (=1/5076) which is far too low if the real max is 2050.

4. **Wait on cross-wheel asymmetry fixes until 1-3 are done.** The R-side friction issue is real but small (8%) and irrelevant while we're operating in saturation. After 1-3, re-test on the ground to see if the chassis still pulls.

## Files

```
docs/motor_sync_logs/
├── REPORT.md                              ← this document
├── extract_bag.py                         ← bag → CSV, run on Jetson
├── analyze.py                             ← full-window plots + stats
├── analyze_pure_forward.py                ← filter to no-steer windows
├── motor_sync_bag/                        ← on-ground bag (27 s)
├── motor_sync_bag_wheels_up/              ← wheels-up bag (73 s)
├── csv/                                   ← on-ground CSVs (4 topics)
├── csv_wheels_up/                         ← wheels-up CSVs
├── wheel_sync.png                         ← on-ground time series
├── wheel_cmd_vs_meas_scatter.png          ← on-ground scatter
├── wheel_sync_pure_forward.png            ← on-ground pure-forward subset
├── wheel_sync_wheels_up.png               ← wheels-up time series
├── wheel_cmd_vs_meas_scatter_wheels_up.png ← wheels-up scatter
└── jetson_crash_boot_journal.log          ← brown-out forensics from earlier
```
