# Bench test findings — 2026-04-23

Autonomous exploration results from phase scripts. Raw data in `data/*.csv`.

## System state — verified

| Item | Result |
|---|---|
| Bus voltage | 12.06 V (healthy) |
| CAN traffic | 208 tx/s, 623 rx/s (both SparkMAXes streaming normally) |
| STATUS_0 + STATUS_2 decode | working (voltage + velocity + position) |
| PARAMETER_WRITE at cls=14 | verified (kP=0.00042069 sentinel landed) |
| Brake idle mode | engaged (solid cyan LED, sub-second stops) |
| Duty-cycle command | works cleanly |

## Mechanical findings

### Stiction (Phase 3)
- **Left**: 0.030 duty to start motion — healthy (<0.05)
- **Right**: 0.060 duty to start motion — **2× the left's stiction**
- Right wheel shows classic "stuck → break-loose" dead-band:
  duties 0.010–0.050 produced 0 RPM, then sudden jump to 98 RPM at 0.060

### Duty-vs-RPM curve (Phase 4)
Linear fit r²=1.000 on both wheels:
- **L**: slope 5667 RPM/duty, intercept −135, extrapolated max @ 100% duty = **5532 RPM = 1.84 m/s** (97.5% of NEO free speed)
- **R**: slope 5286 RPM/duty, intercept −214, extrapolated max @ 100% duty = **5072 RPM = 1.69 m/s** (89.4% of NEO free speed)
- 8% L/R asymmetry — right has more running friction (consistent with 2× stiction)

### Long-duration stability (Phase 5)
- Velocity mode L1500: CoV 1.0% on both wheels, no periodic dips
- Duty mode 0.15: CoV 1.4-1.8% on both, no periodic dips
- **No hardware catches or chatter** — rules out bad teeth / flat bearings / binding

## Control findings

### PID tuning (Phase 6)
- Using empirical kFF = 1/R_max = **0.000197**
- Tracking below 2500 RPM: clean, 82% ratio at kP=0.0004
- **HARD CEILING at 2450 RPM** in velocity mode, regardless of setpoint:

| target RPM | L measured | R measured |
|---|---|---|
| 3000 | 2450 | 2471 |
| 4000 | 2454 | 2458 |
| 4514 | 2492 | 2453 |

- 2450 RPM equals ~0.47 effective output duty (per Phase 4's linear fit).
  Evidence strongly suggests **`kOutputMax_0` is clamped at ~0.47**.

## The blocker

**`kOutputMax_0` in PID slot 0 is set to ~0.47 instead of default 1.0.**

This ONLY affects velocity-PID mode (not duty-cycle mode). Duty mode reaches full output and hits 97%/89% of NEO free speed. Velocity mode with any setpoint above 3000 RPM is clamped to 2450 RPM.

Without this fix, **max velocity-mode ground speed = ~0.82 m/s**.
With fix, both wheels can track up to ~1.5 m/s cleanly.

## User action required

In REV Hardware Client (each SparkMAX, USB-C):

1. **PID Controllers → Slot 0 → Output Range**
   - Set **Max** to `1.0` (currently ~0.47)
   - Set **Min** to `-1.0` (currently may be ~-0.47)
   - Click **Save/Burn**
2. **Advanced → Voltage Compensation**
   - Confirm set to `0` (disabled)

After both SparkMAXes are fixed, re-run `./phase6_pid_tune.py`. Validation
should show tracking at all setpoints up to 4514 RPM (1.5 m/s target).

## Tuned gains

For velocity mode, the tune converged at:
- `kFF = 0.000197` (= 1 / R_max from Phase 4)
- `kP  = 0.0004`
- `kI = 0`, `kD = 0`

These will saturate the ~0.47 clamp at ~2400 RPM until the clamp is raised.
After fix, same gains should track cleanly up to the slower wheel's max.

## What the scripts produced

| Phase | Script | Output |
|---|---|---|
| 1 | `phase1_handspin.py` | interactive `data/phase1_handspin_*.md` |
| 2 | `phase2_baseline.py` | `data/phase2_baseline_*.csv` |
| 3 | `phase3_stiction.py` | `data/phase3_stiction_*.csv` |
| 4 | `phase4_duty_sweep.py` | `data/phase4_duty_sweep_*.csv` |
| 5 | `phase5_stability.py` | `data/phase5_stability_*.csv` (full time series) |
| 6 | `phase6_pid_tune.py` | `data/phase6_pid_tune_*.csv` (initial kP-only tune) |
| 6b | `phase6b_pid_fine_tune.py` | `data/phase6b_pid_fine_*.csv` (kP + kI fine-tune) |
| 6c | `phase6c_pid_verify.py` | `data/phase6c_pid_verify_*.csv` (precision verification) |
| 7 | `phase7_burn_verify.py` | `data/phase7_burn_verify_*.csv` (needs manual power cycle) |

All scripts share `teensy_bridge.py` (Teensy class + common helpers).
Firmware (`teensy_diff_drive.ino`) untouched throughout.

---

## Phase 6b / 6c update (same session — PID fine-tune + verification)

After the initial Phase 6 run hit a "2450 RPM ceiling" that was suspected as
`kOutputMax_0`, the user verified in Hardware Client that Output Range was
`[-1, +1]` on both controllers. Ceiling was actually **velocity-mode PID +
Brake-idle oscillation** under saturated kFF. Re-tuned at lower kFF + added kI.

### Final gains (burned to flash)

```
kFF = 0.000197
kP  = 0.0004
kI  = 9e-07
kD  = 0
```

### Phase 6c verification metrics

| Metric | Value | Verdict |
|---|---|---|
| Repeatability at 1500 RPM (5 trials) | L = 1479 ± 3.5 (0.24%), R = 1477 ± 3.6 (0.25%) | Excellent precision |
| Tracking accuracy at 1500 RPM | ~1.4% offset | kI still converging — acceptable |
| Step response | 10.7% overshoot, ±5% settle at 338 ms | Acceptable |
| Sync at 1500 RPM | L-R mean delta = 12.4 RPM (0.83%), max 40 RPM | Synced |
| Validation 500 – 3000 RPM | 99 – 100% tracking on both wheels | Excellent |
| Max tracking at 4514 RPM | 66% of cmd on both wheels | **Power-rail-limited** |

### Power rail ceiling identified

Commanding 4514 RPM sustained → bus voltage oscillates between 8.5 V and
12 V (48 V → 12 V buck sags). Both wheels plateau at ~3000 RPM on average.
PID is functioning — supply can't provide enough current at high duty.

**Practical sustained ground-speed ceiling: ~1.0 m/s** (3010 motor RPM) on
the current power architecture. Fixing the shared 12 V rail (dedicated
48 V → 19 V buck for Jetson) is the blocker for higher speeds.

### 15-second stability at 1.0 m/s — clean

`L3010 R3010` for 15 s continuous, 679 steady-state samples:
- Left mean: 99.6% of commanded, **0 dips** below 90% for >40 ms
- Right mean: 99.6% of commanded, **0 dips**
- V_bus held at 11.99–12.01 V throughout
- One-off R-track dip seen earlier on a 5-second run did NOT reproduce — classified as transient, not structural

### PARAMETER_WRITE + BURN both verified end-to-end

- `K[PIDF]<val>` writes land on SparkMAX (sentinel readback from Hardware Client confirmed)
- `BURN` command (cls=63 idx=15, magic 0x3AA3 LE) persists gains across power cycle — **tested by burning new gains and reading them back**

The full parameter path (firmware → CAN → SparkMAX flash) is now proven working.

---

## 2026-05-12 — multi-agent root-cause analysis: top-speed ceiling

User reported the car was not exceeding a certain ground speed. Two parallel research agents (codebase audit + REVLib API audit) plus on-disk evidence converged on the cause.

### Root cause (Rank 1, confirmed)

Firmware `MAX_RPM` was set to **3000**. The actuator node's `max_linear_mps = 1.5` at `m_per_motor_rev = 0.01994` issues setpoints up to:

`1.5 / 0.01994 × 60 = 4514 motor RPM`

Every value above 3000 was silently truncated in `setVelocity()`. Effective ground-speed cap:

`3000 × 0.01994 / 60 = 0.997 m/s`

Exactly matches the "1.0 m/s sustained ceiling" recorded in the Phase 6c section above — that ceiling was attributed entirely to the 12 V rail sag, but the firmware clamp would have been the hard cap even with a healthy rail.

**Fix applied**: `MAX_RPM = 4600.0f` in `teensy_diff_drive.ino:48` (4600 × 0.01994 / 60 = 1.527 m/s ground, ~19 % below NEO free speed of 5676 RPM).

### Rank 2 — power rail (still applies, see Phase 6c section)

The 12 V buck is shared between Jetson and both SparkMAXes. At sustained high duty `V_bus` sags 8.5–12 V and both wheels plateau at ~3000 RPM regardless of any firmware-side cap. Raising `MAX_RPM` alone unblocks short bursts and headroom for the slew-rate limit; sustained > 1.0 m/s still needs a dedicated 48 V → 19 V buck for the Jetson.

### Rank 3 — SparkMAX parameters firmware never initializes (silent hidden caps)

From REV's `SPARK-MAX-Types.proto` (REVrobotics/SPARK-MAX-Server), these slot-0 parameters default to values that can silently cap velocity-mode output. Firmware only writes IDs 13-16 (kP/kI/kD/kFF) — none of the below.

| ID  | Name | Default | How it caps top speed |
|---|---|---|---|
| 60  | `kSmartCurrentFreeLimit` | **20 A** | At RPM > `kSmartCurrentConfig` (10000 default, so always) the controller hard-clamps current to 20 A. Likely the cause of the 8 % R-wheel asymmetry under load (Phase 4 measured right track at 5072 RPM extrapolated vs left at 5532 — controller pulled duty back as friction torque rose). |
| 114 | `kClosedLoopRampRate` | 0 (off) | If a prior Hardware Client session left a non-zero %DC/sec, PID can't ramp output fast enough to track step setpoints. |
| 74 / 75 | `kClosedLoopVoltageMode` / `kCompensatedNominalVoltage` | 0 | Mode 2 + a comp voltage below `V_bus` scales output by `Vcomp / V_bus` — compounds the 12 V rail sag. |
| 76 / 77 | `kSmartMotionMaxVelocity_0` / `kSmartMotionMaxAccel_0` | 0 | Non-zero overrides the velocity setpoint with a Smart Motion trapezoid. |
| 113 | `kVelocityConversionFactor` | 1.0 | Silent scaling — verified 1.0 via Hardware Client. |

Wire format for all of the above is identical to the existing PID writes: `cls=14 idx=0`, `[param_id_u8, float32_LE]`, then `BURN` (`cls=63 idx=15`, magic `0x3AA3` LE) to persist.

### Recommended follow-ups

- [ ] Run REV Hardware Client check on both controllers: verify `kSmartCurrentFreeLimit ≥ 60 A`, `kClosedLoopRampRate = 0`, `kClosedLoopVoltageMode = 0`, `kSmartMotionMaxVelocity_0 = 0`.
- [ ] Consider explicit firmware-side initialization of those params in `setup()` (same path as PID writes) so a fresh SparkMAX never ships with the 20 A free-limit silently throttling under load.
- [ ] Consider bumping `max_linear_accel_mps2 0.5 → 1.0` in `actuator_params.yaml` — current ramp is 3.0 s to reach 1.5 m/s.
- [ ] Re-run `phase4_duty_sweep.py` and `phase6c_pid_verify.py` after the firmware flash to confirm 4500 RPM tracking on both wheels (will still saturate on the 12 V rail sag in sustained mode, but should reach the new cap on bursts).

