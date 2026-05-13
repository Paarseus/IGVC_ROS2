# Firmware-limit characterization report (no firmware changes made)

**Date:** 2026-05-12, CPP grass, 30 min session
**Constraint:** no SparkMAX flash writes, no Teensy reflash, no `K`/`BURN` commands. Runtime ROS parameters only.

## TL;DR

| Limit | Source | Value on grass |
|---|---|---|
| **Top linear speed** | SparkMAX `kOutputMax_0` ≈ 0.47 (firmware) | **0.70 m/s** (2114 RPM × 0.01994 m/rev / 60) |
| **Top angular speed** | actuator `max_angular_rps=1.0` (YAML) + grass slip | **0.68 rad/s peak / 0.43 rad/s mean** |
| **IGVC speed cap** | 5 mph rule | 2.24 m/s |
| **Achievable / allowed** | — | **31%** of allowed linear |

Bench-fix of `kOutputMax_0` (REV Hardware Client → raise to 1.0 → BURN) is **required** before we can ramp toward competition speed.

## Hypotheses

| # | Hypothesis | Verdict | Evidence |
|---|---|---|---|
| H1 | `kOutputMax_0 ≈ 0.47` clamps per-wheel duty | ✅ CONFIRMED | `actuator_state.throttle` saturates at **0.466** at every setpoint ≥1.0 m/s |
| H2 | Linear ceiling ≈ 0.81 m/s | ✅ CONFIRMED (revised to 0.70 m/s on grass) | `L_meas_rpm` and `R_meas_rpm` both saturate at **2114 RPM** at 1.0 m/s+ commanded |
| H3 | Right-wheel 8% friction asymmetry visible per-wheel | ❌ INCONCLUSIVE | Heading-hold compensates: at low speed it commands +24% to right wheel; measured L/R asymmetry stays <1%. The asymmetry IS there in mechanics but the actuator masks it. |
| H4 | Angular ceiling is mechanical (grass), not firmware | ⚠ PARTIAL | Two caps: (a) `max_angular_rps=1.0` clamps the COMMAND in actuator; (b) grass slip then limits achievable ω even at that cap |
| H5 | `max_linear_mps=1.5` never limits us | ✅ CONFIRMED | At 1.5 m/s commanded, `L_cmd_rpm = 4541` (actuator passes it through), but SparkMAX-side kOutputMax caps the duty |
| H6 (new) | `max_angular_rps=1.0` clamps angular commands | ✅ CONFIRMED | At 1.5 and 2.0 rad/s commanded, `L_cmd_rpm` plateaus at ~1370 RPM (the value for ω≈1.0 with heading-hold overhead) |
| H7 (new) | Runtime `ros2 param set` updates actuator caps | ❌ REJECTED | Bumping `max_angular_rps` 1.0 → 3.0 at runtime had ZERO effect on commanded RPM. The node reads the param at init only. |

## F1 — Linear sweep data

| v_cmd (m/s) | Peak v_x | Mean v_x (const-window) | Efficiency | L_cmd RPM | L_meas RPM | Throttle |
|---|---|---|---|---|---|---|
| 0.3 | 0.21 | 0.17 | 57% | 906 | 600 | 0.135 |
| 0.5 | 0.36 | 0.28 | 56% | 1522 | 1086 | 0.240 |
| 0.8 | 0.83 | 0.40 | 50% | 2421 | 1703 | 0.374 |
| 1.0 | 0.70 | 0.45 | 45% | 3032 | **2114** | **0.466** ← cap |
| 1.5 | 0.70 | 0.46 | 31% | **4541** | **2114** | **0.466** ← cap |

**Two ceilings visible side-by-side:**
- `L_cmd_rpm` keeps rising with command (906 → 4541) — actuator faithfully translates `/cmd_vel`
- `L_meas_rpm` saturates at 2114 — SparkMAX duty-cap hit
- `Throttle` saturates at 0.466 — confirms `kOutputMax_0 ≈ 0.47`

## F2 — Angular sweep data

| ω_cmd (rad/s) | Peak wz | Mean wz | L_cmd RPM | Meas diff RPM | Implied actual ω |
|---|---|---|---|---|---|
| 0.5 | 0.28 | 0.14 | 701 | 577 | 0.26 rad/s |
| 1.0 | 0.69 | 0.30 | 1369 | 817 | 0.37 rad/s |
| 1.5 | 0.72 | 0.40 | **1370** ← cap | 1480 | 0.67 rad/s |
| 2.0 | 0.70 | 0.43 | **1324** ← cap | 1508 | 0.68 rad/s |

**Two-stage cap:**
- `L_cmd_rpm` plateaus at ~1370 RPM at ω_cmd ≥ 1.5 — that's the `max_angular_rps=1.0` clamp + heading-hold overhead
- Even at that capped command, grass slip means each wheel only spins ~760 RPM measured (vs ~2114 RPM straight-drive ceiling)

## F3 — Runtime parameter override (negative result)

Bumped `max_angular_rps` from 1.0 → 3.0 via `ros2 param set`, commanded 2.0 rad/s, observed:

| Metric | F2 (cap=1.0) | F3 (cap=3.0 runtime) | Theory for 2.0 rad/s |
|---|---|---|---|
| peak cmd_diff | 2648 RPM | **2603 RPM** | 4434 RPM |
| peak meas_diff | 1508 RPM | **1520 RPM** | — |
| implied actual ω | 0.68 rad/s | **0.69 rad/s** | — |

**Identical to F2.** Actuator_node didn't re-read the parameter. To actually unlock the cap, edit `src/avros_bringup/config/actuator_params.yaml` and relaunch.

## What you can do without touching firmware

### Lift the angular cap (cheap)
```yaml
# src/avros_bringup/config/actuator_params.yaml
max_angular_rps: 1.5  # was 1.0
```
Then `colcon build --packages-select avros_bringup --symlink-install` + relaunch. **Expected gain:** angular peak might go from 0.68 → ~1.0-1.2 rad/s before grass slip caps it. Not huge, but a 50% improvement.

**However:** higher angular cap also means more aggressive turning during corrections — MPPI will issue tighter steering commands. Field-test the change before committing.

### Tune actuator slew rates (cheap)
```yaml
max_linear_accel: 0.5   # could try 0.7 for faster ramp
max_angular_accel: 1.0  # could try 2.0 for snappier turns
```
Same caveat: more aggressive = more wear on motors and more violent reaction to commands.

### Acceptance: drive at 0.7 m/s and plan for it
Set MPPI's `vx_max: 0.7` (currently 0.5 for safety; was going to ramp to 1.5). The robot WILL refuse to go faster. Plan IGVC routes assuming 0.7 m/s top speed:
- 6 min run × 0.7 m/s = **252 m** total path achievable
- IGVC course path is ~150 ft = 46 m, well within reach time-wise
- Path planner needs to NOT command faster than 0.7

This is the "live with it" option if a bench session for kOutputMax isn't possible before competition.

## Required to reach IGVC competition speed (firmware change)

The fix is well-documented in `firmware/teensy_diff_drive/CLAUDE.md` and `FINDINGS.md`. You'd do (you, not me):

1. Power down robot
2. Unplug CAN cable from each SparkMAX (CLAUDE.md known issue — Hardware Client and Teensy fight for the bus)
3. USB-C from laptop into each SparkMAX
4. REV Hardware Client → Basic tab → `kOutputMax_0` slider → 1.0 (or whatever you're comfortable with)
5. **BURN** parameters (persist to flash)
6. Repeat for both SparkMAXes
7. Reconnect CAN cables
8. Re-verify with `firmware/teensy_diff_drive/phase4_duty_sweep.py`
9. New top speed: 5532 RPM × 0.01994 / 60 = **1.84 m/s** (per FINDINGS.md Phase 4 extrapolation). Reaches 82% of IGVC's 2.24 m/s cap — competition-viable.

Total bench time: ~30 min (per BRING_UP.md procedures).

## Data artifacts

```
/tmp/T3_bags/F1_*/    F1 linear sweep bag (12 MiB)
/tmp/T3_bags/F2_*/    F2 angular sweep bag (8.7 MiB)
/tmp/T3_bags/F3_*/    F3 param-bump test bag (1.2 MiB)
/tmp/F1_markers.txt   F1 phase timestamps
/tmp/F2_markers.txt   F2 phase timestamps
/tmp/F3_markers.txt   F3 phase timestamps
```

All retained on Jetson. Rsync with:
```
rsync -av jetson:/tmp/T3_bags/F* ~/F_bags/
```

## Final answer

**Is there anything wrong with the firmware? YES — `kOutputMax_0` is clamped at ~0.47, hard-limiting top speed to 0.70 m/s on grass.**

This is the single largest blocker to running at IGVC competition speed. The fix is bench-side (REV Hardware Client + BURN), not field-side, takes ~30 min, and is well documented in the firmware folder.

**Without that fix, the robot drives at 31% of allowed competition speed.** It can still complete the course — the math works out time-wise — but every speed margin is gone.

Secondary findings:
- `max_angular_rps=1.0` in actuator YAML is a soft cap; raising to 1.5 would help a bit (modest gain on grass due to slip)
- `ros2 param set` doesn't update actuator params at runtime (init-only) — file edit + relaunch required
- Heading-hold is doing its job masking the 8% L/R mechanical asymmetry

result: F1-F3 executed, kOutputMax_0 ≈ 0.47 confirmed by direct measurement (throttle saturates at 0.466), grass ceiling 0.70 m/s linear / 0.68 rad/s peak angular. max_angular_rps=1.0 found as a second soft cap on ω. Runtime ros2 param set doesn't take effect (init-only). Bench-fix of kOutputMax via REV Hardware Client + BURN is the only path to IGVC speed.
