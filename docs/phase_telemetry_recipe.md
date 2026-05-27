# Phase 1 — Telemetry Recipe for Empirical PID/Controller Research

**Status:** ready to use. Phase 1 deliverable.
**Purpose:** single-source-of-truth bag-record + extract + analyze recipe for the empirical phases 2-6 of the PID/controller research plan.

---

## TL;DR

The instrumentation we need **already exists** in the codebase:

- `actuator_node.py:272-284` publishes `/avros/wheel_debug` (`std_msgs/Float32MultiArray`, 16 fields) at 50 Hz with everything an analyst needs about the inner control loop: commanded vs measured L/R RPM, encoder position, the (v, ω) target through every stage of the actuator pipeline (raw target → slewed → after IMU heading-hold), IMU yaw, heading-locked flag, e-stop state.
- `docs/motor_sync_logs/extract_bag.py` proved the pattern works for offline analysis (motor sync investigation 2026-05-05, see `docs/motor_sync_logs/REPORT.md`).

Phase 1 deliverables:
1. **`scripts/extract_bag.py`** — generalized bag-to-CSV extractor that handles the topics needed for Phases 2-6 (wheel_debug + actuator_state/cmd + wheel_odom + odometry/filtered + cmd_vel + imu/data + gnss + tf).
2. **This recipe doc** — the exact `ros2 bag record` command and analysis flow used in every empirical phase.

No new ROS message types, no new publishers, no actuator_node code changes. The work was already done in a prior session.

---

## What's in `/avros/wheel_debug`

16 floats per message at 50 Hz. Field order (mirrors `actuator_node.py:275-284`):

| Index | Field | What it tells you |
|---|---|---|
| 0 | `L_cmd_rpm` | L wheel RPM commanded over serial to Teensy (post Mandow correction) |
| 1 | `R_cmd_rpm` | R wheel RPM commanded |
| 2 | `L_meas_rpm` | L wheel RPM measured from SparkMAX encoder, via Teensy `E` line |
| 3 | `R_meas_rpm` | R wheel RPM measured |
| 4 | `L_pos_rev` | L cumulative motor revolutions (signed) |
| 5 | `R_pos_rev` | R cumulative motor revolutions |
| 6 | `v_target` | (v, ω) the user/Nav2 requested — *before* slew |
| 7 | `w_target` | |
| 8 | `v_slewed` | (v, ω) after slew-rate limiting |
| 9 | `w_slewed` | |
| 10 | `v_after_imu` | (v, ω) after IMU heading-hold P correction (this is what goes to inverse kinematics) |
| 11 | `w_after_imu` | |
| 12 | `yaw` | IMU yaw (rad), used by heading-hold |
| 13 | `yaw_rate` | IMU `angular_velocity.z` |
| 14 | `heading_locked` | 1.0 if heading-hold is currently engaged, 0.0 otherwise |
| 15 | `estop` | 1.0 if e-stop active |

Why this is enough to answer "is the inner PID delivering what we command?":
- `L_meas_rpm / L_cmd_rpm` (and R) = inner-PID delivery ratio (filter to active ticks, > 200 RPM say, to avoid divide-by-zero).
- `(v_target, w_target) → (v_slewed, w_slewed) → (v_after_imu, w_after_imu) → (L_cmd_rpm, R_cmd_rpm)` is the full forward path; you can isolate where any gap comes in.
- `L_meas - L_cmd` and `R_meas - R_cmd` give you per-wheel asymmetry (the right track is ~8% stiffer; you'll see it here).

---

## Bag-record command (the single source of truth)

Use the **CycloneDDS** environment to match the launched stack — without these env vars CLI tools default to FastDDS and either miss CycloneDDS topics or corrupt action goals (see CLAUDE.md "Known Issues" §CLI commands).

### On the Jetson (where the stack runs)

```bash
source /opt/ros/humble/setup.bash
source ~/IGVC/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/dinosaur/IGVC/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml

# Make the bag dir on the Jetson side
TS=$(date +%Y%m%d_%H%M%S)
BAG=/tmp/phase_bag_${TS}
mkdir -p "$BAG"

ros2 bag record -o "$BAG" \
  /avros/wheel_debug \
  /avros/actuator_state \
  /avros/actuator_command \
  /wheel_odom \
  /odometry/filtered \
  /odometry/global \
  /odometry/gps \
  /cmd_vel \
  /imu/data \
  /gnss \
  /tf \
  /tf_static \
  --qos-profile-overrides-path \
    <(printf '/tf_static:\n  durability: transient_local\n  reliability: reliable\n  history: keep_last\n  depth: 1\n')
```

**Note on `/tf_static`:** it's transient-local + reliable. Without the QoS override, `ros2 bag record` opens it as best-effort and misses the latched message — your bag will have no `/tf_static` and every TF lookup will fail. The `<(printf ...)` heredoc passes the override inline so you don't need a file.

**Note on `/odometry/global`:** record for Phases 2-3 (baseline + local-EKF integrity) so we capture what we're about to remove. **After Phase B step 6 (decommission map EKF), drop it from this list.**

### Bag size estimate

At Jetson rates: `/avros/wheel_debug` 50 Hz × ~80 B = 4 KB/s. `/imu/data` 100 Hz × ~300 B = 30 KB/s. `/gnss` 4 Hz × ~150 B = 0.6 KB/s. Odometry topics ~10-30 Hz × ~600 B each = ~20 KB/s. TF ~50 Hz × ~200 B = 10 KB/s. **Total: ~65 KB/s** — a 5-minute test run is ~20 MB. Trivial.

---

## Extraction to CSV

```bash
# Anywhere a Humble + avros_msgs install is sourced:
source /opt/ros/humble/setup.bash
source ~/IGVC/install/setup.bash   # or wherever avros_msgs is built

# Single bag shard (default case for short runs):
python3 scripts/extract_bag.py /tmp/phase_bag_<TS>/phase_bag_<TS>_0.db3 /tmp/phase_csv

# Or pass the bag directory (handles multiple .db3 shards if recording was long):
python3 scripts/extract_bag.py /tmp/phase_bag_<TS> /tmp/phase_csv
```

Output: one CSV per topic in `/tmp/phase_csv/`, t-zero anchored across all shards. Use pandas / matplotlib / numpy for analysis — no ROS dependency once you have the CSVs.

The extractor handles every topic in the bag-record list above. Unhandled topics are skipped with a `(UNHANDLED — skipped)` message; add a writer to `scripts/extract_bag.py:HANDLERS` if you need a new one.

---

## Per-phase topic checklist

Which topics each downstream phase actually consumes:

| Phase | Topics needed | Why |
|---|---|---|
| **2** baseline | all of the above | One drive, fan-out analyzers |
| **3** local EKF | `/wheel_odom`, `/odometry/filtered`, `/imu/data`, `/avros/wheel_debug` | All in odom frame; compare wheel-derived vs EKF-fused |
| **4** GNSS carrot (new) | `/odometry/filtered`, `/gnss`, `/cmd_vel`, `/tf` | Validate carrot node convergence |
| **5** MPPI fidelity | `/cmd_vel`, `/odometry/filtered`, `/avros/wheel_debug` | Compare emitted vs delivered |
| **6** grass surface | same as phase 2 | Re-run subset |

After Phase B step 6 (decommission map EKF): drop `/odometry/global` from all phases.

---

## Quick analysis snippets (pandas)

These are the queries every downstream phase will run. Put them in a notebook or as scripts under `docs/<phase>_logs/`.

### Inner-PID delivery ratio

```python
import pandas as pd
d = pd.read_csv('/tmp/phase_csv/avros_wheel_debug.csv')
# Only count ticks where the motor is being driven (not idle)
active = d[(d.L_cmd_rpm.abs() > 200) & (d.estop == 0)]
print('L delivery:', (active.L_meas_rpm / active.L_cmd_rpm).median())
print('R delivery:', (active.R_meas_rpm / active.R_cmd_rpm).median())
```

### Wheel-odom vs filtered (local EKF) divergence

```python
wo = pd.read_csv('/tmp/phase_csv/wheel_odom.csv').set_index('t_rel')
of = pd.read_csv('/tmp/phase_csv/odometry_filtered.csv').set_index('t_rel')
# Resample to common 10 Hz grid
common = wo.reindex(of.index, method='nearest', tolerance=0.05)
print('vx gap (filtered - wheel_odom), m/s:', (of.vx - common.vx).describe())
print('wz gap (filtered - wheel_odom), rad/s:', (of.wz - common.wz).describe())
```

### map → odom drift (until Phase B removes it)

```python
tf = pd.read_csv('/tmp/phase_csv/tf.csv')
m2o = tf[(tf.parent == 'map') & (tf.child == 'odom')].set_index('t_rel')
# Stationary 60 s window: |x|, |y|, yaw should be near zero and roughly constant
print(m2o.describe()[['x', 'y', 'yaw']])
```

### MPPI emitted (`/cmd_vel`) vs delivered (`/odometry/filtered.twist`)

```python
cv = pd.read_csv('/tmp/phase_csv/cmd_vel.csv').set_index('t_rel')
of = pd.read_csv('/tmp/phase_csv/odometry_filtered.csv').set_index('t_rel')
common = cv.reindex(of.index, method='nearest', tolerance=0.1)
gap_vx = (of.vx - common.vx)
gap_wz = (of.wz - common.wz)
print('vx gap (filtered - mppi cmd), m/s:', gap_vx.describe())
print('wz gap (filtered - mppi cmd), rad/s:', gap_wz.describe())
```

---

## Sanity check before every phase

Before a real drive, run a 10 s stationary capture and verify:

```bash
ros2 topic hz /avros/wheel_debug      # expect 50 Hz ± a few
ros2 topic hz /imu/data               # expect 100 Hz
ros2 topic hz /odometry/filtered      # expect 30 Hz
ros2 topic hz /gnss                   # expect 4-5 Hz
ros2 topic echo --once /tf_static     # must return immediately; otherwise QoS override missing
```

If `/avros/wheel_debug` isn't at 50 Hz, `actuator_node` isn't running or the control_rate is misconfigured. If `/tf_static` hangs, your bag-record QoS override is wrong (you'll have no `base_link → velodyne` etc. and every transform lookup in analysis will fail).

---

## Reference

- Existing analysis precedent: `docs/motor_sync_logs/REPORT.md` (2026-05-05 motor sync investigation — same telemetry channel, same extraction pattern).
- Field-definition source: `src/avros_control/avros_control/actuator_node.py:275-284, 505-522`.
- Extractor: `scripts/extract_bag.py` (this commit).
