# Yaw Diagnostic — Session Log (2026-05-27)

Live log. Append observations as the session progresses. This becomes the seed for the eventual `docs/yaw_diag_session_2026_05_27.md` report.

---

## Session metadata

| Field | Value |
|---|---|
| Date | 2026-05-27 |
| Location | (fill in: indoor concrete / outdoor grass / parking lot) |
| Surface | (grass / concrete / asphalt) |
| Temperature | |
| GPS sky visibility | (clear / partial / urban canyon) |
| Battery start V | |
| Battery end V | |
| Bag path | `/tmp/yaw_diag_YYYYMMDD_HHMMSS` |

---

## Pre-flight (`scripts/preflight_check.sh` output)

```
(paste output here)
```

Decisions / overrides:
-

---

## M0 — Stationary baseline (60 s)

Live monitor (`scripts/live_yaw_monitor.py`) — last row at t=60s:

```
(paste line)
```

Per the decision-thresholds doc M0 table:

| Metric | Observed | Expected | Pass/Fail |
|---|---|---|---|
| `/imu/data.wz` mean | | <0.05°/s | |
| `/imu/data.wz` σ | | <0.15°/s | |
| `/odometry/filtered` xy drift | | <1mm/s | |
| `map→odom` drift rate | | <0.5cm/s (M4 was 0.26) | |
| IMU vs EKF yaw delta | | <0.1° drift over 60s | |

Verdict: **(PASS / MARGINAL / FAIL — explain)**

---

## M1 — Test A: 5 m straight (DECISIVE)

Goal sent: `(x, y) = (5.0, 0.0)` in map frame
Actual end position from `/odometry/filtered`: `(x, y) = ___, ___`
Distance traveled: ___ m
Drive duration: ___ s

Live monitor — last row before stop:

```
(paste line)
```

Post-bag analysis (`python3 scripts/analyze_M1.py <csv_dir>`):

```
(paste full output — comparison table + verdict)
```

---

## M2 — Test B: Rotation (0.5 then 0.8 rad/s)

### Window 1: 0.5 rad/s × ~12.6s (target 360°)

Live monitor — last row at end of window:

```
(paste line)
```

### Window 2: 0.8 rad/s × ~7.85s (target 360°)

Live monitor — last row at end of window:

```
(paste line)
```

Post-bag analysis (`python3 scripts/analyze_M2.py <csv_dir>`):

```
(paste full output — per-window tables + cross-window rate-dependence)
```

ZED VIO present and used? (yes / no — if no, note why)

---

## M3 — Test C: 1×1 m closing square

Start pose (from `/odometry/filtered`): `(x, y, yaw) = ___, ___, ___`
4 goals sent (relative offsets): `(1,0), (1,1), (0,1), (0,0)`

Behavior observed:
- (notes — did all 4 goals reach SUCCEEDED? any retries? any aborts?)

Post-bag analysis (`python3 scripts/analyze_M3.py <csv_dir>`):

```
(paste full output — closing metrics + per-leg drift)
```

---

## Bag handling

```
# copy to laptop
scp -r jetson:/tmp/yaw_diag_YYYYMMDD_HHMMSS ~/IGVC_ROS2/bags/

# extract CSVs
python3 scripts/extract_bag.py ~/IGVC_ROS2/bags/yaw_diag_YYYYMMDD_HHMMSS \
    ~/IGVC_ROS2/bags/yaw_diag_YYYYMMDD_HHMMSS_csv

# verify topics captured
ls ~/IGVC_ROS2/bags/yaw_diag_YYYYMMDD_HHMMSS_csv/
```

Expected files (per `extract_bag.py` HANDLERS):
- `imu_data.csv` ✅
- `wheel_odom.csv` ✅
- `odometry_filtered.csv` ✅
- `odometry_global.csv` (may be empty if map EKF was off)
- `odometry_gps.csv` (may be empty)
- `cmd_vel.csv` ✅
- `gnss.csv` ✅
- `zed_front_zed_node_odom.csv` (only if `enable_zed_front:=true`)
- `tf.csv` ✅
- `tf_static.csv` ✅
- `avros_wheel_debug.csv` ✅
- `avros_actuator_command.csv`
- `avros_actuator_state.csv`

Missing any of the ✅-marked? **STOP** — analysis won't run without them.

---

## Final decision

Per `docs/yaw_diag_decision_thresholds.md` decision matrix:

**Matched row:** (fill in which row of the matrix)

**Action:** (Nothing / Fix 1 / Fix 1+2 / Fix 4 / Hold)

**Apply commands:**
```
# (example)
git apply docs/yaw_diag_patches/fix1_yaw_process_noise.patch
# (if Fix 2)
git apply docs/yaw_diag_patches/fix2_wheel_odom_vyaw_cov.patch
ssh jetson "cd ~/IGVC && colcon build --symlink-install --packages-select avros_control && source install/setup.bash"
```

Re-test (P3.2): re-run **Test A only** after applying. Append re-test results here.

---

## Anomalies / surprises

(Anything weird that doesn't fit one of the structured sections above. E.g., topic dropped during M2, GPS fix lost mid-drive, motor brown-out, etc.)

-
-
-
