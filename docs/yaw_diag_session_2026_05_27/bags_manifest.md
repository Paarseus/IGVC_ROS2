# Bags Manifest — 2026-05-27 Yaw Diagnostic Session

> See [README.md](README.md) for folder index. See [SESSION_FINAL.md](SESSION_FINAL.md) for session results. See [TOMORROW.md](TOMORROW.md) for next-session plan.

Updated as bags are recorded. After session ends, bags should be copied from Jetson `/tmp/` to laptop `~/IGVC_ROS2/bags/` for analysis.

---

## Bag 1: primary M0+M1+M2+M3 session

| Field | Value |
|---|---|
| **Path on Jetson** | `/tmp/yaw_diag_20260527_134442` |
| **Path on laptop (after copy)** | `~/IGVC_ROS2/bags/yaw_diag_20260527_134442` (pending) |
| **Started** | 2026-05-27 13:44:42 UTC |
| **Status** | 🟢 recording |
| **Phases captured** | M0 (in progress) |
| **Topics recorded** | 13 (see below) |
| **Topics NOT recorded** | `/zed_front/...` (cameras off), `/velodyne_*` (LiDAR off) |
| **Expected size** | ~25 MB for full M0+M1+M2+M3 session |

### Topics recorded
1. `/avros/wheel_debug` (50 Hz, 16-field telemetry)
2. `/avros/actuator_state` (20 Hz)
3. `/avros/actuator_command` (variable)
4. `/wheel_odom` (20 Hz)
5. `/odometry/filtered` (30 Hz, local EKF)
6. `/odometry/global` (30 Hz, map EKF)
7. `/odometry/gps` (~4 Hz)
8. `/cmd_vel` (20 Hz when goal active)
9. `/imu/data` (100 Hz)
10. `/imu/mag` (variable, for Fix 4 hypothesis)
11. `/gnss` (~4 Hz)
12. `/tf` (high rate)
13. `/tf_static` (latched, QoS override applied)

### Copy command (after session) — IRRELEVANT, BAG LOST IN CRASH
~~scp -r jetson:/tmp/yaw_diag_20260527_134442 ~/IGVC_ROS2/bags/~~

**Bag 1 was wiped when /tmp was cleared on the ~14:13 brown-out reboot.** Only the in-conversation snapshot docs preserve session-1 numbers.

---

## Bag 2: clean post-crash session — yaw_diag.launch.py (no Nav2 contention)

| Field | Value |
|---|---|
| **Path on Jetson** | `/home/dinosaur/IGVC/bags/yaw_diag_s2_20260527_144425` |
| **Path on laptop (after copy)** | `~/IGVC_ROS2/bags/yaw_diag_s2_20260527_144425` (pending) |
| **Started** | 2026-05-27 14:44:25 UTC |
| **Status** | 🟢 recording on PERSISTENT DISK (won't be lost in next reboot) |
| **Stack** | `yaw_diag.launch.py` — no Nav2, /cmd_vel publishers = 0 (clean) |
| **Topics recorded** | 13 (same as session 1) |
| **Phases to capture** | M0 → M1c → M1b → (later: M1d, M2 a-d) |

### Copy command
```bash
scp -r jetson:/home/dinosaur/IGVC/bags/yaw_diag_s2_20260527_144425 ~/IGVC_ROS2/bags/
```

### Extract command
```bash
python3 ~/IGVC_ROS2/scripts/extract_bag.py \
  ~/IGVC_ROS2/bags/yaw_diag_s2_20260527_144425 \
  ~/IGVC_ROS2/bags/yaw_diag_s2_20260527_144425_csv
```

---

## Bag 3: yaw_diag_s3_20260527_160356 — datum-fix validation (session 3 first bag)

| Field | Value |
|---|---|
| **Path on Jetson** | `/home/dinosaur/IGVC/bags/yaw_diag_s3_20260527_160356` |
| **Started** | 2026-05-27 16:03:57 UTC |
| **Duration** | ~6 min (361s) |
| **Status** | preserved on persistent disk |
| **Stack** | `yaw_diag.launch.py` (no Nav2) |
| **Used for** | Verifying the navsat datum fix (commit 27d6b41) actually eliminated the 145° map-frame rotation — confirmed +0.10° gap |

CSVs on laptop at `~/IGVC_ROS2/bags/yaw_diag_s3_20260527_160356_csv/`.

---

## Bag 4: yaw_diag_s3_20260527_161406 — M2 rotations + M3 square

| Field | Value |
|---|---|
| **Path on Jetson** | `/home/dinosaur/IGVC/bags/yaw_diag_s3_20260527_161406` |
| **Started** | 2026-05-27 16:14:07 UTC |
| **Duration** | ~5 min (273s) |
| **Status** | preserved on persistent disk |
| **Stack** | `yaw_diag.launch.py` (no Nav2) |
| **Used for** | M2 (4 in-place rotations) + M3 (1m square — script bug, ignore M3) |
| **Headline** | CW rotation delivered ~50% commanded ω, CCW ~100% — confirms L motor weakness affects rotation too |

CSVs NOT yet on laptop.

---

## Bag 5: obstacle_avoid_20260527_164508 — full Nav2 + LiDAR obstacle avoidance

| Field | Value |
|---|---|
| **Path on Jetson** | `/home/dinosaur/IGVC/bags/obstacle_avoid_20260527_164508` |
| **Started** | 2026-05-27 16:45:08 UTC |
| **Duration** | ~15 min (905s) |
| **Size** | **9.2 GB** (includes /velodyne_points) |
| **Status** | preserved on persistent disk |
| **Stack** | `navigation.launch.py` (full Nav2 + Velodyne) |
| **Topics extra** | /velodyne_points, /local_costmap/costmap, /global_costmap/costmap, /plan |
| **Used for** | Goal at (+9.805, +1.966); SUCCEEDED within 0.49 m at t=599s |
| **Findings** | Reactive avoidance worked; MPPI did wide arc around human obstacle. Drift analysis confirmed drift didn't cause any failures. |

CSVs on laptop at `~/IGVC_ROS2/bags/obstacle_avoid_20260527_164508_csv/` (43 MB, excludes huge LiDAR + costmap topics).

### Copy command
```bash
scp -r jetson:/home/dinosaur/IGVC/bags/obstacle_avoid_20260527_164508 ~/IGVC_ROS2/bags/   # 9.2 GB!
```

---

## Validation bags expected tomorrow (per TOMORROW.md)

- `l_motor_validation_<TS>` — Test 2 output: post-fix chassis behavior
- `nav_validation_<TS>` — Test 3 output: end-to-end nav with all fixes applied
