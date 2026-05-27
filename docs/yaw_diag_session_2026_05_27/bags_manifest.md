# Bags Manifest — 2026-05-27 Yaw Diagnostic Session

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
