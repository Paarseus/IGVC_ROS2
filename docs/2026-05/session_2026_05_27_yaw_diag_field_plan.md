# 2026-05-27 — Yaw Diagnostic Field Session Plan (v2)

**Goal:** Decide the root cause of the 14.7° IMU-vs-EKF yaw disagreement reported in [issue #13](https://github.com/Paarseus/IGVC_ROS2/issues/13), and simultaneously collect the M1/M2/M3 motion bags that issue #12 needs to advance Phase 2 → Phase 3.

**One bag, four tests, ~25 minutes of driving.** v2 expands M1 and M2 to bidirectional + multi-speed for decisive coverage. Test A is still the load-bearing test for the Fix decision.

DO NOT apply any code fixes during the session. Fix 1, Fix 2, Fix 4 are pre-staged in `docs/yaw_diag_patches/` — apply only after analysis picks one.

---

## TL;DR — what you're doing

1. Outdoors, clear ~8 m × 8 m of flat surface (grass preferred — IGVC competition is grass).
2. Launch sensors + nav + ZED VIO. Start `live_yaw_monitor.py` in a 5th terminal. Run `preflight_check.sh` to verify the stack.
3. Start one bag that records the whole session.
4. Run M0 → M1 (4 runs) → M2 (4 runs) → M3.
5. Sanity-check the bag, stop it, copy off the Jetson.
6. Hand the bag path back — Phase 2 analysis takes over.

**Time budget:** 5 min pre-flight + 25 min driving + 5 min bag transfer ≈ 35 min total.

---

## Terminal layout (you'll have 5 SSH sessions open)

```
┌────────────────────┬────────────────────┐
│ T1: launch         │ T2: topic verify   │
│ navigation.launch  │ ros2 topic hz ...  │
│ (leave running)    │ (close after pre)  │
├────────────────────┼────────────────────┤
│ T3: bag recorder   │ T4: test commands  │
│ ros2 bag record    │ goal pubs, /cmd_vel│
│ (leave running)    │ (active)           │
├────────────────────┴────────────────────┤
│ T5: live monitor — live_yaw_monitor.py  │
│ (leave running through all tests)       │
└─────────────────────────────────────────┘
```

---

## Pre-flight (5 min)

```bash
ssh jetson
cd ~/IGVC

# 1) Stop the webui systemd service — it holds /dev/ttyACM0
sudo systemctl stop avros-webui
systemctl is-active avros-webui     # expect: inactive

# 2) Quick env sanity for CLI tools
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/dinosaur/IGVC/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml

# 3) Source ROS overlays
source /opt/ros/humble/setup.bash
source ~/IGVC/install/setup.bash

# 4) Check Xsens device is up (just confirms the kernel sees it)
ls -l /dev/ttyUSB*
```

### Pre-flight watchouts

| Check | Looking for | If wrong |
|---|---|---|
| `/dev/ttyACM0` free | `lsof /dev/ttyACM0` returns nothing | `sudo systemctl stop avros-webui` + `pkill -f actuator_node` |
| IMU not stuck-bias | `/imu/data.wz` < 0.005 rad/s while stationary (we'll verify after launch) | **USB power-cycle the Xsens** (CLAUDE.md known issue) |
| GPS fix | `status >= 0` and < 1 m peak-to-peak (verified later) | Move to clearer sky; M4 GPS data degraded but M1-M3 still work |
| Battery | > 24 V resting, > 22 V under M2 high-rate rotation | Charge first — brown-outs corrupt bags |
| Clear test area | 8×8m grass, no obstacles | Move location |

---

## Launch the stack (T1, stays running)

```bash
# Terminal 1 — DO NOT close
ros2 launch avros_bringup navigation.launch.py enable_zed_front:=true
```

Wait ~15 s for the stack to come up. You should see lines from `ekf_filter_node`, `actuator_node`, `velodyne_driver`, `xsens_mti_node`, `zed_camera`.

**Why `enable_zed_front:=true`:** Test B needs ZED VIO as the third independent yaw source. Without ZED, two-source ties have no breaker.

---

## Verify topics + IMU bias (T2)

```bash
# Terminal 2 — verify rates (Ctrl+C each after ~5s)
ros2 topic hz /imu/data                              # ~100 Hz
ros2 topic hz /wheel_odom                            # ~20 Hz
ros2 topic hz /odometry/filtered                     # ~30 Hz
ros2 topic hz /zed_front/zed_node/odom               # ~15 Hz  ← critical for Test B
ros2 topic hz /avros/wheel_debug                     # 50 Hz
ros2 topic hz /gnss                                  # ~4 Hz
```

If `/zed_front/zed_node/odom` is missing after 30s, ZED didn't enumerate — see "What can go wrong" below.

### Run pre-flight script

```bash
# T2 — comprehensive pre-flight check
bash ~/IGVC/scripts/preflight_check.sh
```

This verifies env, port, topic rates, AND samples 5s of IMU stationary bias. **If it exits non-zero, fix what it reports before going further** — don't waste a session on a bad baseline.

---

## Start the live monitor (T5, stays running)

```bash
# Terminal 5 — keep visible throughout the session
python3 ~/IGVC/scripts/live_yaw_monitor.py
```

This prints every 500 ms:

```
   t   IMU_Δ    EKF_Δ  wheel_Δ    ZED_Δ  |IMU-EKF| |IMU-wheel| |IMU-ZED|  IMU_wz  wheel_wz
  0.5    0.00    0.00     0.00     0.00       0.00        0.00      0.00   0.000     0.000
  1.0    0.01   -0.01     0.00     0.00       0.02        0.01      0.01  -0.001    -0.000
  ...
```

**Watch for early warning signs:**
- M0: any of the Δ columns > 0.3° in 60 s of standing still = drift problem, abort and investigate.
- During tests: |IMU-wheel| growing monotonically with motion = the bias we're hunting for.
- Ctrl+\\ (SIGQUIT) zeros the baseline — useful between tests.

---

## Start the bag (T3, stays running)

```bash
# Terminal 3 — DO NOT close until M3 finishes
TS=$(date +%Y%m%d_%H%M%S)
BAG=/tmp/yaw_diag_${TS}
mkdir -p "$BAG"
echo "Bag: $BAG"

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
  /imu/mag \
  /gnss \
  /zed_front/zed_node/odom \
  /zed_front/zed_node/imu/data \
  /tf \
  /tf_static \
  --qos-profile-overrides-path \
    <(printf '/tf_static:\n  durability: transient_local\n  reliability: reliable\n  history: keep_last\n  depth: 1\n')
```

**New topics in v2:** `/imu/mag` (Xsens magnetometer — directly informs Fix 4 mag-interference hypothesis) and `/zed_front/zed_node/imu/data` (ZED's internal IMU — 4th independent yaw source).

---

## M0 — Baseline (60 s stationary)

T4 only: time-marker.

```bash
# T4
echo "=== M0_START $(date +%s.%N) ==="
sleep 60
echo "=== M0_END $(date +%s.%N) ==="
```

Robot completely still. Hands off. Watch T5 (live monitor) — Δ columns should stay near zero. **If any Δ > 0.3° at t=60s, abort the session** — something is wrong (IMU stuck bias, EKF mis-tune, sensor wiring). Investigate before continuing.

---

## M1 — Test A: Multi-condition straight drive (THE decisive test)

4 runs covering forward/reverse and slow/fast. Roughly 5 m each.

**Why expanded:** original v1 plan tested forward-only at MPPI's natural speed. v2 adds reverse + slow speed because:
- Wheel slip on grass is often asymmetric (different friction forward vs reverse)
- Bias scaling with speed separates "transient slip" from "systematic friction asymmetry"
- One repeat checks within-condition variance

### M1a — MPPI 5 m forward (matches original #13 conditions)

```bash
# T4
echo "=== M1a_START $(date +%s.%N) ==="
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
echo "=== M1a_END $(date +%s.%N) ==="
```

Expected: robot drives ~5 m forward; per issue #13 may stop ~3 m short due to MPPI's 2 m goal_tolerance.

**Glance at T5 monitor at end:** record the IMU_Δ, EKF_Δ, wheel_Δ values.

Pause 10 s. Re-position robot ~5 m back to starting area for the next run.

### M1b — Direct /cmd_vel forward at slow speed (clean test, no MPPI)

```bash
# T4
echo "=== M1b_START $(date +%s.%N) ==="
timeout 14 ros2 topic pub --rate 20 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.35}}'
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{}'
echo "=== M1b_END $(date +%s.%N) ==="
```

14 s × 0.35 m/s ≈ 5 m. Direct cmd_vel bypasses MPPI — purer test of actuator + EKF without planner noise.

Pause 5 s. Robot stays put (or move slightly back for safety).

### M1c — Direct /cmd_vel REVERSE at slow speed

```bash
# T4 — make sure 5m of clear space BEHIND robot
echo "=== M1c_START $(date +%s.%N) ==="
timeout 14 ros2 topic pub --rate 20 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: -0.35}}'
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{}'
echo "=== M1c_END $(date +%s.%N) ==="
```

Same speed magnitude, opposite direction. **If wheel bias is direction-asymmetric, M1b vs M1c will show it.**

Pause 5 s.

### M1d — Direct /cmd_vel forward at HIGH speed

```bash
# T4 — clear 5m forward again
echo "=== M1d_START $(date +%s.%N) ==="
timeout 7 ros2 topic pub --rate 20 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.7}}'
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{}'
echo "=== M1d_END $(date +%s.%N) ==="
```

7 s × 0.7 m/s ≈ 5 m. Same distance, double the speed. **If bias scales with speed → systematic friction-slip. If invariant → transient.**

### M1 abort criteria

If at any M1 run:
- Robot oscillates side-to-side > 0.3 m amplitude → stop, mark M1 invalid, skip to M2
- Live monitor IMU_wz spikes > 1.0 rad/s while commanded straight → IMU spike event, note timestamp
- Robot doesn't move at all → check T1 launch log for errors, restart actuator_node maybe

---

## M2 — Test B: Bidirectional rotation at 2 rates

4 windows: CCW slow, CW slow, CCW fast, CW fast. Direct /cmd_vel bypasses MPPI.

**Why bidirectional:** if magnetic interference is heading-dependent (magnetometer crosses a "bad" heading on one side), CW vs CCW reveals it. v1 only tested CCW.

### M2a — CCW 0.5 rad/s (one full 360°)

```bash
# T4 — clear 1.5m radius around robot
echo "=== M2a_START $(date +%s.%N) ==="
timeout 12.6 ros2 topic pub --rate 20 /cmd_vel geometry_msgs/Twist \
  '{angular: {z: 0.5}}'
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{}'
echo "=== M2a_END $(date +%s.%N) ==="
sleep 3
```

12.6 s × 0.5 rad/s = 2π = 360°. Pause 3 s for IMU + EKF + ZED settle.

### M2b — CW 0.5 rad/s

```bash
# T4
echo "=== M2b_START $(date +%s.%N) ==="
timeout 12.6 ros2 topic pub --rate 20 /cmd_vel geometry_msgs/Twist \
  '{angular: {z: -0.5}}'
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{}'
echo "=== M2b_END $(date +%s.%N) ==="
sleep 3
```

### M2c — CCW 0.8 rad/s

```bash
# T4
echo "=== M2c_START $(date +%s.%N) ==="
timeout 7.85 ros2 topic pub --rate 20 /cmd_vel geometry_msgs/Twist \
  '{angular: {z: 0.8}}'
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{}'
echo "=== M2c_END $(date +%s.%N) ==="
sleep 3
```

7.85 s × 0.8 rad/s = 2π = 360°.

### M2d — CW 0.8 rad/s

```bash
# T4
echo "=== M2d_START $(date +%s.%N) ==="
timeout 7.85 ros2 topic pub --rate 20 /cmd_vel geometry_msgs/Twist \
  '{angular: {z: -0.8}}'
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{}'
echo "=== M2d_END $(date +%s.%N) ==="
sleep 3
```

### M2 abort criteria

- 12 V rail brown-out (Jetson reboots, monitor dies) during M2c/M2d → drop high-rate runs, keep 0.5 rad/s ones, flag in session log
- ZED VIO yaw drops out mid-rotation (T5 monitor's ZED_Δ stops updating) → Test B partially degraded; complete rotations but flag in log

---

## M3 — Test C: 1×1 m closing square (auto-snapshot pose, no manual edit)

The original v1 plan required you to manually edit SX/SY. v2 snapshots automatically.

```bash
# T4 — clear 3m × 3m area around robot
echo "=== M3_START $(date +%s.%N) ==="

# Snapshot current map pose into shell vars
SX=$(ros2 topic echo /odometry/filtered --once --field pose.pose.position.x 2>/dev/null | grep -E '^-?[0-9]' | head -1)
SY=$(ros2 topic echo /odometry/filtered --once --field pose.pose.position.y 2>/dev/null | grep -E '^-?[0-9]' | head -1)
echo "Square start pose: ($SX, $SY)"

# 4 goals: trace a 1m square CCW, closing at start
for OFFSET in "1.0 0.0" "1.0 1.0" "0.0 1.0" "0.0 0.0"; do
  DX=$(echo $OFFSET | awk '{print $1}')
  DY=$(echo $OFFSET | awk '{print $2}')
  X=$(python3 -c "print($SX + $DX)")
  Y=$(python3 -c "print($SY + $DY)")
  echo "Sending goal ($X, $Y) ..."
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {w: 1.0}}}}"
  sleep 2
done

echo "=== M3_END $(date +%s.%N) ==="
```

**Expected duration:** ~2 min.

**Truth metric:** position-closing error vs Ollman (1–5% of 4 m perimeter = 4–20 cm) and Mandow §5 (<10° yaw closing).

### M3 abort criteria

- Any NavigateToPose returns ABORTED → log which goal, continue with remaining goals (partial data still useful)
- Robot drives wildly off square (visible > 1 m offset by goal 2) → stop M3, mark invalid

---

## Stop, sanity check, save

```bash
# T2 — sanity check the bag BEFORE killing recording
ros2 bag info "$BAG"
```

**Expect:** 15 topics listed (13 originals + 2 v2 additions). Message count > 0 for each. If any topic shows 0 messages, that topic was never published — investigate before tearing down.

```bash
# T3 — Ctrl+C to stop bag recording
# T1 — Ctrl+C to stop the launch stack
# T5 — Ctrl+C to stop the live monitor

# Then from the LAPTOP:
mkdir -p ~/IGVC_ROS2/bags
scp -r jetson:$BAG ~/IGVC_ROS2/bags/

# Verify on laptop
ls -lh ~/IGVC_ROS2/bags/yaw_diag_*/
```

Hand the bag path back to me (e.g., `~/IGVC_ROS2/bags/yaw_diag_20260527_154532`). Phase 2 analysis takes over.

---

## What can go wrong (and what to do)

| Symptom | Likely cause | Fix |
|---|---|---|
| `enable_zed_front:=true` but no `/zed_front/zed_node/odom` topic | ZED capture card needs real DRM display (NoMachine virtual won't work for Argus) | Plug HDMI dummy plug; OR proceed without ZED — Test B will lack tiebreaker but still record valid IMU/wheel data |
| `ros2 action send_goal` returns `(0,0)` goal | RMW mismatch — CLI on FastDDS, nodes on Cyclone | Re-`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in T4 shell |
| M1a robot doesn't move | webui still publishing actuator_command and winning the freshness race | `sudo systemctl stop avros-webui` and confirm actuator_node log shows `cmd_vel` source |
| IMU drift > 0.3° in 60 s of M0 stationary | Xsens stuck-bias incident | **USB power-cycle the Xsens**, restart launch, redo M0 |
| Bag > 500 MB | Point cloud topic accidentally recorded | Stop, verify topic list doesn't include `/velodyne_points` or `/zed_*/point_cloud/...` |
| 12 V rail brown-out during M2c/M2d | Shared 12 V rail; motor inrush sags Jetson | Skip high-rate (M2c/M2d). Keep 0.5 rad/s data. Flag in session log. |
| ZED VIO yaw drift > 5° during M0 stationary | ZED still initializing features | Drive a small figure-8 manually first, then redo M0 |
| MPPI fails on M1a with "Optimizer fail to compute path" | RViz running on Jetson eating CPU OR LiDAR clutter | `pkill -x rviz2` on Jetson; check `nav2_lidar_obstacle_avoidance_test_2026_05_21.md` |

---

## What this session does NOT cover

- **Phase B Step 1 (costmap 40→20m):** defer until baseline numbers exist
- **Phase B Step 2 (DWB fallback):** orthogonal track
- **Watchdog node for |IMU_yaw - EKF_yaw|** (#13 open Q2): only if chosen fix isn't fully decisive
- **Stationary motor-spin test** (wheels locked, isolating electrical from motion-coupled mag interference): requires wheel block, hard logistics
- **Re-test:** P3.2 — separate ~5 min session after fix application

---

## Time budget summary

| Phase | Duration |
|---|---|
| Pre-flight + launch + verify | 5 min |
| M0 baseline | 1 min |
| M1 (4 runs, with re-positioning) | 8 min |
| M2 (4 rotations + settles) | 3 min |
| M3 (closing square) | 2 min |
| Sanity check + tear down | 3 min |
| Bag transfer | 3 min |
| **Total** | **~25 min** |

---

**References:** issue [#12](https://github.com/Paarseus/IGVC_ROS2/issues/12), issue [#13](https://github.com/Paarseus/IGVC_ROS2/issues/13), `docs/yaw_diag_decision_thresholds.md`, `docs/yaw_diag_analysis_strategy.md`, `docs/phase_telemetry_recipe.md`, `docs/skid_steer_kinematics_findings_2026_05_18.md`.
