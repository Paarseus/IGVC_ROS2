# Tomorrow's Test Plan

**Start here when picking up this session.** Three tests, ~30 minutes total. Validates everything we changed today.

For background, read [SESSION_FINAL.md](SESSION_FINAL.md) first.

---

## Pre-flight (2 minutes)

### Connectivity
```bash
# from laptop:
ping -c 3 100.93.121.3                # should respond
ssh jetson 'uptime'                    # should connect
```

### Pull latest code on Jetson
```bash
ssh jetson 'cd ~/IGVC && git fetch && git log --oneline HEAD..origin/main'
# expect: a620200, 681ed60, e46dc42, 27d6b41, f71d8dc (5 commits ahead until pulled)

ssh jetson 'cd ~/IGVC && git pull origin main'
```

---

## Test 1 — Revert BT XML (5 seconds) — P6.3 ⚠️ CRITICAL

The Jetson's working tree still has unsafe test-mode BT values (Timeout 120s, retries 10) from yesterday. **MUST revert before competition (IGVC 60s Hold-up-Traffic DQ).**

```bash
ssh jetson 'cd ~/IGVC && git checkout -- src/avros_bringup/config/navigate_igvc_autonav_humble.xml'
```

**Verify (expect 45000 / 3):**
```bash
ssh jetson 'grep -E "Timeout msec|number_of_retries=" ~/IGVC/src/avros_bringup/config/navigate_igvc_autonav_humble.xml | head -3'
```

Expected output:
```
    <Timeout msec="45000">
      <RecoveryNode number_of_retries="3" name="NavigateRecovery">
```

If you see `120000` or `10`, the revert didn't take — re-run the git checkout.

---

## Test 2 — L Motor Fix Validation (10 minutes)

Validates yesterday's mechanical fix actually closed the 6% L-vs-R delivery gap in forward direction.

### Setup
```bash
ssh jetson 'sudo systemctl stop avros-webui'
ssh jetson 'lsof /dev/ttyACM0 2>&1 | head -3'   # should be empty

# Launch chassis-only stack (no Nav2, no LiDAR — clean cmd_vel test)
ssh jetson 'setsid nohup bash -c "
  source /opt/ros/humble/setup.bash &&
  source /home/dinosaur/IGVC/install/setup.bash &&
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
  export CYCLONEDDS_URI=file:///home/dinosaur/IGVC/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml &&
  exec ros2 launch avros_bringup yaw_diag.launch.py
" > /tmp/yaw_diag.log 2>&1 < /dev/null &'

# WAIT 150 SECONDS for Xsens to settle (issue #15 — operational rule)
sleep 150
```

### Run test
```bash
# scp the test script to Jetson if not already there
scp /tmp/send_rev_fwd_with_imu.py jetson:/tmp/

# Run the rev+fwd test
ssh jetson 'source /opt/ros/humble/setup.bash && source /home/dinosaur/IGVC/install/setup.bash &&
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
  export CYCLONEDDS_URI=file:///home/dinosaur/IGVC/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml &&
  python3 /tmp/send_rev_fwd_with_imu.py'
```

### Pass criteria

| Metric | Yesterday (pre-fix) | Expected (post-fix) | Pass? |
|---|---|---|---|
| Forward distance per 9 s leg | ~1.3 m | **~2.5–2.7 m** | If ≥ 2.3 m |
| L_meas/L_cmd (forward) | 84% | **~93–96%** | If ≥ 92% |
| R_meas/R_cmd (forward) | 90% | ~93–96% | If ≥ 92% |
| L–R measured RPM asymmetry (forward) | -7.8% | **< 2%** | If \|asym\| < 3% |
| Yaw drift over forward leg (clean run) | -1° to -13° (variable) | **< 0.5°** | If \|yaw_drift\| < 1° |

Capture a bag during this test (~50 MB):
```bash
ssh jetson 'bash -c "
  source /opt/ros/humble/setup.bash &&
  source /home/dinosaur/IGVC/install/setup.bash &&
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
  export CYCLONEDDS_URI=file:///home/dinosaur/IGVC/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml &&
  TS=\$(date +%Y%m%d_%H%M%S) &&
  BAG=/home/dinosaur/IGVC/bags/l_motor_validation_\${TS} &&
  ros2 bag record -o \"\$BAG\" /imu/data /wheel_odom /odometry/filtered /cmd_vel /avros/wheel_debug --max-bag-duration 60
"' &
```

### If fail
- L still under-delivering = mechanical fix didn't take. Re-check the kFF on L SparkMAX via REV Hardware Client.
- L > R now (over-delivery) = bumped too far. Reduce kFF slightly.

---

## Test 3 — Combined Nav2 obstacle avoidance (15 minutes)

Validates: L motor fix + datum fix + STVL anti-flicker tuning don't interact badly. End-to-end check.

### Setup
```bash
# kill yaw_diag stack
ssh jetson 'pkill -9 -f "ros2 launch.*yaw_diag" && pkill -9 -f "ros2 bag record" && sleep 3'

# launch full nav stack
ssh jetson 'setsid nohup bash -c "
  source /opt/ros/humble/setup.bash &&
  source /home/dinosaur/IGVC/install/setup.bash &&
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
  export CYCLONEDDS_URI=file:///home/dinosaur/IGVC/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml &&
  exec ros2 launch avros_bringup navigation.launch.py
" > /tmp/nav.log 2>&1 < /dev/null &'

# WAIT 150 SECONDS for Xsens to settle
sleep 150
```

### Pre-flight checks
```bash
ssh jetson 'source /opt/ros/humble/setup.bash && source /home/dinosaur/IGVC/install/setup.bash &&
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
  export CYCLONEDDS_URI=file:///home/dinosaur/IGVC/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml &&
  ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger 2>&1 | tail -2'

# cap MPPI vx_max for safety
ssh jetson '... && ros2 param set /controller_server FollowPath.vx_max 0.35'

# clear costmaps
ssh jetson '... && ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap'
ssh jetson '... && ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap'
```

### Start a bag (on persistent disk, no /tmp!)
```bash
ssh jetson 'TS=$(date +%Y%m%d_%H%M%S) &&
  BAG=/home/dinosaur/IGVC/bags/nav_validation_${TS} &&
  echo BAG=$BAG > /tmp/bag_path_nav_val.env &&
  setsid nohup bash -c "ros2 bag record -o $BAG \
    /imu/data /wheel_odom /odometry/filtered /odometry/global /odometry/gps /gnss \
    /cmd_vel /avros/wheel_debug /tf /tf_static \
    /local_costmap/costmap /global_costmap/costmap /plan" > /tmp/bag_nav_val.log 2>&1 &'
```

### Visual check on Foxglove (laptop)

Open Foxglove Studio → connect to `ws://100.93.121.3:8765` → add panels:
- 3D panel with `/local_costmap/costmap` overlay
- Topic monitor with `/cmd_vel`, `/imu/data`, `/odometry/filtered`

**Pass criterion (visual):** noticeably less flicker in the local costmap compared to yesterday's runs. Obstacles should stay marked for several seconds at a stretch instead of every-few-seconds disappearing.

### Send obstacle test goal

Position obstacle (you or a cone) ~3m in front of robot. Send goal 10m forward:

```bash
# read current pose first to compute the goal accurately
ssh jetson '... && ros2 topic echo /odometry/filtered --once --field pose.pose'

# compute goal (10m in robot's heading direction) — same as yesterday's pattern
# then send via ros2 action send_goal
```

### Pass criteria

| Metric | Yesterday | Expected post-fix |
|---|---|---|
| Path efficiency (straight / path) | 71.8% | **> 80%** (less curving = less wasted path) |
| Number of recovery firings | 3+ (per BT log) | < 2 (with smoother chassis behavior) |
| Goal SUCCEEDED status | YES at 0.49m | **YES at < 0.5m** |
| Visible costmap flicker (Foxglove) | "every few seconds" | **smooth** |
| Map→odom drift during goal | 16 cm/s avg | similar (drift is environmental) |

### Done — competition-ready

If all three tests pass, this chassis is ready for IGVC AutoNav scenarios. Stop bags, confirm metadata.yaml exists, summarize results in this folder.

---

## If anything fails

- **L motor still under-delivers** → mechanical fix needs more work; do not run Nav2 until resolved
- **Costmap still flickers** → try Option 2 from earlier advice (raise voxel_decay further); commit & retest
- **Goal aborts repeatedly** → likely L motor not fully fixed; chassis curves trigger MPPI corrections that overshoot
- **Drift suddenly worse than 2 cm/s baseline** → check GPS fix quality (sky obstruction, multipath at this site)

---

## Cleanup after testing

```bash
# stop all bags + stack cleanly
ssh jetson 'pkill -SIGINT -f "ros2 bag record" && sleep 3'
ssh jetson 'pkill -9 -f "ros2 launch"'
ssh jetson 'pkill -9 -f "actuator_node|xsens_mti|ekf_node|navsat|robot_state_publisher|foxglove_bridge"'

# verify clean
ssh jetson 'pgrep -af "ros2|actuator|xsens|ekf|navsat" | head -5 || echo "ALL CLEAN"'

# copy validation bags to laptop (small)
scp -r jetson:/home/dinosaur/IGVC/bags/l_motor_validation_* ~/IGVC_ROS2/bags/
scp -r jetson:/home/dinosaur/IGVC/bags/nav_validation_* ~/IGVC_ROS2/bags/
```

---

## After validation passes

Update this folder with:
- `validation_results_2026_05_28.md` — your numbers from the three tests
- Mark P6.3 task as completed
- File results comment on issue #14 (L motor) — close it if fix held
- Comment on issue #15 (Xsens warmup) — operational rule now battle-tested

---

## Things to skip

Per yesterday's analysis these are NOT blocking:
- Lever-arm measurement (only matters for long curves)
- Stationary GPS noise improvement (environmental at this site)
- Live monitor wheel-integration bug (issue #16, just don't trust live numbers)
- Phase B costmap shrink P5.1 / DWB fallback P5.2 (deferred)
- Xsens stationary yaw jumps investigation (curiosity-driven only)
