# Field Test: HSV Lane Detector → Nav2 Local Costmap

**Goal:** verify end-to-end that the HSV lane detector publishes `/lane_points`, and that Nav2's local costmap consumes those points via the dedicated `lane_obstacle_layer` plugin, without needing GPS lock or the vehicle to move.

**Branch:** `feature/lane-detection` (PR [#1](https://github.com/Paarseus/IGVC_ROS2/pull/1))

## What this test verifies

1. `avros_perception.lane_detector_node` initializes, caches `CameraInfo`, and publishes `/lane_points` at ~30 Hz
2. `controller_server` loads all four local-costmap plugins in order:
   - `voxel_layer` (Velodyne + camera_depth)
   - `voxel_inflation_layer` (1.8 m barrel buffer)
   - `lane_obstacle_layer` (lane points, marking-only, no raytrace)
   - `lane_inflation_layer` (0.10 m tight lane buffer)
3. `/local_costmap/costmap_raw` publishes at the configured rate (~2 Hz)
4. **Visual confirmation via Foxglove Studio** that lane points appear as lethal cells in the costmap
5. **Regression test:** Velodyne raytracing does NOT clear lane cells (the plugin-instance isolation from the plan review)

This is a **stationary** test. No movement, no GPS lock, no planner, no BT — just the perception → costmap path.

## Prerequisites

- Vehicle outside (or anywhere with the D455 pointing at a painted line / white tape / white paper)
- Jetson powered, Velodyne on the vehicle LAN at `192.168.13.11`, Xsens on `/dev/ttyUSB0`, D455 on USB
- Laptop with [Foxglove Studio](https://foxglove.dev/download) installed
- Jetson reachable from laptop (via Tailscale, vehicle LAN, or same WiFi)
- Repo at `~/IGVC_ROS2` on the Jetson, `feature/lane-detection` branch built:
  ```bash
  cd ~/IGVC_ROS2
  git checkout feature/lane-detection
  vcs import src < avros.repos
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install
  ```

## Launch sequence — three background processes

No patched launch file required. Run these as three separate background processes on the Jetson:

```bash
ssh jetson
cd ~/IGVC_ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/IGVC_ROS2/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml

# 1. Sensors + static TF + controller_server (hosts local_costmap)
nohup ros2 launch avros_bringup costmap_test.launch.py rviz:=false \
  > /tmp/costmap_test.log 2>&1 < /dev/null &
disown
sleep 15   # wait for RealSense + Velodyne + Xsens + controller_server activation

# 2. Lane detector
nohup ros2 run avros_perception lane_detector_node \
  --ros-args --params-file ~/IGVC_ROS2/src/avros_bringup/config/lane_detector_params.yaml \
  > /tmp/lane.log 2>&1 < /dev/null &
disown

# 3. Foxglove bridge on port 8765
nohup ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 \
  > /tmp/fg.log 2>&1 < /dev/null &
disown
```

### Required runtime workaround — `align_depth.enable`

The entry in `realsense.yaml` doesn't get honored by `realsense-ros` 4.56.4 when loaded via `--params-file` (nested period-separated params quirk). After `costmap_test.launch.py` is up, **enable alignment at runtime**:

```bash
ros2 param set /camera/camera align_depth.enable true
```

Without this the `/camera/camera/aligned_depth_to_color/image_raw` topic exists but publishes at 0 Hz, and the lane detector's `ApproximateTimeSynchronizer` never fires. See the "Known issues" section below for the permanent fix TODO.

## Verify topic flow

Before connecting Foxglove, sanity-check the pipeline on the Jetson:

```bash
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw   # expect ~30 Hz
ros2 topic hz /lane_points                                       # expect ~30 Hz
ros2 topic hz /lane_detector/debug_mask                          # expect ~30 Hz
ros2 topic hz /local_costmap/costmap_raw                         # expect ~2 Hz
ros2 topic hz /velodyne_points                                   # expect ~20 Hz
ros2 topic hz /imu/data                                          # expect ~100 Hz
ros2 lifecycle get /controller_server                            # expect "active [3]"
```

Expected `controller_server` startup log (confirms all four layers init):
```
[local_costmap.local_costmap]: Using plugin "voxel_layer"
[local_costmap.local_costmap]: Subscribed to Topics: velodyne_points camera_depth
[local_costmap.local_costmap]: Using plugin "voxel_inflation_layer"
[local_costmap.local_costmap]: Using plugin "lane_obstacle_layer"
[local_costmap.local_costmap]: Subscribed to Topics: lane_points
[local_costmap.local_costmap]: Using plugin "lane_inflation_layer"
```

A harmless `WARN` about `inflation_radius (0.100) is smaller than the computed inscribed radius (0.794)` is expected — that's the whole point of the tight lane inflation and does NOT block costmap operation.

## Connect Foxglove Studio

Open Foxglove → `Open connection` → WebSocket → pick the Jetson URL that matches your laptop's network:

| URL | When |
|---|---|
| `ws://192.168.13.10:8765` | Laptop on the vehicle LAN |
| `ws://100.93.121.3:8765` | Laptop on Tailscale |
| `ws://<jetson-wifi-ip>:8765` | Laptop on same WiFi (run `hostname -I` on Jetson) |

### Panels to add

| Panel | Topic | What to look for |
|---|---|---|
| **Image** | `/camera/camera/color/image_raw` | What the camera sees |
| **Image** | `/lane_detector/debug_mask` | Binary mask — white where HSV matched, black elsewhere |
| **3D** | `/lane_points` | Points at detected lane surfaces in `camera_color_optical_frame` |
| **3D** | `/velodyne_points` | LiDAR cloud for comparison |
| **Map** | `/local_costmap/costmap` | **The money shot** — lethal cells where the lane layer marks, tight 0.10 m inflation halo |

## The two critical tests

### 1. Lane → costmap flow

Point the camera at a painted line (or a piece of white tape/paper as a stand-in). Expected:

1. Bright line lights up in `/lane_detector/debug_mask`
2. Points appear on the line in `/lane_points` 3D view
3. **Lethal cells appear in `/local_costmap/costmap`** at the corresponding world position
4. Tight 0.10 m inflation halo surrounds those cells — much smaller than the 1.8 m halo you'd see around a Velodyne-detected barrel

### 2. Raytrace isolation (the regression test the plan review caught)

This is the one that proves the separate-plugin-instance architecture works.

1. Place a painted line in front of the robot and confirm lane cells appear in the costmap
2. **Keep the line visible to the LiDAR** — do not block it
3. Watch `/local_costmap/costmap` over several update cycles

**Pass:** lane cells persist for up to `observation_persistence: 1.0 s` then expire naturally. Velodyne raytracing sweeping through those cells does NOT clear them.

**Fail:** lane cells vanish immediately when the Velodyne sweeps through them. This would mean the plugin-instance isolation is broken — the lane layer is sharing a grid with the voxel layer, which it shouldn't be.

If this test fails, the fix is architectural, not a tuning problem. Double-check that `nav2_params*.yaml` has `lane_obstacle_layer` as a **separate plugin entry** in the `plugins:` list, not as an observation source on `voxel_layer`.

## HSV tuning (live, no restart)

All 24 detector parameters are live-tunable via `rqt_reconfigure` or `ros2 param set`. Example common adjustments:

```bash
# If mask is too noisy (catching grass highlights):
ros2 param set /lane_detector_node v_min_white 200

# If mask is too sparse (missing parts of the line):
ros2 param set /lane_detector_node v_min_white 160

# Enable yellow detection (for on-road use, not IGVC):
ros2 param set /lane_detector_node enable_yellow true

# Tighten/loosen ROI crop (0.0 = whole frame, 0.5 = bottom half only):
ros2 param set /lane_detector_node roi_y_start_frac 0.35

# Reduce PointCloud2 bandwidth:
ros2 param set /lane_detector_node pixel_stride 6
```

Once satisfied, dump the current values for a per-lighting profile:
```bash
ros2 param dump /lane_detector_node > ~/IGVC_ROS2/src/avros_bringup/config/hsv_<profile>.yaml
```

Then on the next launch, override the defaults by passing that file to `lane_detector_node` via `--params-file`.

## Shutdown

```bash
ssh jetson 'pkill -9 -f webui_node; pkill -9 -f actuator_node; \
  pkill -9 -f realsense2_camera_node; pkill -9 -f lane_detector_node; \
  pkill -9 -f xsens_mti_node; pkill -9 -f velodyne_driver; pkill -9 -f velodyne_convert; \
  pkill -9 -f ntrip_client; pkill -9 -f robot_state_publisher; pkill -9 -f foxglove_bridge; \
  pkill -9 -f controller_server; pkill -9 -f lifecycle_manager'
```

Verify clean:
```bash
ssh jetson 'pgrep -af "webui|actuator|realsense2|lane_detector|xsens_mti_node|velodyne|foxglove|ros2cli|_server|lifecycle_manager|ekf_node|navsat" | grep -v pgrep || echo ALL_CLEAN'
```

## Known issues observed during field test

### align_depth.enable not loaded from YAML

**Symptom:** `/camera/camera/aligned_depth_to_color/image_raw` topic exists but publishes at 0 Hz. The lane detector logs `lane_detector_node ready` and `CameraInfo cached` but `/lane_detector/debug_mask` never publishes because the sync never fires.

**Root cause:** `realsense-ros` 4.56.4 doesn't honor nested period-separated params from `--params-file`. The line `align_depth.enable: true` in `realsense.yaml` is silently ignored.

**Current workaround:** `ros2 param set /camera/camera align_depth.enable true` after launch.

**Permanent fix (TODO):** either
1. Restructure `realsense.yaml` to use nested YAML keys instead of the dotted form:
   ```yaml
   align_depth:
     enable: true
   ```
2. Or pass `align_depth.enable:=true` via `launch_arguments` in `sensors.launch.py` (consistent with `enable_color` / `enable_depth` / `enable_gyro` / `enable_accel`).

Option 2 is preferred — it matches the pattern already used for the other boolean toggles and is immune to the yaml-parser quirk.

### rgb_camera.power_line_frequency param range warning

Cosmetic. The driver tries to set power line frequency to `3` (Auto) but D455 firmware 5.13.0.50 only accepts `[0, 2]`. The `WARN` is logged once and has no effect on streaming.

### Velodyne azimuth cache warning

```
[velodyne_pointcloud]: No Azimuth Cache configured for model VLP16
```
Cosmetic. VLP-16 works fine without the cache; it's a pre-compute optimization for higher-ring-count sensors.

### RealSense HID/IMU disabled

```
[camera.camera]: No HID info provided, IMU is disabled
```
Expected and correct. The D455's onboard IMU cannot be accessed via the RSUSB userspace backend on JetPack 6. Xsens provides all IMU data for AVROS; RealSense gyro/accel are explicitly disabled in `realsense.yaml` and `sensors.launch.py`.

## What was verified on this test run

This document was written during an actual field-test session. Confirmed live on `feature/lane-detection` / commit `be09f0f`:

- All four local-costmap plugins loaded: `voxel_layer`, `voxel_inflation_layer`, `lane_obstacle_layer`, `lane_inflation_layer`
- `controller_server` lifecycle: active
- `/lane_points` @ 29.9 Hz, `/lane_detector/debug_mask` @ 29.9 Hz
- `/local_costmap/costmap_raw` @ 1.7 Hz
- `/velodyne_points` @ 19.8 Hz, `/imu/data` @ 99.9 Hz
- `CameraInfo cached: fx=640.7, fy=639.9, cx=647.7, cy=361.2`
- Post-workaround `/camera/camera/aligned_depth_to_color/image_raw` @ 30.0 Hz

Still to do on the next test session:
- Visual Foxglove verification that lane cells land correctly in `/local_costmap/costmap`
- Raytrace-isolation regression test with Velodyne line-of-sight through lane cells
- Per-lighting HSV profile calibration against a bag recorded on actual grass with real painted lines
