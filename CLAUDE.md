# IGVC_ROS2 - Autonomous Vehicle ROS2 Platform

## Project Overview

Migration of the AV2.1-API autonomous vehicle codebase to ROS2 (Humble). Five custom packages + upstream drivers (Velodyne, RealSense, Xsens) + Nav2 + robot_localization.

**Naming:** the GitHub repo is `IGVC_ROS2` (https://github.com/Paarseus/IGVC_ROS2). The live Jetson workspace lives at `~/IGVC/`. The older `~/AVROS/` directory on the Jetson is a dead feature branch — do **not** use it; all work goes through `~/IGVC/`.

**Source reference:** `~/AV2.1-API`
**Build:** `cd ~/IGVC && colcon build --symlink-install`
**Source overlay:** `source install/setup.bash`
**Target hardware:** NVIDIA Jetson Orin (Ubuntu, `ssh jetson` = 100.93.121.3 via Tailscale, user `dinosaur`)
**SSH shortcut:** `ssh jetson` (configured in `~/.ssh/config`)

---

## Build & Test

```bash
# Clone source dependencies (one-time setup, requires python3-vcstool)
cd ~/IGVC
vcs import src < avros.repos
# Clones src/realsense-ros/ (4.56.4) and src/xsens_mti/ (ros2 branch, includes xsens_mti_ros2_driver + ntrip)

# Build all packages (avros_msgs must build first for message generation)
colcon build --symlink-install --packages-select avros_msgs
colcon build --symlink-install

# Test static TF
ros2 launch avros_bringup sensors.launch.py
ros2 run tf2_tools view_frames

# Test actuator (bench)
ros2 launch avros_bringup actuator.launch.py
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}}'

# Test localization
ros2 launch avros_bringup localization.launch.py

# Keyboard teleop (bench test)
ros2 launch avros_bringup teleop.launch.py
# Keys: i=forward, ,=backward, j=left, l=right, k=stop, q/z=speed up/down

# Phone web UI (bench test — proportional joystick + mode buttons)
ros2 launch avros_bringup webui.launch.py
# Open https://<jetson-ip>:8000 on phone (accept self-signed cert)

# Full autonomous stack
ros2 launch avros_bringup navigation.launch.py

# Send a navigation goal (click "Nav2 Goal" in RViz or use CLI)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 50.0, y: 30.0}}}}"

# Regenerate campus road graph (offline, requires osmnx)
python3 src/avros_navigation/scripts/generate_graph.py \
  --output src/avros_bringup/config/cpp_campus_graph.geojson

# E-stop
ros2 topic pub --once /avros/actuator_command avros_msgs/msg/ActuatorCommand \
  '{estop: true}'
```

---

## Packages

| Package | Build Type | Purpose |
|---------|-----------|---------|
| `avros_msgs` | ament_cmake | ActuatorCommand.msg, ActuatorState.msg, PlanRoute.srv |
| `avros_bringup` | ament_python | Launch files, URDF, all YAML configs, RViz config |
| `avros_control` | ament_python | `actuator_node`: cmd_vel / ActuatorCommand → diff-drive inverse + IMU heading-hold + slew-rate → Teensy serial → SparkMAX velocity PID |
| `avros_webui` | ament_python | `webui_node`: phone joystick WebSocket → ActuatorCommand (direct control) |
| `avros_navigation` | ament_python | `generate_graph.py`: offline OSMnx → nav2_route GeoJSON graph tool |
| `avros_perception` | ament_python | `perception_node`: ZED X RGB/cloud → swappable Pipeline (stub/HSV/ONNX) → mono8 mask + organized cloud + LabelInfo for `kiwicampus/semantic_segmentation_layer` |

No `avros_sensors` — upstream drivers used directly. Source dependencies are managed via `avros.repos` (vcstool manifest) and git-ignored. `vcs import src < avros.repos` clones `realsense-ros` (4.56.4), the Xsens monorepo (`xsens_mti_ros2_driver` + `ntrip`), `zed-ros2-wrapper`, and `semantic_segmentation_layer` (kiwicampus, humble branch — requires patch, see `scripts/apply_kiwicampus_patches.sh`). Velodyne uses `ros-humble-velodyne` (apt).

---

## Network Inventory (192.168.13.0/24)

| IP | Device | MAC | Notes |
|----|--------|-----|-------|
| 192.168.13.10 | Jetson Orin | — | Compute platform, runs all ROS2 nodes |
| 192.168.13.11 | Velodyne VLP-16 | 60:76:88:38:0F:20 | Reconfigured from factory 192.168.1.201 |
| 192.168.13.31 | Gateway/router | — | Network gateway |

---

## Sensors

### Velodyne VLP-16

- **Package:** `ros-humble-velodyne` (apt, official)
- **Config:** `avros_bringup/config/velodyne.yaml`
- **IP:** 192.168.13.11, port 2368
- **Topics:** `/velodyne_packets` (~32 Hz), `/velodyne_points` (~70 Hz)
- **Nodes:** `velodyne_driver_node` (raw UDP → packets), `velodyne_convert_node` (packets → PointCloud2)
- **Range filter:** min 1.0m (car body), max 50.0m
- **Verified working** — data confirmed via tcpdump and topic echo

### Intel RealSense D455

- **Package:** Built from source — `realsense-ros` 4.56.4 in `~/IGVC/src/realsense-ros/` + librealsense 2.57.6 at `/usr/local/` (built from `~/librealsense` with RSUSB backend)
- **Firmware:** 5.13.0.50 (downgraded from 5.17.0.9)
- **Serial:** 215122251311
- **USB:** 3.2
- **Config:** `avros_bringup/config/realsense.yaml`
- **Resolution:** 1280x720 @ 30fps (color + depth)
- **Features:** depth align enabled, pointcloud disabled (Nav2 uses VoxelLayer instead)
- **IMU:** Disabled (`enable_gyro: false`, `enable_accel: false`) — D455 HID/IMU fails with RSUSB backend on JetPack 6; Xsens provides IMU instead
- **Verified working** — color 30fps, depth streaming, rqt_image_view confirmed
- **Visualization:** `~/Desktop/visualize_camera.sh` on Jetson (launches camera + rqt_image_view)

### Xsens MTi-680G (IMU + GNSS)

- **Package:** `xsens_mti_ros2_driver` (build from source, ros2 branch)
- **Device ID:** 0080005BF5
- **Firmware:** 1.12.0, build 42
- **Filter:** General_RTK
- **Config:** `avros_bringup/config/xsens.yaml`
- **Port:** `/dev/ttyUSB0` at 115200 baud
- **Output rate:** 100 Hz
- **Topics:** `/imu/data`, `/gnss` (NavSatFix), plus quaternion, euler, acceleration, mag, twist, NMEA
- **GNSS lever arm:** `[0.0, 0.0, 0.0]` — TODO: measure antenna offset on vehicle
- **u-Blox platform:** Automotive (type 4)
- **Required deps:** `ros-humble-mavros-msgs`, `ros-humble-nmea-msgs` (apt)
- **Verified working** — 100Hz IMU data confirmed on /dev/ttyUSB0

### NTRIP Client (RTK Corrections)

- **Package:** `ntrip` (C++, ament_cmake — from `Xsens_MTi_ROS_Driver_and_Ntrip_Client` repo, ros2 branch)
- **Config:** `avros_bringup/config/ntrip_params.yaml`
- **Data flow:** xsens `/nmea` (GPGGA) → ntrip_client → NTRIP caster → RTCM3 → `/rtcm` → xsens MTi-680G → RTK FIXED/FLOAT
- **Topics:** subscribes `/nmea`, publishes `/rtcm`
- **Launch:** enabled by default in sensors.launch.py; disable with `enable_ntrip:=false`
- **Credentials:** edit `ntrip_params.yaml` — set `mountpoint`, `username`, `password` for your NTRIP caster
- **Default caster:** rtk2go.com:2101 (free, requires mountpoint selection)
- **Setup:** Included in `avros.repos` — `vcs import src < avros.repos` clones the full Xsens monorepo to `src/xsens_mti/`, which contains both `xsens_mti_ros2_driver` and `ntrip` packages. Then `colcon build` discovers both automatically.
- **Not tracked in git** — `src/xsens_mti/` is in `.gitignore`, built from source like `src/realsense-ros/`

### ZED X Front Camera (GMSL2)

- **Package:** `zed_wrapper` (metapackage from `stereolabs/zed-ros2-wrapper`, pinned to `v5.2.2` in `avros.repos`). **The camera is a composable component (`stereolabs::ZedCamera`), not a standalone executable** — launched via `IncludeLaunchDescription` of the wrapper's own `zed_camera.launch.py`, not a raw `Node()`.
- **Hardware:** ZED X, serial `49910017`, mounted front-center, connected via ZED Link Quad capture card on Jetson Orin. Confirmed live 2026-04-24 on Jetson (SDK 5.2.0, JetPack 6 L4T R36).
- **Config:** `avros_bringup/config/zed_front.yaml` — override layered on wrapper's `common_stereo.yaml` + `zedx.yaml` via `ros_params_override_path`. Uses `/**:` wildcard as top-level key. Launch args set `camera_name`, `camera_model`, `serial_number`, `publish_tf`.
- **Resolution/rate:** `HD1080 @ 15 fps` grab, `NEURAL_LIGHT` depth, point cloud at 15 Hz (match image for ApproximateTime sync). Published image is 540×960 (pub_downscale_factor 2.0); published cloud is 256×448 (`point_cloud_res: COMPACT`) — **see avros_perception note below**.
- **Namespace / node name:** `/zed_front/zed_node/...` — **pass ONLY `camera_name` to the launch include** (not `namespace`/`node_name`), or the wrapper silently collapses the tree to `/zed_front/zed_front/...`.
- **Key topics (verified 2026-04-24):**
  - `/zed_front/zed_node/rgb/color/rect/image` (sensor_msgs/Image, rectified color; **note v5.x path**, not `rgb/image_rect_color`)
  - `/zed_front/zed_node/point_cloud/cloud_registered` (organized PointCloud2, `height > 1`)
  - `/zed_front/zed_node/rgb/color/rect/camera_info`
- **TF:** `base_link → zed_front_camera_link → zed_front_camera_center / _left_camera_frame(_optical) / _right_camera_frame(_optical) / _imu_link` — full frame chain provided by `<xacro:zed_camera>` macro in the URDF. Mount joint is `base_link → zed_front_camera_link` (TODO measure real offset).
- **Launch:** off by default — `ros2 launch avros_bringup sensors.launch.py enable_zed_front:=true`.
- **Required on Jetson:** ZED SDK installed at `/usr/local/zed`; wrapper rebuilt against the installed SDK (major.minor must match the `v5.2.2` pin). After any SDK upgrade, `colcon build --packages-select zed_wrapper zed_components`.
- **Verification:** `ZED_Explorer` should see serial 49910017 before launching the ROS node.

### Left / Right / Back ZED X (future)

- `zed_left.yaml` (serial 43779087) and `zed_right.yaml` (new unit — replacing faulted 47753729, serial TBD) are not yet wired into launch; Phase 5 work.
- `zed_back_camera_center` URDF frame is preserved but gated behind `enable_zed_back:=true` xacro arg (default false). Serial 49910017 was repurposed to front.

---

## TF Tree

```
map                                    ← navsat_transform_node
 └── odom                              ← robot_localization EKF
      └── base_link                    ← robot_state_publisher (URDF)
           ├── imu_link                ← static (URDF) — TODO: measure mount position
           ├── velodyne                ← static (URDF) — TODO: measure mount position
           ├── camera_link             ← static (URDF, RealSense) — TODO: measure mount
           │    ├── camera_color_optical_frame  ← realsense driver
           │    └── camera_depth_optical_frame  ← realsense driver
           ├── zed_front_camera_link   ← static (URDF, via zed_macro.urdf.xacro) — TODO: measure mount
           │    ├── zed_front_camera_center
           │    ├── zed_front_left_camera_frame
           │    │    └── zed_front_left_camera_frame_optical   ← cloud/image frame_id
           │    ├── zed_front_right_camera_frame
           │    │    └── zed_front_right_camera_frame_optical
           │    └── zed_front_imu_link
           └── base_footprint          ← static (URDF)
```

Sensor mount positions in URDF (`avros.urdf.xacro`) are approximate — measure on real vehicle.

---

## Key Topics

| Topic | Type | Source |
|-------|------|--------|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 controller / teleop_twist_keyboard |
| `/avros/actuator_state` | `avros_msgs/ActuatorState` | actuator_node @ 20 Hz |
| `/avros/actuator_command` | `avros_msgs/ActuatorCommand` | webui_node (direct control) / e-stop |
| `/plan_route` | `nav2_msgs/action/ComputeRoute` | route_server (nav2_route) |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | velodyne_convert_node (~70 Hz) |
| `/velodyne_packets` | `velodyne_msgs/VelodyneScan` | velodyne_driver_node (~32 Hz) |
| `/imu/data` | `sensor_msgs/Imu` | xsens_mti_node (100 Hz) |
| `/gnss` | `sensor_msgs/NavSatFix` | xsens_mti_node |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF (robot_localization) |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | realsense2_camera_node |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | realsense2_camera_node |
| `/nmea` | `nmea_msgs/Sentence` | xsens_mti_node (GPGGA ~4 Hz) |
| `/rtcm` | `mavros_msgs/RTCM` | ntrip_client (RTCM3 corrections) |

---

## Messages

`ActuatorCommand`, `ActuatorState`, `PlanRoute` — see `src/avros_msgs/msg/*.msg` and `src/avros_msgs/srv/*.srv` for field definitions.

---

## Actuator Control

### Architecture (diff-drive track chassis — AndyMark Raptor)

```
/cmd_vel (Twist)           ──┐
/avros/actuator_command ─────┤ → actuator_node (Jetson)
                             │   ├─ slew-rate limit (v, ω)
                             │   ├─ IMU heading-hold (straight) + gyro-stabilized turns (turning)
                             │   ├─ diff-drive inverse → L_mps, R_mps → motor RPM
                             │   └─ pyserial to /dev/ttyACM0 @ 115200
                             │       └─ Teensy 4.1 USB-serial ↔ CAN1
                             │           ├─ Universal Heartbeat 0x01011840 @ 50 Hz
                             │           ├─ VELOCITY_SETPOINT cls=0 idx=0 → SparkMAX velocity PID
                             │           └─ STATUS_0/STATUS_2 decode → E line
                             │               └─ SparkMAX FW 26.1.4 → NEO brushless → 12.75:1 ToughBox Mini
                             │                   → 22T:22T #35 chain → 20T × 0.5" pulley → timing belt track
                             └─ back: /wheel_odom (integrated from E-line positions) → EKF
```

### Serial Protocol (115200 baud, line-oriented)

Documented in `firmware/teensy_diff_drive/CLAUDE.md`. Summary:

| Host → Teensy | Effect |
|---|---|
| `L<rpm> R<rpm>` | velocity mode setpoint per wheel — SparkMAX onboard PID handles loop |
| `UL<d> UR<d>` | duty-cycle mode setpoint (-0.3..0.3 clamped) |
| `S` | stop — switches to MODE_DUTY=0 so SparkMAX Brake idle engages |
| `D` | DIAG line with tx/rx counts, watchdog state, mode, L/R meas/cmd, bus voltage |
| `K[PIDF]<val>` | write PID slot-0 gain to both SparkMAXes via PARAMETER_WRITE (cls=14 idx=0) |
| `BURN` | PERSIST_PARAMETERS (cls=63 idx=15) — write RAM gains to SparkMAX flash |

### Control priority (actuator_node)

Unified (v, ω) target computed from whichever input is freshest:
1. Fresh `/avros/actuator_command` (< 500 ms) — webui path. `throttle/brake/steer` mapped to (v, ω).
2. Fresh `/cmd_vel` (< 500 ms) — teleop / Nav2 path. `linear.x`, `angular.z` used directly.
3. Neither → (0, 0) target.

**Both paths go through the same slew-rate limiter + heading-hold**, so webui and Nav2 commands behave identically.

---

## Diff-Drive Parameters

- **Track gauge (centerline to centerline):** 0.7366 m (29 inches)
- **Ground per motor revolution:** 0.01994 m (π × 80.85 mm drive-pulley pitch dia / 12.75:1 gearbox)
- **Theoretical top speed:** 1.89 m/s at NEO free speed (5676 RPM)
- **Measured Phase 4 max RPM extrapolated:** L = 5532 (97.5% free), R = 5072 (89.4% free) — right track has 8% higher friction
- **SparkMAX PID gains (tuned in Phase 6):** kFF=0.000197, kP=0.0004, kI=0, kD=0
- **Actuator-node slew caps:** max_linear_accel = 1.0 m/s², max_linear_decel = 1.5 m/s², max_angular_accel = 2.0 rad/s²
- **Speed caps:** max_linear_mps = 1.5, max_angular_rps = 1.0
- **WebUI max_throttle:** 1.0 (full cmd_vel range; clamped by max_linear_mps)
- **Robot radius:** 0.8 m, inflation: 0.7 m
- **Idle mode:** **Brake** (set via REV Hardware Client on both SparkMAXes — required for quick stops)
- **Motor inversion:** one SparkMAX has `Motor Inverted = true` so `L+ R+` produces forward ground motion on both tracks

---

## Nav2 Config

- **Route Server:** nav2_route with GeoJSON campus road graph (52 nodes, 113 edges)
- **Planner:** SmacPlannerHybrid (DUBIN, min radius 2.31 m) — fallback for off-graph planning
- **Controller:** Regulated Pure Pursuit (lookahead 3-20 m)
- **BT:** `navigate_route_graph.xml` — ComputeRoute → FollowPath (no spin/backup recovery)
- **Local costmap:** VoxelLayer (LiDAR) + InflationLayer, 10x10 m
- **Global costmap:** ObstacleLayer + InflationLayer, 100x100 m rolling
- **Goal tolerance:** 2.0 m xy, 0.5 rad yaw
- **Datum:** 34.059270, -117.820934 (fixed in navsat.yaml, used by route graph)

---

## Web UI (avros_webui)

Phone-based joystick controller for bench testing. FastAPI + WebSocket + nipplejs.

- **Launch:** `ros2 launch avros_bringup webui.launch.py`
- **URL:** `https://<jetson-ip>:8000` (self-signed cert required for phone WebSocket)
- **Control path:** phone joystick → WebSocket → webui_node → `/avros/actuator_command` → actuator_node (diff-drive inverse + heading-hold + slew-rate) → Teensy serial → SparkMAX velocity PID
- **Priority:** ActuatorCommand (direct) takes precedence over cmd_vel (PID). When webui stops publishing, timeout expires and Nav2's cmd_vel takes over.
- **Safety:** WebSocket disconnect → e-stop published automatically
- **Features:** proportional joystick (throttle/brake/steer), E-STOP button, drive mode buttons (N/D/S/R), live telemetry from ActuatorState
- **Config:** `avros_bringup/config/webui_params.yaml` — port, SSL paths, max_throttle
- **SSL setup:** `mkdir -p ~/avros_certs && openssl req -x509 -newkey rsa:2048 -keyout ~/avros_certs/key.pem -out ~/avros_certs/cert.pem -days 365 -nodes -subj '/CN=AVROS'`
- **Pip deps:** `pip install fastapi uvicorn[standard] websockets`
- **On Jetson:** SSL cert paths set in webui_params.yaml to `/home/dinosaur/avros_certs/{cert,key}.pem`

---

## Remote Desktop (NoMachine)

NoMachine 9.4.14 installed on the Jetson — full GNOME desktop over Tailscale with hardware H.264 encoding on the Jetson Orin GPU.

- **Server:** Jetson, listening on port `4000` (NX protocol)
- **Connect from laptop:** NoMachine client → host `100.93.121.3` (Tailscale) → port `4000` → user `dinosaur` (same password as `ssh jetson`)
- **Headless config:** `gdm.service` is stopped + disabled. NoMachine creates its own virtual X display on demand (`:1001` typical), so there's no GDM greeter to mirror — connections always get a fresh GNOME session via `DefaultDesktopCommand`.
- **Why GDM is disabled:** if GDM is running but no user is logged in graphically, NoMachine attaches to the empty greeter and shows a black screen. See [NoMachine KB AR03P00973](https://kb.nomachine.com/AR03P00973). Bring it back with `sudo systemctl enable --now gdm` only if you attach a real monitor + want auto-login.

### Launching GUI apps from this host (Claude or user) into the live NoMachine session

Anything sent with `DISPLAY=:1001` renders inside the active NoMachine window. `setsid` + redirected I/O detaches the child so it survives SSH closing.

```bash
# Pattern
ssh jetson "DISPLAY=:1001 setsid <command> >/tmp/<name>.log 2>&1 < /dev/null &"

# RViz with a saved config (sources ROS env first)
ssh jetson "DISPLAY=:1001 setsid bash -c 'source /opt/ros/humble/setup.bash && source /home/dinosaur/IGVC/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && exec rviz2 -d /home/dinosaur/IGVC/src/avros_bringup/rviz/three_cam.rviz' >/tmp/rviz.log 2>&1 < /dev/null &"

# Quick gnome-terminal in the session
ssh jetson "DISPLAY=:1001 setsid gnome-terminal >/dev/null 2>&1 < /dev/null &"
```

The display number `:1001` is the default for the first NoMachine virtual session; verify with `ls /tmp/.X11-unix/` (look for `X1001`). It changes if you're using multi-user / multiple parallel NoMachine sessions.

### Works / doesn't work on the virtual display

| Works | Broken on virtual display |
|---|---|
| gnome-terminal, file manager, browsers, gedit, rqt, rqt_image_view | ZED_Explorer / ZED_Depth_Viewer / anything Argus-based |
| RViz2 (uses software OpenGL via llvmpipe — fine for TF + costmaps + small clouds, slow with dense clouds) | Gazebo, Isaac Sim |
| Foxglove Studio (if you'd rather run it on the Jetson than the laptop) | Anything needing CUDA-EGL interop |

ZED tools fail because Argus needs GPU-backed EGL which NoMachine's virtual display doesn't provide (`No current CUDA context available; nvbufsurface: Failed to create EGLImage`). Fix: HDMI dummy plug on the Jetson — Jetson then runs a real DRM display, NoMachine mirrors that, and CUDA-EGL works. For routine perception verification, just use the ZED ROS wrapper + Foxglove instead — same data, no GPU-display issue.

### NoMachine install reference

```bash
# Server (Jetson, arm64) — current as of 2026-04-27
wget https://web9001.nomachine.com/download/9.4/Arm/nomachine_9.4.14_1_arm64.deb -O /tmp/nm_arm64.deb
sudo dpkg -i /tmp/nm_arm64.deb
# postinst hangs in `nxserver --subscription` on ARM — kill it after a few minutes,
# then `sudo dpkg --configure -a` to finalize. Daemon is functional once port 4000 listens.

# Client (laptop, amd64)
wget https://web9001.nomachine.com/download/9.4/Linux/nomachine_9.4.14_1_amd64.deb -O /tmp/nm_amd64.deb
sudo dpkg -i /tmp/nm_amd64.deb     # CUDA-init warning is harmless — laptop has no NVIDIA GPU
```

---

## Launch Files

| Launch File | What it starts |
|-------------|---------------|
| `sensors.launch.py` | robot_state_publisher + velodyne driver/convert + realsense + **zed_front (conditional)** + xsens + ntrip |
| `actuator.launch.py` | actuator_node only |
| `teleop.launch.py` | actuator_node + teleop_twist_keyboard |
| `webui.launch.py` | actuator_node + webui_node |
| `localization.launch.py` | EKF + navsat_transform |
| `navigation.launch.py` | Full stack: sensors + localization + Nav2 + route_server + **perception (conditional)** |
| `perception.launch.py` (avros_perception) | `perception_node` for one ZED camera (camera_name:=front default) |

---

## Config Files

| Config | Used By |
|--------|---------|
| `actuator_params.yaml` | actuator_node — serial port, track width, speed/accel limits, IMU heading-hold gains, SparkMAX PID gains pushed on startup |
| `velodyne.yaml` | velodyne_driver_node + velodyne_convert_node |
| `realsense.yaml` | realsense2_camera_node |
| `xsens.yaml` | xsens_mti_node — IMU/GNSS, lever arm, output rate |
| `webui_params.yaml` | webui_node — port, SSL, max throttle |
| `ekf.yaml` | robot_localization EKF |
| `navsat.yaml` | navsat_transform_node |
| `nav2_params.yaml` | Nav2 (planner, controller, costmaps, BT, route_server) |
| `navigate_route_graph.xml` | BT tree using ComputeRoute (nav2_route) instead of ComputePathToPose |
| `cpp_campus_graph.geojson` | Pre-built CPP campus road graph for nav2_route (map-frame coords) |
| `cyclonedds.xml` | CycloneDDS config — shared memory disabled, socket buffer 10MB |
| `ntrip_params.yaml` | ntrip_client — NTRIP caster host, port, mountpoint, credentials |
| `zed_front.yaml` | zed_wrapper (front) — camera_model zedx, serial 49910017, HD720@15fps, organized pointcloud |
| `zed_left.yaml`, `zed_right.yaml`, `zed_back.yaml` | unused in Phase 3; retained for Phase 5 multi-camera bring-up |
| `avros_perception/config/perception.yaml` | `perception_node` — camera_name, pipeline selector (stub/hsv/onnx), sync slop, stub stripe params |
| `avros_perception/config/class_map.yaml` | single source of truth for class ID ↔ name ↔ RGB; consumed by node + `vision_msgs/LabelInfo` publisher |

---

## DDS Config

CycloneDDS (`cyclonedds.xml`):
- Socket receive buffer: 10 MB minimum
- Shared memory: **disabled** (iceoryx RouDi daemon not running; `<SharedMemory><Enable>false</Enable></SharedMemory>`)
- Network: auto-detect interface
- Set via: `CYCLONEDDS_URI=file://<path>` in sensors.launch.py
- **Must also set** `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (defaults to FastDDS otherwise)

---

## Known Issues & Fixes

SparkMAX FW 26.1.4 CAN protocol gotchas (cls=14 PARAMETER_WRITE, cls=0 VELOCITY_SETPOINT, STATUS_2 enable): see `firmware/teensy_diff_drive/CLAUDE.md`. RealSense one-time install issues: see `docs/REALSENSE_SETUP.md`.

| Issue | Fix |
|-------|-----|
| Port 8000 held after webui crash/disconnect | `fuser -k 8000/tcp` before relaunch |
| RealSense USB interface busy on relaunch | `pkill -f realsense2_camera_node` + wait 2s before relaunching |
| Xsens driver package name | Correct name is `xsens_mti_ros2_driver` (not `xsens_ros_mti_driver`) |
| Xsens driver missing deps | `ros-humble-mavros-msgs` and `ros-humble-nmea-msgs` must be installed via apt |
| CycloneDDS iceoryx/RouDi errors on launch | SharedMemory must be disabled in `cyclonedds.xml` unless RouDi daemon is running — set `<SharedMemory><Enable>false</Enable></SharedMemory>` |
| numpy binary incompatibility on Jetson | Pin `numpy<2` — numpy 2.x breaks system matplotlib/scipy on JetPack 6 |
| RMW_IMPLEMENTATION not set | Must export `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in addition to `CYCLONEDDS_URI` — defaults to FastDDS otherwise |
| CLI commands get (0,0) goals / RMW mismatch | Launch file sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` for nav2 nodes, but `ros2` CLI tools use the shell default (FastDDS). FastDDS→CycloneDDS interop corrupts action goal payloads (poses arrive zeroed). Fix: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in `.bashrc` or before any `ros2` CLI command |
| route_server "Failed to transform from '' to map" | `global_frame` param missing from route_server config — defaults to empty string, so `getRobotPose()` uses empty frame_id. Fix: add `global_frame: "map"` to route_server params |
| NTRIP client no data | Verify credentials and mountpoint in `ntrip_params.yaml`; check internet access from Jetson; try `enable_ntrip:=false` to isolate |
| NTRIP `mountpoint` still `CHANGE_ME` | Edit `ntrip_params.yaml` — pick a nearby mountpoint from your caster (e.g. rtk2go.com mount list) |
| **Jetson crashes randomly during motor testing** | Shared 12 V rail — Jetson and SparkMAXes both fed from the 48V→12V buck. Motor inrush (~200A transient) sags the rail below Jetson brown-out threshold. Fix: dedicated 48V→19V buck for Jetson, separate from motor rail. Persistent journald now enabled for post-crash forensics. |
| SparkMAX velocity mode caps at ~2450 RPM | Not `kOutputMax_0` — it's velocity-PID + Brake-idle oscillation. Slew-rate limit in actuator_node + proper kFF (= 1/max_loaded_RPM) fixes it. |
| Motors spin opposite directions under `L+ R+` | Mirror-mounted motors. Fix: check "Motor Inverted" on ONE SparkMAX via REV Hardware Client (Basic tab). Inverts both output and encoder sign so firmware sees consistent direction. |
| Blinking magenta on SparkMAX | "Brushless + Coast + NO valid signal" — heartbeat gap > 100 ms. Most commonly caused by overly aggressive `!Serial` gating on the Teensy during USB CDC traffic. Current firmware has no `!Serial` guard. |
| Hardware Client unreachable over CAN | Unplug CAN wire from the SparkMAX before USB-C config — Hardware Client and Teensy fight for the bus otherwise. |
| kiwicampus/semantic_segmentation_layer fails to build on Humble | `humble` branch depends on modern Nav2 imported CMake targets that don't exist on Humble, and is missing `#include <deque>`. Apply [PR #1](https://github.com/kiwicampus/semantic_segmentation_layer/pull/1) via `scripts/apply_kiwicampus_patches.sh` after `vcs import`. |
| kiwicampus layer silently ignores frames | Usually a topic-contract mismatch. Mask and pointcloud MUST share `header.stamp` (avros_perception uses the image stamp on both); pointcloud MUST be organized (`height > 1`); `vision_msgs/LabelInfo` MUST be published with `transient_local` + `reliable` QoS so late-joining plugin gets the latched message. Also the mask HxW MUST equal the cloud HxW — they diverge by default (cloud uses `point_cloud_res: COMPACT` independent of image downscale), so `perception_node` resizes the image to the cloud shape before running the pipeline. |
| kiwicampus error: "no class types defined for source X" | `class_types: [...]` + per-type blocks must live **inside** the per-source block (`semantic_front.front.class_types`), not at the plugin top level. README formatting is ambiguous; the plugin source declares them under `layer_name.source.class_types` only. |
| ZED wrapper launches a "libexec directory not found" error | `zed_wrapper` is a metapackage — the camera is a composable component (`stereolabs::ZedCamera`) in `zed_components`, loaded by `zed_wrapper/launch/zed_camera.launch.py`. Use `IncludeLaunchDescription` of that file, not a raw `Node(package='zed_wrapper', executable='zed_wrapper')`. |
| ZED topics show up at `/zed_front/zed_front/...` instead of `/zed_front/zed_node/...` | Passing both `namespace` and `node_name` to `zed_camera.launch.py` silently overwrites `node_name` with `camera_name` (`zed_camera.launch.py:242-245`). Fix: pass only `camera_name` + `camera_model` + `serial_number`; let the wrapper default `namespace = camera_name`, `node_name = 'zed_node'`. |
| ZED image/cloud `frame_id` is unknown in TF tree | `zed-ros2-wrapper` publishes in frames created by `zed_wrapper/urdf/zed_macro.urdf.xacro` (`<cam>_camera_link`, `_left_camera_frame_optical`, etc.). A hand-rolled `zed_front_camera_center` link leaves those unresolved. Fix: in your URDF, `<xacro:include filename="$(find zed_wrapper)/urdf/zed_macro.urdf.xacro"/>` + `<xacro:zed_camera name="zed_front" model="zedx">` + a `base_link → zed_front_camera_link` joint. |
| ZED `InvalidParameterValueException` during init | v5.2 deprecated the old depth-mode and ZED-X resolution enums. `depth_mode` must be `NONE | NEURAL_LIGHT | NEURAL | NEURAL_PLUS` (no more `PERFORMANCE/QUALITY/ULTRA`); `grab_resolution` for ZED X must be `HD1200 | HD1080 | SVGA | AUTO` (no `HD720`). Verify the wrapper's own `config/zedx.yaml` for the authoritative enum. |
| ZED v5 topic names differ from v4 | v5 uses `/rgb/color/rect/image` and `/rgb/color/rect/camera_info`. Older examples / tutorials may reference `/rgb/image_rect_color` which was v4. Downstream subscribers (perception_node) default to the v5 path. |
| ZED `serial_number` in YAML appears ignored | It's a **launch arg**, not a YAML param — the wrapper's launch overrides any YAML value. Pass `'serial_number': '<N>'` inside the `launch_arguments` dict on the `IncludeLaunchDescription`; don't put it in the override YAML. |

---

RealSense D455 install procedure (RSUSB build, FW 5.13.0.50 downgrade, apt removal): see `docs/REALSENSE_SETUP.md`.

---

## TODOs

### Sensors / perception
- [ ] Measure physical sensor mount positions on vehicle (URDF imu_link, velodyne, camera_link)
- [ ] Calibrate GNSS lever arm in xsens.yaml (antenna offset from IMU)
- [ ] Test localization stack (EKF + navsat)
- [ ] Configure NTRIP credentials in ntrip_params.yaml (mountpoint, username, password)
- [ ] Verify RTK FIXED/FLOAT status with NTRIP corrections enabled
- [ ] **Re-evaluate `twist0: /filter/twist` removal from `ekf.yaml`** — removed 2026-04-28 on the assumption it was broken (CLI `ros2 topic echo` refused the topic due to dual-publisher type conflict). On reflection that was a CLI limitation, not a confirmed runtime fault; EKF subscription showed it had bound to the correct `TwistWithCovarianceStamped` publisher. Re-add the block, A/B test EKF position accuracy with vs without it (compare `/odometry/filtered` to `/odometry/global` GPS-anchored ground truth over a fixed drive path). Decide based on data, not the earlier inference.
- [ ] **Investigate `/filter/twist` dual-publisher origin** — `ros2 topic info /filter/twist --verbose` should name both nodes. Likely the legitimate one is the Xsens driver (`TwistWithCovarianceStamped`); the other is some accidental relay or duplicate config. Tied to the previous TODO — once we know which publisher is which, removal/keep decision becomes obvious.

### Actuator / drivetrain
- [ ] **Dedicated 48V→19V buck for Jetson** (separate from motor rail) — blocks safe field testing
- [ ] Phase 7 BURN persistence verification (manual power-cycle readback)
- [ ] Ground-drive test with `/imu/data` active (launch sensors.launch.py alongside webui)
- [ ] Tune heading_kp / yaw_rate_kp if drift or wobble under real driving
- [ ] Loosen right track (optional) — Phase 3 showed 2× stiction asymmetry vs left
- [ ] **5 m straight-line wheel-odom calibration test** — drive 5 m on a tape-measured line, compare EKF `pose.position.x` to actual. Validates `m_per_rev` (wheel diameter / gear ratio) and trans-velocity covariance. 360° spin already done 2026-04-28 — wheels +0.7% (effective track ~0.7416m vs configured 0.7366m); EKF fused result 358.34° (acceptable). Translational accuracy still untested.

### Navigation / integration
- [ ] Test full Nav2 navigation stack after power rail is fixed
- [ ] Commit SSL cert paths for Jetson (currently only set locally)
- [ ] Run `vcs import src < avros.repos` on Jetson to standardize source deps

### Perception / semantic segmentation
- [ ] **Write `kiwicampus_align_purge_clocks.patch`** — proper architectural fix for the dual-clock decay bug we worked around 2026-04-28 by bumping `tile_map_decay_time: 1.5 → 5.0`. `bufferSegmentation` purges with `cloud_time_seconds` (sensor stamp); `updateBounds` purges with `node->now()` (wall clock); same observation gets evaluated against two different "now"s. Standard fix: change `semantic_segmentation_layer.cpp:339` to use the buffer's last-observation cloud stamp instead of `node->now()`. Once applied, decay can revert to kiwicampus default 1.5 in both `perception_test_params.yaml` and `nav2_params_humble.yaml` (×3). Add to `scripts/apply_kiwicampus_patches.sh` after `kiwicampus_pr2_mutex.patch`.
- [ ] **Tighten HSV `pothole_*` thresholds in `pipelines/hsv.py`** — concrete walkway is being misclassified as pothole at ~14% of frame in outdoor field tests (verified 2026-04-28 by overlay snapshot). Pipeline supports live tuning via dynamic params per `ace533e` ("Live-tune HSV params"). Tighten H/S/V ranges to exclude medium-gray concrete shadows.
- [ ] **Document kiwicampus tile_map `frame_id="map"` mislabel** — the `/local_costmap/<source>/tile_map` viz topic is published with hardcoded `frame_id: "map"` regardless of the buffer's actual `global_frame_` (which is `odom` for a local_costmap). Coordinates *in* the message are correct, the frame_id label is just wrong. Add to `patches/README.md` (or create one) so the next person debugging spatial issues in Foxglove/RViz doesn't waste time on this.
- [ ] Re-tighten `max_obstacle_distance` (currently 100m for bench-test) back to ~5–8m before field tests
- [ ] Measure + commit real `zed_front` mount offset in URDF (currently placeholder `1.0 0 0.9`)
- [ ] Phase 5: scale to 3 cameras (front + left + right); decide whether to drop RealSense from VoxelLayer
