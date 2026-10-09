# Vehicle Integration Test Plan — W (14 Phases, post-STVL, NTRIP-off)

**Purpose:** comprehensive granular ladder for validating every subsystem of the IGVC_ROS2 stack on the real vehicle. Each phase isolates one subsystem; each step has a precise Action / Go / No-go gate. Designed to be run from bench (P0–P4) through to full IGVC-like integration scenarios (P13).

**Supersedes:** `docs/V_vehicle_integration_test_plan.md` (V0–V11). That plan was written before:
1. The STVL migration (commits `4450003` + `4d04e3a`) — local_costmap LiDAR source moved from `nav2_costmap_2d::VoxelLayer` to `spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer`. Clearing semantics completely changed (time-based decay + frustum acceleration, not raytrace).
2. NTRIP disabled (commit context, IGVC §I.2 rule) — GPS standalone/SBAS only, ~1–3 m accuracy instead of cm-class.
3. `observation_persistence` revert (commit `9ea082d`) — the b0b028b setting of 2.0 made clearing slower, not faster.

V0–V11 specific failure modes from the old plan have either been resolved (clearing on stationary robot, by STVL migration) or are still applicable but renumbered into P13.

**Doc staleness corrections this plan calls out:**
- CLAUDE.md "Nav2 Config" → planner is `nav2_navfn_planner/NavfnPlanner` (Dijkstra holonomic), NOT `SmacPlannerHybrid/DUBIN/2.31 m`. Smac was reverted because its 2.31 m turning radius can't fit IGVC 2-3 m lanes, and tracked diff-drive has 0 m turn radius. (Verify: `ros2 param get /planner_server GridBased.plugin`.)
- CLAUDE.md TF tree → `navsat_transform_node` has `broadcast_cartesian_transform: false`; map→odom is broadcast by the **global EKF** (`ekf_filter_node_map`), not navsat_transform.

---

## Safety contract (unchanged from V-plan)

- **Wireless e-stop in your hand is primary safety.** Foxglove/Tailscale is secondary.
- **One operator at a time** with visual line of sight to the robot.
- **Speed cap stays low** until P11 passes. Default `vx_max: 0.7`; bump to 1.0 only after P10.
- **Fail-closed.** Any unexpected behavior → e-stop → rollback to prior phase → debug → retry.
- **No chaining phases in one launch.** Stop and re-launch fresh between phases that change params.

## Universal pre-session checklist (every session)

1. Battery >70 %.
2. E-stop button physically tested today.
3. `ssh jetson` succeeds, returns `dinosaur` in <2 s.
4. `ping -c 1 192.168.13.11` → Velodyne reachable.
5. `ros2 topic hz /gnss` → ≥4 Hz with `status ≥0`. If empty or status=−1, do not proceed.
6. `ZED_Explorer` (via NoMachine) sees serial `49910017`.
7. `navsat.yaml` datum within ~100 m of test site.
8. Re-verify P0 and P1 once per session before doing anything new.

---

## PHASE 0 — Workstation + Jetson Pre-flight (5 steps, ~10 min, BENCH)

**Goal:** baseline that everything outside ROS is ready before launching.

**P0.1 — Battery / power rail.** Visually inspect HV pack voltage; multimeter on Jetson 12 V rail. **Go:** pack ≥48.0 V, Jetson rail 11.8–12.4 V steady. **No-go:** rail sags >0.3 V on touch, or pack <46 V (brown-out risk).

**P0.2 — SSH reachability.** `ssh -o ConnectTimeout=5 jetson 'uptime && whoami'`. **Go:** returns `dinosaur` and load avg in <2 s. **No-go:** timeout, wrong user, or "Host key changed".

**P0.3 — Workspace integrity.** `ssh jetson 'ls ~/IGVC/install/setup.bash ~/IGVC/src/avros_bringup/config/cyclonedds.xml && du -s ~/IGVC/install'`. **Go:** both files exist, `install/` non-empty. **No-go:** missing file → not built (`~/AVROS` is dead, do not use).

**P0.4 — NoMachine display socket.** `ssh jetson 'ls /tmp/.X11-unix/'`. **Go:** `X1001` (or current NoMachine session) present. **No-go:** only `X0` / empty → NoMachine session not attached; GUI launches will fail silently.

**P0.5 — ros2 daemon clean state.** `ssh jetson 'source /opt/ros/humble/setup.bash && ros2 daemon stop; ros2 daemon start; ros2 daemon status'`. **Go:** "The daemon is running". **No-go:** "could not contact daemon" → stale Python env or ROS not sourced.

---

## PHASE 1 — DDS Hygiene (7 steps, ~10 min, BENCH)

**Goal:** prevent the RMW-mismatch class of bugs (CLAUDE.md "CLI commands get (0,0) goals").

**P1.1 — Shell RMW env vars set.** `ssh jetson 'printenv RMW_IMPLEMENTATION CYCLONEDDS_URI'`. **Go:** prints `rmw_cyclonedds_cpp` and `file:///home/dinosaur/IGVC/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml`. **No-go:** either var empty → CLI will default to FastDDS and corrupt action goals.

**P1.2 — cyclonedds.xml has SharedMemory disabled.** `grep -i -A1 SharedMemory ~/IGVC/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml`. **Go:** `<Enable>false</Enable>`. **No-go:** `true` or missing → iceoryx/RouDi errors on every launch.

**P1.3 — RouDi daemon NOT running.** `pgrep -a iox-roudi; pgrep -a RouDi; echo done`. **Go:** prints only `done`. **No-go:** any PID → conflicts with disabled-SHM config.

**P1.4 — ros2 doctor baseline.** `ros2 doctor --report | head -60`. **Go:** `middleware name : rmw_cyclonedds_cpp`, no ERROR lines, platform Humble/jammy. **No-go:** middleware = fastrtps, or version mismatch.

**P1.5 — Multicast loopback.** Terminal A `ros2 multicast receive`; terminal B `ros2 multicast send`. **Go:** receiver prints "Received from..." within 1 s. **No-go:** timeout → firewall, wrong NIC, or no multicast route.

**P1.6 — Launch-vs-CLI RMW parity.** Launch sensors, then in a new terminal `ros2 topic list | grep velodyne && ros2 topic hz /velodyne_points`. **Go:** `/velodyne_points` listed and ~10 Hz. **No-go:** empty list with launch alive → re-export P1.1 vars in the new terminal.

**P1.7 — Daemon RMW matches shell.** `ros2 daemon status && ros2 node list | head`. **Go:** daemon running, node list includes `/velodyne_driver_node`, `/xsens_mti_node`. **No-go:** empty node list → daemon started under stale RMW; rerun P0.5.

---

## PHASE 2 — URDF + Static TFs (12 steps, ~15 min, BENCH)

**Goal:** every sensor mount frame loaded correctly from URDF.

**Pre-conditions:** `ros2 launch avros_bringup sensors.launch.py` running.

**P2.1 — Launch console clean.** **Go:** no `[ERROR]` lines, all nodes spawn. **No-go:** xacro parse error, missing dep.

**P2.2 — /robot_description populated.** `ros2 param get /robot_state_publisher robot_description`. **Go:** non-empty XML (>1000 chars), contains `<robot>`, `<link name="base_link">`. **No-go:** empty or malformed.

**P2.3 — /tf and /tf_static topics exist.** `ros2 topic list | grep -E "tf|transform"`. **Go:** both present.

**P2.4 — base_link → imu_link.** `ros2 run tf2_ros tf2_echo base_link imu_link`. **Go:** translation `[0, 0, 0.5556]`, rotation identity `[0, 0, 0, 1]`. **No-go:** mismatch >0.001 m or frame not found.

**P2.5 — base_link → velodyne.** `tf2_echo base_link velodyne`. **Go:** translation `[0.089, 0.0, 0.7146]`, rotation identity.

**P2.6 — base_link → zed_front_camera_link.** **Go:** translation `[0.6795, 0.0, 0.4476]`, rotation = pitch +15° quaternion ≈ `[0, 0.1305, 0, 0.9914]`. **No-go:** translation off >0.001 m, or yaw/roll non-zero.

**P2.7 — base_link → zed_left_camera_link.** **Go:** translation `[0.098, 0.286, 0.6126]`, rotation = yaw +90° quat `[0, 0, 0.7071, 0.7071]`. (Will only succeed if `enable_zed_left:=true`; otherwise skip.)

**P2.8 — base_link → zed_right_camera_link.** **Go:** translation `[0.098, -0.286, 0.6126]`, rotation = yaw −90° quat `[0, 0, -0.7071, 0.7071]`. Skip if not enabled.

**P2.9 — base_link → base_footprint.** **Go:** translation `[0, 0, 0]`, rotation identity.

**P2.10 — view_frames PDF.** `ros2 run tf2_tools view_frames.py && ls -la /tmp/frames_*.pdf`. **Go:** PDF created, >0 bytes.

**P2.11 — PDF sanity inspect.** Frames present: `base_link`, `base_footprint`, `imu_link`, `velodyne`, `zed_front_camera_link` (+ subframes `_camera_center`, `_left_camera_frame`, `_right_camera_frame`, `_left_camera_frame_optical`, `_right_camera_frame_optical`). **No-go:** orphan or cycle.

**P2.12 — robot_state_publisher params present.** `ros2 param list | grep robot_state_publisher`. **Go:** includes `:robot_description` and `:use_sim_time`.

---

## PHASE 3 — Sensor Raw Data (12 steps, ~20 min, BENCH)

**Goal:** each sensor publishes correct topic at correct rate with correct frame_id.

**Pre-conditions:** `ros2 launch avros_bringup sensors.launch.py enable_zed_front:=true enable_ntrip:=false` running.

### Velodyne VLP-16 (4 steps)
**P3.1 — Reachability.** `ping -c 3 192.168.13.11` then `sudo tcpdump -i any -c 5 udp port 2368`. **Go:** ≥3 ICMP replies AND ≥5 UDP packets.

**P3.2 — Topic existence + frame_id.** `ros2 topic list | grep velodyne` and `ros2 topic echo /velodyne_points --once --field header.frame_id`. **Go:** both `/velodyne_packets` + `/velodyne_points`; frame_id == `velodyne`.

**P3.3 — Rates.** `timeout 10 ros2 topic hz /velodyne_points` and `/velodyne_packets`. **Go:** points 9–11 Hz, packets 750–760 Hz.

**P3.4 — Organized cloud sanity.** `ros2 topic echo /velodyne_points --once --no-arr --field height/width/point_step/is_dense`. **Go:** height==16, width≈1800±200, point_step==22, is_dense==false (organize_cloud=true confirmed; NaN no-return points present).

### Xsens MTi-680G (4 steps)
**P3.5 — Device present.** `ls -l /dev/ttyUSB0 && udevadm info /dev/ttyUSB0 | grep -i xsens`. **Go:** device exists, vendor==Xsens.

**P3.6 — Topics + frame_id.** `ros2 topic list | grep -E '^/(imu|gnss|nmea|filter|status)'` and `ros2 topic echo /imu/data --once --field header.frame_id`. **Go:** `/imu/data`, `/gnss`, `/nmea`, `/filter/quaternion`, `/status` present; frame_id == `imu_link`.

**P3.7 — Rates.** **Go:** `/imu/data` 95–105 Hz, `/gnss` and `/nmea` 3.6–4.4 Hz.

**P3.8 — Data sanity + NTRIP off.** `ros2 topic echo /imu/data --once` → orientation_covariance[0] != -1 AND any diagonal > 0 (filter converged). `ros2 topic echo /gnss --once` → status.status ≥0, lat ≈34.06, lon ≈−117.82. `ros2 topic hz /rtcm` → no messages (confirms NTRIP truly off).

### ZED X Front (4 steps)
**P3.9 — SDK probe.** `ZED_Explorer --probe` (needs real DRM display per CLAUDE.md; NoMachine virtual display will fail with CUDA-EGL error). **Go:** serial 49910017 listed. **Alternative:** `/dev/video*` enumerates.

**P3.10 — Topics + frame_id.** `ros2 topic list | grep zed_front/zed_node` and `ros2 topic echo .../rect/image --once --field header.frame_id`. **Go:** rect/image, camera_info, point_cloud/cloud_registered all under `/zed_front/zed_node/...` (NOT `/zed_front/zed_front/...`); frame_id == `zed_front_left_camera_frame_optical`.

**P3.11 — Rates.** **Go:** rect/image and point_cloud both 13.5–16.5 Hz.

**P3.12 — Data sanity.** `ros2 topic echo .../rect/image --once --no-arr`: encoding ∈ {bgra8, bgr8, rgb8}, height==540, width==960. Point cloud: height==256, width==448, point_step==32, is_dense==false. Camera_info K[0]>0.

---

## PHASE 4 — avros_perception Pipeline (13 steps, ~20 min, BENCH)

**Goal:** validate the contract between perception_node and kiwicampus semantic_segmentation_layer.

**Pre-conditions:** `enable_perception:=true perception_cameras:=front`.

**P4.1 — Topic existence + rates.** All five topics — `/perception/front/{semantic_mask, semantic_confidence, semantic_points, label_info, overlay}` — exist; first four publish at ≥8 Hz; `label_info` published once at startup.

**P4.2 — QoS compliance.** `ros2 topic info /perception/front/semantic_mask -v` → BEST_EFFORT + VOLATILE. `/perception/front/label_info -v` → RELIABLE + TRANSIENT_LOCAL. **No-go:** wrong QoS on label_info → kiwicampus won't receive latched message.

**P4.3 — Sync fires.** Subscribe to mask + check header.stamp vs latest image/cloud timestamp. **Go:** mask stamp matches input within sync_slop=0.02 s. **No-go:** stamps 3+ frames stale = sync dropping pairs.

**P4.4 — label_info latched.** Kill perception_node, subscribe to label_info, then relaunch. **Go:** new subscriber gets LabelInfo immediately (latched); contains ids [0,1,2,3,255] mapped to names matching class_map.yaml. **No-go:** subscriber waits for next frame = QoS bug.

**P4.5 — Mask H×W == Cloud H×W.** Spot 5 frames. **Go:** identical (perception_node resizes mask to point_cloud_res=COMPACT, 256×448).

**P4.6 — Confidence H×W matches.** Same check, confidence vs cloud. **Go:** identical.

**P4.7 — Overlay H×W matches mask.** **Go:** identical.

**P4.8 — Mask contains expected class IDs.** Capture frame with known lane. **Go:** unique values include 0 (free) + 1 (lane_white). **No-go:** all 0 (no detection) or all 1 (oversaturation) or id 255 (unknown leakage).

**P4.9 — HSV lane detection.** Point camera at white stripe on grass in daylight. **Go:** stripe pixels have V ≥130 (lane_low[2]), S ≤80 (lane_high[1]); mask labels them id 1. Stripe occupies 0.1–5% of mask.

**P4.10 — Adaptive V-floor stabilization.** Run 10 frames static, move to darker scene, 5 more frames. **Go:** V-floor recomputes every 5 frames; lane detection sensitivity tracks ambient brightness.

**P4.11 — Header stamps propagate.** Mask + confidence + points all have identical header.stamp and matching frame_id (the input image frame_id).

**P4.12 — RViz overlay visualization.** Add `/perception/front/overlay` as Image display. **Go:** lane pixels tinted yellow (BGR [0,255,255]), barrel pixels orange, free pixels original color.

**P4.13 — Sync failure recovery.** Pause point cloud publisher 2 s, resume. **Go:** sync queue clears, first new pair produces mask at expected rate; no crash.

---

## PHASE 5 — STVL Local Costmap (10 steps, ~25 min, FIELD)

**Goal:** validate the JUST-MIGRATED `spatio_temporal_voxel_layer`. THE headline test of this whole exercise.

**Pre-conditions:** full nav stack running (`navigation.launch.py enable_velodyne:=true enable_zed_front:=true enable_perception:=true perception_cameras:=front enable_mission_manager:=false enable_ntrip:=false`). Robot stationary in empty area.

**Reference math (current params: voxel_decay=5.0 linear, decay_acceleration=5.0, voxel_size=0.1, model_type=1, FOV ±15°×360°):** With frustum re-observing empty at 10 Hz, **effective TTL ≈ voxel_decay / (1 + decay_accel) ≈ 0.83 s** plus 1 costmap tick (~0.2 s) projection lag → **stationary clear time ~1.0 s**.

**P5.1 — Plugin loaded.** `ros2 param get /local_costmap/local_costmap stvl_layer.plugin`. **Go:** returns `spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer`. **No-go:** still returns `nav2_costmap_2d::VoxelLayer` → migration didn't take.

**P5.2 — voxel_grid topic alive.** `ros2 topic hz /local_costmap/voxel_grid` 10 s + `ros2 topic info -v`. **Go:** type `nav2_msgs/msg/VoxelGrid`, rate ≈5 Hz (matches local_costmap update_freq), non-zero size_x/y/z.

**P5.3 — Mark latency.** Place 0.55 m traffic barrel 3 m forward, start stopwatch. Watch `/local_costmap/costmap` for value 254 at (x=3, y=0). **Go:** LETHAL within **0.4 s** (2 costmap frames). **No-go:** >0.5 s or never → check barrel top >0.2 m above ground.

**P5.4 — Decay-only clear.** SKIP by design — stationary obstacle that keeps re-marking can't trigger decay-only path. Document "N/A".

**P5.5 — Frustum-accelerated clear (HEADLINE).** With barrel from P5.3, snatch it laterally out of frame. Start stopwatch. **Go:** cell at (x=3, y=0) drops from 254 → <50 within **1.5 s ±0.5 s**. **No-go (5 s):** decay_acceleration not engaging → check `clearing: true`, `model_type: 1`, source frame_id matches LiDAR optical frame. **No-go (never clears):** STVL issue #187 — bounce costmap or check `combination_method: 1`.

**P5.6 — Voxel-size / cell snap.** Drive within 2 m of a long flat wall (e.g. shed). **Go:** LETHAL band along wall is 1–2 cells thick (voxel_size 0.1 m on costmap resolution 0.05 m = 2 cells max).

**P5.7 — Range gate.** Place barrel at 14 m forward, confirm marked. Move to 16 m, confirm not marked. 2 s settle each. **Go:** marked at 14, clear at 16 (obstacle_range=15.0).

**P5.8 — Height filter.** Lay flat objects at heights 0.0, 0.15, 0.25, 0.50 m at 4 m forward (cardboard on blocks). **Go:** only ≥0.25 m marks LETHAL. **No-go:** 0.15 marks → filter applied in sensor frame instead of global (frame interpretation bug, would re-open the Phase B finding from research).

**P5.9 — Frustum vertical limits.** Hold a 0.3 m foam target 1.5 m directly above velodyne (~90° elevation, way outside ±15°), hold 3 s. **Go:** not marked.

**P5.10 — Bag-record protocol for failures.**
```bash
ros2 bag record -o stvl_p5_fail \
  /velodyne_points /tf /tf_static \
  /local_costmap/costmap /local_costmap/costmap_updates \
  /local_costmap/voxel_grid \
  /local_costmap/published_footprint \
  /odometry/filtered /imu/data
```
Capture 10 s before + during + 10 s after each failure.

**STVL gotchas to watch:**
- Issue #187: voxels clear in OpenVDB grid but 2D projection lags. If P5.5 hangs at "voxel_grid empty but costmap red", bounce local_costmap.
- `decay_acceleration > 0` requires `model_type: 1` (3D LiDAR). Setting `model_type: 0` (depth camera / 2D) makes it a silent no-op.
- Disable `voxel_grid` RViz display before bag recording — backpressure can drop costmap updates.

---

## PHASE 6 — kiwicampus semantic_layer (9 steps, ~20 min, FIELD)

**Goal:** validate the camera-based lane LETHAL marking + decay clear path.

**Pre-conditions:** P4 + P5 passed. White tape strips available.

**P6.1 — Plugin loaded.** `ros2 param get /local_costmap/local_costmap plugins`. **Go:** list contains `"semantic_layer"` in slot 1; no pluginlib exception in costmap log.

**P6.2 — Subscriber wiring.** `ros2 topic info /perception/front/{semantic_mask, semantic_points, label_info} -v`. **Go:** all show ≥1 subscriber with matching QoS (label_info subscriber must be RELIABLE + TRANSIENT_LOCAL).

**P6.3 — LabelInfo latched delivery.** Start perception → start nav2 (plugin joins late). **Go:** one latched message arrives at plugin within 1 s of activation; costmap log shows "received label info". **No-go:** no callback → publisher QoS wrong or plugin still inactive.

**P6.4 — class_types nesting.** `ros2 param get /local_costmap/local_costmap semantic_layer.<source>.class_types`. **Go:** non-empty per-source list (e.g. `["lane"]`); top-level `ros2 param get ...class_types` fails. No "no class types defined for source X" in startup log. **No-go:** error appears → fix YAML nesting; plugin exits with `-1` per source line 175.

**P6.5 — H×W parity gate.** Mask + cloud H×W identical (P4.5 already verified; this re-confirms in nav2 context).

**P6.6 — Mark LETHAL (white tape, 3 m forward).** Lay 1 m of white tape on grass 3 m forward. **Go:** ≥1 cell at 254 within 2 s along tape footprint; aligned to robot heading ±0.2 m.

**P6.7 — Decay clear (tape removal).** Pick up tape, stopwatch. **Go:** LETHAL → FREE within `tile_map_decay_time + 1 costmap tick` (~0.3 + 0.2 = 0.5 s). No lingering gradient (would indicate half-life misconfig). **No-go:** persist >1 s → PR#1 raytrace-clear patch not built; verify `scripts/apply_kiwicampus_patches.sh` ran pre-colcon.

**P6.8 — Log-spam silence regression.** Run 5 min. **Go:** `journalctl | grep -cE "no class type defined"` returns 0. **No-go:** spam returned → check commit c9083d7 silencing patch.

**P6.9 — Lifecycle bounce.** `nav2 lifecycle deactivate /local_costmap/local_costmap; activate`. Repeat P6.3/P6.6. **Go:** plugin re-subscribes, re-receives latched LabelInfo, re-marks within 2 s.

---

## PHASE 7 — Localization (13 steps, ~25 min, FIELD)

**Goal:** EKF + navsat stack works without RTK, drift bounded.

**Pre-conditions:** clear sky view, robot stationary on flat ground.

**P7.1 — All three nodes alive.** `ros2 node list | grep -E "ekf_filter_node_odom|ekf_filter_node_map|navsat_transform"`. **Go:** all 3 present.

**P7.2 — /odometry/filtered rate.** **Go:** 28–32 Hz (configured 30 Hz in ekf.yaml).

**P7.3 — /odometry/global rate.** **Go:** 28–32 Hz. **No-go (~4 Hz):** global EKF not interpolating with IMU between GPS fixes.

**P7.4 — /odometry/gps rate.** **Go:** 3.6–4.4 Hz (tracks `/gnss`).

**P7.5 — TF root is map.** `view_frames.py` PDF inspection. **Go:** `map` is root, then `map→odom→base_link`. `map→odom` broadcaster is `ekf_filter_node_map` (NOT navsat_transform — `broadcast_cartesian_transform: false`).

**P7.6 — Datum sanity.** Check current GPS fix vs `navsat.yaml` datum (34.059270, −117.820934). **Go:** |Δx|, |Δy| < 100 m. **No-go:** site moved → update datum or `/fromLL` returns bogus offsets.

**P7.7 — Stationary drift /odometry/global.** Stand robot still 60 s. **Go:** Δx, Δy < 2.0 m. (Was 6 mm with RTK per T3.0b; without RTK we expect more but bounded.) **No-go:** >5 m → check GPS multipath, reposition.

**P7.8 — Stationary drift /odometry/filtered.** Same 60 s. **Go:** < 0.05 m. **No-go:** wheel-odom feeding into stationary EKF.

**P7.9 — GNSS status without NTRIP.** `ros2 topic echo /gnss --once --field status.status`. **Go:** ≥0 (STATUS_FIX or STATUS_SBAS=1). **No-go:** −1 (no fix) → wait longer or relocate.

**P7.10 — Manual 90° rotation.** Turn robot 90° via webui or by hand. **Go:** `/odometry/filtered` yaw changes by 90 ± 5°. **No-go:** smaller than 80° or larger than 100° → IMU calibration or ZED VIO yaw fusion broken.

**P7.11 — Manual 5 m translation.** Drive 5 m forward via webui. **Go:** `/odometry/filtered.x` advances 4.5–5.5 m. **No-go:** wheel-odom calibration off → measure track gauge + ground-per-rev.

**P7.12 — Global tracks local.** During P7.11, `/odometry/global.x` should also advance ≈5 m (within ±2 m due to GPS noise without RTK).

**P7.13 — (Stretch) 10-min loop.** Drive a slow 10-min loop returning to start. **Go:** start/end position envelope <3 m. Pass = enough for IGVC's waypoint tolerance (2.0 m).

---

## PHASE 8 — Costmap Integration (14 steps, ~20 min, FIELD)

**Goal:** local + global costmaps interact correctly; inflation, services, layer ordering.

**Pre-conditions:** full nav stack, robot stationary in open area.

**Note:** local plugins post-STVL = `[stvl_layer, semantic_layer, inflation_layer]`. Global plugins = `[obstacle_layer, inflation_layer]` (LiDAR-only). With `combination_method: 1 (Max)`, LETHAL survives across layers regardless of plugin order — order matters only if anything ever drops to Overwrite (0).

**P8.1 — Lifecycle active.** `ros2 lifecycle get /local_costmap/local_costmap` and `/global_costmap/global_costmap`. **Go:** both `active`.

**P8.2 — /local_costmap/costmap publishing.** **Go:** ~10 Hz, occupancy values present.

**P8.3 — /global_costmap/costmap publishing.** **Go:** ~0.5 Hz (publish_freq=0.5; update_freq=5 is internal).

**P8.4 — Frame correctness.** `ros2 topic echo /local_costmap/costmap --once --field header.frame_id` = `odom`. Global = `map`.

**P8.5 — Costmap origin moves with robot (rolling).** Drive 5 m. `/local_costmap/costmap.info.origin.position` should track. **Go:** origin advances ~5 m in robot's heading.

**P8.6 — STVL voxel LETHAL placement.** Place barrel 3 m forward. **Go:** LETHAL cell (254) at correct position in /local_costmap/costmap.

**P8.7 — Inflation halo geometry.** Same barrel. Inspect costmap raw values around LETHAL. **Go:** halo of values 253 down to ~50, width ~1.8 m (inflation_radius minus robot inscribed). Cost decay = `253·exp(-cost_scaling_factor·d)`.

**P8.8 — Semantic-over-stvl override (Max).** Lay tape + barrel such that tape crosses the stvl FREE region next to the barrel. **Go:** combined costmap shows LETHAL along tape line AND barrel position (Max combination wins).

**P8.9 — Double-detection stability.** Place barrel where both stvl AND semantic see it (orange barrel class). **Go:** LETHAL cells stable (no flicker between layers).

**P8.10 — Footprint self-clear.** **Go:** robot's own footprint (rectangle 1.0×0.74 m) never marks LETHAL on itself.

**P8.11 — clear_around service.** `ros2 service call /local_costmap/clear_around_local_costmap nav2_msgs/srv/ClearCostmapAroundRobot "{reset_distance: 2.0}"`. **Go:** cells within 2 m of robot cleared; new obs immediately re-marks where appropriate.

**P8.12 — clear_entirely.** `ros2 service call /local_costmap/clear_entirely_local_costmap std_srvs/srv/Empty`. **Go:** all cells reset (briefly), then new obs marks again.

**P8.13 — Local lag vs global during motion.** Drive 5 m, monitor both costmaps. **Go:** local updates visibly within 0.1 s, global within ~2 s (publish_freq 0.5 Hz).

**P8.14 — Corridor inflation sanity.** Lay tape lanes 2 m apart. **Go:** inflation halos from both tape lines just touch in middle (or leave a narrow free corridor); robot footprint should fit.

---

## PHASE 9 — Planner (10 steps, ~20 min, FIELD)

**Goal:** Navfn produces valid paths in open + obstructed scenarios.

**Note:** Live planner is `nav2_navfn_planner/NavfnPlanner`, NOT SmacPlannerHybrid. The autonav BT (`navigate_igvc_autonav_humble.xml`) uses `ComputePathToPose planner_id="GridBased"`. The route_server's `ComputeRoute` action is in a separate BT and is NOT in the autonav BT.

**P9.1 — planner_server lifecycle active.**

**P9.2 — Action surface.** `ros2 action list | grep -i -E "compute_path|navigate"`. **Go:** `/navigate_to_pose`, `/compute_path_to_pose` present. **Go (autonav BT):** `/compute_route` absent (only present when route_server launched).

**P9.3 — 3 m forward goal in empty area.** Send `ComputePathToPose` action goal at (x=3, y=0) in map. **Go:** path returned within 2 s, in `map` frame, >2 poses.

**P9.4 — Path length sanity.** Integrate Euclidean distance between consecutive poses of returned path. **Go:** ≈3.0 m ±10%.

**P9.5 — Detour around barrel.** Place barrel at (1.5, 0). Re-send same goal. **Go:** path bends around barrel with ≥0.4 m clearance (robot_radius 0.8 + inflation halo). Visualize `/plan` in RViz.

**P9.6 — Long-range goal.** Send goal at (50, 0). **Go:** path returned (global costmap is 100×100). **No-go:** path truncated → check global costmap size.

**P9.7 — Unreachable goal.** Send goal inside an obstacle, or beyond costmap. **Go:** planner returns FAILURE within ~2 s, not hang. **Note:** Navfn lacks explicit `max_planning_time` (unlike Smac); failure = wavefront never reaches goal cell.

**P9.8 — Planner params correct.** `ros2 param get /planner_server GridBased.plugin`. **Go:** `nav2_navfn_planner/NavfnPlanner`. **Sanity:** `tolerance: 0.5`, `allow_unknown: true`. **No-go:** returns SmacPlannerHybrid → YAML reverted to Smac (won't fit IGVC lanes).

**P9.9 — Planner frequency vs BT RateController.** Send a goal, watch BT log. **Go:** planner re-plans at `expected_planner_frequency: 2.0` Hz while goal active. **No-go:** plans only once → BT RateController issue.

**P9.10 — Costmap-clear branch on planner fail.** Place barrel right on robot (deliberately invalid start), send goal. **Go:** BT's planner-failure recovery branch fires `clear_entirely_global_costmap` (per `navigate_igvc_autonav_humble.xml:141`).

---

## PHASE 10 — Controller MPPI (12 steps, ~25 min, FIELD)

**Goal:** MPPI follows paths, /cmd_vel flows, speed caps respected.

**Pre-conditions:** P9 passed. Open area, ≥8 m clear forward. **Hand on e-stop.**

**P10.1 — controller_server lifecycle active.**

**P10.2 — Action surface.** `ros2 action list | grep -i follow`. **Go:** `/follow_path` present.

**P10.3 — Static-pose null.** Send NavigateToPose goal AT robot's current location. **Go:** /cmd_vel publishes near-zero (no drive-in-place). **No-go:** robot creeps or oscillates.

**P10.4 — 3 m path tracking.** Goal at (3, 0). **Go:** robot reaches xy_goal_tolerance (2.0 m) with lateral error <10 cm RMS along path. **No-go:** swerve, oscillation, overshoot.

**P10.5 — /cmd_vel control rate.** During drive, `ros2 topic hz /cmd_vel`. **Go:** matches `controller_frequency` (usually 20–50 Hz).

**P10.6 — Speed caps respected.** Watch `/cmd_vel.linear.x` over a drive. **Go:** ≤ vx_max (currently 0.7; raise to 1.0 in PRE-P10), ≤ vx_min (negative), |angular.z| ≤ max_angular_rps. **Note:** Nav2 issue #4970 (`vx_max not reached` bias at 0.5 m/s) — bump local_costmap horizon if speed clamps.

**P10.7 — CostCritic detour.** Barrel near path (offset 0.5 m). **Go:** MPPI steers cmd_vel away (visible lateral component); ≥0.5 m clearance maintained. **Tune:** if too close, bump CostCritic.cost_weight from 3.81 → 5.0.

**P10.8 — Dynamic reconfigure.** `ros2 param set /controller_server FollowPath.vx_max 1.0`. Drive again. **Go:** commanded speed obeys new cap.

**P10.9 — Tracked-vehicle slew consistency.** During turns, MPPI commands ω; actuator_node slews at 2.0 rad/s². **Go:** measured ω lags commanded by no more than the slew constant. **No-go:** drift between planned and real trajectory.

**P10.10 — Infeasible path → BT recovery.** Feed MPPI a path cutting through LETHAL cells. **Go:** BT fires Wait / BackUp / DriveOnHeading (forward-ref to P11). **No-go:** robot stalls or collides.

**P10.11 — (Stretch) Reverse motion.** Send goal behind robot. **Go:** MPPI uses vx_min, drives in reverse cleanly.

**P10.12 — (Stretch) 5-min long-haul.** Repeated short goals, accumulate 5 min runtime. **Go:** no MPPI hang, no controller crash, no cmd_vel gap >0.5 s.

**MPPI failure modes to watch:**
- Speed clamp at 0.5 m/s (issue #4970) — `time_steps × model_dt` too short relative to vx_max × lookahead.
- Close-maneuver wobble near goal (issue #5375) — bump `vx_std`.
- Slew-rate drift between MPPI's instantaneous ω model and actuator_node's slewed ω.

---

## PHASE 11 — Recovery + Drive Train (12 steps, ~25 min, FIELD)

**Goal:** Wait/BackUp/DriveOnHeading work in isolation AND in BT; e-stop and watchdog work; cmd_vel→motor math is correct.

**Pre-conditions:** P10 passed.

### Recovery behaviors
**P11.1 — behavior_server alive, 3 actions present.** `ros2 action list | grep -E "backup|drive_on_heading|wait"`. **Go:** all 3. Spin is intentionally absent.

**P11.2 — BackUp direct call.** `ros2 action send_goal /backup nav2_msgs/action/BackUp '{target: {x: 0.3}, speed: 0.1}'`. **Go:** robot reverses ~30 cm and stops. **No-go (immediate fail):** check `simulate_ahead_time: 2.0` — too aggressive collision-ahead may abort.

**P11.3 — DriveOnHeading direct call.** Same, 1 m straight. **Go:** robot moves 1 m forward.

**P11.4 — Wait 5 s.** **Go:** /cmd_vel = 0 for 5 s, no motion.

**P11.5 — BT recovery fires on blocked path.** Send goal with cone dropped in path. **Go:** `/bt_navigator/log` shows `BackUp` or `Wait` ticking in the recovery branch.

### Drive train
**P11.6 — cmd_vel → RPM math.** Publish `linear.x=0.3, angular.z=0`. Watch actuator_node log. **Go:** L_mps=0.3, R_mps=0.3, L_RPM ≈ 903 (= 0.3 / 0.01994 × 60), R_RPM ≈ 903. **No-go:** asymmetry or wrong sign → motor inversion config.

**P11.7 — Heading-hold at v=0.** Tilt IMU by hand briefly. **Go:** actuator_node log shows gyro-derived angular.z correction (won't move motors at v=0, but the algebra fires).

**P11.8 — Slew ramp.** Publish `linear.x=1.0` from 0. **Go:** /cmd_vel.linear.x reaches 1.0 in ≥1.0 s (max_linear_accel=1.0). **No-go:** instant 1.0 → slew limiter disabled.

**P11.9 — E-stop ≤100 ms.** While driving 0.3 m/s, publish `/avros/actuator_command estop=true`. **Go:** motors stop within ~100 ms; cmd_vel may still flow but is ignored.

**P11.10 — cmd_vel watchdog.** Stop publishing cmd_vel during drive. **Go:** motors brake within ~500 ms (watchdog timeout).

**P11.11 — (Stretch) Pivot turn at raised max_angular_rps.** Command angular.z = 1.0 rad/s, linear.x=0. **Go:** robot rotates in place at ~1 rad/s. Right track friction (8% higher per CLAUDE.md) may cause slight L/R speed mismatch.

**P11.12 — (Stretch) cmd_vel → actuator_state round-trip.** Measure latency. **Go:** <60 ms.

---

## PHASE 12 — mission_manager (11 steps, ~20 min, FIELD)

**Goal:** waypoint orchestration is robust.

**Config refs (from source):** `PROXIMITY_TICK_HZ: 5.0`, `DONE_HEARTBEAT_S: 10.0`, `acceptance_radius_m: 2.0` (default), `FROMLL_WAIT_TIMEOUT_S: 30.0`. Launch has 20 s TimerAction before mission_manager spawn (commit a44220e). Current waypoints.yaml has 4 entries near CPP datum.

**P12.1 — Node alive after 20 s.** Launch with `enable_mission_manager:=true`. **Go:** `ros2 topic list | grep mission_manager` shows topics within 25 s.

**P12.2 — Waypoints loaded.** Mission_manager log. **Go:** "Loaded 4 waypoints from /config/waypoints.yaml: (lat, lon), ...".

**P12.3 — /fromLL service available.** `ros2 service list | grep fromLL`. **Go:** present.

**P12.4 — Lat/lon → map conversion.** Log shows "Converted wp0 (lat, lon) → map frame (X, Y)". **Go:** within ±100 m of expected map position given GPS datum.

**P12.5 — First goal sent.** Log + `/navigate_to_pose` action feedback. **Go:** "Sending wp0 → NavigateToPose (X, Y) in map" then "wp0 accepted".

**P12.6 — Proximity advance.** Drive robot to within 2.0 m of wp0. **Go:** ≤200 ms after entering radius, log "wp0 reached, advancing → Sending wp1".

**P12.7 — Race protection.** Send goal, immediately publish cancel, then trigger proximity. **Go:** result for index=0 arrives as CANCELED but cursor stays at 1; no crash.

**P12.8 — DONE heartbeat.** Reach all 4 waypoints. **Go:** "all 4 waypoints reached — mission complete"; then every 10 s "mission complete — idling".

**P12.9 — Skip-on-failure.** Make wp1 unreachable (place fixed obstacle). **Go:** wp1 ABORTED → cursor advances to wp2 without crash; log "skip-on-failure, advancing".

**P12.10 — Launch arg gating.** Launch with `enable_mission_manager:=false`. **Go:** mission_manager not spawned; no goal traffic.

**P12.11 — Acceptance radius param override.** Launch with `acceptance_radius_m:=5.0`. **Go:** robot completes wp0 at 4 m radius (<5 m).

---

## PHASE 13 — IGVC Integration Scenarios (12 steps, ~90 min, FIELD)

**Goal:** the actual deliverable. Increasingly hard end-to-end runs.

**Scoring reality (IGVC 2026 AutoNav):** finishers ranked by shortest adjusted time; non-finishers by longest adjusted distance. Penalties from boundary crossings and obstacle hits. Speed band 1–5 mph (0.45–2.24 m/s); our 1.0 m/s is mid-band.

**P13.1 — 5 m forward, empty.** RViz Nav2 Goal (x=5, y=0). **Go:** within xy_goal_tolerance 2.0, no oscillation. **No-go:** >15 s or swerve >0.5 m.

**P13.2 — 10 m at sustained 1.0 m/s.** Goal (10, 0). **Go:** mean `/odometry/filtered.linear.x` ∈ [0.85, 1.05] for ≥6 s. **No-go:** clamps at 1.5 m/s for >1 s OR sustained <0.7 m/s.

**P13.3 — Single barrel midpath.** Barrel at (2.5, 0). Goal (5, 0). **Go:** clears ≥0.4 m, reaches goal. **No-go:** contact or NO_VALID_PATH.

**P13.4 — Lane corridor 2 m × 6 m, no obstacles.** Tape two lines 2 m apart, 6 m long. Goal (6, 0). **Go:** robot inside lanes, zero crossings.

**P13.5 — Narrow corridor 1.5 m × 6 m.** **Go:** completes without contact. **No-go:** controller_server reports FAILED_TO_MAKE_PROGRESS → reduce inflation_radius from 0.65 to 0.35 m for narrow lanes.

**P13.6 — Lane corridor 2 m × 8 m + barrel at (4, 0).** **Go:** weaves around barrel, stays in lanes. **No-go:** crosses lane to dodge (real-run penalty).

**P13.7 — 3-waypoint triangle via mission_manager.** Waypoints at (5,0), (2.5, 4.33), (0,0). **Go:** all 3 in <90 s, no idle >5 s between segments.

**P13.8 — 30 m sustained drive.** Goal (30, 0). **Go:** 30–40 s; Jetson CPU <85%, SoC temp <75°C end-of-run (`tegrastats` log). **No-go:** thermal throttle or vx droops <0.7 m/s.

**P13.9 — Single-line lane.** Only right tape, 6 m, left covered. Goal 1 m left of right line. **Go:** maintains ~1 m offset from visible line. **No-go:** drifts to unseen side.

**P13.10 — Stop-and-resume.** Drive to (15, 0). At t=6 s, `pkill -f controller_server`; wait respawn; resend goal. **Go:** Nav2 lifecycle re-activates, robot resumes, completes.

**P13.11 — Camera-blinded (GPS+LiDAR fallback).** Tape opaque cover on ZED X. Goal (5, 0). **Go:** Velodyne marks obstacles, GPS/EKF holds pose, reaches goal. **No-go:** semantic layer spams class errors or planner stalls.

**P13.12 — Mini-IGVC course.** 25 m corridor: lanes 3 m apart, barrel at 8 m, pothole at 15 m, cone at 20 m, GPS goal at 25 m. Send via `nav2_msgs/action/NavigateToPose` at waypoint UTM. **Go:** <40 s, zero boundary crossings, zero contacts (clean scoring run).

**What good competition performance looks like:**
- **Finish the course.** Finishers always rank above non-finishers. Tune for completion, not speed.
- **Zero boundary crossings, zero contacts.** Penalties are subtracted from adjusted time; a clean 1.0 m/s run beats a fast 1.5 m/s with two nicks.
- **Mid-band speed.** Stay 0.7–1.5 m/s; never below 0.45 m/s (lower bound of IGVC speed band).

---

## Recommended session breakdown

| Session | Phases | Where | Duration |
|---|---|---|---|
| **S1: Bench** | P0–P4 | Workstation + Jetson on cart, sensors active, robot stationary | 75 min |
| **S2: Field Bring-up** | P5–P6 | Empty grass / parking lot, robot stationary | 45 min |
| **S3: Field Localization** | P7 + brief P5/P6 re-verify | Same site, sky view | 30 min |
| **S4: Field Costmap + Planner** | P8–P9 | Open area, occasional obstacle | 40 min |
| **S5: Field Controller + Drive** | P10–P11 | Open area, e-stop in hand | 50 min |
| **S6: Field Mission + Single Goals** | P12 + P13.1–P13.3 | Open area, simple barrel | 60 min |
| **S7: Field Scenarios** | P13.4–P13.12 | Lay lanes + obstacles | 90 min |

**Total ~5 h of focused testing across 7 sessions.** Don't run all in one day; pace by daylight, battery, operator fatigue.

## Speed ramp

| Session | `vx_max` |
|---|---|
| S1–S4 | 0.7 (current default) |
| S5 (PRE-P10 step) | 1.0 |
| S6–S7 | 1.0 (firmware ceiling 1.06) |

Don't bump above 1.0 — firmware/SparkMAX cap is ~1.06 m/s peak / 0.58 m/s mean per F1c. Higher just samples rollouts the motors can't execute.

## Diagnosis triage (if a phase fails)

Capture:
- `ros2 bag record /odometry/global /odometry/filtered /cmd_vel /local_costmap/costmap /global_costmap/costmap /local_costmap/voxel_grid /plan /perception/front/overlay /tf /tf_static` (30 s window covering the failure)
- Foxglove or RViz screenshots at failure moment

Most likely culprits, in order of frequency (from research):
1. RMW mismatch (P1)
2. STVL plugin not loaded or wrong params (P5)
3. min_obstacle_height frame interpretation (P5.8 = canary)
4. HSV thresholds wrong for current lighting (P4.9 = canary)
5. GPS multipath / poor sky view (P7.7 = canary)
6. EKF tuning after VIO yaw-only change (P7.13 = canary)
7. kiwicampus class_types nesting (P6.4 = canary)
8. `enable_perception` false in launch args (check P0.5 startup args)
9. Build artifact stale on Jetson — rebuild `avros_bringup --symlink-install`

## What this plan does NOT cover

- Ramps / inclines (no test materials)
- Switchbacks (defer until lane following is rock-solid)
- IGVC's No-Man's-Land 4-waypoint hint section (no mock available)
- Self-Drive challenge (out of scope per team memory)
- Weather / dusk variation (note conditions; re-tune HSV per session)
- Long-duration thermal beyond P13.8 (6-min IGVC run on full charge)

## Final reminder

The robot can hurt you and damage itself. **Hand on the e-stop. Eyes on the robot. Foxglove on the side.** When in doubt: stop, return to bench, debug.
