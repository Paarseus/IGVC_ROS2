# LiDAR stack: preprocessor + STVL 3D voxels (2026-10-09)

Velodyne VLP-16 obstacle handling for IGVC_ROS2. The standalone `avros_lidar` package no longer exists; its pieces were merged into the packages the rest of the stack already uses. The last version of the old package is in git history at commit `4ec5c89` (branch `lidar_testing`).

> **Status:** works on the bench at 10 Hz end to end. Merged into `avros_perception` / `avros_bringup` and committed on `lidar_testing`. NTRIP/RTK, the STVL tuning below and measured rates added 2026-10-13. **Not yet tested driving with the real EKF or the full navigation launch.** See [Open issues](#open-issues-and-next-steps).

## Where everything lives

| Piece | Location |
|-------|----------|
| Preprocessor node (`lidar_preprocessor`) | `avros_perception/avros_perception/lidar_preprocessor.py` |
| Preprocessor parameters | `avros_perception/config/lidar.yaml` |
| Preprocessor-only launch | `avros_perception/launch/lidar_preprocessor.launch.py` |
| **Individual lidar launch (drivers + preprocessor + STVL costmap + RViz)** | `avros_bringup/launch/lidar.launch.py` |
| Bench-only param overlay | `avros_bringup/config/lidar_overrides.yaml` |
| RViz layout | `avros_bringup/rviz/lidar.rviz` |
| STVL settings (the real ones) | `avros_bringup/config/nav2_params_igvc_autonav.yaml` (`local_costmap` and `global_costmap`, `stvl_layer`, source `lidar_obstacles`) |
| Sensor unicast re-pointing | `avros_bringup/launch/sensors.launch.py` |
| Full-stack start of the preprocessor | `avros_bringup/launch/navigation.launch.py` (`enable_lidar_preprocessor`, default true) |
| Webots version | `avros_sim/launch/sim_lidar.launch.py` |

## How it fits together

```
VLP-16 ──UDP (unicast)──> velodyne_driver_node ──> velodyne_transform_node ──> /velodyne_points (frame: velodyne)
                                                                                    │
                                                                         lidar_preprocessor
              range (0.7 to 15 m) / vertical FOV (±15°) → voxel downsample 0.1 m → transform to base_link
              → vehicle-body box removal → RANSAC ground removal → height band 0.2 to 2.0 m
                                                                                    │  (frame: base_link, original scan stamp)
                                                  /perception/lidar/obstacles, /perception/lidar/health
                                                                                    ▼
                                                    Nav2 STVL layer (local: odom, global: map)
                                                                                    ▼
                                    /local_costmap/voxel_grid (3D voxels)     /local_costmap/costmap (2D)
```

The preprocessor stays in `base_link` and publishes with the original scan stamp. STVL looks up the transform from the cloud's frame to its own global frame at that stamp, so it does the `odom` / `map` transform. The preprocessor's ground fit, body exclusion and height band need an upright, robot-fixed frame.

The preprocessor is the **authority for what is marked**; STVL is set to pass its output through.

| What | Set in | Value | STVL setting (pass-through) |
|------|--------|-------|-----------------------------|
| Max distance | `avros_perception/config/lidar.yaml` `range_max` | **15 m** | `obstacle_range: 15.0` |
| Min distance | `lidar.yaml` `range_min` | 0.7 m | none (driver transform also uses 0.7) |
| Height band | `lidar.yaml` `height_band_min/max` | 0.2 to 2.0 m (above ground, `base_link`) | **local:** `min_obstacle_height: 0.2`, `max_obstacle_height: 0.8` (2026-10-13: STVL now narrows the preprocessor band to 0.2 to 0.8 m, so nothing above 0.8 m is marked locally). **global:** `0.1` / `2.1` (wider pass-through) |
| Vertical FOV | `lidar.yaml` ±15° | point filter | STVL's `vertical_fov_angle` etc. are unchanged: they only describe the sensor's view cone for clearing, not a filter |
| Vehicle body | `lidar.yaml` footprint box | measured chassis + 0.05 m | `update_footprint_enabled` (both apply) |
| Downsample | `lidar.yaml` 0.1 m | | `voxel_size: 0.1` |
| Ghost suppression | | | `mark_threshold`: **local 2** (2026-10-13), global 1 |

STVL can only narrow what the preprocessor lets through, so wherever STVL's limit was tighter (8 m / 10 m range, 0.4 to 1.0 / 0.8 m heights) STVL used to win. Keep the two in step if you change either.

## Running it

```bash
cd ~/IGVC_ROS2 && source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select avros_perception avros_bringup
source install/setup.bash
```

**Individual lidar launch with RViz (bench):**

```bash
# drivers already running (percept stack / sensors.launch.py): never run two drivers
ros2 launch avros_bringup lidar.launch.py enable_sensors:=false rviz_display:=:1001
# nothing running: also starts the Velodyne driver and URDF TFs
ros2 launch avros_bringup lidar.launch.py rviz_display:=:1001
```

It starts the preprocessor, static `odom→base_link` and `map→odom` stand-ins, a minimal Nav2 `controller_server` (just its local costmap, with the **same** `nav2_params_igvc_autonav.yaml` as the real run plus `lidar_overrides.yaml`, which only drops the ZED semantic layer), and RViz. The robot is treated as stationary, so the voxels do not move with the robot; this is a "what does the lidar see" check, not a driving test. Do not run it together with `localization.launch.py` / `navigation.launch.py` (the static `odom→base_link` would fight the EKF).

Arguments: `enable_ntrip` (false; true starts the NTRIP client so the Xsens GNSS gets RTK corrections, e.g. `enable_ntrip:=true`), `enable_sensors` (true), `lidar_input_topic` (`/velodyne_points`; Webots uses `/velodyne_points/point_cloud`), `rviz` (true), `rviz_display` (empty = `$DISPLAY`), `software_gl` (true, needed on NoMachine), `use_sim_time` (false).

**Full navigation:** `ros2 launch avros_bringup navigation.launch.py` now also starts the preprocessor (`enable_lidar_preprocessor:=true`) because both STVL layers read `/perception/lidar/obstacles`. Note `navigation.launch.py` defaults to the competition params file `nav2_params_igvc_autonav.yaml`, and its mission manager auto-starts a waypoint run about 20 s after launch unless `enable_mission_manager:=false`.

**CLI tools** need the CycloneDDS setup the launches use:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/IGVC_ROS2/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml
```

**Check it:**

```bash
ros2 topic hz /velodyne_points                   # 10 Hz, one full scan per message
ros2 topic hz /perception/lidar/obstacles        # 10 Hz
ros2 topic hz /local_costmap/voxel_grid          # 10 Hz (costmap timer)
ros2 topic echo --once /perception/lidar/health  # data: true
ros2 topic echo --once /velodyne_points --field header.frame_id   # must print: velodyne
```

In RViz (`lidar.rviz`, fixed frame `odom`): the obstacle cloud, `/local_costmap/voxel_grid` as 0.1 m boxes, `/local_costmap/costmap`, TF. Under software OpenGL RViz may log a GLSL error for the costmap panel; it is cosmetic. The RViz config's `Frame Rate: 15` is only the redraw cap, not the data rate.

STVL 2.3.4 publishes a single `voxel_grid` topic under the costmap namespace (`/local_costmap/voxel_grid`). There is no `voxel_marked_cloud`, despite an old comment in the params files.

## STVL decay and marking parameters (local costmap)

Set in `nav2_params_igvc_autonav.yaml`, `local_costmap.stvl_layer`. Values as of 2026-10-13:

| Parameter | Value | What it does |
|-----------|-------|--------------|
| `voxel_decay` | 3.0 s | Timeout: a voxel not re-confirmed by a lidar return for this long is removed. Lower clears faster. |
| `decay_acceleration` | 1.0 (was 0.5) | Extra decay speed for voxels inside the sensor's field of view that get no return (1/s^2). Higher clears faster but can make real obstacles flicker with the sparse VLP-16 (earlier tuning saw flicker at 2.0 and above). |
| `mark_threshold` | 2 (was 1) | Returns a voxel needs before it is marked. Higher filters stray hits but marks thin or distant objects more slowly. |
| `min_obstacle_height` / `max_obstacle_height` | 0.2 / 0.8 m | Height band, in the global (`odom`) frame. |
| `clearing` | true | Rays also clear free space, but this is weak with 16 beams; clearing is mostly time-based. |

The global costmap keeps its own values (`mark_threshold` 1, `decay_acceleration` 0.5, heights 0.1 / 2.1). These local values were tuned on the bench; they are not field-validated, and they are the same file the competition run uses.

## GNSS / RTK (Xsens MTi-680G)

`sensors.launch.py` starts `xsens_mti_node` (IMU + GNSS) and, with `enable_ntrip:=true`, the NTRIP client (caster `ntrip.earthscope.org:2101`, settings in `ntrip_params.yaml`). Measured 2026-10-13 on the bench: `/imu/data` 100 Hz, `/gnss` 4 Hz, `/rtcm` about 6.5 Hz (bursty, gaps up to 0.9 s). Fix status was `STATUS_FIX` (standard, not RTK) shortly after start; RTK convergence was not checked.

## Sensor setup: unicast (important)

The VLP-16 stores where it sends its UDP packets. It **must send to this machine, not broadcast**:

```bash
curl -s http://192.168.13.11/cgi/settings.json | python3 -c "import sys,json; print(json.load(sys.stdin)['host'])"   # read
curl -s -o /dev/null -w "%{http_code}\n" -d "addr=192.168.13.10&dport=2368&tport=8308" http://192.168.13.11/cgi/setting/host   # set; 204 = OK
```

`sensors.launch.py` now does this on every start (`set_lidar_host:=true`, `lidar_ip` 192.168.13.11, `lidar_iface` eno1), so a power-cycled sensor does not fall back to broadcast. (A stack that was already running when this was added does not have it until it is restarted.)

**Broadcast symptom:** the sensor's destination reads `255.255.255.255`, every packet arrives twice (about 1,460/s instead of 754) and some out of order, and the driver's `cut_angle` logic then emits 100+ thin partial scans per second instead of one 360° scan at 10 Hz. Measured 2026-10-09:

| | Broadcast | Unicast |
|---|---|---|
| Packets/s (port 2368) | about 1,460 | **754** |
| Identical duplicate packets | 1,447 of 3,000 | **0** |
| Backward azimuth steps | 160 | **0** |
| `/velodyne_points` | about 107 to 144 msgs/s, 2 to 7 packets each, median 19° of azimuth | **10.0 Hz**, 75 to 76 packets, 356° |

If fragments return, check the sensor's destination first. As a temporary workaround set `merge_window_sec` in `lidar.yaml` to about 0.08 (the preprocessor then merges partial scans). With full scans it must stay at `0.0`: a window of 0.1 s or more merges two scans and halves the output rate to 5 Hz.

## Parameters worth knowing (`avros_perception/config/lidar.yaml`)

`input_topic /velodyne_points`, `sensor_frame velodyne`, `target_frame base_link`, `range_min 0.7`, `range_max 15.0`, `vertical_fov ±15`, `downsample_voxel_size 0.1`, RANSAC (50 iterations, 0.5 m candidate band, 0.1 m threshold, smoothed over frames), `height_band 0.2 to 2.0`, `sensor_timeout_sec 0.5` (health goes false after that long with no cloud), `merge_window_sec 0.0`, plus the footprint box (defaults in the node).

## Work log

1. Built the original package: preprocessor, a custom log-odds voxel mapper (CUDA raycasting), launch, RViz. Committed as `4ec5c89`.
2. Compared with the existing stack: Nav2 already ran STVL on `/velodyne_points`. The custom mapper duplicated it, kept its grid in `base_link` (moving frame, no odometry compensation, so stored voxels would smear while driving), and nothing read its topics. **Removed the mapper**, kept the preprocessor.
3. Profiled the preprocessor on a live 57,600-point scan (loaded Jetson): RANSAC 1476 ms (Python loop of small matrix products), cloud→numpy 161 ms (per-point Python loop), downsample 56 ms (`np.unique axis=0`). Vectorized all three; whole chain 1734 ms → 166 ms, and → 17.6 ms with `OPENBLAS_NUM_THREADS=1` (numpy's thread pool only adds contention on a busy CPU; the launch now sets it). Output matched the old code on the same scan (identical downsampled cloud, same obstacle count; plane differs in the 3rd decimal because the RANSAC sampling order changed).
4. Found the driver emitting thin fragments, measured raw azimuth coverage (100% of the circle within 0.25 s to 0.5 s), added a merge window as a workaround.
5. Found the root cause: the sensor was on broadcast (duplicate, out-of-order packets). Fixed by switching it to unicast. Turned the merge window off.
6. Merged the package into `avros_perception` / `avros_bringup` and removed `avros_lidar` (this document's layout).
7. **A memory blowup was seen once**: during the first launch after the merge, `lidar_preprocessor` grew to about 56 GB resident within roughly two minutes and was OOM-killed by the kernel. Cause not found. It does not reproduce: the same code under a 24 GB address-space cap ran 25 s at 63 MB, and the relaunched node held steady at 66 to 74 MB over several minutes. The node now caps its own address space (4 GiB, `AVROS_LIDAR_MAX_VMEM_GB`, 0 disables) so a runaway raises `MemoryError` with a traceback in that process instead of OOM-killing others. Normal use is under 1 GB of address space.

Rates now (bench, live; re-measured 2026-10-13): `/velodyne_points` 10.0 Hz, `/perception/lidar/obstacles` 10.0 Hz, `/local_costmap/voxel_grid` 10.0 Hz, costmap 2D 6 Hz (2026-10-09). Obstacle latency adds about 18 ms to the scan's own age (median message age 75 ms at the driver output, 93 ms after the preprocessor, measured before the merge). The preprocessor uses about 23% of one core.

## Open issues and next steps

1. **Test with the real EKF / driving.** Everything above is stationary on the bench with static `odom`/`map` TFs. Not yet run through `navigation.launch.py`.
2. **Compare against the old path on a bag** before relying on it: lethal-cell counts, flicker, low-obstacle detection. The old STVL path (`/velodyne_points`, 0.4 to 1.0 m) is in `nav2_params_igvc_autonav.yaml.bak_before_lidar_merge_2026-10-09`.
3. **Grass and curbs:** the local STVL band is now 0.2 to 0.8 m, so the 0.2 m floor and the preprocessor ground removal decide; people and overhead objects above 0.8 m are not marked locally. Site measurements showed about 5% of returns at 0.2 to 0.5 m (grass, curbs); those may now mark.
4. **RANSAC ground removal risk:** a bad plane fit (grass, ramps, facing a wall) could delete real low obstacles; STVL's old fixed floor could not.
5. **STVL height limits are in the global frame's z.** Confirm the EKFs hold z at 0 (2D mode) so the band lines up with the ground.
6. **STVL `vertical_fov_angle` (0.523 rad):** whether STVL treats it as full or half angle was not confirmed from source.
7. **The memory blowup (above)** is unexplained; watch for it. If it recurs, the traceback from the capped node should point at the line.
8. **Tests:** no unit tests for the preprocessor (RANSAC, downsample, merge). Lint tests (`flake8`, `pep257`) in `avros_perception` were not run against the new module.
9. **`mark_threshold`:** local raised to 2 on 2026-10-13 (global still 1). Not measured yet: check that barrels and cones at the edge of range still mark.
10. **Webots:** `sim_lidar.launch.py` was updated but not run since the merge. `sim_navigation.launch.py` still remaps to `/velodyne_points` and was not changed; it needs checking if it uses the competition params file.
11. **Git:** the merge is committed on `lidar_testing`. Old version: `4ec5c89` (and a stash of the pre-merge working tree). The nav2 params commit contains only the lidar/STVL edits; unrelated local edits (MPPI speed limits, `odom_topic`) were left uncommitted.
12. **Decay tuning:** run the flicker test (stationary barrel, 30 s) and the ghost test (remove it, time the clearing) before relying on the current decay values.

## Troubleshooting

| Symptom | Likely cause / fix |
|---------|-------------------|
| `/velodyne_points` is 100+ thin messages per second | Sensor on broadcast. Re-point (see above). Workaround: `merge_window_sec` about 0.08 |
| Obstacle output about 5 Hz | `merge_window_sec` is 0.1 s or more while full 10 Hz scans arrive. Set 0.0 |
| `ros2 topic list` shows nothing | Shell on FastDDS. Export the CycloneDDS variables |
| Two publishers on `/perception/lidar/obstacles` (20 Hz) | Two preprocessors running (e.g. one from a separate launch plus `navigation.launch.py`). Check `pgrep -x lidar_preproces` |
| `health: false` | No cloud for 0.5 s: check `ros2 topic hz /velodyne_points`, the cable, power |
| `TF lookup base_link<-velodyne failed` at start | Normal for the first second. If it persists, nothing is publishing `base_link→velodyne` (`sensors.launch.py` does through the URDF) |
| Bench errors about TF `odom`/`map` | Only the bench launch publishes them; no EKF at the same time |
| Preprocessor killed / `MemoryError` | See work log item 7. Capture the traceback; the node's own cap is `AVROS_LIDAR_MAX_VMEM_GB` |
| `/velodyne_points` frame is `odom` | Deskew swap in `velodyne.yaml` did not apply; the preprocessor's lookup would break |
| RViz slow or black on NoMachine | Keep `software_gl:=true`; close unused RViz windows (about 1 to 2 cores each) |

Related: `avros_bringup/config/velodyne.yaml` (driver and deskew), `docs/lidar_obstacle_avoidance_test_2026_05_21.md`, `docs/lidar_rotation_phantom_*_2026_05_29.md`.
