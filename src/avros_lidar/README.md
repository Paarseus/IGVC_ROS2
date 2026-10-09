# avros_lidar

Velodyne VLP-16 LiDAR preprocessing and 3D voxel obstacle mapping for AVROS (IGVC_ROS2).

A single launch file brings up the LiDAR driver, the TF frames it needs, the preprocessing node, the voxel mapper, and (optionally) RViz. The result is a live obstacle map around the vehicle, published on `/perception/lidar/*` topics.

## Pipeline

```
VLP-16 ──UDP──> velodyne_driver_node ──> velodyne_transform_node ──> /velodyne_points
                                                                         │
                                                                         ▼
                                                              lidar_preprocessor
                                      range/FOV filter → voxel downsample → transform to base_link
                                      → self-footprint removal → RANSAC ground removal → height band
                                                                         │
                          /perception/lidar/obstacles, /origin, /health  ▼
                                                                   voxel_mapper
                                      log-odds voxel grid, raycasting (CUDA on Jetson), decay, inflation
                                                                         │
                                                                         ▼
                                                    /perception/lidar/voxels, voxel_grid, costmap_2d, ...
```

| Node | Executable | Role |
|------|-----------|------|
| `lidar_preprocessor` | `avros_lidar lidar_preprocessor` | Cleans `/velodyne_points` and outputs obstacle-only points in `base_link` |
| `voxel_mapper` | `avros_lidar voxel_mapper_node` | Fuses obstacle points into a persistent, base_link-centered 3D voxel grid |

## Prerequisites

- Jetson (or any ROS 2 Humble machine) with this workspace built: `~/IGVC_ROS2`
- VLP-16 powered, on the network, reachable at `192.168.13.11` (default) via interface `eno1` (default)
- `velodyne_driver`, `velodyne_pointcloud` and `avros_bringup` available in the workspace
- Optional: `numba` + CUDA for GPU raycasting. Without it the node warns once and falls back to CPU automatically.

## Build

```bash
cd ~/IGVC_ROS2
colcon build --symlink-install --packages-select avros_lidar
source install/setup.bash
```

## Launch

```bash
cd ~/IGVC_ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch avros_lidar lidar.launch.py
```

This starts, in one go:

1. Static TFs `base_link → velodyne` and `base_link → imu_link` (values copied from the `avros_bringup` URDF)
2. Velodyne driver + point cloud transform node
3. `lidar_preprocessor` and `voxel_mapper`
4. RViz with the `lidar.rviz` config

The launch file sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and the `avros_bringup` CycloneDDS config for the nodes it starts. **CLI tools in your own shell do not inherit that.** In any terminal where you run `ros2 topic ...`, first run:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/IGVC_ROS2/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml
```

Otherwise you will not see the topics.

On startup the launch file also re-points the VLP-16's UDP destination at this machine's current IP on `lidar_iface`, so it survives DHCP changes. You should see `[lidar.launch] lidar 192.168.13.11 now streams to <ip>:2368`.

### Common launch recipes

```bash
# Headless (no RViz), e.g. over plain SSH or during navigation
ros2 launch avros_lidar lidar.launch.py rviz:=false

# RViz inside a NoMachine session, from an SSH shell
ros2 launch avros_lidar lidar.launch.py rviz_display:=:1001

# Sensors/TF are already running via avros_bringup sensors.launch.py
ros2 launch avros_lidar lidar.launch.py enable_sensors:=false

# Coarser grid / no raycasting if the CPU is struggling
ros2 launch avros_lidar lidar.launch.py voxel_resolution:=0.2 enable_raycasting:=false

# Replaying a bag
ros2 launch avros_lidar lidar.launch.py enable_sensors:=false use_sim_time:=true
```

> Never run two Velodyne drivers. If `avros_bringup` `sensors.launch.py` is up, use `enable_sensors:=false`.
>
> Avoid RViz on the Jetson while the robot is navigating. It eats a lot of CPU over NoMachine. Use `rviz:=false` and view from a laptop (Foxglove) instead.

### Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `enable_sensors` | `true` | Start the Velodyne driver and static TFs. Set `false` if already running elsewhere |
| `tf_source` | `static` | `static` = fixed transforms; `urdf` = `robot_state_publisher` with the avros URDF |
| `set_lidar_host` | `true` | Point the VLP-16 stream at this machine's current IP before starting the driver |
| `lidar_ip` | `192.168.13.11` | VLP-16 address |
| `lidar_iface` | `eno1` | NIC the VLP-16 is plugged into |
| `deskew_frame` | `velodyne` | `velodyne` = no deskew (no odom needed). Use `odom` only when the localization EKF is running |
| `enable_xsens` | `false` | Also start the Xsens IMU driver |
| `rviz` | `true` | Launch RViz |
| `rviz_display` | *(empty)* | X display for RViz (e.g. `:1001`); empty uses `$DISPLAY` |
| `software_gl` | `true` | Software OpenGL for RViz (needed on NoMachine) |
| `voxel_resolution` | `0.1` | Voxel size in metres |
| `enable_raycasting` | `true` | Clear free space between the sensor and each hit |
| `use_gpu_raycast` | `true` | CUDA raycasting; falls back to CPU if unavailable |
| `lidar_input_topic` | `/velodyne_points` | Input cloud. In Webots sim use `/velodyne_points/point_cloud` |
| `use_sim_time` | `false` | Use `/clock` (bag replay) |

Tunable processing parameters (range, height band, grid extents, persistence, inflation, etc.) live in `config/lidar_params.yaml`.

## Verifying it works

In a new terminal (with the CycloneDDS exports above):

```bash
ros2 topic hz /velodyne_points                 # ~10 Hz from the driver
ros2 topic echo /perception/lidar/health       # data: true
ros2 topic echo /perception/lidar/sensor_status  # data: nominal
ros2 topic hz /perception/lidar/voxels
```

If `health` is `false` or `sensor_status` is `lidar_unavailable`, no cloud has arrived for 0.5 s. See Troubleshooting.

## What you get from the full launch

All outputs are in the `base_link` frame (origin at ground level under the vehicle) unless stated otherwise.

### Main outputs

| Topic | Type | What it is | Use it for |
|-------|------|-----------|-----------|
| `/perception/lidar/obstacles` | `sensor_msgs/PointCloud2` | Filtered obstacle points for the current scan (ground, vehicle body and out-of-range points removed) | Debugging the preprocessor; input to the mapper |
| `/perception/lidar/voxels` | `sensor_msgs/PointCloud2` | Centres of all currently **occupied** voxels | Viewing the 3D obstacle map; feeding other consumers |
| `/perception/lidar/voxels_inflated` | `sensor_msgs/PointCloud2` (x, y, z, intensity) | Occupied voxels plus an inflated safety margin; `intensity` = cost | Visualising keep-out zones. Only computed while someone is subscribed |
| `/perception/lidar/obstacle_costmap_2d` | `nav_msgs/OccupancyGrid` | Top-down 2D projection of the voxel grid: `100` occupied, `0` free, `-1` unknown | Planning / costmap input, quick top-down view |
| `/perception/lidar/voxel_grid` | `std_msgs/Int8MultiArray` | The full 3D grid, flattened row-major over `(x, y, z)`: `100` occupied, `0` free, `-1` unknown | Programmatic access to the raw grid |
| `/perception/lidar/voxel_grid_metadata` | `std_msgs/String` (JSON) | How to interpret `voxel_grid` (see below) | Always read alongside `voxel_grid` |
| `/perception/lidar/voxel_markers` | `visualization_msgs/MarkerArray` | Red cube list of occupied voxels | Foxglove / RViz display |

### Health and status

| Topic | Type | Meaning |
|-------|------|---------|
| `/perception/lidar/health` | `std_msgs/Bool` | `true` while clouds are arriving, `false` after 0.5 s without data |
| `/perception/lidar/sensor_status` | `std_msgs/String` | `nominal` when healthy, `lidar_unavailable` otherwise |
| `/perception/lidar/origin` | `geometry_msgs/PointStamped` | LiDAR position in `base_link` (used as the raycast origin) |

### Reading the voxel grid

`voxel_grid_metadata` is JSON:

```json
{
  "frame_id": "base_link",
  "size_x": 20.0, "size_y": 20.0, "size_z": 1.8,
  "resolution": 0.1,
  "grid_nx": 200, "grid_ny": 200, "grid_nz": 18,
  "origin": [-10.0, -10.0, 0.2],
  "inflation_radius": 1.2,
  "timestamp": 0.0,
  "sensor_mode": "nominal"
}
```

(Numbers shown are the defaults from `lidar_params.yaml`.) To get the world position of cell `(ix, iy, iz)`:

```
x = origin[0] + (ix + 0.5) * resolution
y = origin[1] + (iy + 0.5) * resolution
z = origin[2] + (iz + 0.5) * resolution
```

Reshape with `np.array(msg.data).reshape(grid_nx, grid_ny, grid_nz)`.

Before trusting the grid, check `sensor_mode == "nominal"`.

### Default coverage

- Grid: 20 m × 20 m centred on the vehicle (±10 m in x and y), at 0.1 m resolution
- Height: obstacles between **0.2 m and 2.0 m** above ground
- Range: 0.7 m to 50 m from the sensor, though only the ±10 m grid is mapped
- Persistence: voxels not re-observed fade after about 0.6 s, so the map follows moving obstacles
- Inflation: 1.2 m safety margin (on `voxels_inflated`)

### In RViz

The bundled `lidar.rviz` (fixed frame `base_link`) shows `LidarObstacles`, `Voxels`, `VoxelsInflated`, `VoxelMarkers` and `ObstacleCostmap2D`. Walk in front of the sensor: you should see voxels appear on you and fade shortly after you leave.

## Troubleshooting

| Symptom | Likely cause / fix |
|---------|-------------------|
| Driver logs `poll() timeout`, no `/velodyne_points` | The sensor is streaming to a stale IP. Check the `[lidar.launch] ... now streams to` line; confirm `lidar_iface` has an IPv4 address and `lidar_ip` is reachable (`ping 192.168.13.11`) |
| `ros2 topic list` shows nothing from the launch | Your shell is on FastDDS. Export the CycloneDDS variables above |
| `health: false` | No cloud for 0.5 s. Check `ros2 topic hz /velodyne_points` and the cable/power |
| TF errors about `velodyne` or `base_link` | Static TFs are only started when `enable_sensors:=true`. If you disabled it, make sure something else publishes them |
| Warning: `use_gpu_raycast requested but unavailable` | numba/CUDA not usable; running on CPU. Harmless, but use `voxel_resolution:=0.2` if it is too slow |
| RViz is black or crashes on NoMachine | Keep `software_gl:=true` and set `rviz_display` to the NoMachine display (`ls /tmp/.X11-unix/`) |
| Transform errors when using the sim | Pass `lidar_input_topic:=/velodyne_points/point_cloud` and `use_sim_time:=true` |
| System load high, other nodes starved | Run `rviz:=false`; use a coarser `voxel_resolution` |

## Package layout

```
avros_lidar/
├── avros_lidar/
│   ├── lidar_preprocessor.py   # filtering, ground removal, TF to base_link
│   ├── voxel_mapper_node.py    # ROS node: publishes grid, costmap, markers
│   └── voxel_grid.py           # log-odds voxel grid, raycasting, decay, inflation
├── config/lidar_params.yaml    # tuning parameters for both nodes
├── launch/lidar.launch.py      # full bringup
└── rviz/lidar.rviz             # RViz layout
```
