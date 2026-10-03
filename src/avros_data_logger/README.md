# AVROS Data Logger

## Purpose

`avros_dataLogger` records external RGB and thermal camera images together with vehicle controls and sensor streams. The output is organized for three uses:

- Object-detection datasets from the saved images
- Imitation-learning datasets from images paired with control commands
- Visual-inertial/GNSS processing from the original-rate IMU and GNSS logs

The cameras are opened directly with OpenCV because they do not have official ROS drivers. A camera is enabled only when its ROS camera-index parameter is set to a non-negative value. If one camera is unavailable, the logger skips that image and continues logging any available camera. IMU, GNSS, odometry, and actuator values are received from ROS 2 topics.

## Output Structure

Each run writes to `data_logger` by default:

```text
data_logger/
├── images/
│   ├── rgb/
│   │   └── rgb_000012_20261001T143015.123456Z.jpg
│   └── thermal/
│       └── thermal_000012_20261001T143015.123456Z.jpg
├── camera_log.csv
├── imu_log.csv
└── gnss_log.csv
```

All three CSV files are created when the node starts and are appended and flushed while the node is running. They can be inspected during the run. When enabled, the logger also serves a browser preview with RGB above thermal and a live image count.

## Data Logs

### `camera_log.csv`

One row is written for each saved RGB/thermal image pair. It contains:

- Image counter, episode ID, capture timestamp, and elapsed `time_seconds`
- Relative paths to the RGB and thermal images
- Throttle, steering, brake, mode, and emergency-stop state
- `speed_mps` and `yaw_rate_rps` from odometry

The throttle, steering, and brake values are the commanded controls used as the primary imitation-learning labels.

### `imu_log.csv`

Every received IMU message is written at the original topic rate. It contains:

- The ROS message timestamp and frame ID
- Orientation quaternion
- Orientation covariance
- Angular velocity and covariance
- Linear acceleration and covariance

This preserves the high-rate IMU stream needed for visual-inertial processing.

### `gnss_log.csv`

Every received GNSS message is written at the original topic rate. It contains:

- The ROS message timestamp and frame ID
- Fix status and service
- Latitude, longitude, and altitude
- All nine position-covariance values
- Covariance type

## Configuration

The camera indices and image capture rate are defined near the top of `data_logger_node.py`:

```python
# Default RGB index; -1 disables RGB until a ROS parameter is supplied.
RGB_CAMERA_INDEX = -1
# Default thermal index; -1 disables thermal until a ROS parameter is supplied.
THERMAL_CAMERA_INDEX = -1
# Number of image pairs to capture per second.
CAPTURE_RATE_HZ = 10.0
# Number of digits used for zero-padding image counters.
IMAGE_COUNT_WIDTH = 6
```

Use `v4l2-ctl --list-devices` to identify the camera indices. A value of `-1` disables that camera. Set each camera explicitly at runtime:

```bash
ros2 run avros_data_logger avros_dataLogger --ros-args \
  -p rgb_camera_index:=0 \
  -p thermal_camera_index:=1 \
  -p capture_rate_hz:=10.0
```

The default ROS topics are:

| Data | Topic | Message type |
| --- | --- | --- |
| IMU | `/imu/data` | `sensor_msgs/Imu` |
| GNSS | `/gnss` | `sensor_msgs/NavSatFix` |
| Odometry | `/wheel_odom` | `nav_msgs/Odometry` |
| Command | `/avros/actuator_command` | `avros_msgs/ActuatorCommand` |

Override topic names when necessary:

```bash
ros2 run avros_data_logger avros_dataLogger --ros-args \
  -p imu_topic:=/imu/data \
  -p gps_topic:=/gnss \
  -p odometry_topic:=/wheel_odom \
  -p command_topic:=/avros/actuator_command
```

## Browser Preview

The package includes `data_logger.launch.py`. It opens the cameras and starts
the browser preview on port `8080`, but it does not write data immediately.
Use **Start Recording** after checking the camera panels. Use **Stop
Recording** to stop writing while leaving the preview active.

```bash
ros2 launch avros_data_logger data_logger.launch.py \
  rgb_camera_index:=6 \
  thermal_camera_index:=7 \
  episode_id:=2 \
  output_directory:=data_logger/episode_2
```

On the Jetson, create an SSH tunnel from your computer:

```bash
ssh -L 8080:127.0.0.1:8080 dinosaur@JETSON_IP
```

Then open `http://localhost:8080` in a browser. The page shows RGB in the
upper panel, thermal in the lower panel, black `Unavailable` panels for missing
cameras, recording controls, and the number of captured images.

The preview can be disabled with `web_preview:=false`. The bind address can be
changed explicitly with `web_bind_address:=0.0.0.0`, but this exposes the live
camera page to the network and should only be used intentionally.

## Building and Running

From the workspace root:

```bash
colcon build --packages-select avros_data_logger
source install/setup.bash
ros2 run avros_data_logger avros_dataLogger
```

Choose another output directory with:

```bash
ros2 run avros_data_logger avros_dataLogger --ros-args \
  -p output_directory:=/path/to/run_data
```

## Episode IDs

Each camera-log row contains an `episode_id`. The value is supplied when
starting the logger, and the default ID is `1`:

```bash
ros2 run avros_data_logger avros_dataLogger --ros-args \
  -p episode_id:=2 \
  -p output_directory:=/path/to/episode_2
```

This allows later dataset tools to group or split samples by episode. The
logger does not currently detect driving start/stop events automatically.

## Timestamp and Synchronization Behavior

The RGB and thermal cameras are read in the same timer callback. Their files receive the same image counter and capture timestamp, but ordinary USB cameras are not guaranteed to expose their images at exactly the same instant. Exact exposure synchronization requires hardware triggering or a shared timing source.

The camera log uses the nearest recent command and odometry message. The IMU and GNSS logs do not downsample or wait for camera frames; every received message is written with its own ROS timestamp. This is important for later visual-inertial and GNSS processing.

## Testing

After building the workspace, run the package tests with:

```bash
colcon test --packages-select avros_data_logger
colcon test-result --verbose
```

The tests cover camera-device opening behavior, zero-padded image names, CSV headers, and writing individual IMU and GNSS messages. They do not require physical cameras because camera access is mocked.

## Ending a Run

Press `Ctrl-C` to stop the node. The node flushes and closes all three CSV files and releases both cameras.

## Troubleshooting

### A camera cannot be opened

Check the connected video devices:

```bash
v4l2-ctl --list-devices
```

Then set the corresponding ROS camera parameter. The thermal camera must appear as a standard Linux video device for OpenCV to read it directly. A camera index of `-1` disables that camera.

### Sensor data is missing

Check that the topics are publishing:

```bash
ros2 topic list
ros2 topic echo /imu/data
ros2 topic echo /gnss
ros2 topic echo /wheel_odom
```

If the topics use different names, pass the appropriate topic parameters when starting the node.
