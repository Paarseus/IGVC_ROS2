# AVROS Data Logger

## Purpose

`avros_dataLogger` records ROS RGB and thermal image topics together with vehicle controls and sensor streams. The output is organized for three uses:

- Object-detection datasets from the saved images
- Imitation-learning datasets from images paired with control commands
- Visual-inertial/GNSS processing from the original-rate IMU and GNSS logs

The included `avros_camera_bridge` publishes two UVC cameras as ROS `sensor_msgs/Image` topics using the device settings in `config/cameras.yaml`. The logger subscribes to those topics, pairs frames by their ROS timestamps, and saves RGB and thermal frames losslessly as PNG. IMU, GNSS, odometry, and actuator values are received from ROS 2 topics.

The recording path does not apply contrast enhancement, resize, or JPEG
compression. RGB and thermal frames are saved as lossless PNG files with
their ROS timestamps recorded in `camera_log.csv`.

## Output Structure

Each run writes to `data_logger` by default:

```text
data_logger/
├── images/
│   ├── rgb/
│   │   └── rgb_000012_123456789.png
│   └── thermal/
│       └── thermal_000012_123456789.png
├── camera_log.csv
├── imu_log.csv
└── gnss_log.csv
```

All three CSV files are created when the node starts and updated while the node is running.

## Data Logs

### `camera_log.csv`

One row is written for each saved RGB/thermal image pair. It contains:

- RGB/thermal timestamps, receive times, pair delta, and pair validity
- Relative paths to the RGB and thermal images
- Throttle, steering, brake, mode, and emergency-stop state
- `vehicle_stopped`, which is true when no fresh actuator command exists,
  emergency stop is active, or throttle and brake are both zero
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

Edit `config/cameras.yaml` to set the two camera devices. Stable
`/dev/v4l/by-id/...` paths are preferred over numeric indexes:

```yaml
rgb_device: /dev/v4l/by-id/your-rgb-camera
thermal_device: /dev/v4l/by-id/your-thermal-camera
rgb_width: 680
rgb_height: 480
thermal_width: 640
thermal_height: 512
capture_rate_hz: 30.0
```

Start the camera bridge and logger separately:

```bash
ros2 launch avros_data_logger cameras.launch.py
ros2 launch avros_data_logger data_logger.launch.py
```

The logger uses `/imu/data` at its existing 100 Hz rate. `pair_tolerance_ms`
controls the maximum RGB/thermal timestamp difference; the default is 10 ms.

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
