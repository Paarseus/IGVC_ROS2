# AVROS Data Collection

## Purpose

`avros_dataCollection` records camera images and vehicle sensor values during a vehicle run. It is a standalone ROS 2 node located in the `avros_data_collection` package.

The node uses OpenCV for two external USB cameras:

- One RGB camera
- One thermal camera

The node uses ROS 2 subscriptions for the IMU, GPS, and vehicle velocity values.

## Requirements

The cameras must be available to Linux as video devices, such as `/dev/video0` and `/dev/video1`. The cameras do not need official ROS drivers. They must be readable by OpenCV through `cv2.VideoCapture`.

The required ROS topics are:

| Data | Default topic | Message type |
| --- | --- | --- |
| IMU | `/imu/data` | `sensor_msgs/Imu` |
| GPS | `/gnss` | `sensor_msgs/NavSatFix` |
| Vehicle velocity | `/wheel_odom` | `nav_msgs/Odometry` |

## Camera Configuration

The camera indices and capture rate are defined near the top of `data_collection_node.py`:

```python
# Camera device index for the RGB camera.
RGB_CAMERA_INDEX = 0
# Camera device index for the thermal camera.
THERMAL_CAMERA_INDEX = 1
# Number of image pairs to capture per second.
CAPTURE_RATE_HZ = 10.0
# Number of digits used for zero-padding image counters.
IMAGE_COUNT_WIDTH = 6
```

Use `v4l2-ctl --list-devices` to identify which index belongs to each camera. For example, if the thermal camera is `/dev/video2`, its index is normally `2`.

The same values can be overridden at runtime without editing the file:

```bash
ros2 run avros_data_collection avros_dataCollection --ros-args \
  -p rgb_camera_index:=0 \
  -p thermal_camera_index:=1 \
  -p capture_rate_hz:=10.0
```

## Building and Running

Build the package from the workspace root:

```bash
colcon build --packages-select avros_data_collection
source install/setup.bash
```

Start the data-collection node:

```bash
ros2 run avros_data_collection avros_dataCollection
```

The default output directory is `data_collection` in the directory where the node is started. A different output directory can be selected with:

```bash
ros2 run avros_data_collection avros_dataCollection --ros-args \
  -p output_directory:=/path/to/run_data
```

## How Data Collection Works

At the configured capture rate, the node performs the following actions:

1. Reads one frame from the RGB camera using OpenCV.
2. Reads one frame from the thermal camera using OpenCV.
3. Records the current ROS timestamp.
4. Finds the nearest available IMU, GPS, and odometry messages.
5. Finds the nearest actuator command and actual actuator state.
6. Saves both images using the same image counter.
7. Appends the row to `data.csv` and flushes it immediately.

The two camera reads happen in the same timer callback. This provides consistent software timing, but ordinary USB cameras are not guaranteed to expose their images at exactly the same instant. Exact exposure synchronization requires cameras with hardware triggering or another shared timing mechanism.

Sensor values are matched by timestamp. If a sensor value is not available within the configured age limit, its CSV field is left blank.

## Output Format

The default output has this structure:

```text
data_collection/
├── data.csv
└── imgs/
    ├── rgb/
    │   └── rgb_<zero_padded_image_count>_<date>.jpg
    └── thermal/
        └── thermal_<zero_padded_image_count>_<date>.jpg
```

For example:

```text
rgb_000012_20260930T143015.123456Z.jpg
thermal_000012_20260930T143015.123456Z.jpg
```

The matching image counter indicates that the RGB and thermal files belong to the same capture cycle.

The CSV contains:

- Image count, capture timestamp, and relative image paths
- Commanded throttle, steering, brake, mode, and emergency-stop state
- Actual throttle, steering, brake, mode, emergency-stop state, and watchdog state
- IMU orientation, angular velocity, and linear acceleration
- GPS validity, latitude, longitude, altitude, status, and horizontal covariance
- Linear velocity from odometry
- Angular velocity, including the turning rate around the vertical axis

The commanded control values are the primary imitation-learning labels. The
actual actuator values are included to show what the vehicle executed, which
can differ from the command because of limits, delays, or a watchdog.

## Ending a Run

The CSV header is written when the node starts. Each captured sample is appended and flushed immediately while the node is running, so the file can be inspected during the run and recent data is less likely to be lost. Press `Ctrl-C` to stop the node; it flushes and closes the CSV, then releases both cameras.

## Troubleshooting

### A camera cannot be opened

Check the connected devices:

```bash
v4l2-ctl --list-devices
```

Then update `RGB_CAMERA_INDEX` or `THERMAL_CAMERA_INDEX`, or provide different ROS parameter values at runtime.

### The thermal camera is not detected

USB-C describes the connector, not necessarily the camera protocol. The thermal camera must appear as a standard Linux video device for OpenCV to open it directly. A camera requiring a vendor-specific SDK will need an additional capture interface before this node can use it.

### Sensor fields are blank

Confirm that the expected ROS topics are active:

```bash
ros2 topic list
ros2 topic echo /imu/data
ros2 topic echo /gnss
ros2 topic echo /wheel_odom
```

If the system uses different topic names, update the parameters `imu_topic`, `gps_topic`, and `odometry_topic`.
