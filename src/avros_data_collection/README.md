# avros_dataCollection

This standalone ROS 2 node opens one external RGB camera and one external
thermal camera with OpenCV, records the pair at a fixed rate, and stores the
vehicle state nearest to the capture timestamp.

Run it after building the workspace:

```bash
ros2 run avros_data_collection avros_dataCollection
```

Useful overrides include:

```bash
ros2 run avros_data_collection avros_dataCollection --ros-args \
  -p rgb_camera_index:=0 -p thermal_camera_index:=1 \
  -p output_directory:=/path/to/run \
  -p capture_rate_hz:=10.0
```

The editable constants `RGB_CAMERA_INDEX` and `THERMAL_CAMERA_INDEX` are at
the top of `data_collection_node.py`; the ROS parameters override them. The
output contains `imgs/rgb`, `imgs/thermal`, and `data.csv`. Pressing
Ctrl-C ends the run and flushes the CSV.
