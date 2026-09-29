# avros_dataCollection

This standalone ROS 2 node records synchronized RGB and thermal image pairs
and the vehicle state nearest to the paired image timestamp.

Run it after building the workspace:

```bash
ros2 run avros_data_collection avros_dataCollection
```

Useful overrides include:

```bash
ros2 run avros_data_collection avros_dataCollection --ros-args \
  -p thermal_topic:=/my_thermal/image_raw \
  -p output_directory:=/path/to/run \
  -p capture_rate_hz:=10.0
```

The output contains `imgs/rgb`, `imgs/thermal`, and `data.csv`. Pressing
Ctrl-C ends the run and flushes the CSV.
