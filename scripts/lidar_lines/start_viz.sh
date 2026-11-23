#!/usr/bin/env bash
# (Re)start the LiDAR line detector + its RViz view on the NoMachine display.
# Assumes the Ouster is already up (sensors.launch.py). Log: /tmp/lidar_lines_launch.log
#   scripts/lidar_lines/start_viz.sh [extra launch args, e.g. use_sim_time:=true]
pkill -f "[l]idar_lines.launch.py" ; pkill -f "[l]ib/avros_perception/lidar_line_detector" ; pkill -f "[r]viz_lidar_lines"
sleep 2
export DISPLAY=${DISPLAY:-:1001}
setsid bash -c "source /opt/ros/humble/setup.bash && source $HOME/IGVC_ROS2/install/setup.bash && exec ros2 launch avros_perception lidar_lines.launch.py rviz:=true $*" \
  > /tmp/lidar_lines_launch.log 2>&1 < /dev/null &
echo "started; log: /tmp/lidar_lines_launch.log"
