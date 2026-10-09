#!/usr/bin/env bash
# Record the Phase 0 topics to a bag. Read-only (subscribes only).
# Usage: phase0_record.sh <seconds> <output_dir>
set -e   # no -u: ROS setup files reference unset variables
SECS=${1:-600}
OUT=${2:-$HOME/field_2026_09_27/phase0_static}
source /opt/ros/humble/setup.bash
source "$HOME/IGVC_ROS2/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/IGVC_ROS2/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml
mkdir -p "$(dirname "$OUT")"
timeout -s INT "$SECS" ros2 bag record -o "$OUT" \
  /gnss /filter/positionlla /status /nmea /rtcm \
  /imu/data /wheel_odom /avros/wheel_debug /avros/actuator_state \
  /odometry/filtered /odometry/global /odometry/gps \
  /cmd_vel /avros/actuator_command /tf /tf_static || true
echo "bag written: $OUT"
