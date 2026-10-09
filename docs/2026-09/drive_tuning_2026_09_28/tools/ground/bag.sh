#!/bin/bash
# Record a ROS bag for one test run (Scopes 6-8, and alongside motor runs if wanted).
#   bag.sh <test_id> <rep> [seconds]      e.g.  bag.sh 6.2 3 60      (Ctrl-C ends early)
# Bags go to $GT_SESSION/bags/<test>_r<rep>_<time>/ and a row is added to $GT_SESSION/runs/journal.csv.
: ${GT_SESSION:?run session_start.sh and export GT_SESSION first}
TEST=${1:?usage: bag.sh <test_id> <rep> [seconds]}; REP=${2:?rep}; SECS=${3:-0}
source /opt/ros/humble/setup.bash; source ~/IGVC_ROS2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/IGVC_ROS2/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml
NAME=${TEST}_r${REP}_$(date +%H%M%S)
OUT=$GT_SESSION/bags/$NAME
TOPICS="/cmd_vel /wheel_odom /avros/wheel_debug /avros/actuator_state /avros/actuator_command \
/imu/data /gnss /nmea /rtcm /odometry/filtered /odometry/global /odometry/gps /tf /tf_static \
/diagnostics /rosout /plan /local_plan /parameter_events"
START=$(date '+%Y-%m-%d %H:%M:%S'); T0=$(date +%s)
if [ "$SECS" -gt 0 ]; then
  timeout -s INT $SECS ros2 bag record -o $OUT $TOPICS
else
  ros2 bag record -o $OUT $TOPICS
fi
J=$GT_SESSION/runs/journal.csv
[ -f $J ] || echo "start_local,test,rep,run,dir,duration_s,stopped_by_key,note" > $J
echo "$START,$TEST,$REP,bag,bags/$NAME,$(( $(date +%s) - T0 )),,ros bag" >> $J
echo "recorded $OUT"
