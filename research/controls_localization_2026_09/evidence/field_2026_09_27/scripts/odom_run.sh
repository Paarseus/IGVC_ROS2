# Odometry accuracy run: heading-hold off, equal track speeds, record, drive, restore. Args: name speed seconds
source /tmp/rtk_env.sh
NAME=$1; V=$2; SECS=$3
D=~/field_2026_09_27/odometry/$NAME; mkdir -p $(dirname $D)
e=$(timeout 5 ros2 topic echo --once /avros/actuator_state --field estop 2>/dev/null | head -1 | tr A-Z a-z)
[ "$e" != "false" ] && { echo "NOT DRIVING: E-stopped ($e)"; exit 0; }
a=$(timeout 3 ros2 topic hz /avros/actuator_command 2>&1 | grep -c "average rate")
[ "$a" != "0" ] && { echo "NOT DRIVING: phone sending joystick (press AUTO)"; exit 0; }
old=$(ros2 param get /actuator_node heading_hold_deadband | grep -oE "[0-9.]+$")
ros2 param set /actuator_node heading_hold_deadband 0.0 > /dev/null && echo "heading-hold OFF (was $old)"
timeout -s INT $((SECS + 10)) ros2 bag record -o $D /gnss /status /imu/data /avros/wheel_debug /wheel_odom /odometry/filtered /cmd_vel > /dev/null 2>&1 &
sleep 3
echo "driving at $V m/s for $SECS s"
timeout -s INT $SECS ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: $V}}" > /dev/null 2>&1
wait
ros2 param set /actuator_node heading_hold_deadband $old > /dev/null && echo "heading-hold restored to $old"
python3 ~/field_2026_09_27/tools/odom_analyze.py $D 2>&1 | grep -v rosbag2_storage
