# Record, drive 0.4 m/s straight for 10 s via /cmd_vel, stop, analyse. Arg: run name.
source /tmp/rtk_env.sh
D=~/field_2026_09_27/straight/$1; mkdir -p $(dirname $D)
timeout -s INT 20 ros2 bag record -o $D /filter/positionlla /gnss /status /imu/data /avros/wheel_debug /cmd_vel /avros/actuator_command /odometry/filtered > /dev/null 2>&1 &
sleep 3
timeout -s INT 10 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}}" > /dev/null 2>&1
wait
python3 ~/field_2026_09_27/tools/straight_analyze.py $D 2>&1 | grep -v rosbag2_storage
