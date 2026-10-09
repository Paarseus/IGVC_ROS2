#!/bin/bash
# Run pipeline_scopes.py with the standard setup (v2b stop-to-idle, Brake, 50 A, P 0.0003, I 0), RAM only, then restore.
source /opt/ros/humble/setup.bash; source ~/IGVC_ROS2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/IGVC_ROS2/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml
B=~/IGVC_ROS2/docs/drive_tuning_2026_09_28/tools/bench
stop_act() { for p in $(pgrep -f "[r]os2 launch avros_bringup") $(pgrep -f "[l]ib/avros_control/actuator_node"); do kill $p; done; sleep 3; for p in $(pgrep -f "[l]ib/avros"); do kill -9 $p; done; sleep 1; }
stop_act
python3 $B/bench.py "CF B" "PW B idleMode 1" "PW B smartStallA 50" "PW B smartFreeA 50" >/dev/null
setsid bash -c 'exec ros2 launch avros_bringup actuator.launch.py' > /tmp/act_pipe.log 2>&1 < /dev/null & sleep 9
ros2 param set /actuator_node kP 0.0003 >/dev/null; ros2 param set /actuator_node kI 0.0 >/dev/null; ros2 param set /actuator_node kIZone 0.0 >/dev/null
python3 $B/pipeline_scopes.py
stop_act
python3 $B/bench.py D | grep -o "sf=[^ ]*" | sed 's/^/sticky faults after: /'
python3 $B/bench.py "PW B idleMode 0" "PW B smartStallA 80" "PW B smartFreeA 20" KP0.0007 KI2.5e-07 KZ600 | grep -c "res=0" | sed 's/^/restored confirmed writes: /'
echo PIPEDONE
