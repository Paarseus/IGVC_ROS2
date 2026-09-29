#!/bin/bash
# Bench acceptance (tracks off the ground): firmware/motor checks, then the actuator_node path.
# Stops the web UI / actuator_node first and restarts the web UI at the end.
source /opt/ros/humble/setup.bash; source ~/IGVC_ROS2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/IGVC_ROS2/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml
B=~/IGVC_ROS2/docs/drive_tuning_2026_09_28/tools/bench
stop_all() { for p in $(pgrep -f "[r]os2 launch avros_bringup") $(pgrep -f "[l]ib/avros_control/actuator_node") $(pgrep -f "[l]ib/avros_webui/webui_node"); do kill $p; done; sleep 3; for p in $(pgrep -f "[l]ib/avros_control") $(pgrep -f "[l]ib/avros_webui"); do kill -9 $p; done; fuser -k 8000/tcp 2>/dev/null; sleep 1; }
stop_all
if [ "$1" != "part2" ]; then
  echo "===== PART 1: firmware and motor layer"
  timeout 600 python3 $B/acceptance_serial.py
fi
echo "===== PART 2: actuator_node command path"
Y=~/IGVC_ROS2/install/avros_bringup/share/avros_bringup/config/actuator_params.yaml
setsid bash -c "exec ros2 run avros_control actuator_node --ros-args -r __node:=actuator_node --params-file $Y" > /tmp/act_accept.log 2>&1 < /dev/null & sleep 9
grep -E "gains set" /tmp/act_accept.log | sed 's/.*]: /  /'
timeout 300 python3 $B/acceptance_pipeline.py
stop_all
setsid bash -c 'exec ros2 launch avros_bringup webui.launch.py' > /tmp/webui.log 2>&1 < /dev/null &
sleep 10; echo "web UI restarted: $(ss -ltn | grep -c ':8000') listener"
echo ACCEPTDONE
