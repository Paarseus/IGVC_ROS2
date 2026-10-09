#!/bin/bash
# Low-speed verification: kS sweep at the motor layer, then cmd_vel through actuator_node (kS 0 and kS = $1).
source /opt/ros/humble/setup.bash; source ~/IGVC_ROS2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/IGVC_ROS2/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml
B=~/IGVC_ROS2/docs/drive_tuning_2026_09_28/tools/bench
Y=~/IGVC_ROS2/install/avros_bringup/share/avros_bringup/config/actuator_params.yaml
stop_all() { for p in $(pgrep -f "[r]os2 launch avros_bringup") $(pgrep -f "[r]os2 run avros_control") $(pgrep -f "[l]ib/avros_control/actuator_node") $(pgrep -f "[l]ib/avros_webui/webui_node"); do kill $p; done; sleep 3; for p in $(pgrep -f "[l]ib/avros_control") $(pgrep -f "[l]ib/avros_webui"); do kill -9 $p; done; fuser -k 8000/tcp 2>/dev/null; sleep 1; }
stop_all
if [ -z "$1" ]; then
  for K in 0.18 0.22 0.26; do echo "##### motor layer, kS=$K V"; timeout 200 python3 $B/scopes.py s3low 0.0002 $K | sed 's/S3.8 low speed //'; done
else
  for K in 0.0 $1; do
    echo "##### actuator_node path, kS_left=kS_right=$K V"
    setsid bash -c "exec ros2 run avros_control actuator_node --ros-args -r __node:=actuator_node --params-file $Y -p kS_left:=$K -p kS_right:=$K" > /tmp/act_low.log 2>&1 < /dev/null & sleep 9
    grep -E "gains set" /tmp/act_low.log | sed 's/.*]: /  /'
    ACCEPT_KS=$K timeout 200 python3 $B/acceptance_pipeline.py | grep -E "^P5|passed"
    stop_all
  done
fi
setsid bash -c 'exec ros2 launch avros_bringup webui.launch.py' > /tmp/webui.log 2>&1 < /dev/null &
sleep 8; echo LOWDONE
