#!/bin/bash
# Pipeline comparison on FW 26 (no flashing): actuator_node with the yaml gains, kP overridden per case,
# hall depth set in RAM per case. Runs actuator_stop_test.py + pipeline_scopes.py per case. Tracks off the ground.
source /opt/ros/humble/setup.bash; source ~/IGVC_ROS2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/IGVC_ROS2/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml
B=~/IGVC_ROS2/docs/drive_tuning_2026_09_28/tools/bench
stop_act() { for p in $(pgrep -f "[r]os2 launch avros_bringup") $(pgrep -f "[r]os2 run avros_control") $(pgrep -f "[l]ib/avros_control/actuator_node"); do kill $p; done; sleep 3; for p in $(pgrep -f "[l]ib/avros_control"); do kill -9 $p; done; sleep 1; }
# usage: run_pipeline_fw26.sh [P_DEPTH_PERIOD ...]   e.g. 0.0002_2_0.016   (default: the 3 original cases)
[ $# -eq 0 ] && set -- 0.0003_3_0.03125 0.0002_3_0.03125 0.0002_2_0.03125
for CASE in "$@"; do
  IFS=_ read -r P DEPTH PERIOD <<< "$CASE"
  echo "=== CASE kP=$P hallAvgDepth=$DEPTH hallSamplePeriod=$PERIOD (yaml kFF 0.0023, kI 0, Brake, v2d stop-to-idle)"
  stop_act
  python3 $B/bench.py "CF B" "PW B idleMode 1" "PW B hallAvgDepth $DEPTH" "PW B hallSamplePeriod $PERIOD" >/dev/null
  # run the node directly with the yaml + a kP override (no ros2 CLI param service: it hangs under load)
  Y=~/IGVC_ROS2/install/avros_bringup/share/avros_bringup/config/actuator_params.yaml
  setsid bash -c "exec ros2 run avros_control actuator_node --ros-args -r __node:=actuator_node --params-file $Y -p kP:=$P" > /tmp/act_fw26.log 2>&1 < /dev/null & sleep 9
  grep -E "OK K(F|P|I|Z)=" /tmp/act_fw26.log | sed 's/.*ack: /  pushed: /' | head -4
  timeout 120 python3 $B/actuator_stop_test.py
  timeout 180 python3 $B/pipeline_scopes.py | grep -E "S9_twist|S4b|S5_H6"
  stop_act
  python3 $B/bench.py D | grep -o "sf=[^ ]*" | sed 's/^/  sticky faults: /'
done
python3 $B/bench.py "PW B hallAvgDepth 2" "PW B hallSamplePeriod 0.03125" "CF B" | grep -c "res=0" | sed 's/^/restored depth 2 and period 0.03125, confirmed writes: /'
echo FW26PIPEDONE
