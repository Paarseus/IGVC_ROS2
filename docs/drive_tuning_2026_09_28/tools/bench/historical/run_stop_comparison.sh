#!/bin/bash
# A/B stop test through actuator_node: (A) deployed v2 + yaml gains, (B) stop-to-brake v2b + standard settings.
# RAM-only parameter changes; restores the original SPARK settings at the end. Tracks off the ground.

source /opt/ros/humble/setup.bash; source ~/IGVC_ROS2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/IGVC_ROS2/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml
B=~/IGVC_ROS2/docs/drive_tuning_2026_09_28/tools/bench
stop_act() { for p in $(pgrep -f "[r]os2 launch avros_bringup") $(pgrep -f "[l]ib/avros_control/actuator_node"); do kill $p; done; sleep 3; for p in $(pgrep -f "[l]ib/avros"); do kill -9 $p; done; sleep 1; }
start_act() { setsid bash -c 'exec ros2 launch avros_bringup actuator.launch.py' > /tmp/act_$1.log 2>&1 < /dev/null & sleep 9; }
flash() { stop_act; teensy_loader_cli --mcu=TEENSY41 -s -w $1 >/dev/null 2>&1; sleep 4; }

echo "=== A: deployed v2 (S = velocity 0), yaml gains (P 0.0007, I 2.5e-7), coast, 80 A"
flash ~/fw_v2/teensy_diff_drive_v2.ino.hex
python3 $B/bench.py "CF B" "PW B idleMode 0" "PW B smartStallA 80" "PW B smartFreeA 20" >/dev/null
start_act A
python3 $B/actuator_stop_test.py
stop_act
python3 $B/bench.py D | grep -o "sf=[^ ]*" | sed 's/^/  sticky faults after A: /'

echo "=== B: v2b (S / watchdog = duty 0 -> Brake), standard: P 0.0003, I 0, IZone 0, brake, 50 A"
flash ~/fw_v2b/teensy_diff_drive_v2.ino.hex
python3 $B/bench.py "CF B" "PW B idleMode 1" "PW B smartStallA 50" "PW B smartFreeA 50" >/dev/null
start_act B
ros2 param set /actuator_node kP 0.0003 >/dev/null; ros2 param set /actuator_node kI 0.0 >/dev/null; ros2 param set /actuator_node kIZone 0.0 >/dev/null
sleep 1; grep -E "OK K[PIZ]=" /tmp/act_B.log | tail -3 | sed 's/.*ack: /  /'
python3 $B/actuator_stop_test.py
stop_act
python3 $B/bench.py D | grep -o "sf=[^ ]*" | sed 's/^/  sticky faults after B: /'

echo "=== restore SPARK RAM settings (coast, 80/20 A, yaml gains); firmware left on v2b"
python3 $B/bench.py "PW B idleMode 0" "PW B smartStallA 80" "PW B smartFreeA 20" KP0.0007 KI2.5e-07 KZ600 | grep -c "res=0" | sed 's/^/  confirmed writes: /'
echo DONE
