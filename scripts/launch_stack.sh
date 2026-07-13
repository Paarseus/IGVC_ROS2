#!/usr/bin/env bash
# launch_stack.sh — launch the full IGVC autonomous stack WITH camera + perception.
#
# Brings up navigation.launch.py with:
#   enable_zed_front  + enable_perception  (front ZED + lane perception)
#   enable_velodyne   + enable_ntrip       (LiDAR + RTK corrections)
# plus everything navigation.launch.py already includes (EKF map+odom, navsat,
# Nav2 servers, autonomy_monitor for the §I.2 safety light, foxglove_bridge).
#
# Stops the avros-webui service first so its actuator_node releases /dev/ttyACM0
# (two actuator_nodes on the same serial port fight -> motor stepping).
#
# Does NOT start RViz on purpose — RViz over NoMachine starves the MPPI control
# loop (watch from laptop Foxglove via the foxglove_bridge this launch starts).
#
# Foreground: Ctrl-C stops the whole stack cleanly. Pass extra launch args
# through, e.g.:  scripts/launch_stack.sh enable_mission_manager:=true
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "[launch_stack] stopping avros-webui (frees Teensy /dev/ttyACM0)..."
if sudo -n systemctl stop avros-webui.service 2>/dev/null; then
  echo "  webui stopped"
else
  echo "  could NOT stop webui non-interactively -> run: sudo systemctl stop avros-webui"
fi

# wait for the serial port to actually free up (webui actuator_node release)
for _ in 1 2 3 4 5; do
  fuser /dev/ttyACM0 >/dev/null 2>&1 || break
  sleep 0.5
done

set +u  # ROS setup.bash references unset vars (AMENT_TRACE_SETUP_FILES); nounset aborts otherwise
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "$WS_ROOT/install/setup.bash"
set -u
# navigation.launch.py sets RMW_IMPLEMENTATION + CYCLONEDDS_URI itself.

echo "[launch_stack] launching navigation.launch.py (camera + perception + velodyne + ntrip)"
exec ros2 launch avros_bringup navigation.launch.py \
  enable_zed_front:=true \
  enable_perception:=true \
  enable_velodyne:=true \
  enable_ntrip:=true \
  "$@"
