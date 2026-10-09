#!/usr/bin/env bash
# kill_ros2.sh — kill ALL ROS 2 on this machine, cleanly.
#
# Order matters:
#   1. stop the avros-webui systemd service (Restart=on-failure would otherwise
#      respawn it, and it holds /dev/ttyACM0 + runs its own actuator_node).
#   2. SIGINT every `ros2 launch` parent first so actuator_node runs its shutdown
#      and sends the Teensy an 'S' stop (motors brake) — graceful.
#   3. SIGKILL anything still alive (nodes, rviz, daemon, component containers,
#      and the session goal/perception helper scripts).
#   4. verify nothing is left.
#
# Run as a file (bash scripts/kill_ros2.sh) so the pkill patterns can't match
# this shell's own command line. Safe to run repeatedly. Needs sudo for step 1.
set -u

echo "[kill_ros2] stopping avros-webui service (frees /dev/ttyACM0)..."
if sudo -n systemctl stop avros-webui.service 2>/dev/null; then
  echo "  webui stopped"
else
  echo "  could NOT stop webui non-interactively -> run: sudo systemctl stop avros-webui"
fi

# 1) graceful SIGINT to launch parents (orderly node shutdown -> motors stop)
LAUNCHES="$(pgrep -f 'ros2 launch' || true)"
if [ -n "$LAUNCHES" ]; then
  echo "[kill_ros2] SIGINT ros2 launch parents: $(echo "$LAUNCHES" | tr '\n' ' ')"
  echo "$LAUNCHES" | xargs -r kill -INT 2>/dev/null
  sleep 6
fi

# 2) SIGKILL everything still alive
echo "[kill_ros2] SIGKILL remaining ROS processes..."
pkill -KILL -f -- "--ros-args"     2>/dev/null
pkill -KILL -f "ros2 launch"       2>/dev/null
pkill -KILL -f "ros2 run"          2>/dev/null
pkill -KILL -f "ros2 daemon"       2>/dev/null
pkill -KILL -x rviz2               2>/dev/null
pkill -KILL -f component_container 2>/dev/null
pkill -KILL -f _ros2cli            2>/dev/null
pkill -KILL -f autonomy_monitor    2>/dev/null
pkill -KILL -f perception_node     2>/dev/null
# session helper scripts (goal senders / perception capture)
pkill -KILL -f send_forward        2>/dev/null
pkill -KILL -f send_midpoint       2>/dev/null
pkill -KILL -f set_and_grab        2>/dev/null
pkill -KILL -f health_check        2>/dev/null
pkill -KILL -f grab_frame          2>/dev/null
sleep 2

# 3) verify
echo "[kill_ros2] ---- survivors ----"
SURV="$( { pgrep -af -- '--ros-args'; \
           pgrep -af 'rviz2|nav2_|component_container|zed_node|bt_navigator|autonomy_monitor'; \
         } 2>/dev/null | grep -v 'pgrep' || true )"
if [ -n "$SURV" ]; then
  echo "  STILL ALIVE:"; echo "$SURV"
else
  echo "  ALL ROS 2 DEAD"
fi
echo "[kill_ros2] loadavg:$(cut -d' ' -f1-3 /proc/loadavg)"
ls /dev/ttyACM* >/dev/null 2>&1 && echo "[kill_ros2] Teensy present: $(ls /dev/ttyACM*)"
