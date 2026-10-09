#!/bin/bash
# Start a ground-test session (GROUND_TEST_PLAN.md): creates the session folder, records the robot
# configuration, software version, clocks and sensor-link settings. Run on the Jetson.
#   session_start.sh <surface> [note]        e.g.  session_start.sh asphalt "lot B, dry, 24 C"
# Then paste the printed "export GT_SESSION=..." line into every shell used for testing.
SURF=${1:?usage: session_start.sh <asphalt|grass|...> [note]}
NOTE=${2:-}
WS=~/IGVC_ROS2
T=$WS/docs/drive_tuning_2026_09_28/tools
S=~/ground_tests/$(date +%Y-%m-%d_%H%M)_$SURF
mkdir -p $S/runs $S/bags $S/config $S/results
export GT_SESSION=$S

# software and configuration
( cd $WS && git rev-parse HEAD && git status --short ) > $S/config/git.txt 2>&1
( cd $WS && git diff ) > $S/config/git_diff.patch 2>/dev/null
for f in actuator_params.yaml ekf.yaml navsat.yaml xsens.yaml ntrip_params.yaml nav2_params_igvc_autonav.yaml; do
  cp $WS/src/avros_bringup/config/$f $S/config/src_$f 2>/dev/null
  cp $WS/install/avros_bringup/share/avros_bringup/config/$f $S/config/installed_$f 2>/dev/null
done
cp $WS/src/avros_bringup/urdf/avros.urdf.xacro $S/config/ 2>/dev/null

# clocks and serial links (GROUND_TEST_PLAN 0.5, 0.6)
{ date; echo; chronyc tracking 2>&1 || timedatectl 2>&1; echo
  for f in /sys/bus/usb-serial/devices/*/latency_timer; do echo "$f: $(cat $f)"; done
  echo; ls -l /dev/serial/by-id/; } > $S/config/clocks_and_ports.txt 2>&1

# motor controllers, only if nothing else owns the Teensy port
PORT=$(ls /dev/serial/by-id/usb-Teensyduino* 2>/dev/null | head -1)
if [ -n "$PORT" ] && ! fuser "$(readlink -f $PORT)" >/dev/null 2>&1; then
  python3 $T/ground/config_snapshot.py start
else
  echo "!! Teensy port busy or missing: stop actuator_node / web UI and run: python3 $T/ground/config_snapshot.py start"
fi

cat > $S/session.md <<MD
# Ground session $(basename $S)

| Item | Value |
|---|---|
| Date / start | $(date '+%Y-%m-%d %H:%M') |
| Surface | $SURF |
| Place, weather, air temperature | $NOTE |
| Battery position / payload | (fill in) |
| Track tension | (fill in) |
| Battery voltage start / end | (fill in) |
| IMU powered on at | (fill in; warm-up >= 10 min before test 0.2) |
| Operator / safety person | (fill in) |
| Software | $(cd $WS && git rev-parse --short HEAD) $( [ -s $S/config/git_diff.patch ] && echo '(+ local changes, see config/git_diff.patch)') |

## Log (what happened, in order: anything unusual, stops, changes made)
- $(date +%H:%M) session started
MD
echo
echo "Session folder: $S"
echo "Paste in every test shell:   export GT_SESSION=$S"
