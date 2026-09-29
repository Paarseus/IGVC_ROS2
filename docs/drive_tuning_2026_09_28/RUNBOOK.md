# Runbook: Commands for Bench and Ground Tests

Current setup: SPARK MAX firmware 26.1.5, Teensy firmware v2d, gains from `src/avros_bringup/config/actuator_params.yaml`. Log in with `ssh -t jetson`; the `-t` makes the SPACE stop key work. The workspace is `~/IGVC_ROS2`.

## 1. Health check (every session)
```bash
cd ~/IGVC_ROS2/docs/drive_tuning_2026_09_28/tools/bench
python3 bench.py CHK "#sleep 3"        # must print: CHK OK   (stop the web UI first, see §2)
```
If it prints `CHK FAIL`, do not drive. The reasons are listed on the line: motor type, idle mode L ≠ R, kV, kS parameter, filter.

## 2. One program on the motor serial port at a time
```bash
# stop the web UI / actuator_node before any direct Teensy tool
for p in $(pgrep -f "[r]os2 launch avros_bringup") $(pgrep -f "[l]ib/avros_control/actuator_node") $(pgrep -f "[l]ib/avros_webui/webui_node"); do kill $p; done
fuser -k 8000/tcp
# restart the web UI afterwards
setsid bash -c 'source /opt/ros/humble/setup.bash && source ~/IGVC_ROS2/install/setup.bash && exec ros2 launch avros_bringup webui.launch.py' > /tmp/webui.log 2>&1 < /dev/null &
```

## 3. Bench acceptance (tracks off the ground, about 10 minutes)
```bash
~/IGVC_ROS2/docs/drive_tuning_2026_09_28/tools/bench/run_acceptance.sh
```
It stops the web UI, runs every bench check with pass/fail limits, and restarts the web UI. Results: `~/bench_scopes_2026_09_28/acceptance_*.json`.

## 4. Ground tests (GROUND_ACCEPTANCE.md)
Sensors for ground tests (IMU, GNSS, RTK corrections; no actuator_node):
```bash
source /opt/ros/humble/setup.bash && source ~/IGVC_ROS2/install/setup.bash
ros2 launch avros_bringup sensors.launch.py
```
Tuning tool shell:
```bash
source /opt/ros/humble/setup.bash && source ~/IGVC_ROS2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/IGVC_ROS2/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml
cd ~/IGVC_ROS2/docs/drive_tuning_2026_09_28/tools; T=./drive_tuner.py; A=./analyze.py
```

| Test | Commands | Result |
|---|---|---|
| G1 motor model | `$T ramp --name qs_fwd --max 7 --rate 0.5 --ros` (also `--dir rev`), `$T steps --name dyn_fwd --levels 3,5,7 --ros` (also `--dir rev`), 3 repeats; then `$A ff runs/*qs_* runs/*dyn_*` | kS, kV per track and direction |
| set kS / kV | edit `kS_left`, `kS_right` (and `kFF` if kV changed > 5 %) in actuator_params.yaml; restart actuator_node | actuator_node log shows `OK KSL=…`, `OK KSR=…` |
| G2 speed delivery | run actuator_node (§5), then `/cmd_vel` steps with the pipeline tool, or `$T vel --name g2 --seq "0.1,0:4 0.3,0:4 0.5,0:4 0.7,0:4 1.0,0:4 0,0:2" --ros` | `$A steps runs/*g2*` |
| G3 distance | `$T vel --name straight_05 --seq "0.5,0:24 0,0:2" --ros` (also 1.0 m/s and reverse) | `$A distance runs/*straight*` |
| G4 turning | `$T vel --name spin_06 --seq "0,0:4 0,0.6:21 0,0:3" --ros --m-per-rev <G3>`; arcs `0.5,0.5` / `0.5,0.25` / `0.5,0.125` | `$A turn runs/*spin* --m-per-rev <G3>` |
| G5 stopping | pipeline stop test: `python3 bench/actuator_stop_test.py` with actuator_node running | distance from wheel position |

## 5. actuator_node with a one-off gain override (without editing the yaml)
```bash
Y=~/IGVC_ROS2/install/avros_bringup/share/avros_bringup/config/actuator_params.yaml
ros2 run avros_control actuator_node --ros-args -r __node:=actuator_node --params-file $Y -p kP:=0.0002
```
(`ros2 param set` can hang on the Jetson when the load is high.)

## 6. Save settings to the motor controllers
Only after a test has passed:
```bash
python3 bench/bench.py "PW B <name> <value>" "#sleep 0.5" BURN "#sleep 2.5" CHK "#sleep 3"
```
Expect `OK BURN result L=0 R=0` and `CHK OK`. The gains (kV, P, I) are also pushed from the yaml at every actuator_node start, so the yaml is the source of truth for them.

## 7. Firmware
Build and flash: `firmware/README.md`. After any SPARK MAX firmware update, run §1: the 25 → 26 update reset the motor type to brushed.
