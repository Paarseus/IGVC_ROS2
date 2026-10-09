#!/bin/bash
# Speed delivery campaign (GROUND_TEST_PLAN 3.1/3.2), Teensy direct, M20 (about 0.33 m/s^2), guards on faults and bus V.
export GT_SESSION=/home/dinosaur/ground_tests/2026-09-30_1059_concrete
source /opt/ros/humble/setup.bash; source ~/IGVC_ROS2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/IGVC_ROS2/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml
cd ~/IGVC_ROS2/docs/drive_tuning_2026_09_28/tools
LOG=/tmp/delivery_campaign.log
say() { echo "$(date +%H:%M:%S) $*" | tee -a $LOG; }
: > $LOG
say "push gains + read back"
python3 drive_tuner.py cmd KF0.0023 KP0.0002 KI0 KD0 KZ0 KSL0.18 KSR0.18 D < /dev/null 2>&1 | grep -E "OK K|DIAG" | sed -E 's/DIAG.*(ks=[0-9.\/]+).*/DIAG \1/' | tee -a $LOG
run() { name=$1; seq=$2; rep=$3
  python3 drive_tuner.py vel --name $name --pre 3 --seq "$seq" --m 20 --ros --test 3.1 --rep $rep --note "delivery, M20, concrete sidewalk" < /dev/null > /tmp/delrun.log 2>&1
  d=$(ls -d $GT_SESSION/runs/*_$name | tail -1)
  python3 - "$d" "$name" <<PY | tee -a $LOG
import csv, sys, json, re
d, name = sys.argv[1], sys.argv[2]
r = list(csv.DictReader(open(d + "/teensy.csv")))
f = lambda k: [float(x[k]) for x in r]
meta = json.load(open(d + "/meta.json"))
m = re.search(r"sf=(0x\w+/0x\w+)", meta.get("diag_end") or ""); sfl = m.group(1) if m else "?"
bus = [min(a, b) for a, b in zip(f("L_bus_V"), f("R_bus_V")) if a > 1 and b > 1]
dl = (f("L_pos_rot")[-1] - f("L_pos_rot")[0]) * 0.01994; dr = (f("R_pos_rot")[-1] - f("R_pos_rot")[0]) * 0.01994
pk = max(max(abs(v) for v in f("L_vel_rpm")), max(abs(v) for v in f("R_vel_rpm"))); pa = max(max(f("L_current_A")), max(f("R_current_A")))
print(f"{name:16s} travel L {dl:+.2f} R {dr:+.2f} m | peak {pk:.0f} RPM | min bus {min(bus):.1f} V | peak {pa:.0f} A | faults {sfl}")
open("/tmp/delflag", "w").write("BAD" if (sfl not in ("0x00/0x00", "?") or min(bus) < 9.5) else "ok")
PY
  [ "$(cat /tmp/delflag)" = "BAD" ] && { say "!! STOP: fault or low bus voltage after $name"; exit 1; }
  grep -q "STOP key" /tmp/delrun.log && { say "!! stopped by key"; exit 1; }
  sleep 4; }
A="0.05,0:4 0.1,0:3.5 0.3,0:3.5 0.5,0:3.5 0,0:3.5"
Ar="-0.05,0:4 -0.1,0:3.5 -0.3,0:3.5 -0.5,0:3.5 0,0:3.5"
B="0.7,0:5 0,0:4"
Br="-0.7,0:5 0,0:4"
for rep in 1 2 3 4 5; do
  say "== repeat $rep"
  run del_A_fwd_r$rep "$A" $rep
  run del_A_rev_r$rep "$Ar" $rep
  run del_B_fwd_r$rep "$B" $rep
  run del_B_rev_r$rep "$Br" $rep
done
for rep in 1 2 3; do
  say "== spin repeat $rep"
  run del_S_ccw_r$rep "0,0.1:6 0,0.3:6 0,0:3.5" $rep
  run del_S_cw_r$rep  "0,-0.1:6 0,-0.3:6 0,0:3.5" $rep
done
say "CAMPAIGN DONE"
