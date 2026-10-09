#!/bin/bash
# Run the tuning protocol for ONE parameter set (docs/drive_tuning_2026_09_28/TUNING_LOG.md §3).
#   tune_set.sh <label> "<gain commands>" <reps> <straight|spin|both> ["<extra;commands>" ["<restore;commands>"]]
# Gain commands are Teensy commands, e.g. "KF0.0023 KP0.0002 KI0 KD0 KZ0 KSL0.18 KSR0.18". Extra commands are
# typed SPARK writes for bench.py, e.g. "PW B kIMaxAccum 0.2". Every value is verified from the controllers' own
# replies before any run. Guards (fault flags, bus < 9.5 V, current > 45 A, speed ripple > 60 RPM) abort the set and
# restore the baseline gains. A power-cycle of the drivers (e-stop) discards that run and aborts: resume with
# REP_FROM=<rep> in the environment. All changes are RAM only. The speed ramp M is 20 (about 0.33 m/s^2) during each run.
LABEL=$1; GAINS=$2; REPS=${3:-3}; PROTO=${4:-both}; EXTRA=${5:-}; RESTORE_EXTRA=${6:-}
BASE="KF0.0023 KP0.0002 KI0 KD0 KZ0 KSL0.18 KSR0.18"
export GT_SESSION=${GT_SESSION:-/home/dinosaur/ground_tests/2026-09-30_1059_concrete}
source /opt/ros/humble/setup.bash; source ~/IGVC_ROS2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/IGVC_ROS2/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml
TOOLS=~/IGVC_ROS2/docs/drive_tuning_2026_09_28/tools
cd $TOOLS
R=$GT_SESSION/runs; RES=$GT_SESSION/results; mkdir -p $RES
LOG=/tmp/tune_${LABEL}.log; : > $LOG
say() { echo "$(date +%H:%M:%S) $*" | tee -a $LOG; }

push() {   # push "<gain tokens>"; verify every token against the Teensy reply
  python3 drive_tuner.py cmd $1 D < /dev/null > /tmp/tune_push.txt 2>&1
  python3 - "$1" /tmp/tune_push.txt <<'PY'
import re, sys
tokens = sys.argv[1].split(); text = open(sys.argv[2]).read()
bad = []
for tk in tokens:
    m = re.match(r'^(KSL|KSR|KF|KP|KI|KD|KZ)(-?[0-9.]+(?:e-?[0-9]+)?)$', tk)
    if not m: bad.append(tk + ' (unparsed)'); continue
    name, want = m.group(1), float(m.group(2))
    r = re.findall(r'OK ' + name + r'=(-?[0-9.eE+-]+)', text)
    if not r: bad.append(tk + ' (no reply)'); continue
    got = float(r[-1])
    if abs(got - want) > 1e-3 * abs(want) + 1e-9: bad.append(f'{tk} (read back {got})')
if bad: print('VERIFY FAILED: ' + ', '.join(bad)); sys.exit(1)
print('verified: ' + ' '.join(tokens))
PY
}
extra() {  # extra "<cmd;cmd>" : typed SPARK writes, each must answer res=0 on both sides
  IFS=';' read -ra CMDS <<< "$1"
  for c in "${CMDS[@]}"; do
    [ -z "$c" ] && continue
    n=$(python3 bench/bench.py "$c" 2>&1 | grep -c "res=0")
    [ "$n" -ge 2 ] && echo "verified: $c" || { echo "VERIFY FAILED: $c ($n confirmations)"; return 1; }
  done
}
abort() { say "!! ABORT: $1 -> restoring baseline gains"; push "$BASE" | tee -a $LOG; [ -n "$RESTORE_EXTRA" ] && extra "$RESTORE_EXTRA" | tee -a $LOG; exit 2; }

say "== set $LABEL: $GAINS ${EXTRA:+| $EXTRA} | reps $REPS | $PROTO"
push "$GAINS" | tee -a $LOG || abort "gain verification"
[ -n "$EXTRA" ] && { extra "$EXTRA" | tee -a $LOG || abort "extra verification"; }

run() { name=$1; seq=$2; rep=$3
  python3 drive_tuner.py vel --name $name --pre 3 --seq "$seq" --m 20 --ros --test 3.x --rep $rep --note "tuning $LABEL" < /dev/null > /tmp/tune_run.log 2>&1
  d=$(ls -d $R/*_$name | tail -1)
  out=$(python3 analyze.py guard $d --window 1.2 --min-seg 3.0 2>&1); rc=$?
  say "$out"
  if [ $rc -ne 0 ]; then
    case "$out" in
      *"counter reset"*)   # drivers were power-cycled (physical e-stop): this run is void; do NOT drive again by ourselves
        mv "$d" "$(echo $d | sed 's#_tun_#_BADRUN-tun-#')"
        abort "drivers power-cycled during $name (e-stop?): run discarded; resume with REP_FROM=$rep" ;;
      *) abort "guard: $out" ;;
    esac
  fi
  grep -q "STOP key" /tmp/tune_run.log && abort "stopped by key"
  sleep 4; }

A="0.05,0:4 0.1,0:3.5 0.2,0:3.5 0.3,0:3.5 0,0:3.5";   Ar="-0.05,0:4 -0.1,0:3.5 -0.2,0:3.5 -0.3,0:3.5 0,0:3.5"
C="0.5,0:3.2 0.7,0:3.2 0,0:3.5";                      Cr="-0.5,0:3.2 -0.7,0:3.2 0,0:3.5"
S="0,0.1:6 0,0.3:6 0,0.6:6 0,0:3.5";                  Sr="0,-0.1:6 0,-0.3:6 0,-0.6:6 0,0:3.5"
for rep in $(seq ${REP_FROM:-1} $REPS); do
  if [ "$PROTO" = straight ] || [ "$PROTO" = both ]; then
    run tun_${LABEL}_A_fwd_r$rep "$A" $rep;  run tun_${LABEL}_A_rev_r$rep "$Ar" $rep
    run tun_${LABEL}_C_fwd_r$rep "$C" $rep;  run tun_${LABEL}_C_rev_r$rep "$Cr" $rep
  fi
  if [ "$PROTO" = spin ] || [ "$PROTO" = both ]; then
    run tun_${LABEL}_S_ccw_r$rep "$S" $rep;  run tun_${LABEL}_S_cw_r$rep "$Sr" $rep
  fi
done
say "== analysis $LABEL"
python3 analyze.py delivery $R/*_tun_${LABEL}_* --window 1.2 --min-seg 3.0 --csv $RES/tuning_results.csv --label $LABEL --gains "$GAINS ${EXTRA}" 2>&1 | tee $RES/tune_${LABEL}.txt | tee -a $LOG | tail -n +1 > /dev/null
say "SET DONE $LABEL (table in $RES/tune_${LABEL}.txt)"
