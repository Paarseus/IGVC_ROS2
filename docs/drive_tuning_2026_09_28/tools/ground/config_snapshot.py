#!/usr/bin/env python3
"""Snapshot of the motor-controller configuration for a ground-test session (GROUND_TEST_PLAN.md §1).

Reads CHK, firmware versions, every tunable SPARK parameter and DIAG, saves
$GT_SESSION/config/spark_<label>.json, and lists differences from the saved final
configuration (results/raw/s2_manifest_fw26_final.json). Needs the serial port free
(actuator_node / web UI stopped). Read-only: writes nothing to the controllers.

  python3 config_snapshot.py start      # at the start of a session
  python3 config_snapshot.py end        # at the end
"""
import json, os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'bench'))
from tio import Teensy  # noqa: E402

label = sys.argv[1] if len(sys.argv) > 1 else 'snapshot'
sess = os.environ.get('GT_SESSION') or sys.exit('GT_SESSION is not set: run session_start.sh first')
out_dir = os.path.join(sess, 'config')
os.makedirs(out_dir, exist_ok=True)
golden = json.load(open(os.path.join(HERE, '..', '..', 'results', 'raw', 's2_manifest_fw26_final.json')))

t = Teensy()
t.send('X0', 0.3)
chk = [l for l in t.send('CHK', 3.5) if l.startswith('CHK')]
names = [l.split()[3] for l in t.send('PT', 1.0) if l.startswith('PT ')]
man = {n: t.pr(n) for n in names}
man['_chk'] = chk
man['_firmware'] = [l for l in t.send('FV', 0.8) if l.startswith('FV ')]
man['_diag'] = t.diag()
man['_time'] = time.strftime('%Y-%m-%d %H:%M:%S')
path = os.path.join(out_dir, f'spark_{label}.json')
json.dump(man, open(path, 'w'), indent=1, default=str)

diff = []
for n, v in golden.items():
    if n.startswith('_') or n not in man:
        continue
    for s in 'LR':
        a, b = v.get(s), man[n].get(s)
        try:
            same = abs(float(a) - float(b)) <= 1e-9 + 1e-6 * abs(float(a))
        except (TypeError, ValueError):
            same = a == b
        if not same:
            diff.append(f'{n} {s}: saved {a}, now {b}')
print(chk[-1] if chk else 'CHK: no reply')
print(man['_firmware'])
print(f'{len(names)} parameters read; {len(diff)} differ from the saved final configuration')
for d in diff:
    print('  ', d)
print('   (kP/kV/kS differences are expected after actuator_node has pushed yaml values; the controllers keep\n'
      '    them in RAM until power-off)')
print(f'saved {path}')
