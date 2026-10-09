# Ground Test Field Kit

This folder holds the commands for each ground session. The plan and pass limits are in `../../GROUND_TEST_PLAN.md`.

## What gets recorded

Everything for a session goes into one folder on the Jetson: `~/ground_tests/<date>_<time>_<surface>/`.

| Path | Content | Written by |
|---|---|---|
| `session.md` | Surface, weather, battery position and voltage, track tension, operator. Also a timed log of anything unusual. | `session_start.sh` creates it; **you fill in the blanks during the session** |
| `config/` | git version and local changes, all drive and localization yaml files, URDF, clocks and USB settings, motor controller configuration at the start and end (with any difference from the saved setup) | `session_start.sh`, `config_snapshot.py` |
| `runs/<time>_<name>/` | Motor telemetry at 50 Hz, raw Teensy lines, IMU, GNSS, RTK quality, and test arguments (`meta.json`) | `drive_tuner.py` |
| `runs/journal.csv` | One row per run and per bag: time, test ID, repeat, folder, duration, stopped early?, note | `drive_tuner.py`, `bag.sh` |
| `bags/<test>_r<rep>_<time>/` | ROS bag: commands, odometry, IMU, GNSS, RTK, EKF, TF, diagnostics, plans | `bag.sh` (Scopes 6–8) |
| `results/` | Analysis output, copied from the terminal | you (or Claude) |

Copy the session to the laptop afterwards (outside the repo; bags are large):

```bash
rsync -a jetson:~/ground_tests/ ~/ground_tests/
```

Results tables go into the repo as `docs/drive_tuning_2026_09_28/results/GROUND_<date>.md`. Use `GROUND_RESULTS_TEMPLATE.md` for them.

## Before the first session: reference marks and equipment
- Two downward pointers on the robot (a bolt with a pointed tip, or a plumb line): one at the `base_link` point (the IMU spot), one about 1 m ahead of it on the centre line. All tape measurements start from these.
- A 30 m steel tape, chalk or spray chalk, and a straight edge about 2 m long for the start lines.
- Start each straight with the robot squared to a chalk line through both pointers.
- RTK is needed only for tests 0.4, 7.5, 7.6 and 8.1–8.2. Sessions 1–2 run without it (plan §1.1).

## Start of every session

**Shell A:** sensors only, for IMU, GNSS and RTK. `actuator_node` is not running.

```bash
ssh -t jetson
source /opt/ros/humble/setup.bash && source ~/IGVC_ROS2/install/setup.bash
ros2 launch avros_bringup sensors.launch.py
```

**Shell B:** the test shell.

```bash
ssh -t jetson
# stop the web UI / actuator_node first (RUNBOOK.md §2), then:
~/IGVC_ROS2/docs/drive_tuning_2026_09_28/tools/ground/session_start.sh asphalt "place, weather, temperature"
export GT_SESSION=<the folder it printed>
source /opt/ros/humble/setup.bash && source ~/IGVC_ROS2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/IGVC_ROS2/install/avros_bringup/share/avros_bringup/config/cyclonedds.xml
cd ~/IGVC_ROS2/docs/drive_tuning_2026_09_28/tools; T=./drive_tuner.py; A=./analyze.py; R=$GT_SESSION/runs
```

Every `drive_tuner.py` run takes:
- `--test <ID>` and `--rep <n>`: repeats 1–3 are for fitting, 4–5 for validation;
- `--ros`, so the IMU and GNSS are logged;
- optionally `--note "..."`.

SPACE stops the motors at any time.

## Session 1 (asphalt): Scopes 0–3

| Test | Command | Analysis | Pass |
|---|---|---|---|
| 0.1 boot check | shown by `session_start.sh` | — | `CHK OK` |
| 0.2 IMU bias | wait ≥ 10 min after IMU power-on, then `$T listen --name still --secs 180 --ros --test 0.2 --rep 1` | `$A still $R/*_still` | bias ≤ 0.02 °/s |
| 0.4 RTK static | the same run | the same | 100 % FIXED, sd ≤ 2 cm |
| 0.3 gyro scale | `$T vel --name gyro_ccw --pre 5 --seq "0,0.3:110 0,0:5" --ros --test 0.3 --rep 1`, and the same with `-0.3` (`gyro_cw`) | count whole turns; draw a chalk line along the centre-line pointers before and after, measure the angle between them; `$A gyroscale $R/*_gyro_* --turns 5 --residual-deg <angle>` (RTK column is a cross-check) | ≤ 0.3 % |
| 4.5 spin centre | mark the `base_link` pointer, spin half a turn slowly (web UI is fine), mark again; 3 times | half the distance between the marks = spin-centre offset (RTK: `$A circle`) | report the offset |
| 1.2 watchdog | `$T vel --name wd --seq "0.5,0:8" --ros --test 1.2 --rep N`; after about 4 s, from a third shell: `pkill -9 -f "[d]rive_tuner.py vel"` (the commands stop; the Teensy must stop the motors) | time and distance from the last command to standstill, from `lines.log` / `teensy.csv` (the log is cut at the kill; use the GNSS log) | 250–350 ms, 10/10 |
| 1.3 braked stop | `$T vel --name brake --seq "0.7,0:6 0,0:3" --ros --test 1.3 --rep N` | `$A steps` | no backward motion, 10/10 |
| 2.1 voltage ramp | `$T ramp --name qs_fwd --max 7 --rate 0.5 --ros --test 2.1 --rep N`, and `--dir rev` (`qs_rev`) | `$A ff $R/*_qs_* $R/*_dyn_*` | fit r² > 0.9 on reps 4–5 |
| 2.2 voltage steps | `$T steps --name dyn_fwd --levels 2,4,6 --ros --test 2.2 --rep N`, and `--dir rev` | same | same |
| 3.1 delivery | `$T vel --name hold --seq "0.05,0:6 0.1,0:5 0.3,0:5 0.5,0:5 0.7,0:5 1.0,0:5 0,0:3" --ros --test 3.1 --rep N`, and negative speeds | `$A steps $R/*_hold*` | ≤ 2 % at ≥ 0.1 m/s; ≤ 10 % at 0.05 m/s |
| 3.3 step response | `$T vel --name step --seq "0.3,0:4 0.7,0:4 0.3,0:4 0,0:3" --ros --test 3.3 --rep N` | `$A steps $R/*_step*` | settle ≤ 0.4 s after the ramp, overshoot ≤ 5 % |
| 3.5 headroom | `$T vel --name top --seq "0.5,0:4 1.5,0:5 0,0:4" --ros --test 3.5 --rep N` | power (`L/R_applied`) in `teensy.csv` | < 0.95 at 4600 RPM |

Note on `vel`: it uses the plain track geometry (multiplier 1, 0.01994 m/rev). Motor-layer tests are meant to see the uncorrected robot.

At the end of the session:

```bash
python3 ground/config_snapshot.py end
```

Then write the battery voltage and anything unusual in `session.md`.

## Session 2 (asphalt): Scopes 4–5

| Test | Command | Analysis |
|---|---|---|
| 4.1 / 4.2 distance and left/right match | 20 m straight: `$T vel --name s20_03 --pre 10 --seq "0.3,0:70 0,0:10" --ros --test 4.1 --rep N`. Also 0.7 m/s (`0.7,0:31`) and reverse. The 10 s holds average the RTK ends. | Tape from the start mark of the `base_link` pointer to its end mark (along and sideways): `$A tape $R/*_s20_03* --along 20.10,… --side 0.35,…`. RTK cross-check: `$A distance $R/*_s20_*`. Drift per metre = heading change ÷ distance. |
| 4.3 spins | `$T vel --name spin_03 --pre 5 --seq "0,0.3:110 0,0:5" --ros --test 4.3 --rep N` at 0.1 / 0.3 / 0.6 / 1.0 rad/s (5 turns), CW and CCW | `$A turn` (multiplier), `$A circle` (spin centre), `$A gyroscale` |
| 4.4 arcs | radius 1.52 m: `$T vel --name arc152_045 --pre 5 --seq "0.45,0.296:45 0,0:5" --ros --test 4.4 --rep N`. Also 3 m (`0.45,0.15`) and 0.7 m/s. | `$A turn`, `$A circle` (driven radius vs v/ω) |
| 4.6 square | web UI or a scripted square (tool to add); 5 CW + 5 CCW | RTK return error |
| 5.1 stops | `$T vel --name stop07 --seq "0.7,0:6 0,0:4" --ros --test 5.1 --rep N` at 0.5 / 0.7 / 1.0 m/s | `$A distance` of the stop segment (tool extension to add) |

## Sessions 3–4 (ROS 2 command path, odometry, MPPI model)

Start `actuator_node` (RUNBOOK §5). Record each run with `ground/bag.sh <test> <rep>` while the test drives through `/cmd_vel`.

**Still to write before these sessions:**
- `odom_quality.py`: tests 7.1–7.4;
- `random_cmd.py` and `fopdt_fit.py`: tests 8.1–8.2;
- stop distance from GNSS for 6.2.

Decisions F1–F5 in the plan are also needed first.
