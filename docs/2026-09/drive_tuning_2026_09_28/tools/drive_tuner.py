#!/usr/bin/env python3
"""Field tuning tool for the Teensy v2 motor bridge (docs/drive_tuning_2026_09_28/STRATEGY.md).

Owns the Teensy serial port, runs scripted tests, and logs everything per run:
  runs/<stamp>_<name>/teensy.csv   X telemetry (50 Hz) + host time + active command
  runs/<stamp>_<name>/lines.log    every raw Teensy line with host time
  runs/<stamp>_<name>/imu.csv      /imu/data   (only with --ros)
  runs/<stamp>_<name>/gnss.csv     /gnss       (only with --ros)
  runs/<stamp>_<name>/gga.csv      /nmea GGA fix quality (4 = RTK FIXED)
  runs/<stamp>_<name>/meta.json    test arguments, firmware/battery info, --test/--rep/--note
  runs/journal.csv                 one row per run: time, test ID, repeat, run dir, stopped?, note
With GT_SESSION set (tools/ground/session_start.sh), runs go to $GT_SESSION/runs.

Safety: stop actuator_node first (one writer on the port). SPACE or Ctrl-C = stop (sends S).
The Teensy's 300 ms watchdog stops the motors if this tool dies; commands are re-sent every 50 ms.

Examples (run on the Jetson):
  ./drive_tuner.py preflight
  ./drive_tuner.py cmd "PW B kS 0.12"                        # any raw command(s), waits for replies
  ./drive_tuner.py ramp  --name qs_fwd --max 7 --rate 0.5          # both tracks, volts, forward
  ./drive_tuner.py ramp  --name qs_spin_ccw --max 6 --spin ccw     # L=-V, R=+V
  ./drive_tuner.py steps --name dyn_fwd --levels 3,5,7 --hold 2.5
  ./drive_tuner.py steps --name dyn_fwd --levels 2,4,6 --hold 3 --chain      # 2 V steps, smooth ramp-down at the end (--rampdown V/s)
  ./drive_tuner.py vel   --name step_03_07 --seq "0.3,0:3 0.7,0:3 0,0:2" --ros
  ./drive_tuner.py vel   --name spin_06 --seq "0,0.6:12 0,0:2" --ros
  ./drive_tuner.py vel   --name arc_r2 --seq "0.5,0.25:15 0,0:2" --ros   # R = v/w = 2 m
"""
import argparse
import csv
import glob
import json
import math
import os
import re
import select
import shutil
import sys
import termios
import threading
import time
import tty

import serial

DEFAULT_PORT = (glob.glob('/dev/serial/by-id/usb-Teensyduino_USB_Serial_*') or ['/dev/ttyACM0'])[0]
TRACK_W = 0.7366          # m, physical track centre spacing
M_PER_REV = 0.01994       # m of track per motor revolution (nominal; calibrated in step E)

X_FIELDS = ['teensy_us',
            'L_vel_rpm', 'L_pos_rot', 'L_s2_rx_us', 'L_applied', 'L_current_A', 'L_bus_V', 'L_temp_C',
            'R_vel_rpm', 'R_pos_rot', 'R_s2_rx_us', 'R_applied', 'R_current_A', 'R_bus_V', 'R_temp_C',
            'sp_L', 'sp_R', 'sp_tx_us_L', 'sp_tx_us_R', 'mode']


class Teensy:
    def __init__(self, port, logdir=None):
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        self.lock = threading.Lock()
        self.lines = []            # (host_t, line) recent, for waiting on replies
        self.lines_lock = threading.Lock()
        self.cmd_active = None     # string re-sent every 50 ms (watchdog keepalive)
        self.cmd_label = ''
        self.running = True
        self.logdir = logdir
        self.xw = self.logf = None
        if logdir:
            os.makedirs(logdir, exist_ok=True)
            self.logf = open(os.path.join(logdir, 'lines.log'), 'w')
            self.xf = open(os.path.join(logdir, 'teensy.csv'), 'w', newline='')
            self.xw = csv.writer(self.xf)
            self.xw.writerow(['host_t'] + X_FIELDS + ['cmd'])
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._keepalive, daemon=True).start()

    def write(self, line):
        # REVIEW 2026-09-28: a SerialException here used to propagate out of stop()/end_session(), skipping
        # the terminal restore and the log close. A dead port now just marks the link down; the Teensy's
        # 300 ms watchdog stops the motors.
        try:
            with self.lock:
                self.ser.write((line + '\n').encode('ascii'))
        except (serial.SerialException, OSError) as e:
            if self.running:
                print(f'\n!! serial write failed: {e}  (motors stop via the Teensy 300 ms watchdog)')
            self.running = False
            return
        if self.logf and not self.logf.closed:
            self.logf.write(f'{time.time():.6f} > {line}\n')

    def _reader(self):
        buf = ''
        while self.running:
            try:
                chunk = self.ser.read(512).decode('ascii', errors='replace')
            except (serial.SerialException, OSError) as e:
                if self.running:
                    print(f'\n!! serial error: {e}  (Teensy USB dropped? motors stop via watchdog)')
                self.running = False
                return
            if not chunk:
                continue
            buf += chunk
            while '\n' in buf:
                line, buf = buf.split('\n', 1)
                line = line.strip()
                t = time.time()
                if not line:
                    continue
                if line.startswith('X ') and self.xw and not self.xf.closed:
                    parts = line.split()
                    # X us L 7f R 7f SP 4f mode
                    try:
                        vals = [parts[1]] + parts[3:10] + parts[11:18] + parts[19:23] + [parts[23]]
                        self.xw.writerow([f'{t:.6f}'] + vals + [self.cmd_label])
                    except IndexError:
                        pass
                    continue
                if line.startswith('E L'):
                    continue
                if self.logf and not self.logf.closed:
                    self.logf.write(f'{t:.6f} < {line}\n')
                with self.lines_lock:
                    self.lines.append((t, line))
                    self.lines = self.lines[-500:]
                if not line.startswith(('OK L=', 'OK UL=', 'OK UVL=', 'OK S')):
                    print(f'  < {line}')

    def _keepalive(self):
        while self.running:
            c = self.cmd_active
            if c:
                self.write(c)
            time.sleep(0.05)

    def set_cmd(self, line, label=None):
        self.cmd_active = line
        self.cmd_label = label if label is not None else (line or '')
        if line:
            self.write(line)

    def stop(self):
        self.cmd_active = None
        self.cmd_label = 'S'
        for _ in range(3):
            self.write('S')
            time.sleep(0.02)

    def wait_for(self, pattern, timeout=1.5, since=None):
        rx = re.compile(pattern)
        since = since or (time.time() - 0.01)
        t_end = time.time() + timeout
        found = []
        while time.time() < t_end:
            with self.lines_lock:
                found = [l for (t, l) in self.lines if t >= since and rx.search(l)]
            if found:
                return found
            time.sleep(0.02)
        return found

    def close(self):
        self.stop()
        time.sleep(0.1)
        self.running = False
        time.sleep(0.1)            # let the reader thread leave its read() before the files close
        if self.logf:
            self.logf.close()
            self.xf.close()
        try:
            self.ser.close()
        except (serial.SerialException, OSError):
            pass


class StopKey:
    """SPACE (or q) in the terminal sets .hit; restores the terminal on exit."""
    def __init__(self):
        self.hit = False
        self.fd = sys.stdin.fileno() if sys.stdin.isatty() else None
        if self.fd is None:
            print('!! stdin is not a terminal: SPACE-stop disabled, use Ctrl-C (run over `ssh -t`)')
        if self.fd is not None:
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            threading.Thread(target=self._watch, daemon=True).start()

    def _watch(self):
        while not self.hit:
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if r and sys.stdin.read(1) in (' ', 'q', 's'):
                self.hit = True
                print('\n!! STOP key')

    def restore(self):
        if self.fd is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


class RosLogger:
    """Optional: logs /imu/data and /gnss to CSV with host time (same clock as teensy.csv)."""
    def __init__(self, logdir):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu, NavSatFix
        rclpy.init()
        self.rclpy = rclpy
        self.node = Node('drive_tuner_logger')
        self.fi = open(os.path.join(logdir, 'imu.csv'), 'w', newline='')
        self.fg = open(os.path.join(logdir, 'gnss.csv'), 'w', newline='')
        self.wi, self.wg = csv.writer(self.fi), csv.writer(self.fg)
        self.wi.writerow(['host_t', 'stamp', 'gx', 'gy', 'gz', 'ax', 'ay', 'az', 'qx', 'qy', 'qz', 'qw'])
        self.wg.writerow(['host_t', 'stamp', 'lat', 'lon', 'alt', 'status', 'cov_e', 'cov_n'])
        self.n = {'imu': 0, 'gnss': 0, 'gga': 0}
        self.q_last = None
        self.closed = False
        self.node.create_subscription(Imu, '/imu/data', self._imu, qos_profile_sensor_data)
        self.node.create_subscription(NavSatFix, '/gnss', self._gnss, qos_profile_sensor_data)
        # RTK state: GGA fix quality (4 = RTK FIXED, 5 = FLOAT) is unambiguous; NavSatStatus is not
        try:
            from nmea_msgs.msg import Sentence
            self.fq = open(os.path.join(logdir, 'gga.csv'), 'w', newline='')
            self.wq = csv.writer(self.fq)
            self.wq.writerow(['host_t', 'quality', 'nsat', 'hdop'])
            self.node.create_subscription(Sentence, '/nmea', self._nmea, 50)
        except ImportError:
            self.fq = None
        self.th = threading.Thread(target=self._spin, daemon=True)
        self.th.start()
        # REVIEW 2026-09-28: fail loudly in the field, not at analysis time, if no sensor data arrives
        # (sensors.launch.py not running, or this shell not on CycloneDDS -- see CLAUDE.md "DDS Config").
        t_end = time.time() + 4.0
        while time.time() < t_end and (self.n['imu'] == 0 or self.n['gnss'] == 0):
            time.sleep(0.1)
        print(f'  ros: imu={self.n["imu"]} gnss={self.n["gnss"]} gga={self.n["gga"]} msgs in first {4.0 - max(0.0, t_end - time.time()):.1f}s'
              f'  GGA quality={self.q_last} (4 = RTK FIXED)')
        if self.n['imu'] == 0 or self.n['gnss'] == 0:
            print('  !! no /imu/data or /gnss: is sensors.launch.py running and is RMW_IMPLEMENTATION='
                  'rmw_cyclonedds_cpp + CYCLONEDDS_URI exported in THIS shell?')

    def _spin(self):
        try:
            self.rclpy.spin(self.node)
        except Exception:
            pass

    def _imu(self, m):
        if self.closed:
            return
        self.n['imu'] += 1
        s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        a, g, q = m.linear_acceleration, m.angular_velocity, m.orientation
        self.wi.writerow([f'{time.time():.6f}', f'{s:.6f}', g.x, g.y, g.z, a.x, a.y, a.z, q.x, q.y, q.z, q.w])

    def _gnss(self, m):
        if self.closed:
            return
        self.n['gnss'] += 1
        s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        c = m.position_covariance
        self.wg.writerow([f'{time.time():.6f}', f'{s:.6f}', f'{m.latitude:.9f}', f'{m.longitude:.9f}',
                          m.altitude, m.status.status, c[0], c[4]])

    def _nmea(self, m):
        if self.closed:
            return
        f = m.sentence.split(',')
        if len(f) > 8 and f[0].endswith('GGA'):
            self.n['gga'] += 1
            self.q_last = f[6]
            self.wq.writerow([f'{time.time():.6f}', f[6], f[7], f[8]])

    def close(self):
        self.closed = True
        print(f'  ros: logged imu={self.n["imu"]} gnss={self.n["gnss"]} gga={self.n["gga"]}  last GGA quality={self.q_last}')
        try:
            self.node.destroy_node()
            self.rclpy.shutdown()
        except Exception:
            pass
        time.sleep(0.05)
        self.fi.close()
        self.fg.close()
        if self.fq:
            self.fq.close()


# ---------------------------------------------------------------- helpers
def vw_to_rpm(v, w, m_per_rev=M_PER_REV, track=TRACK_W, mult=1.0):
    """Nominal diff-drive inverse (no skid correction by default: we are measuring it)."""
    b = track * mult
    vl, vr = v - w * b / 2.0, v + w * b / 2.0
    k = 60.0 / m_per_rev
    return vl * k, vr * k


def run_profile(t, key, profile, dt=0.05):
    """profile: list of (duration_s, command_line, label). Returns False if stopped."""
    for dur, line, label in profile:
        t.set_cmd(line, label)
        t_end = time.time() + dur
        while time.time() < t_end:
            if key.hit or not t.running:
                t.stop()
                return False
            time.sleep(dt)
    t.stop()
    return True


def volt_line(vl, vr):
    return f'UVL{vl:.3f} UVR{vr:.3f}'


def soft_stop(prof, vl, vr, rate, dt=0.05):
    """Append a linear voltage ramp from (vl, vr) down to 0 at `rate` V/s (larger side), so a test ends without
    the hard S brake. Labelled 'softstop' (not fitted by analyze.py ff). rate <= 0 appends nothing."""
    if rate <= 0 or max(abs(vl), abs(vr)) < 1e-6:
        return
    n = max(1, int(math.ceil(max(abs(vl), abs(vr)) / rate / dt)))
    for i in range(1, n + 1):
        f = max(0.0, 1.0 - i / n)
        prof.append((dt, volt_line(vl * f, vr * f), 'softstop'))


def new_run(args, name):
    stamp = time.strftime('%Y%m%d_%H%M%S')
    d = os.path.join(args.outdir, f'{stamp}_{name}')
    os.makedirs(d, exist_ok=True)
    return d


def write_meta(d, args, extra=None):
    meta = {k: v for k, v in vars(args).items() if k != 'func'}
    meta['start_host_t'] = time.time()
    meta.update(extra or {})
    with open(os.path.join(d, 'meta.json'), 'w') as f:
        json.dump(meta, f, indent=2)


def check_port_free(port):
    real = os.path.realpath(port)
    if not shutil.which('fuser'):
        print('!! fuser not found (apt install psmisc): cannot check that actuator_node has released the port')
        return
    busy = os.popen(f'fuser {real} 2>/dev/null').read().strip()
    if busy:
        sys.exit(f'{real} is in use by PID(s) {busy}. Stop actuator_node / webui launch first.')


def session(args, name, extra_meta=None, telemetry=True):
    check_port_free(args.port)
    d = new_run(args, name) if name else None
    t = Teensy(args.port, d)
    if d and getattr(args, 'ros', False) and os.environ.get('RMW_IMPLEMENTATION') != 'rmw_cyclonedds_cpp':
        print('!! RMW_IMPLEMENTATION is not rmw_cyclonedds_cpp in this shell: the sensors (launched on '
              'CycloneDDS) may be invisible. export it + CYCLONEDDS_URI (CLAUDE.md "DDS Config").')
    ros = RosLogger(d) if (d and getattr(args, 'ros', False)) else None
    key = StopKey()
    if telemetry:
        t.write('X1')
    if getattr(args, 'cap', None) is not None:
        s0 = time.time()
        t.write(f'MD{args.cap:.3f}')
        t.cap_set = True
        if not t.wait_for(r'^OK MD=', 1.0, s0):
            t.close()
            key.restore()
            sys.exit('!! no "OK MD=" reply: is firmware v2 flashed? (v1 treats MD as the M slew command)')
    if getattr(args, 'm', None) is not None:
        s0 = time.time()
        t.write(f'M{args.m}')
        t.m_set = True
        if not t.wait_for(r'^OK M=', 1.0, s0):
            t.close()
            key.restore()
            sys.exit('!! no "OK M=" reply to the speed-ramp command')
    t.write('D')
    diag = t.wait_for(r'^DIAG', 1.0)
    if d:
        write_meta(d, args, dict(extra_meta or {}, diag_start=diag[-1] if diag else None))
    return t, ros, key, d


def end_session(t, ros, key, d):
    try:
        t.write('D')
        diag = t.wait_for(r'^DIAG', 1.0)
        t.write('X0')
        if getattr(t, 'cap_set', False):
            t.write('MD0.30')     # back to the firmware default cap (it otherwise persists until reboot)
        if getattr(t, 'm_set', False):
            t.write('M100')       # back to the firmware default speed ramp (100 RPM per 20 ms)
        t.close()
        if ros:
            ros.close()
    finally:
        key.restore()
    if d:
        with open(os.path.join(d, 'meta.json')) as f:
            meta = json.load(f)
        meta['diag_end'] = diag[-1] if diag else None
        meta['end_host_t'] = time.time()
        meta['stopped_by_key'] = bool(key.hit)
        with open(os.path.join(d, 'meta.json'), 'w') as f:
            json.dump(meta, f, indent=2)
        append_journal(d, meta)
        print(f'logged: {d}')


JOURNAL_FIELDS = ['start_local', 'test', 'rep', 'run', 'dir', 'duration_s', 'stopped_by_key', 'note']


def append_journal(d, meta):
    """One row per logged run in <outdir>/journal.csv: which test and repeat each run directory belongs to."""
    path = os.path.join(os.path.dirname(d), 'journal.csv')
    new = not os.path.exists(path)
    with open(path, 'a', newline='') as f:
        w = csv.writer(f)
        if new:
            w.writerow(JOURNAL_FIELDS)
        w.writerow([time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(meta['start_host_t'])),
                    meta.get('test') or '', meta.get('rep') or '', meta.get('name') or os.path.basename(d),
                    os.path.basename(d), f"{meta['end_host_t'] - meta['start_host_t']:.1f}",
                    meta['stopped_by_key'], meta.get('note') or ''])


# ---------------------------------------------------------------- subcommands
TUNING_PARAMS = ['kP', 'kI', 'kD', 'kV', 'kIZone', 'kS', 'kA', 'kIMaxAccum', 'kDFilter',
                 'outputMin', 'outputMax', 'idleMode', 'inverted', 'feedbackSensor',
                 'smartStallA', 'smartFreeA', 'smartLimitRpm', 'voltCompMode', 'nominalVoltage',
                 'closedLoopRamp', 'openLoopRamp', 'hallSamplePeriod', 'hallAvgDepth',
                 'posConvFactor', 'velConvFactor', 'status0Period', 'status1Period', 'status2Period']


def cmd_preflight(args):
    t, ros, key, d = session(args, 'preflight', telemetry=False)
    try:
        for c, pat in [('FV', r'^FV [LR] '), ('CF B', r'^OK CF'), ('PT', r'^PT ')]:
            s = time.time()
            t.write(c)
            t.wait_for(pat, 2.0, s)
            time.sleep(0.3)
        print('--- read-back (PR); TIMEOUT/UNSUPPORTED means reads are not answered on this firmware')
        for p in TUNING_PARAMS:
            s = time.time()
            t.write(f'PR B {p}')
            t.wait_for(r'^PRD R ', 0.6, s)
        time.sleep(1.0)   # let STATUS_1 fault lines arrive
    finally:
        end_session(t, ros, key, d)


def cmd_cmd(args):
    t, ros, key, d = session(args, 'cmd' if args.log else None, telemetry=False)
    try:
        for c in args.lines:
            s = time.time()
            t.write(c)
            t.wait_for(r'^(PWR|PRD|FV|OK|ERR)', 1.5, s)
            time.sleep(0.4)   # collect trailing PWR lines
    finally:
        end_session(t, ros, key, d)


def cmd_ramp(args):
    sign = -1.0 if args.dir == 'rev' else 1.0
    n = int(round(args.max / args.rate / 0.05))
    prof = [(1.0, volt_line(0, 0), 'hold0')]
    last = (0.0, 0.0)
    for i in range(1, n + 1):
        v = sign * i * 0.05 * args.rate
        if args.spin:
            s = 1.0 if args.spin == 'ccw' else -1.0
            last = (-s * abs(v), s * abs(v))
        else:
            last = (v, v)
        prof.append((0.05, volt_line(*last), f'ramp {v:.3f}'))
    soft_stop(prof, last[0], last[1], args.rampdown)
    prof.append((1.0, volt_line(0, 0), 'hold0'))
    t, ros, key, d = session(args, args.name, {'type': 'quasistatic_ramp'})
    try:
        print(f'ramp to {args.max} V at {args.rate} V/s ({"spin " + args.spin if args.spin else args.dir}); SPACE = stop')
        run_profile(t, key, prof)
    finally:
        end_session(t, ros, key, d)


def cmd_steps(args):
    sign = -1.0 if args.dir == 'rev' else 1.0
    prof = []
    levels = [float(x) for x in args.levels.split(',')]
    if args.chain:
        prof.append((1.0, volt_line(0, 0), 'rest'))
    last = (0.0, 0.0)
    for k, lv in enumerate(levels):
        v = sign * lv
        if args.spin:
            s = 1.0 if args.spin == 'ccw' else -1.0
            last = (-s * lv, s * lv)
        else:
            last = (v, v)
        if not args.chain:
            prof.append((1.0, volt_line(0, 0), 'rest'))
        prof.append((args.hold, volt_line(*last), f'step {v:.2f}'))
        if not args.chain:                     # each level on its own: smooth ramp-down, then the S rest
            soft_stop(prof, last[0], last[1], args.rampdown)
            prof.append((args.rest, 'S', 'stop'))
    if args.chain:                             # levels back to back (small steps), one smooth ramp-down at the end
        soft_stop(prof, last[0], last[1], args.rampdown)
        prof.append((args.rest, 'S', 'stop'))
    t, ros, key, d = session(args, args.name, {'type': 'dynamic_steps'})
    try:
        print(f'voltage steps {args.levels} V, hold {args.hold}s; SPACE = stop')
        run_profile(t, key, prof)
    finally:
        end_session(t, ros, key, d)


def cmd_vel(args):
    """--seq "v,w:secs v,w:secs ..." in m/s and rad/s (nominal geometry, multiplier 1 unless --mult).
    A segment "S:secs" sends the braked stop (Teensy S) and keeps logging for secs."""
    # stationary hold first: analyze.py estimates the gyro z bias from it (Xsens does not remove its bias
    # estimate from the published rate of turn, L2 finding 12 / L2-S38) and it holds the tracks at 0 RPM.
    prof = [(args.pre, 'L0 R0', 'v=0.0 w=0.0')] if args.pre > 0 else []
    for seg in args.seq.split():
        vw, secs = seg.split(':')
        if vw.upper() == 'S':
            # braked stop: the Teensy S command (duty 0 -> SPARK Brake idle), kept logging for secs
            prof.append((float(secs), 'S', 'stop'))
            continue
        v, w = (float(x) for x in vw.split(','))
        l, r = vw_to_rpm(v, w, args.m_per_rev, args.track, args.mult)
        prof.append((float(secs), f'L{l:.0f} R{r:.0f}', f'v={v} w={w}'))
    t, ros, key, d = session(args, args.name, {'type': 'velocity_sequence',
                                               'segments': [(p[0], p[1], p[2]) for p in prof]})
    try:
        print(f'velocity sequence: {args.seq}; SPACE = stop')
        run_profile(t, key, prof)
    finally:
        end_session(t, ros, key, d)


def cmd_listen(args):
    """Just log telemetry (e.g. while pushing the robot by hand, or during a manual test)."""
    t, ros, key, d = session(args, args.name)
    try:
        print(f'logging for {args.secs}s; SPACE = end')
        t_end = time.time() + args.secs
        while time.time() < t_end and not key.hit and t.running:
            time.sleep(0.1)
    finally:
        end_session(t, ros, key, d)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=DEFAULT_PORT)
    # a ground-test session (tools/ground/session_start.sh) exports GT_SESSION; runs then go into it
    default_out = (os.path.join(os.environ['GT_SESSION'], 'runs') if os.environ.get('GT_SESSION')
                   else os.path.expanduser('~/drive_tuning_2026_09_28/runs'))
    ap.add_argument('--outdir', default=default_out)
    sub = ap.add_subparsers(required=True)

    def add(name, fn, **kw):
        p = sub.add_parser(name, **kw)
        p.set_defaults(func=fn)
        p.add_argument('--ros', action='store_true', help='also log /imu/data and /gnss')
        p.add_argument('--cap', type=float, default=None, help='duty cap MD (voltage cap = 12*cap)')
        p.add_argument('--test', default=None, help='test ID from GROUND_TEST_PLAN.md, e.g. 4.3 (goes to meta + journal)')
        p.add_argument('--rep', type=int, default=None, help='repeat number (1-3 fit, 4-5 validate)')
        p.add_argument('--note', default=None, help='free text: surface, conditions, anything unusual')
        return p

    add('preflight', cmd_preflight)
    p = add('cmd', cmd_cmd)
    p.add_argument('lines', nargs='+')
    p.add_argument('--log', action='store_true')
    for nm, fn in (('ramp', cmd_ramp), ('steps', cmd_steps)):
        p = add(nm, fn)
        p.add_argument('--name', required=True)
        p.add_argument('--dir', choices=['fwd', 'rev'], default='fwd')
        p.add_argument('--spin', choices=['cw', 'ccw'], default=None)
        p.add_argument('--rampdown', type=float, default=1.5,
                       help='V/s smooth ramp-down before the final stop (0 = old behaviour, hard S from speed)')
        if nm == 'ramp':
            p.add_argument('--max', type=float, default=7.0)
            p.add_argument('--rate', type=float, default=0.5)
        else:
            p.add_argument('--levels', default='3,5,7')
            p.add_argument('--hold', type=float, default=2.5)
            p.add_argument('--rest', type=float, default=1.5)
            p.add_argument('--chain', action='store_true',
                           help='run the levels back to back (e.g. 2,4,6 = steps of 2 V), no return to zero between them')
        p.set_defaults(cap=0.6)
    p = add('vel', cmd_vel)
    p.add_argument('--name', required=True)
    p.add_argument('--seq', required=True)
    p.add_argument('--m-per-rev', dest='m_per_rev', type=float, default=M_PER_REV)
    p.add_argument('--track', type=float, default=TRACK_W)
    p.add_argument('--mult', type=float, default=1.0)
    p.add_argument('--pre', type=float, default=3.0, help='stationary hold (s) before the sequence, for the gyro bias')
    p.add_argument('--m', type=int, default=None,
                   help='Teensy speed ramp M (RPM per 20 ms) for this run; 20 = about 0.33 m/s^2 per track, like actuator_node. Restored to 100 at the end')
    p = add('listen', cmd_listen)
    p.add_argument('--name', required=True)
    p.add_argument('--secs', type=float, default=60)
    args = ap.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
