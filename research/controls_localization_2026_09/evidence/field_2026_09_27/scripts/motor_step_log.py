#!/usr/bin/env python3
"""Read-only motor-loop logger + analyzer (field test companion to 01_motor_pid_and_ramp.md).

NEVER publishes, never touches serial or parameters.

  record : python3 motor_step_log.py record out.csv
           subscribes /avros/wheel_debug (50 Hz, actuator_node) + /imu/data, writes one CSV row
           per wheel_debug message with a receive timestamp and the latest IMU yaw rate.
  analyze: python3 motor_step_log.py analyze out.csv [--m-per-rev 0.01994]

Metrics (per constant-command segment, velocity from ENCODER POSITION, not the 185 ms-lagged
reported RPM): delivery at 0.5/1/2 s and at segment end, overshoot, steady-state CoV,
L/R delivery mismatch, time-to-95%, reported/position velocity ratio.
Per stop (command drops to 0): stopping distance, reverse travel after stop, drift in the
following hold. Per turning segment: IMU yaw rate vs commanded w_target.
"""
import csv
import sys
import time

import numpy as np

LABELS = ['L_cmd_rpm', 'R_cmd_rpm', 'L_meas_rpm', 'R_meas_rpm', 'L_pos_rev', 'R_pos_rev',
          'v_target', 'w_target', 'v_slewed', 'w_slewed', 'v_after_imu', 'w_after_imu',
          'yaw', 'yaw_rate', 'heading_locked', 'estop']


def record(path):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Float32MultiArray

    class Rec(Node):
        def __init__(self):
            super().__init__('motor_step_log')
            self.f = open(path, 'w', newline='')
            self.w = csv.writer(self.f)
            self.w.writerow(['t'] + LABELS + ['imu_wz'])
            self.wz = float('nan')
            self.create_subscription(Imu, '/imu/data', self.on_imu, 50)
            self.create_subscription(Float32MultiArray, '/avros/wheel_debug', self.on_dbg, 100)

        def on_imu(self, m):
            self.wz = m.angular_velocity.z

        def on_dbg(self, m):
            self.w.writerow([f'{time.time():.4f}'] + [f'{x:.4f}' for x in m.data] + [f'{self.wz:.5f}'])

    rclpy.init()
    n = Rec()
    print(f'recording to {path} (Ctrl-C to stop)')
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.f.close()


def pos_rpm(t, p, win=0.10):
    """Centered-difference velocity from cumulative position (rev) -> RPM."""
    out = np.full_like(p, np.nan)
    for i in range(len(t)):
        a = np.searchsorted(t, t[i] - win / 2)
        b = min(np.searchsorted(t, t[i] + win / 2), len(t) - 1)
        if t[b] > t[a]:
            out[i] = (p[b] - p[a]) / (t[b] - t[a]) * 60.0
    return out


def mean_at(t, x, t0, a, b):
    m = (t >= t0 + a) & (t < t0 + b)
    return np.nanmean(x[m]) if m.any() else np.nan


def analyze(path, m_per_rev=0.01994):
    r = list(csv.DictReader(open(path)))
    t = np.array([float(x['t']) for x in r]); t -= t[0]
    col = lambda k: np.array([float(x[k]) for x in r])
    c = {s: col(s + '_cmd_rpm') for s in 'LR'}
    meas = {s: col(s + '_meas_rpm') for s in 'LR'}
    v = {s: pos_rpm(t, col(s + '_pos_rev')) for s in 'LR'}
    pos = {s: col(s + '_pos_rev') for s in 'LR'}
    wz, wt = col('imu_wz'), col('w_target')

    # constant-command segments (both wheels within 2 RPM for >= 1.5 s)
    i, segs = 0, []
    while i < len(t):
        j = i
        while j + 1 < len(t) and abs(c['L'][j + 1] - c['L'][i]) <= 2 and abs(c['R'][j + 1] - c['R'][i]) <= 2:
            j += 1
        if t[j] - t[i] >= 1.5:
            segs.append((i, j))
        i = j + 1

    print('seg  t0     dur  | side cmd   d0.5  d1    d2    dEnd  ovs%  t95   CoV%  rep/pos | L/R mism%  imu_w/w_cmd')
    for i, j in segs:
        t0, dur = t[i], t[j] - t[i]
        res = {}
        for s in 'LR':
            cmd = c[s][i]
            if abs(cmd) < 20:
                continue
            d = lambda a, b: 100 * mean_at(t, v[s], t0, a, b) / cmd
            seg = (t >= t0) & (t <= t[j])
            ratio = v[s][seg] / cmd
            ovs = 100 * (np.nanmax(ratio) - 1)
            hit = np.where(ratio >= 0.95)[0]
            t95 = t[seg][hit[0]] - t0 if len(hit) else np.nan
            tail = seg & (t >= t[j] - min(1.0, dur / 2))
            cov = 100 * np.nanstd(v[s][tail]) / abs(np.nanmean(v[s][tail]))
            rep = np.nanmean(meas[s][tail]) / np.nanmean(v[s][tail])
            res[s] = d(dur - 0.5, dur)
            print(f'{t0:6.1f} {dur:5.1f} | {s} {cmd:6.0f} {d(0.25, 0.75):5.0f} {d(0.75, 1.25):5.0f} '
                  f'{d(1.75, 2.25):5.0f} {res[s]:5.0f} {ovs:5.1f} {t95:5.2f} {cov:5.1f}  {rep:5.2f}', end='')
            if s == 'R':
                mm = res.get('L', np.nan) - res['R']
                wr = mean_at(t, wz, t0, dur - 1.0, dur) / wt[i] if abs(wt[i]) > 0.02 else np.nan
                print(f' | {mm:6.1f}     {wr:5.2f}')
            else:
                print()

    # stops: the requested v drops to ~0 (cmd_vel 0 / stale / estop) while the robot was moving
    vt, est = col('v_target'), col('estop')
    print('\nstop t  kind   side v_before_mps stop_dist_m reverse_m drift_next_5s_m')
    for k in range(1, len(t)):
        start = (abs(vt[k]) < 1e-3 and abs(vt[k - 1]) > 0.03) or (est[k] > 0.5 and est[k - 1] < 0.5)
        if not start:
            continue
        kind = 'estop' if est[k] > 0.5 else 'cmd0'
        e = min(np.searchsorted(t, t[k] + 2.5), len(t) - 1)
        h = min(np.searchsorted(t, t[k] + 7.5), len(t) - 1)
        for s in 'LR':
            vb = v[s][k - 1] * m_per_rev / 60
            sgn = 1.0 if vb >= 0 else -1.0
            tr = sgn * (pos[s][k:e + 1] - pos[s][k])      # travel in the prior direction (rev)
            dist, back = tr.max(), tr.max() - tr[-1]
            drift = pos[s][h] - pos[s][e]
            print(f'{t[k]:6.1f} {kind:5s}  {s}  {vb:8.3f}    {dist * m_per_rev:8.3f}  {back * m_per_rev:8.3f}  '
                  f'{drift * m_per_rev:10.4f}')


if __name__ == '__main__':
    if len(sys.argv) >= 3 and sys.argv[1] == 'record':
        record(sys.argv[2])
    elif len(sys.argv) >= 3 and sys.argv[1] == 'analyze':
        mpr = float(sys.argv[sys.argv.index('--m-per-rev') + 1]) if '--m-per-rev' in sys.argv else 0.01994
        analyze(sys.argv[2], mpr)
    else:
        print(__doc__)
