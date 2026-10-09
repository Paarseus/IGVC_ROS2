#!/usr/bin/env python3
"""Offline RTK / global-localization analysis for field tests (READ-ONLY).

Reads a rosbag2 directory; never creates a ROS node, never publishes.
Run on the Jetson (needs xsens_mti_ros2_driver msgs sourced for /status):

  source /opt/ros/humble/setup.bash && source ~/IGVC_ROS2/install/setup.bash
  python3 rtk_field_analyze.py <bag_dir> <mode> [--datum LAT LON] [--mark LAT LON]
                               [--lever 0.74] [--t0 SEC] [--t1 SEC]

Modes (see 02_rtk_and_global_localization.md, tests A-E):
  static  : time-to-FIXED, per-RTK-state scatter vs reported sigma, state
            transitions + position step at each, /odometry/global + map->odom span.
  stops   : auto-detects stationary segments (>= 6 s); per segment the FIXED-only
            mean of /filter/positionlla (IMU point = truth), /gnss (antenna) and
            /odometry/global (EKF base_link), heading, antenna offset in the
            vehicle frame, and EKF error vs truth. Used for tests B and E.
  line    : auto-detects straight segments (|wz|<0.03, v>0.2, >= 5 s); per segment
            GNSS course vs IMU yaw (heading offset), map-vs-ENU rotation of
            /odometry/gps, along/cross-track error of /odometry/global vs truth.
            Fits along-track error = a - v*tau over segments (a = lever residual,
            tau = effective GPS lag).                                  (test C)
  spin    : in-place rotation segments; Kasa circle fits of antenna, IMU point and
            /odometry/global -> ICR location in body frame, lever arm.   (test D)

Truth = /filter/positionlla (MTi fused, lever-arm-corrected, IMU point) only
while /status rtk_status == 2. Positions are local ENU about --datum (defaults
to the navsat datum below); map frame should equal this ENU if navsat is right.
"""
import argparse
import math
import os
import sys
from bisect import bisect_left

import numpy as np

DATUM = (34.05930007, -117.82186044)
TOPICS = ['/gnss', '/filter/positionlla', '/status', '/imu/data', '/odometry/global',
          '/odometry/gps', '/odometry/filtered', '/tf', '/rtcm', '/nmea']


# ---------------------------------------------------------------- bag reading
def read_bag(path, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    sid = 'mcap' if any(f.endswith('.mcap') for f in os.listdir(path)) else 'sqlite3'
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=sid),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = [t for t in topics if t in types]
    r.set_filter(rosbag2_py.StorageFilter(topics=want))
    cls = {t: get_message(types[t]) for t in want}
    out = {t: [] for t in topics}
    while r.has_next():
        t, data, trx = r.read_next()
        out[t].append((trx * 1e-9, deserialize_message(data, cls[t])))
    return out


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class ENU:
    """Local tangent-plane approx (WGS84 radii); < 1 mm error over a few 100 m."""
    def __init__(self, lat0, lon0):
        a, e2 = 6378137.0, 6.69437999014e-3
        s = math.sin(math.radians(lat0))
        self.M = a * (1 - e2) / (1 - e2 * s * s) ** 1.5
        self.N = a / math.sqrt(1 - e2 * s * s)
        self.lat0, self.lon0, self.c = lat0, lon0, math.cos(math.radians(lat0))

    def __call__(self, lat, lon):
        return (math.radians(lon - self.lon0) * self.N * self.c,
                math.radians(lat - self.lat0) * self.M)


# ---------------------------------------------------------------- series
def build(bag, enu):
    S = {}
    S['rtk'] = [(stamp(m) if hasattr(m, 'header') else trx, m.rtk_status) for trx, m in bag['/status']]
    S['rtk_t'] = [t for t, _ in S['rtk']]
    S['gnss'] = [(stamp(m), *enu(m.latitude, m.longitude), m.status.status,
                  math.sqrt(max(m.position_covariance[0], 0))) for _, m in bag['/gnss']]
    S['pos'] = [(stamp(m), *enu(m.vector.x, m.vector.y)) for _, m in bag['/filter/positionlla']]
    S['yaw'] = [(stamp(m), yaw_of(m.orientation), m.angular_velocity.z) for _, m in bag['/imu/data']]
    S['yaw_t'] = [x[0] for x in S['yaw']]
    for k, tp in (('glob', '/odometry/global'), ('gps', '/odometry/gps'), ('filt', '/odometry/filtered')):
        S[k] = [(stamp(m), m.pose.pose.position.x, m.pose.pose.position.y,
                 yaw_of(m.pose.pose.orientation), m.twist.twist.linear.x,
                 m.twist.twist.angular.z) for _, m in bag[tp]]
    S['m2o'] = [(stamp(tf), tf.transform.translation.x, tf.transform.translation.y,
                 yaw_of(tf.transform.rotation)) for _, msg in bag['/tf'] for tf in msg.transforms
                if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom']
    S['rtcm_t'] = [trx for trx, _ in bag['/rtcm']]
    return S


def rtk_at(S, t):
    i = bisect_left(S['rtk_t'], t)
    return S['rtk'][min(max(i - 1, 0), len(S['rtk']) - 1)][1] if S['rtk'] else -1


def interp(series, t, cols):
    ts = [x[0] for x in series]
    i = bisect_left(ts, t)
    if i == 0 or i >= len(ts):
        return None
    a, b = series[i - 1], series[i]
    w = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0
    return [a[c] + w * (b[c] - a[c]) for c in cols]


def yaw_at(S, t):
    i = min(bisect_left(S['yaw_t'], t), len(S['yaw']) - 1)
    return S['yaw'][i][1]


# ---------------------------------------------------------------- segments
def segments(S, pred, min_len):
    segs, start = [], None
    for x in S['filt']:
        if pred(x):
            start = x[0] if start is None else start
            end = x[0]
        elif start is not None:
            if end - start >= min_len:
                segs.append((start, end))
            start = None
    if start is not None and end - start >= min_len:
        segs.append((start, end))
    return segs


def fixed_pts(S, key, t0, t1):
    return np.array([(p[1], p[2]) for p in S[key] if t0 <= p[0] <= t1 and rtk_at(S, p[0]) == 2])


# ---------------------------------------------------------------- modes
def mode_static(S, a):
    g = [x for x in S['gnss'] if a.t0 <= x[0] <= a.t1]
    if not g:
        sys.exit('no /gnss in window')
    t_start = g[0][0]
    first = next((x[0] for x in g if x[3] == 2), None)
    print(f'duration {g[-1][0]-t_start:.0f} s; /gnss n={len(g)}; '
          f'time to first FIXED: {"never" if first is None else f"{first-t_start:.0f} s"}')
    for st, name in ((0, 'SPS/DGPS'), (1, 'FLOAT'), (2, 'FIXED')):
        p = np.array([(x[1], x[2]) for x in g if x[3] == st])
        if len(p) < 5:
            continue
        sd = p.std(axis=0)
        r = np.hypot(*(p - p.mean(axis=0)).T).max()
        rep = np.median([x[4] for x in g if x[3] == st])
        print(f'  {name:8s} n={len(p):5d} ({100*len(p)/len(g):4.1f}%) sigma E/N {100*sd[0]:.1f}/{100*sd[1]:.1f} cm, '
              f'max radius {100*r:.1f} cm, reported sigma median {100*rep:.1f} cm, '
              f'ratio measured/reported {np.hypot(*sd)/math.sqrt(2)/rep:.1f}')
    for u, v in zip(g, g[1:]):
        if u[3] != v[3]:
            print(f'  transition {u[3]}->{v[3]} at t+{v[0]-t_start:.1f} s, step {100*math.hypot(v[1]-u[1], v[2]-u[2]):.1f} cm')
    gaps = np.diff([t for t in S['rtcm_t'] if a.t0 <= t <= a.t1])
    if len(gaps):
        print(f'  /rtcm msgs {len(gaps)+1}, max gap {gaps.max():.1f} s, gaps>2s: {int((gaps > 2).sum())}')
    for k, name in (('glob', '/odometry/global'), ('m2o', 'map->odom')):
        p = np.array([(x[1], x[2]) for x in S[k] if a.t0 <= x[0] <= a.t1])
        if len(p) > 2:
            st = np.hypot(*np.diff(p, axis=0).T)
            print(f'  {name}: span {100*np.ptp(p[:,0]):.1f} x {100*np.ptp(p[:,1]):.1f} cm, max single step {100*st.max():.1f} cm')


def mode_stops(S, a):
    segs = segments(S, lambda x: abs(x[4]) < 0.02 and abs(x[5]) < 0.02 and a.t0 <= x[0] <= a.t1, 6.0)
    mark = ENU(*a.datum)(*a.mark) if a.mark else None
    print('seg  t0-t1 [s]   yaw[deg]  truth E,N [m]      ant_fwd/left [m]  EKF-truth fwd/left [cm]  truth-mark [cm]')
    for t0, t1 in segs:
        tr, an = fixed_pts(S, 'pos', t0, t1), fixed_pts(S, 'gnss', t0, t1)
        ek = np.array([(x[1], x[2]) for x in S['glob'] if t0 <= x[0] <= t1])
        if len(tr) < 10:
            print(f'{t0:.0f}-{t1:.0f}: not FIXED, skipped')
            continue
        y = yaw_at(S, (t0 + t1) / 2)
        R = np.array([[math.cos(y), math.sin(y)], [-math.sin(y), math.cos(y)]])  # world->body
        tm = tr.mean(axis=0)
        ant = R @ (an.mean(axis=0) - tm) if len(an) else [float('nan')] * 2
        err = R @ (ek.mean(axis=0) - tm) if len(ek) else [float('nan')] * 2
        mk = f'{100*math.hypot(tm[0]-mark[0], tm[1]-mark[1]):.1f}' if mark else '-'
        print(f'{t0:.0f}-{t1:.0f}  {math.degrees(y):8.2f}  {tm[0]:8.3f},{tm[1]:8.3f}  '
              f'{ant[0]:6.3f}/{ant[1]:6.3f}     {100*err[0]:7.1f}/{100*err[1]:6.1f}            {mk}  '
              f'(truth sd {100*tr.std(axis=0).max():.1f} cm)')


def mode_line(S, a):
    segs = segments(S, lambda x: abs(x[5]) < 0.03 and abs(x[4]) > 0.2 and a.t0 <= x[0] <= a.t1, 5.0)
    ekf_err, speeds = [], []
    for t0, t1 in segs:
        tr = fixed_pts(S, 'pos', t0, t1)
        gp = np.array([(x[1], x[2]) for x in S['gps'] if t0 <= x[0] <= t1])
        gn = np.array([(x[1], x[2]) for x in S['gnss'] if t0 <= x[0] <= t1])
        if len(tr) < 50:
            print(f'{t0:.0f}-{t1:.0f}: not FIXED, skipped')
            continue
        d = tr[-1] - tr[0]
        v = np.mean([x[4] for x in S['filt'] if t0 <= x[0] <= t1])
        course = math.atan2(d[1], d[0]) if v > 0 else math.atan2(-d[1], -d[0])
        yaws = [x[1] for x in S['yaw'] if t0 <= x[0] <= t1]
        yimu = math.atan2(np.mean(np.sin(yaws)), np.mean(np.cos(yaws)))
        rot = float('nan')
        if len(gp) > 4 and len(gn) > 4:  # map vs ENU rotation of the same fixes
            dg, dn = gp[-1] - gp[0], gn[-1] - gn[0]
            rot = math.degrees(wrap(math.atan2(dg[1], dg[0]) - math.atan2(dn[1], dn[0])))
        # along/cross-track error of EKF vs truth (truth interpolated to EKF stamps)
        c, s = math.cos(yimu), math.sin(yimu)
        al, cr = [], []
        for x in S['glob']:
            if t0 + 1 <= x[0] <= t1 - 1 and rtk_at(S, x[0]) == 2:
                p = interp(S['pos'], x[0], (1, 2))
                if p:
                    ex, ey = x[1] - p[0], x[2] - p[1]
                    al.append(c * ex + s * ey)
                    cr.append(-s * ex + c * ey)
        if al:
            ekf_err.append(np.mean(al)); speeds.append(v)
        print(f'{t0:.0f}-{t1:.0f} {math.hypot(*d):5.1f} m v={v:+.2f}  course-IMUyaw {math.degrees(wrap(course-yimu)):+.2f} deg  '
              f'map-vs-ENU rot {rot:+.2f} deg  EKF err along {100*np.mean(al) if al else float("nan"):+.1f} cm '
              f'cross {100*np.mean(cr) if cr else float("nan"):+.1f} cm')
    if len(speeds) >= 3:
        A = np.vstack([np.ones(len(speeds)), -np.array(speeds)]).T
        (lev, tau), *_ = np.linalg.lstsq(A, np.array(ekf_err), rcond=None)
        print(f'fit along-track EKF error = {lev:+.3f} m - v*{tau*1000:.0f} ms  '
              f'(expect +{a.lever:.2f} m lever arm if uncorrected; tau = effective GPS lag)')


def kasa(p):
    A = np.c_[2 * p, np.ones(len(p))]
    b = (p ** 2).sum(axis=1)
    (cx, cy, k), *_ = np.linalg.lstsq(A, b, rcond=None)
    r = math.sqrt(k + cx * cx + cy * cy)
    return np.array([cx, cy]), r, np.abs(np.hypot(*(p - [cx, cy]).T) - r).std()


def mode_spin(S, a):
    segs = segments(S, lambda x: abs(x[5]) > 0.1 and abs(x[4]) < 0.05 and a.t0 <= x[0] <= a.t1, 8.0)
    for t0, t1 in segs:
        tr, an = fixed_pts(S, 'pos', t0, t1), fixed_pts(S, 'gnss', t0, t1)
        ek = np.array([(x[1], x[2]) for x in S['glob'] if t0 <= x[0] <= t1])
        turned = sum(abs(wrap(b[1] - c[1])) for c, b in zip(S['yaw'], S['yaw'][1:]) if t0 <= b[0] <= t1)
        if len(tr) < 50 or len(an) < 10 or turned < math.radians(270):
            print(f'{t0:.0f}-{t1:.0f}: turned {math.degrees(turned):.0f} deg / not FIXED, skipped')
            continue
        (ci, ri, ei), (ca, ra, ea) = kasa(tr), kasa(an)
        out = f'{t0:.0f}-{t1:.0f} turned {math.degrees(turned):.0f} deg: IMU-point r={ri:.3f} (fit sd {100*ei:.1f} cm), antenna r={ra:.3f} (sd {100*ea:.1f} cm), centres differ {100*np.hypot(*(ci-ca)):.1f} cm'
        if len(ek) > 10:
            ce, re, _ = kasa(ek)
            out += f', /odometry/global r={re:.3f}'
        # ICR in body frame: mean of R^T(yaw)*(centre - imu_point)
        v = []
        for p in S['pos']:
            if t0 <= p[0] <= t1:
                y = yaw_at(S, p[0]); d = ci - p[1:3]
                v.append((math.cos(y) * d[0] + math.sin(y) * d[1], -math.sin(y) * d[0] + math.cos(y) * d[1]))
        icr = np.mean(v, axis=0)
        print(out + f'\n    ICR in body frame (from IMU/base_link): fwd {icr[0]:+.3f} m, left {icr[1]:+.3f} m; '
              f'implied antenna fwd offset {ri+ra if icr[0]>0 else ra-ri:.3f} m (configured {a.lever})')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag'); ap.add_argument('mode', choices=['static', 'stops', 'line', 'spin'])
    ap.add_argument('--datum', nargs=2, type=float, default=DATUM)
    ap.add_argument('--mark', nargs=2, type=float, help='surveyed lat lon of the marked point')
    ap.add_argument('--lever', type=float, default=0.74)
    ap.add_argument('--t0', type=float, default=0.0, help='abs start time [s], default all')
    ap.add_argument('--t1', type=float, default=1e12)
    a = ap.parse_args()
    S = build(read_bag(a.bag, TOPICS), ENU(*a.datum))
    {'static': mode_static, 'stops': mode_stops, 'line': mode_line, 'spin': mode_spin}[a.mode](S, a)


if __name__ == '__main__':
    main()
