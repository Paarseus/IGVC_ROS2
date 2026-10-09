#!/usr/bin/env python3
"""Read-only offline analysis for the 2026-09-27 odometry / kinematics / local-EKF field tests.

Reads ONE rosbag2 (sqlite3 or mcap) and prints metrics. Never publishes, never touches the robot.
Run where the ROS 2 Humble overlay is sourced (needs rosbag2_py + xsens_mti_ros2_driver msgs), e.g. the
Jetson AFTER the test or a laptop with the IGVC overlay:

  python3 odom_kinematics_analyze.py BAG --test straight   # T1 distance scale + course vs IMU yaw
  python3 odom_kinematics_analyze.py BAG --test spin       # T2 alpha + rotation-centre fit
  python3 odom_kinematics_analyze.py BAG --test arc        # T3 heading-hold interference
  python3 odom_kinematics_analyze.py BAG --test loop       # T4 local-EKF vs RTK truth
  python3 odom_kinematics_analyze.py BAG --test clock      # header-stamp vs receive-time offsets
Options: --antenna-x 0.76 (antenna ahead of base_link, m), --m-per-rev 0.01994, --track 0.7366

Topics used: /wheel_odom /imu/data /odometry/filtered /gnss /status /cmd_vel /avros/wheel_debug
(wheel_debug layout: L_cmd_rpm R_cmd_rpm L_meas_rpm R_meas_rpm L_pos_rev R_pos_rev v_target w_target
 v_slewed w_slewed v_after_imu w_after_imu yaw yaw_rate heading_locked estop)
"""
import argparse
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = ['/wheel_odom', '/imu/data', '/odometry/filtered', '/gnss', '/status', '/cmd_vel',
          '/avros/wheel_debug']


def read_bag(path):
    fmt = 'mcap' if path.endswith('.mcap') else ''
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=fmt),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = {t: [] for t in TOPICS}
    while r.has_next():
        topic, raw, t_ns = r.read_next()
        if topic in out:
            out[topic].append((t_ns * 1e-9, deserialize_message(raw, get_message(types[topic]))))
    return out


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def arrays(b):
    A = {}
    A['imu'] = np.array([(t, yaw_q(m.orientation), m.angular_velocity.z) for t, m in b['/imu/data']])
    A['wo'] = np.array([(t, m.twist.twist.linear.x, m.twist.twist.angular.z) for t, m in b['/wheel_odom']])
    A['ekf'] = np.array([(t, m.pose.pose.position.x, m.pose.pose.position.y, yaw_q(m.pose.pose.orientation),
                          m.twist.twist.linear.x) for t, m in b['/odometry/filtered']])
    A['cmd'] = np.array([(t, m.linear.x, m.angular.z) for t, m in b['/cmd_vel']]) if b['/cmd_vel'] else None
    A['dbg'] = np.array([[t] + list(m.data) for t, m in b['/avros/wheel_debug']]) if b['/avros/wheel_debug'] else None
    g = [(t, m.latitude, m.longitude) for t, m in b['/gnss'] if m.status.status >= 0]
    if g:
        g = np.array(g)
        lat0, lon0 = g[0, 1], g[0, 2]
        k = 6378137.0
        e = np.radians(g[:, 2] - lon0) * k * math.cos(math.radians(lat0))
        n = np.radians(g[:, 1] - lat0) * k
        A['gnss'] = np.column_stack([g[:, 0], e, n])
    else:
        A['gnss'] = None
    st = [(t, getattr(m, 'rtk_status', -1)) for t, m in b['/status']]
    A['rtk'] = np.array(st) if st else None
    return A


def interp(t, T, V):
    return np.interp(t, T, V)


def unwrap_interp(t, T, Y):
    return np.interp(t, T, np.unwrap(Y))


def rtk_fixed_frac(A, t0, t1):
    if A['rtk'] is None:
        return float('nan')
    s = A['rtk'][(A['rtk'][:, 0] >= t0) & (A['rtk'][:, 0] <= t1)]
    return float(np.mean(s[:, 1] == 2)) if len(s) else float('nan')


def segments(A, key_v=1, thr=0.03, min_len=2.0, rot=False):
    """Motion segments from wheel_odom (|v|>thr, or |w|>thr for spins)."""
    wo = A['wo']
    col = 2 if rot else key_v
    moving = np.abs(wo[:, col]) > thr
    segs, start = [], None
    for i, mv in enumerate(moving):
        if mv and start is None:
            start = wo[i, 0]
        if not mv and start is not None:
            if wo[i, 0] - start >= min_len:
                segs.append((start - 0.5, wo[i, 0] + 1.5))  # include stop tail
            start = None
    return segs


def enc_track_dist(A, t0, t1, mpr):
    """Lag-free track displacements from cumulative encoder revs in /avros/wheel_debug."""
    d = A['dbg']
    if d is None:
        return None
    s = d[(d[:, 0] >= t0) & (d[:, 0] <= t1)]
    if len(s) < 2:
        return None
    return (s[-1, 5] - s[0, 5]) * mpr, (s[-1, 6] - s[0, 6]) * mpr  # L, R metres


def base_from_antenna(A, ax):
    """base_link ENU position = antenna - R(yaw)*[ax,0]; yaw from IMU (ENU)."""
    g = A['gnss']
    yaw = unwrap_interp(g[:, 0], A['imu'][:, 0], A['imu'][:, 1])
    bx = g[:, 1] - ax * np.cos(yaw)
    by = g[:, 2] - ax * np.sin(yaw)
    return g[:, 0], bx, by, yaw


def test_straight(A, a):
    print('T1 straight: per segment (need RTK FIXED fraction = 1.00)')
    print(' seg  dur  wheel_enc  wheel_int  ekf_len  rtk_disp  scale_enc  scale_ekf  course-imuYaw  lat_dev  fixed')
    for i, (t0, t1) in enumerate(segments(A)):
        L_R = enc_track_dist(A, t0, t1, a.m_per_rev)
        enc = abs(sum(L_R) / 2) if L_R else float('nan')
        wo = A['wo'][(A['wo'][:, 0] >= t0) & (A['wo'][:, 0] <= t1)]
        wint = abs(np.sum(wo[1:, 1] * np.diff(wo[:, 0])))
        ek = A['ekf'][(A['ekf'][:, 0] >= t0) & (A['ekf'][:, 0] <= t1)]
        elen = np.sum(np.hypot(np.diff(ek[:, 1]), np.diff(ek[:, 2])))
        rd = cdev = latd = float('nan')
        if A['gnss'] is not None:
            g = A['gnss'][(A['gnss'][:, 0] >= t0) & (A['gnss'][:, 0] <= t1)]
            if len(g) > 3:
                dx, dy = g[-1, 1] - g[0, 1], g[-1, 2] - g[0, 2]
                rd = math.hypot(dx, dy)
                course = math.atan2(dy, dx)
                im = A['imu'][(A['imu'][:, 0] >= t0 + 1) & (A['imu'][:, 0] <= t1 - 1.5)]
                if len(im):
                    my = math.atan2(np.mean(np.sin(im[:, 1])), np.mean(np.cos(im[:, 1])))
                    if wo[len(wo) // 2, 1] < 0:
                        my += math.pi  # reversing: course is opposite heading
                    cdev = math.degrees(math.atan2(math.sin(course - my), math.cos(course - my)))
                ux, uy = dx / rd, dy / rd
                latd = np.max(np.abs(-(g[:, 1] - g[0, 1]) * uy + (g[:, 2] - g[0, 2]) * ux))
        print(f' {i:3d} {t1 - t0:5.1f} {enc:9.3f} {wint:9.3f} {elen:8.3f} {rd:8.3f} {enc / rd:9.4f} '
              f'{elen / rd:9.4f} {cdev:12.2f} {latd:8.3f} {rtk_fixed_frac(A, t0, t1):6.2f}')
    print(' PASS: |scale-1| < 0.02 grass / 0.01 pavement, std(scale) < 0.005; |course-imuYaw| < 2 deg after warm-up')


def fit_circle(x, y):
    Am = np.column_stack([x, y, np.ones_like(x)])
    c, *_ = np.linalg.lstsq(Am, x * x + y * y, rcond=None)
    cx, cy = c[0] / 2, c[1] / 2
    return cx, cy, math.sqrt(c[2] + cx * cx + cy * cy)


def test_spin(A, a):
    print('T2 spin: per segment')
    print(' seg  cmd_deg  imu_deg  wodom_deg  enc_deg  deliv(imu/cmd)  alpha(imu/enc)  wodom/imu  r_ant  x_c  ekf_move  true_base_move')
    for i, (t0, t1) in enumerate(segments(A, rot=True)):
        im = A['imu'][(A['imu'][:, 0] >= t0) & (A['imu'][:, 0] <= t1)]
        imu_d = math.degrees(np.unwrap(im[:, 1])[-1] - np.unwrap(im[:, 1])[0])
        wo = A['wo'][(A['wo'][:, 0] >= t0) & (A['wo'][:, 0] <= t1)]
        wo_d = math.degrees(np.sum(wo[1:, 2] * np.diff(wo[:, 0])))
        cmd_d = float('nan')
        if A['cmd'] is not None:
            c = A['cmd'][(A['cmd'][:, 0] >= t0) & (A['cmd'][:, 0] <= t1)]
            if len(c) > 1:  # actuator slew ignored: use w_slewed from wheel_debug when present
                cmd_d = math.degrees(np.sum(c[1:, 2] * np.diff(c[:, 0])))
        if A['dbg'] is not None:
            d = A['dbg'][(A['dbg'][:, 0] >= t0) & (A['dbg'][:, 0] <= t1)]
            cmd_d = math.degrees(np.sum(d[1:, 10] * np.diff(d[:, 0])))  # w_slewed
        LR = enc_track_dist(A, t0, t1, a.m_per_rev)
        enc_d = math.degrees((LR[1] - LR[0]) / a.track) if LR else float('nan')
        r = xc = em = tbm = float('nan')
        ek = A['ekf'][(A['ekf'][:, 0] >= t0) & (A['ekf'][:, 0] <= t1)]
        em = math.hypot(ek[-1, 1] - ek[0, 1], ek[-1, 2] - ek[0, 2])
        if A['gnss'] is not None and abs(imu_d) > 90:
            g = A['gnss'][(A['gnss'][:, 0] >= t0) & (A['gnss'][:, 0] <= t1)]
            if len(g) > 10:
                cx, cy, r = fit_circle(g[:, 1], g[:, 2])
                xc = a.antenna_x - r  # rotation centre ahead of base_link (assumes it lies on centreline)
                T, bx, by, _ = base_from_antenna(A, a.antenna_x)
                s = (T >= t0) & (T <= t1)
                tbm = np.max(np.hypot(bx[s] - bx[s][0], by[s] - by[s][0]))
        print(f' {i:3d} {cmd_d:8.1f} {imu_d:8.1f} {wo_d:9.1f} {enc_d:8.1f} {imu_d / cmd_d:14.3f} {imu_d / enc_d:14.3f} '
              f'{wo_d / imu_d:10.3f} {r:6.3f} {xc:5.2f} {em:8.3f} {tbm:14.3f}')
    print(' new multiplier (pure spin) = current_mult / deliv.  x_c = rotation-centre offset ahead of base_link.')
    print(' PASS: deliv 0.95-1.05; ekf_move vs true_base_move differ < 0.10 m (else base_link != rotation centre)')


def test_arc(A, a):
    d = A['dbg']
    if d is None:
        sys.exit('need /avros/wheel_debug')
    mv = np.abs(d[:, 9]) > 0.05  # v_slewed moving
    small = mv & (np.abs(d[:, 8]) > 1e-3) & (np.abs(d[:, 8]) < 0.05)  # nonzero w_target inside deadband
    locked = d[:, 15] > 0.5
    print(f'T3 arc: samples moving {mv.sum()}, heading-hold locked {np.mean(locked[mv]):.2%} of moving time')
    print(f'  small nonzero w_target (<0.05) {small.sum()} samples; of those overridden by lock: '
          f'{np.mean(locked[small]) if small.any() else float("nan"):.2%}')
    if small.any():
        disagree = small & locked & (np.sign(d[:, 12]) != np.sign(d[:, 8]))
        print(f'  lock output opposite sign to w_target: {disagree.sum() / small.sum():.2%}')
    t = d[:, 0]
    imu_w = interp(t, A['imu'][:, 0], A['imu'][:, 2])
    for lo, hi in [(0.0, 0.05), (0.05, 0.15), (0.15, 0.5), (0.5, 2.0)]:
        s = mv & (np.abs(d[:, 8]) >= lo) & (np.abs(d[:, 8]) < hi) & (np.abs(d[:, 8]) > 1e-3)
        if s.sum() > 20:
            print(f'  |w_target| in [{lo},{hi}): mean imu_w/w_target = {np.mean(imu_w[s] / d[s, 8]):.3f} (n={s.sum()})')
    print(' PASS: with deadband active, imu_w/w_target within 0.8-1.2 in every band incl. [0,0.05); '
          'lock-overrides <10% of small-w samples under Nav2')


def se2_align(P, Q):
    """Rigid 2D fit Q ~ R P + t; returns aligned P."""
    mp, mq = P.mean(0), Q.mean(0)
    H = (P - mp).T @ (Q - mq)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1] *= -1
        R = Vt.T @ U.T
    return (R @ (P - mp).T).T + mq


def test_loop(A, a):
    if A['gnss'] is None:
        sys.exit('need /gnss')
    T, bx, by, _ = base_from_antenna(A, a.antenna_x)
    ek = A['ekf']
    ex, ey = interp(T, ek[:, 0], ek[:, 1]), interp(T, ek[:, 0], ek[:, 2])
    s = (T >= ek[0, 0]) & (T <= ek[-1, 0])
    P, Q = np.column_stack([ex[s], ey[s]]), np.column_stack([bx[s], by[s]])
    Pa = se2_align(P, Q)
    err = np.hypot(*(Pa - Q).T)
    dist = np.sum(np.hypot(*np.diff(Q, axis=0).T))
    print(f'T4 loop: RTK-fixed frac {rtk_fixed_frac(A, T[s][0], T[s][-1]):.2f}, path {dist:.1f} m')
    print(f'  local-EKF ATE (SE2-aligned): RMS {np.sqrt(np.mean(err ** 2)):.3f} m, max {err.max():.3f} m '
          f'({100 * err.max() / dist:.2f}% of path)')
    # relative error over ~5 m windows (what matters for gap threading)
    cum = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(Q, axis=0).T))])
    rel = []
    j = 0
    for i in range(len(Q)):
        while j < len(Q) and cum[j] - cum[i] < 5.0:
            j += 1
        if j >= len(Q):
            break
        rel.append(abs(np.hypot(*(P[j] - P[i])) - np.hypot(*(Q[j] - Q[i]))))
    if rel:
        print(f'  5 m relative distance error: median {np.median(rel):.3f} m, p95 {np.percentile(rel, 95):.3f} m')
    e_close = math.hypot(ek[-1, 1] - ek[0, 1], ek[-1, 2] - ek[0, 2])
    t_close = math.hypot(bx[s][-1] - bx[s][0], by[s][-1] - by[s][0])
    print(f'  EKF closure {e_close:.3f} m vs RTK closure {t_close:.3f} m')
    print(' PASS: ATE max < 1% of path (<0.20 m on 20 m); 5 m rel p95 < 0.10 m')


def test_clock(b):
    print('clock: header stamp minus bag receive time (s); |median| > 0.1 means clock/stamp problem')
    for tp in ['/imu/data', '/wheel_odom', '/odometry/filtered', '/gnss']:
        v = [stamp(m) - t for t, m in b[tp]]
        if v:
            print(f'  {tp:22s} median {np.median(v):+.3f} p95 {np.percentile(v, 95):+.3f} n={len(v)}')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('bag')
    p.add_argument('--test', required=True, choices=['straight', 'spin', 'arc', 'loop', 'clock'])
    p.add_argument('--antenna-x', type=float, default=0.76)
    p.add_argument('--m-per-rev', type=float, default=0.01994)
    p.add_argument('--track', type=float, default=0.7366)
    a = p.parse_args()
    b = read_bag(a.bag)
    if a.test == 'clock':
        return test_clock(b)
    A = arrays(b)
    {'straight': test_straight, 'spin': test_spin, 'arc': test_arc, 'loop': test_loop}[a.test](A, a)


if __name__ == '__main__':
    main()
