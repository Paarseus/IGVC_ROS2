#!/usr/bin/env python3
"""Phase 0 analysis: static health of RTK, IMU, odometry and position filters.

Reads a bag recorded by phase0_record.sh (robot parked). Read-only.
Usage: python3 phase0_analyze.py <bag_dir> <out_dir> [launch_epoch_s]
Writes <out_dir>/phase0_metrics.json and prints a results table (markdown).
"""
import json
import math
import os
import statistics as st
import sys
from collections import defaultdict

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG, OUT = sys.argv[1], sys.argv[2]
LAUNCH_T = float(sys.argv[3]) if len(sys.argv) > 3 else None
os.makedirs(OUT, exist_ok=True)

# ---------------------------------------------------------------- read bag
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
cls = {n: get_message(t) for n, t in types.items()}
msgs = defaultdict(list)          # topic -> [(receive_time_s, msg)]
while reader.has_next():
    topic, data, t_ns = reader.read_next()
    msgs[topic].append((t_ns * 1e-9, deserialize_message(data, cls[topic])))

t_all = [t for v in msgs.values() for t, _ in v]
T0, T1 = min(t_all), max(t_all)


def hstamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def q_to_rpy(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def unwrap(a):
    out, off = [a[0]], 0.0
    for p, c in zip(a, a[1:]):
        d = c - p
        if d > math.pi:
            off -= 2 * math.pi
        elif d < -math.pi:
            off += 2 * math.pi
        out.append(c + off)
    return out


def pct(v, p):
    s = sorted(v)
    return s[min(len(s) - 1, int(p / 100 * len(s)))] if s else float('nan')


def slope(ts, ys):
    n = len(ts)
    if n < 2:
        return float('nan')
    mt, my = sum(ts) / n, sum(ys) / n
    den = sum((t - mt) ** 2 for t in ts)
    return sum((t - mt) * (y - my) for t, y in zip(ts, ys)) / den if den else float('nan')


# WGS84 local east/north (metres) around a reference point
A_WGS, E2 = 6378137.0, 6.69437999014e-3


def enu_factory(lat0, lon0):
    s = math.sin(math.radians(lat0))
    n_rad = A_WGS / math.sqrt(1 - E2 * s * s)
    m_rad = A_WGS * (1 - E2) / (1 - E2 * s * s) ** 1.5
    ke = math.radians(1) * n_rad * math.cos(math.radians(lat0))
    kn = math.radians(1) * m_rad
    return lambda lat, lon: ((lon - lon0) * ke, (lat - lat0) * kn)


def spread(xy):
    """std per axis, horizontal RMS about the mean, max distance from mean."""
    if len(xy) < 2:
        return None
    mx = sum(p[0] for p in xy) / len(xy)
    my = sum(p[1] for p in xy) / len(xy)
    d = [math.hypot(p[0] - mx, p[1] - my) for p in xy]
    return {'n': len(xy), 'std_e': st.pstdev(p[0] for p in xy), 'std_n': st.pstdev(p[1] for p in xy),
            'rms': math.sqrt(sum(x * x for x in d) / len(d)), 'max': max(d), 'mean': (mx, my)}


R = {'bag': BAG, 'duration_s': T1 - T0, 'topics_present': sorted(msgs)}

# ---------------------------------------------------------------- timing
RATE_TARGET = {'/imu/data': 95, '/wheel_odom': 19, '/odometry/filtered': 28,
               '/odometry/global': 28, '/gnss': 3.5, '/filter/positionlla': 3.5,
               '/odometry/gps': 3.5, '/rtcm': 0.8, '/status': 3.5}
timing = {}
for tp, v in sorted(msgs.items()):
    ts = [t for t, _ in v]
    if len(ts) < 2:
        timing[tp] = {'count': len(ts)}
        continue
    gaps = [b - a for a, b in zip(ts, ts[1:])]
    row = {'count': len(ts), 'rate_hz': (len(ts) - 1) / (ts[-1] - ts[0]), 'max_gap_s': max(gaps)}
    if hasattr(v[0][1], 'header') and tp not in ('/tf', '/tf_static'):
        lat = [t - hstamp(m) for t, m in v]
        row['delay_median_ms'] = st.median(lat) * 1e3
        row['delay_p95_ms'] = pct(lat, 95) * 1e3
    timing[tp] = row
R['timing'] = timing

# ---------------------------------------------------------------- RTK state timeline
status = [(t, m.rtk_status) for t, m in msgs.get('/status', [])]


def rtk_at(t):
    lo, hi, best = 0, len(status) - 1, None
    while lo <= hi:
        mid = (lo + hi) // 2
        if status[mid][0] <= t:
            best, lo = status[mid][1], mid + 1
        else:
            hi = mid - 1
    return best


rtk = {}
rtcm_t = [t for t, _ in msgs.get('/rtcm', [])]
gnss_t = [t for t, _ in msgs.get('/gnss', [])]
first_fix = next((t for t, s in status if s == 2), None)
rtk['bag_start_to_first_gnss_s'] = (gnss_t[0] - T0) if gnss_t else None
rtk['first_corrections_to_first_fixed_s'] = (first_fix - rtcm_t[0]) if (first_fix and rtcm_t) else None
rtk['launch_to_first_fixed_s'] = (first_fix - LAUNCH_T) if (first_fix and LAUNCH_T) else None
rtk['ever_fixed'] = first_fix is not None
if status:
    counts = defaultdict(int)
    for _, s in status:
        counts[s] += 1
    rtk['state_counts_all'] = dict(counts)
if first_fix:
    after = [s for t, s in status if t >= first_fix]
    rtk['fixed_share_after_first_fix'] = after.count(2) / len(after)
    drops = []
    for (ta, sa), (tb, sb) in zip(status, status[1:]):
        if ta >= first_fix and sa == 2 and sb != 2:
            before = [t for t in rtcm_t if tb - 5 <= t <= tb]
            g = [b - a for a, b in zip(before, before[1:])]
            drops.append({'t_s': tb - T0, 'to_state': sb,
                          'max_corrections_gap_5s_before': max(g) if g else None})
    rtk['drops_out_of_fixed'] = drops
if len(rtcm_t) > 1:
    g = [b - a for a, b in zip(rtcm_t, rtcm_t[1:])]
    rtk['corrections_msgs_per_s'] = (len(rtcm_t) - 1) / (rtcm_t[-1] - rtcm_t[0])
    rtk['corrections_max_gap_s'] = max(g)
    rtk['corrections_gaps_over_2s'] = sum(1 for x in g if x > 2)

# satellites / HDOP from GGA, by fix quality
gga = defaultdict(list)
for t, m in msgs.get('/nmea', []):
    f = m.sentence.split(',')
    if len(f) > 8 and f[0].endswith('GGA') and f[6] and f[7] and f[8]:
        gga[int(f[6])].append((int(f[7]), float(f[8])))
rtk['gga_by_quality'] = {q: {'n': len(v), 'sats_min': min(a for a, _ in v), 'sats_median': st.median(a for a, _ in v),
                             'hdop_median': st.median(b for _, b in v), 'hdop_max': max(b for _, b in v)}
                         for q, v in gga.items()}

# position spread and reported vs measured accuracy
gnss = [(t, m) for t, m in msgs.get('/gnss', []) if not math.isnan(m.latitude)]
enu = None
if gnss:
    fixed = [(t, m) for t, m in gnss if rtk_at(t) == 2]
    ref = fixed or gnss
    lat0 = st.mean(m.latitude for _, m in ref)
    lon0 = st.mean(m.longitude for _, m in ref)
    enu = enu_factory(lat0, lon0)
    rtk['reference_point'] = [lat0, lon0]
    by_state = defaultdict(list)
    for t, m in gnss:
        by_state[rtk_at(t)].append(m)
    acc = {}
    for s, ms in by_state.items():
        xy = [enu(m.latitude, m.longitude) for m in ms]
        sp = spread(xy)
        rep = [math.sqrt(max(m.position_covariance[0], 0)) for m in ms]
        err_vs_fixed = [math.hypot(x, y) for x, y in xy] if fixed else None
        acc[str(s)] = {
            'n': len(ms),
            'spread': sp and {k: v for k, v in sp.items() if k != 'mean'},
            'reported_sigma_median_m': st.median(rep),
            'rms_error_vs_fixed_mean_m': math.sqrt(st.mean(e * e for e in err_vs_fixed)) if err_vs_fixed else None,
            'alt_std_m': st.pstdev(m.altitude for m in ms) if len(ms) > 1 else None,
        }
    rtk['accuracy_by_state'] = acc
R['rtk'] = rtk

# ---------------------------------------------------------------- antenna offset (static)
lever = {}
imu_msgs = msgs.get('/imu/data', [])
pl = msgs.get('/filter/positionlla', [])
if enu and pl and gnss and imu_msgs:
    pl_t = [t for t, _ in pl]
    imu_t = [t for t, _ in imu_msgs]

    def nearest(ts, t):
        import bisect
        i = bisect.bisect_left(ts, t)
        c = [j for j in (i - 1, i) if 0 <= j < len(ts)]
        return min(c, key=lambda j: abs(ts[j] - t))

    d, ang = [], []
    for t, m in gnss:
        if rtk_at(t) != 2:
            continue
        j = nearest(pl_t, t)
        if abs(pl_t[j] - t) > 0.1:
            continue
        ae, an = enu(m.latitude, m.longitude)
        ie, inn = enu(pl[j][1].vector.x, pl[j][1].vector.y)
        de, dn = ae - ie, an - inn
        k = nearest(imu_t, t)
        yaw = q_to_rpy(imu_msgs[k][1].orientation)[2]
        d.append(math.hypot(de, dn))
        diff = math.atan2(dn, de) - yaw
        ang.append(math.degrees(math.atan2(math.sin(diff), math.cos(diff))))
    if d:
        lever = {'n': len(d), 'distance_median_m': st.median(d), 'distance_std_m': st.pstdev(d),
                 'bearing_minus_imu_heading_median_deg': st.median(ang)}
R['antenna_offset'] = lever

# ---------------------------------------------------------------- IMU static
imu = {}
if imu_msgs:
    ms = [m for _, m in imu_msgs]
    ts = [t for t, _ in imu_msgs]
    for ax in 'xyz':
        w = [getattr(m.angular_velocity, ax) for m in ms]
        imu[f'gyro_{ax}_mean_dps'] = math.degrees(st.mean(w))
        imu[f'gyro_{ax}_std_dps'] = math.degrees(st.pstdev(w))
    a = [math.sqrt(m.linear_acceleration.x ** 2 + m.linear_acceleration.y ** 2 + m.linear_acceleration.z ** 2)
         for m in ms]
    imu['gravity_mean_mps2'] = st.mean(a)
    imu['accel_norm_std_mps2'] = st.pstdev(a)
    rpy = [q_to_rpy(m.orientation) for m in ms]
    imu['roll_mean_deg'] = math.degrees(st.mean(r for r, _, _ in rpy))
    imu['pitch_mean_deg'] = math.degrees(st.mean(p for _, p, _ in rpy))
    yaw = unwrap([y for _, _, y in rpy])
    imu['yaw_drift_deg_per_min'] = math.degrees(slope(ts, yaw)) * 60
    imu['yaw_total_change_deg'] = math.degrees(yaw[-1] - yaw[0])
    # drift over the second half only (after start-up settling)
    h = len(ts) // 2
    imu['yaw_drift_second_half_deg_per_min'] = math.degrees(slope(ts[h:], yaw[h:])) * 60
    # heading change the gyro alone would predict (bias x time); the rest is the
    # Xsens filter correcting its own heading
    imu['yaw_change_from_gyro_deg'] = imu['gyro_z_mean_dps'] * (ts[-1] - ts[0])
    # heading drift per minute, in 1-minute windows (shows settling)
    per_min = []
    t_start = ts[0]
    while t_start + 60 <= ts[-1] + 1e-6:
        idx = [i for i, t in enumerate(ts) if t_start <= t < t_start + 60]
        if len(idx) > 100:
            per_min.append(round(math.degrees(yaw[idx[-1]] - yaw[idx[0]]), 3))
        t_start += 60
    imu['yaw_change_per_minute_deg'] = per_min
    imu['yaw_heading_mean_deg_enu'] = math.degrees(st.mean(yaw))
    imu['reported_cov_orientation'] = ms[-1].orientation_covariance[0]
    imu['reported_cov_gyro'] = ms[-1].angular_velocity_covariance[0]
    imu['reported_cov_accel'] = ms[-1].linear_acceleration_covariance[0]
R['imu'] = imu

# ---------------------------------------------------------------- wheels and filters
still = {}
wd = msgs.get('/avros/wheel_debug', [])
if wd:
    lab = wd[0][1].layout.dim[0].label.split(',') if wd[0][1].layout.dim else []
    if 'L_pos_rev' in lab:
        iL, iR = lab.index('L_pos_rev'), lab.index('R_pos_rev')
        imL, imR = lab.index('L_meas_rpm'), lab.index('R_meas_rpm')
        still['wheel_pos_change_rev'] = [wd[-1][1].data[iL] - wd[0][1].data[iL],
                                         wd[-1][1].data[iR] - wd[0][1].data[iR]]
        still['wheel_rpm_max_abs'] = max(max(abs(m.data[imL]), abs(m.data[imR])) for _, m in wd)
wo = msgs.get('/wheel_odom', [])
if wo:
    still['wheel_odom_speed_max_mps'] = max(abs(m.twist.twist.linear.x) for _, m in wo)
    p0, p1 = wo[0][1].pose.pose.position, wo[-1][1].pose.pose.position
    still['wheel_odom_pose_change_m'] = math.hypot(p1.x - p0.x, p1.y - p0.y)


def odom_drift(tp):
    v = msgs.get(tp, [])
    if not v:
        return None
    p0 = v[0][1].pose.pose.position
    ex = max(math.hypot(m.pose.pose.position.x - p0.x, m.pose.pose.position.y - p0.y) for _, m in v)
    y = unwrap([q_to_rpy(m.pose.pose.orientation)[2] for _, m in v])
    p1 = v[-1][1].pose.pose.position
    return {'start_to_end_m': math.hypot(p1.x - p0.x, p1.y - p0.y), 'max_excursion_m': ex,
            'yaw_change_deg': math.degrees(y[-1] - y[0])}


still['local_filter'] = odom_drift('/odometry/filtered')
still['robot_moved'] = bool(
    (still.get('wheel_pos_change_rev') and max(abs(x) for x in still['wheel_pos_change_rev']) > 0.05)
    or (still.get('wheel_odom_pose_change_m', 0) > 0.05))
R['static'] = still

# map position while FIXED, and jumps when RTK re-fixes
glob = {}
go = msgs.get('/odometry/global', [])
if go:
    # skip the first 60 s: the filter starts at (0, 0) and jumps to the real position
    t_go0 = go[0][0]
    fx = [(m.pose.pose.position.x, m.pose.pose.position.y) for t, m in go if rtk_at(t) == 2 and t - t_go0 > 60]
    sp = spread(fx)
    glob['map_position_spread_fixed'] = sp and {k: v for k, v in sp.items() if k != 'mean'}
    glob['map_position_last'] = [go[-1][1].pose.pose.position.x, go[-1][1].pose.pose.position.y]
    jumps = []
    for (ta, sa), (tb, sb) in zip(status, status[1:]):
        if sa != 2 and sb == 2:
            pre = [(m.pose.pose.position.x, m.pose.pose.position.y) for t, m in go if tb - 2 <= t < tb]
            post = [(m.pose.pose.position.x, m.pose.pose.position.y) for t, m in go if tb + 1 <= t < tb + 3]
            if pre and post:
                a, b = spread(pre) or {'mean': pre[0]}, spread(post) or {'mean': post[0]}
                jumps.append({'t_s': tb - T0, 'jump_m': math.hypot(b['mean'][0] - a['mean'][0],
                                                                   b['mean'][1] - a['mean'][1])})
    glob['jumps_on_refix'] = jumps
mo = []
for t, m in msgs.get('/tf', []):
    for tr in m.transforms:
        if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom' and rtk_at(t) == 2 and t - T0 > 60:
            mo.append((tr.transform.translation.x, tr.transform.translation.y))
sp = spread(mo)
glob['map_to_odom_spread_fixed'] = sp and {k: v for k, v in sp.items() if k != 'mean'}
R['global'] = glob

with open(os.path.join(OUT, 'phase0_metrics.json'), 'w') as f:
    json.dump(R, f, indent=2, default=str)

# ---------------------------------------------------------------- results table
rows = []


def add(group, name, value, target, ok):
    rows.append((group, name, value, target, '—' if ok is None else ('PASS' if ok else 'FAIL')))


def f(x, n=2, unit=''):
    return 'n/a' if x is None or (isinstance(x, float) and math.isnan(x)) else f'{x:.{n}f}{unit}'


add('Recording', 'Length', f(R['duration_s'] / 60, 1, ' min'), '≥ 10 min', R['duration_s'] >= 590)
add('Recording', 'Robot stayed still', 'yes' if not still['robot_moved'] else 'NO — results invalid', 'yes',
    not still['robot_moved'])

ttf = rtk.get('first_corrections_to_first_fixed_s')
add('RTK', 'Corrections start → first FIXED', f(ttf and ttf / 60, 1, ' min') if ttf else 'never', '< 3 min',
    ttf is not None and ttf < 180)
if rtk.get('launch_to_first_fixed_s') is not None:
    add('RTK', 'Launch → first FIXED', f(rtk['launch_to_first_fixed_s'] / 60, 1, ' min'), 'record', None)
sh = rtk.get('fixed_share_after_first_fix')
add('RTK', 'Time FIXED after first FIXED', f(sh and sh * 100, 1, ' %'), '≥ 95 %', sh is not None and sh >= 0.95)
drops = rtk.get('drops_out_of_fixed', [])
unexplained = [d for d in drops if not (d['max_corrections_gap_5s_before'] or 0) > 2]
add('RTK', 'Drops out of FIXED (not explained by a correction gap)', f'{len(drops)} ({len(unexplained)})',
    '0 unexplained', len(unexplained) == 0)
fa = (rtk.get('accuracy_by_state') or {}).get('2')
if fa and fa['spread']:
    add('RTK', 'FIXED spread (std east / north)',
        f"{fa['spread']['std_e']*100:.1f} / {fa['spread']['std_n']*100:.1f} cm", '≤ 1.5 cm',
        max(fa['spread']['std_e'], fa['spread']['std_n']) <= 0.015)
    add('RTK', 'FIXED largest distance from average', f"{fa['spread']['max']*100:.1f} cm", '≤ 4 cm',
        fa['spread']['max'] <= 0.04)
for s, name in (('2', 'FIXED'), ('1', 'FLOAT')):
    a = (rtk.get('accuracy_by_state') or {}).get(s)
    if a and a['spread']:
        meas = a['rms_error_vs_fixed_mean_m'] if s == '1' and a['rms_error_vs_fixed_mean_m'] else a['spread']['rms']
        ratio = meas / a['reported_sigma_median_m'] if a['reported_sigma_median_m'] else float('inf')
        add('RTK', f'{name}: reported accuracy vs measured error',
            f"{a['reported_sigma_median_m']*100:.1f} vs {meas*100:.1f} cm (×{ratio:.1f})", 'within 2×',
            0.5 <= ratio <= 2)
g = (rtk.get('gga_by_quality') or {}).get(4)
if g:
    add('RTK', 'Satellites while FIXED (min / median)', f"{g['sats_min']} / {g['sats_median']:.0f}", '≥ 15',
        g['sats_min'] >= 15)
    add('RTK', 'HDOP while FIXED (median / max)', f"{g['hdop_median']:.1f} / {g['hdop_max']:.1f}", '≤ 1.2',
        g['hdop_median'] <= 1.2)
add('RTK', 'Correction messages per second', f(rtk.get('corrections_msgs_per_s'), 1), 'steady', None)
mg = rtk.get('corrections_max_gap_s')
add('RTK', 'Longest correction gap', f(mg, 1, ' s'), '≤ 2 s', mg is not None and mg <= 2)

if lever:
    add('Antenna', 'Antenna ↔ IMU distance (Xsens outputs)', f"{lever['distance_median_m']:.3f} m", '0.74 ± 0.03 m',
        abs(lever['distance_median_m'] - 0.74) <= 0.03)
    add('Antenna', 'Offset direction vs IMU heading', f"{lever['bearing_minus_imu_heading_median_deg']:+.1f}°",
        'within ±5°', abs(lever['bearing_minus_imu_heading_median_deg']) <= 5)

if imu:
    gb = max(abs(imu[f'gyro_{a}_mean_dps']) for a in 'xyz')
    add('IMU', 'Gyro bias x / y / z',
        ' / '.join(f"{imu[f'gyro_{a}_mean_dps']:+.3f}" for a in 'xyz') + ' °/s', '< 0.05 °/s', gb < 0.05)
    add('IMU', 'Gyro noise x / y / z', ' / '.join(f"{imu[f'gyro_{a}_std_dps']:.3f}" for a in 'xyz') + ' °/s',
        'record', None)
    add('IMU', 'Heading drift, whole recording', f"{imu['yaw_drift_deg_per_min']:+.3f} °/min", 'record', None)
    add('IMU', 'Heading drift, second half (settled)', f"{imu['yaw_drift_second_half_deg_per_min']:+.3f} °/min",
        '< 0.2 °/min', abs(imu['yaw_drift_second_half_deg_per_min']) < 0.2)
    add('IMU', 'Total heading change vs what the gyro bias explains',
        f"{imu['yaw_total_change_deg']:+.2f}° vs {imu['yaw_change_from_gyro_deg']:+.2f}°", 'record', None)
    add('IMU', 'Heading change per minute', ', '.join(f'{x:+.2f}' for x in imu['yaw_change_per_minute_deg']) + ' °',
        'record', None)
    add('IMU', 'Roll / pitch', f"{imu['roll_mean_deg']:+.2f}° / {imu['pitch_mean_deg']:+.2f}°", 'record', None)
    add('IMU', 'Gravity', f"{imu['gravity_mean_mps2']:.3f} m/s²", '9.81 ± 0.05',
        abs(imu['gravity_mean_mps2'] - 9.81) <= 0.05)
    add('IMU', 'Reported covariance (orientation / gyro / accel)',
        f"{imu['reported_cov_orientation']:.2g} / {imu['reported_cov_gyro']:.2g} / {imu['reported_cov_accel']:.2g}",
        'non-zero', min(imu['reported_cov_orientation'], imu['reported_cov_gyro'], imu['reported_cov_accel']) > 0)

if 'wheel_rpm_max_abs' in still:
    add('Odometry', 'Wheel speed while parked (max)', f"{still['wheel_rpm_max_abs']:.0f} RPM", '0', still['wheel_rpm_max_abs'] < 1)
if 'wheel_odom_pose_change_m' in still:
    add('Odometry', 'Wheel odometry movement while parked', f"{still['wheel_odom_pose_change_m']*100:.1f} cm", '0',
        still['wheel_odom_pose_change_m'] < 0.005)
lf = still.get('local_filter')
if lf:
    add('Filters', 'Local position drift (largest)', f"{lf['max_excursion_m']*100:.1f} cm", '< 1 cm',
        lf['max_excursion_m'] < 0.01)
    add('Filters', 'Local heading change (follows the IMU heading)', f"{lf['yaw_change_deg']:+.2f}°", '< 0.1°',
        abs(lf['yaw_change_deg']) < 0.1)
gs = glob.get('map_position_spread_fixed')
if gs:
    add('Filters', 'Map position spread while FIXED (std / largest)', f"{gs['rms']*100:.1f} / {gs['max']*100:.1f} cm",
        '≤ 2 cm largest', gs['max'] <= 0.02)
js = glob.get('jumps_on_refix', [])
if js:
    mj = max(j['jump_m'] for j in js)
    add('Filters', 'Map position jump when RTK re-fixes (largest, count)', f"{mj*100:.1f} cm ({len(js)})", '< 10 cm', mj < 0.10)
ms = glob.get('map_to_odom_spread_fixed')
if ms:
    add('Filters', 'Map→odom correction spread while FIXED (largest)', f"{ms['max']*100:.1f} cm", '≤ 2 cm', ms['max'] <= 0.02)

for tp, tgt in RATE_TARGET.items():
    r = timing.get(tp)
    if r and 'rate_hz' in r:
        add('Timing', f'{tp} rate / longest gap', f"{r['rate_hz']:.1f} Hz / {r['max_gap_s']*1000:.0f} ms",
            f'≥ {tgt} Hz', r['rate_hz'] >= tgt)
    elif tp in RATE_TARGET:
        add('Timing', f'{tp} rate', 'missing', f'≥ {tgt} Hz', False)
for tp in ('/imu/data', '/wheel_odom', '/gnss'):
    r = timing.get(tp, {})
    if 'delay_median_ms' in r:
        add('Timing', f'{tp} delay (median / 95th pct)', f"{r['delay_median_ms']:.0f} / {r['delay_p95_ms']:.0f} ms",
            '< 50 ms', r['delay_median_ms'] < 50)

table = ['| Area | Measurement | Result | Target | Pass |', '|---|---|---|---|---|']
table += [f'| {a} | {b} | {c} | {d} | {e} |' for a, b, c, d, e in rows]
with open(os.path.join(OUT, 'phase0_table.md'), 'w') as fh:
    fh.write('\n'.join(table) + '\n')
print('\n'.join(table))
