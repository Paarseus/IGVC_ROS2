#!/usr/bin/env python3
"""Summarize a capture_static.py run. Usage: analyze_static.py <capture_dir>

Prints rates, latencies, noise, drift and reported covariances per topic.
Assumes the robot is stationary; motion indicators are printed first so a
contaminated run can be rejected.
"""
import gzip
import json
import math
import os
import statistics as st
import sys
from collections import Counter

D = sys.argv[1]


def load(name):
    p = os.path.join(D, name + '.jsonl.gz')
    if not os.path.exists(p):
        return []
    with gzip.open(p, 'rt') as f:
        return [json.loads(l) for l in f]


def rate(recs, key='t_rx'):
    if len(recs) < 2:
        return 0.0
    return (len(recs) - 1) / (recs[-1][key] - recs[0][key])


def lat_ms(recs):
    lat = [(r['t_rx'] - r['t']) * 1e3 for r in recs if 't' in r]
    if not lat:
        return 'n/a'
    lat.sort()
    return f"median {st.median(lat):.1f} ms, p95 {lat[int(0.95 * len(lat)) - 1]:.1f} ms, min {lat[0]:.1f} ms"


def dt_jitter(recs):
    d = [(b['t'] - a['t']) * 1e3 for a, b in zip(recs, recs[1:])]
    return f"stamp dt median {st.median(d):.1f} ms, max {max(d):.1f} ms" if d else 'n/a'


def unwrap(a):
    out, off = [a[0]], 0.0
    for p, c in zip(a, a[1:]):
        dlt = c - p
        if dlt > math.pi:
            off -= 2 * math.pi
        elif dlt < -math.pi:
            off += 2 * math.pi
        out.append(c + off)
    return out


def section(t):
    print(f"\n## {t}")


wd = load('avros_wheel_debug')
section('Motion check')
if wd:
    print(f"wheel |meas rpm| max L {max(abs(r['l_meas']) for r in wd):.0f} R {max(abs(r['r_meas']) for r in wd):.0f}; "
          f"|cmd rpm| max {max(max(abs(r['l_cmd']), abs(r['r_cmd'])) for r in wd):.0f}; "
          f"L pos span {max(r['l_pos'] for r in wd) - min(r['l_pos'] for r in wd):.4f} rev, "
          f"R pos span {max(r['r_pos'] for r in wd) - min(r['r_pos'] for r in wd):.4f} rev; estop seen {any(r['estop'] for r in wd)}")
print(f"cmd_vel messages: {len(load('cmd_vel'))}")

imu = load('imu_data')
section('IMU /imu/data')
if imu:
    dur = imu[-1]['t'] - imu[0]['t']
    print(f"n={len(imu)} rate {rate(imu):.1f} Hz, frame '{imu[0]['frame']}', latency {lat_ms(imu)}, {dt_jitter(imu)}")
    y = unwrap([r['yaw'] for r in imu])
    wz = [r['wz'] for r in imu]
    print(f"yaw start {math.degrees(y[0]):.3f} deg, drift over {dur:.0f} s: {math.degrees(y[-1] - y[0]):+.3f} deg "
          f"({math.degrees(y[-1] - y[0]) / dur * 60:+.3f} deg/min), yaw std {math.degrees(st.pstdev(y)):.4f} deg")
    print(f"gyro z mean {math.degrees(st.mean(wz)):+.4f} deg/s, std {math.degrees(st.pstdev(wz)):.4f} deg/s; "
          f"gyro x/y std {math.degrees(st.pstdev([r['wx'] for r in imu])):.4f}/{math.degrees(st.pstdev([r['wy'] for r in imu])):.4f} deg/s")
    print(f"accel mean (x,y,z) = ({st.mean(r['ax'] for r in imu):.3f}, {st.mean(r['ay'] for r in imu):.3f}, {st.mean(r['az'] for r in imu):.3f}) m/s^2")
    c = imu[0]
    print(f"reported cov: orientation diag {[c['ocov'][i] for i in (0, 4, 8)]}, angvel diag {[c['wcov'][i] for i in (0, 4, 8)]}, accel diag {[c['acov'][i] for i in (0, 4, 8)]}")
    print(f"orientation cov constant over run: {len({tuple(r['ocov']) for r in imu}) == 1}")

g = load('gnss')
section('GNSS /gnss (raw receiver PVT)')
if g:
    lat0 = st.mean(r['lat'] for r in g)
    lon0 = st.mean(r['lon'] for r in g)
    k = 111320.0
    e = [(r['lon'] - lon0) * k * math.cos(math.radians(lat0)) for r in g]
    n = [(r['lat'] - lat0) * k for r in g]
    print(f"n={len(g)} rate {rate(g):.2f} Hz, frame '{g[0]['frame']}', latency {lat_ms(g)}, status counts {dict(Counter(r['status'] for r in g))}")
    print(f"horizontal std E {st.pstdev(e) * 100:.2f} cm, N {st.pstdev(n) * 100:.2f} cm; max radius {max(math.hypot(a, b) for a, b in zip(e, n)) * 100:.2f} cm; alt std {st.pstdev(r['alt'] for r in g) * 100:.2f} cm")
    hc = [math.sqrt(r['cov'][0]) * 100 for r in g]
    print(f"reported horizontal sigma: median {st.median(hc):.2f} cm, range {min(hc):.2f}-{max(hc):.2f} cm")

s = load('status')
section('Xsens status')
if s:
    print(f"rtk_status counts {dict(Counter(r['rtk'] for r in s))} (0 none, 1 float, 2 fixed); filter_mode {dict(Counter(r['filter_mode'] for r in s))}")

for topic in ('odometry_gps', 'wheel_odom', 'odometry_filtered', 'odometry_global'):
    o = load(topic)
    section(topic)
    if not o:
        print('no data')
        continue
    dur = o[-1]['t'] - o[0]['t']
    print(f"n={len(o)} rate {rate(o):.1f} Hz, frame '{o[0]['frame']}'->'{o[0]['child']}', latency {lat_ms(o)}")
    xs, ys = [r['x'] for r in o], [r['y'] for r in o]
    yw = unwrap([r['yaw'] for r in o])
    print(f"position span x {(max(xs) - min(xs)) * 100:.2f} cm, y {(max(ys) - min(ys)) * 100:.2f} cm; "
          f"net drift {math.hypot(xs[-1] - xs[0], ys[-1] - ys[0]) * 100:.2f} cm over {dur:.0f} s; yaw drift {math.degrees(yw[-1] - yw[0]):+.3f} deg")
    print(f"twist |vx| max {max(abs(r['vx']) for r in o):.4f} m/s, |vy| max {max(abs(r['vy']) for r in o):.4f}, |wz| max {max(abs(r['wz']) for r in o):.4f} rad/s")
    last = o[-1]
    print(f"reported pose cov (x,y,yaw) {['%.3g' % v for v in last['pcov']]}, twist cov (vx,vy,wz) {['%.3g' % v for v in last['tcov']]}")

tf = load('tf')
section('TF')
for key in ('map->odom', 'odom->base_link', 'map->base_link'):
    v = [r[key] for r in tf if isinstance(r.get(key), list)]
    if not v:
        print(f"{key}: unavailable ({tf[0].get(key) if tf else 'no samples'})")
        continue
    x, y, yw = [a[0] for a in v], [a[1] for a in v], unwrap([a[2] for a in v])
    print(f"{key}: n={len(v)} start ({x[0]:.3f}, {y[0]:.3f}, {math.degrees(yw[0]):.2f} deg); "
          f"span x {(max(x) - min(x)) * 100:.2f} cm, y {(max(y) - min(y)) * 100:.2f} cm, yaw {math.degrees(max(yw) - min(yw)):.3f} deg")
