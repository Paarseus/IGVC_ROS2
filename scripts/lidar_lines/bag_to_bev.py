#!/usr/bin/env python3
"""
Run the LiDAR line detector offline over a bag and write top-down images.

Uses exactly avros_perception.lidar_line_detector.detect_lines (no deskew:
offline there is usually no odom TF; fine for slow drives). For every
--every-th scan it writes <out>/frame_XXXXX.png with three panels, robot at the
centre, +x (forward) up, 5 cm cells:
  left    ground reflectivity (0..--refl-max, grey)
  middle  reflectivity / ring background (1x black .. 4x white)
  right   detected line points (green) over the dim ground
and prints ground reflectivity percentiles per range band, which are the
numbers to tune min_reflectivity / contrast_ratio / contrast_delta from.

  source install/setup.bash
  python3 scripts/lidar_lines/bag_to_bev.py ~/bags/lidar_lines/asphalt_01 \
      --out /tmp/bev --every 10 [--set contrast_ratio=2.5 --set min_reflectivity=25]
"""

import argparse
import os
import time

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

from avros_perception.lidar_line_detector import LineParams, detect_lines, organized_arrays
from avros_perception.lidar_preprocessor import transform_matrix_from_stamped

CELL = 0.05
HALF = 15.0


def _reader(uri, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=''),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r


def static_tf(uri, parent, child):
    r = _reader(uri, ['/tf_static'])
    while r.has_next():
        _, data, _ = r.read_next()
        for tf in deserialize_message(data, TFMessage).transforms:
            if tf.header.frame_id == parent and tf.child_frame_id == child:
                return transform_matrix_from_stamped(tf)
    return None


def raster(xyz, val, shape, reduce='max'):
    n = shape[0]
    px = ((HALF - xyz[:, 0]) / CELL).astype(int)   # +x up
    py = ((HALF - xyz[:, 1]) / CELL).astype(int)   # +y left
    ok = (px >= 0) & (px < n) & (py >= 0) & (py < n)
    img = np.zeros(shape, np.float32)
    np.maximum.at(img, (px[ok], py[ok]), val[ok])
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--out', default='bev_out')
    ap.add_argument('--every', type=int, default=10)
    ap.add_argument('--topic', default='/ouster/points')
    ap.add_argument('--refl-max', type=float, default=100.0)
    ap.add_argument('--set', action='append', default=[], metavar='NAME=VALUE')
    a = ap.parse_args()

    params = LineParams()
    for kv in a.set:
        k, v = kv.split('=', 1)
        setattr(params, k, type(getattr(params, k))(float(v)))

    t_bs = static_tf(a.bag, 'base_link', 'os_sensor')
    if t_bs is None:
        print('no base_link->os_sensor in /tf_static; assuming z=0.67 m, yaw=pi')
        t_bs = np.diag([-1.0, -1.0, 1.0, 1.0])
        t_bs[2, 3] = 0.67
    os.makedirs(a.out, exist_ok=True)

    n = int(2 * HALF / CELL)
    plane = None
    bands = [(1.5, 3), (3, 5), (5, 8), (8, 12), (12, 15)]
    stats = {b: [] for b in bands}
    r = _reader(a.bag, [a.topic])
    i = 0
    ms = []
    while r.has_next():
        _, data, _ = r.read_next()
        msg = deserialize_message(data, PointCloud2)
        arr = organized_arrays(msg)
        xyz = np.stack((arr['x'], arr['y'], arr['z']), axis=2)
        t0 = time.perf_counter()
        res = detect_lines(xyz, arr['reflectivity'], t_bs, params, plane)
        ms.append((time.perf_counter() - t0) * 1e3)
        plane = res.plane
        d = np.hypot(res.ground_xyz[:, 0], res.ground_xyz[:, 1])
        for b in bands:
            sel = (d >= b[0]) & (d < b[1])
            if sel.any():
                stats[b].append(res.ground_refl[sel])
        if i % a.every == 0:
            g = raster(res.ground_xyz, res.ground_refl, (n, n))
            left = np.clip(g / a.refl_max * 255, 0, 255).astype(np.uint8)
            c = raster(res.ground_xyz, res.ground_contrast, (n, n))
            mid = np.clip((c - 1.0) / 3.0 * 255, 0, 255).astype(np.uint8)
            right = cv2.cvtColor((left * 0.4).astype(np.uint8), cv2.COLOR_GRAY2BGR)
            ln = raster(res.lines_xyz, np.ones(res.lines_xyz.shape[0]), (n, n)) > 0
            right[ln] = (0, 255, 0)
            panels = [cv2.cvtColor(left, cv2.COLOR_GRAY2BGR),
                      cv2.cvtColor(mid, cv2.COLOR_GRAY2BGR), right]
            for p in panels:
                cv2.drawMarker(p, (n // 2, n // 2), (0, 0, 255), cv2.MARKER_TRIANGLE_UP, 12, 2)
            img = np.hstack(panels)
            cv2.putText(img, f'scan {i}  lines {res.lines_xyz.shape[0]}  cand {res.candidates}',
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
            cv2.imwrite(os.path.join(a.out, f'frame_{i:05d}.png'), img)
            print(f'scan {i}: lines={res.lines_xyz.shape[0]} candidates={res.candidates} '
                  f'ground={res.ground_xyz.shape[0]} plane={np.round(res.plane, 3)}')
        i += 1

    print(f'\n{i} scans, detect_lines mean {np.mean(ms):.1f} ms, '
          f'p95 {np.percentile(ms, 95):.1f} ms')
    print('ground reflectivity by horizontal distance (p10 / p50 / p90 / p99 / p99.9):')
    for b in bands:
        if stats[b]:
            v = np.concatenate(stats[b])
            pc = np.percentile(v, [10, 50, 90, 99, 99.9]).round(1)
            print(f'  {b[0]:>4}-{b[1]:<4} m  n={v.size:>8}  {pc}')


if __name__ == '__main__':
    main()
