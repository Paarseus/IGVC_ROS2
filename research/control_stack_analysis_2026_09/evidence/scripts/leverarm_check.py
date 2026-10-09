#!/usr/bin/env python3
"""Static check of which point /gnss refers to.

Compares the raw receiver position (/gnss, NavSatFix) with the MTi's fused,
lever-arm-corrected position (/filter/positionlla, Vector3Stamped: x=lat,
y=lon, z=alt) while the robot is stationary. If /gnss is the antenna position
and the lever arm is [0.74, 0, 0] m in the IMU frame, the vector
filter -> gnss should be ~0.74 m long and point along the vehicle heading
(IMU yaw, ENU). Read-only. Usage: leverarm_check.py <seconds>
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix

SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
gnss, filt, yaws = [], [], []


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    n = rclpy.create_node('leverarm_check')
    n.create_subscription(NavSatFix, '/gnss', lambda m: gnss.append((m.latitude, m.longitude)), qos_profile_sensor_data)
    n.create_subscription(Vector3Stamped, '/filter/positionlla', lambda m: filt.append((m.vector.x, m.vector.y)), qos_profile_sensor_data)
    n.create_subscription(Imu, '/imu/data', lambda m: yaws.append(yaw(m.orientation)), qos_profile_sensor_data)
    t0 = time.time()
    while time.time() - t0 < SECS:
        rclpy.spin_once(n, timeout_sec=0.05)
    if not (gnss and filt and yaws):
        print(f'missing data: gnss {len(gnss)} filter {len(filt)} imu {len(yaws)}')
        return
    lat0 = sum(g[0] for g in gnss) / len(gnss)
    k = 111320.0
    ce = k * math.cos(math.radians(lat0))
    ge = sum(g[1] for g in gnss) / len(gnss) * ce
    gn = sum(g[0] for g in gnss) / len(gnss) * k
    fe = sum(f[1] for f in filt) / len(filt) * ce
    fn = sum(f[0] for f in filt) / len(filt) * k
    de, dn = ge - fe, gn - fn
    y = math.atan2(sum(math.sin(a) for a in yaws), sum(math.cos(a) for a in yaws))
    fwd = de * math.cos(y) + dn * math.sin(y)       # component along heading (ENU yaw)
    left = -de * math.sin(y) + dn * math.cos(y)
    print(f'samples: gnss {len(gnss)}, filter {len(filt)}, imu {len(yaws)}')
    print(f'IMU yaw (ENU) {math.degrees(y):.1f} deg')
    print(f'offset filter -> gnss: east {de:+.3f} m, north {dn:+.3f} m, length {math.hypot(de, dn):.3f} m, '
          f'bearing (ENU) {math.degrees(math.atan2(dn, de)):.1f} deg')
    print(f'in vehicle frame: forward {fwd:+.3f} m, left {left:+.3f} m   (configured lever arm: +0.740 forward)')


if __name__ == '__main__':
    main()
