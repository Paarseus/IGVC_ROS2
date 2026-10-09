#!/usr/bin/env python3
"""Read-only capture of the localization stack for static (robot not moving) analysis.

Subscribes only; publishes nothing. Writes one JSON-lines file per topic plus a
TF sample file into OUT_DIR. Usage (on the Jetson, stack running):

    python3 capture_static.py <seconds> <out_dir>
"""
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformListener
from xsens_mti_ros2_driver.msg import XsStatusWord

SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
OUT = sys.argv[2] if len(sys.argv) > 2 else '/tmp/ctrl_capture'


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class Cap(Node):
    def __init__(self):
        super().__init__('ctrl_stack_capture')
        os.makedirs(OUT, exist_ok=True)
        self.f = {}
        self.clock_now = lambda: self.get_clock().now().nanoseconds * 1e-9
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/gnss', self.on_gnss, qos_profile_sensor_data)
        for t in ('/odometry/gps', '/wheel_odom', '/odometry/filtered', '/odometry/global'):
            self.create_subscription(Odometry, t, lambda m, t=t: self.on_odom(t, m), 20)
        self.create_subscription(XsStatusWord, '/status', self.on_status, 20)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 20)
        self.create_subscription(Float32MultiArray, '/avros/wheel_debug', self.on_wdbg, 50)
        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, self)
        self.create_timer(0.2, self.on_tf)

    def w(self, name, rec):
        if name not in self.f:
            self.f[name] = open(os.path.join(OUT, name.strip('/').replace('/', '_') + '.jsonl'), 'w')
        rec['t_rx'] = self.clock_now()
        self.f[name].write(json.dumps(rec) + '\n')

    def on_imu(self, m):
        self.w('/imu/data', {
            't': stamp(m.header), 'frame': m.header.frame_id, 'yaw': yaw(m.orientation),
            'wx': m.angular_velocity.x, 'wy': m.angular_velocity.y, 'wz': m.angular_velocity.z,
            'ax': m.linear_acceleration.x, 'ay': m.linear_acceleration.y, 'az': m.linear_acceleration.z,
            'ocov': list(m.orientation_covariance), 'wcov': list(m.angular_velocity_covariance),
            'acov': list(m.linear_acceleration_covariance)})

    def on_gnss(self, m):
        self.w('/gnss', {'t': stamp(m.header), 'frame': m.header.frame_id, 'status': m.status.status,
                         'lat': m.latitude, 'lon': m.longitude, 'alt': m.altitude,
                         'cov': list(m.position_covariance)})

    def on_odom(self, topic, m):
        p, tw = m.pose.pose, m.twist.twist
        self.w(topic, {'t': stamp(m.header), 'frame': m.header.frame_id, 'child': m.child_frame_id,
                       'x': p.position.x, 'y': p.position.y, 'yaw': yaw(p.orientation),
                       'vx': tw.linear.x, 'vy': tw.linear.y, 'wz': tw.angular.z,
                       'pcov': [m.pose.covariance[i] for i in (0, 7, 35)],
                       'tcov': [m.twist.covariance[i] for i in (0, 7, 35)]})

    def on_status(self, m):
        self.w('/status', {'rtk': m.rtk_status, 'filter_valid': m.filter_valid, 'gnss_fix': m.gnss_fix,
                           'filter_mode': m.filter_mode})

    def on_cmd(self, m):
        self.w('/cmd_vel', {'vx': m.linear.x, 'wz': m.angular.z})

    def on_wdbg(self, m):
        d = list(m.data)
        self.w('/avros/wheel_debug', {'l_cmd': d[0], 'r_cmd': d[1], 'l_meas': d[2], 'r_meas': d[3],
                                      'l_pos': d[4], 'r_pos': d[5], 'estop': d[15]})

    def on_tf(self):
        rec = {}
        for parent, child in (('map', 'odom'), ('odom', 'base_link'), ('map', 'base_link')):
            try:
                tr = self.tfb.lookup_transform(parent, child, rclpy.time.Time())
                t = tr.transform
                rec[f'{parent}->{child}'] = [t.translation.x, t.translation.y, yaw(t.rotation),
                                             stamp(tr.header)]
            except Exception as e:  # noqa: BLE001 - record and continue
                rec[f'{parent}->{child}'] = str(e)[:80]
        self.w('tf', rec)


def main():
    rclpy.init()
    n = Cap()
    t0 = time.time()
    while time.time() - t0 < SECS:
        rclpy.spin_once(n, timeout_sec=0.05)
    for f in n.f.values():
        f.close()
    print('wrote', sorted(n.f))


if __name__ == '__main__':
    main()
