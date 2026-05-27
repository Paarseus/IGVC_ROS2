#!/usr/bin/env python3
"""Real-time yaw divergence monitor.

Subscribes to /imu/data, /odometry/filtered, /wheel_odom, and (if
available) /zed_front/zed_node/odom. Every 500 ms prints a one-line
table showing accumulated Δyaw since the monitor started, plus
pairwise differences.

Run in a 4th terminal during the field session. Catches IMU stuck-bias
or sensor failures live (before you've wasted a drive on a bad bag).

Reset baseline at any time with Ctrl+\\ (SIGQUIT).

Usage on the Jetson:
    python3 scripts/live_yaw_monitor.py
"""
import math
import signal
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class LiveYawMonitor(Node):
    def __init__(self):
        super().__init__('live_yaw_monitor')
        self.imu_yaw0 = None
        self.ekf_yaw0 = None
        self.wheel_yaw_acc = 0.0
        self.wheel_t_prev = None
        self.zed_yaw0 = None

        self.imu_yaw = None
        self.ekf_yaw = None
        self.zed_yaw = None

        self.imu_wz = 0.0
        self.ekf_wz = 0.0
        self.wheel_wz = 0.0
        self.zed_wz = 0.0

        self.create_subscription(Imu, '/imu/data', self.on_imu, 50)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 self.on_ekf, 50)
        self.create_subscription(Odometry, '/wheel_odom',
                                 self.on_wheel, 50)
        self.create_subscription(Odometry, '/zed_front/zed_node/odom',
                                 self.on_zed, 50)

        self.create_timer(0.5, self.print_row)
        signal.signal(signal.SIGQUIT, self.reset)

        self.t_start = self.get_clock().now().nanoseconds / 1e9
        self.print_header()

    def reset(self, *_):
        self.imu_yaw0 = None
        self.ekf_yaw0 = None
        self.wheel_yaw_acc = 0.0
        self.wheel_t_prev = None
        self.zed_yaw0 = None
        self.t_start = self.get_clock().now().nanoseconds / 1e9
        print('\n[reset baseline]\n')
        self.print_header()

    def print_header(self):
        print('# Live yaw monitor — Δyaw since launch (deg), vyaw (rad/s)')
        print(f'{"t":>6s}  '
              f'{"IMU_Δ":>8s} {"EKF_Δ":>8s} {"wheel_Δ":>8s} {"ZED_Δ":>8s}  '
              f'{"|IMU-EKF|":>9s} {"|IMU-wheel|":>11s} {"|IMU-ZED|":>9s}  '
              f'{"IMU_wz":>7s} {"wheel_wz":>9s}')
        sys.stdout.flush()

    def on_imu(self, msg):
        y = yaw_from_quat(msg.orientation)
        if self.imu_yaw0 is None:
            self.imu_yaw0 = y
        self.imu_yaw = y
        self.imu_wz = msg.angular_velocity.z

    def on_ekf(self, msg):
        y = yaw_from_quat(msg.pose.pose.orientation)
        if self.ekf_yaw0 is None:
            self.ekf_yaw0 = y
        self.ekf_yaw = y
        self.ekf_wz = msg.twist.twist.angular.z

    def on_wheel(self, msg):
        t = (msg.header.stamp.sec +
             msg.header.stamp.nanosec / 1e9)
        wz = msg.twist.twist.angular.z
        if self.wheel_t_prev is not None:
            dt = t - self.wheel_t_prev
            if 0 < dt < 1.0:
                self.wheel_yaw_acc += wz * dt
        self.wheel_t_prev = t
        self.wheel_wz = wz

    def on_zed(self, msg):
        y = yaw_from_quat(msg.pose.pose.orientation)
        if self.zed_yaw0 is None:
            self.zed_yaw0 = y
        self.zed_yaw = y
        self.zed_wz = msg.twist.twist.angular.z

    def print_row(self):
        now = self.get_clock().now().nanoseconds / 1e9
        t = now - self.t_start

        def d(y, y0):
            if y is None or y0 is None:
                return float('nan')
            return math.degrees(wrap_pi(y - y0))

        imu_d = d(self.imu_yaw, self.imu_yaw0)
        ekf_d = d(self.ekf_yaw, self.ekf_yaw0)
        wheel_d = math.degrees(self.wheel_yaw_acc)
        zed_d = d(self.zed_yaw, self.zed_yaw0)

        def gap(a, b):
            if math.isnan(a) or math.isnan(b):
                return float('nan')
            return abs(a - b)

        imu_ekf = gap(imu_d, ekf_d)
        imu_wheel = gap(imu_d, wheel_d)
        imu_zed = gap(imu_d, zed_d)

        def fmt(v, w):
            if math.isnan(v):
                return f'{"--":>{w}s}'
            return f'{v:>{w}.2f}'

        line = (f'{t:>6.1f}  '
                f'{fmt(imu_d,8)} {fmt(ekf_d,8)} {fmt(wheel_d,8)} {fmt(zed_d,8)}  '
                f'{fmt(imu_ekf,9)} {fmt(imu_wheel,11)} {fmt(imu_zed,9)}  '
                f'{self.imu_wz:>7.3f} {self.wheel_wz:>9.3f}')
        print(line)
        sys.stdout.flush()


def main():
    rclpy.init()
    n = LiveYawMonitor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
