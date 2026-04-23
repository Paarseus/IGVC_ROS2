"""Actuator bridge node: cmd_vel / ActuatorCommand -> Teensy serial -> SparkMAX.

Diff-drive kinematics for an AndyMark Raptor track-drive chassis:
    motor RPM -> ground m/s via 12.75:1 gearbox + 20T drive pulley
    commanded (linear, angular) -> (L_rpm, R_rpm) via diff-drive inverse

Heading-hold: when commanded angular velocity is near zero, IMU yaw is
locked and a proportional correction is applied to ω. Makes the robot
drive straight under teleop or any cmd_vel source without requiring
Nav2 path-tracking feedback.

Gyro-stabilized turns: when commanded angular velocity is non-zero,
IMU yaw rate is used as feedback to close the loop on ω so "turn at
0.5 rad/s" produces 0.5 rad/s regardless of surface friction.

Subscribes:
  /cmd_vel                  geometry_msgs/Twist      (Nav2, teleop)
  /avros/actuator_command   avros_msgs/ActuatorCommand (webui direct)
  /imu/data                 sensor_msgs/Imu          (Xsens yaw + rate)

Publishes:
  /avros/actuator_state     avros_msgs/ActuatorState @ 20 Hz
  /wheel_odom               nav_msgs/Odometry @ 50 Hz (for EKF fusion)
"""

import math
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from avros_msgs.msg import ActuatorCommand, ActuatorState


def yaw_from_quaternion(q) -> float:
    """Extract yaw (Z rotation) from a geometry_msgs.Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a: float) -> float:
    """Wrap radians to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class ActuatorNode(Node):
    """Diff-drive bridge: ROS2 commands <-> Teensy serial protocol."""

    def __init__(self):
        super().__init__('actuator_node')

        # ---- parameters ----
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('track_width_m', 0.7366)       # 29 inches
        self.declare_parameter('m_per_motor_rev', 0.01994)    # Raptor + TBMini 12.75:1 + 20T pulley
        self.declare_parameter('max_linear_mps', 1.5)
        self.declare_parameter('max_angular_rps', 1.0)
        self.declare_parameter('heading_hold_deadband', 0.05) # rad/s threshold
        self.declare_parameter('heading_kp', 1.5)             # heading-hold P gain
        self.declare_parameter('yaw_rate_kp', 0.3)            # gyro-stabilized turn P gain
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('state_pub_rate_hz', 20.0)
        # SparkMAX PID gains to set on startup (from Phase 6 tuning)
        self.declare_parameter('kFF', 0.000197)
        self.declare_parameter('kP', 0.0004)
        self.declare_parameter('kI', 0.0)
        self.declare_parameter('kD', 0.0)
        # Odom frame names
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        p = self.get_parameter
        self._port = p('serial_port').value
        self._baud = p('serial_baud').value
        self._track_w = p('track_width_m').value
        self._m_per_rev = p('m_per_motor_rev').value
        self._max_v = p('max_linear_mps').value
        self._max_w = p('max_angular_rps').value
        self._hh_deadband = p('heading_hold_deadband').value
        self._heading_kp = p('heading_kp').value
        self._yaw_rate_kp = p('yaw_rate_kp').value
        self._cmd_timeout = p('cmd_timeout_s').value
        self._odom_frame = p('odom_frame').value
        self._base_frame = p('base_frame').value

        control_rate = p('control_rate_hz').value
        state_rate = p('state_pub_rate_hz').value

        # ---- serial ----
        self._serial = serial.Serial(self._port, self._baud, timeout=0.1)
        time.sleep(0.3)
        self._serial.reset_input_buffer()
        self._serial_lock = threading.Lock()
        self.get_logger().info(f'Serial open: {self._port} @ {self._baud}')

        # Set PID gains on the Teensy (pushes to both SparkMAXes)
        for name, val in [('KF', p('kFF').value), ('KP', p('kP').value),
                          ('KI', p('kI').value), ('KD', p('kD').value)]:
            self._serial_write(f'{name}{val}')
            time.sleep(0.2)
        self.get_logger().info(
            f'SparkMAX gains set: kFF={p("kFF").value} kP={p("kP").value} '
            f'kI={p("kI").value} kD={p("kD").value}'
        )

        # ---- state ----
        # Command targets (set by callbacks; read by control loop)
        self._target_v = 0.0       # m/s
        self._target_w = 0.0       # rad/s
        self._estop = False
        self._last_cmd_vel_t = None           # rclpy Time or None
        self._last_actuator_cmd_t = None

        # Heading-hold state
        self._heading_locked = False
        self._heading_target = 0.0
        self._current_yaw = 0.0
        self._current_yaw_rate = 0.0
        self._imu_fresh = False

        # Feedback state (from Teensy E-lines)
        self._l_meas_rpm = 0.0
        self._r_meas_rpm = 0.0
        self._l_meas_pos = 0.0    # motor revolutions, cumulative signed
        self._r_meas_pos = 0.0
        self._l_pos_prev = None
        self._r_pos_prev = None
        self._fb_lock = threading.Lock()

        # Integrated odometry (pose from wheel encoders)
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._odom_last_t = self.get_clock().now()

        # ---- subscribers ----
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(
            ActuatorCommand, '/avros/actuator_command',
            self._on_actuator_cmd, 10
        )
        self.create_subscription(Imu, '/imu/data', self._on_imu, 20)

        # ---- publishers ----
        self._state_pub = self.create_publisher(
            ActuatorState, '/avros/actuator_state', 10
        )
        self._odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)

        # ---- threads / timers ----
        self._running = True
        self._reader_thread = threading.Thread(
            target=self._serial_reader, daemon=True
        )
        self._reader_thread.start()

        self._ctrl_dt = 1.0 / control_rate
        self.create_timer(self._ctrl_dt, self._control_loop)
        self.create_timer(1.0 / state_rate, self._publish_state)

        self.get_logger().info('Actuator node ready — diff-drive with heading-hold')

    # ------------------------------------------------------------- callbacks
    def _on_cmd_vel(self, msg: Twist):
        self._target_v = max(-self._max_v, min(self._max_v, msg.linear.x))
        self._target_w = max(-self._max_w, min(self._max_w, msg.angular.z))
        self._last_cmd_vel_t = self.get_clock().now()

    def _on_actuator_cmd(self, msg: ActuatorCommand):
        self._last_actuator_cmd_t = self.get_clock().now()
        if msg.estop:
            self._estop = True
            self._target_v = 0.0
            self._target_w = 0.0
            self.get_logger().warn('E-STOP via actuator_command')
            return
        self._estop = False
        # Map webui throttle/brake/steer -> (v, ω)
        v = (msg.throttle - msg.brake) * self._max_v
        w = msg.steer * self._max_w
        self._target_v = max(-self._max_v, min(self._max_v, v))
        self._target_w = max(-self._max_w, min(self._max_w, w))

    def _on_imu(self, msg: Imu):
        self._current_yaw = yaw_from_quaternion(msg.orientation)
        self._current_yaw_rate = msg.angular_velocity.z
        self._imu_fresh = True

    # ------------------------------------------------------------- control
    def _control_loop(self):
        now = self.get_clock().now()

        # Command freshness (priority: actuator_command > cmd_vel > stop)
        has_actuator = (
            self._last_actuator_cmd_t is not None
            and (now - self._last_actuator_cmd_t).nanoseconds / 1e9 < self._cmd_timeout
        )
        has_cmd_vel = (
            self._last_cmd_vel_t is not None
            and (now - self._last_cmd_vel_t).nanoseconds / 1e9 < self._cmd_timeout
        )

        if self._estop:
            v, w = 0.0, 0.0
        elif has_actuator or has_cmd_vel:
            v = self._target_v
            w = self._target_w
        else:
            v, w = 0.0, 0.0

        # IMU-based filters on ω
        if self._imu_fresh:
            if abs(w) < self._hh_deadband and abs(v) > 0.02:
                # Straight-line intent → heading hold
                if not self._heading_locked:
                    self._heading_target = self._current_yaw
                    self._heading_locked = True
                yaw_err = wrap_angle(self._heading_target - self._current_yaw)
                w = self._heading_kp * yaw_err
                # cap the correction so it never fights the user
                w = max(-self._max_w * 0.5, min(self._max_w * 0.5, w))
            else:
                # Turning → release hold, optionally close loop on ω via IMU
                self._heading_locked = False
                w_err = w - self._current_yaw_rate
                w = w + self._yaw_rate_kp * w_err

        # Diff-drive inverse kinematics
        l_mps = v - w * self._track_w / 2.0
        r_mps = v + w * self._track_w / 2.0
        l_rpm = l_mps / self._m_per_rev * 60.0
        r_rpm = r_mps / self._m_per_rev * 60.0

        # Send setpoint (or S on idle/estop)
        if self._estop or (not has_actuator and not has_cmd_vel and abs(v) < 1e-6 and abs(w) < 1e-6):
            self._serial_write('S')
        else:
            self._serial_write(f'L{l_rpm:.0f} R{r_rpm:.0f}')

    # ------------------------------------------------------------- publishing
    def _publish_state(self):
        now = self.get_clock().now()
        with self._fb_lock:
            l_rpm = self._l_meas_rpm
            r_rpm = self._r_meas_rpm

        # Legacy ActuatorState contract (for webui compatibility)
        msg = ActuatorState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._base_frame
        msg.estop = self._estop
        # Approximate throttle/brake/steer from measured wheel RPMs so webui
        # UI reflects reality, not command
        avg_mps = ((l_rpm + r_rpm) / 2.0) * self._m_per_rev / 60.0
        diff_mps = ((r_rpm - l_rpm)) * self._m_per_rev / 60.0
        throttle = max(0.0, min(1.0, avg_mps / self._max_v))
        brake = max(0.0, min(1.0, -avg_mps / self._max_v))
        steer = max(-1.0, min(1.0,
                              (diff_mps / self._track_w) / self._max_w)) if self._max_w > 0 else 0.0
        msg.throttle = throttle
        msg.brake = brake
        msg.steer = steer
        msg.mode = 'D' if not self._estop else 'N'
        msg.watchdog_active = False
        self._state_pub.publish(msg)

        # Integrate + publish wheel odometry
        self._publish_odom(now, l_rpm, r_rpm)

    def _publish_odom(self, now, l_rpm, r_rpm):
        dt = (now - self._odom_last_t).nanoseconds / 1e9
        if dt <= 0 or dt > 0.5:
            self._odom_last_t = now
            return
        self._odom_last_t = now

        l_mps = l_rpm * self._m_per_rev / 60.0
        r_mps = r_rpm * self._m_per_rev / 60.0
        v = (l_mps + r_mps) / 2.0
        w = (r_mps - l_mps) / self._track_w

        self._odom_yaw = wrap_angle(self._odom_yaw + w * dt)
        self._odom_x += v * math.cos(self._odom_yaw) * dt
        self._odom_y += v * math.sin(self._odom_yaw) * dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        odom.pose.pose.orientation.z = math.sin(self._odom_yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._odom_yaw / 2.0)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self._odom_pub.publish(odom)

    # ------------------------------------------------------------- serial I/O
    def _serial_write(self, line: str):
        try:
            with self._serial_lock:
                self._serial.write((line + '\n').encode('ascii'))
        except Exception as e:
            self.get_logger().error(f'serial write failed: {e}')

    def _serial_reader(self):
        """Background thread: read E lines, update measured state."""
        import re
        E_RE = re.compile(r"E L(-?\d+) (-?[\d.]+) R(-?\d+) (-?[\d.]+)")
        buf = ''
        while self._running:
            try:
                chunk = self._serial.read(256).decode('ascii', errors='replace')
                if not chunk:
                    continue
                buf += chunk
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    m = E_RE.match(line.strip())
                    if m:
                        with self._fb_lock:
                            self._l_meas_rpm = float(m.group(1))
                            self._l_meas_pos = float(m.group(2))
                            self._r_meas_rpm = float(m.group(3))
                            self._r_meas_pos = float(m.group(4))
            except Exception as e:
                self.get_logger().error(f'serial reader: {e}')
                time.sleep(0.1)

    # ------------------------------------------------------------- shutdown
    def destroy_node(self):
        self.get_logger().info('shutdown — sending S')
        self._running = False
        try:
            self._serial_write('S')
            time.sleep(0.1)
            self._serial.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
