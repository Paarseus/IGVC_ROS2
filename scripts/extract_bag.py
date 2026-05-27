"""Dump a ros2 bag (sqlite3 db3) to per-topic CSVs.

General-purpose extractor for IGVC_ROS2 telemetry analysis. Handles the
topics required by the Phase 2-6 empirical research plan (see
docs/phase_telemetry_recipe.md):

  /avros/wheel_debug        Float32MultiArray (16 fields)
  /avros/actuator_command   ActuatorCommand
  /avros/actuator_state     ActuatorState
  /wheel_odom               Odometry
  /odometry/filtered        Odometry (local EKF)
  /odometry/global          Odometry (global EKF -- deprecated after Phase B)
  /cmd_vel                  Twist (MPPI controller output)
  /imu/data                 Imu (Xsens MTi-680G)
  /gnss                     NavSatFix
  /tf, /tf_static           TFMessage

Run on the Jetson (or any machine with ROS2 Humble + avros_msgs sourced):

  source /opt/ros/humble/setup.bash
  source ~/IGVC/install/setup.bash
  python3 scripts/extract_bag.py /path/to/bagdir/bag_0.db3 /path/to/out_csv

If the bag has been split across multiple .db3 shards, pass the directory
instead and the script will concatenate them in lexical order.
"""
import csv
import math
import sqlite3
import sys
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# /avros/wheel_debug schema -- mirrors actuator_node.py:275-284
WHEEL_DEBUG_COLS = [
    'L_cmd_rpm', 'R_cmd_rpm',
    'L_meas_rpm', 'R_meas_rpm',
    'L_pos_rev', 'R_pos_rev',
    'v_target', 'w_target',
    'v_slewed', 'w_slewed',
    'v_after_imu', 'w_after_imu',
    'yaw', 'yaw_rate',
    'heading_locked', 'estop',
]


def yaw_from_quat(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def write_odometry(writer, rows, t0):
    writer.writerow(['t_rel', 'frame_id', 'child_frame', 'x', 'y', 'z',
                     'yaw', 'qx', 'qy', 'qz', 'qw',
                     'vx', 'vy', 'wz'])
    cls = get_message('nav_msgs/msg/Odometry')
    for ts, blob in rows:
        msg = deserialize_message(blob, cls)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.twist.twist
        writer.writerow([
            (ts - t0) / 1e9,
            msg.header.frame_id, msg.child_frame_id,
            p.x, p.y, p.z,
            yaw_from_quat(q.x, q.y, q.z, q.w),
            q.x, q.y, q.z, q.w,
            t.linear.x, t.linear.y, t.angular.z,
        ])


def write_twist(writer, rows, t0):
    writer.writerow(['t_rel', 'vx', 'vy', 'wz'])
    cls = get_message('geometry_msgs/msg/Twist')
    for ts, blob in rows:
        msg = deserialize_message(blob, cls)
        writer.writerow([(ts - t0) / 1e9,
                         msg.linear.x, msg.linear.y, msg.angular.z])


def write_imu(writer, rows, t0):
    writer.writerow(['t_rel', 'yaw', 'qx', 'qy', 'qz', 'qw',
                     'wx', 'wy', 'wz', 'ax', 'ay', 'az'])
    cls = get_message('sensor_msgs/msg/Imu')
    for ts, blob in rows:
        msg = deserialize_message(blob, cls)
        q = msg.orientation
        w = msg.angular_velocity
        a = msg.linear_acceleration
        writer.writerow([(ts - t0) / 1e9,
                         yaw_from_quat(q.x, q.y, q.z, q.w),
                         q.x, q.y, q.z, q.w,
                         w.x, w.y, w.z, a.x, a.y, a.z])


def write_navsat(writer, rows, t0):
    writer.writerow(['t_rel', 'lat', 'lon', 'alt', 'status', 'service',
                     'cov_e', 'cov_n', 'cov_u'])
    cls = get_message('sensor_msgs/msg/NavSatFix')
    for ts, blob in rows:
        msg = deserialize_message(blob, cls)
        writer.writerow([(ts - t0) / 1e9,
                         msg.latitude, msg.longitude, msg.altitude,
                         msg.status.status, msg.status.service,
                         msg.position_covariance[0],
                         msg.position_covariance[4],
                         msg.position_covariance[8]])


def write_tf(writer, rows, t0):
    """TF transforms — flattened: one row per transform (most TFMessages
    contain a single transform, but loop in case)."""
    writer.writerow(['t_rel', 'parent', 'child', 'x', 'y', 'z', 'yaw'])
    cls = get_message('tf2_msgs/msg/TFMessage')
    for ts, blob in rows:
        msg = deserialize_message(blob, cls)
        for tr in msg.transforms:
            t = tr.transform.translation
            q = tr.transform.rotation
            writer.writerow([(ts - t0) / 1e9,
                             tr.header.frame_id, tr.child_frame_id,
                             t.x, t.y, t.z,
                             yaw_from_quat(q.x, q.y, q.z, q.w)])


def write_wheel_debug(writer, rows, t0):
    writer.writerow(['t_rel'] + WHEEL_DEBUG_COLS)
    cls = get_message('std_msgs/msg/Float32MultiArray')
    for ts, blob in rows:
        msg = deserialize_message(blob, cls)
        if len(msg.data) != 16:
            continue
        writer.writerow([(ts - t0) / 1e9] + list(msg.data))


def write_actuator_command(writer, rows, t0):
    writer.writerow(['t_rel', 'estop', 'throttle', 'brake', 'steer', 'mode'])
    cls = get_message('avros_msgs/msg/ActuatorCommand')
    for ts, blob in rows:
        msg = deserialize_message(blob, cls)
        writer.writerow([(ts - t0) / 1e9, int(msg.estop),
                         msg.throttle, msg.brake, msg.steer, msg.mode])


def write_actuator_state(writer, rows, t0):
    writer.writerow(['t_rel', 'estop', 'throttle', 'brake', 'steer', 'mode',
                     'watchdog'])
    cls = get_message('avros_msgs/msg/ActuatorState')
    for ts, blob in rows:
        msg = deserialize_message(blob, cls)
        writer.writerow([(ts - t0) / 1e9, int(msg.estop),
                         msg.throttle, msg.brake, msg.steer, msg.mode,
                         int(msg.watchdog_active)])


# Dispatch table: ROS topic name -> writer function. Anything not listed
# uses a generic dump (timestamp + raw msg fields where possible).
HANDLERS = {
    '/avros/wheel_debug':       write_wheel_debug,
    '/avros/actuator_command':  write_actuator_command,
    '/avros/actuator_state':    write_actuator_state,
    '/wheel_odom':              write_odometry,
    '/odometry/filtered':       write_odometry,
    '/odometry/global':         write_odometry,
    '/odometry/gps':            write_odometry,
    '/cmd_vel':                 write_twist,
    '/imu/data':                write_imu,
    '/gnss':                    write_navsat,
    '/tf':                      write_tf,
    '/tf_static':               write_tf,
}


def open_db(path: Path):
    """Open a single .db3, OR a bag dir containing one or more .db3 shards."""
    if path.is_file():
        return [sqlite3.connect(path)]
    shards = sorted(path.glob('*.db3'))
    if not shards:
        raise SystemExit(f'no .db3 files under {path}')
    return [sqlite3.connect(p) for p in shards]


def collect_rows(cons, topic_id):
    """Pull (timestamp, data) rows for a topic_id across all shards in order."""
    rows = []
    for con in cons:
        rows.extend(con.cursor().execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topic_id,),
        ).fetchall())
    return rows


def main():
    if len(sys.argv) < 2:
        raise SystemExit(
            f'usage: {sys.argv[0]} <bag.db3 | bagdir> [out_csv_dir]'
        )
    bag = Path(sys.argv[1])
    out = Path(sys.argv[2] if len(sys.argv) > 2 else '/tmp/bag_csv')
    out.mkdir(exist_ok=True, parents=True)

    cons = open_db(bag)
    # topics table is identical across shards of the same bag
    topics = {tid: (name, typ) for tid, name, typ in
              cons[0].cursor().execute("SELECT id, name, type FROM topics").fetchall()}
    print(f'topics ({len(topics)}):')
    for tid, (n, t) in topics.items():
        print(f'  [{tid}] {n} ({t})')

    # Compute t0 across all shards so timestamps are zero-anchored
    t0 = min(
        con.cursor().execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
        for con in cons
    )
    print(f't0 = {t0}')

    for tid, (name, _typ) in topics.items():
        safe = name.strip('/').replace('/', '_') or 'root'
        rows = collect_rows(cons, tid)
        if not rows:
            print(f'  {name}: 0 msgs (skipping)')
            continue
        csv_path = out / f'{safe}.csv'
        handler = HANDLERS.get(name)
        if handler is None:
            print(f'  {name}: {len(rows)} msgs -> {csv_path} (UNHANDLED — skipped)')
            continue
        with csv_path.open('w', newline='') as f:
            handler(csv.writer(f), rows, t0)
        print(f'  {name}: {len(rows)} msgs -> {csv_path}')

    print('done.')


if __name__ == '__main__':
    main()
