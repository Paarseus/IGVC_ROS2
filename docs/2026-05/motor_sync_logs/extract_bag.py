"""Dump motor_sync_bag to CSV. Run on the Jetson with Humble + avros_msgs sourced."""
import csv
import sqlite3
import sys
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = Path(sys.argv[1] if len(sys.argv) > 1 else '/tmp/motor_sync_bag/motor_sync_bag_0.db3')
OUT = Path(sys.argv[2] if len(sys.argv) > 2 else '/tmp/motor_sync_csv')
OUT.mkdir(exist_ok=True)

con = sqlite3.connect(BAG)
cur = con.cursor()

topics = {tid: (name, typ) for tid, name, typ in
          cur.execute("SELECT id, name, type FROM topics").fetchall()}
print(f'topics: {topics}')

# Fixed labels for /avros/wheel_debug (Float32MultiArray with 16 elements)
DEBUG_COLS = [
    'L_cmd_rpm', 'R_cmd_rpm',
    'L_meas_rpm', 'R_meas_rpm',
    'L_pos_rev', 'R_pos_rev',
    'v_target', 'w_target',
    'v_slewed', 'w_slewed',
    'v_after_imu', 'w_after_imu',
    'yaw', 'yaw_rate',
    'heading_locked', 'estop',
]

# Find t0 across all topics for relative timestamps
t0 = cur.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
print(f't0 = {t0}')

for tid, (name, typ) in topics.items():
    safe = name.strip('/').replace('/', '_')
    out = OUT / f'{safe}.csv'
    msg_cls = get_message(typ)
    rows = cur.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)
    ).fetchall()
    print(f'  {name}: {len(rows)} msgs -> {out}')
    with out.open('w', newline='') as f:
        w = csv.writer(f)
        if name == '/avros/wheel_debug':
            w.writerow(['t_rel'] + DEBUG_COLS)
            for ts, blob in rows:
                msg = deserialize_message(blob, msg_cls)
                w.writerow([(ts - t0) / 1e9] + list(msg.data))
        elif name == '/avros/actuator_command':
            w.writerow(['t_rel', 'estop', 'throttle', 'brake', 'steer', 'mode'])
            for ts, blob in rows:
                msg = deserialize_message(blob, msg_cls)
                w.writerow([(ts - t0) / 1e9, int(msg.estop),
                            msg.throttle, msg.brake, msg.steer, msg.mode])
        elif name == '/avros/actuator_state':
            w.writerow(['t_rel', 'estop', 'throttle', 'brake', 'steer', 'mode', 'watchdog'])
            for ts, blob in rows:
                msg = deserialize_message(blob, msg_cls)
                w.writerow([(ts - t0) / 1e9, int(msg.estop),
                            msg.throttle, msg.brake, msg.steer, msg.mode,
                            int(msg.watchdog_active)])
        elif name == '/wheel_odom':
            w.writerow(['t_rel', 'x', 'y', 'yaw_q_z', 'yaw_q_w', 'vx', 'wz'])
            for ts, blob in rows:
                msg = deserialize_message(blob, msg_cls)
                p = msg.pose.pose
                t = msg.twist.twist
                w.writerow([(ts - t0) / 1e9, p.position.x, p.position.y,
                            p.orientation.z, p.orientation.w,
                            t.linear.x, t.angular.z])

print('done.')
