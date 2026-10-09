import sys, statistics as st
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
T = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
acc, cmd = [], []
while r.has_next():
    tp, d, t = r.read_next(); t *= 1e-9
    if tp == '/imu/data':
        m = deserialize_message(d, T[tp]); acc.append((t, m.linear_acceleration.x, m.linear_acceleration.y))
    elif tp == '/avros/wheel_debug':
        m = deserialize_message(d, T[tp]); cmd.append((t, m.data[8]))   # v_slewed
ramp = [t for t, v in cmd if 0.02 < v < 0.38]          # speeding up forward
if not ramp:
    sys.exit('no forward ramp found')
t0, t1 = min(ramp), max(t for t in ramp if t < min(ramp) + 2.0)
rest = [a for t, a, _ in acc if t < t0 - 0.2]
up = [a for t, a, _ in acc if t0 + 0.2 <= t <= t1]
rest_y = [b for t, _, b in acc if t < t0 - 0.2]
up_y = [b for t, _, b in acc if t0 + 0.2 <= t <= t1]
print(f'{sys.argv[1].split("/")[-1]}: while speeding up forward, IMU x-axis acceleration changed by '
      f'{st.mean(up) - st.mean(rest):+.3f} m/s^2 (y-axis {st.mean(up_y) - st.mean(rest_y):+.3f}); expected about +0.3 if mounted facing forward')
