import sys, math, time, os, subprocess
sys.path.insert(0, '/home/dinosaur')
import make_waypoints as m
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from action_msgs.srv import CancelGoal
rclpy.init(); n = rclpy.create_node('watch'); s = {}
n.create_subscription(NavSatFix, '/gnss', lambda x: s.update(g=x), qos_profile_sensor_data)
n.create_subscription(Odometry, '/odometry/global', lambda x: s.update(o=x), 10)
n.create_subscription(Twist, '/cmd_vel', lambda x: s.update(c=x), 10)
cli = n.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
def stop(why):
    print('!! STOPPING:', why, flush=True)
    for pid in subprocess.run("ps -eo pid,args | awk '/lib\\/avros_navigation\\/mission_manager/ && !/awk/ {print $1}'", shell=True, capture_output=True, text=True).stdout.split():
        os.kill(int(pid), 15)
    if cli.wait_for_service(timeout_sec=2.0): cli.call_async(CancelGoal.Request())
    t = time.time()
    while time.time() - t < 2: rclpy.spin_once(n, timeout_sec=0.1)
t0 = time.time()
while 'g' not in s and time.time() - t0 < 10: rclpy.spin_once(n, timeout_sec=0.1)
g0 = s['g']; la0, lo0 = g0.latitude, g0.longitude
tgt = (34.059554, -117.821200); te, tn = m.to_enu(tgt[0], tgt[1], la0, lo0); need = math.degrees(math.atan2(tn, te))
dur = float(sys.argv[1]); last = 0; stopped = False
while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.1)
    if time.time() - last < 3 or 'g' not in s: continue
    last = time.time(); g = s['g']; e, nn = m.to_enu(g.latitude, g.longitude, la0, lo0); d = math.hypot(e, nn)
    o = s.get('o'); c = s.get('c'); q = o.pose.pose.orientation if o else None
    yaw = math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))) if q else float('nan')
    course = math.degrees(math.atan2(nn, e)) if d > 1.0 else float('nan')
    err = (course - need + 180) % 360 - 180 if d > 1.0 else float('nan')
    print(f"t{time.time()-t0:4.0f}s moved {d:5.2f} m  GPS course {course:6.1f}  needed {need:5.1f}  err {err:6.1f}  map yaw {yaw:6.1f}  yaw-course {((yaw-course+180)%360-180) if d>1.0 else float('nan'):6.1f}  cmd v={c.linear.x if c else 0:4.2f} w={c.angular.z if c else 0:5.2f}  rtk={g.status.status}", flush=True)
    if d > 2.5 and abs(err) > 60 and not stopped:
        stopped = True; stop(f'GPS course {course:.0f} deg vs needed {need:.0f} deg (error {err:.0f}) after {d:.1f} m'); break
