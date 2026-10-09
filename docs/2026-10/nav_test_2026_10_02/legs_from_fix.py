import sys, math, time
sys.path.insert(0, '/home/dinosaur')
import make_waypoints as m
import rclpy
from tf2_ros import Buffer, TransformListener
lat, lon, status, sd = m.read_gnss()
tgt = (34.059554, -117.821200)
e, n = m.to_enu(tgt[0], tgt[1], lat, lon)
dist = math.hypot(e, n); brg = math.degrees(math.atan2(e, n)) % 360
rclpy.init(); node = rclpy.create_node('y'); b = Buffer(); TransformListener(b, node)
t = time.time()
while time.time() - t < 5: rclpy.spin_once(node, timeout_sec=0.1)
q = b.lookup_transform('map', 'base_link', rclpy.time.Time()).transform.rotation
yaw = math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
need = math.degrees(math.atan2(n, e))
d = (need - yaw + 180) % 360 - 180
print(f"fix {lat:.7f},{lon:.7f} status {status} sigma {sd:.2f} m")
print(f"target {n:.1f} m N, {e:.1f} m E = {dist:.1f} m, bearing {brg:.0f} deg from north")
print(f"robot map yaw {yaw:.0f} deg, needed {need:.0f} deg -> must turn {d:+.0f} deg (+ = left)")
first = 8.0; rest = dist - first; k = max(1, math.ceil(rest / 26.0))
dists = [first + rest * i / k for i in range(1, k + 1)]
pts = [m.from_enu(e * x / dist, n * x / dist, lat, lon) for x in [first] + dists]
with open('/home/dinosaur/nav_tests/waypoints_navtest2.yaml', 'w') as f:
    f.write(f"# robot start {lat:.7f},{lon:.7f} -> {tgt[0]},{tgt[1]} ({dist:.1f} m); first leg {first:.0f} m as heading check\nwaypoints:\n")
    for p in pts: f.write("  - {lat: %.8f, lon: %.8f}\n" % p)
print("legs (m from start):", [round(x, 1) for x in [first] + dists]); print(open('/home/dinosaur/nav_tests/waypoints_navtest2.yaml').read())
