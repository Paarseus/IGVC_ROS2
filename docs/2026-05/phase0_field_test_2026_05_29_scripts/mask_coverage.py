#!/usr/bin/env python3
"""Sample /perception/front/semantic_mask for N seconds and report the per-class
pixel coverage. High coverage of danger classes (lane_white/barrel/pothole) when
the scene is mostly bare ground = false positives -> phantom costmap obstacles.
"""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class MaskCheck(Node):
    def __init__(self):
        super().__init__('mask_coverage')
        self.frames = []
        self.create_subscription(Image, '/perception/front/semantic_mask', self.cb, 10)

    def cb(self, m):
        a = np.frombuffer(bytes(m.data), dtype=np.uint8)
        self.frames.append((m.height, m.width, a))


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0
    rclpy.init()
    n = MaskCheck()
    t = time.time()
    while time.time() - t < dur:
        rclpy.spin_once(n, timeout_sec=0.2)
    if not n.frames:
        print("NO MASK FRAMES received"); return
    h, w, _ = n.frames[0]
    allpix = np.concatenate([f[2] for f in n.frames])
    tot = allpix.size
    vals, counts = np.unique(allpix, return_counts=True)
    print(f"mask {h}x{w}, frames={len(n.frames)}, total_px={tot}")
    # class ids per class_map: 0=free, 255=unknown; everything else marks the costmap
    marked = 0
    for v, c in zip(vals, counts):
        pct = 100.0 * c / tot
        tag = "free" if v == 0 else ("unknown" if v == 255 else "MARKS-COSTMAP")
        print(f"  class_id {v:3d}: {pct:6.2f}%   {tag}")
        if v != 0 and v != 255:
            marked += c
    print(f"TOTAL marking (non-free, non-unknown): {100.0*marked/tot:.2f}%")
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
