#!/usr/bin/env python3
"""Capture the live locked path + live cone map; report waypoint clearance to cones."""
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from mfe_msgs.msg import Cone, Track


class Probe(Node):
    def __init__(self):
        super().__init__('locked_path_probe')
        self.path = None
        self.cones = None
        self.create_subscription(Path, '/planning/centerline', self._p, 10)
        self.create_subscription(Track, '/ground_truth/track_colored', self._c, 10)

    def _p(self, msg):
        self.path = [(q.pose.position.x, q.pose.position.y) for q in msg.poses]

    def _c(self, msg):
        self.cones = [(c.location.x, c.location.y, c.color) for c in msg.track]


rclpy.init()
n = Probe()
t0 = time.time()
while (n.path is None or n.cones is None) and time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.2)

if not n.path or not n.cones:
    print(f"MISSING: path={bool(n.path)} cones={bool(n.cones)}")
else:
    print(f"path: {len(n.path)} wpts | cones: {len(n.cones)}")
    bad = 0
    worst = []
    for i, (px, py) in enumerate(n.path):
        dmin, cx, cy, col = min(
            (math.hypot(px - x, py - y), x, y, c) for x, y, c in n.cones)
        if dmin < 0.9:
            bad += 1
            worst.append((dmin, i, px, py, col))
    worst.sort()
    print(f"waypoints closer than 0.9 m to a cone: {bad}/{len(n.path)}")
    for d, i, px, py, col in worst[:12]:
        print(f"  wp[{i:3d}] ({px:7.2f},{py:7.2f})  clearance {d:.2f} m  cone color={col}")
    # Also segment spacing stats
    import numpy as np
    p = np.array(n.path)
    seg = np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1]))
    print(f"segment spacing: min {seg.min():.2f}  max {seg.max():.2f}  mean {seg.mean():.2f}")
rclpy.shutdown()
