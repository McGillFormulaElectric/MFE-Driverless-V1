#!/usr/bin/env python3
"""Diagnostic: sample /planning/centerline vs car pose for N seconds.

For each sample prints:
  car position/heading, path waypoint count,
  cross-track error (car -> nearest waypoint),
  forward extent (how far the path reaches AHEAD of the car along its heading),
  behind extent (how far it reaches behind).
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Path, Odometry


class Diag(Node):
    def __init__(self):
        super().__init__('diag_path_probe')
        self.path = None
        self.odom = None
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Path, '/planning/centerline', self._path_cb, reliable)
        self.create_subscription(Odometry, '/ground_truth/state_odom', self._odom_cb, best_effort)

    def _path_cb(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        v = msg.twist.twist.linear
        self.odom = (p.x, p.y, yaw, math.hypot(v.x, v.y))


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    interval = 1.0
    rclpy.init()
    node = Diag()
    t0 = time.time()
    next_print = t0
    print(f"{'t':>5} {'car x':>7} {'car y':>7} {'yaw°':>6} {'v':>5} {'wpts':>4} "
          f"{'xtrack':>7} {'ahead':>6} {'behind':>7}")
    while time.time() - t0 < duration:
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.time()
        if now < next_print:
            continue
        next_print = now + interval
        t = now - t0
        if node.odom is None:
            print(f"{t:5.1f}  no odom yet")
            continue
        cx, cy, yaw, v = node.odom
        if not node.path:
            print(f"{t:5.1f} {cx:7.2f} {cy:7.2f} {math.degrees(yaw):6.1f} {v:5.2f}    0  NO PATH")
            continue
        hx, hy = math.cos(yaw), math.sin(yaw)
        xtrack = min(math.hypot(px - cx, py - cy) for px, py in node.path)
        fwd = [ (px - cx) * hx + (py - cy) * hy for px, py in node.path ]
        ahead = max(fwd)
        behind = min(fwd)
        print(f"{t:5.1f} {cx:7.2f} {cy:7.2f} {math.degrees(yaw):6.1f} {v:5.2f} "
              f"{len(node.path):4d} {xtrack:7.2f} {ahead:6.1f} {behind:7.1f}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
