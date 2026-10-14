"""Compares the main planner's centerline (ground-truth cones, drives the car) with the validation planner's (LiDAR detections + GTSAM SLAM).

Both paths are expressed in their own car frame (main: ground-truth pose, validation: SLAM pose) and only points 0-15 m ahead are
compared, so the metric is "how wrong is the path the car would see", independent of the global frame offset.
"""
import csv, math, os, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path, Odometry

QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def to_car(path_xy, pose):
    x, y, th = pose
    d = path_xy - np.array([x, y]); c, s = math.cos(-th), math.sin(-th)
    return np.c_[c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1]]


def dist_to_polyline(pts, line):
    out = []
    for p in pts:
        a, b = line[:-1], line[1:]; ab = b - a
        t = np.clip(((p - a) * ab).sum(1) / np.maximum((ab * ab).sum(1), 1e-9), 0, 1)
        out.append(np.min(np.hypot(*(a + ab * t[:, None] - p).T)))
    return np.array(out)


class Compare(Node):
    def __init__(self):
        super().__init__('compare_node')
        self.declare_parameter('log_csv', '')
        self.declare_parameter('horizon', 15.0)
        self.h = self.get_parameter('horizon').value
        self.main = self.val = None; self.gt = self.slam = None
        self.create_subscription(Path, '/planning/centerline', lambda m: setattr(self, 'main', m), QOS)
        self.create_subscription(Path, '/validation/centerline', self.on_val, QOS)
        self.create_subscription(Odometry, '/ground_truth/state_odom', lambda m: setattr(self, 'gt', self.pose(m)), QOS)
        self.create_subscription(Odometry, '/slam/odom', lambda m: setattr(self, 'slam', self.pose(m)), QOS)
        self.dev, self.n_val, self.n_miss = [], 0, 0
        self.create_timer(10.0, self.report)
        self.csv = self.get_parameter('log_csv').value

    @staticmethod
    def pose(m):
        return (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))

    def on_val(self, m):
        self.n_val += 1
        if self.main is None or self.gt is None or self.slam is None or len(m.poses) < 2 or len(self.main.poses) < 2:
            self.n_miss += 1; return
        v = np.array([[p.pose.position.x, p.pose.position.y] for p in m.poses])
        r = np.array([[p.pose.position.x, p.pose.position.y] for p in self.main.poses])
        vc, rc = to_car(v, self.slam), to_car(r, self.gt)
        vc = vc[(vc[:, 0] > 0) & (vc[:, 0] < self.h)]; rc = rc[(rc[:, 0] > -2) & (rc[:, 0] < self.h + 3)]
        if len(vc) < 2 or len(rc) < 2:
            self.n_miss += 1; return
        self.dev.append(dist_to_polyline(vc, rc))

    def report(self):
        if not self.dev:
            self.get_logger().info(f'no comparable paths yet (validation paths received: {self.n_val}, unusable: {self.n_miss})'); return
        d = np.concatenate(self.dev)
        self.get_logger().info(f'validation path vs main path (0-{self.h:.0f} m ahead, car frame): mean {d.mean():.2f} m  p95 {np.percentile(d, 95):.2f} m  '
                               f'max {d.max():.2f} m  over {len(self.dev)} paths')
        if self.csv:
            new = not os.path.exists(self.csv)
            with open(self.csv, 'a') as f:
                w = csv.writer(f)
                if new: w.writerow(['wall_time', 'paths', 'mean_m', 'p95_m', 'max_m'])
                w.writerow([f'{time.time():.0f}', len(self.dev), f'{d.mean():.3f}', f'{np.percentile(d, 95):.3f}', f'{d.max():.3f}'])
        self.dev = []


def main():
    rclpy.init(); rclpy.spin(Compare()); rclpy.shutdown()
