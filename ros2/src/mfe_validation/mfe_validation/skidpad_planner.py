"""Skidpad planner driven by detected cones: 2 laps left, 2 laps right, exit.

The skidpad layout is fixed by the rules, so instead of a path hardcoded in the start frame, this registers the known cone
layout (eufs skidpad.csv) onto the SLAM landmarks (ICP, SE2) and drives the template path through that fit. If the
landmarks show the track is where the template says, the path equals the stock one; if SLAM/start pose is off, the fit
shifts the whole path onto the cones that were actually seen.

In:  mfe_msgs/Track landmarks (start-relative frame) on /skidpad/cones.   Out: /planning/centerline (nav_msgs/Path).
"""
import csv, math, os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from mfe_msgs.msg import Track
from mfe_path_planning.path_planner_node import _make_skidpad_path

QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
CSV = os.path.join(os.environ.get('EUFS_MASTER', os.path.expanduser('~/Develop/MFE26-eufs-sim')), 'eufs_sim/eufs_tracks/csv/skidpad.csv')


def rigid(a, b):
    """SE2 (theta, tx, ty) taking points a onto b (Kabsch)."""
    ca, cb = a.mean(0), b.mean(0)
    h = (a - ca).T @ (b - cb)
    th = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    c, s = math.cos(th), math.sin(th)
    return th, *(cb - np.array([c * ca[0] - s * ca[1], s * ca[0] + c * ca[1]]))


def apply(T, p):
    th, tx, ty = T
    c, s = math.cos(th), math.sin(th)
    return np.c_[c * p[:, 0] - s * p[:, 1] + tx, s * p[:, 0] + c * p[:, 1] + ty]


class SkidpadPlanner(Node):
    def __init__(self):
        super().__init__('skidpad_planner')
        self.declare_parameter('gate', 1.5)          # landmark <-> template cone match distance (m)
        self.declare_parameter('min_match', 6)
        self.declare_parameter('max_resid', 0.5)
        self.declare_parameter('max_shift', 2.0)     # reject fits that move the template more than this (m)
        self.gate, self.min_match = self.get_parameter('gate').value, self.get_parameter('min_match').value
        self.max_resid, self.max_shift = self.get_parameter('max_resid').value, self.get_parameter('max_shift').value
        rows = list(csv.DictReader(open(CSV)))
        cs = [r for r in rows if r['tag'] == 'car_start'][0]
        x0, y0, yaw = float(cs['x']), float(cs['y']), float(cs['direction'])
        m = np.array([[float(r['x']) - x0, float(r['y']) - y0] for r in rows if r['tag'] in ('blue', 'yellow', 'orange', 'big_orange')])
        c, s = math.cos(-yaw), math.sin(-yaw)
        self.tpl_cones = np.c_[c * m[:, 0] - s * m[:, 1], s * m[:, 0] + c * m[:, 1]]   # start-relative frame, same as SLAM
        self.tpl_path = _make_skidpad_path()
        self.T = (0.0, 0.0, 0.0)
        # Progress along the template (monotonic). pure_pursuit resets its own path index to 0 on every new path and then
        # snaps to the nearest point anywhere on it, so a path that revisits the same circle (2 laps) traps it on lap 1.
        # Publishing only the path ahead of the car (never lap 1 again) makes 2L + 2R work.
        self.idx = 0
        self.window = 25       # points published ahead (~24 m, under one 57 m lap so the path never overlaps itself)
        self.create_subscription(Odometry, '/slam/odom', self.on_pose, QOS)
        self.pub = self.create_publisher(Path, '/planning/centerline', QOS)
        self.create_subscription(Track, '/skidpad/cones', self.on_cones, QOS)
        self.n = 0
        self.get_logger().info(f'skidpad_planner up: {len(self.tpl_cones)} template cones, {len(self.tpl_path)} path points')

    def fit(self, lm):
        T = self.T
        for _ in range(15):
            q = apply(T, self.tpl_cones)
            d = np.linalg.norm(lm[:, None] - q[None], axis=2)
            j = d.argmin(1); ok = d[np.arange(len(lm)), j] < self.gate
            if ok.sum() < self.min_match:
                return None
            T = rigid(self.tpl_cones[j[ok]], lm[ok])
        resid = np.linalg.norm(apply(T, self.tpl_cones[j[ok]]) - lm[ok], axis=1).mean()
        if resid > self.max_resid or math.hypot(T[1], T[2]) > self.max_shift:
            return None
        return T, int(ok.sum()), resid

    def on_pose(self, msg):
        p = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        q = apply(self.T, self.tpl_path[self.idx:self.idx + 10])   # small window: can't jump to the overlapping next lap
        self.idx += int(np.linalg.norm(q - p, axis=1).argmin())

    def on_cones(self, msg):
        lm = np.array([[c.location.x, c.location.y] for c in msg.track]).reshape(-1, 2)
        if len(lm) >= self.min_match:
            r = self.fit(lm)
            if r:
                self.T = r[0]
                self.n += 1
                if self.n % 20 == 1:
                    self.get_logger().info(f'template fit: {r[1]} cones matched, resid {r[2]:.2f} m, shift ({self.T[1]:.2f},{self.T[2]:.2f}) m, rot {math.degrees(self.T[0]):.1f} deg')
        path = Path(); path.header.frame_id = 'map'; path.header.stamp = self.get_clock().now().to_msg()
        for x, y in apply(self.T, self.tpl_path[self.idx:self.idx + self.window]):
            ps = PoseStamped(); ps.header = path.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.orientation.w = float(x), float(y), 1.0
            path.poses.append(ps)
        self.pub.publish(path)


def main():
    rclpy.init(); rclpy.spin(SkidpadPlanner()); rclpy.shutdown()
