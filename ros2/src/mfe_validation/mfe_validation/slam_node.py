"""Live cone-landmark SLAM (optimiser behind the `backend` parameter, see slam_backend.py).

In : /sim/xsens/state_odom (yaw and forward/lateral speed are integrated; its absolute position is not used),
     /perception/cones_uncolored (LiDAR detections, sensor frame).
Out: /slam/odom (start-relative frame), /validation/cones (mfe_msgs/Track of confirmed landmarks).
Colours with color_mode='gt' come from /ground_truth/track_colored (validation only; positions never use ground truth).
Accuracy against /ground_truth/state_odom is logged for scoring only.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from mfe_msgs.msg import Track, Cone
from mfe_validation.common import QOS, car_start, read_track, wrap, yaw_of
from mfe_validation.slam_backend import compose, make_backend, transform_from

PARAMS = dict(track_csv='', keyframe_m=1.5, gate=1.0, rng_sigma=0.15, brg_sigma=0.03, max_range=15.0,
              min_obs_publish=3, publish_radius=25.0, publish_hz=2.0, color_radius=2.0,
              cones_topic='/validation/cones', color_mode='gt', keyframe_dt=0.5, freeze=True, dup_radius=0.6,
              use_lateral_vel=False, backend='gtsam')


class SlamNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        for k, v in PARAMS.items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value
        self.keyframe_m, self.gate, self.max_range = p('keyframe_m'), p('gate'), p('max_range')
        self.color_mode, self.keyframe_dt = p('color_mode'), p('keyframe_dt')
        self.p_freeze, self.p_dup = p('freeze'), p('dup_radius')
        # Integrating lateral velocity too: forward-speed-only odometry drifts sideways through corners (the car slides)
        # and the optimiser then bends the heading to fit the LiDAR landmarks.
        self.use_vy = bool(p('use_lateral_vel'))
        self.min_obs, self.pub_radius, self.color_radius = p('min_obs_publish'), p('publish_radius'), p('color_radius')
        self.backend = make_backend(p('backend'), obs_sigmas=(p('brg_sigma'), p('rng_sigma')))

        # Track start pose: the GT colour map is in the csv frame, SLAM is start-relative.
        self.x0, self.y0, self.yaw0 = (0.0, 0.0, 0.0)
        if p('track_csv'):
            self.x0, self.y0, self.yaw0 = car_start(read_track(p('track_csv'))) or (0.0, 0.0, 0.0)
        self.gt_cones = None            # Nx2 landmark-frame GT cones, with colours
        self.gt_colors = []
        self.n_keys = 0                 # keyframes so far; keyframe i is backend pose i
        self.samples = []               # (sim_t, yaw, vx, vy) since the last keyframe
        self.kf_pose = (0.0, 0.0, 0.0)  # estimate of the last keyframe
        self.frozen = {}                # landmark id -> position first published (the planner's cone map is append-only)
        self.kf_t = None
        self.lms, self.lm_obs, self.tent, self.nxt = {}, {}, [], 0
        self.gt_pose = None
        self.xs_pose = None
        self.errs, self.errs_raw = [], []
        self.create_subscription(Odometry, '/sim/xsens/state_odom', self.on_odom, QOS)
        self.create_subscription(Odometry, '/ground_truth/state_odom', self.on_gt, QOS)
        self.create_subscription(PointCloud2, '/perception/cones_uncolored', self.on_cones, QOS)
        if self.color_mode == 'gt':
            self.create_subscription(Track, '/ground_truth/track_colored', self.on_gt_track, 1)
        self.pub_odom = self.create_publisher(Odometry, '/slam/odom', QOS)
        self.pub_cones = self.create_publisher(Track, p('cones_topic'), QOS)
        self.create_timer(1.0 / max(0.1, float(p('publish_hz'))), self.publish_cones)
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'slam_node up: velocity+yaw odometry, cone landmarks, colour mode={self.color_mode}, output {p("cones_topic")}')

    # ---- inputs ----
    def on_gt_track(self, m):
        c, s = math.cos(self.yaw0), math.sin(self.yaw0)
        xy = np.array([[q.location.x - self.x0, q.location.y - self.y0] for q in m.track]).reshape(-1, 2)
        self.gt_cones = np.c_[c * xy[:, 0] + s * xy[:, 1], -s * xy[:, 0] + c * xy[:, 1]]
        self.gt_colors = [q.color for q in m.track]

    def on_gt(self, m):
        self.gt_pose = (m.pose.pose.position.x, m.pose.pose.position.y)

    def on_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.xs_pose = (m.pose.pose.position.x, m.pose.pose.position.y)
        self.samples.append((t, yaw_of(m.pose.pose.orientation), m.twist.twist.linear.x, m.twist.twist.linear.y))
        if len(self.samples) % 5 == 0 and self.kf_t is not None:
            self.publish_pose(m.header.stamp)

    def delta_since_kf(self):
        """Body-frame motion (dx, dy, dtheta) integrated since the last keyframe."""
        s = self.samples
        if len(s) < 2:
            return (0.0, 0.0, 0.0)
        a = np.array(s); dt = np.diff(a[:, 0]); th = np.array([wrap(y - a[0, 1]) for y in a[:, 1]])
        mid = 0.5 * (th[1:] + th[:-1])
        vx = a[1:, 2]; vy = a[1:, 3] if self.use_vy else 0.0 * vx
        return (float(((vx * np.cos(mid) - vy * np.sin(mid)) * dt).sum()),
                float(((vx * np.sin(mid) + vy * np.cos(mid)) * dt).sum()), float(th[-1]))

    def publish_pose(self, stamp):
        pose = compose(self.kf_pose, self.delta_since_kf())
        o = Odometry(); o.header.stamp = stamp; o.header.frame_id = 'map'; o.child_frame_id = 'base_footprint'
        o.pose.pose.position.x, o.pose.pose.position.y = pose[0], pose[1]
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        o.twist.twist.linear.x = self.samples[-1][2]
        self.pub_odom.publish(o)
        if self.gt_pose:
            self.errs.append(math.hypot(pose[0] - self.gt_pose[0], pose[1] - self.gt_pose[1]))
            if self.xs_pose:
                self.errs_raw.append(math.hypot(self.xs_pose[0] - self.gt_pose[0], self.xs_pose[1] - self.gt_pose[1]))

    # ---- LiDAR frame -> keyframe + observations ----
    def on_cones(self, m):
        if not self.samples:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        pts = np.array([[a[0], a[1]] for a in pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)]).reshape(-1, 2)
        first = self.kf_t is None
        if first:
            est = (0.0, 0.0, 0.0)           # SLAM frame origin = start pose (known at the line)
            self.backend.add_pose_prior(0, est, (0.05, 0.05, 0.01))
            self.backend.init_pose(0, est)
        else:
            d = self.delta_since_kf()
            n = math.hypot(d[0], d[1])
            if n < self.keyframe_m and abs(d[2]) < math.radians(5) and t - self.kf_t < self.keyframe_dt:
                return
            est = compose(self.kf_pose, d)
            sig = (0.05 + 0.08 * n, 0.05 + 0.08 * n, 0.01)
            self.backend.add_odometry(self.n_keys - 1, self.n_keys, d[0], d[1], d[2], sig)
            self.backend.init_pose(self.n_keys, est)
        key = self.n_keys
        for p in pts:
            rr = float(np.hypot(*p))
            if rr > self.max_range or rr < 1.0:
                continue
            bearing = math.atan2(p[1], p[0])
            wxy = np.array(transform_from(est, (float(p[0]), float(p[1]))))
            best, bd = None, self.gate
            for j, lm in self.lms.items():
                dd = float(np.hypot(*(lm - wxy)))
                if dd < bd:
                    best, bd = j, dd
            if best is not None:
                self.backend.add_landmark_observation(key, best, bearing, rr); self.lm_obs[best] += 1
                continue
            for tl in self.tent:
                if float(np.hypot(*(tl[0] - wxy))) < self.gate:
                    tl[1].append((key, bearing, rr))
                    if len(tl[1]) >= 2:     # seen from two keyframes: promote to a landmark
                        self.backend.init_landmark(self.nxt, tl[0])
                        for kk, b, r_ in tl[1]:
                            self.backend.add_landmark_observation(kk, self.nxt, b, r_)
                        self.lms[self.nxt] = tl[0]; self.lm_obs[self.nxt] = len(tl[1]); self.nxt += 1
                        self.tent[:] = [x for x in self.tent if x is not tl]
                    break
            else:
                self.tent.append([wxy, [(key, bearing, rr)]])
        self.backend.update()
        self.n_keys += 1; self.kf_t = t
        self.kf_pose = self.backend.pose(key)
        for j in self.lms:
            self.lms[j] = np.array(self.backend.landmark(j))
        self.tent[:] = self.tent[-200:]
        self.samples = self.samples[-1:]    # restart integration from the keyframe sample
        self.publish_pose(m.header.stamp)

    def publish_cones(self):
        if not self.n_keys:
            return
        cx, cy = self.kf_pose[0], self.kf_pose[1]
        msg = Track(); now = self.get_clock().now().to_msg()
        for j, lm in self.lms.items():
            if self.lm_obs[j] < self.min_obs or math.hypot(lm[0] - cx, lm[1] - cy) > self.pub_radius:
                continue
            if self.p_freeze:
                # The planner never forgets a cone and keys it on a 0.5 m grid, so a moving estimate leaves ghosts.
                if j not in self.frozen:
                    if any(np.hypot(*(f - lm)) < self.p_dup for f in self.frozen.values()):
                        continue
                    self.frozen[j] = lm.copy()
                lm = self.frozen[j]
            cone = Cone(); cone.header.stamp = now; cone.header.frame_id = 'map'
            cone.location.x, cone.location.y = float(lm[0]), float(lm[1])
            cone.color = Cone.UNKNOWN
            if self.color_mode == 'geometric':      # left of the start heading = blue, right = yellow
                cone.color = Cone.BLUE if lm[1] > 0 else Cone.YELLOW
            elif self.gt_cones is not None and len(self.gt_cones):
                d = np.hypot(*(self.gt_cones - lm).T); i = int(np.argmin(d))
                if d[i] <= self.color_radius:
                    cone.color = self.gt_colors[i]
            msg.track.append(cone)
        self.pub_cones.publish(msg)

    def report(self):
        if not self.errs:
            return
        e, r = np.array(self.errs), np.array(self.errs_raw) if self.errs_raw else np.array([float('nan')])
        self.get_logger().info(f'SLAM pose err vs GT: mean {e.mean():.2f} m max {e.max():.2f} m | raw Xsens {np.nanmean(r):.2f}/{np.nanmax(r):.2f} m | '
                               f'{self.n_keys} keyframes, {len(self.lms)} landmarks ({sum(v >= self.min_obs for v in self.lm_obs.values())} published)')
        self.errs, self.errs_raw = [], []


def main():
    rclpy.init(); rclpy.spin(SlamNode()); rclpy.shutdown()
