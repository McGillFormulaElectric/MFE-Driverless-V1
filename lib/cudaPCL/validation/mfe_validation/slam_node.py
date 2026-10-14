"""Live cone-landmark GraphSLAM (GTSAM iSAM2).

Inputs : /sim/xsens/state_odom (yaw + forward speed are integrated; its noisy absolute position is NOT used),
         /perception/cones_uncolored (LiDAR detections, sensor frame).
Outputs: /slam/odom (Odometry, start-relative frame = the odom frame), /validation/cones (mfe_msgs/Track: confirmed landmarks).
Colours: the sim has no LiDAR intensity, so landmark colours are looked up from /ground_truth/track_colored (positions stay
LiDAR/SLAM-only). This is a validation-only crutch and is logged as such.
Also logs live accuracy vs /ground_truth/state_odom (used for scoring only, never fed back).
"""
import csv, math
import numpy as np
import gtsam
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from mfe_msgs.msg import Track, Cone

X, L = gtsam.symbol_shorthand.X, gtsam.symbol_shorthand.L
QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class SlamNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        for k, v in dict(track_csv='', keyframe_m=1.5, gate=1.0, rng_sigma=0.15, brg_sigma=0.03, max_range=15.0,
                         min_obs_publish=3, publish_radius=25.0, color_radius=2.0,
                         cones_topic='/validation/cones', color_mode='gt', keyframe_dt=0.5, freeze=True, dup_radius=0.6).items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value
        self.keyframe_m, self.gate, self.max_range = p('keyframe_m'), p('gate'), p('max_range')
        self.color_mode, self.keyframe_dt = p('color_mode'), p('keyframe_dt')
        self.p_freeze, self.p_dup = p('freeze'), p('dup_radius')
        self.min_obs, self.pub_radius, self.color_radius = p('min_obs_publish'), p('publish_radius'), p('color_radius')
        self.obs_noise = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.345),
            gtsam.noiseModel.Diagonal.Sigmas(np.array([p('brg_sigma'), p('rng_sigma')])))
        # track CSV start pose: GT colour map is in the csv/map frame, SLAM frame is start-relative
        self.x0 = self.y0 = self.yaw0 = 0.0
        if p('track_csv'):
            for r in csv.DictReader(open(p('track_csv'))):
                if r['tag'] == 'car_start':
                    self.x0, self.y0, self.yaw0 = float(r['x']), float(r['y']), float(r['direction'])
        self.gt_cones = None            # Nx2 (odom frame), colours
        self.gt_colors = []
        self.isam = gtsam.ISAM2(gtsam.ISAM2Params())
        self.keys = []                  # keyframe index list
        self.samples = []               # (sim_t, yaw, vx) since last keyframe
        self.kf_pose = gtsam.Pose2(0, 0, 0)   # current estimate of the last keyframe
        self.frozen = {}                # landmark id -> position first published (the planner's cone map is append-only)
        self.kf_t = None
        self.lms, self.lm_obs, self.tent, self.nxt = {}, {}, [], 0
        self.gt_pose = None
        self.xs_pose = None
        self.errs, self.errs_raw = [], []
        self.create_subscription(Odometry, '/sim/xsens/state_odom', self.on_odom, QOS)
        self.create_subscription(Odometry, '/ground_truth/state_odom', self.on_gt, QOS)
        self.create_subscription(PointCloud2, '/perception/cones_uncolored', self.on_cones, QOS)
        if self.color_mode == 'gt':      # 'geometric' needs no ground truth at all
            self.create_subscription(Track, '/ground_truth/track_colored', self.on_gt_track, 1)
        self.pub_odom = self.create_publisher(Odometry, '/slam/odom', QOS)
        self.pub_cones = self.create_publisher(Track, p('cones_topic'), QOS)
        self.create_timer(0.5, self.publish_cones)
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'slam_node up: velocity+yaw odometry, cone landmarks, colour mode={self.color_mode}, output {p("cones_topic")}')

    # ---- inputs ----
    def on_gt_track(self, m):
        c, s = math.cos(self.yaw0), math.sin(self.yaw0)
        xy = np.array([[q.location.x - self.x0, q.location.y - self.y0] for q in m.track]).reshape(-1, 2)
        self.gt_cones = np.c_[c * xy[:, 0] + s * xy[:, 1], -s * xy[:, 0] + c * xy[:, 1]]   # map -> start-relative odom frame
        self.gt_colors = [q.color for q in m.track]

    def on_gt(self, m):
        self.gt_pose = (m.pose.pose.position.x, m.pose.pose.position.y)

    def on_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.xs_pose = (m.pose.pose.position.x, m.pose.pose.position.y)
        self.samples.append((t, yaw_of(m.pose.pose.orientation), m.twist.twist.linear.x))
        if len(self.samples) % 5 == 0 and self.kf_t is not None:
            self.publish_pose(m.header.stamp)

    def delta_since_kf(self):
        s = self.samples
        if len(s) < 2:
            return gtsam.Pose2(0, 0, 0)
        a = np.array(s); dt = np.diff(a[:, 0]); th = np.array([wrap(y - a[0, 1]) for y in a[:, 1]])
        mid = 0.5 * (th[1:] + th[:-1])
        return gtsam.Pose2(float((a[1:, 2] * dt * np.cos(mid)).sum()), float((a[1:, 2] * dt * np.sin(mid)).sum()), float(th[-1]))

    def publish_pose(self, stamp):
        pose = self.kf_pose.compose(self.delta_since_kf())
        o = Odometry(); o.header.stamp = stamp; o.header.frame_id = 'map'; o.child_frame_id = 'base_footprint'
        o.pose.pose.position.x, o.pose.pose.position.y = pose.x(), pose.y()
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(pose.theta() / 2), math.cos(pose.theta() / 2)
        o.twist.twist.linear.x = self.samples[-1][2] if self.samples else 0.0
        self.pub_odom.publish(o)
        if self.gt_pose:
            self.errs.append(math.hypot(pose.x() - self.gt_pose[0], pose.y() - self.gt_pose[1]))
            if self.xs_pose: self.errs_raw.append(math.hypot(self.xs_pose[0] - self.gt_pose[0], self.xs_pose[1] - self.gt_pose[1]))

    # ---- LiDAR frame -> keyframe + observations ----
    def on_cones(self, m):
        if not self.samples:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        pts = np.array([[a[0], a[1]] for a in pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)]).reshape(-1, 2)
        graph, init = gtsam.NonlinearFactorGraph(), gtsam.Values()
        first = self.kf_t is None
        if first:
            est = gtsam.Pose2(0, 0, 0)       # SLAM frame origin = start pose (known at the line)
            graph.add(gtsam.PriorFactorPose2(X(0), est, gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.05, 0.01]))))
            init.insert(X(0), est)
        else:
            d = self.delta_since_kf()
            n = math.hypot(d.x(), d.y())
            if n < self.keyframe_m and abs(d.theta()) < math.radians(5) and t - self.kf_t < self.keyframe_dt:
                return
            est = self.kf_pose.compose(d)
            noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05 + 0.08 * n, 0.05 + 0.08 * n, 0.01]))
            graph.add(gtsam.BetweenFactorPose2(X(len(self.keys) - 1), X(len(self.keys)), d, noise))
            init.insert(X(len(self.keys)), est)
        key = X(len(self.keys))
        for p in pts:
            rr = float(np.hypot(*p))
            if rr > self.max_range or rr < 1.0:
                continue
            br = gtsam.Rot2.fromAngle(math.atan2(p[1], p[0]))
            w = est.transformFrom(gtsam.Point2(float(p[0]), float(p[1]))); wxy = np.array([w[0], w[1]])
            best, bd = None, self.gate
            for j, lm in self.lms.items():
                dd = float(np.hypot(*(lm - wxy)))
                if dd < bd: best, bd = j, dd
            if best is not None:
                graph.add(gtsam.BearingRangeFactor2D(key, L(best), br, rr, self.obs_noise)); self.lm_obs[best] += 1
                continue
            for tl in self.tent:
                if float(np.hypot(*(tl[0] - wxy))) < self.gate:
                    tl[1].append((key, br, rr))
                    if len(tl[1]) >= 2:
                        init.insert(L(self.nxt), gtsam.Point2(float(tl[0][0]), float(tl[0][1])))
                        for kk, b, r_ in tl[1]:
                            graph.add(gtsam.BearingRangeFactor2D(kk, L(self.nxt), b, r_, self.obs_noise))
                        self.lms[self.nxt] = tl[0]; self.lm_obs[self.nxt] = len(tl[1]); self.nxt += 1
                        self.tent[:] = [x for x in self.tent if x is not tl]
                    break
            else:
                self.tent.append([wxy, [(key, br, rr)]])
        self.isam.update(graph, init)
        res = self.isam.calculateEstimate()
        self.keys.append(key); self.kf_t = t
        self.kf_pose = res.atPose2(key)
        for j in self.lms:
            q = res.atPoint2(L(j)); self.lms[j] = np.array([q[0], q[1]])
        self.tent[:] = self.tent[-200:]
        self.samples = self.samples[-1:]     # restart integration from the keyframe sample
        self.publish_pose(m.header.stamp)

    def publish_cones(self):
        if not self.keys:
            return
        c = self.kf_pose
        msg = Track(); now = self.get_clock().now().to_msg()
        for j, lm in self.lms.items():
            if self.lm_obs[j] < self.min_obs or math.hypot(lm[0] - c.x(), lm[1] - c.y()) > self.pub_radius:
                continue
            if self.p_freeze:
                # planner never forgets a cone and keys it on a 0.5 m grid, so a moving estimate leaves ghosts: publish each landmark once, at a fixed position
                if j not in self.frozen:
                    if any(np.hypot(*(f - lm)) < self.p_dup for f in self.frozen.values()):
                        continue
                    self.frozen[j] = lm.copy()
                lm = self.frozen[j]
            cone = Cone(); cone.header.stamp = now; cone.header.frame_id = 'map'
            cone.location.x, cone.location.y = float(lm[0]), float(lm[1])
            cone.color = Cone.UNKNOWN
            if self.color_mode == 'geometric':      # left of the start heading = blue, right = yellow (accel/straight-lane layout)
                cone.color = Cone.BLUE if lm[1] > 0 else Cone.YELLOW
            elif self.gt_cones is not None and len(self.gt_cones):
                d = np.hypot(*(self.gt_cones - lm).T); i = int(np.argmin(d))
                if d[i] <= self.color_radius: cone.color = self.gt_colors[i]
            msg.track.append(cone)
        self.pub_cones.publish(msg)

    def report(self):
        if not self.errs:
            return
        e, r = np.array(self.errs), np.array(self.errs_raw) if self.errs_raw else np.array([float('nan')])
        self.get_logger().info(f'SLAM pose err vs GT: mean {e.mean():.2f} m max {e.max():.2f} m | raw Xsens {np.nanmean(r):.2f}/{np.nanmax(r):.2f} m | '
                               f'{len(self.keys)} keyframes, {len(self.lms)} landmarks ({sum(v >= self.min_obs for v in self.lm_obs.values())} published)')
        self.errs, self.errs_raw = [], []


def main():
    rclpy.init(); rclpy.spin(SlamNode()); rclpy.shutdown()
