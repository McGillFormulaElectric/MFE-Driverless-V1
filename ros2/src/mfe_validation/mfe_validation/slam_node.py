"""Live cone-landmark SLAM (optimiser behind the `backend` parameter, see slam_backend.py).

In : odom_topic (default /sim/xsens/state_odom; yaw and forward/lateral speed are integrated,
     its absolute position is not used),
     /perception/cones_uncolored (LiDAR detections, sensor frame).
Out: /slam/odom (start-relative frame), /validation/cones (mfe_msgs/Track of confirmed landmarks).
Colours: color_mode='gt' comes from /ground_truth/track_colored (validation only; positions never use ground truth);
'geometric' is left=blue/right=yellow; 'yolo' gives each LiDAR landmark the colour the camera's YOLO detections vote for
(see cam_fusion.py), with the 'color_fallback' for landmarks the camera has not settled. 'orange_only': /planning/cones
stays all UNKNOWN (colourless path), and only the landmarks the camera settles as ORANGE_BIG/ORANGE_SMALL are published
on orange_cones_topic (no geometric fallback). 'yolo_lidar': camera colour or UNKNOWN on /planning/cones (no geometric
guess), and orange landmarks also go on orange_cones_topic.
Accuracy against /ground_truth/state_odom is logged for scoring only.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from mfe_msgs.msg import Track, Cone
from mfe_validation.cam_fusion import CamFusion, PoseHistory, decide
from mfe_common.geometry import pointcloud2_to_xy_array, wrap_angle, yaw_from_quaternion
from mfe_common.qos import RELIABLE_QOS
from mfe_validation.common import car_start, read_track
from mfe_validation.slam_backend import compose, make_backend, transform_from
from mfe_common.topics import (
    GROUND_TRUTH_STATE_ODOM,
    GROUND_TRUTH_TRACK_COLORED,
    IMAGE_CONES,
    PERCEPTION_CONES_UNCOLORED,
    PLANNING_CONES_CAMERA_COLOURS,
    PLANNING_ORANGE_CONES,
    SIM_XSENS_STATE_ODOM,
    SLAM_ODOM,
    VALIDATION_CONES,
)

PARAMS = dict(track_csv='', keyframe_m=1.5, gate=0.5, rng_sigma=0.15, brg_sigma=0.03, max_range=15.0,
              min_obs_publish=3, publish_radius=25.0, publish_hz=2.0, color_radius=2.0,
              cones_topic=VALIDATION_CONES, color_mode='gt', keyframe_dt=0.5, freeze=True, dup_radius=0.6,
              use_lateral_vel=False, backend='gtsam',
              yolo_topic=IMAGE_CONES, cam_hfov_deg=87.0, cam_width=640, cam_height=480, cam_pitch_deg=-3.0,   # fitted: YOLO boxes sit within a few px of landmarks at -3 deg (URDF says +10.15 but the sim image is ~level)

              cam_x=0.12, cam_y=0.0, cam_z=0.76, cone_z=0.16, pix_du=0.0, pix_dv=0.0, match_px=40.0,
              color_min_range=2.0, color_max_range=18.0, min_color_votes=2, color_vote_share=0.6, color_fallback='geometric',
              orange_cones_topic=PLANNING_ORANGE_CONES, odom_topic=SIM_XSENS_STATE_ODOM,
              ground_truth_odom_topic=GROUND_TRUTH_STATE_ODOM,
              ground_truth_track_topic=GROUND_TRUTH_TRACK_COLORED)


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
        self.poses = PoseHistory()          # SLAM pose history, for placing each camera frame where the car was when it was taken
        self.votes = {}                     # landmark id -> colour votes [blue, yellow, orange big, orange small, unknown]
        self.pending, self.pending_stamp = [], None
        self.n_frames = self.n_votes = 0
        self.min_votes, self.vote_share, self.fallback = p('min_color_votes'), p('color_vote_share'), p('color_fallback')
        self.fusion = CamFusion(p('cam_hfov_deg'), p('cam_width'), p('cam_height'), (p('cam_x'), p('cam_y'), p('cam_z')),
                                p('cam_pitch_deg'), p('cone_z'), p('pix_du'), p('pix_dv'), p('match_px'),
                                p('color_min_range'), p('color_max_range'))
        self.create_subscription(Odometry, p('odom_topic'), self.on_odom, RELIABLE_QOS)
        self.create_subscription(Odometry, p('ground_truth_odom_topic'), self.on_gt, RELIABLE_QOS)
        self.create_subscription(PointCloud2, PERCEPTION_CONES_UNCOLORED, self.on_cones, RELIABLE_QOS)
        if self.color_mode == 'gt':
            self.create_subscription(Track, p('ground_truth_track_topic'), self.on_gt_track, 1)
        if self.color_mode in ('yolo', 'orange_only', 'yolo_lidar'):
            self.create_subscription(Cone, p('yolo_topic'), self.on_yolo, RELIABLE_QOS)
        self.pub_odom = self.create_publisher(Odometry, SLAM_ODOM, RELIABLE_QOS)
        self.pub_cones = self.create_publisher(Track, p('cones_topic'), RELIABLE_QOS)
        self.pub_orange = (self.create_publisher(Track, p('orange_cones_topic'), RELIABLE_QOS)
                           if self.color_mode in ('orange_only', 'yolo_lidar') else None)
        # DISPLAY ONLY (not read by the planner): every landmark with its camera colour vote (cam_fusion.decide)
        self.pub_camcol = self.create_publisher(Track, PLANNING_CONES_CAMERA_COLOURS, RELIABLE_QOS)
        self.create_timer(1.0 / max(0.1, float(p('publish_hz'))), self.publish_cones)
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'slam_node up: velocity+yaw odometry, cone landmarks, colour mode={self.color_mode}, output {p("cones_topic")}')

    # ---- inputs ----
    def on_gt_track(self, m):
        c, s = math.cos(self.yaw0), math.sin(self.yaw0)
        xy = np.array([[q.location.x - self.x0, q.location.y - self.y0] for q in m.track]).reshape(-1, 2)
        self.gt_cones = np.c_[c * xy[:, 0] + s * xy[:, 1], -s * xy[:, 0] + c * xy[:, 1]]
        self.gt_colors = [q.color for q in m.track]

    def on_yolo(self, m):
        """One message per detection (pixel centre + colour class), all sharing the camera frame's capture stamp."""
        stamp = (m.header.stamp.sec, m.header.stamp.nanosec)
        if self.pending_stamp is not None and stamp != self.pending_stamp:
            self.flush_yolo()
        self.pending_stamp = stamp
        if m.header.frame_id == 'camera_pixel':
            self.pending.append((m.location.x, m.location.y, int(m.color)))

    def flush_yolo(self):
        sec, nsec = self.pending_stamp
        dets, self.pending = self.pending, []
        pose = self.poses.at(sec + nsec * 1e-9)
        if pose is None or not dets or not self.lms:
            return
        self.n_frames += 1
        for j, col in self.fusion.vote(dets, {j: lm for j, lm in self.lms.items()}, pose):
            self.votes.setdefault(j, np.zeros(5))[col] += 1.0; self.n_votes += 1

    def landmark_colour(self, j, lm):
        c = decide(self.votes[j], self.min_votes, self.vote_share) if j in self.votes else None
        if c is not None:
            return c
        if self.fallback == 'geometric':
            return Cone.BLUE if lm[1] > 0 else Cone.YELLOW
        return Cone.UNKNOWN

    def orange_colour(self, j):
        """orange_only: ORANGE_BIG or ORANGE_SMALL once the camera settles on orange, else None (no fallback)."""
        c = decide(self.votes[j], self.min_votes, self.vote_share) if j in self.votes else None
        return c if c in (Cone.ORANGE_BIG, Cone.ORANGE_SMALL) else None

    def on_gt(self, m):
        self.gt_pose = (m.pose.pose.position.x, m.pose.pose.position.y)

    def on_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.xs_pose = (m.pose.pose.position.x, m.pose.pose.position.y)
        self.samples.append((t, yaw_from_quaternion(m.pose.pose.orientation), m.twist.twist.linear.x, m.twist.twist.linear.y))
        if len(self.samples) % 5 == 0 and self.kf_t is not None:
            self.publish_pose(m.header.stamp)

    def delta_since_kf(self):
        """Body-frame motion (dx, dy, dtheta) integrated since the last keyframe."""
        s = self.samples
        if len(s) < 2:
            return (0.0, 0.0, 0.0)
        a = np.array(s); dt = np.diff(a[:, 0]); th = np.array([wrap_angle(y - a[0, 1]) for y in a[:, 1]])
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
        self.poses.add(stamp.sec + stamp.nanosec * 1e-9, pose[0], pose[1], pose[2])
        if self.gt_pose:
            self.errs.append(math.hypot(pose[0] - self.gt_pose[0], pose[1] - self.gt_pose[1]))
            if self.xs_pose:
                self.errs_raw.append(math.hypot(self.xs_pose[0] - self.gt_pose[0], self.xs_pose[1] - self.gt_pose[1]))

    # ---- LiDAR frame -> keyframe + observations ----
    def on_cones(self, m):
        if not self.samples:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        pts = pointcloud2_to_xy_array(m)
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
        msg = Track(); orange = Track(); now = self.get_clock().now().to_msg()
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
            elif self.color_mode == 'yolo':
                cone.color = self.landmark_colour(j, lm)
            elif self.color_mode == 'yolo_lidar':   # camera colour or UNKNOWN (no geometric guess); LiDAR planner picks the walls
                c = decide(self.votes[j], self.min_votes, self.vote_share) if j in self.votes else None
                cone.color = c if c is not None else Cone.UNKNOWN
                if c in (Cone.ORANGE_BIG, Cone.ORANGE_SMALL):
                    oc_msg = Cone(); oc_msg.header = cone.header; oc_msg.color = c
                    oc_msg.location = cone.location
                    orange.track.append(oc_msg)
            elif self.color_mode == 'orange_only':
                oc = self.orange_colour(j)
                if oc is not None:
                    oc_msg = Cone(); oc_msg.header = cone.header; oc_msg.color = oc
                    oc_msg.location = cone.location
                    orange.track.append(oc_msg)
            elif self.gt_cones is not None and len(self.gt_cones):
                d = np.hypot(*(self.gt_cones - lm).T); i = int(np.argmin(d))
                if d[i] <= self.color_radius:
                    cone.color = self.gt_colors[i]
            msg.track.append(cone)
        self.pub_cones.publish(msg)
        if self.pub_orange is not None:
            self.pub_orange.publish(orange)
        dbg = Track()   # mfe_msgs/Track has no header: stamp each Cone
        for j, lm in self.lms.items():
            c = decide(self.votes[j], self.min_votes, self.vote_share) if j in self.votes else None
            dc = Cone(); dc.header.stamp = now; dc.header.frame_id = 'map'
            dc.location.x, dc.location.y = float(lm[0]), float(lm[1])
            dc.color = c if c is not None else Cone.UNKNOWN
            dbg.track.append(dc)
        self.pub_camcol.publish(dbg)

    def report(self):
        if not self.errs:
            return
        e, r = np.array(self.errs), np.array(self.errs_raw) if self.errs_raw else np.array([float('nan')])
        self.get_logger().info(f'SLAM pose err vs GT: mean {e.mean():.2f} m max {e.max():.2f} m | raw Xsens {np.nanmean(r):.2f}/{np.nanmax(r):.2f} m | '
                               f'{self.n_keys} keyframes, {len(self.lms)} landmarks ({sum(v >= self.min_obs for v in self.lm_obs.values())} published)')
        if self.color_mode == 'yolo_lidar':   # unsettled = UNKNOWN (4), no geometric guess
            cols = [decide(self.votes[j], self.min_votes, self.vote_share) if j in self.votes else None for j in self.lms]
            cols = [Cone.UNKNOWN if c is None else c for c in cols]
            self.get_logger().info(f'YOLO colours: {self.n_frames} camera frames, {self.n_votes} votes | landmarks '
                                   f'blue {cols.count(0)} yellow {cols.count(1)} orange {cols.count(2) + cols.count(3)} none {cols.count(4)}')
        if self.color_mode == 'yolo':
            cols = [self.landmark_colour(j, lm) for j, lm in self.lms.items()]
            self.get_logger().info(f'YOLO colours: {self.n_frames} camera frames, {self.n_votes} votes | landmarks '
                                   f'blue {cols.count(0)} yellow {cols.count(1)} orange {cols.count(2) + cols.count(3)} none {cols.count(4)}')
        if self.color_mode == 'orange_only':
            n_or = sum(self.orange_colour(j) is not None for j in self.lms)
            self.get_logger().info(f'orange_only: {n_or} orange landmarks of {len(self.lms)} '
                                   f'({self.n_frames} camera frames, {self.n_votes} votes)')
        self.errs, self.errs_raw = [], []


def main():
    rclpy.init(); rclpy.spin(SlamNode()); rclpy.shutdown()
