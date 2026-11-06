"""Camera colour for the SLAM cone map: the stage after slam_node (see docs/interfaces.md).

In : slam_cones_topic (mfe_msgs/Track, the SLAM cone map; UNKNOWN colours in the camera modes),
     odom_topic (/slam/odom, to place each camera frame where the car was when it was taken),
     yolo_topic (/image/cones, one message per camera detection).
Out: cones_topic (planner map), orange_cones_topic (orange_only, yolo_lidar), camera_colours_topic (display).
The voting itself is in cam_fusion.py.

SLAM freezes each landmark at its first position, so a cone comes back at the same coordinates on every map
message. Track has no ids, so a cone takes the id of the nearest cone seen before within assoc_radius, or a new id.
Only the cones of the latest map can be projected into a camera frame, so a vote for a cone that SLAM has not
published yet is not kept.
"""
import math

import numpy as np
import rclpy
from mfe_common.geometry import yaw_from_quaternion
from mfe_common.qos import LATEST_CONES_QOS, RELIABLE_QOS
from mfe_common.topics import (IMAGE_CONES, PLANNING_CONES, PLANNING_CONES_CAMERA_COLOURS,
                               PLANNING_ORANGE_CONES, SLAM_CONES, SLAM_ODOM)
from mfe_msgs.msg import Cone, Track
from mfe_validation.cam_fusion import CAMERA_COLOUR_MODES, CamFusion, PoseHistory, decide
from nav_msgs.msg import Odometry
from rclpy.node import Node

PARAMS = dict(color_mode='yolo', slam_cones_topic=SLAM_CONES, odom_topic=SLAM_ODOM, yolo_topic=IMAGE_CONES,
              cones_topic=PLANNING_CONES, orange_cones_topic=PLANNING_ORANGE_CONES,
              camera_colours_topic=PLANNING_CONES_CAMERA_COLOURS, color_fallback='geometric', min_color_votes=2,
              color_vote_share=0.6, assoc_radius=0.3,
              cam_hfov_deg=87.0, cam_width=640, cam_height=480,
              cam_pitch_deg=-3.0,   # fitted: YOLO boxes sit within a few px of landmarks at -3 deg (URDF says +10.15)
              cam_x=0.12, cam_y=0.0, cam_z=0.76, cone_z=0.16, pix_du=0.0, pix_dv=0.0, match_px=40.0,
              color_min_range=2.0, color_max_range=18.0)


class ConeColourFusion(Node):
    def __init__(self):
        super().__init__('cone_colour_fusion_node')
        for k, v in PARAMS.items():
            self.declare_parameter(k, v)
        p = self.get_parameter
        self.color_mode = p('color_mode').value
        if self.color_mode not in CAMERA_COLOUR_MODES:
            raise ValueError(f'color_mode {self.color_mode!r} has no camera colour, use one of {CAMERA_COLOUR_MODES}')
        self.fallback = p('color_fallback').value
        self.min_votes = p('min_color_votes').value
        self.vote_share = p('color_vote_share').value
        self.assoc_radius = p('assoc_radius').value
        self.fusion = CamFusion(p('cam_hfov_deg').value, p('cam_width').value, p('cam_height').value,
                                (p('cam_x').value, p('cam_y').value, p('cam_z').value),
                                p('cam_pitch_deg').value, p('cone_z').value, p('pix_du').value, p('pix_dv').value,
                                p('match_px').value, p('color_min_range').value, p('color_max_range').value)
        self.poses = PoseHistory()      # car poses from /slam/odom, the stream slam_node publishes
        self.pos = {}                   # cone id -> last seen position, for every cone seen in a map
        self.next_id = 0
        self.lms = {}                   # cone id -> position, cones of the latest map
        self.votes = {}                 # cone id -> colour votes [blue, yellow, orange big, orange small, unknown]
        self.pending, self.pending_stamp = [], None
        self.n_frames = self.n_votes = 0
        self.create_subscription(Odometry, p('odom_topic').value, self.on_odom, RELIABLE_QOS)
        self.create_subscription(Track, p('slam_cones_topic').value, self.on_map, LATEST_CONES_QOS)
        self.create_subscription(Cone, p('yolo_topic').value, self.on_yolo, RELIABLE_QOS)
        self.pub_cones = self.create_publisher(Track, p('cones_topic').value, RELIABLE_QOS)
        self.pub_orange = None
        if self.color_mode in ('orange_only', 'yolo_lidar'):
            self.pub_orange = self.create_publisher(Track, p('orange_cones_topic').value, RELIABLE_QOS)
        # DISPLAY ONLY (not read by the planner): every cone of the latest map with its camera colour
        self.pub_display = self.create_publisher(Track, p('camera_colours_topic').value, RELIABLE_QOS)
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'cone_colour_fusion_node up: colour mode={self.color_mode}, '
                               f'{p("slam_cones_topic").value} -> {p("cones_topic").value}')

    # ---- inputs ----
    def on_odom(self, m):
        self.poses.add(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x,
                       m.pose.pose.position.y, yaw_from_quaternion(m.pose.pose.orientation))

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
        for j, col in self.fusion.vote(dets, self.lms, pose):
            self.votes.setdefault(j, np.zeros(5))[col] += 1.0
            self.n_votes += 1

    def identify(self, xy):
        best, best_d = None, self.assoc_radius
        for j, q in self.pos.items():
            d = math.hypot(q[0] - xy[0], q[1] - xy[1])
            if d < best_d:
                best, best_d = j, d
        if best is None:
            best = self.next_id
            self.next_id += 1
        self.pos[best] = xy
        return best

    def on_map(self, m):
        self.lms = {}
        cones = []
        for c in m.track:
            j = self.identify((c.location.x, c.location.y))
            self.lms[j] = np.array([c.location.x, c.location.y])
            cones.append((j, c))
        planner, orange, display = Track(), Track(), Track()
        for j, c in cones:
            lm, cam = self.lms[j], self.camera_colour(j)
            out = Cone()
            out.header, out.location = c.header, c.location
            out.color = self.planner_colour(lm, cam)
            planner.track.append(out)
            shown = Cone()
            shown.header, shown.location = c.header, c.location
            shown.color = Cone.UNKNOWN if cam is None else cam
            display.track.append(shown)
            if self.pub_orange is not None and cam in (Cone.ORANGE_BIG, Cone.ORANGE_SMALL):
                oc = Cone()
                oc.header, oc.location, oc.color = c.header, c.location, cam
                orange.track.append(oc)
        self.pub_cones.publish(planner)
        if self.pub_orange is not None:
            self.pub_orange.publish(orange)
        self.pub_display.publish(display)

    # ---- colours ----
    def camera_colour(self, j):
        """Camera colour of a cone once it has enough agreeing votes, else None."""
        if j not in self.votes:
            return None
        return decide(self.votes[j], self.min_votes, self.vote_share)

    def planner_colour(self, lm, cam):
        if self.color_mode == 'orange_only':    # planner cones stay UNKNOWN, orange ones go on orange_cones_topic
            return Cone.UNKNOWN
        if cam is not None:
            return cam
        if self.color_mode == 'yolo' and self.fallback == 'geometric':   # left of the start heading = blue
            return Cone.BLUE if lm[1] > 0 else Cone.YELLOW
        return Cone.UNKNOWN

    def report(self):
        if not self.lms:
            return
        cols = [self.camera_colour(j) for j in self.lms]
        self.get_logger().info(
            f'{self.n_frames} camera frames, {self.n_votes} votes | cones {len(cols)}: '
            f'blue {cols.count(Cone.BLUE)} yellow {cols.count(Cone.YELLOW)} '
            f'orange {cols.count(Cone.ORANGE_BIG) + cols.count(Cone.ORANGE_SMALL)} none {cols.count(None)}')


def main():
    rclpy.init()
    rclpy.spin(ConeColourFusion())
    rclpy.shutdown()
