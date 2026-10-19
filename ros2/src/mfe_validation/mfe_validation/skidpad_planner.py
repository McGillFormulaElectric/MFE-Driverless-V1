"""Skidpad planner: 2 laps left, 2 laps right, exit.

Registers the known skidpad layout onto SLAM landmarks and drives the template path through the fit.
Registration (param registration_mode):
- 'library': fsd_path_planning SkidpadRelocalizer, latches the first plausible fit.
- 'custom': SE2 Kabsch/ICP against eufs skidpad.csv, refitted on every cone message.

In : mfe_msgs/Track landmarks (start-relative) on /skidpad/cones, nav_msgs/Odometry on /slam/odom.
Out: /planning/centerline (nav_msgs/Path).
"""
import math, os
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from mfe_msgs.msg import Track
from mfe_path_planning.path_planner_node import _make_skidpad_path
from mfe_validation.common import QOS, car_start, read_track, yaw_of

_EUFS_MASTER = os.environ.get('EUFS_MASTER', os.path.expanduser('~/Develop/MFE26-eufs-sim'))
# Older upstream layout nests the tracks under eufs_sim/.
CSV = next((c for c in (f'{_EUFS_MASTER}/eufs_tracks/csv/skidpad.csv', f'{_EUFS_MASTER}/eufs_sim/eufs_tracks/csv/skidpad.csv')
            if os.path.exists(c)), f'{_EUFS_MASTER}/eufs_tracks/csv/skidpad.csv')


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


def check_registration(transformer, lib_offset, origin, yaw0, max_err, max_yaw):
    """Accept a library fit if the template start lands on the first SLAM position with matching heading. Returns (ok, err_m, yaw_err_deg)."""
    st = transformer(np.array([0.0, 0.0]) + lib_offset, 0.0)[0]
    ce = transformer(np.array([14.4, 0.0]) + lib_offset, 0.0)[0]
    err = float(np.linalg.norm(st - origin))
    yaw_err = abs(math.degrees((math.atan2(ce[1] - st[1], ce[0] - st[0]) - yaw0 + math.pi) % (2 * math.pi) - math.pi))
    return err <= max_err and yaw_err <= max_yaw, err, yaw_err


class SkidpadPlanner(Node):
    def __init__(self):
        super().__init__('skidpad_planner')
        self.declare_parameter('registration_mode', 'library')
        self.declare_parameter('gate', 1.5)              # custom: landmark-to-template match gate (m)
        self.declare_parameter('min_match', 6)
        self.declare_parameter('max_resid', 0.5)
        self.declare_parameter('max_shift', 2.0)         # custom: max template shift (m)
        self.declare_parameter('max_start_err_m', 2.0)   # library: max start error (m)
        self.declare_parameter('max_start_yaw_deg', 15.0)  # library: max heading error (deg)
        self.declare_parameter('max_reloc_rejects', 200)   # library: give up after this many rejected fits
        self.registration_mode = self.get_parameter('registration_mode').value
        if self.registration_mode not in ('library', 'custom'):
            raise ValueError(f"registration_mode must be 'library' or 'custom', got {self.registration_mode!r}")
        g = self.get_parameter
        self.gate, self.min_match = g('gate').value, g('min_match').value
        self.max_resid, self.max_shift = g('max_resid').value, g('max_shift').value
        self.max_start_err, self.max_start_yaw = g('max_start_err_m').value, g('max_start_yaw_deg').value
        self.max_rejects = g('max_reloc_rejects').value
        self.rejects = 0
        self.gave_up = False
        self.start_pos = self.start_yaw = None          # first SLAM pose: template start, facing +x

        rows = read_track(CSV)
        x0, y0, yaw = car_start(rows)
        m = np.array([[float(r['x']) - x0, float(r['y']) - y0] for r in rows if r['tag'] in ('blue', 'yellow', 'orange', 'big_orange')])
        c, s = math.cos(-yaw), math.sin(-yaw)
        self.tpl_cones = np.c_[c * m[:, 0] - s * m[:, 1], s * m[:, 0] + c * m[:, 1]]   # start-relative, same as SLAM
        self.tpl_path = _make_skidpad_path()

        # Library: fixed translation aligning the circle centres of the two frames.
        self.relocalizer = None
        self.lib_offset = np.zeros(2)       # template -> library known frame
        self.transformer = None             # library: known -> car frame
        self.vehicle_position = None        # library: latest SLAM position
        self.vehicle_direction = None       # library: latest SLAM heading (unit vector)
        if self.registration_mode == 'library':
            from fsd_path_planning.relocalization.skidpad.skidpad_relocalizer import (
                SkidpadRelocalizer, calculate_reference_centers_for_skidpad_path)
            self.relocalizer = SkidpadRelocalizer()
            lib_centers = self.relocalizer.reference_centers
            tpl_centers = calculate_reference_centers_for_skidpad_path(self.tpl_path)
            self.lib_offset = lib_centers.mean(0) - tpl_centers.mean(0)
            self.get_logger().info(f'library frame offset from template: {self.lib_offset.round(3).tolist()} m')

        self.T = (0.0, 0.0, 0.0)            # custom: (theta, tx, ty)

        # Publish only a window ahead of the car: pure_pursuit snaps to the nearest point, and the 2 laps overlap.
        self.idx = 0
        self.window = 25       # ~24 m ahead, under one 57 m lap
        self.create_subscription(Odometry, '/slam/odom', self.on_pose, QOS)
        self.pub = self.create_publisher(Path, '/planning/centerline', QOS)
        self.create_subscription(Track, '/skidpad/cones', self.on_cones, QOS)
        self.n = 0
        self.get_logger().info(f'skidpad_planner up ({self.registration_mode}): {len(self.tpl_cones)} template cones, {len(self.tpl_path)} path points')

    def template_to_car(self, pts):
        """Map template-frame points into the car (SLAM) frame using the current registration."""
        if self.registration_mode == 'library':
            if self.transformer is None:
                return np.asarray(pts, dtype=float)
            return np.array([self.transformer(p + self.lib_offset, 0.0)[0] for p in pts])
        return apply(self.T, pts)

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
        if self.registration_mode == 'library':
            yaw = yaw_of(msg.pose.pose.orientation)
            if self.start_pos is None:
                self.start_pos, self.start_yaw = p, yaw
            self.vehicle_position = p
            self.vehicle_direction = np.array([math.cos(yaw), math.sin(yaw)])
        q = self.template_to_car(self.tpl_path[self.idx:self.idx + 10])   # small window: cannot jump to the overlapping next lap
        self.idx += int(np.linalg.norm(q - p, axis=1).argmin())

    def on_cones(self, msg):
        lm = np.array([[c.location.x, c.location.y] for c in msg.track]).reshape(-1, 2)

        if self.registration_mode == 'library':
            # Needs the car pose and a few cones; latches once it succeeds.
            if self.transformer is None and not self.gave_up and self.vehicle_position is not None and len(lm) >= 3:
                try:
                    self.relocalizer.attempt_relocalization_calculation(list(lm), self.vehicle_position, self.vehicle_direction)
                except Exception as e:  # library raises on degenerate cone sets; retry on the next message
                    self.get_logger().warn(f'library relocalizer error: {e}', throttle_duration_sec=5.0)
                if self.relocalizer.is_relocalized:
                    tr = self.relocalizer._known_map_to_original_transformer
                    ok, err, yaw_err = check_registration(tr, self.lib_offset, self.start_pos, self.start_yaw,
                                                          self.max_start_err, self.max_start_yaw)
                    if ok:
                        self.transformer = tr
                        c = self.template_to_car(np.array([[14.4, 0.0]]))[0]
                        self.get_logger().info(f'library relocalized with {len(lm)} landmarks; template centre (14.4,0) -> car frame ({c[0]:.2f},{c[1]:.2f}) m')
                    else:
                        # Single-shot relocalizer: retry with a fresh instance.
                        self.rejects += 1
                        self.get_logger().warn(f'rejected library fit #{self.rejects}: start off by {err:.2f} m, heading off by {yaw_err:.1f} deg')
                        self.relocalizer = type(self.relocalizer)()
                        if self.rejects >= self.max_rejects:
                            self.gave_up = True
                            self.get_logger().error(f'{self.rejects} library fits rejected; using the un-registered template')
        elif len(lm) >= self.min_match:
            r = self.fit(lm)
            if r:
                self.T = r[0]
                self.n += 1
                if self.n % 20 == 1:
                    self.get_logger().info(f'template fit: {r[1]} cones matched, resid {r[2]:.2f} m, shift ({self.T[1]:.2f},{self.T[2]:.2f}) m, rot {math.degrees(self.T[0]):.1f} deg')

        path = Path(); path.header.frame_id = 'map'; path.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.template_to_car(self.tpl_path[self.idx:self.idx + self.window]):
            ps = PoseStamped(); ps.header = path.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.orientation.w = float(x), float(y), 1.0
            path.poses.append(ps)
        self.pub.publish(path)


def main():
    rclpy.init(); rclpy.spin(SkidpadPlanner()); rclpy.shutdown()
