"""Skidpad planner driven by detected cones: 2 laps left, 2 laps right, exit.

The skidpad layout is fixed by the rules, so instead of a path hardcoded in the start frame, this registers the known skidpad
layout onto the SLAM landmarks and drives the template path through that fit. If the landmarks show the track is where the
template says, the path equals the stock one; if SLAM/start pose is off, the fit shifts the whole path onto the cones that
were actually seen.

Registration modes (ROS param `registration_mode`):
- 'library' (default): fsd_path_planning.relocalization.skidpad.SkidpadRelocalizer (AMZ ft-fsd-path-planning, circle fitting +
  DBSCAN clustering). It latches the first plausible fit; the template is mapped with its known->car-frame transformer.
- 'custom': hand-rolled SE2 Kabsch/ICP against the eufs skidpad.csv cone layout (re-fitted every cone message).

In:  mfe_msgs/Track landmarks (start-relative frame) on /skidpad/cones, nav_msgs/Odometry on /slam/odom.
Out: /planning/centerline (nav_msgs/Path).
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
_EUFS_MASTER = os.environ.get('EUFS_MASTER', os.path.expanduser('~/Develop/MFE26-eufs-sim'))
CSV = f'{_EUFS_MASTER}/eufs_sim/eufs_tracks/csv/skidpad.csv'


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
    """Sanity-check a library fit: the template start (0,0) must land on the car's first SLAM position and the
    start->centre (template +x) direction must match its first heading. Returns (ok, err_m, yaw_err_deg)."""
    st = transformer(np.array([0.0, 0.0]) + lib_offset, 0.0)[0]
    ce = transformer(np.array([14.4, 0.0]) + lib_offset, 0.0)[0]
    err = float(np.linalg.norm(st - origin))
    yaw_err = abs(math.degrees((math.atan2(ce[1] - st[1], ce[0] - st[0]) - yaw0 + math.pi) % (2 * math.pi) - math.pi))
    return err <= max_err and yaw_err <= max_yaw, err, yaw_err


class SkidpadPlanner(Node):
    def __init__(self):
        super().__init__('skidpad_planner')
        self.declare_parameter('registration_mode', 'library')  # 'library' (default) or 'custom'
        self.declare_parameter('gate', 1.5)          # landmark <-> template cone match distance (m) — custom mode only
        self.declare_parameter('min_match', 6)       # custom mode only
        self.declare_parameter('max_resid', 0.5)     # custom mode only
        self.declare_parameter('max_shift', 2.0)     # reject fits that move the template more than this (m) — custom only
        self.registration_mode = self.get_parameter('registration_mode').value
        if self.registration_mode not in ('library', 'custom'):
            raise ValueError(f"registration_mode must be 'library' or 'custom', got {self.registration_mode!r}")
        self.gate, self.min_match = self.get_parameter('gate').value, self.get_parameter('min_match').value
        self.max_resid, self.max_shift = self.get_parameter('max_resid').value, self.get_parameter('max_shift').value
        # Library mode: the fit is occasionally wrong, so it must put the template start at the car's start pose.
        self.declare_parameter('max_start_err_m', 2.0)      # max distance of the mapped template start from the first SLAM position
        self.declare_parameter('max_start_yaw_deg', 15.0)   # max heading error of the mapped start->centre vs the first SLAM yaw
        self.declare_parameter('max_reloc_rejects', 200)    # give up (un-registered template) after this many rejected fits (BS03 replay needed 40)
        self.max_start_err = self.get_parameter('max_start_err_m').value
        self.max_start_yaw = self.get_parameter('max_start_yaw_deg').value
        self.max_rejects = self.get_parameter('max_reloc_rejects').value
        self.rejects = 0
        self.gave_up = False
        self.start_pos = self.start_yaw = None      # first SLAM pose (the template start, facing template +x)

        # Template cones (custom mode)
        rows = list(csv.DictReader(open(CSV)))
        cs = [r for r in rows if r['tag'] == 'car_start'][0]
        x0, y0, yaw = float(cs['x']), float(cs['y']), float(cs['direction'])
        m = np.array([[float(r['x']) - x0, float(r['y']) - y0] for r in rows if r['tag'] in ('blue', 'yellow', 'orange', 'big_orange')])
        c, s = math.cos(-yaw), math.sin(-yaw)
        self.tpl_cones = np.c_[c * m[:, 0] - s * m[:, 1], s * m[:, 0] + c * m[:, 1]]   # start-relative frame, same as SLAM
        self.tpl_path = _make_skidpad_path()

        # Library mode: the relocalizer's known frame is defined by its own reference path, not by _make_skidpad_path().
        # Bridge the two with a fixed translation that aligns the template's circle centres onto the library's.
        self.relocalizer = None
        self.lib_offset = np.zeros(2)       # template frame -> library known frame
        self.transformer = None             # library mode: known frame -> car (SLAM) frame, callable (pos2d, yaw)
        self.vehicle_position = None        # library mode: latest SLAM position (first value latched by the relocalizer)
        self.vehicle_direction = None       # library mode: latest SLAM heading as a unit 2D vector
        if self.registration_mode == 'library':
            from fsd_path_planning.relocalization.skidpad.skidpad_relocalizer import (
                SkidpadRelocalizer, calculate_reference_centers_for_skidpad_path)
            self.relocalizer = SkidpadRelocalizer()
            lib_centers = self.relocalizer.reference_centers
            tpl_centers = calculate_reference_centers_for_skidpad_path(self.tpl_path)
            self.lib_offset = lib_centers.mean(0) - tpl_centers.mean(0)
            self.get_logger().info(f'library frame offset from template: {self.lib_offset.round(3).tolist()} m')

        # Transform state (custom mode)
        self.T = (0.0, 0.0, 0.0)  # (theta, tx, ty)

        # Progress along the template (monotonic). pure_pursuit resets its own path index to 0 on every new path and then
        # snaps to the nearest point anywhere on it, so a path that revisits the same circle (2 laps) traps it on lap 1.
        # Publishing only the path ahead of the car (never lap 1 again) makes 2L + 2R work.
        self.idx = 0
        self.window = 25       # points published ahead (~24 m, under one 57 m lap so the path never overlaps itself)
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
            q = msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            if self.start_pos is None:
                self.start_pos, self.start_yaw = p, yaw
            self.vehicle_position = p
            self.vehicle_direction = np.array([math.cos(yaw), math.sin(yaw)])
        q = self.template_to_car(self.tpl_path[self.idx:self.idx + 10])   # small window: can't jump to the overlapping next lap
        self.idx += int(np.linalg.norm(q - p, axis=1).argmin())

    def on_cones(self, msg):
        lm = np.array([[c.location.x, c.location.y] for c in msg.track]).reshape(-1, 2)

        if self.registration_mode == 'library':
            # Library relocalizer: needs the car pose (it latches the first one) and a few cones; latches once it succeeds.
            if self.transformer is None and not self.gave_up and self.vehicle_position is not None and len(lm) >= 3:
                try:
                    self.relocalizer.attempt_relocalization_calculation(
                        list(lm), self.vehicle_position, self.vehicle_direction)
                except Exception as e:  # the library raises on degenerate cone sets; keep trying on the next message
                    self.get_logger().warn(f'library relocalizer error: {e}', throttle_duration_sec=5.0)
                if self.relocalizer.is_relocalized:
                    # The relocalizer's fwd transform maps original -> known; we need known -> car for the template.
                    tr = self.relocalizer._known_map_to_original_transformer
                    ok, err, yaw_err = check_registration(tr, self.lib_offset, self.start_pos, self.start_yaw,
                                                          self.max_start_err, self.max_start_yaw)
                    if ok:
                        self.transformer = tr
                        c = self.template_to_car(np.array([[14.4, 0.0]]))[0]
                        self.get_logger().info(f'library relocalized with {len(lm)} landmarks; template centre (14.4,0) -> car frame ({c[0]:.2f},{c[1]:.2f}) m')
                    else:
                        # The relocalizer is single-shot (latches once), so retry with a fresh instance on the next message.
                        self.rejects += 1
                        self.get_logger().warn(f'rejected library fit #{self.rejects}: start off by {err:.2f} m, heading off by {yaw_err:.1f} deg')
                        self.relocalizer = type(self.relocalizer)()
                        if self.rejects >= self.max_rejects:
                            self.gave_up = True
                            self.get_logger().error(f'{self.rejects} library fits rejected; using the un-registered template')
        else:
            if len(lm) >= self.min_match:
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
