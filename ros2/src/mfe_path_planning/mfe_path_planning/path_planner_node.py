#!/usr/bin/env python3
"""
ft-fsd-path-planning ROS 2 wrapper node.

Library: https://github.com/papalotis/ft-fsd-path-planning
Install: pip install ft-fsd-path-planning

Subscribes:
    /planning/cones       (mfe_msgs/Track)        — fused, colored cone list
    /ekf/output           (nav_msgs/Odometry)      — vehicle pose + heading

Publishes:
    /planning/centerline  (nav_msgs/Path)          — computed centerline
    /planning/track_left  (nav_msgs/Path)          — left boundary
    /planning/track_right (nav_msgs/Path)          — right boundary
"""

import math
import numpy as np

try:
    from scipy.optimize import minimize as _sp_minimize
    _SCIPY_AVAILABLE = True
except ImportError:
    _SCIPY_AVAILABLE = False

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from mfe_msgs.msg import Cone, Track

try:
    from fsd_path_planning import PathPlanner, ConeTypes, MissionTypes
    _LIB_AVAILABLE = True
except ImportError:
    try:
        from ft_fsd_path_planning import PathPlanner, ConeTypes, MissionTypes
        _LIB_AVAILABLE = True
    except ImportError:
        _LIB_AVAILABLE = False


_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# mfe_msgs/Cone color enum → fsd_path_planning ConeTypes
# BLUE=0 is left boundary, YELLOW=1 is right boundary (FSAE standard)
# fsd_path_planning ConeTypes: UNKNOWN=0, RIGHT=1, LEFT=2,
#   START_FINISH_AREA=3, START_FINISH_LINE=4
if _LIB_AVAILABLE:
    _MFE_TO_FSD = {
        Cone.BLUE:         ConeTypes.LEFT,
        Cone.YELLOW:       ConeTypes.RIGHT,
        Cone.ORANGE_BIG:   ConeTypes.UNKNOWN,   # keep planner from triggering finish logic
        Cone.ORANGE_SMALL: ConeTypes.UNKNOWN,
        Cone.UNKNOWN:      ConeTypes.UNKNOWN,
    }
else:
    _MFE_TO_FSD = {}


def _yaw_from_quaternion(q) -> float:
    """Extract yaw angle (radians) from a geometry_msgs/Quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _path_from_xy(xy: np.ndarray, frame_id: str, stamp) -> Path:
    """Convert Nx2 numpy array of (x,y) points to nav_msgs/Path."""
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = stamp
    for point in xy:
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = stamp
        ps.pose.position.x = float(point[0])
        ps.pose.position.y = float(point[1])
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
    return path


# Peanut figure-8 centerline — right loop first, then left loop.
# Generated from EUFS peanut.csv boundary cones (midpoint of blue/yellow).
# Driving sequence: (0,0) → gate (x≈6) → right loop → crossing → left loop → (0,0)
_PEANUT_PATH = np.array([
    [0.0000, 0.0000], [1.5000, 0.0000], [3.0000, -0.0500], [4.5000, -0.0500],
    [6.4599, -0.0319], [7.1290, -0.1867], [7.7982, -0.3415], [8.4673, -0.4963],
    [9.1041, -0.7449], [9.7345, -1.0104], [10.3404, -1.3364], [10.9105, -1.7183],
    [11.4670, -2.1226], [12.0558, -2.4656], [12.6917, -2.7201], [13.3275, -2.9747],
    [14.0055, -3.0360], [14.6929, -3.0379], [15.3705, -2.9622], [16.0337, -2.8081],
    [16.6901, -2.6109], [17.3474, -2.4135], [17.9904, -2.1686], [18.5806, -1.8404],
    [19.0732, -1.3613], [19.5446, -0.8631], [19.9219, -0.2917], [20.2769, 0.2944],
    [20.6280, 0.8785], [20.9181, 1.4823], [21.1407, 2.1259], [21.3530, 2.7756],
    [21.4752, 3.4380], [21.5358, 4.1193], [21.5963, 4.8005], [21.5458, 5.4801],
    [21.4670, 6.1593], [21.3582, 6.8272], [21.1483, 7.4584], [20.9381, 8.0896],
    [20.6231, 8.6745], [20.2607, 9.2099], [19.8520, 9.7260], [19.4037, 10.1961],
    [18.9085, 10.5554], [18.3547, 10.8738], [17.8009, 11.1922], [17.2717, 11.3447],
    [16.7212, 11.4423], [16.1815, 11.5277], [15.6796, 11.5247], [15.1473, 11.4773],
    [14.6149, 11.4298], [14.0403, 11.2664], [13.4647, 11.1006], [12.8921, 10.8660],
    [12.3140, 10.5494], [11.7215, 10.2462], [11.1594, 9.8687], [10.6075, 9.4659],
    [10.0215, 9.1201], [9.4631, 8.7399], [8.9085, 8.3550], [8.3674, 7.9559],
    [7.8266, 7.5566], [7.2732, 7.2286], [6.7144, 6.9227], [6.1520, 6.6612],
    [5.5796, 6.4637], [4.9792, 6.4180], [4.3439, 6.4891], [3.7133, 6.6530],
    [3.1570, 7.0577], [2.6054, 7.4677], [2.1336, 7.9614], [1.7003, 8.4933],
    [1.2575, 9.0160], [0.7693, 9.4997], [0.2674, 9.9705], [-0.2362, 10.4393],
    [-0.7717, 10.8697], [-1.3099, 11.2968], [-1.8601, 11.7064], [-2.4224, 12.1000],
    [-2.9888, 12.4859], [-3.5770, 12.8373], [-4.1741, 13.1768], [-4.7888, 13.4655],
    [-5.4438, 13.6613], [-6.1047, 13.8455], [-6.7748, 13.9741], [-7.4606, 14.0236],
    [-8.1464, 14.0730], [-8.8262, 14.0359], [-9.5026, 13.9092], [-10.1732, 13.7690],
    [-10.7608, 13.4281], [-11.3332, 13.0470], [-11.8912, 12.6487], [-12.3815, 12.1724],
    [-12.8444, 11.6657], [-13.2777, 11.1313], [-13.6871, 10.5809], [-14.0290, 9.9838],
    [-14.3675, 9.3851], [-14.6298, 8.7532], [-14.8555, 8.1037], [-15.0677, 7.4508],
    [-15.2199, 6.7808], [-15.3722, 6.1109], [-15.4953, 5.4346], [-15.5265, 4.7552],
    [-15.5349, 4.0734], [-15.5075, 3.3878], [-15.4625, 2.7060], [-15.3335, 2.0335],
    [-15.1873, 1.3623], [-15.0104, 0.6990], [-14.7959, 0.0455], [-14.5761, -0.6057],
    [-14.2418, -1.2047], [-13.8850, -1.7919], [-13.5202, -2.3746], [-13.1076, -2.9144],
    [-12.6772, -3.4398], [-12.2104, -3.9302], [-11.6948, -4.3306], [-11.1218, -4.6770],
    [-10.5360, -4.9343], [-9.9510, -5.0921], [-9.3427, -5.2034], [-8.7321, -5.1111],
    [-8.1271, -5.0087], [-7.5291, -4.8414], [-6.9572, -4.6012], [-6.3854, -4.3609],
    [-5.8160, -4.1037], [-5.2622, -3.7484], [-4.7084, -3.3932], [-4.1489, -3.0273],
    [-3.6105, -2.5990], [-3.0583, -2.1985], [-2.4759, -1.8690], [-1.9006, -1.5312],
    [-1.3130, -1.2400], [-0.6864, -1.0161], [-0.0599, -0.7922], [0.5845, -0.5989],
    [1.2445, -0.4270], [1.9211, -0.3349], [2.6079, -0.3042], [3.2953, -0.2969],
    [3.9801, -0.2572], [4.6649, -0.2176],
    [3.5000, -0.1000], [2.0000, -0.0500], [0.5000, 0.0000], [0.0000, 0.0000],
])


def _make_skidpad_path() -> np.ndarray:
    """
    Generate the full skidpad trajectory for the EUFS Gazebo track.

    Sequence (per user spec): 2 laps LEFT (upper circle, counterclockwise)
    then 2 laps RIGHT (lower circle, clockwise), then exit.

    Parameters tuned to the EUFS standard skidpad track dimensions:
      inner cone radius = 7.625 m (standard FSAE), outer = 10.6 m
      path radius = midpoint = 9.1 m, circle centres at (14.4, ±9.3)
    """
    cx, cy_l, cy_r = 14.4, 9.3, -9.3   # x shared, y for left/right circles
    R = 9.1                              # centreline radius

    N = 50    # waypoints per full lap (keep message small; ~220 total waypoints)

    # Entry straight from (0,0) to left-circle 6-o'clock entry
    n_entry = 40
    entry = np.column_stack([
        np.linspace(0, cx, n_entry),
        np.zeros(n_entry),
    ])

    # Left circle (upper, counterclockwise): entry at 6 o'clock (θ = -π/2), car moving +x
    theta_l = np.linspace(-math.pi / 2, -math.pi / 2 + 4 * math.pi, N * 2 + 1)
    left_circ = np.column_stack([cx + R * np.cos(theta_l), cy_l + R * np.sin(theta_l)])

    # Right circle (lower, clockwise): entry at 12 o'clock (θ = +π/2), car moving +x
    theta_r = np.linspace(math.pi / 2, math.pi / 2 - 4 * math.pi, N * 2 + 1)
    right_circ = np.column_stack([cx + R * np.cos(theta_r), cy_r + R * np.sin(theta_r)])

    # Exit straight from right-circle 12 o'clock back to y=0 then to end zone
    n_exit = 40
    exit_pts = np.column_stack([
        np.linspace(cx, 40.0, n_exit),
        np.zeros(n_exit),
    ])

    # Stitch: omit duplicate junction point between sections
    return np.vstack([entry[:-1], left_circ, right_circ, exit_pts])


def _min_curvature_path(
        centerline_xy: np.ndarray,
        left_xy: np.ndarray,
        right_xy: np.ndarray,
        safety: float = 0.8) -> np.ndarray:
    """
    Minimum curvature racing line optimizer (Heilmeier et al. 2019, QP formulation).

    Each centerline waypoint slides laterally by alpha_i within the track
    boundaries.  Minimises sum||q_{i+1} - 2*q_i + q_{i-1}||² subject to
    alpha_i in [-w_right_i, w_left_i], which maximises effective corner
    radii → outside-inside-outside racing line.

    No new dependencies: uses scipy.optimize which is already present.
    Falls back to the original centerline if scipy is missing or inputs are thin.
    """
    if not _SCIPY_AVAILABLE:
        return centerline_xy

    N = len(centerline_xy)
    if N < 4 or len(left_xy) == 0 or len(right_xy) == 0:
        return centerline_xy

    # Unit tangents (central differences) and left-pointing normals
    tg = np.zeros_like(centerline_xy)
    tg[1:-1] = centerline_xy[2:] - centerline_xy[:-2]
    tg[0]    = centerline_xy[1]  - centerline_xy[0]
    tg[-1]   = centerline_xy[-1] - centerline_xy[-2]
    tg /= np.linalg.norm(tg, axis=1, keepdims=True).clip(1e-9)
    nrm = np.column_stack([-tg[:, 1], tg[:, 0]])  # rotate 90° CCW → left

    # Half-widths: distance from each centerline point to the nearest
    # left / right boundary cone, scaled by the safety margin.
    def _half_widths(boundary_xy: np.ndarray) -> np.ndarray:
        w = np.empty(N)
        for i in range(N):
            w[i] = np.linalg.norm(boundary_xy - centerline_xy[i], axis=1).min()
        return w * safety

    w_l = _half_widths(left_xy)
    w_r = _half_widths(right_xy)
    bounds = [(-w_r[i], w_l[i]) for i in range(N)]

    # Objective: sum of squared discrete curvatures, with analytic gradient.
    # q_i = centerline_i + alpha_i * normal_i
    # d2_i = q_{i+1} - 2*q_i + q_{i-1}   (discrete second derivative)
    # d(||d2_i||²)/d(alpha_j):
    #   j == i-1 → +1 coefficient on path[i-1]  → dot(2*d2_i,  nrm[j])
    #   j == i   → -2 coefficient on path[i]    → dot(2*d2_i, -2*nrm[j])
    #   j == i+1 → +1 coefficient on path[i+1]  → dot(2*d2_i,  nrm[j])
    def _f_and_g(alpha: np.ndarray):
        q  = centerline_xy + alpha[:, None] * nrm
        d2 = q[2:] - 2.0 * q[1:-1] + q[:-2]          # (N-2, 2)
        obj = float(np.einsum('ij,ij->', d2, d2))
        td  = 2.0 * d2                                  # (N-2, 2)
        grad = np.zeros(N)
        grad[:-2]  += np.einsum('ij,ij->i', td,          nrm[:-2])
        grad[1:-1] += np.einsum('ij,ij->i', td, -2.0  * nrm[1:-1])
        grad[2:]   += np.einsum('ij,ij->i', td,          nrm[2:])
        return obj, grad

    res = _sp_minimize(
        _f_and_g, np.zeros(N), jac=True, bounds=bounds,
        method='L-BFGS-B',
        options={'maxiter': 500, 'ftol': 1e-10, 'gtol': 1e-6},
    )
    return centerline_xy + res.x[:, None] * nrm


class PathPlannerNode(Node):

    def __init__(self):
        super().__init__('path_planner_node')

        if not _LIB_AVAILABLE:
            self.get_logger().error(
                'ft_fsd_path_planning is not installed. '
                'Run: pip install ft-fsd-path-planning')

        # ---------- Parameters ----------
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('use_racing_line', True)
        self.declare_parameter('mission', 'autocross')  # autocross | trackdrive | acceleration | skidpad

        self._map_frame = self.get_parameter('map_frame').value
        mission_str = self.get_parameter('mission').value
        self._use_racing_line = self.get_parameter('use_racing_line').value
        self._mission = mission_str

        _HARDCODED_MISSIONS = {'skidpad', 'peanut'}
        if _LIB_AVAILABLE and mission_str not in _HARDCODED_MISSIONS:
            mission_map = {
                'autocross':   MissionTypes.autocross,
                'trackdrive':  MissionTypes.trackdrive,
                'acceleration': MissionTypes.acceleration,
            }
            mission = mission_map.get(mission_str, MissionTypes.autocross)
            self._planner = PathPlanner(mission)
        else:
            self._planner = None

        # Pre-compute hardcoded paths for missions ft-fsd can't handle
        self._skidpad_path_published = False
        if mission_str == 'skidpad':
            self._skidpad_path = _make_skidpad_path()
            self._peanut_path  = None
            self.get_logger().info(
                f'PathPlannerNode started. Mission: skidpad | '
                f'hardcoded path: {len(self._skidpad_path)} waypoints, '
                f'sequence: 2 laps LEFT + 2 laps RIGHT + exit')
        elif mission_str == 'peanut':
            self._skidpad_path = None
            self._peanut_path  = _PEANUT_PATH
            self.get_logger().info(
                f'PathPlannerNode started. Mission: peanut | '
                f'hardcoded path: {len(self._peanut_path)} waypoints, '
                f'sequence: (0,0) → right loop → crossing → left loop → (0,0)')
        else:
            self._skidpad_path = None
            self._peanut_path  = None
            self.get_logger().info(f'PathPlannerNode started. Mission: {mission_str}')

        # ---------- Counters ----------
        self._opt_ok = 0
        self._opt_fail = 0

        # ---------- State ----------
        self._car_pos = np.array([0.0, 0.0])
        self._car_dir = np.array([1.0, 0.0])   # unit vector, default facing +X
        self._odom_received = False

        # Cone map: accumulated global map of all seen cones across frames.
        # Key: (rx, ry) rounded to nearest 0.5 m grid. Value: cone color (ConeTypes).
        self._cone_map: dict = {}

        # ---------- Subscribers ----------
        self.create_subscription(Track, '/planning/cones', self._cones_cb, _QOS)
        self.create_subscription(Odometry, '/ekf/output', self._odom_cb, _QOS)

        # ---------- Publishers ----------
        self._pub_centerline = self.create_publisher(Path, '/planning/centerline', _QOS)
        self._pub_left       = self.create_publisher(Path, '/planning/track_left',  _QOS)
        self._pub_right      = self.create_publisher(Path, '/planning/track_right', _QOS)

    # ------------------------------------------------------------------ #

    def _odom_cb(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        self._car_pos = np.array([pos.x, pos.y])
        yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        self._car_dir = np.array([math.cos(yaw), math.sin(yaw)])
        self._odom_received = True

    def _cones_cb(self, msg: Track) -> None:
        # Hardcoded missions: publish pre-computed path on every cone callback
        # so pure_pursuit picks it up reliably on startup.
        if self._mission in ('skidpad', 'peanut'):
            if self._odom_received:
                hardcoded = self._skidpad_path if self._mission == 'skidpad' else self._peanut_path
                stamp = self.get_clock().now().to_msg()
                self._pub_centerline.publish(
                    _path_from_xy(hardcoded, self._map_frame, stamp))
                if not self._skidpad_path_published:
                    self._skidpad_path_published = True
                    self.get_logger().info(
                        f'{self._mission} path first publish: {len(hardcoded)} waypoints')
            return

        if not _LIB_AVAILABLE or self._planner is None:
            return

        if not self._odom_received:
            self.get_logger().warn('No odometry yet — skipping path calculation.', throttle_duration_sec=5.0)
            return

        if not msg.track:
            return

        # --- Cone map accumulation ---
        # Merge incoming cones into the global map (rounded to 0.5 m grid).
        # This builds up a global picture of all seen cones as the car drives.
        _GRID = 0.5
        _CONE_MAP_MAX = 500
        for cone in msg.track:
            rx = round(cone.location.x / _GRID) * _GRID
            ry = round(cone.location.y / _GRID) * _GRID
            key = (rx, ry)
            if key not in self._cone_map:
                if len(self._cone_map) >= _CONE_MAP_MAX:
                    # Remove an arbitrary oldest entry to stay within cap
                    self._cone_map.pop(next(iter(self._cone_map)))
                self._cone_map[key] = _MFE_TO_FSD.get(cone.color, ConeTypes.UNKNOWN)
            else:
                # If we get a colored update for a previously unknown cone, upgrade it
                new_type = _MFE_TO_FSD.get(cone.color, ConeTypes.UNKNOWN)
                if self._cone_map[key] == ConeTypes.UNKNOWN and new_type != ConeTypes.UNKNOWN:
                    self._cone_map[key] = new_type

        # Build cones_by_type: list of Nx2 arrays indexed by ConeTypes enum value
        # ft-fsd-path-planning expects: [unknown[], right[], left[], orange_big[], orange_small[]]
        #
        # Visibility filter: keep only cones from the accumulated map that are:
        #   (a) within CONE_VIS_R metres of the car, AND
        #   (b) not more than CONE_BEHIND_M metres behind the car's heading direction.
        # This prevents far-away and behind-car cones on crossing tracks (figure-8/peanut)
        # from confusing the planner with cones from the other loop.
        # The full accumulated map (not just the current frame) is used so perception mode
        # builds up a global picture as the car drives around.
        CONE_VIS_R    = 20.0   # max radius (m) — keep large for sparse tracks (small_track has 17m gaps)
        CONE_BEHIND_M = 12.0   # allow up to 12 m behind car — 6m was cutting left-boundary cones mid-turn
        n_types = len(ConeTypes)
        buckets: list[list] = [[] for _ in range(n_types)]

        for (rx, ry), cone_type in self._cone_map.items():
            dx = rx - self._car_pos[0]
            dy = ry - self._car_pos[1]
            if dx * dx + dy * dy > CONE_VIS_R * CONE_VIS_R:
                continue
            # forward projection: positive = ahead of car, negative = behind
            fwd = dx * self._car_dir[0] + dy * self._car_dir[1]
            if fwd < -CONE_BEHIND_M:
                continue
            buckets[cone_type].append([rx, ry])

        cones_by_type = [
            np.array(b, dtype=np.float64).reshape(-1, 2) if b
            else np.zeros((0, 2), dtype=np.float64)
            for b in buckets
        ]

        n_left    = len(buckets[ConeTypes.LEFT])
        n_right   = len(buckets[ConeTypes.RIGHT])
        n_unknown = len(buckets[ConeTypes.UNKNOWN])
        self.get_logger().info(
            f'planner input: left={n_left} right={n_right} unknown={n_unknown} '
            f'pos=({self._car_pos[0]:.1f},{self._car_pos[1]:.1f}) '
            f'dir=({self._car_dir[0]:.2f},{self._car_dir[1]:.2f})',
            throttle_duration_sec=2.0)

        if n_left + n_right == 0:
            self.get_logger().warn(
                f'No colored cones visible — skipping planner.',
                throttle_duration_sec=5.0)
            return

        try:
            result = self._planner.calculate_path_in_global_frame(
                cones_by_type,
                self._car_pos,
                self._car_dir,
                return_intermediate_results=True,
            )
            path_xy, left_xy, right_xy, *_ = result
        except Exception as e:
            self.get_logger().error(
                f'Path calculation failed ({type(e).__name__}): {e} | '
                f'left={n_left} right={n_right} unknown={n_unknown}',
                throttle_duration_sec=2.0)
            return

        self.get_logger().info(
            f'planner OK: path_pts={len(path_xy) if path_xy is not None else 0}',
            throttle_duration_sec=2.0)

        stamp = self.get_clock().now().to_msg()
        frame = self._map_frame

        # final_path columns: (spline_parameter, path_x, path_y, curvature)
        # Use columns 1:3 for actual (x, y); col 0 is the spline parameter, not a coordinate.
        # sorted_left / sorted_right are Mx2 (x,y) already — use :2
        if path_xy is not None and len(path_xy) > 0:
            centerline = np.array(path_xy)[:, 1:3]

            # Run racing line optimizer when we have enough points and boundaries
            if (self._use_racing_line and len(path_xy) > 3
                    and left_xy is not None and len(left_xy) > 3
                    and right_xy is not None and len(right_xy) > 3):
                left   = np.array(left_xy)[:, :2]
                right  = np.array(right_xy)[:, :2]
                self.get_logger().info(
                        f'Racing line : {self._opt_ok} ok — {self._opt_fail} fail.',
                        throttle_duration_sec=5.0)
                try:
                    optimized = _min_curvature_path(centerline, left, right, safety=0.7)
                    self.get_logger().info(
                        'Racing line optimizer applied.',
                        throttle_duration_sec=5.0)
                    self._opt_ok += 1
                except Exception as opt_e:
                    self.get_logger().warn(
                        f'Racing line optimizer failed ({type(opt_e).__name__}): {opt_e} — '
                        'falling back to raw centerline.',
                        throttle_duration_sec=5.0)
                    optimized = centerline
                    self._opt_fail += 1
            else:
                optimized = centerline

            self._pub_centerline.publish(_path_from_xy(optimized, frame, stamp))

        if left_xy is not None and len(left_xy) > 0:
            self._pub_left.publish(_path_from_xy(np.array(left_xy)[:, :2], frame, stamp))
        if right_xy is not None and len(right_xy) > 0:
            self._pub_right.publish(_path_from_xy(np.array(right_xy)[:, :2], frame, stamp))


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
