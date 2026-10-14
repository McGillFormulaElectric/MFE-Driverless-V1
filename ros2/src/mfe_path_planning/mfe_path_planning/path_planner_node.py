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

from mfe_path_planning.amz_planner import AmzPlanner


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


# ---------------------------------------------------------------------------
# MFE25 vehicle physical constants (from vehicle.yaml)
# ---------------------------------------------------------------------------
_M             = 268.0     # kg  total mass (vehicle + driver)
_G             = 9.81      # m/s²
_RHO           = 1.225     # kg/m³ air density
_CD            = 1.77      # drag coefficient
_CL            = 4.26      # downforce coefficient
_AREF          = 1.106     # m² frontal area
_MU            = 1.6       # tyre friction coefficient (slicks, dry)
_CRR           = 0.012     # rolling resistance coefficient
_P_MAX         = 66700.0   # W  max pack power
_F_ENGINE_MAX  = 7676.0    # N  peak tractive force


# ---------------------------------------------------------------------------
# Physics helpers for PSO lap-time fitness
# ---------------------------------------------------------------------------

def _menger_kappa(path):
    """Menger curvature at each waypoint of path (N, 2)."""
    N = len(path)
    kappa = np.zeros(N)
    for i in range(1, N - 1):
        a, b, c = path[i - 1], path[i], path[i + 1]
        ab = np.linalg.norm(b - a)
        bc = np.linalg.norm(c - b)
        ca = np.linalg.norm(a - c)
        cross = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
        denom = ab * bc * ca
        kappa[i] = (2 * cross / denom) if denom > 1e-9 else 0.0
    kappa[0] = kappa[1]
    kappa[-1] = kappa[-2]
    return kappa


def _grip_speed(kappa):
    """Lateral-grip speed limit (m/s) at each point, aero-corrected."""
    r = np.where(kappa > 1e-4, 1.0 / kappa, 1e6)
    denom = _M - _MU * 0.5 * _RHO * _CL * _AREF * r
    denom = np.maximum(denom, 1.0)
    v2 = _MU * _M * _G * r / denom
    return np.sqrt(np.clip(v2, 0, 900))   # cap at 30 m/s


def _backward_pass(v_lat, ds):
    """Backward deceleration pass: ensure car can brake to each corner speed."""
    v = v_lat.copy()
    for i in range(len(v) - 2, -1, -1):
        f_drag = 0.5 * _RHO * _CD * _AREF * v[i + 1] ** 2
        f_net_brake = (_MU * (_M * _G + 0.5 * _RHO * _CL * _AREF * v[i + 1] ** 2)
                       - f_drag - _CRR * _M * _G)
        a_max_brake = f_net_brake / _M
        v_reachable = math.sqrt(v[i + 1] ** 2 + 2 * a_max_brake * ds[i])
        v[i] = min(v[i], v_reachable)
    return v


def _forward_pass(v_after_backward, ds):
    """Forward acceleration pass with pack-power and force limits."""
    v = v_after_backward.copy()
    for i in range(len(v) - 1):
        f_drag = 0.5 * _RHO * _CD * _AREF * v[i] ** 2
        f_engine = min(_F_ENGINE_MAX, _P_MAX / max(v[i], 0.1))
        f_net = f_engine - f_drag - _CRR * _M * _G
        a = max(f_net / _M, 0.0)
        v_next = math.sqrt(v[i] ** 2 + 2 * a * ds[i])
        v[i + 1] = min(v[i + 1], v_next)
    return v


def _lap_time(v, ds):
    """Estimated lap time (s) from velocity profile and arc-lengths."""
    v_avg = 0.5 * (v[:-1] + v[1:])
    v_avg = np.maximum(v_avg, 0.1)
    return float(np.sum(ds / v_avg))


# ---------------------------------------------------------------------------
# PSO race-line optimizer
# ---------------------------------------------------------------------------

def _pso_raceline(centerline_xy, left_xy, right_xy, safety=0.8,
                  n_particles=30, n_iter=200, penalty_weight=5.0):
    """
    PSO race line optimizer. Finds lateral offsets alpha_i from centerline
    that minimise physics-simulated lap time.

    Returns optimised path (N, 2) array. Falls back to centerline on failure.
    """
    import random  # noqa: F401 — kept for spec compliance; np.random used internally
    N = len(centerline_xy)
    if N < 4 or len(left_xy) == 0 or len(right_xy) == 0:
        return centerline_xy

    # Unit tangents and left-pointing normals (same as _min_curvature_path)
    tg = np.zeros_like(centerline_xy)
    tg[1:-1] = centerline_xy[2:] - centerline_xy[:-2]
    tg[0] = centerline_xy[1] - centerline_xy[0]
    tg[-1] = centerline_xy[-1] - centerline_xy[-2]
    norm_mag = np.linalg.norm(tg, axis=1, keepdims=True).clip(1e-9)
    tg /= norm_mag
    nrm = np.column_stack([-tg[:, 1], tg[:, 0]])

    def _hw(boundary):
        w = np.empty(N)
        for i in range(N):
            w[i] = np.linalg.norm(boundary - centerline_xy[i], axis=1).min()
        return w * safety

    w_l = _hw(left_xy)
    w_r = _hw(right_xy)

    def fitness(alpha):
        path = centerline_xy + alpha[:, None] * nrm
        # Boundary penalty
        penalty = float(np.sum(
            np.maximum(0, alpha - w_l) ** 2 + np.maximum(0, -alpha - w_r) ** 2
        ))
        # Arc lengths
        diff = np.diff(path, axis=0)
        ds = np.linalg.norm(diff, axis=1).clip(1e-6)
        # Physics
        kappa = _menger_kappa(path)
        v_lat = _grip_speed(kappa)
        v = _backward_pass(v_lat, ds)
        v = _forward_pass(v, ds)
        t = _lap_time(v, ds)
        return t + penalty_weight * penalty

    # Initialise particles; first particle starts at centerline (alpha=0)
    pos = np.zeros((n_particles, N))
    pos[1:] = (np.random.uniform(-1, 1, (n_particles - 1, N))
               * np.minimum(w_l, w_r) * 0.3)
    vel = np.zeros_like(pos)
    pbest = pos.copy()
    pbest_fit = np.array([fitness(p) for p in pos])
    gbest_idx = int(np.argmin(pbest_fit))
    gbest = pbest[gbest_idx].copy()
    gbest_fit = pbest_fit[gbest_idx]

    stagnant = 0
    for it in range(n_iter):
        # Dynamic PSO coefficients (Negura et al. 2025)
        frac = it / max(n_iter - 1, 1)
        w_inertia = 0.8 - 0.7 * frac   # 0.8 → 0.1
        c1 = 1.5 - 1.0 * frac          # 1.5 → 0.5
        c2 = 1.5 + 1.0 * frac          # 1.5 → 2.5

        r1 = np.random.rand(n_particles, N)
        r2 = np.random.rand(n_particles, N)
        vel = (w_inertia * vel
               + c1 * r1 * (pbest - pos)
               + c2 * r2 * (gbest - pos))
        pos = pos + vel
        # Clamp to track bounds
        pos = np.clip(pos, -w_r, w_l)

        improved = False
        for i, p in enumerate(pos):
            f = fitness(p)
            if f < pbest_fit[i]:
                pbest[i] = p.copy()
                pbest_fit[i] = f
                if f < gbest_fit:
                    gbest = p.copy()
                    gbest_fit = f
                    improved = True

        if not improved:
            stagnant += 1
        else:
            stagnant = 0

        # Escape local optima: perturb worst 20 % of swarm near gbest
        if stagnant >= 10:
            n_perturb = max(1, n_particles // 5)
            worst_idx = np.argsort(pbest_fit)[-n_perturb:]
            for idx in worst_idx:
                noise = np.random.randn(N) * np.minimum(w_l, w_r) * 0.1
                pos[idx] = np.clip(gbest + noise, -w_r, w_l)
                vel[idx] = 0.0
            stagnant = 0

    return centerline_xy + gbest[:, None] * nrm


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
    # Capped at 2.0 m: where a boundary wall has a gap (e.g. a figure-8 crossing,
    # missed detections) the nearest-cone distance blows up and the optimizer would
    # swing the line through the gap into off-corridor cones. FSAE track half-width
    # is ~1.5-2.5 m, so 2.0 m keeps the line inside a plausible corridor everywhere.
    _W_MAX = 2.0

    def _half_widths(boundary_xy: np.ndarray) -> np.ndarray:
        w = np.empty(N)
        for i in range(N):
            w[i] = np.linalg.norm(boundary_xy - centerline_xy[i], axis=1).min()
        return np.minimum(w, _W_MAX) * safety

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
        self.declare_parameter('mission', 'autocross')  # autocross | trackdrive | acceleration | skidpad | peanut
        self.declare_parameter('skidpad_mode', 'cone_based')  # cone_based | hardcoded
        self.declare_parameter('use_pso_optimizer', True)
        # Local (non-global) planning backend: fasttube = fsd_path_planning library,
        # amz = Delaunay + beam search after the AMZ Driverless paper (amz_planner.py).
        self.declare_parameter('planner_backend', 'fasttube')  # fasttube | amz

        self._map_frame = self.get_parameter('map_frame').value
        mission_str = self.get_parameter('mission').value
        self._use_pso_optimizer = self.get_parameter('use_pso_optimizer').value
        self._mission = mission_str
        self._skidpad_mode = self.get_parameter('skidpad_mode').value
        self._backend = self.get_parameter('planner_backend').value
        self._amz = AmzPlanner() if self._backend == 'amz' else None

        # skidpad can use either hardcoded path or cone-based planner.
        # trackdrive-type missions (known closed track) plan GLOBALLY over the full cone
        # map — required on tracks whose boundary walls pass near each other (e.g. a
        # figure-8 crossing): a local radius filter makes the boundary trace jump walls,
        # while the global trace follows each wall around its own loop by ring continuity.
        # Local-type missions (autocross, acceleration) keep the visibility filter.
        _HARDCODED_MISSIONS = {'skidpad'} if self._skidpad_mode == 'hardcoded' else set()
        self._global_planning = False
        if _LIB_AVAILABLE and mission_str not in _HARDCODED_MISSIONS:
            mission_map = {
                'autocross':    MissionTypes.autocross,
                'trackdrive':   MissionTypes.trackdrive,
                'acceleration': MissionTypes.acceleration,
                'peanut':       MissionTypes.trackdrive,   # closed figure-8 → global planning
                'skidpad':      MissionTypes.autocross,    # cone-based planning for skidpad (local)
            }
            mission = mission_map.get(mission_str, MissionTypes.autocross)
            self._planner = PathPlanner(mission)
            self._global_planning = (mission == MissionTypes.trackdrive)
        else:
            self._planner = None

        # Pre-compute hardcoded paths for missions ft-fsd can't handle
        self._skidpad_path_published = False
        if mission_str == 'skidpad':
            if self._skidpad_mode == 'hardcoded':
                self._skidpad_path = _make_skidpad_path()
                self.get_logger().info(
                    f'PathPlannerNode started. Mission: skidpad (hardcoded mode) | '
                    f'path: {len(self._skidpad_path)} waypoints, '
                    f'sequence: 2 laps LEFT + 2 laps RIGHT + exit')
            else:
                self._skidpad_path = None
                self._init_skidpad_state_machine()
                self.get_logger().info(
                    f'PathPlannerNode started. Mission: skidpad (cone_based mode) | '
                    f'state machine initialized')
        else:
            self._skidpad_path = None
            self.get_logger().info(
                f'PathPlannerNode started. Mission: {mission_str} | local planner: {self._backend}')

        # ---------- State ----------
        self._car_pos = np.array([0.0, 0.0])
        self._car_dir = np.array([1.0, 0.0])   # unit vector, default facing +X
        self._car_yaw = 0.0                    # yaw angle in radians
        self._car_yaw_rate = 0.0               # angular velocity (z-component of twist)
        self._odom_received = False

        # Cone map: accumulated global map of all seen cones across frames.
        # Key: (rx, ry) rounded to nearest 0.5 m grid. Value: cone color (ConeTypes).
        self._cone_map: dict = {}

        # Lap-1 lock-in (global/trackdrive missions): while exploring, record the car's
        # driven trail. When the car closes the loop, build the full-lap centerline from
        # the accumulated cone map ordered by the trail, optimize it once, and lock it —
        # no more per-cycle re-planning, and the trail ordering resolves self-crossing
        # tracks (figure-8) because it follows the topology the car actually drove.
        self._trail: list = []            # breadcrumb positions, ≥0.5 m apart
        self._trail_dist = 0.0            # cumulative distance driven
        self._trail_max_r = 0.0           # max distance from start (excursion guard)
        self._locked_path = None          # Nx2 closed-loop path once lap 1 completes

        # Skidpad state machine (cone-based mode): tracks which segment and lap count
        if mission_str == 'skidpad' and self._skidpad_mode == 'cone_based':
            self._skidpad_state = 'APPROACH'      # APPROACH → LEFT_CIRCLE → RIGHT_CIRCLE → EXIT
            self._skidpad_circle_lap_count = 0    # laps completed in current circle (0, 1, or 2)
            self._skidpad_left_locked = None      # locked left circle path (Nx2)
            self._skidpad_right_locked = None     # locked right circle path (Nx2)
            self._skidpad_left_trail = []         # trail for left circle
            self._skidpad_left_start = None       # position where left circle started
            self._skidpad_left_trail_dist = 0.0
            self._skidpad_left_max_r = 0.0
            self._skidpad_right_trail = []
            self._skidpad_right_start = None
            self._skidpad_right_trail_dist = 0.0
            self._skidpad_right_max_r = 0.0
            self._skidpad_sustained_yaw_rate_sign = None  # +1 (left/CCW), -1 (right/CW), or None
            self._skidpad_yaw_rate_history = []   # recent yaw rates for filtering

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
        self._car_yaw = yaw
        self._car_dir = np.array([math.cos(yaw), math.sin(yaw)])
        # Extract yaw rate (angular velocity around z-axis)
        self._car_yaw_rate = float(msg.twist.twist.angular.z)
        self._odom_received = True

        # Breadcrumb trail during lap 1 (global missions, before lock-in)
        if self._global_planning and self._locked_path is None:
            if not self._trail:
                self._trail.append(self._car_pos.copy())
            else:
                step = float(np.linalg.norm(self._car_pos - self._trail[-1]))
                if step >= 0.5:
                    self._trail.append(self._car_pos.copy())
                    self._trail_dist += step
                    self._trail_max_r = max(
                        self._trail_max_r,
                        float(np.linalg.norm(self._car_pos - self._trail[0])))

    def _init_skidpad_state_machine(self) -> None:
        """Initialize skidpad cone-based state machine."""
        self._skidpad_state = 'APPROACH'
        self._skidpad_circle_lap_count = 0
        self._skidpad_left_locked = None
        self._skidpad_right_locked = None
        self._skidpad_left_trail = []
        self._skidpad_left_start = None
        self._skidpad_left_trail_dist = 0.0
        self._skidpad_left_max_r = 0.0
        self._skidpad_right_trail = []
        self._skidpad_right_start = None
        self._skidpad_right_trail_dist = 0.0
        self._skidpad_right_max_r = 0.0
        self._skidpad_sustained_yaw_rate_sign = None
        self._skidpad_yaw_rate_history = []

    def _update_skidpad_yaw_rate_sign(self) -> None:
        """Update sustained yaw-rate sign (moving average for filtering noise)."""
        if self._mission != 'skidpad' or self._skidpad_mode != 'cone_based':
            return
        # Keep history of last 5 frames (~0.5 s at 10 Hz)
        self._skidpad_yaw_rate_history.append(self._car_yaw_rate)
        if len(self._skidpad_yaw_rate_history) > 5:
            self._skidpad_yaw_rate_history.pop(0)
        # Average yaw rate; if consistently positive (left/CCW) or negative (right/CW)
        avg_yaw_rate = float(np.mean(self._skidpad_yaw_rate_history))
        threshold = 0.1  # rad/s
        if abs(avg_yaw_rate) > threshold:
            new_sign = 1 if avg_yaw_rate > 0 else -1
            if self._skidpad_sustained_yaw_rate_sign != new_sign:
                self._skidpad_sustained_yaw_rate_sign = new_sign
                circle_dir = "LEFT (CCW)" if new_sign > 0 else "RIGHT (CW)"
                self.get_logger().debug(f'Skidpad: detected sustained turn {circle_dir}')

    def _build_skidpad_circle(self, trail: list, trail_dist: float, trail_max_r: float,
                              trail_start: np.ndarray) -> np.ndarray:
        """
        Build a locked circle path for skidpad, similar to _build_locked_loop
        but working with a specific subset of the cone map for one circle.

        Returns Nx2 closed path (first ≈ last) or None if the map is too thin.
        """
        if trail_start is None:
            return None
        blue = np.array([k for k, v in self._cone_map.items() if v == ConeTypes.LEFT])
        yellow = np.array([k for k, v in self._cone_map.items() if v == ConeTypes.RIGHT])
        if len(blue) < 4 or len(yellow) < 4 or len(trail) < 15:
            return None
        trail_arr = np.array(trail)
        # Walk the driven trail; at each step take the midpoint of nearest left/right cones
        centerline = []
        for tx, ty in trail_arr[::2]:  # every ~1 m of trail
            db = np.hypot(blue[:, 0] - tx, blue[:, 1] - ty)
            dy = np.hypot(yellow[:, 0] - tx, yellow[:, 1] - ty)
            ib, iy = int(np.argmin(db)), int(np.argmin(dy))
            if db[ib] <= 5.0 and dy[iy] <= 5.0:
                p = ((blue[ib, 0] + yellow[iy, 0]) / 2.0,
                     (blue[ib, 1] + yellow[iy, 1]) / 2.0)
            else:
                p = (float(tx), float(ty))
            if not centerline or (p[0]-centerline[-1][0])**2 + (p[1]-centerline[-1][1])**2 >= 0.75**2:
                centerline.append(p)
        centerline = np.array(centerline)
        if len(centerline) < 8:
            return None
        # Light smoothing (3-point moving average, cyclic)
        prev_c = np.roll(centerline, 1, axis=0)
        next_c = np.roll(centerline, -1, axis=0)
        centerline = (prev_c + centerline + next_c) / 3.0
        raw = centerline.copy()
        # One-time racing line optimization (gentle)
        try:
            centerline = _min_curvature_path(centerline, blue, yellow, safety=0.5)
        except Exception as e:
            self.get_logger().debug(f'Skidpad circle optimizer failed: {e}')
        # Clearance guard
        all_cones = np.vstack([blue, yellow])
        reverted = 0
        for i in range(len(centerline)):
            d = np.hypot(all_cones[:, 0] - centerline[i, 0],
                        all_cones[:, 1] - centerline[i, 1]).min()
            if d < 1.0:
                centerline[i] = raw[i]
                reverted += 1
        # Resample to uniform ~1.0 m spacing
        closed = np.vstack([centerline, centerline[:1]])
        seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(seg)])
        total = float(s[-1])
        n = max(int(round(total)), 15)
        return np.column_stack([
            np.interp(np.linspace(0.0, total, n + 1), s, closed[:, 0]),
            np.interp(np.linspace(0.0, total, n + 1), s, closed[:, 1]),
        ])

    def _build_locked_loop(self):
        """
        Build the full-lap closed centerline from the live-accumulated cone map,
        ordered by the car's lap-1 breadcrumb trail.

        The trail ordering is what makes this robust on self-crossing tracks
        (figure-8): midpoints are sequenced by where the car actually drove, so
        the crossing is traversed in the correct order. Returns Nx2 closed path
        (first point == last point) or None if the map is too thin.
        """
        blue = np.array([k for k, v in self._cone_map.items() if v == ConeTypes.LEFT])
        yellow = np.array([k for k, v in self._cone_map.items() if v == ConeTypes.RIGHT])
        if len(blue) < 4 or len(yellow) < 4 or len(self._trail) < 20:
            self.get_logger().warn(
                f'Loop closed but map too thin to lock '
                f'(blue={len(blue)} yellow={len(yellow)} trail={len(self._trail)})')
            return None

        trail = np.array(self._trail)

        # Walk the driven trail; at each step take the midpoint of the nearest
        # left (blue) and right (yellow) wall cone. Anchoring on the trail keeps
        # the sequence continuous (no gaps/ordering problems, correct through a
        # figure-8 crossing) and the local wall midpoint recenters the line even
        # where the car drove off-center during exploration.
        centerline = []
        for tx, ty in trail[::2]:                      # every ~1 m of trail
            db = np.hypot(blue[:, 0] - tx, blue[:, 1] - ty)
            dy = np.hypot(yellow[:, 0] - tx, yellow[:, 1] - ty)
            ib, iy = int(np.argmin(db)), int(np.argmin(dy))
            if db[ib] <= 5.0 and dy[iy] <= 5.0:
                p = ((blue[ib, 0] + yellow[iy, 0]) / 2.0,
                     (blue[ib, 1] + yellow[iy, 1]) / 2.0)
            else:
                p = (float(tx), float(ty))             # walls not visible: keep drive line
            if not centerline or (p[0]-centerline[-1][0])**2 + (p[1]-centerline[-1][1])**2 >= 0.75**2:
                centerline.append(p)
        centerline = np.array(centerline)
        if len(centerline) < 10:
            return None

        # Light smoothing (3-point moving average, cyclic) to kill midpoint jitter
        prev_c = np.roll(centerline, 1, axis=0)
        next_c = np.roll(centerline, -1, axis=0)
        centerline = (prev_c + centerline + next_c) / 3.0
        raw = centerline.copy()

        # One-time racing line optimization within the mapped boundaries (gentle)
        try:
            centerline = _min_curvature_path(centerline, blue, yellow, safety=0.5)
        except Exception as e:
            self.get_logger().warn(
                f'Lock-in optimizer failed ({type(e).__name__}: {e}) — using raw centerline.')

        # Clearance guard: any waypoint the optimizer pushed within 1.0 m of a cone
        # reverts to its raw (centered) position — a clipped cone costs more than a
        # slightly slower line.
        all_cones = np.vstack([blue, yellow])
        reverted = 0
        for i in range(len(centerline)):
            d = np.hypot(all_cones[:, 0] - centerline[i, 0],
                         all_cones[:, 1] - centerline[i, 1]).min()
            if d < 1.0:
                centerline[i] = raw[i]
                reverted += 1
        if reverted:
            self.get_logger().info(f'Lock-in clearance guard reverted {reverted} waypoints.')

        # Resample to uniform ~1.0 m spacing. The midpoint walk collapses
        # consecutive trail steps onto the same wall-cone midpoint, leaving the
        # line as coarse as the wall-cone spacing (measured 1.75 m avg, 56 wpts
        # for a 97.7 m peanut lap). That is too coarse for pure pursuit at the
        # hairpins (measured 2.7 m cross-track at the SE corner) and halves the
        # index separation between figure-8 crossing passes toward the 30-wpt
        # closest-search window (the skidpad lap-skip failure mode).
        # Closing point included, so pure pursuit wrap-around engages
        # (path[0] == path[-1]).
        closed = np.vstack([centerline, centerline[:1]])
        seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(seg)])
        total = float(s[-1])
        n = max(int(round(total)), 20)
        si = np.linspace(0.0, total, n + 1)
        return np.column_stack([
            np.interp(si, s, closed[:, 0]),
            np.interp(si, s, closed[:, 1]),
        ])

    def _handle_skidpad_cone_based(self, msg: Track) -> None:
        """
        Skidpad state machine for cone-based planning.

        Sequence: APPROACH (local planning) → LEFT_CIRCLE (detect & lock, 2 laps) →
        RIGHT_CIRCLE (detect & lock, 2 laps) → EXIT (local planning).

        Uses yaw-rate sign to disambiguate which circle, loop closure for lap counting,
        cone color as corroborating signal, and separate start references.
        """
        if not self._odom_received:
            return

        # Update yaw-rate sign tracking
        self._update_skidpad_yaw_rate_sign()

        # --- Cone map accumulation (same as other missions) ---
        _GRID = 0.5
        _CONE_MAP_MAX = 500
        for cone in msg.track:
            if cone.color in (Cone.ORANGE_BIG, Cone.ORANGE_SMALL):
                continue
            if not (math.isfinite(cone.location.x) and math.isfinite(cone.location.y)):
                continue
            rx = round(cone.location.x / _GRID) * _GRID
            ry = round(cone.location.y / _GRID) * _GRID
            key = (rx, ry)
            if key not in self._cone_map:
                if len(self._cone_map) >= _CONE_MAP_MAX:
                    self._cone_map.pop(next(iter(self._cone_map)))
                self._cone_map[key] = _MFE_TO_FSD.get(cone.color, ConeTypes.UNKNOWN)
            else:
                new_type = _MFE_TO_FSD.get(cone.color, ConeTypes.UNKNOWN)
                if self._cone_map[key] == ConeTypes.UNKNOWN and new_type != ConeTypes.UNKNOWN:
                    self._cone_map[key] = new_type

        # Track trails for each circle (analogous to global mission trail tracking)
        if self._skidpad_state == 'LEFT_CIRCLE':
            if not self._skidpad_left_trail:
                self._skidpad_left_trail.append(self._car_pos.copy())
            else:
                step = float(np.linalg.norm(self._car_pos - self._skidpad_left_trail[-1]))
                if step >= 0.5:
                    self._skidpad_left_trail.append(self._car_pos.copy())
                    self._skidpad_left_trail_dist += step
                    if self._skidpad_left_start is not None:
                        self._skidpad_left_max_r = max(
                            self._skidpad_left_max_r,
                            float(np.linalg.norm(self._car_pos - self._skidpad_left_start)))
        elif self._skidpad_state == 'RIGHT_CIRCLE':
            if not self._skidpad_right_trail:
                self._skidpad_right_trail.append(self._car_pos.copy())
            else:
                step = float(np.linalg.norm(self._car_pos - self._skidpad_right_trail[-1]))
                if step >= 0.5:
                    self._skidpad_right_trail.append(self._car_pos.copy())
                    self._skidpad_right_trail_dist += step
                    if self._skidpad_right_start is not None:
                        self._skidpad_right_max_r = max(
                            self._skidpad_right_max_r,
                            float(np.linalg.norm(self._car_pos - self._skidpad_right_start)))

        # --- State machine ---
        if self._skidpad_state == 'APPROACH':
            # Use local cone-based planning until we detect a sustained turn into a circle
            if self._skidpad_sustained_yaw_rate_sign is not None:
                if self._skidpad_sustained_yaw_rate_sign > 0:
                    self.get_logger().info('Skidpad: entering LEFT_CIRCLE')
                    self._skidpad_state = 'LEFT_CIRCLE'
                    self._skidpad_circle_lap_count = 0
                    self._skidpad_left_start = self._car_pos.copy()
                    self._skidpad_left_trail = [self._car_pos.copy()]
                    self._skidpad_left_trail_dist = 0.0
                    self._skidpad_left_max_r = 0.0
                else:
                    self.get_logger().info('Skidpad: entering RIGHT_CIRCLE')
                    self._skidpad_state = 'RIGHT_CIRCLE'
                    self._skidpad_circle_lap_count = 0
                    self._skidpad_right_start = self._car_pos.copy()
                    self._skidpad_right_trail = [self._car_pos.copy()]
                    self._skidpad_right_trail_dist = 0.0
                    self._skidpad_right_max_r = 0.0

        elif self._skidpad_state == 'LEFT_CIRCLE':
            # Check for lap closure: trail dist > 40 m, max excursion > 10 m, back within 4 m of start
            if (self._skidpad_left_trail_dist > 40.0
                    and self._skidpad_left_max_r > 10.0
                    and float(np.linalg.norm(self._car_pos - self._skidpad_left_start)) < 4.0):
                if self._skidpad_left_locked is None:
                    # First lap closure: build and lock the left circle
                    locked = self._build_skidpad_circle(
                        self._skidpad_left_trail, self._skidpad_left_trail_dist,
                        self._skidpad_left_max_r, self._skidpad_left_start)
                    if locked is not None:
                        self._skidpad_left_locked = locked
                        self._skidpad_circle_lap_count = 1
                        arc = float(np.sum(np.hypot(np.diff(locked[:, 0]), np.diff(locked[:, 1]))))
                        self.get_logger().info(
                            f'Skidpad LEFT lap 1 complete — locked circle: {len(locked)} waypoints, '
                            f'{arc:.1f} m')
                        # Reset trail for lap 2 counting
                        self._skidpad_left_trail = [self._car_pos.copy()]
                        self._skidpad_left_trail_dist = 0.0
                        self._skidpad_left_max_r = 0.0
                elif self._skidpad_circle_lap_count == 1:
                    # Second lap closure: transition to RIGHT_CIRCLE
                    self._skidpad_circle_lap_count = 2
                    self.get_logger().info('Skidpad LEFT lap 2 complete — transitioning to RIGHT_CIRCLE')
                    # Check sustained turn direction before transitioning
                    if self._skidpad_sustained_yaw_rate_sign is not None and self._skidpad_sustained_yaw_rate_sign < 0:
                        self._skidpad_state = 'RIGHT_CIRCLE'
                        self._skidpad_circle_lap_count = 0
                        self._skidpad_right_start = self._car_pos.copy()
                        self._skidpad_right_trail = [self._car_pos.copy()]
                        self._skidpad_right_trail_dist = 0.0
                        self._skidpad_right_max_r = 0.0
                        self._skidpad_yaw_rate_history.clear()  # reset filter
                    else:
                        self.get_logger().warn('Skidpad: expected right turn to enter RIGHT_CIRCLE, still turning left')

        elif self._skidpad_state == 'RIGHT_CIRCLE':
            # Check for lap closure
            if (self._skidpad_right_trail_dist > 40.0
                    and self._skidpad_right_max_r > 10.0
                    and float(np.linalg.norm(self._car_pos - self._skidpad_right_start)) < 4.0):
                if self._skidpad_right_locked is None:
                    # First lap closure: build and lock the right circle
                    locked = self._build_skidpad_circle(
                        self._skidpad_right_trail, self._skidpad_right_trail_dist,
                        self._skidpad_right_max_r, self._skidpad_right_start)
                    if locked is not None:
                        self._skidpad_right_locked = locked
                        self._skidpad_circle_lap_count = 1
                        arc = float(np.sum(np.hypot(np.diff(locked[:, 0]), np.diff(locked[:, 1]))))
                        self.get_logger().info(
                            f'Skidpad RIGHT lap 1 complete — locked circle: {len(locked)} waypoints, '
                            f'{arc:.1f} m')
                        # Reset trail for lap 2 counting
                        self._skidpad_right_trail = [self._car_pos.copy()]
                        self._skidpad_right_trail_dist = 0.0
                        self._skidpad_right_max_r = 0.0
                elif self._skidpad_circle_lap_count == 1:
                    # Second lap closure: transition to EXIT
                    self._skidpad_circle_lap_count = 2
                    self.get_logger().info('Skidpad RIGHT lap 2 complete — transitioning to EXIT')
                    self._skidpad_state = 'EXIT'
                    self._skidpad_yaw_rate_history.clear()

        # Use locked paths when available, else fall through to local cone-based planning
        if self._skidpad_state == 'LEFT_CIRCLE' and self._skidpad_left_locked is not None:
            stamp = self.get_clock().now().to_msg()
            self._pub_centerline.publish(
                _path_from_xy(self._skidpad_left_locked, self._map_frame, stamp))
            return
        elif self._skidpad_state == 'RIGHT_CIRCLE' and self._skidpad_right_locked is not None:
            stamp = self.get_clock().now().to_msg()
            self._pub_centerline.publish(
                _path_from_xy(self._skidpad_right_locked, self._map_frame, stamp))
            return
        elif self._skidpad_state == 'EXIT':
            # Fall through to local cone-based planning for exit
            pass

        # If we haven't locked a path yet, use local cone-based planning (from ft-fsd)
        # This handles APPROACH, incomplete circles, and EXIT states
        self._handle_local_cone_planning(msg)

    def _handle_local_cone_planning(self, msg: Track) -> None:
        """
        Handle local cone-based planning using ft-fsd for states that don't have locked paths.
        Reused for APPROACH, EXIT, and incomplete circle states.
        """
        if not _LIB_AVAILABLE or self._planner is None:
            return

        if not msg.track:
            return

        # Build cones_by_type array
        CONE_VIS_R    = 50.0
        CONE_BEHIND_M = 3.0
        n_types = len(ConeTypes)
        buckets: list[list] = [[] for _ in range(n_types)]

        for (rx, ry), cone_type in self._cone_map.items():
            dx = rx - self._car_pos[0]
            dy = ry - self._car_pos[1]
            if dx * dx + dy * dy > CONE_VIS_R * CONE_VIS_R:
                continue
            if dx * self._car_dir[0] + dy * self._car_dir[1] < -CONE_BEHIND_M:
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

        if n_left + n_right == 0 and n_unknown > 0:
            # Geometric assignment for LiDAR-only mode
            for (rx, ry), cone_type in self._cone_map.items():
                if cone_type != ConeTypes.UNKNOWN:
                    continue
                dx = rx - self._car_pos[0]
                dy = ry - self._car_pos[1]
                fwd = dx * self._car_dir[0] + dy * self._car_dir[1]
                if fwd < -CONE_BEHIND_M:
                    continue
                cross_z = self._car_dir[0] * dy - self._car_dir[1] * dx
                if cross_z > 0:
                    buckets[ConeTypes.LEFT].append([rx, ry])
                else:
                    buckets[ConeTypes.RIGHT].append([rx, ry])
            buckets[ConeTypes.UNKNOWN] = []
            cones_by_type = [
                np.array(b, dtype=np.float64).reshape(-1, 2) if b
                else np.zeros((0, 2), dtype=np.float64)
                for b in buckets
            ]
            n_left  = len(buckets[ConeTypes.LEFT])
            n_right = len(buckets[ConeTypes.RIGHT])

        if n_left + n_right == 0:
            self.get_logger().warn(
                'No colored or assignable cones — skipping planner.',
                throttle_duration_sec=5.0)
            return

        all_xy = np.array(list(self._cone_map.keys()), dtype=np.float64)
        d_nearest = float(np.min(np.hypot(all_xy[:, 0] - self._car_pos[0],
                                          all_xy[:, 1] - self._car_pos[1])))
        if d_nearest > 20.0:
            self.get_logger().warn(
                f'Car is {d_nearest:.0f} m from nearest cone — off track?',
                throttle_duration_sec=2.0)
            return

        try:
            result = self._planner.calculate_path_in_global_frame(
                cones_by_type, self._car_pos, self._car_dir,
                return_intermediate_results=True,
            )
            path_xy, left_xy, right_xy, *_ = result
        except Exception as e:
            self.get_logger().error(
                f'Path calculation failed: {type(e).__name__}: {e}',
                throttle_duration_sec=2.0)
            return

        stamp = self.get_clock().now().to_msg()
        frame = self._map_frame

        if path_xy is not None and len(path_xy) > 0:
            centerline = np.array(path_xy)[:, 1:3]
            # Run racing line optimizer for local missions
            if (len(path_xy) > 3
                    and left_xy is not None and len(left_xy) > 3
                    and right_xy is not None and len(right_xy) > 3):
                left   = np.array(left_xy)[:, :2]
                right  = np.array(right_xy)[:, :2]
                try:
                    optimized = _min_curvature_path(centerline, left, right, safety=0.7)
                except Exception as opt_e:
                    self.get_logger().debug(f'Optimizer failed: {opt_e}')
                    optimized = centerline
            else:
                optimized = centerline

            self._pub_centerline.publish(_path_from_xy(optimized, frame, stamp))

        if left_xy is not None and len(left_xy) > 0:
            self._pub_left.publish(_path_from_xy(np.array(left_xy)[:, :2], frame, stamp))
        if right_xy is not None and len(right_xy) > 0:
            self._pub_right.publish(_path_from_xy(np.array(right_xy)[:, :2], frame, stamp))

    def _cones_cb(self, msg: Track) -> None:
        # Locked loop (global missions, lap 2+): the full-lap path was built from the
        # live-accumulated cone map when the car closed lap 1. Publish it unchanged —
        # no re-planning. Pure pursuit wraps around closed paths for endless laps.
        if self._locked_path is not None:
            stamp = self.get_clock().now().to_msg()
            self._pub_centerline.publish(
                _path_from_xy(self._locked_path, self._map_frame, stamp))
            return

        # Skidpad: use either cone-based state machine or hardcoded path
        if self._mission == 'skidpad':
            if self._skidpad_mode == 'hardcoded':
                if self._odom_received:
                    stamp = self.get_clock().now().to_msg()
                    self._pub_centerline.publish(
                        _path_from_xy(self._skidpad_path, self._map_frame, stamp))
                    if not self._skidpad_path_published:
                        self._skidpad_path_published = True
                        self.get_logger().info(
                            f'skidpad hardcoded path first publish: {len(self._skidpad_path)} waypoints')
                return
            else:  # cone_based
                self._handle_skidpad_cone_based(msg)
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
            # Orange pylons mark the start/finish gate, not track boundaries.
            # They are on /planning/cones for the finish detector's gate-pass
            # detection; as UNKNOWN cones in the map they distort the ft-fsd
            # boundary trace, so drop them here entirely.
            if cone.color in (Cone.ORANGE_BIG, Cone.ORANGE_SMALL):
                continue
            # A single non-finite cone (bad depth pixel upstream) must not kill
            # the planner: round(NaN) raises ValueError.
            if not (math.isfinite(cone.location.x) and math.isfinite(cone.location.y)):
                continue
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

        # --- Lap-1 completion → lock the loop (global missions) ---
        # Same spirit as the finish detector: the car must have driven far enough AND
        # ventured away from the start before a return to the start counts as a lap.
        if (self._global_planning
                and self._trail
                and self._trail_dist > 60.0
                and self._trail_max_r > 15.0
                and float(np.linalg.norm(self._car_pos - self._trail[0])) < 4.0):
            locked = self._build_locked_loop()
            if locked is not None:
                self._locked_path = locked
                arc = float(np.sum(np.hypot(np.diff(locked[:, 0]), np.diff(locked[:, 1]))))
                self.get_logger().info(
                    f'Lap 1 complete — locked full-loop path: {len(locked)} waypoints, '
                    f'{arc:.1f} m, built from {len(self._cone_map)} live-mapped cones. '
                    f'Re-planning stopped.')
                stamp = self.get_clock().now().to_msg()
                self._pub_centerline.publish(
                    _path_from_xy(locked, self._map_frame, stamp))
                return

        # Build cones_by_type: list of Nx2 arrays indexed by ConeTypes enum value
        # ft-fsd-path-planning expects: [unknown[], right[], left[], orange_big[], orange_small[]]
        #
        # Visibility filter — local planning only. Global (trackdrive) planning needs the
        # entire cone map: the global boundary trace follows each wall around the whole
        # track, which is what disambiguates near-passing walls (figure-8 crossings).
        CONE_VIS_R    = 50.0   # local: large radius for sparse tracks
        CONE_BEHIND_M = 3.0    # small tolerance so the cone pair straddling the car survives

        n_types = len(ConeTypes)
        buckets: list[list] = [[] for _ in range(n_types)]

        for (rx, ry), cone_type in self._cone_map.items():
            if not self._global_planning:
                dx = rx - self._car_pos[0]
                dy = ry - self._car_pos[1]
                if dx * dx + dy * dy > CONE_VIS_R * CONE_VIS_R:
                    continue
                # forward projection onto heading: negative = behind the car
                if dx * self._car_dir[0] + dy * self._car_dir[1] < -CONE_BEHIND_M:
                    continue
            buckets[cone_type].append([rx, ry])

        cones_by_type = [
            np.array(b, dtype=np.float64).reshape(-1, 2) if b
            else np.zeros((0, 2), dtype=np.float64)
            for b in buckets
        ]

        if self._amz is not None and not self._global_planning:
            self._plan_amz()
            return

        n_left    = len(buckets[ConeTypes.LEFT])
        n_right   = len(buckets[ConeTypes.RIGHT])
        n_unknown = len(buckets[ConeTypes.UNKNOWN])
        self.get_logger().info(
            f'planner input: left={n_left} right={n_right} unknown={n_unknown} '
            f'pos=({self._car_pos[0]:.1f},{self._car_pos[1]:.1f}) '
            f'dir=({self._car_dir[0]:.2f},{self._car_dir[1]:.2f})',
            throttle_duration_sec=2.0)

        if n_left + n_right == 0 and n_unknown > 0:
            # No colored cones (LiDAR-only mode) — assign UNKNOWN cones geometrically.
            # Cross product of car heading with cone direction: positive = left, negative = right.
            for (rx, ry), cone_type in self._cone_map.items():
                if cone_type != ConeTypes.UNKNOWN:
                    continue
                dx = rx - self._car_pos[0]
                dy = ry - self._car_pos[1]
                fwd = dx * self._car_dir[0] + dy * self._car_dir[1]
                if fwd < -CONE_BEHIND_M:
                    continue
                cross_z = self._car_dir[0] * dy - self._car_dir[1] * dx
                if cross_z > 0:
                    buckets[ConeTypes.LEFT].append([rx, ry])
                else:
                    buckets[ConeTypes.RIGHT].append([rx, ry])
            buckets[ConeTypes.UNKNOWN] = []
            cones_by_type = [
                np.array(b, dtype=np.float64).reshape(-1, 2) if b
                else np.zeros((0, 2), dtype=np.float64)
                for b in buckets
            ]
            n_left  = len(buckets[ConeTypes.LEFT])
            n_right = len(buckets[ConeTypes.RIGHT])
            self.get_logger().info(
                f'LiDAR-only: geometry L/R assignment → left={n_left} right={n_right}',
                throttle_duration_sec=2.0)

        if n_left + n_right == 0:
            self.get_logger().warn(
                'No colored or assignable cones — skipping planner.',
                throttle_duration_sec=5.0)
            return

        # Off-map guard: if the car is far from every mapped cone it has left the
        # track. ft-fsd happily extrapolates a straight path into the void in that
        # state (measured: car cruised 2.6 km off the peanut map at max speed) —
        # publish nothing instead, so pure pursuit's stale-path failsafe brakes.
        all_xy = np.array(list(self._cone_map.keys()), dtype=np.float64)
        d_nearest = float(np.min(np.hypot(all_xy[:, 0] - self._car_pos[0],
                                          all_xy[:, 1] - self._car_pos[1])))
        if d_nearest > 20.0:
            self.get_logger().warn(
                f'Car is {d_nearest:.0f} m from the nearest mapped cone — off track? '
                'Skipping planner so the stale-path failsafe stops the car.',
                throttle_duration_sec=2.0)
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

            # Run racing line optimizer when we have enough points and boundaries.
            # NOT in global-planning exploration: those paths are ephemeral (the
            # lap-1 lock-in replaces them and does its own one-time optimization),
            # and the per-cycle optimizer pushed the exploration replan period to
            # ~2 s — exactly pure pursuit's stale threshold (measured spurious
            # stale-brakes mid-straight on peanut at 6 m/s).
            if (not self._global_planning
                    and len(path_xy) > 3
                    and left_xy is not None and len(left_xy) > 3
                    and right_xy is not None and len(right_xy) > 3):
                left   = np.array(left_xy)[:, :2]
                right  = np.array(right_xy)[:, :2]
                optimized = self._racing_line(centerline, left, right)
            else:
                optimized = centerline

            self._pub_centerline.publish(_path_from_xy(optimized, frame, stamp))

        if left_xy is not None and len(left_xy) > 0:
            self._pub_left.publish(_path_from_xy(np.array(left_xy)[:, :2], frame, stamp))
        if right_xy is not None and len(right_xy) > 0:
            self._pub_right.publish(_path_from_xy(np.array(right_xy)[:, :2], frame, stamp))


    def _racing_line(self, centerline, left, right):
        """Race line through the corridor: PSO (use_pso_optimizer) with the curvature
        minimiser as fallback, else the curvature minimiser alone; raw centerline if both fail.
        Shared by the library and AMZ backends so they get identical post-processing."""
        if self._use_pso_optimizer:
            try:
                out = _pso_raceline(centerline, left, right, safety=0.7)
                self.get_logger().info('PSO race line optimizer applied.', throttle_duration_sec=5.0)
                return out
            except Exception as opt_e:
                self.get_logger().warn(
                    f'PSO optimizer failed: {opt_e} — falling back to curvature minimizer',
                    throttle_duration_sec=5.0)
        try:
            out = _min_curvature_path(centerline, left, right, safety=0.7)
            self.get_logger().info('Racing line optimizer applied.', throttle_duration_sec=5.0)
            return out
        except Exception as opt_e:
            self.get_logger().warn(
                f'Racing line optimizer failed ({type(opt_e).__name__}): {opt_e} — '
                'falling back to raw centerline.',
                throttle_duration_sec=5.0)
            return centerline

    def _plan_amz(self) -> None:
        """Local planning with the AMZ-style planner over the live cone map.
        Cones keep their colour, or UNKNOWN in LiDAR-only mode; the planner's colour term
        then simply has nothing to penalise, and width/angle/spacing terms carry the choice."""
        xy = np.array(list(self._cone_map.keys()), dtype=np.float64)
        col = np.array([int(t) for t in self._cone_map.values()])
        d_nearest = float(np.min(np.hypot(xy[:, 0] - self._car_pos[0], xy[:, 1] - self._car_pos[1])))
        if d_nearest > 20.0:  # same off-map guard as the library path
            self.get_logger().warn(
                f'Car is {d_nearest:.0f} m from the nearest mapped cone — off track? '
                'Skipping planner so the stale-path failsafe stops the car.',
                throttle_duration_sec=2.0)
            return
        res = self._amz.plan(xy, col, self._car_pos, self._car_dir)
        if res is None:
            self.get_logger().warn(f'AMZ planner found no path: {self._amz.last_debug}',
                                   throttle_duration_sec=2.0)
            return
        path, left, right = res
        dbg = self._amz.last_debug
        self.get_logger().info(
            f'amz planner OK: {len(path)} pts, {dbg.get("length", 0):.1f} m over {dbg.get("n_nodes")} gates, '
            f'{dbg.get("n_cones")} cones in range, cost {dbg.get("cost", 0):.2f}',
            throttle_duration_sec=2.0)
        # same racing-line post-processing the library backend gets
        optimized = path
        if len(path) > 3 and len(left) > 3 and len(right) > 3:
            optimized = self._racing_line(path, left, right)
        stamp = self.get_clock().now().to_msg()
        self._pub_centerline.publish(_path_from_xy(optimized, self._map_frame, stamp))
        if len(left):
            self._pub_left.publish(_path_from_xy(left, self._map_frame, stamp))
        if len(right):
            self._pub_right.publish(_path_from_xy(right, self._map_frame, stamp))


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
