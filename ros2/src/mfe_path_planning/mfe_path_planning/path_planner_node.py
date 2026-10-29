#!/usr/bin/env python3
"""ROS 2 path planner: fused cone map -> centerline, boundaries and an optional race line.

Subscribes:
    /planning/cones       (mfe_msgs/Track)        fused, colored cone list
    /ekf/output           (nav_msgs/Odometry)     vehicle pose + twist
Publishes:
    /planning/centerline  (nav_msgs/Path)
    /planning/track_left  (nav_msgs/Path)
    /planning/track_right (nav_msgs/Path)
    /planning/path_horizon (std_msgs/Float32)     colourless local planning: path length ahead (m)
"""

import csv
import hashlib
import json
import math
import os
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32, Float64MultiArray, Int32
from mfe_msgs.msg import Cone, Track
from sensor_msgs.msg import PointCloud2

try:
    from fsd_path_planning import PathPlanner, ConeTypes, MissionTypes
    _LIB_AVAILABLE = True
except ImportError:
    try:
        from ft_fsd_path_planning import PathPlanner, ConeTypes, MissionTypes
        _LIB_AVAILABLE = True
    except ImportError:
        _LIB_AVAILABLE = False

from mfe_path_planning import raceline as _raceline
from mfe_path_planning.boundary_tracer import BoundaryTracer
from mfe_path_planning.planner_utils import (
    arc_length, clamp_to_corridor, clearance_guard, closed_resample, cone_arrays,
    extend_trail, make_skidpad_path as _make_skidpad_path, midpoint_walk, path_from_xy,
    smooth_cyclic, yaw_from_quaternion,
)
from mfe_path_planning.racing_line import (
    _backward_pass, _forward_pass, _menger_kappa, _min_curvature_path, _pso_raceline,
)


_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
# Depth 1: every /planning/cones message carries the whole cone map, so only the newest matters.
_QOS_LATEST = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# mfe_msgs/Cone colour -> fsd_path_planning ConeTypes. BLUE is the left wall, YELLOW the right.
# Orange pylons are the start/finish gate: UNKNOWN so the planner's finish logic is not triggered.
if _LIB_AVAILABLE:
    _MFE_TO_FSD = {
        Cone.BLUE: ConeTypes.LEFT,
        Cone.YELLOW: ConeTypes.RIGHT,
        Cone.ORANGE_BIG: ConeTypes.UNKNOWN,
        Cone.ORANGE_SMALL: ConeTypes.UNKNOWN,
        Cone.UNKNOWN: ConeTypes.UNKNOWN,
    }
else:
    _MFE_TO_FSD = {}

_GRID = 0.5                # cone map key resolution (m)


def _debug_flip_u(seed, key) -> float:
    """DEBUG colour-flip draw in [0, 1): deterministic per 0.5 m map cell, so one physical cone is always flipped or never."""
    ix = round(key[0] / _GRID)
    iy = round(key[1] / _GRID)
    digest = hashlib.blake2b(f'{seed}:{ix}:{iy}'.encode(), digest_size=8).digest()
    return int.from_bytes(digest, 'big') / 2.0 ** 64
_RECOLOUR = os.environ.get('MFE_PLANNER_RECOLOUR', '0') == '1'   # experiment: a later colour replaces the stored one
_MAP_CAP = 1000            # cone map size cap (local planning / global missions)
_SKIDPAD_MAP_CAP = 500
_RL_MAX_OFF_M = 2.0        # race line: car further than this from the line -> tracer path for that cycle
_RL_MOVE_M = 0.5           # race line: the lap-1 trail starts once the car is this far from its first pose
_CONE_VIS_R = 50.0         # local planning visibility radius (m)
_CONE_BEHIND_M = 3.0       # tolerance behind the car for the visibility filter
_OFF_MAP_M = 20.0          # car this far from every mapped cone: off track, publish nothing
_CIRCLE_LAP_DIST = 40.0    # skidpad lap closure: min distance driven (m) ...
_CIRCLE_MAX_R = 10.0       # ... min excursion from the circle start (m) ...
_CIRCLE_CLOSE_R = 4.0      # ... and back within this of the start
_YAW_HIST_LEN = 5
_YAW_RATE_THRESH = 0.1     # rad/s sustained turn threshold
_JUMP_HEAD_M = 5.0         # jump gate: heading is taken over the first 5 m of the path (as the diag 'jumps' metric)
_JUMP_LAT_M = (5.0, 10.0)  # jump gate: lateral offsets checked at these arc lengths ahead (m)
_JUMP_WARN_S = 1.0         # jump gate: throttle for the rejection WARN
_JUMP_LOG_EVERY = 50       # jump gate: counter log period (cycles)


# acceleration midline refit (accel_refit_midline): the straight follows the midline of mapped left/right cone pairs
_ACCEL_PAIR_LAT_M = (2.0, 5.0)     # opposite-side lateral separation accepted as one pair (m)
_ACCEL_PAIR_LON_M = 3.0            # longitudinal offset allowed between the two cones of a pair (m)
_ACCEL_FIT_RANGE_M = 60.0          # only pairs this far ahead of the car are fitted (m); orange cones are never mapped
_ACCEL_MIN_SPREAD_M = 6.0          # midpoints must span this far along the line: 2 pairs 3 m apart gave a -5.7 deg fit (m)
_ACCEL_MAX_DEV_DEG = 10.0          # a fit this far from the car heading is rejected (deg)
_ACCEL_MAX_STEP_DEG = 1.0          # a fit this far from the held line in one cycle is ignored for that cycle (deg)


def accel_pair_midpoints(cones_xy, car_pos, car_dir, lat=_ACCEL_PAIR_LAT_M, lon=_ACCEL_PAIR_LON_M,
                         rng=_ACCEL_FIT_RANGE_M) -> np.ndarray:
    """Midpoints (Nx2, map frame) of left/right cone pairs ahead of the car. Side = sign of the lateral offset in the
    car frame. Each left cone pairs with its nearest right cone that lies within `lon` m along the car and `lat`
    m across. Pure numpy, no ROS."""
    cones = np.asarray(cones_xy, dtype=float).reshape(-1, 2)
    if len(cones) < 2:
        return np.zeros((0, 2))
    fwd = np.asarray(car_dir, dtype=float)
    fwd = fwd / np.linalg.norm(fwd)
    lft = np.array([-fwd[1], fwd[0]])
    rel = cones - np.asarray(car_pos, dtype=float)
    lon_m = rel @ fwd
    lat_m = rel @ lft
    ahead = (lon_m > 0.0) & (lon_m < rng)
    left = np.where(ahead & (lat_m > 0.0))[0]
    right = np.where(ahead & (lat_m < 0.0))[0]
    mids = []
    for i in left:
        if len(right) == 0:
            break
        dlon = np.abs(lon_m[right] - lon_m[i])
        dlat = lat_m[i] - lat_m[right]
        ok = np.where((dlon <= lon) & (dlat >= lat[0]) & (dlat <= lat[1]))[0]
        if len(ok) == 0:
            continue
        j = ok[int(np.argmin(np.hypot(dlon[ok], dlat[ok])))]
        mids.append(0.5 * (cones[i] + cones[right[j]]))
    return np.array(mids, dtype=float).reshape(-1, 2)


def accel_midline_fit(mids_xy, car_dir, min_pairs=2, min_spread=_ACCEL_MIN_SPREAD_M,
                      max_dev_deg=_ACCEL_MAX_DEV_DEG):
    """Total least-squares line through the pair midpoints. Returns (theta, d): heading theta (rad, map frame) and
    offset d with n . x = d, n = (-sin theta, cos theta). None when there are too few pairs, they do not spread along
    the line, or the heading is more than max_dev_deg off the car heading."""
    m = np.asarray(mids_xy, dtype=float).reshape(-1, 2)
    if len(m) < min_pairs:
        return None
    fwd = np.asarray(car_dir, dtype=float)
    fwd = fwd / np.linalg.norm(fwd)
    c = m.mean(axis=0)
    _, _, vt = np.linalg.svd(m - c, full_matrices=False)
    u = vt[0]
    if u @ fwd < 0.0:
        u = -u
    if np.ptp(m @ u) < min_spread:
        return None
    theta = math.atan2(u[1], u[0])
    dev = (theta - math.atan2(fwd[1], fwd[0]) + math.pi) % (2.0 * math.pi) - math.pi
    if abs(dev) > math.radians(max_dev_deg):
        return None
    n = np.array([-u[1], u[0]])
    return theta, float(n @ c)


@dataclass
class _Circle:
    """One skidpad circle: driven trail since the last lap start, excursion stats, locked path."""
    start: np.ndarray
    trail: list = field(default_factory=list)
    dist: float = 0.0
    max_r: float = 0.0
    laps: int = 0
    locked: Optional[np.ndarray] = None


def _counts(buckets):
    return len(buckets[ConeTypes.LEFT]), len(buckets[ConeTypes.RIGHT]), len(buckets[ConeTypes.UNKNOWN])


_VOTE_CAP_TOTAL = 50       # colour_vote: halve a cell's vote counts once their total exceeds this (lets it adapt)


def _vote_update(votes, obs, current, left, right, unknown, vote_min, vote_margin):
    """Colour vote for one grid cell (pure, no ROS). votes = [left, right, unknown] counts, updated in place.
    Returns the cell colour: the majority of coloured votes once there are at least vote_min coloured votes and
    the winner leads by vote_margin; otherwise `current` (UNKNOWN for a new cell, or the colour already held)."""
    if obs == left:
        votes[0] += 1
    elif obs == right:
        votes[1] += 1
    else:
        votes[2] += 1
    if sum(votes) > _VOTE_CAP_TOTAL:
        votes[:] = [v // 2 for v in votes]
    nb, ny = votes[0], votes[1]
    if nb + ny >= vote_min and abs(nb - ny) >= vote_margin:
        return left if nb > ny else right
    return current


def _in_sense_range(dx, dy, fwd_x, fwd_y, radius, behind_m):
    """Same visibility test as _bucket_cones: within radius and not behind the car by more than behind_m.
    dx, dy: offset from the car; fwd_x, fwd_y: car heading (unit vector)."""
    return dx * dx + dy * dy <= radius * radius and dx * fwd_x + dy * fwd_y >= -behind_m


def _expiry_due(count, last_seen, cycle, in_range, min_obs, expiry_cycles):
    """cone_expiry rule (pure): drop only cells observed fewer than min_obs times, unseen for expiry_cycles
    cycles, and currently inside the sensing range (out-of-range absence is not evidence)."""
    return count < min_obs and cycle - last_seen >= expiry_cycles and in_range


def lap_return_banned(cone_keys, centre_xy, radius_m) -> set:
    """Map cell keys within radius_m of centre_xy (radius_m None: every key). Pure."""
    if radius_m is None:
        return set(cone_keys)
    cx, cy = float(centre_xy[0]), float(centre_xy[1])
    return {k for k in cone_keys if math.hypot(k[0] - cx, k[1] - cy) <= radius_m}


# A cell banned at the lap-return trigger is accepted again once a LiDAR detection of a scan taken after the
# trigger lies within this distance of it (a re-observation), so stale copies stay banned and re-seen cones return.
LAP_REOBS_TOL_M = 0.75


def lap_reobserved(banned, det_xy, tol_m=LAP_REOBS_TOL_M) -> set:
    """Banned cell keys with a detection (map frame, one scan) within tol_m. Pure."""
    if not banned or len(det_xy) == 0:
        return set()
    keys = list(banned)
    k = np.asarray(keys, dtype=float).reshape(-1, 2)
    d = np.asarray(det_xy, dtype=float).reshape(-1, 2)
    d2 = np.sum((k[:, None, :] - d[None, :, :]) ** 2, axis=2)
    hit = np.min(d2, axis=1) <= tol_m * tol_m
    return {key for key, h in zip(keys, hit) if h}


# A banned cell more than this far behind the car (along its heading) counts as passed. The LiDAR reports only points
# ahead of the car (x >= ~1.6 m in every rgen7_a scan), so a cell behind it can never be re-seen, and the tracer's
# start gate sits behind the car on the driven path: passing a cell is the only later evidence for it.
LAP_PASSED_M = 1.0


def lap_passed(banned, car_xy, car_dir, behind_m=LAP_PASSED_M) -> set:
    """Banned cell keys the car has driven past (more than behind_m behind it along car_dir). Pure."""
    if not banned:
        return set()
    c, s = float(car_dir[0]), float(car_dir[1])
    cx, cy = float(car_xy[0]), float(car_xy[1])
    return {k for k in banned if (k[0] - cx) * c + (k[1] - cy) * s < -behind_m}


def _cloud_xy(msg):
    """x, y of the finite points of a PointCloud2 (sensor-frame metres)."""
    from sensor_msgs_py import point_cloud2   # ROS runtime helper, imported only when a cloud arrives
    arr = point_cloud2.read_points(msg, field_names=('x', 'y'), skip_nans=True)
    return np.asarray(arr['x'], dtype=float), np.asarray(arr['y'], dtype=float)


class LapReturnReset:
    """Lap-return trigger (pure). Fires once per lap when the car comes back into the start area after
    travelling min(min_travel_m, len_frac * previous lap length) since it last left the area.

    The start area is start_r_m around the first odometry position. The car must first leave beyond
    2 * start_r_m (re-arming the trigger) before a return counts. The lap length is the travel from the last
    departure to the previous return (None until a lap has completed, so min_travel_m applies on lap 1).
    """

    def __init__(self, start_r_m, min_travel_m, len_frac=0.6):
        self.start_r = float(start_r_m)
        self.min_travel = float(min_travel_m)
        self.len_frac = float(len_frac)
        self.start = None
        self.prev = None
        self.lap_travel = 0.0
        self.lap_len = None
        self.away = False        # left the start area since the last return
        self.triggered = False   # fired on this lap already
        self.fires = 0

    def min_travel_eff(self) -> float:
        if self.lap_len is None:
            return self.min_travel
        return min(self.min_travel, self.len_frac * self.lap_len)

    def update(self, pos) -> bool:
        """Feed one odometry position. True on the step the trigger fires."""
        pos = np.asarray(pos, dtype=float)
        if self.start is None:
            self.start = pos.copy()
            self.prev = pos.copy()
            return False
        self.lap_travel += float(np.hypot(*(pos - self.prev)))
        self.prev = pos.copy()
        d = float(np.hypot(*(pos - self.start)))
        if d > 2.0 * self.start_r and not self.away:
            self.away = True
            self.triggered = False
            self.lap_travel = 0.0
        if d < self.start_r and self.away:
            fired = False
            if not self.triggered and self.lap_travel >= self.min_travel_eff():
                self.triggered = True
                self.fires += 1
                fired = True
            if self.lap_travel > 2.0 * self.start_r:   # ignore wiggles at the start line
                self.lap_len = self.lap_travel
            self.away = False
            return fired
        return False


def _arc_length(xy) -> float:
    """Total length (m) of the polyline xy (N x 2)."""
    xy = np.asarray(xy, dtype=float)
    return float(np.sum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1])))) if len(xy) >= 2 else 0.0


def _remaining_ahead(xy, pos) -> float:
    """Arc length (m) from the point of xy nearest to pos (projected onto its segments) to the end of xy."""
    xy = np.asarray(xy, dtype=float)
    if len(xy) < 2:
        return 0.0
    a, ab = xy[:-1], np.diff(xy, axis=0)
    seg = np.hypot(ab[:, 0], ab[:, 1])
    t = np.clip(np.sum((pos - a) * ab, axis=1) / np.maximum(np.sum(ab * ab, axis=1), 1e-12), 0.0, 1.0)
    i = int(np.argmin(np.linalg.norm(pos - (a + t[:, None] * ab), axis=1)))
    return max(float(np.sum(seg) - np.sum(seg[:i]) - t[i] * seg[i]), 0.0)


class PathPlannerNode(Node):

    def __init__(self):
        super().__init__('path_planner_node')

        if not _LIB_AVAILABLE:
            self.get_logger().error(
                'ft_fsd_path_planning is not installed. '
                'Run: pip install ft-fsd-path-planning')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('mission', 'autocross')  # autocross | trackdrive | acceleration | skidpad | peanut
        self.declare_parameter('skidpad_mode', 'cone_based')  # cone_based | hardcoded
        self.declare_parameter('use_pso_optimizer', True)
        # Local (non-global) backend: fasttube = fsd_path_planning.
        self.declare_parameter('planner_backend', 'fasttube')
        # Uncoloured cones (LiDAR-only), local missions:
        #   trace         — boundary_tracer labels the walls; its gate-midpoint centreline is used directly
        #   trace_backend — same wall labels fed to the backend as left/right
        #   heading       — legacy: side of the car's heading (fails in corners)
        #   fasttube      — uncoloured cones go to ft-fsd unchanged; its own sorting picks the sides
        #   colour_hint   — trace always (colours stripped); cone colours only vote which traced wall is left/right
        self.declare_parameter('unknown_cone_mode', 'trace')
        # Racing-line optimiser on colourless (traced) paths. Off by default: traced paths are short and the
        # optimiser's ~0.27 s per cycle made the path lag the car.
        self.declare_parameter('colourless_raceline', False)
        # Latch between the trace and library pipelines (local missions). Off: the pipeline is picked per cycle
        # from the visible coloured count, which flip-flops when YOLO colours appear/disappear at the window edge.
        #   min_coloured_each — trace -> library once the window holds this many BLUE and YELLOW cones each
        #   revert_after      — library -> trace after this many consecutive cycles with no coloured cone
        # Colour laps (local missions, unknown_cone_mode colour_hint): from lap colour_after_lap on (laps counted on
        # /planning/laps_completed), plan with the camera's BLUE/YELLOW cones through the library backend instead of
        # the tracer. UNKNOWN cones take the side the lap-1 tracer gave their map cell. 0 = off.
        self.declare_parameter('colour_after_lap', 0)
        # Racing-line optimiser on the colour-lap library path.
        self.declare_parameter('colour_lap_raceline', False)
        self.declare_parameter('pipeline_hysteresis', False)
        self.declare_parameter('min_coloured_each', 2)
        self.declare_parameter('revert_after', 20)
        # Jump gate on library / colourless centrelines (before publish). A candidate whose heading (first 5 m)
        # turns more than jump_heading_deg from the last published path, or whose points at 5 m and 10 m are
        # more than jump_lateral_m from it, is replaced by the last published path for up to jump_hold_max
        # consecutive cycles; after that the candidate is accepted so a genuine change gets through.
        self.declare_parameter('jump_gate', False)
        self.declare_parameter('jump_heading_deg', 25.0)
        self.declare_parameter('jump_lateral_m', 1.5)
        self.declare_parameter('jump_hold_max', 5)
        # jump_min_path_m: the gate is skipped when the candidate or the held path is shorter than this (the 5 m and
        #   10 m probes clamp to the path end on short paths). jump_remaining_guard: never hold a stale path whose
        #   remaining length ahead of the car is shorter than the fresh candidate's (only with odometry).
        self.declare_parameter('jump_min_path_m', 10.0)
        # accel_refit_midline (acceleration only): publish a straight fitted to the midline of the mapped left/right
        # cone pairs ahead of the car, re-fitted every cycle. accel_midline_alpha: low-pass gain on heading and offset.
        # False restores the library/racing-line path.
        self.declare_parameter('accel_refit_midline', True)
        self.declare_parameter('accel_midline_alpha', 0.3)
        self._accel_refit = bool(self.get_parameter('accel_refit_midline').value)
        self._accel_alpha = float(self.get_parameter('accel_midline_alpha').value)
        self._accel_line = None            # (theta, d) low-passed midline, or None before the first fit
        self.declare_parameter('jump_remaining_guard', True)
        # Cone-map colour votes and expiry (see _merge_cones). All off by default: behaviour is unchanged.
        self.declare_parameter('colour_vote', False)
        self.declare_parameter('vote_min', 3)          # coloured votes needed before a cell takes a colour
        self.declare_parameter('vote_margin', 2)       # winner's lead over the other colour
        self.declare_parameter('cone_expiry', False)
        self.declare_parameter('expiry_min_obs', 3)    # cells seen at least this often never expire
        self.declare_parameter('expiry_cycles', 30)    # unseen for this many merge cycles (inside sensing range)
        # Lap-return map reset (local missions, boundary tracer input). On the return to the start after a lap,
        # the cone cells mapped within lap_return_radius_m of the START POSE are excluded from the tracer's map for
        # the rest of the run, so drifted duplicates of the start-area cones cannot close the free-track triangle.
        # Centred on the start pose, not the car: the trigger fires ~start_r away, and copies 15-18 m from the car
        # were missed (replay: 15 m car-centred leaves stalls; 15 m start-centred leaves the path outside).
        #   lap_return_min_travel_m — travel since leaving the start area before a return counts (capped at
        #                             lap_return_len_frac * the previous lap length once a lap is known)
        #   lap_return_start_r_m    — the start area: the return is inside it, and leaving beyond 2x it re-arms
        # Banned cells come back when the LiDAR re-sees them (lap_reobserved, LAP_REOBS_TOL_M), so a low-drift map
        # does not lose the start-gate cones to a ban that never lifts.
        self.declare_parameter('lap_return_reset', True)
        self.declare_parameter('lap_return_radius_m', 25.0)
        self.declare_parameter('lap_return_min_travel_m', 250.0)
        self.declare_parameter('lap_return_start_r_m', 15.0)
        self.declare_parameter('lap_return_len_frac', 0.6)
        # Planner-cycle timing: logs mean/p95/max of whole _cones_cb every 50 cycles. Off by default.
        self.declare_parameter('planner_profile', False)
        # DEBUG ablation only: swap BLUE<->YELLOW for a fixed fraction of cone cells (orange never flipped). 0.0 = off.
        self.declare_parameter('debug_flip_prob', 0.0)
        self.declare_parameter('debug_flip_seed', 1)
        # DEBUG ablation only: publish the unoptimised library/midpoint path (PSO / min-curvature bypassed). False = normal.
        self.declare_parameter('debug_disable_raceline', False)
        # DEBUG: append one JSON line per planning cycle (library inputs + intermediate sorting/matching/path). Off = no-op.
        self.declare_parameter('debug_dump_intermediate', False)
        self.declare_parameter('debug_dump_path', '/tmp/fsd_intermediate.jsonl')
        # DEBUG: follow a fixed precomputed closed path (x,y CSV, header row, 0.5 m spacing, planner frame) instead of
        # planning. The next 40 points from the nearest point (searched near the last index) go through _publish_plan.
        # Jump gate is disabled in this mode. '' = off (normal behaviour).
        self.declare_parameter('debug_fixed_path_csv', '')
        # Offline race line for laps >= raceline_after_lap (raceline.py). Built once from the lap-1 cone union and
        # trail, then frozen: each cycle publishes raceline_window_m of it from the car's projection, with target
        # speeds on /planning/target_speeds. False = behaviour unchanged.
        self.declare_parameter('raceline_enable', False)
        self.declare_parameter('raceline_after_lap', 1)
        self.declare_parameter('raceline_extra_margin_m', 0.25)
        self.declare_parameter('raceline_a_lat', 6.0)
        self.declare_parameter('raceline_v_max', 8.0)
        self.declare_parameter('raceline_window_m', 35.0)

        self._map_frame = self.get_parameter('map_frame').value
        mission_str = self.get_parameter('mission').value
        self._use_pso_optimizer = self.get_parameter('use_pso_optimizer').value
        if self._use_pso_optimizer:  # compile the PSO hot loops now, not on the first planning cycle
            _w = np.c_[np.linspace(0.0, 3.0, 4), np.zeros(4)]
            _backward_pass(_forward_pass(_menger_kappa(_w), np.ones(3)), np.ones(3))
        self._mission = mission_str
        self._skidpad_mode = self.get_parameter('skidpad_mode').value
        self._backend = self.get_parameter('planner_backend').value
        self._unknown_mode = self.get_parameter('unknown_cone_mode').value
        self._colourless_raceline = bool(self.get_parameter('colourless_raceline').value)
        self._colour_after_lap = int(self.get_parameter('colour_after_lap').value)
        self._colour_lap_raceline = bool(self.get_parameter('colour_lap_raceline').value)
        self._laps_done = 0                    # from /planning/laps_completed (finish_detector)
        self._colour_lap_active = False        # colour planning has started (logged once)
        self._side_votes: dict = {}            # (x, y) map cell -> lap-1 tracer side votes (+ left, - right)
        self._hint_swapped = False             # set by _colour_hint_sides when it swapped the traced walls
        self._colour_cycles = 0
        self._colour_fallbacks = 0
        self._colour_cam_sum = 0               # camera-coloured cones per colour-lap cycle (for the mean)
        self._colour_fill_sum = 0              # side-vote-filled UNKNOWN cones per colour-lap cycle
        self._last_lib_error = None            # reason set by _run_library when it returns None on an exception
        self._pipeline_hysteresis = bool(self.get_parameter('pipeline_hysteresis').value)
        self._min_coloured_each = int(self.get_parameter('min_coloured_each').value)
        self._revert_after = int(self.get_parameter('revert_after').value)
        self._pipeline_trace = True      # latch state: True = trace pipeline, False = library
        self._pipeline_cycles = 0
        self._pipeline_switches = 0
        self._coloured_absent = 0        # consecutive cycles with no BLUE/YELLOW cone in the window
        self._jump_gate_on = bool(self.get_parameter('jump_gate').value)
        self._jump_heading_deg = float(self.get_parameter('jump_heading_deg').value)
        self._jump_lateral_m = float(self.get_parameter('jump_lateral_m').value)
        self._jump_hold_max = int(self.get_parameter('jump_hold_max').value)
        self._jump_min_path_m = float(self.get_parameter('jump_min_path_m').value)
        self._jump_remaining_guard = bool(self.get_parameter('jump_remaining_guard').value)
        self._fixed_xy: Optional[np.ndarray] = None   # debug_fixed_path_csv: fixed closed path, or None (off)
        self._fixed_idx: Optional[int] = None
        _fixed_csv = str(self.get_parameter('debug_fixed_path_csv').value)
        if _fixed_csv:
            self._fixed_xy = self._load_fixed_path(_fixed_csv)
            self._jump_gate_on = False   # the fixed path has no jumps to gate; the gate would only add rejections
            self.get_logger().info(
                f'debug_fixed_path_csv={_fixed_csv}: fixed path mode ({len(self._fixed_xy)} pts, '
                f'jump gate disabled in this mode)')
        # Cone-map hygiene (both off by default). colour_vote: per-cell colour votes instead of latching the first
        # colour. cone_expiry: drop sparsely-observed cells that have stopped being seen inside the sensing range.
        self._colour_vote = bool(self.get_parameter('colour_vote').value)
        self._vote_min = int(self.get_parameter('vote_min').value)
        self._vote_margin = int(self.get_parameter('vote_margin').value)
        self._cone_expiry = bool(self.get_parameter('cone_expiry').value)
        self._expiry_min_obs = int(self.get_parameter('expiry_min_obs').value)
        self._expiry_cycles = int(self.get_parameter('expiry_cycles').value)
        self._planner_profile = bool(self.get_parameter('planner_profile').value)
        self._debug_disable_raceline = bool(self.get_parameter('debug_disable_raceline').value)
        self._dump_on = bool(self.get_parameter('debug_dump_intermediate').value)
        self._dump_fh = None
        if self._dump_on:
            self._dump_fh = open(str(self.get_parameter('debug_dump_path').value), 'w', buffering=1)
            self.get_logger().warn(f'debug_dump_intermediate ON: writing {self._dump_fh.name}')
        if self._debug_disable_raceline:
            self.get_logger().info('debug_disable_raceline=True: optimisers bypassed')
        self._tracer = BoundaryTracer()
        # Per-cycle planner debug log (JSON lines, one per colourless planning cycle). '' = off.
        self.declare_parameter('planner_debug_log', '')
        self._dbg_fh = None
        _dbg_path = str(self.get_parameter('planner_debug_log').value)
        if _dbg_path:
            self._dbg_fh = open(_dbg_path, 'w', buffering=1)
            self.get_logger().info(f'planner_debug_log: writing {_dbg_path}')
        # Lap-return map reset (see the lap_return_* parameters). Not for skidpad (own state machine) or
        # acceleration (a single straight, no lap to return to).
        self._lap_reset = None
        self._lap_radius = float(self.get_parameter('lap_return_radius_m').value)
        self._lap_banned: set = set()          # cone cells excluded from the tracer's map until re-observed
        self._lap_reobs = 0                    # banned cells re-observed by the LiDAR since the start
        self._lap_skip_logged = False
        if (bool(self.get_parameter('lap_return_reset').value)
                and mission_str not in ('skidpad', 'acceleration')):
            self._lap_reset = LapReturnReset(
                self.get_parameter('lap_return_start_r_m').value,
                self.get_parameter('lap_return_min_travel_m').value,
                self.get_parameter('lap_return_len_frac').value)
            # Re-observation needs the current scan (the cumulative SLAM map cannot tell a re-seen cone from a
            # remembered one). Subscribed only when the reset is armed.
            self.create_subscription(PointCloud2, '/perception/cones_uncolored', self._lidar_cb, _QOS_LATEST)
        self._hint_cycles = 0                                  # colour_hint: planning cycles traced
        self._hint_counts = {'override': 0, 'agree': 0, 'fallback': 0}

        # Trackdrive plans GLOBALLY over the full cone map: the global boundary trace follows each wall
        # around the whole track, which disambiguates walls that pass near each other (figure-8 crossings).
        # Other missions use the local visibility filter.
        self._global_planning = False
        self._planner = None
        if _LIB_AVAILABLE and not self._hardcoded_skidpad():
            # No per-track entries: an unknown mission string (e.g. a track name) plans as autocross.
            mission_map = {
                'autocross': MissionTypes.autocross,
                'trackdrive': MissionTypes.trackdrive,
                'acceleration': MissionTypes.acceleration,
                'skidpad': MissionTypes.autocross,
            }
            mission = mission_map.get(mission_str, MissionTypes.autocross)
            self._planner = PathPlanner(mission)
            self._global_planning = (mission == MissionTypes.trackdrive)
            if mission == MissionTypes.acceleration and self._planner.relocalizer is not None:
                self._trim_accel_known_path()

        # ---------- State ----------
        self._car_pos = np.array([0.0, 0.0])
        self._car_dir = np.array([1.0, 0.0])   # unit vector, default facing +X
        self._car_yaw_rate = 0.0
        self._odom_received = False
        self._cone_map: dict = {}              # (x, y) on the 0.5 m grid -> ConeTypes
        self._cell_votes: dict = {}            # colour_vote: (x, y) -> [left, right, unknown] observation counts
        self._cell_obs: dict = {}              # cone_expiry: (x, y) -> [cycles observed, last cycle seen]
        self._merge_cycle = 0
        self._colour_flips = 0                 # cells whose decided colour reversed (BLUE <-> YELLOW)
        self._cones_expired = 0
        self._flip_prob = float(self.get_parameter('debug_flip_prob').value)
        self._flip_seed = int(self.get_parameter('debug_flip_seed').value)
        self._flip_seen: set = set()           # DEBUG: distinct 0.5 m cells observed with a BLUE/YELLOW colour
        self._flip_set: set = set()            # DEBUG: those cells whose colour is flipped
        if self._flip_prob > 0.0:
            self.get_logger().warn(f'DEBUG colour flip ON: prob={self._flip_prob} seed={self._flip_seed}')
        self._prof_ms: list = []               # planner_profile: _cones_cb durations since the last report
        self._skidpad_path_published = False
        self._skidpad_path = None
        self._reset_skidpad_state()

        # Lap-1 lock-in (global missions): record the driven trail; on closing the loop, build the full-lap
        # centerline from the cone map ordered by the trail, optimise it once and lock it.
        self._trail: list = []
        self._trail_dist = 0.0
        self._trail_max_r = 0.0
        self._locked_path = None

        # Jump gate state: last published library/colourless centreline, consecutive holds, counters.
        self._jump_prev: Optional[np.ndarray] = None
        self._jump_hold = 0
        self._jump_cycles = 0
        self._jump_last = 'off'            # jump-gate decision of the last cycle (planner_debug_log)
        self._jump_rejects = 0
        self._jump_accepted_after_hold = 0

        if mission_str == 'skidpad':
            if self._skidpad_mode == 'hardcoded':
                self._skidpad_path = _make_skidpad_path()
                self.get_logger().info(
                    f'PathPlannerNode started. Mission: skidpad (hardcoded mode) | '
                    f'path: {len(self._skidpad_path)} waypoints, '
                    f'sequence: 2 laps LEFT + 2 laps RIGHT + exit')
            else:
                self.get_logger().info(
                    'PathPlannerNode started. Mission: skidpad (cone_based mode) | '
                    'state machine initialized')
        else:
            self.get_logger().info(
                f'PathPlannerNode started. Mission: {mission_str} | local planner: {self._backend}')

        # Offline race line state (see _rl_* methods): 'off' -> 'collect' -> 'built', or 'failed' (tracer for good).
        self._rl_on = bool(self.get_parameter('raceline_enable').value)
        self._rl_after = int(self.get_parameter('raceline_after_lap').value)
        self._rl_margin = float(self.get_parameter('raceline_extra_margin_m').value)
        self._rl_a_lat = float(self.get_parameter('raceline_a_lat').value)
        self._rl_v_max = float(self.get_parameter('raceline_v_max').value)
        self._rl_window_m = float(self.get_parameter('raceline_window_m').value)
        self._rl_state = 'collect' if self._rl_on else 'off'
        self._rl_p0 = None                       # first odometry position (movement is measured from it)
        self._rl_trail: list = []                # car trail from first movement, until the build
        self._rl_union = _raceline.ConeUnion() if self._rl_on else None
        self._rl = None                          # the built race line
        self._rl_idx: Optional[int] = None       # race-line index of the last published window
        self._rl_cycles = 0
        self._rl_fallbacks = 0
        self._rl_build_s: Optional[float] = None

        # ---------- Subscribers / publishers ----------
        self.create_subscription(Track, '/planning/cones', self._cones_cb, _QOS_LATEST)
        # finish_detector publishes laps on a RELIABLE, KEEP_LAST depth-10 QoS (same as _QOS).
        self.create_subscription(Int32, '/planning/laps_completed', self._laps_cb, _QOS)
        self.create_subscription(Odometry, '/ekf/output', self._odom_cb, _QOS)
        self._pub_centerline = self.create_publisher(Path, '/planning/centerline', _QOS)
        self._pub_left = self.create_publisher(Path, '/planning/track_left', _QOS)
        self._pub_right = self.create_publisher(Path, '/planning/track_right', _QOS)
        self._pub_horizon = self.create_publisher(Float32, '/planning/path_horizon', _QOS)
        if self._rl_on:
            self._pub_speeds = self.create_publisher(Float64MultiArray, '/planning/target_speeds', _QOS)

    def _accel_midline_path(self, centerline: np.ndarray) -> np.ndarray:
        """Acceleration: the straight follows the midline of the mapped cone pairs ahead of the car. ft-fsd's
        AccelerationRelocalizer fits only the left cones, once, and the path inherits that tilt. Heading and offset
        are low-passed. With fewer than 2 pairs the previous line is kept; before the first fit the library path is
        published unchanged."""
        fit = accel_midline_fit(accel_pair_midpoints(self._cone_xy(), self._car_pos, self._car_dir), self._car_dir)
        if fit is not None and self._accel_line is not None:
            step = (fit[0] - self._accel_line[0] + math.pi) % (2.0 * math.pi) - math.pi
            if abs(step) > math.radians(_ACCEL_MAX_STEP_DEG):
                fit = None                      # a jump this large is a few-pair artefact: hold the line
        if fit is not None:
            theta, d = fit
            if self._accel_line is None:
                self._accel_line = (theta, d)
            else:
                th0, d0 = self._accel_line
                dth = (theta - th0 + math.pi) % (2.0 * math.pi) - math.pi
                self._accel_line = (th0 + self._accel_alpha * dth, d0 + self._accel_alpha * (d - d0))
        elif self._accel_line is None:
            return centerline
        theta, d = self._accel_line
        u = np.array([math.cos(theta), math.sin(theta)])
        n = np.array([-u[1], u[0]])
        car = np.asarray(self._car_pos, dtype=float)
        foot = car - (n @ car - d) * n          # the car's projection onto the line
        s = np.linspace(0.0, max(arc_length(centerline), 1.0), len(centerline))
        return foot + s[:, None] * u[None, :]

    def _hardcoded_skidpad(self) -> bool:
        return self._mission == 'skidpad' and self._skidpad_mode == 'hardcoded'

    def _trim_accel_known_path(self) -> None:
        """ft-fsd's known acceleration path is a loop; its pathing splices the return leg into the path
        (replayed HA02: 7 deg swerve). Keep only the outbound straight."""
        known = self._planner.relocalizer.get_known_global_path()
        ret = known[:, 1] > 1.0
        if ret.any():
            known = known[:int(np.argmax(ret))]
            self._planner.relocalizer.get_known_global_path = lambda: known

    def _reset_skidpad_state(self) -> None:
        self._skidpad_state = 'APPROACH'   # APPROACH -> LEFT_CIRCLE -> RIGHT_CIRCLE -> EXIT
        self._skidpad_left: Optional[_Circle] = None
        self._skidpad_right: Optional[_Circle] = None
        self._skidpad_sustained_yaw_rate_sign = None   # +1 (left/CCW), -1 (right/CW), None
        self._skidpad_yaw_rate_history = []

    # ------------------------------------------------------------------ callbacks

    def _odom_cb(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        self._car_pos = np.array([pos.x, pos.y])
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self._car_dir = np.array([math.cos(yaw), math.sin(yaw)])
        self._car_yaw_rate = float(msg.twist.twist.angular.z)
        self._odom_received = True
        self._tracer.add_pose(self._car_pos)
        if self._rl_state == 'collect' and self._laps_done < self._rl_after:
            self._rl_collect_odom()
        if self._lap_reset is not None and self._lap_reset.update(self._car_pos):
            self._lap_return_fire()
        if self._lap_banned:
            self._lap_lift()

        if self._global_planning and self._locked_path is None:
            step = extend_trail(self._trail, self._car_pos)
            if step is not None:
                self._trail_dist += step
                self._trail_max_r = max(
                    self._trail_max_r, float(np.linalg.norm(self._car_pos - self._trail[0])))

    def _cones_cb(self, msg: Track) -> None:
        if not self._planner_profile:
            self._cones_cb_body(msg)
            return
        t0 = time.perf_counter()
        self._cones_cb_body(msg)
        self._prof_ms.append((time.perf_counter() - t0) * 1e3)
        if len(self._prof_ms) >= 50:
            ms = np.array(self._prof_ms)
            self.get_logger().info(
                f'planner_cycle_ms mean={ms.mean():.2f} p95={np.percentile(ms, 95):.2f} '
                f'max={ms.max():.2f} n={len(ms)} debug_disable_raceline={self._debug_disable_raceline}')
            self._prof_ms = []

    @staticmethod
    def _load_fixed_path(path: str) -> np.ndarray:
        """Read x,y rows from a CSV (header row skipped); returns (N,2) float array."""
        rows = []
        with open(path, newline='') as f:
            for r in csv.reader(f):
                try:
                    rows.append((float(r[0]), float(r[1])))
                except (ValueError, IndexError):
                    continue
        xy = np.array(rows, dtype=float).reshape(-1, 2)
        if len(xy) < 2:
            raise RuntimeError(f'debug_fixed_path_csv {path}: need at least 2 x,y rows, got {len(xy)}')
        return xy

    def _publish_fixed_path(self) -> None:
        """Fixed path mode: publish the next 40 points (20 m) from the nearest path point, wrapping the loop."""
        xy = self._fixed_xy
        n = len(xy)
        d_all = np.hypot(xy[:, 0] - self._car_pos[0], xy[:, 1] - self._car_pos[1])
        if self._fixed_idx is None:
            idx = int(np.argmin(d_all))
        else:  # search only near the last index so the loop passing close to itself cannot pull the index across
            cand = (self._fixed_idx + np.arange(-30, 31)) % n
            idx = int(cand[int(np.argmin(d_all[cand]))])
        self._fixed_idx = idx
        self._publish_plan(xy[(idx + np.arange(40)) % n], None, None)

    def _cones_cb_body(self, msg: Track) -> None:
        if self._fixed_xy is not None:
            if self._odom_received:
                self._publish_fixed_path()
            return
        if self._locked_path is not None:
            self._publish_centerline(self._locked_path)
            return

        if self._mission == 'skidpad':
            if self._skidpad_mode == 'hardcoded':
                if self._odom_received:
                    self._publish_centerline(self._skidpad_path)
                    if not self._skidpad_path_published:
                        self._skidpad_path_published = True
                        self.get_logger().info(
                            f'skidpad hardcoded path first publish: {len(self._skidpad_path)} waypoints')
            else:
                self._handle_skidpad_cone_based(msg)
            return

        if not _LIB_AVAILABLE or self._planner is None:
            return
        if not self._odom_received:
            self.get_logger().warn('No odometry yet — skipping path calculation.', throttle_duration_sec=5.0)
            return
        if not msg.track:
            return

        self._merge_cones(msg)
        self._evict_far_cones(_MAP_CAP)

        if (self._global_planning and self._trail and self._trail_dist > 60.0
                and self._trail_max_r > 15.0
                and float(np.linalg.norm(self._car_pos - self._trail[0])) < 4.0):
            locked = self._build_locked_loop()
            if locked is not None:
                self._locked_path = locked
                self.get_logger().info(
                    f'Lap 1 complete — locked full-loop path: {len(locked)} waypoints, '
                    f'{arc_length(locked):.1f} m, built from {len(self._cone_map)} live-mapped cones. '
                    f'Re-planning stopped.')
                self._publish_centerline(locked)
                return

        # Offline race line (raceline_enable): collects the lap-1 union, builds once, then follows the line.
        if self._rl_state != 'off' and self._rl_cycle(msg):
            return

        # Acceleration also gets the full map: ft-fsd's AccelerationRelocalizer needs >= 4 cones on the
        # car's left, and the visibility-filtered set never held more than 3 (replayed bags: 0.3-1.4 m off).
        use_full_map = self._global_planning or self._mission == 'acceleration'
        buckets = self._bucket_cones(visible_only=not use_full_map)
        cones_by_type = cone_arrays(buckets)
        n_left, n_right, n_unknown = _counts(buckets)

        # Colour laps: from colour_after_lap completed laps on, the camera colours plan through the library backend
        # (UNKNOWN cones side-filled from the lap-1 tracer votes). Before the colour_hint check; off with 0.
        if self._colour_after_lap > 0 and self._laps_done >= self._colour_after_lap and not use_full_map:
            if not self._colour_lap_active:
                self._colour_lap_active = True
                self.get_logger().info(
                    f'COLOUR LAPS ON: laps_completed={self._laps_done} (colour_after_lap={self._colour_after_lap}) '
                    f't={self.get_clock().now().nanoseconds * 1e-9:.1f}s: planning with camera BLUE/YELLOW cones '
                    f'via the library backend; side votes={len(self._side_votes)} cells, '
                    f'raceline={self._colour_lap_raceline}')
            self._plan_colour_lap(buckets)
            return

        # colour_hint: always trace the walls (colours stripped); the colours only pick the left/right labels.
        # Checked first so it wins over the pipeline_hysteresis latch, which is trace/trace_backend only.
        if not use_full_map and self._unknown_mode == 'colour_hint':
            self._plan_colourless(colour_hint=True)
            return

        # Colourless local planning: trace the walls instead of guessing sides from the heading.
        trace_ok = not use_full_map and self._unknown_mode in ('trace', 'trace_backend')
        if self._pipeline_hysteresis:
            if trace_ok:
                self._update_pipeline_mode(n_left, n_right)
            trace_now = trace_ok and self._pipeline_trace and n_unknown > 0
        else:
            trace_now = trace_ok and n_left + n_right == 0 and n_unknown > 0
        if trace_now:
            self._plan_colourless()
            return

        self.get_logger().info(
            f'planner input: left={n_left} right={n_right} unknown={n_unknown} '
            f'pos=({self._car_pos[0]:.1f},{self._car_pos[1]:.1f}) '
            f'dir=({self._car_dir[0]:.2f},{self._car_dir[1]:.2f})',
            throttle_duration_sec=2.0)

        # fasttube: hand uncoloured cones to ft-fsd unchanged so its own sorting picks the sides.
        colourless_native = (n_left + n_right == 0 and n_unknown > 0 and self._unknown_mode == 'fasttube')
        if n_left + n_right == 0 and n_unknown > 0 and not colourless_native:
            self._assign_unknown_sides(buckets)
            cones_by_type = cone_arrays(buckets)
            n_left, n_right = len(buckets[ConeTypes.LEFT]), len(buckets[ConeTypes.RIGHT])
            self.get_logger().info(
                f'LiDAR-only: geometry L/R assignment → left={n_left} right={n_right}',
                throttle_duration_sec=2.0)

        if n_left + n_right == 0 and not colourless_native:
            self.get_logger().warn(
                'No colored or assignable cones — skipping planner.',
                throttle_duration_sec=5.0)
            return

        result = self._run_library(cones_by_type, n_left, n_right, n_unknown)
        if result is None:
            return
        path_xy, left_xy, right_xy = result
        self.get_logger().info(
            f'planner OK: path_pts={len(path_xy) if path_xy is not None else 0}',
            throttle_duration_sec=2.0)

        # final_path columns: (spline_parameter, x, y, curvature); boundaries are already (x, y).
        centerline = None
        if path_xy is not None and len(path_xy) > 0:
            centerline = np.array(path_xy)[:, 1:3]
            # Race line for local missions. Not during global exploration: those paths are ephemeral (the
            # lap-1 lock-in optimises once) and a per-cycle optimiser pushes the replan period past pure
            # pursuit's stale threshold.
            if (not self._global_planning
                    and not self._debug_disable_raceline
                    and (self._colourless_raceline or not colourless_native)
                    and len(path_xy) > 3
                    and left_xy is not None and len(left_xy) > 3
                    and right_xy is not None and len(right_xy) > 3):
                centerline = self._racing_line(centerline, np.array(left_xy)[:, :2], np.array(right_xy)[:, :2])
            if self._accel_refit and self._mission == 'acceleration' and len(centerline) > 1:
                centerline = self._accel_midline_path(np.asarray(centerline))
        self._publish_plan(centerline, left_xy, right_xy)

    def _handle_skidpad_cone_based(self, msg: Track) -> None:
        """Skidpad state machine: APPROACH (local) -> LEFT_CIRCLE (lock after 1 lap, 2 laps) ->
        RIGHT_CIRCLE (same) -> EXIT (local). Yaw-rate sign picks the circle; the cone colour and the
        trail give the lap count."""
        if not self._odom_received:
            return

        self._update_skidpad_yaw_rate_sign()
        self._merge_cones(msg, insert_cap=_SKIDPAD_MAP_CAP)

        if self._skidpad_state == 'LEFT_CIRCLE':
            self._advance_circle(self._skidpad_left)
        elif self._skidpad_state == 'RIGHT_CIRCLE':
            self._advance_circle(self._skidpad_right)
        self._step_skidpad_state()

        circle = {'LEFT_CIRCLE': self._skidpad_left, 'RIGHT_CIRCLE': self._skidpad_right}.get(self._skidpad_state)
        if circle is not None and circle.locked is not None:
            self._publish_centerline(circle.locked)
            return

        self._handle_local_cone_planning(msg)

    def _handle_local_cone_planning(self, msg: Track) -> None:
        """Local ft-fsd planning for skidpad states without a locked circle (APPROACH, EXIT, incomplete)."""
        if not _LIB_AVAILABLE or self._planner is None:
            return
        if not msg.track:
            return

        buckets = self._bucket_cones(visible_only=True)
        cones_by_type = cone_arrays(buckets)
        n_left, n_right, n_unknown = _counts(buckets)
        if n_left + n_right == 0 and n_unknown > 0:
            self._assign_unknown_sides(buckets)
            cones_by_type = cone_arrays(buckets)
            n_left, n_right = len(buckets[ConeTypes.LEFT]), len(buckets[ConeTypes.RIGHT])

        if n_left + n_right == 0:
            self.get_logger().warn(
                'No colored or assignable cones — skipping planner.',
                throttle_duration_sec=5.0)
            return

        result = self._run_library(cones_by_type, n_left, n_right, n_unknown)
        if result is None:
            return
        path_xy, left_xy, right_xy = result
        centerline = None
        if path_xy is not None and len(path_xy) > 0:
            centerline = np.array(path_xy)[:, 1:3]
            if (len(path_xy) > 3 and not self._debug_disable_raceline
                    and left_xy is not None and len(left_xy) > 3
                    and right_xy is not None and len(right_xy) > 3):
                try:
                    centerline = _min_curvature_path(
                        centerline, np.array(left_xy)[:, :2], np.array(right_xy)[:, :2], safety=0.7)
                except Exception as opt_e:
                    self.get_logger().debug(f'Optimizer failed: {opt_e}')
        self._publish_plan(centerline, left_xy, right_xy)

    # ------------------------------------------------------------------ cone map

    def _merge_cones(self, msg: Track, insert_cap: Optional[int] = None) -> None:
        """Merge cones into the map on the 0.5 m grid. Colour only upgrades UNKNOWN cones.

        With insert_cap, the oldest entry is evicted before a new key would exceed the cap.
        colour_vote replaces the upgrade/latch rules with per-cell votes; cone_expiry then drops stale sparse cells.
        """
        self._merge_cycle += 1
        for cone in msg.track:
            if cone.color in (Cone.ORANGE_BIG, Cone.ORANGE_SMALL):
                continue
            # one non-finite cone (bad depth pixel upstream) must not kill the planner: round(NaN) raises
            if not (math.isfinite(cone.location.x) and math.isfinite(cone.location.y)):
                continue
            rx = round(cone.location.x / _GRID) * _GRID
            ry = round(cone.location.y / _GRID) * _GRID
            key = (rx, ry)
            color = cone.color
            if self._flip_prob > 0.0 and color in (Cone.BLUE, Cone.YELLOW):
                self._flip_seen.add(key)
                if _debug_flip_u(self._flip_seed, key) < self._flip_prob:
                    self._flip_set.add(key)
                    color = Cone.YELLOW if color == Cone.BLUE else Cone.BLUE
            new_type = _MFE_TO_FSD.get(color, ConeTypes.UNKNOWN)
            if self._colour_vote:
                if key not in self._cone_map:
                    self._make_room(insert_cap)
                    self._cone_map[key] = ConeTypes.UNKNOWN   # no colour until the votes decide one
                    self._cell_votes[key] = [0, 0, 0]
                self._set_colour(key, _vote_update(
                    self._cell_votes[key], new_type, self._cone_map[key], ConeTypes.LEFT, ConeTypes.RIGHT,
                    ConeTypes.UNKNOWN, self._vote_min, self._vote_margin))
            elif key not in self._cone_map:
                self._make_room(insert_cap)
                self._cone_map[key] = new_type
            elif self._cone_map[key] == ConeTypes.UNKNOWN and new_type != ConeTypes.UNKNOWN:
                self._cone_map[key] = new_type
            elif _RECOLOUR and new_type != ConeTypes.UNKNOWN and self._cone_map[key] != new_type:
                self._set_colour(key, new_type)    # sources that refine colours over time (camera votes) overwrite earlier guesses
            if self._cone_expiry:
                self._note_obs(key)
        if self._cone_expiry:
            self._expire_cells()
        if self._flip_prob > 0.0 and self._merge_cycle % 50 == 0:
            self.get_logger().info(f'flip_stats: flipped={len(self._flip_set)} distinct={len(self._flip_seen)}')
        if (self._colour_vote or self._cone_expiry) and self._merge_cycle % 50 == 0:
            self.get_logger().info(
                f'colour_flips={self._colour_flips} cones_expired={self._cones_expired} '
                f'map={len(self._cone_map)}')

    def _make_room(self, insert_cap: Optional[int]) -> None:
        """Skidpad FIFO: evict the oldest entry before a new key would exceed insert_cap."""
        if insert_cap is not None and len(self._cone_map) >= insert_cap:
            oldest = next(iter(self._cone_map))
            self._cone_map.pop(oldest)
            self._forget_cell(oldest)

    def _forget_cell(self, key) -> None:
        self._cell_votes.pop(key, None)
        self._cell_obs.pop(key, None)

    def _set_colour(self, key, value) -> None:
        old = self._cone_map[key]
        if old != value and old != ConeTypes.UNKNOWN and value != ConeTypes.UNKNOWN:
            self._colour_flips += 1
        self._cone_map[key] = value

    def _note_obs(self, key) -> None:
        """Count a cell at most once per merge cycle (a duplicated cone in one message is one sighting)."""
        rec = self._cell_obs.get(key)
        if rec is None:
            rec = self._cell_obs[key] = [0, -1]
        if rec[1] != self._merge_cycle:
            rec[0] += 1
            rec[1] = self._merge_cycle

    def _expire_cells(self) -> None:
        """Drop cells observed fewer than expiry_min_obs times, unseen for expiry_cycles cycles, while inside the
        sensing range (the _bucket_cones visibility test): a cone behind or beyond the range is not evidence."""
        fwd_x, fwd_y = self._car_dir
        for key, (count, last) in list(self._cell_obs.items()):
            if key not in self._cone_map:
                self._forget_cell(key)
                continue
            dx = key[0] - self._car_pos[0]
            dy = key[1] - self._car_pos[1]
            in_range = _in_sense_range(dx, dy, fwd_x, fwd_y, _CONE_VIS_R, _CONE_BEHIND_M)
            if _expiry_due(count, last, self._merge_cycle, in_range, self._expiry_min_obs, self._expiry_cycles):
                del self._cone_map[key]
                self._forget_cell(key)
                self._cones_expired += 1

    def _evict_far_cones(self, cap: int) -> None:
        """Over the cap, drop the cones farthest from the car. (Evicting the oldest key instead churned the
        map every message once it held more than the cap, dropping cones next to the car.)"""
        if len(self._cone_map) > cap:
            keys = list(self._cone_map.keys())
            d2 = np.sum((np.array(keys) - self._car_pos) ** 2, axis=1)
            for i in np.argsort(d2)[cap:]:
                del self._cone_map[keys[i]]
                self._forget_cell(keys[i])

    def _cone_xy(self) -> np.ndarray:
        return np.array(list(self._cone_map.keys()), dtype=np.float64)

    def _lap_return_fire(self) -> None:
        """Lap-return trigger: exclude the map cells within lap_return_radius_m of the start pose from the tracer."""
        if self._global_planning or self._locked_path is not None:
            if not self._lap_skip_logged:
                self._lap_skip_logged = True
                self.get_logger().info('lap_return_reset skipped: lap-1 lock-in active (or global planning); '
                                       'the tracer is not used here')
            return
        keys = list(self._cone_map.keys())
        start = self._lap_reset.start
        new = lap_return_banned(keys, start, self._lap_radius)
        self._lap_banned |= new
        self.get_logger().info(
            f'lap_return_reset TRIGGERED #{self._lap_reset.fires} at car=({self._car_pos[0]:.1f},'
            f'{self._car_pos[1]:.1f}) lap_travel={self._lap_reset.lap_travel:.0f} m: excluded {len(new)} of '
            f'{len(keys)} mapped cells within {self._lap_radius:.0f} m of start ({start[0]:.1f},{start[1]:.1f}); '
            f'total excluded {len(self._lap_banned)}')

    def _lidar_cb(self, msg: PointCloud2) -> None:
        """Lap-return lift on each scan. The scan is in the velodyne frame (base_footprint + 0.3 m up, no horizontal
        offset), so a point maps to the map by the car pose and heading."""
        if not self._lap_banned or not self._odom_received:
            return
        xs, ys = _cloud_xy(msg)
        c, s = self._car_dir
        det = np.c_[self._car_pos[0] + c * xs - s * ys, self._car_pos[1] + s * xs + c * ys]
        self._lap_lift(det)

    def _lap_lift(self, det=None) -> None:
        """Accept banned cells again: re-observed by the current scan (det, map frame: within LAP_REOBS_TOL_M), or
        passed by the car (lap_passed). Cells not yet seen or passed stay banned."""
        hit = lap_passed(self._lap_banned, self._car_pos, self._car_dir)
        if det is not None:
            hit |= lap_reobserved(self._lap_banned, det)
        if hit:
            self._lap_banned -= hit
            self._lap_reobs += len(hit)
            self.get_logger().info(
                f'lap_return_reset: {len(hit)} banned cells accepted again (total {self._lap_reobs}); '
                f'still banned {len(self._lap_banned)}', throttle_duration_sec=1.0)

    def _tracer_cone_xy(self) -> np.ndarray:
        """Map cells handed to the boundary tracer: the cone map minus the lap-return exclusions."""
        if not self._lap_banned:
            return self._cone_xy()
        keys = [k for k in self._cone_map if k not in self._lap_banned]
        return np.array(keys, dtype=np.float64).reshape(-1, 2)

    def _map_walls(self):
        """Blue (left) and yellow (right) cone positions in the map."""
        blue = np.array([k for k, v in self._cone_map.items() if v == ConeTypes.LEFT])
        yellow = np.array([k for k, v in self._cone_map.items() if v == ConeTypes.RIGHT])
        return blue, yellow

    def _bucket_cones(self, visible_only: bool):
        """Map cones grouped by ConeTypes. visible_only keeps those within _CONE_VIS_R and not behind the car."""
        buckets = [[] for _ in range(len(ConeTypes))]
        for (rx, ry), cone_type in self._cone_map.items():
            if visible_only:
                dx = rx - self._car_pos[0]
                dy = ry - self._car_pos[1]
                if dx * dx + dy * dy > _CONE_VIS_R * _CONE_VIS_R:
                    continue
                if dx * self._car_dir[0] + dy * self._car_dir[1] < -_CONE_BEHIND_M:
                    continue
            buckets[cone_type].append([rx, ry])
        return buckets

    def _update_pipeline_mode(self, n_left: int, n_right: int) -> None:
        """pipeline_hysteresis latch. Trace -> library once both walls hold min_coloured_each coloured cones;
        library -> trace after revert_after consecutive cycles with no coloured cone at all."""
        self._pipeline_cycles += 1
        self._coloured_absent = self._coloured_absent + 1 if n_left + n_right == 0 else 0
        if self._pipeline_trace:
            if n_left >= self._min_coloured_each and n_right >= self._min_coloured_each:
                self._set_pipeline_trace(False, n_left, n_right)
        elif self._coloured_absent >= self._revert_after:
            self._set_pipeline_trace(True, n_left, n_right)
        self.get_logger().debug(
            f'pipeline mode={"trace" if self._pipeline_trace else "library"} '
            f'left={n_left} right={n_right} absent={self._coloured_absent}')
        if self._pipeline_cycles % 50 == 0:
            self.get_logger().info(f'pipeline_switches={self._pipeline_switches}')

    def _set_pipeline_trace(self, trace: bool, n_left: int, n_right: int) -> None:
        self._pipeline_trace = trace
        self._pipeline_switches += 1
        self._coloured_absent = 0
        self.get_logger().info(
            f'pipeline: {"library->trace" if trace else "trace->library"} '
            f'(left={n_left} right={n_right})', throttle_duration_sec=1.0)

    def _assign_unknown_sides(self, buckets) -> None:
        """LiDAR-only: sort UNKNOWN cones into LEFT/RIGHT by the sign of their cross product with the heading."""
        for (rx, ry), cone_type in self._cone_map.items():
            if cone_type != ConeTypes.UNKNOWN:
                continue
            dx = rx - self._car_pos[0]
            dy = ry - self._car_pos[1]
            fwd = dx * self._car_dir[0] + dy * self._car_dir[1]
            if fwd < -_CONE_BEHIND_M:
                continue
            cross_z = self._car_dir[0] * dy - self._car_dir[1] * dx
            buckets[ConeTypes.LEFT if cross_z > 0 else ConeTypes.RIGHT].append([rx, ry])
        buckets[ConeTypes.UNKNOWN] = []

    # ------------------------------------------------------------------ skidpad

    def _update_skidpad_yaw_rate_sign(self) -> None:
        """Sign of the sustained yaw rate over the last few frames (filters noise)."""
        if self._mission != 'skidpad' or self._skidpad_mode != 'cone_based':
            return
        hist = self._skidpad_yaw_rate_history
        hist.append(self._car_yaw_rate)
        if len(hist) > _YAW_HIST_LEN:
            hist.pop(0)
        avg_yaw_rate = float(np.mean(hist))
        if abs(avg_yaw_rate) > _YAW_RATE_THRESH:
            new_sign = 1 if avg_yaw_rate > 0 else -1
            if self._skidpad_sustained_yaw_rate_sign != new_sign:
                self._skidpad_sustained_yaw_rate_sign = new_sign
                circle_dir = 'LEFT (CCW)' if new_sign > 0 else 'RIGHT (CW)'
                self.get_logger().debug(f'Skidpad: detected sustained turn {circle_dir}')

    def _advance_circle(self, circle: Optional[_Circle]) -> None:
        if circle is None:
            return
        step = extend_trail(circle.trail, self._car_pos)
        if step is not None:
            circle.dist += step
            circle.max_r = max(circle.max_r, float(np.linalg.norm(self._car_pos - circle.start)))

    def _lap_closed(self, circle: _Circle) -> bool:
        return (circle.dist > _CIRCLE_LAP_DIST and circle.max_r > _CIRCLE_MAX_R
                and float(np.linalg.norm(self._car_pos - circle.start)) < _CIRCLE_CLOSE_R)

    def _enter_circle(self, left: bool) -> None:
        circle = _Circle(start=self._car_pos.copy(), trail=[self._car_pos.copy()])
        if left:
            self._skidpad_left = circle
            self._skidpad_state = 'LEFT_CIRCLE'
        else:
            self._skidpad_right = circle
            self._skidpad_state = 'RIGHT_CIRCLE'

    def _step_skidpad_state(self) -> None:
        sign = self._skidpad_sustained_yaw_rate_sign
        if self._skidpad_state == 'APPROACH':
            if sign is not None:
                left = sign > 0
                self.get_logger().info(f'Skidpad: entering {"LEFT" if left else "RIGHT"}_CIRCLE')
                self._enter_circle(left)

        elif self._skidpad_state == 'LEFT_CIRCLE':
            circle = self._skidpad_left
            if self._lap_closed(circle):
                if circle.locked is None:
                    self._lock_circle(circle, 'LEFT')
                elif circle.laps == 1:
                    circle.laps = 2
                    self.get_logger().info('Skidpad LEFT lap 2 complete — transitioning to RIGHT_CIRCLE')
                    if sign is not None and sign < 0:
                        self._enter_circle(left=False)
                        self._skidpad_yaw_rate_history.clear()
                    else:
                        self.get_logger().warn('Skidpad: expected right turn to enter RIGHT_CIRCLE, still turning left')

        elif self._skidpad_state == 'RIGHT_CIRCLE':
            circle = self._skidpad_right
            if self._lap_closed(circle):
                if circle.locked is None:
                    self._lock_circle(circle, 'RIGHT')
                elif circle.laps == 1:
                    circle.laps = 2
                    self.get_logger().info('Skidpad RIGHT lap 2 complete — transitioning to EXIT')
                    self._skidpad_state = 'EXIT'
                    self._skidpad_yaw_rate_history.clear()

    def _lock_circle(self, circle: _Circle, name: str) -> None:
        locked = self._build_skidpad_circle(circle.trail, circle.start)
        if locked is None:
            return
        circle.locked = locked
        circle.laps = 1
        self.get_logger().info(
            f'Skidpad {name} lap 1 complete — locked circle: {len(locked)} waypoints, '
            f'{arc_length(locked):.1f} m')
        circle.trail = [self._car_pos.copy()]
        circle.dist = 0.0
        circle.max_r = 0.0

    def _build_skidpad_circle(self, trail: list, trail_start) -> Optional[np.ndarray]:
        """Closed (first ~ last) path for one skidpad circle, from the trail and the map walls."""
        if trail_start is None:
            return None
        blue, yellow = self._map_walls()
        if len(blue) < 4 or len(yellow) < 4 or len(trail) < 15:
            return None
        centerline = midpoint_walk(np.array(trail), blue, yellow)
        if len(centerline) < 8:
            return None
        return self._finish_loop(centerline, blue, yellow, min_n=15, label='Skidpad circle')

    def _finish_loop(self, centerline, blue, yellow, min_n: int, label: str) -> np.ndarray:
        """Smooth a midpoint loop, optimise it inside the walls, revert waypoints near cones, resample."""
        centerline = smooth_cyclic(centerline)
        raw = centerline.copy()
        try:
            centerline = _min_curvature_path(centerline, blue, yellow, safety=0.5)
        except Exception as e:
            self.get_logger().warn(
                f'{label} optimizer failed ({type(e).__name__}: {e}) — using raw centerline.')
        reverted = clearance_guard(centerline, raw, np.vstack([blue, yellow]))
        if reverted:
            self.get_logger().info(f'{label} clearance guard reverted {reverted} waypoints.')
        return closed_resample(centerline, min_n)

    # ------------------------------------------------------------------ global lock-in

    def _build_locked_loop(self) -> Optional[np.ndarray]:
        """Full-lap closed centerline from the cone map, ordered by the lap-1 trail (handles figure-8s)."""
        blue, yellow = self._map_walls()
        if len(blue) < 4 or len(yellow) < 4 or len(self._trail) < 20:
            self.get_logger().warn(
                f'Loop closed but map too thin to lock '
                f'(blue={len(blue)} yellow={len(yellow)} trail={len(self._trail)})')
            return None
        centerline = midpoint_walk(np.array(self._trail), blue, yellow)
        if len(centerline) < 10:
            return None
        return self._finish_loop(centerline, blue, yellow, min_n=20, label='Lock-in')

    # ------------------------------------------------------------------ planning helpers

    def _off_map(self, xy: np.ndarray) -> bool:
        """True (and warns) when the car is far from every mapped cone: publish nothing, so pure
        pursuit's stale-path failsafe brakes."""
        d_nearest = float(np.min(np.hypot(xy[:, 0] - self._car_pos[0], xy[:, 1] - self._car_pos[1])))
        if d_nearest > _OFF_MAP_M:
            self.get_logger().warn(
                f'Car is {d_nearest:.0f} m from the nearest mapped cone — off track? '
                'Skipping planner so the stale-path failsafe stops the car.',
                throttle_duration_sec=2.0)
            return True
        return False

    def _run_library(self, cones_by_type, n_left, n_right, n_unknown):
        """ft-fsd call. Returns (path, left, right) or None when skipped or failed."""
        if self._off_map(self._cone_xy()):
            return None
        try:
            out = self._planner.calculate_path_in_global_frame(
                cones_by_type, self._car_pos, self._car_dir,
                return_intermediate_results=True,
            )
        except Exception as e:
            self._last_lib_error = f'{type(e).__name__}: {e}'
            self.get_logger().error(
                f'Path calculation failed ({type(e).__name__}): {e} | '
                f'left={n_left} right={n_right} unknown={n_unknown}',
                throttle_duration_sec=2.0)
            return None
        path_xy, left_xy, right_xy = out[0], out[1], out[2]
        if self._dump_on:
            try:
                self._dump_intermediate(cones_by_type, self._car_pos.copy(), self._car_dir.copy(), out)
            except Exception as de:  # the dump must never change planner behaviour
                self.get_logger().warn(f'debug dump failed: {de}', throttle_duration_sec=5.0)
        return path_xy, left_xy, right_xy

    def _dump_intermediate(self, cones_by_type, car_pos, car_dir, out) -> None:
        """debug_dump_intermediate: one JSON line per planning cycle. out = library return tuple:
        (final_path Nx4, sorted_left, sorted_right, left_with_virtual, right_with_virtual, l2r_idx, r2l_idx)."""
        def xy(a):
            a = np.asarray(a, dtype=float).reshape(-1, 2)
            return [[round(float(x), 2), round(float(y), 2)] for x, y in a]
        inputs, parts = {}, []
        for i, a in enumerate(cones_by_type):
            a = np.asarray(a, dtype=float).reshape(-1, 2)
            inputs[ConeTypes(i).name] = {'n': len(a), 'xy': xy(a)}
            parts.append(a)
        all_in = np.vstack(parts) if parts else np.zeros((0, 2))

        def virtual(arr):  # matched cones that are not within 0.3 m of any input cone
            arr = np.asarray(arr, dtype=float).reshape(-1, 2)
            if len(arr) == 0 or len(all_in) == 0:
                return arr
            d = np.hypot(arr[:, None, 0] - all_in[None, :, 0], arr[:, None, 1] - all_in[None, :, 1]).min(axis=1)
            return arr[d > 0.3]

        path = np.asarray(out[0], dtype=float)
        rec = {
            't_ros': round(self.get_clock().now().nanoseconds * 1e-9, 3),
            't_wall': round(time.time(), 3),
            'car': xy([car_pos])[0],
            'car_dir': [round(float(car_dir[0]), 3), round(float(car_dir[1]), 3)],
            'inputs': inputs,
            'sorted_left': xy(out[1]),
            'sorted_right': xy(out[2]),
            'matched_left_n': int(len(out[3])),
            'matched_right_n': int(len(out[4])),
            'virtual_left': xy(virtual(out[3])),
            'virtual_right': xy(virtual(out[4])),
            'path_n': int(len(path)),
            'path_xy': xy(path[:40, 1:3]) if path.ndim == 2 and len(path) else [],
        }
        self._dump_fh.write(json.dumps(rec, separators=(',', ':')) + '\n')

    def _racing_line(self, centerline, left, right):
        """Race line through the corridor: PSO (if enabled) with the curvature minimiser as fallback,
        else the curvature minimiser alone; the raw centerline if both fail. Shared by all backends."""
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

    def _publish_centerline(self, xy: np.ndarray) -> None:
        stamp = self.get_clock().now().to_msg()
        self._pub_centerline.publish(path_from_xy(xy, self._map_frame, stamp))

    @staticmethod
    def _path_point_at(xy: np.ndarray, s: float) -> np.ndarray:
        """Point at arc length s along the polyline xy (clamped to its end)."""
        cum = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1])))])
        return np.array([np.interp(s, cum, xy[:, 0]), np.interp(s, cum, xy[:, 1])])

    @staticmethod
    def _dist_to_polyline(pt: np.ndarray, xy: np.ndarray) -> float:
        a, ab = xy[:-1], np.diff(xy, axis=0)
        t = np.clip(np.sum((pt - a) * ab, axis=1) / np.maximum(np.sum(ab * ab, axis=1), 1e-12), 0.0, 1.0)
        return float(np.min(np.linalg.norm(pt - (a + t[:, None] * ab), axis=1)))

    def _jump_gate(self, xy: np.ndarray) -> np.ndarray:
        """Return the centreline to publish: xy, or the last published path when xy jumps (see jump_gate)."""
        self._jump_last = 'skip'
        if not self._jump_gate_on or self._jump_prev is None or len(xy) < 2 or len(self._jump_prev) < 2:
            self._jump_prev = xy
            self._jump_hold = 0
            return xy

        prev = self._jump_prev
        # Short paths: the 5 m / 10 m probes clamp to the path end, so the heading and lateral test is meaningless.
        if _arc_length(xy) < self._jump_min_path_m or _arc_length(prev) < self._jump_min_path_m:
            self._jump_prev = xy
            self._jump_hold = 0
            return xy
        h_new = math.atan2(*(self._path_point_at(xy, _JUMP_HEAD_M) - xy[0])[::-1])
        h_prev = math.atan2(*(self._path_point_at(prev, _JUMP_HEAD_M) - prev[0])[::-1])
        dh = math.degrees(abs((h_new - h_prev + math.pi) % (2 * math.pi) - math.pi))
        lat = [self._dist_to_polyline(self._path_point_at(xy, s), prev) for s in _JUMP_LAT_M]
        bad = dh > self._jump_heading_deg or max(lat) > self._jump_lateral_m
        self._jump_cycles += 1
        # Never hold a stale path that has less road ahead of the car than the fresh candidate: that would trade
        # a long path for a shorter one.
        hold_ok = self._jump_hold < self._jump_hold_max
        if bad and hold_ok and self._jump_remaining_guard and self._odom_received:
            if _remaining_ahead(prev, self._car_pos) < _remaining_ahead(xy, self._car_pos):
                hold_ok = False

        if bad and hold_ok:
            self._jump_last = 'hold'
            self._jump_hold += 1
            self._jump_rejects += 1
            self.get_logger().warn(
                f'jump gate: rejected centreline (dheading={dh:.1f} deg, lat5={lat[0]:.2f} m, lat10={lat[1]:.2f} m; '
                f'hold {self._jump_hold}/{self._jump_hold_max}) — republishing previous path',
                throttle_duration_sec=_JUMP_WARN_S)
            xy = prev
        else:
            if bad:
                self._jump_accepted_after_hold += 1
                self.get_logger().warn(
                    f'jump gate: accepting jump after {self._jump_hold} held cycles '
                    f'(dheading={dh:.1f} deg, lat5={lat[0]:.2f} m, lat10={lat[1]:.2f} m)',
                    throttle_duration_sec=_JUMP_WARN_S)
            self._jump_last = 'accept_after_hold' if bad else 'ok'
            self._jump_hold = 0
            self._jump_prev = xy

        if self._jump_cycles % _JUMP_LOG_EVERY == 0:
            self.get_logger().info(
                f'jump_rejects={self._jump_rejects} accepted_after_hold={self._jump_accepted_after_hold} '
                f'cycles={self._jump_cycles}')
        return xy

    def _publish_plan(self, centerline, left, right) -> None:
        """Publish centerline (skipped when None), then left and right walls (when non-empty)."""
        stamp = self.get_clock().now().to_msg()
        if centerline is not None:
            centerline = self._jump_gate(np.asarray(centerline))
            self._pub_centerline.publish(path_from_xy(centerline, self._map_frame, stamp))
        if left is not None and len(left) > 0:
            self._pub_left.publish(path_from_xy(np.array(left)[:, :2], self._map_frame, stamp))
        if right is not None and len(right) > 0:
            self._pub_right.publish(path_from_xy(np.array(right)[:, :2], self._map_frame, stamp))

    # ------------------------------------------------------------------ offline race line (raceline_enable)

    def _rl_collect_odom(self) -> None:
        """Lap-1 trail: from the first odometry point more than _RL_MOVE_M from the first pose, until the build."""
        pos = self._car_pos
        if self._rl_p0 is None:
            self._rl_p0 = pos.copy()
        if self._rl_trail or float(np.hypot(*(pos - self._rl_p0))) > _RL_MOVE_M:
            self._rl_trail.append((float(pos[0]), float(pos[1])))

    def _rl_cycle(self, msg: Track) -> bool:
        """Race-line cycle. True when the race-line window was published, so the tracer is skipped this cycle."""
        if self._rl_state == 'collect':
            if self._laps_done < self._rl_after:
                if self._rl_trail:   # union of the cones seen since first movement (lap 1)
                    xy = np.array([(c.location.x, c.location.y) for c in msg.track], float).reshape(-1, 2)
                    col = np.array([c.color for c in msg.track], int)
                    ok = np.isfinite(xy).all(axis=1)
                    self._rl_union.add(xy[ok], col[ok])
                return False
            self._rl_build()
        if self._rl_state != 'built':
            return False
        return self._rl_follow()

    def _rl_build(self) -> None:
        """Build the race line once, from the union and trail frozen at laps_completed = raceline_after_lap."""
        n_cones, n_trail = self._rl_union.n, len(self._rl_trail)
        params = _raceline.RaceLineParams(extra_margin_m=self._rl_margin, a_lat=self._rl_a_lat, v_max=self._rl_v_max)
        t0 = time.perf_counter()
        rl, err = None, None
        try:
            rl = _raceline.build(self._rl_union.xy, np.array(self._rl_trail, float), params,
                                 colours=self._rl_union.colours)
        except Exception as e:  # noqa: BLE001 - a failed build must not stop the tracer
            err = f'{type(e).__name__}: {e}'
        self._rl_build_s = time.perf_counter() - t0
        self._rl_union = None
        self._rl_trail = []
        if rl is None:
            self._rl_state = 'failed'
            self.get_logger().warn(
                f'race line: build failed ({err or "degenerate input"}) after {self._rl_build_s:.2f} s; '
                f'{n_cones} union cones, {n_trail} trail points. Tracer path for the rest of the run.')
            return
        self._rl = rl
        self._rl_state = 'built'
        self.get_logger().info(
            f'RACE LINE built at laps_completed={self._laps_done}: length {rl.length_m:.1f} m, '
            f'predicted lap {rl.lap_s:.2f} s, min side width {rl.min_side_width_m:.2f} m, '
            f'build {self._rl_build_s:.2f} s ({n_cones} union cones, {n_trail} trail points, '
            f'a_lat {self._rl_a_lat:g}, margin +{self._rl_margin:g} m)')

    def _rl_follow(self) -> bool:
        """Publish the race-line window of raceline_window_m from the car's projection, with target speeds.
        False (tracer this cycle, logged) when the car is more than _RL_MAX_OFF_M from the line."""
        rl = self._rl
        idx, dist = _raceline.project(rl, self._car_pos, self._rl_idx)
        if dist > _RL_MAX_OFF_M:
            self._rl_fallbacks += 1
            self.get_logger().warn(
                f'race line: car {dist:.2f} m from the line, tracer path this cycle (fallbacks={self._rl_fallbacks})',
                throttle_duration_sec=2.0)
            return False
        self._rl_idx = idx
        ii = _raceline.segment_indices(rl, idx, self._rl_window_m)
        stamp = self.get_clock().now().to_msg()
        self._pub_centerline.publish(path_from_xy(rl.xy[ii], self._map_frame, stamp))
        speeds = Float64MultiArray()
        speeds.data = [float(v) for v in rl.v[ii]]
        self._pub_speeds.publish(speeds)
        self._rl_cycles += 1
        if self._rl_cycles % 100 == 0:
            self.get_logger().info(
                f'race line: {self._rl_cycles} windows published, {self._rl_fallbacks} tracer fallbacks')
        return True

    # ------------------------------------------------------------------ colourless / AMZ

    def _plan_colourless(self, colour_hint: bool = False) -> None:
        """No cone colours: boundary_tracer labels each wall from the driven trajectory and grows both walls
        ahead together. The path ends where the paired walls end. colour_hint: the traced walls keep the tracer's
        sides unless the cone colours vote otherwise (see _colour_hint_sides)."""
        xy = self._tracer_cone_xy()
        if len(xy) == 0 or self._off_map(xy):
            self._log_cycle(len(xy), 'no cones' if len(xy) == 0 else 'off map', None)
            return
        res = self._tracer.plan(xy, self._car_pos, self._car_dir)
        if res is None:
            self._log_cycle(len(xy), self._tracer.debug.get('reason'), self._tracer.debug)
            self.get_logger().warn(f'boundary tracer: no path ({self._tracer.debug.get("reason")})',
                                   throttle_duration_sec=2.0)
            return
        path, left, right = res.path, res.left, res.right
        self._hint_swapped = False
        if colour_hint:
            left, right = self._colour_hint_sides(left, right)
        self._record_side_votes(xy, res.labels, self._hint_swapped)
        if self._unknown_mode == 'trace_backend':
            path = self._backend_on_labels(xy, res)
            if path is None:
                return
        optimized = path
        if (self._colourless_raceline and not self._debug_disable_raceline
                and len(path) > 3 and len(left) > 3 and len(right) > 3):
            # the optimiser sizes its corridor from the nearest wall cone, which past the traced walls is the wrong cone
            optimized = clamp_to_corridor(self._racing_line(path, left, right), path, left, right)
        horizon = float(np.sum(np.linalg.norm(np.diff(optimized, axis=0), axis=1))) if len(optimized) > 1 else 0.0
        self.get_logger().info(
            f'boundary tracer: left={len(left)} right={len(right)} gates={res.debug.get("n_gates")} '
            f'labelled={res.debug.get("n_labelled")} horizon={horizon:.1f} m',
            throttle_duration_sec=2.0)
        self._publish_plan(optimized, left, right)
        self._pub_horizon.publish(Float32(data=horizon))
        self._log_cycle(len(xy), 'ok', self._tracer.debug, horizon=horizon)

    def _record_side_votes(self, xy, labels, swapped: bool) -> None:
        """Colour laps: remember which side the tracer put each map cell on (+1 left, -1 right). labels is aligned
        with xy; swapped means _colour_hint_sides exchanged the walls, so the published left is the traced right."""
        sign = -1 if swapped else 1
        for (x, y), lab in zip(xy, labels):
            if lab == 0:
                continue
            key = (float(x), float(y))
            self._side_votes[key] = self._side_votes.get(key, 0) + sign * (1 if lab > 0 else -1)

    def _laps_cb(self, msg: Int32) -> None:
        self._laps_done = int(msg.data)

    def _fill_sides_from_votes(self, buckets):
        """Colour laps: UNKNOWN cones take the side the lap-1 tracer gave their cell (vote > 0 LEFT, < 0 RIGHT).
        Camera colours are never changed. Returns (camera-coloured count, filled count, unfilled count)."""
        n_cam = len(buckets[ConeTypes.LEFT]) + len(buckets[ConeTypes.RIGHT])
        keep = []
        n_fill = 0
        for rx, ry in buckets[ConeTypes.UNKNOWN]:
            v = self._side_votes.get((rx, ry), 0)
            if v > 0:
                buckets[ConeTypes.LEFT].append([rx, ry])
                n_fill += 1
            elif v < 0:
                buckets[ConeTypes.RIGHT].append([rx, ry])
                n_fill += 1
            else:
                keep.append([rx, ry])
        buckets[ConeTypes.UNKNOWN] = keep
        return n_cam, n_fill, len(keep)

    def _plan_colour_lap(self, buckets) -> None:
        """Colour laps: plan with the camera's BLUE/YELLOW cones (UNKNOWN ones side-filled from the lap-1 votes)
        through the library backend. Falls back to the colour_hint tracer for the cycle when the library fails."""
        n_cam, n_fill, n_unk = self._fill_sides_from_votes(buckets)
        n_left, n_right = len(buckets[ConeTypes.LEFT]), len(buckets[ConeTypes.RIGHT])
        n_total = n_left + n_right + n_unk
        reason = None
        result = None
        if n_left + n_right == 0:
            reason = 'no blue/yellow cones after side fill'
        else:
            self._last_lib_error = None
            try:
                result = self._run_library(cone_arrays(buckets), n_left, n_right, n_unk)
            except Exception as e:  # noqa: BLE001 - a library failure must fall back, not stop planning
                result = None
                reason = f'{type(e).__name__}: {e}'
            if reason is None and result is None:
                reason = self._last_lib_error or 'library returned None'
            elif reason is None and (result[0] is None or len(result[0]) == 0):
                reason = 'empty path'
        if reason is not None:
            self._colour_fallbacks += 1
            self.get_logger().warn(
                f'colour lap: library fallback to colour_hint tracer ({reason}); fallbacks={self._colour_fallbacks}',
                throttle_duration_sec=5.0)
            self._log_cycle(n_total, f'fallback: {reason}', None, mode='colour_lib')
            self._plan_colourless(colour_hint=True)
            return
        path_xy, left_xy, right_xy = result
        centerline = np.array(path_xy)[:, 1:3]
        if (self._colour_lap_raceline and not self._global_planning and not self._debug_disable_raceline
                and len(path_xy) > 3 and left_xy is not None and len(left_xy) > 3
                and right_xy is not None and len(right_xy) > 3):
            centerline = self._racing_line(centerline, np.array(left_xy)[:, :2], np.array(right_xy)[:, :2])
        horizon = float(np.sum(np.linalg.norm(np.diff(centerline, axis=0), axis=1))) if len(centerline) > 1 else 0.0
        self._publish_plan(centerline, left_xy, right_xy)
        self._colour_cycles += 1
        self._colour_cam_sum += n_cam
        self._colour_fill_sum += n_fill
        self._log_cycle(n_total, 'ok', None, horizon=horizon, mode='colour_lib')
        if self._colour_cycles % 50 == 0:
            self.get_logger().info(
                f'colour laps: colour_cycles={self._colour_cycles} fallbacks={self._colour_fallbacks} '
                f'mean_camera_coloured={self._colour_cam_sum / self._colour_cycles:.1f} '
                f'mean_side_filled={self._colour_fill_sum / self._colour_cycles:.1f} laps_done={self._laps_done}')

    def _log_cycle(self, n_cones, reason, dbg, horizon=None, mode=None) -> None:
        """planner_debug_log: one JSON line per colourless cycle (car pose, cones, tracer reason, gates, path,
        jump-gate decision, lap-return reset state). Never raises into the planning loop."""
        if self._dbg_fh is None:
            return
        try:
            dbg = dbg or {}
            rec = dict(
                t=round(self.get_clock().now().nanoseconds * 1e-9, 3),
                car=[round(float(self._car_pos[0]), 2), round(float(self._car_pos[1]), 2)],
                yaw=round(math.degrees(math.atan2(float(self._car_dir[1]), float(self._car_dir[0]))), 1),
                n_cones=int(n_cones), reason=reason,
                n_gates=dbg.get('n_gates'),
                path_m=None if horizon is None else round(float(horizon), 2),
                jump=self._jump_last,
                reset=[self._lap_reset.fires if self._lap_reset is not None else None, len(self._lap_banned)])
            if mode is not None:
                rec['mode'] = mode
                rec['laps'] = self._laps_done
            self._dbg_fh.write(json.dumps(rec) + '\n')
        except Exception as e:  # noqa: BLE001 - logging must not break planning
            self.get_logger().warn(f'planner_debug_log write failed: {e}', throttle_duration_sec=5.0)

    def _wall_colour_vote(self, wall) -> int:
        """+1 when the wall's coloured cones are mostly BLUE, -1 when mostly YELLOW, 0 without a confident vote
        (needs >= 2 coloured cones and a lead of >= 1)."""
        blue = sum(1 for x, y in wall if self._cone_map.get((float(x), float(y))) == ConeTypes.LEFT)
        yellow = sum(1 for x, y in wall if self._cone_map.get((float(x), float(y))) == ConeTypes.RIGHT)
        if blue + yellow < 2 or blue == yellow:
            return 0
        return 1 if blue > yellow else -1

    def _colour_hint_sides(self, left, right):
        """colour_hint: keep the tracer's geometric left/right unless both walls vote confidently and oppositely
        (one blue-majority, one yellow-majority) against it. Returns the (left, right) walls to publish."""
        self._hint_cycles += 1
        self._hint_swapped = False
        v_left, v_right = self._wall_colour_vote(left), self._wall_colour_vote(right)
        if v_left != 0 and v_right != 0 and v_left != v_right:
            if v_left == 1:
                self._hint_counts['agree'] += 1
            else:
                self._hint_counts['override'] += 1
                self._hint_swapped = True
                self.get_logger().info(
                    f'colour hint overrides geometric sides: traced left wall ({len(left)} cones) votes YELLOW, '
                    f'traced right wall ({len(right)} cones) votes BLUE — swapping walls',
                    throttle_duration_sec=5.0)
                left, right = right, left
        else:
            self._hint_counts['fallback'] += 1
        if self._hint_cycles % 50 == 0:
            self.get_logger().info(
                f'colour hint: hint_overrides={self._hint_counts["override"]} '
                f'hint_agree={self._hint_counts["agree"]} hint_fallback={self._hint_counts["fallback"]}')
        return left, right

    def _backend_on_labels(self, xy, res):
        """Run the configured backend on the tracer's left/right labels, then cut its path where the traced
        walls end (backends extrapolate past the last cones)."""
        rel = xy - self._car_pos
        ahead = rel @ self._car_dir >= -3.0
        Lm = ahead & (res.labels > 0)
        Rm = ahead & (res.labels < 0)
        cones = [np.zeros((0, 2)) for _ in range(len(ConeTypes))]
        cones[ConeTypes.LEFT] = xy[Lm]
        cones[ConeTypes.RIGHT] = xy[Rm]
        try:
            path = np.array(self._planner.calculate_path_in_global_frame(
                cones, self._car_pos, self._car_dir))[:, 1:3]
        except Exception as e:
            self.get_logger().error(f'Path calculation failed: {type(e).__name__}: {e}',
                                    throttle_duration_sec=2.0)
            return None
        if len(path) < 2:
            return None
        end = res.path[-1]
        j = int(np.argmin(np.linalg.norm(path - end, axis=1)))
        return path[:j + 1] if j >= 1 else None


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
