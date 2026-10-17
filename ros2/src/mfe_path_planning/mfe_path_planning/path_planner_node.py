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

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32
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
_MAP_CAP = 1000            # cone map size cap (local planning / global missions)
_SKIDPAD_MAP_CAP = 500
_CONE_VIS_R = 50.0         # local planning visibility radius (m)
_CONE_BEHIND_M = 3.0       # tolerance behind the car for the visibility filter
_OFF_MAP_M = 20.0          # car this far from every mapped cone: off track, publish nothing
_CIRCLE_LAP_DIST = 40.0    # skidpad lap closure: min distance driven (m) ...
_CIRCLE_MAX_R = 10.0       # ... min excursion from the circle start (m) ...
_CIRCLE_CLOSE_R = 4.0      # ... and back within this of the start
_YAW_HIST_LEN = 5
_YAW_RATE_THRESH = 0.1     # rad/s sustained turn threshold


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
        # Local (non-global) backend: fasttube = fsd_path_planning, amz = Delaunay + beam search (amz_planner.py).
        self.declare_parameter('planner_backend', 'fasttube')  # fasttube | amz
        # Uncoloured cones (LiDAR-only), local missions:
        #   trace         — boundary_tracer labels the walls; its gate-midpoint centreline is used directly
        #   trace_backend — same wall labels fed to the backend as left/right
        #   heading       — legacy: side of the car's heading (fails in corners)
        #   fasttube      — uncoloured cones go to ft-fsd unchanged; its own sorting picks the sides
        self.declare_parameter('unknown_cone_mode', 'trace')
        # Racing-line optimiser on colourless (traced) paths. Off by default: traced paths are short and the
        # optimiser's ~0.27 s per cycle made the path lag the car.
        self.declare_parameter('colourless_raceline', False)

        self._map_frame = self.get_parameter('map_frame').value
        mission_str = self.get_parameter('mission').value
        self._use_pso_optimizer = self.get_parameter('use_pso_optimizer').value
        if self._use_pso_optimizer:  # compile the PSO hot loops now, not on the first planning cycle
            _w = np.c_[np.linspace(0.0, 3.0, 4), np.zeros(4)]
            _backward_pass(_forward_pass(_menger_kappa(_w), np.ones(3)), np.ones(3))
        self._mission = mission_str
        self._skidpad_mode = self.get_parameter('skidpad_mode').value
        self._backend = self.get_parameter('planner_backend').value
        self._amz = AmzPlanner() if self._backend == 'amz' else None
        self._unknown_mode = self.get_parameter('unknown_cone_mode').value
        self._colourless_raceline = bool(self.get_parameter('colourless_raceline').value)
        self._tracer = BoundaryTracer()

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
        self._skidpad_path_published = False
        self._skidpad_path = None
        self._reset_skidpad_state()

        # Lap-1 lock-in (global missions): record the driven trail; on closing the loop, build the full-lap
        # centerline from the cone map ordered by the trail, optimise it once and lock it.
        self._trail: list = []
        self._trail_dist = 0.0
        self._trail_max_r = 0.0
        self._locked_path = None

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

        # ---------- Subscribers / publishers ----------
        self.create_subscription(Track, '/planning/cones', self._cones_cb, _QOS_LATEST)
        self.create_subscription(Odometry, '/ekf/output', self._odom_cb, _QOS)
        self._pub_centerline = self.create_publisher(Path, '/planning/centerline', _QOS)
        self._pub_left = self.create_publisher(Path, '/planning/track_left', _QOS)
        self._pub_right = self.create_publisher(Path, '/planning/track_right', _QOS)
        self._pub_horizon = self.create_publisher(Float32, '/planning/path_horizon', _QOS)

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

        if self._global_planning and self._locked_path is None:
            step = extend_trail(self._trail, self._car_pos)
            if step is not None:
                self._trail_dist += step
                self._trail_max_r = max(
                    self._trail_max_r, float(np.linalg.norm(self._car_pos - self._trail[0])))

    def _cones_cb(self, msg: Track) -> None:
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

        # Acceleration also gets the full map: ft-fsd's AccelerationRelocalizer needs >= 4 cones on the
        # car's left, and the visibility-filtered set never held more than 3 (replayed bags: 0.3-1.4 m off).
        use_full_map = self._global_planning or self._mission == 'acceleration'
        buckets = self._bucket_cones(visible_only=not use_full_map)
        cones_by_type = cone_arrays(buckets)
        n_left, n_right, n_unknown = _counts(buckets)

        # Colourless local planning: trace the walls instead of guessing sides from the heading.
        if (not use_full_map and n_left + n_right == 0 and n_unknown > 0
                and self._unknown_mode in ('trace', 'trace_backend')):
            self._plan_colourless()
            return

        if self._amz is not None and not self._global_planning:
            self._plan_amz()
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
                    and (self._colourless_raceline or not colourless_native)
                    and len(path_xy) > 3
                    and left_xy is not None and len(left_xy) > 3
                    and right_xy is not None and len(right_xy) > 3):
                centerline = self._racing_line(centerline, np.array(left_xy)[:, :2], np.array(right_xy)[:, :2])
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
            if (len(path_xy) > 3
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
        """
        for cone in msg.track:
            if cone.color in (Cone.ORANGE_BIG, Cone.ORANGE_SMALL):
                continue
            # one non-finite cone (bad depth pixel upstream) must not kill the planner: round(NaN) raises
            if not (math.isfinite(cone.location.x) and math.isfinite(cone.location.y)):
                continue
            rx = round(cone.location.x / _GRID) * _GRID
            ry = round(cone.location.y / _GRID) * _GRID
            key = (rx, ry)
            new_type = _MFE_TO_FSD.get(cone.color, ConeTypes.UNKNOWN)
            if key not in self._cone_map:
                if insert_cap is not None and len(self._cone_map) >= insert_cap:
                    self._cone_map.pop(next(iter(self._cone_map)))
                self._cone_map[key] = new_type
            elif self._cone_map[key] == ConeTypes.UNKNOWN and new_type != ConeTypes.UNKNOWN:
                self._cone_map[key] = new_type

    def _evict_far_cones(self, cap: int) -> None:
        """Over the cap, drop the cones farthest from the car. (Evicting the oldest key instead churned the
        map every message once it held more than the cap, dropping cones next to the car.)"""
        if len(self._cone_map) > cap:
            keys = list(self._cone_map.keys())
            d2 = np.sum((np.array(keys) - self._car_pos) ** 2, axis=1)
            for i in np.argsort(d2)[cap:]:
                del self._cone_map[keys[i]]

    def _cone_xy(self) -> np.ndarray:
        return np.array(list(self._cone_map.keys()), dtype=np.float64)

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
            path_xy, left_xy, right_xy, *_ = self._planner.calculate_path_in_global_frame(
                cones_by_type, self._car_pos, self._car_dir,
                return_intermediate_results=True,
            )
        except Exception as e:
            self.get_logger().error(
                f'Path calculation failed ({type(e).__name__}): {e} | '
                f'left={n_left} right={n_right} unknown={n_unknown}',
                throttle_duration_sec=2.0)
            return None
        return path_xy, left_xy, right_xy

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

    def _publish_plan(self, centerline, left, right) -> None:
        """Publish centerline (skipped when None), then left and right walls (when non-empty)."""
        stamp = self.get_clock().now().to_msg()
        if centerline is not None:
            self._pub_centerline.publish(path_from_xy(centerline, self._map_frame, stamp))
        if left is not None and len(left) > 0:
            self._pub_left.publish(path_from_xy(np.array(left)[:, :2], self._map_frame, stamp))
        if right is not None and len(right) > 0:
            self._pub_right.publish(path_from_xy(np.array(right)[:, :2], self._map_frame, stamp))

    # ------------------------------------------------------------------ colourless / AMZ

    def _plan_colourless(self) -> None:
        """No cone colours: boundary_tracer labels each wall from the driven trajectory and grows both walls
        ahead together. The path ends where the paired walls end."""
        xy = self._cone_xy()
        if self._off_map(xy):
            return
        res = self._tracer.plan(xy, self._car_pos, self._car_dir)
        if res is None:
            self.get_logger().warn(f'boundary tracer: no path ({self._tracer.debug.get("reason")})',
                                   throttle_duration_sec=2.0)
            return
        path, left, right = res.path, res.left, res.right
        if self._unknown_mode == 'trace_backend':
            path = self._backend_on_labels(xy, res)
            if path is None:
                return
        optimized = path
        if self._colourless_raceline and len(path) > 3 and len(left) > 3 and len(right) > 3:
            # the optimiser sizes its corridor from the nearest wall cone, which past the traced walls is the wrong cone
            optimized = clamp_to_corridor(self._racing_line(path, left, right), path, left, right)
        horizon = float(np.sum(np.linalg.norm(np.diff(optimized, axis=0), axis=1))) if len(optimized) > 1 else 0.0
        self.get_logger().info(
            f'boundary tracer: left={len(left)} right={len(right)} gates={res.debug.get("n_gates")} '
            f'labelled={res.debug.get("n_labelled")} horizon={horizon:.1f} m',
            throttle_duration_sec=2.0)
        self._publish_plan(optimized, left, right)
        self._pub_horizon.publish(Float32(data=horizon))

    def _backend_on_labels(self, xy, res):
        """Run the configured backend on the tracer's left/right labels, then cut its path where the traced
        walls end (backends extrapolate past the last cones)."""
        rel = xy - self._car_pos
        ahead = rel @ self._car_dir >= -3.0
        Lm = ahead & (res.labels > 0)
        Rm = ahead & (res.labels < 0)
        if self._amz is not None:
            col = np.where(res.labels > 0, 2, np.where(res.labels < 0, 1, 0))   # ConeTypes LEFT=2 RIGHT=1
            out = self._amz.plan(xy, col, self._car_pos, self._car_dir)
            if out is None:
                return None
            path = out[0]
        else:
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

    def _plan_amz(self) -> None:
        """AMZ-style planner over the live cone map. Colour is kept (UNKNOWN in LiDAR-only mode)."""
        xy = self._cone_xy()
        col = np.array([int(t) for t in self._cone_map.values()])
        if self._off_map(xy):
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
        optimized = path
        if len(path) > 3 and len(left) > 3 and len(right) > 3:
            optimized = self._racing_line(path, left, right)
        self._publish_plan(optimized, left, right)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
