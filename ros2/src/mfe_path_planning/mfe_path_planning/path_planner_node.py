#!/usr/bin/env python3
"""
ROS 2 path planner: fused cone map -> centerline, boundaries and an optional race line.

Subscribes:
    /planning/cones       (mfe_msgs/Track)        fused, colored cone list
    /ekf/output           (nav_msgs/Odometry)     vehicle pose + twist
Publishes:
    /planning/centerline  (nav_msgs/Path)
    /planning/track_left  (nav_msgs/Path)
    /planning/track_right (nav_msgs/Path)
    /planning/path_horizon (std_msgs/Float32)     colourless local planning: path length ahead (m)

The planning logic lives in mfe_path_planning.core (no ROS imports there). This node reads the parameters, converts
the messages, calls the core, publishes the results and writes the log lines.
"""

import json
import math
import os
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node

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

from mfe_path_planning.core.accel import AccelMidline
from mfe_path_planning.core.boundary_tracer import BoundaryTracer
from mfe_path_planning.core.colour import ColourHint, PipelineLatch, SideVotes
from mfe_path_planning.core.cone_map import FSD_TYPES, ConeMap
from mfe_path_planning.core.jump_gate import JumpGate
from mfe_path_planning.core.lap import LapBan, LapReturnReset, LoopTrail, build_locked_loop
from mfe_path_planning.core.params import DEFAULTS, PlannerParams
from mfe_path_planning.core.planning import LocalPlanner, PlanContext
from mfe_path_planning.core.raceline_runner import RaceLineRunner, raceline_gate
from mfe_path_planning.core.skidpad import SkidpadMachine, make_skidpad_path
from mfe_path_planning.planner_utils import path_from_xy
from mfe_common.geometry import arc_length, cloud_xy, yaw_from_quaternion
from mfe_common.qos import LATEST_CONES_QOS, RELIABLE_QOS
from mfe_common.topics import (
    EKF_OUTPUT,
    PERCEPTION_CONES_UNCOLORED,
    PLANNING_CENTERLINE,
    PLANNING_CONES,
    PLANNING_LAPS_COMPLETED,
    PLANNING_PATH_HORIZON,
    PLANNING_TARGET_SPEEDS,
    PLANNING_TRACK_LEFT,
    PLANNING_TRACK_RIGHT,
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

_RECOLOUR = os.environ.get('MFE_PLANNER_RECOLOUR', '0') == '1'   # experiment: a later colour replaces the stored one
_MAP_CAP = 1000            # cone map size cap (local planning / global missions)
_SKIDPAD_MAP_CAP = 500
_LOCK_DIST_M = 60.0        # lap-1 lock-in: distance driven before a return to the start can close the loop
_LOCK_MAX_R_M = 15.0       # ... minimum excursion from the trail start
_LOCK_CLOSE_M = 4.0        # ... and the car back within this of the start


def _cone_arrays(msg: Track):
    """Cone positions (Nx2) and colours of a Track message, for the race-line union."""
    xy = np.array([(c.location.x, c.location.y) for c in msg.track], float).reshape(-1, 2)
    col = np.array([c.color for c in msg.track], int)
    return xy, col


class PathPlannerNode(Node):

    def __init__(self):
        super().__init__('path_planner_node')
        self._last_emit = {}  # (level, text) -> node-clock seconds of the last emit, for _log_events

        if not _LIB_AVAILABLE:
            self.get_logger().error(
                'ft_fsd_path_planning is not installed. '
                'Run: pip install ft-fsd-path-planning')

        cfg = self._declare_params()
        self._setup_state(cfg)
        self._setup_planning(cfg)
        self._setup_io()
        self._log_started(cfg)

    # ------------------------------------------------------------------ setup

    def _declare_params(self) -> PlannerParams:
        """Declare every planner parameter with its default (core.params) and read the values back."""
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default)
        return PlannerParams(**{name: self.get_parameter(name).value for name in DEFAULTS})

    def _setup_state(self, cfg: PlannerParams) -> None:
        """Car pose, cone map, skidpad, lap trails and the core helpers that keep their own state."""
        self._map_frame = cfg.map_frame
        self._mission = cfg.mission
        self._skidpad_mode = cfg.skidpad_mode
        self._planner_profile = cfg.planner_profile
        self._prof_ms: list = []               # planner_profile: _cones_cb durations since the last report
        self._car_pos = np.array([0.0, 0.0])
        self._car_dir = np.array([1.0, 0.0])   # unit vector, default facing +X
        self._car_yaw_rate = 0.0
        self._odom_received = False
        self._laps_done = 0                    # from /planning/laps_completed (finish_detector)
        self._cones = ConeMap(FSD_TYPES, _MFE_TO_FSD, skip_colours=(Cone.ORANGE_BIG, Cone.ORANGE_SMALL),
                              recolour=_RECOLOUR, colour_vote=cfg.colour_vote, vote_min=cfg.vote_min,
                              vote_margin=cfg.vote_margin, cone_expiry=cfg.cone_expiry,
                              expiry_min_obs=cfg.expiry_min_obs, expiry_cycles=cfg.expiry_cycles)
        self._cone_hygiene_log = cfg.colour_vote or cfg.cone_expiry
        if _LIB_AVAILABLE and (int(ConeTypes.UNKNOWN), int(ConeTypes.RIGHT), int(ConeTypes.LEFT), len(ConeTypes)) \
                != (FSD_TYPES.unknown, FSD_TYPES.right, FSD_TYPES.left, FSD_TYPES.n):
            self.get_logger().error('fsd_path_planning ConeTypes differ from core.cone_map.FSD_TYPES')
        self._skid = SkidpadMachine(self._cones.walls)
        self._skidpad_path = None
        self._skidpad_path_published = False
        # Lap-1 lock-in (global missions): record the driven trail; on closing the loop, build the full-lap
        # centerline from the cone map ordered by the trail, optimise it once and lock it.
        self._lap_trail = LoopTrail()
        self._locked_path = None
        self._lap_ban = LapBan()               # cone cells excluded from the tracer's map until re-observed
        self._lap_skip_logged = False
        self._side_votes = SideVotes()         # (x, y) map cell -> lap-1 tracer side votes (+ left, - right)
        self._hint = ColourHint()              # colour_hint: wall-side votes and counters
        self._pipeline = PipelineLatch(cfg.min_coloured_each, cfg.revert_after)
        self._jump = JumpGate(cfg.jump_gate, cfg.jump_heading_deg, cfg.jump_lateral_m, cfg.jump_hold_max,
                              cfg.jump_min_path_m, cfg.jump_remaining_guard)
        self._accel = AccelMidline(cfg.accel_midline_alpha)
        # Offline race line (core.raceline_runner): collects the lap-1 union, builds once, then follows the line.
        self._rl = RaceLineRunner(raceline_gate(self._mission, cfg.raceline_enable), cfg.raceline_after_lap,
                                  cfg.raceline_extra_margin_m,
                                  cfg.raceline_a_lat, cfg.raceline_v_max, cfg.raceline_window_m)

    def _setup_planning(self, cfg: PlannerParams) -> None:
        """Set up the library planner, the boundary tracer, the lap-return reset, the debug log and the pipelines."""
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
            mission = mission_map.get(self._mission, MissionTypes.autocross)
            self._planner = PathPlanner(mission)
            self._global_planning = (mission == MissionTypes.trackdrive)
            if mission == MissionTypes.acceleration and self._planner.relocalizer is not None:
                self._trim_accel_known_path()
        if self._hardcoded_skidpad():
            self._skidpad_path = make_skidpad_path()
        self._tracer = BoundaryTracer()
        # Per-cycle planner debug log (JSON lines, one per planning cycle). '' = off.
        self._dbg_fh = None
        if cfg.planner_debug_log:
            self._dbg_fh = open(cfg.planner_debug_log, 'w', buffering=1)
            self.get_logger().info(f'planner_debug_log: writing {cfg.planner_debug_log}')
        # Lap-return map reset (local missions, boundary tracer input). On the return to the start after a lap,
        # the cone cells mapped within lap_return_radius_m of the START POSE are excluded from the tracer's map.
        # Not for skidpad (own state machine) or acceleration (a single straight, no lap to return to).
        self._lap_reset = None
        self._lap_radius = float(cfg.lap_return_radius_m)
        if cfg.lap_return_reset and self._mission not in ('skidpad', 'acceleration'):
            self._lap_reset = LapReturnReset(cfg.lap_return_start_r_m, cfg.lap_return_min_travel_m,
                                             cfg.lap_return_len_frac)
            # Re-observation needs the current scan (the cumulative SLAM map cannot tell a re-seen cone from a
            # remembered one). Subscribed only when the reset is armed.
            self.create_subscription(PointCloud2, PERCEPTION_CONES_UNCOLORED, self._lidar_cb, LATEST_CONES_QOS)
        self._local = LocalPlanner(
            FSD_TYPES, self._planner, self._tracer, self._cones, self._lap_ban, self._side_votes, self._hint,
            self._pipeline, self._accel, unknown_mode=cfg.unknown_cone_mode,
            colour_after_lap=cfg.colour_after_lap, pipeline_hysteresis=cfg.pipeline_hysteresis,
            accel_refit=cfg.accel_refit_midline)

    def _setup_io(self) -> None:
        """Create the subscribers and publishers."""
        self.create_subscription(Track, PLANNING_CONES, self._cones_cb, LATEST_CONES_QOS)
        # finish_detector publishes laps on a RELIABLE, KEEP_LAST depth-10 QoS (same as RELIABLE_QOS).
        self.create_subscription(Int32, PLANNING_LAPS_COMPLETED, self._laps_cb, RELIABLE_QOS)
        self.create_subscription(Odometry, EKF_OUTPUT, self._odom_cb, RELIABLE_QOS)
        self._pub_centerline = self.create_publisher(Path, PLANNING_CENTERLINE, RELIABLE_QOS)
        self._pub_left = self.create_publisher(Path, PLANNING_TRACK_LEFT, RELIABLE_QOS)
        self._pub_right = self.create_publisher(Path, PLANNING_TRACK_RIGHT, RELIABLE_QOS)
        self._pub_horizon = self.create_publisher(Float32, PLANNING_PATH_HORIZON, RELIABLE_QOS)
        if self._rl.enabled:
            self._pub_speeds = self.create_publisher(Float64MultiArray, PLANNING_TARGET_SPEEDS, RELIABLE_QOS)

    def _log_started(self, cfg: PlannerParams) -> None:
        if self._mission == 'skidpad':
            if self._skidpad_mode == 'hardcoded':
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
                f'PathPlannerNode started. Mission: {self._mission} | local planner: {cfg.planner_backend}')

    def _hardcoded_skidpad(self) -> bool:
        return self._mission == 'skidpad' and self._skidpad_mode == 'hardcoded'

    def _trim_accel_known_path(self) -> None:
        """
        ft-fsd's known acceleration path is a loop; its pathing splices the return leg into the path.

        (replayed HA02: 7 deg swerve). Keep only the outbound straight.
        """
        known = self._planner.relocalizer.get_known_global_path()
        ret = known[:, 1] > 1.0
        if ret.any():
            known = known[:int(np.argmax(ret))]
            self._planner.relocalizer.get_known_global_path = lambda: known

    # ------------------------------------------------------------------ callbacks

    def _odom_cb(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        self._car_pos = np.array([pos.x, pos.y])
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self._car_dir = np.array([math.cos(yaw), math.sin(yaw)])
        self._car_yaw_rate = float(msg.twist.twist.angular.z)
        self._odom_received = True
        self._tracer.add_pose(self._car_pos)
        if self._rl.collect_wanted(self._laps_done):
            self._rl.collect_odom(self._car_pos)
        if self._lap_reset is not None and self._lap_reset.update(self._car_pos):
            self._lap_return_fire()
        if self._lap_ban.banned:
            self._lap_lift()

        if self._global_planning and self._locked_path is None:
            self._lap_trail.advance(self._car_pos)

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
                f'max={ms.max():.2f} n={len(ms)}')
            self._prof_ms = []

    def _cones_cb_body(self, msg: Track) -> None:
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
        self._cones.evict_far(_MAP_CAP, self._car_pos)

        if (self._global_planning and self._lap_trail.trail
                and self._lap_trail.closed(self._car_pos, _LOCK_DIST_M, _LOCK_MAX_R_M, _LOCK_CLOSE_M)):
            locked = self._build_locked_loop()
            if locked is not None:
                self._locked_path = locked
                self.get_logger().info(
                    f'Lap 1 complete — locked full-loop path: {len(locked)} waypoints, '
                    f'{arc_length(locked):.1f} m, built from {len(self._cones)} live-mapped cones. '
                    f'Re-planning stopped.')
                self._publish_centerline(locked)
                return

        # Offline race line (raceline_enable): collects the lap-1 union, builds once, then follows the line.
        window, events = self._rl.cycle(self._car_pos, self._laps_done, lambda: _cone_arrays(msg))
        self._log_events(events)
        if window is not None:
            self._publish_race_line(*window)
            return

        self._apply_plan(self._local.plan(self._plan_ctx()))

    def _handle_skidpad_cone_based(self, msg: Track) -> None:
        """Skidpad state machine (core.skidpad): a locked circle is published; otherwise local planning."""
        if not self._odom_received:
            return
        debug = self._skid.update_yaw_rate(self._car_yaw_rate)
        if debug is not None:
            self.get_logger().debug(debug)
        self._merge_cones(msg, insert_cap=_SKIDPAD_MAP_CAP)
        self._skid.advance(self._car_pos)
        self._log_events(self._skid.step(self._car_pos))
        locked = self._skid.active_locked()
        if locked is not None:
            self._publish_centerline(locked)
            return
        self._apply_plan(self._local.skidpad_local(self._plan_ctx(), bool(msg.track)))

    def _laps_cb(self, msg: Int32) -> None:
        self._laps_done = int(msg.data)

    def _lidar_cb(self, msg: PointCloud2) -> None:
        """
        Lap-return lift on each scan.

        The scan is in the velodyne frame (base_footprint + 0.3 m up, no horizontal offset), so a point maps to the
        map by the car pose and heading.
        """
        if not self._lap_ban.banned or not self._odom_received:
            return
        xs, ys = cloud_xy(msg)
        c, s = self._car_dir
        det = np.c_[self._car_pos[0] + c * xs - s * ys, self._car_pos[1] + s * xs + c * ys]
        self._lap_lift(det)

    # ------------------------------------------------------------------ cone map and lap-return reset (node state)

    def _build_locked_loop(self) -> Optional[np.ndarray]:
        """Lap-1 lock-in: full-lap closed centreline from the cone walls, ordered by the lap-1 trail."""
        events: list = []
        blue, yellow = self._cones.walls()
        locked = build_locked_loop(self._lap_trail.trail, blue, yellow, events)
        self._log_events(events)
        return locked

    def _merge_cones(self, msg: Track, insert_cap: Optional[int] = None) -> None:
        """Merge the cones into the map (ConeMap.merge) and log the colour-vote and expiry counters."""
        self._cones.merge([(c.location.x, c.location.y, c.color) for c in msg.track],
                          self._car_pos, self._car_dir, insert_cap)
        if self._cone_hygiene_log and self._cones.merge_cycle % 50 == 0:
            self.get_logger().info(
                f'colour_flips={self._cones.colour_flips} cones_expired={self._cones.cones_expired} '
                f'map={len(self._cones)}')

    def _lap_return_fire(self) -> None:
        """Lap-return trigger: exclude the map cells within lap_return_radius_m of the start pose from the tracer."""
        if self._global_planning or self._locked_path is not None:
            if not self._lap_skip_logged:
                self._lap_skip_logged = True
                self.get_logger().info('lap_return_reset skipped: lap-1 lock-in active (or global planning); '
                                       'the tracer is not used here')
            return
        keys = self._cones.keys()
        start = self._lap_reset.start
        new = self._lap_ban.fire(keys, start, self._lap_radius)
        self.get_logger().info(
            f'lap_return_reset TRIGGERED #{self._lap_reset.fires} at car=({self._car_pos[0]:.1f},'
            f'{self._car_pos[1]:.1f}) lap_travel={self._lap_reset.lap_travel:.0f} m: excluded {new} of '
            f'{len(keys)} mapped cells within {self._lap_radius:.0f} m of start ({start[0]:.1f},{start[1]:.1f}); '
            f'total excluded {len(self._lap_ban.banned)}')

    def _lap_lift(self, det=None) -> None:
        """
        Accept banned cells again: re-observed by the current scan (det, map frame: within.

        LAP_REOBS_TOL_M), or passed by the car (lap_passed). Cells not yet seen or passed stay banned.
        """
        hit = self._lap_ban.lift(self._car_pos, self._car_dir, det)
        if hit:
            self.get_logger().info(
                f'lap_return_reset: {hit} banned cells accepted again (total {self._lap_ban.reobs}); '
                f'still banned {len(self._lap_ban.banned)}', throttle_duration_sec=1.0)

    # ------------------------------------------------------------------ publishing and logging

    def _plan_ctx(self) -> PlanContext:
        return PlanContext(car_pos=self._car_pos, car_dir=self._car_dir, laps_done=self._laps_done,
                           mission=self._mission, global_planning=self._global_planning,
                           now_s=self.get_clock().now().nanoseconds * 1e-9)

    def _apply_plan(self, plan) -> None:
        """Write a core Plan out: log lines and debug records, then centreline and walls, then the path horizon."""
        self._log_events(plan.events)
        for rec in plan.pre_cycles:
            self._log_cycle(**rec)
        if plan.publish:
            self._publish_plan(plan.centerline, plan.left, plan.right)
        if plan.horizon is not None:
            self._pub_horizon.publish(Float32(data=plan.horizon))
        for rec in plan.cycles:
            self._log_cycle(**rec)

    def _log_events(self, events) -> None:
        """
        Write the (level, text[, kwargs]) events returned by core code to the node logger.

        Throttling is done here in plain Python, not by rclpy. rclpy keys its throttle state by call
        site, so one call site given different throttle_duration_sec values raises "Requested logging
        filters cannot be changed between calls". Severity is also cached per call site, so each
        severity has its own call. Each (level, text) is emitted at most once per its throttle window,
        on the node clock.
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        for ev in events:
            level, text = ev[0], ev[1]
            throttle = float(ev[2].get('throttle_duration_sec', 0.0)) if len(ev) > 2 else 0.0
            if throttle > 0.0:
                key = (level, text)
                last = self._last_emit.get(key)
                if last is not None and now - last < throttle:
                    continue
                self._last_emit[key] = now
                if len(self._last_emit) > 512:
                    # bound the table: keep only keys emitted in the last minute
                    self._last_emit = {k: t for k, t in self._last_emit.items() if now - t < 60.0}
            # one literal call per severity: rclpy also caches the severity per call site
            if level == 'warn':
                self.get_logger().warn(text)
            elif level == 'error':
                self.get_logger().error(text)
            elif level == 'debug':
                self.get_logger().debug(text)
            else:
                self.get_logger().info(text)

    def _publish_centerline(self, xy: np.ndarray) -> None:
        stamp = self.get_clock().now().to_msg()
        self._pub_centerline.publish(path_from_xy(xy, self._map_frame, stamp))

    def _publish_race_line(self, xy: np.ndarray, speeds) -> None:
        """Race-line window (centreline and target speeds)."""
        stamp = self.get_clock().now().to_msg()
        self._pub_centerline.publish(path_from_xy(xy, self._map_frame, stamp))
        msg = Float64MultiArray()
        msg.data = speeds
        self._pub_speeds.publish(msg)

    def _jump_gate(self, xy: np.ndarray) -> np.ndarray:
        """Centreline to publish: xy, or the last published path when xy jumps (core.jump_gate)."""
        out, events = self._jump.apply(xy, self._car_pos, self._odom_received)
        self._log_events(events)
        return out

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

    def _log_cycle(self, n_cones, reason, dbg, horizon=None, mode=None) -> None:
        """
        planner_debug_log: one JSON line per planning cycle (car pose, cones, tracer reason, gates, path.

        jump-gate decision, lap-return reset state). Never raises into the planning loop.
        """
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
                jump=self._jump.last,
                reset=[self._lap_reset.fires if self._lap_reset is not None else None, len(self._lap_ban.banned)])
            if mode is not None:
                rec['mode'] = mode
                rec['laps'] = self._laps_done
            self._dbg_fh.write(json.dumps(rec) + '\n')
        except Exception as e:  # noqa: BLE001 - logging must not break planning
            self.get_logger().warn(f'planner_debug_log write failed: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
