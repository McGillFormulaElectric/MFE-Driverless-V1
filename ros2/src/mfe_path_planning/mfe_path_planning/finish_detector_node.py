#!/usr/bin/env python3
"""
Finish-line detector for FSAE events.

Detection strategy per mission:

ACCELERATION
  1. LiDAR: dense returns in forward zone (1.5–15 m ahead) once past approach distance.
  2. Orange-cone gate: subscribe to /planning/cones, find orange cone cluster with
     x > approach_x, use their centroid-x as the dynamic finish gate.
     Triggers when car_x >= orange_gate_x.
  3. Fallback: hardcoded finish_x (78 m) if no orange cones are detected.

SKIDPAD
  Sequence (per FSAE rules + path_planner): 2 laps LEFT circle -> 2 laps RIGHT circle -> exit.
  Circle centers are derived from orange inner-cone positions in /planning/cones.
  If insufficient orange cones, defaults to (14.4, +/-9.3).

  Phase machine:
    ENTRY        -- heading toward left circle
    LEFT_CIRCLE  -- counting CCW angle accumulation, 2x2pi = 2 laps
    RIGHT_CIRCLE -- counting CW  angle accumulation, 2x2pi = 2 laps
    EXIT         -- detect orange end-gate or position threshold

AUTOCROSS / TRACKDRIVE / PEANUT
  Return-to-start lap counting (unchanged from original).

When finish is detected:
  - Publishes /ros_can/mission_completed (Bool True)
  - Publishes /planning/mission_finished (Bool True, latched)
  - Publishes full brake to /control/command at 50 Hz
"""

import math
from collections import deque
from enum import IntEnum, auto

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32
from fs_msgs.msg import ControlCommand
from mfe_msgs.msg import Cone, Track


def _extract_xyz(msg: PointCloud2) -> np.ndarray:
    """Return Nx3 float32 array of valid (x,y,z) points from a PointCloud2."""
    field_map = {f.name: f for f in msg.fields}
    if not all(k in field_map for k in ('x', 'y', 'z')):
        return np.zeros((0, 3), dtype=np.float32)
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3), dtype=np.float32)
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    step = msg.point_step

    def get_col(name: str) -> np.ndarray:
        offset = field_map[name].offset
        idx = np.arange(n) * step + offset
        col = np.stack([raw[idx], raw[idx+1], raw[idx+2], raw[idx+3]], axis=1)
        return col.view(np.float32).reshape(-1)

    pts = np.stack([get_col('x'), get_col('y'), get_col('z')], axis=1)
    return pts[np.isfinite(pts).all(axis=1)].astype(np.float32)


class _SkidpadPhase(IntEnum):
    ENTRY        = auto()
    LEFT_CIRCLE  = auto()
    RIGHT_CIRCLE = auto()
    EXIT         = auto()


# Default skidpad circle geometry (EUFS standard track, matches path_planner_node.py)
_SKIDPAD_CX   = 14.4
_SKIDPAD_CY_L =  9.3
_SKIDPAD_CY_R = -9.3
_SKIDPAD_R    =  9.1


class FinishDetectorNode(Node):

    def __init__(self):
        super().__init__('finish_detector')

        # -- General params
        self.declare_parameter('mission', 'autocross')
        self.declare_parameter('approach_x', 60.0)
        self.declare_parameter('finish_x', 78.0)
        self.declare_parameter('min_travel_m', 0.0)
        self.declare_parameter('return_to_start_r', 0.0)
        self.declare_parameter('detect_fwd_min_m', 1.5)
        self.declare_parameter('detect_fwd_max_m', 15.0)
        self.declare_parameter('detect_lateral_m', 4.0)
        self.declare_parameter('min_finish_points', 1)
        self.declare_parameter('num_laps', 1)

        self._mission      = self.get_parameter('mission').value
        self._approach_x   = self.get_parameter('approach_x').value
        self._finish_x     = self.get_parameter('finish_x').value
        self._min_travel_m = self.get_parameter('min_travel_m').value
        self._return_r     = self.get_parameter('return_to_start_r').value
        self._fwd_min      = self.get_parameter('detect_fwd_min_m').value
        self._fwd_max      = self.get_parameter('detect_fwd_max_m').value
        self._lat_max      = self.get_parameter('detect_lateral_m').value
        self._min_pts      = self.get_parameter('min_finish_points').value
        self._num_laps     = max(1, self.get_parameter('num_laps').value)

        # -- Odometry / travel
        self._car_x               = 0.0
        self._car_y               = 0.0
        self._start_x             = None
        self._start_y             = None
        self._prev_x              = None
        self._prev_y              = None
        self._travel_m            = 0.0
        self._max_dist_from_start = 0.0
        self._laps_completed      = 0
        self._finished            = False

        # -- Acceleration: dynamic orange-cone finish gate
        self._orange_gate_x           = None
        self._orange_gate_candidates  = deque(maxlen=30)

        # -- Skidpad phase machine
        self._skidpad_phase      = _SkidpadPhase.ENTRY
        self._left_cx            = _SKIDPAD_CX
        self._left_cy            = _SKIDPAD_CY_L
        self._right_cx           = _SKIDPAD_CX
        self._right_cy           = _SKIDPAD_CY_R
        self._circle_r           = _SKIDPAD_R
        self._orange_cones_left  = []
        self._orange_cones_right = []
        self._angle_prev         = None
        self._angle_accum        = 0.0
        self._laps_left          = 0
        self._laps_right         = 0

        # -- QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # -- Subscribers
        self.create_subscription(PointCloud2, '/lidar/points_raw', self._lidar_cb, sensor_qos)
        self.create_subscription(Odometry, '/ekf/output', self._odom_cb, reliable_qos)
        self.create_subscription(Track, '/planning/cones', self._cones_cb, reliable_qos)

        # -- Publishers
        self._mission_completed_pub = self.create_publisher(
            Bool, '/ros_can/mission_completed', reliable_qos)
        self._finished_pub = self.create_publisher(
            Bool, '/planning/mission_finished', latched_qos)
        self._cmd_pub = self.create_publisher(
            ControlCommand, '/control/command', reliable_qos)
        self._laps_pub = self.create_publisher(
            Int32, '/planning/laps_completed', reliable_qos)

        init_msg = Bool()
        init_msg.data = False
        self._finished_pub.publish(init_msg)

        self.create_timer(0.02, self._brake_loop)

        self.get_logger().info(
            f'FinishDetectorNode | mission={self._mission} | num_laps={self._num_laps} | '
            f'min_travel={self._min_travel_m} m | fallback_finish_x={self._finish_x} m')

    # -- Finish latch -----------------------------------------------------------

    def _latch_finished(self, source: str) -> None:
        if self._finished:
            return
        self.get_logger().info(
            f'Finish detected ({source}) at ({self._car_x:.1f}, {self._car_y:.1f}) -- BRAKING!')
        self._finished = True
        done = Bool()
        done.data = True
        self._mission_completed_pub.publish(done)
        self._finished_pub.publish(done)

    # -- Cone map callback (accel + skidpad) ------------------------------------

    def _cones_cb(self, msg: Track) -> None:
        if self._finished:
            return
        orange_colors = {Cone.ORANGE_BIG, Cone.ORANGE_SMALL}
        orange = [c for c in msg.track if c.color in orange_colors]
        if not orange:
            return

        if self._mission == 'acceleration':
            self._update_accel_gate(orange)
        elif self._mission == 'skidpad':
            self._update_skidpad_circles(orange)

    def _update_accel_gate(self, orange_cones: list) -> None:
        candidates = [c for c in orange_cones if c.location.x > self._approach_x]
        if len(candidates) < 2:
            return
        gate_x = float(np.mean([c.location.x for c in candidates]))
        self._orange_gate_candidates.append(gate_x)
        if len(self._orange_gate_candidates) >= 5:
            stable_x = float(np.median(self._orange_gate_candidates))
            if self._orange_gate_x is None:
                self.get_logger().info(
                    f'[accel] Orange finish gate locked at x={stable_x:.1f} m '
                    f'({len(candidates)} cones). Replaces hardcoded {self._finish_x} m.')
            self._orange_gate_x = stable_x

    def _update_skidpad_circles(self, orange_cones: list) -> None:
        for c in orange_cones:
            pt = (c.location.x, c.location.y)
            if c.location.y > 0:
                self._orange_cones_left.append(pt)
            else:
                self._orange_cones_right.append(pt)

        if len(self._orange_cones_left) >= 3:
            arr = np.array(self._orange_cones_left)
            cx, cy = float(np.median(arr[:, 0])), float(np.median(arr[:, 1]))
            if abs(cx - self._left_cx) > 0.1 or abs(cy - self._left_cy) > 0.1:
                self.get_logger().info(f'[skidpad] Left circle centre -> ({cx:.2f}, {cy:.2f})')
            self._left_cx, self._left_cy = cx, cy

        if len(self._orange_cones_right) >= 3:
            arr = np.array(self._orange_cones_right)
            cx, cy = float(np.median(arr[:, 0])), float(np.median(arr[:, 1]))
            if abs(cx - self._right_cx) > 0.1 or abs(cy - self._right_cy) > 0.1:
                self.get_logger().info(f'[skidpad] Right circle centre -> ({cx:.2f}, {cy:.2f})')
            self._right_cx, self._right_cy = cx, cy

    # -- Odometry callback ------------------------------------------------------

    def _odom_cb(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._car_x = x
        self._car_y = y

        if self._start_x is None:
            self._start_x, self._start_y = x, y

        if self._prev_x is not None:
            self._travel_m += math.hypot(x - self._prev_x, y - self._prev_y)
        self._prev_x, self._prev_y = x, y

        if self._finished:
            return

        if self._mission == 'acceleration':
            self._check_acceleration(x)
        elif self._mission == 'skidpad':
            self._check_skidpad(x, y)
        elif self._mission in ('autocross', 'trackdrive', 'peanut'):
            self._check_return_to_start(x, y)

    # -- Acceleration -----------------------------------------------------------

    def _check_acceleration(self, x: float) -> None:
        if self._min_travel_m > 0 and self._travel_m < self._min_travel_m:
            return
        gate_x = self._orange_gate_x if self._orange_gate_x is not None else self._finish_x
        if x >= gate_x:
            src = (f'orange-cone gate x={gate_x:.1f} m'
                   if self._orange_gate_x is not None
                   else f'position fallback x={gate_x:.1f} m (no orange cones found)')
            self._latch_finished(src)

    # -- Skidpad phase machine --------------------------------------------------

    def _check_skidpad(self, x: float, y: float) -> None:
        phase = self._skidpad_phase

        if phase == _SkidpadPhase.ENTRY:
            dist_left = math.hypot(x - self._left_cx, y - self._left_cy)
            if dist_left < self._circle_r + 2.0 and self._travel_m > 5.0:
                self.get_logger().info('[skidpad] Entering LEFT circle.')
                self._skidpad_phase = _SkidpadPhase.LEFT_CIRCLE
                self._angle_prev    = math.atan2(y - self._left_cy, x - self._left_cx)
                self._angle_accum   = 0.0

        elif phase == _SkidpadPhase.LEFT_CIRCLE:
            angle_now = math.atan2(y - self._left_cy, x - self._left_cx)
            if self._angle_prev is not None:
                delta = angle_now - self._angle_prev
                if delta >  math.pi: delta -= 2 * math.pi
                if delta < -math.pi: delta += 2 * math.pi
                self._angle_accum += delta    # CCW = positive
            self._angle_prev = angle_now
            laps = int(abs(self._angle_accum) / (2 * math.pi))
            if laps > self._laps_left:
                self._laps_left = laps
                lp = Int32(); lp.data = self._laps_left; self._laps_pub.publish(lp)
                self.get_logger().info(f'[skidpad] LEFT lap {self._laps_left}/2 done.')
            if self._laps_left >= 2:
                self.get_logger().info('[skidpad] 2 LEFT laps complete -> RIGHT circle.')
                self._skidpad_phase = _SkidpadPhase.RIGHT_CIRCLE
                self._angle_prev    = math.atan2(y - self._right_cy, x - self._right_cx)
                self._angle_accum   = 0.0

        elif phase == _SkidpadPhase.RIGHT_CIRCLE:
            angle_now = math.atan2(y - self._right_cy, x - self._right_cx)
            if self._angle_prev is not None:
                delta = angle_now - self._angle_prev
                if delta >  math.pi: delta -= 2 * math.pi
                if delta < -math.pi: delta += 2 * math.pi
                self._angle_accum += delta    # CW = negative, use abs
            self._angle_prev = angle_now
            laps = int(abs(self._angle_accum) / (2 * math.pi))
            if laps > self._laps_right:
                self._laps_right = laps
                lp = Int32(); lp.data = self._laps_left + self._laps_right; self._laps_pub.publish(lp)
                self.get_logger().info(f'[skidpad] RIGHT lap {self._laps_right}/2 done.')
            if self._laps_right >= 2:
                self.get_logger().info('[skidpad] 2 RIGHT laps complete -> EXIT.')
                self._skidpad_phase = _SkidpadPhase.EXIT
                self._angle_prev    = None

        elif phase == _SkidpadPhase.EXIT:
            exit_x = self._orange_gate_x if self._orange_gate_x is not None else (self._right_cx + 20.0)
            if x >= exit_x and abs(y) < 4.0:
                self._latch_finished(
                    f'skidpad exit x={exit_x:.1f} m (2L+2R laps done, y={y:.1f} m)')

    # -- Autocross / trackdrive / peanut ----------------------------------------

    def _check_return_to_start(self, x: float, y: float) -> None:
        if self._start_x is None:
            return
        dist_from_start = math.hypot(x - self._start_x, y - self._start_y)
        if dist_from_start > self._max_dist_from_start:
            self._max_dist_from_start = dist_from_start

        if (self._return_r > 0.0
                and self._travel_m >= self._min_travel_m
                and self._max_dist_from_start >= 15.0
                and dist_from_start <= self._return_r):
            self._laps_completed += 1
            lp = Int32(); lp.data = self._laps_completed; self._laps_pub.publish(lp)
            if self._laps_completed >= self._num_laps:
                self._latch_finished(
                    f'return-to-start lap {self._laps_completed}/{self._num_laps} '
                    f'(dist={dist_from_start:.1f} m, travel={self._travel_m:.0f} m)')
            else:
                self.get_logger().info(
                    f'Lap {self._laps_completed}/{self._num_laps} done -- continuing.')
                self._max_dist_from_start = 0.0
        elif self._car_x >= self._finish_x:
            self._latch_finished(f'position fallback (car_x={self._car_x:.1f} >= {self._finish_x})')

    # -- LiDAR gate (accel fast-path) -------------------------------------------

    def _lidar_cb(self, msg: PointCloud2) -> None:
        if self._finished or self._mission != 'acceleration':
            return
        if self._min_travel_m > 0.0:
            if self._travel_m < self._min_travel_m:
                return
        elif self._car_x < self._approach_x:
            return
        pts = _extract_xyz(msg)
        if pts.shape[0] == 0:
            return
        mask = (
            (pts[:, 0] > self._fwd_min) &
            (pts[:, 0] < self._fwd_max) &
            (np.abs(pts[:, 1]) < self._lat_max) &
            (pts[:, 2] > -0.15)
        )
        if int(np.sum(mask)) >= self._min_pts:
            self._latch_finished(f'LiDAR ({int(np.sum(mask))} pts in forward zone)')

    # -- 50 Hz brake loop -------------------------------------------------------

    def _brake_loop(self) -> None:
        if not self._finished:
            return
        cmd = ControlCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'map'
        cmd.throttle = 0.0
        cmd.brake    = 1.0
        cmd.steering = 0.0
        self._cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FinishDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
