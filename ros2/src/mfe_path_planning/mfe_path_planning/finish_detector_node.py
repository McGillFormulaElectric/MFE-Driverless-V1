#!/usr/bin/env python3
"""
Finish-line detector for FSAE events. Latches a full brake once any enabled check fires:

1. Orange-gate passes (gate_crossings > 0): clusters perceived ORANGE_BIG cones from /planning/cones into
   gates and counts car passes through them. Finish after gate_crossings passes (+ stop_after_gate_m).
2. Return-to-start (closed loops): counts lap crossings of the start line (within return_to_start_r,
   heading-aligned, after min_travel_m and a 15 m excursion). Finish after num_laps crossings.
3. LiDAR (linear tracks, disabled in gate mode): dense returns in the forward zone past approach_x.
4. Position fallback: odometry x >= finish_x.

On finish it publishes /ros_can/mission_completed and the latched /planning/mission_finished, and
publishes full brake on /control/command at 50 Hz.
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32
from fs_msgs.msg import ControlCommand
from mfe_msgs.msg import Track, Cone

_LAP_EXCURSION_M = 15.0        # must leave this far from start before a lap crossing counts
_SPAWN_DEBOUNCE_M = 5.0        # gate passes before this much travel are ignored
_GATE_CLUSTER_M = 8.0          # single-linkage distance between pylons of one gate
_GATE_TRACK_M = 5.0            # a cluster keeps its pass state if its centroid moves less than this
_GATE_HYSTERESIS_M = 2.0       # car must leave gate_enter_r + this before the next pass counts


def _extract_xyz(msg: PointCloud2) -> np.ndarray:
    """Return Nx3 float32 array of finite (x, y, z) points from a PointCloud2."""
    field_map = {f.name: f for f in msg.fields}
    n = msg.width * msg.height
    if not all(k in field_map for k in ('x', 'y', 'z')) or n == 0:
        return np.zeros((0, 3), dtype=np.float32)
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)

    def get_col(name: str) -> np.ndarray:
        idx = np.arange(n) * msg.point_step + field_map[name].offset
        return np.stack([raw[idx + k] for k in range(4)], axis=1).view(np.float32).reshape(-1)

    pts = np.stack([get_col('x'), get_col('y'), get_col('z')], axis=1)
    return pts[np.isfinite(pts).all(axis=1)].astype(np.float32)


class FinishDetectorNode(Node):
    """Detects the finish line and latches a full brake command."""

    def __init__(self):
        super().__init__('finish_detector')

        self.declare_parameter('approach_x', 60.0)
        self.declare_parameter('finish_x', 78.0)          # position fallback trigger
        self.declare_parameter('min_travel_m', 0.0)       # cumulative travel before lap detection
        self.declare_parameter('return_to_start_r', 0.0)  # >0 enables return-to-start lap counting
        self.declare_parameter('detect_fwd_min_m', 1.5)
        self.declare_parameter('detect_fwd_max_m', 15.0)
        self.declare_parameter('detect_lateral_m', 4.0)
        self.declare_parameter('min_finish_points', 1)
        self.declare_parameter('num_laps', 1)
        self.declare_parameter('gate_crossings', 0)       # >0 enables orange-gate counting
        self.declare_parameter('stop_after_gate_m', 0.0)  # run-out after the final gate pass
        self.declare_parameter('gate_enter_r', 4.5)       # gate pass = within this radius of centroid

        self._approach_x = self.get_parameter('approach_x').value
        self._finish_x = self.get_parameter('finish_x').value
        self._min_travel_m = self.get_parameter('min_travel_m').value
        self._return_r = self.get_parameter('return_to_start_r').value
        self._fwd_min = self.get_parameter('detect_fwd_min_m').value
        self._fwd_max = self.get_parameter('detect_fwd_max_m').value
        self._lat_max = self.get_parameter('detect_lateral_m').value
        self._min_pts = self.get_parameter('min_finish_points').value
        self._num_laps = max(1, self.get_parameter('num_laps').value)
        self._gate_crossings = self.get_parameter('gate_crossings').value
        self._stop_after_gate_m = self.get_parameter('stop_after_gate_m').value
        self._gate_enter_r = self.get_parameter('gate_enter_r').value

        self._car_x = 0.0
        self._start_x = None
        self._start_y = None
        self._start_yaw = None
        self._prev_long = None   # previous position along the start heading (m)
        self._prev_x = None
        self._prev_y = None
        self._travel_m = 0.0
        self._max_dist_from_start = 0.0   # re-armed after each lap
        self._laps_completed = 0
        self._finished = False

        self._gate_cones = {}     # 1 m grid cell -> np[x, y] of accumulated ORANGE_BIG pylons
        self._gate_states = []    # per gate cluster: {'c': np[x, y], 'inside': bool | None}
        self._gate_passes = 0
        self._stop_at_travel = None

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

        self.create_subscription(PointCloud2, '/lidar/points_raw', self._lidar_cb, sensor_qos)
        self.create_subscription(Odometry, '/ekf/output', self._odom_cb, reliable_qos)
        if self._gate_crossings > 0:
            self.create_subscription(Track, '/planning/cones', self._cones_cb, sensor_qos)

        self._mission_completed_pub = self.create_publisher(
            Bool, '/ros_can/mission_completed', reliable_qos)
        self._finished_pub = self.create_publisher(
            Bool, '/planning/mission_finished', latched_qos)
        self._cmd_pub = self.create_publisher(ControlCommand, '/control/command', reliable_qos)
        self._laps_pub = self.create_publisher(Int32, '/planning/laps_completed', reliable_qos)

        # Clear any stale latched True from a previous run.
        init_msg = Bool()
        init_msg.data = False
        self._finished_pub.publish(init_msg)

        # Faster than pure pursuit (20 Hz) so the brake dominates.
        self.create_timer(0.02, self._brake_loop)

        self.get_logger().info(
            f'FinishDetectorNode started | num_laps={self._num_laps} | '
            f'gate_crossings={self._gate_crossings} '
            f'(stop_after_gate={self._stop_after_gate_m} m) | '
            f'min_travel={self._min_travel_m} m | '
            f'return_to_start_r={self._return_r} m | '
            f'LiDAR range=[{self._fwd_min},{self._fwd_max}] m | '
            f'position fallback: finish_x={self._finish_x} m')

    def _latch_finished(self, source: str) -> None:
        if self._finished:
            return
        self.get_logger().info(
            f'Finish line detected ({source}) at car_x={self._car_x:.1f} m — BRAKING!')
        self._finished = True

        done = Bool()
        done.data = True
        self._mission_completed_pub.publish(done)   # -> EUFS state machine
        self._finished_pub.publish(done)            # -> pure_pursuit

    def _publish_count(self, value: int) -> None:
        msg = Int32()
        msg.data = value
        self._laps_pub.publish(msg)

    def _cones_cb(self, msg: Track) -> None:
        if self._finished:
            return
        changed = False
        for cone in msg.track:
            if cone.color != Cone.ORANGE_BIG:
                continue
            key = (round(cone.location.x), round(cone.location.y))
            if key not in self._gate_cones:
                self._gate_cones[key] = np.array([cone.location.x, cone.location.y])
                changed = True
        if changed:
            self._rebuild_gates()

    def _rebuild_gates(self) -> None:
        """Cluster pylons into gates, carrying each gate's pass state over from the previous clustering."""
        clusters: list = []
        for p in self._gate_cones.values():
            for cl in clusters:
                if any(math.hypot(p[0] - q[0], p[1] - q[1]) < _GATE_CLUSTER_M for q in cl):
                    cl.append(p)
                    break
            else:
                clusters.append([p])
        old = self._gate_states
        self._gate_states = []
        for cl in clusters:
            c = np.mean(cl, axis=0)
            prev = next((s for s in old
                         if math.hypot(s['c'][0] - c[0], s['c'][1] - c[1]) < _GATE_TRACK_M), None)
            # inside=None: initialised on the next odometry tick without counting a pass, so a gate
            # the car already sits on (spawn, or a camera gate that first forms under it) is not counted.
            self._gate_states.append({'c': c, 'inside': prev['inside'] if prev is not None else None})

    def _check_gate_passes(self, x: float, y: float) -> None:
        for g in self._gate_states:
            d = math.hypot(x - g['c'][0], y - g['c'][1])
            if g['inside'] is None:
                g['inside'] = d <= self._gate_enter_r
            elif not g['inside'] and d <= self._gate_enter_r:
                g['inside'] = True
                if self._travel_m > _SPAWN_DEBOUNCE_M:
                    self._gate_passes += 1
                    self._publish_count(self._gate_passes)
                    if self._gate_passes >= self._gate_crossings and self._stop_at_travel is None:
                        if self._stop_after_gate_m <= 0.0:
                            self._latch_finished(
                                f'orange gate pass {self._gate_passes}/{self._gate_crossings}')
                            return
                        self._stop_at_travel = self._travel_m + self._stop_after_gate_m
                        self.get_logger().info(
                            f'Final orange gate pass {self._gate_passes}/{self._gate_crossings} '
                            f'— braking after {self._stop_after_gate_m:.0f} m run-out.')
                    else:
                        self.get_logger().info(
                            f'Orange gate pass {self._gate_passes}/{self._gate_crossings} '
                            f'(travel={self._travel_m:.0f} m)')
            elif g['inside'] and d > self._gate_enter_r + _GATE_HYSTERESIS_M:
                g['inside'] = False
        if self._stop_at_travel is not None and self._travel_m >= self._stop_at_travel:
            self._latch_finished(f'orange gate + {self._stop_after_gate_m:.0f} m run-out')

    def _odom_cb(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._car_x = x

        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self._start_x is None:
            self._start_x, self._start_y, self._start_yaw = x, y, yaw
        # Position relative to the start pose: along the start heading (long) and across it (lat).
        dx, dy = x - self._start_x, y - self._start_y
        c, s = math.cos(self._start_yaw), math.sin(self._start_yaw)
        long, lat = c * dx + s * dy, -s * dx + c * dy
        prev_long, self._prev_long = self._prev_long, long

        if self._prev_x is not None:
            self._travel_m += math.hypot(x - self._prev_x, y - self._prev_y)
        self._prev_x, self._prev_y = x, y

        if self._finished:
            return

        dist_from_start = math.hypot(x - self._start_x, y - self._start_y)
        if dist_from_start > self._max_dist_from_start:
            self._max_dist_from_start = dist_from_start

        if self._gate_crossings > 0:
            self._check_gate_passes(x, y)
            if self._finished:
                return

        # A lap counts when long crosses from behind to past the start, within return_r sideways and
        # heading within 60 deg of the start heading. Distance alone fired early on peanut.
        if (self._return_r > 0.0 and self._travel_m >= self._min_travel_m
                and self._max_dist_from_start >= _LAP_EXCURSION_M):
            heading_ok = math.cos(yaw - self._start_yaw) > 0.5
            crossed = prev_long is not None and prev_long < 0.0 <= long
            if crossed and abs(lat) <= self._return_r and heading_ok:
                self._laps_completed += 1
                self._publish_count(self._laps_completed)

                if self._laps_completed >= self._num_laps:
                    self._latch_finished(
                        f'return-to-start lap {self._laps_completed}/{self._num_laps} '
                        f'(dist={dist_from_start:.1f} m, travel={self._travel_m:.0f} m)')
                else:
                    self.get_logger().info(
                        f'Lap {self._laps_completed}/{self._num_laps} complete '
                        f'— dist={dist_from_start:.1f} m, total_travel={self._travel_m:.0f} m. '
                        f'Continuing to lap {self._laps_completed + 1}.')
                    self._max_dist_from_start = 0.0   # re-arm the excursion guard
                return

        if self._car_x >= self._finish_x:
            self._latch_finished(f'position (car_x={self._car_x:.1f} >= finish_x={self._finish_x})')

    def _lidar_cb(self, msg: PointCloud2) -> None:
        if self._finished or self._gate_crossings > 0:
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
            self._latch_finished(f'LiDAR ({int(np.sum(mask))} pts)')

    def _brake_loop(self) -> None:
        if not self._finished:
            return
        cmd = ControlCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'map'
        cmd.throttle = 0.0
        cmd.brake = 1.0
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
