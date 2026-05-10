#!/usr/bin/env python3
"""
Finish-line detector for FSAE events.

Detection hierarchy:
1. Return-to-start (closed-loop tracks): once the car has covered min_travel_m and
   ventured at least 15 m from start, it counts a lap crossing each time it returns
   within return_to_start_r metres of the start position.  After num_laps crossings
   the mission is finished.  The 15 m excursion guard prevents false positives at
   the peanut figure-8 crossing where the car briefly passes near the origin.

2. LiDAR (linear tracks / final-lap gate): watches /lidar/points_raw for dense
   returns in the forward zone once the car passes approach_x.  Orange finish cones
   show up as a cluster 1.5–15 m ahead.

3. Position fallback: if odometry x exceeds finish_x we declare complete.  Fires
   when the Gazebo VLP-16 fails to return points from the orange cones.

When finish is detected:
  - Publishes /ros_can/mission_completed (Bool True) so the EUFS state machine
    transitions to MISSION_COMPLETED:TRUE on /ros_can/state_str.
  - Publishes /planning/mission_finished (Bool True, latched) so pure_pursuit stops.
  - Publishes full brake to /control/command at 50 Hz.
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


class FinishDetectorNode(Node):
    """Detects finish line and latches a full brake command."""

    def __init__(self):
        super().__init__('finish_detector')

        # ---------- Parameters ----------
        self.declare_parameter('approach_x', 60.0)
        self.declare_parameter('finish_x', 78.0)       # position fallback trigger
        self.declare_parameter('min_travel_m', 0.0)    # cumulative travel before first lap detection
        self.declare_parameter('return_to_start_r', 0.0)  # >0: lap gate radius around start position
        self.declare_parameter('detect_fwd_min_m', 1.5)
        self.declare_parameter('detect_fwd_max_m', 15.0)
        self.declare_parameter('detect_lateral_m', 4.0)
        self.declare_parameter('min_finish_points', 1)
        self.declare_parameter('num_laps', 1)          # stop after this many lap crossings

        self._approach_x        = self.get_parameter('approach_x').value
        self._finish_x          = self.get_parameter('finish_x').value
        self._min_travel_m      = self.get_parameter('min_travel_m').value
        self._return_r          = self.get_parameter('return_to_start_r').value
        self._fwd_min           = self.get_parameter('detect_fwd_min_m').value
        self._fwd_max           = self.get_parameter('detect_fwd_max_m').value
        self._lat_max           = self.get_parameter('detect_lateral_m').value
        self._min_pts           = self.get_parameter('min_finish_points').value
        self._num_laps          = max(1, self.get_parameter('num_laps').value)

        self._car_x               = 0.0
        self._start_x             = None   # recorded on first odometry message
        self._start_y             = None
        self._prev_x              = None
        self._prev_y              = None
        self._travel_m            = 0.0
        self._max_dist_from_start = 0.0    # reset each lap to re-arm the 15 m excursion guard
        self._laps_completed      = 0      # gate crossings so far
        self._finished            = False

        # ---------- QoS ----------
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

        # ---------- Subscribers ----------
        self.create_subscription(PointCloud2, '/lidar/points_raw', self._lidar_cb, sensor_qos)
        self.create_subscription(Odometry, '/ekf/output', self._odom_cb, reliable_qos)

        # ---------- Publishers ----------
        self._mission_completed_pub = self.create_publisher(
            Bool, '/ros_can/mission_completed', reliable_qos)
        self._finished_pub = self.create_publisher(
            Bool, '/planning/mission_finished', latched_qos)
        self._cmd_pub = self.create_publisher(
            ControlCommand, '/control/command', reliable_qos)
        # Lap counter — visible in Foxglove as /planning/laps_completed
        self._laps_pub = self.create_publisher(
            Int32, '/planning/laps_completed', reliable_qos)

        # Clear any stale latched True from a previous run
        init_msg = Bool()
        init_msg.data = False
        self._finished_pub.publish(init_msg)

        # 50 Hz brake loop — faster than pure_pursuit (20 Hz) so brake dominates
        self.create_timer(0.02, self._brake_loop)

        self.get_logger().info(
            f'FinishDetectorNode started | num_laps={self._num_laps} | '
            f'min_travel={self._min_travel_m} m | '
            f'return_to_start_r={self._return_r} m | '
            f'LiDAR range=[{self._fwd_min},{self._fwd_max}] m | '
            f'position fallback: finish_x={self._finish_x} m')

    # ------------------------------------------------------------------ #

    def _latch_finished(self, source: str) -> None:
        if self._finished:
            return
        self.get_logger().info(
            f'Finish line detected ({source}) at car_x={self._car_x:.1f} m — BRAKING!')
        self._finished = True

        done = Bool()
        done.data = True
        self._mission_completed_pub.publish(done)   # → EUFS state machine
        self._finished_pub.publish(done)             # → pure_pursuit

    def _odom_cb(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._car_x = x

        if self._start_x is None:
            self._start_x, self._start_y = x, y

        if self._prev_x is not None:
            self._travel_m += math.hypot(x - self._prev_x, y - self._prev_y)
        self._prev_x, self._prev_y = x, y

        if self._finished:
            return

        dist_from_start = math.hypot(x - self._start_x, y - self._start_y)
        if dist_from_start > self._max_dist_from_start:
            self._max_dist_from_start = dist_from_start

        # Return-to-start gate: counts lap crossings at the orange-cone start/finish gate.
        # Guards:
        #   min_travel_m  — must cover at least one partial lap before first crossing counts
        #   max_dist >= 15 m — reset each lap; prevents firing at the figure-8 midpoint
        #                      crossing where the car briefly passes near the origin
        if self._return_r > 0.0 and self._travel_m >= self._min_travel_m and self._max_dist_from_start >= 15.0:
            if dist_from_start <= self._return_r:
                self._laps_completed += 1
                lap_msg = Int32()
                lap_msg.data = self._laps_completed
                self._laps_pub.publish(lap_msg)

                if self._laps_completed >= self._num_laps:
                    self._latch_finished(
                        f'return-to-start lap {self._laps_completed}/{self._num_laps} '
                        f'(dist={dist_from_start:.1f} m, travel={self._travel_m:.0f} m)')
                else:
                    self.get_logger().info(
                        f'Lap {self._laps_completed}/{self._num_laps} complete '
                        f'— dist={dist_from_start:.1f} m, total_travel={self._travel_m:.0f} m. '
                        f'Continuing to lap {self._laps_completed + 1}.')
                    # Re-arm excursion guard so the next lap crossing only counts after the
                    # car has ventured 15 m away from start again.
                    self._max_dist_from_start = 0.0
                return

        # Position-based fallback (linear tracks / sim VLP-16 LiDAR gap)
        if self._car_x >= self._finish_x:
            self._latch_finished(f'position (car_x={self._car_x:.1f} >= finish_x={self._finish_x})')

    def _lidar_cb(self, msg: PointCloud2) -> None:
        if self._finished:
            return
        # Use travel distance gate if configured, else fall back to x-position gate
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
