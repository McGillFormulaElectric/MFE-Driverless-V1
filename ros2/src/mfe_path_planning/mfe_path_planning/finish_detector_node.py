#!/usr/bin/env python3
"""
Finish-line detector for FSAE events.

Detection hierarchy:
1. LiDAR (primary perception): watches /lidar/points_raw for dense returns in the
   forward zone once the car passes approach_x.  Orange finish cones show up as a
   cluster 1.5–15 m ahead.

2. Position fallback: if the car's odometry x exceeds finish_x (track parameter),
   we declare the mission complete.  This fires when the Gazebo VLP-16 model fails
   to return points from the orange cones (a known sim limitation).

When finish is detected:
  - Publishes /ros_can/mission_completed (Bool True) so the EUFS state machine
    transitions to MISSION_COMPLETED:TRUE on /ros_can/state_str.
  - Publishes /planning/mission_finished (Bool True, latched) so pure_pursuit stops.
  - Publishes full brake to /control/command at 50 Hz.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
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
        self.declare_parameter('detect_fwd_min_m', 1.5)
        self.declare_parameter('detect_fwd_max_m', 15.0)
        self.declare_parameter('detect_lateral_m', 4.0)
        self.declare_parameter('min_finish_points', 1)

        self._approach_x = self.get_parameter('approach_x').value
        self._finish_x   = self.get_parameter('finish_x').value
        self._fwd_min    = self.get_parameter('detect_fwd_min_m').value
        self._fwd_max    = self.get_parameter('detect_fwd_max_m').value
        self._lat_max    = self.get_parameter('detect_lateral_m').value
        self._min_pts    = self.get_parameter('min_finish_points').value

        self._car_x      = 0.0
        self._finished   = False

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
        # Tell EUFS state machine the mission is done
        self._mission_completed_pub = self.create_publisher(
            Bool, '/ros_can/mission_completed', reliable_qos)
        # Tell pure_pursuit to stop
        self._finished_pub = self.create_publisher(
            Bool, '/planning/mission_finished', latched_qos)
        # Brake command (50 Hz, overrides pure_pursuit at 20 Hz)
        self._cmd_pub = self.create_publisher(
            ControlCommand, '/control/command', reliable_qos)

        # Clear any stale latched True from a previous run so new subscribers start fresh
        init_msg = Bool()
        init_msg.data = False
        self._finished_pub.publish(init_msg)

        # 50 Hz brake loop — faster than pure_pursuit (20 Hz) so brake dominates
        self.create_timer(0.02, self._brake_loop)

        self.get_logger().info(
            f'FinishDetectorNode started | '
            f'LiDAR: approach_x={self._approach_x} m, range=[{self._fwd_min},{self._fwd_max}] m | '
            f'Position fallback: finish_x={self._finish_x} m')

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
        self._car_x = msg.pose.pose.position.x

        # Position-based fallback: fires when LiDAR detection was missed
        if not self._finished and self._car_x >= self._finish_x:
            self._latch_finished(f'position (car_x={self._car_x:.1f} >= finish_x={self._finish_x})')

    def _lidar_cb(self, msg: PointCloud2) -> None:
        if self._finished or self._car_x < self._approach_x:
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
