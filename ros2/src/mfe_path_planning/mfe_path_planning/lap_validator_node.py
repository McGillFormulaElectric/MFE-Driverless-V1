#!/usr/bin/env python3
"""
Lap Validator Node.

Subscribes:
    /ground_truth/state         (eufs_msgs/CarState)   — vehicle position and speed
    /ground_truth/track_colored (mfe_msgs/Track)        — full static cone map for hit detection
    /planning/centerline        (nav_msgs/Path)         — planned path for deviation calc
    /planning/mission_finished  (std_msgs/Bool)         — stop logging signal

On each completed lap, prints a summary and appends a row to
~/mfe_logs/validation_<timestamp>.csv with columns:
    lap, lap_time_s, avg_speed_ms, max_speed_ms, avg_deviation_m, max_deviation_m, cones_hit

Also logs a summary line every 5 seconds with current speed and deviation.
"""

import csv
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from mfe_msgs.msg import Track, Cone

try:
    from eufs_msgs.msg import CarState
    _CAR_STATE_AVAILABLE = True
except ImportError:
    _CAR_STATE_AVAILABLE = False

_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# Lap completion thresholds
_LAP_RETURN_RADIUS_M = 5.0   # car must come within this distance of start to close a lap
_LAP_MIN_DIST_M      = 60.0  # car must travel at least this far before a lap can be closed

# Cone hit: standard FSAE cone base ~0.228m radius + car half-width ~0.575m
_CONE_HIT_RADIUS = 0.40      # m — distance from car centre to cone centre to count as a hit

# How often (seconds) to print a status summary
_STATUS_INTERVAL_S = 5.0


class LapValidatorNode(Node):

    def __init__(self):
        super().__init__('lap_validator')

        if not _CAR_STATE_AVAILABLE:
            self.get_logger().error(
                'eufs_msgs is not available — cannot import CarState. '
                'lap_validator_node will not function. '
                'Install eufs_msgs or build the EUFS sim packages.')
            return

        # ---------- CSV output ----------
        log_dir = os.path.expanduser('~/mfe_logs')
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = os.path.join(log_dir, f'validation_{timestamp}.csv')
        self._csv_file = open(self._csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(
            ['lap', 'lap_time_s', 'avg_speed_ms', 'max_speed_ms',
             'avg_deviation_m', 'max_deviation_m', 'cones_hit'])
        self._csv_file.flush()
        self.get_logger().info(f'LapValidatorNode started — logging to {self._csv_path}')

        # ---------- State ----------
        self._mission_finished = False

        # Position tracking
        self._start_pos = None        # (x, y) from first odometry message
        self._last_pos  = None        # (x, y) previous position for dist accumulation

        # Lap counters
        self._lap_count      = 0
        self._lap_start_time = None
        self._lap_total_dist = 0.0

        # Per-lap accumulators
        self._speeds: list[float]        = []
        self._lat_accels: list[float]    = []
        self._path_deviations: list[float] = []

        # Current planned path waypoints (list of (x, y))
        self._current_path: list[tuple[float, float]] = []

        # Full static cone map from /ground_truth/track_colored (Nx2 arrays)
        self._gt_blues:   np.ndarray = np.zeros((0, 2), dtype=np.float32)
        self._gt_yellows: np.ndarray = np.zeros((0, 2), dtype=np.float32)

        # Per-lap cone hit tracking
        self._lap_cones_hit   = 0
        self._total_cones_hit = 0
        self._hit_cooldown    = 0.0   # monotonic time until next hit can register (debounce)

        # Status logging timer
        self._last_status_time = 0.0
        self._current_speed    = 0.0
        self._current_dev      = 0.0

        # ---------- Subscribers ----------
        self.create_subscription(
            CarState, '/ground_truth/state', self._state_cb, _QOS)
        self.create_subscription(
            Track, '/ground_truth/track_colored', self._gt_cones_cb, _QOS)
        self.create_subscription(
            Path, '/planning/centerline', self._centerline_cb, _QOS)
        self.create_subscription(
            Bool, '/planning/mission_finished', self._mission_finished_cb, _QOS)

    # ------------------------------------------------------------------ #

    def _mission_finished_cb(self, msg: Bool) -> None:
        if msg.data and not self._mission_finished:
            self._mission_finished = True
            self.get_logger().info(
                'Mission finished signal received — lap validation logging stopped.')
            self._flush_and_close()

    def _gt_cones_cb(self, msg: Track) -> None:
        """Cache the full GT cone map split by color for hit detection."""
        blues, yellows = [], []
        for c in msg.track:
            if c.color == Cone.BLUE:
                blues.append([c.location.x, c.location.y])
            elif c.color == Cone.YELLOW:
                yellows.append([c.location.x, c.location.y])
        self._gt_blues   = np.array(blues,   dtype=np.float32) if blues   else np.zeros((0, 2), dtype=np.float32)
        self._gt_yellows = np.array(yellows, dtype=np.float32) if yellows else np.zeros((0, 2), dtype=np.float32)

    def _centerline_cb(self, msg: Path) -> None:
        """Cache the latest planned path as a list of (x, y) tuples."""
        self._current_path = [
            (ps.pose.position.x, ps.pose.position.y)
            for ps in msg.poses
        ]

    def _state_cb(self, msg: CarState) -> None:
        if self._mission_finished:
            return

        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Extract speed (magnitude of linear velocity)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.sqrt(vx * vx + vy * vy)

        # Lateral acceleration from y-component of twist (m/s² in body frame)
        lat_accel = abs(msg.twist.twist.linear.y)

        # Initialise on first message
        if self._start_pos is None:
            self._start_pos      = (x, y)
            self._last_pos       = (x, y)
            self._lap_start_time = time.monotonic()
            self._last_status_time = time.monotonic()
            self.get_logger().info(
                f'LapValidator: start position set to ({x:.2f}, {y:.2f})')
            return

        # Distance increment since last callback
        dx = x - self._last_pos[0]
        dy = y - self._last_pos[1]
        dist_inc = math.sqrt(dx * dx + dy * dy)
        self._lap_total_dist += dist_inc
        self._last_pos = (x, y)

        # Compute deviation from planned path
        deviation = self._compute_deviation(x, y)

        # Accumulate per-lap data
        self._speeds.append(speed)
        self._lat_accels.append(lat_accel)
        if deviation is not None:
            self._path_deviations.append(deviation)

        self._current_speed = speed
        self._current_dev   = deviation if deviation is not None else 0.0

        # Cone hit detection against full GT map (debounced to 1 s to avoid double-counting)
        now = time.monotonic()
        if now >= self._hit_cooldown and self._check_cone_hit(x, y):
            self._lap_cones_hit   += 1
            self._total_cones_hit += 1
            self._hit_cooldown = now + 1.0
            self.get_logger().warn(
                f'[LapValidator] CONE HIT at ({x:.1f}, {y:.1f}) — '
                f'lap total: {self._lap_cones_hit}  session total: {self._total_cones_hit}')

        # Periodic status log
        if now - self._last_status_time >= _STATUS_INTERVAL_S:
            self._last_status_time = now
            self.get_logger().info(
                f'[LapValidator] lap={self._lap_count + 1} '
                f'dist={self._lap_total_dist:.1f}m '
                f'speed={speed:.2f}m/s '
                f'deviation={self._current_dev:.3f}m '
                f'cones_hit={self._lap_cones_hit}')

        # Lap completion check
        sx, sy = self._start_pos
        dist_to_start = math.sqrt((x - sx) ** 2 + (y - sy) ** 2)
        if (dist_to_start <= _LAP_RETURN_RADIUS_M
                and self._lap_total_dist >= _LAP_MIN_DIST_M):
            self._complete_lap()

    def _compute_deviation(self, x: float, y: float) -> float | None:
        """Return minimum distance from (x, y) to any waypoint on the current path."""
        if not self._current_path:
            return None
        min_dist = float('inf')
        for (wx, wy) in self._current_path:
            d = math.sqrt((x - wx) ** 2 + (y - wy) ** 2)
            if d < min_dist:
                min_dist = d
        return min_dist

    def _complete_lap(self) -> None:
        """Record lap statistics, print summary, and append to CSV."""
        self._lap_count += 1
        lap_num = self._lap_count

        now = time.monotonic()
        lap_time = now - self._lap_start_time

        avg_speed = (sum(self._speeds) / len(self._speeds)) if self._speeds else 0.0
        max_speed = max(self._speeds) if self._speeds else 0.0

        if self._path_deviations:
            avg_dev = sum(self._path_deviations) / len(self._path_deviations)
            max_dev = max(self._path_deviations)
        else:
            avg_dev = 0.0
            max_dev = 0.0

        cones_hit = self._lap_cones_hit

        summary = (
            f'LAP {lap_num} COMPLETE | '
            f'time={lap_time:.2f}s | '
            f'avg_speed={avg_speed:.2f}m/s | '
            f'max_speed={max_speed:.2f}m/s | '
            f'avg_dev={avg_dev:.3f}m | '
            f'max_dev={max_dev:.3f}m | '
            f'cones_hit={cones_hit}'
        )
        self.get_logger().info(summary)
        print(summary, flush=True)

        self._csv_writer.writerow([
            lap_num,
            f'{lap_time:.3f}',
            f'{avg_speed:.4f}',
            f'{max_speed:.4f}',
            f'{avg_dev:.4f}',
            f'{max_dev:.4f}',
            cones_hit,
        ])
        self._csv_file.flush()

        # Reset per-lap state for the next lap
        self._lap_start_time  = time.monotonic()
        self._lap_total_dist  = 0.0
        self._lap_cones_hit   = 0
        self._speeds.clear()
        self._lat_accels.clear()
        self._path_deviations.clear()

    def _check_cone_hit(self, car_x: float, car_y: float) -> bool:
        """True if car centre is within _CONE_HIT_RADIUS of any cone in the full GT map."""
        r2 = _CONE_HIT_RADIUS ** 2
        for arr in (self._gt_blues, self._gt_yellows):
            if len(arr) == 0:
                continue
            diff = arr - np.array([car_x, car_y], dtype=np.float32)
            if float((diff ** 2).sum(axis=1).min()) < r2:
                return True
        return False

    def _flush_and_close(self) -> None:
        try:
            self._csv_file.flush()
            self._csv_file.close()
        except Exception:
            pass

    def destroy_node(self) -> None:
        self._flush_and_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LapValidatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
