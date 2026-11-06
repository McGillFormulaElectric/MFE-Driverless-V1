#!/usr/bin/env python3
"""
Finish-line detector for FSAE events.

Latches a full brake once any enabled check fires:

1. Orange-gate passes (gate_crossings > 0): clusters perceived ORANGE_BIG cones from /planning/cones into
   gates and counts car passes through them. Finish after gate_crossings passes (+ stop_after_gate_m).
2. Timing line (line_crossing, closed loops): the line is the one through the orange cones at the start
   (orange_timing_line), or a virtual line line_offset_m ahead of the start pose when no orange pair is known
   by the time min_travel_m is reached. Each signed-side crossing of the line counts a lap; on the final lap the
   brake latches stop_after_line_m of travel after the crossing.
3. Return-to-start (legacy, line_crossing off): counts lap crossings of the start line (within return_to_start_r,
   heading-aligned, after min_travel_m and a 15 m excursion). Finish after num_laps crossings.
4. LiDAR (linear tracks, lidar_enabled): dense returns in the forward zone past approach_x.
5. Position fallback (position_fallback): odometry x >= finish_x.

On finish it latches /planning/mission_finished (pure pursuit brake) at once and publishes
/ros_can/mission_completed only once the car is at standstill. Full brake goes on /control/command at 50 Hz.
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32
from fs_msgs.msg import ControlCommand
from mfe_msgs.msg import Track, Cone
from mfe_common.geometry import yaw_from_quaternion
from mfe_path_planning.core.finish import (
    LAP_EXCURSION_M, REANCHOR_LOG_M, STANDSTILL_MPS, STOP_AFTER_LINE_MAX_M, extract_xyz, gate_pass_step,
    line_crossed,
    lidar_forward_count, orange_timing_line, rebuild_gate_states, stop_travel_after_line,
)
from mfe_common.qos import LATCHED_QOS, RELIABLE_QOS, SHALLOW_SENSOR_QOS
from mfe_common.topics import (
    CONTROL_COMMAND,
    EKF_OUTPUT,
    LIDAR_POINTS_RAW,
    PLANNING_CONES,
    PLANNING_LAPS_COMPLETED,
    PLANNING_MISSION_FINISHED,
    ROS_CAN_MISSION_COMPLETED,
    SKIDPAD_CONES,
)


class FinishDetectorNode(Node):
    """Detects the finish line and latches a full brake command."""

    def __init__(self):
        super().__init__('finish_detector')

        self.declare_parameter('approach_x', 60.0)
        self.declare_parameter('finish_x', 78.0)          # position fallback trigger
        self.declare_parameter('position_fallback', True)  # False: finish_x never latches (closed loops, skidpad)
        self.declare_parameter('min_travel_m', 0.0)       # cumulative travel before lap detection
        self.declare_parameter('return_to_start_r', 0.0)  # >0 enables return-to-start lap counting
        self.declare_parameter('detect_fwd_min_m', 1.5)
        self.declare_parameter('detect_fwd_max_m', 15.0)
        self.declare_parameter('detect_lateral_m', 4.0)
        self.declare_parameter('min_finish_points', 1)
        self.declare_parameter('lidar_enabled', True)     # False: no LiDAR forward-zone finish trigger
        self.declare_parameter('num_laps', 1)
        self.declare_parameter('gate_crossings', 0)       # >0 enables orange-gate counting
        self.declare_parameter('stop_after_gate_m', 0.0)  # run-out after the final gate pass
        self.declare_parameter('gate_enter_r', 4.5)       # gate pass = within this radius of centroid
        self.declare_parameter('line_crossing', False)    # True: lap = signed crossing of the orange timing line
        self.declare_parameter('orange_topic', PLANNING_CONES)   # BIG and SMALL orange cones for the timing line
        self.declare_parameter('line_search_r', 25.0)     # orange pylons this close to the start pose are used
        self.declare_parameter('line_margin_m', 2.0)      # timing line width = pylon separation + 2 * margin
        self.declare_parameter('line_offset_m', 6.79)     # fallback virtual line: start + this, along start heading
        self.declare_parameter('stop_after_line_m', 5.0)  # run-out from the final line crossing before braking
        # Re-anchor the timing line on lap return: after the car has been >= LAP_EXCURSION_M from the start, the
        # orange pylons seen near the car (fresh message) replace the stored line. Drift moves the stored line.
        self.declare_parameter('finish_line_reanchor', True)
        # Re-anchor only while the car is within this radius of the START pose and has travelled > min_travel_m.
        # Mid-lap re-anchoring (observed: re-anchored at car x=31.9 m during lap 1) moved the line off the
        # real start line. Pylons are also ignored when farther than line_search_r from the stored line.
        self.declare_parameter('reanchor_start_r_m', 20.0)

        self._approach_x = self.get_parameter('approach_x').value
        self._finish_x = self.get_parameter('finish_x').value
        self._position_fallback = bool(self.get_parameter('position_fallback').value)
        self._min_travel_m = self.get_parameter('min_travel_m').value
        self._return_r = self.get_parameter('return_to_start_r').value
        self._fwd_min = self.get_parameter('detect_fwd_min_m').value
        self._fwd_max = self.get_parameter('detect_fwd_max_m').value
        self._lat_max = self.get_parameter('detect_lateral_m').value
        self._min_pts = self.get_parameter('min_finish_points').value
        self._lidar_enabled = bool(self.get_parameter('lidar_enabled').value)
        self._num_laps = max(1, self.get_parameter('num_laps').value)
        self._gate_crossings = self.get_parameter('gate_crossings').value
        self._stop_after_gate_m = self.get_parameter('stop_after_gate_m').value
        self._gate_enter_r = self.get_parameter('gate_enter_r').value
        self._line_crossing = bool(self.get_parameter('line_crossing').value)
        self._orange_topic = str(self.get_parameter('orange_topic').value)
        self._line_search_r = float(self.get_parameter('line_search_r').value)
        self._line_margin_m = float(self.get_parameter('line_margin_m').value)
        self._line_offset_m = float(self.get_parameter('line_offset_m').value)
        self._stop_after_line_m = float(self.get_parameter('stop_after_line_m').value)
        self._line_reanchor = bool(self.get_parameter('finish_line_reanchor').value)
        self._reanchor_start_r = float(self.get_parameter('reanchor_start_r_m').value)
        if self._stop_after_line_m > STOP_AFTER_LINE_MAX_M:
            self.get_logger().warn(f'stop_after_line_m={self._stop_after_line_m} exceeds {STOP_AFTER_LINE_MAX_M} m '
                                   f'(DD.4.4.4); clamped')

        self._car_x = 0.0
        self._car_y = 0.0
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
        self._completed_sent = False      # mission_completed published (at standstill)
        # Lap line in the start frame: (long, lat, width). Return-to-start mode: the start line itself. Timing-line
        # mode: None until the orange pair is known, or the virtual fallback at min_travel_m.
        if self._line_crossing:
            self._line = None
        else:
            self._line = (0.0, 0.0, 2.0 * self._return_r) if self._return_r > 0.0 else None
        self._line_locked = False         # line fixed: a lap has counted on it, or the fallback was taken
        self._line_reanchored = False     # the line now comes from fresh pylons near the car (start-pose search off)

        self._gate_cones = {}     # 1 m grid cell -> np[x, y] of accumulated ORANGE_BIG pylons
        self._gate_states = []    # per gate cluster: {'c': np[x, y], 'inside': bool | None}
        self._gate_passes = 0
        self._stop_at_travel = None
        self._orange_pts = {}     # 1 m grid cell -> (x, y) of ORANGE_BIG and ORANGE_SMALL cones (timing line)

        sensor_qos = SHALLOW_SENSOR_QOS
        reliable_qos = RELIABLE_QOS
        latched_qos = LATCHED_QOS

        self.create_subscription(PointCloud2, LIDAR_POINTS_RAW, self._lidar_cb, sensor_qos)
        self.create_subscription(Odometry, EKF_OUTPUT, self._odom_cb, reliable_qos)
        if self._gate_crossings > 0:
            self.create_subscription(Track, PLANNING_CONES, self._cones_cb, sensor_qos)
            # skidpad runs publish the cone map on SKIDPAD_CONES
            self.create_subscription(Track, SKIDPAD_CONES, self._cones_cb, sensor_qos)
            if self._orange_topic not in (PLANNING_CONES, SKIDPAD_CONES):
                # orange_only colour mode: ORANGE_BIG pylons are published on orange_topic, not on /planning/cones
                self.create_subscription(Track, self._orange_topic, self._cones_cb, sensor_qos)
        elif self._line_crossing:
            self.create_subscription(Track, self._orange_topic, self._cones_cb, sensor_qos)

        self._mission_completed_pub = self.create_publisher(
            Bool, ROS_CAN_MISSION_COMPLETED, reliable_qos)
        self._finished_pub = self.create_publisher(
            Bool, PLANNING_MISSION_FINISHED, latched_qos)
        self._cmd_pub = self.create_publisher(ControlCommand, CONTROL_COMMAND, reliable_qos)
        self._laps_pub = self.create_publisher(Int32, PLANNING_LAPS_COMPLETED, reliable_qos)

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
            f'line_crossing={self._line_crossing} (stop_after_line={self._stop_after_line_m} m, '
            f'fallback offset={self._line_offset_m} m) | '
            f'min_travel={self._min_travel_m} m | '
            f'return_to_start_r={self._return_r} m | '
            f'LiDAR={self._lidar_enabled} range=[{self._fwd_min},{self._fwd_max}] m | '
            f'position fallback={self._position_fallback} finish_x={self._finish_x} m')

    def _latch_finished(self, source: str) -> None:
        if self._finished:
            return
        self.get_logger().info(
            f'Finish line detected ({source}) at car_x={self._car_x:.1f} m — BRAKING!')
        self._finished = True
        done = Bool()
        done.data = True
        self._finished_pub.publish(done)            # -> pure_pursuit (brake now)

    def _check_standstill(self, speed: float) -> None:
        """mission_completed (EUFS state machine) goes out only once the car has stopped."""
        if self._finished and not self._completed_sent and speed < STANDSTILL_MPS:
            done = Bool()
            done.data = True
            self._mission_completed_pub.publish(done)
            self._completed_sent = True
            self.get_logger().info(f'Car at standstill (v={speed:.2f} m/s) — mission_completed')

    def _publish_count(self, value: int) -> None:
        msg = Int32()
        msg.data = value
        self._laps_pub.publish(msg)

    def _line_center_map(self):
        """Map-frame (x, y) of the stored timing line's midpoint, or None."""
        if self._line is None or self._start_x is None:
            return None
        long, lat, _ = self._line
        c, s = math.cos(self._start_yaw), math.sin(self._start_yaw)
        return (self._start_x + c * long - s * lat, self._start_y + s * long + c * lat)

    def _cones_cb(self, msg: Track) -> None:
        if self._finished:
            return
        # Lap-return re-anchor: once the car has been away from the start, the orange pylons in this message
        # near the car give the line in the current frame (the stored line carries the SLAM drift). No fresh pair:
        # the stored line stays. Guard: the car must be near the START pose after min_travel, and only pylons within
        # line_search_r of the stored line are used.
        near_start = (self._start_x is not None
                      and math.hypot(self._car_x - self._start_x, self._car_y - self._start_y)
                      <= self._reanchor_start_r)
        centre = self._line_center_map() if near_start and self._travel_m > self._min_travel_m else None
        if (self._line_crossing and self._line_reanchor and self._start_x is not None
                and self._stop_at_travel is None and self._max_dist_from_start >= LAP_EXCURSION_M
                and centre is not None):
            fresh = [(cone.location.x, cone.location.y) for cone in msg.track
                     if cone.color in (Cone.ORANGE_BIG, Cone.ORANGE_SMALL)
                     and math.hypot(cone.location.x - centre[0], cone.location.y - centre[1]) <= self._line_search_r]
            line = orange_timing_line(fresh, self._start_x, self._start_y, self._start_yaw,
                                      self._line_search_r, self._line_margin_m, center=centre) \
                if fresh else None
            if line is not None:
                moved = (max(abs(line[i] - self._line[i]) for i in range(3))
                         if self._line is not None else float('inf'))
                if moved > REANCHOR_LOG_M:
                    old = self._line
                    self.get_logger().info(
                        f'Orange timing line re-anchored near car=({self._car_x:.1f},{self._car_y:.1f}): '
                        f'long={line[0]:.2f} lat={line[1]:.2f} width={line[2]:.2f} m (start frame)'
                        + ('' if old is None else f', was long={old[0]:.2f} lat={old[1]:.2f} width={old[2]:.2f}'))
                self._line = line
                self._line_reanchored = True
        if self._line_crossing and not self._line_locked and not self._line_reanchored and self._start_x is not None:
            changed = False
            for cone in msg.track:
                if cone.color not in (Cone.ORANGE_BIG, Cone.ORANGE_SMALL):
                    continue
                key = (round(cone.location.x), round(cone.location.y))
                if key not in self._orange_pts:
                    self._orange_pts[key] = (cone.location.x, cone.location.y)
                    changed = True
            if changed:
                line = orange_timing_line(self._orange_pts.values(), self._start_x, self._start_y, self._start_yaw,
                                          self._line_search_r, self._line_margin_m)
                if line is not None and line != self._line:
                    self._line = line
                    self.get_logger().info(f'Orange timing line: long={line[0]:.2f} lat={line[1]:.2f} '
                                           f'width={line[2]:.2f} m (start frame)')
        if self._gate_crossings <= 0:
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
        """Cluster the pylons into gates, carrying each gate's pass state over (core.finish.rebuild_gate_states)."""
        self._gate_states = rebuild_gate_states(self._gate_cones.values(), self._gate_states)

    def _check_gate_passes(self, x: float, y: float) -> None:
        for g in self._gate_states:
            d = math.hypot(x - g['c'][0], y - g['c'][1])
            if gate_pass_step(g, d, self._travel_m, self._gate_enter_r):
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
        if self._stop_at_travel is not None and self._travel_m >= self._stop_at_travel:
            self._latch_finished(f'orange gate + {self._stop_after_gate_m:.0f} m run-out')

    def _odom_cb(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._car_x = x
        self._car_y = y
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q)
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
            self._check_standstill(speed)
            return

        dist_from_start = math.hypot(x - self._start_x, y - self._start_y)
        if dist_from_start > self._max_dist_from_start:
            self._max_dist_from_start = dist_from_start

        if self._stop_at_travel is not None and self._travel_m >= self._stop_at_travel:
            self._latch_finished(f'timing line + {self._stop_after_line_m:.0f} m run-out')
            self._check_standstill(speed)
            return

        if self._gate_crossings > 0:
            self._check_gate_passes(x, y)
            if self._finished:
                self._check_standstill(speed)
                return

        # Virtual timing line when no orange pair is known by the time a lap can count (lap 1 is armed at min_travel_m).
        if self._line_crossing and self._line is None and self._travel_m >= self._min_travel_m:
            self._line = (self._line_offset_m, 0.0, 2.0 * self._return_r if self._return_r > 0.0 else 10.0)
            self._line_locked = True
            self.get_logger().info(f'No orange pair known at min_travel: virtual line at {self._line_offset_m} m '
                                   f'along the start heading')

        # A lap counts when long crosses from behind to past the line, within its width, with the
        # heading within 60 deg of the start heading. Distance alone fired early on peanut.
        if ((self._return_r > 0.0 or self._line_crossing) and self._line is not None and self._stop_at_travel is None
                and self._travel_m >= self._min_travel_m and self._max_dist_from_start >= LAP_EXCURSION_M):
            heading_ok = math.cos(yaw - self._start_yaw) > 0.5
            if line_crossed(prev_long, long, lat, self._line, heading_ok):
                self._laps_completed += 1
                self._publish_count(self._laps_completed)
                self._line_locked = True

                if self._laps_completed >= self._num_laps:
                    if self._line_crossing and self._stop_after_line_m > 0.0:
                        self._stop_at_travel = stop_travel_after_line(self._travel_m, self._stop_after_line_m)
                        self.get_logger().info(
                            f'Final timing line crossing (lap {self._laps_completed}/{self._num_laps}, '
                            f'travel={self._travel_m:.0f} m) — braking after {self._stop_after_line_m:.1f} m run-out.')
                    else:
                        self._latch_finished(
                            f'timing line lap {self._laps_completed}/{self._num_laps} '
                            f'(dist={dist_from_start:.1f} m, travel={self._travel_m:.0f} m)')
                        self._check_standstill(speed)
                        return
                else:
                    self.get_logger().info(
                        f'Lap {self._laps_completed}/{self._num_laps} complete '
                        f'— dist={dist_from_start:.1f} m, total_travel={self._travel_m:.0f} m. '
                        f'Continuing to lap {self._laps_completed + 1}.')
                    self._max_dist_from_start = 0.0   # re-arm the excursion guard
                return

        if self._position_fallback and self._car_x >= self._finish_x:
            self._latch_finished(f'position (car_x={self._car_x:.1f} >= finish_x={self._finish_x})')
            self._check_standstill(speed)

    def _lidar_cb(self, msg: PointCloud2) -> None:
        if self._finished or self._gate_crossings > 0 or not self._lidar_enabled:
            return
        if self._min_travel_m > 0.0:
            if self._travel_m < self._min_travel_m:
                return
        elif self._car_x < self._approach_x:
            return
        pts = extract_xyz(msg)
        if pts.shape[0] == 0:
            return
        n = lidar_forward_count(pts, self._fwd_min, self._fwd_max, self._lat_max)
        if n >= self._min_pts:
            self._latch_finished(f'LiDAR ({n} pts)')

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
