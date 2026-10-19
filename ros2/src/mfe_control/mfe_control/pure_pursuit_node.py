"""Pure pursuit controller node for the MFE FSAE driverless car."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry, Path
from fs_msgs.msg import ControlCommand
from std_msgs.msg import Bool, Header, Float32, Float64MultiArray

from mfe_control.pursuit_math import (
    Pt2Anticipator,
    curvature_speed_target,
    find_lookahead,
    local_curvature,
    path_end_speed_cap,
    pi_throttle_brake,
    pure_pursuit_steering,
    remaining_length,
    smooth_path,
    yaw_from_quaternion,
)

_PARAM_DEFAULTS = {
    'K_v': 0.5,                       # look-ahead gain (m per m/s)
    'ld_min': 0.5,
    'ld_max': 15.0,
    'max_speed': 10.0,
    'wheelbase': 1.56,
    'rear_axle_offset': 0.0,          # pose origin to rear axle (m); pure pursuit geometry is about the rear axle
    'max_steering_deg': 25.0,
    'map_frame': 'map',
    'max_lateral_accel': 8.0,
    'max_deceleration': 10.0,
    'path_end_decel_frac': 0.6,
    'path_end_margin_m': 1.5,
    'lookahead_waypoints': 40,        # corner-speed scan length (points)
    'speed_smooth_window': 1,         # moving-average window for the speed scan only; 1 = off
    'control_rate_hz': 50.0,
    'steer_pt2_tau': 0.05,
    'steer_pt2_zeta': 0.7,
    'steer_pt2_filter_tau': 0.012,
    'speed_kp': 0.5,
    'speed_ki': 0.1,
    'speed_kb': 0.25,                 # brake per m/s of overspeed
    'brake_deadband_mps': 0.3,
}


class PurePursuitNode(Node):
    """
    Steering: pure pursuit with adaptive look-ahead, plus a PT2 feedforward on the steering command
    to compensate actuator lag.
    Speed: planner target_speeds when length-matched, otherwise a corner-speed profile over the next
    lookahead_waypoints. A PI loop converts the target speed into throttle or brake.

    Subscribes to:
        /planning/centerline        (nav_msgs/Path)
        /ekf/output                 (nav_msgs/Odometry)
        /planning/mission_finished  (std_msgs/Bool)
        /planning/path_horizon      (std_msgs/Float32)
        /planning/target_speeds     (std_msgs/Float64MultiArray)

    Publishes:
        /control/command            (fs_msgs/ControlCommand)
    """

    def __init__(self):
        super().__init__('pure_pursuit_node')
        for name, default in _PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name in _PARAM_DEFAULTS}

        self._K_v = p['K_v']
        self._ld_min = p['ld_min']
        self._ld_max = p['ld_max']
        self._max_speed = p['max_speed']
        self._wheelbase = p['wheelbase']
        self._rear_axle_offset = p['rear_axle_offset']
        self._max_steering_rad = math.radians(p['max_steering_deg'])
        self._map_frame = p['map_frame']
        self._max_lateral_accel = p['max_lateral_accel']
        self._max_deceleration = p['max_deceleration']
        self._path_end_decel = p['path_end_decel_frac'] * p['max_deceleration']
        self._path_end_margin = p['path_end_margin_m']
        self._lookahead_waypoints = p['lookahead_waypoints']
        self._speed_smooth_window = max(1, int(p['speed_smooth_window']))
        self._speed_kp = p['speed_kp']
        self._speed_ki = p['speed_ki']
        self._speed_kb = p['speed_kb']
        self._brake_deadband = p['brake_deadband_mps']

        self._dt = 1.0 / float(p['control_rate_hz'])
        self._pt2 = Pt2Anticipator(p['steer_pt2_tau'], p['steer_pt2_zeta'], p['steer_pt2_filter_tau'], self._dt)

        self._path = None
        self._speed_path = None   # smoothed copy of _path used by the speed scan
        self._car_x = None
        self._car_y = None
        self._car_yaw = None
        self._car_speed = None
        self._mission_finished = False
        self._path_idx = 0
        self._target_speeds = []
        self._horizon_time = None  # node-clock receipt time of the last /planning/path_horizon
        self._path_time = None     # ... and of the last path
        self._speed_integral = 0.0

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        best_effort_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Path, '/planning/centerline', self._path_callback, reliable_qos)
        self.create_subscription(Odometry, '/ekf/output', self._odom_callback, reliable_qos)
        self.create_subscription(Bool, '/planning/mission_finished', self._finished_callback, reliable_qos)
        self.create_subscription(Float32, '/planning/path_horizon', self._horizon_callback, reliable_qos)
        self.create_subscription(
            Float64MultiArray, '/planning/target_speeds', self._target_speeds_callback, best_effort_qos)

        self._cmd_pub = self.create_publisher(ControlCommand, '/control/command', reliable_qos)
        self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f'pure_pursuit_node started | rate={1.0 / self._dt:.0f} Hz  '
            f'K_v={self._K_v}  ld=[{self._ld_min},{self._ld_max}] m  '
            f'max_speed={self._max_speed} m/s  '
            f'a_lat={self._max_lateral_accel} m/s²  a_brake={self._max_deceleration} m/s²  '
            f'scan={self._lookahead_waypoints} wp  '
            f'PT2 tau={p["steer_pt2_tau"]} zeta={p["steer_pt2_zeta"]} filter_tau={p["steer_pt2_filter_tau"]}  '
            f'PI Kp={self._speed_kp} Ki={self._speed_ki}'
        )

    # Callbacks

    def _finished_callback(self, msg: Bool):
        if msg.data and not self._mission_finished:
            self.get_logger().info('Mission finished — pure pursuit stopped.')
            self._mission_finished = True

    def _horizon_callback(self, msg: Float32) -> None:
        self._horizon_time = self.get_clock().now().nanoseconds * 1e-9

    def _target_speeds_callback(self, msg: Float64MultiArray) -> None:
        self._target_speeds = list(msg.data)

    def _path_callback(self, msg: Path):
        if not msg.poses:
            return
        self._path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._path_time = self.get_clock().now().nanoseconds * 1e-9
        self._path_idx = 0
        self._speed_path = smooth_path(self._path, self._speed_smooth_window)

    def _odom_callback(self, msg: Odometry):
        self._car_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self._car_x = msg.pose.pose.position.x - self._rear_axle_offset * math.cos(self._car_yaw)
        self._car_y = msg.pose.pose.position.y - self._rear_axle_offset * math.sin(self._car_yaw)
        self._car_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    # Control loop

    def _control_loop(self):
        if self._mission_finished:
            return

        if self._path is None or self._car_x is None:
            self.get_logger().warn('Waiting for path/odometry...', throttle_duration_sec=2.0)
            self._reset_controller_state()
            self._publish_command(0.0, 0.0, 0.0)
            return

        # Look-ahead distance from the local curvature, sampled before the look-ahead search.
        kappa = local_curvature(self._path, self._path_idx)
        ld = self._K_v * self._car_speed * math.sqrt(1.0 + kappa ** 2)
        ld = max(self._ld_min, min(self._ld_max, ld))

        target, self._path_idx = find_lookahead(
            self._path, self._path_idx, self._car_x, self._car_y, ld)
        steering_norm = pure_pursuit_steering(
            self._car_x, self._car_y, self._car_yaw, target, self._wheelbase, self._max_steering_rad)
        steering_cmd = self._pt2(steering_norm)

        v_target = self._cap_to_path_end(self._compute_target_speed())
        throttle, brake, self._speed_integral = pi_throttle_brake(
            v_target, self._car_speed, self._speed_integral,
            self._speed_kp, self._speed_ki, self._speed_kb, self._brake_deadband, self._dt)

        self._publish_command(steering_cmd, throttle, brake)

    def _compute_target_speed(self) -> float:
        if (self._target_speeds
                and len(self._target_speeds) == len(self._path)
                and self._path_idx < len(self._target_speeds)):
            return max(0.5, min(float(self._target_speeds[self._path_idx]), self._max_speed))
        return curvature_speed_target(
            self._speed_path or self._path, self._path_idx, self._lookahead_waypoints,
            self._car_speed, self._max_speed, self._max_lateral_accel, self._max_deceleration)

    def _cap_to_path_end(self, v_target: float) -> float:
        """Limit speed so the car can stop before the end of a horizon-truncated path."""
        if self._horizon_time is None:
            return v_target
        if self._path_time - self._horizon_time > 1.0:
            return v_target   # paths now come without horizons (e.g. a locked full-lap path)
        # No new path at all keeps the cap: the car stops at the end of the last one.
        v_stop = path_end_speed_cap(
            remaining_length(self._path, self._path_idx), self._path_end_decel, self._path_end_margin)
        return min(v_target, v_stop)

    def _reset_controller_state(self):
        self._pt2.reset_history()
        self._speed_integral = 0.0

    def _publish_command(self, steering: float, throttle: float, brake: float):
        cmd = ControlCommand()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self._map_frame
        cmd.steering = float(steering)
        cmd.throttle = float(throttle)
        cmd.brake = float(brake)
        self._cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
