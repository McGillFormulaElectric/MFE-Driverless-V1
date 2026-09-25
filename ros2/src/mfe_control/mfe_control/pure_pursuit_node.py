"""Pure pursuit controller node for MFE FSAE driverless car."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry, Path
from fs_msgs.msg import ControlCommand
from std_msgs.msg import Bool, Header, Float64MultiArray


class PurePursuitNode(Node):
    """
    Pure pursuit lateral controller with lookahead velocity profile.

    Steering:  pure pursuit geometry (classic) + PT2 feedforward anticipator
               to compensate ~50ms actuator lag.
    Speed:     scan upcoming path curvature → compute corner speed targets
               → brake hard if within stopping distance, full throttle otherwise.
               PI closed-loop velocity controller converts v_target to throttle.

    Subscribes to:
        /planning/centerline  (nav_msgs/Path)    — waypoints in map frame
        /ekf/output           (nav_msgs/Odometry) — vehicle pose + velocity

    Publishes:
        /control/command      (fs_msgs/ControlCommand)
    """

    def __init__(self):
        super().__init__('pure_pursuit_node')

        # --- Parameters ---
        self.declare_parameter('lookahead_distance', 5.0)
        self.declare_parameter('max_speed', 10.0)
        self.declare_parameter('wheelbase', 1.56)
        self.declare_parameter('max_steering_deg', 25.0)
        self.declare_parameter('map_frame', 'map')

        # Velocity profile parameters
        self.declare_parameter('max_lateral_accel', 8.0)
        self.declare_parameter('max_deceleration', 10.0)
        self.declare_parameter('lookahead_waypoints', 40)

        # Fallback for when car speed is unknown (startup)
        self.declare_parameter('speed_reduction_factor', 0.3)

        # PT2 steer delay compensation: tau (s), zeta (damping ratio)
        self.declare_parameter('steer_pt2_tau', 0.05)
        self.declare_parameter('steer_pt2_zeta', 0.7)

        # Longitudinal PI controller
        self.declare_parameter('speed_kp', 0.5)
        self.declare_parameter('speed_ki', 0.1)

        self._lookahead_distance   = self.get_parameter('lookahead_distance').value
        self._max_speed            = self.get_parameter('max_speed').value
        self._wheelbase            = self.get_parameter('wheelbase').value
        self._max_steering_rad     = math.radians(self.get_parameter('max_steering_deg').value)
        self._map_frame            = self.get_parameter('map_frame').value
        self._max_lateral_accel    = self.get_parameter('max_lateral_accel').value
        self._max_deceleration     = self.get_parameter('max_deceleration').value
        self._lookahead_waypoints  = self.get_parameter('lookahead_waypoints').value
        self._speed_reduction_factor = self.get_parameter('speed_reduction_factor').value

        self._pt2_tau  = self.get_parameter('steer_pt2_tau').value
        self._pt2_zeta = self.get_parameter('steer_pt2_zeta').value
        self._speed_kp = self.get_parameter('speed_kp').value
        self._speed_ki = self.get_parameter('speed_ki').value

        # Control loop sample period (matches timer below)
        self._dt = 1.0 / 20.0

        # State
        self._path             = None
        self._car_x            = None
        self._car_y            = None
        self._car_yaw          = None
        self._car_speed        = None   # scalar m/s from odometry twist
        self._mission_finished = False
        self._path_idx         = 0
        self._target_speeds: list[float] = []   # per-waypoint speed from /planning/target_speeds

        # PT2 anticipator state: last two raw (pre-compensation) steer commands
        self._steer_prev1 = 0.0  # r[k-1]
        self._steer_prev2 = 0.0  # r[k-2]

        # PI velocity controller state
        self._speed_integral = 0.0

        # QoS
        reliable_qos    = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        best_effort_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Path,     '/planning/centerline',       self._path_callback,     best_effort_qos)
        self.create_subscription(Odometry, '/ekf/output',                self._odom_callback,     best_effort_qos)
        self.create_subscription(Bool,     '/planning/mission_finished', self._finished_callback, reliable_qos)
        self.create_subscription(
            Float64MultiArray,
            '/planning/target_speeds',
            self._target_speeds_callback,
            best_effort_qos)

        self._cmd_pub = self.create_publisher(ControlCommand, '/control/command', reliable_qos)
        self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f'pure_pursuit_node started | '
            f'lookahead={self._lookahead_distance} m  max_speed={self._max_speed} m/s  '
            f'a_lat={self._max_lateral_accel} m/s²  a_brake={self._max_deceleration} m/s²  '
            f'scan={self._lookahead_waypoints} wp  '
            f'PT2 tau={self._pt2_tau} zeta={self._pt2_zeta}  '
            f'PI Kp={self._speed_kp} Ki={self._speed_ki}'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _finished_callback(self, msg: Bool):
        if msg.data and not self._mission_finished:
            self.get_logger().info('Mission finished — pure pursuit stopped.')
            self._mission_finished = True

    def _target_speeds_callback(self, msg: Float64MultiArray) -> None:
        self._target_speeds = list(msg.data)

    def _path_callback(self, msg: Path):
        if not msg.poses:
            return
        new_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._path_idx = 0
        self._path = new_path

    def _odom_callback(self, msg: Odometry):
        self._car_x     = msg.pose.pose.position.x
        self._car_y     = msg.pose.pose.position.y
        self._car_yaw   = self._quaternion_to_yaw(msg.pose.pose.orientation)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._car_speed = math.hypot(vx, vy)

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self):
        if self._mission_finished:
            return

        if self._path is None or self._car_x is None:
            self.get_logger().warn('Waiting for path/odometry...', throttle_duration_sec=2.0)
            self._reset_controller_state()
            self._publish_command(0.0, 0.0, 0.0)
            return

        lookahead_point = self._find_lookahead_point()
        if lookahead_point is None:
            self._reset_controller_state()
            self._publish_command(0.0, 0.0, 0.0)
            return

        # --- Steering (pure pursuit geometry) ---
        dx = lookahead_point[0] - self._car_x
        dy = lookahead_point[1] - self._car_y
        alpha = math.atan2(dy, dx) - self._car_yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        steering_rad  = math.atan2(2.0 * self._wheelbase * math.sin(alpha), self._lookahead_distance)
        steering_norm = max(-1.0, min(1.0, steering_rad / self._max_steering_rad))

        # Apply PT2 feedforward anticipator to compensate steering actuator lag
        steering_cmd = self._apply_pt2_anticipator(steering_norm)

        # --- Speed (PI closed-loop on ramped v_target) ---
        v_target = self._compute_target_speed(steering_norm)
        throttle, brake = self._compute_throttle_brake_pi(v_target)

        self._publish_command(steering_cmd, throttle, brake)

    # ------------------------------------------------------------------
    # PT2 steer delay compensator
    # ------------------------------------------------------------------

    def _apply_pt2_anticipator(self, r_k: float) -> float:
        """
        Discrete PT2 inverse (Backward Euler) feedforward anticipator.

        Given the desired steering r[k], outputs a pre-emphasis signal u[k]
        such that the actuator output (modeled as a PT2) tracks r[k] rather
        than lagging behind it.

        PT2 continuous TF:  G(s) = 1 / (tau^2 s^2 + 2*zeta*tau*s + 1)
        Inverse (Backward Euler, T=dt):
            u[k] = A*r[k] - B*r[k-1] + C*r[k-2]
        where:
            A = 1 + 2*zeta*tau/T + tau^2/T^2
            B = 2*zeta*tau/T + 2*tau^2/T^2
            C = tau^2/T^2
        """
        tau  = self._pt2_tau
        zeta = self._pt2_zeta
        T    = self._dt

        ratio  = tau / T
        ratio2 = ratio * ratio

        A =  1.0 + 2.0 * zeta * ratio + ratio2
        B =        2.0 * zeta * ratio + 2.0 * ratio2
        C =                             ratio2

        u_k = A * r_k - B * self._steer_prev1 + C * self._steer_prev2

        # Shift history
        self._steer_prev2 = self._steer_prev1
        self._steer_prev1 = r_k

        return max(-1.0, min(1.0, u_k))

    # ------------------------------------------------------------------
    # Velocity profile  →  target speed
    # ------------------------------------------------------------------

    def _compute_target_speed(self, steering_norm: float) -> float:
        """
        Lookahead velocity profile: returns a scalar v_target (m/s).

        Scans the next `lookahead_waypoints` path points, finds the tightest
        corner, and ramps speed down early enough for the car to arrive safely.
        """
        v_now = self._car_speed

        if v_now is None:
            return self._max_speed * (1.0 - self._speed_reduction_factor * abs(steering_norm))

        # Use pre-computed speed profile from path planner if available and length-matched
        if (self._target_speeds
                and self._path is not None
                and len(self._target_speeds) == len(self._path)
                and self._path_idx < len(self._target_speeds)):
            v_target = float(self._target_speeds[self._path_idx])
            v_target = max(0.5, min(v_target, self._max_speed))
            # Still apply the PI controller (or ramp logic) with this v_target
            # ... (keep the existing d_brake / ramp_dist block below, just skip the curvature scan)
            d_brake = max(0.0, (v_now**2 - v_target**2) / (2.0 * self._max_deceleration))
            ramp_dist = max(20.0, d_brake * 5.0)
            dist_to_target = 0.0   # already at the constraint point
            if v_target < self._max_speed:
                return (max(0.1, v_target / self._max_speed), 0.0)
            return (1.0, 0.0)
        # else: fall through to existing curvature scan

        path     = self._path
        idx      = self._path_idx
        n        = len(path)
        scan_end = min(n, idx + self._lookahead_waypoints)

        v_target       = self._max_speed
        dist_to_target = float('inf')
        cum_dist       = 0.0

        for i in range(idx, scan_end - 2):
            if i > idx:
                cum_dist += math.hypot(path[i][0] - path[i-1][0],
                                       path[i][1] - path[i-1][1])

            p0, p1, p2 = path[i], path[i+1], path[i+2]
            dx1 = p1[0]-p0[0];  dy1 = p1[1]-p0[1]
            dx2 = p2[0]-p1[0];  dy2 = p2[1]-p1[1]
            cross = abs(dx1*dy2 - dy1*dx2)
            l1 = math.hypot(dx1, dy1)
            l2 = math.hypot(dx2, dy2)

            if l1 < 0.01 or l2 < 0.01:
                continue

            kappa   = cross / (l1 * l2 * (l1 + l2) / 2.0)
            R       = 1.0 / kappa if kappa > 1e-4 else float('inf')
            v_corner = min(self._max_speed, math.sqrt(self._max_lateral_accel * R))

            if v_corner < v_target:
                v_target       = v_corner
                dist_to_target = cum_dist

        d_brake   = max(0.0, (v_now**2 - v_target**2) / (2.0 * self._max_deceleration))
        ramp_dist = max(20.0, d_brake * 5.0)

        if v_target < self._max_speed and dist_to_target <= ramp_dist:
            t     = dist_to_target / ramp_dist
            v_cmd = v_target + t * (self._max_speed - v_target)
            return max(0.0, v_cmd)
        else:
            return self._max_speed

    # ------------------------------------------------------------------
    # PI longitudinal controller
    # ------------------------------------------------------------------

    def _compute_throttle_brake_pi(self, v_target: float):
        """
        Closed-loop PI velocity controller.

        throttle = Kp * v_error + Ki * integral(v_error)

        Anti-windup: integral state is clamped to [-1.0, 1.0].
        Throttle output is clamped to [0.0, 1.0].
        Brake is not modified by the PI; existing brake logic is preserved
        (currently always 0.0 from the velocity profile path).
        """
        v_now = self._car_speed
        if v_now is None:
            return (max(0.0, min(1.0, v_target / self._max_speed)), 0.0)

        v_error = v_target - v_now

        self._speed_integral += v_error * self._dt
        self._speed_integral  = max(-1.0, min(1.0, self._speed_integral))

        throttle = self._speed_kp * v_error + self._speed_ki * self._speed_integral
        throttle = max(0.0, min(1.0, throttle))

        return (throttle, 0.0)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _reset_controller_state(self):
        self._steer_prev1    = 0.0
        self._steer_prev2    = 0.0
        self._speed_integral = 0.0

    def _find_lookahead_point(self):
        path = self._path
        car_x, car_y = self._car_x, self._car_y
        ld = self._lookahead_distance

        n = len(path)
        search_end = min(n, self._path_idx + 60)

        min_dist    = float('inf')
        closest_idx = self._path_idx
        for i in range(self._path_idx, search_end):
            wx, wy = path[i]
            d = math.hypot(wx - car_x, wy - car_y)
            if d < min_dist:
                min_dist    = d
                closest_idx = i

        if min_dist > 10.0:
            for i, (wx, wy) in enumerate(path):
                d = math.hypot(wx - car_x, wy - car_y)
                if d < min_dist:
                    min_dist    = d
                    closest_idx = i

        self._path_idx = closest_idx

        for i in range(closest_idx, n):
            wx, wy = path[i]
            if math.hypot(wx - car_x, wy - car_y) >= ld:
                return (wx, wy)

        return path[-1]

    def _publish_command(self, steering: float, throttle: float, brake: float):
        cmd = ControlCommand()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self._map_frame
        cmd.steering  = float(steering)
        cmd.throttle  = float(throttle)
        cmd.brake     = float(brake)
        self._cmd_pub.publish(cmd)

    @staticmethod
    def _quaternion_to_yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


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
