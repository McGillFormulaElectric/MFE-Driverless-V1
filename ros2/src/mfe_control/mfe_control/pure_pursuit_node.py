"""Pure pursuit controller node for MFE FSAE driverless car."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry, Path
from fs_msgs.msg import ControlCommand
from std_msgs.msg import Bool, Header


class PurePursuitNode(Node):
    """
    Pure pursuit lateral controller with adaptive look-ahead and velocity profile.

    Steering:  pure pursuit geometry (classic) with adaptive look-ahead distance.
               ld = K_v * v * sqrt(1 + κ²)   where κ is local path curvature.
               Note: this formula increases ld on curves (improves anticipation /
               reduces oscillation on curvy sections); it is intentionally *larger*
               than the straight-line value, not smaller.
               ld is clamped to [ld_min, ld_max].
    Speed:     scan upcoming path curvature → compute corner speed targets
               → brake hard if within stopping distance, full throttle otherwise.
               Falls back to steering-proportional reduction when no speed data.

    Subscribes to:
        /planning/centerline  (nav_msgs/Path)    — waypoints in map frame
        /ekf/output           (nav_msgs/Odometry) — vehicle pose + velocity

    Publishes:
        /control/command      (fs_msgs/ControlCommand)
    """

    def __init__(self):
        super().__init__('pure_pursuit_node')

        # --- Parameters ---
        # K_v: look-ahead gain (m per m/s). Base ld = K_v * v.
        # At 10 m/s this gives ld=5.0 m (same as the old fixed default).
        self.declare_parameter('K_v', 0.5)
        # Adaptive look-ahead clamp bounds (metres)
        self.declare_parameter('ld_min', 0.5)
        self.declare_parameter('ld_max', 15.0)
        self.declare_parameter('max_speed', 10.0)
        self.declare_parameter('wheelbase', 1.56)
        self.declare_parameter('max_steering_deg', 25.0)
        self.declare_parameter('map_frame', 'map')

        # Velocity profile parameters
        # max lateral acceleration used to compute corner speed target: v = sqrt(a_lat * R)
        self.declare_parameter('max_lateral_accel', 8.0)
        # max deceleration (m/s²) — determines how late we can brake
        self.declare_parameter('max_deceleration', 10.0)
        # how many waypoints ahead to scan for corners
        self.declare_parameter('lookahead_waypoints', 40)

        # Fallback for when car speed is unknown (startup)
        self.declare_parameter('speed_reduction_factor', 0.3)

        self._K_v                  = self.get_parameter('K_v').value
        self._ld_min               = self.get_parameter('ld_min').value
        self._ld_max               = self.get_parameter('ld_max').value
        self._max_speed            = self.get_parameter('max_speed').value
        self._wheelbase            = self.get_parameter('wheelbase').value
        self._max_steering_rad     = math.radians(self.get_parameter('max_steering_deg').value)
        self._map_frame            = self.get_parameter('map_frame').value
        self._max_lateral_accel    = self.get_parameter('max_lateral_accel').value
        self._max_deceleration     = self.get_parameter('max_deceleration').value
        self._lookahead_waypoints  = self.get_parameter('lookahead_waypoints').value
        self._speed_reduction_factor = self.get_parameter('speed_reduction_factor').value

        # State
        self._path             = None
        self._car_x            = None
        self._car_y            = None
        self._car_yaw          = None
        self._car_speed        = None   # scalar m/s from odometry twist
        self._mission_finished = False
        self._path_idx         = 0

        # QoS
        reliable_qos   = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        best_effort_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Path,     '/planning/centerline',     self._path_callback,     best_effort_qos)
        self.create_subscription(Odometry, '/ekf/output',              self._odom_callback,     best_effort_qos)
        self.create_subscription(Bool,     '/planning/mission_finished', self._finished_callback, reliable_qos)

        self._cmd_pub = self.create_publisher(ControlCommand, '/control/command', reliable_qos)
        self.create_timer(1.0 / 20.0, self._control_loop)

        self.get_logger().info(
            f'pure_pursuit_node started | '
            f'K_v={self._K_v}  ld=[{self._ld_min},{self._ld_max}] m  '
            f'max_speed={self._max_speed} m/s  '
            f'a_lat={self._max_lateral_accel} m/s²  a_brake={self._max_deceleration} m/s²  '
            f'scan={self._lookahead_waypoints} wp'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _finished_callback(self, msg: Bool):
        if msg.data and not self._mission_finished:
            self.get_logger().info('Mission finished — pure pursuit stopped.')
            self._mission_finished = True

    def _path_callback(self, msg: Path):
        if not msg.poses:
            return
        new_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._path_idx = 0  # always reset — planner regenerates from car's current position
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
            self._publish_command(0.0, 0.0, 0.0)
            return

        # --- Adaptive look-ahead: ld = K_v * v * sqrt(1 + κ²) ---
        # κ is sampled from the waypoints near the vehicle's current position
        # so ld is determined before searching for the lookahead point.
        v = self._car_speed if self._car_speed is not None else 0.0
        kappa = self._local_curvature_at_idx(self._path_idx)
        ld = self._K_v * v * math.sqrt(1.0 + kappa ** 2)
        ld = max(self._ld_min, min(self._ld_max, ld))

        lookahead_point = self._find_lookahead_point(ld)
        if lookahead_point is None:
            self._publish_command(0.0, 0.0, 0.0)
            return

        # --- Steering (pure pursuit) ---
        dx = lookahead_point[0] - self._car_x
        dy = lookahead_point[1] - self._car_y
        alpha = math.atan2(dy, dx) - self._car_yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        steering_rad  = math.atan2(2.0 * self._wheelbase * math.sin(alpha), ld)
        steering_norm = max(-1.0, min(1.0, steering_rad / self._max_steering_rad))

        # --- Speed (lookahead velocity profile) ---
        throttle, brake = self._compute_throttle_brake(steering_norm)

        self._publish_command(steering_norm, throttle, brake)

    # ------------------------------------------------------------------
    # Velocity profile
    # ------------------------------------------------------------------

    def _compute_throttle_brake(self, steering_norm: float):
        """
        Lookahead velocity profile: brake late and hard into corners.

        Scans the next `lookahead_waypoints` path points, computes each
        corner's maximum safe speed from curvature, then decides whether to
        brake now (to arrive at that speed in time) or go full throttle.
        """
        v_now = self._car_speed

        # Fallback before first odometry: steering-proportional reduction
        if v_now is None:
            speed = self._max_speed * (1.0 - self._speed_reduction_factor * abs(steering_norm))
            return (speed / self._max_speed, 0.0)

        path     = self._path
        idx      = self._path_idx
        n        = len(path)
        scan_end = min(n, idx + self._lookahead_waypoints)

        # Find the tightest corner ahead and its distance
        v_target      = self._max_speed
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

            # Menger curvature κ = cross / (l1 · l2 · (l1+l2)/2)
            kappa = cross / (l1 * l2 * (l1 + l2) / 2.0)
            R = 1.0 / kappa if kappa > 1e-4 else float('inf')
            v_corner = min(self._max_speed, math.sqrt(self._max_lateral_accel * R))

            if v_corner < v_target:
                v_target       = v_corner
                dist_to_target = cum_dist

        # Command corner speed early enough for the EUFS velocity controller to respond.
        # Start ramping down 20 m before the corner (or 5× kinematic braking distance,
        # whichever is larger) so the controller has time to actually decelerate.
        d_brake = max(0.0, (v_now**2 - v_target**2) / (2.0 * self._max_deceleration))
        ramp_dist = max(20.0, d_brake * 5.0)

        if v_target < self._max_speed and dist_to_target <= ramp_dist:
            # Linear ramp: max_speed far out, v_target at the corner
            t = dist_to_target / ramp_dist          # 1.0 = far, 0.0 = at corner
            v_cmd = v_target + t * (self._max_speed - v_target)
            return (max(0.1, v_cmd / self._max_speed), 0.0)
        else:
            return (1.0, 0.0)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _local_curvature_at_idx(self, idx: int) -> float:
        """
        Estimate the Menger curvature κ at `idx` from the surrounding waypoints.

        Uses a window of ±2 waypoints averaged to reduce noise from sparse
        waypoints. Returns 0.0 when there are not enough points.
        """
        path = self._path
        if path is None or len(path) < 3:
            return 0.0

        n = len(path)
        kappas = []
        # Sample a few triplets centred near idx
        for centre in range(max(1, idx - 2), min(n - 1, idx + 3)):
            p0 = path[centre - 1]
            p1 = path[centre]
            p2 = path[centre + 1] if centre + 1 < n else path[centre]
            dx1 = p1[0] - p0[0];  dy1 = p1[1] - p0[1]
            dx2 = p2[0] - p1[0];  dy2 = p2[1] - p1[1]
            cross = abs(dx1 * dy2 - dy1 * dx2)
            l1 = math.hypot(dx1, dy1)
            l2 = math.hypot(dx2, dy2)
            if l1 < 0.01 or l2 < 0.01:
                continue
            # Menger curvature κ = |cross| / (l1 · l2 · (l1+l2)/2)
            kappas.append(cross / (l1 * l2 * (l1 + l2) / 2.0))

        return sum(kappas) / len(kappas) if kappas else 0.0

    def _find_lookahead_point(self, ld: float):
        path = self._path
        car_x, car_y = self._car_x, self._car_y

        # Find closest waypoint to the car, searching forward from _path_idx.
        # _path_idx is reset to 0 on every new path, so this window covers the
        # relevant portion and advances naturally as the car moves.
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

        # If nothing close was found in the window, fall back to global search
        if min_dist > 10.0:
            for i, (wx, wy) in enumerate(path):
                d = math.hypot(wx - car_x, wy - car_y)
                if d < min_dist:
                    min_dist    = d
                    closest_idx = i

        self._path_idx = closest_idx

        # Find the first waypoint at or beyond the lookahead distance
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
