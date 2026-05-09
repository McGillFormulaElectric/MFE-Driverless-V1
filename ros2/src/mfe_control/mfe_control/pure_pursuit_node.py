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
    Pure pursuit lateral controller with lookahead velocity profile.

    Steering:  pure pursuit geometry (classic).
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
        self.declare_parameter('lookahead_distance', 5.0)
        self.declare_parameter('max_speed', 10.0)
        self.declare_parameter('wheelbase', 1.5)
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

        self._lookahead_distance   = self.get_parameter('lookahead_distance').value
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
            f'lookahead={self._lookahead_distance} m  max_speed={self._max_speed} m/s  '
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
        if self._path is None or len(new_path) != len(self._path):
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
            self._publish_command(0.0, 0.0, 0.0)
            return

        lookahead_point = self._find_lookahead_point()
        if lookahead_point is None:
            self._publish_command(0.0, 0.0, 0.0)
            return

        # --- Steering (pure pursuit) ---
        dx = lookahead_point[0] - self._car_x
        dy = lookahead_point[1] - self._car_y
        alpha = math.atan2(dy, dx) - self._car_yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        steering_rad  = math.atan2(2.0 * self._wheelbase * math.sin(alpha), self._lookahead_distance)
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

        # Braking distance required to decelerate from v_now to v_target
        d_brake = max(0.0, (v_now**2 - v_target**2) / (2.0 * self._max_deceleration))

        # Only brake if we are genuinely over the corner speed AND within braking distance.
        # If v_now <= v_target the car still needs to accelerate into the corner.
        if v_now > v_target and dist_to_target <= d_brake + 1.0:
            overspeed  = v_now - v_target
            brake_norm = min(1.0, overspeed / max(1.0, self._max_speed * 0.3))
            self.get_logger().debug(
                f'BRAKE  v={v_now:.1f}→{v_target:.1f} m/s  '
                f'd={dist_to_target:.1f}m  d_brake={d_brake:.1f}m  b={brake_norm:.2f}',
                throttle_duration_sec=0.1,
            )
            return (0.0, brake_norm)
        else:
            # Accelerate — full throttle towards max_speed
            return (1.0, 0.0)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _find_lookahead_point(self):
        path = self._path
        car_x, car_y = self._car_x, self._car_y
        ld = self._lookahead_distance

        search_start = self._path_idx
        search_end   = min(len(path), search_start + 30)

        min_dist    = float('inf')
        closest_idx = search_start
        for i in range(search_start, search_end):
            wx, wy = path[i]
            d = math.hypot(wx - car_x, wy - car_y)
            if d < min_dist:
                min_dist    = d
                closest_idx = i

        if min_dist > 20.0:
            for i, (wx, wy) in enumerate(path):
                d = math.hypot(wx - car_x, wy - car_y)
                if d < min_dist:
                    min_dist    = d
                    closest_idx = i

        self._path_idx = closest_idx

        # Find the first waypoint at or beyond the lookahead distance
        lookahead_point = None
        for i in range(closest_idx, len(path)):
            wx, wy = path[i]
            d = math.hypot(wx - car_x, wy - car_y)
            if d >= ld:
                lookahead_point = (wx, wy)
                break

        # If no point is far enough, use the last point in path
        if lookahead_point is None:
            lookahead_point = path[-1]

        return lookahead_point

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
