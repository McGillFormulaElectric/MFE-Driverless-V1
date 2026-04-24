"""Pure pursuit controller node for MFE FSAE driverless car."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry, Path
from fs_msgs.msg import ControlCommand
from std_msgs.msg import Header


class PurePursuitNode(Node):
    """
    Pure pursuit lateral controller.

    Subscribes to:
        /planning/centerline  (nav_msgs/Path)   — sparse waypoints in map frame
        /ekf/output           (nav_msgs/Odometry) — vehicle pose in map frame

    Publishes:
        /control/command      (fs_msgs/ControlCommand)
    """

    def __init__(self):
        super().__init__('pure_pursuit_node')

        # Parameters
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('max_speed', 5.0)
        self.declare_parameter('wheelbase', 1.5)
        self.declare_parameter('max_steering_deg', 25.0)
        self.declare_parameter('speed_reduction_factor', 0.5)
        self.declare_parameter('map_frame', 'map')

        self._lookahead_distance = self.get_parameter('lookahead_distance').value
        self._max_speed = self.get_parameter('max_speed').value
        self._wheelbase = self.get_parameter('wheelbase').value
        self._max_steering_rad = math.radians(
            self.get_parameter('max_steering_deg').value
        )
        self._speed_reduction_factor = self.get_parameter('speed_reduction_factor').value
        self._map_frame = self.get_parameter('map_frame').value

        # State
        self._path = None          # list of (x, y) tuples
        self._car_x = None
        self._car_y = None
        self._car_yaw = None

        # QoS
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        best_effort_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.create_subscription(
            Path,
            '/planning/centerline',
            self._path_callback,
            best_effort_qos,
        )
        self.create_subscription(
            Odometry,
            '/ekf/output',
            self._odom_callback,
            best_effort_qos,
        )

        # Publisher
        self._cmd_pub = self.create_publisher(
            ControlCommand,
            '/control/command',
            reliable_qos,
        )

        # Control timer at 20 Hz
        self.create_timer(1.0 / 20.0, self._control_loop)

        self.get_logger().info(
            f'pure_pursuit_node started | lookahead={self._lookahead_distance} m, '
            f'max_speed={self._max_speed} m/s, wheelbase={self._wheelbase} m, '
            f'max_steering={math.degrees(self._max_steering_rad):.1f} deg'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _path_callback(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn('Received empty path — ignoring.')
            return
        self._path = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]

    def _odom_callback(self, msg: Odometry):
        self._car_x = msg.pose.pose.position.x
        self._car_y = msg.pose.pose.position.y
        self._car_yaw = self._quaternion_to_yaw(msg.pose.pose.orientation)

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self):
        # Publish zero command if state is not yet available
        if self._path is None or self._car_x is None:
            if self._path is None:
                self.get_logger().warn(
                    'Waiting for path...', throttle_duration_sec=2.0
                )
            if self._car_x is None:
                self.get_logger().warn(
                    'Waiting for odometry...', throttle_duration_sec=2.0
                )
            self._publish_command(steering=0.0, throttle=0.0, brake=0.0)
            return

        lookahead_point = self._find_lookahead_point()
        if lookahead_point is None:
            self.get_logger().warn(
                'No valid lookahead point found — publishing zero command.',
                throttle_duration_sec=2.0,
            )
            self._publish_command(steering=0.0, throttle=0.0, brake=0.0)
            return

        lx, ly = lookahead_point

        # Angle from car heading to lookahead point (in vehicle frame)
        dx = lx - self._car_x
        dy = ly - self._car_y
        angle_to_point = math.atan2(dy, dx)
        alpha = angle_to_point - self._car_yaw

        # Wrap alpha to [-pi, pi]
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        # Pure pursuit steering angle
        steering_angle_rad = math.atan2(
            2.0 * self._wheelbase * math.sin(alpha),
            self._lookahead_distance,
        )

        # Normalise to [-1, 1]
        steering_norm = steering_angle_rad / self._max_steering_rad
        steering_norm = max(-1.0, min(1.0, steering_norm))

        # Speed: reduce proportionally to steering magnitude
        speed = self._max_speed * (
            1.0 - self._speed_reduction_factor * abs(steering_norm)
        )
        speed = max(0.0, min(self._max_speed, speed))

        # Throttle: map speed to [0, 1]
        throttle = speed / self._max_speed

        self._publish_command(steering=steering_norm, throttle=throttle, brake=0.0)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _find_lookahead_point(self):
        """
        Find the furthest waypoint still within lookahead_distance of the car.

        Strategy:
          1. Find the index of the closest waypoint.
          2. Walk forward from there; keep the last point that is within
             lookahead_distance.
          3. If no point from the closest onward is within lookahead_distance,
             return the closest waypoint itself (end-of-path fallback).
        """
        path = self._path
        car_x, car_y = self._car_x, self._car_y
        ld = self._lookahead_distance

        # Find the index of the closest waypoint
        min_dist = float('inf')
        closest_idx = 0
        for i, (wx, wy) in enumerate(path):
            d = math.hypot(wx - car_x, wy - car_y)
            if d < min_dist:
                min_dist = d
                closest_idx = i

        # Walk forward from closest, keep the last point within lookahead circle
        lookahead_point = None
        for i in range(closest_idx, len(path)):
            wx, wy = path[i]
            d = math.hypot(wx - car_x, wy - car_y)
            if d <= ld:
                lookahead_point = (wx, wy)
            else:
                # Once we exceed lookahead distance we can stop (path is ordered)
                break

        if lookahead_point is None:
            # No waypoint within lookahead circle — use the closest one
            lookahead_point = path[closest_idx]

        return lookahead_point

    def _publish_command(self, steering: float, throttle: float, brake: float):
        cmd = ControlCommand()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self._map_frame
        cmd.steering = float(steering)
        cmd.throttle = float(throttle)
        cmd.brake = float(brake)
        self._cmd_pub.publish(cmd)

    @staticmethod
    def _quaternion_to_yaw(q) -> float:
        """Extract yaw from a geometry_msgs/Quaternion."""
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
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
