import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
SensorDataQoS = lambda: QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

from mfe_state_estimation.filters.extended_kalman_filter import ExtendedKalmanFilter

from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

R_EARTH = 6_371_000.0  # metres


class ExtendedKalmanFilterNode(Node):
    def __init__(self):
        super().__init__("ekf_node")

        self.declare_parameter("imu_frequency", value=20)
        self.declare_parameter("gps_frequency", value=2)
        self.imu_freq = self.get_parameter("imu_frequency").get_parameter_value().integer_value
        self.gps_freq = self.get_parameter("gps_frequency").get_parameter_value().integer_value

        # Configure necessary topics names to subscribe to
        self.declare_parameter("imu_topic", value="/imu/data")
        self.declare_parameter("gps_topic", value="/gps")

        self.imu_topic_name = self.get_parameter("imu_topic").get_parameter_value().string_value
        self.gps_topic_name = self.get_parameter("gps_topic").get_parameter_value().string_value

        # Declare and configure covariance matrices (which indicate noise) in the EKF
        self.declare_parameter("var_imu_acc")
        self.declare_parameter("var_imu_w")
        self.declare_parameter("var_gps")

        self.var_imu_acc = self.get_parameter("var_imu_acc").get_parameter_value().double_array_value
        self.var_imu_w = self.get_parameter("var_imu_w").get_parameter_value().double_value

        self.var_gps_param = self.get_parameter("var_gps").get_parameter_value().double_array_value
        self.var_gps = np.array(self.var_gps_param)

        self.declare_parameter("output_topic", value="/ekf/output")
        self.output_topic_name = self.get_parameter("output_topic")

        self.create_subscription(
            Imu,
            self.imu_topic_name,
            self.imu_callback,
            SensorDataQoS()
        )
        self.create_subscription(
            NavSatFix,
            self.gps_topic_name,
            self.gps_callback,
            SensorDataQoS()
        )

        self.odom_pub_ = self.create_publisher(Odometry, self.output_topic_name.value, 10)

        self._last_imu_time = None

        # EKF instance — built lazily on first valid GPS fix so we have a local origin
        self._ekf: ExtendedKalmanFilter = None

        # GPS reference datum (lat/lon in radians), set on first valid fix
        self._gps_origin = None  # tuple (lat0_rad, lon0_rad)

        # IMU-integrated state
        self._velocity = 0.0   # forward velocity integrated from ax  (m/s)
        self.imu_w = 0.0       # latest angular velocity from IMU     (rad/s)
        self.imu_theta = 0.0   # latest yaw from IMU quaternion       (rad)
        self.imu_acc_x = 0.0   # latest linear acceleration x         (m/s²)

        self._dt = 1.0 / self.imu_freq  # default timestep until first pair of IMU msgs

        self.get_logger().info(
            f"Initialized EKF Node for the following topics: {self.imu_topic_name} "
            f"{self.gps_topic_name} -> {self.output_topic_name.value}")

    # ------------------------------------------------------------------
    # EKF construction
    # ------------------------------------------------------------------

    def _build_ekf(self, x0: float, y0: float, theta0: float) -> None:
        """Construct the EKF instance with closures over self for motion and observation models."""

        # ---- motion model factory ----
        def motion_model():
            def g(mu, u, dt):
                v, omega = u[0], u[1]
                return np.array([
                    mu[0] + v * math.cos(mu[2]) * dt,
                    mu[1] + v * math.sin(mu[2]) * dt,
                    mu[2] + omega * dt,
                ])

            def G(mu, u, dt):
                v = u[0]
                return np.array([
                    [1.0, 0.0, -v * math.sin(mu[2]) * dt],
                    [0.0, 1.0,  v * math.cos(mu[2]) * dt],
                    [0.0, 0.0,  1.0                      ],
                ])

            def V(mu, u, dt):
                return np.array([
                    [math.cos(mu[2]) * dt,  0.0],
                    [math.sin(mu[2]) * dt,  0.0],
                    [0.0,                   dt ],
                ])

            return g, G, V

        # ---- observation model factory ----
        def observation_model():
            def h(mu):
                return mu[:2]

            def H(mu):
                return np.array([
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                ])

            return h, H

        initial_state = np.array([x0, y0, theta0])
        initial_covariance = np.diag([5.0, 5.0, 0.1])

        self._ekf = ExtendedKalmanFilter(
            initial_state,
            initial_covariance,
            motion_model,
            observation_model,
            proc_noise_std=[0.1, 0.1, 0.05],
            obs_noise_std=[0.5, 0.5],
        )
        self.get_logger().info(
            f"EKF built — initial pose: x={x0:.2f} m, y={y0:.2f} m, theta={theta0:.3f} rad")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def imu_callback(self, msg: Imu):
        self.imu_msg = msg
        self.imu_acc_x = msg.linear_acceleration.x
        self.imu_acc_y = msg.linear_acceleration.y
        self.imu_w = msg.angular_velocity.z

        # Extract yaw from quaternion using numpy
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        # yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
        self.imu_theta = np.arctan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))

        now = self.get_clock().now()
        if self._last_imu_time is not None:
            self._dt = (now - self._last_imu_time).nanoseconds * 1e-9
        else:
            self._dt = 1.0 / self.imu_freq
        self._last_imu_time = now

        # Integrate forward velocity from x-axis acceleration
        self._velocity += self.imu_acc_x * self._dt

        self.predict()

    def gps_callback(self, msg: NavSatFix):
        # Reject fixes with no lock
        if msg.status.status < 0:
            self.get_logger().warn("GPS fix rejected — no satellite lock.")
            return

        lat_rad = math.radians(msg.latitude)
        lon_rad = math.radians(msg.longitude)

        # First valid fix: store origin and lazily build the EKF
        if self._gps_origin is None:
            self._gps_origin = (lat_rad, lon_rad)
            theta0 = self.imu_theta  # best heading estimate available at initialisation
            self._build_ekf(x0=0.0, y0=0.0, theta0=theta0)
            return  # nothing to update against — origin IS the first fix

        if self._ekf is None:
            # Should not happen, but guard anyway
            return

        lat0_rad, lon0_rad = self._gps_origin

        # Flat-earth ENU conversion
        x_local = R_EARTH * math.cos(lat0_rad) * (lon_rad - lon0_rad)
        y_local = R_EARTH * (lat_rad - lat0_rad)

        z = np.array([x_local, y_local])
        gps_dt = 1.0 / self.gps_freq

        self._ekf.update(z, gps_dt)
        self._publish_odom()

    # ------------------------------------------------------------------
    # EKF predict step
    # ------------------------------------------------------------------

    def predict(self):
        if self._ekf is None:
            return

        u = np.array([self._velocity, self.imu_w])
        self._ekf.predict(u, self._dt)
        self._publish_odom()

    def update(self):
        return

    # ------------------------------------------------------------------
    # Odometry publisher
    # ------------------------------------------------------------------

    def _publish_odom(self):
        if self._ekf is None:
            return

        mu = self._ekf.mu  # [x, y, theta]
        x, y, yaw = float(mu[0]), float(mu[1]), float(mu[2])

        # Yaw → quaternion (rotation about z only)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_footprint"

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

        # 6×6 row-major covariance — fill x, y, yaw diagonal from EKF Sigma
        sigma = self._ekf.Sigma
        cov = [0.0] * 36
        cov[0]  = float(sigma[0, 0])   # x-x
        cov[7]  = float(sigma[1, 1])   # y-y
        cov[35] = float(sigma[2, 2])   # yaw-yaw
        msg.pose.covariance = cov

        self.odom_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    ekf_node = ExtendedKalmanFilterNode()
    rclpy.spin(ekf_node)

    ekf_node.destroy_node()
    rclpy.shutdown()
