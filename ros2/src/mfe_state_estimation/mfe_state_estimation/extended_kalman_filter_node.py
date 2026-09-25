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

# ---------------------------------------------------------------------------
# 8-state EKF: [x, y, θ, v, ω, b_gps_x, b_gps_y, b_gyro]
# ---------------------------------------------------------------------------
# State vector layout (indices):
#   0: x          — position east  (m, local ENU frame)
#   1: y          — position north (m, local ENU frame)
#   2: θ (theta)  — heading        (rad)
#   3: v          — forward speed  (m/s, unicycle convention: body-frame longitudinal)
#   4: ω (omega)  — yaw rate       (rad/s)
#   5: b_gps_x    — GPS x bias     (m)  — slow-drifting correlated GPS error
#   6: b_gps_y    — GPS y bias     (m)
#   7: b_gyro     — gyro yaw-rate bias (rad/s) — constant-like IMU offset
#
# Measurement sources:
#   GPS:        z = [x + b_gps_x, y + b_gps_y]        H: 2×8
#   Wheelspeed: z = [v]                                H: 1×8
#   Gyro:       z = [ω + b_gyro]                       H: 1×8
#
# Motion model (unicycle + bias random walk):
#   x_new       = x + v * cos(θ) * dt
#   y_new       = y + v * sin(θ) * dt
#   θ_new       = θ + ω * dt
#   v_new       = v                        (constant, corrected by wheelspeed)
#   ω_new       = ω                        (constant, corrected by gyro)
#   b_gps_x_new = b_gps_x                 (random walk — noise in Q)
#   b_gps_y_new = b_gps_y
#   b_gyro_new  = b_gyro
#
# Anti-windup bias clamp bounds (applied after every predict/update):
_BIAS_GPS_MAX_M = 2.0        # GPS bias: ±2 m (a few × OU sigma=0.5 m)
_BIAS_GYRO_MAX_RAD_S = 0.05  # Gyro bias: ±0.05 rad/s (~3°/s — generous)
#
# Backwards compatibility:
#   /ekf/output (nav_msgs/Odometry) is populated from mu[0:3] exactly as before.
#   twist.linear.x ← v (mu[3]), twist.angular.z ← ω (mu[4]).
#   pose covariance [0,7,35] ← sigma[0,0], sigma[1,1], sigma[2,2] as before.
#   Downstream consumers reading pose.{x,y} and pose.orientation are unaffected.
#
# Wheelspeed topic:
#   No dedicated wheel encoder topic exists in the sim yet. The node subscribes to
#   `/ground_truth/state_odom` (nav_msgs/Odometry, published by the bridge node) and
#   uses twist.twist.linear.x as the wheelspeed proxy. In hardware this should be
#   remapped to a real encoder topic. The topic name is configurable via the
#   `wheelspeed_topic` parameter.
# ---------------------------------------------------------------------------

# Process noise standard deviations (tuning knobs)
# Order matches the 8-state vector: [x, y, θ, v, ω, b_gps_x, b_gps_y, b_gyro]
_DEFAULT_PROC_NOISE = [
    0.05,    # x          (m)       — small: driven by kinematics
    0.05,    # y          (m)
    0.02,    # θ          (rad)
    0.5,     # v          (m/s)    — allow speed to drift between wheelspeed updates
    0.1,     # ω          (rad/s)  — allow yaw rate to drift between gyro updates
    0.005,   # b_gps_x    (m)      — slow GPS bias drift
    0.005,   # b_gps_y    (m)
    0.0001,  # b_gyro     (rad/s)  — very slow gyro bias drift
]


def _build_8state_motion_model():
    """
    Returns (g, G, V) for the 8-state unicycle + bias random walk system.

    g : state transition function  mu_{t} = g(mu_{t-1}, u=None, dt)
    G : Jacobian of g w.r.t. state (8×8)
    V : Jacobian of g w.r.t. control (not used here — returns identity for compatibility)
    """

    def g(mu, u, dt):
        x, y, theta, v, omega, bgx, bgy, bgyro = mu
        return np.array([
            x + v * math.cos(theta) * dt,
            y + v * math.sin(theta) * dt,
            theta + omega * dt,
            v,
            omega,
            bgx,
            bgy,
            bgyro,
        ])

    def G(mu, u, dt):
        _, _, theta, v, _, _, _, _ = mu
        # Jacobian of g w.r.t. mu
        Gmat = np.eye(8)
        Gmat[0, 2] = -v * math.sin(theta) * dt   # dx/dtheta
        Gmat[0, 3] =  math.cos(theta) * dt        # dx/dv
        Gmat[1, 2] =  v * math.cos(theta) * dt    # dy/dtheta
        Gmat[1, 3] =  math.sin(theta) * dt        # dy/dv
        Gmat[2, 4] =  dt                           # dtheta/domega
        return Gmat

    def V(mu, u, dt):
        # Kept for EKF compatibility; 8×8 identity since we don't use external control input
        return np.eye(8)

    return g, G, V


def _H_gps():
    """H matrix for GPS measurement z=[x+b_gps_x, y+b_gps_y]. Shape 2×8."""
    H = np.zeros((2, 8))
    H[0, 0] = 1.0   # x
    H[0, 5] = 1.0   # b_gps_x
    H[1, 1] = 1.0   # y
    H[1, 6] = 1.0   # b_gps_y
    return H


def _H_wheelspeed():
    """H matrix for wheelspeed measurement z=[v]. Shape 1×8."""
    H = np.zeros((1, 8))
    H[0, 3] = 1.0   # v
    return H


def _H_gyro():
    """H matrix for gyro measurement z=[omega + b_gyro]. Shape 1×8."""
    H = np.zeros((1, 8))
    H[0, 4] = 1.0   # omega
    H[0, 7] = 1.0   # b_gyro  (so gyro bias is observable from raw gyro measurements)
    return H


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
        self.declare_parameter("wheelspeed_topic", value="/ground_truth/state_odom")

        self.imu_topic_name = self.get_parameter("imu_topic").get_parameter_value().string_value
        self.gps_topic_name = self.get_parameter("gps_topic").get_parameter_value().string_value
        self.wheelspeed_topic_name = self.get_parameter("wheelspeed_topic").get_parameter_value().string_value

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

        # ---- measurement noise matrices ----
        # GPS: [x, y] — use var_gps param
        gps_var = float(self.var_gps[0]) if len(self.var_gps) > 0 else 4.0
        gps_var_y = float(self.var_gps[1]) if len(self.var_gps) > 1 else gps_var
        self._R_gps = np.diag([gps_var, gps_var_y])

        # Wheelspeed: proxy from odometry twist, treated as ~0.05 m/s std
        _WHEELSPEED_VAR = 0.0025   # (0.05 m/s)^2
        self._R_wheelspeed = np.array([[_WHEELSPEED_VAR]])

        # Gyro: use var_imu_w param
        self._R_gyro = np.array([[float(self.var_imu_w)]])

        # Pre-compute constant H matrices
        self._H_gps = _H_gps()
        self._H_ws = _H_wheelspeed()
        self._H_gyro = _H_gyro()

        # ---- subscriptions ----
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
        # Wheelspeed proxy: read twist.linear.x from ground truth or sim odom.
        # On hardware, remap wheelspeed_topic to a real encoder topic.
        self.create_subscription(
            Odometry,
            self.wheelspeed_topic_name,
            self.wheelspeed_callback,
            SensorDataQoS()
        )

        self.odom_pub_ = self.create_publisher(Odometry, self.output_topic_name.value, 10)

        self._last_imu_time = None

        # EKF instance — built lazily on first valid GPS fix so we have a local origin
        self._ekf: ExtendedKalmanFilter = None

        # GPS reference datum (lat/lon in radians), set on first valid fix
        self._gps_origin = None  # tuple (lat0_rad, lon0_rad)

        # Latest sensor readings (used by predict step)
        self._velocity = 0.0   # forward velocity from wheelspeed proxy (m/s)
        self.imu_w = 0.0       # latest raw angular velocity from IMU     (rad/s)
        self.imu_theta = 0.0   # latest yaw from IMU quaternion           (rad)
        self.imu_acc_x = 0.0   # latest linear acceleration x             (m/s²)
        self.imu_acc_y = 0.0   # latest linear acceleration y             (m/s²)

        self._dt = 1.0 / self.imu_freq  # default timestep until first pair of IMU msgs

        self.get_logger().info(
            f"Initialized 8-state EKF Node: {self.imu_topic_name} (IMU), "
            f"{self.gps_topic_name} (GPS), "
            f"{self.wheelspeed_topic_name} (wheelspeed proxy) "
            f"-> {self.output_topic_name.value}")
        self.get_logger().info(
            "State: [x, y, θ, v, ω, b_gps_x, b_gps_y, b_gyro]  "
            "(bias anti-windup: GPS ±%.1fm, gyro ±%.4f rad/s)" %
            (_BIAS_GPS_MAX_M, _BIAS_GYRO_MAX_RAD_S))

    # ------------------------------------------------------------------
    # EKF construction
    # ------------------------------------------------------------------

    def _build_ekf(self, x0: float, y0: float, theta0: float) -> None:
        """Construct the 8-state EKF instance."""

        def motion_model():
            return _build_8state_motion_model()

        # Observation model is not used for multi-source updates (we call update
        # directly per measurement type). Supply GPS model as the default so the
        # ExtendedKalmanFilter constructor doesn't fail.
        def observation_model():
            H_const = _H_gps()

            def h(mu):
                # Predicted GPS measurement = [x + b_gps_x, y + b_gps_y]
                return np.array([mu[0] + mu[5], mu[1] + mu[6]])

            def H(mu):
                return H_const

            return h, H

        initial_state = np.array([x0, y0, theta0, 0.0, 0.0, 0.0, 0.0, 0.0])
        initial_covariance = np.diag([
            5.0,    # x
            5.0,    # y
            0.1,    # theta
            1.0,    # v
            0.1,    # omega
            1.0,    # b_gps_x  (initialised with some uncertainty — not zero)
            1.0,    # b_gps_y
            0.01,   # b_gyro
        ])

        self._ekf = ExtendedKalmanFilter(
            initial_state,
            initial_covariance,
            motion_model,
            observation_model,
            proc_noise_std=_DEFAULT_PROC_NOISE,
            obs_noise_std=[math.sqrt(self._R_gps[0, 0]),
                           math.sqrt(self._R_gps[1, 1])],
        )
        self.get_logger().info(
            f"8-state EKF built — initial pose: x={x0:.2f} m, y={y0:.2f} m, theta={theta0:.3f} rad")

    # ------------------------------------------------------------------
    # Bias anti-windup clamp
    # ------------------------------------------------------------------

    def _clamp_biases(self) -> None:
        """Clamp GPS and gyro bias estimates to prevent unbounded drift."""
        if self._ekf is None:
            return
        mu = self._ekf.mu
        mu[5] = float(np.clip(mu[5], -_BIAS_GPS_MAX_M,    _BIAS_GPS_MAX_M))
        mu[6] = float(np.clip(mu[6], -_BIAS_GPS_MAX_M,    _BIAS_GPS_MAX_M))
        mu[7] = float(np.clip(mu[7], -_BIAS_GYRO_MAX_RAD_S, _BIAS_GYRO_MAX_RAD_S))
        self._ekf.mu = mu

    # ------------------------------------------------------------------
    # Per-measurement EKF update (bypasses single observation_model)
    # ------------------------------------------------------------------

    def _update_with_measurement(self, z: np.ndarray, H: np.ndarray, R: np.ndarray) -> None:
        """
        Generic EKF update step for heterogeneous measurements.

        Uses the pre-built EKF's Sigma and mu; applies a standard correction step
        with the provided H (measurement Jacobian) and R (measurement noise covariance).
        This bypasses the single observation_model stored in the ExtendedKalmanFilter
        instance, allowing multi-source fusion.

        K = Sigma H^T (H Sigma H^T + R)^{-1}
        mu += K (z - H mu)
        Sigma = (I - K H) Sigma
        """
        mu = self._ekf.mu
        Sigma = self._ekf.Sigma

        S = H @ Sigma @ H.T + R
        K = Sigma @ H.T @ np.linalg.inv(S)

        innovation = z - H @ mu
        self._ekf.mu = mu + K @ innovation
        I = np.eye(len(mu))
        self._ekf.Sigma = (I - K @ H) @ Sigma

        self._clamp_biases()

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

        # Gyro update: fuse raw gyro measurement z=[imu_w] with H_gyro
        # This makes b_gyro observable (H_gyro has +1 on both ω and b_gyro columns)
        if self._ekf is not None:
            z_gyro = np.array([self.imu_w])
            self._update_with_measurement(z_gyro, self._H_gyro, self._R_gyro)

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

        # GPS update: z=[x_meas, y_meas], H accounts for GPS bias states
        z_gps = np.array([x_local, y_local])
        self._update_with_measurement(z_gps, self._H_gps, self._R_gps)
        self._publish_odom()

    def wheelspeed_callback(self, msg: Odometry):
        """
        Wheelspeed update from odometry twist.linear.x proxy.

        In simulation, /ground_truth/state_odom provides the ground truth speed.
        On hardware, remap 'wheelspeed_topic' to a real encoder/CAN topic that
        publishes nav_msgs/Odometry (or adapt this callback for the actual message type).
        """
        if self._ekf is None:
            return

        wheelspeed = msg.twist.twist.linear.x
        self._velocity = wheelspeed

        z_ws = np.array([wheelspeed])
        self._update_with_measurement(z_ws, self._H_ws, self._R_wheelspeed)
        self._publish_odom()

    # ------------------------------------------------------------------
    # EKF predict step
    # ------------------------------------------------------------------

    def predict(self):
        if self._ekf is None:
            return

        # Control input is not used in the new motion model (v and ω are states).
        # Pass zero placeholder for API compatibility.
        u = np.zeros(2)
        self._ekf.predict(u, self._dt)
        self._clamp_biases()
        self._publish_odom()

    def update(self):
        return

    # ------------------------------------------------------------------
    # Odometry publisher (backwards compatible with 3-state consumers)
    # ------------------------------------------------------------------

    def _publish_odom(self):
        if self._ekf is None:
            return

        mu = self._ekf.mu  # [x, y, theta, v, omega, b_gps_x, b_gps_y, b_gyro]
        x, y, yaw = float(mu[0]), float(mu[1]), float(mu[2])
        v, omega   = float(mu[3]), float(mu[4])

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
        # Backwards-compatible: same indices as the original 3-state node
        sigma = self._ekf.Sigma
        cov = [0.0] * 36
        cov[0]  = float(sigma[0, 0])   # x-x
        cov[7]  = float(sigma[1, 1])   # y-y
        cov[35] = float(sigma[2, 2])   # yaw-yaw
        msg.pose.covariance = cov

        # Populate twist for consumers that read velocity (new capability)
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = omega

        twist_cov = [0.0] * 36
        twist_cov[0]  = float(sigma[3, 3])   # v-v
        twist_cov[35] = float(sigma[4, 4])   # omega-omega
        msg.twist.covariance = twist_cov

        self.odom_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    ekf_node = ExtendedKalmanFilterNode()
    rclpy.spin(ekf_node)

    ekf_node.destroy_node()
    rclpy.shutdown()
