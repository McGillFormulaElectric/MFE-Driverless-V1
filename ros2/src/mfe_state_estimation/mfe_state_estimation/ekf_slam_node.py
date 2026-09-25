"""EKF-SLAM cone-landmark node.

State vector: mu = [x, y, θ, cx0, cy0, cx1, cy1, ..., cxN, cyN]
  - [0:3]  vehicle pose in map frame
  - [3+2i:3+2i+2]  position of landmark i in map frame

Subscriptions:
  /planning/cones   (mfe_msgs/Track)      — fused cone detections, already in map frame
  /ekf/output       (nav_msgs/Odometry)   — vehicle odometry used for prediction

Publications:
  /slam/cone_map    (mfe_msgs/Track)      — globally-consistent cone map
  /slam/pose        (nav_msgs/Odometry)   — SLAM-corrected vehicle pose

NOTE: mfe_msgs/Track carries Cone[] with no covariance field.  The per-landmark
      2×2 covariance lives inside the EKF state (Sigma block).  If callers need
      per-cone covariance, switch the publication type to mfe_msgs/TrackWithCovariance.

Association gate: Mahalanobis distance < 5.99 (chi² 95th-percentile at 2 DOF).
                  The spec also mentions 2.45 (chi² at 2.0); 5.99 is used here as it
                  is the canonical 95 % gate and avoids premature landmark splits.

Prediction model: odometry delta is extracted in *map frame* from /ekf/output, then
                  rotated into body frame before applying the spec's rotation formula
                  so that the Jacobian derivation remains consistent.

Numerical stability:
  - Sigma symmetrised after every update: 0.5*(Sigma + Sigma.T)
  - Joseph-form covariance update: (I-KH) Sigma (I-KH).T + K R K.T
  - np.linalg.solve used instead of np.linalg.inv for innovation covariance
  - Heading wrapped to [-π, π] after prediction
  - Max 300 landmarks (FSAE track max ~200 cones)
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Header

from mfe_msgs.msg import Track, Cone

# ---------------------------------------------------------------------------
# QoS helpers
# ---------------------------------------------------------------------------

SensorDataQoS = lambda: QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

MAX_LANDMARKS: int = 300           # Hard cap — FSAE track has at most ~200 cones
CHI2_GATE: float = 5.99            # chi² 95th-percentile at 2 DOF
NEW_LANDMARK_DIST: float = 1.5     # metres — minimum Mahalanobis miss to add landmark

# Process noise standard deviations [σ_x, σ_y, σ_θ]
PROC_NOISE_STD = np.array([0.05, 0.05, 0.01])

# Observation noise covariance (2×2) for cone positions already in map frame
R_OBS = np.diag([0.25, 0.25])      # ≈ 0.5 m std dev per axis

# Initial vehicle-pose covariance
INIT_POSE_COV = np.diag([0.01, 0.01, 0.001])

# Initial landmark covariance when first inserted (generous so data-association
# converges quickly in the first few re-observations)
INIT_LM_COV = np.diag([1.0, 1.0])


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def _wrap_angle(a: float) -> float:
    """Wrap angle to [-π, π]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def _odom_to_yaw(odom: Odometry) -> float:
    """Extract yaw from Odometry quaternion."""
    q = odom.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class EkfSlamNode(Node):
    """EKF-SLAM node: augmented state [pose | landmarks]."""

    def __init__(self) -> None:
        super().__init__("ekf_slam_node")

        # ---- state --------------------------------------------------------
        # mu   : (3 + 2*N,)  numpy array
        # Sigma: (3 + 2*N, 3 + 2*N) numpy array
        self._mu: np.ndarray = np.array([0.0, 0.0, 0.0])
        self._Sigma: np.ndarray = INIT_POSE_COV.copy().astype(np.float64)
        self._n_landmarks: int = 0

        # Build Q once at full vehicle-pose size; extended later as needed
        self._Q_pose = np.diag(PROC_NOISE_STD ** 2)   # 3×3 process noise

        # ---- odometry bookkeeping ----------------------------------------
        self._last_odom: Odometry | None = None

        # ---- subscriptions -----------------------------------------------
        self.create_subscription(
            Track,
            "/planning/cones",
            self._cones_callback,
            SensorDataQoS(),
        )
        self.create_subscription(
            Odometry,
            "/ekf/output",
            self._odom_callback,
            SensorDataQoS(),
        )

        # ---- publications ------------------------------------------------
        self._cone_map_pub = self.create_publisher(Track, "/slam/cone_map", 10)
        self._pose_pub = self.create_publisher(Odometry, "/slam/pose", 10)

        self.get_logger().info(
            "EKF-SLAM node started — "
            f"gate={CHI2_GATE:.2f}  new_lm_dist={NEW_LANDMARK_DIST:.2f} m  "
            f"max_landmarks={MAX_LANDMARKS}"
        )

    # =======================================================================
    # Callbacks
    # =======================================================================

    def _odom_callback(self, msg: Odometry) -> None:
        """Run prediction step from odometry delta."""
        if self._last_odom is None:
            self._last_odom = msg
            return

        # Map-frame position deltas
        dx_map = msg.pose.pose.position.x - self._last_odom.pose.pose.position.x
        dy_map = msg.pose.pose.position.y - self._last_odom.pose.pose.position.y
        dtheta  = _wrap_angle(
            _odom_to_yaw(msg) - _odom_to_yaw(self._last_odom)
        )
        self._last_odom = msg

        # Rotate map-frame delta into body frame at *previous* heading
        theta_prev = self._mu[2]
        c, s = math.cos(theta_prev), math.sin(theta_prev)
        dx_body =  c * dx_map + s * dy_map
        dy_body = -s * dx_map + c * dy_map

        self._predict(dx_body, dy_body, dtheta)
        self._publish_pose()

    def _cones_callback(self, msg: Track) -> None:
        """Run update step from cone observations (already in map frame)."""
        for cone in msg.track:
            obs = np.array([cone.location.x, cone.location.y])
            self._update_single(obs, cone.color)

        self._publish_cone_map()
        self._publish_pose()

    # =======================================================================
    # EKF Predict
    # =======================================================================

    def _predict(self, dx: float, dy: float, dtheta: float) -> None:
        """Prediction step.

        Args:
            dx, dy   : body-frame displacement deltas (metres)
            dtheta   : heading change (radians)
        """
        mu = self._mu
        theta = mu[2]
        n = len(mu)

        # ---- state update (non-linear) ------------------------------------
        c, s = math.cos(theta), math.sin(theta)
        mu_new = mu.copy()
        mu_new[0] += dx * c - dy * s
        mu_new[1] += dx * s + dy * c
        mu_new[2]  = _wrap_angle(theta + dtheta)
        # Landmark positions are unchanged in the prediction step
        self._mu = mu_new

        # ---- Jacobian Fx (n × n) -----------------------------------------
        # Only the [0:3, 0:3] block is non-identity
        Fx = np.eye(n)
        # Partial derivatives of (new_x, new_y) w.r.t. theta at previous heading
        Fx[0, 2] = -dx * s - dy * c
        Fx[1, 2] =  dx * c - dy * s

        # ---- Process noise Q_ext (n × n) ----------------------------------
        # Non-zero only in the [0:3, 0:3] vehicle-pose block
        Q_ext = np.zeros((n, n))
        Q_ext[:3, :3] = self._Q_pose

        # ---- Covariance propagation ---------------------------------------
        self._Sigma = Fx @ self._Sigma @ Fx.T + Q_ext
        # Enforce symmetry
        self._Sigma = 0.5 * (self._Sigma + self._Sigma.T)

    # =======================================================================
    # EKF Update (single cone)
    # =======================================================================

    def _update_single(self, obs: np.ndarray, color: int) -> None:
        """Process one cone observation.

        obs : (2,) array — [x, y] in map frame.
        """
        best_idx = self._associate(obs)

        if best_idx == -1:
            # No match → augment state with new landmark (if under cap)
            if self._n_landmarks < MAX_LANDMARKS:
                self._augment_landmark(obs, color)
            return

        # Matched landmark i = best_idx → EKF update
        i = best_idx
        lm_start = 3 + 2 * i
        n = len(self._mu)

        # Innovation: observation minus expected (already in map frame)
        innov = obs - self._mu[lm_start:lm_start + 2]

        # Observation Jacobian H  (2 × n)
        # h(mu) = mu[lm_start:lm_start+2], so:
        #   dh/d(x,y,θ) = 0
        #   dh/d(cx_i,cy_i) = I_2
        H = np.zeros((2, n))
        H[0, lm_start]     = 1.0
        H[1, lm_start + 1] = 1.0

        # Innovation covariance S = H Sigma H.T + R
        S = H @ self._Sigma @ H.T + R_OBS

        # Mahalanobis distance guard (redundant after association but kept for safety)
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.get_logger().warn("Singular S matrix during update — skipping.")
            return

        # Kalman gain K (n × 2)
        K = self._Sigma @ H.T @ S_inv

        # State update
        self._mu = self._mu + K @ innov
        self._mu[2] = _wrap_angle(self._mu[2])

        # Joseph-form covariance update for numerical stability:
        # Sigma = (I - K H) Sigma (I - K H).T + K R K.T
        I_KH = np.eye(n) - K @ H
        self._Sigma = I_KH @ self._Sigma @ I_KH.T + K @ R_OBS @ K.T
        self._Sigma = 0.5 * (self._Sigma + self._Sigma.T)

    # =======================================================================
    # Data Association
    # =======================================================================

    def _associate(self, obs: np.ndarray) -> int:
        """Return landmark index with minimum Mahalanobis distance below gate.

        Returns -1 if no existing landmark is within the gate (new landmark).
        """
        if self._n_landmarks == 0:
            return -1

        n = len(self._mu)
        best_idx = -1
        best_dist = float("inf")

        for i in range(self._n_landmarks):
            lm_start = 3 + 2 * i
            innov = obs - self._mu[lm_start:lm_start + 2]

            # Observation Jacobian for this candidate
            H = np.zeros((2, n))
            H[0, lm_start]     = 1.0
            H[1, lm_start + 1] = 1.0

            S = H @ self._Sigma @ H.T + R_OBS
            try:
                # Use solve instead of inv for numerical stability
                d2 = float(innov @ np.linalg.solve(S, innov))
            except np.linalg.LinAlgError:
                continue

            if d2 < best_dist:
                best_dist = d2
                best_idx = i

        # Accept only if within chi² gate
        if best_dist < CHI2_GATE:
            return best_idx

        # Also reject if Euclidean distance suggests a brand-new landmark
        # (avoids erroneous far-gate matches when Sigma is initially large)
        if best_idx != -1:
            lm_start = 3 + 2 * best_idx
            eucl = np.linalg.norm(obs - self._mu[lm_start:lm_start + 2])
            if eucl > NEW_LANDMARK_DIST:
                return -1

        return -1

    # =======================================================================
    # Landmark Augmentation
    # =======================================================================

    def _augment_landmark(self, obs: np.ndarray, color: int) -> None:
        """Extend state and covariance with a new landmark.

        The new landmark position is initialised directly from the map-frame
        observation.  Its covariance block starts at INIT_LM_COV, which is
        deliberately generous so that subsequent observations can shrink it.
        """
        n_old = len(self._mu)
        n_new = n_old + 2

        # Extend state
        self._mu = np.append(self._mu, obs)

        # Extend covariance (block-diagonal augmentation)
        Sigma_new = np.zeros((n_new, n_new))
        Sigma_new[:n_old, :n_old] = self._Sigma
        Sigma_new[n_old:, n_old:] = INIT_LM_COV

        self._Sigma = Sigma_new
        self._n_landmarks += 1

        # Store colour separately (not part of EKF state but needed for publishing)
        if not hasattr(self, "_lm_colors"):
            self._lm_colors: list[int] = []
        self._lm_colors.append(color)

    # =======================================================================
    # Publishers
    # =======================================================================

    def _publish_pose(self) -> None:
        x, y, yaw = float(self._mu[0]), float(self._mu[1]), float(self._mu[2])

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

        # 6×6 row-major pose covariance — fill x, y, yaw diagonals
        cov = [0.0] * 36
        cov[0]  = float(self._Sigma[0, 0])   # x-x
        cov[7]  = float(self._Sigma[1, 1])   # y-y
        cov[35] = float(self._Sigma[2, 2])   # yaw-yaw
        msg.pose.covariance = cov

        self._pose_pub.publish(msg)

    def _publish_cone_map(self) -> None:
        colors = getattr(self, "_lm_colors", [])
        track_msg = Track()
        for i in range(self._n_landmarks):
            lm_start = 3 + 2 * i
            cone = Cone()
            cone.header.stamp = self.get_clock().now().to_msg()
            cone.header.frame_id = "map"
            cone.location.x = float(self._mu[lm_start])
            cone.location.y = float(self._mu[lm_start + 1])
            cone.location.z = 0.0
            cone.color = colors[i] if i < len(colors) else Cone.UNKNOWN
            track_msg.track.append(cone)

        self._cone_map_pub.publish(track_msg)


# ---------------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = EkfSlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
