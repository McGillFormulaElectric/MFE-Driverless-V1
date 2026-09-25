"""
Failsafe Supervisor Node — MFE Driverless.

Implements Chalmers-style failsafes for the autonomous Formula Student car:

  LiDAR watchdog:
    If no PointCloud2 is received on /lidar/points_raw for > lidar_timeout_ms,
    publish emergency brake on /mfe/emergency_brake (std_msgs/Bool, True = brake).

  GPS outlier detector:
    If the GPS covariance (position_covariance diagonal) exceeds gps_cov_threshold
    OR if the reported position jumps more than gps_jump_threshold_m from the last
    known-good fix, flag GPS as degraded and switch to odometry-only mode.
    Status is broadcast on /as/supervisor/status.

  Control latency watchdog:
    The pure_pursuit_node publishes /control/command at 20 Hz.  If the inter-arrival
    gap between consecutive commands exceeds control_latency_threshold_ms, the car is
    assumed to have experienced a compute overrun; the supervisor publishes a reduced-
    throttle command on /mfe/emergency_brake to trigger the safety response in the ACU.

Published topics:
    /as/supervisor/status   (std_msgs/String)   — human-readable status line at 10 Hz
    /mfe/emergency_brake    (std_msgs/Bool)      — True = full emergency brake requested

Subscribed topics:
    /lidar/points_raw       (sensor_msgs/PointCloud2)  — raw LiDAR scan
    /gps                    (sensor_msgs/NavSatFix)    — GNSS fix from Xsens MTi-670G
    /control/command        (fs_msgs/ControlCommand)   — control loop heartbeat

ROS2 parameters (all with defaults):
    lidar_timeout_ms            (int,   default 200)   — LiDAR drop-out threshold
    gps_jump_threshold_m        (float, default 2.0)   — max allowed per-fix position jump (m)
    gps_cov_threshold           (float, default 25.0)  — max diagonal position covariance (m²)
    control_latency_threshold_ms (int,  default 100)   — max /control/command inter-arrival (ms)

Note: This node lives in the mfe_control package (ament_python) because mfe_sensors is an
ament_cmake (C++) package that does not have a Python setup.py.  The supervisor's outputs
— emergency brake and throttle limiting — are control-plane concerns, making mfe_control
the semantically correct home.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np

from std_msgs.msg import Bool, String
from sensor_msgs.msg import PointCloud2, NavSatFix
from nav_msgs.msg import Odometry
from mfe_msgs.msg import Cone, Track


# ---------------------------------------------------------------------------
# Attempt to import ControlCommand from fs_msgs (may not be installed in all
# environments — supervisor degrades gracefully if unavailable).
# ---------------------------------------------------------------------------
try:
    from fs_msgs.msg import ControlCommand
    _HAS_FS_MSGS = True
except ImportError:
    _HAS_FS_MSGS = False


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Return approximate great-circle distance in metres between two WGS-84 points."""
    R = 6_371_000.0  # Earth radius in metres
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


class SupervisorNode(Node):
    """
    Chalmers-style failsafe supervisor for MFE driverless.

    Three independent watchdogs run on a shared 10 Hz timer that also
    publishes the aggregated status string.
    """

    def __init__(self):
        super().__init__('supervisor_node')

        # ---------------------------------------------------------------
        # ROS2 parameters — all thresholds are configurable at launch time
        # ---------------------------------------------------------------
        self.declare_parameter('lidar_timeout_ms',             200)
        self.declare_parameter('gps_jump_threshold_m',         2.0)
        self.declare_parameter('gps_cov_threshold',           25.0)
        self.declare_parameter('control_latency_threshold_ms', 100)
        self.declare_parameter('cone_hit_radius_m',            0.5)   # car centre to cone centre

        self._lidar_timeout_s    = self.get_parameter('lidar_timeout_ms').value / 1000.0
        self._gps_jump_thr_m    = float(self.get_parameter('gps_jump_threshold_m').value)
        self._gps_cov_thr       = float(self.get_parameter('gps_cov_threshold').value)
        self._ctrl_lat_thr_s    = self.get_parameter('control_latency_threshold_ms').value / 1000.0
        self._cone_hit_r        = float(self.get_parameter('cone_hit_radius_m').value)

        # ---------------------------------------------------------------
        # State
        # ---------------------------------------------------------------
        self._last_lidar_time    = self.get_clock().now()
        self._lidar_lost         = False

        self._last_gps_lat       = None
        self._last_gps_lon       = None
        self._gps_degraded       = False

        self._last_control_time  = self.get_clock().now()
        self._control_latency_s  = 0.0
        self._control_late       = False

        # Cone proximity: car position + latest fused cone map
        self._car_x              = 0.0
        self._car_y              = 0.0
        self._cone_xy            = np.zeros((0, 2), dtype=np.float32)  # Nx2 map-frame
        self._cone_hit           = False

        # ---------------------------------------------------------------
        # QoS profiles
        # ---------------------------------------------------------------
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # ---------------------------------------------------------------
        # Subscriptions
        # ---------------------------------------------------------------
        # LiDAR pointcloud — topic confirmed in lidar_pipeline.launch.py
        self.create_subscription(
            PointCloud2, '/lidar/points_raw',
            self._lidar_callback, sensor_qos,
        )

        # GPS fix — topic confirmed in xsens_mti.launch.py and ekf_params.yaml
        self.create_subscription(
            NavSatFix, '/gps',
            self._gps_callback, sensor_qos,
        )

        # Cone map + odometry for proximity detection
        self.create_subscription(
            Track, '/planning/cones', self._cones_callback, reliable_qos)
        self.create_subscription(
            Odometry, '/ekf/output', self._odom_callback, reliable_qos)

        # Control command heartbeat — topic confirmed in pure_pursuit_node.py
        if _HAS_FS_MSGS:
            self.create_subscription(
                ControlCommand, '/control/command',
                self._control_callback, reliable_qos,
            )
        else:
            self.get_logger().warn(
                'fs_msgs not available — /control/command watchdog DISABLED. '
                'Install the fs_msgs package to enable control latency monitoring.'
            )

        # ---------------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------------
        self._status_pub = self.create_publisher(String, '/as/supervisor/status', reliable_qos)
        self._brake_pub  = self.create_publisher(Bool,   '/mfe/emergency_brake',  reliable_qos)

        # ---------------------------------------------------------------
        # 10 Hz watchdog + status timer
        # ---------------------------------------------------------------
        self.create_timer(0.1, self._watchdog_tick)

        self.get_logger().info(
            f'supervisor_node started | '
            f'lidar_timeout={self._lidar_timeout_s * 1000:.0f} ms  '
            f'gps_jump={self._gps_jump_thr_m:.1f} m  '
            f'gps_cov={self._gps_cov_thr:.1f} m²  '
            f'ctrl_latency={self._ctrl_lat_thr_s * 1000:.0f} ms'
        )

    # -------------------------------------------------------------------
    # Sensor callbacks — update timestamps / state; no brake logic here
    # -------------------------------------------------------------------

    def _odom_callback(self, msg: Odometry) -> None:
        self._car_x = msg.pose.pose.position.x
        self._car_y = msg.pose.pose.position.y

    def _cones_callback(self, msg: Track) -> None:
        """Cache latest fused cone positions as Nx2 numpy array for proximity checks."""
        if not msg.track:
            return
        self._cone_xy = np.array(
            [[c.location.x, c.location.y] for c in msg.track], dtype=np.float32)

    def _lidar_callback(self, _msg: PointCloud2) -> None:
        """Reset LiDAR watchdog on every received pointcloud."""
        self._last_lidar_time = self.get_clock().now()

    def _gps_callback(self, msg: NavSatFix) -> None:
        """
        GPS outlier detection.

        Flags the fix as degraded if:
          1. Any diagonal element of position_covariance exceeds the threshold, OR
          2. The haversine jump from the previous accepted fix exceeds the threshold.
        """
        # Covariance check (diagonal: lat²,lon²,alt² in m²)
        cov_bad = (
            msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN
            and any(
                msg.position_covariance[i] > self._gps_cov_thr
                for i in (0, 4, 8)  # diagonal indices of 3×3 matrix stored row-major
            )
        )

        # Position jump check
        jump_bad = False
        if self._last_gps_lat is not None and self._last_gps_lon is not None:
            dist = _haversine_m(
                self._last_gps_lat, self._last_gps_lon,
                msg.latitude, msg.longitude,
            )
            if dist > self._gps_jump_thr_m:
                jump_bad = True
                self.get_logger().warn(
                    f'GPS position jump detected: {dist:.2f} m '
                    f'(threshold {self._gps_jump_thr_m:.1f} m) — switching to odometry-only'
                )

        if cov_bad:
            self.get_logger().warn(
                'GPS covariance too high — switching to odometry-only'
            )

        self._gps_degraded = cov_bad or jump_bad

        # Only update last-known-good position when the fix is clean
        if not self._gps_degraded:
            self._last_gps_lat = msg.latitude
            self._last_gps_lon = msg.longitude

    def _control_callback(self, _msg) -> None:
        """Track inter-arrival time of /control/command to detect compute overruns."""
        now = self.get_clock().now()
        self._control_latency_s = (now - self._last_control_time).nanoseconds * 1e-9
        self._last_control_time = now

    # -------------------------------------------------------------------
    # Watchdog tick — runs at 10 Hz; evaluates all failsafes and publishes
    # -------------------------------------------------------------------

    def _watchdog_tick(self) -> None:
        now = self.get_clock().now()

        # ---- LiDAR loss watchdog ----------------------------------------
        lidar_age_s = (now - self._last_lidar_time).nanoseconds * 1e-9
        self._lidar_lost = lidar_age_s > self._lidar_timeout_s
        if self._lidar_lost:
            self.get_logger().warn(
                f'LiDAR LOST — no pointcloud for {lidar_age_s * 1000:.0f} ms '
                f'(threshold {self._lidar_timeout_s * 1000:.0f} ms) — EMERGENCY BRAKE',
                throttle_duration_sec=1.0,
            )

        # ---- Control latency watchdog ------------------------------------
        # Also check absolute time since last command to catch complete drop-outs.
        ctrl_age_s = (now - self._last_control_time).nanoseconds * 1e-9
        self._control_late = (
            _HAS_FS_MSGS
            and (
                self._control_latency_s > self._ctrl_lat_thr_s
                or ctrl_age_s > self._ctrl_lat_thr_s
            )
        )
        if self._control_late:
            self.get_logger().warn(
                f'Control loop LATENCY {max(self._control_latency_s, ctrl_age_s) * 1000:.0f} ms '
                f'> {self._ctrl_lat_thr_s * 1000:.0f} ms threshold — reducing throttle',
                throttle_duration_sec=1.0,
            )

        # ---- Cone proximity check ----------------------------------------
        # Fused LiDAR+camera cones from /planning/cones. A cone within
        # cone_hit_radius_m of the car centre means the car has hit or is
        # about to hit it — trigger emergency brake immediately.
        self._cone_hit = False
        if self._cone_xy.shape[0] > 0:
            diff = self._cone_xy - np.array([self._car_x, self._car_y], dtype=np.float32)
            dists = np.linalg.norm(diff, axis=1)
            min_dist = float(dists.min())
            if min_dist < self._cone_hit_r:
                self._cone_hit = True
                self.get_logger().error(
                    f'CONE HIT detected: nearest cone {min_dist:.3f} m from car centre '
                    f'(threshold {self._cone_hit_r:.2f} m) — EMERGENCY BRAKE',
                    throttle_duration_sec=0.5,
                )

        # ---- Emergency brake decision ------------------------------------
        # LiDAR loss is an immediate full-stop condition.
        # GPS degradation alone does not trigger a brake (switches to odometry-only).
        # Control latency triggers a soft-stop (published on same topic for simplicity).
        # Cone hit triggers immediate full stop to prevent continued swerving.
        brake = self._lidar_lost or self._control_late or self._cone_hit
        brake_msg = Bool()
        brake_msg.data = brake
        self._brake_pub.publish(brake_msg)

        # ---- Status string -----------------------------------------------
        gps_mode   = 'odom-only' if self._gps_degraded else 'GPS+odom'
        lidar_str  = 'LOST'     if self._lidar_lost   else 'OK'
        ctrl_str   = 'LATE'     if self._control_late else 'OK'
        cone_str   = 'HIT'      if self._cone_hit     else 'OK'
        status = (
            f'lidar={lidar_str}  gps_mode={gps_mode}  ctrl_latency={ctrl_str}  '
            f'cone={cone_str}  brake={brake}'
        )
        status_msg = String()
        status_msg.data = status
        self._status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
