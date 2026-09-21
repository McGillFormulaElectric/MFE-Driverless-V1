#!/usr/bin/env python3
"""
Xsens MTi-670G noise injector (sim only).

Subscribes to /ground_truth/state_odom (perfect Gazebo ground truth) and republishes a
noise-corrupted copy on /sim/xsens/state_odom, using the accuracy figures from the
MTi-670G GNSS/INS datasheet (canalgeomatics.com/products/xsens-mti-670g-gnss-ins) as the
noise magnitude:

  Position:    <1 m CEP           -> sigma_xy = CEP / 1.1774 (circular-normal CEP50 relation)
  Roll/Pitch:  0.2 deg RMS
  Yaw:         0.8 deg RMS
  Velocity:    0.05 m/s RMS
  Gyro (yaw rate): 0.007 deg/s/sqrt(Hz) noise density, assumed ~50 Hz effective bandwidth
                    -> sigma ~= density * sqrt(bandwidth)

The error is modeled as an Ornstein-Uhlenbeck (correlated random-walk) process per axis,
not i.i.d. white noise per sample: real fused GNSS/INS output is the output of the
device's own internal filter, so its error is smoothly time-varying (bias-like), not
independent from one 200 Hz sample to the next. This also matters mechanically — an
earlier white-noise version made consecutive position samples spatially discontinuous,
which broke any consumer that integrates pose deltas over time (e.g. finish_detector's
min_travel_m gate and lap_validator's distance-traveled stat both summed near-random
per-sample jumps and blew up to thousands of fake meters within seconds, on top of
degrading pure_pursuit's tracking). The OU process keeps the same marginal sigma (still
sized from the datasheet figures below) while being correlated over a ~2 s timescale,
so consecutive high-rate samples stay close together like a real sensor's output.

This exists so sim runs stop relying on /ground_truth/state_odom (perfect pose) for
localization and instead see something representative of what the real Xsens MTi-670G
would hand the stack. See ros2/src/mfe_sensors/launch/xsens_mti.launch.py for the real
hardware path (Xsens driver -> /imu, /gps -> ekf_node), which this does NOT replace —
this node is a much cheaper stand-in for sim-only planner/controller testing.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation

_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)

# --- MTi-670G datasheet-derived noise sigmas ---------------------------------------------
_CEP_POSITION_M = 1.0
_SIGMA_XY_M = _CEP_POSITION_M / 1.1774   # ~0.849 m, circular-normal CEP50 -> sigma
_SIGMA_Z_M = 1.5 * _SIGMA_XY_M            # GNSS altitude error typically ~1.5x horizontal
_SIGMA_ROLL_PITCH_RAD = math.radians(0.2)
_SIGMA_YAW_RAD = math.radians(0.8)
_SIGMA_VEL_MS = 0.05
_GYRO_NOISE_DENSITY_RAD_S_SQRTHZ = math.radians(0.007)
_ASSUMED_BANDWIDTH_HZ = 50.0
_SIGMA_YAWRATE_RAD_S = _GYRO_NOISE_DENSITY_RAD_S_SQRTHZ * math.sqrt(_ASSUMED_BANDWIDTH_HZ)

# OU correlation time — how slowly the injected error wanders. Real GNSS/INS bias-like
# error (multipath, residual filter transients) actually varies over tens of seconds,
# not sub-second — and empirically, even a 2s tau still added enough fake distance at
# 200 Hz to falsely satisfy finish_detector's min_travel_m gate early (sim-tested: 2s
# tau inflated an ~100m real peanut run's finish-detector travel estimate to ~544m).
# 10s trades a bit of noise "freshness" for consumers that integrate raw pose deltas.
# Tune via the 'noise_correlation_time_s' parameter if it doesn't fit the track speed.
_DEFAULT_TAU_S = 10.0


class _OUState:
    """One Ornstein-Uhlenbeck noise channel: mean-reverting, stationary variance = sigma^2."""

    def __init__(self, rng: np.random.Generator, sigma: float, tau: float):
        self._rng = rng
        self._sigma = sigma
        self._tau = tau
        self.value = float(rng.normal(0.0, sigma))  # start already at steady-state variance

    def step(self, dt: float) -> float:
        if dt <= 0.0:
            return self.value
        decay = math.exp(-dt / self._tau)
        drive_std = self._sigma * math.sqrt(max(0.0, 1.0 - decay * decay))
        self.value = self.value * decay + self._rng.normal(0.0, drive_std)
        return self.value


class XsensNoiseNode(Node):

    def __init__(self):
        super().__init__('xsens_noise_node')

        self.declare_parameter('input_topic', '/ground_truth/state_odom')
        self.declare_parameter('output_topic', '/sim/xsens/state_odom')
        self.declare_parameter('seed', 0)  # 0 -> unseeded (different noise every run)
        self.declare_parameter('noise_correlation_time_s', _DEFAULT_TAU_S)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        seed = self.get_parameter('seed').value
        tau = self.get_parameter('noise_correlation_time_s').value
        self._rng = np.random.default_rng(seed if seed else None)

        self._ou = {
            name: _OUState(self._rng, sigma, tau)
            for name, sigma in [
                ('x', _SIGMA_XY_M), ('y', _SIGMA_XY_M), ('z', _SIGMA_Z_M),
                ('roll', _SIGMA_ROLL_PITCH_RAD), ('pitch', _SIGMA_ROLL_PITCH_RAD),
                ('yaw', _SIGMA_YAW_RAD),
                ('vx', _SIGMA_VEL_MS), ('vy', _SIGMA_VEL_MS),
                ('yawrate', _SIGMA_YAWRATE_RAD_S),
            ]
        }
        self._last_t = None

        self._pub = self.create_publisher(Odometry, output_topic, _QOS)
        self.create_subscription(Odometry, input_topic, self._odom_cb, _QOS)

        self.get_logger().info(
            f'Xsens MTi-670G noise model: {input_topic} -> {output_topic} '
            f'(sigma_xy={_SIGMA_XY_M:.3f}m, sigma_yaw={math.degrees(_SIGMA_YAW_RAD):.2f}deg, '
            f'sigma_vel={_SIGMA_VEL_MS:.3f}m/s, correlation_time={tau:.2f}s)')

    def _odom_cb(self, msg: Odometry) -> None:
        t = self.get_clock().now()
        dt = (t - self._last_t).nanoseconds * 1e-9 if self._last_t is not None else 0.0
        self._last_t = t
        for state in self._ou.values():
            state.step(dt)

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id

        # --- position ---
        out.pose.pose.position.x = msg.pose.pose.position.x + self._ou['x'].value
        out.pose.pose.position.y = msg.pose.pose.position.y + self._ou['y'].value
        out.pose.pose.position.z = msg.pose.pose.position.z + self._ou['z'].value

        # --- orientation: perturb roll/pitch/yaw, convert back to quaternion ---
        q = msg.pose.pose.orientation
        rpy = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
        rpy[0] += self._ou['roll'].value
        rpy[1] += self._ou['pitch'].value
        rpy[2] += self._ou['yaw'].value
        nq = Rotation.from_euler('xyz', rpy).as_quat()
        out.pose.pose.orientation.x = float(nq[0])
        out.pose.pose.orientation.y = float(nq[1])
        out.pose.pose.orientation.z = float(nq[2])
        out.pose.pose.orientation.w = float(nq[3])

        # --- covariance: report the injected variance (diagonal only) ---
        cov = [0.0] * 36
        cov[0]  = _SIGMA_XY_M ** 2          # x-x
        cov[7]  = _SIGMA_XY_M ** 2          # y-y
        cov[14] = _SIGMA_Z_M ** 2           # z-z
        cov[21] = _SIGMA_ROLL_PITCH_RAD ** 2   # roll-roll
        cov[28] = _SIGMA_ROLL_PITCH_RAD ** 2   # pitch-pitch
        cov[35] = _SIGMA_YAW_RAD ** 2          # yaw-yaw
        out.pose.covariance = cov

        # --- velocity ---
        out.twist.twist.linear.x = msg.twist.twist.linear.x + self._ou['vx'].value
        out.twist.twist.linear.y = msg.twist.twist.linear.y + self._ou['vy'].value
        out.twist.twist.linear.z = msg.twist.twist.linear.z
        out.twist.twist.angular.x = msg.twist.twist.angular.x
        out.twist.twist.angular.y = msg.twist.twist.angular.y
        out.twist.twist.angular.z = (
            msg.twist.twist.angular.z + self._ou['yawrate'].value)

        twist_cov = [0.0] * 36
        twist_cov[0]  = _SIGMA_VEL_MS ** 2
        twist_cov[7]  = _SIGMA_VEL_MS ** 2
        twist_cov[35] = _SIGMA_YAWRATE_RAD_S ** 2
        out.twist.covariance = twist_cov

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = XsensNoiseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
