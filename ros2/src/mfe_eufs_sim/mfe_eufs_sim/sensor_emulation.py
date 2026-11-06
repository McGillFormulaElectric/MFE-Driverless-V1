"""Sensor models for the sim: the Xsens MTi-670G IMU and GNSS, and the CAN wheel speed.

Each model turns ground-truth motion into what the car's sensor would report: the IMU and GNSS
as the Xsens driver publishes them (sensor_msgs/Imu on /imu, sensor_msgs/NavSatFix on /gps), and
the wheel speed as the CAN node will publish it. Shared by xsens_noise_node, imu_gps_emulator_node
and the sim bridge. Nothing here touches a node, so the models are unit-tested on their own.
"""
import math

import numpy as np
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

from mfe_common.geometry import EARTH_RADIUS_M

# --- MTi-670G datasheet-derived noise sigmas ---------------------------------------------
# Source: canalgeomatics.com/products/xsens-mti-670g-gnss-ins, as used by xsens_noise_node.
CEP_POSITION_M = 1.0
SIGMA_XY_M = CEP_POSITION_M / 1.1774       # circular-normal CEP50 relation, ~0.849 m
SIGMA_Z_M = 1.5 * SIGMA_XY_M                # GNSS altitude error typically ~1.5x horizontal
SIGMA_ROLL_PITCH_RAD = math.radians(0.2)
SIGMA_YAW_RAD = math.radians(0.8)
SIGMA_VEL_MS = 0.05
GYRO_NOISE_DENSITY_RAD_S_SQRTHZ = math.radians(0.007)
ASSUMED_BANDWIDTH_HZ = 50.0
SIGMA_YAWRATE_RAD_S = GYRO_NOISE_DENSITY_RAD_S_SQRTHZ * math.sqrt(ASSUMED_BANDWIDTH_HZ)

# Not in the datasheet figures above: assumed values for the accelerometer and the bias drift.
GRAVITY_M_S2 = 9.80665
ACCEL_NOISE_DENSITY_M_S2_SQRTHZ = 60e-6 * GRAVITY_M_S2
SIGMA_ACCEL_M_S2 = ACCEL_NOISE_DENSITY_M_S2_SQRTHZ * math.sqrt(ASSUMED_BANDWIDTH_HZ)
GYRO_BIAS_WALK_RAD_S_SQRTS = 2e-6          # ~10 deg/h of drift after 10 minutes
ACCEL_BIAS_WALK_M_S2_SQRTS = 5e-5

# OU correlation time of the fused GNSS/INS error (see xsens_noise_node for the reasoning).
CORRELATION_TIME_S = 10.0

# CAN wheel-speed encoder noise; matches the EKF's wheel-speed measurement variance (0.05 m/s)^2.
WHEEL_SPEED_SIGMA_M_S = 0.05
WHEEL_SPEED_RATE_HZ = 100.0

# Flat-earth datum of the emulated GNSS. Arbitrary: the EKF takes its first fix as its origin.
DEFAULT_DATUM_LAT_DEG = 49.3280
DEFAULT_DATUM_LON_DEG = 8.5650


def stamp_ns(stamp: TimeMsg) -> int:
    """Nanoseconds of a builtin_interfaces/Time."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class OUState:
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


class RandomWalk:
    """One bias channel: a random walk with diffusion sigma_per_sqrt_s, starting at zero."""

    def __init__(self, rng: np.random.Generator, sigma_per_sqrt_s: float):
        self._rng = rng
        self._sigma_per_sqrt_s = sigma_per_sqrt_s
        self.value = 0.0

    def step(self, dt: float) -> float:
        if dt > 0.0:
            self.value += float(self._rng.normal(0.0, self._sigma_per_sqrt_s * math.sqrt(dt)))
        return self.value


class RateGate:
    """Decimates a stream to rate_hz: due() is True once per period, at the first sample after it.

    Ground truth arrives at its own rate (about 200 Hz in the bags), so the gate picks the samples
    that fall due. A small tolerance absorbs timing jitter, and a stall skips periods rather than
    bursting.
    """

    _TOLERANCE_FRACTION = 0.01

    def __init__(self, rate_hz: float):
        self._period_ns = round(1e9 / rate_hz)
        self._tolerance_ns = self._period_ns * self._TOLERANCE_FRACTION
        self._next_ns = None

    def due(self, t_ns: int) -> bool:
        if self._next_ns is None:
            self._next_ns = t_ns
        if t_ns < self._next_ns - self._tolerance_ns:
            return False
        self._next_ns += self._period_ns
        while self._next_ns <= t_ns + self._tolerance_ns:
            self._next_ns += self._period_ns
        return True


def enu_to_latlon(east_m: float, north_m: float, lat0_deg: float, lon0_deg: float):
    """Latitude and longitude (deg) of a local ENU point: the inverse of the EKF's fix conversion.

    The EKF computes x = R cos(lat0) (lon - lon0) and y = R (lat - lat0)
    (extended_kalman_filter_node), so this returns coordinates the EKF maps back to
    (east_m, north_m).
    """
    lat0 = math.radians(lat0_deg)
    lat = lat0 + north_m / EARTH_RADIUS_M
    lon = math.radians(lon0_deg) + east_m / (EARTH_RADIUS_M * math.cos(lat0))
    return math.degrees(lat), math.degrees(lon)


def _vector3_diag_covariance(sigmas) -> list:
    cov = [0.0] * 9
    for i, sigma in enumerate(sigmas):
        cov[4 * i] = sigma * sigma
    return cov


class ImuEmulator:
    """sensor_msgs/Imu as the MTi-670G driver publishes it, from the ground-truth planar motion.

    Gyro and accelerometer carry white noise (density x sqrt(bandwidth) per sample) and a
    random-walk bias. The orientation error is an OU process, as in xsens_noise_node. The
    accelerometer reads the specific force: body acceleration (differenced between emitted samples,
    with the centripetal term) plus gravity on z.
    """

    def __init__(self, rng: np.random.Generator, rate_hz: float, frame_id: str):
        self._rng = rng
        self._frame_id = frame_id
        self._gate = RateGate(rate_hz)
        self._prev = None
        self._attitude = [
            OUState(rng, SIGMA_ROLL_PITCH_RAD, CORRELATION_TIME_S),
            OUState(rng, SIGMA_ROLL_PITCH_RAD, CORRELATION_TIME_S),
            OUState(rng, SIGMA_YAW_RAD, CORRELATION_TIME_S),
        ]
        self._gyro_bias = [RandomWalk(rng, GYRO_BIAS_WALK_RAD_S_SQRTS) for _ in range(3)]
        self._accel_bias = [RandomWalk(rng, ACCEL_BIAS_WALK_M_S2_SQRTS) for _ in range(3)]

    def on_ground_truth(self, stamp: TimeMsg, quat, vx: float, vy: float, yaw_rate: float):
        """Return the Imu sample due at this ground-truth stamp, or None.

        quat is the ground-truth orientation (x, y, z, w); vx, vy the body velocity; yaw_rate the
        body yaw rate.
        """
        t_ns = stamp_ns(stamp)
        if not self._gate.due(t_ns):
            return None

        if self._prev is None:
            accel_x = accel_y = 0.0
            dt = 0.0
        else:
            prev_t, prev_vx, prev_vy = self._prev
            dt = (t_ns - prev_t) * 1e-9
            accel_x = (vx - prev_vx) / dt - yaw_rate * vy
            accel_y = (vy - prev_vy) / dt + yaw_rate * vx
        self._prev = (t_ns, vx, vy)

        rpy = Rotation.from_quat(quat).as_euler('xyz')
        for i, state in enumerate(self._attitude):
            rpy[i] += state.step(dt)
        orientation = Rotation.from_euler('xyz', rpy).as_quat()

        for bias in self._gyro_bias + self._accel_bias:
            bias.step(dt)
        gyro = self._rng.normal(0.0, SIGMA_YAWRATE_RAD_S, 3)
        gyro[0] += self._gyro_bias[0].value
        gyro[1] += self._gyro_bias[1].value
        gyro[2] += yaw_rate + self._gyro_bias[2].value
        specific = self._rng.normal(0.0, SIGMA_ACCEL_M_S2, 3)
        specific[0] += accel_x + self._accel_bias[0].value
        specific[1] += accel_y + self._accel_bias[1].value
        specific[2] += GRAVITY_M_S2 + self._accel_bias[2].value

        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.orientation = Quaternion(x=float(orientation[0]), y=float(orientation[1]),
                                     z=float(orientation[2]), w=float(orientation[3]))
        msg.orientation_covariance = _vector3_diag_covariance(
            [SIGMA_ROLL_PITCH_RAD, SIGMA_ROLL_PITCH_RAD, SIGMA_YAW_RAD])
        msg.angular_velocity.x = float(gyro[0])
        msg.angular_velocity.y = float(gyro[1])
        msg.angular_velocity.z = float(gyro[2])
        msg.angular_velocity_covariance = _vector3_diag_covariance([SIGMA_YAWRATE_RAD_S] * 3)
        msg.linear_acceleration.x = float(specific[0])
        msg.linear_acceleration.y = float(specific[1])
        msg.linear_acceleration.z = float(specific[2])
        msg.linear_acceleration_covariance = _vector3_diag_covariance([SIGMA_ACCEL_M_S2] * 3)

        return msg


class GnssEmulator:
    """sensor_msgs/NavSatFix as the MTi-670G driver publishes it, from the ground-truth position.

    The position error is an OU process per axis (the correlated, bias-like error of the fused
    output), sized from the datasheet figures. The local ENU point is converted to latitude and
    longitude on a
    fixed datum, the same flat-earth model the EKF inverts.
    """

    def __init__(self, rng: np.random.Generator, rate_hz: float, frame_id: str,
                 datum_lat_deg: float, datum_lon_deg: float, datum_alt_m: float):
        self._frame_id = frame_id
        self._gate = RateGate(rate_hz)
        self._datum = (datum_lat_deg, datum_lon_deg, datum_alt_m)
        self._ou = [
            OUState(rng, SIGMA_XY_M, CORRELATION_TIME_S),
            OUState(rng, SIGMA_XY_M, CORRELATION_TIME_S),
            OUState(rng, SIGMA_Z_M, CORRELATION_TIME_S),
        ]
        self._last_t_ns = None

    def on_ground_truth(self, stamp: TimeMsg, x: float, y: float, z: float):
        """Return the NavSatFix due at this stamp, or None. x east, y north, z up (m)."""
        t_ns = stamp_ns(stamp)
        if not self._gate.due(t_ns):
            return None
        dt = 0.0 if self._last_t_ns is None else (t_ns - self._last_t_ns) * 1e-9
        err_x, err_y, err_z = (state.step(dt) for state in self._ou)

        lat_deg, lon_deg = enu_to_latlon(x + err_x, y + err_y, self._datum[0], self._datum[1])
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = lat_deg
        msg.longitude = lon_deg
        msg.altitude = self._datum[2] + z + err_z
        msg.position_covariance = _vector3_diag_covariance([SIGMA_XY_M, SIGMA_XY_M, SIGMA_Z_M])
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self._last_t_ns = t_ns
        return msg


class WheelSpeedEmulator:
    """nav_msgs/Odometry carrying the forward wheel speed, as the CAN node will publish it.

    Only twist.twist.linear.x is set (the EKF reads it). The noise is white, sized to the EKF's
    measurement variance.
    """

    def __init__(self, rng: np.random.Generator, rate_hz: float, base_frame: str):
        self._rng = rng
        self._base_frame = base_frame
        self._gate = RateGate(rate_hz)

    def on_ground_truth(self, stamp: TimeMsg, vx: float):
        """Return the wheel-speed Odometry due at this ground-truth stamp, or None."""
        if not self._gate.due(stamp_ns(stamp)):
            return None
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._base_frame
        msg.child_frame_id = self._base_frame
        msg.twist.twist.linear.x = vx + float(self._rng.normal(0.0, WHEEL_SPEED_SIGMA_M_S))
        msg.twist.covariance[0] = WHEEL_SPEED_SIGMA_M_S * WHEEL_SPEED_SIGMA_M_S
        return msg
