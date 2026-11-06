"""Tests for the Xsens and CAN sensor models in sensor_emulation.py.

The GNSS round trip uses the EKF's own fix conversion (local_enu_from_latlon), so the test checks
the emulator against the code the EKF runs, not against a copy of it.
"""
import math

import numpy as np
import pytest
from builtin_interfaces.msg import Time as TimeMsg
from mfe_state_estimation.extended_kalman_filter_node import local_enu_from_latlon

from mfe_eufs_sim import sensor_emulation as se
from mfe_eufs_sim.sensor_emulation import (
    DEFAULT_DATUM_LAT_DEG, DEFAULT_DATUM_LON_DEG, GnssEmulator, ImuEmulator, OUState, RandomWalk,
    RateGate, WheelSpeedEmulator, enu_to_latlon,
)

IDENTITY_QUAT = (0.0, 0.0, 0.0, 1.0)
NS = 1_000_000_000


def _stamp(t_ns: int) -> TimeMsg:
    return TimeMsg(sec=t_ns // NS, nanosec=t_ns % NS)


def _run_imu(duration_ms, rate_hz=100.0, seed=1, quat=IDENTITY_QUAT, vx=0.0, vy=0.0, wz=0.0):
    """Feed ground truth at 1 kHz for duration_ms; return the Imu messages emitted at rate_hz."""
    emu = ImuEmulator(np.random.default_rng(seed), rate_hz, 'imu_link')
    out = []
    for k in range(duration_ms):
        msg = emu.on_ground_truth(_stamp(k * NS // 1000), quat, vx, vy, wz)
        if msg is not None:
            out.append(msg)
    return out


def test_ou_stationary_sigma_matches_request():
    rng = np.random.default_rng(3)
    state = OUState(rng, 0.85, 10.0)
    values = np.array([state.step(0.1) for _ in range(200_000)])
    assert values.std() == pytest.approx(0.85, rel=0.05)


def test_random_walk_grows_with_sqrt_time():
    rng = np.random.default_rng(4)
    walks = [RandomWalk(rng, 1e-3) for _ in range(4000)]
    for walk in walks:
        for _ in range(100):
            walk.step(0.01)  # 1 s in total
    final = np.array([w.value for w in walks])
    assert final.std() == pytest.approx(1e-3, rel=0.05)


def test_gyro_white_noise_matches_density_times_sqrt_bandwidth():
    msgs = _run_imu(60_000, rate_hz=100.0)  # 60 s of ground truth, 6000 IMU samples
    gyro_z = np.array([m.angular_velocity.z for m in msgs])
    # Differencing removes the slow bias walk, leaving sqrt(2) times the white sigma.
    white = np.diff(gyro_z).std() / math.sqrt(2.0)
    assert white == pytest.approx(se.SIGMA_YAWRATE_RAD_S, rel=0.05)


def test_specific_force_reads_gravity_when_stationary():
    msgs = _run_imu(20_000, rate_hz=100.0)
    az = np.array([m.linear_acceleration.z for m in msgs])
    assert az.mean() == pytest.approx(se.GRAVITY_M_S2, abs=5e-3)


def test_centripetal_acceleration_has_the_right_sign():
    # Turning left (yaw rate > 0) at 10 m/s needs 5 m/s^2 towards the left (+y) in the body frame.
    msgs = _run_imu(3000, vx=10.0, wz=0.5)
    ay = np.array([m.linear_acceleration.y for m in msgs])
    ax = np.array([m.linear_acceleration.x for m in msgs])
    assert ay[1:].mean() == pytest.approx(5.0, abs=0.05)
    assert ax[1:].mean() == pytest.approx(0.0, abs=0.05)


def test_longitudinal_acceleration_from_speed_change():
    # Speed ramps at 1 m/s^2 between samples, so the body x acceleration reads 1 m/s^2.
    emu = ImuEmulator(np.random.default_rng(5), 100.0, 'imu_link')
    readings = []
    for k in range(3000):
        t_ns = k * NS // 1000
        msg = emu.on_ground_truth(_stamp(t_ns), IDENTITY_QUAT, t_ns * 1e-9, 0.0, 0.0)
        if msg is not None:
            readings.append(msg.linear_acceleration.x)
    assert np.mean(readings[1:]) == pytest.approx(1.0, abs=0.05)


def test_imu_message_frame_fields_and_covariance():
    (msg, *_) = _run_imu(50, quat=(0.0, 0.0, math.sin(0.25), math.cos(0.25)))
    assert msg.header.frame_id == 'imu_link'
    assert msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0
    cov_rpy = msg.orientation_covariance
    assert cov_rpy[0] == pytest.approx(se.SIGMA_ROLL_PITCH_RAD ** 2)
    assert cov_rpy[4] == pytest.approx(se.SIGMA_ROLL_PITCH_RAD ** 2)
    assert cov_rpy[8] == pytest.approx(se.SIGMA_YAW_RAD ** 2)
    assert msg.angular_velocity_covariance[0] == pytest.approx(se.SIGMA_YAWRATE_RAD_S ** 2)
    assert msg.linear_acceleration_covariance[8] == pytest.approx(se.SIGMA_ACCEL_M_S2 ** 2)
    assert msg.orientation_covariance[1] == 0.0


def test_imu_orientation_follows_ground_truth_yaw_within_datasheet_sigma():
    yaw = 0.25
    msgs = _run_imu(2000, quat=(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
    yaws = []
    for m in msgs:
        q = m.orientation
        yaws.append(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    assert np.mean(yaws) == pytest.approx(yaw, abs=3 * se.SIGMA_YAW_RAD)


def test_imu_rate_is_100_hz_and_gate_picks_samples_from_189_hz_truth():
    ground_truth_hz = 189.0
    emu = ImuEmulator(np.random.default_rng(6), 100.0, 'imu_link')
    emitted = 0
    for k in range(int(10 * ground_truth_hz)):
        t_ns = round(k * NS / ground_truth_hz)
        if emu.on_ground_truth(_stamp(t_ns), IDENTITY_QUAT, 0.0, 0.0, 0.0) is not None:
            emitted += 1
    assert emitted == pytest.approx(1000, abs=2)


def test_gnss_rate_is_5_hz():
    emu = GnssEmulator(np.random.default_rng(7), 5.0, 'gnss_link',
                       DEFAULT_DATUM_LAT_DEG, DEFAULT_DATUM_LON_DEG, 0.0)
    emitted = 0
    for k in range(10_000):
        if emu.on_ground_truth(_stamp(k * NS // 1000), 0.0, 0.0, 0.0) is not None:
            emitted += 1
    assert emitted == pytest.approx(50, abs=1)


def test_rate_gate_skips_stalls_without_bursting():
    gate = RateGate(10.0)
    assert gate.due(0)
    assert not gate.due(50_000_000)
    assert gate.due(1_000_000_000)     # a one-second stall emits once, not ten times
    assert not gate.due(1_050_000_000)
    assert gate.due(1_100_000_000)


def test_gnss_position_error_matches_datasheet_sigma_and_is_correlated():
    datum = (DEFAULT_DATUM_LAT_DEG, DEFAULT_DATUM_LON_DEG)
    emu = GnssEmulator(np.random.default_rng(8), 5.0, 'gnss_link', *datum, 0.0)
    errors_x, errors_y = [], []
    for k in range(20_000):
        fix = emu.on_ground_truth(_stamp(k * NS // 5), 0.0, 0.0, 0.0)
        if fix is not None:
            ex, ey = local_enu_from_latlon(math.radians(fix.latitude), math.radians(fix.longitude),
                                           math.radians(datum[0]), math.radians(datum[1]))
            errors_x.append(ex)
            errors_y.append(ey)
    errors_x = np.array(errors_x)
    assert errors_x.std() == pytest.approx(se.SIGMA_XY_M, rel=0.1)
    assert np.std(errors_y) == pytest.approx(se.SIGMA_XY_M, rel=0.1)
    # Correlated: consecutive 5 Hz fixes stay close, unlike white noise (lag-1 correlation ~ 1).
    lag1 = np.corrcoef(errors_x[:-1], errors_x[1:])[0, 1]
    assert lag1 > 0.9


def test_gnss_fix_message_fields():
    emu = GnssEmulator(np.random.default_rng(9), 5.0, 'gnss_link',
                       DEFAULT_DATUM_LAT_DEG, DEFAULT_DATUM_LON_DEG, 12.0)
    fix = emu.on_ground_truth(_stamp(0), 1.0, 2.0, 0.5)
    assert fix.header.frame_id == 'gnss_link'
    assert fix.status.status == 0           # STATUS_FIX, accepted by the EKF (status < 0 rejects)
    assert fix.position_covariance_type == 2  # COVARIANCE_TYPE_DIAGONAL_KNOWN
    assert fix.position_covariance[0] == pytest.approx(se.SIGMA_XY_M ** 2)
    assert fix.position_covariance[8] == pytest.approx(se.SIGMA_Z_M ** 2)
    assert fix.altitude == pytest.approx(12.0 + 0.5, abs=10 * se.SIGMA_Z_M)


def test_gnss_round_trip_to_ekf_enu_within_1_cm_without_noise():
    rng = np.random.default_rng(10)
    datum = (math.radians(DEFAULT_DATUM_LAT_DEG), math.radians(DEFAULT_DATUM_LON_DEG))
    worst = 0.0
    for _ in range(2000):
        east, north = rng.uniform(-2000.0, 2000.0, 2)
        lat_deg, lon_deg = enu_to_latlon(east, north, DEFAULT_DATUM_LAT_DEG, DEFAULT_DATUM_LON_DEG)
        x, y = local_enu_from_latlon(math.radians(lat_deg), math.radians(lon_deg), *datum)
        worst = max(worst, math.hypot(x - east, y - north))
    assert worst < 0.01


def test_wheel_speed_noise_and_fields():
    emu = WheelSpeedEmulator(np.random.default_rng(11), 100.0, 'base_footprint')
    readings = []
    for k in range(100_000):
        msg = emu.on_ground_truth(_stamp(k * NS // 1000), 8.0)
        if msg is not None:
            readings.append(msg.twist.twist.linear.x)
            last = msg
    readings = np.array(readings)
    assert readings.mean() == pytest.approx(8.0, abs=0.01)
    assert readings.std() == pytest.approx(se.WHEEL_SPEED_SIGMA_M_S, rel=0.05)
    assert last.header.frame_id == 'base_footprint'
    assert last.child_frame_id == 'base_footprint'
    assert last.twist.covariance[0] == pytest.approx(se.WHEEL_SPEED_SIGMA_M_S ** 2)
