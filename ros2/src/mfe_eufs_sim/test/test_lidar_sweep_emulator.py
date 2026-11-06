"""
Pure-numpy tests for the sweep emulator.

The stack's MotionDistortionCorrector has no Python binding, so its maths is re-implemented here
from include/lidar_cone_detector/motion_distortion.hpp (correct, correctPoint, sweepTime). The
scene is a ring of cone surface points at 8 m, sampled uniformly in azimuth.
"""

import math

import numpy as np
import pytest

from mfe_eufs_sim.lidar_sweep_emulator import emulate_sweep, sweep_times

PERIOD = 0.1
START = -math.pi
SPEED = 10.0
N = 3600
RING_RADIUS = 8.0
SENSOR_X = -0.15  # EUFS URDF mount; the sim emulator's default
CAR_SENSOR_X = 0.0  # sensor_mounts.yaml velodyne frame; the corrector's default on the car

_SEAM_TOL = 1e-6
_YAW_RATE_EPS = 1e-9


def _ring_scene():
    """Instantaneous scan at the sweep start: N points on a ring, ascending azimuth from START."""
    az = START + np.arange(N) * (2.0 * math.pi / N)
    xyz = np.stack([RING_RADIUS * np.cos(az), RING_RADIUS * np.sin(az),
                    np.full(N, 0.5)], axis=1)
    return xyz


def _emulate(xyz, vx, vy, yaw, sensor_x=0.0):
    return emulate_sweep(xyz, vx, vy, yaw, period_s=PERIOD, start_azimuth=START,
                         counter_clockwise=True, sensor_x=sensor_x)


def _sweep_time_from_azimuth(xyz):
    """SweepModel azimuth fallback, as sweepTime() in motion_distortion.hpp."""
    az = np.arctan2(xyz[:, 1], xyz[:, 0])
    delta = az - START
    swept = np.maximum(np.mod(delta + _SEAM_TOL, 2.0 * math.pi) - _SEAM_TOL, 0.0)
    return swept / (2.0 * math.pi) * PERIOD


def _correct(xyz, tau, vx, vy, omega, sensor_x=0.0, sensor_y=0.0):
    """MotionDistortionCorrector::correctPoint for each point, sweep start as reference."""
    delta = -tau
    theta = omega * delta
    if abs(omega) < _YAW_RATE_EPS:
        a = delta
        b = np.zeros_like(delta)
    else:
        a = np.sin(theta) / omega
        b = (1.0 - np.cos(theta)) / omega
    u = vx - omega * sensor_y
    w = vy + omega * sensor_x
    dx = a * u - b * w
    dy = b * u + a * w
    qx = xyz[:, 0] - dx
    qy = xyz[:, 1] - dy
    c = np.cos(theta)
    s = np.sin(theta)
    return np.stack([c * qx + s * qy, -s * qx + c * qy, xyz[:, 2]], axis=1)


def _sweep_displacement(tau, u, w, yaw):
    """Start-frame displacement of the sensor over [0, tau], integrated numerically."""
    s = np.linspace(0.0, 1.0, 2001)[None, :] * tau[:, None]
    theta = yaw * s
    rx = np.cos(theta) * u - np.sin(theta) * w
    ry = np.sin(theta) * u + np.cos(theta) * w
    return np.trapz(rx, s, axis=1), np.trapz(ry, s, axis=1)


def test_corrector_displacement_matches_numeric_integral():
    # Delta = -tau: d(Delta) = -integral_0^tau R(-omega s) v_s ds. Undo the rotation of
    # _correct's output on a zero input point: R(theta) R(-theta) (0 - d) = -d.
    tau = np.linspace(0.0, PERIOD, 50)
    omega = 0.3
    u, w = SPEED, omega * CAR_SENSOR_X
    tx, ty = _sweep_displacement(tau, u, w, -omega)
    theta = omega * (-tau)
    corrected = _correct(np.zeros((len(tau), 3)), tau, SPEED, 0.0, omega,
                         sensor_x=CAR_SENSOR_X)
    rotated_back = np.stack([np.cos(theta) * corrected[:, 0] - np.sin(theta) * corrected[:, 1],
                             np.sin(theta) * corrected[:, 0] + np.cos(theta) * corrected[:, 1]],
                            axis=1)
    assert np.allclose(rotated_back[:, 0], tx, atol=1e-9)
    assert np.allclose(rotated_back[:, 1], ty, atol=1e-9)


def test_points_at_sweep_start_are_unchanged():
    xyz = _ring_scene()
    out, tau, _ = _emulate(xyz, SPEED, 0.0, 0.3)
    assert tau[0] == pytest.approx(0.0, abs=1e-12)
    assert np.allclose(out[0], xyz[0], atol=1e-12)


def test_seam_point_displacement_is_v_times_period():
    # Straight line, no yaw: a point seen just before the seam was measured at ~T, so
    # re-expressed at sweep start it lies v*T behind where it was measured.
    xyz = _ring_scene()[-1:]
    out, _, _ = _emulate(xyz, SPEED, 0.0, 0.0)
    displacement = np.linalg.norm(out[0, :2] - xyz[0, :2])
    assert displacement == pytest.approx(SPEED * PERIOD * (1.0 - 1.0 / N), rel=1e-3)


def test_ray_on_start_edge_is_not_wrapped_to_sweep_end():
    # A sim ray on the start edge, a float rounding error below it, is at sweep time 0.
    start_edge = np.array([[math.cos(START - 1e-7), math.sin(START - 1e-7), 0.0]])
    tau = sweep_times(np.arctan2(start_edge[:, 1], start_edge[:, 0]), START, PERIOD, True)
    assert tau[0] == pytest.approx(0.0, abs=1e-9)


def test_emulated_sweep_time_is_the_measured_azimuth():
    # The point's azimuth in the sensor frame at its sweep time is the sweep angle.
    out, tau, _ = _emulate(_ring_scene(), SPEED, 0.0, 0.3, sensor_x=SENSOR_X)
    assert np.allclose(_sweep_time_from_azimuth(out), tau, atol=1e-8)


def test_emulated_scan_is_time_ordered():
    xyz = _ring_scene()
    _, tau, order = _emulate(xyz, SPEED, 0.0, 0.3)
    assert np.array_equal(order, np.arange(N))
    assert np.all(np.diff(tau[order]) >= 0.0)


def test_non_finite_points_pass_through_last():
    xyz = np.vstack([_ring_scene()[:5], [[np.nan, np.nan, np.nan]]])
    out, tau, order = _emulate(xyz, SPEED, 0.0, 0.3)
    assert np.isnan(out[5]).all()
    assert np.isnan(tau[5])
    assert order[-1] == 5


@pytest.mark.parametrize('yaw', [0.3, 0.0])
@pytest.mark.parametrize('corrector_sensor_x', [SENSOR_X, CAR_SENSOR_X])
def test_emulate_then_correct_with_time_field_recovers_points_within_1cm(yaw, corrector_sensor_x):
    xyz = _ring_scene()
    out, tau, order = _emulate(xyz, SPEED, 0.0, yaw, sensor_x=SENSOR_X)
    corrected = _correct(out[order], tau[order], SPEED, 0.0, yaw, sensor_x=corrector_sensor_x)
    assert np.max(np.abs(corrected - xyz[order])) < 0.01


@pytest.mark.parametrize('yaw', [0.3, 0.0])
def test_emulate_then_correct_with_azimuth_fallback_recovers_points_within_1cm(yaw):
    xyz = _ring_scene()
    out, _, order = _emulate(xyz, SPEED, 0.0, yaw, sensor_x=SENSOR_X)
    tau = _sweep_time_from_azimuth(out[order])
    corrected = _correct(out[order], tau, SPEED, 0.0, yaw, sensor_x=SENSOR_X)
    assert np.max(np.abs(corrected - xyz[order])) < 0.01
