"""
Pure-numpy tests for the sweep emulator.

The stack's MotionDistortionCorrector has no Python binding, so its maths is re-implemented here
from include/lidar_cone_detector/motion_distortion.hpp (correctMotionDistortion and
correctPointMotion). The scene is a ring of cone surface points at 8 m, sampled uniformly in
azimuth so that the corrector's index-based time model is exact for a full sweep.
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


def _ring_scene():
    """Instantaneous scan at the sweep start: N points on a ring, ascending azimuth from START."""
    az = START + np.arange(N) * (2.0 * math.pi / N)
    xyz = np.stack([RING_RADIUS * np.cos(az), RING_RADIUS * np.sin(az),
                    np.full(N, 0.5)], axis=1)
    return xyz


def _emulate(xyz, vx, vy, yaw, sensor_x=0.0):
    return emulate_sweep(xyz, vx, vy, yaw, period_s=PERIOD, start_azimuth=START,
                         counter_clockwise=True, sensor_x=sensor_x)


def _sweep_displacement(tau, u, w, yaw):
    """Start-frame displacement of the sensor over [0, tau], integrated numerically."""
    s = np.linspace(0.0, 1.0, 2001)[None, :] * tau[:, None]
    theta = yaw * s
    rx = np.cos(theta) * u - np.sin(theta) * w
    ry = np.sin(theta) * u + np.cos(theta) * w
    return np.trapz(rx, s, axis=1), np.trapz(ry, s, axis=1)


def _inverse_to_sweep_start(out_xyz, tau, yaw, u, w):
    """Undo the emulation: rotate by +theta and add the start-frame displacement."""
    tx, ty = _sweep_displacement(tau, u, w, yaw)
    theta = yaw * tau
    c = np.cos(theta)
    s = np.sin(theta)
    x = out_xyz[:, 0]
    y = out_xyz[:, 1]
    return np.stack([c * x - s * y + tx, s * x + c * y + ty, out_xyz[:, 2]], axis=1)


def _stack_corrector(cloud, vx, vy, omega, scan_duration=PERIOD):
    """MotionDistortionCorrector::correctMotionDistortion, time t_i = i / N * T (index based)."""
    n = len(cloud)
    t = np.arange(n) / n * scan_duration
    dt = scan_duration - t
    c = np.cos(omega * dt)
    s = np.sin(omega * dt)
    x = cloud[:, 0]
    y = cloud[:, 1]
    return np.stack([c * x - s * y + vx * dt, s * x + c * y + vy * dt, cloud[:, 2]], axis=1)


def test_points_at_sweep_start_are_unchanged():
    xyz = _ring_scene()
    out, _ = _emulate(xyz, SPEED, 0.0, 0.3)
    tau = sweep_times(np.arctan2(xyz[:, 1], xyz[:, 0]), START, PERIOD, True)
    assert tau[0] == pytest.approx(0.0, abs=1e-12)
    assert np.allclose(out[0], xyz[0], atol=1e-12)


def test_seam_point_displacement_is_v_times_period():
    # Straight line, no yaw: a point seen just before the seam was measured at ~T, so
    # re-expressed at sweep start it lies v*T behind where it was measured.
    xyz = _ring_scene()[-1:]
    out, _ = _emulate(xyz, SPEED, 0.0, 0.0)
    displacement = np.linalg.norm(out[0, :2] - xyz[0, :2])
    assert displacement == pytest.approx(SPEED * PERIOD * (1.0 - 1.0 / N), rel=1e-3)


def test_ray_on_start_edge_is_not_wrapped_to_sweep_end():
    # A sim ray on the start edge, a float rounding error below it, is at sweep time 0.
    start_edge = np.array([[math.cos(START - 1e-7), math.sin(START - 1e-7), 0.0]])
    tau = sweep_times(np.arctan2(start_edge[:, 1], start_edge[:, 0]), START, PERIOD, True)
    assert tau[0] == pytest.approx(0.0, abs=1e-9)


def test_emulated_scan_is_time_ordered():
    xyz = _ring_scene()
    _, order = _emulate(xyz, SPEED, 0.0, 0.3)
    assert np.array_equal(order, np.arange(N))


def test_non_finite_points_pass_through_last():
    xyz = np.vstack([_ring_scene()[:5], [[np.nan, np.nan, np.nan]]])
    out, order = _emulate(xyz, SPEED, 0.0, 0.3)
    assert np.isnan(out[5]).all()
    assert order[-1] == 5


def test_emulated_points_invert_to_instant_scan_within_1cm():
    xyz = _ring_scene()
    out, order = _emulate(xyz, SPEED, 0.0, 0.3, sensor_x=-0.15)
    tau = sweep_times(np.arctan2(xyz[:, 1], xyz[:, 0]), START, PERIOD, True)[order]
    u = SPEED
    w = 0.0 + 0.3 * -0.15
    recovered = _inverse_to_sweep_start(out, tau, 0.3, u, w)
    assert np.max(np.abs(recovered - xyz[order])) < 0.01


@pytest.mark.xfail(strict=True, reason=(
    'MotionDistortionCorrector (motion_distortion.hpp) applies translation +v*dt with '
    'dt = T - t, so for an emulated (time-ascending) scan it moves points the wrong way. '
    'Fails until the stack corrector sign or index order is fixed.'))
def test_stack_corrector_as_written_recovers_points_within_1cm():
    xyz = _ring_scene()
    out, order = _emulate(xyz, SPEED, 0.0, 0.1)
    corrected = _stack_corrector(out[order], SPEED, 0.0, 0.1)
    assert np.max(np.abs(corrected - xyz[order])) < 0.01


def test_stack_corrector_recovers_points_only_with_descending_index_order():
    # The corrector's index fraction is read as time-to-end. Its maths is the start-frame
    # transform, which is only exact when index 0 is the last point of the sweep.
    xyz = _ring_scene()
    out, order = _emulate(xyz, SPEED, 0.0, 0.1)
    corrected = _stack_corrector(out[order][::-1], SPEED, 0.0, 0.1)
    assert np.max(np.abs(corrected - xyz[order][::-1])) < 0.01
