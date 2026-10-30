"""
Acceleration midline (core.accel): pairs, the total-least-squares fit and the low-passed straight.

No ROS.
"""
import math

import numpy as np

from mfe_path_planning.core import accel


def _pairs_on_line(y_mid=0.5, xs=(5.0, 10.0, 15.0, 20.0, 25.0)):
    cones = []
    for x in xs:
        cones.append((x, y_mid + 2.5))      # left cone (lateral +2.5)
        cones.append((x, y_mid - 2.5))      # right cone (lateral -2.5)
    return np.array(cones)


def test_pair_midpoints_are_the_pair_centres():
    mids = accel.accel_pair_midpoints(_pairs_on_line(), car_pos=(0.0, 0.0), car_dir=(1.0, 0.0))
    assert len(mids) == 5
    assert np.allclose(mids[:, 1], 0.5)
    assert np.allclose(np.sort(mids[:, 0]), [5.0, 10.0, 15.0, 20.0, 25.0])


def test_fit_recovers_heading_and_offset():
    mids = accel.accel_pair_midpoints(_pairs_on_line(), car_pos=(0.0, 0.0), car_dir=(1.0, 0.0))
    fit = accel.accel_midline_fit(mids, car_dir=(1.0, 0.0))
    assert fit is not None
    theta, d = fit
    assert abs(theta) < 1e-6
    assert abs(d - 0.5) < 1e-6


def test_fit_rejects_too_few_pairs_short_spread_and_large_heading_error():
    car = (0.0, 0.0)
    few = accel.accel_pair_midpoints(_pairs_on_line(xs=(5.0,)), car, (1.0, 0.0))
    assert accel.accel_midline_fit(few, (1.0, 0.0)) is None
    tight = accel.accel_pair_midpoints(_pairs_on_line(xs=(5.0, 6.0)), car, (1.0, 0.0))
    assert accel.accel_midline_fit(tight, (1.0, 0.0)) is None
    angled = np.array([[x, math.tan(math.radians(30.0)) * x] for x in (10.0, 15.0, 20.0)])
    assert accel.accel_midline_fit(angled, (1.0, 0.0)) is None


def test_midline_state_keeps_the_line_without_a_fit_and_starts_from_the_first_fit():
    mid = accel.AccelMidline(alpha=0.3)
    base = np.column_stack([np.linspace(0.0, 30.0, 7), np.zeros(7)])
    out, _ = mid.path(base, np.zeros((0, 2)), car_pos=(0.0, 0.0), car_dir=(1.0, 0.0))
    assert np.array_equal(out, base)                       # no fit yet: library path unchanged
    out, _ = mid.path(base, _pairs_on_line(), car_pos=(0.0, 0.0), car_dir=(1.0, 0.0))
    assert mid.line is not None and abs(mid.line[1] - 0.5) < 1e-6
    assert len(out) == len(base)
    assert np.allclose(out[:, 1], 0.5)                     # the straight sits on the midline
    out2, _ = mid.path(base, np.zeros((0, 2)), car_pos=(0.0, 0.0), car_dir=(1.0, 0.0))
    assert np.allclose(out2[:, 1], 0.5)                    # held without new pairs


def test_pairs_behind_the_car_feed_the_fit():
    """The car drifted 1.5 m right of the corridor; the pairs it has just passed still fit the midline."""
    cones = []
    for x in (-15.0, -10.0, -5.0, 0.0):
        cones.append((x, 2.5))      # left wall
        cones.append((x, -2.5))     # right wall
    cones = np.array(cones)
    car, heading = np.array([0.0, -1.5]), np.array([1.0, 0.0])
    mids = accel.accel_pair_midpoints(cones, car, heading)
    assert len(mids) == 4 and np.allclose(mids[:, 1], 0.0)
    fit, reason = accel.accel_midline_fit_reason(mids, heading)
    assert reason == 'ok' and fit is not None
    theta, d = fit
    n = np.array([-math.sin(theta), math.cos(theta)])
    assert abs(theta) < 1e-6 and abs(d) < 1e-6
    assert abs((n @ car - d) - (-1.5)) < 1e-6                # the car sits 1.5 m right of the fitted line


def test_rejection_reasons_are_reported():
    _, reason = accel.accel_midline_fit_reason(np.array([[5.0, 0.0]]), (1.0, 0.0))
    assert reason.startswith('1 pairs')
    tight = np.array([[5.0, 0.0], [6.0, 0.0]])
    assert accel.accel_midline_fit_reason(tight, (1.0, 0.0))[1].startswith('pairs span')
    angled = np.array([[x, math.tan(math.radians(30.0)) * x] for x in (10.0, 15.0, 20.0)])
    assert accel.accel_midline_fit_reason(angled, (1.0, 0.0))[1].startswith('heading')


def test_midline_path_reports_acceptance():
    mid = accel.AccelMidline(alpha=0.3)
    base = np.column_stack([np.linspace(0.0, 30.0, 7), np.zeros(7)])
    _, status = mid.path(base, np.zeros((0, 2)), car_pos=(0.0, 0.0), car_dir=(1.0, 0.0))
    assert status[0] is False
    _, status = mid.path(base, _pairs_on_line(), car_pos=(0.0, 0.0), car_dir=(1.0, 0.0))
    assert status == (True, 'ok')
