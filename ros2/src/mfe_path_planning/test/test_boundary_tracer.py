"""
Boundary tracer (core.boundary_tracer): corridor, bend, staggered straight and the no-path reasons.

No ROS.
"""
import math

import numpy as np

from mfe_path_planning.core.boundary_tracer import BoundaryTracer


def _drive(tracer, poses):
    for p in poses:
        tracer.add_pose(np.asarray(p, dtype=float))


def _straight_corridor(half_w=3.0, step=2.0, x_end=60.0):
    """Paired cones along the x axis, 2 m apart on each wall (the tracer needs cones this close)."""
    xs = np.arange(0.0, x_end + 1e-9, step)
    left = np.column_stack([xs, np.full(len(xs), half_w)])
    right = np.column_stack([xs, np.full(len(xs), -half_w)])
    return np.vstack([left, right])


def _drive_x(tracer, x_end):
    _drive(tracer, [(x, 0.0) for x in np.arange(0.0, x_end + 1e-9, 0.5)])


def test_straight_corridor_path_stays_on_centreline():
    tracer = BoundaryTracer()
    _drive_x(tracer, 10.0)
    cones = _straight_corridor()
    res = tracer.plan(cones, car_pos=(10.0, 0.0), car_dir=(1.0, 0.0))
    assert res is not None, tracer.debug
    assert res.horizon >= 20.0
    assert np.all(res.left[:, 1] > 2.0) and np.all(res.right[:, 1] < -2.0)
    assert np.max(np.abs(res.path[:, 1])) < 0.5
    assert np.all(res.path[:, 0] >= 9.5)
    assert res.labels.shape == (len(cones),)
    assert np.count_nonzero(res.labels > 0) >= len(res.left)     # every traced left cone is labelled +1


def test_bend_path_stays_between_walls():
    radius, half_w = 12.0, 3.0
    centre = np.array([0.0, radius])        # car starts at the origin heading +x, the bend turns left
    angles = np.linspace(-math.pi / 2, 0.0, 120)
    trail = [(radius * math.cos(a) + centre[0], radius * math.sin(a) + centre[1]) for a in angles[:60]]
    tracer = BoundaryTracer()
    _drive(tracer, trail)
    arc = np.linspace(-math.pi / 2, math.pi / 4, 40)
    inner = [(radius - half_w) * np.array([math.cos(a), math.sin(a)]) + centre for a in arc]
    outer = [(radius + half_w) * np.array([math.cos(a), math.sin(a)]) + centre for a in arc]
    cones = np.vstack([np.array(inner), np.array(outer)])
    car = trail[-1]
    heading = angles[59] + math.pi / 2
    res = tracer.plan(cones, car_pos=car, car_dir=(math.cos(heading), math.sin(heading)))
    assert res is not None, tracer.debug
    dist_to_centre = np.abs(np.hypot(res.path[:, 0], res.path[:, 1] - radius) - radius)
    assert np.max(dist_to_centre) < 1.0


def test_staggered_straight_still_traces_a_path():
    tracer = BoundaryTracer()
    _drive_x(tracer, 10.0)
    left = np.column_stack([np.arange(0.0, 61.0, 2.0), np.full(31, 3.0)])
    right = np.column_stack([np.arange(1.0, 62.0, 2.0), np.full(31, -3.0)])
    res = tracer.plan(np.vstack([left, right]), car_pos=(10.0, 0.0), car_dir=(1.0, 0.0))
    assert res is not None, tracer.debug
    assert res.horizon >= 20.0
    assert np.max(np.abs(res.path[:, 1])) < 0.8


def test_too_few_cones_returns_none_without_reason():
    tracer = BoundaryTracer()
    assert tracer.plan(np.array([[1.0, 2.0]]), (0.0, 0.0), (1.0, 0.0)) is None
    assert tracer.debug == {}


def test_one_sided_cones_report_no_start_gate():
    tracer = BoundaryTracer()
    _drive_x(tracer, 10.0)
    left_only = np.column_stack([np.arange(0.0, 61.0, 2.0), np.full(31, 3.0)])
    res = tracer.plan(left_only, car_pos=(10.0, 0.0), car_dir=(1.0, 0.0))
    assert res is None
    assert tracer.debug.get('reason') == 'no start gate (need a cone on each side near the car)'


def test_short_corridor_reports_the_path_length_reason():
    tracer = BoundaryTracer()
    _drive_x(tracer, 10.0)
    short = _straight_corridor(x_end=12.0)
    res = tracer.plan(short, car_pos=(10.0, 0.0), car_dir=(1.0, 0.0))
    assert res is None
    reason = tracer.debug.get('reason', '')
    assert reason.startswith('path ') and reason.endswith('m (short corridor)'), reason
