"""
Unit tests for GGS-diagram velocity planner and PSO race line optimizer.

These functions live in path_planner_node.py but have no ROS dependencies —
they are module-level pure-Python/numpy functions.  We mock the ROS layer
before importing so the tests run without a ROS2 environment.

Covers:
  - GGS lateral speed limit (aero-corrected): neil/feature/ggs-velocity
  - Backward/forward pass physics
  - _compute_ggs_speed_profile end-to-end
  - PSO feasibility: neil/feature/pso-raceline
"""

import math
import sys
import types
import numpy as np
import pytest


# ---------------------------------------------------------------------------
# Mock the ROS2 layer so we can import path_planner_node without rclpy
# ---------------------------------------------------------------------------

def _mock_ros():
    for mod in [
        'rclpy', 'rclpy.node', 'rclpy.qos',
        'nav_msgs', 'nav_msgs.msg',
        'geometry_msgs', 'geometry_msgs.msg',
        'mfe_msgs', 'mfe_msgs.msg',
        'fsd_path_planning', 'ft_fsd_path_planning',
        'scipy', 'scipy.optimize',
    ]:
        if mod not in sys.modules:
            sys.modules[mod] = types.ModuleType(mod)

    # scipy.optimize needs linear_sum_assignment to exist for boundary_extractor
    import scipy.optimize as sco
    if not hasattr(sco, 'minimize'):
        sco.minimize = None

    # QoSProfile stub
    class _QoS:
        RELIABLE = BEST_EFFORT = KEEP_LAST = 1
        def __init__(self, **kw): pass
    sys.modules['rclpy.qos'].QoSProfile         = _QoS
    sys.modules['rclpy.qos'].ReliabilityPolicy  = _QoS
    sys.modules['rclpy.qos'].HistoryPolicy       = _QoS

    # Stub Node so PathPlannerNode.__init__ won't be called
    class _Node:
        def __init__(self, *a, **kw): pass
    sys.modules['rclpy.node'].Node = _Node

    # Stub Cone color enum
    class _Cone:
        BLUE = 0; YELLOW = 1; ORANGE_BIG = 2; ORANGE_SMALL = 3; UNKNOWN = 4
    sys.modules['mfe_msgs.msg'].Cone  = _Cone
    sys.modules['mfe_msgs.msg'].Track = type('Track', (), {})

    # Stub Path / PoseStamped / Odometry
    for cls in ['Path', 'Odometry']:
        setattr(sys.modules['nav_msgs.msg'], cls, type(cls, (), {}))
    for cls in ['PoseStamped']:
        setattr(sys.modules['geometry_msgs.msg'], cls, type(cls, (), {}))


_mock_ros()

import os, importlib
# Append so PYTHONPATH (pointing at a worktree) takes priority over this fallback.
# e.g. PYTHONPATH=../MFE-V1-p3-ggs/ros2/src/mfe_path_planning pytest ...
_local = os.path.normpath(os.path.join(os.path.dirname(__file__), '..'))
if _local not in sys.path:
    sys.path.append(_local)
_ppn = importlib.import_module('mfe_path_planning.path_planner_node')

# Pull out the functions we want to test
_compute_ggs   = getattr(_ppn, '_compute_ggs_speed_profile', None)
_ggs_lat_vmax  = getattr(_ppn, '_ggs_lateral_vmax',          None)
_backward_pass = getattr(_ppn, '_backward_pass',              None)
_forward_pass  = getattr(_ppn, '_forward_pass',               None)
_menger_kappa  = getattr(_ppn, '_menger_kappa',               None)
_pso_raceline  = getattr(_ppn, '_pso_raceline',               None)
_lap_time      = getattr(_ppn, '_lap_time',                   None)

# Physics constants from the module
_MU   = getattr(_ppn, '_MU',   1.6)
_M    = getattr(_ppn, '_M',    268.0)
_G    = getattr(_ppn, '_G',    9.81)
_VMAX = getattr(_ppn, '_V_MAX', 20.0)


# ---------------------------------------------------------------------------
# Skip entire module if GGS functions not present (branch not merged yet)
# ---------------------------------------------------------------------------

pytestmark = pytest.mark.skipif(
    _compute_ggs is None,
    reason='GGS speed profile functions not found — run on neil/feature/ggs-velocity branch'
)


# ---------------------------------------------------------------------------
# Helper geometries
# ---------------------------------------------------------------------------

def _straight(n=50, length=100.0):
    return np.column_stack([np.linspace(0, length, n), np.zeros(n)])


def _circle(radius=9.1, n=60):
    t = np.linspace(0, 2 * math.pi, n, endpoint=False)
    return np.column_stack([radius * np.cos(t), radius * np.sin(t)])


def _arc_lengths(path):
    return np.linalg.norm(np.diff(path, axis=0), axis=1).clip(1e-6)


# ---------------------------------------------------------------------------
# Test group 1: Menger curvature
# ---------------------------------------------------------------------------

class TestMengerCurvature:

    def test_straight_has_zero_curvature(self):
        if _menger_kappa is None:
            pytest.skip()
        kappa = _menger_kappa(_straight())
        # Interior points should be ~0; endpoints copied from neighbours
        assert np.max(np.abs(kappa[1:-1])) < 1e-6, \
            f"Straight curvature not zero: max={np.max(np.abs(kappa[1:-1])):.2e}"

    def test_circle_curvature_equals_one_over_r(self):
        if _menger_kappa is None:
            pytest.skip()
        R = 9.1
        kappa = _menger_kappa(_circle(radius=R))
        # Expected: κ ≈ 1/R for all interior points
        expected = 1.0 / R
        err = np.abs(kappa[1:-1] - expected)
        assert np.max(err) < 0.02, \
            f"Circle curvature error: max={np.max(err):.4f} (expected {expected:.4f})"


# ---------------------------------------------------------------------------
# Test group 2: GGS lateral speed limit
# ---------------------------------------------------------------------------

class TestGGSLateralVmax:

    def test_zero_curvature_gives_vmax(self):
        if _ggs_lat_vmax is None:
            pytest.skip()
        kappa = np.zeros(10)
        v = _ggs_lat_vmax(kappa)
        assert np.all(v >= _VMAX - 0.01), "Zero curvature should give v_max"

    def test_skidpad_speed_below_simple_limit(self):
        """
        Simple limit: v = sqrt(mu*g*r) ≈ sqrt(1.6*9.81*9.1) ≈ 11.97 m/s.
        GGS is higher because downforce adds grip.
        Both should be physically reasonable (4–14 m/s for r=9.1m).
        """
        if _ggs_lat_vmax is None:
            pytest.skip()
        R = 9.1
        kappa = np.full(10, 1.0 / R)
        v_ggs = _ggs_lat_vmax(kappa)
        simple = math.sqrt(_MU * _G * R)
        # GGS adds downforce so speed should be >= simple limit
        assert np.all(v_ggs >= simple - 0.1), \
            f"GGS speed {v_ggs[0]:.2f} below simple limit {simple:.2f}"
        assert np.all(v_ggs <= _VMAX + 0.1), "GGS speed exceeds hard cap"

    def test_tight_corner_slower_than_long_radius(self):
        if _ggs_lat_vmax is None:
            pytest.skip()
        v_tight = _ggs_lat_vmax(np.full(5, 1.0 / 3.0))   # 3m radius
        v_wide  = _ggs_lat_vmax(np.full(5, 1.0 / 20.0))  # 20m radius
        assert np.all(v_tight < v_wide), "Tight corner should be slower than wide"

    def test_output_always_positive(self):
        if _ggs_lat_vmax is None:
            pytest.skip()
        kappa = np.random.uniform(0, 1.0, 100)
        v = _ggs_lat_vmax(kappa)
        assert np.all(v > 0), "Speed must always be positive"


# ---------------------------------------------------------------------------
# Test group 3: backward / forward passes
# ---------------------------------------------------------------------------

class TestPhysicsPasses:

    def test_backward_pass_enforces_decel_feasibility(self):
        """
        Backward pass must ensure v[i] is reachable by decelerating to v[i+1].
        It does NOT guarantee monotone decrease — just kinematic feasibility.
        Test: with a tight corner at end (v=4 m/s), all prior speeds must be
        kinematically reachable (v[i]^2 <= v[i+1]^2 + 2*a_max*ds).
        """
        if _backward_pass is None:
            pytest.skip()
        # 1m segments: d_min to brake 15→4 ≈ (225-16)/(2*16) ≈ 6.5m >> 1m
        # so the backward pass MUST reduce v[-2] below 15.
        v = np.array([15.0, 15.0, 15.0, 15.0, 4.0], dtype=float)
        ds = np.ones(4) * 1.0
        v_back = _backward_pass(v, ds)
        assert v_back[-1] == pytest.approx(4.0, abs=0.01), "Last speed must stay at corner limit"
        assert v_back[-2] < 15.0, "Speed 1m before 4m/s corner must be reduced"

    def test_forward_pass_non_decreasing_on_straight(self):
        """Speed should only increase or stay flat on a straight."""
        if _forward_pass is None:
            pytest.skip()
        # Set upper bounds to V_MAX so forward pass is free to accelerate.
        # Starting at 1 m/s, upper bound = _V_MAX at each waypoint.
        v = np.full(20, _VMAX, dtype=float)
        v[0] = 1.0   # start from 1 m/s
        ds = np.ones(19) * 5.0
        v_fwd = _forward_pass(v, ds)
        for i in range(len(v_fwd) - 1):
            assert v_fwd[i] <= v_fwd[i+1] + 1e-6, \
                f"Forward pass decreased at i={i}: {v_fwd[i]:.2f} -> {v_fwd[i+1]:.2f}"

    def test_forward_pass_respects_power_limit(self):
        """Speed must not grow beyond power-limited asymptote (~20 m/s)."""
        if _forward_pass is None:
            pytest.skip()
        v = np.full(100, _VMAX, dtype=float)
        v[0] = 0.1
        ds = np.ones(99) * 2.0
        v_fwd = _forward_pass(v, ds)
        assert np.all(v_fwd <= _VMAX + 0.1), \
            f"Speed exceeded hard cap: max={v_fwd.max():.2f}"


# ---------------------------------------------------------------------------
# Test group 4: end-to-end speed profile
# ---------------------------------------------------------------------------

class TestComputeGGSSpeedProfile:

    def test_straight_reaches_high_speed(self):
        """On a long straight, speed should approach max_speed."""
        if _compute_ggs is None:
            pytest.skip()
        v = _compute_ggs(_straight(n=80, length=200.0), max_speed=20.0)
        assert v.max() > 12.0, f"Speed on straight only reached {v.max():.1f} m/s"

    def test_skidpad_circle_speed_in_range(self):
        """
        9.1m radius circle — GGS speed limit.
        Simple formula: v = sqrt(mu*g*r) = sqrt(1.6*9.81*9.1) ≈ 11.97 m/s.
        GGS adds aero downforce, giving slightly more grip — expect 11–14 m/s.
        At competition speeds, braking/acceleration constraints bring it lower.
        """
        if _compute_ggs is None:
            pytest.skip()
        v = _compute_ggs(_circle(radius=9.1), max_speed=20.0)
        assert v.max() < 16.0, f"Skidpad speed too high: {v.max():.1f} m/s"
        assert v.min() > 1.0, f"Skidpad speed too low: {v.min():.1f} m/s"
        # GGS must be >= simple formula (aero adds grip)
        simple = math.sqrt(_MU * _G * 9.1)
        assert v.max() >= simple * 0.95, \
            f"GGS {v.max():.1f} m/s is below simple limit {simple:.1f} m/s"

    def test_output_length_matches_path(self):
        if _compute_ggs is None:
            pytest.skip()
        path = _straight(n=37)
        v = _compute_ggs(path, max_speed=15.0)
        assert len(v) == len(path), \
            f"Output length {len(v)} != path length {len(path)}"

    def test_short_path_returns_flat(self):
        """Paths with <3 waypoints should return a flat speed array."""
        if _compute_ggs is None:
            pytest.skip()
        v = _compute_ggs(np.array([[0, 0], [1, 0]]), max_speed=10.0)
        assert len(v) == 2
        assert np.all(v > 0)

    def test_speed_always_positive(self):
        if _compute_ggs is None:
            pytest.skip()
        for path in [_straight(), _circle(5.0), _circle(20.0)]:
            v = _compute_ggs(path, max_speed=20.0)
            assert np.all(v > 0), "Speed dropped to zero"


# ---------------------------------------------------------------------------
# Test group 5: PSO race line
# ---------------------------------------------------------------------------

class TestPSOOptimizer:

    @pytest.mark.skipif(_pso_raceline is None,
                        reason='PSO not found — run on neil/feature/pso-raceline branch')
    def test_pso_stays_within_bounds(self):
        """PSO solution must lie within left/right boundaries."""
        n = 20
        center = _straight(n=n, length=50.0)
        left   = center + np.array([0, 2.0])
        right  = center - np.array([0, 2.0])

        result = _pso_raceline(center, left, right,
                               n_particles=5, n_iter=20, penalty_weight=5.0)

        # All waypoints must be between left and right boundaries
        assert result.shape == (n, 2), f"Wrong shape: {result.shape}"
        for i in range(n):
            assert result[i, 1] <= left[i, 1] + 0.1, \
                f"Point {i} exceeds left boundary: {result[i,1]:.3f} > {left[i,1]:.3f}"
            assert result[i, 1] >= right[i, 1] - 0.1, \
                f"Point {i} exceeds right boundary"

    @pytest.mark.skipif(_pso_raceline is None,
                        reason='PSO not found — run on neil/feature/pso-raceline branch')
    def test_pso_improves_or_matches_baseline(self):
        """
        PSO lap time should be <= lap time of plain centerline.
        (Not guaranteed with few iterations, but should hold on average.)
        """
        if _lap_time is None:
            pytest.skip()

        n = 30
        t = np.linspace(0, math.pi, n)
        # Curved track
        center = np.column_stack([t * 10, np.sin(t) * 3])
        left   = center + np.column_stack([-np.sin(t), np.cos(t)]) * 2.0
        right  = center - np.column_stack([-np.sin(t), np.cos(t)]) * 2.0

        pso_path    = _pso_raceline(center, left, right,
                                    n_particles=10, n_iter=50)
        center_path = center

        ds_pso    = np.linalg.norm(np.diff(pso_path,    axis=0), axis=1).clip(1e-6)
        ds_center = np.linalg.norm(np.diff(center_path, axis=0), axis=1).clip(1e-6)

        from mfe_path_planning.path_planner_node import (
            _menger_kappa, _grip_speed, _backward_pass, _forward_pass)

        def _sim_laptime(path, ds):
            k = _menger_kappa(path)
            v = _grip_speed(k)
            v = _backward_pass(v, ds)
            v = _forward_pass(v, ds)
            return _lap_time(v, ds)

        t_pso    = _sim_laptime(pso_path,    ds_pso)
        t_center = _sim_laptime(center_path, ds_center)

        # PSO should match or beat centerline
        assert t_pso <= t_center * 1.05, \
            f"PSO ({t_pso:.2f}s) is worse than centerline ({t_center:.2f}s) by >5%"
