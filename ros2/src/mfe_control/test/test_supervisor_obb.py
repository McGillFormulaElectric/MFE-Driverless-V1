"""
Unit tests for the supervisor's oriented bounding box (OBB) cone hit detection.

Tests the vectorised OBB check added in neil/feature/supervisor-bbox.
No ROS2 runtime needed (conftest.py mocks all imports).
"""

import sys
import os
import math
import numpy as np
import pytest

_local = os.path.normpath(os.path.join(os.path.dirname(__file__), '..'))
if _local not in sys.path:
    sys.path.append(_local)

try:
    from mfe_control.supervisor_node import SupervisorNode
    _SUPERVISOR_AVAILABLE = True
except Exception:
    _SUPERVISOR_AVAILABLE = False


def _make_supervisor(half_length=1.5, half_width=0.8):
    """Instantiate SupervisorNode, bypassing ROS __init__."""
    s = object.__new__(SupervisorNode)
    s._car_x          = 0.0
    s._car_y          = 0.0
    s._car_yaw        = 0.0
    s._car_half_length = half_length
    s._car_half_width  = half_width
    s._cone_xy        = np.zeros((0, 2), dtype=np.float32)
    s._cone_hit       = False
    return s


def _run_obb_check(s, cones_xy):
    """Replicate the OBB check from _watchdog_tick."""
    if len(cones_xy) == 0:
        return False
    cone_arr = np.array(cones_xy, dtype=np.float32)
    dx = cone_arr[:, 0] - s._car_x
    dy = cone_arr[:, 1] - s._car_y
    c, si = math.cos(s._car_yaw), math.sin(s._car_yaw)
    local_x =  dx * c + dy * si
    local_y = -dx * si + dy * c
    inside = (np.abs(local_x) < s._car_half_length) & \
             (np.abs(local_y) < s._car_half_width)
    return bool(inside.any())


pytestmark = pytest.mark.skipif(
    not _SUPERVISOR_AVAILABLE,
    reason='SupervisorNode not available — run on neil/feature/supervisor-bbox branch'
)


# ---------------------------------------------------------------------------
# Test group 1: no-hit cases
# ---------------------------------------------------------------------------

class TestNoHit:

    def test_no_cones_no_hit(self):
        s = _make_supervisor()
        assert not _run_obb_check(s, [])

    def test_cone_far_ahead_no_hit(self):
        """Cone 10m ahead — outside half_length=1.5m."""
        s = _make_supervisor()
        assert not _run_obb_check(s, [(10.0, 0.0)])

    def test_cone_far_to_side_no_hit(self):
        """Cone 5m to the side — outside half_width=0.8m."""
        s = _make_supervisor()
        assert not _run_obb_check(s, [(0.0, 5.0)])

    def test_cone_at_safe_lateral_clearance_no_hit(self):
        """Cone at 1.0m lateral (>0.8m half_width) — no hit."""
        s = _make_supervisor()
        assert not _run_obb_check(s, [(0.0, 1.0)])

    def test_cone_just_outside_front_no_hit(self):
        """Cone at 1.6m ahead (>1.5m half_length) — no hit."""
        s = _make_supervisor()
        assert not _run_obb_check(s, [(1.6, 0.0)])

    def test_cone_diagonal_outside_box_no_hit(self):
        """Cone at (2.0, 1.0) — outside both dimensions."""
        s = _make_supervisor()
        assert not _run_obb_check(s, [(2.0, 1.0)])


# ---------------------------------------------------------------------------
# Test group 2: hit cases
# ---------------------------------------------------------------------------

class TestHit:

    def test_cone_at_car_centre_hit(self):
        s = _make_supervisor()
        assert _run_obb_check(s, [(0.0, 0.0)])

    def test_cone_just_inside_front_hit(self):
        """Cone at 1.4m ahead (< half_length=1.5m) — hit."""
        s = _make_supervisor()
        assert _run_obb_check(s, [(1.4, 0.0)])

    def test_cone_just_inside_side_hit(self):
        """Cone at 0.7m to side (< half_width=0.8m) — hit."""
        s = _make_supervisor()
        assert _run_obb_check(s, [(0.0, 0.7)])

    def test_rear_cone_inside_box_hit(self):
        """Cone behind the car but within half_length — still a hit."""
        s = _make_supervisor()
        assert _run_obb_check(s, [(-1.0, 0.0)])

    def test_side_swipe_hit(self):
        """Cone directly beside the car at 0.5m lateral — side-swipe hit."""
        s = _make_supervisor()
        assert _run_obb_check(s, [(0.0, 0.5)])


# ---------------------------------------------------------------------------
# Test group 3: yaw rotation
# ---------------------------------------------------------------------------

class TestYawRotation:

    def test_cone_ahead_after_90deg_turn(self):
        """
        Car is facing +Y (yaw=π/2). Cone is at (0, 2) in map frame
        → in car frame: local_x=2 (forward), local_y=0 (centred).
        With half_length=1.5, this is outside → no hit.
        """
        s = _make_supervisor()
        s._car_yaw = math.pi / 2
        assert not _run_obb_check(s, [(0.0, 2.0)])

    def test_cone_beside_after_90deg_turn_hit(self):
        """
        Car facing +Y (yaw=π/2). Cone at (0.5, 0) in map frame
        → in car frame: local_x=0 (forward), local_y=-0.5 (left side).
        Inside half_width=0.8 → hit.
        """
        s = _make_supervisor()
        s._car_yaw = math.pi / 2
        assert _run_obb_check(s, [(0.5, 0.0)])

    def test_multiple_cones_one_hit(self):
        """One of three cones is inside the box — should trigger hit."""
        s = _make_supervisor()
        cones = [
            (10.0, 0.0),   # far ahead — no hit
            (0.0, 5.0),    # far side — no hit
            (0.0, 0.3),    # inside box — HIT
        ]
        assert _run_obb_check(s, cones)

    def test_all_cones_outside_no_hit(self):
        """Multiple cones all outside — no hit."""
        s = _make_supervisor()
        cones = [(5.0, 0.0), (-5.0, 0.0), (0.0, 3.0), (0.0, -3.0)]
        assert not _run_obb_check(s, cones)

    def test_hit_consistent_across_yaw_angles(self):
        """
        A cone at (0, 0) (car centre) must always be a hit regardless of yaw.
        """
        s = _make_supervisor()
        for yaw in np.linspace(0, 2 * math.pi, 36):
            s._car_yaw = yaw
            assert _run_obb_check(s, [(0.0, 0.0)]), \
                f"Missed cone at car centre with yaw={math.degrees(yaw):.1f}°"
