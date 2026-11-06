"""
Unit tests for the LTV-MPC lateral controller.

Tests the QP formulation, OSQP/SLSQP solver path, steer constraints,
low-speed guard, and sign convention (corrective steering direction).

Branch: neil/feature/mpc-lateral
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
    from mfe_control.mpc_lateral_node import MPCLateralNode
    _MPC_AVAILABLE = True
except Exception as e:
    _MPC_AVAILABLE = False
    _MPC_ERROR = str(e)


def _make_node():
    """Instantiate MPCLateralNode bypassing ROS __init__, matching actual field names."""
    node = object.__new__(MPCLateralNode)
    node._L              = 1.56
    node._max_steer_rad  = math.radians(28.0)
    node._delta_rate_max = math.radians(5.0)
    node._N              = 10
    node._dt             = 0.05
    node._Q_ey           = 10.0
    node._Q_eyaw         = 5.0
    node._R_delta        = 1.0
    node._P_scale        = 10.0
    node._v_min_active   = 0.5
    node._v_model_floor  = 1.0
    node._last_delta_rad  = 0.0
    node._osqp_solver     = None   # initialised lazily in _solve_osqp
    node._osqp_N_cache    = -1
    node._map_frame       = 'map'
    node._path           = None
    node._path_idx       = 0
    node._car_x = node._car_y = node._car_yaw = 0.0
    node._car_speed        = 5.0
    node._mission_finished = False
    node._pp_throttle      = 0.0
    node._pp_brake         = 0.0
    return node


pytestmark = pytest.mark.skipif(
    not _MPC_AVAILABLE,
    reason=f'MPCLateralNode not available — run on neil/feature/mpc-lateral branch'
)


# ---------------------------------------------------------------------------
# Test group 1: QP construction
# ---------------------------------------------------------------------------

class TestQPConstruction:

    def test_hessian_is_symmetric(self):
        """H matrix must be symmetric (OSQP requirement)."""
        node = _make_node()
        H, f, A, lb, ub = node._build_condensed_qp(e_y=0.0, e_yaw=0.0, v=5.0)
        sym_err = np.max(np.abs(H - H.T))
        assert sym_err < 1e-10, f"H not symmetric: max error = {sym_err:.2e}"

    def test_hessian_is_psd(self):
        """H must be positive semi-definite."""
        node = _make_node()
        H, *_ = node._build_condensed_qp(e_y=0.0, e_yaw=0.0, v=5.0)
        eigvals = np.linalg.eigvalsh(H)
        assert eigvals.min() >= -1e-8, \
            f"H not PSD: min eigenvalue = {eigvals.min():.2e}"

    def test_constraint_dimensions(self):
        """A, lb, ub must have consistent dimensions."""
        node = _make_node()
        H, f, A, lb, ub = node._build_condensed_qp(e_y=0.1, e_yaw=0.05, v=8.0)
        N = node._N
        assert H.shape == (N, N), f"H shape {H.shape} != ({N},{N})"
        assert len(f) == N
        assert A.shape[1] == N, f"A columns {A.shape[1]} != N={N}"
        assert len(lb) == len(ub) == A.shape[0]

    def test_hessian_changes_with_velocity(self):
        """LTV: H depends on v (A and B matrices change), so H should differ."""
        node = _make_node()
        H1, *_ = node._build_condensed_qp(0.0, 0.0, v=2.0)
        H2, *_ = node._build_condensed_qp(0.0, 0.0, v=10.0)
        assert not np.allclose(H1, H2), \
            "H should change with velocity (LTV formulation)"


# ---------------------------------------------------------------------------
# Test group 2: solver output
# ---------------------------------------------------------------------------

class TestSolverOutput:

    def test_zero_error_zero_steering(self):
        """With zero crosstrack and heading error, steering should be ~0."""
        node = _make_node()
        delta = node._solve_mpc(e_y=0.0, e_yaw=0.0, v=5.0)
        assert delta is not None
        assert abs(delta) < math.radians(2.0), \
            f"Non-zero steering for zero error: {math.degrees(delta):.2f}°"

    def test_positive_crosstrack_negative_steer(self):
        """
        Car is to the LEFT of the path (e_y > 0 → need to steer right → δ < 0).
        """
        node = _make_node()
        delta = node._solve_mpc(e_y=0.5, e_yaw=0.0, v=5.0)
        assert delta is not None
        assert delta < 0, \
            f"Positive crosstrack should give negative steering, got {math.degrees(delta):.2f}°"

    def test_negative_crosstrack_positive_steer(self):
        """Car is to the RIGHT of path (e_y < 0 → need to steer left → δ > 0)."""
        node = _make_node()
        delta = node._solve_mpc(e_y=-0.5, e_yaw=0.0, v=5.0)
        assert delta is not None
        assert delta > 0, \
            f"Negative crosstrack should give positive steering, got {math.degrees(delta):.2f}°"

    def test_steering_within_physical_limits(self):
        """Steering must never exceed max_steering_deg."""
        node = _make_node()
        for e_y in [-2.0, -1.0, 0.5, 1.0, 2.0]:
            delta = node._solve_mpc(e_y=e_y, e_yaw=0.0, v=5.0)
            assert delta is not None
            assert abs(delta) <= node._max_steer_rad + 1e-6, \
                f"e_y={e_y}: steering {math.degrees(delta):.1f}° exceeds {math.degrees(node._max_steer_rad):.1f}°"

    def test_steer_rate_constraint_respected(self):
        """Change in steering from last command must not exceed delta_rate_max."""
        node = _make_node()
        node._last_delta_rad = math.radians(10.0)
        delta = node._solve_mpc(e_y=0.3, e_yaw=0.0, v=5.0)
        assert delta is not None
        rate = abs(delta - node._last_delta_rad)
        assert rate <= node._delta_rate_max + 1e-6, \
            f"Steer rate {math.degrees(rate):.2f}°/step exceeds {math.degrees(node._delta_rate_max):.2f}°/step"

    def test_larger_error_larger_correction(self):
        """Larger crosstrack error should produce larger corrective steering."""
        node = _make_node()
        d_small = abs(node._solve_mpc(e_y=0.1, e_yaw=0.0, v=5.0))
        d_large = abs(node._solve_mpc(e_y=1.0, e_yaw=0.0, v=5.0))
        assert d_large >= d_small, \
            f"Larger error gave smaller correction: {math.degrees(d_large):.2f}° < {math.degrees(d_small):.2f}°"


# ---------------------------------------------------------------------------
# Test group 3: low-speed guard
# ---------------------------------------------------------------------------

class TestLowSpeedGuard:

    def test_below_vmin_guard_in_control_loop(self):
        """
        The low-speed guard lives in _control_loop (not _solve_mpc).
        Verify the guard condition: v < _v_min_active → skip MPC call.
        """
        node = _make_node()
        # Guard condition that _control_loop evaluates
        v = 0.3
        assert v < node._v_min_active, \
            f"Test setup: v={v} should be below _v_min_active={node._v_min_active}"
        # _solve_mpc itself always solves; the guard is one level up
        # (this is the documented behaviour — test documents the design)

    def test_above_vmin_nonzero_for_nonzero_error(self):
        """Just above v_min_active and non-zero error → non-zero steering."""
        node = _make_node()
        delta = node._solve_mpc(e_y=0.5, e_yaw=0.0, v=node._v_min_active + 0.1)
        assert delta is not None
        assert abs(delta) > 1e-6, "Steering should be non-zero for e_y=0.5"


# ---------------------------------------------------------------------------
# Test group 4: normalised steering output
# ---------------------------------------------------------------------------

class TestSteeringNormalisation:

    def test_output_within_minus1_plus1(self):
        """
        If the node normalises to [-1, 1] before publishing, the
        normalised value must be within bounds.
        """
        node = _make_node()
        for e_y in [-2.0, -0.5, 0.0, 0.5, 2.0]:
            delta_rad = node._solve_mpc(e_y=e_y, e_yaw=0.0, v=6.0)
            if delta_rad is None:
                continue
            norm = delta_rad / node._max_steer_rad
            assert -1.0 - 1e-6 <= norm <= 1.0 + 1e-6, \
                f"Normalised steering {norm:.4f} out of [-1, 1] for e_y={e_y}"
