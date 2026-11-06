"""
Min-curvature optimiser (core.raceline._min_curvature_path) against the pre-move reference. No ROS.

The reference is the copy of core/racing_line.py that lived in the package before the optimiser
moved into core/raceline.py. Outputs must match it to 1e-9 on straight, constant-radius, hairpin
and S-bend corridors.
"""
import numpy as np
import pytest

from mfe_path_planning.core import raceline as rl_mod

try:
    from scipy.optimize import minimize as _sp_minimize
    _SCIPY_AVAILABLE = True
except ImportError:
    _SCIPY_AVAILABLE = False


_W_MAX = 2.0


def _too_thin(centerline, left, right) -> bool:
    return len(centerline) < 4 or len(left) == 0 or len(right) == 0


def _unit_normals(centerline):
    tg = np.zeros_like(centerline)
    tg[1:-1] = centerline[2:] - centerline[:-2]
    tg[0] = centerline[1] - centerline[0]
    tg[-1] = centerline[-1] - centerline[-2]
    tg /= np.linalg.norm(tg, axis=1, keepdims=True).clip(1e-9)
    return np.column_stack([-tg[:, 1], tg[:, 0]])


def _half_widths(centerline, boundary, safety, cap=None):
    w = np.empty(len(centerline))
    for i in range(len(centerline)):
        w[i] = np.linalg.norm(boundary - centerline[i], axis=1).min()
    if cap is not None:
        w = np.minimum(w, cap)
    return w * safety


def _min_curvature_path(centerline_xy, left_xy, right_xy, safety=0.8):
    if not _SCIPY_AVAILABLE:
        return centerline_xy
    N = len(centerline_xy)
    if _too_thin(centerline_xy, left_xy, right_xy):
        return centerline_xy

    nrm = _unit_normals(centerline_xy)
    w_l = _half_widths(centerline_xy, left_xy, safety, _W_MAX)
    w_r = _half_widths(centerline_xy, right_xy, safety, _W_MAX)
    bounds = [(-w_r[i], w_l[i]) for i in range(N)]

    def _f_and_g(alpha):
        q = centerline_xy + alpha[:, None] * nrm
        d2 = q[2:] - 2.0 * q[1:-1] + q[:-2]
        obj = float(np.einsum('ij,ij->', d2, d2))
        td = 2.0 * d2
        grad = np.zeros(N)
        grad[:-2] += np.einsum('ij,ij->i', td, nrm[:-2])
        grad[1:-1] += np.einsum('ij,ij->i', td, -2.0 * nrm[1:-1])
        grad[2:] += np.einsum('ij,ij->i', td, nrm[2:])
        return obj, grad

    res = _sp_minimize(
        _f_and_g, np.zeros(N), jac=True, bounds=bounds,
        method='L-BFGS-B',
        options={'maxiter': 500, 'ftol': 1e-10, 'gtol': 1e-6},
    )
    return centerline_xy + res.x[:, None] * nrm


def _offset_corridor(centre, half_width):
    tg = np.gradient(centre, axis=0)
    nrm = np.column_stack([-tg[:, 1], tg[:, 0]])
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True)
    return centre, centre + half_width * nrm, centre - half_width * nrm


def _straight():
    x = np.linspace(0.0, 40.0, 81)
    return _offset_corridor(np.column_stack([x, np.zeros_like(x)]), 2.0)


def _corner():
    t = np.linspace(0.0, np.pi / 2, 61)
    return _offset_corridor(np.column_stack([10 * np.cos(t), 10 * np.sin(t)]), 1.5)


def _hairpin():
    a = np.linspace(-np.pi / 2, np.pi / 2, 41)
    arc = np.column_stack([6 * np.cos(a), 6 * np.sin(a)])
    lead = np.column_stack([np.linspace(-10.0, 0.0, 21), np.full(21, -6.0)])
    tail = np.column_stack([np.linspace(0.0, -10.0, 21), np.full(21, 6.0)])
    return _offset_corridor(np.vstack([lead, arc[1:], tail[1:]]), 2.0)


def _s_bend():
    x = np.linspace(0.0, 40.0, 161)
    return _offset_corridor(np.column_stack([x, 3.0 * np.sin(2.0 * np.pi * x / 20.0)]), 1.5)


CORRIDORS = {'straight': _straight, 'corner': _corner, 'hairpin': _hairpin, 's_bend': _s_bend}


@pytest.mark.parametrize('safety', [0.7, 0.8])
@pytest.mark.parametrize('name', sorted(CORRIDORS))
def test_matches_reference(name, safety):
    centre, left, right = CORRIDORS[name]()
    expected = _min_curvature_path(centre, left, right, safety=safety)
    actual = rl_mod._min_curvature_path(centre, left, right, safety=safety)
    np.testing.assert_allclose(actual, expected, rtol=0.0, atol=1e-9)


@pytest.mark.parametrize('name', ['corner', 'hairpin', 's_bend'])
def test_curved_corridors_are_optimised(name):
    centre, left, right = CORRIDORS[name]()
    out = rl_mod._min_curvature_path(centre, left, right, safety=0.8)
    assert np.abs(out - centre).max() > 1e-3
