"""Race-line optimiser: minimum-curvature path inside the wall corridor (scipy L-BFGS-B)."""
import numpy as np

try:
    from scipy.optimize import minimize as _sp_minimize
    _SCIPY_AVAILABLE = True
except ImportError:
    _SCIPY_AVAILABLE = False


_W_MAX = 2.0      # cap on half-widths for the curvature optimiser (see _min_curvature_path)


def _too_thin(centerline, left, right) -> bool:
    return len(centerline) < 4 or len(left) == 0 or len(right) == 0


def _unit_normals(centerline):
    """Left-pointing unit normals from central-difference tangents."""
    tg = np.zeros_like(centerline)
    tg[1:-1] = centerline[2:] - centerline[:-2]
    tg[0] = centerline[1] - centerline[0]
    tg[-1] = centerline[-1] - centerline[-2]
    tg /= np.linalg.norm(tg, axis=1, keepdims=True).clip(1e-9)
    return np.column_stack([-tg[:, 1], tg[:, 0]])


def _half_widths(centerline, boundary, safety, cap=None):
    """Distance from each centerline point to the nearest boundary cone, times the safety factor."""
    w = np.empty(len(centerline))
    for i in range(len(centerline)):
        w[i] = np.linalg.norm(boundary - centerline[i], axis=1).min()
    if cap is not None:
        w = np.minimum(w, cap)
    return w * safety


def _min_curvature_path(centerline_xy, left_xy, right_xy, safety=0.8):
    """Minimum-curvature racing line (Heilmeier et al. 2019, QP form).

    Each centerline waypoint slides laterally within the track boundaries, minimising the
    sum of squared discrete second differences. Returns the centerline if scipy is missing
    or the inputs are too thin.
    """
    if not _SCIPY_AVAILABLE:
        return centerline_xy
    N = len(centerline_xy)
    if _too_thin(centerline_xy, left_xy, right_xy):
        return centerline_xy

    nrm = _unit_normals(centerline_xy)
    # capped: a gap in a wall makes the nearest-cone distance blow up, and the line would swing through it
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
