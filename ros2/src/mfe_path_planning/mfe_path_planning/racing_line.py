"""Race-line optimisers: physics-based PSO and minimum-curvature QP (scipy L-BFGS-B)."""
import math

import numpy as np

try:  # the PSO fitness runs ~6000 times per cycle; compiled it is ~6x faster
    from numba import njit as _njit
except ImportError:
    def _njit(fn):
        return fn

try:
    from scipy.optimize import minimize as _sp_minimize
    _SCIPY_AVAILABLE = True
except ImportError:
    _SCIPY_AVAILABLE = False

# MFE25 vehicle constants (vehicle.yaml)
_M = 268.0        # kg  total mass (vehicle + driver)
_G = 9.81         # m/s^2
_RHO = 1.225      # kg/m^3 air density
_CD = 1.77        # drag coefficient
_CL = 4.26        # downforce coefficient
_AREF = 1.106     # m^2 frontal area
_MU = 1.6         # tyre friction coefficient (slicks, dry)
_CRR = 0.012      # rolling resistance coefficient
_P_MAX = 66700.0  # W  max pack power
_F_ENGINE_MAX = 7676.0  # N  peak tractive force

_W_MAX = 2.0      # cap on half-widths for the curvature optimiser (see _min_curvature_path)


@_njit
def _menger_kappa(path):
    """Menger curvature at each waypoint of path (N, 2)."""
    N = len(path)
    kappa = np.zeros(N)
    for i in range(1, N - 1):
        a, b, c = path[i - 1], path[i], path[i + 1]
        ab = np.linalg.norm(b - a)
        bc = np.linalg.norm(c - b)
        ca = np.linalg.norm(a - c)
        cross = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
        denom = ab * bc * ca
        kappa[i] = (2 * cross / denom) if denom > 1e-9 else 0.0
    kappa[0] = kappa[1]
    kappa[-1] = kappa[-2]
    return kappa


def _grip_speed(kappa):
    """Lateral-grip speed limit (m/s) at each point, aero-corrected."""
    r = np.where(kappa > 1e-4, 1.0 / kappa, 1e6)
    denom = _M - _MU * 0.5 * _RHO * _CL * _AREF * r
    denom = np.maximum(denom, 1.0)
    v2 = _MU * _M * _G * r / denom
    return np.sqrt(np.clip(v2, 0, 900))   # cap at 30 m/s


@_njit
def _backward_pass(v_lat, ds):
    """Backward deceleration pass: ensure the car can brake to each corner speed."""
    v = v_lat.copy()
    for i in range(len(v) - 2, -1, -1):
        f_drag = 0.5 * _RHO * _CD * _AREF * v[i + 1] ** 2
        f_net_brake = (_MU * (_M * _G + 0.5 * _RHO * _CL * _AREF * v[i + 1] ** 2)
                       - f_drag - _CRR * _M * _G)
        a_max_brake = f_net_brake / _M
        v_reachable = math.sqrt(v[i + 1] ** 2 + 2 * a_max_brake * ds[i])
        v[i] = min(v[i], v_reachable)
    return v


@_njit
def _forward_pass(v_after_backward, ds):
    """Forward acceleration pass with pack-power and force limits."""
    v = v_after_backward.copy()
    for i in range(len(v) - 1):
        f_drag = 0.5 * _RHO * _CD * _AREF * v[i] ** 2
        f_engine = min(_F_ENGINE_MAX, _P_MAX / max(v[i], 0.1))
        f_net = f_engine - f_drag - _CRR * _M * _G
        a = max(f_net / _M, 0.0)
        v_next = math.sqrt(v[i] ** 2 + 2 * a * ds[i])
        v[i + 1] = min(v[i + 1], v_next)
    return v


def _lap_time(v, ds):
    """Estimated lap time (s) from velocity profile and arc-lengths."""
    v_avg = 0.5 * (v[:-1] + v[1:])
    v_avg = np.maximum(v_avg, 0.1)
    return float(np.sum(ds / v_avg))


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


def _pso_raceline(centerline_xy, left_xy, right_xy, safety=0.8,
                  n_particles=30, n_iter=200, penalty_weight=5.0):
    """PSO race line: lateral offsets from the centerline that minimise the physics lap time.

    Returns the optimised (N, 2) path, or the centerline when the inputs are too thin.
    """
    N = len(centerline_xy)
    if _too_thin(centerline_xy, left_xy, right_xy):
        return centerline_xy

    nrm = _unit_normals(centerline_xy)
    w_l = _half_widths(centerline_xy, left_xy, safety)
    w_r = _half_widths(centerline_xy, right_xy, safety)

    def fitness(alpha):
        path = centerline_xy + alpha[:, None] * nrm
        penalty = float(np.sum(
            np.maximum(0, alpha - w_l) ** 2 + np.maximum(0, -alpha - w_r) ** 2
        ))
        diff = np.diff(path, axis=0)
        ds = np.linalg.norm(diff, axis=1).clip(1e-6)
        kappa = _menger_kappa(path)
        v_lat = _grip_speed(kappa)
        v = _backward_pass(v_lat, ds)
        v = _forward_pass(v, ds)
        t = _lap_time(v, ds)
        return t + penalty_weight * penalty

    # first particle starts at the centerline (alpha = 0)
    pos = np.zeros((n_particles, N))
    pos[1:] = (np.random.uniform(-1, 1, (n_particles - 1, N))
               * np.minimum(w_l, w_r) * 0.3)
    vel = np.zeros_like(pos)
    pbest = pos.copy()
    pbest_fit = np.array([fitness(p) for p in pos])
    gbest_idx = int(np.argmin(pbest_fit))
    gbest = pbest[gbest_idx].copy()
    gbest_fit = pbest_fit[gbest_idx]

    stagnant = 0
    for it in range(n_iter):
        # dynamic coefficients (Negura et al. 2025)
        frac = it / max(n_iter - 1, 1)
        w_inertia = 0.8 - 0.7 * frac
        c1 = 1.5 - 1.0 * frac
        c2 = 1.5 + 1.0 * frac

        r1 = np.random.rand(n_particles, N)
        r2 = np.random.rand(n_particles, N)
        vel = (w_inertia * vel
               + c1 * r1 * (pbest - pos)
               + c2 * r2 * (gbest - pos))
        pos = pos + vel
        pos = np.clip(pos, -w_r, w_l)

        improved = False
        for i, p in enumerate(pos):
            f = fitness(p)
            if f < pbest_fit[i]:
                pbest[i] = p.copy()
                pbest_fit[i] = f
                if f < gbest_fit:
                    gbest = p.copy()
                    gbest_fit = f
                    improved = True

        stagnant = 0 if improved else stagnant + 1

        # escape local optima: re-seed the worst 20 % of the swarm near gbest
        if stagnant >= 10:
            n_perturb = max(1, n_particles // 5)
            worst_idx = np.argsort(pbest_fit)[-n_perturb:]
            for idx in worst_idx:
                noise = np.random.randn(N) * np.minimum(w_l, w_r) * 0.1
                pos[idx] = np.clip(gbest + noise, -w_r, w_l)
                vel[idx] = 0.0
            stagnant = 0

    return centerline_xy + gbest[:, None] * nrm


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
