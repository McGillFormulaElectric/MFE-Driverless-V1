"""Acceleration midline: a straight fitted to the midline of the mapped left/right cone pairs (pure numpy)."""
import math

import numpy as np

from mfe_common.geometry import arc_length, wrap_angle

# acceleration midline refit (accel_refit_midline): the straight follows the midline of mapped left/right cone pairs
_ACCEL_PAIR_LAT_M = (2.0, 5.0)     # opposite-side lateral separation accepted as one pair (m)
_ACCEL_PAIR_LON_M = 3.0            # longitudinal offset allowed between the two cones of a pair (m)
_ACCEL_FIT_RANGE_M = 60.0          # only pairs within this distance of the car, ahead or behind, are fitted (m)
_ACCEL_MIN_SPREAD_M = 6.0          # midpoints must span this far along the line (2 pairs 3 m apart: -5.7 deg fit)
_ACCEL_MAX_DEV_DEG = 10.0          # a fit this far from the car heading is rejected (deg)
_ACCEL_MAX_STEP_DEG = 1.0          # a fit this far from the held line in one cycle is ignored for that cycle (deg)


def accel_pair_midpoints(cones_xy, car_pos, car_dir, lat=_ACCEL_PAIR_LAT_M, lon=_ACCEL_PAIR_LON_M,
                         rng=_ACCEL_FIT_RANGE_M) -> np.ndarray:
    """
    Midpoints (Nx2, map frame) of left/right cone pairs within rng m of the car, ahead or behind.

    Pairs behind the car (the start gate, once driven past) are kept: the car may sit to one side of the corridor
    and the cones it has just passed still show where the walls are. Side = sign of the lateral offset in the car
    frame. Each left cone pairs with its nearest right cone that lies within `lon` m along the car and `lat` m across.
    Pure numpy, no ROS.
    """
    cones = np.asarray(cones_xy, dtype=float).reshape(-1, 2)
    if len(cones) < 2:
        return np.zeros((0, 2))
    fwd = np.asarray(car_dir, dtype=float)
    fwd = fwd / np.linalg.norm(fwd)
    lft = np.array([-fwd[1], fwd[0]])
    rel = cones - np.asarray(car_pos, dtype=float)
    lon_m = rel @ fwd
    lat_m = rel @ lft
    in_window = np.abs(lon_m) < rng
    left = np.where(in_window & (lat_m > 0.0))[0]
    right = np.where(in_window & (lat_m < 0.0))[0]
    mids = []
    for i in left:
        if len(right) == 0:
            break
        dlon = np.abs(lon_m[right] - lon_m[i])
        dlat = lat_m[i] - lat_m[right]
        ok = np.where((dlon <= lon) & (dlat >= lat[0]) & (dlat <= lat[1]))[0]
        if len(ok) == 0:
            continue
        j = ok[int(np.argmin(np.hypot(dlon[ok], dlat[ok])))]
        mids.append(0.5 * (cones[i] + cones[right[j]]))
    return np.array(mids, dtype=float).reshape(-1, 2)


def accel_midline_fit(mids_xy, car_dir, min_pairs=2, min_spread=_ACCEL_MIN_SPREAD_M,
                      max_dev_deg=_ACCEL_MAX_DEV_DEG):
    """
    Total least-squares line through the pair midpoints (see accel_midline_fit_reason).

    Returns (theta, d): heading theta (rad, map frame) and offset d with n . x = d, n = (-sin theta, cos theta), or
    None when the fit is rejected.
    """
    return accel_midline_fit_reason(mids_xy, car_dir, min_pairs, min_spread, max_dev_deg)[0]


def accel_midline_fit_reason(mids_xy, car_dir, min_pairs=2, min_spread=_ACCEL_MIN_SPREAD_M,
                             max_dev_deg=_ACCEL_MAX_DEV_DEG):
    """
    Fit a line through the pair midpoints and say why it was accepted or rejected.

    Returns ((theta, d) or None, reason). reason is 'ok', or the rejection: too few pairs, the midpoints span less
    than min_spread along the line, or the heading is more than max_dev_deg off the car heading.
    """
    m = np.asarray(mids_xy, dtype=float).reshape(-1, 2)
    if len(m) < min_pairs:
        return None, f'{len(m)} pairs (need {min_pairs})'
    fwd = np.asarray(car_dir, dtype=float)
    fwd = fwd / np.linalg.norm(fwd)
    c = m.mean(axis=0)
    _, _, vt = np.linalg.svd(m - c, full_matrices=False)
    u = vt[0]
    if u @ fwd < 0.0:
        u = -u
    span = float(np.ptp(m @ u))
    if span < min_spread:
        return None, f'pairs span {span:.1f} m (need {min_spread:.1f} m)'
    theta = math.atan2(u[1], u[0])
    dev = wrap_angle(theta - math.atan2(fwd[1], fwd[0]))
    if abs(dev) > math.radians(max_dev_deg):
        return None, f'heading {math.degrees(dev):.1f} deg off the car (max {max_dev_deg:.0f} deg)'
    n = np.array([-u[1], u[0]])
    return (theta, float(n @ c)), 'ok'


class AccelMidline:
    """
    Low-passed midline state for the acceleration straight (accel_refit_midline).

    Heading and offset are low-passed with gain alpha; a fit that jumps more than _ACCEL_MAX_STEP_DEG from the
    held line is ignored.
    """

    def __init__(self, alpha: float):
        self.alpha = float(alpha)
        self.line = None            # (theta, d) low-passed midline, or None before the first fit

    def path(self, centerline: np.ndarray, cones_xy, car_pos, car_dir):
        """
        Return (path, (accepted, reason)): the straight that follows the midline of the mapped cone pairs near the car.

        ft-fsd's AccelerationRelocalizer fits only the left cones, once, and the path inherits that tilt. With fewer
        than 2 pairs the previous line is kept; before the first fit the library path is returned unchanged.
        """
        fit, reason = accel_midline_fit_reason(accel_pair_midpoints(cones_xy, car_pos, car_dir), car_dir)
        if fit is not None and self.line is not None:
            step = (fit[0] - self.line[0] + math.pi) % (2.0 * math.pi) - math.pi
            if abs(step) > math.radians(_ACCEL_MAX_STEP_DEG):
                fit = None                      # a jump this large is a few-pair artefact: hold the line
                reason = f'step {math.degrees(abs(step)):.1f} deg from the held line (max {_ACCEL_MAX_STEP_DEG:.0f})'
        if fit is not None:
            theta, d = fit
            if self.line is None:
                self.line = (theta, d)
            else:
                th0, d0 = self.line
                dth = (theta - th0 + math.pi) % (2.0 * math.pi) - math.pi
                self.line = (th0 + self.alpha * dth, d0 + self.alpha * (d - d0))
        elif self.line is None:
            return centerline, (False, reason)
        theta, d = self.line
        u = np.array([math.cos(theta), math.sin(theta)])
        n = np.array([-u[1], u[0]])
        car = np.asarray(car_pos, dtype=float)
        foot = car - (n @ car - d) * n          # the car's projection onto the line
        s = np.linspace(0.0, max(arc_length(centerline), 1.0), len(centerline))
        return foot + s[:, None] * u[None, :], (fit is not None, reason)
