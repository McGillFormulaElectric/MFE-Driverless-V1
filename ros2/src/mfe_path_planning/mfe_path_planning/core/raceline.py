"""
Offline min-curvature race line for laps 2+ (pure Python, no ROS).

Port of the TUM prototype (raceline_tum/proto/raceline.py, see KB planning-control/race-line-laps2plus.md).

Inputs, all in the planner's map frame:
  cones_xy  Nx2 union map of every /planning/cones observation since first movement (ConeUnion).
  trail_xy  Mx2 car trail of lap 1, from first movement to the lap-1 completion (raw; build trims it).
  colours   optional N ints, mfe_msgs/Cone colour enum; ORANGE_BIG cones are not used as walls.

build() splits the cones into left and right walls by the sign of the cross product with the trail, builds a
centreline from wall cross-sections, takes widths to the open wall chains (minus 0.7 m half width, 0.3 m safety and
params.extra_margin_m), runs opt_min_curv inside those widths and computes the speed profile.

quadprog is imported only inside opt_min_curv, on the first build() call.
"""
import math
from dataclasses import dataclass

import numpy as np

from ..third_party.tph.calc_head_curv_num import calc_head_curv_num
from ..third_party.tph.calc_splines import calc_splines
from ..third_party.tph.calc_vel_profile import calc_vel_profile
from ..third_party.tph.opt_min_curv import opt_min_curv

COL_BIG_ORANGE = 2             # mfe_msgs/Cone ORANGE_BIG (walls exclude it)
CAR_W = 1.4                    # eufs configDry kinematics.axle_width
HALF_W = CAR_W / 2.0
SAFETY = 0.3                   # clearance from the walls (m)
WHEELBASE = 1.58               # eufs configDry kinematics.l
STEER_MAX = 0.52               # eufs configDry input_ranges.steering.max
KAPPA_BOUND = math.tan(STEER_MAX) / WHEELBASE   # kinematic curvature limit (1/m), 0.362
A_ACC = 3.0                    # eufs configDry acceleration.max (m/s^2)
A_BRK = 10.0                   # eufs configDry acceleration.min, magnitude (m/s^2)
V_GRID_MAX = 30.0              # speed grid top for calc_vel_profile
MASS = 225.0                   # eufs configDry inertia.m (unused with drag 0)
DYN_EXP = 2.0                  # friction ellipse exponent
MERGE_M = 0.5                  # union map: observations this close are one cone (m)
TRIM_FRAC = 0.6                # trail end is searched in its last 40 %
TRAIL_DS = 0.5                 # cross-section spacing along the trail (m)
LAT_AMBIG = 0.5                # cones this close to the trail line take their neighbour's side (m)
WALL_TRAIL_MAX = 4.0           # cones further than this from the trail are not walls (m)
PAIR_MAX = 10.0                # max left+right distance for a centreline point (m)
SEG_S_WIN = 10.0               # wall segments used for a cross-section lie within this trail arc (m)
DS = 1.0                       # race-line point spacing (m)
SMOOTH_SIGMA_PTS = 1.5         # Gaussian smoothing sigma of the centreline (points)


@dataclass(frozen=True)
class RaceLineParams:
    extra_margin_m: float = 0.25   # added to the 1.0 m wall margin
    a_lat: float = 6.0             # lateral acceleration limit for the speed profile (m/s^2)
    v_max: float = 8.0             # speed cap (m/s)


@dataclass
class RaceLine:
    xy: np.ndarray          # (N,2) race line, closed loop, unclosed array, ds spacing, xy[0] = loop start
    v: np.ndarray           # (N,) speed profile (m/s), capped at v_max
    kappa: np.ndarray       # (N,) curvature of the race line (1/m)
    s: np.ndarray           # (N,) arc length from xy[0] (m)
    length_m: float         # loop length (m)
    lap_s: float            # predicted lap time (s), mean-speed integration as in the prototype
    w_left: np.ndarray      # (N,) allowed offset to the left of the centreline (m)
    w_right: np.ndarray     # (N,) allowed offset to the right (m)
    bound_left: np.ndarray  # (N,2) left limit of the race line (xy)
    bound_right: np.ndarray  # (N,2) right limit of the race line (xy)
    centre_xy: np.ndarray    # (N,2) centreline the race line is offset from
    min_side_width_m: float  # min over points of min(w_left, w_right)
    closure_gap_m: float    # centreline closure gap before resampling (m)


class ConeUnion:
    """
    Greedy union of cone observations, in arrival order (incremental form of the prototype's union_cones).

    Each observation joins the nearest cluster within merge_m, else starts a new cluster. A cluster position is the
    centroid of its observations; its colour is the colour of its first observation.
    """

    def __init__(self, merge_m: float = MERGE_M):
        self.merge_m = float(merge_m)
        self.n = 0
        self._cen = np.zeros((64, 2))
        self._sum = np.zeros((64, 2))
        self._cnt = np.zeros(64, dtype=int)
        self._col = np.zeros(64, dtype=int)

    def add(self, xy, colours) -> None:
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        colours = np.asarray(colours, dtype=int).reshape(-1)
        for p, c in zip(xy, colours):
            n = self.n
            if n:
                d = np.hypot(self._cen[:n, 0] - p[0], self._cen[:n, 1] - p[1])
                k = int(np.argmin(d))
                if d[k] <= self.merge_m:
                    self._cnt[k] += 1
                    self._sum[k] = self._sum[k] + p
                    self._cen[k] = self._sum[k] / self._cnt[k]
                    continue
            if n == len(self._cnt):
                grow = 2 * n
                self._cen = np.vstack([self._cen, np.zeros((n, 2))])
                self._sum = np.vstack([self._sum, np.zeros((n, 2))])
                self._cnt = np.concatenate([self._cnt, np.zeros(n, dtype=int)])
                self._col = np.concatenate([self._col, np.zeros(n, dtype=int)])
                assert len(self._cnt) == grow
            self._cen[n] = p
            self._sum[n] = p
            self._cnt[n] = 1
            self._col[n] = c
            self.n = n + 1

    @property
    def xy(self) -> np.ndarray:
        return self._cen[:self.n].copy()

    @property
    def colours(self) -> np.ndarray:
        return self._col[:self.n].copy()

    @property
    def counts(self) -> np.ndarray:
        return self._cnt[:self.n].copy()


def _trim_trail(xy: np.ndarray) -> np.ndarray:
    """
    Lap-1 trail ends a few metres past the start pose.

    Cut it where it passes the start again (last 40 % only), so the centreline closes across the finish line.
    """
    seg = np.hypot(*np.diff(xy, axis=0).T)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    if cum[-1] <= 0.0:
        return xy
    tail = np.where(cum >= TRIM_FRAC * cum[-1])[0]
    dstart = np.hypot(*(xy[tail] - xy[0]).T)
    cut = int(tail[int(np.argmin(dstart))]) + 1
    return xy[:cut]


def _split_sides(xy: np.ndarray, trail: np.ndarray):
    """
    +1 left, -1 right, 0 unassigned (far from the trail).

    Also the trail arc length at each cone's nearest point and the cone's distance to the trail.
    """
    seg = np.hypot(*np.diff(trail, axis=0).T)
    s_trail = np.concatenate([[0.0], np.cumsum(seg)])
    side = np.zeros(len(xy), dtype=int)
    lat = np.zeros(len(xy))
    s_at = np.zeros(len(xy))
    dist = np.zeros(len(xy))
    n = len(trail)
    for k, c in enumerate(xy):
        d = np.hypot(trail[:, 0] - c[0], trail[:, 1] - c[1])
        i = int(np.argmin(d))
        dist[k] = d[i]
        s_at[k] = s_trail[i]
        j0, j1 = max(i - 5, 0), min(i + 5, n - 1)
        T = trail[j1] - trail[j0]
        nt = math.hypot(T[0], T[1])
        if nt < 1e-6 or d[i] > WALL_TRAIL_MAX:
            continue
        cross = (T[0] * (c[1] - trail[i, 1]) - T[1] * (c[0] - trail[i, 0])) / nt
        lat[k] = cross
        side[k] = 1 if cross > 0 else -1
    # cones almost on the trail line (for example behind the start pose) have no reliable side: they take the side
    # of the nearest reliable cone
    amb = (np.abs(lat) < LAT_AMBIG) & (side != 0)
    good = (side != 0) & ~amb
    for k in np.where(amb)[0]:
        if good.any():
            dd = np.hypot(*(xy[good] - xy[k]).T)
            side[k] = side[good][int(np.argmin(dd))]
    return side, s_at, dist


def _dist_to_wall(P: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Distance from each point of P to the open wall chain through the ordered cones poly (no wrap segment)."""
    if len(poly) < 2:
        return np.full(len(P), np.inf)
    A = poly[:-1]
    B = poly[1:]
    AB = B - A
    L2 = np.maximum((AB ** 2).sum(1), 1e-12)
    AP = P[:, None, :] - A[None]
    t = np.clip((AP * AB[None]).sum(2) / L2[None], 0.0, 1.0)
    proj = A[None] + t[..., None] * AB[None]
    return np.hypot(*(P[:, None, :] - proj).transpose(2, 0, 1)).min(1)


def _nearest_wall(p: np.ndarray, W: np.ndarray, sW: np.ndarray, s_p: float, trail_len: float):
    """
    Nearest point on the wall chain W to p, using only segments whose trail arc lies within SEG_S_WIN.

    of s_p (stops a wall pairing across a neighbouring section). Returns (point, distance) or (None, inf).
    """
    if len(W) < 2:
        return None, np.inf
    A, B = W[:-1], W[1:]
    mid = np.abs(0.5 * (sW[:-1] + sW[1:]) - s_p)
    mid = np.minimum(mid, trail_len - mid)
    ok = mid <= SEG_S_WIN
    if not ok.any():
        return None, np.inf
    A, B = A[ok], B[ok]
    AB = B - A
    L2 = np.maximum((AB ** 2).sum(1), 1e-12)
    t = np.clip(((p - A) * AB).sum(1) / L2, 0.0, 1.0)
    q = A + t[:, None] * AB
    d = np.hypot(*(q - p).T)
    k = int(np.argmin(d))
    return q[k], float(d[k])


def _build_centreline(trail, L, R, sL, sR, trail_len):
    """
    Centreline from wall cross-sections: for each trail sample (0.5 m) the midpoint of the nearest left.

    and right wall points. Then a closed 1 m resample and Gaussian smoothing. Returns (xy, closure_gap) or (None,
    gap).
    """
    n = max(int(trail_len / TRAIL_DS), 2)
    s_grid = np.linspace(0.0, trail_len, n)
    seg = np.hypot(*np.diff(trail, axis=0).T)
    s_tr = np.concatenate([[0.0], np.cumsum(seg)])
    P = np.column_stack([np.interp(s_grid, s_tr, trail[:, 0]), np.interp(s_grid, s_tr, trail[:, 1])])
    cen_pts = []
    for p, s_p in zip(P, s_grid):
        qL, dL = _nearest_wall(p, L, sL, s_p, trail_len)
        qR, dR = _nearest_wall(p, R, sR, s_p, trail_len)
        if qL is None or qR is None or dL + dR > PAIR_MAX:
            continue
        cen_pts.append(0.5 * (qL + qR))
    if len(cen_pts) < 3:
        return None, float("nan")
    M = np.array(cen_pts)
    Mc = np.vstack([M, M[:1]])
    seg = np.hypot(*np.diff(Mc, axis=0).T)
    first = np.concatenate([[True], seg > 1e-6])
    Mk = Mc[first]
    cum_k = np.concatenate([[0.0], np.cumsum(seg[seg > 1e-6])])
    total = float(cum_k[-1])
    closure_gap = float(np.hypot(*(M[-1] - M[0])))
    if total < 10.0:
        return None, closure_gap
    nout = int(round(total / DS))
    s_new = np.linspace(0.0, total, nout, endpoint=False)
    xr = np.interp(s_new, cum_k, Mk[:, 0])
    yr = np.interp(s_new, cum_k, Mk[:, 1])
    r = int(math.ceil(3 * SMOOTH_SIGMA_PTS))
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / SMOOTH_SIGMA_PTS) ** 2)
    k /= k.sum()
    xs = np.convolve(np.pad(xr, r, mode="wrap"), k, mode="valid")
    ys = np.convolve(np.pad(yr, r, mode="wrap"), k, mode="valid")
    return np.column_stack([xs, ys]), closure_gap


def _profile(xy: np.ndarray, a_lat: float, v_max: float):
    """Curvature, element lengths and speed profile of a closed path (xy unclosed)."""
    ggv = np.array([[0.0, A_BRK, a_lat], [V_GRID_MAX, A_BRK, a_lat]])
    axm = np.array([[0.0, A_ACC], [V_GRID_MAX, A_ACC]])
    el = np.hypot(*(np.roll(xy, -1, axis=0) - xy).T)
    _, kappa = calc_head_curv_num(path=xy, el_lengths=el, is_closed=True)
    v = calc_vel_profile(ax_max_machines=axm, kappa=kappa, el_lengths=el, closed=True, drag_coeff=0.0,
                         m_veh=MASS, ggv=ggv, v_max=v_max, dyn_model_exp=DYN_EXP)
    v_mid = np.maximum(0.5 * (v + np.roll(v, -1)), 0.1)
    return v, kappa, el, float(np.sum(el / v_mid))


def build(cones_xy, trail_xy, params: RaceLineParams = None, colours=None):
    """
    Build the closed race line from the union cone map and the lap-1 trail, or None when no race line can be built.

    Raises on unexpected numerical errors (the caller decides the fallback); returns None on degenerate input.
    """
    params = params or RaceLineParams()
    cones = np.asarray(cones_xy, dtype=float).reshape(-1, 2)
    col = np.zeros(len(cones), dtype=int) if colours is None else np.asarray(colours, dtype=int).reshape(-1)
    trail = _trim_trail(np.asarray(trail_xy, dtype=float).reshape(-1, 2))
    if len(trail) < 3 or len(cones) < 4:
        return None

    side, s_at, mdist = _split_sides(cones, trail)
    keep = (side != 0) & (mdist <= WALL_TRAIL_MAX) & (col != COL_BIG_ORANGE)
    lm = keep & (side > 0)
    rm = keep & (side < 0)
    oL = np.argsort(s_at[lm], kind="stable")
    oR = np.argsort(s_at[rm], kind="stable")
    L, sL = cones[lm][oL], s_at[lm][oL]
    R, sR = cones[rm][oR], s_at[rm][oR]

    trail_len = float(np.hypot(*np.diff(trail, axis=0).T).sum())
    cen, closure_gap = _build_centreline(trail, L, R, sL, sR, trail_len)
    if cen is None:
        return None

    margin = HALF_W + SAFETY + params.extra_margin_m
    wl = np.maximum(_dist_to_wall(cen, L) - margin, 0.0)
    wr = np.maximum(_dist_to_wall(cen, R) - margin, 0.0)

    reftrack = np.column_stack([cen, wr, wl])
    path_cl = np.vstack([cen, cen[:1]])
    _, _, A, normvec = calc_splines(path=path_cl)
    alpha, _curv_err = opt_min_curv(reftrack, normvec, A, KAPPA_BOUND, 0.0, closed=True)
    race = cen + alpha[:, None] * normvec

    v, kappa, el, lap_s = _profile(race, params.a_lat, params.v_max)
    return RaceLine(
        xy=race,
        v=v,
        kappa=kappa,
        s=np.concatenate([[0.0], np.cumsum(el[:-1])]),
        length_m=float(el.sum()),
        lap_s=lap_s,
        w_left=wl,
        w_right=wr,
        bound_left=cen + wl[:, None] * normvec,
        bound_right=cen - wr[:, None] * normvec,
        centre_xy=cen,
        min_side_width_m=float(min(wl.min(), wr.min())),
        closure_gap_m=closure_gap,
    )


def project(rl: RaceLine, pos, last_idx=None, window_pts: int = 60, max_dist_m: float = 2.0):
    """
    Index of the race-line point nearest to pos and its distance.

    The search first looks within window_pts of last_idx (so a loop that passes close to itself cannot pull the
    index across), then globally. Returns (index, distance); the caller compares the distance with max_dist_m.
    """
    xy = rl.xy
    n = len(xy)
    d = np.hypot(xy[:, 0] - pos[0], xy[:, 1] - pos[1])
    if last_idx is not None:
        cand = (int(last_idx) + np.arange(-window_pts, window_pts + 1)) % n
        k = int(cand[int(np.argmin(d[cand]))])
        if d[k] <= max_dist_m:
            return k, float(d[k])
    k = int(np.argmin(d))
    return k, float(d[k])


def segment_indices(rl: RaceLine, idx: int, window_m: float) -> np.ndarray:
    """
    Return the indices of the race-line points from idx forward, wrapping around the closed loop, up to the first.

    point at least window_m of arc length ahead (so the segment is window_m long, plus at most one point spacing).
    """
    n = len(rl.xy)
    order = (int(idx) + np.arange(n)) % n
    el = np.diff(np.append(rl.s, rl.length_m))          # element lengths: point i to point i + 1
    cum = np.concatenate([[0.0], np.cumsum(el[order])[:-1]])  # arc length from idx to each point in order
    k = min(int(np.searchsorted(cum, window_m, side="left")), n - 1)
    return order[:k + 1]
