"""Left/right boundary tracing for uncoloured LiDAR cones. Pure numpy/scipy, no ROS.

Cones are labelled by the side of the driven path they lie on. Both walls then grow from a start gate
(beam search over spacing, turn, width, gate skew and no wall crossing). The centreline is the sequence of gate midpoints.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import cKDTree


@dataclass
class TracerParams:
    # Labelling passed cones from the driven trajectory
    label_radius: float = 10.0
    lat_max: float = 4.0           # half the widest track (~5 m) plus car off-centre
    lat_min: float = 0.3           # a cone on the driven line itself has no side
    trail_window: float = 40.0
    trail_step: float = 0.5
    assoc_radius: float = 0.7
    # Wall growth
    start_back: float = 5.0
    d_min: float = 0.5
    d_max: float = 8.0             # up to ~2 consecutive missed cones on an FS wall
    turn_max_deg: float = 110.0    # inner wall of a square corner turns 90 deg
    back_frac: float = 0.5         # the inner wall of a hairpin can step sideways or back past the apex
    local_dir: bool = True
    w_min: float = 2.4             # FS minimum track width 3 m, minus map noise
    w_max: float = 7.0
    w_rel_max: float = 1.7         # and at most this times the recent gate widths
    gate_skew_max_deg: float = 75.0  # a wall that jumps to the other wall makes a gate along the track (~90 deg)
    dir_span: float = 3.0
    infer_r: float = 3.5
    infer_ratio: float = 1.6
    gate_clear: float = 0.6        # cones are never mid-track
    across_max: float = 0.65       # max sideways step towards the other wall, in track widths
    opp_clear: float = 2.0
    max_length: float = 35.0       # planning horizon
    max_steps: int = 60
    branch: int = 2
    beam_width: int = 4
    branch_both: bool = True       # also branch on the other wall's best step (hairpin tips)
    prune_margin: float = 1.5
    branch_margin: float = 0.5
    progress_gain: float = 0.3
    length_tol: float = 2.0
    # Step cost weights (dimensionless, each ~1 at its limit)
    w_angle: float = 0.6
    w_spacing: float = 1.0
    w_width: float = 1.0
    w_rot: float = 1.0
    # output
    resample: float = 0.5


@dataclass
class TraceResult:
    path: np.ndarray               # Mx2, starts at the car, ends at the last paired gate
    left: np.ndarray               # Kx2 left wall (driving order)
    right: np.ndarray              # Kx2 right wall
    horizon: float                 # path length ahead of the car (m)
    labels: np.ndarray             # per input cone: +1 left, -1 right, 0 unassigned
    debug: dict = field(default_factory=dict)


def _seg_intersect(p1, p2, q1, q2) -> bool:
    d1 = p2 - p1
    d2 = q2 - q1
    den = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(den) < 1e-9:
        return False
    w = q1 - p1
    t = (w[0] * d2[1] - w[1] * d2[0]) / den
    u = (w[0] * d1[1] - w[1] * d1[0]) / den
    return 1e-6 < t < 1 - 1e-6 and 1e-6 < u < 1 - 1e-6


def _angles(a, V) -> np.ndarray:
    """Unsigned angles (rad) between vector a and each row of V."""
    na = math.hypot(a[0], a[1])
    nv = np.hypot(V[:, 0], V[:, 1])
    c = (V @ a) / np.maximum(na * nv, 1e-12)
    return np.arccos(np.clip(c, -1.0, 1.0))


class BoundaryTracer:
    def __init__(self, params: TracerParams | None = None):
        self.p = params or TracerParams()
        self._trail: list = []
        self._trail_s: list = []
        self._lab_xy = np.zeros((0, 2))
        self._lab_votes = np.zeros(0)

    def add_pose(self, pos) -> None:
        pos = np.asarray(pos, dtype=float)
        if not self._trail:
            self._trail.append(pos.copy())
            self._trail_s.append(0.0)
            return
        step = float(np.linalg.norm(pos - self._trail[-1]))
        if step >= self.p.trail_step:
            self._trail.append(pos.copy()); self._trail_s.append(self._trail_s[-1] + step)
            if len(self._trail) > 4 * int(self.p.trail_window / self.p.trail_step) + 50:
                k = len(self._trail) // 2
                self._trail = self._trail[k:]
                self._trail_s = self._trail_s[k:]

    def _recent_trail(self, car_pos):
        """Recent driven polyline ending at the car."""
        if not self._trail:
            return np.asarray(car_pos, float)[None]
        s = np.asarray(self._trail_s)
        k = int(np.searchsorted(s, s[-1] - self.p.trail_window))
        T = np.asarray(self._trail[k:])
        if np.linalg.norm(T[-1] - car_pos) > 1e-3:
            T = np.vstack([T, car_pos])
        return T

    def _labels_for(self, xy: np.ndarray) -> np.ndarray:
        """Persistent side-vote sum for each cone in xy (0 if never labelled)."""
        out = np.zeros(len(xy))
        if len(self._lab_xy) == 0 or len(xy) == 0:
            return out
        d, j = cKDTree(self._lab_xy).query(xy, distance_upper_bound=self.p.assoc_radius)
        ok = np.isfinite(d)
        out[ok] = self._lab_votes[j[ok]]
        return out

    def _vote(self, xy: np.ndarray, side: np.ndarray) -> None:
        if len(xy) == 0:
            return
        if len(self._lab_xy):
            d, j = cKDTree(self._lab_xy).query(xy, distance_upper_bound=self.p.assoc_radius)
            hit = np.isfinite(d)
        else:
            hit = np.zeros(len(xy), bool)
            j = np.zeros(len(xy), int)
        for k in np.flatnonzero(hit):
            self._lab_votes[j[k]] += side[k]
            self._lab_xy[j[k]] = xy[k]
        new = ~hit
        if new.any():
            self._lab_xy = np.vstack([self._lab_xy, xy[new]])
            self._lab_votes = np.concatenate([self._lab_votes, side[new].astype(float)])

    def _label_passed(self, xy, car_pos, car_dir) -> None:
        """Vote left/right for cones beside or behind the car, from the driven trajectory."""
        p = self.p
        rel = xy - car_pos
        near = np.flatnonzero(np.einsum('ij,ij->i', rel, rel) < p.label_radius ** 2)
        if len(near) == 0:
            return
        fwd = rel[near] @ car_dir
        T = self._recent_trail(car_pos)
        sides = np.zeros(len(near))
        if len(T) < 3:
            # car has not moved yet: classify cones abeam or behind, in the car frame
            lat = rel[near, 0] * (-car_dir[1]) + rel[near, 1] * car_dir[0]
            ok = (fwd <= 0.5) & (fwd >= -4.0) & (np.abs(lat) >= p.lat_min) & (np.abs(lat) <= p.lat_max)
            sides[ok] = np.sign(lat[ok])
        else:
            A = T[:-1]; D = T[1:] - A
            L2 = np.einsum('ij,ij->i', D, D).clip(1e-9)
            C = xy[near]
            W = C[:, None, :] - A[None, :, :]
            u = np.clip(np.einsum('kij,ij->ki', W, D) / L2[None, :], 0.0, 1.0)
            P = A[None, :, :] + u[..., None] * D[None, :, :]
            dist = np.linalg.norm(C[:, None, :] - P, axis=2)
            i = np.argmin(dist, axis=1)
            ui = u[np.arange(len(near)), i]
            Di = D[i]
            Wi = W[np.arange(len(near)), i]
            lat = (Di[:, 0] * Wi[:, 1] - Di[:, 1] * Wi[:, 0]) / np.sqrt(L2[i])
            dmin = dist[np.arange(len(near)), i]
            beyond_end = (i == len(A) - 1) & (ui >= 1.0 - 1e-6)
            before_start = (i == 0) & (ui <= 1e-6)
            passed = ~before_start & (~beyond_end | (fwd <= 0.0))
            ok = passed & (dmin <= p.lat_max) & (np.abs(lat) >= p.lat_min)
            sides[ok] = np.sign(lat[ok])
        m = sides != 0
        self._vote(xy[near[m]], sides[m])

    def plan(self, xy: np.ndarray, car_pos, car_dir) -> TraceResult | None:
        p = self.p
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        car_pos = np.asarray(car_pos, dtype=float)
        car_dir = np.asarray(car_dir, dtype=float)
        car_dir = car_dir / max(1e-9, float(np.linalg.norm(car_dir)))
        self.debug = {}
        if len(xy) < 2:
            return None
        self._label_passed(xy, car_pos, car_dir)
        votes = self._labels_for(xy)
        lab = np.sign(votes)
        lab = self._infer_labels(xy, lab)

        start = self._start_gate(xy, lab, car_pos, car_dir)
        if start is None:
            self.debug["reason"] = "no start gate (need a cone on each side near the car)"
            return None
        iL, iR, tdir = start
        tree = cKDTree(xy)
        self._nbrs = [np.asarray(n, dtype=int) for n in tree.query_ball_point(xy, p.d_max)]
        self._clear_lab = {}
        for sd in (1, -1):
            pts = xy[lab == sd]
            self._clear_lab[sd] = (cKDTree(pts).query(xy)[0] if len(pts) else np.full(len(xy), np.inf))
        L, R, gates = self._grow(xy, lab, iL, iR, tdir)

        mids = np.array([(xy[a] + xy[b]) / 2.0 for a, b in gates])
        path = self._path_from(mids, car_pos, car_dir)
        labels = lab.copy()
        labels[L] = 1; labels[R] = -1
        self.debug.update(n_gates=len(gates), n_left=len(L), n_right=len(R), n_labelled=int(np.count_nonzero(lab)))
        self.debug["walls"] = (xy[L], xy[R])
        if path is None:
            self.debug["reason"] = "walls end behind the car"
            return None
        horizon = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))
        return TraceResult(path=path, left=xy[L], right=xy[R], horizon=horizon, labels=labels, debug=self.debug)

    def _infer_labels(self, xy, lab):
        """Give unlabelled cones (e.g. beside the start line, first seen when the lap closes) the side of
        the clearly nearest labelled cone. One hop only."""
        p = self.p
        un = np.flatnonzero(lab == 0)
        Lp, Rp = xy[lab > 0], xy[lab < 0]
        if len(un) == 0 or len(Lp) == 0 or len(Rp) == 0:
            return lab
        dl = cKDTree(Lp).query(xy[un])[0]
        dr = cKDTree(Rp).query(xy[un])[0]
        out = lab.copy()
        out[un[(dl <= p.infer_r) & (dr >= p.infer_ratio * dl)]] = 1
        out[un[(dr <= p.infer_r) & (dl >= p.infer_ratio * dr)]] = -1
        return out

    def _start_gate(self, xy, lab, car_pos, car_dir):
        """(left, right) cone gate a few metres behind the car on the driven path."""
        p = self.p
        T = self._recent_trail(car_pos)
        if len(T) >= 3:
            seg = np.linalg.norm(np.diff(T, axis=0), axis=1)
            s = np.concatenate([[0.0], np.cumsum(seg)])
            sb = max(0.0, s[-1] - p.start_back)
            k = int(np.searchsorted(s, sb))
            k = min(max(k, 1), len(T) - 1)
            ref = T[k]
            tdir = T[min(len(T) - 1, k + 2)] - T[max(0, k - 2)]
            if np.linalg.norm(tdir) < 1e-6:
                tdir = car_dir
        else:
            ref, tdir = car_pos, car_dir
        tdir = tdir / np.linalg.norm(tdir)
        rel = xy - ref
        d = np.hypot(rel[:, 0], rel[:, 1])
        fwd = rel @ tdir
        lat = rel[:, 0] * (-tdir[1]) + rel[:, 1] * tdir[0]
        out = []
        for side in (1, -1):
            # labelled cones first; unlabelled ones only if no labelled cone is on that side
            cand = (lab == side) & (d <= p.lat_max + 1.0) & (fwd > -3.0) & (fwd < 4.0)
            if not cand.any():
                cand = (lab == 0) & (np.sign(lat) == side) & (np.abs(lat) >= p.lat_min) & \
                       (np.abs(lat) <= p.lat_max) & (fwd > -3.0) & (fwd < 4.0)
            if not cand.any():
                return None
            idx = np.flatnonzero(cand)
            out.append(int(idx[np.argmin(np.abs(fwd[idx]) + 0.5 * np.abs(lat[idx]))]))
        iL, iR = out
        return iL, iR, tdir

    def _grow(self, xy, lab, iL, iR, tdir):
        """Beam search over zipper growth. Each state adds one cone to the wall whose best new gate is
        shorter. The longest consistent corridor wins, so branches that dead-end (widths past w_max,
        crossing walls) lose."""
        p = self.p
        turn_max = math.radians(p.turn_max_deg)
        rot_max = math.radians(p.gate_skew_max_deg)
        used0 = np.zeros(len(xy), bool)
        used0[iL] = used0[iR] = True
        root = dict(walls={1: [iL], -1: [iR]}, used=used0, gates=[(iL, iR)],
                    widths=[float(np.linalg.norm(xy[iL] - xy[iR]))], tdir=tdir, length=0.0, cost=0.0,
                    mids=[(xy[iL] + xy[iR]) / 2], tdir0=tdir, span=p.dir_span)
        beam = [root]
        done = []
        for _ in range(p.max_steps):
            grown = []
            for st in beam:
                opts = {}
                for side in (1, -1):
                    opts[side] = self._candidates(xy, lab, st["used"], st["walls"], side, st["tdir"],
                                                  st["widths"], turn_max, rot_max, p.branch)
                avail = [sd for sd in (1, -1) if opts[sd]]
                if not avail or st["length"] >= p.max_length:
                    done.append(st)
                    continue
                side = min(avail, key=lambda sd: opts[sd][0][2])
                c0 = opts[side][0][1]
                for k, cost, gw in opts[side]:
                    if cost > c0 + p.branch_margin:
                        break
                    grown.append(self._extend(xy, st, side, k, cost, gw))
                # the other wall may also move first (hairpin tips)
                if p.branch_both and opts[-side]:
                    k, cost, gw = opts[-side][0]
                    grown.append(self._extend(xy, st, -side, k, cost, gw))
            if not grown:
                break
            grown.sort(key=lambda st: st["cost"] - p.progress_gain * st["length"])
            beam, seen = [], set()
            k0 = grown[0]["cost"] - p.progress_gain * grown[0]["length"]
            for st in grown:
                if st["cost"] - p.progress_gain * st["length"] > k0 + p.prune_margin:
                    break
                key = (tuple(st["walls"][1][-2:]), tuple(st["walls"][-1][-2:]))
                if key not in seen:
                    seen.add(key)
                    beam.append(st)
                if len(beam) >= p.beam_width:
                    break
            # branches reconverging on the same wall heads are merged above, keeping the cheaper one
        done.extend(beam)
        Lmax = max(st["length"] for st in done)
        good = [st for st in done if st["length"] >= Lmax - p.length_tol]
        best = min(good, key=lambda st: st["cost"] / max(1, len(st["gates"]) - 1))
        return best["walls"][1], best["walls"][-1], best["gates"]

    @staticmethod
    def _extend(xy, st, side, k, cost, gw):
        walls = {1: list(st["walls"][1]), -1: list(st["walls"][-1])}
        hl, hr = walls[1][-1], walls[-1][-1]
        m_old = (xy[hl] + xy[hr]) / 2
        walls[side].append(k)
        used = st["used"].copy()
        used[k] = True
        hl, hr = walls[1][-1], walls[-1][-1]
        m_new = (xy[hl] + xy[hr]) / 2
        mids = st["mids"] + [m_new]
        length = st["length"] + float(np.linalg.norm(m_new - m_old))
        # Track direction: centreline over the last dir_span metres. The gate normal is too noisy,
        # since zipper gates are diagonal. Blend with the start direction while the centreline is short.
        k = len(mids) - 1
        acc = 0.0
        while k > 0 and acc < st["span"]:
            acc += float(np.linalg.norm(mids[k] - mids[k - 1]))
            k -= 1
        d = mids[-1] - mids[k]
        nd = float(np.linalg.norm(d))
        w = min(1.0, acc / st["span"])
        tdir = w * (d / nd if nd > 1e-6 else st["tdir0"]) + (1.0 - w) * st["tdir0"]
        tdir = tdir / max(1e-9, float(np.linalg.norm(tdir)))
        return dict(walls=walls, used=used, gates=st["gates"] + [(hl, hr)], widths=st["widths"] + [gw],
                    tdir=tdir, length=length, cost=st["cost"] + cost, mids=mids, tdir0=st["tdir0"], span=st["span"])

    def _candidates(self, xy, lab, used, walls, side, tdir, widths, turn_max, rot_max, n_best):
        """Up to n_best feasible next cones for wall `side`, cheapest first: (index, step cost, new gate length)."""
        p = self.p
        W = walls[side]; O = walls[-side]
        h = xy[W[-1]]; ho = xy[O[-1]]
        wdir = h - xy[W[-2]] if len(W) >= 2 else tdir
        if len(W) >= 2:
            spac = sorted(float(np.hypot(*(xy[W[i + 1]] - xy[W[i]]))) for i in range(max(0, len(W) - 6), len(W) - 1))
            s_ref = min(max(spac[len(spac) // 2], 1.5), 4.0)
        else:
            s_ref = 3.0
        wr = sorted(widths[-6:])
        w_ref = wr[len(wr) // 2]
        # Blend the centreline direction with both walls' last steps: the centreline lags in a hairpin.
        if p.local_dir:
            u = []
            for Wk in (walls[1], walls[-1]):
                if len(Wk) >= 2:
                    v = xy[Wk[-1]] - xy[Wk[-2]]; nv = float(np.hypot(v[0], v[1]))
                    if nv > 0.3:
                        u.append(v / nv)
            if u:
                tl = np.mean(u, axis=0) + tdir
                nt = float(np.hypot(tl[0], tl[1]))
                if nt > 0.3:
                    tdir = tl / nt
        nrm = np.array([-tdir[1], tdir[0]])
        nb = self._nbrs[W[-1]]
        cand = nb[~used[nb] & (lab[nb] != -side)]
        if len(cand) == 0:
            return []
        P = xy[cand]
        V = P - h
        dc = np.hypot(V[:, 0], V[:, 1])
        ang = _angles(wdir, V)
        Gn = (P - ho) if side == 1 else (ho - P)
        gw = np.hypot(Gn[:, 0], Gn[:, 1])
        rot = _angles(nrm, Gn)
        # Cones known to be on the other wall: its recent traced cones and every cone labelled for it.
        Oxy = xy[O[-8:]]
        clear = np.minimum(np.min(np.hypot(P[:, None, 0] - Oxy[None, :, 0], P[:, None, 1] - Oxy[None, :, 1]), axis=1),
                           self._clear_lab[-side][cand])
        # A wall step must not cross the track: sideways motion towards the other wall stays
        # well below a track width.
        across = -side * (V @ nrm)
        ok = ((dc >= p.d_min) & (ang <= turn_max) & (V @ tdir >= -p.back_frac * dc) & (gw >= p.w_min) & (gw <= p.w_max) & (gw <= p.w_rel_max * w_ref)
              & (rot <= rot_max) & (clear >= p.opp_clear) & (across <= p.across_max * w_ref))
        if not ok.any():
            return []
        cost = (p.w_angle * (ang / turn_max) ** 2
                + p.w_spacing * np.maximum(0.0, np.log(np.maximum(dc, 1e-3) / s_ref)) ** 2   # only gaps cost: walls tighten in corners
                + p.w_width * ((gw - w_ref) / w_ref) ** 2
                + p.w_rot * (rot / rot_max) ** 2)
        out = []
        for i in np.flatnonzero(ok)[np.argsort(cost[ok])]:
            c = P[i]
            if any(_seg_intersect(h, c, xy[O[j]], xy[O[j + 1]]) for j in range(max(0, len(O) - 6), len(O) - 1)):
                continue
            # Free track: the triangle the step adds holds no cone. Otherwise the candidate sits
            # behind the real wall, e.g. a neighbouring section's wall.
            near = np.concatenate([nb, self._nbrs[O[-1]]])
            if self._cone_inside(xy, near, k_excl=cand[i], tri=(h, c, ho)):
                continue
            # No cone on the new gate: it would graze a cone and cross a wall. The own wall's head is
            # exempt, since at a bend the other wall lags and the gate passes through it.
            if self._cone_on_segment(xy, near[near != W[-1]], c, ho, p.gate_clear):
                continue
            out.append((int(cand[i]), float(cost[i]), float(gw[i])))
            if len(out) >= n_best:
                break
        return out

    @staticmethod
    def _cone_on_segment(xy, near, a, b, clear) -> bool:
        e = b - a
        L2 = float(e @ e)
        if L2 < 1e-9:
            return False
        Q = xy[near] - a
        t = (Q @ e) / L2
        dist = np.abs(Q[:, 0] * e[1] - Q[:, 1] * e[0]) / math.sqrt(L2)
        return bool(np.any((t > 0.1) & (t < 0.9) & (dist < clear)))

    @staticmethod
    def _cone_inside(xy, near, k_excl, tri, margin=0.3) -> bool:
        """Any cone (among `near`) strictly inside triangle tri?"""
        a, b, c = tri
        lo = np.minimum(np.minimum(a, b), c) - 0.1
        hi = np.maximum(np.maximum(a, b), c) + 0.1
        box = near[np.all((xy[near] >= lo) & (xy[near] <= hi), axis=1)]
        if len(box) == 0:
            return False
        P = xy[box]
        def edge(p, q):   # signed distance of P from edge p->q (positive = left)
            e = q - p
            n = math.hypot(e[0], e[1]) or 1e-9
            return (e[0] * (P[:, 1] - p[1]) - e[1] * (P[:, 0] - p[0])) / n
        d1, d2, d3 = edge(a, b), edge(b, c), edge(c, a)
        if (d1 + d2 + d3).sum() < 0:      # clockwise triangle: flip so inside = positive
            d1, d2, d3 = -d1, -d2, -d3
        inside = (d1 > margin) & (d2 > margin) & (d3 > margin)
        inside &= box != k_excl
        return bool(inside.any())

    def _path_from(self, mids, car_pos, car_dir):
        """Gate midpoints -> smoothed path from the car to the last gate, resampled."""
        p = self.p
        if len(mids) < 2:
            return None
        keep = [0]
        for i in range(1, len(mids)):
            if np.linalg.norm(mids[i] - mids[keep[-1]]) > 0.3:
                keep.append(i)
        M = mids[keep]
        if len(M) < 2:
            return None
        # Smooth interior points: zipper midpoints alternate slightly.
        if len(M) >= 3:
            S = M.copy()
            S[1:-1] = 0.25 * M[:-2] + 0.5 * M[1:-1] + 0.25 * M[2:]
            M = S
        A = M[:-1]; D = M[1:] - A
        L2 = np.einsum('ij,ij->i', D, D).clip(1e-9)
        u = np.clip(((car_pos - A) * D).sum(1) / L2, 0, 1)
        P = A + u[:, None] * D
        dist = np.linalg.norm(P - car_pos, axis=1)
        i = int(np.argmin(dist))
        if i == len(A) - 1 and u[i] >= 1.0 - 1e-6:
            return None
        Q = np.vstack([car_pos, M[i + 1:]]) if dist[i] > 0.05 else np.vstack([P[i], M[i + 1:]])
        seg = np.linalg.norm(np.diff(Q, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(seg)])
        if s[-1] < 0.5:
            return None
        ss = np.arange(0.0, s[-1], p.resample)
        ss = np.append(ss, s[-1]) if s[-1] - ss[-1] > 1e-3 else ss
        return np.c_[np.interp(ss, s, Q[:, 0]), np.interp(ss, s, Q[:, 1])]
