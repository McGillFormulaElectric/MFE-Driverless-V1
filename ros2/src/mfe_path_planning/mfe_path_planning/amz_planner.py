"""Local path planner after AMZ Driverless (Kabzan et al., "AMZ Driverless: The Full Autonomous
Racing System", J. Field Robotics 2020), boundary estimation / path planning section.

Cone-based only: it plans from the cones the car currently knows near it, with no knowledge of
the track ahead. Steps:

1. Delaunay-triangulate the cones within `sensor_range` of the car (and not behind it).
2. Path candidates are sequences of midpoints of triangle edges that cross the track: an edge
   is a candidate if its length is a plausible track width and it does not join two cones of
   the same known colour. Consecutive midpoints must belong to the same triangle.
3. Beam search from the car over that midpoint graph, scoring each path on the paper's cost
   terms: largest heading change between segments, spread of track width, spread of cone
   spacing along each boundary, colour violations (blue on the right / yellow on the left),
   and how far the path length falls short of the sensor range.
4. The best path is smoothed with a spline and resampled for pure pursuit.

Weights are not given in the paper; the defaults here are our own and are exposed as
constructor arguments.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.interpolate import splev, splprep
from scipy.spatial import Delaunay

# Colour codes accepted in `colors` (match fsd_path_planning.ConeTypes values)
UNKNOWN, RIGHT, LEFT = 0, 1, 2   # RIGHT = yellow, LEFT = blue


@dataclass
class _Partial:
    nodes: list            # midpoint indices, in driving order
    tris: set              # triangles already passed through (no revisits)
    cost: float = 0.0
    pts: list = field(default_factory=list)   # car pos + midpoint coordinates


class AmzPlanner:
    def __init__(self, sensor_range: float = 20.0, min_width: float = 2.5, max_width: float = 7.0,
                 beam_width: int = 12, max_depth: int = 12, max_turn_deg: float = 65.0,
                 w_angle: float = 1.0, w_width: float = 1.0, w_spacing: float = 0.5,
                 w_color: float = 2.0, w_length: float = 1.0, resample: float = 0.5):
        self.R = sensor_range
        self.min_w, self.max_w = min_width, max_width
        self.beam, self.depth = beam_width, max_depth
        self.max_turn = np.radians(max_turn_deg)
        self.w = dict(angle=w_angle, width=w_width, spacing=w_spacing, color=w_color, length=w_length)
        self.ds = resample
        self.last_debug = {}

    # ------------------------------------------------------------------ #
    def plan(self, cones: np.ndarray, colors: np.ndarray, pos: np.ndarray, heading: np.ndarray):
        """cones Nx2, colors N (UNKNOWN/RIGHT/LEFT), pos 2, heading unit 2.
        Returns (path Mx2, left_boundary Kx2, right_boundary Kx2) or None."""
        self.last_debug = {}
        rel = cones - pos
        dist = np.hypot(rel[:, 0], rel[:, 1])
        fwd = rel @ heading
        keep = (dist < self.R) & (fwd > -3.0)
        idx = np.flatnonzero(keep)
        if len(idx) < 4:
            self.last_debug["reason"] = f"only {len(idx)} cones in range"
            return None
        P, C = cones[idx], colors[idx]
        try:
            tri = Delaunay(P)
        except Exception as e:  # degenerate (collinear) cone sets
            self.last_debug["reason"] = f"delaunay failed: {e}"
            return None

        # candidate edges -> midpoint nodes
        edge_id: dict = {}
        mids, ends, tri_edges = [], [], []
        for t, s in enumerate(tri.simplices):
            row = []
            for a, b in ((s[0], s[1]), (s[1], s[2]), (s[2], s[0])):
                key = (min(a, b), max(a, b))
                if key not in edge_id:
                    L = float(np.linalg.norm(P[a] - P[b]))
                    ok = self.min_w <= L <= self.max_w and not (C[a] != UNKNOWN and C[a] == C[b])
                    edge_id[key] = len(mids) if ok else -1
                    if ok:
                        mids.append((P[a] + P[b]) / 2)
                        ends.append(key)
                row.append(edge_id[key])
            tri_edges.append(row)
        if not mids:
            self.last_debug["reason"] = "no track-crossing edges"
            return None
        mids = np.array(mids)
        # node -> triangles containing it, and adjacency through shared triangles
        node_tris = [[] for _ in mids]
        for t, row in enumerate(tri_edges):
            for e in row:
                if e >= 0:
                    node_tris[e].append(t)

        # start: midpoints ahead of the car, close enough to be the next gate
        rel_m = mids - pos
        f_m = rel_m @ heading
        d_m = np.hypot(rel_m[:, 0], rel_m[:, 1])
        start = [i for i in np.argsort(d_m) if f_m[i] > -0.5 and d_m[i] < 8.0
                 and self._turn(heading, mids[i] - pos) < self.max_turn][:4]
        if not start:
            self.last_debug["reason"] = "no midpoint ahead of the car"
            return None

        beam = [_Partial([i], set(node_tris[i]), 0.0, [pos.copy(), mids[i]]) for i in start]
        finished = list(beam)
        for _ in range(self.depth - 1):
            grown = []
            for p in beam:
                last = p.nodes[-1]
                prev_dir = p.pts[-1] - p.pts[-2]
                for t in node_tris[last]:
                    for nxt in tri_edges[t]:
                        if nxt < 0 or nxt in p.nodes:
                            continue
                        new_tris = set(node_tris[nxt]) - p.tris
                        if not new_tris and len(p.nodes) > 1:
                            continue   # would go back through a triangle already crossed
                        step = mids[nxt] - p.pts[-1]
                        if np.linalg.norm(step) < 0.3 or self._turn(prev_dir, step) > self.max_turn:
                            continue
                        q = _Partial(p.nodes + [nxt], p.tris | set(node_tris[nxt]), 0.0, p.pts + [mids[nxt]])
                        q.cost = self._cost(q, P, C, ends, final=False)
                        grown.append(q)
            if not grown:
                break
            grown.sort(key=lambda q: q.cost)
            beam = grown[: self.beam]
            finished.extend(beam)

        # Compare paths that reach (close to) the planning horizon; otherwise a one-gate path
        # always wins because every extra segment can only add angle/width/spacing cost.
        lengths = [self._length(q) for q in finished]
        reach = 0.8 * max(lengths)
        best = min((q for q, L in zip(finished, lengths) if L >= reach),
                   key=lambda q: self._cost(q, P, C, ends, final=True))
        pts = np.array(best.pts)
        self.last_debug.update(n_cones=len(P), n_mids=len(mids), n_nodes=len(best.nodes),
                               length=float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1))),
                               cost=float(self._cost(best, P, C, ends, final=True)))
        left, right = self._boundaries(best, P, ends)
        return self._smooth(pts), left, right

    # ------------------------------------------------------------------ #
    @staticmethod
    def _length(q: _Partial) -> float:
        return float(np.sum(np.linalg.norm(np.diff(np.array(q.pts), axis=0), axis=1)))

    @staticmethod
    def _turn(a, b) -> float:
        na, nb = np.linalg.norm(a), np.linalg.norm(b)
        if na < 1e-6 or nb < 1e-6:
            return 0.0
        return float(np.arccos(np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0)))

    def _sides(self, q: _Partial, P, ends):
        """For each midpoint on the path, which edge end is on the left / right of travel."""
        out = []
        pts = q.pts
        for k, n in enumerate(q.nodes):
            a, b = ends[n]
            d = pts[k + 1] - pts[k] if k + 1 < len(pts) else pts[k] - pts[k - 1]
            if k + 2 < len(pts):
                d = pts[k + 2] - pts[k]   # direction through the gate
            cross = d[0] * (P[a][1] - pts[k + 1][1]) - d[1] * (P[a][0] - pts[k + 1][0])
            out.append((a, b) if cross > 0 else (b, a))   # (left, right)
        return out

    def _cost(self, q: _Partial, P, C, ends, final: bool) -> float:
        pts = np.array(q.pts)
        segs = np.diff(pts, axis=0)
        ang = [self._turn(segs[i], segs[i + 1]) for i in range(len(segs) - 1)]
        c_angle = max(ang) if ang else 0.0
        widths = [np.linalg.norm(P[a] - P[b]) for a, b in (ends[n] for n in q.nodes)]
        c_width = float(np.std(widths)) if len(widths) > 1 else 0.0
        sides = self._sides(q, P, ends)
        c_spacing = 0.0
        for col in (0, 1):
            seq = [s[col] for s in sides]
            seq = [c for i, c in enumerate(seq) if i == 0 or c != seq[i - 1]]
            if len(seq) > 2:
                c_spacing += float(np.std(np.linalg.norm(np.diff(P[seq], axis=0), axis=1)))
        viol = sum((C[l] == RIGHT) + (C[r] == LEFT) for l, r in sides)
        c_color = viol / max(1, len(sides))
        length = float(np.sum(np.linalg.norm(segs, axis=1)))
        # partial paths: reward progress so the beam prefers longer consistent paths
        c_length = ((self.R - length) / self.R) ** 2 if final else -0.05 * length
        w = self.w
        return (w["angle"] * c_angle + w["width"] * c_width + w["spacing"] * c_spacing
                + w["color"] * c_color + w["length"] * c_length)

    def _boundaries(self, q: _Partial, P, ends):
        sides = self._sides(q, P, ends)
        left = [s[0] for s in sides]
        right = [s[1] for s in sides]
        dedup = lambda seq: [c for i, c in enumerate(seq) if i == 0 or c != seq[i - 1]]
        return P[dedup(left)], P[dedup(right)]

    def _smooth(self, pts: np.ndarray) -> np.ndarray:
        keep = np.r_[True, np.linalg.norm(np.diff(pts, axis=0), axis=1) > 0.2]
        pts = pts[keep]
        if len(pts) < 3:
            return pts
        try:
            k = min(3, len(pts) - 1)
            tck, u = splprep([pts[:, 0], pts[:, 1]], s=len(pts) * 0.3, k=k)
            length = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
            uu = np.linspace(0, 1, max(4, int(length / self.ds)))
            x, y = splev(uu, tck)
            return np.c_[x, y]
        except Exception:
            return pts
