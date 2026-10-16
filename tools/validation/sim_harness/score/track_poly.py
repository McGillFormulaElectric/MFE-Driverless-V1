"""Boundary polygons for a closed EUFS track, from the GT blue/yellow cones in the track csv.

Each colour's cones are chained nearest-neighbour into a loop; if that loop self-intersects (garden_light: the NN
chain jumps across tight sections) the csv row order is used instead when it forms a clean loop. The polygon with
the larger area is the outer boundary, the other the inner; on track = inside outer AND outside inner, each with
`margin` metres of tolerance measured as true distance to the cone line (orientation-independent).

History: up to 2026-10-15 on_track used matplotlib contains_points(radius=+/-margin). That radius is a stroke
WIDTH (offset = margin/2) and its sign depends on the polygon's winding, so on peanut/its_a_mess (yellow CCW outer,
blue CW inner) margin=0.5 really meant +0.25 m outside the outer line and -0.25 m (stricter) at the inner line,
and on clockwise-wound tracks (comp_2021, hairpins) the outer line became -0.25 m. legacy is still the DEFAULT
(so a batch already running keeps one rule); set OFFTRACK_EXACT=1 (run_heldout.sh does) or pass legacy=False
for the exact, orientation-independent margin. Rescore old peanut/its_a_mess bags with OFFTRACK_EXACT=1 to compare.

Points must be in the MAP frame (csv frame). /ground_truth/state_odom is relative to the car_start pose: use
to_map(P, track) first (identity on peanut/its_a_mess/boa_constrictor, whose car_start is 0,0,0).
"""
import csv, os, numpy as np
from matplotlib.path import Path

CSV_DIR = os.environ.get("EUFS_TRACKS_CSV", os.path.join(os.environ.get("EUFS_WS", os.path.expanduser("~/Develop/MFE26-eufs-sim")), "eufs_tracks", "csv"))


def rows(track):
    return list(csv.DictReader(open(f"{CSV_DIR}/{track}.csv")))


def car_start(track):
    cs = [r for r in rows(track) if r["tag"] == "car_start"][0]
    return float(cs["x"]), float(cs["y"]), float(cs["direction"] or 0.0)


def to_map(P, track):
    """odom-frame (car_start-relative) xy -> map/csv frame."""
    x0, y0, yaw = car_start(track)
    c, s = np.cos(yaw), np.sin(yaw); P = np.asarray(P, float)
    return np.c_[x0 + c * P[:, 0] - s * P[:, 1], y0 + s * P[:, 0] + c * P[:, 1]]


def chain(pts):
    left = list(range(len(pts))); order = [left.pop(0)]
    while left:
        d = np.linalg.norm(pts[left] - pts[order[-1]], axis=1)
        order.append(left.pop(int(np.argmin(d))))
    return pts[order]


def self_crossings(q):
    """Number of proper crossings between non-adjacent edges of the closed polygon q."""
    a = q; b = np.roll(q, -1, axis=0); n = len(q)
    # O[i, j] = side of edge i (a_i -> b_i) that point s_j lies on
    side = lambda s: np.sign((b[:, None, 0] - a[:, None, 0]) * (s[None, :, 1] - a[:, None, 1])
                             - (b[:, None, 1] - a[:, None, 1]) * (s[None, :, 0] - a[:, None, 0]))
    Sa, Sb = side(a), side(b)
    X = (Sa * Sb < 0) & (Sa.T * Sb.T < 0)
    i, j = np.triu_indices(n, 2); keep = ~((i == 0) & (j == n - 1))
    return int(X[i[keep], j[keep]].sum())


def area(q):
    return 0.5 * (np.dot(q[:, 0], np.roll(q[:, 1], -1)) - np.dot(q[:, 1], np.roll(q[:, 0], -1)))


def boundary(P):
    """Closed loop through one colour's cones: NN chain unless it self-intersects, then csv order if that is clean."""
    q = chain(P)
    if self_crossings(q) == 0:
        return q, "nn"
    if self_crossings(P) == 0:
        return P, "csv"
    raise ValueError("neither nearest-neighbour chain nor csv order gives a simple polygon")


def polys(track, with_method=False):
    R = rows(track)
    B = np.array([(float(r["x"]), float(r["y"])) for r in R if r["tag"] == "blue"])
    Y = np.array([(float(r["x"]), float(r["y"])) for r in R if r["tag"] == "yellow"])
    (b, mb), (y, my) = boundary(B), boundary(Y)
    return (b, y, mb, my) if with_method else (b, y)


def dist_to_loop(P, q):
    """Distance from each point in P to the closed polyline q."""
    a = q; ab = np.roll(q, -1, axis=0) - q
    ap = P[:, None, :] - a[None, :, :]
    t = np.clip((ap * ab[None]).sum(-1) / np.maximum((ab * ab).sum(-1), 1e-12)[None], 0, 1)
    return np.linalg.norm(ap - t[..., None] * ab[None], axis=-1).min(1)


def on_track(P, track, margin=0.5, legacy=None):
    """P: Nx2 map-frame points. True where inside the outer cone line (or within margin of it) and outside the
    inner cone line (or within margin of it)."""
    if legacy is None:   # default stays legacy so batches already running (run_pt0.sh) keep one scoring rule
        legacy = os.environ.get("OFFTRACK_EXACT", "0") != "1"
    P = np.asarray(P, float)
    b, y = polys(track)
    outer, inner = (y, b) if abs(area(y)) > abs(area(b)) else (b, y)
    if legacy:   # pre-2026-10-15 behaviour, see module docstring
        return Path(outer).contains_points(P, radius=margin) & ~Path(inner).contains_points(P, radius=-margin)
    in_o = Path(outer).contains_points(P); in_i = Path(inner).contains_points(P)
    ok_o = in_o | (dist_to_loop(P, outer) <= margin)
    ok_i = ~in_i | (dist_to_loop(P, inner) <= margin)
    return ok_o & ok_i


if __name__ == "__main__":
    import sys, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    for t in sys.argv[1:]:
        b, y = polys(t); plt.figure(figsize=(7, 7))
        for q, c in ((b, "b"), (y, "y")): plt.plot(*np.vstack([q, q[:1]]).T, c + "-o", ms=3)
        plt.axis("equal"); plt.title(t); out = os.path.join(os.environ.get("RESULTS", os.path.expanduser("~/mfe_sim_results")), "poly_%s.png" % t)
        os.makedirs(os.path.dirname(out), exist_ok=True); plt.savefig(out, dpi=70); plt.close()
