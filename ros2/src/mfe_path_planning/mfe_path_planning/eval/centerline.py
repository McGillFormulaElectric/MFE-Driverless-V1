"""
Ground truth centerline builder from EUFS track CSV cones (pure Python, no ROS).

Computes smooth, uniformly resampled centerlines with per-point track widths
and signed curvature for static EUFS tracks (closed circuits, acceleration
straight, and FSAE skidpad) and procedural tracks.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import math
from pathlib import Path
from typing import Optional

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial import Delaunay

from mfe_path_planning.eval.tracks import (
    CenterlineData,
    STATIC_TRACKS,
    TrackData,
    find_eufs_tracks_dir,
    load_track_csv,
    save_centerline_csv,
)


def _compute_curvature(xy: np.ndarray, closed: bool) -> np.ndarray:
    """Compute signed curvature (1/m) along a uniformly sampled 2D polyline."""
    n = len(xy)
    if n < 3:
        return np.zeros(n, dtype=np.float64)
    if closed:
        p_prev = np.roll(xy, 1, axis=0)
        p_next = np.roll(xy, -1, axis=0)
    else:
        p_prev = np.vstack([xy[0:1], xy[:-1]])
        p_next = np.vstack([xy[1:], xy[-1:]])

    v1 = xy - p_prev
    v2 = p_next - xy
    v3 = p_next - p_prev
    cross = v1[:, 0] * v2[:, 1] - v1[:, 1] * v2[:, 0]
    denom = (
        np.hypot(v1[:, 0], v1[:, 1])
        * np.hypot(v2[:, 0], v2[:, 1])
        * np.hypot(v3[:, 0], v3[:, 1])
    )
    kappa = np.zeros(n, dtype=np.float64)
    np.divide(2.0 * cross, denom, out=kappa, where=denom > 1e-9)
    if not closed and n >= 3:
        kappa[0] = kappa[1]
        kappa[-1] = kappa[-2]
    return kappa


def _estimate_widths(
    xy: np.ndarray,
    tangents: np.ndarray,
    blue: np.ndarray,
    yellow: np.ndarray,
    default_half_width: float = 2.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Estimate left (blue) and right (yellow) perpendicular half-widths."""
    n = len(xy)
    lw = np.full(n, default_half_width, dtype=np.float64)
    rw = np.full(n, default_half_width, dtype=np.float64)

    normals = np.column_stack([-tangents[:, 1], tangents[:, 0]])

    def _side_width(cones: np.ndarray, sign: float) -> np.ndarray:
        out = np.full(n, default_half_width, dtype=np.float64)
        if len(cones) == 0:
            return out
        for i in range(n):
            rel = cones - xy[i]
            lon = rel @ tangents[i]
            lat = (rel @ normals[i]) * sign
            # Cones within +-4 m longitudinally and on the expected side
            mask = (np.abs(lon) <= 4.0) & (lat > 0.2) & (lat < 8.0)
            if np.any(mask):
                cand_d = np.hypot(lon[mask], lat[mask])
                best_k = np.argsort(cand_d)[:2]
                out[i] = float(np.median(lat[mask][best_k]))
            else:
                dists = np.hypot(rel[:, 0], rel[:, 1])
                nearest = int(np.argmin(dists))
                out[i] = max(0.5, float(dists[nearest]))
        return out

    lw = _side_width(blue, +1.0)
    rw = _side_width(yellow, -1.0)
    return lw, rw


def _resample_spline(
    raw_xy: np.ndarray,
    closed: bool,
    ds: float = 0.5,
) -> tuple[np.ndarray, np.ndarray]:
    """Resample an ordered polyline via cubic spline at uniform arc spacing ds."""
    pts = [raw_xy[0]]
    for p in raw_xy[1:]:
        if np.hypot(*(p - pts[-1])) > 1e-4:
            pts.append(p)
    arr = np.asarray(pts, dtype=np.float64)
    if closed and np.hypot(*(arr[-1] - arr[0])) > 1e-4:
        arr = np.vstack([arr, arr[0:1]])

    seg = np.hypot(np.diff(arr[:, 0]), np.diff(arr[:, 1]))
    s_raw = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(s_raw[-1])
    if total < 1e-3 or len(arr) < 4:
        return s_raw, arr

    bc = 'periodic' if closed else 'not-a-knot'
    cs_x = CubicSpline(s_raw, arr[:, 0], bc_type=bc)
    cs_y = CubicSpline(s_raw, arr[:, 1], bc_type=bc)

    n_out = max(int(round(total / ds)), 10)
    if closed:
        s_new = np.linspace(0.0, total, n_out, endpoint=False)
    else:
        s_new = np.linspace(0.0, total, n_out + 1, endpoint=True)
    xy_new = np.column_stack([cs_x(s_new), cs_y(s_new)])
    return s_new, xy_new


def _smooth_points(xy: np.ndarray, closed: bool, passes: int = 2) -> np.ndarray:
    """Apply gentle 3-point binomial smoothing to suppress cone-stagger zigzags."""
    pts = xy.copy()
    if len(pts) < 5:
        return pts
    for _ in range(passes):
        if closed:
            pts = 0.25 * np.roll(pts, 1, axis=0) + 0.5 * pts + 0.25 * np.roll(pts, -1, axis=0)
        else:
            nxt = pts.copy()
            nxt[1:-1] = 0.25 * pts[:-2] + 0.5 * pts[1:-1] + 0.25 * pts[2:]
            pts = nxt
    return pts


def _build_acceleration_centerline(track: TrackData, ds: float = 0.5) -> CenterlineData:
    """Build exact straight-line centerline for the EUFS acceleration track."""
    sx, sy, yaw = track.car_start
    u = np.array([math.cos(yaw), math.sin(yaw)], dtype=np.float64)
    n_vec = np.array([-u[1], u[0]], dtype=np.float64)

    # Fit midline offset from paired blue/yellow cones
    if len(track.blue) > 0 and len(track.yellow) > 0:
        blue_lat = float(np.median((track.blue - np.array([sx, sy])) @ n_vec))
        yellow_lat = float(np.median((track.yellow - np.array([sx, sy])) @ n_vec))
        mid_lat = 0.5 * (blue_lat + yellow_lat)
        half_w = 0.5 * abs(blue_lat - yellow_lat)
    else:
        mid_lat = 0.0
        half_w = 1.5

    all_c = track.all_cones
    max_lon = float(np.max((all_c - np.array([sx, sy])) @ u)) if len(all_c) > 0 else 80.0
    end_lon = max(max_lon - 2.0, 20.0)

    n_pts = max(int(round(end_lon / ds)), 10)
    s = np.linspace(0.0, end_lon, n_pts + 1)
    origin = np.array([sx, sy]) + mid_lat * n_vec
    xy = origin[None, :] + s[:, None] * u[None, :]
    lw = np.full(len(s), half_w, dtype=np.float64)
    rw = np.full(len(s), half_w, dtype=np.float64)
    kappa = np.zeros(len(s), dtype=np.float64)
    return CenterlineData(s=s, xy=xy, left_width=lw, right_width=rw, curvature=kappa, closed=False)


def _build_skidpad_centerline(track: TrackData, ds: float = 0.5) -> CenterlineData:
    """
    Build canonical FSAE skidpad centerline in the CSV's coordinate frame.

    Sequence: entry straight -> 2x left circle (CCW) -> 2x right circle (CW) -> exit straight.
    """
    sx, sy, yaw = track.car_start
    # Derive circle centre X and radius directly from track cones (handles EUFS CSV geometry)
    # In EUFS skidpad.csv: car_start=(0, -14.4, pi/2), circles centred at (-9.25, 0) & (+9.25, 0)
    # with inner R=7.6125, outer R=10.6125 -> centerline R=9.1125, half-width=1.5 m.
    cx_abs = 9.25
    radius = 9.1125
    half_w = 1.5

    if len(track.midpoints) >= 10:
        pos_x = track.midpoints[track.midpoints[:, 0] > 1.0]
        if len(pos_x) >= 5:
            # Farthest midpoint on +X side sits at (cx + R, 0)
            max_x = float(np.max(pos_x[:, 0]))
            max_y = float(np.max(np.abs(pos_x[:, 1])))
            if 7.0 < max_y < 11.0 and 15.0 < max_x < 22.0:
                radius = max_y
                cx_abs = max_x - radius

    # Entry: (0, sy) -> (0, 0)
    entry_len = abs(sy) if abs(sy) > 1.0 else 14.4
    n_entry = max(int(round(entry_len / ds)), 10)
    entry = np.column_stack([np.zeros(n_entry), np.linspace(sy, 0.0, n_entry, endpoint=False)])

    # Left circle: centre (-cx_abs, 0), CCW starting at angle 0 -> 4*pi (2 full laps)
    circ_len = 2.0 * math.pi * radius
    n_lap = max(int(round(circ_len / ds)), 40)
    th_l = np.linspace(0.0, 4.0 * math.pi, 2 * n_lap, endpoint=False)
    left_circ = np.column_stack([
        -cx_abs + radius * np.cos(th_l),
        radius * np.sin(th_l),
    ])

    # Right circle: centre (+cx_abs, 0), CW starting at angle pi -> -3*pi (2 full laps)
    th_r = np.linspace(math.pi, -3.0 * math.pi, 2 * n_lap, endpoint=False)
    right_circ = np.column_stack([
        cx_abs + radius * np.cos(th_r),
        radius * np.sin(th_r),
    ])

    # Exit: (0, 0) -> (0, 19.0)
    exit_end = 19.0
    if len(track.orange) > 0:
        exit_end = min(float(np.max(track.orange[:, 1])) - 2.0, 20.0)
    n_exit = max(int(round(exit_end / ds)), 10)
    exit_seg = np.column_stack([np.zeros(n_exit + 1), np.linspace(0.0, exit_end, n_exit + 1)])

    xy = np.vstack([entry, left_circ, right_circ, exit_seg])
    seg = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    s = np.concatenate([[0.0], np.cumsum(seg)])
    lw = np.full(len(s), half_w, dtype=np.float64)
    rw = np.full(len(s), half_w, dtype=np.float64)
    kappa = _compute_curvature(xy, closed=False)
    return CenterlineData(s=s, xy=xy, left_width=lw, right_width=rw, curvature=kappa, closed=False)


def _trace_closed_delaunay(
    track: TrackData,
    max_edge_m: float = 9.0,
) -> np.ndarray:
    """
    Extract ordered closed-loop centerline midpoints via Delaunay blue-yellow cross-edges.

    Each Delaunay triangle containing both blue and yellow vertices has exactly two
    blue-yellow cross-edges whose midpoints are adjacent along the corridor.
    """
    blue = track.blue
    yellow = track.yellow
    if len(blue) < 3 or len(yellow) < 3:
        raise ValueError(f'Track {track.name} needs >=3 blue and >=3 yellow cones')

    # Include start-gate big_orange cones classified by side of car_start heading if helpful
    pts = np.vstack([blue, yellow])
    n_blue = len(blue)
    is_blue = np.zeros(len(pts), dtype=bool)
    is_blue[:n_blue] = True

    tri = Delaunay(pts)

    # Map canonical cross-edge (b_idx, y_idx) -> midpoint index, and build adjacency
    edge_id: dict[tuple[int, int], int] = {}
    midpoints: list[np.ndarray] = []
    edge_lengths: list[float] = []
    adj: dict[int, set[int]] = defaultdict(set)

    def _get_mid_id(u: int, v: int) -> Optional[int]:
        b_idx, y_idx = (u, v) if is_blue[u] else (v, u)
        key = (b_idx, y_idx)
        if key in edge_id:
            return edge_id[key]
        dist = float(np.hypot(*(pts[b_idx] - pts[y_idx])))
        if dist > max_edge_m:
            return None
        idx = len(midpoints)
        edge_id[key] = idx
        midpoints.append(0.5 * (pts[b_idx] + pts[y_idx]))
        edge_lengths.append(dist)
        return idx

    for simplex in tri.simplices:
        b_verts = [int(v) for v in simplex if is_blue[v]]
        y_verts = [int(v) for v in simplex if not is_blue[v]]
        if not b_verts or not y_verts:
            continue
        # Mixed triangle: 2 blue + 1 yellow OR 1 blue + 2 yellow -> exactly 2 cross-edges
        cross = []
        for bv in b_verts:
            for yv in y_verts:
                mid = _get_mid_id(bv, yv)
                if mid is not None:
                    cross.append(mid)
        if len(cross) == 2:
            a, b = cross
            adj[a].add(b)
            adj[b].add(a)

    if not midpoints:
        raise ValueError(f'No valid cross-edges found for track {track.name}')

    mids = np.asarray(midpoints, dtype=np.float64)

    # Prune dead-end spurs (degree-1 vertices) iteratively
    changed = True
    while changed:
        changed = False
        for node in list(adj.keys()):
            if len(adj[node]) == 1:
                nbr = next(iter(adj[node]))
                adj[nbr].discard(node)
                del adj[node]
                changed = True
            elif len(adj[node]) == 0:
                del adj[node]

    valid_nodes = [n for n, nbrs in adj.items() if len(nbrs) >= 2]
    if len(valid_nodes) < 6:
        raise ValueError(f'Delaunay graph collapsed for track {track.name}')

    # Pick start node closest to car_start
    sx, sy, yaw = track.car_start
    car_pos = np.array([sx, sy], dtype=np.float64)
    car_dir = np.array([math.cos(yaw), math.sin(yaw)], dtype=np.float64)
    dists_to_start = [float(np.hypot(*(mids[n] - car_pos))) for n in valid_nodes]
    start_node = valid_nodes[int(np.argmin(dists_to_start))]

    # Pick initial neighbor aligned with car_start yaw (or blue-on-left rule)
    best_first = None
    best_align = -2.0
    for nbr in adj[start_node]:
        vec = mids[nbr] - mids[start_node]
        norm = float(np.hypot(vec[0], vec[1]))
        if norm < 1e-6:
            continue
        align = float((vec / norm) @ car_dir)
        if align > best_align:
            best_align = align
            best_first = nbr

    if best_first is None:
        best_first = next(iter(adj[start_node]))

    # Walk the cycle preferring smooth heading continuation at any junction
    order = [start_node, best_first]
    visited = {start_node, best_first}
    while True:
        curr = order[-1]
        prev = order[-2]
        heading = mids[curr] - mids[prev]
        h_norm = float(np.hypot(heading[0], heading[1]))
        heading = heading / h_norm if h_norm > 1e-9 else car_dir

        candidates = [n for n in adj[curr] if n != prev]
        if not candidates:
            break

        # Close loop once we return to start_node after sufficient steps
        if start_node in candidates and len(order) >= max(10, len(valid_nodes) // 2):
            break

        unvisited = [n for n in candidates if n not in visited]
        if not unvisited:
            break

        def _score(n: int) -> float:
            step = mids[n] - mids[curr]
            s_norm = float(np.hypot(step[0], step[1]))
            if s_norm < 1e-9:
                return -2.0
            return float((step / s_norm) @ heading)

        nxt = max(unvisited, key=_score)
        order.append(nxt)
        visited.add(nxt)

    ordered_xy = mids[order]

    # Verify winding: blue cones should sit on the LEFT (+normal) of the path
    t0 = ordered_xy[1] - ordered_xy[0]
    t0 = t0 / max(float(np.hypot(t0[0], t0[1])), 1e-9)
    left_n = np.array([-t0[1], t0[0]])
    b_near = blue[int(np.argmin(np.hypot(blue[:, 0] - ordered_xy[0, 0],
                                         blue[:, 1] - ordered_xy[0, 1])))]
    if float((b_near - ordered_xy[0]) @ left_n) < 0.0:
        ordered_xy = ordered_xy[::-1]
        # Rotate back so index 0 is closest to car_start
        idx0 = int(np.argmin(np.hypot(ordered_xy[:, 0] - sx, ordered_xy[:, 1] - sy)))
        ordered_xy = np.roll(ordered_xy, -idx0, axis=0)

    return ordered_xy


def build_ground_truth_centerline(track: TrackData, ds: float = 0.5) -> CenterlineData:
    """Build a smooth, resampled ground truth CenterlineData for any EUFS TrackData."""
    if track.name == 'acceleration':
        return _build_acceleration_centerline(track, ds=ds)
    if track.name == 'skidpad':
        return _build_skidpad_centerline(track, ds=ds)

    # Adaptive max cross-edge length: try tight bound first, relax if graph doesn't close
    raw_xy = None
    for max_edge in (7.5, 9.0, 11.0, 14.0):
        try:
            cand = _trace_closed_delaunay(track, max_edge_m=max_edge)
            # Check closure gap between end and start
            gap = float(np.hypot(*(cand[-1] - cand[0])))
            if len(cand) >= 12 and gap < max_edge * 1.2:
                raw_xy = cand
                break
        except ValueError:
            continue

    if raw_xy is None:
        raise RuntimeError(f'Failed to extract closed centerline for track {track.name}')

    # Deduplicate points closer than 0.25 m before smoothing & spline fitting
    dedup = [raw_xy[0]]
    for p in raw_xy[1:]:
        if float(np.hypot(*(p - dedup[-1]))) >= 0.25:
            dedup.append(p)
    if len(dedup) > 3 and float(np.hypot(*(dedup[-1] - dedup[0]))) < 0.25:
        dedup.pop()
    dedup_arr = np.asarray(dedup, dtype=np.float64)

    smoothed = _smooth_points(dedup_arr, closed=True, passes=2)
    s, xy = _resample_spline(smoothed, closed=True, ds=ds)

    cl_tmp = CenterlineData(
        s=s,
        xy=xy,
        left_width=np.zeros(len(s)),
        right_width=np.zeros(len(s)),
        curvature=np.zeros(len(s)),
        closed=True,
    )
    lw, rw = _estimate_widths(xy, cl_tmp.tangents, track.blue, track.yellow)
    kappa = _compute_curvature(xy, closed=True)
    return CenterlineData(s=s, xy=xy, left_width=lw, right_width=rw, curvature=kappa, closed=True)


def build_centerline_from_xy(
    raw_xy: np.ndarray,
    blue: np.ndarray,
    yellow: np.ndarray,
    closed: bool = True,
    ds: float = 0.5,
) -> CenterlineData:
    """Build CenterlineData directly from known ordered centerline XY (e.g. TrackGenerator)."""
    s, xy = _resample_spline(np.asarray(raw_xy, dtype=np.float64), closed=closed, ds=ds)
    cl_tmp = CenterlineData(
        s=s,
        xy=xy,
        left_width=np.zeros(len(s)),
        right_width=np.zeros(len(s)),
        curvature=np.zeros(len(s)),
        closed=closed,
    )
    lw, rw = _estimate_widths(xy, cl_tmp.tangents, blue, yellow)
    kappa = _compute_curvature(xy, closed=closed)
    return CenterlineData(
        s=s, xy=xy, left_width=lw, right_width=rw, curvature=kappa, closed=closed
    )


def _render_verification_sheet(
    results: list[tuple[TrackData, CenterlineData]],
    plot_out: Path,
) -> None:
    """Render a multi-panel PNG showing GT cones and computed GT centerlines."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    n = len(results)
    cols = 4
    rows = int(math.ceil(n / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(4.5 * cols, 4.0 * rows), dpi=140)
    axes_flat = np.atleast_1d(axes).ravel()

    for ax, (track, cl) in zip(axes_flat, results):
        if len(track.blue):
            ax.scatter(track.blue[:, 0], track.blue[:, 1], s=8, c='#1f77b4', label='blue')
        if len(track.yellow):
            ax.scatter(track.yellow[:, 0], track.yellow[:, 1], s=8, c='#e6b800', label='yellow')
        if len(track.big_orange):
            ax.scatter(track.big_orange[:, 0], track.big_orange[:, 1], s=14, c='#ff6600',
                       marker='^')
        if len(track.orange):
            ax.scatter(track.orange[:, 0], track.orange[:, 1], s=8, c='#ff9933', marker='^')
        plot_xy = np.vstack([cl.xy, cl.xy[0:1]]) if cl.closed else cl.xy
        ax.plot(plot_xy[:, 0], plot_xy[:, 1], c='#d62728', lw=1.4)
        sx, sy, yaw = track.car_start
        ax.arrow(sx, sy, 3.0 * math.cos(yaw), 3.0 * math.sin(yaw),
                 head_width=1.0, head_length=1.2, fc='#2ca02c', ec='#2ca02c', zorder=5)
        ax.set_aspect('equal', adjustable='datalim')
        ax.set_title(
            f'{track.name}\nL={cl.total_length_m:.1f}m, N={len(cl.xy)}, '
            f'w={np.median(cl.left_width + cl.right_width):.1f}m',
            fontsize=8,
        )
        ax.tick_params(labelsize=6)

    for ax in axes_flat[n:]:
        ax.axis('off')

    fig.tight_layout()
    plot_out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(plot_out)
    plt.close(fig)


def main(argv: Optional[list[str]] = None) -> int:
    """CLI entry point to generate GT centerline fixtures and verification plots."""
    parser = argparse.ArgumentParser(
        description='Generate GT centerline fixtures for EUFS tracks.'
    )
    parser.add_argument('--tracks-dir', type=str, default='', help='EUFS eufs_tracks/csv path.')
    parser.add_argument('--out-dir', type=str, required=True, help='Output fixture directory.')
    parser.add_argument('--ds', type=float, default=0.5, help='Resampling spacing (m).')
    parser.add_argument('--plot-out', type=str, default='', help='Optional verification PNG path.')
    args = parser.parse_args(argv)

    tracks_dir = Path(args.tracks_dir) if args.tracks_dir else find_eufs_tracks_dir()
    if tracks_dir is None or not tracks_dir.is_dir():
        raise SystemExit(
            'Could not find EUFS tracks directory; pass --tracks-dir or EUFS_MASTER.'
        )

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    results: list[tuple[TrackData, CenterlineData]] = []

    for name in STATIC_TRACKS:
        csv_path = tracks_dir / f'{name}.csv'
        if not csv_path.is_file():
            print(f'[SKIP] missing {csv_path}')
            continue
        track = load_track_csv(csv_path)
        cl = build_ground_truth_centerline(track, ds=args.ds)
        save_centerline_csv(out_dir / f'{name}.csv', cl)
        results.append((track, cl))
        print(
            f'[OK] {name:32s} len={cl.total_length_m:6.1f}m  pts={len(cl.xy):4d}  '
            f'med_w={np.median(cl.left_width + cl.right_width):.2f}m  '
            f'max_k={np.max(np.abs(cl.curvature)):.3f}/m'
        )

    if args.plot_out and results:
        _render_verification_sheet(results, Path(args.plot_out))
        print(f'[PLOT] saved verification sheet to {args.plot_out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
