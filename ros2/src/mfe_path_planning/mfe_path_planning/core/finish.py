"""
Finish-line detection helpers (pure numpy, no ROS): the orange timing line, the lap crossing test.

LiDAR forward-zone counts and the orange-gate clustering and pass counting. The finish detector node feeds
them.
"""
import math

import numpy as np

LAP_EXCURSION_M = 15.0          # must leave this far from start before a lap crossing counts
SPAWN_DEBOUNCE_M = 5.0          # gate passes before this much travel are ignored
GATE_CLUSTER_M = 8.0            # single-linkage distance between pylons of one gate
GATE_TRACK_M = 5.0              # a cluster keeps its pass state if its centroid moves less than this
GATE_HYSTERESIS_M = 2.0         # car must leave gate_enter_r + this before the next pass counts
GATE_NEW_PASS_TRAVEL_M = 20.0   # a gate first seen with the car inside it counts a pass after this much driving
STANDSTILL_MPS = 0.1            # mission_completed is published once the speed drops below this
STOP_AFTER_LINE_MAX_M = 30.0    # DD.4.4.4: the run-out past the timing line may not exceed this
REANCHOR_LOG_M = 0.05           # re-anchor logged only when a line term moves by more than this (m)


def extract_xyz(msg) -> np.ndarray:
    """
    Nx3 float32 array of finite (x, y, z) points from a PointCloud2-like message (fields, width.

    height, point_step, data).
    """
    """Return Nx3 float32 array of finite (x, y, z) points from a PointCloud2."""
    field_map = {f.name: f for f in msg.fields}
    n = msg.width * msg.height
    if not all(k in field_map for k in ('x', 'y', 'z')) or n == 0:
        return np.zeros((0, 3), dtype=np.float32)
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)

    def get_col(name: str) -> np.ndarray:
        idx = np.arange(n) * msg.point_step + field_map[name].offset
        return np.stack([raw[idx + k] for k in range(4)], axis=1).view(np.float32).reshape(-1)

    pts = np.stack([get_col('x'), get_col('y'), get_col('z')], axis=1)
    return pts[np.isfinite(pts).all(axis=1)].astype(np.float32)


def orange_timing_line(points, sx, sy, syaw, search_r, margin_m, pylon_m=1.5, pair_long_tol=2.0, center=None):
    """
    Return the timing line through the orange cones at the start, in the start frame: (long, lat, width) or None.

    points: iterable of (x, y) orange cone positions in the map frame. Detections closer than pylon_m are
    merged into pylons. A left pylon (lat > 0) and a right pylon (lat < 0) whose long coordinates agree within
    pair_long_tol form a gate pair; the pair whose midpoint is nearest the start wins. The line runs
    perpendicular to the start heading through the pair's midpoint, and its width is the pylon separation
    plus 2 * margin_m.

    center: optional map-frame (x, y). Pylons are searched within search_r of it instead of the start pose, and
    the pair whose midpoint is nearest its long coordinate wins (the lap-return re-anchor uses the car position).
    The line itself stays in the start frame.
    """
    c, s = math.cos(syaw), math.sin(syaw)
    cx, cy = (sx, sy) if center is None else center
    car_long = c * (cx - sx) + s * (cy - sy)
    rel = []
    for x, y in points:
        dx, dy = x - cx, y - cy
        if math.hypot(dx, dy) <= search_r:
            rel.append((c * (x - sx) + s * (y - sy), -s * (x - sx) + c * (y - sy)))
    clusters = [[p] for p in rel]
    merged = True
    while merged:
        merged = False
        for i in range(len(clusters)):
            for j in range(i + 1, len(clusters)):
                if any(math.hypot(a[0] - b[0], a[1] - b[1]) < pylon_m for a in clusters[i] for b in clusters[j]):
                    clusters[i] += clusters.pop(j)
                    merged = True
                    break
            if merged:
                break
    pylons = [(float(np.mean([p[0] for p in cl])), float(np.mean([p[1] for p in cl]))) for cl in clusters]
    left = [p for p in pylons if p[1] > 0.0]
    right = [p for p in pylons if p[1] < 0.0]
    best = None
    for pl in left:
        for pr in right:
            if abs(pl[0] - pr[0]) > pair_long_tol:
                continue
            mid_long = 0.5 * (pl[0] + pr[0])
            key = abs(mid_long - car_long)
            if best is None or key < best[0]:
                best = (key, mid_long, 0.5 * (pl[1] + pr[1]), abs(pl[1] - pr[1]) + 2.0 * margin_m)
    if best is None:
        return None
    return best[1], best[2], best[3]


def line_crossed(prev_long, long, lat, line, heading_ok) -> bool:
    """
    Return True when the car moves from behind to ahead of the timing line (long, lat, width) this step.

    within the line's width, while heading with the start direction (heading_ok).
    """
    if prev_long is None or line is None or not heading_ok:
        return False
    line_long, line_lat, width = line
    return prev_long - line_long < 0.0 <= long - line_long and abs(lat - line_lat) <= width / 2.0


def stop_travel_after_line(travel_at_crossing: float, stop_after_m: float) -> float:
    """Travel at which the brake latches after the final timing-line crossing (stop_after_m clamped to 0..30 m)."""
    return travel_at_crossing + min(max(stop_after_m, 0.0), STOP_AFTER_LINE_MAX_M)


def rebuild_gate_states(points, old_states) -> list:
    """
    Cluster the orange pylons into gates (single linkage at GATE_CLUSTER_M), carrying each gate's pass.

    state over from the previous clustering. points: map-frame (x, y) arrays; old_states: [{'c': centroid,
    'inside': bool|None}]. inside=None: initialised on the next odometry tick without counting a pass, so a gate
    the car already sits on (spawn, or a camera gate that first forms under it) is not counted.
    """
    clusters: list = []
    for p in points:
        for cl in clusters:
            if any(math.hypot(p[0] - q[0], p[1] - q[1]) < GATE_CLUSTER_M for q in cl):
                cl.append(p)
                break
        else:
            clusters.append([p])
    states = []
    for cl in clusters:
        c = np.mean(cl, axis=0)
        prev = next((s for s in old_states if math.hypot(s['c'][0] - c[0], s['c'][1] - c[1]) < GATE_TRACK_M), None)
        states.append({'c': c, 'inside': prev['inside'] if prev is not None else None})
    return states


def gate_pass_step(gate: dict, d: float, travel_m: float, enter_r: float) -> bool:
    """
    One odometry tick for one gate, d = distance from the car to the gate centroid.

    Updates gate['inside'] and returns True when a pass is counted (the car entered the gate after
    SPAWN_DEBOUNCE_M of travel).
    """
    new_under_car = False
    if gate['inside'] is None:
        gate['inside'] = d <= enter_r
        new_under_car = gate['inside'] and travel_m > GATE_NEW_PASS_TRAVEL_M
    if new_under_car or (not gate['inside'] and d <= enter_r):
        gate['inside'] = True
        return travel_m > SPAWN_DEBOUNCE_M
    if gate['inside'] and d > enter_r + GATE_HYSTERESIS_M:
        gate['inside'] = False
    return False


def lidar_forward_count(pts: np.ndarray, fwd_min: float, fwd_max: float, lat_max: float) -> int:
    """Count the LiDAR points in the forward finish zone (fwd_min < x < fwd_max, |y| < lat_max, z > -0.15)."""
    mask = (
        (pts[:, 0] > fwd_min) &
        (pts[:, 0] < fwd_max) &
        (np.abs(pts[:, 1]) < lat_max) &
        (pts[:, 2] > -0.15)
    )
    return int(np.sum(mask))
