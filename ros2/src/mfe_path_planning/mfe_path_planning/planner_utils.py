"""Geometry, trail and cone-array helpers shared by path_planner_node."""
import math

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

TRAIL_STEP = 0.5   # m between driven breadcrumbs


def yaw_from_quaternion(q) -> float:
    """Yaw (rad) of a geometry_msgs/Quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def path_from_xy(xy: np.ndarray, frame_id: str, stamp) -> Path:
    """Nx2 array of (x, y) -> nav_msgs/Path with identity orientation."""
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = stamp
    for point in xy:
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = stamp
        ps.pose.position.x = float(point[0])
        ps.pose.position.y = float(point[1])
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
    return path


def inside_polygon(P: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Even-odd point-in-polygon for each row of P (Nx2) against a closed polygon (Mx2)."""
    x, y = P[:, 0][:, None], P[:, 1][:, None]
    x1, y1 = poly[:, 0][None, :], poly[:, 1][None, :]
    x2, y2 = np.roll(poly[:, 0], -1)[None, :], np.roll(poly[:, 1], -1)[None, :]
    with np.errstate(divide='ignore', invalid='ignore'):
        xc = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
    cross = ((y1 > y) != (y2 > y)) & (x < xc)
    return (np.count_nonzero(cross, axis=1) % 2) == 1


def arc_length(xy: np.ndarray) -> float:
    return float(np.sum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))))


def cone_arrays(buckets) -> list:
    """Per-type lists of [x, y] -> per-type float64 Nx2 arrays."""
    return [np.array(b, dtype=np.float64).reshape(-1, 2) if b
            else np.zeros((0, 2), dtype=np.float64)
            for b in buckets]


def extend_trail(trail: list, pos: np.ndarray):
    """Append pos once it is TRAIL_STEP past the last breadcrumb (or if the trail is empty).

    Returns the distance advanced, or None when nothing was appended.
    """
    if not trail:
        trail.append(pos.copy())
        return None
    step = float(np.linalg.norm(pos - trail[-1]))
    if step < TRAIL_STEP:
        return None
    trail.append(pos.copy())
    return step


def midpoint_walk(trail: np.ndarray, blue: np.ndarray, yellow: np.ndarray) -> np.ndarray:
    """Sample the trail every ~1 m and take the midpoint of the nearest blue/yellow cones.

    Keeps the drive-line point where either wall is unseen within 5 m. Points closer than
    0.75 m to the previous one are dropped.
    """
    centerline = []
    for tx, ty in trail[::2]:
        db = np.hypot(blue[:, 0] - tx, blue[:, 1] - ty)
        dy = np.hypot(yellow[:, 0] - tx, yellow[:, 1] - ty)
        ib, iy = int(np.argmin(db)), int(np.argmin(dy))
        if db[ib] <= 5.0 and dy[iy] <= 5.0:
            p = ((blue[ib, 0] + yellow[iy, 0]) / 2.0,
                 (blue[ib, 1] + yellow[iy, 1]) / 2.0)
        else:
            p = (float(tx), float(ty))
        if not centerline or (p[0] - centerline[-1][0]) ** 2 + (p[1] - centerline[-1][1]) ** 2 >= 0.75 ** 2:
            centerline.append(p)
    return np.array(centerline)


def smooth_cyclic(centerline: np.ndarray) -> np.ndarray:
    """3-point moving average over a closed loop."""
    prev_c = np.roll(centerline, 1, axis=0)
    next_c = np.roll(centerline, -1, axis=0)
    return (prev_c + centerline + next_c) / 3.0


def clearance_guard(centerline: np.ndarray, raw: np.ndarray, cones: np.ndarray,
                    min_dist: float = 1.0) -> int:
    """Revert (in place) waypoints within min_dist of a cone to their raw position. Returns the count."""
    d = np.hypot(cones[None, :, 0] - centerline[:, None, 0],
                 cones[None, :, 1] - centerline[:, None, 1]).min(axis=1)
    bad = d < min_dist
    centerline[bad] = raw[bad]
    return int(np.count_nonzero(bad))


def closed_resample(centerline: np.ndarray, min_n: int) -> np.ndarray:
    """Resample a loop to ~1 m spacing, closing it (first point == last point)."""
    closed = np.vstack([centerline, centerline[:1]])
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(s[-1])
    n = max(int(round(total)), min_n)
    si = np.linspace(0.0, total, n + 1)
    return np.column_stack([
        np.interp(si, s, closed[:, 0]),
        np.interp(si, s, closed[:, 1]),
    ])


def clamp_to_corridor(optimized, path, left, right):
    """Points of an optimised line outside the traced corridor revert to the centreline."""
    corridor = np.vstack([left, right[::-1]])
    bad = ~inside_polygon(optimized, corridor)
    bad[0] = False
    if bad.any() and len(optimized) == len(path):
        bad = np.convolve(bad.astype(float), np.ones(5))[2:2 + len(bad)] > 0   # dilate by 2 points
        optimized = np.where(bad[:, None], path, optimized)
    return optimized


def make_skidpad_path() -> np.ndarray:
    """Hardcoded EUFS skidpad trajectory: 2 laps LEFT (counterclockwise), 2 laps RIGHT, then exit."""
    # Centres +-9.125, R 9.125: the circles touch at the crossing (was +-9.3 / 9.1, which clipped the inner cones).
    cx, cy_l, cy_r = 14.4, 9.125, -9.125
    R = 9.125
    N = 50    # waypoints per full lap

    n_entry = 40
    entry = np.column_stack([np.linspace(0, cx, n_entry), np.zeros(n_entry)])

    theta_l = np.linspace(-math.pi / 2, -math.pi / 2 + 4 * math.pi, N * 2 + 1)
    left_circ = np.column_stack([cx + R * np.cos(theta_l), cy_l + R * np.sin(theta_l)])

    theta_r = np.linspace(math.pi / 2, math.pi / 2 - 4 * math.pi, N * 2 + 1)
    right_circ = np.column_stack([cx + R * np.cos(theta_r), cy_r + R * np.sin(theta_r)])

    n_exit = 40
    exit_pts = np.column_stack([np.linspace(cx, 40.0, n_exit), np.zeros(n_exit)])

    return np.vstack([entry[:-1], left_circ, right_circ[1:], exit_pts[1:]])
