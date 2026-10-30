"""Lap-return map reset, loop trails and the lap-1 lock-in (pure numpy, no ROS)."""
import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from mfe_path_planning.core.paths import clearance_guard, closed_resample, extend_trail, midpoint_walk, smooth_cyclic
from mfe_path_planning.core.racing_line import _min_curvature_path as min_curvature_path


def lap_return_banned(cone_keys, centre_xy, radius_m) -> set:
    """
    Map cell keys within radius_m of centre_xy (radius_m None: every key).

    Pure.
    """
    if radius_m is None:
        return set(cone_keys)
    cx, cy = float(centre_xy[0]), float(centre_xy[1])
    return {k for k in cone_keys if math.hypot(k[0] - cx, k[1] - cy) <= radius_m}


# A cell banned at the lap-return trigger is accepted again once a LiDAR detection of a scan taken after the
# trigger lies within this distance of it (a re-observation), so stale copies stay banned and re-seen cones return.
LAP_REOBS_TOL_M = 0.75


def lap_reobserved(banned, det_xy, tol_m=LAP_REOBS_TOL_M) -> set:
    """
    Banned cell keys with a detection (map frame, one scan) within tol_m.

    Pure.
    """
    if not banned or len(det_xy) == 0:
        return set()
    keys = list(banned)
    k = np.asarray(keys, dtype=float).reshape(-1, 2)
    d = np.asarray(det_xy, dtype=float).reshape(-1, 2)
    d2 = np.sum((k[:, None, :] - d[None, :, :]) ** 2, axis=2)
    hit = np.min(d2, axis=1) <= tol_m * tol_m
    return {key for key, h in zip(keys, hit) if h}


# A banned cell more than this far behind the car (along its heading) counts as passed. The LiDAR reports only points
# ahead of the car (x >= ~1.6 m in every scan), so a cell behind it can never be re-seen, and the tracer's
# start gate sits behind the car on the driven path: passing a cell is the only later evidence for it.
LAP_PASSED_M = 1.0


def lap_passed(banned, car_xy, car_dir, behind_m=LAP_PASSED_M) -> set:
    """
    Banned cell keys the car has driven past (more than behind_m behind it along car_dir).

    Pure.
    """
    if not banned:
        return set()
    c, s = float(car_dir[0]), float(car_dir[1])
    cx, cy = float(car_xy[0]), float(car_xy[1])
    return {k for k in banned if (k[0] - cx) * c + (k[1] - cy) * s < -behind_m}


class LapReturnReset:
    """
    Lap-return trigger (pure).

    Fires once per lap when the car comes back into the start area after travelling min(min_travel_m, len_frac *
    previous lap length) since it last left the area.

    The start area is start_r_m around the first odometry position. The car must first leave beyond
    2 * start_r_m (re-arming the trigger) before a return counts. The lap length is the travel from the last
    departure to the previous return (None until a lap has completed, so min_travel_m applies on lap 1).
    """

    def __init__(self, start_r_m, min_travel_m, len_frac=0.6):
        self.start_r = float(start_r_m)
        self.min_travel = float(min_travel_m)
        self.len_frac = float(len_frac)
        self.start = None
        self.prev = None
        self.lap_travel = 0.0
        self.lap_len = None
        self.away = False        # left the start area since the last return
        self.triggered = False   # fired on this lap already
        self.fires = 0

    def min_travel_eff(self) -> float:
        if self.lap_len is None:
            return self.min_travel
        return min(self.min_travel, self.len_frac * self.lap_len)

    def update(self, pos) -> bool:
        """
        Feed one odometry position.

        True on the step the trigger fires.
        """
        pos = np.asarray(pos, dtype=float)
        if self.start is None:
            self.start = pos.copy()
            self.prev = pos.copy()
            return False
        self.lap_travel += float(np.hypot(*(pos - self.prev)))
        self.prev = pos.copy()
        d = float(np.hypot(*(pos - self.start)))
        if d > 2.0 * self.start_r and not self.away:
            self.away = True
            self.triggered = False
            self.lap_travel = 0.0
        if d < self.start_r and self.away:
            fired = False
            if not self.triggered and self.lap_travel >= self.min_travel_eff():
                self.triggered = True
                self.fires += 1
                fired = True
            if self.lap_travel > 2.0 * self.start_r:   # ignore wiggles at the start line
                self.lap_len = self.lap_travel
            self.away = False
            return fired
        return False


@dataclass
class LoopTrail:
    """
    Driven trail of one loop (the lap-1 lock-in, or one skidpad circle): breadcrumbs, distance driven.

    the largest excursion from the loop start, the laps completed on the locked path, and that path once locked.

    `start` is the first position fed (the loop start, kept after a re-lock on the next lap).
    """

    start: Optional[np.ndarray] = None
    trail: list = field(default_factory=list)
    dist: float = 0.0
    max_r: float = 0.0
    laps: int = 0
    locked: Optional[np.ndarray] = None

    def advance(self, pos) -> None:
        """Add one odometry position to the trail (breadcrumbs every TRAIL_STEP) and update dist and max_r."""
        if self.start is None:
            self.start = pos.copy()
        step = extend_trail(self.trail, pos)
        if step is not None:
            self.dist += step
            self.max_r = max(self.max_r, float(np.linalg.norm(pos - self.start)))

    def closed(self, pos, min_dist: float, min_r: float, close_r: float) -> bool:
        """
        Return True when the loop is closed.

        Closed means: driven more than min_dist, excursion more than min_r, back within close_r of the start.
        """
        return (self.dist > min_dist and self.max_r > min_r
                and float(np.linalg.norm(pos - self.start)) < close_r)

    def restart(self, pos) -> None:
        """Start the next lap: the trail restarts at pos, the distance and excursion are cleared (start is kept)."""
        self.trail = [pos.copy()]
        self.dist = 0.0
        self.max_r = 0.0


def finish_loop(centerline, blue, yellow, min_n: int, label: str, events: list) -> np.ndarray:
    """
    Smooth a midpoint loop, optimise it inside the walls, revert waypoints near cones, resample.

    Log messages (level, text) are appended to events.
    """
    centerline = smooth_cyclic(centerline)
    raw = centerline.copy()
    try:
        centerline = min_curvature_path(centerline, blue, yellow, safety=0.5)
    except Exception as e:
        events.append(('warn', f'{label} optimizer failed ({type(e).__name__}: {e}) — using raw centerline.'))
    reverted = clearance_guard(centerline, raw, np.vstack([blue, yellow]))
    if reverted:
        events.append(('info', f'{label} clearance guard reverted {reverted} waypoints.'))
    return closed_resample(centerline, min_n)


def build_locked_loop(trail, blue, yellow, events: list):
    """
    Full-lap closed centreline from the cone walls, ordered by the lap-1 trail (handles figure-8s).

    None when the map is too thin or the walk too short.
    """
    if len(blue) < 4 or len(yellow) < 4 or len(trail) < 20:
        events.append(('warn', f'Loop closed but map too thin to lock '
                               f'(blue={len(blue)} yellow={len(yellow)} trail={len(trail)})'))
        return None
    centerline = midpoint_walk(np.array(trail), blue, yellow)
    if len(centerline) < 10:
        return None
    return finish_loop(centerline, blue, yellow, min_n=20, label='Lock-in', events=events)


class LapBan:
    """
    Map cells excluded from the boundary tracer after a lap-return trigger.

    A banned cell is accepted again when the LiDAR re-sees it (lap_reobserved) or the car drives past it
    (lap_passed).
    """

    def __init__(self):
        self.banned: set = set()
        self.reobs = 0                  # banned cells accepted again since the start

    def fire(self, keys, start_xy, radius_m) -> int:
        """
        Ban the keys within radius_m of start_xy.

        Returns the number newly banned.
        """
        new = lap_return_banned(keys, start_xy, radius_m)
        self.banned |= new
        return len(new)

    def lift(self, car_xy, car_dir, det=None) -> int:
        """
        Accept banned cells passed by the car or, when det (map-frame LiDAR points) is given, re-observed.

        Returns the number accepted.
        """
        hit = lap_passed(self.banned, car_xy, car_dir)
        if det is not None:
            hit |= lap_reobserved(self.banned, det)
        if hit:
            self.banned -= hit
            self.reobs += len(hit)
        return len(hit)

    def visible(self, keys) -> list:
        """Return the keys that are not banned."""
        return [k for k in keys if k not in self.banned]
