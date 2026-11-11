"""
Fused cone map on a 0.5 m grid: colour merge, colour votes, expiry, eviction and visibility buckets.

Pure numpy. The node passes its library's cone-type codes in (`types`) and the colour-to-type table
(`colour_to_type`, the mfe_msgs/Cone colours that are skipped in `skip_colours`).
"""
import math
from collections import namedtuple

import numpy as np

GRID = 0.5                 # cone map key resolution (m)
VOTE_CAP_TOTAL = 50        # colour_vote: halve a cell's vote counts once their total exceeds this (lets it adapt)
CONE_VIS_R = 50.0          # local planning visibility radius (m)
CONE_BEHIND_M = 3.0        # tolerance behind the car for the visibility filter

# Cone-type codes of fsd_path_planning.ConeTypes (UNKNOWN 0, RIGHT 1, LEFT 2; n = number of types).
FSD_TYPES = namedtuple('FSD_TYPES', 'unknown right left n')(unknown=0, right=1, left=2, n=5)


def vote_update(votes, obs, current, left, right, unknown, vote_min, vote_margin):
    """
    Colour vote for one grid cell (pure).

    votes = [left, right, unknown] counts, updated in place. Returns the cell colour: the majority of coloured
    votes once there are at least vote_min coloured votes and the winner leads by vote_margin; otherwise `current`
    (UNKNOWN for a new cell, or the colour already held).
    """
    if obs == left:
        votes[0] += 1
    elif obs == right:
        votes[1] += 1
    else:
        votes[2] += 1
    if sum(votes) > VOTE_CAP_TOTAL:
        votes[:] = [v // 2 for v in votes]
    nb, ny = votes[0], votes[1]
    if nb + ny >= vote_min and abs(nb - ny) >= vote_margin:
        return left if nb > ny else right
    return current


def in_sense_range(dx, dy, fwd_x, fwd_y, radius, behind_m):
    """
    Return whether a point is in sensing range.

    The same test as ConeMap.bucket: within radius and not behind the car by more than behind_m.

    dx, dy: offset from the car; fwd_x, fwd_y: car heading (unit vector).
    """
    return dx * dx + dy * dy <= radius * radius and dx * fwd_x + dy * fwd_y >= -behind_m


def expiry_due(count, last_seen, cycle, in_range, min_obs, expiry_cycles):
    """
    cone_expiry rule (pure): drop only cells observed fewer than min_obs times, unseen for.

    expiry_cycles cycles, and currently inside the sensing range (out-of-range absence is not evidence).
    """
    return count < min_obs and cycle - last_seen >= expiry_cycles and in_range


def bucket_counts(buckets, types):
    """(left, right, unknown) counts of a bucket list indexed by type code."""
    return len(buckets[types.left]), len(buckets[types.right]), len(buckets[types.unknown])


class ConeMap:
    """
    Map cells keyed by (x, y) on the GRID, each holding a type code.

    Colour only upgrades UNKNOWN cells unless colour_vote (per-cell votes) or recolour (a later colour replaces
    the stored one) is set.
    """

    def __init__(self, types, colour_to_type, skip_colours=(), recolour=False, colour_vote=False,
                 vote_min=3, vote_margin=2, cone_expiry=False, expiry_min_obs=3, expiry_cycles=30, grid=GRID):
        self.types = types
        self._colour_to_type = colour_to_type
        self._skip = tuple(skip_colours)
        self._recolour = recolour
        self._colour_vote = colour_vote
        self._vote_min = vote_min
        self._vote_margin = vote_margin
        self._cone_expiry = cone_expiry
        self._expiry_min_obs = expiry_min_obs
        self._expiry_cycles = expiry_cycles
        self._grid = float(grid)
        self._cells: dict = {}          # (x, y) -> type code
        self._votes: dict = {}          # colour_vote: (x, y) -> [left, right, unknown] observation counts
        self._obs: dict = {}            # cone_expiry: (x, y) -> [cycles observed, last cycle seen]
        self.merge_cycle = 0
        self.colour_flips = 0           # cells whose decided colour reversed (BLUE <-> YELLOW)
        self.cones_expired = 0

    # ------------------------------------------------------------------ dict-like access

    def __len__(self):
        return len(self._cells)

    def __contains__(self, key):
        return key in self._cells

    def get(self, key, default=None):
        return self._cells.get(key, default)

    def keys(self) -> list:
        return list(self._cells.keys())

    # ------------------------------------------------------------------ merge

    def merge(self, points, car_pos, car_dir, insert_cap=None):
        """
        Merge (x, y, colour) points into the map.

        With insert_cap, the oldest entry is evicted before a new key would exceed the cap. colour_vote replaces the
        upgrade/latch rules with per-cell votes; cone_expiry then drops stale sparse cells that are inside the sensing
        range.
        """
        self.merge_cycle += 1
        for x, y, color in points:
            if color in self._skip:
                continue
            # one non-finite cone (bad depth pixel upstream) must not kill the planner: round(NaN) raises
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            rx = round(x / self._grid) * self._grid
            ry = round(y / self._grid) * self._grid
            key = (rx, ry)
            new_type = self._colour_to_type.get(color, self.types.unknown)
            if self._colour_vote:
                if key not in self._cells:
                    self._make_room(insert_cap)
                    self._cells[key] = self.types.unknown     # no colour until the votes decide one
                    self._votes[key] = [0, 0, 0]
                self._set_colour(key, vote_update(
                    self._votes[key], new_type, self._cells[key], self.types.left, self.types.right,
                    self.types.unknown, self._vote_min, self._vote_margin))
            elif key not in self._cells:
                self._make_room(insert_cap)
                self._cells[key] = new_type
            elif self._cells[key] == self.types.unknown and new_type != self.types.unknown:
                self._cells[key] = new_type
            elif self._recolour and new_type != self.types.unknown and self._cells[key] != new_type:
                self._set_colour(key, new_type)    # sources that refine colours over time (camera votes) overwrite
            if self._cone_expiry:
                self._note_obs(key)
        if self._cone_expiry:
            self._expire_cells(car_pos, car_dir)

    def _make_room(self, insert_cap):
        """Skidpad FIFO: evict the oldest entry before a new key would exceed insert_cap."""
        if insert_cap is not None and len(self._cells) >= insert_cap:
            oldest = next(iter(self._cells))
            self._cells.pop(oldest)
            self._forget_cell(oldest)

    def _forget_cell(self, key):
        self._votes.pop(key, None)
        self._obs.pop(key, None)

    def _set_colour(self, key, value):
        old = self._cells[key]
        if old != value and old != self.types.unknown and value != self.types.unknown:
            self.colour_flips += 1
        self._cells[key] = value

    def _note_obs(self, key):
        """Count a cell at most once per merge cycle (a duplicated cone in one message is one sighting)."""
        rec = self._obs.get(key)
        if rec is None:
            rec = self._obs[key] = [0, -1]
        if rec[1] != self.merge_cycle:
            rec[0] += 1
            rec[1] = self.merge_cycle

    def _expire_cells(self, car_pos, car_dir):
        """
        Drop cells observed fewer than expiry_min_obs times, unseen for expiry_cycles cycles, while inside.

        the sensing range (the bucket visibility test): a cone behind or beyond the range is not evidence.
        """
        fwd_x, fwd_y = car_dir
        for key, (count, last) in list(self._obs.items()):
            if key not in self._cells:
                self._forget_cell(key)
                continue
            dx = key[0] - car_pos[0]
            dy = key[1] - car_pos[1]
            in_range = in_sense_range(dx, dy, fwd_x, fwd_y, CONE_VIS_R, CONE_BEHIND_M)
            if expiry_due(count, last, self.merge_cycle, in_range, self._expiry_min_obs, self._expiry_cycles):
                del self._cells[key]
                self._forget_cell(key)
                self.cones_expired += 1

    def evict_far(self, cap, car_pos):
        """
        Over the cap, drop the cones farthest from the car.

        (Evicting the oldest key instead churned the map every message once it held more than the cap, dropping cones
        next to the car.)
        """
        if len(self._cells) > cap:
            keys = list(self._cells.keys())
            d2 = np.sum((np.array(keys) - car_pos) ** 2, axis=1)
            for i in np.argsort(d2)[cap:]:
                del self._cells[keys[i]]
                self._forget_cell(keys[i])

    # ------------------------------------------------------------------ views

    def xy(self) -> np.ndarray:
        """All cells as an Nx2 float64 array (shape (0,) when empty, as before the move)."""
        return np.array(list(self._cells.keys()), dtype=np.float64)

    def walls(self):
        """Blue (left) and yellow (right) cell positions."""
        blue = np.array([k for k, v in self._cells.items() if v == self.types.left])
        yellow = np.array([k for k, v in self._cells.items() if v == self.types.right])
        return blue, yellow

    def bucket(self, visible_only, car_pos, car_dir):
        """
        Cells grouped by type code.

        visible_only keeps those within CONE_VIS_R and not behind the car.
        """
        buckets = [[] for _ in range(self.types.n)]
        for (rx, ry), cone_type in self._cells.items():
            if visible_only:
                dx = rx - car_pos[0]
                dy = ry - car_pos[1]
                if dx * dx + dy * dy > CONE_VIS_R * CONE_VIS_R:
                    continue
                if dx * car_dir[0] + dy * car_dir[1] < -CONE_BEHIND_M:
                    continue
            buckets[cone_type].append([rx, ry])
        return buckets

    def assign_unknown_sides(self, car_pos, car_dir, buckets):
        """LiDAR-only: sort UNKNOWN cells into LEFT/RIGHT by the sign of their cross product with the heading."""
        for (rx, ry), cone_type in self._cells.items():
            if cone_type != self.types.unknown:
                continue
            dx = rx - car_pos[0]
            dy = ry - car_pos[1]
            fwd = dx * car_dir[0] + dy * car_dir[1]
            if fwd < -CONE_BEHIND_M:
                continue
            cross_z = car_dir[0] * dy - car_dir[1] * dx
            buckets[self.types.left if cross_z > 0 else self.types.right].append([rx, ry])
        buckets[self.types.unknown] = []

    def wall_colour_vote(self, wall) -> int:
        """
        +1 when the wall's coloured cells are mostly BLUE, -1 when mostly YELLOW, 0 without a confident.

        vote (needs >= 2 coloured cells and a lead of >= 1).
        """
        blue = sum(1 for x, y in wall if self._cells.get((float(x), float(y))) == self.types.left)
        yellow = sum(1 for x, y in wall if self._cells.get((float(x), float(y))) == self.types.right)
        if blue + yellow < 2 or blue == yellow:
            return 0
        return 1 if blue > yellow else -1
