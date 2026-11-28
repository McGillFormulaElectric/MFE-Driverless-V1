"""Track CSV and ground truth centerline I/O (pure Python, no ROS)."""

from __future__ import annotations

import csv
from dataclasses import dataclass
import os
from pathlib import Path
from typing import Optional

import numpy as np

STATIC_TRACKS = (
    'acceleration',
    'skidpad',
    'small_track',
    'rectangle',
    'garden_light',
    'boa_constrictor',
    'comp_2021',
    'hairpins_increasing_difficulty',
    'its_a_mess',
    'peanut',
    'bone',
)

CLOSED_LOOP_TRACKS = (
    'small_track',
    'rectangle',
    'garden_light',
    'boa_constrictor',
    'comp_2021',
    'hairpins_increasing_difficulty',
    'its_a_mess',
    'peanut',
    'bone',
)

TRACK_MISSIONS = {
    'acceleration': 'acceleration',
    'skidpad': 'skidpad',
    'peanut': 'peanut',
    'small_track': 'autocross',
    'rectangle': 'autocross',
    'garden_light': 'autocross',
    'boa_constrictor': 'autocross',
    'comp_2021': 'autocross',
    'hairpins_increasing_difficulty': 'autocross',
    'its_a_mess': 'autocross',
    'bone': 'autocross',
}


@dataclass(frozen=True)
class TrackData:
    """Static cone positions and vehicle spawn pose loaded from an EUFS track CSV."""

    name: str
    blue: np.ndarray
    yellow: np.ndarray
    big_orange: np.ndarray
    orange: np.ndarray
    midpoints: np.ndarray
    car_start: tuple[float, float, float]

    @property
    def all_wall_cones(self) -> np.ndarray:
        """Return stacked blue and yellow wall cone positions (N x 2)."""
        parts = [arr for arr in (self.blue, self.yellow) if len(arr) > 0]
        if not parts:
            return np.zeros((0, 2), dtype=np.float64)
        return np.vstack(parts)

    @property
    def all_cones(self) -> np.ndarray:
        """Return all physical cones (blue, yellow, big_orange, orange) as N x 2."""
        parts = [
            arr for arr in (self.blue, self.yellow, self.big_orange, self.orange)
            if len(arr) > 0
        ]
        if not parts:
            return np.zeros((0, 2), dtype=np.float64)
        return np.vstack(parts)


@dataclass(frozen=True)
class CenterlineData:
    """Ordered ground truth centerline with arc length, half-widths, and curvature."""

    s: np.ndarray
    xy: np.ndarray
    left_width: np.ndarray
    right_width: np.ndarray
    curvature: np.ndarray
    closed: bool = True

    @property
    def total_length_m(self) -> float:
        """Return total centerline arc length in metres."""
        return float(self.s[-1]) if len(self.s) > 0 else 0.0

    @property
    def tangents(self) -> np.ndarray:
        """Return unit tangent vectors (N x 2) at each centerline waypoint."""
        n = len(self.xy)
        if n < 2:
            return np.tile(np.array([[1.0, 0.0]]), (n, 1))
        if self.closed:
            diff = np.roll(self.xy, -1, axis=0) - np.roll(self.xy, 1, axis=0)
        else:
            diff = np.zeros_like(self.xy)
            diff[1:-1] = self.xy[2:] - self.xy[:-2]
            diff[0] = self.xy[1] - self.xy[0]
            diff[-1] = self.xy[-1] - self.xy[-2]
        norms = np.hypot(diff[:, 0], diff[:, 1])
        norms = np.where(norms > 1e-12, norms, 1.0)
        return diff / norms[:, None]


def find_eufs_tracks_dir() -> Optional[Path]:
    """Locate the MFE26-eufs-sim `eufs_tracks/csv` directory on host or inside Docker."""
    candidates: list[Path] = []
    eufs_master = os.environ.get('EUFS_MASTER', '')
    if eufs_master:
        candidates.append(Path(eufs_master) / 'eufs_tracks' / 'csv')

    here = Path(__file__).resolve()
    for parent in here.parents:
        candidates.append(parent / 'MFE26-eufs-sim' / 'eufs_tracks' / 'csv')

    candidates.extend([
        Path('/root/Develop/MFE26-eufs-sim/eufs_tracks/csv'),
        Path.home() / 'Develop' / 'MFE26-eufs-sim' / 'eufs_tracks' / 'csv',
    ])

    for path in candidates:
        if path.is_dir() and any(path.glob('*.csv')):
            return path
    return None


def load_track_csv(csv_path: str | Path) -> TrackData:
    """Load an EUFS track CSV, ignoring `inactive_noise` rows."""
    path = Path(csv_path)
    buckets: dict[str, list[list[float]]] = {
        'blue': [],
        'yellow': [],
        'big_orange': [],
        'orange': [],
        'midpoint': [],
    }
    car_start = (0.0, 0.0, 0.0)

    with path.open('r', newline='') as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            tag = (row.get('tag') or '').strip()
            if not tag or tag == 'inactive_noise':
                continue
            x = float(row['x'])
            y = float(row['y'])
            if tag == 'car_start':
                yaw = float(row.get('direction') or 0.0)
                car_start = (x, y, yaw)
            elif tag in buckets:
                buckets[tag].append([x, y])

    def _to_arr(rows: list[list[float]]) -> np.ndarray:
        if not rows:
            return np.zeros((0, 2), dtype=np.float64)
        return np.asarray(rows, dtype=np.float64).reshape(-1, 2)

    return TrackData(
        name=path.stem,
        blue=_to_arr(buckets['blue']),
        yellow=_to_arr(buckets['yellow']),
        big_orange=_to_arr(buckets['big_orange']),
        orange=_to_arr(buckets['orange']),
        midpoints=_to_arr(buckets['midpoint']),
        car_start=car_start,
    )


def save_centerline_csv(csv_path: str | Path, cl: CenterlineData) -> None:
    """Save a CenterlineData object to CSV with header metadata comment."""
    path = Path(csv_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as fh:
        fh.write(f'# closed={1 if cl.closed else 0}\n')
        writer = csv.writer(fh)
        writer.writerow(['s', 'x', 'y', 'left_width', 'right_width', 'curvature'])
        for i in range(len(cl.s)):
            writer.writerow([
                f'{cl.s[i]:.4f}',
                f'{cl.xy[i, 0]:.6f}',
                f'{cl.xy[i, 1]:.6f}',
                f'{cl.left_width[i]:.4f}',
                f'{cl.right_width[i]:.4f}',
                f'{cl.curvature[i]:.6f}',
            ])


def load_centerline_csv(csv_path: str | Path) -> CenterlineData:
    """Load a ground truth centerline CSV saved by `save_centerline_csv`."""
    path = Path(csv_path)
    closed = True
    lines: list[str] = []
    with path.open('r', newline='') as fh:
        for line in fh:
            if line.startswith('#'):
                if 'closed=0' in line:
                    closed = False
                elif 'closed=1' in line:
                    closed = True
                continue
            lines.append(line)

    reader = csv.DictReader(lines)
    s_list, xy_list, lw_list, rw_list, k_list = [], [], [], [], []
    for row in reader:
        s_list.append(float(row['s']))
        xy_list.append([float(row['x']), float(row['y'])])
        lw_list.append(float(row['left_width']))
        rw_list.append(float(row['right_width']))
        k_list.append(float(row['curvature']))

    return CenterlineData(
        s=np.asarray(s_list, dtype=np.float64),
        xy=np.asarray(xy_list, dtype=np.float64).reshape(-1, 2),
        left_width=np.asarray(lw_list, dtype=np.float64),
        right_width=np.asarray(rw_list, dtype=np.float64),
        curvature=np.asarray(k_list, dtype=np.float64),
        closed=closed,
    )
