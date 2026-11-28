"""
Standardized path planning evaluation metrics (pure Python, no ROS).

Shared identically between Tier 1 offline replay tests, Tier 2 live Gazebo
evaluation (`path_evaluator_node`), Tier 3 robustness sweeps, and Phase 5
baseline diffs.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

import numpy as np

from mfe_common.geometry import arc_length_guarded, dist_to_polyline, wrap_angle
from mfe_path_planning.core.paths import path_point_at
from mfe_path_planning.eval.tracks import CenterlineData


def to_car_frame(xy: np.ndarray, car_pos: np.ndarray, car_dir: np.ndarray) -> np.ndarray:
    """Transform Nx2 map-frame points into the car frame (+X forward, +Y left)."""
    pts = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    if len(pts) == 0:
        return pts
    pos = np.asarray(car_pos, dtype=np.float64)
    fwd = np.asarray(car_dir, dtype=np.float64)
    norm = float(np.hypot(fwd[0], fwd[1]))
    c, s = (fwd[0] / norm, fwd[1] / norm) if norm > 1e-12 else (1.0, 0.0)
    d = pts - pos[None, :]
    return np.column_stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]])


def summarize_curvature(xy: np.ndarray) -> tuple[float, float]:
    """Return (mean_abs_curvature, max_abs_curvature) in 1/m for a polyline."""
    pts = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    if len(pts) < 3:
        return 0.0, 0.0
    v1 = pts[1:-1] - pts[:-2]
    v2 = pts[2:] - pts[1:-1]
    v3 = pts[2:] - pts[:-2]
    l1 = np.hypot(v1[:, 0], v1[:, 1])
    l2 = np.hypot(v2[:, 0], v2[:, 1])
    l3 = np.hypot(v3[:, 0], v3[:, 1])
    valid = (l1 > 0.15) & (l2 > 0.15) & (l3 > 0.25)
    if not np.any(valid):
        return 0.0, 0.0
    cross = np.abs(v1[valid, 0] * v2[valid, 1] - v1[valid, 1] * v2[valid, 0])
    kappa = 2.0 * cross / (l1[valid] * l2[valid] * l3[valid])
    return float(np.mean(kappa)), float(np.max(kappa))


def _project_index(
    cl: CenterlineData,
    car_pos: np.ndarray,
    car_dir: np.ndarray,
    hint_idx: Optional[int] = None,
) -> int:
    """Find the GT centerline index nearest car_pos, preferring local window + aligned heading."""
    n = len(cl.xy)
    tangents = cl.tangents
    if hint_idx is not None and n > 20:
        # Search +/- 40 waypoints around hint to disambiguate crossings (skidpad/figure-8)
        win = 40
        if cl.closed:
            idxs = np.arange(hint_idx - win, hint_idx + win + 1) % n
        else:
            lo = max(0, hint_idx - win)
            hi = min(n, hint_idx + win + 1)
            idxs = np.arange(lo, hi)
        d = np.hypot(cl.xy[idxs, 0] - car_pos[0], cl.xy[idxs, 1] - car_pos[1])
        align = tangents[idxs] @ car_dir
        # Soft penalty against opposite-heading leg at crossings
        score = d + np.where(align < -0.2, 10.0, 0.0)
        best = int(idxs[int(np.argmin(score))])
        if float(np.hypot(*(cl.xy[best] - car_pos))) < 8.0:
            return best

    d_all = np.hypot(cl.xy[:, 0] - car_pos[0], cl.xy[:, 1] - car_pos[1])
    align_all = tangents @ car_dir
    score_all = d_all + np.where(align_all < -0.2, 10.0, 0.0)
    return int(np.argmin(score_all))


def _local_gt_slice(
    cl: CenterlineData,
    center_idx: int,
    back_m: float = 5.0,
    ahead_m: float = 25.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Extract a contiguous slice of the GT centerline around center_idx."""
    n = len(cl.xy)
    ds = cl.total_length_m / max(n - (0 if cl.closed else 1), 1)
    ds = max(ds, 0.25)
    n_back = int(math.ceil(back_m / ds))
    n_ahead = int(math.ceil(ahead_m / ds))
    if cl.closed:
        idxs = np.arange(center_idx - n_back, center_idx + n_ahead + 1) % n
    else:
        lo = max(0, center_idx - n_back)
        hi = min(n, center_idx + n_ahead + 1)
        idxs = np.arange(lo, hi)
    return cl.xy[idxs], cl.tangents[idxs], cl.left_width[idxs], cl.right_width[idxs]


def _slice_ahead_by_arc(
    path_xy: np.ndarray,
    car_pos: np.ndarray,
    car_dir: np.ndarray,
    max_horizon_m: float,
) -> np.ndarray:
    """Keep waypoints ahead of the car (x_car >= -0.2 m) up to max_horizon_m arc length."""
    pts = np.asarray(path_xy, dtype=np.float64).reshape(-1, 2)
    if len(pts) < 2:
        return pts
    car_xy = to_car_frame(pts, car_pos, car_dir)
    # Start from the point closest to the car that isn't far behind
    fwd_mask = car_xy[:, 0] >= -0.2
    if not np.any(fwd_mask):
        return np.zeros((0, 2), dtype=np.float64)
    start_i = int(np.argmax(fwd_mask))
    sub = pts[start_i:]
    if len(sub) < 2:
        return sub
    seg = np.hypot(np.diff(sub[:, 0]), np.diff(sub[:, 1]))
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    keep = cum <= max_horizon_m
    if int(np.count_nonzero(keep)) < 2:
        return sub[:2]
    return sub[keep]


@dataclass
class _Bucket:
    """Accumulates per-cycle arrays for one evaluation group (e.g. lap 1 or lap 2+)."""

    cycles_total: int = 0
    cycles_empty: int = 0
    ctes: list = None
    clearances: list = None
    oob_counts: int = 0
    total_eval_pts: int = 0
    horizons: list = None
    max_curvatures: list = None
    mean_curvatures: list = None
    gt_mean_curvatures: list = None
    jump_lat5: list = None
    jump_head_deg: list = None
    cycle_ms: list = None

    def __post_init__(self):
        self.ctes = []
        self.clearances = []
        self.horizons = []
        self.max_curvatures = []
        self.mean_curvatures = []
        self.gt_mean_curvatures = []
        self.jump_lat5 = []
        self.jump_head_deg = []
        self.cycle_ms = []

    def to_summary(self) -> dict:
        """Convert accumulated samples into a JSON-serializable summary dictionary."""
        cte_all = np.concatenate(self.ctes) if self.ctes else np.zeros(0)
        clr = np.asarray(self.clearances, dtype=np.float64)
        hor = np.asarray(self.horizons, dtype=np.float64)
        k_max = np.asarray(self.max_curvatures, dtype=np.float64)
        k_mean = np.asarray(self.mean_curvatures, dtype=np.float64)
        gt_k = np.asarray(self.gt_mean_curvatures, dtype=np.float64)
        j_lat = np.asarray(self.jump_lat5, dtype=np.float64)
        j_hd = np.asarray(self.jump_head_deg, dtype=np.float64)
        cms = np.asarray(self.cycle_ms, dtype=np.float64)

        def _stat(arr: np.ndarray, fn, default: float = 0.0) -> float:
            return float(fn(arr)) if len(arr) > 0 else default

        jump_events = int(np.count_nonzero((j_lat > 1.0) | (j_hd > 20.0))) if len(j_lat) else 0
        gt_k_mean = _stat(gt_k, np.mean)
        plan_k_mean = _stat(k_mean, np.mean)
        curv_ratio = (plan_k_mean / gt_k_mean) if gt_k_mean > 1e-6 else 1.0

        return {
            'cycles_total': self.cycles_total,
            'cycles_empty': self.cycles_empty,
            'empty_path_frac': (self.cycles_empty / self.cycles_total)
            if self.cycles_total > 0 else 1.0,
            'cte_mean_m': _stat(cte_all, np.mean),
            'cte_p95_m': _stat(cte_all, lambda a: np.percentile(a, 95)),
            'cte_max_m': _stat(cte_all, np.max),
            'min_cone_clearance_m': _stat(clr, np.min, default=0.0),
            'p05_cone_clearance_m': _stat(clr, lambda a: np.percentile(a, 5), default=0.0),
            'out_of_bounds_frac': (self.oob_counts / self.total_eval_pts)
            if self.total_eval_pts > 0 else 0.0,
            'horizon_mean_m': _stat(hor, np.mean),
            'horizon_min_m': _stat(hor, np.min),
            'max_curvature_inv_m': _stat(k_max, np.max),
            'mean_curvature_inv_m': plan_k_mean,
            'curvature_ratio_vs_gt': float(curv_ratio),
            'jump_lat5_p95_m': _stat(j_lat, lambda a: np.percentile(a, 95)),
            'jump_head_p95_deg': _stat(j_hd, lambda a: np.percentile(a, 95)),
            'jump_rate': (jump_events / len(j_lat)) if len(j_lat) > 0 else 0.0,
            'cycle_ms_mean': _stat(cms, np.mean),
            'cycle_ms_p95': _stat(cms, lambda a: np.percentile(a, 95)),
            'cycle_ms_max': _stat(cms, np.max),
        }


class MetricsAccumulator:
    """Evaluates planned paths against a GT centerline and GT wall cones cycle by cycle."""

    def __init__(
        self,
        centerline: CenterlineData,
        wall_cones: np.ndarray,
        eval_horizon_m: float = 15.0,
        worst_k: int = 5,
    ):
        self.cl = centerline
        self.wall_cones = np.asarray(wall_cones, dtype=np.float64).reshape(-1, 2)
        self.eval_horizon_m = float(eval_horizon_m)
        self.worst_k = int(worst_k)

        self._overall = _Bucket()
        self._by_group: dict[str, _Bucket] = {}
        self._last_gt_idx: Optional[int] = None
        self._prev_path: Optional[np.ndarray] = None
        self._worst_frames: list[dict] = []

    def add_cycle(
        self,
        car_pos: np.ndarray,
        car_dir: np.ndarray,
        planned_xy: Optional[np.ndarray],
        cycle_ms: float = 0.0,
        group: str = 'lap_1',
    ) -> dict:
        """Record one planning cycle and return per-cycle instantaneous metrics."""
        car_pos = np.asarray(car_pos, dtype=np.float64)
        car_dir = np.asarray(car_dir, dtype=np.float64)
        norm = float(np.hypot(car_dir[0], car_dir[1]))
        car_dir = car_dir / norm if norm > 1e-12 else np.array([1.0, 0.0])

        grp = self._by_group.setdefault(group, _Bucket())
        for b in (self._overall, grp):
            b.cycles_total += 1
            b.cycle_ms.append(float(cycle_ms))

        if planned_xy is None or len(planned_xy) < 2:
            for b in (self._overall, grp):
                b.cycles_empty += 1
                b.horizons.append(0.0)
            self._prev_path = None
            return {'empty': True, 'cte_max_m': None, 'horizon_m': 0.0}

        pts = np.asarray(planned_xy, dtype=np.float64).reshape(-1, 2)
        if not np.all(np.isfinite(pts)):
            for b in (self._overall, grp):
                b.cycles_empty += 1
                b.horizons.append(0.0)
            self._prev_path = None
            return {'empty': True, 'cte_max_m': None, 'horizon_m': 0.0}

        # Project car onto GT centerline and slice local reference window
        gt_idx = _project_index(self.cl, car_pos, car_dir, self._last_gt_idx)
        self._last_gt_idx = gt_idx
        gt_xy, gt_tan, gt_lw, gt_rw = _local_gt_slice(
            self.cl, gt_idx, back_m=4.0, ahead_m=self.eval_horizon_m + 10.0
        )

        # Slice planned path to 0..eval_horizon_m ahead of the car
        ahead = _slice_ahead_by_arc(pts, car_pos, car_dir, self.eval_horizon_m)
        horizon_m = arc_length_guarded(pts)
        if len(ahead) < 2:
            for b in (self._overall, grp):
                b.cycles_empty += 1
                b.horizons.append(horizon_m)
            self._prev_path = pts
            return {'empty': True, 'cte_max_m': None, 'horizon_m': horizon_m}

        # Lateral error against local GT centerline polyline
        cte = np.array([dist_to_polyline(p, gt_xy) for p in ahead], dtype=np.float64)

        # Corridor containment against local GT left/right widths
        oob = 0
        for p in ahead:
            d_gt = np.hypot(gt_xy[:, 0] - p[0], gt_xy[:, 1] - p[1])
            j = int(np.argmin(d_gt))
            t = gt_tan[j]
            n_left = np.array([-t[1], t[0]])
            signed_lat = float((p - gt_xy[j]) @ n_left)
            if signed_lat > gt_lw[j] or signed_lat < -gt_rw[j]:
                oob += 1

        # Cone clearance across evaluated planned points
        if len(self.wall_cones) > 0:
            d_cones = np.hypot(
                self.wall_cones[None, :, 0] - ahead[:, None, 0],
                self.wall_cones[None, :, 1] - ahead[:, None, 1],
            )
            min_clr = float(np.min(d_cones))
        else:
            min_clr = float('inf')

        mean_k, max_k = summarize_curvature(ahead)
        gt_mean_k, _ = summarize_curvature(gt_xy)

        # Frame-to-frame jumpiness at 5 m ahead
        j_lat5 = None
        j_head = None
        if (
            self._prev_path is not None
            and arc_length_guarded(pts) >= 6.0
            and arc_length_guarded(self._prev_path) >= 6.0
        ):
            p5 = path_point_at(pts, 5.0)
            j_lat5 = dist_to_polyline(p5, self._prev_path)
            h_new = math.atan2(p5[1] - pts[0, 1], p5[0] - pts[0, 0])
            p5_prev = path_point_at(self._prev_path, 5.0)
            h_prev = math.atan2(
                p5_prev[1] - self._prev_path[0, 1], p5_prev[0] - self._prev_path[0, 0]
            )
            j_head = math.degrees(abs(wrap_angle(h_new - h_prev)))
        self._prev_path = pts

        for b in (self._overall, grp):
            b.ctes.append(cte)
            b.clearances.append(min_clr)
            b.oob_counts += oob
            b.total_eval_pts += len(ahead)
            b.horizons.append(horizon_m)
            b.max_curvatures.append(max_k)
            b.mean_curvatures.append(mean_k)
            b.gt_mean_curvatures.append(gt_mean_k)
            if j_lat5 is not None:
                b.jump_lat5.append(j_lat5)
                b.jump_head_deg.append(j_head)

        cte_max = float(np.max(cte))
        self._maybe_record_worst(
            cte_max=cte_max,
            min_clr=min_clr,
            car_pos=car_pos,
            car_dir=car_dir,
            planned_xy=ahead,
            gt_s=float(self.cl.s[gt_idx]),
            group=group,
        )
        return {
            'empty': False,
            'cte_mean_m': float(np.mean(cte)),
            'cte_max_m': cte_max,
            'min_cone_clearance_m': min_clr,
            'horizon_m': horizon_m,
        }

    def _maybe_record_worst(
        self,
        cte_max: float,
        min_clr: float,
        car_pos: np.ndarray,
        car_dir: np.ndarray,
        planned_xy: np.ndarray,
        gt_s: float,
        group: str,
    ) -> None:
        """Keep top-K worst CTE frames for diagnostic plotting."""
        if self.worst_k <= 0:
            return
        if len(self._worst_frames) < self.worst_k or cte_max > self._worst_frames[-1]['cte_max_m']:
            entry = {
                'group': group,
                'gt_s': round(gt_s, 2),
                'cte_max_m': round(cte_max, 4),
                'min_cone_clearance_m': round(min_clr, 4),
                'car_pos': [round(float(car_pos[0]), 3), round(float(car_pos[1]), 3)],
                'car_dir': [round(float(car_dir[0]), 4), round(float(car_dir[1]), 4)],
                'planned_xy': [
                    [round(float(x), 3), round(float(y), 3)] for x, y in planned_xy[::2]
                ],
            }
            self._worst_frames.append(entry)
            self._worst_frames.sort(key=lambda r: r['cte_max_m'], reverse=True)
            del self._worst_frames[self.worst_k:]

    def summarize(self) -> dict:
        """Return full JSON-serializable summary (overall + per-group + worst frames)."""
        return {
            'overall': self._overall.to_summary(),
            'groups': {name: bucket.to_summary() for name, bucket in self._by_group.items()},
            'worst_frames': list(self._worst_frames),
        }
