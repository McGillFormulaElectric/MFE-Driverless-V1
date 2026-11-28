"""
Offline virtual-car replay harness for mfe_path_planning.core (pure Python, no ROS).

Steps a virtual vehicle along a track's ground truth centerline, simulates
range/FOV-limited cone observations (coloured, uncoloured LiDAR-only, or
mixed camera+LiDAR), invokes the real planning core (`ConeMap`, `LocalPlanner`,
`BoundaryTracer`, `JumpGate`, `RaceLineRunner`, `fsd_path_planning`), and
evaluates every cycle via `MetricsAccumulator`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import time
from typing import Callable, Optional

import numpy as np

from mfe_path_planning.core.accel import AccelMidline
from mfe_path_planning.core.boundary_tracer import BoundaryTracer
from mfe_path_planning.core.colour import ColourHint, PipelineLatch, SideVotes
from mfe_path_planning.core.cone_map import ConeMap, FSD_TYPES
from mfe_path_planning.core.jump_gate import JumpGate
from mfe_path_planning.core.lap import LapBan, LapReturnReset
from mfe_path_planning.core.params import PlannerParams
from mfe_path_planning.core.planning import LocalPlanner, PlanContext
from mfe_path_planning.core.raceline_runner import RaceLineRunner, raceline_gate
from mfe_path_planning.eval.metrics import MetricsAccumulator
from mfe_path_planning.eval.tracks import CenterlineData, TRACK_MISSIONS, TrackData

# Match mfe_msgs/Cone integer constants without importing ROS messages
CONE_BLUE = 0
CONE_YELLOW = 1
CONE_ORANGE_BIG = 2
CONE_ORANGE_SMALL = 3
CONE_UNKNOWN = 4

MFE_TO_FSD = {
    CONE_BLUE: FSD_TYPES.left,
    CONE_YELLOW: FSD_TYPES.right,
    CONE_ORANGE_BIG: FSD_TYPES.unknown,
    CONE_ORANGE_SMALL: FSD_TYPES.unknown,
    CONE_UNKNOWN: FSD_TYPES.unknown,
}


@dataclass
class ReplayConfig:
    """Configuration for one offline planning replay run."""

    mode: str = 'coloured'                  # coloured | trace | colour_hint | fasttube_uncoloured
    mission: Optional[str] = None           # defaults to TRACK_MISSIONS[track.name]
    step_m: float = 1.0                     # pose spacing along GT centerline (m)
    n_laps: int = 1                         # number of laps (closed tracks only)
    sensor_range_m: float = 20.0            # cone sensing radius (m)
    sensor_fov_deg: float = 180.0           # total forward field of view (deg)
    camera_colour_range_m: float = 14.0     # colour_hint: max distance with colour labels
    eval_horizon_m: float = 15.0            # car-frame horizon evaluated vs GT centerline
    raceline_enable: bool = False           # enable lap-2+ offline min-curvature raceline
    planner_overrides: dict = field(default_factory=dict)
    # Optional hook: fn(obs, car_pos, car_dir, step_idx) -> (obs, car_pos, car_dir)
    perturbation: Optional[Callable] = None


def _build_lib_planner(mission: str):
    """Instantiate real fsd_path_planning.PathPlanner matching PathPlannerNode."""
    try:
        from fsd_path_planning import MissionTypes, PathPlanner
    except ImportError:
        from ft_fsd_path_planning import MissionTypes, PathPlanner

    mission_map = {
        'autocross': MissionTypes.autocross,
        'trackdrive': MissionTypes.trackdrive,
        'acceleration': MissionTypes.acceleration,
        'skidpad': MissionTypes.autocross,
    }
    m_enum = mission_map.get(mission, MissionTypes.autocross)
    planner = PathPlanner(m_enum)
    # Match PathPlannerNode._trim_accel_known_path
    if m_enum == MissionTypes.acceleration and planner.relocalizer is not None:
        known = planner.relocalizer.get_known_global_path()
        ret = known[:, 1] > 1.0
        if ret.any():
            known = known[:int(np.argmax(ret))]
            planner.relocalizer.get_known_global_path = lambda: known
    return planner, (m_enum == MissionTypes.trackdrive)


def _visible_cones(
    track: TrackData,
    car_pos: np.ndarray,
    car_dir: np.ndarray,
    cfg: ReplayConfig,
) -> list[tuple[float, float, int]]:
    """Simulate range + FOV-limited cone detections for the current car pose."""
    half_fov_cos = math.cos(math.radians(cfg.sensor_fov_deg * 0.5))
    r2_max = cfg.sensor_range_m ** 2
    cam_r2 = cfg.camera_colour_range_m ** 2

    all_sets = [
        (track.blue, CONE_BLUE),
        (track.yellow, CONE_YELLOW),
        (track.big_orange, CONE_ORANGE_BIG),
        (track.orange, CONE_ORANGE_SMALL),
    ]
    obs: list[tuple[float, float, int]] = []
    for arr, true_col in all_sets:
        if len(arr) == 0:
            continue
        rel = arr - car_pos[None, :]
        d2 = rel[:, 0] ** 2 + rel[:, 1] ** 2
        in_r = d2 <= r2_max
        if not np.any(in_r):
            continue
        sub_arr = arr[in_r]
        sub_rel = rel[in_r]
        sub_d2 = d2[in_r]
        norms = np.sqrt(np.maximum(sub_d2, 1e-12))
        cos_a = (sub_rel @ car_dir) / norms
        # Keep cones inside forward FOV or within 3 m behind the car (matches LiDAR/map overlap)
        in_fov = (cos_a >= half_fov_cos) | (sub_d2 <= 9.0)
        for (x, y), dist2 in zip(sub_arr[in_fov], sub_d2[in_fov]):
            if cfg.mode == 'coloured':
                col = true_col
            elif cfg.mode in ('trace', 'fasttube_uncoloured'):
                col = CONE_UNKNOWN if true_col in (CONE_BLUE, CONE_YELLOW) else true_col
            elif cfg.mode == 'colour_hint':
                if true_col in (CONE_BLUE, CONE_YELLOW) and dist2 > cam_r2:
                    col = CONE_UNKNOWN
                else:
                    col = true_col
            else:
                raise ValueError(f'Unknown replay mode: {cfg.mode}')
            obs.append((float(x), float(y), int(col)))
    return obs


def _build_pose_sequence(
    cl: CenterlineData,
    step_m: float,
    n_laps: int,
) -> list[tuple[np.ndarray, np.ndarray, int]]:
    """Build ordered (car_pos, car_dir, lap_index) poses along a centerline."""
    tangents = cl.tangents
    n = len(cl.xy)
    ds = cl.total_length_m / max(n - (0 if cl.closed else 1), 1)
    stride = max(1, int(round(step_m / max(ds, 1e-3))))
    indices = list(range(0, n, stride))

    poses: list[tuple[np.ndarray, np.ndarray, int]] = []
    laps = max(1, n_laps) if cl.closed else 1
    for lap in range(laps):
        for idx in indices:
            poses.append((cl.xy[idx].copy(), tangents[idx].copy(), lap))
    return poses


def run_offline_replay(
    track: TrackData,
    centerline: CenterlineData,
    config: Optional[ReplayConfig] = None,
) -> dict:
    """Execute a full offline replay on one track and return structured metrics."""
    cfg = config or ReplayConfig()
    mission = cfg.mission or TRACK_MISSIONS.get(track.name, 'autocross')

    unknown_mode_map = {
        'coloured': 'trace',
        'trace': 'trace',
        'colour_hint': 'colour_hint',
        'fasttube_uncoloured': 'fasttube',
    }
    params_kw = dict(
        mission=mission,
        unknown_cone_mode=unknown_mode_map[cfg.mode],
        raceline_enable=cfg.raceline_enable,
    )
    params_kw.update(cfg.planner_overrides)
    params = PlannerParams(**params_kw)

    lib_planner, global_planning = _build_lib_planner(mission)
    cones = ConeMap(
        FSD_TYPES,
        MFE_TO_FSD,
        skip_colours=(CONE_ORANGE_BIG, CONE_ORANGE_SMALL),
        colour_vote=params.colour_vote,
        vote_min=params.vote_min,
        vote_margin=params.vote_margin,
        cone_expiry=params.cone_expiry,
        expiry_min_obs=params.expiry_min_obs,
        expiry_cycles=params.expiry_cycles,
    )
    tracer = BoundaryTracer()
    lap_ban = LapBan()
    side_votes = SideVotes()
    hint = ColourHint()
    pipeline = PipelineLatch(params.min_coloured_each, params.revert_after)
    jump = JumpGate(
        params.jump_gate,
        params.jump_heading_deg,
        params.jump_lateral_m,
        params.jump_hold_max,
        params.jump_min_path_m,
        params.jump_remaining_guard,
    )
    accel = AccelMidline(params.accel_midline_alpha)
    rl = RaceLineRunner(
        raceline_gate(mission, params.raceline_enable),
        params.raceline_after_lap,
        params.raceline_extra_margin_m,
        params.raceline_a_lat,
        params.raceline_v_max,
        params.raceline_window_m,
    )
    lap_reset = None
    if params.lap_return_reset and mission not in ('skidpad', 'acceleration'):
        lap_reset = LapReturnReset(
            params.lap_return_start_r_m,
            params.lap_return_min_travel_m,
            params.lap_return_len_frac,
        )

    local = LocalPlanner(
        FSD_TYPES,
        lib_planner,
        tracer,
        cones,
        lap_ban,
        side_votes,
        hint,
        pipeline,
        accel,
        unknown_mode=params.unknown_cone_mode,
        colour_after_lap=params.colour_after_lap,
        pipeline_hysteresis=params.pipeline_hysteresis,
        accel_refit=params.accel_refit_midline,
    )

    acc = MetricsAccumulator(
        centerline=centerline,
        wall_cones=track.all_wall_cones,
        eval_horizon_m=cfg.eval_horizon_m,
    )

    poses = _build_pose_sequence(centerline, cfg.step_m, cfg.n_laps)
    raceline_built = False

    for step_idx, (car_pos, car_dir, lap_idx) in enumerate(poses):
        obs = _visible_cones(track, car_pos, car_dir, cfg)
        if cfg.perturbation is not None:
            obs, car_pos, car_dir = cfg.perturbation(obs, car_pos, car_dir, step_idx)

        # Simulate _odom_cb side effects
        tracer.add_pose(car_pos)
        if rl.collect_wanted(lap_idx):
            rl.collect_odom(car_pos)
        if lap_reset is not None and lap_reset.update(car_pos):
            lap_ban.fire(cones.keys(), lap_reset.start, params.lap_return_radius_m)
        if lap_ban.banned:
            det = np.array([(x, y) for x, y, _ in obs], dtype=np.float64).reshape(-1, 2)
            lap_ban.lift(car_pos, car_dir, det)

        # Timed planning cycle
        t0 = time.perf_counter()
        cones.merge(obs, car_pos, car_dir)
        cones.evict_far(1000, car_pos)

        def _obs_arrays():
            if not obs:
                return np.zeros((0, 2), dtype=np.float64), np.zeros(0, dtype=int)
            arr = np.array([(x, y) for x, y, _ in obs], dtype=np.float64).reshape(-1, 2)
            cols = np.array([c for _, _, c in obs], dtype=int)
            return arr, cols

        planned_xy = None
        used_raceline = False
        window, _ = rl.cycle(car_pos, lap_idx, _obs_arrays)
        if window is not None:
            planned_xy = window[0]
            used_raceline = True
            raceline_built = True
        else:
            ctx = PlanContext(
                car_pos=car_pos,
                car_dir=car_dir,
                laps_done=lap_idx,
                mission=mission,
                global_planning=global_planning,
                now_s=float(step_idx) * 0.1,
            )
            plan = local.plan(ctx)
            if plan.publish and plan.centerline is not None and len(plan.centerline) >= 2:
                planned_xy, _ = jump.apply(np.asarray(plan.centerline), car_pos, True)
        cycle_ms = (time.perf_counter() - t0) * 1e3

        group = 'raceline' if used_raceline else f'lap_{lap_idx + 1}'
        acc.add_cycle(car_pos, car_dir, planned_xy, cycle_ms=cycle_ms, group=group)

    summary = acc.summarize()
    summary['meta'] = {
        'track': track.name,
        'mode': cfg.mode,
        'mission': mission,
        'n_laps': cfg.n_laps,
        'step_m': cfg.step_m,
        'raceline_enabled': params.raceline_enable,
        'raceline_built': raceline_built,
        'raceline_state': rl.state,
        'colour_flips': cones.colour_flips,
        'cones_expired': cones.cones_expired,
        'jump_rejects': jump.rejects,
    }
    return summary
