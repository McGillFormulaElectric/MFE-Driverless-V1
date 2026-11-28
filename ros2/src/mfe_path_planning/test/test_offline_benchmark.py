"""
Tier 1 offline multi-track path planning benchmarks against GT centerlines.

Tests both:
  - `coloured` (`no_perception` / GT cone colours via fsd_path_planning + local raceline)
  - `trace` and `colour_hint` (`perception` / LiDAR-only & mixed camera+LiDAR)
  - 2-lap offline raceline validation (when `quadprog` is installed)

Run via `scripts/run_planning_tests.sh tier1` or `pytest`.
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path

import pytest

from mfe_path_planning.eval.replay import ReplayConfig, run_offline_replay
from mfe_path_planning.eval.tracks import (
    CLOSED_LOOP_TRACKS,
    STATIC_TRACKS,
    load_centerline_csv,
    load_track_csv,
)

# Real fsd_path_planning + scipy are required for full offline benchmarks
pytest.importorskip('scipy.spatial')
pytest.importorskip('fsd_path_planning')

# Tracks benchmarked across all three modes (coloured, trace, colour_hint)
BENCHMARK_TRACKS = (
    'small_track',
    'rectangle',
    'garden_light',
    'boa_constrictor',
    'comp_2021',
    'its_a_mess',
    'peanut',
    'bone',
)

# Tracks with tightly packed parallel corridors where ft-fsd + local raceline has
# known higher baseline apex/corridor spread than standard circuits.
TIGHT_CORRIDOR_TRACKS = {'rectangle', 'garden_light', 'its_a_mess'}

# Representative subset for 2-lap offline raceline validation (requires quadprog)
RACELINE_TRACKS = (
    'small_track',
    'peanut',
    'garden_light',
)

# Pass/fail gates calibrated to the current planner pipelines:
# - `trace` / `colour_hint` output true corridor centerlines -> tight CTE & clearance gates.
# - `coloured` (autocross) runs per-cycle `_racing_line()` apex optimization -> wider CTE
#   allowance on centerline error, gated on OOB containment, horizon, and mean CTE.
CENTERLINE_MODE_GATES = {
    'cte_mean_max_m': 0.25,
    'cte_p95_max_m': 0.45,
    'p05_clearance_min_m': 0.80,
    'oob_frac_max': 0.025,
    'jump_lat5_p95_max_m': 0.50,
    'empty_frac_max': 0.15,
    'horizon_mean_min_m': 12.0,
    'cycle_ms_p95_max': 250.0,
}

COLOURED_STANDARD_GATES = {
    'cte_mean_max_m': 0.90,
    'cte_p95_max_m': 1.85,
    'p05_clearance_min_m': 0.10,
    'oob_frac_max': 0.03,
    'jump_lat5_p95_max_m': 1.25,
    'empty_frac_max': 0.05,
    'horizon_mean_min_m': 15.0,
    'cycle_ms_p95_max': 600.0,
}

COLOURED_TIGHT_GATES = {
    'cte_mean_max_m': 1.40,
    'cte_p95_max_m': 5.50,
    'p05_clearance_min_m': 0.08,
    'oob_frac_max': 0.12,
    'jump_lat5_p95_max_m': 5.50,
    'empty_frac_max': 0.05,
    'horizon_mean_min_m': 15.0,
    'cycle_ms_p95_max': 600.0,
}


@pytest.fixture(scope='module', autouse=True)
def warmup_ft_fsd():
    """Trigger one-time Numba JIT compilation before timing benchmark cycles."""
    import numpy as np
    from fsd_path_planning import MissionTypes, PathPlanner

    planner = PathPlanner(MissionTypes.autocross)
    xs = np.array([3.0, 6.0, 9.0, 12.0, 15.0, 18.0])
    ys = 0.02 * xs * xs
    cones = [np.zeros((0, 2), dtype=np.float64) for _ in range(5)]
    cones[1] = np.column_stack([xs, ys - 2.0])
    cones[2] = np.column_stack([xs, ys + 2.0])
    try:
        planner.calculate_path_in_global_frame(
            cones, np.array([0.0, 0.0]), np.array([1.0, 0.0]), return_intermediate_results=True
        )
    except Exception:
        pass


def _gates_for(track_name: str, mode: str) -> dict:
    """Return the calibrated pass/fail gate dictionary for (track_name, mode)."""
    if mode in ('trace', 'colour_hint'):
        return CENTERLINE_MODE_GATES
    if track_name in TIGHT_CORRIDOR_TRACKS:
        return COLOURED_TIGHT_GATES
    return COLOURED_STANDARD_GATES


def _assert_gates(track_name: str, mode: str, ov: dict, gates: dict) -> None:
    """Assert every benchmark metric against its configured gate."""
    tag = f'{track_name}/{mode}'
    assert ov['cycles_total'] >= 10, f'{tag}: too few cycles ({ov["cycles_total"]})'
    assert ov['empty_path_frac'] <= gates['empty_frac_max'], (
        f'{tag}: empty_path_frac={ov["empty_path_frac"]:.3f} > {gates["empty_frac_max"]}'
    )
    assert ov['cte_mean_m'] <= gates['cte_mean_max_m'], (
        f'{tag}: cte_mean={ov["cte_mean_m"]:.3f} m > {gates["cte_mean_max_m"]} m'
    )
    assert ov['cte_p95_m'] <= gates['cte_p95_max_m'], (
        f'{tag}: cte_p95={ov["cte_p95_m"]:.3f} m > {gates["cte_p95_max_m"]} m'
    )
    assert ov['p05_cone_clearance_m'] >= gates['p05_clearance_min_m'], (
        f'{tag}: p05_cone_clearance={ov["p05_cone_clearance_m"]:.3f} m '
        f'< {gates["p05_clearance_min_m"]} m'
    )
    assert ov['out_of_bounds_frac'] <= gates['oob_frac_max'], (
        f'{tag}: out_of_bounds_frac={ov["out_of_bounds_frac"]:.3f} > {gates["oob_frac_max"]}'
    )
    assert ov['jump_lat5_p95_m'] <= gates['jump_lat5_p95_max_m'], (
        f'{tag}: jump_lat5_p95={ov["jump_lat5_p95_m"]:.3f} m > {gates["jump_lat5_p95_max_m"]} m'
    )
    assert ov['horizon_mean_m'] >= gates['horizon_mean_min_m'], (
        f'{tag}: horizon_mean={ov["horizon_mean_m"]:.2f} m < {gates["horizon_mean_min_m"]} m'
    )
    assert ov['cycle_ms_p95'] <= gates['cycle_ms_p95_max'], (
        f'{tag}: cycle_ms_p95={ov["cycle_ms_p95"]:.1f} ms > {gates["cycle_ms_p95_max"]} ms'
    )


@pytest.fixture(scope='module')
def report_collector(report_out_path):
    """Collect per-case metric summaries and dump JSON on module teardown if requested."""
    records: list[dict] = []
    yield records
    if report_out_path and records:
        out_path = Path(report_out_path)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            'schema_version': 1,
            'suite': 'tier1_offline',
            'generated_utc': datetime.now(timezone.utc).isoformat(),
            'cases': records,
        }
        out_path.write_text(json.dumps(payload, indent=2) + '\n')


def test_all_static_gt_centerline_fixtures_present(centerline_fixtures_dir):
    """Verify every static EUFS track has a valid committed GT centerline fixture."""
    for name in STATIC_TRACKS:
        fixture = centerline_fixtures_dir / f'{name}.csv'
        assert fixture.is_file(), f'Missing GT centerline fixture: {fixture}'
        cl = load_centerline_csv(fixture)
        assert len(cl.xy) >= 20, f'Fixture {name} too short ({len(cl.xy)} pts)'
        assert cl.total_length_m >= 50.0, f'Fixture {name} length too small'


@pytest.mark.parametrize('track_name', BENCHMARK_TRACKS)
@pytest.mark.parametrize('mode', ('coloured', 'trace', 'colour_hint'))
def test_closed_loop_track_against_gt_centerline(
    track_name,
    mode,
    eufs_tracks_dir,
    centerline_fixtures_dir,
    report_collector,
):
    """Validate planner centerline accuracy, safety, horizon, and speed on static circuits."""
    track = load_track_csv(eufs_tracks_dir / f'{track_name}.csv')
    cl = load_centerline_csv(centerline_fixtures_dir / f'{track_name}.csv')

    cfg = ReplayConfig(
        mode=mode,
        step_m=2.0,
        n_laps=1,
        sensor_range_m=20.0,
        sensor_fov_deg=180.0,
        raceline_enable=False,
    )
    res = run_offline_replay(track, cl, cfg)
    report_collector.append(res)
    _assert_gates(track_name, mode, res['overall'], _gates_for(track_name, mode))


def test_acceleration_straight_against_gt_centerline(
    eufs_tracks_dir,
    centerline_fixtures_dir,
    report_collector,
):
    """Validate acceleration straight-line accuracy and clearance."""
    track = load_track_csv(eufs_tracks_dir / 'acceleration.csv')
    cl = load_centerline_csv(centerline_fixtures_dir / 'acceleration.csv')

    cfg = ReplayConfig(
        mode='coloured',
        mission='acceleration',
        step_m=2.0,
        n_laps=1,
        sensor_range_m=25.0,
        raceline_enable=False,
    )
    res = run_offline_replay(track, cl, cfg)
    report_collector.append(res)

    ov = res['overall']
    assert ov['empty_path_frac'] <= 0.10
    assert ov['cte_p95_m'] <= 0.25, f'accel cte_p95={ov["cte_p95_m"]:.3f} m'
    assert ov['min_cone_clearance_m'] >= 1.00, (
        f'accel clearance={ov["min_cone_clearance_m"]:.3f} m'
    )


@pytest.mark.parametrize('track_name', RACELINE_TRACKS)
def test_two_lap_raceline_corridor_and_curvature(
    track_name,
    eufs_tracks_dir,
    centerline_fixtures_dir,
    report_collector,
):
    """
    Validate Lap 1 centerline + Lap 2 offline min-curvature raceline.

    Lap 2+ intentionally apex-cuts away from the centerline, so it is gated on
    corridor containment, cone clearance, and non-increasing curvature vs. GT.
    """
    pytest.importorskip('quadprog', reason='quadprog not installed (required by core.raceline)')

    track = load_track_csv(eufs_tracks_dir / f'{track_name}.csv')
    cl = load_centerline_csv(centerline_fixtures_dir / f'{track_name}.csv')

    cfg = ReplayConfig(
        mode='coloured',
        step_m=1.5,
        n_laps=2,
        sensor_range_m=20.0,
        raceline_enable=True,
    )
    res = run_offline_replay(track, cl, cfg)
    report_collector.append(res)

    assert res['meta']['raceline_built'] is True, (
        f'{track_name}: offline raceline failed to build (state={res["meta"]["raceline_state"]})'
    )
    rl_grp = res['groups'].get('raceline')
    assert rl_grp is not None and rl_grp['cycles_total'] >= 10
    assert rl_grp['empty_path_frac'] <= 0.05
    assert rl_grp['min_cone_clearance_m'] >= 0.50, (
        f'{track_name}/raceline: min_cone_clearance={rl_grp["min_cone_clearance_m"]:.3f} m'
    )
    assert rl_grp['out_of_bounds_frac'] <= 0.02, (
        f'{track_name}/raceline: out_of_bounds_frac={rl_grp["out_of_bounds_frac"]:.3f}'
    )
    assert rl_grp['curvature_ratio_vs_gt'] <= 1.15, (
        f'{track_name}/raceline: curvature_ratio_vs_gt={rl_grp["curvature_ratio_vs_gt"]:.3f}'
    )


@pytest.mark.parametrize('mode', ('coloured', 'trace'))
def test_hairpins_track_offline_benchmark(
    mode,
    eufs_tracks_dir,
    centerline_fixtures_dir,
    report_collector,
):
    """Validate long multi-hairpin circuit (825 m) in both coloured and trace modes."""
    track = load_track_csv(eufs_tracks_dir / 'hairpins_increasing_difficulty.csv')
    cl = load_centerline_csv(centerline_fixtures_dir / 'hairpins_increasing_difficulty.csv')

    cfg = ReplayConfig(
        mode=mode,
        step_m=4.0,
        n_laps=1,
        sensor_range_m=20.0,
        raceline_enable=False,
    )
    res = run_offline_replay(track, cl, cfg)
    report_collector.append(res)
    _assert_gates(
        'hairpins_increasing_difficulty',
        mode,
        res['overall'],
        _gates_for('hairpins_increasing_difficulty', mode),
    )


def test_closed_loop_tracks_constant_is_subset_of_static():
    """Guard static track registry consistency."""
    assert set(CLOSED_LOOP_TRACKS).issubset(set(STATIC_TRACKS))
