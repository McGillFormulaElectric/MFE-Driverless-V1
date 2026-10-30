"""
Race line (core.raceline) on a closed oval: the line stays inside the walls with margin, and the.

speed profile respects v_max and the lateral acceleration limit. No ROS.
"""
import math

import numpy as np

from mfe_path_planning.core import raceline as rl_mod
from mfe_path_planning.core.raceline import RaceLineParams

A, B = 20.0, 10.0           # oval semi-axes (m)
HALF_TRACK = 3.0            # walls sit 3 m either side of the driven line


def _walls(n_pts=400):
    """Blue (inside) and yellow (outside) wall cones of the oval, 0.5 m spaced along each wall."""
    t = np.linspace(0.0, 2.0 * math.pi, n_pts, endpoint=False)
    inner = np.column_stack([(A - HALF_TRACK) * np.cos(t), (B - HALF_TRACK) * np.sin(t)])
    outer = np.column_stack([(A + HALF_TRACK) * np.cos(t), (B + HALF_TRACK) * np.sin(t)])
    cones = np.vstack([inner, outer])
    colours = np.array([0] * n_pts + [1] * n_pts)   # blue inside, yellow outside
    return cones, colours


def _trail(n_pts=600):
    t = np.linspace(0.0, 2.0 * math.pi, n_pts, endpoint=False)
    return np.column_stack([A * np.cos(t), B * np.sin(t)])


def _build():
    cones, colours = _walls()
    params = RaceLineParams(extra_margin_m=0.25, a_lat=6.0, v_max=8.0)
    return rl_mod.build(cones, _trail(), params, colours=colours), cones, params


def test_oval_line_is_built_and_closed():
    race, _, _ = _build()
    assert race is not None
    assert len(race.xy) > 50
    assert abs(race.length_m - 2 * math.pi * (A + B) / 2 * 1.0) < 0.1 * race.length_m


def test_line_stays_inside_walls_with_margin():
    race, cones, params = _build()
    margin = rl_mod.HALF_W + rl_mod.SAFETY + params.extra_margin_m
    dmin = np.min(np.hypot(race.xy[:, None, 0] - cones[None, :, 0], race.xy[:, None, 1] - cones[None, :, 1]), axis=1)
    assert dmin.min() > margin - 0.3, dmin.min()
    assert race.min_side_width_m >= 0.0
    assert np.all(race.w_left >= 0.0) and np.all(race.w_right >= 0.0)


def test_speed_profile_respects_caps():
    race, _, params = _build()
    assert np.all(race.v <= params.v_max + 1e-6)
    assert np.all(race.v > 0.0)
    lat_acc = race.v ** 2 * np.abs(race.kappa)
    assert np.percentile(lat_acc, 99) <= params.a_lat * 1.05


def test_project_and_window_follow_the_line():
    race, _, _ = _build()
    idx, dist = rl_mod.project(race, race.xy[10] + 0.05, None)
    assert dist < 0.2 and abs(idx - 10) <= 1
    ii = rl_mod.segment_indices(race, 10, 35.0)
    assert ii[0] == 10
    seg = np.sum(np.linalg.norm(np.diff(race.xy[ii], axis=0), axis=1))
    assert 34.0 <= seg <= 37.0


def test_degenerate_input_returns_none():
    assert rl_mod.build(np.zeros((2, 2)), np.zeros((2, 2))) is None
