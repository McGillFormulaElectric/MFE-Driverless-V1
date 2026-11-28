"""
Unit tests for pure-Python mfe_path_planning.core and eval subcomponents.

Covers ConeMap, JumpGate, ColourHint, SideVotes, PipelineLatch, LapReturnReset,
LapBan, AccelMidline, SkidpadMachine, RaceLineRunner, and MetricsAccumulator.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from mfe_path_planning.core.accel import (
    AccelMidline,
    accel_midline_fit_reason,
    accel_pair_midpoints,
)
from mfe_path_planning.core.colour import ColourHint, PipelineLatch, SideVotes
from mfe_path_planning.core.cone_map import ConeMap, FSD_TYPES
from mfe_path_planning.core.jump_gate import JumpGate
from mfe_path_planning.core.lap import LapBan, LapReturnReset
from mfe_path_planning.core.raceline_runner import RaceLineRunner, raceline_gate
from mfe_path_planning.core.skidpad import SkidpadMachine, make_skidpad_path
from mfe_path_planning.eval.metrics import MetricsAccumulator
from mfe_path_planning.eval.replay import (
    CONE_BLUE,
    CONE_ORANGE_BIG,
    CONE_ORANGE_SMALL,
    CONE_UNKNOWN,
    CONE_YELLOW,
    MFE_TO_FSD,
)
from mfe_path_planning.eval.tracks import (
    CenterlineData,
    load_centerline_csv,
    save_centerline_csv,
)


def _straight_path(x0: float, y0: float, length: float, yaw: float = 0.0, n: int = 31):
    """Generate an N x 2 straight path starting at (x0, y0) along yaw."""
    s = np.linspace(0.0, length, n)
    return np.column_stack([x0 + s * math.cos(yaw), y0 + s * math.sin(yaw)])


class TestConeMap:
    """Tests for ConeMap grid merging, voting, expiry, eviction, and visibility."""

    def _make_map(self, **kwargs):
        return ConeMap(
            FSD_TYPES,
            MFE_TO_FSD,
            skip_colours=(CONE_ORANGE_BIG, CONE_ORANGE_SMALL),
            **kwargs,
        )

    def test_merge_snaps_skips_orange_and_ignores_nan(self):
        cmap = self._make_map()
        pos = np.array([0.0, 0.0])
        heading = np.array([1.0, 0.0])
        cmap.merge(
            [
                (5.1, 1.9, CONE_UNKNOWN),
                (5.05, 2.05, CONE_BLUE),      # snaps to same (5.0, 2.0) -> upgrades UNKNOWN
                (6.0, 2.0, CONE_ORANGE_BIG),  # skipped
                (float('nan'), 1.0, CONE_YELLOW),
                (1.0, float('inf'), CONE_BLUE),
            ],
            pos,
            heading,
        )
        assert len(cmap) == 1
        assert cmap.get((5.0, 2.0)) == FSD_TYPES.left

    def test_colour_vote_requires_min_and_margin(self):
        cmap = self._make_map(colour_vote=True, vote_min=3, vote_margin=2)
        pos, heading = np.array([0.0, 0.0]), np.array([1.0, 0.0])
        cmap.merge([(4.0, 1.5, CONE_BLUE)], pos, heading)
        cmap.merge([(4.0, 1.5, CONE_BLUE)], pos, heading)
        assert cmap.get((4.0, 1.5)) == FSD_TYPES.unknown  # only 2 votes
        cmap.merge([(4.0, 1.5, CONE_YELLOW)], pos, heading)
        assert cmap.get((4.0, 1.5)) == FSD_TYPES.unknown  # margin is 1 < 2
        cmap.merge([(4.0, 1.5, CONE_BLUE)], pos, heading)
        assert cmap.get((4.0, 1.5)) == FSD_TYPES.left     # 3 vs 1 -> margin 2

    def test_cone_expiry_drops_only_sparse_in_range_cells(self):
        cmap = self._make_map(cone_expiry=True, expiry_min_obs=3, expiry_cycles=4)
        pos, heading = np.array([0.0, 0.0]), np.array([1.0, 0.0])
        # Sparse cell ahead (seen once) vs persistent cell ahead (seen 3 times) vs behind car
        cmap.merge([(10.0, 1.5, CONE_BLUE), (12.0, -1.5, CONE_YELLOW), (-10.0, 1.5, CONE_BLUE)],
                   pos, heading)
        for _ in range(2):
            cmap.merge([(12.0, -1.5, CONE_YELLOW)], pos, heading)
        # Advance enough empty cycles for sparse in-range cell to expire
        for _ in range(4):
            cmap.merge([], pos, heading)
        assert (10.0, 1.5) not in cmap       # sparse + in-range -> expired
        assert (12.0, -1.5) in cmap          # >= min_obs -> never expires
        assert (-10.0, 1.5) in cmap          # behind car (out of range) -> kept
        assert cmap.cones_expired == 1

    def test_evict_far_keeps_nearest_cells(self):
        cmap = self._make_map()
        pos, heading = np.array([0.0, 0.0]), np.array([1.0, 0.0])
        cmap.merge([(float(d), 1.0, CONE_BLUE) for d in (2, 5, 10, 30, 50)], pos, heading)
        cmap.evict_far(3, pos)
        assert len(cmap) == 3
        assert (2.0, 1.0) in cmap and (5.0, 1.0) in cmap and (10.0, 1.0) in cmap
        assert (50.0, 1.0) not in cmap


class TestJumpGate:
    """Tests for JumpGate lateral/heading rejection, hold cap, and remaining guard."""

    def test_accepts_smooth_and_holds_heading_jump_until_cap(self):
        gate = JumpGate(
            enabled=True,
            heading_deg=20.0,
            lateral_m=1.5,
            hold_max=2,
            min_path_m=10.0,
            remaining_guard=True,
        )
        car = np.array([0.0, 0.0])
        p0 = _straight_path(0.0, 0.0, 20.0, yaw=0.0)
        out0, _ = gate.apply(p0, car, True)
        assert np.allclose(out0, p0)

        # Sudden 45 deg jump -> held twice, then accepted on 3rd cycle
        p_jump = _straight_path(0.0, 0.0, 20.0, yaw=math.radians(45.0))
        out1, _ = gate.apply(p_jump, car, True)
        assert gate.last == 'hold'
        assert np.allclose(out1, p0)

        out2, _ = gate.apply(p_jump, car, True)
        assert gate.last == 'hold'
        assert np.allclose(out2, p0)

        out3, _ = gate.apply(p_jump, car, True)
        assert gate.last == 'accept_after_hold'
        assert np.allclose(out3, p_jump)
        assert gate.rejects == 2 and gate.accepted_after_hold == 1

    def test_remaining_guard_does_not_hold_shorter_stale_path(self):
        gate = JumpGate(
            enabled=True,
            heading_deg=20.0,
            lateral_m=1.5,
            hold_max=5,
            min_path_m=10.0,
            remaining_guard=True,
        )
        p_short = _straight_path(0.0, 0.0, 12.0, yaw=0.0)
        gate.apply(p_short, np.array([0.0, 0.0]), True)

        # Car moved to x=8 m -> only 4 m left on p_short, vs 20 m on p_fresh
        p_fresh = _straight_path(8.0, 0.0, 20.0, yaw=math.radians(40.0))
        out, _ = gate.apply(p_fresh, np.array([8.0, 0.0]), True)
        assert gate.last == 'accept_after_hold'
        assert np.allclose(out, p_fresh)


class TestColourHelpers:
    """Tests for SideVotes, ColourHint, and PipelineLatch."""

    def test_side_votes_records_swapped_and_fills_unknowns(self):
        sv = SideVotes()
        xy = np.array([[5.0, 2.0], [5.0, -2.0]])
        # Raw tracer labels (+1 left, -1 right), but colour_hint swapped them
        sv.record(xy, np.array([1, -1]), swapped=True)
        buckets = [[] for _ in range(FSD_TYPES.n)]
        buckets[FSD_TYPES.unknown] = [[5.0, 2.0], [5.0, -2.0], [9.0, 0.0]]
        n_cam, n_fill, n_unk = sv.fill(buckets)
        assert (n_cam, n_fill, n_unk) == (0, 2, 1)
        assert buckets[FSD_TYPES.right] == [[5.0, 2.0]]
        assert buckets[FSD_TYPES.left] == [[5.0, -2.0]]

    def test_colour_hint_swaps_when_both_walls_vote_opposite(self):
        cmap = ConeMap(FSD_TYPES, MFE_TO_FSD)
        pos, heading = np.array([0.0, 0.0]), np.array([1.0, 0.0])
        cmap.merge(
            [(4.0, 2.0, CONE_YELLOW), (8.0, 2.0, CONE_YELLOW),
             (4.0, -2.0, CONE_BLUE), (8.0, -2.0, CONE_BLUE)],
            pos,
            heading,
        )
        hint = ColourHint()
        left = np.array([[4.0, 2.0], [8.0, 2.0]])
        right = np.array([[4.0, -2.0], [8.0, -2.0]])
        new_l, new_r, _ = hint.sides(left, right, cmap)
        assert hint.swapped is True
        assert np.allclose(new_l, right) and np.allclose(new_r, left)

    def test_pipeline_latch_hysteresis(self):
        latch = PipelineLatch(min_coloured_each=2, revert_after=3)
        assert latch.trace is True
        latch.update(1, 2)
        assert latch.trace is True
        latch.update(2, 2)
        assert latch.trace is False
        for _ in range(2):
            latch.update(0, 0)
            assert latch.trace is False
        latch.update(0, 0)
        assert latch.trace is True
        assert latch.switches == 2


class TestLapResetAndBan:
    """Tests for LapReturnReset trigger and LapBan unbanning."""

    def test_lap_return_fires_after_excursion_and_min_travel(self):
        reset = LapReturnReset(start_r_m=5.0, min_travel_m=50.0, len_frac=0.6)
        # Circle of radius 20 m centred at (0, 20) starting at (0, 0) -> circumference ~125.7 m
        fired_steps = []
        for i in range(126):
            th = -math.pi / 2 + 2.0 * math.pi * i / 120.0
            pos = np.array([20.0 * math.cos(th), 20.0 + 20.0 * math.sin(th)])
            if reset.update(pos):
                fired_steps.append(i)
        assert len(fired_steps) == 1
        assert reset.fires == 1

    def test_lap_ban_fire_and_lift_on_reobs_and_pass(self):
        ban = LapBan()
        keys = [(0.0, 1.5), (5.0, 1.5), (40.0, 1.5)]
        n_banned = ban.fire(keys, np.array([0.0, 0.0]), radius_m=10.0)
        assert n_banned == 2
        assert (40.0, 1.5) not in ban.banned

        # Car at (3.0, 0.0) facing +X: (0.0, 1.5) is >1 m behind (passed),
        # and detection at (5.1, 1.4) re-observes (5.0, 1.5).
        lifted = ban.lift(
            np.array([3.0, 0.0]),
            np.array([1.0, 0.0]),
            det=np.array([[5.1, 1.4]]),
        )
        assert lifted == 2
        assert len(ban.banned) == 0


class TestAccelAndSkidpad:
    """Tests for AccelMidline fitting and SkidpadMachine transitions."""

    def test_accel_midline_fits_clean_straight(self):
        xs = np.arange(0.0, 40.0, 5.0)
        blue = np.column_stack([xs, np.full_like(xs, 1.5)])
        yellow = np.column_stack([xs, np.full_like(xs, -1.5)])
        cones = np.vstack([blue, yellow])
        mids = accel_pair_midpoints(cones, np.array([0.0, 0.0]), np.array([1.0, 0.0]))
        fit, reason = accel_midline_fit_reason(mids, np.array([1.0, 0.0]))
        assert fit is not None and reason == 'ok'
        theta, offset = fit
        assert abs(theta) < 1e-6 and abs(offset) < 1e-6

        mid = AccelMidline(alpha=0.3)
        base = _straight_path(0.0, 0.2, 30.0)
        out, (accepted, _) = mid.path(base, cones, np.array([0.0, 0.2]), np.array([1.0, 0.0]))
        assert accepted is True
        assert np.max(np.abs(out[:, 1])) < 1e-6

    def test_skidpad_hardcoded_path_is_valid(self):
        pts = make_skidpad_path()
        assert len(pts) > 150
        assert np.all(np.isfinite(pts))
        assert np.allclose(pts[0], [0.0, 0.0])
        assert pts[-1, 0] > 35.0 and abs(pts[-1, 1]) < 1e-6

    def test_skidpad_machine_enters_left_circle_on_positive_yaw_rate(self):
        def _empty_walls():
            return np.zeros((0, 2)), np.zeros((0, 2))

        sm = SkidpadMachine(_empty_walls)
        assert sm.state == 'APPROACH'
        for _ in range(5):
            sm.update_yaw_rate(0.4)
        sm.step(np.array([14.4, 0.0]))
        assert sm.state == 'LEFT_CIRCLE'


class TestRaceLineGateAndMetrics:
    """Tests for RaceLineRunner mission gate and MetricsAccumulator accuracy math."""

    @pytest.mark.parametrize('mission,expected', [
        ('autocross', True),
        ('trackdrive', True),
        ('peanut', True),
        ('acceleration', False),
        ('skidpad', False),
    ])
    def test_raceline_gate_missions(self, mission, expected):
        assert raceline_gate(mission, True) is expected
        assert raceline_gate(mission, False) is False

    def test_raceline_runner_disabled_is_noop(self):
        rl = RaceLineRunner(False, 1, 0.25, 6.0, 8.0, 35.0)
        win, events = rl.cycle(np.array([0.0, 0.0]), 2, lambda: (np.zeros((0, 2)), np.zeros(0)))
        assert win is None and events == []

    def test_metrics_accumulator_exact_cte_and_oob(self, tmp_path):
        s = np.linspace(0.0, 40.0, 81)
        xy = np.column_stack([s, np.zeros_like(s)])
        cl = CenterlineData(
            s=s,
            xy=xy,
            left_width=np.full_like(s, 1.5),
            right_width=np.full_like(s, 1.5),
            curvature=np.zeros_like(s),
            closed=False,
        )
        # Round-trip save/load check
        csv_p = tmp_path / 'cl.csv'
        save_centerline_csv(csv_p, cl)
        cl2 = load_centerline_csv(csv_p)
        assert cl2.closed is False
        assert np.allclose(cl2.xy, cl.xy, atol=1e-4)

        walls = np.vstack([
            np.column_stack([np.arange(0.0, 45.0, 5.0), np.full(9, 1.5)]),
            np.column_stack([np.arange(0.0, 45.0, 5.0), np.full(9, -1.5)]),
        ])
        acc = MetricsAccumulator(cl2, walls, eval_horizon_m=15.0)

        # Cycle 1: exact GT centerline -> CTE = 0, OOB = 0, min clearance ~1.5 m
        acc.add_cycle(np.array([0.0, 0.0]), np.array([1.0, 0.0]), _straight_path(0.0, 0.0, 20.0),
                      cycle_ms=2.0, group='exact')
        # Cycle 2: parallel shifted by +0.5 m -> CTE = 0.5 m, OOB = 0, clearance ~1.0 m
        acc.add_cycle(np.array([0.0, 0.0]), np.array([1.0, 0.0]), _straight_path(0.0, 0.5, 20.0),
                      cycle_ms=3.0, group='shifted')
        # Cycle 3: outside wall at y = +2.0 m -> OOB = 1.0
        acc.add_cycle(np.array([0.0, 0.0]), np.array([1.0, 0.0]), _straight_path(0.0, 2.0, 20.0),
                      cycle_ms=4.0, group='oob')

        summary = acc.summarize()
        assert summary['groups']['exact']['cte_max_m'] == pytest.approx(0.0, abs=1e-6)
        assert summary['groups']['exact']['out_of_bounds_frac'] == 0.0
        assert summary['groups']['exact']['min_cone_clearance_m'] == pytest.approx(1.5, abs=0.05)

        assert summary['groups']['shifted']['cte_mean_m'] == pytest.approx(0.5, abs=1e-4)
        assert summary['groups']['shifted']['cte_p95_m'] == pytest.approx(0.5, abs=1e-4)
        assert summary['groups']['shifted']['out_of_bounds_frac'] == 0.0

        assert summary['groups']['oob']['out_of_bounds_frac'] == pytest.approx(1.0, abs=1e-6)
