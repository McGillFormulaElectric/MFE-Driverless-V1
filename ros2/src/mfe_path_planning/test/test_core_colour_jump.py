"""
Colour-lap helpers (core.colour: side votes, colour hint, pipeline latch) and the jump gate.

(core.jump_gate). No ROS.
"""
import numpy as np

from mfe_path_planning.core import cone_map as cm
from mfe_path_planning.core.colour import ColourHint, PipelineLatch, SideVotes
from mfe_path_planning.core.jump_gate import JumpGate

TYPES = cm.FSD_TYPES


def test_side_votes_fill_unknown_cells_and_keep_camera_colours():
    votes = SideVotes()
    votes.record(np.array([[1.0, 2.0], [3.0, -2.0], [5.0, 0.0]]), np.array([1, -1, 0]), swapped=False)
    assert len(votes) == 2
    buckets = [[] for _ in range(TYPES.n)]
    buckets[TYPES.left] = [[9.0, 9.0]]
    buckets[TYPES.unknown] = [[1.0, 2.0], [3.0, -2.0], [5.0, 0.0]]
    n_cam, n_fill, n_unk = votes.fill(buckets)
    assert (n_cam, n_fill, n_unk) == (1, 2, 1)
    assert buckets[TYPES.left] == [[9.0, 9.0], [1.0, 2.0]]
    assert buckets[TYPES.right] == [[3.0, -2.0]]
    assert buckets[TYPES.unknown] == [[5.0, 0.0]]


def test_swapped_walls_flip_the_vote_sign():
    votes = SideVotes()
    votes.record(np.array([[1.0, 1.0]]), np.array([1]), swapped=True)
    assert votes.votes[(1.0, 1.0)] == -1


def test_colour_hint_swaps_only_when_both_walls_vote_opposite():
    cones = cm.ConeMap(TYPES, {0: TYPES.left, 1: TYPES.right})
    cones.merge([(0.0, 3.0, 1), (2.0, 3.0, 1), (0.0, -3.0, 0), (2.0, -3.0, 0)], (0.0, 0.0), (1.0, 0.0))
    left = [(0.0, 3.0), (2.0, 3.0)]        # traced left wall is mostly YELLOW
    right = [(0.0, -3.0), (2.0, -3.0)]     # traced right wall is mostly BLUE
    hint = ColourHint()
    new_left, new_right, events = hint.sides(left, right, cones)
    assert hint.swapped and hint.counts['override'] == 1
    assert new_left == right and new_right == left
    assert events and events[0][0] == 'info'
    hint2 = ColourHint()
    l2, r2, _ = hint2.sides(right, left, cones)
    assert not hint2.swapped and hint2.counts['agree'] == 1 and l2 == right


def test_pipeline_latch_switches_on_two_coloured_each_and_reverts_after_absence():
    latch = PipelineLatch(min_coloured_each=2, revert_after=3)
    assert latch.trace
    latch.update(2, 2)
    assert not latch.trace and latch.switches == 1
    for _ in range(2):
        latch.update(0, 0)
    assert not latch.trace
    latch.update(0, 0)
    assert latch.trace and latch.switches == 2


def test_jump_gate_holds_a_jumping_candidate_then_accepts_it():
    gate = JumpGate(enabled=True, heading_deg=25.0, lateral_m=1.5, hold_max=2, min_path_m=5.0,
                    remaining_guard=False)
    base = np.column_stack([np.linspace(0.0, 30.0, 31), np.zeros(31)])
    out, _ = gate.apply(base, np.array([0.0, 0.0]), odom_received=False)
    assert np.array_equal(out, base) and gate.last == 'skip'     # no previous path yet
    out, _ = gate.apply(base, np.array([0.0, 0.0]), odom_received=False)
    assert np.array_equal(out, base) and gate.last == 'ok'
    jumped = base + np.array([0.0, 6.0])
    out, events = gate.apply(jumped, np.array([0.0, 0.0]), odom_received=False)
    assert gate.last == 'hold' and np.array_equal(out, base)
    assert events and 'rejected centreline' in events[0][1]
    gate.apply(jumped, np.array([0.0, 0.0]), odom_received=False)
    assert gate.last == 'hold'
    out, events = gate.apply(jumped, np.array([0.0, 0.0]), odom_received=False)
    assert gate.last == 'accept_after_hold' and np.array_equal(out, jumped)
    assert gate.accepted_after_hold == 1


def test_jump_gate_disabled_and_short_paths_pass_through():
    gate = JumpGate(enabled=False, heading_deg=25.0, lateral_m=1.5, hold_max=2, min_path_m=5.0,
                    remaining_guard=False)
    a = np.column_stack([np.linspace(0.0, 30.0, 31), np.zeros(31)])
    b = a + np.array([0.0, 6.0])
    gate.apply(a, np.array([0.0, 0.0]), False)
    out, _ = gate.apply(b, np.array([0.0, 0.0]), False)
    assert np.array_equal(out, b) and gate.last == 'skip'
