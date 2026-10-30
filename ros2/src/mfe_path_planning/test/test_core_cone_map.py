"""
Cone map (core.cone_map): colour merge rules, colour votes, expiry, eviction, visibility buckets.

No ROS.
"""
import numpy as np

from mfe_path_planning.core import cone_map as cm

BLUE, YELLOW, ORANGE_BIG, ORANGE_SMALL, UNKNOWN_COLOUR = 0, 1, 2, 3, 4
TYPES = cm.FSD_TYPES
COLOUR_TO_TYPE = {BLUE: TYPES.left, YELLOW: TYPES.right, ORANGE_BIG: TYPES.unknown,
                  ORANGE_SMALL: TYPES.unknown, UNKNOWN_COLOUR: TYPES.unknown}
SKIP = (ORANGE_BIG, ORANGE_SMALL)
ORIGIN = (0.0, 0.0)
FORWARD = (1.0, 0.0)


def _map(**kw):
    return cm.ConeMap(TYPES, COLOUR_TO_TYPE, skip_colours=SKIP, **kw)


def test_unknown_upgrades_to_colour_but_colour_is_not_replaced():
    cones = _map()
    cones.merge([(1.0, 1.0, UNKNOWN_COLOUR)], ORIGIN, FORWARD)
    cones.merge([(1.0, 1.0, BLUE)], ORIGIN, FORWARD)
    assert cones.get((1.0, 1.0)) == TYPES.left
    cones.merge([(1.0, 1.0, YELLOW)], ORIGIN, FORWARD)
    assert cones.get((1.0, 1.0)) == TYPES.left             # no recolour without the experiment flag
    assert cones.colour_flips == 0


def test_orange_and_non_finite_cones_are_skipped():
    cones = _map()
    cones.merge([(1.0, 1.0, ORANGE_BIG), (2.0, 2.0, ORANGE_SMALL), (np.nan, 0.0, BLUE), (3.0, np.inf, BLUE)],
                ORIGIN, FORWARD)
    assert len(cones) == 0


def test_grid_rounds_to_half_metre():
    cones = _map()
    cones.merge([(1.2, -0.7, YELLOW)], ORIGIN, FORWARD)
    assert cones.keys() == [(1.0, -0.5)]


def test_colour_vote_needs_votes_and_margin_before_a_colour():
    cones = _map(colour_vote=True, vote_min=3, vote_margin=2)
    cones.merge([(4.0, 0.0, BLUE)], ORIGIN, FORWARD)
    cones.merge([(4.0, 0.0, BLUE)], ORIGIN, FORWARD)
    assert cones.get((4.0, 0.0)) == TYPES.unknown            # two votes: below vote_min
    cones.merge([(4.0, 0.0, BLUE)], ORIGIN, FORWARD)
    assert cones.get((4.0, 0.0)) == TYPES.left
    cones.merge([(4.0, 0.0, YELLOW)], ORIGIN, FORWARD)
    assert cones.get((4.0, 0.0)) == TYPES.left               # 3 blue vs 1 yellow still leads by 2


def test_expiry_drops_sparse_cells_unseen_inside_range():
    cones = _map(cone_expiry=True, expiry_min_obs=3, expiry_cycles=2)
    cones.merge([(5.0, 0.0, BLUE)], ORIGIN, FORWARD)
    cones.merge([(-30.0, 30.0, BLUE)], ORIGIN, FORWARD)
    for _ in range(3):
        cones.merge([(-30.0, 30.0, BLUE)], ORIGIN, FORWARD)
    assert (5.0, 0.0) not in cones
    assert cones.cones_expired == 1


def test_evict_far_keeps_the_nearest_cones():
    cones = _map()
    cones.merge([(float(x), 0.0, BLUE) for x in range(10)], ORIGIN, FORWARD)
    cones.evict_far(4, np.array([0.0, 0.0]))
    assert len(cones) == 4
    assert all(k[0] <= 3.0 for k in cones.keys())


def test_visible_bucket_excludes_far_and_behind_cones():
    cones = _map()
    cones.merge([(10.0, 0.0, BLUE), (60.0, 0.0, BLUE), (-10.0, 0.0, YELLOW), (0.0, 5.0, BLUE)], ORIGIN, FORWARD)
    buckets = cones.bucket(True, np.array([0.0, 0.0]), np.array(FORWARD))
    assert buckets[TYPES.left] == [[10.0, 0.0], [0.0, 5.0]]
    assert buckets[TYPES.right] == []
    everything = cones.bucket(False, np.array([0.0, 0.0]), np.array(FORWARD))
    assert len(everything[TYPES.left]) == 3 and len(everything[TYPES.right]) == 1


def test_unknown_cones_take_the_side_of_the_heading_cross_product():
    cones = _map()
    cones.merge([(5.0, 2.0, UNKNOWN_COLOUR), (5.0, -2.0, UNKNOWN_COLOUR)], ORIGIN, FORWARD)
    buckets = cones.bucket(False, np.array([0.0, 0.0]), np.array(FORWARD))
    cones.assign_unknown_sides(np.array([0.0, 0.0]), np.array(FORWARD), buckets)
    assert buckets[TYPES.left] == [[5.0, 2.0]]
    assert buckets[TYPES.right] == [[5.0, -2.0]]
    assert buckets[TYPES.unknown] == []


def test_wall_colour_vote_majority_and_ties():
    cones = _map()
    cones.merge([(0.0, 3.0, BLUE), (2.0, 3.0, BLUE), (4.0, 3.0, YELLOW), (6.0, -3.0, YELLOW)], ORIGIN, FORWARD)
    assert cones.wall_colour_vote([(0.0, 3.0), (2.0, 3.0)]) == 1
    assert cones.wall_colour_vote([(4.0, 3.0), (6.0, -3.0)]) == -1
    assert cones.wall_colour_vote([(0.0, 3.0), (4.0, 3.0)]) == 0


def test_bucket_counts_and_walls():
    cones = _map()
    cones.merge([(0.0, 3.0, BLUE), (0.0, -3.0, YELLOW), (9.0, 9.0, UNKNOWN_COLOUR)], ORIGIN, FORWARD)
    blue, yellow = cones.walls()
    assert blue.tolist() == [[0.0, 3.0]] and yellow.tolist() == [[0.0, -3.0]]
    buckets = cones.bucket(False, np.array([0.0, 0.0]), np.array(FORWARD))
    assert cm.bucket_counts(buckets, TYPES) == (1, 1, 1)
