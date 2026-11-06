"""Camera colour voting: projection, detection matching, the colour decision, the pose history."""
import pytest

from mfe_validation.cam_fusion import (BLUE, ORANGE_BIG, ORANGE_SMALL, UNKNOWN, YELLOW,
                                       CamFusion, PoseHistory, decide)


def test_decide_needs_enough_votes():
    assert decide([1, 0, 0, 0, 0]) is None
    assert decide([2, 0, 0, 0, 0]) == BLUE


def test_decide_needs_agreement():
    assert decide([2, 2, 0, 0, 0]) is None
    assert decide([3, 2, 0, 0, 0]) == BLUE            # 3 of 5 votes = 0.6


def test_decide_orange_is_one_group_split_by_big_share():
    assert decide([0, 0, 3, 1, 0]) == ORANGE_BIG      # 3 of 4 orange votes are big
    assert decide([0, 0, 0, 4, 0]) == ORANGE_SMALL
    assert decide([0, 0, 1, 3, 0], big_share=0.3) == ORANGE_SMALL


def test_project_ahead_and_behind():
    fusion = CamFusion()
    u, v, rng = fusion.project((5.0, 0.0), (0.0, 0.0, 0.0))
    assert rng == pytest.approx(5.0)
    assert u == pytest.approx(fusion.cx, abs=1.0)
    assert fusion.project((-5.0, 0.0), (0.0, 0.0, 0.0)) is None
    assert fusion.project((1.0, 0.0), (0.0, 0.0, 0.0)) is None   # closer than min_range


def test_vote_matches_each_detection_to_its_landmark():
    fusion = CamFusion()
    pose = (0.0, 0.0, 0.0)
    landmarks = {0: (6.0, 2.0), 1: (6.0, -2.0)}
    u0, v0, _ = fusion.project(landmarks[0], pose)
    u1, v1, _ = fusion.project(landmarks[1], pose)
    detections = [(u0, v0, ORANGE_BIG), (u1, v1, YELLOW), (u1, v1, UNKNOWN)]
    assert sorted(fusion.vote(detections, landmarks, pose)) == [(0, ORANGE_BIG), (1, YELLOW)]


def test_pose_history_interpolates_and_refuses_long_gaps():
    h = PoseHistory()
    h.add(0.0, 0.0, 0.0, 0.0)
    h.add(0.4, 0.8, 0.0, 0.0)
    assert h.at(0.2) == pytest.approx((0.4, 0.0, 0.0))
    h.add(5.0, 9.0, 0.0, 0.0)
    assert h.at(3.0) is None                          # neighbours more than max_gap apart
