"""Supervisor cone oriented-bounding-box hit test and GPS distance (mfe_control.supervisor_logic). No ROS."""
import math

import numpy as np

from mfe_control.supervisor_logic import haversine_m, obb_hits

HALF_L, HALF_W = 1.5, 0.8


def _hit(cones, yaw=0.0):
    cone_xy = np.asarray(cones, dtype=np.float32).reshape(-1, 2)
    if len(cone_xy) == 0:
        return False
    inside, _, _ = obb_hits(cone_xy, 0.0, 0.0, yaw, HALF_L, HALF_W)
    return bool(inside.any())


class TestNoHit:

    def test_no_cones_no_hit(self):
        assert not _hit([])

    def test_cone_far_ahead_no_hit(self):
        assert not _hit([(10.0, 0.0)])

    def test_cone_far_to_side_no_hit(self):
        assert not _hit([(0.0, 5.0)])

    def test_cone_at_safe_lateral_clearance_no_hit(self):
        assert not _hit([(0.0, 1.0)])

    def test_cone_just_outside_front_no_hit(self):
        assert not _hit([(1.6, 0.0)])

    def test_cone_diagonal_outside_box_no_hit(self):
        assert not _hit([(2.0, 1.0)])


class TestHit:

    def test_cone_at_car_centre_hit(self):
        assert _hit([(0.0, 0.0)])

    def test_cone_just_inside_front_hit(self):
        assert _hit([(1.4, 0.0)])

    def test_cone_just_inside_side_hit(self):
        assert _hit([(0.0, 0.7)])

    def test_rear_cone_inside_box_hit(self):
        assert _hit([(-1.0, 0.0)])

    def test_side_swipe_hit(self):
        assert _hit([(0.0, 0.5)])


class TestYawRotation:

    def test_cone_ahead_after_90deg_turn_is_outside(self):
        assert not _hit([(0.0, 2.0)], yaw=math.pi / 2)

    def test_cone_beside_after_90deg_turn_hits(self):
        assert _hit([(0.5, 0.0)], yaw=math.pi / 2)

    def test_multiple_cones_one_hit(self):
        assert _hit([(10.0, 0.0), (0.0, 5.0), (0.0, 0.3)])

    def test_all_cones_outside_no_hit(self):
        assert not _hit([(5.0, 0.0), (-5.0, 0.0), (0.0, 3.0), (0.0, -3.0)])

    def test_car_centre_is_a_hit_at_every_yaw(self):
        for yaw in np.linspace(0, 2 * math.pi, 36):
            assert _hit([(0.0, 0.0)], yaw=yaw), f'missed the car centre at yaw={math.degrees(yaw):.1f} deg'


def test_local_frame_is_forward_and_left():
    inside, lx, ly = obb_hits(np.array([[0.0, 2.0]], dtype=np.float32), 0.0, 0.0, math.pi / 2, HALF_L, HALF_W)
    assert abs(lx[0] - 2.0) < 1e-6 and abs(ly[0]) < 1e-6
    inside, lx, ly = obb_hits(np.array([[0.0, 1.0]], dtype=np.float32), 0.0, 0.0, 0.0, HALF_L, HALF_W)
    assert abs(lx[0]) < 1e-6 and abs(ly[0] - 1.0) < 1e-6


def test_haversine_distance():
    assert haversine_m(0.0, 0.0, 0.0, 0.0) == 0.0
    one_degree_lat = haversine_m(45.0, 7.0, 46.0, 7.0)
    assert abs(one_degree_lat - 111_195.0) < 200.0
    assert abs(haversine_m(0.0, 0.0, 0.0, 1.0) - 111_195.0) < 200.0
