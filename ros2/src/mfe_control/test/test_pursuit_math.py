"""Pure pursuit maths (mfe_control.pursuit_math): steering sign, lookahead selection, speed rules. No ROS."""
import math

import pytest

from mfe_control import pursuit_math as pm


def _straight(n=51, step=1.0):
    return [(i * step, 0.0) for i in range(n)]


class TestSteering:

    def test_target_on_the_left_steers_left(self):
        assert pm.pure_pursuit_steering(0.0, 0.0, 0.0, (5.0, 2.0), 1.56, math.radians(28.0)) > 0.0

    def test_target_on_the_right_steers_right(self):
        assert pm.pure_pursuit_steering(0.0, 0.0, 0.0, (5.0, -2.0), 1.56, math.radians(28.0)) < 0.0

    def test_target_dead_ahead_gives_no_steering(self):
        assert pm.pure_pursuit_steering(0.0, 0.0, 0.0, (5.0, 0.0), 1.56, math.radians(28.0)) == pytest.approx(0.0)

    def test_heading_is_taken_into_account(self):
        assert pm.pure_pursuit_steering(0.0, 0.0, math.pi / 2, (0.0, 5.0), 1.56, math.radians(28.0)) == \
            pytest.approx(0.0, abs=1e-9)

    def test_steering_is_normalised_and_clamped(self):
        far_left = pm.pure_pursuit_steering(0.0, 0.0, 0.0, (1.0, 1.0), 1.56, math.radians(28.0))
        assert far_left == pytest.approx(1.0)
        assert -1.0 <= pm.pure_pursuit_steering(0.0, 0.0, 0.0, (-5.0, -50.0), 1.56, 0.1) <= 1.0


class TestLookahead:

    def test_lookahead_is_the_first_waypoint_at_or_beyond_ld(self):
        point, closest = pm.find_lookahead(_straight(), 0, 0.0, 0.0, ld=5.0)
        assert point == (5.0, 0.0) and closest == 0

    def test_closest_index_follows_the_car_along_the_path(self):
        point, closest = pm.find_lookahead(_straight(), 0, 20.0, 0.0, ld=5.0)
        assert closest == 20 and point == (25.0, 0.0)

    def test_car_far_from_the_start_index_relocates_the_search(self):
        point, closest = pm.find_lookahead(_straight(), 0, 40.0, 0.0, ld=5.0, relocate_m=10.0)
        assert closest == 40 and point == (45.0, 0.0)

    def test_lookahead_past_the_end_returns_the_last_point(self):
        point, _ = pm.find_lookahead(_straight(), 0, 48.0, 0.0, ld=20.0)
        assert point == (50.0, 0.0)


class TestSpeed:

    def test_corner_speed_obeys_lateral_acceleration_and_the_cap(self):
        assert pm.corner_speed(0.0, 6.0, 8.0) == 8.0
        assert pm.corner_speed(1.0 / 9.0, 6.0, 8.0) == pytest.approx(math.sqrt(6.0 * 9.0))  # capped at 8 m/s
        assert pm.corner_speed(1.0 / 4.0, 6.0, 8.0) == pytest.approx(math.sqrt(6.0 * 4.0))

    def test_menger_curvature_of_a_circle(self):
        r = 2.0
        pts = [(r * math.cos(a), r * math.sin(a)) for a in (0.0, 0.2, 0.4)]
        assert pm.menger_curvature(*pts) == pytest.approx(1.0 / r, rel=1e-2)   # a discrete estimate, not exact
        assert pm.menger_curvature((0.0, 0.0), (0.001, 0.0), (1.0, 0.0)) is None

    def test_straight_path_keeps_top_speed_and_a_corner_ahead_slows_the_target(self):
        assert pm.curvature_speed_target(_straight(), 0, 30, 5.0, 8.0, 6.0, 3.0) == 8.0
        bend = [(float(i), 0.0) for i in range(20)] + [(19.0 + 3.0 * math.sin(a / 2), 3.0 * (1 - math.cos(a / 2)))
                                                        for a in [k * 0.2 for k in range(1, 16)]]
        assert pm.curvature_speed_target(bend, 0, 30, 8.0, 8.0, 2.0, 3.0) < 8.0

    def test_end_of_path_speed_cap(self):
        assert pm.path_end_speed_cap(remaining=10.0, decel=2.0, margin=0.0) == pytest.approx(math.sqrt(40.0))
        assert pm.path_end_speed_cap(remaining=1.0, decel=2.0, margin=2.0) == 0.0

    def test_pi_throttle_brake_accelerates_or_brakes(self):
        thr, brk, _ = pm.pi_throttle_brake(v_target=5.0, v_now=3.0, integral=0.0, kp=0.5, ki=0.1, kb=1.0,
                                           deadband=0.2, dt=0.05)
        assert thr > 0.0 and brk == 0.0
        thr, brk, _ = pm.pi_throttle_brake(v_target=3.0, v_now=6.0, integral=0.0, kp=0.5, ki=0.1, kb=1.0,
                                           deadband=0.2, dt=0.05)
        assert thr == 0.0 and brk > 0.0


class TestPathHelpers:

    def test_smoothing_keeps_length_and_endpoints(self):
        path = [(float(i), math.sin(i)) for i in range(20)]
        out = pm.smooth_path(path, 5)
        assert len(out) == len(path)
        assert out[0] == path[0] and out[-1] == path[-1]

    def test_remaining_length_of_a_straight_path(self):
        assert pm.remaining_length(_straight(), 10) == pytest.approx(40.0)


class TestPt2Anticipator:

    def test_zero_time_constant_is_a_pass_through(self):
        ant = pm.Pt2Anticipator(tau=0.0, zeta=0.7, filter_tau=0.0, dt=0.05)
        assert [round(ant(v), 9) for v in (0.1, 0.4, -0.2)] == [0.1, 0.4, -0.2]

    def test_output_is_clamped_to_the_steering_range(self):
        ant = pm.Pt2Anticipator(tau=0.1, zeta=0.7, filter_tau=0.0, dt=0.05)
        out = [ant(1.0 if k == 0 else 0.0) for k in range(5)]
        assert all(-1.0 <= u <= 1.0 for u in out)
