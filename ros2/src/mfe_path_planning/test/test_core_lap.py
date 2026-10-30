"""
Lap-return reset, lap-return ban, loop trails and the lock-in builder (core.lap).

No ROS.
"""
import numpy as np

from mfe_path_planning.core import lap


def test_lap_return_banned_radius_and_none():
    keys = [(0.0, 0.0), (10.0, 0.0), (30.0, 0.0)]
    assert lap.lap_return_banned(keys, (0.0, 0.0), 15.0) == {(0.0, 0.0), (10.0, 0.0)}
    assert lap.lap_return_banned(keys, (0.0, 0.0), None) == set(keys)


def test_lap_passed_only_behind_the_car():
    banned = {(-5.0, 0.0), (5.0, 0.0)}
    passed = lap.lap_passed(banned, car_xy=(0.0, 0.0), car_dir=(1.0, 0.0))
    assert passed == {(-5.0, 0.0)}


def test_lap_reobserved_within_tolerance():
    banned = {(0.0, 0.0), (10.0, 0.0)}
    det = np.array([[0.5, 0.0]])
    assert lap.lap_reobserved(banned, det, tol_m=0.75) == {(0.0, 0.0)}
    assert lap.lap_reobserved(set(), det) == set()


def test_lap_return_reset_fires_once_on_return_after_min_travel():
    reset = lap.LapReturnReset(start_r_m=15.0, min_travel_m=100.0, len_frac=0.6)
    path = [(0.0, 0.0)] + [(float(x), 0.0) for x in range(1, 51)]          # out to 50 m
    path += [(50.0 - float(x), 0.0) for x in range(1, 51)]                 # back to the start
    fired = [reset.update(p) for p in path]
    assert sum(fired) == 0                                                  # travel 100 m is not above min
    reset2 = lap.LapReturnReset(start_r_m=15.0, min_travel_m=60.0)
    out = [(float(x), 0.0) for x in range(0, 61)] + [(float(60 - x), 0.0) for x in range(1, 61)]
    fires = [reset2.update(p) for p in out]
    assert sum(fires) == 1 and reset2.fires == 1
    # the lap is measured from the departure (x = 31 m, beyond 2 * start_r) to the return (x = 14 m): 29 + 46 m
    assert reset2.lap_len is not None and abs(reset2.lap_len - 75.0) < 1e-6


def test_lap_ban_fire_lift_and_visible():
    ban = lap.LapBan()
    n = ban.fire([(0.0, 0.0), (20.0, 0.0), (-2.0, 0.0)], start_xy=(0.0, 0.0), radius_m=5.0)
    assert n == 2 and ban.banned == {(0.0, 0.0), (-2.0, 0.0)}
    assert ban.visible([(0.0, 0.0), (20.0, 0.0)]) == [(20.0, 0.0)]
    hit = ban.lift(car_xy=(0.0, 0.0), car_dir=(1.0, 0.0))             # (-2, 0) is behind the car by 2 m
    assert hit == 1 and ban.reobs == 1 and ban.banned == {(0.0, 0.0)}


def test_loop_trail_closes_only_after_distance_excursion_and_return():
    trail = lap.LoopTrail()
    for x in np.arange(0.0, 45.0, 0.5):
        trail.advance(np.array([x, 0.0]))
    assert trail.dist > 40.0 and trail.max_r > 10.0
    assert trail.closed(np.array([1.0, 0.0]), 40.0, 10.0, 4.0)
    assert not trail.closed(np.array([30.0, 0.0]), 40.0, 10.0, 4.0)
    trail.restart(np.array([1.0, 0.0]))
    assert trail.dist == 0.0 and trail.max_r == 0.0 and trail.trail[0].tolist() == [1.0, 0.0]
    assert trail.start.tolist() == [0.0, 0.0]                          # the loop start is kept across laps


def test_locked_loop_needs_a_thick_enough_map():
    events = []
    out = lap.build_locked_loop(np.zeros((30, 2)), np.zeros((2, 2)), np.zeros((2, 2)), events)
    assert out is None
    assert events and events[0][0] == 'warn' and 'too thin' in events[0][1]
