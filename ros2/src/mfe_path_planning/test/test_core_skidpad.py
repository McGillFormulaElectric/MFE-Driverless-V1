"""
Skidpad: the hardcoded template path and the cone-based state machine (core.skidpad).

No ROS.
"""
import math

import numpy as np

from mfe_path_planning.core import skidpad


def test_template_path_has_two_left_two_right_laps_and_exit():
    path = skidpad.make_skidpad_path()
    assert path.shape[1] == 2 and len(path) > 200
    assert np.allclose(path[0], [0.0, 0.0])
    assert abs(path[-1, 0] - 40.0) < 1e-6 and abs(path[-1, 1]) < 1e-6
    assert path[:, 1].max() > 15.0 and path[:, 1].min() < -15.0    # both circles are driven


def test_yaw_rate_sign_needs_a_sustained_turn():
    machine = skidpad.SkidpadMachine(walls=lambda: (np.zeros((0, 2)), np.zeros((0, 2))))
    assert machine.update_yaw_rate(0.0) is None and machine.sign is None
    msg = None
    for _ in range(6):
        msg = machine.update_yaw_rate(0.5) or msg
    assert machine.sign == 1 and msg == 'Skidpad: detected sustained turn LEFT (CCW)'
    for _ in range(10):
        machine.update_yaw_rate(-0.5)
    assert machine.sign == -1


def _walls_around(centre, r_in=6.0, r_out=12.0, n=90):
    t = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    blue = np.column_stack([centre[0] + r_out * np.cos(t), centre[1] + r_out * np.sin(t)])
    yellow = np.column_stack([centre[0] + r_in * np.cos(t), centre[1] + r_in * np.sin(t)])
    return blue, yellow


def test_left_circle_locks_after_one_lap_and_flags_missing_right_turn_after_two():
    centre = np.array([0.0, 0.0])
    walls = _walls_around(centre)
    machine = skidpad.SkidpadMachine(walls=lambda: walls)
    for _ in range(6):
        machine.update_yaw_rate(0.5)
    machine.advance(np.array([9.0, 0.0]))
    events = machine.step(np.array([9.0, 0.0]))
    assert machine.state == 'LEFT_CIRCLE' and any('entering LEFT_CIRCLE' in e[1] for e in events)
    angles = np.linspace(0.0, 4.0 * math.pi, 400)          # two laps counter-clockwise, radius 9 m
    locked_seen = False
    lap2_events = []
    for a in angles:
        pos = np.array([9.0 * math.cos(a), 9.0 * math.sin(a)])
        machine.advance(pos)
        ev = machine.step(pos)
        if machine.active_locked() is not None:
            locked_seen = True
        lap2_events += [e for e in ev if 'expected right turn' in e[1] or 'lap 2' in e[1]]
    assert locked_seen
    assert any('expected right turn' in e[1] for e in lap2_events)
    assert machine.state == 'LEFT_CIRCLE'                  # no right turn sensed, so it stays on the left circle


def test_build_circle_refuses_thin_maps():
    events = []
    assert skidpad.build_skidpad_circle(np.zeros((30, 2)), np.zeros((2, 2)), np.zeros((2, 2)), events) is None
