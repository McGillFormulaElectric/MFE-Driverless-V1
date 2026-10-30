"""
Skidpad: the hardcoded template path and the cone-based state machine (pure numpy, no ROS).

States: APPROACH -> LEFT_CIRCLE (lock after 1 lap, 2 laps) -> RIGHT_CIRCLE (same) -> EXIT. The yaw-rate sign picks
the circle; the caller supplies the cone walls and the car pose, and logs the (level, text) events returned.
"""
import math
from typing import Callable, Optional

import numpy as np

from mfe_common.geometry import arc_length
from mfe_path_planning.core.lap import LoopTrail, finish_loop
from mfe_path_planning.core.paths import midpoint_walk

YAW_HIST_LEN = 5
YAW_RATE_THRESH = 0.1      # rad/s sustained turn threshold
CIRCLE_LAP_DIST = 40.0     # lap closure: min distance driven (m) ...
CIRCLE_MAX_R = 10.0        # ... min excursion from the circle start (m) ...
CIRCLE_CLOSE_R = 4.0       # ... and back within this of the start


def make_skidpad_path() -> np.ndarray:
    """Hardcoded EUFS skidpad trajectory: 2 laps LEFT (counterclockwise), 2 laps RIGHT, then exit."""
    # Centres +-9.125, R 9.125: the circles touch at the crossing (was +-9.3 / 9.1, which clipped the inner cones).
    cx, cy_l, cy_r = 14.4, 9.125, -9.125
    R = 9.125
    N = 50    # waypoints per full lap

    n_entry = 40
    entry = np.column_stack([np.linspace(0, cx, n_entry), np.zeros(n_entry)])

    theta_l = np.linspace(-math.pi / 2, -math.pi / 2 + 4 * math.pi, N * 2 + 1)
    left_circ = np.column_stack([cx + R * np.cos(theta_l), cy_l + R * np.sin(theta_l)])

    theta_r = np.linspace(math.pi / 2, math.pi / 2 - 4 * math.pi, N * 2 + 1)
    right_circ = np.column_stack([cx + R * np.cos(theta_r), cy_r + R * np.sin(theta_r)])

    n_exit = 40
    exit_pts = np.column_stack([np.linspace(cx, 40.0, n_exit), np.zeros(n_exit)])

    return np.vstack([entry[:-1], left_circ, right_circ[1:], exit_pts[1:]])


def build_skidpad_circle(trail, blue, yellow, events: list) -> Optional[np.ndarray]:
    """Build the closed (first ~ last) path for one skidpad circle, from the driven trail and the cone walls."""
    if len(blue) < 4 or len(yellow) < 4 or len(trail) < 15:
        return None
    centerline = midpoint_walk(np.array(trail), blue, yellow)
    if len(centerline) < 8:
        return None
    return finish_loop(centerline, blue, yellow, min_n=15, label='Skidpad circle', events=events)


class SkidpadMachine:
    """
    Cone-based skidpad state machine.

    `walls` is a callable returning (blue, yellow) cone positions, used only when a circle locks.
    """

    def __init__(self, walls: Callable):
        self._walls = walls
        self.state = 'APPROACH'
        self.left: Optional[LoopTrail] = None
        self.right: Optional[LoopTrail] = None
        self.sign: Optional[int] = None      # sustained yaw-rate sign: +1 left (CCW), -1 right (CW), None
        self._yaw_hist: list = []

    def update_yaw_rate(self, yaw_rate: float) -> Optional[str]:
        """
        Sign of the sustained yaw rate over the last few frames (filters noise).

        Returns a debug text when the sign changes.
        """
        hist = self._yaw_hist
        hist.append(yaw_rate)
        if len(hist) > YAW_HIST_LEN:
            hist.pop(0)
        avg_yaw_rate = float(np.mean(hist))
        if abs(avg_yaw_rate) > YAW_RATE_THRESH:
            new_sign = 1 if avg_yaw_rate > 0 else -1
            if self.sign != new_sign:
                self.sign = new_sign
                circle_dir = 'LEFT (CCW)' if new_sign > 0 else 'RIGHT (CW)'
                return f'Skidpad: detected sustained turn {circle_dir}'
        return None

    def _active(self) -> Optional[LoopTrail]:
        return {'LEFT_CIRCLE': self.left, 'RIGHT_CIRCLE': self.right}.get(self.state)

    def advance(self, car_pos) -> None:
        """Feed the car position to the circle being driven (LEFT_CIRCLE or RIGHT_CIRCLE)."""
        if self.state in ('LEFT_CIRCLE', 'RIGHT_CIRCLE'):
            circle = self._active()
            if circle is not None:
                circle.advance(car_pos)

    def active_locked(self) -> Optional[np.ndarray]:
        """Locked path of the circle being driven, or None."""
        circle = self._active()
        return None if circle is None else circle.locked

    def _enter(self, left: bool, car_pos) -> None:
        circle = LoopTrail(start=car_pos.copy(), trail=[car_pos.copy()])
        if left:
            self.left = circle
            self.state = 'LEFT_CIRCLE'
        else:
            self.right = circle
            self.state = 'RIGHT_CIRCLE'

    def _lock(self, circle: LoopTrail, name: str, car_pos, events: list) -> None:
        blue, yellow = self._walls()
        locked = build_skidpad_circle(circle.trail, blue, yellow, events)
        if locked is None:
            return
        circle.locked = locked
        circle.laps = 1
        events.append(('info', f'Skidpad {name} lap 1 complete — locked circle: {len(locked)} waypoints, '
                               f'{arc_length(locked):.1f} m'))
        circle.restart(car_pos)

    def step(self, car_pos) -> list:
        """
        State transitions for one cycle at car_pos.

        Returns the (level, text) log events.
        """
        events: list = []
        sign = self.sign
        if self.state == 'APPROACH':
            if sign is not None:
                left = sign > 0
                events.append(('info', f'Skidpad: entering {"LEFT" if left else "RIGHT"}_CIRCLE'))
                self._enter(left, car_pos)
        elif self.state == 'LEFT_CIRCLE':
            circle = self.left
            if circle.closed(car_pos, CIRCLE_LAP_DIST, CIRCLE_MAX_R, CIRCLE_CLOSE_R):
                if circle.locked is None:
                    self._lock(circle, 'LEFT', car_pos, events)
                elif circle.laps == 1:
                    circle.laps = 2
                    events.append(('info', 'Skidpad LEFT lap 2 complete — transitioning to RIGHT_CIRCLE'))
                    if sign is not None and sign < 0:
                        self._enter(False, car_pos)
                        self._yaw_hist.clear()
                    else:
                        events.append(('warn', 'Skidpad: expected right turn to enter RIGHT_CIRCLE, '
                                               'still turning left'))
        elif self.state == 'RIGHT_CIRCLE':
            circle = self.right
            if circle.closed(car_pos, CIRCLE_LAP_DIST, CIRCLE_MAX_R, CIRCLE_CLOSE_R):
                if circle.locked is None:
                    self._lock(circle, 'RIGHT', car_pos, events)
                elif circle.laps == 1:
                    circle.laps = 2
                    events.append(('info', 'Skidpad RIGHT lap 2 complete — transitioning to EXIT'))
                    self.state = 'EXIT'
                    self._yaw_hist.clear()
        return events
