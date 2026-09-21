"""
Unit tests for finish_detector_node — skidpad phase machine and
acceleration dynamic gate logic.

Tests the pure-Python logic extracted from FinishDetectorNode:
  - Skidpad 2L+2R lap counting via angle accumulation
  - Acceleration orange-cone gate derivation
  - Return-to-start logic for autocross/trackdrive

No ROS2 runtime needed (conftest.py mocks all imports).
Branch: neil/feature/planning-improvements (finish detector rewrite)
"""

import math
import sys
import os
import types
import pytest

# conftest.py already installed ros mocks — but also add fs_msgs stub
if 'fs_msgs.msg' not in sys.modules:
    fs = types.ModuleType('fs_msgs.msg')
    class _CC:
        def __init__(self):
            self.throttle = 0.0; self.brake = 0.0; self.steering = 0.0
            self.header = type('H', (), {'stamp': None, 'frame_id': ''})()
    fs.ControlCommand = _CC
    sys.modules['fs_msgs'] = types.ModuleType('fs_msgs')
    sys.modules['fs_msgs.msg'] = fs

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


# ---------------------------------------------------------------------------
# Import the phase logic directly (avoid __init__ by testing helpers inline)
# ---------------------------------------------------------------------------

def _angle_delta(theta_now, theta_prev):
    """Wrap angle difference to (-pi, pi]."""
    d = theta_now - theta_prev
    if d >  math.pi: d -= 2 * math.pi
    if d < -math.pi: d += 2 * math.pi
    return d


def _simulate_circle(cx, cy, radius, n_laps, clockwise=False):
    """
    Generate (x, y) waypoints for n_laps around a circle.
    Returns list of (x, y) tuples.
    """
    sign = -1 if clockwise else 1
    points = []
    steps_per_lap = 100
    for i in range(n_laps * steps_per_lap + 1):
        theta = sign * 2 * math.pi * i / steps_per_lap
        points.append((cx + radius * math.cos(theta),
                       cy + radius * math.sin(theta)))
    return points


def _count_laps_from_points(points, cx, cy, expected_laps, clockwise=False):
    """
    Replicate the angle-accumulation lap counter from finish_detector_node.
    Returns laps_completed.
    """
    angle_prev  = None
    angle_accum = 0.0
    laps = 0

    for (x, y) in points:
        angle_now = math.atan2(y - cy, x - cx)
        if angle_prev is not None:
            delta = _angle_delta(angle_now, angle_prev)
            angle_accum += delta
        angle_prev = angle_now

        new_laps = int(abs(angle_accum) / (2 * math.pi))
        if new_laps > laps:
            laps = new_laps

    return laps


# ---------------------------------------------------------------------------
# Test group 1: angle-based lap counting
# ---------------------------------------------------------------------------

class TestSkidpadLapCounting:

    # Skidpad geometry (matches path_planner_node defaults)
    CX    = 14.4
    CY_L  =  9.3
    CY_R  = -9.3
    R     =  9.1

    def test_left_circle_2_laps_ccw(self):
        """2 CCW laps around left circle must count as 2 laps."""
        pts = _simulate_circle(self.CX, self.CY_L, self.R, n_laps=2, clockwise=False)
        laps = _count_laps_from_points(pts, self.CX, self.CY_L,
                                       expected_laps=2, clockwise=False)
        assert laps == 2, f"Expected 2 left laps, got {laps}"

    def test_right_circle_2_laps_cw(self):
        """2 CW laps around right circle must count as 2 laps."""
        pts = _simulate_circle(self.CX, self.CY_R, self.R, n_laps=2, clockwise=True)
        laps = _count_laps_from_points(pts, self.CX, self.CY_R,
                                       expected_laps=2, clockwise=True)
        assert laps == 2, f"Expected 2 right laps, got {laps}"

    def test_1_lap_counts_as_1(self):
        pts = _simulate_circle(self.CX, self.CY_L, self.R, n_laps=1, clockwise=False)
        laps = _count_laps_from_points(pts, self.CX, self.CY_L, 1)
        assert laps == 1

    def test_3_laps_counts_as_3(self):
        """Over-lapping (bug guard) — 3 laps still counts correctly."""
        pts = _simulate_circle(self.CX, self.CY_L, self.R, n_laps=3, clockwise=False)
        laps = _count_laps_from_points(pts, self.CX, self.CY_L, 3)
        assert laps == 3

    def test_no_false_positive_on_entry_straight(self):
        """
        Car drives straight from (0,0) toward the circle entry at (CX, 0).
        No laps should be counted (car hasn't entered the circle).
        """
        pts = [(x, 0.0) for x in range(0, int(self.CX))]
        laps = _count_laps_from_points(pts, self.CX, self.CY_L, 0)
        assert laps == 0, f"False positive on entry straight: {laps} laps counted"

    def test_partial_lap_not_counted(self):
        """Half a circle (180°) must not count as a full lap."""
        pts = _simulate_circle(self.CX, self.CY_L, self.R, n_laps=1, clockwise=False)
        half_pts = pts[:len(pts) // 2]
        laps = _count_laps_from_points(half_pts, self.CX, self.CY_L, 0)
        assert laps == 0, f"Partial lap counted as {laps}"


# ---------------------------------------------------------------------------
# Test group 2: acceleration orange-cone gate
# ---------------------------------------------------------------------------

class TestAccelerationGate:

    def _median_gate(self, xs):
        """Replicate the median-of-candidates logic from finish_detector."""
        import numpy as np
        return float(np.median(xs))

    def test_gate_from_two_cones(self):
        """Gate X should be the mean of cone X positions."""
        import numpy as np
        approach_x = 60.0
        cone_xs = [76.0, 80.0]   # 2 orange cones at finish
        candidates = [x for x in cone_xs if x > approach_x]
        assert len(candidates) == 2
        gate = self._median_gate(candidates)
        assert abs(gate - 78.0) < 0.1, f"Gate at wrong X: {gate}"

    def test_cones_before_approach_not_used(self):
        """Cones behind approach_x must not contribute to the gate."""
        approach_x = 60.0
        all_cone_xs = [10.0, 30.0, 50.0, 76.0, 80.0]
        candidates = [x for x in all_cone_xs if x > approach_x]
        assert candidates == [76.0, 80.0], \
            f"Wrong candidates selected: {candidates}"

    def test_fewer_than_2_cones_skipped(self):
        """Need at least 2 orange cones to set the gate."""
        approach_x = 60.0
        only_one = [78.0]
        candidates = [x for x in only_one if x > approach_x]
        # Should not update gate with <2 cones
        assert len(candidates) < 2

    def test_median_stable_with_outlier(self):
        """Median gate is robust to a single outlier cone."""
        import numpy as np
        xs = [75.0, 77.0, 79.0, 200.0]  # 200m is an outlier
        gate = self._median_gate(xs)
        assert 75.0 < gate < 100.0, f"Median unstable: {gate}"


# ---------------------------------------------------------------------------
# Test group 3: return-to-start (autocross/trackdrive)
# ---------------------------------------------------------------------------

class TestReturnToStart:

    def _simulate_return(self, track_radius=30.0, n_laps=1,
                         return_r=5.0, min_travel=60.0):
        """
        Simulate car driving around a circle and returning to start.
        Returns list of (dist_from_start, travel_m, max_dist) tuples.
        """
        import numpy as np
        n_steps = 200 * n_laps
        thetas = np.linspace(0, 2 * math.pi * n_laps, n_steps)
        xs = track_radius * np.cos(thetas)
        ys = track_radius * np.sin(thetas)
        xs[0] = track_radius   # start at 3 o'clock
        ys[0] = 0.0

        start_x, start_y = xs[0], ys[0]
        travel = 0.0
        max_dist = 0.0
        laps = 0
        events = []

        for i in range(1, len(xs)):
            dx = xs[i] - xs[i-1]
            dy = ys[i] - ys[i-1]
            travel += math.hypot(dx, dy)
            dist = math.hypot(xs[i] - start_x, ys[i] - start_y)
            max_dist = max(max_dist, dist)

            gate = (travel >= min_travel
                    and max_dist >= 15.0
                    and dist <= return_r)
            if gate:
                laps += 1
                max_dist = 0.0  # re-arm guard
                events.append(i)

        return laps

    def test_one_lap_counts_once(self):
        laps = self._simulate_return(n_laps=1)
        assert laps == 1, f"Expected 1 lap, got {laps}"

    def test_three_laps_count_three(self):
        laps = self._simulate_return(n_laps=3, min_travel=60.0)
        assert laps == 3, f"Expected 3 laps, got {laps}"

    def test_no_false_positive_below_min_travel(self):
        """Car starting at origin and moving a few metres — no lap."""
        import math
        # Small triangle — total travel well below 60m
        start_x, start_y = 30.0, 0.0
        travel = 0.0; max_dist = 0.0; laps = 0

        waypoints = [(30.0, 0.0), (35.0, 0.0), (30.0, 0.0)]
        for i in range(1, len(waypoints)):
            dx = waypoints[i][0] - waypoints[i-1][0]
            dy = waypoints[i][1] - waypoints[i-1][1]
            travel += math.hypot(dx, dy)
            dist = math.hypot(waypoints[i][0] - start_x, waypoints[i][1] - start_y)
            max_dist = max(max_dist, dist)
            if travel >= 60.0 and max_dist >= 15.0 and dist <= 5.0:
                laps += 1

        assert laps == 0, f"False positive: {laps} laps on short path"

    def test_figure8_midpoint_no_false_positive(self):
        """
        Car crosses near origin mid-figure-8 — the 15m excursion guard
        must prevent this from counting as a lap crossing.
        """
        # Car at start (30, 0), crosses near (30, 0) after only 5m excursion
        start_x, start_y = 30.0, 0.0
        travel = 100.0  # enough travel
        max_dist = 5.0  # NOT 15m — guard should block
        dist_from_start = 3.0  # within return_r=5m

        gate = (travel >= 60.0
                and max_dist >= 15.0   # <-- this blocks
                and dist_from_start <= 5.0)
        assert not gate, "15m excursion guard failed — false positive at figure-8 midpoint"
