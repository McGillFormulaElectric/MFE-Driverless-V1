"""
Finish detector pure logic (core.finish): the orange timing line, the lap crossing test, the.

orange-gate clustering and pass counting, the LiDAR forward zone and the point-cloud read. Calls the real
functions; no ROS.
"""
import struct

import numpy as np
import pytest

from mfe_path_planning.core import finish as fin


def _pylon_pair(start_xy=(0.0, 0.0), yaw=0.0, lat=2.0, long=10.0):
    """Left and right pylons ahead of the start: a gate across the start heading at `long` metres ahead."""
    c, s = np.cos(yaw), np.sin(yaw)

    def world(long_m, lat_m):
        return (start_xy[0] + c * long_m - s * lat_m, start_xy[1] + s * long_m + c * lat_m)
    return [world(long, lat), world(long, -lat)]


class TestTimingLine:

    def test_pair_gives_a_line_across_the_heading_with_the_pylon_separation(self):
        line = fin.orange_timing_line(_pylon_pair(), 0.0, 0.0, 0.0, search_r=25.0, margin_m=2.0)
        long, lat, width = line
        assert long == pytest.approx(10.0, abs=1e-6)
        assert lat == pytest.approx(0.0, abs=1e-6)
        assert width == pytest.approx(4.0 + 2.0 * 2.0, abs=1e-6)

    def test_pylons_on_one_side_give_no_line(self):
        pts = [(10.0, 2.0), (12.0, 2.0)]
        assert fin.orange_timing_line(pts, 0.0, 0.0, 0.0, search_r=25.0, margin_m=2.0) is None

    def test_pylons_outside_the_search_radius_are_ignored(self):
        assert fin.orange_timing_line(_pylon_pair(long=40.0), 0.0, 0.0, 0.0, 25.0, 2.0) is None

    def test_line_follows_the_start_heading(self):
        yaw = np.pi / 2
        line = fin.orange_timing_line(_pylon_pair(yaw=yaw), 0.0, 0.0, yaw, search_r=25.0, margin_m=2.0)
        assert line is not None
        assert line[0] == pytest.approx(10.0, abs=1e-6)


class TestLapCrossing:

    LINE = (10.0, 0.0, 4.0)     # long = 10 m, lat = 0, width 4 m

    def test_crossing_from_behind_to_ahead_inside_the_width_counts(self):
        assert fin.line_crossed(prev_long=9.5, long=10.5, lat=0.5, line=self.LINE, heading_ok=True)

    def test_wrong_heading_never_counts(self):
        assert not fin.line_crossed(9.5, 10.5, 0.5, self.LINE, heading_ok=False)

    def test_outside_the_line_width_does_not_count(self):
        assert not fin.line_crossed(9.5, 10.5, 2.5, self.LINE, heading_ok=True)

    def test_no_previous_position_or_no_line_does_not_count(self):
        assert not fin.line_crossed(None, 10.5, 0.0, self.LINE, True)
        assert not fin.line_crossed(9.5, 10.5, 0.0, None, True)

    def test_stop_travel_after_line_is_clamped_to_the_run_out_limit(self):
        assert fin.stop_travel_after_line(100.0, 5.0) == pytest.approx(105.0)
        assert fin.stop_travel_after_line(100.0, -3.0) == pytest.approx(100.0)
        assert fin.stop_travel_after_line(100.0, 500.0) == pytest.approx(100.0 + fin.STOP_AFTER_LINE_MAX_M)


class TestOrangeGates:

    def test_pylons_cluster_into_gates_by_single_linkage(self):
        pylons = [np.array([0.0, 0.0]), np.array([3.0, 0.0]), np.array([30.0, 0.0]), np.array([33.0, 0.0])]
        states = fin.rebuild_gate_states(pylons, [])
        assert len(states) == 2
        centres = sorted(s['c'][0] for s in states)
        assert centres == pytest.approx([1.5, 31.5])
        assert all(s['inside'] is None for s in states)

    def test_pass_state_is_carried_over_when_the_centroid_stays_put(self):
        states = fin.rebuild_gate_states([np.array([0.0, 0.0]), np.array([2.0, 0.0])], [])
        states[0]['inside'] = True
        again = fin.rebuild_gate_states([np.array([0.0, 0.0]), np.array([2.0, 0.0]), np.array([1.0, 1.0])], states)
        assert again[0]['inside'] is True

    def test_entering_a_gate_counts_one_pass_after_the_debounce_distance(self):
        gate = {'c': np.array([0.0, 0.0]), 'inside': False}
        assert fin.gate_pass_step(gate, d=2.0, travel_m=3.0, enter_r=4.5) is False     # before the debounce
        gate['inside'] = False
        assert fin.gate_pass_step(gate, d=2.0, travel_m=10.0, enter_r=4.5) is True
        assert gate['inside'] is True
        assert fin.gate_pass_step(gate, d=2.0, travel_m=11.0, enter_r=4.5) is False    # still inside: no second pass

    def test_leaving_the_gate_needs_the_hysteresis_before_a_new_pass(self):
        gate = {'c': np.array([0.0, 0.0]), 'inside': True}
        fin.gate_pass_step(gate, d=5.0, travel_m=20.0, enter_r=4.5)                    # 5.0 < 4.5 + 2.0: still inside
        assert gate['inside'] is True
        fin.gate_pass_step(gate, d=7.0, travel_m=21.0, enter_r=4.5)                    # beyond 6.5: left the gate
        assert gate['inside'] is False
        assert fin.gate_pass_step(gate, d=1.0, travel_m=22.0, enter_r=4.5) is True     # a second pass counts

    def test_gate_first_seen_with_the_car_under_it_is_not_counted_at_spawn(self):
        gate = {'c': np.array([0.0, 0.0]), 'inside': None}
        assert fin.gate_pass_step(gate, d=1.0, travel_m=0.0, enter_r=4.5) is False
        assert gate['inside'] is True


class TestLidarForwardZone:

    def test_points_in_the_zone_are_counted(self):
        pts = np.array([[5.0, 0.0, 0.0], [5.0, 1.0, 0.0], [20.0, 0.0, 0.0], [5.0, 5.0, 0.0], [0.5, 0.0, 0.0]])
        assert fin.lidar_forward_count(pts, fwd_min=1.5, fwd_max=15.0, lat_max=4.0) == 2

    def test_points_below_the_ground_band_are_ignored(self):
        pts = np.array([[5.0, 0.0, -0.5], [5.0, 0.0, 0.2]])
        assert fin.lidar_forward_count(pts, 1.5, 15.0, 4.0) == 1


class _FakeCloud:
    """Minimal PointCloud2 stand-in: x, y, z float32 fields at offsets 0, 4, 8 with a 16-byte point step."""

    def __init__(self, rows):
        self.fields = [type('F', (), {'name': n, 'offset': o})() for n, o in (('x', 0), ('y', 4), ('z', 8))]
        self.width = len(rows)
        self.height = 1
        self.point_step = 16
        self.data = b''.join(struct.pack('<fff', *r) + b'\x00' * 4 for r in rows)


def test_extract_xyz_reads_finite_points_only():
    cloud = _FakeCloud([(1.0, 2.0, 0.5), (float('nan'), 0.0, 0.0), (3.0, -4.0, 1.0)])
    pts = fin.extract_xyz(cloud)
    assert pts.dtype == np.float32 and pts.shape == (2, 3)
    assert np.allclose(pts[0], [1.0, 2.0, 0.5]) and np.allclose(pts[1], [3.0, -4.0, 1.0])


def test_extract_xyz_without_fields_is_empty():
    cloud = _FakeCloud([(1.0, 2.0, 0.5)])
    cloud.fields = cloud.fields[:2]
    assert fin.extract_xyz(cloud).shape == (0, 3)
