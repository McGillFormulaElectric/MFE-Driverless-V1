"""Bit-identity checks: each mfe_common helper must equal the expression of the copy it replaced."""
import math
import types

import numpy as np
import pytest

from mfe_common import geometry as g


def _quats(n=500, seed=1):
    rng = np.random.default_rng(seed)
    for _ in range(n):
        v = rng.normal(size=4)
        v /= np.linalg.norm(v)
        yield types.SimpleNamespace(x=float(v[0]), y=float(v[1]), z=float(v[2]), w=float(v[3]))


def test_yaw_matches_planner_and_control_forms():
    for q in _quats():
        planner = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        mpc = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        validation = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        assert g.yaw_from_quaternion(q) == planner == mpc == validation


def test_wrap_angle_matches_modulo_form():
    for a in np.linspace(-20.0, 20.0, 4001):
        expected = (float(a) + math.pi) % (2 * math.pi) - math.pi
        assert g.wrap_angle(float(a)) == expected
        assert -math.pi <= g.wrap_angle(float(a)) < math.pi


def test_wrap_angle_atan2_matches_pursuit_form_and_range():
    for a in np.linspace(-20.0, 20.0, 4001):
        expected = math.atan2(math.sin(float(a)), math.cos(float(a)))
        assert g.wrap_angle_atan2(float(a)) == expected
        assert -math.pi < g.wrap_angle_atan2(float(a)) <= math.pi


def test_the_two_wraps_differ_at_pi_so_both_must_exist():
    # modulo form maps pi to -pi, atan2 form keeps +pi: this is why there are two functions
    assert g.wrap_angle(math.pi) == -math.pi
    assert g.wrap_angle_atan2(math.pi) == math.pi


def test_arc_length_matches_planner_utils_form():
    rng = np.random.default_rng(2)
    for n in (2, 3, 17, 200):
        xy = rng.normal(size=(n, 2))
        expected = float(np.sum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))))
        assert g.arc_length(xy) == expected


def test_arc_length_guarded_matches_node_form_and_guard():
    rng = np.random.default_rng(3)

    def node_form(xy):
        xy = np.asarray(xy, dtype=float)
        return float(np.sum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1])))) if len(xy) >= 2 else 0.0

    for n in (0, 1, 2, 5, 60):
        xy = rng.normal(size=(n, 2))
        assert g.arc_length_guarded(xy) == node_form(xy)
    assert g.arc_length_guarded([]) == 0.0
    assert g.arc_length_guarded([(1.0, 2.0)]) == 0.0
    assert g.arc_length_guarded([(0.0, 0.0), (3.0, 4.0)]) == pytest.approx(5.0)


def test_arc_length_unguarded_on_one_point_is_zero():
    assert g.arc_length(np.array([[1.0, 2.0]])) == 0.0


def _fake_pc2(monkeypatch, result):
    import sys
    import types as t
    mod = t.ModuleType('sensor_msgs_py.point_cloud2')
    calls = []

    def read_points(msg, field_names, skip_nans):
        calls.append((field_names, skip_nans))
        return result
    mod.read_points = read_points
    pkg = t.ModuleType('sensor_msgs_py')
    pkg.point_cloud2 = mod
    monkeypatch.setitem(sys.modules, 'sensor_msgs_py', pkg)
    monkeypatch.setitem(sys.modules, 'sensor_msgs_py.point_cloud2', mod)
    return calls


def test_cloud_xy_reads_x_and_y_and_skips_nans(monkeypatch):
    arr = np.zeros(3, dtype=[('x', 'f4'), ('y', 'f4')])
    arr['x'] = [1.0, 2.0, 3.0]
    arr['y'] = [4.0, 5.0, 6.0]
    calls = _fake_pc2(monkeypatch, arr)
    xs, ys = g.cloud_xy(object())
    assert calls == [(('x', 'y'), True)]
    assert xs.dtype == float and ys.dtype == float
    np.testing.assert_array_equal(xs, [1.0, 2.0, 3.0])
    np.testing.assert_array_equal(ys, [4.0, 5.0, 6.0])


def test_pointcloud2_to_xy_array_reads_xyz_and_returns_nx2(monkeypatch):
    rows = np.zeros(2, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
    rows['x'] = [1.0, 2.0]
    rows['y'] = [3.0, 4.0]
    rows['z'] = [0.5, 0.5]
    calls = _fake_pc2(monkeypatch, rows)
    out = g.pointcloud2_to_xy_array(object())
    assert calls == [(('x', 'y', 'z'), True)]
    assert out.shape == (2, 2)
    np.testing.assert_array_equal(out, [[1.0, 3.0], [2.0, 4.0]])


def test_pointcloud2_to_xy_array_empty_is_nx2(monkeypatch):
    _fake_pc2(monkeypatch, np.zeros(0, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')]))
    out = g.pointcloud2_to_xy_array(object())
    assert out.shape == (0, 2)


def _old_planner_dist(pt, xy):
    # copy of the planner's core/paths.py dist_to_polyline before it moved here
    a, ab = xy[:-1], np.diff(xy, axis=0)
    t = np.clip(np.sum((pt - a) * ab, axis=1) / np.maximum(np.sum(ab * ab, axis=1), 1e-12), 0.0, 1.0)
    return float(np.min(np.linalg.norm(pt - (a + t[:, None] * ab), axis=1)))


def _old_compare_dists(pts, line):
    # copy of the validation compare_node per-point loop before it moved here
    out = []
    for p in pts:
        a, b = line[:-1], line[1:]
        ab = b - a
        t = np.clip(((p - a) * ab).sum(1) / np.maximum((ab * ab).sum(1), 1e-9), 0, 1)
        out.append(np.min(np.hypot(*(a + ab * t[:, None] - p).T)))
    return np.array(out)


def test_dist_to_polyline_matches_planner_form_exactly():
    rng = np.random.default_rng(4)
    for n in (2, 3, 40):
        xy = rng.normal(size=(n, 2))
        for pt in rng.normal(size=(20, 2)):
            assert g.dist_to_polyline(pt, xy) == _old_planner_dist(pt, xy)


def test_dist_to_polyline_matches_compare_loop():
    rng = np.random.default_rng(5)
    line = np.cumsum(rng.normal(scale=0.2, size=(60, 2)), axis=0)
    line = np.vstack([line, line[-1:]])   # repeated last vertex gives a zero-length segment
    pts = rng.normal(scale=3.0, size=(30, 2))
    new = np.array([g.dist_to_polyline(p, line) for p in pts])
    np.testing.assert_allclose(new, _old_compare_dists(pts, line), rtol=0, atol=1e-12)


def test_dist_to_polyline_clamps_to_segment_ends():
    xy = np.array([[0.0, 0.0], [10.0, 0.0]])
    assert g.dist_to_polyline(np.array([5.0, 3.0]), xy) == 3.0
    assert g.dist_to_polyline(np.array([12.0, 0.0]), xy) == 2.0
    assert g.dist_to_polyline(np.array([0.0, 0.0]), xy) == 0.0


def test_quat_to_rot3_matches_evaluator_expression():
    for q in _quats(200):
        x, y, z, w = q.x, q.y, q.z, q.w
        expected = np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)],
        ], dtype=np.float64)
        np.testing.assert_array_equal(g.quat_to_rot3(x, y, z, w), expected)


def test_quat_to_rot3_identity_and_quarter_turn_about_z():
    np.testing.assert_allclose(g.quat_to_rot3(0.0, 0.0, 0.0, 1.0), np.eye(3))
    s = math.sqrt(0.5)
    quarter_turn = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    np.testing.assert_allclose(g.quat_to_rot3(0.0, 0.0, s, s), quarter_turn, atol=1e-12)
