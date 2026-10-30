"""QoS profiles must match the inline QoSProfile(...) calls they replace."""
import pytest

rclpy_qos = pytest.importorskip('rclpy.qos')

from mfe_common import qos  # noqa: E402


def _fields(p):
    return (p.reliability, p.history, p.depth, p.durability)


def test_reliable_depth10_matches_planner_and_validation_profile():
    p = rclpy_qos.QoSProfile(reliability=rclpy_qos.ReliabilityPolicy.RELIABLE,
                             history=rclpy_qos.HistoryPolicy.KEEP_LAST, depth=10)
    assert _fields(qos.RELIABLE_QOS) == _fields(p)


def test_reliable_depth10_matches_control_keyword_form():
    p = rclpy_qos.QoSProfile(depth=10, reliability=rclpy_qos.ReliabilityPolicy.RELIABLE)
    assert _fields(qos.RELIABLE_QOS) == _fields(p)


def test_latest_cones_is_reliable_depth1():
    assert qos.LATEST_CONES_QOS.depth == 1
    assert qos.LATEST_CONES_QOS.reliability == rclpy_qos.ReliabilityPolicy.RELIABLE


def test_sensor_profiles_are_best_effort_with_their_depths():
    p10 = rclpy_qos.QoSProfile(depth=10, reliability=rclpy_qos.ReliabilityPolicy.BEST_EFFORT)
    p5 = rclpy_qos.QoSProfile(reliability=rclpy_qos.ReliabilityPolicy.BEST_EFFORT,
                              history=rclpy_qos.HistoryPolicy.KEEP_LAST, depth=5)
    assert _fields(qos.SENSOR_QOS) == _fields(p10)
    assert _fields(qos.SHALLOW_SENSOR_QOS) == _fields(p5)


def test_latched_is_transient_local_depth1():
    assert qos.LATCHED_QOS.durability == rclpy_qos.DurabilityPolicy.TRANSIENT_LOCAL
    assert qos.LATCHED_QOS.depth == 1
