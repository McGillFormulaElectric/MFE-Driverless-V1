import math

from builtin_interfaces.msg import Time

from mfe_state_estimation.extended_kalman_filter_node import _odom_to_base_tf


def test_odom_to_base_tf_frames_stamp_and_translation():
    stamp = Time(sec=12, nanosec=500)

    tf = _odom_to_base_tf(stamp, 1.5, -2.0, 0.0)

    assert tf.header.frame_id == 'odom'
    assert tf.child_frame_id == 'base_footprint'
    assert tf.header.stamp == stamp
    assert tf.transform.translation.x == 1.5
    assert tf.transform.translation.y == -2.0
    assert tf.transform.translation.z == 0.0


def test_odom_to_base_tf_yaw_quaternion():
    yaw = math.pi / 2

    rotation = _odom_to_base_tf(Time(), 0.0, 0.0, yaw).transform.rotation

    assert rotation.x == 0.0
    assert rotation.y == 0.0
    assert math.isclose(rotation.z, math.sin(math.pi / 4))
    assert math.isclose(rotation.w, math.cos(math.pi / 4))
    assert math.isclose(2.0 * math.atan2(rotation.z, rotation.w), yaw)
