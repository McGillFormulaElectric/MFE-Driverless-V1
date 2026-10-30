"""Pure geometry helpers shared by the planner, control and validation nodes.

Each function reproduces, bit for bit, the expression of the copy it replaces. Where two copies differ
(wrap by modulo or by atan2, arc length with or without the length guard) the copies are kept as separate
named functions so no result changes.
"""
import math

import numpy as np


def yaw_from_quaternion(q):
    """Yaw (rad) of a quaternion-like object with x, y, z, w fields (geometry_msgs/Quaternion)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    """Wrap an angle to [-pi, pi) with the modulo form (used by the planner and the validation nodes)."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def wrap_angle_atan2(a):
    """Wrap an angle to (-pi, pi] with atan2(sin, cos) (used by pure pursuit and MPC heading errors)."""
    return math.atan2(math.sin(a), math.cos(a))


def arc_length(xy):
    """Length (m) of the polyline xy (N x 2 array). No length guard: an empty or one-point array gives 0.0."""
    return float(np.sum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))))


def arc_length_guarded(xy):
    """Length (m) of the polyline xy (any sequence of (x, y)). Returns 0.0 for fewer than two points."""
    xy = np.asarray(xy, dtype=float)
    return float(np.sum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1])))) if len(xy) >= 2 else 0.0


def cloud_xy(msg):
    """x and y arrays of the finite points of a sensor_msgs/PointCloud2 (sensor-frame metres)."""
    from sensor_msgs_py import point_cloud2   # ROS runtime helper, imported only when a cloud is read
    arr = point_cloud2.read_points(msg, field_names=('x', 'y'), skip_nans=True)
    return np.asarray(arr['x'], dtype=float), np.asarray(arr['y'], dtype=float)


def pointcloud2_to_xy_array(msg):
    """Nx2 array of (x, y) for the points of a PointCloud2 whose x, y and z are all finite."""
    from sensor_msgs_py import point_cloud2   # ROS runtime helper, imported only when a cloud is read
    return np.array([[a[0], a[1]] for a in point_cloud2.read_points(
        msg, field_names=('x', 'y', 'z'), skip_nans=True)]).reshape(-1, 2)
