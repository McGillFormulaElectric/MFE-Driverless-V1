#!/usr/bin/env python3
"""
Emulates the motion smear of a spinning VLP-16 on the EUFS sim LiDAR.

The EUFS ray sensor samples every ray at a single instant, so a sim scan has no motion
distortion. A real 10 Hz sweep measures each ray at its own time, so points measured late in
the sweep come from a car that has moved. This node re-expresses each instantaneous sim point
as it would be measured from the sensor pose at the point's sweep time, under a constant body
twist (vx, vy, yaw rate). Sweep time 0 is the start azimuth, so the instantaneous scan is exact
at the start of the sweep.

Output points are sorted by sweep time, matching the index-based time model used by
MotionDistortionCorrector.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

from mfe_common.qos import RELIABLE_QOS


# sensor_msgs PointField datatype codes -> numpy dtype strings.
_FIELD_DTYPES = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
_YAW_RATE_EPS = 1e-9


def sweep_times(azimuth, start_azimuth, period_s, counter_clockwise):
    """Sweep time in [0, period_s) for each azimuth, measured from the sweep start."""
    if counter_clockwise:
        swept = np.mod(azimuth - start_azimuth, 2.0 * math.pi)
    else:
        swept = np.mod(start_azimuth - azimuth, 2.0 * math.pi)
    return swept / (2.0 * math.pi) * period_s


def emulate_sweep(xyz, vx, vy, yaw_rate, *, period_s, start_azimuth, counter_clockwise,
                  sensor_x):
    """
    Re-express instantaneous sensor-frame points as measured over one sweep.

    xyz: (N, 3) float array in the sensor frame at the sweep start.
    vx, vy: body-frame linear velocity (m/s); yaw_rate: rad/s.
    sensor_x: sensor x offset from the body origin (m). Its lever arm adds
    yaw_rate * sensor_x to the sensor's lateral velocity.

    Returns (xyz_out, order): xyz_out holds the emulated points (non-finite inputs pass
    through), order sorts the points by sweep time (non-finite points last).
    """
    finite = np.isfinite(xyz).all(axis=1)
    x = xyz[:, 0]
    y = xyz[:, 1]

    tau = np.full(len(xyz), np.inf)
    tau[finite] = sweep_times(np.arctan2(y[finite], x[finite]), start_azimuth, period_s,
                              counter_clockwise)
    t = tau[finite]

    # Sensor velocity in its own frame (rigid body: v_sensor = v_body + omega x r).
    u = vx
    w = vy + yaw_rate * sensor_x
    if abs(yaw_rate) < _YAW_RATE_EPS:
        a = t
        b = np.zeros_like(t)
    else:
        a = np.sin(yaw_rate * t) / yaw_rate
        b = (1.0 - np.cos(yaw_rate * t)) / yaw_rate
    # Sensor displacement from the sweep start, expressed in the start frame.
    disp_x = a * u - b * w
    disp_y = b * u + a * w

    # The same point seen from the later pose: p_tau = R(-theta) (p_0 - displacement).
    theta = yaw_rate * t
    c = np.cos(theta)
    s = np.sin(theta)
    qx = x[finite] - disp_x
    qy = y[finite] - disp_y

    out = xyz.astype(np.float64, copy=True)
    out[finite, 0] = c * qx + s * qy
    out[finite, 1] = -s * qx + c * qy

    order = np.argsort(tau, kind='stable')
    return out, order


def _point_dtype(msg):
    names, formats, offsets = [], [], []
    for f in msg.fields:
        if f.count != 1 or f.datatype not in _FIELD_DTYPES:
            raise ValueError(
                f'unsupported PointField {f.name} (datatype {f.datatype}, count {f.count})')
        names.append(f.name)
        formats.append(_FIELD_DTYPES[f.datatype])
        offsets.append(f.offset)
    return np.dtype({'names': names, 'formats': formats, 'offsets': offsets,
                     'itemsize': msg.point_step})


class LidarSweepEmulator(Node):

    def __init__(self):
        super().__init__('lidar_sweep_emulator')

        self.declare_parameter('input_topic', '/lidar/points_instant')
        self.declare_parameter('output_topic', '/lidar/points_raw')
        self.declare_parameter('odom_topic', '/ground_truth/state_odom')
        self.declare_parameter('sweep_period_s', 0.1)
        self.declare_parameter('sweep_start_azimuth_rad', -2.0)
        self.declare_parameter('counter_clockwise', True)
        # VLP-16 mount on the EUFS car (eufs_racecar robot.urdf.xacro): x = -0.15 m.
        self.declare_parameter('sensor_x_m', -0.15)

        self._period = self.get_parameter('sweep_period_s').value
        self._start_azimuth = self.get_parameter('sweep_start_azimuth_rad').value
        self._counter_clockwise = self.get_parameter('counter_clockwise').value
        self._sensor_x = self.get_parameter('sensor_x_m').value
        self._vx = 0.0
        self._vy = 0.0
        self._yaw_rate = 0.0

        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._odom_cb, RELIABLE_QOS)
        self._pub = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value, RELIABLE_QOS)
        self.create_subscription(
            PointCloud2, self.get_parameter('input_topic').value, self._cloud_cb, RELIABLE_QOS)

    def _odom_cb(self, msg: Odometry) -> None:
        self._vx = msg.twist.twist.linear.x
        self._vy = msg.twist.twist.linear.y
        self._yaw_rate = msg.twist.twist.angular.z

    def _cloud_cb(self, msg: PointCloud2) -> None:
        n = msg.width * msg.height
        if n == 0:
            self._pub.publish(msg)
            return

        records = np.frombuffer(msg.data, dtype=_point_dtype(msg), count=n)
        xyz = np.stack([records['x'], records['y'], records['z']], axis=1).astype(np.float64)
        out_xyz, order = emulate_sweep(
            xyz, self._vx, self._vy, self._yaw_rate,
            period_s=self._period, start_azimuth=self._start_azimuth,
            counter_clockwise=self._counter_clockwise, sensor_x=self._sensor_x)

        sorted_records = records[order].copy()
        sorted_xyz = out_xyz[order]
        sorted_records['x'] = sorted_xyz[:, 0]
        sorted_records['y'] = sorted_xyz[:, 1]
        sorted_records['z'] = sorted_xyz[:, 2]

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = n
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.point_step * n
        out.is_dense = msg.is_dense
        out.data = sorted_records.tobytes()
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSweepEmulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
