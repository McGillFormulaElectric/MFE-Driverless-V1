#!/usr/bin/env python3
"""
Emulates the motion smear of a spinning VLP-16 on the EUFS sim LiDAR.

The EUFS ray sensor samples every ray at a single instant, so a sim scan has no motion
distortion. A real 10 Hz sweep measures each ray at its own time, so points measured late in
the sweep come from a car that has moved. This node re-expresses each instantaneous sim point
as it would be measured from the sensor pose at the point's sweep time, under a constant body
twist (vx, vy, yaw rate). Sweep time 0 is the start azimuth, so the instantaneous scan is exact
at the start of the sweep.

Each output point carries a FLOAT32 "time" field: seconds since the sweep start, the convention
of the Velodyne ROS 2 driver. Output points are sorted by that time.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry

from mfe_common.qos import RELIABLE_QOS


# sensor_msgs PointField datatype codes -> numpy dtype strings.
_FIELD_DTYPES = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
_YAW_RATE_EPS = 1e-9
_SEAM_TOL = 1e-6  # rad; far below the VLP-16 azimuth resolution (about 3.5e-3 rad)
# The motion moves a point's azimuth by ~0.07 rad; each iteration cuts the error about 50x.
_SWEEP_FIXED_POINT_ITERATIONS = 4


def sweep_times(azimuth, start_azimuth, period_s, counter_clockwise):
    """Sweep time in [0, period_s) for each azimuth, measured from the sweep start."""
    if counter_clockwise:
        delta = azimuth - start_azimuth
    else:
        delta = start_azimuth - azimuth
    # Rays on the start edge can come out a rounding error below it; keep them at time 0
    # instead of wrapping them to the end of the sweep.
    swept = np.maximum(np.mod(delta + _SEAM_TOL, 2.0 * math.pi) - _SEAM_TOL, 0.0)
    return swept / (2.0 * math.pi) * period_s


def _displace(x, y, t, vx, vy, yaw_rate, sensor_x, sensor_y):
    """Sensor-frame (x, y) of points measured at sweep times t, in the sweep-start frame."""
    # Sensor velocity in its own frame (rigid body: v_sensor = v_body + omega x r).
    u = vx - yaw_rate * sensor_y
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
    qx = x - disp_x
    qy = y - disp_y
    return c * qx + s * qy, -s * qx + c * qy


def emulate_sweep(xyz, vx, vy, yaw_rate, *, period_s, start_azimuth, counter_clockwise,
                  sensor_x, sensor_y=0.0):
    """
    Re-express instantaneous sensor-frame points as measured over one sweep.

    xyz: (N, 3) float array in the sensor frame at the sweep start.
    vx, vy: body-frame linear velocity (m/s); yaw_rate: rad/s.
    sensor_x, sensor_y: sensor offset from the body origin (m).

    A point is measured by the ray that fires at its sweep time, so its azimuth in the sensor
    frame at that time is the sweep angle. The sweep time is solved so that holds for the
    emulated point, which keeps the azimuth-based fallback of MotionDistortionCorrector exact.

    Returns (xyz_out, tau, order): xyz_out holds the emulated points (non-finite inputs pass
    through), tau the sweep time of each point (NaN for non-finite inputs), order sorts the
    points by sweep time (NaN last).
    """
    finite = np.isfinite(xyz).all(axis=1)
    x = xyz[finite, 0]
    y = xyz[finite, 1]

    def sweep_time_of(px, py):
        return sweep_times(np.arctan2(py, px), start_azimuth, period_s, counter_clockwise)

    def displace(t):
        return _displace(x, y, t, vx, vy, yaw_rate, sensor_x, sensor_y)

    t = sweep_time_of(x, y)
    for _ in range(_SWEEP_FIXED_POINT_ITERATIONS):
        t = sweep_time_of(*displace(t))
    out_x, out_y = displace(t)

    tau = np.full(len(xyz), np.nan)
    tau[finite] = t
    out = xyz.astype(np.float64, copy=True)
    out[finite, 0] = out_x
    out[finite, 1] = out_y

    order = np.argsort(tau, kind='stable')
    return out, tau, order


_TIME_FIELD = 'time'
_TIME_DATATYPE = 7  # FLOAT32


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
        self.declare_parameter('sensor_y_m', 0.0)

        self._period = self.get_parameter('sweep_period_s').value
        self._start_azimuth = self.get_parameter('sweep_start_azimuth_rad').value
        self._counter_clockwise = self.get_parameter('counter_clockwise').value
        self._sensor_x = self.get_parameter('sensor_x_m').value
        self._sensor_y = self.get_parameter('sensor_y_m').value
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
        out_xyz, tau, order = emulate_sweep(
            xyz, self._vx, self._vy, self._yaw_rate,
            period_s=self._period, start_azimuth=self._start_azimuth,
            counter_clockwise=self._counter_clockwise, sensor_x=self._sensor_x,
            sensor_y=self._sensor_y)

        # The input's own time field, if any, is replaced by the sweep time.
        kept = [f for f in msg.fields if f.name != _TIME_FIELD]
        time_offset = msg.point_step
        out_dtype = np.dtype({
            'names': [f.name for f in kept] + [_TIME_FIELD],
            'formats': [_FIELD_DTYPES[f.datatype] for f in kept] + ['f4'],
            'offsets': [f.offset for f in kept] + [time_offset],
            'itemsize': msg.point_step + 4})

        sorted_xyz = out_xyz[order]
        out_records = np.zeros(n, dtype=out_dtype)
        for f in kept:
            out_records[f.name] = records[f.name][order]
        out_records['x'] = sorted_xyz[:, 0]
        out_records['y'] = sorted_xyz[:, 1]
        out_records['z'] = sorted_xyz[:, 2]
        out_records[_TIME_FIELD] = tau[order]

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = n
        out.fields = kept + [PointField(name=_TIME_FIELD, offset=time_offset,
                                        datatype=_TIME_DATATYPE, count=1)]
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step + 4
        out.row_step = out.point_step * n
        out.is_dense = msg.is_dense
        out.data = out_records.tobytes()
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSweepEmulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
