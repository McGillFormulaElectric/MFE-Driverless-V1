#!/usr/bin/env python3
"""Emulates the car's Xsens MTi-670G IMU and GNSS in sim, on the topics the car uses.

Subscribes to /ground_truth/state_odom and publishes sensor_msgs/Imu on /imu and
sensor_msgs/NavSatFix on /gps, the topics the Xsens driver feeds to the EKF on the car
(xsens_mti.launch.py remaps them). The models are in sensor_emulation.py. Started by
mfe_eufs_sim.launch.py with platform:=sim and imu_gps in sensors.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix

from mfe_common.qos import RELIABLE_QOS
from mfe_common.topics import GPS, GROUND_TRUTH_STATE_ODOM, IMU
from mfe_eufs_sim.sensor_emulation import (
    DEFAULT_DATUM_LAT_DEG, DEFAULT_DATUM_LON_DEG, GnssEmulator, ImuEmulator,
)


class ImuGpsEmulatorNode(Node):

    def __init__(self):
        super().__init__('imu_gps_emulator')

        self.declare_parameter('input_topic', GROUND_TRUTH_STATE_ODOM)
        self.declare_parameter('imu_topic', IMU)
        self.declare_parameter('gps_topic', GPS)
        self.declare_parameter('imu_rate_hz', 100.0)
        self.declare_parameter('gps_rate_hz', 5.0)
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('gps_frame_id', 'gnss_link')
        self.declare_parameter('datum_lat_deg', DEFAULT_DATUM_LAT_DEG)
        self.declare_parameter('datum_lon_deg', DEFAULT_DATUM_LON_DEG)
        self.declare_parameter('datum_alt_m', 0.0)
        self.declare_parameter('seed', 0)  # 0 -> unseeded (different noise every run)

        seed = self.get_parameter('seed').value
        rng = np.random.default_rng(seed if seed else None)
        self._imu = ImuEmulator(
            rng, self.get_parameter('imu_rate_hz').value,
            self.get_parameter('imu_frame_id').value)
        self._gnss = GnssEmulator(
            rng, self.get_parameter('gps_rate_hz').value,
            self.get_parameter('gps_frame_id').value,
            self.get_parameter('datum_lat_deg').value,
            self.get_parameter('datum_lon_deg').value,
            self.get_parameter('datum_alt_m').value)

        self._imu_pub = self.create_publisher(
            Imu, self.get_parameter('imu_topic').value, RELIABLE_QOS)
        self._gps_pub = self.create_publisher(
            NavSatFix, self.get_parameter('gps_topic').value, RELIABLE_QOS)
        self.create_subscription(
            Odometry, self.get_parameter('input_topic').value, self._odom_cb, RELIABLE_QOS)

        self.get_logger().info(
            f"Xsens MTi-670G emulation: {self.get_parameter('input_topic').value} -> "
            f"IMU {self.get_parameter('imu_topic').value} at "
            f"{self.get_parameter('imu_rate_hz').value} Hz, "
            f"GNSS {self.get_parameter('gps_topic').value} at "
            f"{self.get_parameter('gps_rate_hz').value} Hz")

    def _odom_cb(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        twist = msg.twist.twist
        imu = self._imu.on_ground_truth(
            msg.header.stamp, (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                               pose.orientation.w),
            twist.linear.x, twist.linear.y, twist.angular.z)
        if imu is not None:
            self._imu_pub.publish(imu)
        fix = self._gnss.on_ground_truth(msg.header.stamp, pose.position.x, pose.position.y,
                                         pose.position.z)
        if fix is not None:
            self._gps_pub.publish(fix)


def main(args=None):
    rclpy.init(args=args)
    node = ImuGpsEmulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
