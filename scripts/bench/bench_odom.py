#!/usr/bin/env python3
"""Stationary bench stand-in for the EKF: the car is not moving and there is no IMU/GPS.

Publishes /ekf/output (nav_msgs/Odometry, zero pose and twist, current stamp) at 50 Hz, which
gtsam SLAM needs before it accepts cone frames, and the odom -> base_footprint identity TF the
EKF would otherwise publish. Bench use only: never run this on a moving car.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class BenchOdom(Node):
    def __init__(self):
        super().__init__('bench_odom')
        self.pub = self.create_publisher(Odometry, '/ekf/output',
                                         QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        tf.transform.rotation.w = 1.0
        self.tf = StaticTransformBroadcaster(self)
        self.tf.sendTransform(tf)
        self.create_timer(0.02, self.tick)
        self.get_logger().info('bench_odom: stationary /ekf/output at 50 Hz + odom->base_footprint')

    def tick(self):
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_footprint'
        m.pose.pose.orientation.w = 1.0
        self.pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(BenchOdom())


if __name__ == '__main__':
    main()
