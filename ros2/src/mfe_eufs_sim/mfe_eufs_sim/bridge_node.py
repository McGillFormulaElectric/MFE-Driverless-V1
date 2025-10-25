#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time

qos_besteffort = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # sim uses
    history=HistoryPolicy.KEEP_LAST,
    depth=100
)

qos_reliable = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,  # rviz's default
    history=HistoryPolicy.KEEP_LAST,
    depth=100
)

class EufsSimBridge(Node):
    def __init__(self):
        super().__init__('mfe_eufs_sim_bridge')

        # subscribe to EUFS Simulated sensor topics
        self.create_subscription(Image, '/zed/image_raw', self.camera_callback, qos_besteffort)
        self.create_subscription(PointCloud2, '/velodyne_points', self.lidar_callback, qos_besteffort)

        # # publishers for MFE Driverless perception
        self.camera_pub = self.create_publisher(Image, '/camera/image/input', qos_reliable)
        self.lidar_pub = self.create_publisher(PointCloud2, '/lidar/pcl/input', qos_reliable)

    def camera_callback(self, msg):
        self.camera_pub.publish(msg)

    def lidar_callback(self, msg):
        pc_msg = msg
        pc_msg.header.frame_id = msg.header.frame_id
        
        self.lidar_pub.publish(pc_msg)

def main(args=None):
    rclpy.init(args=args)

    node = EufsSimBridge()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()
