#!/usr/bin/env python3

import rclpy

from lidar_cone_detector.cone_detector import LiDARConeNode

def main(args=None):
    rclpy.init(args=args)
    node = LiDARConeNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()