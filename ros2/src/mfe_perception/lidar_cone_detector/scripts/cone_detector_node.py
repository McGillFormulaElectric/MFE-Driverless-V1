#!/usr/bin/env python3

import rclpy

from lidar_cone_detector.cone_detector import LiDARConeNode

def main(args=None):
    rclpy.init(args=args)

    node = LiDARConeNode()

    try:
        rclpy.spin(node)  # Block until Ctrl+C
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()