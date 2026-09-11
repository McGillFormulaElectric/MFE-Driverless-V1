#!/usr/bin/env python3
"""Smoke check after launch_sim.sh: actual messages, bridge frames and sensor TF."""
import argparse
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('perception', 'no_perception'), default='perception')
    parser.add_argument('--timeout', type=float, default=30)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('mfe_sim_smoke_check')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    expected = {'/sim/xsens/state_odom': Odometry}
    if args.mode == 'perception':
        expected.update({'/lidar/points_raw': PointCloud2,
                         '/camera/image_raw': Image, '/camera/camera_info': CameraInfo})
    messages = {}
    subscriptions = [node.create_subscription(
        kind, topic, lambda msg, name=topic: messages.__setitem__(name, msg), qos_profile_sensor_data)
        for topic, kind in expected.items()]
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if set(messages) != set(expected):
                continue
            odom = messages['/sim/xsens/state_odom']
            assert odom.header.frame_id == 'map', odom.header.frame_id
            if args.mode == 'perception':
                points = messages['/lidar/points_raw']
                image = messages['/camera/image_raw']
                info = messages['/camera/camera_info']
                assert points.width * points.height > 0 and points.data, 'Empty LiDAR cloud'
                assert image.width == info.width == 640 and image.height == info.height == 480
                assert image.data and info.k[0] > 0, 'Missing image / calibration data'
                assert image.header.frame_id == info.header.frame_id == 'd435i_optical_frame'
                frames = [points.header.frame_id, image.header.frame_id]
                if not all(buffer.can_transform('base_footprint', frame, rclpy.time.Time()) for frame in frames):
                    continue
            print('PASS: live odometry' + (', LiDAR, D435i image/calibration and sensor TF' if args.mode == 'perception' else ''))
            return 0
        raise RuntimeError('Missing messages or sensor TF; received: ' + ', '.join(sorted(messages)))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
