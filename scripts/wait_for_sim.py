#!/usr/bin/env python3
"""Wait for real simulator messages using a wall-clock deadline."""
import argparse
import time
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from eufs_msgs.msg import CarState
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('perception', 'no_perception'), required=True)
    parser.add_argument('--timeout', type=float, default=120)
    parser.add_argument('--exit-marker', type=Path, required=True)
    args = parser.parse_args()
    required = {'/ros_can/state_str': String, '/ground_truth/state': CarState}
    if args.mode == 'perception':
        required.update({'/velodyne_points': PointCloud2, '/d435i/image_raw': Image})
    pending = set(required)
    rclpy.init()
    node = rclpy.create_node('mfe_wait_for_sim')
    subscriptions = [node.create_subscription(
        msg_type, topic, lambda msg, name=topic: pending.discard(name), qos_profile_sensor_data)
        for topic, msg_type in required.items()]
    deadline = time.monotonic() + args.timeout
    try:
        while pending and time.monotonic() < deadline:
            if args.exit_marker.exists():
                print('Simulator launch exited before it became ready.', flush=True)
                return 1
            rclpy.spin_once(node, timeout_sec=0.2)
        if pending:
            print('Simulator readiness timed out. No messages on: ' + ', '.join(sorted(pending)), flush=True)
            return 1
        print('Simulator ready: received ' + ', '.join(sorted(required)), flush=True)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
