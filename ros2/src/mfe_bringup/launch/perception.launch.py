"""
MFE Driverless — perception-only launch.

Run this on the perception machine (host PC in sim, or Jetson 1 on the real car).
Starts: LiDAR cone detector, vision (YOLO), boundary extractor, SLAM, EKF.
Does NOT start: path planner, pure pursuit, finish detector.

The compute machine runs compute.launch.py and receives processed cone detections
(/planning/cones) and pose (/ekf/output or /ground_truth/state_odom) over the network.

Usage (sim — perception on host, raw data stays local):
  export ROS_DOMAIN_ID=42
  export CYCLONEDDS_URI=file://$HOME/cyclone_tailscale.xml   # if using Tailscale
  ros2 launch mfe_bringup perception.launch.py \\
    pose_topic:=/ground_truth/state_odom \\
    use_slam:=false use_ekf:=false

Usage (real car — Jetson 1, sensors connected locally):
  ros2 launch mfe_bringup perception.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup = os.path.join(
        get_package_share_directory('mfe_bringup'), 'launch', 'bringup.launch.py'
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={
                'run_perception': 'true',
                'run_compute':    'false',
            }.items(),
        )
    ])
