"""
MFE Driverless — compute-only launch.

Run this on the compute machine (Jetson 2, or any Tailscale peer).
Starts: path planner, pure pursuit, finish detector.
Does NOT start: LiDAR, vision, SLAM, EKF — those run on the perception machine.

Expects to receive over the network:
  /planning/cones          (mfe_msgs/ConeArray)  — from boundary_extractor
  /ekf/output              (nav_msgs/Odometry)   — from EKF, or remap to ground truth in sim
  /ground_truth/state_odom (nav_msgs/Odometry)   — sim only

Usage (sim — compute on Jetson, perception on host):
  export ROS_DOMAIN_ID=42
  export CYCLONEDDS_URI=file://$HOME/cyclone_tailscale.xml
  ros2 launch mfe_bringup compute.launch.py \\
    pose_topic:=/ground_truth/state_odom \\
    mission:=autocross

Usage (real car — Jetson 2):
  ros2 launch mfe_bringup compute.launch.py mission:=trackdrive
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
                'run_perception': 'false',
                'run_compute':    'true',
                'use_slam':       'false',
                'use_ekf':        'false',
            }.items(),
        )
    ])
