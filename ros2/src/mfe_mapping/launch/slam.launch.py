"""SLAM entry point for bringup (slam:=toolbox): slam_toolbox on the LiDAR LaserScan.

Publishes the map->odom TF and /map. It does not publish odometry, so the planner
pose topic is not changed.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('lidar_cone_detector'), 'launch',
                             'pointcloud_to_laserscan.launch.py')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('mfe_mapping'), 'launch',
                             'slam_toolbox.launch.py')),
        ),
    ])
