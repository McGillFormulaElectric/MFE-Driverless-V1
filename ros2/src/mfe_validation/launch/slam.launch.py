"""SLAM entry point for bringup (slam:=gtsam): GTSAM landmark SLAM on the LiDAR cones.

Starts only slam_node, with the same parameters as lidar_only.launch.py. The LiDAR
detector stays in bringup. slam_node publishes /slam/odom and the cone map on
/planning/cones, so bringup routes the planner pose there.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

from mfe_common.topics import PLANNING_ORANGE_CONES, SLAM_CONE_MAP


def generate_launch_description():
    return LaunchDescription([
        Node(package='mfe_validation', executable='slam_node', name='slam_node', output='screen',
             parameters=[{'use_lateral_vel': False, 'cones_topic': SLAM_CONE_MAP,
                          'color_mode': 'geometric', 'orange_cones_topic': PLANNING_ORANGE_CONES,
                          'min_obs_publish': 2, 'max_range': 20.0, 'publish_hz': 5.0,
                          'freeze': True}]),
    ])
