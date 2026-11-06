"""SLAM entry point for bringup (slam:=gtsam): GTSAM landmark SLAM on the LiDAR cones.

Starts only slam_node. The arguments and their defaults are the ones lidar_only.launch.py uses,
so the same values give the same slam_node parameters. The LiDAR detector stays in bringup.
slam_node publishes /slam/odom and the cone map on /planning/cones, so bringup routes the
planner pose there.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from mfe_common.topics import PLANNING_ORANGE_CONES, SLAM_CONE_MAP


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('slam_lateral_vel', default_value='false'),
        DeclareLaunchArgument('color_mode', default_value='geometric'),
        DeclareLaunchArgument('min_obs', default_value='2'),
        DeclareLaunchArgument('max_range', default_value='20.0'),
        DeclareLaunchArgument('publish_hz', default_value='5.0'),
        DeclareLaunchArgument('freeze', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='/sim/xsens/state_odom'),
        Node(package='mfe_validation', executable='slam_node', name='slam_node', output='screen',
             parameters=[{
                 'use_lateral_vel': ParameterValue(LC('slam_lateral_vel'), value_type=bool),
                 'odom_topic': LC('odom_topic'),
                 'cones_topic': SLAM_CONE_MAP,
                 'color_mode': LC('color_mode'),
                 'orange_cones_topic': PLANNING_ORANGE_CONES,
                 'min_obs_publish': ParameterValue(LC('min_obs'), value_type=int),
                 'max_range': ParameterValue(LC('max_range'), value_type=float),
                 'publish_hz': ParameterValue(LC('publish_hz'), value_type=float),
                 'freeze': ParameterValue(LC('freeze'), value_type=bool),
             }]),
    ])
