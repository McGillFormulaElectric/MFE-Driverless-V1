"""Validation stack, run alongside a sim whose main planner uses ground-truth cones (launch_sim_local.sh ... hybrid).

Usage: ros2 launch mfe_validation validation.launch.py mission:=acceleration track_csv:=<path>
Nothing here publishes /cmd or /planning/*, so it cannot affect the car.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
import yaml


def _lidar_perception_params():
    path = os.path.join(get_package_share_directory('lidar_cone_detector'), 'config',
                        'lidar_perception_node_params.yaml')
    with open(path) as f:
        return yaml.safe_load(f)['lidar_perception_node']['ros__parameters']


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mission', default_value='acceleration'),
        DeclareLaunchArgument('track_csv', default_value=''),
        DeclareLaunchArgument('log_csv', default_value=''),
        # The node name differs from lidar_perception_node, so the YAML section is read here and
        # passed as a dict. Motion distortion stays on, as before: lidar_only turns it off.
        Node(package='lidar_cone_detector', executable='lidar_perception_node', name='validation_lidar_perception', output='log',
             parameters=[{**_lidar_perception_params(), 'enable_motion_distortion': True}]),
        Node(package='mfe_validation', executable='slam_node', name='slam_node', output='screen',
             parameters=[{'track_csv': LC('track_csv')}]),
        # Second planner: same code as the main one, fed by perception + SLAM; outputs under /validation.
        Node(package='mfe_path_planning', executable='path_planner_node', name='validation_planner', output='log',
             parameters=[{'mission': LC('mission')}],
             remappings=[('/planning/cones', '/validation/cones'), ('/ekf/output', '/slam/odom'),
                         ('/planning/centerline', '/validation/centerline'),
                         ('/planning/track_left', '/validation/track_left'), ('/planning/track_right', '/validation/track_right')]),
        Node(package='mfe_validation', executable='compare_node', name='compare_node', output='screen',
             parameters=[os.path.join(get_package_share_directory('mfe_validation'), 'config',
                                      'compare_node_params.yaml'),
                         {'log_csv': LC('log_csv')}]),
    ])
