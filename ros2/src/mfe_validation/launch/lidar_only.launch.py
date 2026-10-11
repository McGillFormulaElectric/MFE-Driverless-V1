"""LiDAR-only chain that REPLACES perception for the car's own planner: LiDAR -> detector -> GTSAM SLAM -> /planning/cones + /slam/odom.
Use with launch_sim_local.sh <track> lidar_only (bringup pose_topic=/slam/odom, no ground-truth cones, no boundary_extractor).
Ground truth is only read by slam_node's logging (accuracy score), never fed back.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('min_obs', default_value='3'),
        DeclareLaunchArgument('skidpad', default_value='false'),     # true: skidpad_planner fits the known layout to the landmarks (landmarks go to /skidpad/cones)
        DeclareLaunchArgument('cones_topic', default_value='/planning/cones'),
        DeclareLaunchArgument('color_mode', default_value='geometric'),   # geometric | unknown (no colour; planner infers sides)
        Node(package='lidar_cone_detector', executable='lidar_perception_node', name='lidar_perception_node', output='log',
             parameters=[{'gpu_voxel': False, 'leaf_size': 0.05, 'ground_threshold': 0.04, 'cluster_tolerance': 0.3,
                          'min_cluster_size': 1, 'far_min_cluster_size': 1, 'max_cluster_size': 50, 'cluster_2d': True,
                          'height_min': 0.02, 'enable_motion_distortion': False}]),
        Node(package='mfe_validation', executable='slam_node', name='slam_node', output='screen',
             parameters=[{'cones_topic': LC('cones_topic'), 'color_mode': LC('color_mode'), 'min_obs_publish': ParameterValue(LC('min_obs'), value_type=int)}]),
        Node(package='mfe_validation', executable='skidpad_planner', name='skidpad_planner', output='screen', condition=IfCondition(LC('skidpad'))),
    ])
