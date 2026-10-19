"""LiDAR-only chain replacing perception: LiDAR -> detector -> GTSAM SLAM -> /planning/cones and /slam/odom.

Use via launch_sim_local.sh <track> lidar_only (bringup pose_topic=/slam/odom). Ground truth is only used by slam_node logging.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        # Cone-map look-ahead: 25 m adds pose-error ghost cones when SLAM drifts, so 20 m is the default.
        DeclareLaunchArgument('min_obs', default_value='2'),
        DeclareLaunchArgument('max_range', default_value='20.0'),
        DeclareLaunchArgument('publish_hz', default_value='5.0'),      # planner replans per message
        DeclareLaunchArgument('skidpad', default_value='false'),     # true: skidpad_planner fits the known layout (landmarks on /skidpad/cones)
        DeclareLaunchArgument('registration_mode', default_value='library'),   # skidpad only: library (SkidpadRelocalizer) | custom (Kabsch/ICP)
        DeclareLaunchArgument('cones_topic', default_value='/planning/cones'),
        DeclareLaunchArgument('freeze', default_value='true'),   # publish each landmark once, at a fixed position
        DeclareLaunchArgument('color_mode', default_value='geometric'),   # geometric | unknown (no colour; planner infers sides)
        DeclareLaunchArgument('use_gpu', default_value='true'),   # false: CPU (PCL) pipeline, for load comparisons
        DeclareLaunchArgument('slam_lateral_vel', default_value='false'),   # true: SLAM odometry integrates INS lateral velocity (corner sideslip)
        Node(package='lidar_cone_detector', executable='lidar_perception_node', name='lidar_perception_node', output='log',
             parameters=[{'gpu_voxel': False, 'leaf_size': 0.05, 'ground_threshold': 0.04, 'cluster_tolerance': 0.3,
                          'min_cluster_size': 1, 'far_min_cluster_size': 1, 'max_cluster_size': 50, 'cluster_2d': True,
                          'height_min': 0.02, 'enable_motion_distortion': False,
                          'use_gpu': ParameterValue(LC('use_gpu'), value_type=bool)}]),
        Node(package='mfe_validation', executable='slam_node', name='slam_node', output='screen',
             parameters=[{'use_lateral_vel': ParameterValue(LC('slam_lateral_vel'), value_type=bool), 'cones_topic': LC('cones_topic'), 'color_mode': LC('color_mode'), 'min_obs_publish': ParameterValue(LC('min_obs'), value_type=int),
                          'max_range': ParameterValue(LC('max_range'), value_type=float), 'publish_hz': ParameterValue(LC('publish_hz'), value_type=float),
                          'freeze': ParameterValue(LC('freeze'), value_type=bool)}]),
        Node(package='mfe_validation', executable='skidpad_planner', name='skidpad_planner', output='screen', condition=IfCondition(LC('skidpad')),
             parameters=[{'registration_mode': LC('registration_mode')}]),
    ])
