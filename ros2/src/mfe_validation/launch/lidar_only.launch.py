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
        # Cone-map look-ahead.  Offline replay of 15 peanut/its_a_mess bags + skidpad/accel (skidpad_compare/horizon_ext):
        # 15 m/3 obs/2 Hz -> 20 m/2 obs/5 Hz moves the median first-mapped distance (car moving) 11 -> 16 m and the
        # 10th-pct warning time 1.3-1.8 s -> 3.1-3.7 s, with no false cones before first cone contact and unchanged
        # pose/map error.  25 m adds pose-error ghost cones when SLAM drifts (bag NPF04), so it is not the default.
        DeclareLaunchArgument('min_obs', default_value='2'),           # keyframe sightings before a landmark is published
        DeclareLaunchArgument('max_range', default_value='20.0'),      # LiDAR detections beyond this are ignored by SLAM
        DeclareLaunchArgument('publish_hz', default_value='5.0'),      # cone-map publish rate (planner replans per message)
        DeclareLaunchArgument('skidpad', default_value='false'),     # true: skidpad_planner fits the known layout to the landmarks (landmarks go to /skidpad/cones)
        DeclareLaunchArgument('registration_mode', default_value='library'),   # skidpad only: library (ft-fsd SkidpadRelocalizer) | custom (Kabsch/ICP)
        DeclareLaunchArgument('cones_topic', default_value='/planning/cones'),
        DeclareLaunchArgument('freeze', default_value='true'),   # publish each landmark once, at a fixed position
        DeclareLaunchArgument('color_mode', default_value='geometric'),   # geometric | unknown (no colour; planner infers sides)
        DeclareLaunchArgument('use_gpu', default_value='true'),   # false: LiDAR cone pipeline on the CPU (PCL), for load comparisons
        Node(package='lidar_cone_detector', executable='lidar_perception_node', name='lidar_perception_node', output='log',
             parameters=[{'gpu_voxel': False, 'leaf_size': 0.05, 'ground_threshold': 0.04, 'cluster_tolerance': 0.3,
                          'min_cluster_size': 1, 'far_min_cluster_size': 1, 'max_cluster_size': 50, 'cluster_2d': True,
                          'height_min': 0.02, 'enable_motion_distortion': False,
                          'use_gpu': ParameterValue(LC('use_gpu'), value_type=bool)}]),
        Node(package='mfe_validation', executable='slam_node', name='slam_node', output='screen',
             parameters=[{'cones_topic': LC('cones_topic'), 'color_mode': LC('color_mode'), 'min_obs_publish': ParameterValue(LC('min_obs'), value_type=int),
                          'max_range': ParameterValue(LC('max_range'), value_type=float), 'publish_hz': ParameterValue(LC('publish_hz'), value_type=float),
                          'freeze': ParameterValue(LC('freeze'), value_type=bool)}]),
        Node(package='mfe_validation', executable='skidpad_planner', name='skidpad_planner', output='screen', condition=IfCondition(LC('skidpad')),
             parameters=[{'registration_mode': LC('registration_mode')}]),
    ])
