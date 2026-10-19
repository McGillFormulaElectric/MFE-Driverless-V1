"""Validation stack, run alongside a sim whose main planner uses ground-truth cones (launch_sim_local.sh ... hybrid).

Usage: ros2 launch mfe_validation validation.launch.py mission:=acceleration track_csv:=<path>
Nothing here publishes /cmd or /planning/*, so it cannot affect the car.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mission', default_value='acceleration'),
        DeclareLaunchArgument('track_csv', default_value=''),
        DeclareLaunchArgument('log_csv', default_value=''),
        # CPU voxel: faster than GPU for this workload.
        Node(package='lidar_cone_detector', executable='lidar_perception_node', name='validation_lidar_perception', output='log',
             parameters=[{'gpu_voxel': False, 'leaf_size': 0.05, 'ground_threshold': 0.04, 'cluster_tolerance': 0.3,
                          'min_cluster_size': 1, 'far_min_cluster_size': 1, 'max_cluster_size': 50, 'cluster_2d': True,
                          'height_min': 0.02}]),
        Node(package='mfe_validation', executable='slam_node', name='slam_node', output='screen',
             parameters=[{'track_csv': LC('track_csv')}]),
        # Second planner: same code as the main one, fed by perception + SLAM; outputs under /validation.
        Node(package='mfe_path_planning', executable='path_planner_node', name='validation_planner', output='log',
             parameters=[{'mission': LC('mission')}],
             remappings=[('/planning/cones', '/validation/cones'), ('/ekf/output', '/slam/odom'),
                         ('/planning/centerline', '/validation/centerline'),
                         ('/planning/track_left', '/validation/track_left'), ('/planning/track_right', '/validation/track_right')]),
        Node(package='mfe_validation', executable='compare_node', name='compare_node', output='screen',
             parameters=[{'log_csv': LC('log_csv')}]),
    ])
