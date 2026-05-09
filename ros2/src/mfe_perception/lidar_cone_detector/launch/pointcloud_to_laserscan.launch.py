from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # TODO: Create LaunchConfiguration based on if using sim or not
    # TODO: Create LaunchConfiguration based on current customizable target_frame

    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            name='pointcloud_to_laserscan',
            namespace='lidar',
            executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', 'points_raw'),   # /lidar/points_raw (bridged from /velodyne_points)
                ('scan', 'pcl/laserscan')
            ],
            parameters=[{
                'target_frame':        'velodyne',  # EUFS sim VLP-16 frame
                'transform_tolerance':  0.1,
                'min_height':          -0.5,   # below LiDAR mount — captures cone bodies
                'max_height':           0.5,   # above LiDAR mount
                'angle_min':           -3.14,
                'angle_max':            3.14,
                'scan_time':            0.1,   # matches VLP-16 10 Hz
                'range_min':            0.5,
                'range_max':           30.0,
                'use_inf':             True
            }]
        )
    ])