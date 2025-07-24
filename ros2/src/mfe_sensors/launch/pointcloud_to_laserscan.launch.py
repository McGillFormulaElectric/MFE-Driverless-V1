from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            name='pointcloud_to_laserscan',
            namespace='',
            executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', '/fsds/lidar/Lidar1'),
                ('scan', '/lidar/laserscan')
            ],
            parameters=[{
                'target_frame':       'fsds/FSCar',
                'transform_tolerance': 0.1,
                'min_height':         -1.3,    # ignore points below this (m)
                'max_height':         1.0,    # ignore points above this (m)
                'angle_min':         -3.14,   # start angle of the scan [rad]
                'angle_max':          3.14,   # end angle of the scan [rad]
                'angle_increment':    0.004363, # resolution of the scan [rad]
                'scan_time':          0.1,  # expected time between scans [s]
                'range_min':          0.5,    # minimum valid range [m]
                'range_max':         30.0,     # maximum valid range [m]
                'use_inf':           True,
                'inf_epsilon':        1.0
            }]
        )
    ])