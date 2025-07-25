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
                ('cloud_in', '/sim_lidar_node/data'),
                ('scan', '/lidar/laserscan')
            ],
            parameters=[{
                'target_frame':       'fsds/Lidar1',
                'transform_tolerance': 0.05,
                'min_height':         -1.3,    # ignore points below this (m) (has to be negative)
                'max_height':         0.0,    # ignore points above this (m) (has to be negative)
                'angle_min':         -3.14,   # start angle of the scan [rad]
                'angle_max':          3.14,   # end angle of the scan [rad]
                'scan_time':          0.07,  # expected time between scans [s]
                'range_min':          1.0,    # minimum valid range [m]
                'range_max':          30.0,    # maximum valid range [m]
                'use_inf':           True
            }],
            arguments=['--ros-args', '--log-level', 'DEBUG']
        )
    ])