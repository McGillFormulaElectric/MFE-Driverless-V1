"""
MFE Driverless — Velodyne VLP-16 driver launch.

Raw packets -> point cloud on /lidar/points_raw, the topic the LiDAR cone detector reads.
Packages: velodyne_driver and velodyne_pointcloud (ros-humble-velodyne in the Jetson image).
Frame "velodyne" matches the TF in mfe_bringup/config/sensor_mounts.yaml.

Run standalone for bench testing:
  ros2 launch mfe_sensors velodyne.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value='VLP16',
            description='Velodyne model (VLP16 for the car).',
        ),
        DeclareLaunchArgument(
            'device_ip', default_value='192.168.1.201',
            description='Sensor IP address (default factory address).',
        ),
        DeclareLaunchArgument(
            'port', default_value='2368',
            description='UDP port the sensor sends data packets to.',
        ),
        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            name='velodyne_driver',
            output='screen',
            parameters=[{
                'model': LaunchConfiguration('model'),
                'device_ip': LaunchConfiguration('device_ip'),
                'port': LaunchConfiguration('port'),
                'frame_id': 'velodyne',
            }],
        ),
        Node(
            package='velodyne_pointcloud',
            executable='velodyne_transform_node',
            name='velodyne_transform',
            output='screen',
            parameters=[{
                'model': LaunchConfiguration('model'),
                'frame_id': 'velodyne',
                # Required: without it the node throws std::runtime_error at startup.
                'calibration': os.path.join(get_package_share_directory('velodyne_pointcloud'),
                                            'params', 'VLP16db.yaml'),
                'min_range': 0.9,
                'max_range': 130.0,
            }],
            remappings=[('velodyne_points', '/lidar/points_raw')],
        ),
    ])
