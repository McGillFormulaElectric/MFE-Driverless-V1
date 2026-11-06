"""
MFE Driverless — Intel RealSense D435i camera launch.

Colour image on /camera/image_raw, the topic the YOLO cone detector reads. Package:
realsense2_camera (not in the Jetson image yet; bringup fails early when it is missing).

Run standalone for bench testing:
  ros2 launch mfe_sensors realsense.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_no', default_value='',
            description='D435i serial number. Empty: first device found.',
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='',
            output='screen',
            parameters=[{
                'serial_no': LaunchConfiguration('serial_no'),
                'enable_color': True,
                'enable_depth': False,
                'rgb_camera.profile': '640x480x30',
            }],
            remappings=[('/camera/color/image_raw', '/camera/image_raw')],
        ),
    ])
