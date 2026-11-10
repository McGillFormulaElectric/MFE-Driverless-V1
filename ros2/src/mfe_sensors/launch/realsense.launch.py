"""
MFE Driverless — Intel RealSense D435i camera launch.

Colour image on /camera/image_raw, the topic the YOLO cone detector reads. Depth and the IMU are
on by default (both can be turned off with launch arguments):
  /camera/depth/image_rect_raw                 depth, 640x480 @ 30
  /camera/aligned_depth_to_color/image_raw     depth aligned to the colour image (align_depth)
  /camera/imu                                  gyro + accel united (linear interpolation)
The camera IMU is published on /camera/imu, never /imu: /imu is the Xsens input to the EKF.
Package: realsense2_camera 4.5x (parameter names rgb_camera.color_profile and
depth_module.depth_profile; the older rgb_camera.profile is ignored by 4.58).

Run standalone for bench testing:
  ros2 launch mfe_sensors realsense.launch.py
  ros2 launch mfe_sensors realsense.launch.py enable_depth:=false enable_imu:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _bool(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_no', default_value='',
            description='D435i serial number. Empty: first device found.',
        ),
        DeclareLaunchArgument('enable_depth', default_value='true',
                              description='Depth stream, 640x480 @ 30.'),
        DeclareLaunchArgument('align_depth', default_value='true',
                              description='Publish depth aligned to the colour image (CPU cost).'),
        DeclareLaunchArgument('enable_imu', default_value='true',
                              description='Gyro + accel, united on /camera/imu.'),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='',
            output='screen',
            parameters=[{
                'serial_no': LaunchConfiguration('serial_no'),
                'enable_color': True,
                'rgb_camera.color_profile': '640,480,30',
                'enable_depth': _bool('enable_depth'),
                'depth_module.depth_profile': '640,480,30',
                'align_depth.enable': _bool('align_depth'),
                'enable_gyro': _bool('enable_imu'),
                'enable_accel': _bool('enable_imu'),
                'unite_imu_method': 2,
            }],
            remappings=[('/camera/color/image_raw', '/camera/image_raw')],
        ),
    ])
