"""
MFE Driverless — Xsens MTi-670G GNSS/INS driver launch.

Wraps the external xsens_mti_ros2_driver package (not vendored in this repo —
cloned + built as a workspace overlay by scripts/setup_jetson.sh, see
https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client, ros2 branch)
and remaps its output to the topics mfe_state_estimation's EKF node expects:

  imu/data  (sensor_msgs/Imu)      -> /imu
  gnss      (sensor_msgs/NavSatFix) -> /gps

Run standalone for bench testing:
  ros2 launch mfe_sensors xsens_mti.launch.py port:=/dev/ttyUSB0

Normally launched as part of bringup.launch.py's perception_group (real car only,
condition=use_ekf) — see mfe_bringup/launch/bringup.launch.py.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port the MTi-670G is connected on (add a udev rule for a stable symlink).',
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='MTi-670G serial baudrate.',
    )

    overrides_file = os.path.join(
        get_package_share_directory('mfe_sensors'), 'config', 'xsens_mti.yaml'
    )

    xsens_node = Node(
        package='xsens_mti_ros2_driver',
        executable='xsens_driver',
        name='xsens_mti_node',
        output='screen',
        parameters=[
            overrides_file,
            {
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
            },
        ],
        remappings=[
            ('imu/data', '/imu'),
            ('gnss', '/gps'),
        ],
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        xsens_node,
    ])
