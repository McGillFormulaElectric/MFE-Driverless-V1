import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    manual_mode = LaunchConfiguration("manual_mode")
    manual_mode_launch_arg = DeclareLaunchArgument(
        "manual_mode",
        default_value="False"
    )

    remap_frames = LaunchConfiguration("remap_frames")
    remap_frames_launch_arg = DeclareLaunchArgument(
        "remap_frames",
        default_value="True"
    )

    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'FS/fs_car', 'base_link']
    )

    return LaunchDescription(
        tf_node
    )