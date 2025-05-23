# Launch file for the LiDAR pipeline
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    file_loader_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lidar_cone_detector'),
                'launch',
                'file_loader.launch.py'
            )
        ),
        # condition=IfCondition()  # Check the value at runtime
    )
    
    # Launch RANSAC Node
    ground_plane_removal_node = Node(
        package='lidar_cone_detector',
        namespace='lidar',
        executable='ground_plane_removal',
        output='screen',
        emulate_tty=True,
        parameters=[
            {"run_visualization": False}
        ]
    )


    return LaunchDescription([
        file_loader_node_launch,
        ground_plane_removal_node
    ])