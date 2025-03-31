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
    # Loads argument "load_file" to determine whether to load with file
    load_file_arg = DeclareLaunchArgument(
        'load_file',
        default_value=False,
        description='Whether to launch the file loader node'
    )

    load_file_value = LaunchConfiguration('load_file')

    if (load_file_value):
        # Launch the File Loader by using file_load.launch.py
        file_loader_node_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('lidar_cone_detector'),
                    'launch',
                    'file_loader.launch.py'
                )
            )
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
        load_file_arg,
        file_loader_node_launch,
        ground_plane_removal_node
    ])