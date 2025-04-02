# Launch file for the Camera pipeline
import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    dirname = os.path.join(
        get_package_share_directory('vision_cone_detector'),
        'resource/video.mp4'
    )

    file_loader_node = Node(
        package='vision_cone_detector',
        executable='file_loader',
        name='file_loader_node',
        output='screen',
        parameters=[
            {"run_visualization": False},
            {"timeout": 100.0},
            {"time_interval": 100.0},
            {'dirname': dirname }
        ]
    )
    
    return LaunchDescription([
        file_loader_node
    ])