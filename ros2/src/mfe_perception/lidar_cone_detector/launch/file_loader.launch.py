# Launch file for the LiDAR pipeline
import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    file_loader_node = Node(
        package='lidar_cone_detector',
        executable='file_loader',
        name='file_loader_node',
        output='screen',
        parameters=[
            {"run_visualization", False},
            {"timeout", 100},
            {"time_interval", 100},
            {'dirname': '/dataset/points'} # include the path to the chalmers dataset
        ]
    )
    
    return LaunchDescription([
        file_loader_node,
    ])