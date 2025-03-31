# Launch file for the LiDAR pipeline
import os

from launch import LaunchDescription
from launch_ros.actions import Node

import yaml

def generate_launch_description():
    
    ground_plane_removal_node = Node(
        package='lidar_cone_detector',
        executable='ground_plane_removal',
        output='screen',
        emulate_tty=True,
        parameters=[
            {"run_visualization": False},
            {"timeout", 100},
            {"time_interval", 100},
            {"dirname", "/dataset/points"}
        ]
    )
    
    return LaunchDescription([ground_plane_removal_node])