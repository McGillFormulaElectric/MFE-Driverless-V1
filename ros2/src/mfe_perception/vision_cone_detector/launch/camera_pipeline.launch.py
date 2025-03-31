# Launch file for the Camera pipeline
import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():    

    # Launch cone detection Node
    cone_detection_node = Node(
        package='vision_cone_detector',
        namespace='lidar',
        executable='camera_cone_node',
    )

    return LaunchDescription([
        cone_detection_node
    ])