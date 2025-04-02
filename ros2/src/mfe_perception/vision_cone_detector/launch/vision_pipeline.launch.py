# Launch file for the vision pipeline
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
        default_value='True',
        description='Whether to launch the file loader node'
    )

    load_file_value = LaunchConfiguration('load_file')

    file_loader_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('vision_cone_detector'),
                'launch',
                'file_loader.launch.py'
            )
        ),
        condition=IfCondition(load_file_value)  # Check the value at runtime
    )

    model_path = os.path.join(
        get_package_share_directory('vision_cone_detector'),
        'resource/best.pt'
    )
    
    # Launch cone detection node
    cone_detection_node = Node(
        package='vision_cone_detector',
        namespace='camera',
        executable='cone_detection',
        name='camera_cone_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {"run_visualization": False},
            {'model_path': model_path }
        ]
    )

    return LaunchDescription([
        load_file_arg,
        file_loader_node_launch,
        cone_detection_node 
    ])