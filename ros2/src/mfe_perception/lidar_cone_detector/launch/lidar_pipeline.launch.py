# Launch file for the LiDAR pipeline
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
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

    use_sim_arg = DeclareLaunchArgument(
        'use_sim', 
        default_value='False',
        description="Whether to load lidar topics from a simulation"
    )

    load_file_value = LaunchConfiguration('load_file')
    use_sim_value = LaunchConfiguration('use_sim')

    file_loader_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lidar_cone_detector'),
                'launch',
                'file_loader.launch.py'
            )
        ),
        condition=IfCondition(load_file_value)  # Check the value at runtime
    )

    ground_plane_removal_node_sim = Node(
        package='lidar_cone_detector',
        namespace='lidar',
        name='ground_plane_removal_node',
        executable='ground_plane_removal',
        output='screen',
        emulate_tty=True,
        parameters=[
            {"run_visualization": False}
        ],
        remappings=[
            ('pcl/raw', '/fsds/lidar/Lidar1')
        ],
        condition=IfCondition(use_sim_value)
    )

    ground_plane_removal_node = Node(
        package='lidar_cone_detector',
        namespace='lidar',
        name='ground_plane_removal_node',
        executable='ground_plane_removal',
        output='screen',
        emulate_tty=True,
        parameters=[
            {"run_visualization": False}
        ],
        condition=UnlessCondition(use_sim_value)
    )

    cone_detector_node = Node(
        package='lidar_cone_detector',
        namespace='lidar',
        name='cone_detector_node',
        executable='cone_detector_node.py',
        output='screen',
        emulate_tty=True,
        arguments=[
            '--verbose'
        ]
    )

    return LaunchDescription([
        load_file_arg,
        use_sim_arg,
        file_loader_node_launch,
        ground_plane_removal_node_sim,
        ground_plane_removal_node,
        cone_detector_node
    ])