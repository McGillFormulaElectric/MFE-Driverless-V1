# Launch file for the LiDAR pipeline
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    load_file_value = LaunchConfiguration('load_file')
    sliding_window_value = LaunchConfiguration('sliding_window_acc')

    file_loader_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lidar_cone_detector'),
                'launch',
                'file_loader.launch.py'
            )
        ),
        condition=IfCondition(load_file_value)  # Check the value at runtime
    )

    # Optional: Velocity compensation using sliding window for point accumulation
    sliding_window_preprocessor_node = Node(
        package='lidar_cone_detector',
        namespace='lidar',
        name='lidar_preprocessor',
        executable='lidar_preprocessor',
        parameters=[
            # {'lidar_frame': 'lidar_base'} # basic lidar frame
        ],
        # remappings=[
        # Old remapping: ('pcl/raw', '/mfe_sensors/lidar/data')
        # ],
        condition=IfCondition(sliding_window_value)
    )

    # Unified GPU perception pipeline: filter + ground removal + clustering + centroids
    lidar_perception_node = Node(
        package='lidar_cone_detector',
        executable='lidar_perception_node',
        name='lidar_perception_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'lidar_frame_id': 'velodyne'},
            {'ground_threshold': 0.1},
            {'leaf_size': 0.05},
            {'cluster_tolerance': 0.2},
            {'min_cluster_size': 5},
            {'max_cluster_size': 50},
            {'min_intensity': 0.0},   # set to ~100.0 on real VLP-16 (retroreflective tape); keep 0 in sim
        ],
        remappings=[
            ('/lidar/points_raw', '/velodyne_points'),
        ],
    )

    load_file_arg = DeclareLaunchArgument(
        'load_file',
        default_value='False',
        description='Whether to launch the file loader node'
    )

    sliding_window_arg = DeclareLaunchArgument(
        'sliding_window_acc', 
        default_value='False',
        description="Whether to load lidar topics from AMZ Bag"
    )

    return LaunchDescription([
        load_file_arg,
        sliding_window_arg,
        file_loader_node,
        sliding_window_preprocessor_node,
        lidar_perception_node,
    ])
