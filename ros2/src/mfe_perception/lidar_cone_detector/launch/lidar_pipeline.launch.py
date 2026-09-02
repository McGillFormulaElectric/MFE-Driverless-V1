# Launch file for the LiDAR pipeline
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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
            {'cluster_tolerance': 0.4},
            {'min_cluster_size': 3},
            {'max_cluster_size': 150},
        ],
        remappings=[
            ('/lidar/points_raw', '/velodyne_points'),
        ],
    )

    # Convert to LaserScan for use in 2D Ceres Solver slam_toolbox (Graph-SLAM based)
    launch_pc_to_ls = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([ 
                FindPackageShare('lidar_cone_detector'),
                'launch',
                'pointcloud_to_laserscan.launch.py'
            ])
        ])
    ) 

    # transform detected cones to universal frame
    cone_transformer_node = Node(
        package='lidar_cone_detector',
        namespace='lidar',
        name='cone_transformer_node',
        executable='cone_transformer_node',
        output='screen',
        parameters=[
            {'target_frame': 'universal'}, # temp target frame
        ]
    )

    lidar_tf_broadcaster_node = Node(
        package='lidar_cone_detector',
        namespace='lidar',
        name='lidar_tf_broadcaster_node',
        executable='lidar_tf_broadcaster'
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
        # cone_transformer_node, # FOR SOME REASON THIS LOADS THE FILE. NO IDEA
        # lidar_tf_broadcaster_node,
        # launch_pc_to_ls,
    ])
