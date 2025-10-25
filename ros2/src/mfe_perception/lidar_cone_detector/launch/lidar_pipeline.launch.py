# Launch file for the LiDAR pipeline
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction

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

    # RANSAC Ground Plane Removal after downsampling in preprocessor 
    ground_plane_removal_node = Node(
        package='lidar_cone_detector',
        namespace='lidar',
        name='ground_plane_removal_node',
        executable='ground_plane_removal',
        output='screen',
        emulate_tty=True,
        parameters=[
            {"run_visualization": False},
            # {'lidar_frame': 'lidar_base'}, # basic lidar frame
        ],
        # Old remapping: ('pcl/input', '/mfe_sensors/lidar/data'), # Remap to pcl/acc_cloud if using sliding win.
    )

    # DBSCAN Unsupervised Point Clustering for Cone Detection
    cone_detector_node = Node(
        package='lidar_cone_detector',
        namespace='lidar',
        name='cone_detector_node',
        executable='cone_detector_node',
        output='screen',
        emulate_tty=True,
        arguments=[
            '--verbose'
        ]
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

    # OLD Static TF Broadcaster for Lidar to Map Frame (rviz visualization)
    # static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='lidar_tf_pub',
    #     arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'velodyne'],
    #     output='screen'
    # )


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
        ground_plane_removal_node,
        cone_detector_node,
        launch_pc_to_ls,
    ])