"""
MFE Driverless — full hardware bringup launch file.
Target: Jetson Orin Nano
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # --------------------------------------------------------------------------
    # Launch arguments
    # --------------------------------------------------------------------------
    mission_arg = DeclareLaunchArgument(
        'mission',
        default_value='autocross',
        choices=['autocross', 'trackdrive', 'acceleration', 'skidpad'],
        description='FSAE mission type passed to path planner',
    )

    mission = LaunchConfiguration('mission')

    # --------------------------------------------------------------------------
    # Static TF publishers
    # --------------------------------------------------------------------------
    # base_footprint -> velodyne  (0, 0, 0.3 m above footprint)
    tf_base_to_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_velodyne',
        output='screen',
        arguments=['0', '0', '0.3', '0', '0', '0', 'base_footprint', 'velodyne'],
    )

    # base_footprint -> camera_base  (0.3 m forward, 0.5 m above)
    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_camera',
        output='screen',
        arguments=['0.3', '0', '0.5', '0', '0', '0', 'base_footprint', 'camera_base'],
    )

    # --------------------------------------------------------------------------
    # Perception — LiDAR cone detector (C++)
    # --------------------------------------------------------------------------
    lidar_perception_node = Node(
        package='lidar_cone_detector',
        executable='lidar_perception_node',
        name='lidar_perception_node',
        output='screen',
        parameters=[{
            'leaf_size': 0.05,
            'ground_threshold': 0.1,
            'enable_cuda': True,
        }],
    )

    # --------------------------------------------------------------------------
    # Perception — Vision cone detector (Python/YOLO)
    # --------------------------------------------------------------------------
    vision_cone_detection_node = Node(
        package='vision_cone_detector',
        executable='cone_detection_node',
        name='cone_detection_node',
        output='screen',
        parameters=[{
            'model_path': '',
            'depth_callback': False,
        }],
        remappings=[
            ('image/input', '/camera/image_raw'),
        ],
    )

    # --------------------------------------------------------------------------
    # State estimation — Extended Kalman Filter
    # --------------------------------------------------------------------------
    ekf_node = Node(
        package='mfe_state_estimation',
        executable='extended_kalman_filter_node',
        name='ekf_node',
        output='screen',
    )

    # --------------------------------------------------------------------------
    # Path planning — Boundary extractor (LiDAR + camera fusion)
    # --------------------------------------------------------------------------
    boundary_extractor_node = Node(
        package='mfe_path_planning',
        executable='boundary_extractor',
        name='boundary_extractor',
        output='screen',
    )

    # --------------------------------------------------------------------------
    # Path planning — Path planner
    # --------------------------------------------------------------------------
    path_planner_node = Node(
        package='mfe_path_planning',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen',
        parameters=[{
            'mission': mission,
        }],
    )

    # --------------------------------------------------------------------------
    # Control — Pure pursuit
    # --------------------------------------------------------------------------
    pure_pursuit_node = Node(
        package='mfe_control',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
    )

    # --------------------------------------------------------------------------
    # Assemble description
    # --------------------------------------------------------------------------
    return LaunchDescription([
        # args first
        mission_arg,

        # static transforms
        tf_base_to_velodyne,
        tf_base_to_camera,

        # nodes in pipeline order
        lidar_perception_node,
        vision_cone_detection_node,
        ekf_node,
        boundary_extractor_node,
        path_planner_node,
        pure_pursuit_node,
    ])
