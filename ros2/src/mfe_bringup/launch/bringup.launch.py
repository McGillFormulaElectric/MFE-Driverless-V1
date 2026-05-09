"""
MFE Driverless — full hardware bringup launch file.
Target: Jetson Orin Nano
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# finish_x per mission: position (from odometry) at which finish_detector triggers stop
_FINISH_X = {
    'autocross':    100.0,
    'trackdrive':   100.0,
    'acceleration':  78.0,  # orange finish gate at x=78 m on EUFS acceleration track
    'skidpad':       37.0,  # end of exit corridor on EUFS skidpad track
}


def _make_vision_node(model_path_str: str) -> list:
    """
    Return a list of launch actions for the vision cone detector.

    If model_path is empty the node is skipped entirely (it would crash on startup
    because YOLO(model_path) is called unconditionally in __init__).
    """
    if not model_path_str:
        return [LogInfo(msg=(
            '[bringup] vision_cone_detection_node SKIPPED — '
            'model_path is empty. Pass model_path:=<path/to/weights.pt> to enable.'
        ))]

    return [Node(
        package='vision_cone_detector',
        executable='cone_detection_node',
        name='cone_detection_node',
        output='screen',
        parameters=[{
            'model_path': model_path_str,
            'depth_callback': False,
        }],
        remappings=[
            # vision detector subscribes to 'image/input'; camera publishes /camera/image_raw
            ('image/input', '/camera/image_raw'),
        ],
    )]


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

    vision_model_arg = DeclareLaunchArgument(
        'vision_model_path',
        default_value=os.path.expanduser('~/mfe_models/yolo/yolov8s/weights/best.pt'),
        description=(
            'Absolute path to YOLO weights file for the vision cone detector. '
            'Leave empty to skip. Defaults to trained yolov8s weights.'
        ),
    )

    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='/ekf/output',
        description=(
            'Odometry topic used by path planner and pure pursuit for vehicle pose. '
            'In simulation pass pose_topic:=/ground_truth/state_odom for a consistent '
            'frame with the Gazebo world / TF map frame.'
        ),
    )

    use_perception_arg = DeclareLaunchArgument(
        'use_perception',
        default_value='true',
        description=(
            'Launch boundary_extractor (LiDAR + camera fusion). '
            'Set false in no_perception sim mode to avoid dual publishing on /planning/cones.'
        ),
    )

    mission = LaunchConfiguration('mission')
    vision_model_path = LaunchConfiguration('vision_model_path')
    pose_topic = LaunchConfiguration('pose_topic')
    use_perception = LaunchConfiguration('use_perception')

    # NOTE: Static TF publishers (base_footprint → velodyne, base_footprint → zed_camera_center)
    # are published by mfe_eufs_sim.launch.py in simulation.  In hardware mode add them here.

    # --------------------------------------------------------------------------
    # Perception — LiDAR cone detector (C++)
    # --------------------------------------------------------------------------
    # NOTE: CUDA support is a compile-time flag (#ifdef PLATFORM_JETSON).
    # There is no runtime 'enable_cuda' parameter — the node ignores it.
    # To enable CUDA, rebuild with -DPLATFORM_JETSON=ON.
    lidar_perception_node = Node(
        package='lidar_cone_detector',
        executable='lidar_perception_node',
        name='lidar_perception_node',
        output='screen',
        parameters=[{
            'leaf_size': 0.1,           # coarser voxel fine for 16-beam VLP-16
            'ground_threshold': 0.15,   # 5 cm extra margin so cone bases survive RANSAC
            'cluster_tolerance': 0.3,   # tighter than default 0.4 — prevents adjacent cone merges
            'min_cluster_size': 1,      # VLP-16 gets 1-2 pts per cone at 10 m; default 3 drops all
            'max_cluster_size': 50,     # cones won't produce 150 pts; tighter cap
        }],
    )

    # --------------------------------------------------------------------------
    # Perception — Vision cone detector (Python/YOLO)
    # Launched conditionally via OpaqueFunction — skipped if vision_model_path=''.
    # Publishes mfe_msgs/Track to 'image/track' → consumed by boundary_extractor.
    # --------------------------------------------------------------------------
    vision_node_action = OpaqueFunction(
        function=lambda context: _make_vision_node(
            context.launch_configurations.get('vision_model_path', '')
        )
    )

    # --------------------------------------------------------------------------
    # Perception — PointCloud2 → LaserScan conversion (feeds SLAM toolbox)
    # Input:  lidar/pcl/objects  (PointCloud2 from lidar_cone_detector)
    # Output: /lidar/pcl/laserscan (LaserScan consumed by slam_toolbox)
    # --------------------------------------------------------------------------
    pc2scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lidar_cone_detector'),
                'launch',
                'pointcloud_to_laserscan.launch.py',
            )
        )
    )

    # --------------------------------------------------------------------------
    # Mapping — SLAM Toolbox (async, lifecycle-managed)
    # Subscribes: /lidar/pcl/laserscan (LaserScan)
    # Publishes:  /map (OccupancyGrid), TF map → odom
    # --------------------------------------------------------------------------
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mfe_mapping'),
                'launch',
                'slam_toolbox.launch.py',
            )
        )
    )

    # --------------------------------------------------------------------------
    # State estimation — Extended Kalman Filter (custom MFE node)
    # Subscribes: /imu/data (Imu), /gps (NavSatFix)
    # Publishes:  /ekf/output (nav_msgs/Odometry)
    # --------------------------------------------------------------------------
    ekf_params_file = os.path.join(
        get_package_share_directory('mfe_state_estimation'), 'config', 'ekf_params.yaml'
    )
    ekf_node = Node(
        package='mfe_state_estimation',
        executable='extended_kalman_filter_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_params_file],
        remappings=[('/imu/data', '/imu')],
    )

    # --------------------------------------------------------------------------
    # Path planning — Boundary extractor (LiDAR + camera fusion)
    # --------------------------------------------------------------------------
    boundary_extractor_node = Node(
        package='mfe_path_planning',
        executable='boundary_extractor',
        name='boundary_extractor',
        output='screen',
        condition=IfCondition(use_perception),
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
        remappings=[('/ekf/output', pose_topic)],
    )

    # --------------------------------------------------------------------------
    # Control — Pure pursuit
    # --------------------------------------------------------------------------
    # Pure pursuit speed/lookahead are mission-dependent: skidpad needs lower speed
    # for the tight 9.1m radius circles; other missions use the full 10 m/s.
    def _make_pure_pursuit(context):
        mission_str = context.launch_configurations.get('mission', 'autocross')
        pt = context.launch_configurations.get('pose_topic', '/ekf/output')
        if mission_str == 'skidpad':
            params = [{'max_speed': 4.5, 'lookahead_distance': 3.0, 'speed_reduction_factor': 0.6}]
        elif mission_str == 'acceleration':
            params = [{'max_speed': 13.0, 'lookahead_distance': 8.0, 'speed_reduction_factor': 0.2}]
        else:  # autocross / trackdrive
            params = [{'max_speed': 7.0, 'lookahead_distance': 3.5, 'max_lateral_accel': 5.0, 'speed_reduction_factor': 0.3}]
        return [Node(
            package='mfe_control',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=params,
            remappings=[('/ekf/output', pt)],
        )]

    pure_pursuit_node = OpaqueFunction(function=_make_pure_pursuit)

    # --------------------------------------------------------------------------
    # Mission control — Finish-line detector
    # Parameters (approach_x, finish_x) are mission-dependent via OpaqueFunction.
    # --------------------------------------------------------------------------
    def _make_finish_detector(context):
        mission_str = context.launch_configurations.get('mission', 'autocross')
        fx = _FINISH_X.get(mission_str, 100.0)
        ax = max(0.0, fx - 20.0)   # start LiDAR watch 20 m before finish gate
        pt = context.launch_configurations.get('pose_topic', '/ekf/output')
        # Closed-loop tracks (autocross/trackdrive) use return-to-start detection:
        # once min_travel_m is covered, fire when the car comes back within
        # return_to_start_r metres of its start position.
        # Linear tracks (acceleration, skidpad) keep the original x-position logic.
        if mission_str in ('autocross', 'trackdrive'):
            extra = {'min_travel_m': 60.0, 'return_to_start_r': 5.0}
        else:
            extra = {}
        return [Node(
            package='mfe_path_planning',
            executable='finish_detector_node',
            name='finish_detector',
            output='screen',
            parameters=[{
                'approach_x': ax,
                'finish_x': fx,
                'detect_fwd_min_m': 1.5,
                'detect_fwd_max_m': 15.0,
                'detect_lateral_m': 4.0,
                'min_finish_points': 1,
                **extra,
            }],
            remappings=[('/ekf/output', pt)],
        )]

    finish_detector_node = OpaqueFunction(function=_make_finish_detector)

    # --------------------------------------------------------------------------
    # Evaluation — Perception accuracy evaluator
    # Compares /planning/cones vs /ground_truth/cones_colored (sim only).
    # Publishes diagnostic_msgs/DiagnosticArray to /perception/accuracy at 1 Hz.
    # Logs CSV to ~/mfe_logs/perception_accuracy.csv.
    # --------------------------------------------------------------------------
    evaluator_node = Node(
        package='perception_evaluator',
        executable='evaluator_node',
        name='perception_evaluator',
        output='screen',
    )

    # --------------------------------------------------------------------------
    # Assemble description
    # --------------------------------------------------------------------------
    return LaunchDescription([
        # args first
        mission_arg,
        vision_model_arg,
        pose_topic_arg,
        use_perception_arg,

        # perception
        lidar_perception_node,
        vision_node_action,    # skipped unless vision_model_path is set
        pc2scan_launch,        # PointCloud2 → LaserScan for SLAM

        # mapping
        slam_launch,

        # state estimation
        ekf_node,

        # planning
        boundary_extractor_node,
        path_planner_node,

        # control
        pure_pursuit_node,
        finish_detector_node,

        # evaluation (sim only)
        evaluator_node,
    ])
