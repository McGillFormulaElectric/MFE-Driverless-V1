"""
MFE Driverless — full hardware bringup launch file.

Supports single-machine, two-machine, and three-machine deployments:

  Single machine (sim / dev):
    ros2 launch mfe_bringup bringup.launch.py

  Host PC — sim + perception only (raw data stays local):
    ros2 launch mfe_bringup perception.launch.py

  Perception Jetson (real car — Jetson 1, wired to sensors):
    ros2 launch mfe_bringup perception.launch.py

  Compute Jetson (real car — Jetson 2, or any Tailscale peer):
    ros2 launch mfe_bringup compute.launch.py

All machines must share ROS_DOMAIN_ID and CYCLONEDDS_URI pointing at
config/cyclone_tailscale.xml (or be on the same LAN for multicast).
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
)
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
    'peanut':       100.0,  # unused — peanut uses return-to-start lap counting
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
        choices=['autocross', 'trackdrive', 'acceleration', 'skidpad', 'peanut'],
        description='FSAE mission type passed to path planner',
    )

    vision_model_arg = DeclareLaunchArgument(
        'vision_model_path',
        default_value='',
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

    endless_arg = DeclareLaunchArgument(
        'endless',
        default_value='false',
        description=(
            'Disable finish detection so the car loops indefinitely. '
            'Useful for multi-lap testing and Foxglove visualization.'
        ),
    )

    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='true',
        description=(
            'Launch SLAM toolbox (publishes map→odom TF + /map). '
            'Set false in sim — EUFS GT TF already provides map→odom; two publishers conflict.'
        ),
    )

    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf',
        default_value='true',
        description=(
            'Launch the GPS+IMU EKF node. '
            'Set false in sim when using pose_topic:=/ground_truth/state_odom '
            '(EKF GPS-origin frame does not align with the Gazebo world/TF map frame).'
        ),
    )

    num_laps_arg = DeclareLaunchArgument(
        'num_laps',
        default_value='1',
        description=(
            'Number of laps before the orange-cone finish gate triggers a stop. '
            'On closed-loop tracks (autocross/peanut) each return to the start position '
            'counts as one lap. Ignored when endless:=true.'
        ),
    )

    run_perception_arg = DeclareLaunchArgument(
        'run_perception',
        default_value='true',
        description=(
            'Launch perception nodes (LiDAR, vision, boundary extractor, SLAM, EKF). '
            'Set false on a compute-only node (Jetson 2) that receives processed cones '
            'from a perception node (Jetson 1 / host) over the network.'
        ),
    )

    run_compute_arg = DeclareLaunchArgument(
        'run_compute',
        default_value='true',
        description=(
            'Launch planning and control nodes (path planner, pure pursuit, finish detector). '
            'Set false on a perception-only node so compute runs on a separate machine.'
        ),
    )

    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='0.0',
        description='Override pure pursuit max_speed (m/s). 0 = use mission default.',
    )

    mission = LaunchConfiguration('mission')
    vision_model_path = LaunchConfiguration('vision_model_path')
    pose_topic = LaunchConfiguration('pose_topic')
    use_perception = LaunchConfiguration('use_perception')
    endless = LaunchConfiguration('endless')
    use_slam = LaunchConfiguration('use_slam')
    use_ekf = LaunchConfiguration('use_ekf')
    run_perception = LaunchConfiguration('run_perception')
    run_compute = LaunchConfiguration('run_compute')
    max_speed = LaunchConfiguration('max_speed')

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
            'ground_threshold': 0.05,   # tight: only removes flat ground ±5 cm; larger values
                                        # eat cone-body returns at the RANSAC inlier boundary.
                                        # Sim VLP-16 ±1.33° beams: 0.15 m threshold limits
                                        # detection to <6.5 m; 0.05 m extends it to ~10.8 m.
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
    # Input:  /lidar/points_raw (PointCloud2)
    # Output: /lidar/pcl/laserscan (LaserScan consumed by slam_toolbox)
    # Only launched when use_slam:=true.
    # --------------------------------------------------------------------------
    pc2scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lidar_cone_detector'),
                'launch',
                'pointcloud_to_laserscan.launch.py',
            )
        ),
        condition=IfCondition(use_slam),
    )

    # --------------------------------------------------------------------------
    # Mapping — SLAM Toolbox (async, lifecycle-managed)
    # Subscribes: /lidar/pcl/laserscan (LaserScan)
    # Publishes:  /map (OccupancyGrid), TF map → odom
    # Disabled in sim (use_slam:=false) — EUFS GT TF already publishes map→odom.
    # --------------------------------------------------------------------------
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mfe_mapping'),
                'launch',
                'slam_toolbox.launch.py',
            )
        ),
        condition=IfCondition(use_slam),
    )

    # --------------------------------------------------------------------------
    # State estimation — Extended Kalman Filter (custom MFE node)
    # Subscribes: /imu/data (Imu), /gps (NavSatFix)
    # Publishes:  /ekf/output (nav_msgs/Odometry)
    # Disabled in sim (use_ekf:=false) when pose_topic:=/ground_truth/state_odom —
    # EKF GPS origin ≠ Gazebo world frame so EKF pose cannot be used with TF-derived cones.
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
        condition=IfCondition(use_ekf),
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
        # Base vehicle constants (from vehicle.yaml — inlined here because vehicle.yaml
        # uses a 'vehicle:' namespace, not a valid ROS2 params-file format)
        params = {
            'wheelbase': 1.56,
            'max_steering_deg': 28.0,
            'max_lateral_accel': 8.0,
            'max_deceleration': 10.0,
        }
        if mission_str == 'skidpad':
            params.update({'max_speed': 4.5, 'lookahead_distance': 3.0, 'speed_reduction_factor': 0.6})
        elif mission_str == 'acceleration':
            params.update({'max_speed': 13.0, 'lookahead_distance': 8.0, 'speed_reduction_factor': 0.2})
        elif mission_str == 'peanut':
            # Tighter lookahead + lower speed for the figure-8 crossing pinch
            params.update({'max_speed': 6.0, 'lookahead_distance': 3.5, 'speed_reduction_factor': 0.5})
        else:  # autocross / trackdrive
            params.update({'max_speed': 8.0, 'lookahead_distance': 5.0, 'speed_reduction_factor': 0.3})
        max_speed_override = float(context.launch_configurations.get('max_speed', '0.0'))
        if max_speed_override > 0.0:
            params['max_speed'] = max_speed_override
        return [Node(
            package='mfe_control',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[params],
            remappings=[('/ekf/output', pt)],
        )]

    pure_pursuit_node = OpaqueFunction(function=_make_pure_pursuit)

    # --------------------------------------------------------------------------
    # Mission control — Finish-line detector
    # Parameters (approach_x, finish_x) are mission-dependent via OpaqueFunction.
    # --------------------------------------------------------------------------
    def _make_finish_detector(context):
        if context.launch_configurations.get('endless', 'false').lower() == 'true':
            return [LogInfo(msg='[bringup] finish_detector DISABLED — endless mode, car loops indefinitely')]
        mission_str = context.launch_configurations.get('mission', 'autocross')
        num_laps = int(context.launch_configurations.get('num_laps', '1'))
        fx = _FINISH_X.get(mission_str, 100.0)
        ax = max(0.0, fx - 20.0)   # start LiDAR watch 20 m before finish gate
        pt = context.launch_configurations.get('pose_topic', '/ekf/output')
        # Closed-loop tracks use return-to-start lap counting.
        # Linear tracks (acceleration, skidpad) keep x-position logic — num_laps ignored.
        if mission_str in ('autocross', 'trackdrive', 'peanut'):
            extra = {'min_travel_m': 60.0, 'return_to_start_r': 5.0, 'num_laps': num_laps}
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
    # Evaluation — Perception accuracy evaluators (sim only)
    # evaluator_node:        end-to-end — /planning/cones vs GT (→ /perception/accuracy)
    # fusion_evaluator_node: raw LiDAR+camera fused vs GT (→ /perception/fusion_accuracy)
    # Both require /ground_truth/cones_colored which is only published by the sim bridge.
    # --------------------------------------------------------------------------
    evaluator_node = Node(
        package='perception_evaluator',
        executable='evaluator_node',
        name='perception_evaluator',
        output='screen',
    )

    fusion_evaluator_node = Node(
        package='perception_evaluator',
        executable='fusion_evaluator_node',
        name='fusion_evaluator',
        output='screen',
    )

    # --------------------------------------------------------------------------
    # Assemble description
    # --------------------------------------------------------------------------
    perception_group = GroupAction(
        condition=IfCondition(run_perception),
        actions=[
            lidar_perception_node,
            vision_node_action,         # skipped unless vision_model_path is set
            pc2scan_launch,             # PointCloud2 → LaserScan for SLAM
            slam_launch,
            ekf_node,
            boundary_extractor_node,
            evaluator_node,
            fusion_evaluator_node,
        ],
    )

    lap_validator_node = Node(
        package='mfe_path_planning',
        executable='lap_validator_node',
        name='lap_validator',
        output='screen',
    )

    compute_group = GroupAction(
        condition=IfCondition(run_compute),
        actions=[
            path_planner_node,
            pure_pursuit_node,
            finish_detector_node,
            lap_validator_node,
        ],
    )

    return LaunchDescription([
        # args
        mission_arg,
        vision_model_arg,
        pose_topic_arg,
        use_perception_arg,
        endless_arg,
        num_laps_arg,
        use_slam_arg,
        use_ekf_arg,
        run_perception_arg,
        run_compute_arg,
        max_speed_arg,

        # node groups — each independently enable/disable-able
        perception_group,
        compute_group,
    ])
