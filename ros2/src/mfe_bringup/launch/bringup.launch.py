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

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import (
    get_package_prefix, get_package_share_directory, PackageNotFoundError,
)

from mfe_common.topics import CAN_WHEEL_SPEED, SLAM_ODOM

# Arguments passed from bringup to mfe_validation/launch/slam.launch.py for slam:=gtsam.
GTSAM_ARGS = ('slam_lateral_vel', 'color_mode', 'min_obs', 'max_range', 'publish_hz', 'freeze',
              'odom_topic')

# finish_x per mission: position (from odometry) at which finish_detector triggers stop
_FINISH_X = {
    'autocross':    100.0,
    'trackdrive':   100.0,
    'acceleration':  78.0,  # orange finish gate at x=78 m on EUFS acceleration track
    'skidpad':       30.0,  # 15 m past the crossing: brakes inside the stop zone (orange cones 9-21 m past it);
                            # 37 m braked at ~22 m and every run stopped past the last cones
    'peanut':       100.0,  # unused — peanut uses return-to-start lap counting
}


PLATFORMS = ('legacy', 'sim', 'car')
SENSORS = ('lidar', 'camera', 'imu_gps', 'can')

CAN_NOT_IMPLEMENTED = (
    'CAN interface not implemented (needs DBC); vehicle will not actuate. '
    'Control commands stay on /control/command.'
)


def _platform(context) -> str:
    return context.launch_configurations.get('platform', 'legacy')


def _is_car(context) -> bool:
    return _platform(context) == 'car'


def _start_drivers(context) -> bool:
    return context.launch_configurations.get('start_drivers', 'true').lower() == 'true'


def _sensor_set(context) -> set:
    raw = context.launch_configurations.get('sensors', ','.join(SENSORS))
    names = {name.strip() for name in raw.split(',') if name.strip()}
    unknown = names - set(SENSORS)
    if unknown:
        raise RuntimeError(f'unknown sensors {sorted(unknown)}; choose from {",".join(SENSORS)}')
    return names


# Sensor that owns each frame in config/sensor_mounts.yaml.
_FRAME_SENSOR = {'velodyne': 'lidar', 'camera_base': 'camera'}


def _require_package(package: str, purpose: str) -> None:
    try:
        get_package_prefix(package)
    except PackageNotFoundError:
        raise RuntimeError(
            f'[bringup] {purpose} needs ROS package "{package}", which is not installed. '
            f'Install it or drop the sensor from sensors:=.') from None


def _load_sensor_mounts() -> dict:
    path = os.path.join(get_package_share_directory('mfe_bringup'), 'config', 'sensor_mounts.yaml')
    with open(path) as f:
        return yaml.safe_load(f)


def _static_tf_node(parent: str, child: str, pose: dict) -> Node:
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{child}_tf_pub',
        arguments=['--x', str(pose['x']), '--y', str(pose['y']), '--z', str(pose['z']),
                   '--roll', str(pose['roll']), '--pitch', str(pose['pitch']),
                   '--yaw', str(pose['yaw']),
                   '--frame-id', parent, '--child-frame-id', child],
        output='screen',
    )


def _make_vision_node(model_path_str: str, camera: bool, car: bool) -> list:
    """
    Return a list of launch actions for the vision cone detector.

    The node is skipped without a camera, and when model_path is empty (it would crash on
    startup because YOLO(model_path) is called unconditionally in __init__).
    """
    if not camera:
        return [LogInfo(msg='[bringup] vision_cone_detection_node SKIPPED — no camera in sensors')]
    if not model_path_str:
        return [LogInfo(msg=(
            '[bringup] vision_cone_detection_node SKIPPED — '
            'model_path is empty. Pass model_path:=<path/to/weights.pt> to enable.'
        ))]

    node_params = {
        'model_path': model_path_str,
        'depth_callback': False,
    }
    if car:
        node_params['camera_frame'] = _load_sensor_mounts()['camera_frame']

    return [Node(
        package='vision_cone_detector',
        executable='cone_detection_node',
        name='cone_detection_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('vision_cone_detector'), 'config',
                         'cone_detection_node_params.yaml'),
            node_params,
        ],
        remappings=[
            # vision detector subscribes to 'image/input'; camera publishes /camera/image_raw
            ('image/input', '/camera/image_raw'),
        ],
    )]


def _resolve_derived_defaults(context):
    """Fill the empty-default arguments from platform and sensors. Explicit values win."""
    sensors = _sensor_set(context)
    car = _is_car(context)
    sim = _platform(context) == 'sim'
    use_ekf = context.launch_configurations.get('use_ekf') or (
        'true' if 'imu_gps' in sensors else 'false')
    derived = {
        'use_ekf': 'true' if 'imu_gps' in sensors else 'false',
        'slam': 'toolbox' if 'imu_gps' in sensors else 'gtsam',
        'odom_topic': ('/ekf/output' if car or (sim and use_ekf == 'true')
                       else '/sim/xsens/state_odom'),
    }
    for name, value in derived.items():
        if not context.launch_configurations.get(name):
            context.launch_configurations[name] = value
    if context.launch_configurations['slam'] not in ('toolbox', 'gtsam', 'none'):
        raise RuntimeError(f"slam must be toolbox, gtsam or none, not {context.launch_configurations['slam']!r}")
    return []


def _pose_topic(context) -> str:
    """Odometry topic for planner, pure pursuit and finish detector (slam:=gtsam: /slam/odom)."""
    if context.launch_configurations.get('slam', 'toolbox') == 'gtsam':
        return SLAM_ODOM
    return context.launch_configurations.get('pose_topic', '/ekf/output')


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

    skidpad_mode_arg = DeclareLaunchArgument(
        'skidpad_mode',
        default_value='cone_based',
        choices=['cone_based', 'hardcoded'],
        description=(
            'Skidpad path planning mode: cone_based (cone-detection-driven state machine) '
            'or hardcoded (pre-computed template path). Has no effect on non-skidpad missions.'
        ),
    )

    unknown_cone_mode_arg = DeclareLaunchArgument(
        'unknown_cone_mode',
        default_value='trace',
        description='Planning without cone colours: trace (wall tracer path), trace_backend (tracer labels, '
                    'backend plans), fasttube (ft-fsd sorts the uncoloured cones itself), heading (legacy).')
    planner_params_arg = DeclareLaunchArgument(
        'planner_params_file',
        default_value='',
        description='Optional ROS params YAML (absolute path) merged into path_planner_node, e.g. '
                    'pipeline_hysteresis, jump_gate, colour_vote, cone_expiry, planner_profile.')
    planner_backend_arg = DeclareLaunchArgument(
        'planner_backend',
        default_value='fasttube',
        choices=['fasttube'],
        description='Local path planner: fasttube (fsd_path_planning library).',
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
            'Odometry topic used by path planner, pure pursuit and finish detector for vehicle '
            'pose. Ignored when slam:=gtsam, which uses /slam/odom. '
            'In simulation pass pose_topic:=/sim/xsens/state_odom (same map-frame header as '
            'ground truth, but noise-corrupted to be representative of the real Xsens '
            'MTi-670G — see mfe_eufs_sim/xsens_noise_node.py). Do not use raw '
            '/ground_truth/state_odom as pose_topic — it is noiseless and hides pose error '
            'the real sensor would introduce.'
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

    sim_cones_direct_arg = DeclareLaunchArgument(
        'sim_cones_direct',
        default_value='false',
        description=(
            'Set true when mfe_eufs_sim runs with use_sim_cones_directly:=true, so bridge_node owns '
            '/planning/cones. Together with use_perception:=true the launch fails.'
        ),
    )

    platform_arg = DeclareLaunchArgument(
        'platform',
        default_value='legacy',
        choices=list(PLATFORMS),
        description=(
            'car: real car. Starts the sensor drivers for the listed sensors, publishes the TF tree '
            'from config/sensor_mounts.yaml, points the cone detector at the camera frame. '
            'sim: EUFS simulation; the sim launch provides the sensors and transforms. '
            'legacy (default): the behaviour before the platform argument existed, kept so that '
            'existing commands resolve unchanged. Scripts always pass sim or car.'
        ),
    )

    start_drivers_arg = DeclareLaunchArgument(
        'start_drivers',
        default_value='true',
        choices=['true', 'false'],
        description=(
            'platform:=car: false starts no sensor driver (LiDAR, camera, Xsens); topics come '
            'from a recorded bag. The sensors are still processed.'
        ),
    )

    part_arg = DeclareLaunchArgument(
        'part',
        default_value='all',
        choices=['all', 'sensing', 'estimation'],
        description=(
            'Which half of the perception group runs on this computer. all: everything. '
            'sensing: sensor drivers, TF, LiDAR and camera perception, boundary extractor. '
            'estimation: SLAM (slam:=gtsam), EKF and the evaluators. The car runs sensing on '
            'Jetson 1 and estimation on Jetson 2 (car.launch.py).'
        ),
    )

    sensors_arg = DeclareLaunchArgument(
        'sensors',
        default_value=','.join(SENSORS),
        description=(
            'Comma list of sensors present: lidar, camera, imu_gps, can. Absent sensors are not '
            'started. imu_gps absent: no EKF, pose from SLAM odometry (slam:=gtsam). '
            'camera absent: no YOLO and no camera colouring. can: car logs that no CAN node exists.'
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
            'Launch SLAM toolbox (publishes map→odom TF + /map). Only used with slam:=toolbox. '
            'Set false in sim — EUFS GT TF already provides map→odom; two publishers conflict.'
        ),
    )

    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='',
        description=(
            'SLAM implementation, started from its package launch/slam.launch.py. '
            'toolbox: slam_toolbox (map→odom, gated by use_slam). '
            'gtsam: mfe_validation slam_node, and cone_colour_fusion_node for a camera colour mode. '
            'It publishes the cone map on /planning/cones, so '
            'the boundary extractor is off, and the planner, pure pursuit and finish detector use '
            '/slam/odom. '
            'none: no SLAM node. Empty: toolbox when imu_gps is in sensors, gtsam otherwise '
            '(pose from SLAM odometry).'
        ),
    )

    # Arguments of the gtsam SLAM node (slam:=gtsam only). Names and defaults are lidar_only's.
    slam_lateral_vel_arg = DeclareLaunchArgument(
        'slam_lateral_vel', default_value='false',
        description='slam:=gtsam: integrate INS lateral velocity (lidar_only default false).',
    )
    color_mode_arg = DeclareLaunchArgument(
        'color_mode', default_value='geometric',
        description='slam:=gtsam: cone colour mode (lidar_only default geometric).',
    )
    min_obs_arg = DeclareLaunchArgument(
        'min_obs', default_value='2',
        description='slam:=gtsam: observations before a landmark is published (default 2).',
    )
    max_range_arg = DeclareLaunchArgument(
        'max_range', default_value='20.0',
        description='slam:=gtsam: landmark look-ahead range, m (lidar_only default 20.0).',
    )
    publish_hz_arg = DeclareLaunchArgument(
        'publish_hz', default_value='5.0',
        description='slam:=gtsam: cone map publish rate, Hz (lidar_only default 5.0).',
    )
    freeze_arg = DeclareLaunchArgument(
        'freeze', default_value='true',
        description='slam:=gtsam: publish each landmark once, at a fixed position (default true).',
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='',
        description='slam:=gtsam: odometry for yaw and speed. Empty: /ekf/output on car, '
                    '/sim/xsens/state_odom otherwise.',
    )

    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf',
        default_value='',
        description=(
            'Launch the GPS+IMU EKF node. Empty: on when imu_gps is in sensors, in sim too (the '
            'sim emulates /imu and /gps). Set false in sim when using '
            'pose_topic:=/sim/xsens/state_odom (EKF GPS-origin frame does not align with the '
            'Gazebo world/TF map frame).'
        ),
    )

    num_laps_arg = DeclareLaunchArgument(
        'num_laps',
        default_value='',
        description=(
            'Number of laps before the orange-cone finish gate triggers a stop. Empty = mission default '
            '(trackdrive 10, other missions 1). On closed-loop tracks (autocross/trackdrive/peanut) each '
            'signed crossing of the orange timing line counts as one lap. Ignored when endless:=true.'
        ),
    )

    orange_topic_arg = DeclareLaunchArgument(
        'orange_topic',
        default_value='/planning/cones',
        description='Cone topic the finish detector reads orange cones from for the closed-loop timing line. '
                    'Use /planning/orange_cones with lidar_only color_mode:=orange_only.',
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

    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='false',
        description=(
            'Record a ROS2 bag in MCAP format to ~/mfe_bags/<timestamp>/. '
            'MCAP is preferred over SQLite3: it is append-only (crash-safe), '
            'produces ~1/3 the file size, and supports browser-based replay via Foxglove. '
            'Set record_bag:=true on the perception machine to capture a full run.'
        ),
    )

    bag_output_dir_arg = DeclareLaunchArgument(
        'bag_output_dir',
        default_value=os.path.expanduser('~/mfe_bags'),
        description=(
            'Parent directory for MCAP bag files.  A timestamped subdirectory '
            'is created automatically by ros2 bag record.'
        ),
    )

    steer_pt2_tau_arg = DeclareLaunchArgument(
        'steer_pt2_tau',
        default_value='0.05',
        description=(
            'Pure pursuit PT2 steering anticipator time constant (s). The anticipator '
            'pre-emphasises steering changes to cancel actuator lag; at the 20 Hz control '
            'rate tau=0.05 multiplies each change in the steering request by 3.4x. '
            'The EUFS sim steering has no such lag, so the sim scripts pass 0.0 (pass-through).'
        ),
    )

    lookahead_arg = DeclareLaunchArgument(
        'lookahead',
        default_value='0.0',
        description='Override pure pursuit lookahead_distance (m). 0 = use mission default.',
    )

    mission = LaunchConfiguration('mission')
    skidpad_mode = LaunchConfiguration('skidpad_mode')
    planner_backend = LaunchConfiguration('planner_backend')
    use_perception = LaunchConfiguration('use_perception')
    use_slam = LaunchConfiguration('use_slam')
    use_ekf = LaunchConfiguration('use_ekf')
    run_perception = LaunchConfiguration('run_perception')
    run_compute = LaunchConfiguration('run_compute')
    record_bag = LaunchConfiguration('record_bag')
    bag_output_dir = LaunchConfiguration('bag_output_dir')

    # --------------------------------------------------------------------------
    # Perception — LiDAR cone detector (C++)
    # --------------------------------------------------------------------------
    # NOTE: GPU support is partly a compile-time choice. lidar_cone_detector's CMake defines
    # PLATFORM_DESKTOP_GPU and builds the CUDA filters only when a CUDA compiler is found;
    # otherwise the node is CPU-only. At runtime the 'use_gpu' parameter (default true)
    # selects the GPU path, and the node falls back to CPU PCL if no CUDA device is present.
    # Parameters: lidar_cone_detector's config/lidar_perception_node_params.yaml (the lidar_only
    # set) on every platform.
    def _make_lidar_perception(context):
        if 'lidar' not in _sensor_set(context):
            return [LogInfo(msg='[bringup] lidar_perception_node SKIPPED — no lidar in sensors, '
                                'the stack cannot see cones')]
        return [Node(
            package='lidar_cone_detector',
            executable='lidar_perception_node',
            name='lidar_perception_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('lidar_cone_detector'), 'config',
                                     'lidar_perception_node_params.yaml')],
        )]

    lidar_perception_node = OpaqueFunction(function=_make_lidar_perception)

    # --------------------------------------------------------------------------
    # Perception — Vision cone detector (Python/YOLO)
    # Launched conditionally via OpaqueFunction — skipped if vision_model_path=''.
    # Publishes mfe_msgs/Track to 'image/track' → consumed by boundary_extractor.
    # --------------------------------------------------------------------------
    vision_node_action = OpaqueFunction(
        function=lambda context: _make_vision_node(
            context.launch_configurations.get('vision_model_path', ''),
            'camera' in _sensor_set(context), _is_car(context),
        )
    )

    # --------------------------------------------------------------------------
    # SLAM — chosen by the slam argument; each implementation has launch/slam.launch.py
    #   toolbox: mfe_mapping (LaserScan from the LiDAR cloud, then slam_toolbox), needs use_slam
    #   gtsam:   mfe_validation (landmark SLAM, publishes /slam/odom and the cone map; camera colours on top)
    #   none:    nothing
    # --------------------------------------------------------------------------
    def _make_slam(context):
        if context.launch_configurations.get('slam', 'toolbox') == 'toolbox':
            return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('mfe_mapping'), 'launch',
                                 'slam.launch.py')
                ),
                condition=IfCondition(use_slam),
            )]
        return []

    slam_action = OpaqueFunction(function=_make_slam)

    # slam:=gtsam is outside the perception group: like the harness, the SLAM node runs
    # with run_perception:=false, where the LiDAR detector is started elsewhere.
    def _make_gtsam(context):
        if context.launch_configurations.get('slam') != 'gtsam':
            return []
        if context.launch_configurations.get('part') == 'sensing':
            return []
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('mfe_validation'), 'launch',
                             'slam.launch.py')
            ),
            launch_arguments={name: LaunchConfiguration(name) for name in GTSAM_ARGS}.items(),
        )]

    gtsam_action = OpaqueFunction(function=_make_gtsam)

    # --------------------------------------------------------------------------
    # Sensors — Xsens MTi-670G GNSS/INS (car only; platform:=sim emulates it in mfe_eufs_sim)
    # Publishes: /imu (sensor_msgs/Imu), /gps (sensor_msgs/NavSatFix) — feeds ekf_node below.
    # Gated on use_ekf since that's the only consumer of these topics.
    # --------------------------------------------------------------------------
    def _make_xsens(context):
        if context.launch_configurations.get('use_ekf') != 'true' \
                or _platform(context) == 'sim' or not _start_drivers(context):
            return []
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('mfe_sensors'), 'launch', 'xsens_mti.launch.py'
                )
            ),
        )]

    xsens_mti_launch = OpaqueFunction(function=_make_xsens)

    # --------------------------------------------------------------------------
    # State estimation — Extended Kalman Filter (custom MFE node)
    # Subscribes: /imu/data (Imu), /gps (NavSatFix)
    # Publishes:  /ekf/output (nav_msgs/Odometry)
    # In sim the EKF runs on the emulated /imu and /gps, like the car. launch_sim.sh still passes
    # use_ekf:=false with pose_topic:=/sim/xsens/state_odom; that disables it there. The EKF GPS
    # origin is not the Gazebo world frame, so the EKF pose cannot be used with TF-derived cones.
    # --------------------------------------------------------------------------
    ekf_params_file = os.path.join(
        get_package_share_directory('mfe_state_estimation'), 'config', 'ekf_params.yaml'
    )
    vehicle_params_file = os.path.join(
        get_package_share_directory('mfe_bringup'), 'config', 'vehicle.yaml'
    )

    def _make_ekf(context):
        params = [ekf_params_file]
        if _platform(context) in ('sim', 'car'):
            # Wheel speed from the CAN role: the CAN node on the car, the sim bridge in sim.
            params.append({'wheelspeed_topic': CAN_WHEEL_SPEED})
        if _is_car(context):
            params.append({'publish_tf': True})   # odom -> base_footprint, see _make_car_tf
        return [Node(
            package='mfe_state_estimation',
            executable='extended_kalman_filter_node',
            name='ekf_node',
            output='screen',
            parameters=params,
            remappings=[('/imu/data', '/imu')],
            condition=IfCondition(use_ekf),
        )]

    ekf_node = OpaqueFunction(function=_make_ekf)

    # --------------------------------------------------------------------------
    # Car TF (platform:=car only). Sim gets these from the EUFS launch and mfe_eufs_sim.
    #   base_footprint -> each frame in config/sensor_mounts.yaml (static), for listed sensors
    #   map -> odom: identity (static), unless slam_toolbox publishes it (slam:=toolbox with
    #                use_slam:=true). Never two publishers of map -> odom.
    #   odom -> base_footprint: published by ekf_node (publish_tf), not here.
    # Runs in the perception group: the sensors and the EKF live on the perception machine.
    # --------------------------------------------------------------------------
    def _make_car_tf(context):
        if not _is_car(context):
            return []
        mounts = _load_sensor_mounts()
        sensors = _sensor_set(context)
        actions = [_static_tf_node('base_footprint', frame, pose)
                   for frame, pose in mounts['frames'].items()
                   if frame not in _FRAME_SENSOR or _FRAME_SENSOR[frame] in sensors]
        slam_toolbox_publishes_map_odom = (
            context.launch_configurations.get('slam', 'toolbox') == 'toolbox'
            and context.launch_configurations.get('use_slam', 'true').lower() == 'true'
        )
        if not slam_toolbox_publishes_map_odom:
            actions.append(_static_tf_node('map', 'odom', {
                'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            }))
        return actions

    car_tf_action = OpaqueFunction(function=_make_car_tf)

    # --------------------------------------------------------------------------
    # Car sensor drivers (platform:=car). Each listed sensor starts its driver; a driver package
    # that is not installed stops the launch here with a message that names it.
    # --------------------------------------------------------------------------
    def _make_car_sensors(context):
        if not _is_car(context):
            return []
        sensors = _sensor_set(context)
        actions = [LogInfo(msg=f'[bringup] WARNING: {CAN_NOT_IMPLEMENTED}')]
        if not _start_drivers(context):
            actions.append(LogInfo(msg='[bringup] start_drivers:=false: sensor drivers not started, '
                                       'topics come from a bag'))
            return actions
        drivers = {
            'lidar': ('velodyne_driver', 'velodyne.launch.py'),
            'camera': ('realsense2_camera', 'realsense.launch.py'),
        }
        for sensor, (package, launch_file) in drivers.items():
            if sensor not in sensors:
                continue
            _require_package(package, f'sensor "{sensor}"')
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('mfe_sensors'), 'launch', launch_file)
                ),
            ))
        return actions

    car_sensors_action = OpaqueFunction(function=_make_car_sensors)

    # --------------------------------------------------------------------------
    # Path planning — Boundary extractor (LiDAR + camera fusion)
    # --------------------------------------------------------------------------
    def _make_boundary_extractor(context):
        # slam:=gtsam publishes the cone map on /planning/cones; two publishers would conflict.
        if context.launch_configurations.get('slam', 'toolbox') == 'gtsam':
            return []
        if context.launch_configurations.get('sim_cones_direct', 'false').lower() == 'true':
            if context.launch_configurations.get('use_perception', 'true').lower() == 'true':
                raise RuntimeError(
                    'sim_cones_direct:=true and use_perception:=true both publish /planning/cones '
                    '(bridge_node and boundary_extractor). Pass use_perception:=false.')
            return []
        return [Node(
            package='mfe_path_planning',
            executable='boundary_extractor',
            name='boundary_extractor',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('mfe_path_planning'), 'config',
                                     'boundary_extractor_params.yaml')],
            condition=IfCondition(use_perception),
        )]

    boundary_extractor_node = OpaqueFunction(function=_make_boundary_extractor)

    # --------------------------------------------------------------------------
    # Path planning — Path planner
    # --------------------------------------------------------------------------
    def _make_path_planner(context):
        params = [{
            'mission': mission,
            'skidpad_mode': skidpad_mode,
            'planner_backend': planner_backend,
            'unknown_cone_mode': LaunchConfiguration('unknown_cone_mode'),
        }]
        extra = LaunchConfiguration('planner_params_file').perform(context)
        if extra:
            params.append(extra)   # YAML loaded after the dict, so it can override these
        return [Node(
            package='mfe_path_planning',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
            parameters=params,
            remappings=[('/ekf/output', _pose_topic(context))],
        )]

    # --------------------------------------------------------------------------
    # Control — Pure pursuit
    # --------------------------------------------------------------------------
    # Pure pursuit speed/lookahead are mission-dependent: skidpad needs lower speed
    # for the tight 9.1m radius circles; other missions use the full 10 m/s.
    def _make_pure_pursuit(context):
        mission_str = context.launch_configurations.get('mission', 'autocross')
        pt = _pose_topic(context)
        # Vehicle constants come from vehicle.yaml; MAX_STEER_DEG overrides its max_steering_deg.
        params = {
            # The pose (SLAM/EKF) is the CG, 0.69 m ahead of the rear axle (cg_x 0.44 x wheelbase 1.56), but tracking
            # from the rear axle made sim laps ~20% slower with cone touches, so it is off by default
            'rear_axle_offset': float(os.environ.get('REAR_AXLE_OFFSET', '0.0')),
        }
        if 'MAX_STEER_DEG' in os.environ:
            params['max_steering_deg'] = float(os.environ['MAX_STEER_DEG'])
        if mission_str == 'skidpad':
            params.update({'max_speed': 4.5, 'lookahead_distance': 3.0})
        elif mission_str == 'acceleration':
            params.update({'max_speed': 13.0, 'lookahead_distance': 8.0})
        elif mission_str == 'peanut':
            # Tighter lookahead + lower speed for the figure-8 crossing pinch
            params.update({'max_speed': 6.0, 'lookahead_distance': 3.5})
        else:  # autocross / trackdrive
            params.update({'max_speed': 8.0, 'lookahead_distance': 5.0,
                           'speed_smooth_window': int(os.environ.get('SPEED_SMOOTH', '5'))})
        max_speed_override = float(context.launch_configurations.get('max_speed', '0.0'))
        if max_speed_override > 0.0:
            params['max_speed'] = max_speed_override
        lookahead_override = float(context.launch_configurations.get('lookahead', '0.0'))
        if lookahead_override > 0.0:
            params['lookahead_distance'] = lookahead_override
        params['steer_pt2_tau'] = float(context.launch_configurations.get('steer_pt2_tau', '0.05'))
        # pure_pursuit_node now uses an adaptive look-ahead (ld = K_v*v*sqrt(1+k^2), clamped to
        # [ld_min, ld_max]) and no longer reads lookahead_distance. Keep the per-mission value as
        # the floor: with the node default ld_min=0.5 m the car steered full lock on a straight
        # path at low speed (skidpad, merged PR branch) and wove off the track.
        params['ld_min'] = params['lookahead_distance']
        return [Node(
            package='mfe_control',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[vehicle_params_file, params],
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
        num_laps_str = context.launch_configurations.get('num_laps', '')
        num_laps = int(num_laps_str) if num_laps_str else (10 if mission_str == 'trackdrive' else 1)
        fx = _FINISH_X.get(mission_str, 100.0)
        ax = max(0.0, fx - 20.0)   # start LiDAR watch 20 m before finish gate
        pt = _pose_topic(context)
        # Closed-loop tracks use return-to-start lap counting.
        # Linear tracks keep x-position logic as a fallback only — num_laps ignored.
        # Closed loops: laps are signed crossings of the orange timing line (virtual line 6.79 m past the start
        # if no orange pair is known by min_travel_m). No LiDAR trigger and no x-position trigger on these tracks.
        if mission_str in ('autocross', 'trackdrive', 'peanut'):
            extra = {'min_travel_m': 60.0, 'return_to_start_r': 5.0, 'num_laps': num_laps,
                     'line_crossing': True, 'orange_topic': LaunchConfiguration('orange_topic').perform(context),
                     'stop_after_line_m': 5.0, 'lidar_enabled': False, 'position_fallback': False}
        elif mission_str == 'acceleration':
            # Perception-based: one orange-gate pass finishes the run. finish_x stays
            # wired above as the emergency fallback only (fires if the gate is never
            # perceived) — gate_crossings=1 makes that the PRIMARY path, not x-position.
            extra = {'gate_crossings': 1,
                     'orange_topic': LaunchConfiguration('orange_topic').perform(context)}
        elif mission_str == 'skidpad':
            # Perception-based orange-gate detection: 5 passes (entry + 4 laps of central gate)
            # finish_x stays as emergency fallback only.
            # The 5th pass is the car returning to the gate after the 4th lap; it then exits through the gate and stops
            # in the stop zone ~20 m past the pass point (finish_x=30 did this by position).
            extra = {'gate_crossings': 5, 'stop_after_gate_m': 18.0, 'position_fallback': False,
                     'orange_topic': LaunchConfiguration('orange_topic').perform(context)}
        else:
            extra = {}
        return [Node(
            package='mfe_path_planning',
            executable='finish_detector_node',
            name='finish_detector',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('mfe_path_planning'), 'config',
                             'finish_detector_node_params.yaml'),
                {
                    'approach_x': ax,
                    'finish_x': fx,
                    'detect_fwd_min_m': 1.5,
                    'detect_fwd_max_m': 15.0,
                    'detect_lateral_m': 4.0,
                    'min_finish_points': 1,
                    **extra,
                },
            ],
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
        parameters=[os.path.join(get_package_share_directory('perception_evaluator'), 'config',
                                 'fusion_evaluator_node_params.yaml')],
    )

    # --------------------------------------------------------------------------
    # Data recording — MCAP format (opt-in via record_bag:=true)
    #
    # Why MCAP instead of the default SQLite3 backend?
    #   • Crash-safe: MCAP is append-only; a power cut mid-run leaves a valid,
    #     readable file.  SQLite3 requires a clean COMMIT — a crash corrupts the
    #     whole database.
    #   • Compressed: MCAP with LZ4 compression produces ~1/3 the file size of
    #     uncompressed SQLite3 bags for typical sensor workloads.
    #   • Foxglove-ready: MCAP bags can be loaded directly in the Foxglove Studio
    #     browser app (foxglove.dev) without any conversion step.
    #
    # Topics recorded:
    #   /lidar/points_raw     — raw VLP-16 pointcloud (main debugging artifact)
    #   /gps                  — NavSatFix from Xsens MTi-670G
    #   /imu                  — Imu from Xsens MTi-670G
    #   /ekf/output           — fused pose estimate
    #   /planning/cones       — detected cone array (boundary extractor output)
    #   /control/command      — actuator commands sent to ACU
    #   /as/supervisor/status — failsafe supervisor status
    #   /mfe/emergency_brake  — emergency brake signal
    # --------------------------------------------------------------------------
    mcap_bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--storage-id', 'mcap',          # MCAP: crash-safe, compressed, Foxglove-ready
            '--compression-mode', 'file',     # compress the whole MCAP file with LZ4
            '--compression-format', 'lz4',
            '--output', bag_output_dir,
            '/lidar/points_raw',
            '/gps',
            '/imu',
            '/ekf/output',
            '/planning/cones',
            '/control/command',
            '/as/supervisor/status',
            '/mfe/emergency_brake',
        ],
        output='screen',
        name='mcap_bag_record',
        condition=IfCondition(record_bag),
    )

    # --------------------------------------------------------------------------
    # Failsafe Supervisor Node
    #
    # Chalmers-style watchdog that monitors LiDAR, GPS, and control loop health.
    # Publishes /mfe/emergency_brake (Bool) and /as/supervisor/status (String).
    # All thresholds are ROS2 params — override at launch with e.g.:
    #   lidar_timeout_ms:=300  gps_jump_threshold_m:=3.0
    # --------------------------------------------------------------------------
    # Not on the sensing computer. The estimation computer has no raw LiDAR, so the LiDAR watchdog
    # is fed by the cone list (one message per scan, small) instead of the point cloud.
    def _make_supervisor(context):
        part = context.launch_configurations.get('part', 'all')
        if part == 'sensing':
            return []
        remappings = [('/lidar/points_raw', '/perception/cones_uncolored')] if part == 'estimation' else []
        return [Node(
            package='mfe_control',
            executable='supervisor_node',
            name='supervisor_node',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('mfe_control'), 'config',
                             'supervisor_node_params.yaml'),
                {
                    'lidar_timeout_ms':             200,
                    'gps_jump_threshold_m':         2.0,
                    'gps_cov_threshold':           25.0,
                    'control_latency_threshold_ms': 100,
                },
            ],
            remappings=remappings,
        )]

    supervisor_node = OpaqueFunction(function=_make_supervisor)

    # --------------------------------------------------------------------------
    # Assemble description
    # --------------------------------------------------------------------------
    # part splits the perception group across two car computers (see car.launch.py):
    # sensing (drivers, LiDAR and camera perception, TF) and estimation (SLAM, EKF, evaluators).
    # Each action is listed once, in its order in the single-computer group.
    _PERCEPTION_PARTS = (
        (car_sensors_action, 'sensing'),
        (car_tf_action, 'sensing'),
        (lidar_perception_node, 'sensing'),
        (vision_node_action, 'sensing'),        # skipped unless vision_model_path is set
        (slam_action, 'estimation'),            # slam:=toolbox
        (xsens_mti_launch, 'sensing'),          # Xsens MTi-670G GNSS/INS -> /imu, /gps
        (ekf_node, 'estimation'),
        (boundary_extractor_node, 'sensing'),
        (evaluator_node, 'estimation'),
        (fusion_evaluator_node, 'estimation'),
    )

    def _perception_actions(context):
        part = LaunchConfiguration('part').perform(context)
        return [action for action, owner in _PERCEPTION_PARTS if part in ('all', owner)]

    perception_group = GroupAction(
        condition=IfCondition(run_perception),
        actions=[OpaqueFunction(function=_perception_actions)],
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
            OpaqueFunction(function=_make_path_planner),
            pure_pursuit_node,
            finish_detector_node,
            lap_validator_node,
        ],
    )

    return LaunchDescription([
        # args
        mission_arg,
        skidpad_mode_arg,
        planner_backend_arg,
        unknown_cone_mode_arg,
        planner_params_arg,
        vision_model_arg,
        pose_topic_arg,
        platform_arg,
        sensors_arg,
        part_arg,
        start_drivers_arg,
        use_perception_arg,
        sim_cones_direct_arg,
        endless_arg,
        num_laps_arg,
        orange_topic_arg,
        use_slam_arg,
        slam_arg,
        slam_lateral_vel_arg,
        color_mode_arg,
        min_obs_arg,
        max_range_arg,
        publish_hz_arg,
        freeze_arg,
        odom_topic_arg,
        use_ekf_arg,
        run_perception_arg,
        run_compute_arg,
        max_speed_arg,
        record_bag_arg,
        bag_output_dir_arg,
        lookahead_arg,
        steer_pt2_tau_arg,

        # derived defaults of the low-level arguments above (after every declaration)
        OpaqueFunction(function=_resolve_derived_defaults),

        # node groups — each independently enable/disable-able
        perception_group,
        compute_group,
        gtsam_action,               # slam:=gtsam

        # always-on safety supervisor
        supervisor_node,

        # optional MCAP bag recorder (opt-in: record_bag:=true)
        mcap_bag_record,
    ])
