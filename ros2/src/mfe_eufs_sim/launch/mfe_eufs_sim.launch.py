import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# The sim's LiDAR scan is instantaneous; with emulate_lidar_sweep:=true the bridge publishes the
# raw scan on this intermediate topic and lidar_sweep_emulator republishes it on /lidar/points_raw.
_INSTANT_LIDAR_TOPIC = '/lidar/points_instant'
_SENSORS = ('lidar', 'camera', 'imu_gps', 'can')


def _sensor_set(context) -> set:
    raw = context.launch_configurations.get('sensors', ','.join(_SENSORS))
    names = {name.strip() for name in raw.split(',') if name.strip()}
    unknown = names - set(_SENSORS)
    if unknown:
        raise RuntimeError(f'unknown sensors {sorted(unknown)}; choose from {",".join(_SENSORS)}')
    return names


def _resolve_emulate_default(context):
    """emulate_lidar_sweep defaults to on for the sim platform with a LiDAR; legacy stays off."""
    if context.launch_configurations.get('emulate_lidar_sweep'):
        return []
    sim_lidar = (context.launch_configurations.get('platform', 'legacy') == 'sim'
                 and 'lidar' in _sensor_set(context))
    context.launch_configurations['emulate_lidar_sweep'] = 'true' if sim_lidar else 'false'
    return []


def _bridge_and_sweep_emulator(context):
    sensors = _sensor_set(context)
    bridge_params = {
        'use_sim_cones_directly': LaunchConfiguration('use_sim_cones_directly'),
        'max_speed_ms': LaunchConfiguration('max_speed_ms'),
        'max_steering_deg': LaunchConfiguration('max_steering_deg'),
        'map_frame': 'map',
        'base_frame': 'base_footprint',
        'gear_ratio': LaunchConfiguration('gear_ratio'),
        'wheel_radius_m': LaunchConfiguration('wheel_radius_m'),
        'vehicle_mass_kg': LaunchConfiguration('vehicle_mass_kg'),
        'max_system_power_w': LaunchConfiguration('max_system_power_w'),
        'tire_mu': LaunchConfiguration('tire_mu'),
        'brake_bias_front': LaunchConfiguration('brake_bias_front'),
        'regen_braking_enabled': LaunchConfiguration('regen_braking_enabled'),
    }
    nodes = []
    emulate = LaunchConfiguration('emulate_lidar_sweep').perform(context).lower() == 'true'
    if emulate:
        bridge_params['lidar_output_topic'] = _INSTANT_LIDAR_TOPIC
    if 'lidar' not in sensors:
        # Empty topic: the bridge neither subscribes to the scan nor publishes a LiDAR.
        bridge_params['lidar_output_topic'] = ''
        emulate = False
    nodes.append(Node(
        package='mfe_eufs_sim',
        executable='bridge_node',
        name='mfe_eufs_sim_bridge',
        output='screen',
        parameters=[bridge_params],
    ))
    if emulate:
        nodes.append(Node(
            package='mfe_eufs_sim',
            executable='lidar_sweep_emulator',
            name='lidar_sweep_emulator',
            output='screen',
            parameters=[{'input_topic': _INSTANT_LIDAR_TOPIC,
                         'output_topic': '/lidar/points_raw'}],
        ))
    return nodes


def _sim_sensor_nodes(context):
    """Xsens noise model and the static sensor TFs, for the sensors listed in sensors:=."""
    sensors = _sensor_set(context)
    nodes = [Node(
        package='mfe_eufs_sim',
        executable='xsens_noise_node',
        name='xsens_noise_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('mfe_eufs_sim'), 'config',
                                 'xsens_noise_node_params.yaml')],
    )]
    # Static TFs from EUFS frames to the stack frames; adjust to sensor mounting.
    # LiDAR at car origin, 0.3 m up.
    if 'lidar' in sensors:
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf_pub',
            arguments=['0.0', '0.0', '0.3', '0', '0', '0', '1',
                       'base_footprint', 'velodyne'],
            output='screen',
        ))
    # Camera: 0.3 m ahead of centre, 0.5 m high.
    if 'camera' in sensors:
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_pub',
            arguments=['0.3', '0.0', '0.5', '0', '0', '0', '1',
                       'base_footprint', 'zed_camera_center'],
            output='screen',
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'platform',
            default_value='legacy',
            choices=['legacy', 'sim'],
            description=(
                'sim: the sensors listed in sensors:= are emulated, and the LiDAR gets the sweep '
                'emulator (emulate_lidar_sweep defaults on), the same LiDAR stack as the car. '
                'legacy (default): the behaviour before this argument existed.'
            ),
        ),
        DeclareLaunchArgument(
            'sensors',
            default_value='lidar,camera,imu_gps,can',
            description='Comma list of sensors to emulate: lidar, camera, imu_gps, can.',
        ),
        DeclareLaunchArgument(
            'use_sim_cones_directly',
            default_value='true',
            description=(
                'true  → noisy sim cones go straight to /planning/cones (test path planner). '
                'false → ground truth cones go to /perception/cones_uncolored (test full stack).'
            )
        ),
        DeclareLaunchArgument(
            'max_speed_ms',
            default_value='10.0',
            description='Hard top-speed safety cap (m/s) — zeroes commanded acceleration above this speed. Requires EUFS launched with commandMode:=acceleration.'
        ),
        DeclareLaunchArgument(
            'max_steering_deg',
            default_value='25.0',
            description='Maximum steering angle in degrees.'
        ),

        # Vehicle numbers are placeholders; see drivetrain_model.py.
        DeclareLaunchArgument('gear_ratio', default_value='14.0',
            description='Motor:wheel reduction ratio.'),
        DeclareLaunchArgument('wheel_radius_m', default_value='0.2525',
            description='Wheel radius (m) — from EUFS configDry.yaml, confirm vs. real car.'),
        DeclareLaunchArgument('vehicle_mass_kg', default_value='225.0',
            description='Vehicle mass (kg) — from EUFS configDry.yaml, confirm vs. real car.'),
        DeclareLaunchArgument('max_system_power_w', default_value='80000.0',
            description='Accumulator/inverter power cap (W) — generic FSAE placeholder.'),
        DeclareLaunchArgument('tire_mu', default_value='1.4',
            description='Tire-road friction coefficient used for the traction circle.'),
        DeclareLaunchArgument('brake_bias_front', default_value='0.6',
            description='Front brake bias fraction — reported only, not physically applied (single-track sim model).'),
        DeclareLaunchArgument('regen_braking_enabled', default_value='true',
            description='Whether braking blends in-hub motor regen before friction braking.'),
        DeclareLaunchArgument(
            'emulate_lidar_sweep',
            default_value='',
            description=(
                'true  → sim LiDAR is re-expressed as a 10 Hz spinning sweep (motion smear '
                'from ground-truth velocity) by lidar_sweep_emulator before /lidar/points_raw. '
                'false → instantaneous sim scan on /lidar/points_raw (unchanged).'
            )
        ),

        OpaqueFunction(function=_resolve_emulate_default),
        OpaqueFunction(function=_bridge_and_sweep_emulator),

        OpaqueFunction(function=_sim_sensor_nodes),
    ])
