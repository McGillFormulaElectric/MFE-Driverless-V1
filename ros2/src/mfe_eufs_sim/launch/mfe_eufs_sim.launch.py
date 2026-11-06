import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mfe_common.topics import CAN_WHEEL_SPEED

_SENSORS = ('lidar', 'camera', 'imu_gps', 'can')


def _sensor_set(context) -> set:
    raw = context.launch_configurations.get('sensors', ','.join(_SENSORS))
    names = {name.strip() for name in raw.split(',') if name.strip()}
    unknown = names - set(_SENSORS)
    if unknown:
        raise RuntimeError(f'unknown sensors {sorted(unknown)}; choose from {",".join(_SENSORS)}')
    return names


def _is_sim(context) -> bool:
    return context.launch_configurations.get('platform', 'legacy') == 'sim'


def _bridge(context):
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
    # The CAN role: the wheel speed the EKF reads, on the topic the car's CAN node will publish.
    if _is_sim(context) and 'can' in sensors:
        bridge_params['wheel_speed_topic'] = CAN_WHEEL_SPEED
    if 'lidar' not in sensors:
        # Empty topic: the bridge neither subscribes to the scan nor publishes a LiDAR.
        bridge_params['lidar_output_topic'] = ''
    return [Node(
        package='mfe_eufs_sim',
        executable='bridge_node',
        name='mfe_eufs_sim_bridge',
        output='screen',
        parameters=[bridge_params],
    )]


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
    # Car IMU and GNSS as the Xsens driver publishes them, from ground truth (platform:=sim only).
    if _is_sim(context) and 'imu_gps' in sensors:
        nodes.append(Node(
            package='mfe_eufs_sim',
            executable='imu_gps_emulator',
            name='imu_gps_emulator',
            output='screen',
        ))
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
                'sim: the sensors listed in sensors:= are emulated, and the LiDAR gets the same '
                'stack as the car. '
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

        OpaqueFunction(function=_bridge),

        OpaqueFunction(function=_sim_sensor_nodes),
    ])
