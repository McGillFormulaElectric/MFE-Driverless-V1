from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
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

        Node(
            package='mfe_eufs_sim',
            executable='bridge_node',
            name='mfe_eufs_sim_bridge',
            output='screen',
            parameters=[{
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
            }],
        ),

        # Noise-injected pose (Xsens MTi-670G model) for sensor-realistic sim runs.
        # See xsens_noise_node.py.
        Node(
            package='mfe_eufs_sim',
            executable='xsens_noise_node',
            name='xsens_noise_node',
            output='screen',
        ),

        # Static TFs from EUFS frames to the stack frames; adjust to sensor mounting.
        # LiDAR at car origin, 0.3 m up.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf_pub',
            arguments=['0.0', '0.0', '0.3', '0', '0', '0', '1',
                       'base_footprint', 'velodyne'],
            output='screen',
        ),

        # Camera: 0.3 m ahead of centre, 0.5 m high.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_pub',
            arguments=['0.3', '0.0', '0.5', '0', '0', '0', '1',
                       'base_footprint', 'zed_camera_center'],
            output='screen',
        ),
    ])
