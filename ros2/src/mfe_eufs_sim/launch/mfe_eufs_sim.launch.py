from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ---------------------------------------------------------------- #
        #  Launch arguments                                                  #
        # ---------------------------------------------------------------- #
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
            description='Maximum vehicle speed in m/s sent to EUFS sim.'
        ),
        DeclareLaunchArgument(
            'max_steering_deg',
            default_value='25.0',
            description='Maximum steering angle in degrees sent to EUFS sim.'
        ),

        # ---------------------------------------------------------------- #
        #  Bridge node                                                       #
        # ---------------------------------------------------------------- #
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
            }],
        ),

        # ---------------------------------------------------------------- #
        #  Static TF publishers                                              #
        #                                                                    #
        #  These connect the EUFS sim TF frames to the frames the           #
        #  driverless stack expects. Adjust translations/rotations to        #
        #  match your physical sensor mounting positions.                    #
        # ---------------------------------------------------------------- #

        # velodyne frame → base_footprint (LiDAR mounted at car origin)
        # Args: x y z qx qy qz qw parent child
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf_pub',
            arguments=['0.0', '0.0', '0.3', '0', '0', '0', '1',
                       'base_footprint', 'velodyne'],
            output='screen',
        ),

        # ZED camera frame → base_footprint
        # D435i sits ~0.3 m ahead of the car centre, ~0.5 m high
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_pub',
            arguments=['0.3', '0.0', '0.5', '0', '0', '0', '1',
                       'base_footprint', 'zed_camera_center'],
            output='screen',
        ),
    ])
