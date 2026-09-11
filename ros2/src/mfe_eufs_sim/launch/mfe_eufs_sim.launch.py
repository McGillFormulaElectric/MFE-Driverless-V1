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
            description='Max vehicle speed in m/s (throttle=1). Requires EUFS launched with commandMode:=velocity.'
        ),
        DeclareLaunchArgument(
            'max_steering_deg',
            default_value='25.0',
            description='Maximum steering angle in degrees.'
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
        #  Xsens MTi-670G noise injector                                      #
        #                                                                    #
        #  /ground_truth/state_odom (perfect) -> /sim/xsens/state_odom       #
        #  (noise-corrupted, representative of the real MTi-670G). Point     #
        #  pose_topic at the noisy topic instead of raw ground truth so sim  #
        #  runs actually exercise the stack against sensor-realistic pose    #
        #  estimates. See mfe_eufs_sim/xsens_noise_node.py for the model.    #
        # ---------------------------------------------------------------- #
        Node(
            package='mfe_eufs_sim',
            executable='xsens_noise_node',
            name='xsens_noise_node',
            output='screen',
        ),

        # The simulator robot_state_publisher owns sensor transforms from URDF.
        # Publishing approximate duplicate transforms here corrupts projection.
    ])
