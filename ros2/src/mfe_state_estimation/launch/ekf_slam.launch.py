import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory, geet

def generate_launch_description():
    state_estimation_dir = get_package_share_directory("mfe_state_estimation")
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    localization_params_file = os.path.join(state_estimation_dir, "config", "ekf.yaml")
    # TODO: Add some configurations
    localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="odom_filter_node",
        output="both",
        parameters=[localization_params_file, {"use_sim_time": use_sim_time}],
        remappings=[
            ("cmd_vel", "control/nav_cmd_vel"),
            ("/enable", "/odometry/enable"),
            ("/set_pose", "/odometry/set_pose"),
        ],
    )

    return LaunchDescription(
        localization_node
    )