import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    state_estimation_dir = get_package_share_directory("mfe_state_estimation")
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    localization_params_file = os.path.join(state_estimation_dir, "config", "ekf.yaml")
    localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="odom_filter_node",
        output="both",
        parameters=[localization_params_file],
        remappings=[
            ("cmd_vel", "control/nav_cmd_vel"),
            ("/enable", "/odometry/enable"),
            ("/set_pose", "/odometry/set_pose"),
            ("/odometry/filtered", "imu/odometry")
        ],
    )
    navsat_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("mfe_state_estimation"),
                "launch",
                "gps_transform.launch.py"
            ])
        )
    )

    return LaunchDescription([
        localization_node,
        navsat_node
    ])