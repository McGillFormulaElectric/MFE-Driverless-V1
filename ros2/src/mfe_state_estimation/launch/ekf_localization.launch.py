import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    state_estimation_dir = get_package_share_directory("mfe_state_estimation")

    ekf_params_file = os.path.join(state_estimation_dir, "config", "ekf_params.yaml")

    ekf_node = Node(
        package="mfe_state_estimation",
        executable="extended_kalman_filter_node",
        name="ekf_node",
        output="screen",
        parameters=[ekf_params_file],
        remappings=[("/imu/data", "/imu")],
    )

    return LaunchDescription([
        ekf_node,
    ])
