import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    state_estimation_dir = get_package_share_directory("mfe_state_estimation")

    gps_params_file = os.path.join(state_estimation_dir, "config", "gps.yaml")
    gps_filter_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="gps_filter_node",
        output="both",
        parameters=[
            gps_params_file,
            {
                "magnetic_declination_radians": 13.7, 
                "use_odometry_yaw": False
            }
        ],
        remappings=[
            ("/imu/data", "gps/imu/data"),
            ("/odometry/filtered", "gps/odometry")
        ]
    )

    return LaunchDescription([
        gps_filter_node
    ])