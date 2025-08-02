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
    
    localization_params_file = os.path.join(state_estimation_dir, "config", "ekf_navsat.yaml")

    # Keep this commented out unless we have secondary source of continuous odometry (e.g kinematics)
    local_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[localization_params_file],
        remappings=[
            ("/odometry/filtered", "/odometry/local")
        ],
    )
    
    global_localization_node = Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_map',
	        output='screen',
            parameters=[localization_params_file],
            remappings=[
                ('/odometry/filtered', '/odometry/global')
            ]
    )          
    
    navsat_node = Node(
            package='robot_localization', 
            executable='navsat_transform_node', 
            name='navsat_transform_node',
	        output='screen',
            parameters=[localization_params_file],
            remappings=[('imu', '/fsds/imu'),
                        ('/gps/fix', '/fsds/gps'), 
                        ('/odometry/filtered', '/odometry/global'),
                        ('/odometry/gps', '/odometry/gps'),
                        ('/gps/filtered', '/gps/filtered')]              
    )           

    return LaunchDescription([
        local_localization_node,
        global_localization_node,
        navsat_node
    ])