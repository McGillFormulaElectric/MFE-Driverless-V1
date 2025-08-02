from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription, 
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from ament_index_python import get_package_share_directory

from launch.logging import get_logger

import os

def generate_launch_description():
    logger = get_logger()

    use_sim = LaunchConfiguration('use_sim')
    use_rawdata_bag = LaunchConfiguration('use_rawdata_bag')
    rawdata_bag_file = LaunchConfiguration('rawdata_bag_file')

    nav_processes = []
    
    lidar_sliding_window_acc = 'false'
    if use_rawdata_bag.perform(context=None) == 'true':
        if not rawdata_bag_file.perform(context=None):
            logger.warning("rawdata_bag_file:=<path> is required but was empty")

            return [EmitEvent(event=Shutdown(reason="Missing required 'rawdata_bag_file'"))]
            
        play_bag = ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play',
                rawdata_bag_file,
                '--clock',
                '--no-loop'
            ]
        )

        nav_processes.append(play_bag)
        lidar_sliding_window_acc = 'true'

    if use_sim.perform(context=None) == 'true':
        sim_processes = []
        
        nav_processes.extend(sim_processes)
    else:
        lidar_pipeline_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('lidar_cone_detector'),
                    'launch',
                    'lidar_pipeline.launch.py'
                )
            ),
            launch_arguments={
                'load_file': 'false',
                'sliding_window_acc': lidar_sliding_window_acc
            }.items()
        )

        vision_pipeline_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('vision_cone_detector'),
                    'launch',
                    'vision_pipeline.launch.py'
                )
            ),
            launch_arguments={
                'load_file': 'false'
            }.items()
        )

        nav_processes.extend([lidar_pipeline_launch, vision_pipeline_launch])

    state_estimation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mfe_state_estimation'),
                'launch',
                'ekf_localization.launch.py'
            )
        )
    )
    nav_processes.append(state_estimation_launch)

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mfe_mapping'),
                'launch',
                'slam_toolbox.launch.py'
            )
        ),
        launch_arguments={
            "autostart": 'true'
        }.items()
    )
    nav_processes.append(slam_toolbox_launch)

    use_sim_arg = DeclareLaunchArgument('use_sim', default_value='false')
    use_rawdata_bag_arg = DeclareLaunchArgument('use_rawdata_bag', default_value='false')
    rawdata_bag_file_arg = DeclareLaunchArgument('rawdata_bag_file', default_value='')

    arguments = [
        use_sim_arg,
        use_rawdata_bag_arg, 
        rawdata_bag_file_arg
    ]

    return LaunchDescription(arguments + nav_processes)