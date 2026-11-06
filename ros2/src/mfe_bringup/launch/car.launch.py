"""
MFE Driverless — one computer of the two-Jetson car stack.

The car runs distributed: each Jetson starts its half, and only cone lists, poses and paths
cross the network. Raw point clouds and images stay on the Jetson that owns the sensor.

  part:=sensing      Jetson 1 (mfe-driverless-1, sensors attached): sensor drivers, static TF,
                     LiDAR and camera perception, boundary extractor, Xsens driver.
  part:=estimation   Jetson 2 (mfe-driverless-2): SLAM (gtsam), EKF, path planner, pure pursuit,
                     finish detector, lap validator, supervisor.
  part:=all          both halves on one computer (single-host test; no sensor split).

Both halves run with platform:=car. slam defaults to gtsam: the toolbox needs the raw scan,
which would have to cross the network.

Usage (after the sensing half is up on Jetson 1):
  ros2 launch mfe_bringup car.launch.py part:=sensing sensors:=lidar,camera,imu_gps
  ros2 launch mfe_bringup car.launch.py part:=estimation sensors:=imu_gps,can
run_car.sh starts both halves and checks the preflight conditions.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def _include_bringup(context):
    part = LaunchConfiguration('part').perform(context)
    launch_arguments = {
        'platform': 'car',
        'part': part,
        'run_perception': 'true',
        'run_compute': 'true' if part == 'estimation' else 'false',
        'slam': LaunchConfiguration('slam').perform(context),
        'mission': LaunchConfiguration('mission').perform(context),
        'sensors': LaunchConfiguration('sensors').perform(context),
        'vision_model_path': LaunchConfiguration('vision_model_path').perform(context),
        'endless': LaunchConfiguration('endless').perform(context),
        'num_laps': LaunchConfiguration('num_laps').perform(context),
        'start_drivers': LaunchConfiguration('start_drivers').perform(context),
    }
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mfe_bringup'), 'launch', 'bringup.launch.py')
        ),
        launch_arguments=launch_arguments.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'part', choices=['all', 'sensing', 'estimation'],
            description='Which half of the car stack this computer runs (see module docstring). '
                        'all: both halves on one computer (single-host testing).',
        ),
        DeclareLaunchArgument('start_drivers', default_value='true', choices=['true', 'false'],
                              description='false: no sensor drivers, a bag provides the topics.'),
        DeclareLaunchArgument('mission', default_value='autocross',
                              description='FSAE mission for the path planner.'),
        DeclareLaunchArgument('sensors', default_value='lidar,camera,imu_gps,can',
                              description='Sensors attached to the car (see bringup.launch.py).'),
        DeclareLaunchArgument('slam', default_value='gtsam',
                              choices=['gtsam', 'none'],
                              description='SLAM for the split. toolbox needs the raw scan.'),
        DeclareLaunchArgument('vision_model_path', default_value='',
                              description='YOLO weights on the sensing computer. Empty: no YOLO.'),
        DeclareLaunchArgument('endless', default_value='false',
                              description='Disable finish detection so the car loops indefinitely.'),
        DeclareLaunchArgument('num_laps', default_value='',
                              description='Laps before the finish gate stops the car. Empty: mission default.'),
        OpaqueFunction(function=_include_bringup),
    ])
