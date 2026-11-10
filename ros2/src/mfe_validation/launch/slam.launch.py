"""SLAM entry point for bringup (slam:=gtsam): GTSAM landmark SLAM on the LiDAR cones.

Starts slam_node, and cone_colour_fusion_node for a camera colour mode. The arguments and their defaults are
the ones lidar_only.launch.py uses, so the same values give the same node parameters. The LiDAR detector stays
in bringup. slam_node publishes /slam/odom, and the cone map on /planning/cones (on /slam/cones with a camera
colour mode, where cone_colour_fusion_node writes /planning/cones), so bringup routes the planner pose there.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from mfe_common.topics import PLANNING_ORANGE_CONES, SLAM_CONE_MAP, SLAM_CONES
from mfe_validation.cam_fusion import CAMERA_COLOUR_MODES


def _slam_nodes(context):
    mode = LC('color_mode').perform(context)
    camera = mode in CAMERA_COLOUR_MODES
    nodes = [Node(package='mfe_validation', executable='slam_node', name='slam_node', output='screen',
                  parameters=[{
                      'use_lateral_vel': ParameterValue(LC('slam_lateral_vel'), value_type=bool),
                      'odom_topic': LC('odom_topic'),
                      'cones_topic': SLAM_CONES if camera else SLAM_CONE_MAP,
                      'color_mode': mode,
                      'min_obs_publish': ParameterValue(LC('min_obs'), value_type=int),
                      'max_range': ParameterValue(LC('max_range'), value_type=float),
                      'publish_hz': ParameterValue(LC('publish_hz'), value_type=float),
                      'freeze': ParameterValue(LC('freeze'), value_type=bool),
                      'roi_x_min': ParameterValue(LC('roi_x_min'), value_type=float),
                      'roi_x_max': ParameterValue(LC('roi_x_max'), value_type=float),
                      'roi_half_width': ParameterValue(LC('roi_half_width'), value_type=float),
                  }])]
    if camera:
        nodes.append(Node(package='mfe_validation', executable='cone_colour_fusion_node', name='cone_colour_fusion_node',
                          output='screen',
                          parameters=[{'color_mode': mode, 'cones_topic': SLAM_CONE_MAP,
                                       'orange_cones_topic': PLANNING_ORANGE_CONES}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('slam_lateral_vel', default_value='false'),
        DeclareLaunchArgument('color_mode', default_value='geometric'),
        DeclareLaunchArgument('min_obs', default_value='2'),
        DeclareLaunchArgument('max_range', default_value='20.0'),
        DeclareLaunchArgument('publish_hz', default_value='5.0'),
        DeclareLaunchArgument('freeze', default_value='true'),
        DeclareLaunchArgument('roi_x_min', default_value='0.0'),
        DeclareLaunchArgument('roi_x_max', default_value='100.0'),
        DeclareLaunchArgument('roi_half_width', default_value='0.0'),
        DeclareLaunchArgument('odom_topic', default_value='/sim/xsens/state_odom'),
        OpaqueFunction(function=_slam_nodes),
    ])
