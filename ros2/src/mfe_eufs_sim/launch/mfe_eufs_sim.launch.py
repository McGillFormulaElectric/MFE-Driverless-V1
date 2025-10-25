from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mfe_eufs_sim',
            executable='bridge_node',
            name='mfe_eufs_sim',
            output='screen',
        ),

        # static transform publisher for lidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf_pub',
            # tx, ty, tz, qx, qy, qz, qw, parent_frame, child_frame
            arguments=['0', '0', '0', '0', '0', '0', '1', 'lidar_base', 'velodyne'],
            output='screen'
        ),

        # static transform publisher for camera
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_pub',
            # tx, ty, tz, qx, qy, qz, qw, parent_frame, child_frame
            arguments=['0', '0', '0', '0', '0', '0', '1', 'camera_base', 'zed_camera_center'],
            output='screen'
        ),
    ])