from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mfe_eufs_sim',
            executable='bridge_node',
            name='mfe_eufs_sim',
            output='screen',
            remappings=[
                ('pcl/input', '/lidar/pcl/raw'),  # raw lidar data from eufs sim
            ],
        ),

        # static transform publisher for lidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf_pub',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'velodyne'],
            output='screen'
        ),
    ])