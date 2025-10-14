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
    ])
