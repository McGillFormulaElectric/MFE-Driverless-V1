from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim = LaunchConfiguration('use_simulation')
    declare_use_sim = DeclareLaunchArgument(
        'use_simulation', 
        default_value="true"
    )

    sim_lidar_node = LifecycleNode(
        package='mfe_sensors', executable='sim_lidar_node',
        name='sim_lidar_node',
        namespace='mfe_sensors',
        output='screen',
        remappings=[
            ('/sim/lidar', '/fsds/lidar/Lidar1'),
            ('sim_lidar_node/data', 'lidar/data')
        ],
        condition=IfCondition(use_sim),
    )

    sim_sensor_nodes = ['sim_lidar_node']
    lifecycle_mgr_sim = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_sensors',
        namespace='mfe_sensors',
        output='screen',
        condition=IfCondition(use_sim),
        parameters=[{
            'autostart': True,
            'bond_timeout': 4.0,              # optional watchdog
            'node_names': sim_sensor_nodes # list of nodes to manage
        }]
    ) 
    
    real_sensor_nodes = []
    lifecycle_mgr_real = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_sensors',
        namespace='mfe_sensors',
        output='screen',
        condition=UnlessCondition(use_sim),
        parameters=[{
            'autostart': True,
            'bond_timeout': 4.0,              # optional watchdog
            'node_names': real_sensor_nodes # list of nodes to manage
        }]
    ) 

    return LaunchDescription([
        declare_use_sim,
        sim_lidar_node,
        lifecycle_mgr_sim,
        lifecycle_mgr_real
    ])