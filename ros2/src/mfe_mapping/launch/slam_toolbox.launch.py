import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

from lifecycle_msgs.msg import Transition

def generate_launch_description():
    curr_dir = get_package_share_path("mfe_mapping")

    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    autostart = LaunchConfiguration("autostart")

    slam_params_file = os.path.join(curr_dir, "config", "slam_toolbox_params.yaml")

    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox_node",
        output="both",
        namespace=namespace,
        parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
        remappings=[
            ("/slam_toolbox/graph_visualization", "/slam/graph_visualization"),
            ("/pose", "/slam/pose"),
        ],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node), transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(autostart),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(autostart),
    )

    # Declare the launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value=""
    )

    declare_log_level = DeclareLaunchArgument(
        "log_level",
        default_value="info"
    )

    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true"
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_namespace,
            declare_log_level,
            declare_autostart,
            slam_toolbox_node,
            configure_event,
            activate_event,
        ]
    )