"""
MFE Driverless — fleet launch (multi-machine).

Launches perception on THIS machine and SSHes into Jetson(s) to start compute.
Ctrl+C kills everything on all machines cleanly.

Prerequisites on each Jetson:
  1. SSH key auth:  ssh-copy-id <jetson-host>
  2. ROS2 Humble installed and workspace built:
       cd ~/MFE-Driverless-V1/ros2 && colcon build --packages-skip mfe_eufs_sim
  3. ~/.bashrc (or ~/.profile) contains:
       export ROS_DOMAIN_ID=42
       export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
       export CYCLONEDDS_URI=file://$HOME/cyclone_tailscale.xml
  4. ~/cyclone_tailscale.xml exists with correct Tailscale peer IPs.

Usage (sim — one Jetson for compute):
  ros2 launch mfe_bringup fleet.launch.py \\
    pose_topic:=/ground_truth/state_odom \\
    use_slam:=false use_ekf:=false

Usage (real car — two Jetsons):
  ros2 launch mfe_bringup fleet.launch.py \\
    perception_host:=mfe-driverless-1 \\
    compute_host:=mfe-driverless-2 \\
    local_role:=none \\
    mission:=trackdrive

local_role options:
  perception  — run perception here, compute on compute_host
  compute     — run compute here, perception on perception_host
  both        — run everything locally (single machine, no SSH)
  none        — run nothing locally (both roles on remote Jetsons)
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    LogInfo, RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


_MFE_REPO   = 'https://github.com/McGillFormulaElectric/MFE-Driverless-V1.git'
_MFE_DIR    = '~/Develop/MFE-Driverless-V1'
_SETUP_SCRIPT = f'{_MFE_DIR}/scripts/setup_jetson.sh'
# Marker file present only after a successful colcon build on the remote.
_BUILD_MARKER = f'{_MFE_DIR}/ros2/install/mfe_bringup/share/mfe_bringup/launch/compute.launch.py'

# Bootstrap + launch command sent over SSH.
# Steps:
#   1. Clone the repo if not present (setup_jetson.sh lives inside it)
#   2. Run setup_jetson.sh if the workspace isn't built yet
#   3. Source the workspace and launch
# -t : allocate a TTY so Ctrl+C propagates back to remote processes
_SSH_LAUNCH = (
    "bash -lc '"
    # ── Step 1: clone if needed ──────────────────────────────────────────
    "mkdir -p ~/Develop && "
    "if [ ! -d {mfe_dir} ]; then "
    "  echo \"[fleet] Cloning MFE-Driverless-V1 on $(hostname)...\"; "
    "  git clone {repo} {mfe_dir} || exit 1; "
    "else "
    "  git -C {mfe_dir} pull --ff-only 2>/dev/null || true; "
    "fi && "
    # ── Step 2: build if needed ──────────────────────────────────────────
    "if [ ! -f {marker} ]; then "
    "  echo \"[fleet] Building workspace on $(hostname) — this takes ~5 min...\"; "
    "  bash {setup} || exit 1; "
    "fi && "
    # ── Step 3: launch ───────────────────────────────────────────────────
    "source /opt/ros/humble/setup.bash && "
    "source {mfe_dir}/ros2/install/setup.bash && "
    "ros2 launch mfe_bringup {{launch_file}} {{args}}"
    "'"
).format(
    repo=_MFE_REPO,
    mfe_dir=_MFE_DIR,
    marker=_BUILD_MARKER,
    setup=_SETUP_SCRIPT,
)


def _ssh_cmd(host: str, launch_file: str, extra_args: str = '') -> list:
    remote_cmd = _SSH_LAUNCH.format(launch_file=launch_file, args=extra_args)
    return ['ssh', '-t', '-o', 'StrictHostKeyChecking=no', host, remote_cmd]


def generate_launch_description():
    bringup_dir = get_package_share_directory('mfe_bringup')

    # ── Launch arguments ────────────────────────────────────────────────────
    local_role_arg = DeclareLaunchArgument(
        'local_role',
        default_value='perception',
        choices=['perception', 'compute', 'both', 'none'],
        description=(
            'What runs on THIS machine. '
            '"perception" = LiDAR/vision/SLAM here, compute on Jetson. '
            '"compute" = planning/control here, perception on Jetson. '
            '"both" = everything local (single-machine mode). '
            '"none" = only SSH launcher, nothing local.'
        ),
    )

    perception_host_arg = DeclareLaunchArgument(
        'perception_host',
        default_value='',
        description=(
            'SSH target for the perception Jetson (e.g. mfe@100.84.7.80). '
            'Leave empty to skip SSH launch of perception.'
        ),
    )

    compute_host_arg = DeclareLaunchArgument(
        'compute_host',
        default_value='mfe@100.119.35.6',
        description=(
            'SSH target for the compute Jetson (e.g. mfe-driverless-2 or 100.119.35.6). '
            'Leave empty to skip SSH launch of compute.'
        ),
    )

    mission_arg = DeclareLaunchArgument(
        'mission',
        default_value='autocross',
        choices=['autocross', 'trackdrive', 'acceleration', 'skidpad'],
        description='FSAE mission — forwarded to all remote launches.',
    )

    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='/ekf/output',
        description='Odometry topic — forwarded to all remote launches.',
    )

    use_slam_arg = DeclareLaunchArgument(
        'use_slam', default_value='true',
        description='Enable SLAM toolbox on perception machine.',
    )

    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf', default_value='true',
        description='Enable EKF on perception machine.',
    )

    endless_arg = DeclareLaunchArgument(
        'endless', default_value='false',
        description='Disable finish detector — loop forever.',
    )

    vision_model_arg = DeclareLaunchArgument(
        'vision_model_path',
        default_value=os.path.expanduser('~/mfe_models/yolo/yolo11s/weights/best.pt'),
        # Trained on FSOCO dataset (11 567 images, 50 epochs, mAP50=0.767)
        description='Path to YOLO weights on the perception machine.',
    )

    # ── Local node groups (OpaqueFunction reads launch configs at runtime) ──
    def _local_nodes(context):
        role        = context.launch_configurations.get('local_role', 'perception')
        mission     = context.launch_configurations.get('mission', 'autocross')
        pose_topic  = context.launch_configurations.get('pose_topic', '/ekf/output')
        use_slam    = context.launch_configurations.get('use_slam', 'true')
        use_ekf     = context.launch_configurations.get('use_ekf', 'true')
        endless     = context.launch_configurations.get('endless', 'false')
        model_path  = context.launch_configurations.get('vision_model_path', '')

        if role == 'none':
            return [LogInfo(msg='[fleet] local_role=none — no nodes launched locally')]

        run_perception = 'true' if role in ('perception', 'both') else 'false'
        run_compute    = 'true' if role in ('compute', 'both') else 'false'

        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'bringup.launch.py')
            ),
            launch_arguments={
                'mission':            mission,
                'pose_topic':         pose_topic,
                'use_slam':           use_slam,
                'use_ekf':            use_ekf,
                'endless':            endless,
                'vision_model_path':  model_path,
                'run_perception':     run_perception,
                'run_compute':        run_compute,
            }.items(),
        )]

    # ── Remote SSH launchers ─────────────────────────────────────────────────
    def _remote_nodes(context):
        actions = []

        local_role       = context.launch_configurations.get('local_role', 'perception')
        compute_host     = context.launch_configurations.get('compute_host', '')
        perception_host  = context.launch_configurations.get('perception_host', '')
        mission          = context.launch_configurations.get('mission', 'autocross')
        pose_topic       = context.launch_configurations.get('pose_topic', '/ekf/output')
        use_slam         = context.launch_configurations.get('use_slam', 'true')
        use_ekf          = context.launch_configurations.get('use_ekf', 'true')
        endless          = context.launch_configurations.get('endless', 'false')
        model_path       = context.launch_configurations.get('vision_model_path', '')

        # Common args forwarded to every remote launch
        common = (
            f'mission:={mission} '
            f'pose_topic:={pose_topic} '
            f'endless:={endless}'
        )

        # SSH → compute Jetson (unless compute runs locally)
        if compute_host and local_role not in ('compute', 'both'):
            compute_args = common
            actions.append(LogInfo(msg=f'[fleet] Launching compute on {compute_host}'))
            actions.append(ExecuteProcess(
                cmd=_ssh_cmd(compute_host, 'compute.launch.py', compute_args),
                output='screen',
                name=f'ssh_{compute_host}_compute',
            ))

        # SSH → perception Jetson (unless perception runs locally or locally=both)
        if perception_host and local_role not in ('perception', 'both'):
            perc_args = (
                f'{common} '
                f'use_slam:={use_slam} '
                f'use_ekf:={use_ekf} '
                f'vision_model_path:={model_path}'
            )
            actions.append(LogInfo(msg=f'[fleet] Launching perception on {perception_host}'))
            actions.append(ExecuteProcess(
                cmd=_ssh_cmd(perception_host, 'perception.launch.py', perc_args),
                output='screen',
                name=f'ssh_{perception_host}_perception',
            ))

        if not actions:
            actions.append(LogInfo(msg='[fleet] No remote hosts configured — running fully local'))

        return actions

    from launch.actions import OpaqueFunction
    local_action  = OpaqueFunction(function=_local_nodes)
    remote_action = OpaqueFunction(function=_remote_nodes)

    return LaunchDescription([
        local_role_arg,
        perception_host_arg,
        compute_host_arg,
        mission_arg,
        pose_topic_arg,
        use_slam_arg,
        use_ekf_arg,
        endless_arg,
        vision_model_arg,
        local_action,
        remote_action,
    ])
