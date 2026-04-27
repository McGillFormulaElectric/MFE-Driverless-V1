#!/bin/bash
# =============================================================================
# MFE Driverless — Full Simulation Launch Script
# Opens tmux with 6 panes and launches all components.
# Usage: bash scripts/launch_sim.sh [accel|skidpad]
#   accel   — acceleration event (default)
#   skidpad — skidpad event
# =============================================================================

GAZEBO_ROS_WS=~/Develop/gazebo_ros_pkgs
EUFS_WS=~/Develop/MFE26-eufs-sim
MFE_WS=~/Develop/MFE-Driverless-V1/ros2
LOG_DIR=~/mfe_logs

# Parse event choice
EVENT=${1:-accel}
case "$EVENT" in
    accel|acceleration)
        TRACK=acceleration
        AMI_STATE=1
        ;;
    skidpad)
        TRACK=skidpad
        AMI_STATE=2
        ;;
    *)
        echo "Unknown event '$EVENT'. Use: accel or skidpad"
        exit 1
        ;;
esac

echo "==> Launching event: $TRACK (ami_state=$AMI_STATE)"

# Create log directory
mkdir -p $LOG_DIR

export EUFS_MASTER=$EUFS_WS

SOURCE_ALL="export EUFS_MASTER=$EUFS_WS && \
            source /opt/ros/humble/setup.bash && \
            source $GAZEBO_ROS_WS/install/setup.bash && \
            source $EUFS_WS/install/setup.bash && \
            source $MFE_WS/install/setup.bash"

# Mission set command (run from pane 4 after sim is up)
MISSION_CMD="$SOURCE_ALL && ros2 service call /ros_can/set_mission eufs_msgs/srv/SetCanState '{ami_state: $AMI_STATE, as_state: 2}'"

# Logging command — ground truth state + odometry
LOG_CMD="$SOURCE_ALL && ros2 topic echo /ground_truth/state | tee $LOG_DIR/${TRACK}_\$(date +%Y%m%d_%H%M%S).log"

# Kill any existing session and stale ROS/Gazebo processes
tmux kill-session -t mfe 2>/dev/null || true
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
pkill -f ros2 2>/dev/null || true
pkill -f path_planner_node 2>/dev/null || true
pkill -f lidar_perception_node 2>/dev/null || true
pkill -f static_transform_publisher 2>/dev/null || true
sleep 1

# Create new session
tmux new-session -d -s mfe -x 220 -y 50

# Split into 6 panes: 3 left, 3 right
tmux split-window -h -t mfe          # left | right
tmux split-window -v -t mfe:0.0      # top-left | mid-left
tmux split-window -v -t mfe:0.0      # mid-left | bottom-left
tmux split-window -v -t mfe:0.1      # top-right | mid-right
tmux split-window -v -t mfe:0.3      # mid-right | bottom-right

# Pane 0 (top-left) — EUFS Sim
tmux send-keys -t mfe:0.0 \
    "$SOURCE_ALL && ros2 launch eufs_launcher simulation.launch.py commandMode:=velocity track:=$TRACK gazebo_gui:=false rviz:=false launch_group:=no_perception" Enter

# Small delay so sim starts first
sleep 2

# Pane 1 (mid-left) — MFE Bridge
tmux send-keys -t mfe:0.1 \
    "$SOURCE_ALL && ros2 launch mfe_eufs_sim mfe_eufs_sim.launch.py" Enter

# Pane 2 (bottom-left) — MFE Stack
tmux send-keys -t mfe:0.2 \
    "$SOURCE_ALL && ros2 launch mfe_bringup bringup.launch.py mission:=$TRACK" Enter

# Pane 3 (top-right) — Foxglove Bridge
tmux send-keys -t mfe:0.3 \
    "$SOURCE_ALL && ros2 launch foxglove_bridge foxglove_bridge_launch.xml" Enter

# Pane 4 (mid-right) — Mission control: pre-filled, press Enter when sim is ready
tmux send-keys -t mfe:0.4 "$SOURCE_ALL" Enter
tmux send-keys -t mfe:0.4 \
    "ros2 service call /ros_can/set_mission eufs_msgs/srv/SetCanState '{ami_state: $AMI_STATE, as_state: 2}'"

# Pane 5 (bottom-right) — Logger
tmux send-keys -t mfe:0.5 "$SOURCE_ALL" Enter
tmux send-keys -t mfe:0.5 \
    "ros2 topic echo /ground_truth/state | tee $LOG_DIR/${TRACK}_\$(date +%Y%m%d_%H%M%S).log"

# Attach to session
tmux attach-session -t mfe
