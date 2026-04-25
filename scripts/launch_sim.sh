#!/bin/bash
# =============================================================================
# MFE Driverless — Full Simulation Launch Script
# Opens tmux with 4 panes and launches all components.
# Usage: bash scripts/launch_sim.sh
# =============================================================================

GAZEBO_ROS_WS=~/Develop/gazebo_ros_pkgs
EUFS_WS=~/Develop/MFE26-eufs-sim
MFE_WS=~/Develop/MFE-Driverless-V1/ros2

SOURCE_ALL="source /opt/ros/humble/setup.bash && \
            source $GAZEBO_ROS_WS/install/setup.bash && \
            source $EUFS_WS/install/setup.bash && \
            source $MFE_WS/install/setup.bash"

# Kill any existing session
tmux kill-session -t mfe 2>/dev/null || true

# Create new session with first pane (EUFS Sim)
tmux new-session -d -s mfe -x 220 -y 50

# Split into 4 panes: top-left, top-right, bottom-left, bottom-right
tmux split-window -h -t mfe          # left | right
tmux split-window -v -t mfe:0.0      # top-left | bottom-left
tmux split-window -v -t mfe:0.1      # top-right | bottom-right

# Pane 0 (top-left) — EUFS Sim
tmux send-keys -t mfe:0.0 \
    "$SOURCE_ALL && ros2 launch eufs_launcher eufs_launcher.launch.py commandMode:=velocity" Enter

# Small delay so sim starts first
sleep 2

# Pane 1 (bottom-left) — MFE Bridge
tmux send-keys -t mfe:0.2 \
    "$SOURCE_ALL && ros2 launch mfe_eufs_sim mfe_eufs_sim.launch.py" Enter

# Pane 2 (top-right) — MFE Stack
tmux send-keys -t mfe:0.1 \
    "$SOURCE_ALL && ros2 launch mfe_bringup bringup.launch.py" Enter

# Pane 3 (bottom-right) — Foxglove Bridge
tmux send-keys -t mfe:0.3 \
    "source /opt/ros/humble/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml" Enter

# Attach to session
tmux attach-session -t mfe
