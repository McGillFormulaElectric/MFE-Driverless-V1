#!/bin/bash
# =============================================================================
# MFE Driverless — Full Simulation Launch Script
# Opens tmux with 6 panes and launches all components.
#
# Usage: bash scripts/launch_sim.sh [accel|skidpad] [perception|no_perception] [gui|nogui]
#   accel         — acceleration event (default)
#   skidpad       — skidpad event
#   perception    — full LiDAR pipeline (lidar_cone_detector → boundary_extractor)
#   no_perception — ground truth cones direct to planner (default, faster to test)
#   gui           — show Gazebo GUI window (default)
#   nogui         — headless Gazebo (faster, for Jetson / no display)
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
        MISSION=acceleration
        AMI_STATE=11  # AMI_ACCELERATION
        SPAWN_X=0.0; SPAWN_Y=0.0; SPAWN_YAW=0.0
        ;;
    skidpad)
        TRACK=skidpad
        MISSION=skidpad
        AMI_STATE=12  # AMI_SKIDPAD
        SPAWN_X=0.0; SPAWN_Y=-14.4; SPAWN_YAW=1.5707963
        ;;
    autocross|small_track|peanut|rectangle|garden_light|boa_constrictor|comp_2021|hairpins|rand|its_a_mess)
        case "$EVENT" in
            autocross)    TRACK=small_track ;;
            hairpins)     TRACK=hairpins_increasing_difficulty ;;
            *)            TRACK=$EVENT ;;
        esac
        MISSION=autocross
        AMI_STATE=13  # AMI_AUTOCROSS
        # Per-track spawn positions (from each track's .launch file)
        case "$TRACK" in
            small_track)    SPAWN_X=-13.0; SPAWN_Y=10.3;  SPAWN_YAW=0.0 ;;
            peanut)         SPAWN_X=0.0;   SPAWN_Y=0.0;   SPAWN_YAW=0.0 ;;
            rectangle)      SPAWN_X=0.0;   SPAWN_Y=0.0;   SPAWN_YAW=0.0 ;;
            *)              SPAWN_X=0.0;   SPAWN_Y=0.0;   SPAWN_YAW=0.0 ;;
        esac
        ;;
    *)
        echo "Unknown event '$EVENT'. Use: accel, skidpad, autocross, small_track, peanut, rectangle, garden_light, boa_constrictor, comp_2021, hairpins, rand"
        exit 1
        ;;
esac

# Parse perception mode
MODE=${2:-no_perception}
case "$MODE" in
    perception)
        LAUNCH_GROUP=default
        USE_SIM_CONES=false
        PUBLISH_GT_TF=true
        ;;
    no_perception)
        LAUNCH_GROUP=no_perception
        USE_SIM_CONES=true
        PUBLISH_GT_TF=true
        ;;
    *)
        echo "Unknown mode '$MODE'. Use: perception or no_perception"
        exit 1
        ;;
esac

# Parse GUI mode
GUI_ARG=${3:-gui}
case "$GUI_ARG" in
    gui)   GAZEBO_GUI=true  ;;
    nogui) GAZEBO_GUI=false ;;
    *)
        echo "Unknown gui arg '$GUI_ARG'. Use: gui or nogui"
        exit 1
        ;;
esac

echo "==> Launching event: $TRACK (ami_state=$AMI_STATE) | mode: $MODE | gazebo_gui: $GAZEBO_GUI"

# Create log directory
mkdir -p $LOG_DIR

export EUFS_MASTER=$EUFS_WS

SOURCE_ALL="export EUFS_MASTER=$EUFS_WS && \
            source /opt/ros/humble/setup.bash && \
            ([ -f $GAZEBO_ROS_WS/install/setup.bash ] && source $GAZEBO_ROS_WS/install/setup.bash || true) && \
            source $EUFS_WS/install/setup.bash && \
            source $MFE_WS/install/setup.bash"

# Kill any existing session and stale ROS/Gazebo processes
tmux kill-session -t mfe 2>/dev/null || true
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
pkill -f ros2 2>/dev/null || true
pkill -f path_planner_node 2>/dev/null || true
pkill -f lidar_perception_node 2>/dev/null || true
pkill -f static_transform_publisher 2>/dev/null || true
sleep 1

# Restart ROS2 daemon cleanly after killing all ros2 processes
source /opt/ros/humble/setup.bash
ros2 daemon stop 2>/dev/null || true
ros2 daemon start

# Create new session
tmux new-session -d -s mfe -x 220 -y 50

# Split into 6 panes: 3 left, 3 right
tmux split-window -h -t mfe          # left | right
tmux split-window -v -t mfe:0.0      # top-left | mid-left
tmux split-window -v -t mfe:0.0      # mid-left | bottom-left
tmux split-window -v -t mfe:0.1      # top-right | mid-right
tmux split-window -v -t mfe:0.3      # mid-right | bottom-right

# Pane 0 (top-left) — EUFS Sim (gzserver + gzclient only; car spawned separately below)
# Pass x/y/yaw so eufs_launcher uses the correct start position for this track.
tmux send-keys -t mfe:0.0 \
    "$SOURCE_ALL && ros2 launch eufs_launcher simulation.launch.py commandMode:=velocity track:=$TRACK gazebo_gui:=$GAZEBO_GUI rviz:=false launch_group:=$LAUNCH_GROUP publish_gt_tf:=$PUBLISH_GT_TF x:=$SPAWN_X y:=$SPAWN_Y yaw:=$SPAWN_YAW" Enter

# Wait for gzserver + plugins to fully initialize.
# complex tracks (small_track, peanut) take ~15 s; simple tracks (acceleration) ~5 s.
echo "==> Waiting 15 s for Gazebo to initialize..."
sleep 15

# If the EUFS state machine isn't up yet (car not spawned), do it manually.
source /opt/ros/humble/setup.bash
source $EUFS_WS/install/setup.bash
if ! timeout 3 ros2 topic echo /ros_can/state_str --once 2>/dev/null | grep -q AS:; then
    echo "==> Car not spawned by eufs_launcher — spawning manually at ($SPAWN_X, $SPAWN_Y, yaw=$SPAWN_YAW)..."
    ros2 run gazebo_ros spawn_entity.py \
        -entity eufs \
        -file $EUFS_WS/install/eufs_racecar/share/eufs_racecar/robots/eufs/robot.urdf \
        -x $SPAWN_X -y $SPAWN_Y -z 0.1 -R 0.0 -P 0.0 -Y $SPAWN_YAW \
        -timeout 30.0 --ros-args --log-level warn 2>/dev/null &
    SPAWN_PID=$!
    # Wait up to 20 s for state machine to appear
    for i in $(seq 10); do
        sleep 2
        timeout 2 ros2 topic echo /ros_can/state_str --once 2>/dev/null | grep -q AS: && break
        echo "==> Waiting for spawn... ($i/10)"
    done
    wait $SPAWN_PID 2>/dev/null || true
fi
echo "==> Gazebo ready."

# Pane 1 (mid-left) — MFE Bridge
tmux send-keys -t mfe:0.1 \
    "$SOURCE_ALL && ros2 launch mfe_eufs_sim mfe_eufs_sim.launch.py use_sim_cones_directly:=$USE_SIM_CONES" Enter

# Pane 2 (bottom-left) — MFE Stack
# In no_perception mode:
#   use_perception:=false  → skip boundary_extractor (bridge owns /planning/cones)
#   pose_topic:=/ground_truth/state_odom → consistent frame with Gazebo world / TF map
# In perception mode:
#   use_perception:=true (default) → boundary_extractor runs, EKF provides pose
if [ "$MODE" = "no_perception" ]; then
    BRINGUP_EXTRAS="use_perception:=false pose_topic:=/ground_truth/state_odom"
else
    BRINGUP_EXTRAS=""
fi
tmux send-keys -t mfe:0.2 \
    "$SOURCE_ALL && ros2 launch mfe_bringup bringup.launch.py mission:=$MISSION $BRINGUP_EXTRAS" Enter

# Pane 3 (top-right) — Foxglove Bridge
tmux send-keys -t mfe:0.3 \
    "$SOURCE_ALL && ros2 launch foxglove_bridge foxglove_bridge_launch.xml" Enter

# Pane 4 (mid-right) — Mission control: pre-filled, press Enter when sim is ready
# AS state machine: set ami_state (event) + as_state=READY(1) to start autonomous driving.
# DRIVING(2) is refused directly from OFF; READY triggers the car to accept /cmd inputs.
tmux send-keys -t mfe:0.4 "$SOURCE_ALL" Enter
tmux send-keys -t mfe:0.4 \
    "ros2 service call /ros_can/set_mission eufs_msgs/srv/SetCanState '{ami_state: $AMI_STATE, as_state: 1}'"

# Pane 5 (bottom-right) — Logger
tmux send-keys -t mfe:0.5 "$SOURCE_ALL" Enter
tmux send-keys -t mfe:0.5 \
    "ros2 topic echo /ground_truth/state | tee $LOG_DIR/${TRACK}_\$(date +%Y%m%d_%H%M%S).log" Enter

# Attach to session only when running in a real terminal
if [ -t 0 ]; then
    tmux attach-session -t mfe
else
    echo "==> tmux session 'mfe' started. Attach with: tmux attach-session -t mfe"
fi
