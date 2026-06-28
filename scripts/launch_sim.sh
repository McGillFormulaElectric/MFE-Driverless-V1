#!/bin/bash
# =============================================================================
# MFE Driverless — Full Simulation Launch Script
# Opens tmux with 6 panes and launches all components.
#
# Usage: bash scripts/launch_sim.sh [track] [mode] [gui] [laps]
#   track         — accel (default), skidpad, peanut, small_track, rectangle, ...
#   mode          — no_perception (default) | perception
#   gui           — gui (default) | nogui
#   laps          — number of laps before stopping at orange-cone finish gate (default: 1)
#                   On closed-loop tracks (autocross/peanut) each return to start = 1 lap.
#                   Use 0 to run indefinitely (same as endless mode).
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
        BRIDGE_MAX_SPEED=13.0   # matches bringup pure_pursuit max_speed for acceleration
        ;;
    skidpad)
        TRACK=skidpad
        MISSION=skidpad
        AMI_STATE=12  # AMI_SKIDPAD
        BRIDGE_MAX_SPEED=4.5    # matches bringup pure_pursuit max_speed for skidpad
        ;;
    autocross|small_track|peanut|rectangle|garden_light|boa_constrictor|comp_2021|hairpins|rand|its_a_mess)
        case "$EVENT" in
            autocross)    TRACK=small_track ;;
            hairpins)     TRACK=hairpins_increasing_difficulty ;;
            *)            TRACK=$EVENT ;;
        esac
        MISSION=autocross
        AMI_STATE=13  # AMI_AUTOCROSS
        BRIDGE_MAX_SPEED=10.0   # matches bringup pure_pursuit max_speed for autocross/trackdrive
        ;;
    *)
        echo "Unknown event '$EVENT'. Use: accel, skidpad, autocross, small_track, peanut, rectangle, garden_light, boa_constrictor, comp_2021, hairpins, rand"
        exit 1
        ;;
esac

# Read spawn pose from the track CSV (car_start row: tag,x,y,yaw,...)
CSV_FILE="$EUFS_WS/eufs_tracks/csv/${TRACK}.csv"
if [ -f "$CSV_FILE" ] && grep -q "^car_start" "$CSV_FILE"; then
    IFS=',' read -r _ SPAWN_X SPAWN_Y SPAWN_YAW _ < <(grep "^car_start" "$CSV_FILE")
else
    SPAWN_X=0.0; SPAWN_Y=0.0; SPAWN_YAW=0.0
fi

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

# Parse laps (0 = endless)
LAPS=${4:-1}

echo "==> Launching event: $TRACK (ami_state=$AMI_STATE) | mode: $MODE | gazebo_gui: $GAZEBO_GUI | laps: $LAPS"

# Create log directory
mkdir -p $LOG_DIR

export EUFS_MASTER=$EUFS_WS
# Use a non-default port to avoid conflicts (override with GAZEBO_PORT env var)
GAZEBO_PORT=${GAZEBO_PORT:-11350}
export GAZEBO_MASTER_URI=http://localhost:$GAZEBO_PORT

SOURCE_ALL="export EUFS_MASTER=$EUFS_WS && \
            export GAZEBO_MASTER_URI=http://localhost:$GAZEBO_PORT && \
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
echo "==> Waiting 25 s for Gazebo to initialize..."
sleep 25

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
    "$SOURCE_ALL && ros2 launch mfe_eufs_sim mfe_eufs_sim.launch.py use_sim_cones_directly:=$USE_SIM_CONES max_speed_ms:=$BRIDGE_MAX_SPEED max_steering_deg:=28.0" Enter

# Pane 2 (bottom-left) — MFE Stack
# In no_perception mode:
#   use_perception:=false  → skip boundary_extractor (bridge owns /planning/cones)
#   pose_topic:=/ground_truth/state_odom → consistent frame with Gazebo world / TF map
# In perception mode:
#   use_perception:=true (default) → boundary_extractor runs, EKF provides pose
if [ "$MODE" = "no_perception" ]; then
    # EUFS GT TF already publishes map→odom; SLAM and EKF must be disabled to avoid TF conflict.
    BRINGUP_EXTRAS="use_perception:=false pose_topic:=/ground_truth/state_odom use_slam:=false use_ekf:=false"
else
    # Perception sim: same GT odometry + no SLAM/EKF (EUFS GT TF owns map→odom).
    BRINGUP_EXTRAS="pose_topic:=/ground_truth/state_odom use_slam:=false use_ekf:=false"
fi

# Laps: 0 means endless (disable finish detector), otherwise pass num_laps
if [ "$LAPS" = "0" ]; then
    BRINGUP_EXTRAS="$BRINGUP_EXTRAS endless:=true"
else
    BRINGUP_EXTRAS="$BRINGUP_EXTRAS num_laps:=$LAPS"
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
