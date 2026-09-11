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

set -eo pipefail
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

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
    peanut)
        TRACK=peanut
        MISSION=peanut
        AMI_STATE=13  # AMI_AUTOCROSS
        BRIDGE_MAX_SPEED=8.0    # matches bringup pure_pursuit max_speed for peanut (autocross speed)
        ;;
    autocross|small_track|rectangle|garden_light|boa_constrictor|comp_2021|hairpins|rand|its_a_mess)
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
        echo "Unknown event '$EVENT'. Use: accel, skidpad, peanut, autocross, small_track, rectangle, garden_light, boa_constrictor, comp_2021, hairpins, rand"
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
[[ "$LAPS" =~ ^[0-9]+$ ]] || { echo "laps must be a nonnegative integer" >&2; exit 1; }
if [ "$GAZEBO_GUI" = true ] && { [ -z "${DISPLAY:-}" ] || [ "${MFE_VIRTUAL_DISPLAY:-0}" = 1 ]; }; then
    echo "GUI needs a host display. Use scripts/docker_run.sh with gui, or select nogui." >&2
    exit 1
fi

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

# Keep other simulations/processes intact; this session must have a unique name.
if tmux has-session -t mfe 2>/dev/null; then
    echo "tmux session mfe already exists. Stop it before starting another run." >&2
    exit 1
fi
RUN_DIR=$(mktemp -d /tmp/mfe-launch-XXXXXX)
source /opt/ros/humble/setup.bash
source "$EUFS_WS/install/setup.bash"
source "$MFE_WS/install/setup.bash"

# Create new session
tmux new-session -d -s mfe -x 220 -y 50

# Split into 6 panes: 3 left, 3 right
tmux split-window -h -t mfe          # left | right
tmux split-window -v -t mfe:0.0      # top-left | mid-left
tmux split-window -v -t mfe:0.0      # mid-left | bottom-left
tmux split-window -v -t mfe:0.1      # top-right | mid-right
tmux split-window -v -t mfe:0.3      # mid-right | bottom-right

# Pane 0 (top-left) — EUFS world, car spawning, and optional Gazebo GUI.
# Pass x/y/yaw so eufs_launcher uses the correct start position for this track.
tmux send-keys -t mfe:0.0 \
    "($SOURCE_ALL && ros2 launch eufs_launcher simulation.launch.py commandMode:=velocity track:=$TRACK vehicleModelConfig:=configDry.yaml gazebo_gui:=$GAZEBO_GUI show_rqt_gui:=false rviz:=false launch_group:=$LAUNCH_GROUP publish_gt_tf:=$PUBLISH_GT_TF x:=$SPAWN_X y:=$SPAWN_Y yaw:=$SPAWN_YAW) 2>&1 | tee $RUN_DIR/simulator.log; touch $RUN_DIR/simulator.exited" Enter

echo "==> Waiting for car state and required sensor messages..."
if ! python3 "$SCRIPT_DIR/wait_for_sim.py" --mode "$MODE" \
    --timeout "${MFE_STARTUP_TIMEOUT:-120}" --exit-marker "$RUN_DIR/simulator.exited"; then
    echo "==> Launch failed. Simulator output:" >&2
    tail -n 80 "$RUN_DIR/simulator.log" >&2
    tmux kill-session -t mfe 2>/dev/null || true
    exit 1
fi

# Pane 1 (mid-left) — MFE Bridge
tmux send-keys -t mfe:0.1 \
    "$SOURCE_ALL && ros2 launch mfe_eufs_sim mfe_eufs_sim.launch.py use_sim_cones_directly:=$USE_SIM_CONES max_speed_ms:=$BRIDGE_MAX_SPEED max_steering_deg:=28.0" Enter

# Pane 2 (bottom-left) — MFE Stack
# pose_topic:=/sim/xsens/state_odom → same map-frame header as raw ground truth
# (mfe_eufs_sim/xsens_noise_node.py just perturbs the pose/twist values), but noise-
# corrupted to be representative of the real Xsens MTi-670G instead of perfect GT —
# see mfe_eufs_sim.launch.py. Do NOT point this back at /ground_truth/state_odom.
#
# In no_perception mode:
#   use_perception:=false  → skip boundary_extractor (bridge owns /planning/cones)
# In perception mode:
#   use_perception:=true (default) → boundary_extractor runs, EKF provides pose
if [ "$MODE" = "no_perception" ]; then
    # GT cones go straight to /planning/cones — no perception, SLAM, or EKF needed.
    # run_perception:=false disables the entire perception group (lidar, vision, evaluators).
    BRINGUP_EXTRAS="use_perception:=false run_perception:=false pose_topic:=/sim/xsens/state_odom use_slam:=false use_ekf:=false"
else
    # Perception sim: noisy Xsens-representative odometry, disable SLAM/EKF (EUFS GT TF owns map→odom).
    BRINGUP_EXTRAS="pose_topic:=/sim/xsens/state_odom use_slam:=false use_ekf:=false"
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
