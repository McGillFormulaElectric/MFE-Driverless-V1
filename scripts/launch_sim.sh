#!/bin/bash
# Full simulation launcher: opens a tmux session with 6 panes and starts all components.
#
# Usage: bash scripts/launch_sim.sh [track] [mode] [gui] [laps]
#   track   — accel (default), skidpad, peanut, small_track, rectangle, ...
#   mode    — no_perception (default) | perception
#   gui     — gui (default) | nogui
#   laps    — laps before stopping at the orange-cone finish gate (default 1).
#             On closed-loop tracks each return to start = 1 lap. 0 = endless.

GAZEBO_ROS_WS=~/Develop/gazebo_ros_pkgs
EUFS_WS=~/Develop/MFE26-eufs-sim
MFE_WS=~/Develop/MFE-Driverless-V1/ros2
LOG_DIR=~/mfe_logs

EVENT=${1:-accel}
# BRIDGE_MAX_SPEED must match the bringup pure_pursuit max_speed for each track.
case "$EVENT" in
    accel|acceleration)
        TRACK=acceleration
        MISSION=acceleration
        AMI_STATE=11
        BRIDGE_MAX_SPEED=13.0
        ;;
    skidpad)
        TRACK=skidpad
        MISSION=skidpad
        AMI_STATE=12
        BRIDGE_MAX_SPEED=4.5
        ;;
    peanut)
        TRACK=peanut
        MISSION=peanut
        AMI_STATE=13
        BRIDGE_MAX_SPEED=6.0
        ;;
    autocross|small_track|rectangle|garden_light|boa_constrictor|comp_2021|hairpins|rand|its_a_mess)
        case "$EVENT" in
            autocross)    TRACK=small_track ;;
            hairpins)     TRACK=hairpins_increasing_difficulty ;;
            *)            TRACK=$EVENT ;;
        esac
        MISSION=autocross
        AMI_STATE=13
        BRIDGE_MAX_SPEED=8.0
        ;;
    *)
        echo "Unknown event '$EVENT'. Use: accel, skidpad, peanut, autocross, small_track, rectangle, garden_light, boa_constrictor, comp_2021, hairpins, rand"
        exit 1
        ;;
esac

# car_start row of the track CSV: tag,x,y,yaw,...
CSV_FILE="$EUFS_WS/eufs_tracks/csv/${TRACK}.csv"
if [ -f "$CSV_FILE" ] && grep -q "^car_start" "$CSV_FILE"; then
    IFS=',' read -r _ SPAWN_X SPAWN_Y SPAWN_YAW _ < <(grep "^car_start" "$CSV_FILE")
else
    SPAWN_X=0.0; SPAWN_Y=0.0; SPAWN_YAW=0.0
fi

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

GUI_ARG=${3:-gui}
case "$GUI_ARG" in
    gui)   GAZEBO_GUI=true  ;;
    nogui) GAZEBO_GUI=false ;;
    *)
        echo "Unknown gui arg '$GUI_ARG'. Use: gui or nogui"
        exit 1
        ;;
esac

LAPS=${4:-1}

echo "==> Launching event: $TRACK (ami_state=$AMI_STATE) | mode: $MODE | gazebo_gui: $GAZEBO_GUI | laps: $LAPS"

mkdir -p $LOG_DIR

export EUFS_MASTER=$EUFS_WS
# non-default port to avoid clashing with other Gazebo instances; override via GAZEBO_PORT
GAZEBO_PORT=${GAZEBO_PORT:-11350}
export GAZEBO_MASTER_URI=http://localhost:$GAZEBO_PORT

SOURCE_ALL="export EUFS_MASTER=$EUFS_WS && \
            export GAZEBO_MASTER_URI=http://localhost:$GAZEBO_PORT && \
            source /opt/ros/humble/setup.bash && \
            ([ -f $GAZEBO_ROS_WS/install/setup.bash ] && source $GAZEBO_ROS_WS/install/setup.bash || true) && \
            source $EUFS_WS/install/setup.bash && \
            source $MFE_WS/install/setup.bash"

tmux kill-session -t mfe 2>/dev/null || true
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
pkill -f ros2 2>/dev/null || true
pkill -f path_planner_node 2>/dev/null || true
pkill -f lidar_perception_node 2>/dev/null || true
pkill -f static_transform_publisher 2>/dev/null || true
sleep 1

source /opt/ros/humble/setup.bash
ros2 daemon stop 2>/dev/null || true
ros2 daemon start

tmux new-session -d -s mfe -x 220 -y 50

tmux split-window -h -t mfe
tmux split-window -v -t mfe:0.0
tmux split-window -v -t mfe:0.0
tmux split-window -v -t mfe:0.1
tmux split-window -v -t mfe:0.3

# x/y/yaw put eufs_launcher at this track's start pose.
tmux send-keys -t mfe:0.0 \
    "$SOURCE_ALL && ros2 launch eufs_launcher simulation.launch.py commandMode:=velocity track:=$TRACK gazebo_gui:=$GAZEBO_GUI rviz:=false launch_group:=$LAUNCH_GROUP publish_gt_tf:=$PUBLISH_GT_TF x:=$SPAWN_X y:=$SPAWN_Y yaw:=$SPAWN_YAW" Enter

echo "==> Waiting 25 s for Gazebo to initialize..."
sleep 25

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
    for i in $(seq 10); do
        sleep 2
        timeout 2 ros2 topic echo /ros_can/state_str --once 2>/dev/null | grep -q AS: && break
        echo "==> Waiting for spawn... ($i/10)"
    done
    wait $SPAWN_PID 2>/dev/null || true
fi
echo "==> Gazebo ready."

tmux send-keys -t mfe:0.1 \
    "$SOURCE_ALL && ros2 launch mfe_eufs_sim mfe_eufs_sim.launch.py use_sim_cones_directly:=$USE_SIM_CONES max_speed_ms:=$BRIDGE_MAX_SPEED max_steering_deg:=28.0" Enter

# pose_topic: noisy Xsens-like odom, not ground truth. Do not point it back at /ground_truth/state_odom.
# no_perception: use_perception:=false skips boundary_extractor (bridge owns /planning/cones).
# run_perception:=false disables the whole perception group.
if [ "$MODE" = "no_perception" ]; then
    BRINGUP_EXTRAS="use_perception:=false run_perception:=false pose_topic:=/sim/xsens/state_odom use_slam:=false use_ekf:=false"
else
    # SLAM/EKF off: EUFS GT TF owns map->odom.
    BRINGUP_EXTRAS="pose_topic:=/sim/xsens/state_odom use_slam:=false use_ekf:=false"
fi

if [ "$LAPS" = "0" ]; then
    BRINGUP_EXTRAS="$BRINGUP_EXTRAS endless:=true"
else
    BRINGUP_EXTRAS="$BRINGUP_EXTRAS num_laps:=$LAPS"
fi
tmux send-keys -t mfe:0.2 \
    "$SOURCE_ALL && ros2 launch mfe_bringup bringup.launch.py mission:=$MISSION $BRINGUP_EXTRAS" Enter

tmux send-keys -t mfe:0.3 \
    "$SOURCE_ALL && ros2 launch foxglove_bridge foxglove_bridge_launch.xml" Enter

# Mission pane is pre-filled; press Enter once the sim is ready.
# as_state=READY(1) lets the car accept /cmd inputs (DRIVING(2) is refused from OFF).
tmux send-keys -t mfe:0.4 "$SOURCE_ALL" Enter
tmux send-keys -t mfe:0.4 \
    "ros2 service call /ros_can/set_mission eufs_msgs/srv/SetCanState '{ami_state: $AMI_STATE, as_state: 1}'"

tmux send-keys -t mfe:0.5 "$SOURCE_ALL" Enter
tmux send-keys -t mfe:0.5 \
    "ros2 topic echo /ground_truth/state | tee $LOG_DIR/${TRACK}_\$(date +%Y%m%d_%H%M%S).log" Enter

if [ -t 0 ]; then
    tmux attach-session -t mfe
else
    echo "==> tmux session 'mfe' started. Attach with: tmux attach-session -t mfe"
fi
