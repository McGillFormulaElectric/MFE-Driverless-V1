#!/bin/bash
# =============================================================================
# Launch N independent Gazebo + MFE stack instances for parallel RL training.
#
# Each instance gets:
#   ROS_DOMAIN_ID = 42 + rank
#   GAZEBO_PORT   = 11350 + rank
#
# Usage:
#   bash scripts/start_rl_envs.sh <N> [track]
#
#   N      — number of parallel environments (default: 1)
#   track  — EUFS track name (default: peanut)
#
# After all sims are up, run training:
#   python3 training/rl/train_ppo.py --n-envs <N>
# =============================================================================

N=${1:-1}
TRACK=${2:-peanut}

GAZEBO_ROS_WS=~/Develop/gazebo_ros_pkgs
EUFS_WS=~/Develop/MFE26-eufs-sim
MFE_WS=~/Develop/MFE-Driverless-V1/ros2

# Kill any existing RL training session
tmux kill-session -t mfe_rl 2>/dev/null || true
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
pkill -f "ros2 launch" 2>/dev/null || true
sleep 2

source /opt/ros/humble/setup.bash
ros2 daemon stop 2>/dev/null || true
ros2 daemon start

# Read spawn pose from track CSV
CSV_FILE="$EUFS_WS/eufs_tracks/csv/${TRACK}.csv"
if [ -f "$CSV_FILE" ] && grep -q "^car_start" "$CSV_FILE"; then
    IFS=',' read -r _ SPAWN_X SPAWN_Y SPAWN_YAW _ < <(grep "^car_start" "$CSV_FILE")
else
    SPAWN_X=0.0; SPAWN_Y=0.0; SPAWN_YAW=0.0
fi

echo "==> Launching $N RL environment(s) on track: $TRACK"
echo "==> Spawn: x=$SPAWN_X y=$SPAWN_Y yaw=$SPAWN_YAW"

tmux new-session -d -s mfe_rl -x 220 -y 50

for i in $(seq 0 $((N - 1))); do
    DOMAIN=$((42 + i))
    PORT=$((11350 + i))
    PANE=$i

    if [ $i -gt 0 ]; then
        tmux new-window -t mfe_rl
    fi

    SOURCE_ENV="export ROS_DOMAIN_ID=$DOMAIN && \
                export GAZEBO_MASTER_URI=http://localhost:$PORT && \
                export EUFS_MASTER=$EUFS_WS && \
                source /opt/ros/humble/setup.bash && \
                ([ -f $GAZEBO_ROS_WS/install/setup.bash ] && source $GAZEBO_ROS_WS/install/setup.bash || true) && \
                source $EUFS_WS/install/setup.bash && \
                source $MFE_WS/install/setup.bash"

    echo "==> Starting env $i (ROS_DOMAIN_ID=$DOMAIN, GAZEBO_PORT=$PORT)..."

    tmux send-keys -t mfe_rl:$i \
        "$SOURCE_ENV && ros2 launch eufs_launcher simulation.launch.py \
        commandMode:=velocity track:=$TRACK gazebo_gui:=false rviz:=false \
        launch_group:=no_perception publish_gt_tf:=true \
        x:=$SPAWN_X y:=$SPAWN_Y yaw:=$SPAWN_YAW" Enter

    # Stagger launches so they don't all hammer disk/network at once
    sleep 5
done

echo "==> Waiting 30s for all Gazebo instances to fully initialise..."
sleep 30

# Start the MFE bringup (compute only — no perception) for each env
for i in $(seq 0 $((N - 1))); do
    DOMAIN=$((42 + i))
    PORT=$((11350 + i))

    SOURCE_ENV="export ROS_DOMAIN_ID=$DOMAIN && \
                export GAZEBO_MASTER_URI=http://localhost:$PORT && \
                export EUFS_MASTER=$EUFS_WS && \
                source /opt/ros/humble/setup.bash && \
                source $EUFS_WS/install/setup.bash && \
                source $MFE_WS/install/setup.bash"

    # Split each window to show bringup alongside sim
    tmux split-window -v -t mfe_rl:$i
    tmux send-keys -t mfe_rl:$i \
        "$SOURCE_ENV && ros2 launch mfe_bringup bringup.launch.py \
        mission:=autocross \
        pose_topic:=/ground_truth/state_odom \
        use_perception:=false use_slam:=false use_ekf:=false \
        run_compute:=true run_perception:=false \
        endless:=true" Enter
done

echo ""
echo "==> All $N environments launched."
echo "==> Attach to session:  tmux attach-session -t mfe_rl"
echo ""
echo "==> Now run training:"
echo "    python3 training/rl/train_ppo.py --n-envs $N"
echo ""
echo "==> Monitor in browser:"
echo "    tensorboard --logdir ~/mfe_models/rl/tb_logs/"
echo "    Open: http://localhost:6006"
