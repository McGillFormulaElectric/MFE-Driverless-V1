#!/bin/bash
# One headless simulated run of an event, scored by score_run.py (see README.md).
# Runs INSIDE the sim container. Run it from the host with:
#   docker exec -e ROS_DOMAIN_ID=12 mfe-sim-gpu-test bash <MFE_REPO>/tools/validation/sim_harness/run_sim_test.sh [event] [timeout_s]
# Must run inside the container: on the host, cleanup's tmux kill-server and launch_sim_local.sh's pkills hit the
# user's own sessions (killed every Claude session on 2026-10-13).
[ -f /.dockerenv ] || { echo "ERROR: run inside the sim container: docker exec $SIM_CONTAINER bash $0 $*" >&2; exit 1; }

# Paths (environment overrides; defaults assume the ~/Develop layout, same paths on host and container)
HARNESS=$(cd "$(dirname "$0")" && pwd)
MFE_REPO=${MFE_REPO:-$(cd "$HARNESS/../../.." && pwd)}
SIMWS=${SIMWS:-$HOME/Develop/simws}
EUFS_WS=${EUFS_WS:-$HOME/Develop/MFE26-eufs-sim}
EUFS_TRACKS_CSV=${EUFS_TRACKS_CSV:-$EUFS_WS/eufs_tracks/csv}
RESULTS=${RESULTS:-$HOME/mfe_sim_results}
export MFE_REPO SIMWS EUFS_WS EUFS_TRACKS_CSV RESULTS

# Private tmux server, so kill-server in cleanup only ever kills the sim's panes.
export MFE_TMUX_SOCKET=mfe_test
tmux() { command tmux -L "$MFE_TMUX_SOCKET" "$@"; }

EVENT=${1:-accel}; TIMEOUT=${2:-240}
COLOR="${COLOR_MODE:-geometric}"
export MISSION_OVERRIDE="${MISSION_OVERRIDE:-}" PLANNER_BACKEND="${PLANNER_BACKEND:-fasttube}"
export UNKNOWN_CONE_MODE="${UNKNOWN_CONE_MODE:-}"   # launch_sim_local.sh adds unknown_cone_mode:= when set
case $EVENT in
  accel)   TRACK=acceleration; AMI=11 ;;
  skidpad) TRACK=skidpad;      AMI=12; SKIDPAD=true; CONES_TOPIC=/skidpad/cones ;;
  peanut)  TRACK=peanut;       AMI=13 ;;
  its_a_mess) TRACK=its_a_mess; AMI=13 ;;
  # Autocross tracks. EVENT is passed to launch_sim_local.sh, so it must be one of its event names
  # (hairpins -> hairpins_increasing_difficulty); TRACK is the csv name used for scoring.
  small_track|comp_2021|boa_constrictor|garden_light) TRACK=$EVENT; AMI=13 ;;
  hairpins) TRACK=hairpins_increasing_difficulty; AMI=13 ;;
  *) echo "usage: $0 [accel|skidpad|peanut|its_a_mess|small_track|comp_2021|hairpins|boa_constrictor|garden_light] [timeout_s]"; exit 1 ;;
esac

LOG=$(mktemp -d "${TMPDIR:-/tmp}/mfe_sim_test.XXXXXX"); echo "logs: $LOG"
REC=${REC:-$RESULTS/bags/single_${EVENT}_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$(dirname "$REC")"
ENVSH=$LOG/env.sh
cat > $ENVSH <<ENV
source /opt/ros/humble/setup.bash
source $SIMWS/install/setup.bash
export GAZEBO_MASTER_URI=http://localhost:${GAZEBO_PORT:-11350}
export EUFS_MASTER=$EUFS_WS
ENV

cleanup() {
  echo "==> cleaning up"
  [ -n "$STACK_PID" ] && kill -INT -- -$STACK_PID 2>/dev/null
  pkill -INT -f 'ros2 bag record' 2>/dev/null; sleep 3   # recorder runs in its own session; SIGINT lets it write metadata.yaml
  tmux kill-server 2>/dev/null
  pkill -x gzserver 2>/dev/null; pkill -x gzclient 2>/dev/null
  sleep 2; [ -n "$STACK_PID" ] && kill -KILL -- -$STACK_PID 2>/dev/null
}
trap cleanup EXIT

# 1. sim, headless, car driven from /slam/odom
( cd "$MFE_REPO" && SHELL=/bin/bash setsid bash "$MFE_REPO/tools/validation/launch_sim_local.sh" $EVENT lidar_only nogui ${NUM_LAPS:-1} ground_truth > $LOG/sim.log 2>&1 ) &
for i in $(seq 1 60); do grep -q "Gazebo ready" $LOG/sim.log 2>/dev/null && break; sleep 3; done
grep -q "Gazebo ready" $LOG/sim.log || { echo "sim did not come up, see $LOG/sim.log"; exit 1; }
sleep 15

# 2. LiDAR detector + SLAM
# SLAM lateral velocity (corner sideslip in the odometry): on for autocross/trackdrive, where it fixed comp_2021's
# heading drift (8/8 clean on the regression tracks); off for accel/skidpad, where it made accel worse (3/6 clean,
# up to 1.1 m sideways, vs 4/4 clean without). SLAM_LATVEL overrides.
case "$EVENT" in accel|acceleration|skidpad) LATVEL_DEFAULT=false ;; *) LATVEL_DEFAULT=true ;; esac
setsid bash -c "source $ENVSH; exec ros2 launch mfe_validation lidar_only.launch.py color_mode:=$COLOR min_obs:=${MIN_OBS:-2} skidpad:=${SKIDPAD:-false} cones_topic:=${CONES_TOPIC:-/planning/cones} use_gpu:=${USE_GPU:-true} slam_lateral_vel:=${SLAM_LATVEL:-$LATVEL_DEFAULT}" > $LOG/stack.log 2>&1 &
STACK_PID=$!
for i in $(seq 1 20); do grep -q "slam_node up" $LOG/stack.log 2>/dev/null && break; sleep 2; done
sleep 15

rm -rf "$REC"
setsid bash -c "source $ENVSH; exec ros2 bag record -o $REC /lidar/points_raw /slam/odom /planning/cones /skidpad/cones /planning/centerline /ground_truth/state_odom /control/command /cmd /ground_truth/state" > $LOG/rec.log 2>&1 &

# 3. start scorer, then the mission, and wait for FINISHED / timeout
bash -c "source $ENVSH; python3 $HARNESS/score_run.py $TIMEOUT $TRACK" > $LOG/score.out 2>&1 &
SC=$!
sleep 3
bash -c "source $ENVSH; timeout 40 ros2 service call /ros_can/set_mission eufs_msgs/srv/SetCanState '{ami_state: $AMI, as_state: 1}'" > /dev/null 2>&1
wait $SC
echo; echo "===== $EVENT, LiDAR + GTSAM SLAM only (sim container) ====="
grep -vE "^\s*$|rosbag2_storage" $LOG/score.out
echo "--- SLAM node (last accuracy line):"
grep "SLAM pose err" $LOG/stack.log | tail -1 | sed -E 's/^.*slam_node\]: //'
