#!/bin/bash
# One-command headless run: EUFS sim (no GUI) + LiDAR detector + GTSAM cone SLAM driving the car, scored against ground truth.
#
#   scripts/run_slam_headless.sh [accel|skidpad|peanut] [timeout_s]
#
# The car's planner gets cones from slam_node and pose from /slam/odom (mode lidar_only). Colours use a geometric left/right rule,
# so only accel (straight lane, blue left / yellow right) is meaningful; skidpad/peanut use hardcoded planner paths anyway.
# Needs: ~/Develop/simws built (colcon), tmux, gtsam (pip --user --no-deps), a built MFE26-eufs-sim/MFE workspace.
# Never launch it from a shell whose command line contains "ros2" (launch_sim_local.sh runs `pkill -f ros2`).
EVENT=${1:-accel}; TIMEOUT=${2:-240}
ROOT=$HOME/Develop; SIMWS=$ROOT/simws; CUDAPCL=$ROOT/cudaPCL
COLOR=geometric; export MISSION_OVERRIDE=""
case $EVENT in
  accel)   TRACK=acceleration; AMI=11 ;;
  skidpad) TRACK=skidpad;      AMI=12; SKIDPAD=true; CONES_TOPIC=/skidpad/cones ;;   # cone-fitted skidpad planner (skidpad_planner), stock hardcoded planner stays silent
  # peanut: run the planner in its cone-based autocross mode (NOT its hardcoded peanut path); cones uncoloured (sim LiDAR has no intensity)
  peanut)  TRACK=peanut;       AMI=13; COLOR=${COLOR_OVERRIDE:-unknown}; export MISSION_OVERRIDE=autocross ;;
  # any other closed-loop track in eufs_tracks/csv (e.g. boa_constrictor), cone-based autocross planner, uncoloured cones
  *) [ -f $ROOT/MFE26-eufs-sim/eufs_sim/eufs_tracks/csv/$EVENT.csv ] || { echo "usage: $0 [accel|skidpad|<track name>] [timeout_s]"; exit 1; }
     TRACK=$EVENT; AMI=13; COLOR=${COLOR_OVERRIDE:-unknown}; export MISSION_OVERRIDE=autocross ;;
esac
LOG=$(mktemp -d /tmp/slam_headless.XXXXXX); echo "logs: $LOG"
ENVSH=$LOG/env.sh
cat > $ENVSH <<ENV
source /opt/ros/humble/setup.bash
source $SIMWS/install/setup.bash
export GAZEBO_MASTER_URI=http://localhost:11350
export EUFS_MASTER=$ROOT/MFE26-eufs-sim
ENV

cleanup() {
  echo "==> cleaning up"
  [ -n "$STACK_PID" ] && kill -INT -- -$STACK_PID 2>/dev/null
  tmux kill-server 2>/dev/null
  pkill -x gzserver 2>/dev/null; pkill -x gzclient 2>/dev/null
  sleep 2; [ -n "$STACK_PID" ] && kill -KILL -- -$STACK_PID 2>/dev/null
}
trap cleanup EXIT

# 1. sim, headless (nogui), car driven from /slam/odom
( cd $ROOT/MFE-Driverless-V1 && SHELL=/bin/bash setsid bash $SIMWS/launch_sim_local.sh $EVENT ${MODE:-lidar_only} ${GUI:-nogui} ${LAPS:-1} ground_truth > $LOG/sim.log 2>&1 ) &
for i in $(seq 1 60); do grep -q "Gazebo ready" $LOG/sim.log 2>/dev/null && break; sleep 3; done
grep -q "Gazebo ready" $LOG/sim.log || { echo "sim did not come up, see $LOG/sim.log"; exit 1; }
sleep 15   # let the bridge + control stack finish starting

# 2. LiDAR detector + SLAM (own process group so cleanup can stop it)
if [ "${MODE:-lidar_only}" = lidar_only ]; then
setsid bash -c "source $ENVSH; exec ros2 launch mfe_validation lidar_only.launch.py color_mode:=$COLOR min_obs:=${MIN_OBS:-3} freeze:=${FREEZE:-true} skidpad:=${SKIDPAD:-false} cones_topic:=${CONES_TOPIC:-/planning/cones}" > $LOG/stack.log 2>&1 &
STACK_PID=$!
for i in $(seq 1 20); do grep -q "slam_node up" $LOG/stack.log 2>/dev/null && break; sleep 2; done
sleep 15   # first landmarks while stationary
fi   # MODE=no_perception: control run, ground-truth cones + pose, no LiDAR stack

# optional: REC=/path/bag records what the video renderer needs
[ -n "$REC" ] && { rm -rf "$REC"; setsid bash -c "source $ENVSH; exec ros2 bag record -o $REC /lidar/points_raw /slam/odom /planning/cones /skidpad/cones /planning/centerline /ground_truth/state_odom" > $LOG/rec.log 2>&1 & }

# 3. start scorer, then the mission, and wait for FINISHED / timeout
bash -c "source $ENVSH; python3 $CUDAPCL/eval/score_run.py $TIMEOUT $TRACK" > $LOG/score.out 2>&1 &
SC=$!
sleep 3
bash -c "source $ENVSH; timeout 40 ros2 service call /ros_can/set_mission eufs_msgs/srv/SetCanState '{ami_state: $AMI, as_state: 1}'" > /dev/null 2>&1
wait $SC
echo; echo "===== $EVENT, LiDAR + GTSAM SLAM only ====="
grep -vE "^\s*$|rosbag2_storage" $LOG/score.out
echo "--- SLAM node (last accuracy line):"
grep "SLAM pose err" $LOG/stack.log | tail -1 | sed -E 's/^.*slam_node\]: //'
