#!/bin/bash
# Stationary bench run of the car stack on one computer (Jetson 2: VLP-16 + D435i attached).
# Starts the sensor drivers, a stationary odometry stand-in (no IMU/GPS on the bench), the full
# stack (perception, gtsam SLAM, planner, pure pursuit; no CAN, so nothing actuates) and a
# Foxglove bridge on port 8765. Each part has its own process group and log.
#
# Usage: scripts/bench/start_bench.sh            Stop: scripts/bench/stop_bench.sh
# Environment overrides:
#   MISSION=autocross        planner mission
#   DRIVERS=lidar,camera     sensor drivers to start (empty: none, e.g. desktop test)
#   LIDAR_IP=10.37.1.194     VLP-16 address (the driver drops packets from other addresses)
#   ENGINE=<.engine|.pt>     YOLO weights; missing file: YOLO is skipped with a warning
#   FOXGLOVE=1               0: no Foxglove bridge
# Build first: scripts/bench/build_bench.sh
set -u
REPO=$(cd "$(dirname "$0")/../.." && pwd)
LOCK=$HOME/jetson_busy.lock
RUNS=$HOME/mfe_bench_runs
MISSION=${MISSION:-autocross}
DRIVERS=${DRIVERS-lidar,camera}
LIDAR_IP=${LIDAR_IP:-10.37.1.194}
ENGINE=${ENGINE:-$HOME/mfe_models/yolo/cone_detector_best_24a2d1ce.engine}
FOXGLOVE=${FOXGLOVE:-1}

[ -f "$REPO/ros2/install_bench/setup.bash" ] || { echo "no ros2/install_bench: run scripts/bench/build_bench.sh"; exit 1; }
if [ -f "$RUNS/latest/pids" ] && [ ! -f "$RUNS/latest/stopped" ]; then
    echo "a bench run is already up ($RUNS/latest); run scripts/bench/stop_bench.sh first"; exit 1
fi
flock -n "$LOCK" true || { echo "busy lock $LOCK held; another run is using this computer"; exit 1; }
if [ -f "$ENGINE" ]; then
    VISION="vision_model_path:=$ENGINE"
else
    echo "WARNING: YOLO weights $ENGINE not found; running without YOLO"; VISION=""
fi

RUN=$RUNS/$(date +%Y%m%d_%H%M%S)
mkdir -p "$RUN"; ln -sfn "$RUN" "$RUNS/latest"
git -C "$REPO" log -1 --format='%h %s' > "$RUN/commit.txt"
SETUP="BENCH_REPO=$REPO source $REPO/scripts/bench/bench_env.sh"

start() {  # start <name> <command>: own process group, log in $RUN/<name>.log
    local name=$1; shift
    setsid nohup bash -c "$SETUP && exec $*" > "$RUN/$name.log" 2>&1 < /dev/null &
    echo "$name $!" >> "$RUN/pids"
    echo "started $name (pgid $!) -> $RUN/$name.log"
}

case ",$DRIVERS," in *,lidar,*)  start velodyne  "ros2 launch mfe_sensors velodyne.launch.py device_ip:=$LIDAR_IP" ;; esac
case ",$DRIVERS," in *,camera,*) start realsense "ros2 launch mfe_sensors realsense.launch.py" ;; esac
start bench_odom "python3 $REPO/scripts/bench/bench_odom.py"
start stack "flock -n $LOCK ros2 launch mfe_bringup bringup.launch.py platform:=car part:=all \
    run_perception:=true run_compute:=true slam:=gtsam sensors:=lidar,camera start_drivers:=false \
    mission:=$MISSION endless:=true $VISION"
[ "$FOXGLOVE" = 1 ] && start foxglove "ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"
echo "run dir: $RUN   (commit $(cut -c1-7 "$RUN/commit.txt"), mission $MISSION, drivers '${DRIVERS}', ${VISION:-no YOLO})"
