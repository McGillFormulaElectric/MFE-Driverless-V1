#!/bin/bash
# Build the car stack for bench runs into ros2/build_bench and ros2/install_bench.
# Separate folders, so stale artifacts in ros2/build cannot break the link (seen 2026-11-10).
# Usage: scripts/bench/build_bench.sh [extra colcon args]
set -eu
REPO=$(cd "$(dirname "$0")/../.." && pwd)
cd "$REPO/ros2"
set +u; source /opt/ros/humble/setup.bash; set -u   # ROS setup reads unset variables
MAKEFLAGS=${MAKEFLAGS:--j3} colcon build --symlink-install \
    --build-base build_bench --install-base install_bench \
    --parallel-workers 2 --event-handlers console_direct- \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
    --packages-up-to mfe_bringup mfe_validation mfe_control mfe_sensors vision_cone_detector \
                     mfe_state_estimation mfe_path_planning lidar_cone_detector perception_evaluator \
    "$@"
