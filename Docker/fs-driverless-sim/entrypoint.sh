#!/bin/bash
# =============================================================================
# Container entrypoint — builds workspaces on first run, then execs CMD.
# =============================================================================

set -e

EUFS_WS=/root/Develop/MFE26-eufs-sim
MFE_WS=/root/Develop/MFE-Driverless-V1/ros2

source /opt/ros/humble/setup.bash

# Build EUFS sim if not already built, or if key packages are missing from a stale build
if [ ! -f "$EUFS_WS/install/eufs_plugins/share/eufs_plugins/local_setup.bash" ] || \
   [ ! -f "$EUFS_WS/install/eufs_msgs/share/eufs_msgs/local_setup.bash" ]; then
    echo "==> [entrypoint] Building EUFS sim..."
    cd "$EUFS_WS"
    rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true
    colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
fi

source "$EUFS_WS/install/setup.bash"

# Sentinel: C++ lidar detector binary.  Checked both for existence and freshness vs source.
LIDAR_BIN="$MFE_WS/install/lidar_cone_detector/lib/lidar_cone_detector/lidar_perception_node"
LIDAR_SRC="$MFE_WS/src/mfe_perception/lidar_cone_detector/src/lidar_perception_node.cpp"

# Build MFE stack if not already built
if [ ! -d "$MFE_WS/install" ]; then
    echo "==> [entrypoint] Building MFE stack (first run)..."
    cd "$MFE_WS"
    rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true
    colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
elif [ ! -f "$LIDAR_BIN" ] || [ "$LIDAR_SRC" -nt "$LIDAR_BIN" ]; then
    # Source changed since last build (e.g. Z-filter fix) — rebuild only the C++ package.
    # Python packages with --symlink-install update automatically; only C++ needs recompilation.
    echo "==> [entrypoint] lidar_perception_node source is newer than binary — rebuilding lidar_cone_detector..."
    cd "$MFE_WS"
    colcon build --packages-select lidar_cone_detector --cmake-args -DBUILD_TESTING=OFF
fi

source "$MFE_WS/install/setup.bash"

exec "$@"
