#!/usr/bin/env bash
set -e

EUFS_WS=/root/Develop/MFE26-eufs-sim
MFE_WS=/root/Develop/MFE-Driverless-V1/ros2
source /opt/ros/humble/setup.bash

for package in eufs_launcher eufs_plugins eufs_racecar eufs_sensors eufs_msgs; do
    if [ ! -f "$EUFS_WS/$package/package.xml" ]; then
        echo "Missing $EUFS_WS/$package/package.xml. Run scripts/setup_sim_workspaces.sh on the host." >&2
        echo "EUFS must use the Humble workspace layout; an incomplete folder or Galactic checkout will not work." >&2
        exit 1
    fi
done
if [ ! -f "$MFE_WS/src/fs_msgs/package.xml" ]; then
    echo "Missing fs_msgs. Run scripts/setup_sim_workspaces.sh on the host." >&2
    exit 1
fi

# Incremental builds refresh Python entry points and installed launch files too.
# Clear CMake caches so branch/layout changes cannot retain old source paths.
cd "$EUFS_WS"
colcon build --symlink-install --cmake-clean-cache --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
cd "$MFE_WS"
# EUFS owns this message underlay even if the legacy submodule is populated here.
message_exclusion=()
[ ! -f src/eufs_msgs/package.xml ] || message_exclusion=(--packages-ignore eufs_msgs)
colcon build --symlink-install "${message_exclusion[@]}" --cmake-clean-cache \
    --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash

# Hidden Gazebo windows still need rendering for camera / GPU-ray sensors.
if [ -z "${DISPLAY:-}" ]; then
    export DISPLAY=:99 MFE_VIRTUAL_DISPLAY=1 LIBGL_ALWAYS_SOFTWARE=true
    Xvfb "$DISPLAY" -screen 0 1280x720x24 -nolisten tcp >/tmp/mfe-xvfb.log 2>&1 &
    for attempt in $(seq 30); do
        xdpyinfo -display "$DISPLAY" >/dev/null 2>&1 && break
        sleep 0.1
    done
    if ! xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
        cat /tmp/mfe-xvfb.log >&2
        exit 1
    fi
fi
exec "$@"
