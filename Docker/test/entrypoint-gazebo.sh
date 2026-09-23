#!/bin/bash
set -e

echo "=== MFE Gazebo Test Entrypoint ==="
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash

cd /ws

# Clone eufs_sim if not already present
if [ ! -d "ros2/src/eufs_sim" ]; then
    echo "=== Cloning eufs_sim from GitLab ==="
    mkdir -p ros2/src
    git clone https://gitlab.com/eufs/public/eufs_sim.git ros2/src/eufs_sim
    echo "✓ eufs_sim cloned"
else
    echo "✓ eufs_sim already present"
fi

# Clean and build workspace
echo ""
echo "=== Cleaning previous builds ==="
rm -rf build install log

echo ""
echo "=== Building MFE base packages first (eufs_msgs dependency) ==="
colcon build --packages-select eufs_msgs --symlink-install 2>&1 | grep -E "Building|Finished|ERROR|error" | head -20

echo ""
echo "=== Building full workspace (MFE + eufs_sim) ==="
colcon build --symlink-install 2>&1 | tail -40

# Source the built workspace
source /ws/install/setup.bash

# Verify eufs_sim is available
echo ""
echo "=== Verifying eufs_sim ==="
if ros2 pkg prefix eufs_sim >/dev/null 2>&1; then
    echo "✓ eufs_sim available in workspace"
else
    echo "✗ eufs_sim not found - tests will skip"
fi

# Run Gazebo tests
echo ""
echo "=== Running Gazebo Mission Tests ==="
export PYTHONPATH=/ws/ros2/src:/test_suite:$PYTHONPATH
export GAZEBO_HEADLESS=1
export LIBGL_ALWAYS_SOFTWARE=1
export DISPLAY=:99

python3 -m pytest /test_suite/mfe_tests/test_gazebo_missions.py \
    -v --tb=short --timeout=600

echo ""
echo "=== Test run complete ==="
