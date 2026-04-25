#!/bin/bash
# =============================================================================
# Build Gazebo Classic 11 + ROS2 bindings from source (ARM64 Ubuntu 22.04)
# Required because Gazebo Classic binary packages are not available for ARM64.
# Usage: bash scripts/build_gazebo_from_source.sh
# Estimated time: 1-2 hours
# =============================================================================

set -e

GAZEBO_WS=~/Develop/gazebo11
GAZEBO_ROS_WS=~/Develop/gazebo_ros_pkgs
EUFS_WS=~/Develop/MFE26-eufs-sim

echo "==> [1/5] Building sdformat9 from source..."
cd ~/Develop
if [ ! -d sdformat9 ]; then
    git clone https://github.com/gazebosim/sdformat.git -b sdf9 sdformat9
fi
cd sdformat9
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig
echo "sdformat9 installed."

echo "==> [2/5] Building Gazebo Classic 11 from source..."
cd ~/Develop
if [ ! -d gazebo11 ]; then
    git clone https://github.com/gazebosim/gazebo-classic.git -b gazebo11 gazebo11
fi
cd gazebo11
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig
echo "Gazebo 11 installed."

echo "==> [3/5] Building gazebo_ros_pkgs from source..."
cd ~/Develop
if [ ! -d gazebo_ros_pkgs ]; then
    git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b ros2 gazebo_ros_pkgs
fi
cd gazebo_ros_pkgs
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
echo "gazebo_ros_pkgs installed."

echo "==> [4/5] Building EUFS sim..."
cd $EUFS_WS
source /opt/ros/humble/setup.bash
source $GAZEBO_ROS_WS/install/setup.bash
colcon build --symlink-install
echo "EUFS sim built."

echo "==> [5/5] Done!"
echo ""
echo "Add to ~/.bashrc:"
echo "  source $GAZEBO_ROS_WS/install/setup.bash"
echo "  source $EUFS_WS/install/setup.bash"
echo ""
echo "To launch the sim:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source $GAZEBO_ROS_WS/install/setup.bash"
echo "  source $EUFS_WS/install/setup.bash"
echo "  ros2 launch eufs_launcher eufs_launcher.launch.py commandMode:=velocity"
