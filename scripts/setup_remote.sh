#!/bin/bash
# =============================================================================
# MFE Driverless — Remote Machine Setup Script
# Run once on a fresh Ubuntu 22.04 machine (x86_64 or ARM64/Jetson).
# Usage: bash scripts/setup_remote.sh
# =============================================================================

set -e

EUFS_WS=~/Develop/MFE26-eufs-sim
MFE_WS=~/Develop/MFE-Driverless-V1/ros2
ARCH=$(dpkg --print-architecture)   # amd64 or arm64

echo "==> Detected architecture: $ARCH"

echo "==> [1/7] Installing ROS2 Humble..."
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$ARCH signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-humble-desktop-full

echo "==> [2/7] Installing ROS2 build tools..."
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-auto \
    ros-humble-eigen3-cmake-module

echo "==> [3/7] Installing Gazebo..."
# Add OSRF repo for reliable ARM64 + x86 Gazebo Classic packages
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list'
wget -qO - https://packages.osrfoundation.org/gazebo.key | sudo apt-key add -
sudo apt update
sudo apt install -y gazebo libgazebo-dev

# ROS2 Gazebo bindings — try the standard package, fall back gracefully
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev ros-humble-gazebo-plugins \
    || echo "Warning: some Gazebo ROS packages not found — may need to build from source"

echo "==> [4/7] Installing perception dependencies..."
sudo apt install -y \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-pointcloud-to-laserscan \
    libpcl-dev \
    libeigen3-dev

echo "==> [5/7] Installing navigation/control dependencies..."
sudo apt install -y \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-ackermann-msgs \
    ros-humble-foxglove-bridge \
    ros-humble-slam-toolbox

echo "==> [6/7] Installing Python dependencies..."
pip3 install numpy ultralytics
# ft-fsd-path-planning is not on PyPI — install from source
pip3 install git+https://github.com/papalotis/ft-fsd-path-planning.git

echo "==> [7/7] Sourcing ROS2 in ~/.bashrc..."
grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc \
    || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

echo ""
echo "==> Setup complete! Now build the workspaces:"
echo ""
echo "    # EUFS sim"
echo "    source /opt/ros/humble/setup.bash"
echo "    cd $EUFS_WS"
echo "    rosdep install --from-paths src --ignore-src -r -y"
echo "    colcon build --symlink-install"
echo ""
echo "    # MFE driverless stack"
echo "    source $EUFS_WS/install/setup.bash"
echo "    cd $MFE_WS"
echo "    rosdep install --from-paths src --ignore-src -r -y"
echo "    colcon build --symlink-install"
echo ""
echo "    # Add to ~/.bashrc for persistent sourcing:"
echo "    source $EUFS_WS/install/setup.bash"
echo "    source $MFE_WS/install/setup.bash"
