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

# =============================================================================
echo "==> [1/9] Installing ROS2 Humble..."
# =============================================================================
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$ARCH signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-humble-desktop-full

# =============================================================================
echo "==> [2/9] Installing ROS2 build tools..."
# =============================================================================
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

# =============================================================================
echo "==> [3/9] Adding OSRF repo (required for Gazebo + Ignition on ARM64)..."
# =============================================================================
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$ARCH signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt update

# =============================================================================
echo "==> [4/9] Installing Gazebo Classic build dependencies..."
# =============================================================================
# Required for building Gazebo 11 from source on ARM64
# (binary packages not available for ARM64 Ubuntu 22.04)
sudo apt install -y \
    build-essential cmake pkg-config \
    libtar-dev \
    libqwt-qt5-dev \
    libboost-all-dev \
    libprotobuf-dev protobuf-compiler \
    libsdformat9-dev \
    libtbb-dev \
    libogre-1.9-dev \
    libfreeimage-dev \
    libignition-math6-dev \
    libignition-transport8-dev \
    libignition-msgs5-dev \
    libignition-common3-dev \
    libignition-fuel-tools4-dev \
    libtinyxml2-dev libtinyxml-dev \
    libgts-dev libgdal-dev \
    libavformat-dev libavcodec-dev libavdevice-dev libavutil-dev \
    libswscale-dev \
    libqt5widgets5 qtbase5-dev \
    libbullet-dev libfcl-dev libassimp-dev \
    libusb-1.0-0-dev

# x86 only: try binary Gazebo packages first
if [ "$ARCH" = "amd64" ]; then
    sudo apt install -y gazebo libgazebo-dev \
        ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev ros-humble-gazebo-plugins \
        && echo "Gazebo binaries installed." \
        || echo "Warning: Gazebo binaries not found — build from source using scripts/build_gazebo_from_source.sh"
else
    echo "ARM64 detected — Gazebo must be built from source."
    echo "Run: bash scripts/build_gazebo_from_source.sh"
fi

# =============================================================================
echo "==> [5/9] Installing perception dependencies..."
# =============================================================================
sudo apt install -y \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-pointcloud-to-laserscan \
    libpcl-dev \
    libeigen3-dev

# =============================================================================
echo "==> [6/9] Installing navigation/control dependencies..."
# =============================================================================
sudo apt install -y \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-ackermann-msgs \
    ros-humble-foxglove-bridge \
    ros-humble-slam-toolbox

# =============================================================================
echo "==> [7/9] Installing Python dependencies..."
# =============================================================================
pip3 install numpy ultralytics torch torchvision
# ft-fsd-path-planning is not on PyPI — install from source
pip3 install git+https://github.com/papalotis/ft-fsd-path-planning.git

# =============================================================================
echo "==> [8/9] Sourcing ROS2 in ~/.bashrc..."
# =============================================================================
grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc \
    || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

# =============================================================================
echo "==> [9/9] Done."
# =============================================================================
echo ""
echo "Next steps:"
echo ""
if [ "$ARCH" = "arm64" ]; then
echo "  ARM64: Build Gazebo from source first:"
echo "    bash scripts/build_gazebo_from_source.sh"
echo ""
fi
echo "  # Build EUFS sim"
echo "    source /opt/ros/humble/setup.bash"
echo "    cd $EUFS_WS"
echo "    colcon build --symlink-install"
echo ""
echo "  # Build MFE driverless stack"
echo "    source $EUFS_WS/install/setup.bash"
echo "    cd $MFE_WS"
echo "    colcon build --symlink-install"
echo ""
echo "  # Add to ~/.bashrc:"
echo "    source $EUFS_WS/install/setup.bash"
echo "    source $MFE_WS/install/setup.bash"
