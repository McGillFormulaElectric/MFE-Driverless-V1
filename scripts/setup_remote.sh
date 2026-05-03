#!/bin/bash
# =============================================================================
# MFE Driverless — Full Setup Script
# Clones all repos, installs all dependencies, and builds workspaces.
#
# Usage:
#   bash scripts/setup_remote.sh          # full setup
#   bash scripts/setup_remote.sh --skip-ros  # skip ROS install (inside Docker)
#
# Tested on:
#   Ubuntu 22.04 x86_64 (native + Docker/rocker)
#   Ubuntu 22.04 ARM64  (Jetson Orin Nano — run build_gazebo_from_source.sh first)
# =============================================================================

set -e

DEVELOP_DIR=~/Develop
EUFS_REPO=https://github.com/McGillFormulaElectric/MFE26-eufs-sim.git
MFE_REPO=https://github.com/McGillFormulaElectric/MFE-Driverless-V1.git
FSD_REPO=https://github.com/papalotis/ft-fsd-path-planning.git

EUFS_WS=$DEVELOP_DIR/MFE26-eufs-sim
MFE_WS=$DEVELOP_DIR/MFE-Driverless-V1/ros2
FSD_DIR=$DEVELOP_DIR/ft-fsd-path-planning

ARCH=$(dpkg --print-architecture)   # amd64 or arm64
SKIP_ROS=false
[[ "$1" == "--skip-ros" ]] && SKIP_ROS=true

echo "==> Detected architecture: $ARCH"
echo "==> Develop dir: $DEVELOP_DIR"
mkdir -p $DEVELOP_DIR

# =============================================================================
if [ "$SKIP_ROS" = false ]; then
echo "==> [1/10] Installing ROS 2 Humble..."
# =============================================================================
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$ARCH signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-humble-desktop-full
else
echo "==> [1/10] Skipping ROS 2 install (--skip-ros)"
fi

# =============================================================================
echo "==> [2/10] Installing ROS 2 build tools..."
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

# Initialise rosdep (ignore error if already done)
sudo rosdep init 2>/dev/null || true
rosdep update

# =============================================================================
echo "==> [3/10] Adding OSRF Gazebo repo..."
# =============================================================================
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$ARCH signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt update

# =============================================================================
echo "==> [4/10] Installing Gazebo Classic 11..."
# =============================================================================
if [ "$ARCH" = "amd64" ]; then
    sudo apt install -y \
        gazebo libgazebo-dev \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-dev \
        ros-humble-gazebo-plugins
    echo "Gazebo binaries installed via apt."
else
    echo "ARM64 detected — Gazebo must be built from source."
    echo "Run: bash scripts/build_gazebo_from_source.sh"
    echo "Then re-run this script with --skip-ros"
    # Install build deps for ARM
    sudo apt install -y \
        build-essential cmake pkg-config libtar-dev libqwt-qt5-dev \
        libboost-all-dev libprotobuf-dev protobuf-compiler libsdformat9-dev \
        libtbb-dev libogre-1.9-dev libfreeimage-dev libignition-math6-dev \
        libignition-transport8-dev libignition-msgs5-dev libignition-common3-dev \
        libignition-fuel-tools4-dev libtinyxml2-dev libtinyxml-dev \
        libgts-dev libgdal-dev libavformat-dev libavcodec-dev libavdevice-dev \
        libavutil-dev libswscale-dev libqt5widgets5 qtbase5-dev \
        libbullet-dev libfcl-dev libassimp-dev libusb-1.0-0-dev
fi

# =============================================================================
echo "==> [5/10] Installing perception + nav + control ROS dependencies..."
# =============================================================================
sudo apt install -y \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-ackermann-msgs \
    ros-humble-foxglove-bridge \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-slam-toolbox \
    libpcl-dev \
    libeigen3-dev

# =============================================================================
echo "==> [6/10] Installing Python dependencies..."
# =============================================================================
# Order matters — numpy must be pinned before scipy/sklearn/pandas to avoid
# binary incompatibility (system packages compiled for numpy 1.x)
pip3 install --upgrade pip
pip3 install "numpy<2"
pip3 install "scipy>=1.13"
pip3 install scikit-learn
pip3 install --upgrade --force-reinstall pandas
pip3 install icecream

# ft-fsd-path-planning: pip install from git fails due to broken package name
# Must clone and install directly
if [ ! -d "$FSD_DIR" ]; then
    echo "==> Cloning ft-fsd-path-planning..."
    git clone $FSD_REPO $FSD_DIR
fi
pip3 install $FSD_DIR
# Module installs as 'fsd_path_planning' (not ft_fsd_path_planning)
python3 -c "import fsd_path_planning; print('fsd_path_planning OK')"

# =============================================================================
echo "==> [7/10] Cloning repositories..."
# =============================================================================
mkdir -p $DEVELOP_DIR

if [ ! -d "$EUFS_WS" ]; then
    echo "==> Cloning EUFS sim..."
    git clone $EUFS_REPO $EUFS_WS
else
    echo "==> EUFS sim already present — pulling latest..."
    git -C $EUFS_WS pull
fi

if [ ! -d "$DEVELOP_DIR/MFE-Driverless-V1" ]; then
    echo "==> Cloning MFE Driverless stack..."
    git clone $MFE_REPO $DEVELOP_DIR/MFE-Driverless-V1
else
    echo "==> MFE stack already present — pulling latest..."
    git -C $DEVELOP_DIR/MFE-Driverless-V1 pull
fi

# =============================================================================
echo "==> [8/10] Building EUFS sim..."
# =============================================================================
source /opt/ros/humble/setup.bash

if [ "$ARCH" = "amd64" ]; then
    # x86: gazebo_ros_pkgs installed via apt, no need to build separately
    cd $EUFS_WS
    rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true
    colcon build --symlink-install \
        --cmake-args -DBUILD_TESTING=OFF
else
    # ARM64: source gazebo_ros_pkgs built from source
    GAZEBO_ROS_WS=$DEVELOP_DIR/gazebo_ros_pkgs
    if [ -d "$GAZEBO_ROS_WS/install" ]; then
        source $GAZEBO_ROS_WS/install/setup.bash
    else
        echo "WARNING: gazebo_ros_pkgs not built. Run build_gazebo_from_source.sh first."
    fi
    cd $EUFS_WS
    colcon build --symlink-install \
        --cmake-args -DBUILD_TESTING=OFF \
        --parallel-workers 2
fi

# =============================================================================
echo "==> [9/10] Building MFE Driverless stack..."
# =============================================================================
source $EUFS_WS/install/setup.bash
cd $MFE_WS
rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true
colcon build --symlink-install \
    --cmake-args -DBUILD_TESTING=OFF

# =============================================================================
echo "==> [10/10] Configuring shell..."
# =============================================================================
BASHRC=~/.bashrc
add_to_bashrc() {
    grep -qxF "$1" $BASHRC || echo "$1" >> $BASHRC
}

add_to_bashrc 'source /opt/ros/humble/setup.bash'
add_to_bashrc "export EUFS_MASTER=$EUFS_WS"

if [ "$ARCH" = "amd64" ]; then
    add_to_bashrc "source $EUFS_WS/install/setup.bash"
    add_to_bashrc "source $MFE_WS/install/setup.bash"
else
    GAZEBO_ROS_WS=$DEVELOP_DIR/gazebo_ros_pkgs
    add_to_bashrc "source $GAZEBO_ROS_WS/install/setup.bash"
    add_to_bashrc "source $EUFS_WS/install/setup.bash"
    add_to_bashrc "source $MFE_WS/install/setup.bash"
fi

# =============================================================================
echo ""
echo "============================================================"
echo " MFE Driverless setup complete!"
echo "============================================================"
echo ""
echo " Reload your shell:  source ~/.bashrc"
echo ""
echo " Launch simulation:"
echo "   TMUX= bash $DEVELOP_DIR/MFE-Driverless-V1/scripts/launch_sim.sh accel"
echo "   TMUX= bash $DEVELOP_DIR/MFE-Driverless-V1/scripts/launch_sim.sh skidpad"
echo ""
if [ "$ARCH" = "arm64" ]; then
echo " ARM64: If Gazebo not yet built, run first:"
echo "   bash $DEVELOP_DIR/MFE-Driverless-V1/scripts/build_gazebo_from_source.sh"
echo ""
fi
echo " Visualise with Foxglove Studio → ws://localhost:8765"
echo "============================================================"
