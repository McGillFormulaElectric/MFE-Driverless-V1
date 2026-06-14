#!/bin/bash
# =============================================================================
# MFE Driverless — Jetson compute node setup
#
# Installs ROS2 Humble + CycloneDDS, clones the stack, builds only the
# compute packages (no Gazebo, no sim, no heavy perception deps).
# Run this ONCE on each Jetson. Safe to re-run — skips steps already done.
#
# Usage (run on the Jetson directly, or pipe via SSH from host):
#   bash scripts/setup_jetson.sh
#
# Or from your host PC in one shot:
#   ssh mfe@100.119.35.6 "bash -s" < scripts/setup_jetson.sh
# =============================================================================

set -e

MFE_REPO=https://github.com/McGillFormulaElectric/MFE-Driverless-V1.git
DEVELOP_DIR=~/Develop
MFE_DIR=$DEVELOP_DIR/MFE-Driverless-V1
MFE_WS=$MFE_DIR/ros2

ARCH=$(dpkg --print-architecture)   # arm64 on Jetson
echo "==> Architecture: $ARCH"
echo "==> Target dir:   $MFE_DIR"

# =============================================================================
echo ""
echo "── [1/6] ROS 2 Humble ──────────────────────────────────────────────────"
# =============================================================================
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "Installing ROS 2 Humble..."
    sudo apt update && sudo apt install -y curl gnupg lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$ARCH signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list
    sudo apt update
    sudo apt install -y ros-humble-ros-base   # base only — no GUI, no sim
else
    echo "ROS 2 Humble already installed — skipping."
fi

# =============================================================================
echo ""
echo "── [2/6] ROS dependencies ──────────────────────────────────────────────"
# =============================================================================
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-ackermann-msgs \
    ros-humble-foxglove-bridge

sudo rosdep init 2>/dev/null || true
rosdep update

# =============================================================================
echo ""
echo "── [3/6] Python dependencies ───────────────────────────────────────────"
# =============================================================================
pip3 install --upgrade pip
pip3 install "numpy<2" scipy scikit-learn icecream

# ft-fsd-path-planning (used by path_planner_node)
FSD_DIR=$DEVELOP_DIR/ft-fsd-path-planning
if [ ! -d "$FSD_DIR" ]; then
    git clone https://github.com/papalotis/ft-fsd-path-planning.git $FSD_DIR
fi
pip3 install $FSD_DIR
python3 -c "import fsd_path_planning; print('  fsd_path_planning OK')"

# =============================================================================
echo ""
echo "── [4/6] Clone / update repo ───────────────────────────────────────────"
# =============================================================================
mkdir -p $DEVELOP_DIR
if [ ! -d "$MFE_DIR" ]; then
    echo "Cloning MFE-Driverless-V1..."
    git clone $MFE_REPO $MFE_DIR
else
    echo "Repo already present — pulling latest..."
    git -C $MFE_DIR pull
fi

# =============================================================================
echo ""
echo "── [5/6] Build compute packages ────────────────────────────────────────"
# =============================================================================
# Skip: mfe_eufs_sim (Gazebo), lidar_cone_detector, vision_cone_detector,
#        perception_evaluator, mfe_sensors, mfe_mapping, mfe_state_estimation
# Build: mfe_msgs, mfe_bringup, mfe_path_planning, mfe_control
source /opt/ros/humble/setup.bash
cd $MFE_WS

rosdep install --from-paths src --ignore-src -r -y \
    --skip-keys "gazebo_ros gazebo_plugins mfe_eufs_sim lidar_cone_detector \
                 vision_cone_detector perception_evaluator mfe_sensors \
                 mfe_mapping mfe_state_estimation" 2>/dev/null || true

colcon build --symlink-install \
    --cmake-args -DBUILD_TESTING=OFF \
    --packages-select \
        mfe_msgs \
        mfe_bringup \
        mfe_path_planning \
        mfe_control

# =============================================================================
echo ""
echo "── [6/6] Shell environment ─────────────────────────────────────────────"
# =============================================================================
BASHRC=~/.bashrc
add_line() { grep -qxF "$1" $BASHRC || echo "$1" >> $BASHRC; }

add_line 'source /opt/ros/humble/setup.bash'
add_line "source $MFE_WS/install/setup.bash"
add_line 'export ROS_DOMAIN_ID=42'
add_line 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp'
add_line 'export CYCLONEDDS_URI=file://$HOME/cyclone_tailscale.xml'

# Copy CycloneDDS Tailscale config if not already present
CYCLONE_DST=~/cyclone_tailscale.xml
CYCLONE_SRC=$MFE_DIR/ros2/src/mfe_bringup/config/cyclone_tailscale.xml
if [ ! -f "$CYCLONE_DST" ]; then
    cp $CYCLONE_SRC $CYCLONE_DST
    echo "Copied cyclone_tailscale.xml to ~/"
else
    echo "~/cyclone_tailscale.xml already exists — not overwriting."
fi

echo ""
echo "============================================================"
echo " Jetson setup complete!"
echo "============================================================"
echo ""
echo " Source the new env:  source ~/.bashrc"
echo ""
echo " Test ROS2 is visible from host:"
echo "   ros2 topic list    # should show topics from sim"
echo ""
echo " Or let the host launch it via fleet:"
echo "   ros2 launch mfe_bringup fleet.launch.py \\"
echo "     pose_topic:=/ground_truth/state_odom \\"
echo "     use_slam:=false use_ekf:=false"
echo "============================================================"
