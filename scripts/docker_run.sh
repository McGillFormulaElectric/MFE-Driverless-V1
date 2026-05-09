#!/bin/bash
# =============================================================================
# Run the MFE Driverless sim inside Docker with GPU + display passthrough.
#
# Usage: bash scripts/docker_run.sh [accel|skidpad] [perception|no_perception] [gui|nogui]
#
# Prerequisites on host:
#   - NVIDIA drivers installed (everything else is auto-installed if missing)
# =============================================================================

# Install Docker if missing
if ! command -v docker &>/dev/null; then
  echo "==> Installing Docker..."
  curl -fsSL https://get.docker.com | sudo sh
  sudo usermod -aG docker "$USER"
  echo "Docker installed. NOTE: log out and back in for group membership to take effect."
  echo "Then re-run this script."
  exit 0
fi

# Ensure current user can reach the Docker socket
if ! docker info &>/dev/null 2>&1; then
  if id -nG "$USER" | grep -qw docker; then
    echo "ERROR: You are in the docker group but this session hasn't picked it up yet."
    echo "Please open a new terminal and re-run this script."
  else
    echo "==> Adding $USER to the docker group..."
    sudo usermod -aG docker "$USER"
    echo "Done. Open a new terminal and re-run this script."
  fi
  exit 1
fi

# Install nvidia-container-toolkit if missing (needed for --gpus all)
if ! dpkg -s nvidia-container-toolkit &>/dev/null; then
  echo "==> Installing nvidia-container-toolkit..."
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey |
    sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
  curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list |
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' |
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
  sudo apt update && sudo apt install -y nvidia-container-toolkit
  sudo systemctl restart docker
  echo "nvidia-container-toolkit installed and Docker restarted"
fi

EVENT=${1:-accel}
MODE=${2:-no_perception}
GUI=${3:-gui}

DEVELOP_DIR="$HOME/Develop"
MFE_DIR="$DEVELOP_DIR/MFE-Driverless-V1"

# Clone repos on host if not present (container needs them as bind mounts)
if [ ! -d "$DEVELOP_DIR/MFE26-eufs-sim" ]; then
  echo "==> Cloning EUFS sim..."
  git clone git@github.com:McGillFormulaElectric/MFE26-eufs-sim.git \
    "$DEVELOP_DIR/MFE26-eufs-sim"
fi

# eufs_msgs is a standalone repo required by the EUFS sim workspace
if [ ! -f "$DEVELOP_DIR/MFE26-eufs-sim/eufs_msgs/package.xml" ]; then
  echo "==> Cloning eufs_msgs into EUFS sim workspace..."
  git clone https://gitlab.com/eufs/eufs_msgs.git \
    "$DEVELOP_DIR/MFE26-eufs-sim/eufs_msgs"
fi

if [ ! -d "$MFE_DIR" ]; then
  echo "==> Cloning MFE Driverless..."
  git clone https://github.com/McGillFormulaElectric/MFE-Driverless-V1.git \
    "$MFE_DIR"
fi

# Allow Docker to connect to X server for Gazebo GUI
xhost +local:docker 2>/dev/null || true

# GPU flags — requires nvidia-container-toolkit
GPU_FLAGS="--gpus all --env NVIDIA_DRIVER_CAPABILITIES=all"

echo "==> Starting mfe-driverless-sim container..."
echo "    event=$EVENT  mode=$MODE  gui=$GUI"

# Only allocate a TTY if stdin is a real terminal
TTY_FLAGS=""
[ -t 0 ] && TTY_FLAGS="-it"

docker run --rm $TTY_FLAGS \
  $GPU_FLAGS \
  --env DISPLAY="$DISPLAY" \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume "$DEVELOP_DIR":/root/Develop \
  --network host \
  --ipc host \
  --name mfe-sim \
  mfe-driverless-sim \
  bash /root/Develop/MFE-Driverless-V1/scripts/launch_sim.sh "$EVENT" "$MODE" "$GUI"
