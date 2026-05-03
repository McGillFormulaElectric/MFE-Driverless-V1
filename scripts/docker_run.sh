#!/bin/bash
# =============================================================================
# Run the MFE Driverless sim inside Docker with GPU + display passthrough.
#
# Usage: bash scripts/docker_run.sh [accel|skidpad] [perception|no_perception] [gui|nogui]
#
# Prerequisites on host:
#   - Docker installed
#   - NVIDIA drivers installed (nvidia-container-toolkit is auto-installed if missing)
# =============================================================================

# Install nvidia-container-toolkit if missing (needed for --gpus all)
if ! dpkg -s nvidia-container-toolkit &>/dev/null; then
    echo "==> Installing nvidia-container-toolkit..."
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
        | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
    curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
        | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
        | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
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
    git clone https://github.com/McGillFormulaElectric/MFE26-eufs-sim.git \
        "$DEVELOP_DIR/MFE26-eufs-sim"
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

docker run --rm -it \
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
