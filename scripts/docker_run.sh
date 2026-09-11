#!/usr/bin/env bash
# Usage: bash scripts/docker_run.sh [track] [perception|no_perception] [gui|nogui] [laps]
# MFE_GPU=auto (default), nvidia, or software. Host provisioning is explicit.
set -euo pipefail
MFE_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
DEVELOP_DIR=$(dirname "$MFE_DIR")
EVENT=${1:-accel}
MODE=${2:-no_perception}
GUI=${3:-gui}
LAPS=${4:-1}
case "$GUI" in gui|nogui) ;; *) echo "Use gui or nogui." >&2; exit 1;; esac
docker info >/dev/null || { echo "Docker must be installed, running, and accessible." >&2; exit 1; }
bash "$MFE_DIR/scripts/setup_sim_workspaces.sh"

args=(--rm --init --ipc host --publish 8765:8765 --name mfe-sim
      --mount "type=bind,source=$DEVELOP_DIR,target=/root/Develop")
[ ! -t 0 ] || args+=(-it)
gpu=${MFE_GPU:-auto}
case "$gpu" in auto|nvidia|software) ;; *) echo "MFE_GPU must be auto, nvidia, or software." >&2; exit 1;; esac
if [ "$gpu" = auto ]; then
    gpu=software
    if command -v nvidia-smi >/dev/null && nvidia-smi >/dev/null 2>&1 &&
       docker info --format '{{json .Runtimes}}' | grep -q '"nvidia"'; then
        gpu=nvidia
    fi
fi
if [ "$gpu" = nvidia ]; then
    if ! docker run --rm --runtime=nvidia --gpus all --entrypoint nvidia-smi \
        mfe-driverless-sim >/dev/null; then
        echo "NVIDIA container access failed. Configure NVIDIA Container Toolkit, or use MFE_GPU=software." >&2
        exit 1
    fi
    args+=(--runtime=nvidia --gpus all --env NVIDIA_DRIVER_CAPABILITIES=all)
else
    args+=(--env LIBGL_ALWAYS_SOFTWARE=true)
    echo "Using software rendering (slower); set MFE_GPU=nvidia for configured GPU access."
fi

if [ "$GUI" = gui ]; then
    if [ -z "${DISPLAY:-}" ] || [ ! -d /tmp/.X11-unix ]; then
        echo "GUI mode requires a host desktop DISPLAY and /tmp/.X11-unix. Use nogui otherwise." >&2
        exit 1
    fi
    # Grant only the container's root user access, and undo a grant we added.
    if ! xhost | grep -q 'SI:localuser:root'; then
        xhost +si:localuser:root
        trap 'xhost -si:localuser:root >/dev/null 2>&1 || true' EXIT
    fi
    args+=(--env "DISPLAY=$DISPLAY" --env QT_X11_NO_MITSHM=1
           --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix)
fi
docker run "${args[@]}" mfe-driverless-sim \
    bash /root/Develop/MFE-Driverless-V1/scripts/launch_sim.sh "$EVENT" "$MODE" "$GUI" "$LAPS"
