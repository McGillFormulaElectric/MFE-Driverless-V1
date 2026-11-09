#!/bin/bash
# =============================================================================
# Run the MFE Driverless sim inside Docker on macOS (Apple Silicon / Intel).
#
# Usage: bash scripts/docker_run_mac.sh [track] [perception|no_perception] [gui|nogui] [laps]
#
# Prerequisites:
#   - Docker Desktop for Mac installed and running
#   - For gui mode: XQuartz installed (brew install --cask xquartz)
#     Then: open XQuartz → Preferences → Security → tick "Allow connections from network clients"
#     Log out and back in after installing XQuartz.
# =============================================================================

EVENT=${1:-accel}
MODE=${2:-no_perception}
GUI=${3:-nogui}   # default nogui on Mac — Gazebo without GPU is slow
LAPS=${4:-1}

DEVELOP_DIR="$HOME/Develop"
MFE_DIR="$DEVELOP_DIR/MFE-Driverless-V1"

# Clone repos if not present
if [ ! -d "$DEVELOP_DIR/MFE26-eufs-sim" ]; then
  echo "==> Cloning EUFS sim..."
  git clone git@github.com:McGillFormulaElectric/MFE26-eufs-sim.git \
    "$DEVELOP_DIR/MFE26-eufs-sim"
fi

if [ ! -f "$DEVELOP_DIR/MFE26-eufs-sim/eufs_msgs/package.xml" ]; then
  echo "==> Cloning eufs_msgs..."
  git clone https://gitlab.com/eufs/eufs_msgs.git \
    "$DEVELOP_DIR/MFE26-eufs-sim/eufs_msgs"
fi

if [ ! -d "$MFE_DIR" ]; then
  echo "==> Cloning MFE Driverless..."
  git clone https://github.com/McGillFormulaElectric/MFE-Driverless-V1.git \
    "$MFE_DIR"
fi

# Display setup — only needed for gui mode
DISPLAY_FLAGS=""
if [ "$GUI" = "gui" ]; then
  if ! command -v xquartz &>/dev/null && [ ! -d /Applications/Utilities/XQuartz.app ]; then
    echo "ERROR: XQuartz not found. Install with: brew install --cask xquartz"
    echo "Then log out and back in, enable 'Allow connections from network clients' in XQuartz prefs."
    exit 1
  fi
  # Start XQuartz if not running
  open -a XQuartz 2>/dev/null || true
  sleep 1
  # Allow Docker (localhost) to connect to the X server
  xhost +localhost 2>/dev/null || true
  DISPLAY_FLAGS="--env DISPLAY=host.docker.internal:0"
  echo "==> GUI mode: using XQuartz display"
else
  echo "==> Headless mode (nogui) — no display needed"
fi

# Build image for arm64 if not already built
if ! docker image inspect mfe-driverless-sim &>/dev/null; then
  echo "==> Building Docker image for arm64 (this takes ~10 min on first run)..."
  docker build \
    --platform linux/arm64 \
    -t mfe-driverless-sim \
    -f "$MFE_DIR/Docker/mac/Dockerfile" \
    "$MFE_DIR/Docker/mac"
fi

echo "==> Starting mfe-driverless-sim container..."
echo "    event=$EVENT  mode=$MODE  gui=$GUI  laps=$LAPS"

TTY_FLAGS=""
[ -t 0 ] && TTY_FLAGS="-it"

docker run --rm $TTY_FLAGS \
  --platform linux/arm64 \
  $DISPLAY_FLAGS \
  --volume "$DEVELOP_DIR":/root/Develop \
  --publish 8765:8765 \
  --ipc host \
  --name mfe-sim \
  mfe-driverless-sim \
  bash /root/Develop/MFE-Driverless-V1/scripts/launch_sim.sh "$EVENT" "$MODE" "$GUI" "$LAPS"
