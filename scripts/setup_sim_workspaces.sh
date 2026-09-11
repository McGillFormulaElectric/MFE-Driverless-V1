#!/usr/bin/env bash
# Fetch simulation sources explicitly; never change an existing checkout's branch.
set -euo pipefail
MFE_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
DEVELOP_DIR=$(dirname "$MFE_DIR")
EUFS_WS="$DEVELOP_DIR/MFE26-eufs-sim"

if [ ! -e "$EUFS_WS" ]; then
    git clone --branch humble git@github.com:McGillFormulaElectric/MFE26-eufs-sim.git "$EUFS_WS"
fi
if [ ! -f "$EUFS_WS/eufs_launcher/package.xml" ]; then
    echo "Expected a complete Humble-layout simulator at $EUFS_WS." >&2
    echo "Preserve any existing work, then select a branch based on humble or clone it there." >&2
    exit 1
fi
if [ ! -e "$EUFS_WS/eufs_msgs" ]; then
    git clone https://gitlab.com/eufs/eufs_msgs.git "$EUFS_WS/eufs_msgs"
fi
test -f "$EUFS_WS/eufs_msgs/package.xml" || {
    echo "The existing eufs_msgs directory is incomplete: $EUFS_WS/eufs_msgs" >&2
    exit 1
}
git -C "$MFE_DIR" submodule update --init --recursive -- ros2/src/fs_msgs
echo "Simulation dependencies ready. Existing branches were preserved."
