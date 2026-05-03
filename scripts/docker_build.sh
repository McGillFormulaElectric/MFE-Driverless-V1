#!/bin/bash
# =============================================================================
# Build the MFE Driverless simulation Docker image.
# Usage: bash scripts/docker_build.sh
# =============================================================================

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

echo "==> Building mfe-driverless-sim image..."
docker build \
    -t mfe-driverless-sim \
    "$REPO_ROOT/Docker/fs-driverless-sim"

echo ""
echo "Build complete. Run the sim with:"
echo "  bash scripts/docker_run.sh accel"
