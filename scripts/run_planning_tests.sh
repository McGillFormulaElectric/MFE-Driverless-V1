#!/bin/bash
# Unified local runner for the MFE path planning test suite.
#
# Automatically runs natively on Linux (when ROS 2 Humble is present) or inside
# the `mfe-driverless-sim` Docker container on macOS.
#
# Usage:
#   bash scripts/run_planning_tests.sh tier1 [--report-out PATH] [pytest args...]
#   bash scripts/run_planning_tests.sh lint
#   bash scripts/run_planning_tests.sh colcon
#   bash scripts/run_planning_tests.sh gen-centerlines [--plot-out PATH]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MFE_REPO="$(cd "$SCRIPT_DIR/.." && pwd)"
DEVELOP_DIR="$(cd "$MFE_REPO/.." && pwd)"
DOCKER_IMAGE="${MFE_DOCKER_IMAGE:-mfe-driverless-sim}"

usage() {
    cat <<EOF
Usage: bash scripts/run_planning_tests.sh <command> [args...]

Commands:
  tier1 [pytest args...]       Run Tier 1 unit tests + offline multi-track benchmarks
  lint                         Run ament_flake8 and ament_pep257 on mfe_path_planning
  colcon                       Run full 'colcon test --packages-select mfe_path_planning'
  gen-centerlines [args...]    Regenerate GT centerline fixtures from EUFS track CSVs
  -h, --help                   Show this help message
EOF
}

if [ $# -lt 1 ] || [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    usage
    exit 0
fi

CMD="$1"
shift

# Execute a shell snippet either directly (Linux/inside container) or via Docker (macOS).
run_in_env() {
    local inner_cmd="$1"
    if [ -f /opt/ros/humble/setup.bash ]; then
        bash -lc "
            set -eo pipefail
            export EUFS_MASTER=\"\${EUFS_MASTER:-$DEVELOP_DIR/MFE26-eufs-sim}\"
            source /opt/ros/humble/setup.bash
            [ -f \"$MFE_REPO/ros2/install/setup.bash\" ] && source \"$MFE_REPO/ros2/install/setup.bash\" || true
            export PYTHONPATH=\"$MFE_REPO/ros2/src/mfe_path_planning:$MFE_REPO/ros2/src/mfe_common:\${PYTHONPATH:-}\"
            $inner_cmd
        "
    else
        docker run --rm --platform linux/arm64 \
            --volume "$DEVELOP_DIR":/root/Develop \
            --env EUFS_MASTER=/root/Develop/MFE26-eufs-sim \
            --env PYTHONPATH=/root/Develop/MFE-Driverless-V1/ros2/src/mfe_path_planning:/root/Develop/MFE-Driverless-V1/ros2/src/mfe_common \
            "$DOCKER_IMAGE" \
            bash -lc "
                set -eo pipefail
                source /opt/ros/humble/setup.bash
                [ -f /root/Develop/MFE-Driverless-V1/ros2/install/setup.bash ] && source /root/Develop/MFE-Driverless-V1/ros2/install/setup.bash || true
                $inner_cmd
            "
    fi
}

# Resolve repo root path as seen inside the execution environment
if [ -f /opt/ros/humble/setup.bash ]; then
    ENV_REPO="$MFE_REPO"
else
    ENV_REPO="/root/Develop/MFE-Driverless-V1"
fi

PKG_DIR="$ENV_REPO/ros2/src/mfe_path_planning"

# Quote remaining CLI arguments safely for passing into bash -lc
EXTRA_ARGS=""
for arg in "$@"; do
    EXTRA_ARGS+=" $(printf '%q' "$arg")"
done

case "$CMD" in
    tier1)
        run_in_env "cd $PKG_DIR && python3 -m pytest -v test/test_finish_detector.py test/test_core_components.py test/test_offline_benchmark.py$EXTRA_ARGS"
        ;;
    lint)
        run_in_env "cd $PKG_DIR && ament_flake8 mfe_path_planning/eval test/conftest.py test/test_core_components.py test/test_offline_benchmark.py && ament_pep257 mfe_path_planning/eval test/conftest.py test/test_core_components.py test/test_offline_benchmark.py$EXTRA_ARGS"
        ;;
    colcon)
        run_in_env "cd $ENV_REPO/ros2 && colcon test --packages-select mfe_path_planning --event-handlers console_direct+ && colcon test-result --verbose --test-result-base build/mfe_path_planning"
        ;;
    gen-centerlines)
        run_in_env "python3 -m mfe_path_planning.eval.centerline --out-dir $PKG_DIR/test/fixtures/centerlines$EXTRA_ARGS"
        ;;
    *)
        echo "Unknown command: $CMD" >&2
        usage >&2
        exit 2
        ;;
esac
