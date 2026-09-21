#!/usr/bin/env bash
# =============================================================================
# MFE Driverless — Unit Test Runner
#
# Two modes:
#   ./scripts/run_unit_tests.sh           # standalone (no ROS, no Docker)
#   ./scripts/run_unit_tests.sh --colcon  # inside Docker via colcon test
#
# Standalone mode runs all algorithm tests using plain pytest.
# ROS message types are mocked by conftest.py in each test directory.
#
# Usage examples:
#   ./scripts/run_unit_tests.sh                          # all tests
#   ./scripts/run_unit_tests.sh --filter ekf             # EKF tests only
#   ./scripts/run_unit_tests.sh --filter mpc             # MPC tests only
#   ./scripts/run_unit_tests.sh --filter speed_profile   # GGS/PSO tests
#   ./scripts/run_unit_tests.sh --filter finish          # finish detector
#   ./scripts/run_unit_tests.sh --filter cone_tracking   # cone tracking
#   ./scripts/run_unit_tests.sh --filter supervisor      # supervisor OBB
# =============================================================================

set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

FILTER=""
COLCON_MODE=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --colcon) COLCON_MODE=true ;;
        --filter) FILTER="$2"; shift ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
    shift
done

# ---------------------------------------------------------------------------
# Colcon mode (run inside Docker after `source install/setup.bash`)
# ---------------------------------------------------------------------------
if $COLCON_MODE; then
    echo "Running via colcon test..."
    cd "$REPO_ROOT/ros2"
    PKGS="mfe_state_estimation mfe_path_planning mfe_control"

    colcon test \
        --packages-select $PKGS \
        --pytest-args -v \
        --event-handlers console_cohesion+

    echo ""
    echo "Results:"
    colcon test-result --verbose
    exit 0
fi

# ---------------------------------------------------------------------------
# Standalone mode — plain pytest, no ROS needed
# ---------------------------------------------------------------------------

echo "============================================================"
echo " MFE Driverless — Standalone Unit Tests"
echo " Running from: $REPO_ROOT"
echo "============================================================"

# Build PYTHONPATH so packages can be imported without colcon install
SRC="$REPO_ROOT/ros2/src"
export PYTHONPATH="\
$SRC/mfe_state_estimation:\
$SRC/mfe_path_planning:\
$SRC/mfe_control:\
$SRC/mfe_msgs:\
${PYTHONPATH:-}"

# ---- Per-branch PYTHONPATH overrides ----
# When testing a specific worktree, PYTHONPATH controls which source is used.
# Tests use sys.path.append so PYTHONPATH entries take priority.
# Example:
#   PYTHONPATH=/path/to/MFE-V1-p2-ekf-fix/ros2/src/mfe_state_estimation \
#     ./scripts/run_unit_tests.sh --filter ekf
#
# Worktree → PYTHONPATH mapping (run from repo root):
#   EKF fix:        MFE-V1-p2-ekf-fix/ros2/src/mfe_state_estimation
#   Target speeds:  MFE-V1-p2-target-speeds/ros2/src/mfe_control
#   Supervisor OBB: MFE-V1-p2-supervisor-bbox/ros2/src/mfe_control
#   GGS / PSO:      MFE-V1-p3-ggs/ros2/src/mfe_path_planning
#   Cone tracking:  MFE-V1-p3-tracking/ros2/src/mfe_path_planning
#   MPC lateral:    MFE-V1-p3-mpc/ros2/src/mfe_control

# Collect test files
TESTS=(
    "$SRC/mfe_state_estimation/test/test_ekf_filter.py"
    "$SRC/mfe_path_planning/test/test_speed_profile.py"
    "$SRC/mfe_path_planning/test/test_finish_detector.py"
    "$SRC/mfe_path_planning/test/test_cone_tracking.py"
    "$SRC/mfe_control/test/test_mpc_lateral.py"
    "$SRC/mfe_control/test/test_supervisor_obb.py"
)

# Apply filter if given
if [[ -n "$FILTER" ]]; then
    TESTS=()
    for f in \
        "$SRC/mfe_state_estimation/test/test_ekf_filter.py" \
        "$SRC/mfe_path_planning/test/test_speed_profile.py" \
        "$SRC/mfe_path_planning/test/test_finish_detector.py" \
        "$SRC/mfe_path_planning/test/test_cone_tracking.py" \
        "$SRC/mfe_control/test/test_mpc_lateral.py" \
        "$SRC/mfe_control/test/test_supervisor_obb.py"
    do
        if [[ "$(basename $f)" == *"$FILTER"* ]]; then
            TESTS+=("$f")
        fi
    done
    if [[ ${#TESTS[@]} -eq 0 ]]; then
        echo "No test files matched filter: $FILTER"
        exit 1
    fi
fi

echo ""
echo "Test files:"
for t in "${TESTS[@]}"; do echo "  $(basename $t)"; done
echo ""

# Run pytest
python3 -m pytest \
    "${TESTS[@]}" \
    -v \
    --tb=short \
    --no-header \
    -p no:cacheprovider \
    "$@"   # pass any extra pytest args (e.g. -k "TestEKF")
