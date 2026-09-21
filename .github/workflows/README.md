# CI/CD Workflows

## Summary

✅ **3 test suites** run automatically on every push/PR:
1. **Algorithm Tests** (75 tests, ~30s) — Pure Python, no dependencies
2. **RosBag Tests** (40 tests, ~10s) — Integration tests with recorded data
3. **Gazebo Tests** (11 tests, ~5-10m) — Simulation integration tests on Linux

## unit-tests.yml

Runs comprehensive unit tests across the entire MFE stack:

### 1. Algorithm Unit Tests (Ubuntu Latest)
- **What**: 75 pure Python algorithm tests
- **Where**: `ros2/src/mfe_state_estimation/test/test_ekf_filter.py` (8 tests)
  - `ros2/src/mfe_path_planning/test/test_speed_profile.py` (12 tests)
  - `ros2/src/mfe_path_planning/test/test_finish_detector.py` (14 tests)
  - `ros2/src/mfe_path_planning/test/test_cone_tracking.py` (12 tests)
  - `ros2/src/mfe_control/test/test_mpc_lateral.py` (13 tests)
  - `ros2/src/mfe_control/test/test_supervisor_obb.py` (16 tests)
- **Runtime**: ~30 seconds
- **Dependencies**: pytest, scipy, numpy (no ROS2 needed)

### 2. Gazebo Mission Tests (Docker + ROS2 Humble)
- **What**: 11 integration tests (accel, skidpad, peanut, autocross)
- **Where**: `ros2/src/mfe_tests/test_gazebo_missions.py`
- **Runtime**: ~5-10 minutes
- **Environment**:
  - Docker container: `mfe/a2:humble`
  - Headless Gazebo rendering (GAZEBO_HEADLESS=1)
  - EUFS sim (cloned from GitHub)
  - pytest 7.x (ROS2 Humble compatible)

### 3. RosBag Integration Tests (Ubuntu Latest)
- **What**: 40 LiDAR/EKF/path planning tests from rosbag data
- **Where**: `ros2/src/mfe_perception/test/test_rosbag_lidar.py` (15 tests)
  - `ros2/src/mfe_state_estimation/test/test_rosbag_ekf.py` (12 tests)
  - `ros2/src/mfe_path_planning/test/test_rosbag_path_planning.py` (13 tests)
- **Runtime**: ~10 seconds
- **Skip condition**: Skipped automatically if rosbag is empty (<1 KB)

## Running Tests Locally

### Algorithm Tests Only (Fast)
```bash
cd MFE-V1-unit-tests
pip install pytest scipy numpy
pytest ros2/src/mfe_state_estimation/test/test_ekf_filter.py \
        ros2/src/mfe_path_planning/test/test_speed_profile.py \
        ros2/src/mfe_path_planning/test/test_finish_detector.py \
        ros2/src/mfe_path_planning/test/test_cone_tracking.py \
        ros2/src/mfe_control/test/test_mpc_lateral.py \
        ros2/src/mfe_control/test/test_supervisor_obb.py \
        -v --timeout=300
```

### Gazebo Tests (Docker on macOS/Linux)
```bash
docker build --platform linux/amd64 \
  -t mfe-gazebo-test:latest \
  -f Docker/test/Dockerfile.gazebo-test .

docker run --rm --platform linux/amd64 \
  -v /path/to/MFE-Driverless-V1:/ws \
  -v /path/to/MFE-V1-unit-tests/ros2/src:/ws/ros2/src/test_suite \
  mfe-gazebo-test:latest \
  bash -c "
    cd /ws
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source install/setup.bash
    export PYTHONPATH=/ws/ros2/src:/ws/ros2/src/test_suite:\$PYTHONPATH
    export GAZEBO_HEADLESS=1
    export LIBGL_ALWAYS_SOFTWARE=1
    pytest ros2/src/test_suite/mfe_tests/test_gazebo_missions.py \
      -v --tb=short --timeout=300
  "
```

### RosBag Tests (Fast)
```bash
cd MFE-V1-unit-tests
pip install pytest rosbags scipy numpy
pytest ros2/src/mfe_perception/test/test_rosbag_lidar.py \
        ros2/src/mfe_state_estimation/test/test_rosbag_ekf.py \
        ros2/src/mfe_path_planning/test/test_rosbag_path_planning.py \
        -v --timeout=300
```

## Test Results Format

Each test produces:
- ✅ **PASSED**: Test ran successfully
- ❌ **FAILED**: Test failed (check stderr for assertion details)
- ⊘ **SKIPPED**: Test skipped due to missing dependencies or data

Example output:
```
================== test session starts ==================
platform linux -- Python 3.10.12, pytest-7.4.4
rootdir: /ws
plugins: ament-xmllint-0.12.15, launch-testing-1.0.14
collected 75 items

test_ekf_filter.py::TestCovariancePSD::test_identity_model_stays_psd PASSED
test_ekf_filter.py::TestConvergence::test_stationary_position_converges PASSED
test_speed_profile.py::TestMengerCurvature::test_straight_has_zero_curvature SKIPPED
test_gazebo_missions.py::TestAcceleration::test_accel_completes PASSED
================ 73 passed, 2 skipped in 45.2s ========
```

## Troubleshooting

### Gazebo tests skip in Docker
**Cause**: MFE packages (mfe_bringup, etc.) not built yet.
**Fix**: Ensure `colcon build` completes successfully before running tests.

### pytest plugin version mismatch
**Cause**: ROS2 Humble ships pytest 9.x, but launch-testing expects 7.x.
**Fix**: Use `pip install 'pytest<8'` to downgrade (already in Dockerfile).

### DISPLAY-related errors on macOS
**Cause**: Trying to run headless Gazebo with a display server.
**Fix**: Set `GAZEBO_HEADLESS=1` and unset `DISPLAY`.

## CI Triggers

Tests run automatically on:
- **Push** to `main` or any `neil-*` branch
- **Pull requests** targeting `main`

View results at: https://github.com/YOUR_ORG/MFE-Driverless-V1/actions
