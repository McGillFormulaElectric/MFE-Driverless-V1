# Gazebo Integration Tests — MFE-Driverless-V1

Complete guide to running FSAE mission simulations in Gazebo and validating the full stack.

## Quick Start (5 minutes)

### Docker-based (Recommended)

```bash
# 1. Build the Gazebo test Docker image
cd /Users/neiljoegeorge/Develop/MFE-Driverless-V1
docker build -f Docker/test/Dockerfile.gazebo-test -t mfe/gazebo-test:humble .

# 2. Run the Gazebo tests inside Docker
docker run --rm -it \
  -e ROS_DOMAIN_ID=42 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e GAZEBO_HEADLESS=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /Users/neiljoegeorge/Develop/MFE-Driverless-V1/ros2_ws:/workspace \
  -w /workspace \
  mfe/gazebo-test:humble bash -c "
    cd ros2
    source install/setup.bash
    pytest src/mfe_tests/test_gazebo_missions.py -v --timeout=300
  "
```

---

## Full Setup (for local testing on Linux)

### Prerequisites

```bash
# Install ROS2 Humble
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y ros-humble-desktop

# Install Gazebo + EUFS sim
sudo apt install -y gazebo ros-humble-gazebo-ros-pkgs
git clone https://gitlab.com/eufs/public/eufs_sim.git ~/eufs_sim
cd ~/eufs_sim && git checkout humble
colcon build --symlink-install

# Install X display server (if headless)
sudo apt install -y xvfb mesa-utils
```

### Run Tests

```bash
# 1. Source ROS2 and EUFS
source /opt/ros/humble/setup.bash
source ~/eufs_sim/install/setup.bash
source ~/MFE-Driverless-V1/install/setup.bash

# 2. Start headless display
Xvfb :99 -screen 0 1024x768x24 &
export DISPLAY=:99

# 3. Run Gazebo tests
cd ~/MFE-Driverless-V1/ros2
pytest src/mfe_tests/test_gazebo_missions.py -v --timeout=300
```

---

## Test Structure

### Mission Tests (11 total)

**PR #45 (neil/feature/unit-tests)**

```
test_gazebo_missions.py
├── TestAccelerationMission      [Euler course]
├── TestSkidpadMission           [Cone circle challenge]
└── TestAutocrossMission         [Trackdrive mission]
```

Each test:
1. Launches full MFE stack (EKF, perception, planning, control)
2. Initializes EUFS Gazebo simulation
3. Publishes initial pose to `/initialpose`
4. Waits for `/planning/mission_finished` (max 300s timeout)
5. Validates lap metrics from CSV: lap time, cone hits, off-track violations
6. Asserts thresholds: `lap_time < 120s`, `cone_hits == 0`, `off_track == 0`

### Test Parameters

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `GAZEBO_HEADLESS` | 1 | No GUI window (Docker requirement) |
| `LIBGL_ALWAYS_SOFTWARE` | 1 | Software rendering (no GPU required) |
| `ROS_DOMAIN_ID` | 42 | DDS isolation (prevents interference) |
| `RMW_IMPLEMENTATION` | rmw_cyclonedds_cpp | CycloneDDS transport |
| Timeout | 300s | Max mission runtime before auto-fail |

---

## Running Specific Tests

### One Mission Only
```bash
pytest src/mfe_tests/test_gazebo_missions.py::TestAccelerationMission -v
```

### Two Missions
```bash
pytest src/mfe_tests/test_gazebo_missions.py::TestAccelerationMission \
        src/mfe_tests/test_gazebo_missions.py::TestSkidpadMission -v
```

### Dry-run (No Gazebo)
```bash
pytest src/mfe_tests/test_gazebo_missions.py --collect-only
```

### With Verbose Output
```bash
pytest src/mfe_tests/test_gazebo_missions.py -vv --tb=long
```

---

## Troubleshooting

### "Gazebo not available"
```bash
# Check ROS2
which ros2

# Check EUFS sim
ros2 pkg prefix eufs_sim

# Check display
echo $DISPLAY
# If empty:
Xvfb :99 -screen 0 1024x768x24 &
export DISPLAY=:99
```

### Timeout (>300s)
- Reduce complexity: run `TestAccelerationMission` only (faster)
- Increase timeout: `pytest ... --timeout=600`
- Check CPU: `top` during test (may need more compute)

### Docker Build Fails
```bash
# Force rebuild with no cache
docker build --no-cache -f Docker/test/Dockerfile.gazebo-test -t mfe/gazebo-test:humble .
```

### Memory Issues
- Add `--memory=4g` to docker run
- Reduce resolution: `-e GAZEBO_RENDERING_RESOLUTION=640x480`

---

## CI/CD Integration

Tests are auto-run on every PR via GitHub Actions:

```yaml
.github/workflows/unit-tests.yml
  - gazebo-tests job
    - Runs on: ubuntu-latest (headless)
    - Trigger: Changes to core packages (EKF, planning, control)
    - Pass/fail affects PR merge blocking
```

Check results:
```bash
gh run view <RUN_ID> --log
```

---

## Output Files

After each test run:
```
~/mfe_logs/
├── gazebo_<mission>_<timestamp>.log
├── lap_validator_<mission>.csv              [Metrics: lap time, cone hits, etc.]
├── rosbag/<mission>/                        [ROS2 bag for post-analysis]
└── screenshots/<mission>/                   [Frame captures from Gazebo]
```

---

## Next Steps

1. **Unit tests** (fast): `pytest ros2/ -k "not gazebo"` [~30s]
2. **Gazebo tests** (slow): `pytest ros2/ -k "gazebo"` [~5m for all 3 missions]
3. **Full CI/CD**: Push to `origin` and check GitHub Actions

---

## References

- **EUFS Sim**: https://gitlab.com/eufs/public/eufs_sim
- **ROS2 Humble**: https://docs.ros.org/en/humble/
- **Gazebo**: https://gazebosim.org/
- **MFE Tests**: `ros2/src/mfe_tests/test_gazebo_missions.py`
