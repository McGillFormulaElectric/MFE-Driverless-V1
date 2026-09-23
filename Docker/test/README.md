# Gazebo Test Container for macOS

Runs the full MFE-Driverless-V1 Gazebo mission tests inside Docker on macOS via Rosetta (amd64 emulation).

## Build

```bash
cd /MFE-Driverless-V1/Docker/test
docker build --platform linux/amd64 -t mfe-gazebo-test:latest -f Dockerfile.gazebo-test .
```

Takes ~5 min (adds ROS2 Gazebo packages to mfe/a2:humble base).

## Run

```bash
# Mount the workspace and run tests
docker run --rm --platform linux/amd64 \
  -v /Users/neiljoegeorge/Develop/MFE-Driverless-V1:/ws \
  mfe-gazebo-test:latest \
  bash -c "
    cd /ws
    colcon build --symlink-install
    pytest ros2/src/mfe_tests/test_gazebo_missions.py -v --timeout=300
  "
```

Or interactive shell:

```bash
docker run -it --rm --platform linux/amd64 \
  -v /Users/neiljoegeorge/Develop/MFE-Driverless-V1:/ws \
  mfe-gazebo-test:latest \
  bash
```

Then inside:

```bash
cd /ws
source install/setup.bash
pytest ros2/src/mfe_tests/test_gazebo_missions.py::TestAcceleration -v
pytest ros2/src/mfe_tests/test_gazebo_missions.py::TestSkidpad -v
pytest ros2/src/mfe_tests/test_gazebo_missions.py::TestPeanut -v
```

## What's inside

- ROS2 Humble base (from `mfe/a2:humble`)
- Gazebo Fortress/11 (ros-humble-gazebo-* packages)
- Xvfb (headless X11 display)
- pytest, pytest-timeout, rosbags, scipy
- Entrypoint auto-starts Xvfb on DISPLAY=:99

## Performance

- Rosetta amd64→ARM64 emulation: ~10–15% slowdown
- Gazebo mission tests: ~120s per run (accel: 10s, skidpad: 70s, peanut: 40s)
- Total with colcon build: ~3–4 min per full test cycle

## Troubleshooting

**Xvfb fails**: Make sure 512MB RAM is available. On macOS, Docker Desktop has a 4GB default — should be plenty.

**Gazebo hangs on launch**: First launch takes ~20s (loading physics plugins). Tests have 180s timeout.

**DISPLAY not set**: The entrypoint.sh automatically starts Xvfb and sets DISPLAY=:99. If running a custom command, ensure you source bashrc: `bash -c "source /root/.bashrc && <cmd>"`.

## Example output

```
ros2/src/mfe_tests/test_gazebo_missions.py::TestAcceleration::test_accel_completes PASSED
  Mission: acceleration  success=True
  lap_time=8.23s  avg_speed=9.1m/s  max_speed=12.8m/s
  avg_dev=0.14m  max_dev=0.41m  cones_hit=0

ros2/src/mfe_tests/test_gazebo_missions.py::TestSkidpad::test_skidpad_completes_2l_2r PASSED
  Mission: skidpad  success=True
  lap_time=71.4s  avg_speed=4.2m/s  max_speed=4.6m/s
  avg_dev=0.21m  max_dev=0.63m  cones_hit=0
```
