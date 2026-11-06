# MFE-Driverless-V1

Software for the McGill Formula Electric driverless car. The stack is ROS 2 Humble and runs on
the car's Jetson computers, on a desktop PC for development, and in the EUFS simulator.

Detailed guides are in [docs/](docs/index.md): architecture, nodes, simulation, Docker, Jetson setup.

## Packages

Packages live under `ros2/src/`.

| Package | Purpose |
|---|---|
| `mfe_bringup` | Launch files that start the perception, state estimation, planning and control stack. |
| `mfe_control` | Pure pursuit and MPC lateral controllers, and the supervisor node. |
| `mfe_eufs_sim` | Bridge between EUFS simulator topics and the MFE topics. |
| `mfe_mapping` | SLAM Toolbox launch and configuration for 2D mapping. |
| `mfe_msgs` | Message and service definitions shared by the MFE nodes. |
| `mfe_path_planning` | Boundary tracer, path planner, finish detector and lap validator. |
| `mfe_perception/lidar_cone_detector` | LiDAR preprocessing, ground removal and cone clustering (C++, optional CUDA filters). |
| `mfe_perception/vision_cone_detector` | YOLO cone detection from camera images (Python; needs torch and ultralytics). |
| `mfe_perception/perception_evaluator` | Compares perception output against ground-truth cones. |
| `mfe_sensors` | Sensor interfaces for the LiDAR, camera and IMU hardware, and a simulated LiDAR node. |
| `mfe_simulation` | Connections to external simulators (IPG CarMaker and FSDS). |
| `mfe_state_estimation` | Extended Kalman filter that fuses IMU, GPS and wheel-speed data. |
| `mfe_validation` | Live validation stack: GTSAM cone-landmark SLAM and a second path planner fed only by LiDAR perception. |

Git submodules provide the message packages `fs_msgs` (`ros2/src/fs_msgs`) and `eufs_msgs`
(`ros2/src/eufs_msgs`). `lib/gtsam` is a further submodule
(see `docs/jetson_setup.md` for how it is used on the Jetsons).

## Requirements

- Ubuntu 22.04 with ROS 2 Humble, or the Docker images in `Docker/`.
- Python 3.10.
- A CUDA toolkit is optional. Without one, `lidar_cone_detector` builds a CPU-only version.
  `vision_cone_detector` needs torch and ultralytics.

## Build

```bash
git clone --recurse-submodules https://github.com/McGillFormulaElectric/MFE-Driverless-V1.git
cd MFE-Driverless-V1

# Python dependencies (pinned in requirements.txt)
pip3 install -r requirements.txt

# ROS dependencies
cd ros2
rosdep install --from-paths src --ignore-src -y --rosdistro humble

colcon build
source install/setup.bash
```

If you cloned without `--recurse-submodules`, run `git submodule update --init --recursive`.

`build_and_launch.sh` at the repository root wraps the build and launch steps for the car.

## Tests

```bash
cd ros2
colcon test
colcon test-result --verbose
```

`colcon` does not run the pytest suites of the Python packages, so run those directly:

```bash
for pkg in mfe_control mfe_path_planning mfe_state_estimation; do
  (cd src/$pkg && python3 -m pytest -q test)
done
```

Some lint tests currently fail.

## Quick start

Simulation (desktop, sim container):

```bash
scripts/run_sim.sh --mission autocross --laps 1 --sensors lidar,camera,imu_gps,can
scripts/run_sim.sh --help
```

Car (run on Jetson 1, mfe-driverless-1, with the wired car link up; the stack is split across both Jetsons):

```bash
scripts/run_car.sh --mission autocross --laps 1 --sensors lidar,camera,imu_gps,can
scripts/run_car.sh --dry-run --single-host   # print the preflight and the commands only
scripts/run_car.sh --single-host --replay ~/mfe_replay_bags/<bag>   # one Jetson, bag instead of drivers
scripts/run_car.sh --help
```

`platform` (`sim` or `car`) and `sensors` (a comma list of `lidar,camera,imu_gps,can`) are the two inputs that describe the setup. See [docs/interfaces.md](docs/interfaces.md#platform-and-sensors) for what each sensor switches on and off, and for the two-Jetson split and the car network.

## Launch

The main entry point is `mfe_bringup`'s `bringup.launch.py`:

```bash
source ros2/install/setup.bash
ros2 launch mfe_bringup bringup.launch.py mission:=autocross
```

Main arguments (defaults in brackets):

| Argument | Purpose |
|---|---|
| `mission` | `autocross`, `trackdrive`, `acceleration`, `skidpad` or `peanut` [`autocross`] |
| `planner_params_file` | Absolute path to a YAML file of planner parameters [empty] |
| `unknown_cone_mode` | How to plan without cone colours: `trace`, `trace_backend`, `fasttube` or `heading` [`trace`] |
| `skidpad_mode` | `cone_based` or `hardcoded` (skidpad only) [`cone_based`] |
| `vision_model_path` | YOLO weights file; empty skips the vision detector [empty] |
| `pose_topic` | Odometry topic used for vehicle pose [`/ekf/output`] |
| `platform` | `sim` or `car`; `legacy` (the default) keeps the old behaviour for existing commands [`legacy`] |
| `sensors` | Comma list of `lidar,camera,imu_gps,can`; absent sensors are not started [all] |
| `part` | Car split: `all`, `sensing` (Jetson 1) or `estimation` (Jetson 2) [`all`] |
| `use_perception` / `run_perception` | Launch the perception nodes [`true`] |
| `run_compute` | Launch planning and control nodes [`true`] |
| `use_slam` / `use_ekf` | Launch SLAM Toolbox and the EKF [`true`] |
| `num_laps` | Laps before the finish gate stops the car (empty = mission default) |
| `endless` | Disable finish detection and loop indefinitely [`false`] |
| `max_speed` / `lookahead` | Override pure pursuit values; 0 keeps the mission default |
| `record_bag` | Record an MCAP bag to `bag_output_dir` [`false`] |

Run the same launch file with `--show-args` to see every argument.

The car runs on two computers. `scripts/run_car.sh` starts both halves (see above), and `car.launch.py` starts one half. Other multi-machine setups use `perception.launch.py` on the perception computer and
`compute.launch.py` on the compute computer. `fleet.launch.py` starts perception locally and
starts the Jetsons over SSH. See [docs/fleet.md](docs/fleet.md) and
[docs/jetson_setup.md](docs/jetson_setup.md).

To run the EUFS simulator, use `scripts/launch_sim.sh`. It needs the EUFS workspace; see
[docs/simulation_guide.md](docs/simulation_guide.md).

## Configuration

| Location | Contents |
|---|---|
| `ros2/src/mfe_bringup/config/` | `vehicle.yaml`, `cyclone_tailscale.xml` (DDS config for multi-machine setups) |
| `ros2/src/mfe_state_estimation/config/` | EKF parameters (`ekf_params.yaml`) |
| `ros2/src/mfe_mapping/config/` | `slam_toolbox_params.yaml` |
| `ros2/src/mfe_sensors/config/` | Sensor configs: `xsens_mti.yaml` |

## Lint hooks

Lint checks run on staged files through pre-commit (ruff for Python, clang-format for C++ and
CUDA). Settings are in `ruff.toml` (line length 120) and `.pre-commit-config.yaml`.

```bash
pip3 install pre-commit
pre-commit install
```

## Versioning

The repository has one semantic version, set in every `package.xml` and `setup.py` under
`ros2/src`. Set it with:

```bash
scripts/bump_version.sh X.Y.Z
```

The script does not commit or tag. Release steps:

- Record changes under the current version heading in `CHANGELOG.md` as they are merged.
- When a release is merged to `main`, bump the version, move the changelog entries to a new
  section, and tag the merge commit `vX.Y.Z`.
- Builds that ran on the car are tagged `car-YYYY-MM-DD` so they can be traced back to the code.

Python dependencies are pinned in `requirements.txt`. Docker images are defined in `Docker/`.
