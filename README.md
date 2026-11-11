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

The stack has two entry points, one per platform. Both take the same two inputs:
`platform` (`sim` or `car`, set by the script) and `sensors`, a comma list of
`lidar,camera,imu_gps,can` (default: all four). In the sim every listed sensor is emulated the way
the car's sensor behaves, so a sim run and a car run with the same sensors use the same stack
configuration.

### Simulation (desktop with an NVIDIA GPU)

One-time setup:

```bash
mkdir -p ~/Develop && cd ~/Develop
git clone --recurse-submodules https://github.com/McGillFormulaElectric/MFE-Driverless-V1.git
cd MFE-Driverless-V1
bash scripts/docker_build.sh        # builds the mfe-driverless-sim image
```

Start the sim container. `docker_run.sh` installs Docker and the NVIDIA container toolkit if they
are missing and clones the EUFS simulator into `~/Develop/MFE26-eufs-sim`. The first start builds
the EUFS workspace and the stack inside the container, which takes a few minutes.

```bash
bash scripts/docker_run.sh shell    # interactive shell in the container
```

Then, inside the container, start a run:

```bash
cd /root/Develop/MFE-Driverless-V1
scripts/run_sim.sh --mission autocross --laps 1
scripts/run_sim.sh --track small_track --laps 5 --headless
scripts/run_sim.sh --mission accel --sensors lidar,imu_gps,can   # no camera
scripts/run_sim.sh --help
```

`run_sim.sh` opens a tmux session named `mfe` (Gazebo, sim bridge and sensor emulators, the stack,
Foxglove). Stop it with `tmux kill-session -t mfe`.

### Car (the two Jetsons)

The car stack always runs on the Jetsons. Jetson 1 (`mfe-driverless-1`) has the sensors attached
and runs the sensing half; Jetson 2 (`mfe-driverless-2`) runs SLAM, the EKF, planning, control and
the supervisor. The two are connected through the car's wired switch (`192.168.10.0/24`).

One-time setup on each Jetson (or from your PC: `ssh mfe@<jetson> "bash -s" < scripts/setup_jetson.sh`):

```bash
bash scripts/setup_jetson.sh
```

See [docs/jetson_setup.md](docs/jetson_setup.md) for the full Jetson setup.

Then, on Jetson 1:

```bash
scripts/run_car.sh --dry-run                       # preflight checks and the commands, starts nothing
scripts/run_car.sh --mission autocross --laps 1    # both Jetsons, over the wired link
scripts/run_car.sh --help
```

Before it starts, `run_car.sh` checks the wired link, the ping time and clock offset between the
Jetsons, that both run the same commit, that each listed sensor's driver is installed, that the
LiDAR is streaming, and that the CI is not using the Jetsons.

For bench tests on one Jetson without the sensors, replay a recorded bag instead:

```bash
scripts/run_car.sh --single-host --replay ~/mfe_replay_bags/<bag>
```

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

`scripts/run_sim.sh` (see Quick start) is the way to run the simulator. It uses
`scripts/launch_sim.sh`, which also takes positional arguments for older scripts; see
[docs/simulation_guide.md](docs/simulation_guide.md).

## Configuration

| Location | Contents |
|---|---|
| `ros2/src/mfe_bringup/config/` | `vehicle.yaml`, `cyclone_tailscale.xml` (DDS config for multi-machine setups) |
| `ros2/src/mfe_state_estimation/config/` | EKF parameters (`ekf_params.yaml`) |
| `ros2/src/mfe_mapping/config/` | `slam_toolbox_params.yaml` |
| `ros2/src/mfe_sensors/config/` | Sensor configs: `xsens_mti.yaml` |

Python dependencies are pinned in `requirements.txt`. Docker images are defined in `Docker/`.

## Bench testing (Jetson 2, live LiDAR + camera)

Runs the full stack on Jetson 2 with the VLP-16 and D435i attached. The car stays still: no CAN, and `bench_odom.py` fakes a stationary pose (never use it on a moving car).

Once per Jetson, raise the UDP receive buffers or point clouds get dropped:
```bash
echo -e "net.core.rmem_max=33554432\nnet.core.rmem_default=33554432" | sudo tee /etc/sysctl.d/60-mfe-udp.conf && sudo sysctl --system
```

```bash
scripts/bench/build_bench.sh
scripts/bench/start_bench.sh   # options: MISSION=, DRIVERS=lidar, FOXGLOVE=0, ENGINE=, LIDAR_IP=, STACK_ARGS= (e.g. "color_mode:=none roi_x_min:=1.0 roi_x_max:=10.0 roi_half_width:=2.5": colourless cones in a box ahead)
scripts/bench/stop_bench.sh
```
Logs go to `~/mfe_bench_runs/latest`. Foxglove: `ws://<jetson-ip>:8765` with the layout `scripts/bench/foxglove_bench_layout.json`.
