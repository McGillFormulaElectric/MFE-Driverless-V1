# Docker

Docker images, Dockerfiles, multi-container setup, and port isolation.

---

## Images

| Image | Dockerfile | Used for |
|-------|-----------|---------|
| `mfe-driverless-sim` | `Docker/fs-driverless-sim/Dockerfile` | Main simulation container (primary) |
| `dev` | `Docker/dev/Dockerfile` | Development environment |
| `dev-cuda` | `Docker/dev-cuda/Dockerfile` | CUDA-enabled dev environment |
| `jetson` | `Docker/jetson/Dockerfile` | Jetson Orin Nano deployment |

---

## mfe-driverless-sim (primary sim image)

**Base**: `osrf/ros:humble-desktop`

**Build**:
```bash
bash scripts/docker_build.sh
# Equivalent to:
docker build -t mfe-driverless-sim Docker/fs-driverless-sim
```

**What's installed**:
- ROS 2 Humble desktop (base)
- Gazebo 11 (`gazebo`, `libgazebo-dev`, `ros-humble-gazebo-ros-pkgs`)
- SLAM Toolbox (`ros-humble-slam-toolbox`)
- Foxglove Bridge (`ros-humble-foxglove-bridge`)
- PCL + PointCloud2→LaserScan (`ros-humble-pcl-ros`, `ros-humble-pointcloud-to-laserscan`)
- CV bridge (`ros-humble-cv-bridge`)
- TF2 (`ros-humble-tf2-ros`, `ros-humble-tf2-geometry-msgs`)
- Python: `numpy<2`, `scipy>=1.13`, `scikit-learn`, `icecream`, `pandas`
- `ft-fsd-path-planning` (installed from source — PyPI package is broken)

**Entrypoint**: `Docker/fs-driverless-sim/entrypoint.sh` — sources ROS 2 setup files.

**CUDA note**: `cuda-pcl` (NVIDIA-AI-IOT) ships ARM aarch64 `.so` files only. No x86_64 build exists. The desktop sim image uses CPU PCL. On Jetson, build natively without Docker.

---

## Preparing the workspaces

The image supplies system dependencies; source and build artifacts live in the
mounted host directories. Keep both repositories on compatible branches based
on the EUFS `humble` layout. Existing branches are never switched automatically.

```bash
bash scripts/setup_sim_workspaces.sh
bash scripts/docker_build.sh
```

Setup clones a missing EUFS checkout from `humble`, fetches `eufs_msgs` separately,
and initializes the MFE `fs_msgs` submodule. An incomplete existing
EUFS directory is reported instead of being overwritten. The sensor / launch
fixes must be present in the EUFS branch as well as this repository.

The entrypoint builds EUFS, then **MFE-Driverless-V1/ros2**, incrementally on every
run. Python executable registration and installed launch files are refreshed.
CMake caches are cleared to handle source paths that changed between branches.
Do not build the MFE repository root: the launcher uses `ros2/install`.
The legacy MFE `eufs_msgs` submodule is excluded from that build; EUFS supplies it.

## Running the simulator

```bash
# Gazebo GUI with perception; 0 means run indefinitely
bash scripts/docker_run.sh accel perception gui 0

# Headless with real simulated LiDAR and camera streams
bash scripts/docker_run.sh accel perception nogui 0

# Ground-truth cone bypass without raw perception sensors
bash scripts/docker_run.sh accel no_perception nogui 0
```

No temporary wrapper is needed. Startup waits for actual car-state messages and,
in perception mode, LiDAR and D435i images. Missing data or a launch failure
returns a nonzero exit status and prints simulator logs; it does not announce
readiness and launch the remaining stack. `MFE_STARTUP_TIMEOUT` inside the
container controls the deadline (120 seconds by default).

The mission pane is prefilled. Press Enter there when you want to start driving.
Detaching tmux exits the foreground launch command; a `--rm` container then stops.

### Rendering and GPU access

`MFE_GPU=auto` uses NVIDIA when a functioning driver and Docker's NVIDIA runtime
are available; otherwise it uses Mesa software rendering. Software rendering is
slower. GUI mode uses the host X11 display. Headless mode starts Xvfb inside the
container, because hiding the Gazebo GUI does not remove the cameras' rendering
requirements.

```bash
MFE_GPU=software bash scripts/docker_run.sh accel perception gui 0
MFE_GPU=nvidia bash scripts/docker_run.sh accel perception gui 0
```

The NVIDIA option first tests container GPU access. Install and configure
[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
on the host if this fails. The run script does not install drivers or restart
Docker. GUI mode temporarily grants X11 access to the container's root user and
revokes the grant it added on exit.

### Direct Docker command

The original headless command continues to work with the updated image and
prepared workspaces:

```bash
docker run --rm -it --init \
  --volume "$HOME/Develop":/root/Develop \
  --publish 8765:8765 --ipc host --name mfe-sim \
  mfe-driverless-sim \
  bash /root/Develop/MFE-Driverless-V1/scripts/launch_sim.sh accel perception nogui 0
```

For GUI mode use the helper above, which handles the display mount and GPU check.
Port 8765 exposes Foxglove. The default helper uses Docker's bridge network so
other host ROS processes are isolated; no host network is required for nodes
communicating inside this container. Concurrent runs need different container
names and published Foxglove ports.

## Diagnostics and smoke check

While the container is running:

```bash
docker exec -it mfe-sim bash
source /opt/ros/humble/setup.bash
source /root/Develop/MFE26-eufs-sim/install/setup.bash
source /root/Develop/MFE-Driverless-V1/ros2/install/setup.bash
python3 /root/Develop/MFE-Driverless-V1/scripts/check_sim.py --mode perception
```

The smoke check requires live bridge odometry, nonempty LiDAR/images, matching
camera calibration, and URDF sensor transforms. It does not certify driving or
perception accuracy. EUFS uses the MFE LiDAR and D435i definitions. Its
robot_state_publisher owns sensor TF; the bridge must not publish competing
approximate transforms. The bridge forwards `/d435i/image_raw` and
`/d435i/camera_info` to `/camera/image_raw` and `/camera/camera_info`.

A `load_yaml` deprecation or audio warning is not itself a spawn failure. Read
the simulator pane or its `/tmp/mfe-launch-*/simulator.log` for the actual error.

---

## Jetson deployment

The `Docker/jetson/Dockerfile` targets the Jetson Orin Nano (aarch64, JetPack 6.x). It does not use the EUFS sim or Gazebo. Instead it runs the hardware stack:

- Sensor drivers (`mfe_sensors`)
- LiDAR cone detector (compiled with `-DPLATFORM_JETSON=ON` for CUDA-PCL)
- Vision cone detector (YOLO on GPU)
- Path planning and control stack

Build and deploy natively on the Jetson; Docker is optional on hardware (ROS 2 can run bare-metal on JetPack).
