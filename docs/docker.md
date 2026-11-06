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

## Running the sim container

```bash
bash scripts/docker_run.sh [track] [mode] [gui]
```

The script runs:

```bash
docker run --rm -it \
  --gpus all \
  --env NVIDIA_DRIVER_CAPABILITIES=all \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume ~/Develop:/root/Develop \
  --network host \
  --ipc host \
  --name mfe-sim \
  mfe-driverless-sim \
  bash /root/Develop/MFE-Driverless-V1/scripts/launch_sim.sh "$EVENT" "$MODE" "$GUI"
```

### Key flags

| Flag | Why |
|------|-----|
| `--gpus all` | GPU access for YOLO inference and OpenGL rendering |
| `--network host` | ROS 2 DDS (UDP multicast) works without bridge |
| `--ipc host` | Shared memory for ROS 2 zero-copy |
| `--volume ~/Develop:/root/Develop` | Bind mount so code changes on host are immediately visible inside |
| `--volume /tmp/.X11-unix` | Gazebo GUI display passthrough |
| `--env DISPLAY` | X11 display forwarding |

---

## Multi-container setup

Two containers can run simultaneously on the same host — useful for running a second perception-only eval session alongside the main sim.

### Port conflict problem

Both containers use `--network host`. If both try to bind `GAZEBO_MASTER_URI=http://localhost:11350`, gzserver exits immediately with code 255.

### Solution: GAZEBO_PORT env var

`launch_sim.sh` reads `${GAZEBO_PORT:-11350}`. Override it when starting the second container:

```bash
# Container 1 — mfe-sim, port 11355
docker run --rm -it \
  --gpus all --network host --ipc host \
  --volume ~/Develop:/root/Develop \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --name mfe-sim \
  mfe-driverless-sim \
  bash -c "GAZEBO_PORT=11355 bash /root/Develop/MFE-Driverless-V1/scripts/launch_sim.sh peanut no_perception nogui 5"

# Container 2 — mfe-sim-2, port 11350 (default), isolated ROS domain
docker run --rm -it \
  --gpus all --network host --ipc host \
  --volume ~/Develop:/root/Develop \
  --env DISPLAY=$DISPLAY \
  --env ROS_DOMAIN_ID=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --name mfe-sim-2 \
  mfe-driverless-sim \
  bash /root/Develop/MFE-Driverless-V1/scripts/launch_sim.sh small_track no_perception nogui 0
```

### ROS domain isolation

`ROS_DOMAIN_ID` controls DDS topic namespacing. Different domain IDs prevent topic bleed-through between containers:
- mfe-sim: `ROS_DOMAIN_ID=0` (default)
- mfe-sim-2: `ROS_DOMAIN_ID=1`

---

## Gazebo lock directory

Gazebo writes a lock file at `~/.gazebo/server-<port>/` when gzserver starts. If gzserver crashes without cleanup, the lock persists. The next gzserver instance finds the lock and exits immediately with code 255.

**Fix**:
```bash
rm -rf ~/.gazebo/server-11350   # or your port
```

---

## Bind mounts

The `~/Develop` directory is bind-mounted into the container at `/root/Develop`. This means:

- Code edits on the host are immediately reflected inside the container.
- No rebuild required for Python node changes.
- C++ nodes still require `colcon build` inside the container after changes.
- EUFS sim workspace and MFE workspace are both live-mounted.

---

## Entering a running container

```bash
docker exec -it mfe-sim bash
```

Inside, the workspace is pre-sourced via the entrypoint. You can run ROS 2 commands directly:

```bash
ros2 topic list
ros2 topic echo /planning/laps_completed
ros2 service call /ros_can/set_mission eufs_msgs/srv/SetCanState '{ami_state: 13, as_state: 1}'
```

---

## Jetson deployment

The `Docker/jetson/Dockerfile` targets the Jetson Orin Nano (aarch64, JetPack 6.x). It does not use the EUFS sim or Gazebo. Instead it runs the hardware stack:

- Sensor drivers (`mfe_sensors`)
- LiDAR cone detector (compiled with `-DPLATFORM_JETSON=ON` for CUDA-PCL)
- Vision cone detector (YOLO on GPU)
- Path planning and control stack

Build and deploy natively on the Jetson; Docker is optional on hardware (ROS 2 can run bare-metal on JetPack).
