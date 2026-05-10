# MFE Driverless — Simulation Guide

Complete reference for running the autonomous stack in Gazebo, visualizing in Foxglove, and training the vision model.

---

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [File Reference](#2-file-reference)
3. [Running the Sim](#3-running-the-sim)
4. [Foxglove Visualization](#4-foxglove-visualization)
5. [Training YOLO](#5-training-yolo)
6. [Running Multiple Containers](#6-running-multiple-containers)
7. [Issues & Fixes Log](#7-issues--fixes-log)

---

## 1. Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│  Docker container: mfe-driverless-sim (--network host)      │
│                                                             │
│  tmux session "mfe" (6 panes)                               │
│  ┌──────────────────┬──────────────────┐                    │
│  │ pane 0: EUFS Sim │ pane 3: Foxglove │                    │
│  │ (gzserver)       │ bridge :8765     │                    │
│  ├──────────────────┼──────────────────┤                    │
│  │ pane 1: Bridge   │ pane 4: Mission  │                    │
│  │ (sim↔ROS)        │ control          │                    │
│  ├──────────────────┼──────────────────┤                    │
│  │ pane 2: MFE      │ pane 5: Logger   │                    │
│  │ stack (bringup)  │                  │                    │
│  └──────────────────┴──────────────────┘                    │
└─────────────────────────────────────────────────────────────┘
```

### Data flow — no_perception mode (ground truth cones)

```
EUFS Gazebo sim
  → /ground_truth/cones_colored  (mfe_eufs_sim bridge)
  → /planning/cones              (path_planner_node)
  → /planning/centerline         (pure_pursuit_node)
  → /control/command             (EUFS car actuator)
```

### Data flow — perception mode (real LiDAR + camera)

```
EUFS Gazebo sim
  → /lidar/points_raw            (lidar_cone_detector)
  → lidar/pcl/objects            (boundary_extractor)
  → /camera/image_raw            (vision_cone_detector / YOLO)
  → /planning/cones              (path_planner_node)
  → /planning/centerline         (pure_pursuit_node)
  → /control/command             (EUFS car actuator)
```

### TF frame chain

```
map → odom → base_footprint → velodyne
                             → zed_camera_center
```

`publish_gt_tf:=true` (always set) makes EUFS publish this chain from Gazebo ground truth.
If this is false, the car has no idea where it is in the map frame and drives in circles.

---

## 2. File Reference

### Scripts

| File | Purpose |
|------|---------|
| `scripts/docker_run.sh` | Start a one-shot Docker container and immediately launch the sim. Handles Docker install, nvidia-container-toolkit, X11 auth, and bind mounts. |
| `scripts/launch_sim.sh` | Launch all sim components inside the container via tmux. Accepts `[track] [mode] [gui]`. Kills stale processes, restarts ROS daemon, waits for Gazebo, spawns car if EUFS didn't. |
| `scripts/run_laps.sh` | Automated lap runner — restarts the sim for each lap and records success/fail/time to `/tmp/lap_results.txt`. |
| `scripts/docker_build.sh` | Build the `mfe-driverless-sim` Docker image. |
| `scripts/setup_linux.sh` | Host-side setup: installs dependencies, configures udev rules, sets up WezTerm. |

### ROS packages (`ros2/src/`)

| Package | Node(s) | Role |
|---------|---------|------|
| `mfe_bringup` | — | Launch file only. Assembles and configures all MFE nodes for one command launch. The single entry point for the stack. |
| `mfe_eufs_sim` | `bridge_node` | Translates EUFS/Gazebo topics to MFE topics. In `no_perception` mode sends ground-truth cones directly to `/planning/cones`. Publishes static TF for sensor frames. |
| `mfe_path_planning` | `path_planner_node` | Wraps `ft-fsd-path-planning` library. Converts `/planning/cones` + odometry into `/planning/centerline`. Skidpad uses a hardcoded pre-computed path. |
| `mfe_path_planning` | `boundary_extractor` | Fuses LiDAR clusters and vision detections into `/planning/cones` (only runs in `perception` mode). |
| `mfe_path_planning` | `finish_detector_node` | Detects lap completion via LiDAR point-cloud gate or return-to-start odometry. Publishes brake and signals EUFS state machine. |
| `mfe_control` | `pure_pursuit_node` | Pure pursuit lateral controller. Tracks `/planning/centerline` using current pose from odometry. Publishes `/control/command`. |
| `mfe_state_estimation` | `extended_kalman_filter_node` | Fuses IMU + GPS into `/ekf/output`. In simulation, bypassed by using `/ground_truth/state_odom` directly. |
| `mfe_mapping` | SLAM toolbox | Builds occupancy map from LiDAR laser scans. Publishes map→odom TF in perception mode. |
| `mfe_perception/lidar_cone_detector` | `lidar_perception_node` | C++ node. Voxel filter → RANSAC ground removal → Euclidean clustering. Outputs cone positions as PointCloud2. |
| `mfe_perception/vision_cone_detector` | `cone_detection_node` | Python/YOLO node. Runs YOLOv8/YOLO11 on `/camera/image_raw`, publishes `mfe_msgs/Track`. |
| `mfe_perception/perception_evaluator` | `evaluator_node` | Sim-only. Compares `/planning/cones` vs `/ground_truth/cones_colored`. Reports P/R/F1 and mean distance error. |

### Key source files

| File | What it does |
|------|-------------|
| `ros2/src/mfe_bringup/launch/bringup.launch.py` | The main launch file. Declares all args (`mission`, `pose_topic`, `use_perception`, `endless`, `vision_model_path`). Uses `OpaqueFunction` for mission-dependent node parameters. |
| `ros2/src/mfe_path_planning/mfe_path_planning/path_planner_node.py` | Path planner wrapper. Filters cones to 20 m radius around car before feeding ft-fsd-path-planning. Maps orange cones to UNKNOWN (prevents library finish-line logic from corrupting path). |
| `ros2/src/mfe_path_planning/mfe_path_planning/finish_detector_node.py` | Finish detector. Has two modes: x-position fallback (linear tracks) and return-to-start (closed-loop tracks). Guards figure-8 crossings with `max_dist_from_start >= 15 m` check. |
| `ros2/src/mfe_control/mfe_control/pure_pursuit_node.py` | Pure pursuit. Speed profile uses `speed_reduction_factor` at high curvature. Mission-specific defaults set in bringup launch. |
| `training/yolo/train.py` | Sequential YOLO training: yolov8n → yolov8s → yolo11n → yolo11s on FSOCO dataset. Saves best weights to `~/mfe_models/yolo/cone_detector_best.pt` for ROS. |

### Docker images

| Image | Purpose |
|-------|---------|
| `mfe-driverless-sim` | ROS2 Humble + Gazebo 11 + all MFE dependencies. Source built inside at image build time. Bind mounts `~/Develop` so live code changes are reflected without rebuild. |

---

## 3. Running the Sim

### Prerequisites

```bash
# Build the Docker image once (takes ~15 min)
bash scripts/docker_build.sh

# Or pull if already built:
docker images | grep mfe-driverless-sim
```

### Quick start (from host)

```bash
# Acceleration event, no perception, with Gazebo GUI
bash scripts/docker_run.sh accel no_perception gui

# Peanut figure-8 (autocross), headless
bash scripts/docker_run.sh peanut no_perception nogui

# Small track (autocross), with real LiDAR+camera perception
bash scripts/docker_run.sh small_track perception nogui
```

`docker_run.sh` starts a container named `mfe-sim` and immediately runs `launch_sim.sh` inside it. The container exits when the sim is stopped.

### Manual launch inside an existing container

```bash
# If the container is already running:
docker exec -it mfe-sim bash

# Inside container:
bash /root/Develop/MFE-Driverless-V1/scripts/launch_sim.sh [track] [mode] [gui]
```

### Available tracks

| Argument | Track | Mission | Notes |
|----------|-------|---------|-------|
| `accel` / `acceleration` | acceleration | acceleration | Straight line, 78 m |
| `skidpad` | skidpad | skidpad | Figure-8 circles, hardcoded path |
| `peanut` | peanut | autocross | Figure-8, best for algorithm testing |
| `small_track` / `autocross` | small_track | autocross | Small closed loop |
| `rectangle` | rectangle | autocross | Rectangular closed loop |
| `garden_light` | garden_light | autocross | — |
| `boa_constrictor` | boa_constrictor | autocross | — |
| `comp_2021` | comp_2021 | autocross | — |
| `hairpins` | hairpins_increasing_difficulty | autocross | — |
| `rand` | rand | autocross | Random layout |

### Starting the car (after the sim launches)

The tmux pane 4 has the mission command pre-typed but **not sent**. Press Enter in pane 4, or run:

```bash
# Inside container (source first):
ros2 service call /ros_can/set_mission eufs_msgs/srv/SetCanState '{ami_state: 13, as_state: 1}'
# ami_state: 11=acceleration, 12=skidpad, 13=autocross
```

The EUFS state machine requires `READY` (as_state=1) before it accepts `DRIVING`. Sending `DRIVING` directly from `OFF` is refused.

### Endless (multi-lap) mode

By default, the car stops after one lap. To loop indefinitely for Foxglove visualization:

```bash
# Pass endless:=true to bringup — finish_detector is disabled
ros2 launch mfe_bringup bringup.launch.py \
  mission:=autocross \
  pose_topic:=/ground_truth/state_odom \
  use_perception:=false \
  endless:=true
```

Or override the BRINGUP_EXTRAS in the launch script before sourcing:

```bash
# In the tmux pane 2 (MFE stack), kill current launch and rerun:
ros2 launch mfe_bringup bringup.launch.py \
  mission:=autocross pose_topic:=/ground_truth/state_odom \
  use_perception:=false endless:=true
```

### No_perception vs perception mode

| | `no_perception` | `perception` |
|-|-----------------|--------------|
| Cone source | Gazebo ground truth → bridge → `/planning/cones` | LiDAR clusters + YOLO camera → boundary_extractor → `/planning/cones` |
| Pose source | `/ground_truth/state_odom` | `/ekf/output` (IMU + GPS fusion) |
| `use_perception` arg | `false` | `true` |
| `use_sim_cones_directly` | `true` | `false` |
| Speed | Faster to test | Slower, tests real pipeline |
| Camera required | No | Yes (needs rendering) |

### Speed and controller tuning

Mission-specific parameters in `bringup.launch.py` `_make_pure_pursuit()`:

| Mission | max_speed | lookahead_distance | Notes |
|---------|-----------|--------------------|-------|
| acceleration | 13.0 m/s | 8.0 m | Long straight, high speed |
| autocross | 7.0 m/s | 3.5 m | Tight tracks, short lookahead |
| skidpad | 4.5 m/s | 3.0 m | Tight 9.1 m radius circles |

### Checking car state

```bash
# State machine
ros2 topic echo /ros_can/state_str --once

# Speed and position
ros2 topic echo /ground_truth/state --once

# Path being followed
ros2 topic echo /planning/centerline --once

# Whether lap is done
ros2 topic echo /planning/mission_finished --once
```

### Automated lap testing

```bash
# Run laps 1-10 at 7 m/s, logging results to /tmp/lap_results.txt
bash /root/Develop/MFE-Driverless-V1/scripts/run_laps.sh 1 10 7
```

---

## 4. Foxglove Visualization

Foxglove Bridge runs automatically in tmux pane 3 when the sim starts. It exposes all ROS topics as a WebSocket.

### Connect

1. Open **https://app.foxglove.dev** in your browser (or the Foxglove desktop app)
2. Click **Open connection**
3. Select **Foxglove WebSocket**
4. URL: `ws://localhost:8765`

### Recommended panels

| Panel type | Topic | What you see |
|-----------|-------|-------------|
| 3D | `/planning/centerline` (nav_msgs/Path) | Planned path ahead of car |
| 3D | `/planning/track_left` + `/planning/track_right` | Track boundaries |
| 3D | `/ground_truth/state` | Car pose in world frame |
| 3D | `/lidar/points_raw` | Raw LiDAR point cloud (perception mode) |
| Plot | `/ground_truth/state` → `twist.twist.linear.x` | Car speed over time |
| Raw messages | `/ros_can/state_str` | State machine: AS/AMI/MISSION_COMPLETED |
| Raw messages | `/planning/mission_finished` | `true` when lap complete |
| Image | `/camera/image_raw` | Camera feed (requires X11 rendering — see note) |
| Diagnostics | `/perception/accuracy` | LiDAR/vision P/R/F1 (perception mode) |

### Camera note

`/camera/image_raw` is empty in headless mode because Gazebo's camera sensor requires OpenGL rendering. The rendering thread only starts when gzserver can connect to a display. To get camera images:

- Run with X11 forwarding: `xhost +local:docker` on the host before starting the container (already done by `docker_run.sh`)
- The container needs xauth credentials for root: Gazebo will connect to `DISPLAY=:0` using the host X server
- If images are still empty, the camera sensor isn't publishing — check with `ros2 topic hz /camera/image_raw`

---

## 5. Training YOLO

### Setup

```bash
# Dataset must be prepared first (FSOCO in YOLO format)
# Expected at: ~/mfe_datasets/fsoco_yolo/data.yaml

# Run training (outside Docker, on host GPU):
cd ~/Develop/MFE-Driverless-V1/training/yolo
python3 train.py \
  --models yolov8n yolov8s yolo11n yolo11s \
  --epochs 100 \
  --batch 32
```

### What it trains

Sequential training: yolov8n → yolov8s → yolo11n → yolo11s. Each model is evaluated on the FSOCO dataset. The best model by mAP50 is automatically copied to `~/mfe_models/yolo/cone_detector_best.pt`.

### Using trained weights in the sim

```bash
ros2 launch mfe_bringup bringup.launch.py \
  vision_model_path:=~/mfe_models/yolo/cone_detector_best.pt
```

Or point at a specific run:

```bash
vision_model_path:=~/mfe_models/yolo/yolo11s/weights/best.pt
```

If `vision_model_path` is empty or the file doesn't exist, the vision node is skipped (logged to console). The stack continues with LiDAR-only perception.

### Monitor training

```bash
# GPU utilization
nvidia-smi dmon -s u -d 2

# Training log (Ultralytics writes to the run dir)
tail -f ~/mfe_models/yolo/yolo11s/results.csv
```

### Workers setting

Keep `workers=2` when the sim containers are running. With `workers=8` the data loaders consume all RAM, swap thrashes, and GPU utilization drops to ~3% (the CPU starves the GPU). With `workers=2` and `batch=128`, the RTX 5060 Ti runs at ~13.4 GB / 16 GB.

---

## 6. Running Multiple Containers

Use two containers to run sim and perception eval simultaneously, or to isolate experiments.

### The port conflict problem

Both containers use `--network host`, so they share the host network namespace. Two gzserver instances **cannot** bind the same GAZEBO_MASTER_URI port. The default is 11350.

**Solution:** Set `GAZEBO_PORT` before launching:

```bash
# Container 1 (default port 11350)
docker exec -d mfe-sim bash -c \
  "bash /root/Develop/MFE-Driverless-V1/scripts/launch_sim.sh peanut no_perception nogui"

# Container 2 (different port 11355)
docker exec -d mfe-sim-2 bash -c \
  "GAZEBO_PORT=11355 bash /root/Develop/MFE-Driverless-V1/scripts/launch_sim.sh peanut perception nogui"
```

`launch_sim.sh` reads `${GAZEBO_PORT:-11350}` and propagates it to all tmux panes via `SOURCE_ALL`.

### ROS topic isolation

Both containers also use `--network host`, so ROS DDS traffic from both is visible to both. Use `ROS_DOMAIN_ID` to isolate:

```bash
# Container 2 (domain 1 instead of default 0)
docker exec -d mfe-sim-2 bash -c \
  "ROS_DOMAIN_ID=1 GAZEBO_PORT=11355 bash ...launch_sim.sh..."
```

With different domain IDs, `ros2 topic list` in each container only shows its own topics.

### Starting the second container

```bash
docker run -d \
  --gpus all --env NVIDIA_DRIVER_CAPABILITIES=all \
  --env DISPLAY="$DISPLAY" \
  --env ROS_DOMAIN_ID=1 \
  --volume "$HOME/Develop":/root/Develop \
  --network host --ipc host \
  --name mfe-sim-2 \
  mfe-driverless-sim \
  sleep infinity
```

---

## 7. Issues & Fixes Log

### Car driving in circles (not following track)

**Symptom:** Car spins in place or drives in a tight circle ignoring cones.

**Root cause:** `publish_gt_tf:=false` was set in `no_perception` mode. Without this, the EUFS sim does not publish the `map → odom → base_footprint` TF chain. The path planner and pure pursuit compute paths in the `map` frame, but the car has no pose in `map`, so it can't track anything.

**Fix:** Always set `publish_gt_tf:=true`. In `launch_sim.sh` both perception and no_perception branches now set `PUBLISH_GT_TF=true`.

---

### Car hitting cones / path planning poor on figure-8

**Symptom:** Car goes straight through cones at the crossing, or takes huge arcing paths.

**Three causes fixed:**

1. **Orange cones corrupting path:** `ft-fsd-path-planning` treats `ORANGE_BIG` and `ORANGE_SMALL` cones as `START_FINISH_AREA` and applies special logic near them, which deforms the path at the start gate. Fixed by mapping all orange cones to `ConeTypes.UNKNOWN` in `path_planner_node.py`.

2. **Lookahead too long:** 8 m lookahead on a figure-8 that crosses itself causes the car to "see past" the crossing and aim at the wrong branch. Reduced to 3.5 m for autocross.

3. **Full track visible to planner:** With all cones in view, the planner gets confused at the crossing. Fixed by filtering cones to a 20 m radius around the car in `_cones_cb`.

---

### Car stops at figure-8 crossing (false finish detection)

**Symptom:** Car stops and brakes after crossing the start area mid-lap.

**Root cause:** The finish detector used an x-position threshold (`finish_x=100 m`) which was never reached on the peanut track (max x ≈ 25 m). The detector then fell through to position-based logic that incorrectly fired near the crossing.

**Fix:** Replaced x-position detection for closed-loop tracks with return-to-start logic:
- `min_travel_m=60.0` — must travel at least 60 m before detection activates
- `return_to_start_r=5.0` — fires when car is within 5 m of start position
- `max_dist_from_start >= 15.0` guard — prevents false positive at the figure-8 crossing where the car briefly passes near origin

---

### gzserver exits with code 255

**Symptom:** Gazebo fails immediately with `[gzserver-1]: process has died [exit code 255]`.

**Root cause:** Port 11350 (GAZEBO_MASTER_URI) already in use from a previous crashed run. Gazebo leaves a lock directory at `~/.gazebo/server-11350/` even when the process dies. On next start, gzserver sees the lock and exits.

**Fix:**
```bash
# Clear stale lock
rm -rf ~/.gazebo/server-11350 ~/.gazebo/server-11355

# If a second container is also using 11350, use a different port
GAZEBO_PORT=11355 bash scripts/launch_sim.sh ...
```

`launch_sim.sh` now reads `GAZEBO_PORT` from environment, defaulting to 11350.

---

### Two containers conflicting on port 11350

**Symptom:** One container's gzserver starts, the other's fails immediately with "Address already in use".

**Root cause:** Both containers use `--network host`, sharing the host network namespace. Both `launch_sim.sh` scripts hardcoded `GAZEBO_MASTER_URI=http://localhost:11350`.

**Fix:** Made the port configurable via `GAZEBO_PORT` env var. First container uses 11350 (default), second uses `GAZEBO_PORT=11355`.

---

### YOLO training stuck at 3% GPU utilization

**Symptom:** `nvidia-smi` shows ~944 MB VRAM used (out of 16 GB), GPU at 3%, training at 1 epoch/hour.

**Root cause:** `workers=8` dataloader workers were consuming all 8 GB of swap. The CPU was fully busy swapping pages in/out, starving the GPU of data. Training was IO-bound, not GPU-bound.

**Fix:** Set `workers=2` in `train.py`. Also increased `batch=128` to give the GPU a larger workload per step. Result: 13.4 GB VRAM used, GPU at 80%+.

---

### Corrupted `.pt` weight files

**Symptom:** `RuntimeError: PytorchStreamReader failed reading zip archive: failed finding central directory`.

**Root cause:** Training was killed mid-download of the base model checkpoint (`yolo11n.pt`), leaving a partial file.

**Fix:** Delete and re-download:
```bash
cd training/yolo
rm -f yolo11n.pt yolo11s.pt
python3 train.py --models yolo11n yolo11s
# Ultralytics re-downloads automatically on next run
```

---

### `docker exec` process killed by pkill inside container

**Symptom:** Running `pkill -f ros2` inside a container via `docker exec` terminates the `docker exec` process itself.

**Root cause:** `pkill` matches the process name of the `docker exec` bash shell (which appears as a `ros2` invocation in `/proc/cmdline` if the command is long enough, or kills the shell's own process group).

**Fix:** Kill by specific PID from the host, or use `-d` (detached) `docker exec` for cleanup commands:
```bash
docker exec -d mfe-sim bash -c "pkill -f gzserver; pkill -f path_planner"
```

---

### Car not starting (mission service refused)

**Symptom:** `ros2 service call /ros_can/set_mission` returns `success=False` or times out.

**Root cause:** EUFS state machine starts in `AS:OFF`. It refuses `DRIVING` (as_state=2) directly. Must go to `READY` (as_state=1) first.

**Fix:** Always use `as_state: 1` (READY). The state machine then transitions to DRIVING automatically once the car receives control commands.

```bash
ros2 service call /ros_can/set_mission eufs_msgs/srv/SetCanState \
  '{ami_state: 13, as_state: 1}'
```

---

### Zombie processes accumulating in container

**Symptom:** `ps aux` shows dozens of `[gzserver] <defunct>`, `[python3] <defunct>`. New processes can't start.

**Root cause:** Container PID 1 is `sleep infinity`, which does not call `wait()` to reap child processes. Every process that exits becomes a zombie until the container restarts.

**Mitigation:** Use `--init` flag when creating the container (Docker's tini init reaps zombies), or restart the container between major test runs. The zombie count doesn't grow unboundedly in practice — only orphaned children of the session that's no longer tracked become zombies.

---

### tmux session not persisting from `docker exec -d`

**Symptom:** `docker exec -d mfe-sim bash launch_sim.sh` returns immediately, but tmux session disappears.

**Root cause:** With `-d` (detached), Docker closes the stdin/stdout of the process. `tmux new-session -d` inside a script that was itself started detached gets confused about its controlling terminal.

**Fix:** Use `nohup` and background explicitly:
```bash
docker exec -d mfe-sim bash -c \
  "nohup bash /root/Develop/MFE-Driverless-V1/scripts/launch_sim.sh peanut no_perception nogui \
   > /tmp/launch_sim.log 2>&1 &"
```
