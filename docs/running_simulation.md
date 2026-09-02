# Running the MFE Driverless Simulation

## Prerequisites

- Docker Desktop installed and running
- macOS with Apple Silicon or Intel
- For GUI (Gazebo window): XQuartz installed
  ```bash
  brew install --cask xquartz
  # Then: open XQuartz → Preferences → Security → tick "Allow connections from network clients"
  # Log out and back in after installing
  ```

## Quick Start (Mac)

```bash
cd ~/Develop/MFE-Driverless-V1
bash scripts/docker_run_mac.sh [track] [mode] [gui|nogui] [laps]
```

### Arguments

| Argument | Options | Default | Description |
|---|---|---|---|
| `track` | `accel`, `skidpad`, `peanut`, `autocross`, `small_track`, `rectangle`, `hairpins` | `accel` | Which FSAE track layout to load |
| `mode` | `perception`, `no_perception` | `no_perception` | Whether to run the full perception stack |
| `gui` | `gui`, `nogui` | `nogui` | Whether to open the Gazebo window (slow on Mac without GPU) |
| `laps` | integer, `0` = endless | `1` | Number of laps before stopping |

### Example commands

```bash
# Acceleration track, no perception (test path planner only)
bash scripts/docker_run_mac.sh accel no_perception nogui 1

# Autocross with full lidar perception pipeline
bash scripts/docker_run_mac.sh autocross perception nogui 1

# Peanut track with Gazebo window open
bash scripts/docker_run_mac.sh peanut perception gui 0
```

## Modes explained

### `no_perception` mode
- EUFS sim ground-truth cone positions go directly to `/planning/cones`
- Skips all perception nodes (lidar, vision, evaluators)
- Use this to test the path planner and controller in isolation

### `perception` mode
- LiDAR data from Gazebo (`/velodyne_points`) flows through `lidar_perception_node`
- Camera data flows through `vision_cone_detector`
- Output cone detections feed into path planning
- Use this to test the full stack end-to-end

## What runs inside Docker (tmux panes)

| Pane | What it runs |
|---|---|
| 0 (top-left) | EUFS Gazebo sim (`eufs_launcher`) |
| 1 (mid-left) | MFE bridge node (`mfe_eufs_sim`) |
| 2 (bottom-left) | MFE bringup (full stack) |
| 3 (top-right) | Foxglove bridge (port 8765) |
| 4 (mid-right) | Mission start command (pre-filled, press Enter when ready) |
| 5 (bottom-right) | State logger |

## Starting the car

Once Gazebo is ready (pane 0 shows no errors), go to **pane 4** and press `Enter` to send the mission start signal. The car will begin driving autonomously.

tmux navigation:
- `Ctrl+B` then arrow keys — move between panes
- `Ctrl+B` then `d` — detach (sim keeps running)
- `tmux attach-session -t mfe` — reattach

## Visualising in Foxglove

1. Open [studio.foxglove.dev](https://studio.foxglove.dev)
2. **Open connection** → **Rosbridge WebSocket** → `ws://localhost:8765`

Useful topics to add in a 3D panel:

| Topic | Shows |
|---|---|
| `/velodyne_points` | Raw LiDAR scan |
| `/debug/gpu_objects` | Points after ground removal + height filter (pre-cluster) |
| `/perception/cones_uncolored` | Detected cone centroids |
| `/perception/cones_markers` | Bounding box markers around each detection |
| `/ground_truth/track` | Actual cone positions from sim |
| `/planning/path` | Planned path |

## Testing the lidar pipeline specifically

To run only the lidar pipeline against sim data (without the full bringup):

**Terminal 1 — inside the Docker container:**
```bash
source /opt/ros/humble/setup.bash
source ~/Develop/MFE-Driverless-V1/install/setup.bash
ros2 launch lidar_cone_detector lidar_pipeline.launch.py
```

**Terminal 2 — Foxglove bridge:**
```bash
source /opt/ros/humble/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## Rebuilding the workspace inside Docker

```bash
cd ~/Develop/MFE-Driverless-V1
source /opt/ros/humble/setup.bash
colcon build --packages-select lidar_cone_detector --symlink-install
source install/setup.bash
```

## Tunable lidar parameters (no rebuild required)

Edit `ros2/src/mfe_perception/lidar_cone_detector/launch/lidar_pipeline.launch.py`:

```python
{'leaf_size': 0.05},          # voxel size (m)
{'ground_threshold': 0.1},    # RANSAC inlier band (m)
{'cluster_tolerance': 0.2},   # max gap between cluster points (m)
{'min_cluster_size': 5},      # minimum points to form a cone cluster
{'max_cluster_size': 50},     # maximum points (rejects large objects)
{'min_intensity': 100.0},     # retroreflective tape threshold (0 = disabled)
```

## Known differences between sim and real hardware

| Aspect | Sim | Real |
|---|---|---|
| Ground noise | None (perfect flat) | Slight unevenness → RANSAC less stable |
| NaN points | None | Present (invalid returns → stripped at callback entry) |
| Intensity values | Uniform | Retroreflective tape returns 200–255 |
| False positives | Only cones in scene | Walls, people, chairs can pass filters |
| LiDAR frame rate | ~10 Hz | ~10–20 Hz depending on VLP-16 config |

Sim is ideal for validating geometric detection (cluster size, bounding box, tolerance). Intensity filtering and false-positive rejection require real hardware testing.
