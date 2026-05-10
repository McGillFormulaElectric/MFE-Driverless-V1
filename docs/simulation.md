# Simulation Guide

How to start the EUFS sim, pick a track, configure perception mode, use Foxglove, and run multi-lap sessions.

---

## Quick start

```bash
# 1. Build the Docker image (once)
bash scripts/docker_build.sh

# 2. Run a simulation (inside Docker, everything auto-starts)
bash scripts/docker_run.sh [track] [mode] [gui] [laps]
```

Arguments (all optional, all positional):

| Argument | Default | Options |
|----------|---------|---------|
| track | `accel` | `accel`, `skidpad`, `peanut`, `small_track`, `rectangle`, `autocross`, `garden_light`, `boa_constrictor`, `comp_2021`, `hairpins`, `rand`, `its_a_mess` |
| mode | `no_perception` | `no_perception`, `perception` |
| gui | `gui` | `gui`, `nogui` |
| laps | `1` | any positive integer, or `0` for endless |

**Running interactively**: `docker_run.sh` attaches a tmux session and needs a real TTY. In Claude Code, prefix with `!`:
```
! bash scripts/docker_run.sh peanut no_perception nogui 5
```
Once the tmux session opens, **press Enter in pane 4** (mid-right) to start the car.

Examples:

```bash
# Peanut figure-8, GT cones, headless, 3 laps
bash scripts/docker_run.sh peanut no_perception nogui 3

# Acceleration track, GUI, perception mode (needs YOLO weights)
bash scripts/docker_run.sh accel perception gui 1

# Small track, endless loop for Foxglove visualization
bash scripts/docker_run.sh small_track no_perception nogui 0
```

---

## Track reference

| Track name | Mission | Shape | Notes |
|-----------|---------|-------|-------|
| `accel` / `acceleration` | acceleration | Straight, ~78 m | Finish at x=78 |
| `skidpad` | skidpad | Figure-8 circles | Hardcoded 2L+2R path |
| `peanut` | autocross | Closed figure-8 | Spawn at origin; lap detection via return-to-start |
| `small_track` / `autocross` | autocross | Closed loop | Spawn at (-13, 10.3) |
| `rectangle` | autocross | Closed rectangle | Spawn at origin |
| `garden_light` | autocross | Closed loop | Spawn at origin |
| `boa_constrictor` | autocross | Closed loop | Spawn at origin |
| `comp_2021` | autocross | Competition track | Spawn at origin |
| `hairpins` | autocross | Tight hairpins | Uses `hairpins_increasing_difficulty` track |
| `rand` | autocross | Random | Spawn at origin |
| `its_a_mess` | autocross | Complex | Spawn at origin |

---

## Perception modes

### no_perception (default, recommended for sim)

The bridge reads ALL track cones directly from Gazebo world state and forwards them to the path planner. No camera or LiDAR processing involved.

- `use_sim_cones_directly:=true` → bridge publishes `/planning/cones` directly
- `use_perception:=false` → boundary_extractor not launched (prevents dual publishing)
- `pose_topic:=/ground_truth/state_odom` → uses Gazebo ground truth pose

### perception mode

The full perception pipeline runs: LiDAR cone detector + YOLO vision detector → boundary extractor → path planner.

- Requires YOLO weights at `~/mfe_models/yolo/yolov8s/weights/best.pt` (or pass `vision_model_path:=<path>`)
- Camera images require X11/OpenGL — headless mode will give empty `/camera/image_raw`
- LiDAR works headless

---

## What launch_sim.sh does

`launch_sim.sh` opens a **6-pane tmux session** (`tmux attach-session -t mfe` to view):

| Pane | Name | What it runs |
|------|------|-------------|
| 0 (top-left) | EUFS Sim | `ros2 launch eufs_launcher simulation.launch.py` — Gazebo + car |
| 1 (mid-left) | MFE Bridge | `ros2 launch mfe_eufs_sim mfe_eufs_sim.launch.py` |
| 2 (bottom-left) | MFE Stack | `ros2 launch mfe_bringup bringup.launch.py` |
| 3 (top-right) | Foxglove | `ros2 launch foxglove_bridge foxglove_bridge_launch.xml` |
| 4 (mid-right) | Mission control | `ros2 service call /ros_can/set_mission ...` — press Enter to start |
| 5 (bottom-right) | Logger | `ros2 topic echo /ground_truth/state` → log file in `~/mfe_logs/` |

After launching, **press Enter in pane 4** to start the autonomous driving mission.

---

## Mission control

The EUFS state machine requires two steps to start:

1. Set `ami_state` (the event type) and `as_state=READY(1)`.
2. The car transitions to DRIVING automatically.

`launch_sim.sh` pre-fills this command in pane 4. AMI states by event:

| Event | ami_state |
|-------|-----------|
| acceleration | 11 |
| skidpad | 12 |
| autocross | 13 |

---

## Multi-lap mode

Pass a lap count as the 4th argument:

```bash
bash scripts/docker_run.sh peanut no_perception nogui 3   # 3 laps
bash scripts/docker_run.sh peanut no_perception nogui 0   # endless
```

How lap detection works (closed-loop tracks):
- Finish detector watches odometry and counts each time the car returns within **5 m** of the start position.
- Guard: the car must first travel at least **60 m** AND venture **≥15 m** from start before the first crossing counts (prevents false positives at the peanut figure-8 midpoint).
- After each non-final lap, the excursion guard resets so the next crossing must also satisfy the 15 m rule.
- After `num_laps` crossings, the detector publishes full brake and signals the EUFS state machine.

Track `/planning/laps_completed` (Int32) in Foxglove to see the current lap count.

---

## Foxglove Studio

The Foxglove bridge runs on port **8765** inside the container, sharing the host network.

1. Open [https://app.foxglove.dev](https://app.foxglove.dev)
2. Click **Open connection** → **Foxglove WebSocket**
3. URL: `ws://localhost:8765`

Useful topics to add as panels:

| Topic | Panel type | Notes |
|-------|-----------|-------|
| `/ground_truth/state_odom` | 3D or plot | Car position and speed |
| `/planning/centerline` | 3D | Path waypoints |
| `/planning/cones` | 3D | Cone map fed to planner |
| `/planning/laps_completed` | Gauge | Current lap count |
| `/camera/image_raw` | Image | Empty in headless mode |
| `/map` | 3D | SLAM occupancy grid |
| `/perception/accuracy` | Diagnostics | Cone detection accuracy |

**Camera images in headless mode**: `/camera/image_raw` is empty because Gazebo's camera plugin requires OpenGL rendering. Run with `gui` mode (or pass `DISPLAY` + `xhost +local:docker`) to get camera images.

---

## Two containers (multi-instance)

Running two sim containers simultaneously on the same host requires port isolation:

```bash
# Container 1 (default port)
GAZEBO_PORT=11355 docker run ... --name mfe-sim  ... mfe-driverless-sim bash launch_sim.sh ...

# Container 2 (default port 11350)
docker run ... --name mfe-sim-2 ... -e ROS_DOMAIN_ID=1 mfe-driverless-sim bash launch_sim.sh ...
```

`launch_sim.sh` reads `${GAZEBO_PORT:-11350}` so setting `GAZEBO_PORT` in the environment before launching is sufficient. Different `ROS_DOMAIN_ID` values prevent DDS topic bleed-through between containers.

---

## Resetting a crashed sim

```bash
# Inside the container
rm -rf ~/.gazebo/server-11350   # clear stale Gazebo lock (use your port)
pkill -f gzserver; pkill -f gzclient; pkill -f ros2
ros2 daemon stop && ros2 daemon start

# Then re-run launch_sim.sh
```

See [troubleshooting.md](troubleshooting.md) for more details on common failures.
