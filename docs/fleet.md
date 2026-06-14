# Fleet Deployment — Multi-Machine Launch

How to split the driverless stack across two or more machines using `fleet.launch.py`.

---

## Architecture overview

```
┌─────────────────────────────┐       Tailscale VPN        ┌──────────────────────────────┐
│         Host PC / Jetson 1  │ ─────────────────────────▶ │           Jetson 2            │
│         (Perception)        │                             │           (Compute)           │
│                             │   /planning/cones           │                               │
│  lidar_cone_detector        │ ─────────────────────────▶ │  path_planner_node            │
│  vision_cone_detector       │                             │  pure_pursuit_node            │
│  boundary_extractor         │   /ekf/output               │  finish_detector_node         │
│  slam_toolbox               │ ─────────────────────────▶ │                               │
│  ekf_node                   │                             │   ▼                           │
│                             │        /cmd ◀───────────── │  /cmd (AckermannDriveStamped) │
└─────────────────────────────┘                             └──────────────────────────────┘
```

Only processed topics cross the network link — raw LiDAR / camera data stays local on the perception machine.

---

## Deployment modes

| `local_role` | What runs locally | What runs remotely |
|-------------|-------------------|--------------------|
| `perception` (default) | Perception nodes | Compute on `compute_host` via SSH |
| `compute` | Compute nodes | Perception on `perception_host` via SSH |
| `both` | Everything | Nothing (single machine) |
| `none` | Nothing | Both roles on remote Jetsons |

---

## Quick start

### Sim on host PC — compute on Jetson 2

```bash
# Runs inside the Docker container (source ROS env first if not using Docker)
ros2 launch mfe_bringup fleet.launch.py \
  pose_topic:=/ground_truth/state_odom \
  use_slam:=false \
  use_ekf:=false
```

This runs perception locally and SSHes to `mfe@100.119.35.6` (Jetson 2) to start compute.

### Real car — Jetson 1 (perception) + Jetson 2 (compute), host as SSH launcher

```bash
ros2 launch mfe_bringup fleet.launch.py \
  local_role:=none \
  perception_host:=mfe@100.84.7.80 \
  compute_host:=mfe@100.119.35.6 \
  mission:=trackdrive
```

### All local (development / testing)

```bash
ros2 launch mfe_bringup fleet.launch.py local_role:=both
```

---

## Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `local_role` | `perception` | What runs on this machine |
| `compute_host` | `mfe@100.119.35.6` | SSH target for Jetson 2 |
| `perception_host` | (empty) | SSH target for Jetson 1; empty = skip |
| `mission` | `autocross` | FSAE mission forwarded to all machines |
| `pose_topic` | `/ekf/output` | Odometry topic forwarded to all machines |
| `use_slam` | `true` | Enable SLAM toolbox on perception machine |
| `use_ekf` | `true` | Enable EKF on perception machine |
| `endless` | `false` | Disable finish detector |
| `vision_model_path` | `~/mfe_models/yolo/yolo11s/weights/best.pt` | YOLO weights on perception machine |

---

## Self-bootstrapping remote launch

`fleet.launch.py` automatically prepares any remote Jetson on first run:

1. **Clone** — if `~/Develop/MFE-Driverless-V1` is not present, clones from GitHub.
2. **Pull** — if the repo exists, runs `git pull --ff-only` to update.
3. **Build** — if `mfe_bringup` is not installed (checked via marker file), runs `setup_jetson.sh`.
4. **Launch** — sources the workspace and launches the appropriate launch file.

On subsequent runs steps 1–3 are skipped. Total cold-start time: ~5 minutes (dominated by `colcon build`).

### Remote launch files used

| Role | Launch file on remote |
|------|----------------------|
| Compute | `compute.launch.py` |
| Perception | `perception.launch.py` |

---

## SSH prerequisites

```bash
# 1. Generate key pair on host (once)
ssh-keygen -t ed25519

# 2. Copy key to each Jetson (use IP — MagicDNS cross-org names don't resolve)
ssh-copy-id mfe@100.119.35.6   # Jetson 2
ssh-copy-id mfe@100.84.7.80    # Jetson 1

# 3. Test
ssh mfe@100.119.35.6 echo "OK"
```

fleet.launch.py passes `-o StrictHostKeyChecking=no` to SSH so the first connection doesn't stall on the host-key prompt.

---

## Thin wrappers: perception.launch.py and compute.launch.py

These files exist so remote Jetsons can be launched without knowing `fleet.launch.py` internals:

```bash
# Jetson 1 — perception only
ros2 launch mfe_bringup perception.launch.py mission:=trackdrive

# Jetson 2 — compute only
ros2 launch mfe_bringup compute.launch.py mission:=trackdrive
```

Both are thin wrappers around `bringup.launch.py` that set `run_perception` / `run_compute` appropriately.

---

## Troubleshooting

**Nodes on Jetson not visible from host**
→ See [[networking]] — DDS multicast doesn't work over Tailscale. Ensure `CYCLONEDDS_URI` is set and the XML file has the host IP as a peer.

**SSH launch hangs at "Building workspace"**
→ First-time build takes ~5 min. Check progress by SSH-ing in and running `htop`.

**`git pull` fails on Jetson**
→ SSH key not added for GitHub. Either use HTTPS or add the Jetson's SSH key to GitHub.

**Remote compute node starts but doesn't receive cones**
→ Check that `/planning/cones` is published on the perception machine (`ros2 topic hz /planning/cones`). Verify Tailscale is up on both machines.
