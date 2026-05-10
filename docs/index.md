# MFE Driverless V1 — Documentation

McGill Formula Electric autonomous racecar stack. ROS 2 Humble + Gazebo 11 + YOLO11 + ft-fsd-path-planning, running on NVIDIA Jetson Orin Nano (hardware) and RTX 5060 Ti desktop (simulation).

---

## Docs

| File | What it covers |
|------|----------------|
| [architecture.md](architecture.md) | System overview, full data-flow diagrams, TF frame chain, topic map |
| [simulation.md](simulation.md) | How to start the sim, track list, modes, Foxglove, multi-lap, mission control |
| [packages.md](packages.md) | Every ROS 2 package: purpose, nodes, topics, parameters |
| [nodes.md](nodes.md) | Every node in detail: full topic/param/algorithm reference |
| [scripts.md](scripts.md) | Every shell script: purpose, arguments, what it does |
| [docker.md](docker.md) | Docker images, Dockerfiles, multi-container setup, port isolation |
| [training.md](training.md) | YOLO training pipeline, dataset prep, GPU tuning, using weights in ROS |
| [troubleshooting.md](troubleshooting.md) | Every bug we hit, root cause, and exact fix |

---

## Quick start

```bash
# 1. Build image (once)
bash scripts/docker_build.sh

# 2. Run sim — peanut figure-8, 3 laps, headless
bash scripts/docker_run.sh peanut no_perception nogui 3

# 3. Connect Foxglove
#    Open https://app.foxglove.dev → Foxglove WebSocket → ws://localhost:8765
```

---

## Repo layout

```
MFE-Driverless-V1/
├── docs/                    ← you are here
├── scripts/                 ← orchestration scripts (see scripts.md)
├── Docker/                  ← Docker images (see docker.md)
├── ros2/src/                ← ROS 2 packages (see packages.md / nodes.md)
│   ├── mfe_bringup/         ← single launch entry point
│   ├── mfe_control/         ← pure pursuit controller
│   ├── mfe_eufs_sim/        ← Gazebo ↔ MFE bridge
│   ├── mfe_path_planning/   ← path planner + finish detector + boundary extractor
│   ├── mfe_state_estimation/← EKF (IMU + GPS)
│   ├── mfe_mapping/         ← SLAM Toolbox
│   ├── mfe_perception/      ← LiDAR detector + YOLO vision + evaluator
│   ├── mfe_msgs/            ← custom message types
│   ├── mfe_sensors/         ← sensor driver launch files
│   ├── mfe_simulation/      ← simulation-specific utilities
│   ├── mfe_base/            ← shared utilities
│   ├── eufs_msgs/           ← EUFS simulator message types
│   └── fs_msgs/             ← FSAE common messages (submodule)
└── training/yolo/           ← YOLO training pipeline (see training.md)
```
