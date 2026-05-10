# Architecture

McGill Formula Electric Driverless V1 — system overview, data-flow, TF frames, and topic map.

---

## System diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                     GAZEBO 11 (sim) / hardware                  │
│  VLP-16 LiDAR ──→ /velodyne_points                             │
│  ZED camera  ──→ /zed/left/image_rect_color                    │
│  Car state   ──→ /ground_truth/state   (EUFS CarState)         │
│  Track cones ──→ /ground_truth/track   (ALL cones, map frame)  │
└────────────────────────┬────────────────────────────────────────┘
                         │
                ┌────────▼──────────┐
                │  mfe_eufs_sim     │  bridge_node
                │  (bridge)         │
                └──┬──────────┬─────┘
                   │          │
    /lidar/points_raw    /camera/image_raw
    /ground_truth/state_odom
    /planning/cones  (no_perception mode only)
                   │          │
     ┌─────────────▼──┐   ┌───▼──────────────────┐
     │ lidar_cone_     │   │ vision_cone_detector  │
     │ detector (C++)  │   │ (YOLO11 Python)       │
     └────────┬────────┘   └──────────┬────────────┘
              │                       │
    /perception/cones_uncolored  image/track
              │                       │
              └─────────┬─────────────┘
                        │
               ┌────────▼────────────┐
               │ boundary_extractor  │  (LiDAR + camera fusion)
               │ (mfe_path_planning) │
               └────────┬────────────┘
                        │
                /planning/cones  (mfe_msgs/Track, map frame)
                        │
               ┌────────▼────────────┐
               │  path_planner_node  │  (ft-fsd-path-planning)
               └────────┬────────────┘
                        │
               /planning/centerline  (nav_msgs/Path)
                        │
               ┌────────▼────────────┐
               │  pure_pursuit_node  │  20 Hz control loop
               └────────┬────────────┘
                        │
               /control/command  (fs_msgs/ControlCommand)
                        │
               ┌────────▼────────────┐
               │  mfe_eufs_sim       │  → /cmd (AckermannDriveStamped)
               │  bridge (→ EUFS)    │
               └─────────────────────┘
```

---

## No-perception shortcut (sim only)

In `no_perception` mode the bridge skips the entire perception pipeline:

```
/ground_truth/track (ALL cones, map frame)
        │
        │  bridge: BLUE + YELLOW only, ORANGE filtered
        ▼
/planning/cones  ──→  path_planner_node
```

`boundary_extractor` is not launched (`use_perception:=false` in bringup).

---

## TF frame chain

```
map
 └── odom
      └── base_footprint
           ├── velodyne        (VLP-16 mount point)
           └── zed_camera_center
```

- In simulation: `publish_gt_tf:=true` in `eufs_launcher` makes Gazebo publish this chain from ground truth.
- In hardware: the EKF node and static TF publishers own the chain.
- **Critical**: if `publish_gt_tf:=false`, the car has no pose in the map frame and drives in circles.

---

## Topic map

| Topic | Type | Producer | Consumer(s) |
|-------|------|----------|-------------|
| `/velodyne_points` | PointCloud2 | EUFS sim | bridge |
| `/lidar/points_raw` | PointCloud2 | bridge | lidar_cone_detector, finish_detector |
| `/camera/image_raw` | Image | bridge | vision_cone_detector |
| `/ground_truth/track` | eufs_msgs/ConeArrayWithCovariance | EUFS sim | bridge |
| `/ground_truth/cones` | eufs_msgs/ConeArrayWithCovariance | EUFS sim | bridge |
| `/ground_truth/state` | eufs_msgs/CarState | EUFS sim | bridge |
| `/ground_truth/state_odom` | nav_msgs/Odometry | bridge | path_planner (sim), finish_detector (sim) |
| `/ground_truth/cones_colored` | mfe_msgs/Track | bridge | perception_evaluator |
| `/planning/cones` | mfe_msgs/Track | bridge (no_perception) / boundary_extractor | path_planner |
| `/perception/cones_uncolored` | PointCloud2 | lidar_cone_detector | boundary_extractor |
| `image/track` | mfe_msgs/Track | vision_cone_detector | boundary_extractor |
| `/planning/centerline` | nav_msgs/Path | path_planner_node | pure_pursuit_node |
| `/control/command` | fs_msgs/ControlCommand | pure_pursuit / finish_detector | bridge → /cmd |
| `/cmd` | AckermannDriveStamped | bridge | EUFS sim |
| `/ekf/output` | nav_msgs/Odometry | ekf_node (hardware) | path_planner, pure_pursuit |
| `/planning/mission_finished` | std_msgs/Bool | finish_detector | pure_pursuit |
| `/ros_can/mission_completed` | std_msgs/Bool | finish_detector | EUFS state machine |
| `/planning/laps_completed` | std_msgs/Int32 | finish_detector | Foxglove dashboard |
| `/lidar/pcl/laserscan` | sensor_msgs/LaserScan | pointcloud_to_laserscan | slam_toolbox |
| `/map` | nav_msgs/OccupancyGrid | slam_toolbox | (visualization) |
| `/perception/accuracy` | diagnostic_msgs/DiagnosticArray | perception_evaluator | Foxglove |

---

## Package dependency graph

```
mfe_bringup
├── mfe_perception/lidar_cone_detector
├── mfe_perception/vision_cone_detector
├── mfe_path_planning
│   ├── boundary_extractor
│   ├── path_planner_node  (depends on ft-fsd-path-planning)
│   └── finish_detector_node
├── mfe_control/pure_pursuit_node
├── mfe_state_estimation/ekf_node
├── mfe_mapping/slam_toolbox
└── mfe_perception/perception_evaluator

mfe_eufs_sim (bridge) — launched separately by launch_sim.sh
    depends on: eufs_msgs, mfe_msgs, fs_msgs, ackermann_msgs
```

---

## Mission flow

1. `launch_sim.sh` starts Gazebo with the EUFS stack.
2. Waits 15 s for Gazebo to initialize and the car to spawn.
3. Starts the MFE bridge (`mfe_eufs_sim`), MFE stack (`mfe_bringup`), and Foxglove bridge.
4. Operator presses Enter in the mission control pane to send `SetCanState` → `READY(1)`.
5. The EUFS state machine transitions to DRIVING; the car begins publishing odometry.
6. Path planner receives cones, computes centerline, publishes Path.
7. Pure pursuit follows the path at the configured speed.
8. Finish detector counts laps (or watches x-position for linear tracks) and triggers full brake.
