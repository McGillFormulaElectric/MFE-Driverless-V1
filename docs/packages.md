# ROS 2 Packages

Every package in `ros2/src/`. For node-level detail see [nodes.md](nodes.md).

---

## mfe_bringup

**Purpose**: Single launch entry point for the full MFE driverless stack. Composes all nodes.

**Launch file**: `launch/bringup.launch.py`

**Key arguments**:

| Argument | Default | Description |
|----------|---------|-------------|
| `mission` | `autocross` | `autocross`, `trackdrive`, `acceleration`, `skidpad` |
| `vision_model_path` | `~/mfe_models/yolo/yolov8s/weights/best.pt` | Path to YOLO weights; empty = skip |
| `pose_topic` | `/ekf/output` | Odometry source; use `/ground_truth/state_odom` in sim |
| `use_perception` | `true` | Set `false` in no_perception sim mode |
| `endless` | `false` | Set `true` to disable finish detector |
| `num_laps` | `1` | Stop after this many lap crossings |

**Nodes launched**: lidar_cone_detector, vision_cone_detector (optional), pointcloud_to_laserscan, slam_toolbox, ekf_node, boundary_extractor (conditional), path_planner_node, pure_pursuit_node, finish_detector_node, perception_evaluator.

---

## mfe_eufs_sim

**Purpose**: Bridge between the EUFS Gazebo simulator and the MFE driverless stack. Translates topic types and message formats in both directions.

**Launch file**: `launch/mfe_eufs_sim.launch.py`

**Key parameters**:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_sim_cones_directly` | `true` | GT bypass: forward world cones to `/planning/cones` |
| `max_steering_deg` | `25.0` | Physical steering limit mapped to ±1 normalized |
| `max_speed_ms` | `10.0` | Throttle=1 maps to this speed in m/s |

**Translation table**:

| EUFS topic | Direction | MFE topic |
|-----------|-----------|-----------|
| `/velodyne_points` | → | `/lidar/points_raw` |
| `/zed/left/image_rect_color` | → | `/camera/image_raw` |
| `/ground_truth/state` | → | `/ground_truth/state_odom` |
| `/ground_truth/cones` | → | `/ground_truth/cones_colored` |
| `/ground_truth/track` | → | `/planning/cones` (no_perception only) |
| `/control/command` | → | `/cmd` (AckermannDriveStamped) |

---

## mfe_control

**Purpose**: Pure pursuit lateral controller. Follows the path centerline at a speed determined by lookahead curvature analysis.

**Node**: `pure_pursuit_node`

**Subscribes**: `/planning/centerline` (Path), `/ekf/output` (Odometry), `/planning/mission_finished` (Bool)

**Publishes**: `/control/command` (fs_msgs/ControlCommand) at 20 Hz

**Speed profile**: scans upcoming waypoints for curvature, computes corner speed target from `v = sqrt(a_lat * R)`, brakes if within stopping distance, full throttle otherwise.

---

## mfe_path_planning

**Purpose**: Three nodes — cone fusion, path generation, finish detection.

### boundary_extractor

Fuses LiDAR cone positions (`/perception/cones_uncolored`) with vision color labels (`image/track`) and publishes a colored cone map to `/planning/cones`.

### path_planner_node

Wraps the `ft-fsd-path-planning` library. Receives `/planning/cones` and car pose, runs Delaunay-based path planning, publishes `/planning/centerline` (Path) and `/planning/midpoint` (PointStamped).

- Orange cones map to `ConeTypes.UNKNOWN` (not START_FINISH_AREA) to prevent path deformation near the start gate.
- Cone visibility filtered to **20 m radius** around the car to prevent figure-8 crossing confusion.

### finish_detector_node

Three-tier detection hierarchy:
1. **Return-to-start** (closed-loop tracks): counts crossings within `return_to_start_r` metres of start; stops after `num_laps` crossings.
2. **LiDAR** (linear tracks): watches for dense returns in the forward zone once the car passes `approach_x`.
3. **Position fallback**: fires when `car_x >= finish_x` (catches LiDAR gaps from Gazebo VLP-16).

On finish: publishes full brake at 50 Hz, signals EUFS state machine, latches `/planning/mission_finished`.

---

## mfe_state_estimation

**Purpose**: Extended Kalman Filter fusing IMU and GPS into a consistent odometry estimate. Used on hardware; in simulation use `/ground_truth/state_odom` instead.

**Node**: `extended_kalman_filter_node`

**Subscribes**: `/imu` (sensor_msgs/Imu), `/gps` (sensor_msgs/NavSatFix)

**Publishes**: `/ekf/output` (nav_msgs/Odometry)

**Config file**: `config/ekf_params.yaml`

**Implementation**: `mfe_state_estimation/filters/extended_kalman_filter.py` — custom EKF with IMU prediction model in `models/imu_prediction_model.py`.

---

## mfe_mapping

**Purpose**: SLAM Toolbox lifecycle wrapper for building an occupancy grid map from LiDAR LaserScan data.

**Launch file**: `launch/slam_toolbox.launch.py`

**Subscribes**: `/lidar/pcl/laserscan` (sensor_msgs/LaserScan) via pointcloud_to_laserscan

**Publishes**: `/map` (nav_msgs/OccupancyGrid), TF `map → odom`

---

## mfe_perception

Contains three sub-packages:

### lidar_cone_detector

C++ ROS 2 node. Processes VLP-16 PointCloud2 data:
1. Voxel grid downsampling (leaf_size=0.1 m)
2. RANSAC ground plane removal (threshold=0.15 m)
3. Euclidean clustering (tolerance=0.3 m, min=1, max=50 points)
4. Cluster centroids → `/perception/cones_uncolored` (PointCloud2)
5. Objects → `/lidar/pcl/objects` (PointCloud2, for LaserScan conversion)

CUDA support is a compile-time flag (`-DPLATFORM_JETSON=ON`). Desktop builds use CPU PCL.

### vision_cone_detector

Python YOLO-based cone detector.

**Node**: `cone_detection_node`

**Subscribes**: `image/input` (Image, remapped to `/camera/image_raw`)

**Publishes**: `image/track` (mfe_msgs/Track) — detected cones with color classes

Classes: blue, yellow, orange — mapped to `Cone.BLUE`, `Cone.YELLOW`, `Cone.ORANGE_SMALL`.

### perception_evaluator

Compares `/planning/cones` against `/ground_truth/cones_colored`. Publishes accuracy metrics to `/perception/accuracy` (diagnostic_msgs/DiagnosticArray) at 1 Hz. Logs CSV to `~/mfe_logs/perception_accuracy.csv`.

---

## mfe_msgs

Custom message types:

| Message | Fields | Used for |
|---------|--------|---------|
| `mfe_msgs/Cone` | `header`, `location` (Point), `color` (uint8) | Single cone with position + color |
| `mfe_msgs/Track` | `track[]` (Cone[]) | List of cones |

Color constants on `Cone`: `BLUE=0`, `YELLOW=1`, `ORANGE_BIG=2`, `ORANGE_SMALL=3`, `UNKNOWN=4`.

---

## mfe_sensors

Hardware sensor drivers (not used in simulation). Contains ROS 2 node interfaces for:
- Velodyne VLP-16 LiDAR (`velodyne_lidar_node`)
- Intel RealSense D435 camera (`realsense_camera_node`)
- Simulation stubs (`sim_camera_node`, `sim_lidar_node`)

**Launch file**: `launch/sensors.launch.py`

---

## mfe_simulation

Alternative simulation interfaces (not the primary EUFS path):
- `carmaker_node` / `carmaker_interface` — CarMaker co-simulation bridge
- `fsds_node` (C++) — FS Driverless Sim interface

**Launch files**: `launch/sim.launch.py`, `launch/fs_driverless_sim.launch.py`

---

## mfe_base

Shared utilities:
- `utils/angle_conversions.py` — yaw ↔ quaternion helpers

---

## External packages (submodules / cloned)

| Package | Source | Purpose |
|---------|--------|---------|
| `eufs_msgs` | gitlab.com/eufs/eufs_msgs | EUFS simulator message types |
| `fs_msgs` | GitHub submodule | FSAE common messages (ControlCommand, etc.) |
| `ft-fsd-path-planning` | github.com/papalotis/ft-fsd-path-planning | Cone-based path planning library |
