# Nodes

Every node in detail: full topic/parameter/algorithm reference.

---

## mfe_eufs_sim_bridge

**Package**: `mfe_eufs_sim`  
**Executable**: `bridge_node`  
**File**: `ros2/src/mfe_eufs_sim/mfe_eufs_sim/bridge_node.py`

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map_frame` | `map` | Frame ID for published cone tracks |
| `base_frame` | `base_footprint` | Frame ID for control commands |
| `max_steering_deg` | `25.0` | Physical steering limit (degrees) |
| `max_speed_ms` | `10.0` | Throttle=1 maps to this speed (m/s) |
| `use_sim_cones_directly` | `true` | GT bypass: skip LiDAR/vision, feed cones directly |

### Subscriptions

| Topic | Type | Notes |
|-------|------|-------|
| `/velodyne_points` | PointCloud2 | BEST_EFFORT |
| `/zed/left/image_rect_color` | Image | BEST_EFFORT |
| `/ground_truth/cones` | eufs_msgs/ConeArrayWithCovariance | proximity-limited, base_footprint frame |
| `/ground_truth/track` | eufs_msgs/ConeArrayWithCovariance | ALL cones, map frame |
| `/ground_truth/state` | eufs_msgs/CarState | full vehicle state |
| `/control/command` | fs_msgs/ControlCommand | from pure pursuit / finish detector |

### Publications

| Topic | Type | Notes |
|-------|------|-------|
| `/lidar/points_raw` | PointCloud2 | remapped from velodyne_points |
| `/camera/image_raw` | Image | remapped from ZED |
| `/ground_truth/cones_colored` | mfe_msgs/Track | all colors, for evaluator |
| `/ground_truth/track_colored` | mfe_msgs/Track | full track, for visualization |
| `/ground_truth/state_odom` | nav_msgs/Odometry | converted CarState |
| `/planning/cones` | mfe_msgs/Track | GT bypass only; BLUE+YELLOW, ORANGE filtered |
| `/cmd` | AckermannDriveStamped | to EUFS sim |

### Control conversion

```
steering_rad  = msg.steering * (max_steering_deg × π/180)
speed_ms      = max(0, (msg.throttle - msg.brake) × max_speed_ms)
```

No reverse: speed clamped to ≥ 0.

---

## lidar_perception_node

**Package**: `lidar_cone_detector`  
**Executable**: `lidar_perception_node`  
**File**: `ros2/src/mfe_perception/lidar_cone_detector/src/lidar_perception_node.cpp`

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `leaf_size` | `0.1` | Voxel grid leaf size (m) |
| `ground_threshold` | `0.15` | RANSAC inlier threshold for ground removal (m) |
| `cluster_tolerance` | `0.3` | Euclidean clustering max point distance (m) |
| `min_cluster_size` | `1` | Min points per cluster (VLP-16 gets 1-2 pts/cone at 10 m) |
| `max_cluster_size` | `50` | Max points per cluster |

### Algorithm

1. Voxel grid downsampling
2. RANSAC ground plane removal
3. Euclidean cluster extraction
4. Compute cluster centroids → PointCloud2

### Subscriptions / Publications

| Topic | Type |
|-------|------|
| `/lidar/points_raw` | PointCloud2 (in) |
| `/perception/cones_uncolored` | PointCloud2 (out, centroids) |
| `/lidar/pcl/objects` | PointCloud2 (out, for LaserScan) |

**CUDA note**: `#ifdef PLATFORM_JETSON` compile-time flag enables CUDA-PCL. x86_64 desktop always uses CPU PCL.

---

## cone_detection_node (vision)

**Package**: `vision_cone_detector`  
**Executable**: `cone_detection_node`  
**File**: `ros2/src/mfe_perception/vision_cone_detector/vision_cone_detector/cone_detection_node.py`

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_path` | (required) | Absolute path to YOLO `.pt` weights |
| `depth_callback` | `false` | Enable depth-fused 3D positioning |

### Subscriptions / Publications

| Topic | Type |
|-------|------|
| `image/input` | Image (remapped to `/camera/image_raw`) |
| `image/track` | mfe_msgs/Track (out) |

Skipped at launch (returns empty list) if `model_path` is empty — YOLO instantiation is in `__init__` and would crash without valid weights.

---

## boundary_extractor

**Package**: `mfe_path_planning`  
**Executable**: `boundary_extractor`  
**File**: `ros2/src/mfe_path_planning/mfe_path_planning/boundary_extractor.py`

Fuses LiDAR positions with camera color labels to publish a fully-colored cone map.

### Subscriptions

| Topic | Type |
|-------|------|
| `/perception/cones_uncolored` | PointCloud2 (LiDAR centroids) |
| `image/track` | mfe_msgs/Track (vision colors) |

### Publications

| Topic | Type |
|-------|------|
| `/planning/cones` | mfe_msgs/Track |

Not launched when `use_perception:=false` (no_perception mode).

---

## path_planner_node

**Package**: `mfe_path_planning`  
**Executable**: `path_planner_node`  
**File**: `ros2/src/mfe_path_planning/mfe_path_planning/path_planner_node.py`

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mission` | `autocross` | Mission type passed to ft-fsd-path-planning |

### Subscriptions

| Topic | Type |
|-------|------|
| `/planning/cones` | mfe_msgs/Track |
| `/ekf/output` | nav_msgs/Odometry (remappable via `pose_topic`) |

### Publications

| Topic | Type |
|-------|------|
| `/planning/centerline` | nav_msgs/Path |
| `/planning/midpoint` | geometry_msgs/PointStamped |

### Algorithm notes

- Uses `ft-fsd-path-planning` (Delaunay triangulation of cone pairs, midpoints form centerline).
- Orange cones → `ConeTypes.UNKNOWN` (NOT `START_FINISH_AREA`) to prevent path deformation near start gate.
- Cone visibility filter: only cones within **20 m** of car position are passed to planner — prevents figure-8 crossing confusion.

---

## pure_pursuit_node

**Package**: `mfe_control`  
**Executable**: `pure_pursuit_node`  
**File**: `ros2/src/mfe_control/mfe_control/pure_pursuit_node.py`

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `lookahead_distance` | `5.0` | Pure pursuit lookahead (m) |
| `max_speed` | `10.0` | Maximum speed (m/s) |
| `wheelbase` | `1.5` | Vehicle wheelbase (m) |
| `max_steering_deg` | `25.0` | Physical steering limit |
| `max_lateral_accel` | `8.0` | Corner speed target: v = sqrt(a_lat × R) |
| `max_deceleration` | `10.0` | Braking deceleration (m/s²) |
| `lookahead_waypoints` | `40` | Waypoints ahead to scan for corners |
| `speed_reduction_factor` | `0.3` | Steering-proportional fallback before first odometry |

### Mission-tuned defaults (set in bringup.launch.py)

| Mission | max_speed | lookahead | speed_reduction_factor |
|---------|-----------|-----------|----------------------|
| autocross | 7.0 m/s | 3.5 m | 0.3 |
| acceleration | 13.0 m/s | 8.0 m | 0.2 |
| skidpad | 4.5 m/s | 3.0 m | 0.6 |

### Subscriptions

| Topic | Type |
|-------|------|
| `/planning/centerline` | nav_msgs/Path |
| `/ekf/output` | nav_msgs/Odometry (remappable) |
| `/planning/mission_finished` | std_msgs/Bool |

### Publications

| Topic | Type | Rate |
|-------|------|------|
| `/control/command` | fs_msgs/ControlCommand | 20 Hz |

### Velocity profile algorithm

1. Find closest waypoint to car position.
2. Scan next `lookahead_waypoints` points, compute Menger curvature at each triple.
3. For each corner: `v_corner = sqrt(max_lateral_accel × R)`.
4. Find the tightest (lowest `v_corner`) corner and its distance `d`.
5. Compute braking distance: `d_brake = (v_now² - v_target²) / (2 × max_deceleration)`.
6. If `v_now > v_target AND d <= d_brake + 1 m` → apply proportional brake.
7. Otherwise → full throttle.

---

## finish_detector_node

**Package**: `mfe_path_planning`  
**Executable**: `finish_detector_node`  
**File**: `ros2/src/mfe_path_planning/mfe_path_planning/finish_detector_node.py`

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `approach_x` | `60.0` | x-position where LiDAR watch activates (linear tracks) |
| `finish_x` | `78.0` | x-position fallback trigger |
| `min_travel_m` | `0.0` | Min cumulative travel before first lap crossing counts |
| `return_to_start_r` | `0.0` | Lap gate radius (m); >0 enables return-to-start logic |
| `detect_fwd_min_m` | `1.5` | Min distance ahead for LiDAR finish cone detection |
| `detect_fwd_max_m` | `15.0` | Max distance ahead |
| `detect_lateral_m` | `4.0` | Lateral width for LiDAR detection zone |
| `min_finish_points` | `1` | Min LiDAR points in zone to trigger |
| `num_laps` | `1` | Stop after this many lap crossings (autocross/trackdrive) |

### Autocross/trackdrive bringup defaults

`min_travel_m=60.0`, `return_to_start_r=5.0`, `num_laps=<from launch arg>`

### Subscriptions

| Topic | Type |
|-------|------|
| `/lidar/points_raw` | PointCloud2 |
| `/ekf/output` | nav_msgs/Odometry (remappable) |

### Publications

| Topic | Type | Notes |
|-------|------|-------|
| `/ros_can/mission_completed` | std_msgs/Bool | EUFS state machine |
| `/planning/mission_finished` | std_msgs/Bool | latched (TRANSIENT_LOCAL) |
| `/control/command` | fs_msgs/ControlCommand | full brake at 50 Hz after finish |
| `/planning/laps_completed` | std_msgs/Int32 | current lap count for Foxglove |

---

## extended_kalman_filter_node

**Package**: `mfe_state_estimation`  
**File**: `ros2/src/mfe_state_estimation/mfe_state_estimation/extended_kalman_filter_node.py`

Fuses IMU and GPS into odometry. Used on hardware. In simulation, use `/ground_truth/state_odom` instead.

### Subscriptions

| Topic | Type |
|-------|------|
| `/imu` | sensor_msgs/Imu (remapped from `/imu/data`) |
| `/gps` | sensor_msgs/NavSatFix |

### Publications

| Topic | Type |
|-------|------|
| `/ekf/output` | nav_msgs/Odometry |

### Config

`ros2/src/mfe_state_estimation/config/ekf_params.yaml`

---

## perception_evaluator

**Package**: `perception_evaluator`  
**File**: `ros2/src/mfe_perception/perception_evaluator/perception_evaluator/evaluator_node.py`

Compares detected cones against ground truth for accuracy measurement. Sim-only.

### Subscriptions

| Topic | Type |
|-------|------|
| `/planning/cones` | mfe_msgs/Track |
| `/ground_truth/cones_colored` | mfe_msgs/Track |

### Publications

| Topic | Type | Rate |
|-------|------|------|
| `/perception/accuracy` | diagnostic_msgs/DiagnosticArray | 1 Hz |

Logs to `~/mfe_logs/perception_accuracy.csv`.
