# Stack interfaces

How the stack fits together, the contract a SLAM implementation must meet, how to select one, and which topic goes where.

Paths are relative to `ros2/src/` unless they start with `docs/` or `scripts/`.

---

## Stack

```mermaid
flowchart LR
  subgraph Sensors
    L["LiDAR /lidar/points_raw"]
    IMU["IMU and GNSS /imu /gps"]
    CAM["Camera /camera/image_raw"]
    SO["Sim odometry /sim/xsens/state_odom"]
  end
  subgraph Perception
    LP["lidar_perception_node"]
    YOLO["cone_detection_node (optional)"]
    BE["boundary_extractor"]
  end
  subgraph SLAM["SLAM (slam:= selects one)"]
    TB["slam_toolbox: map to odom TF"]
    GS["slam_node GTSAM: /slam/odom and cone map"]
  end
  subgraph Localisation
    EKF["ekf_node: /ekf/output"]
  end
  subgraph Planning
    PP["path_planner_node"]
    FD["finish_detector_node"]
  end
  subgraph Control
    PU["pure_pursuit_node"]
    SV["supervisor_node"]
  end
  V["Vehicle: /control/command to ACU"]

  L --> LP
  LP -->|"/perception/cones_uncolored"| GS
  LP -->|"/perception/cones_uncolored"| BE
  L --> TB
  CAM --> YOLO -->|"/image/track"| BE
  IMU --> EKF
  SO -.->|"sim odom_topic"| GS
  EKF -.->|"ego velocity"| LP
  EKF -->|"pose_topic (slam:=toolbox, none)"| PP
  GS -->|"/slam/odom (slam:=gtsam)"| PP
  BE -->|"/planning/cones"| PP
  GS -->|"/planning/cones (slam:=gtsam)"| PP
  PP --> PU --> V
  FD --> V
  SV --> V
```

Planner, pure pursuit and finish detector take their pose from `pose_topic` (bringup argument). `slam:=gtsam` overrides it with `/slam/odom`.

---

## SLAM contract

A SLAM implementation is a package with a `launch/slam.launch.py` that starts its nodes. Bringup includes that file by package name (`mfe_mapping` for `toolbox`, `mfe_validation` for `gtsam`). The contract is also written as constants in `mfe_common/mfe_common/topics.py`.

| Direction | Topic | Type | Frame | Notes |
|---|---|---|---|---|
| In | `odom_topic` (parameter) | `nav_msgs/Odometry` | any | Odometry the implementation integrates. Default `/sim/xsens/state_odom` in `mfe_validation`. Not used by bringup, which passes no remap. |
| In | `/perception/cones_uncolored` (`PERCEPTION_CONES_UNCOLORED`) | `sensor_msgs/PointCloud2` | sensor | LiDAR cone centroids from `lidar_perception_node`. Fixed name. |
| In (optional) | camera cones (`/image/cones`, `IMAGE_CONES`) | `mfe_msgs/Cone` | camera | Only for camera colour modes. |
| Out | `/slam/odom` (`SLAM_ODOM`) | `nav_msgs/Odometry` | `map`, child `base_footprint` | Start-relative pose. Required for `slam:=gtsam` to drive the planner. |
| Out | cone map on `cones_topic` (parameter; `SLAM_CONE_MAP` = `/planning/cones` in bringup) | `mfe_msgs/Track` | `map` | Confirmed landmarks. The planner reads `/planning/cones`. |
| Out (optional) | TF `map` to `odom` | TF | | Only if the implementation provides it. `slam_toolbox` does; `gtsam` does not. |

Implementations must not publish to `/planning/cones` while `boundary_extractor` runs. Bringup disables `boundary_extractor` for `slam:=gtsam` for this reason.

---

## Selecting SLAM

Bringup has one argument:

```bash
ros2 launch mfe_bringup bringup.launch.py slam:=toolbox   # default
ros2 launch mfe_bringup bringup.launch.py slam:=gtsam use_ekf:=false   # sim
ros2 launch mfe_bringup bringup.launch.py slam:=none
```

| `slam` | Starts | Pose topic for planner, pure pursuit, finish detector | `boundary_extractor` |
|---|---|---|---|
| `toolbox` (default) | `mfe_mapping/launch/slam.launch.py`: pointcloud_to_laserscan and slam_toolbox, only when `use_slam:=true` | `pose_topic` | on, if `use_perception:=true` |
| `gtsam` | `mfe_validation/launch/slam.launch.py`: `slam_node`, with the arguments below. Runs with `run_perception:=true` or `false` | `/slam/odom` (`pose_topic` ignored) | off |
| `none` | nothing | `pose_topic` | on, if `use_perception:=true` |

`use_slam` and `pose_topic` keep their meaning. `use_slam` only applies to `toolbox`.

### Arguments for `slam:=gtsam`

These pass through to `mfe_validation/launch/slam.launch.py`. Names and defaults are those of `lidar_only.launch.py`, so the same values give the same `slam_node` parameters. They are ignored by the other choices.

| Argument | Default | `slam_node` parameter |
|---|---|---|
| `slam_lateral_vel` | `false` | `use_lateral_vel` (integrate INS lateral velocity) |
| `color_mode` | `geometric` | `color_mode` |
| `min_obs` | `2` | `min_obs_publish` |
| `max_range` | `20.0` | `max_range` |
| `publish_hz` | `5.0` | `publish_hz` |
| `freeze` | `true` | `freeze` |
| `odom_topic` | `/sim/xsens/state_odom` | `odom_topic` (odometry for yaw and speed) |

`slam_lateral_vel` keeps lidar_only's name. The other names have no `slam_` prefix because bringup has no argument they would clash with. The cone topic is fixed to `/planning/cones` and the orange topic to `/planning/orange_cones`, as in the bringup route.

With `slam:=gtsam` the LiDAR detector is not started. Under bringup it comes from the perception group (`run_perception:=true`, bringup's detector parameters). The harness instead starts it from `lidar_only.launch.py`, with its own parameters, so `run_perception:=false` is the usual setting.

### Fleet

`fleet.launch.py` takes `slam` and forwards it to the local bringup and to the remote compute and perception launches, like `pose_topic`. The `slam_*` tuning arguments above are not forwarded, so remote `gtsam` nodes use the defaults.

### Steps to add a SLAM implementation

1. Create a package with `launch/slam.launch.py`. It starts every node the implementation needs, except the LiDAR detector (bringup owns it).
2. Meet the contract above. If the implementation publishes `/slam/odom`, the pose route must point there. If it publishes the cone map on `/planning/cones`, the boundary extractor must be off.
3. In `mfe_bringup/launch/bringup.launch.py`, add the name to the `slam` choices, add a branch in `_make_slam`, and add it to `_pose_topic` and `_make_boundary_extractor` if it publishes odometry or the cone map.
4. Add the package to `mfe_bringup/package.xml` as an `exec_depend`.
5. Test with `ros2 launch mfe_bringup bringup.launch.py slam:=<name> --print` and by starting the stack in the simulator.

---

## Topic table

Publisher and subscriber names are node names. Parameter-driven names are shown with the parameter in brackets.

| Topic | Type | Publisher(s) | Subscriber(s) |
|---|---|---|---|
| `/velodyne_points` | `sensor_msgs/PointCloud2` | EUFS sim (external) | `bridge_node`; `lidar_pipeline` remap target (navigation only) |
| `/lidar/points_raw` | `sensor_msgs/PointCloud2` | `bridge_node` (sim). No hardware publisher in the repo | `lidar_perception_node`; `supervisor_node`; `finish_detector_node`; `pointcloud_to_laserscan` (remap `cloud_in`); MCAP recorder |
| `/lidar/pcl/laserscan` | `sensor_msgs/LaserScan` | `pointcloud_to_laserscan` (external) | `slam_toolbox` (external) |
| `/imu` | `sensor_msgs/Imu` | Xsens MTi driver (external) | `ekf_node` |
| `/gps` | `sensor_msgs/NavSatFix` | Xsens MTi driver (external) | `ekf_node`; `supervisor_node`; MCAP recorder |
| `/ground_truth/state` | `eufs_msgs/CarState` | EUFS (external) | `bridge_node`; `lap_validator_node` |
| `/ground_truth/state_odom` | `nav_msgs/Odometry` | `bridge_node` | `xsens_noise_node`; `ekf_node` (wheelspeed default); `slam_node` [`ground_truth_odom_topic`]; `compare_node` |
| `/ground_truth/cones` | `eufs_msgs/ConeArrayWithCovariance` | EUFS (external) | `bridge_node` |
| `/ground_truth/track` | `eufs_msgs/ConeArrayWithCovariance` | EUFS (external) | `bridge_node` |
| `/ground_truth/cones_colored` | `mfe_msgs/Track` | `bridge_node` | `perception_evaluator` (evaluator); `fusion_evaluator` |
| `/ground_truth/track_colored` | `mfe_msgs/Track` | `bridge_node` | `boundary_extractor`; `slam_node` [`ground_truth_track_topic`, `color_mode=gt`]; `lap_validator_node` |
| `/d435i/color/image_raw`, `/zed/left/image_rect_color` | `sensor_msgs/Image` | EUFS (external) | `bridge_node` |
| `/camera/image_raw` | `sensor_msgs/Image` | `bridge_node` | `cone_detection_node` (remap `image/input`) |
| `/camera/image/raw` | `sensor_msgs/Image` | `file_loader` (vision, `load_file` only); `carmaker_node` (not launched) | none |
| `camera/camera_info` (relative) | `sensor_msgs/CameraInfo` | none in repo | `cone_detection_node`; `fusion_evaluator` |
| `image/input` (relative) | `sensor_msgs/Image` | see `/camera/image_raw` | `cone_detection_node` |
| `image/depth` (relative) | `sensor_msgs/Image` | none in repo | `cone_detection_node` (only with `depth_callback`) |
| `/image/cones` | `mfe_msgs/Cone` | `cone_detection_node` | `slam_node` (camera colour modes) |
| `/image/track` | `mfe_msgs/Track` | `cone_detection_node` | `boundary_extractor`; `fusion_evaluator` |
| `image/cones_image`, `image/cone_centres` | `sensor_msgs/Image`, `PointCloud2` | `cone_detection_node` | none |
| `/lidar/pcl/input` | `sensor_msgs/PointCloud2` | `file_loader` (C++) | `ground_plane_removal` |
| `/lidar/pcl/raw`, `/lidar/pcl/acc_cloud` | `sensor_msgs/PointCloud2` | none (raw); `lidar_preprocessor` (acc) | `lidar_preprocessor` (raw); none (acc) |
| `/lidar/pcl/ground`, `/lidar/pcl/objects` | `sensor_msgs/PointCloud2` | `ground_plane_removal` | none |
| `/sim/lidar` | `sensor_msgs/PointCloud2` | FSDS (external) | `sim_lidar_node` (not launched) |
| `/mfe_sensors/lidar/data` | `sensor_msgs/PointCloud2` | `sim_lidar_node` (not launched) | none |
| `/perception/cones_uncolored` | `sensor_msgs/PointCloud2` | `lidar_perception_node` | `path_planner_node`; `boundary_extractor`; `slam_node`; `fusion_evaluator` |
| `/perception/cones_markers` | `visualization_msgs/MarkerArray` | `lidar_perception_node` | none (RViz) |
| `/debug/gpu_objects` | `sensor_msgs/PointCloud2` | `lidar_perception_node` | none |
| `/ekf/output` | `nav_msgs/Odometry` | `ekf_node` | `path_planner_node`, `pure_pursuit_node`, `finish_detector_node` (remapped to `pose_topic`); `supervisor_node` and `lidar_perception_node` (`odom_topic`, default, not remapped); MCAP recorder |
| `/sim/xsens/state_odom` | `nav_msgs/Odometry` | `xsens_noise_node` | `slam_node` [`odom_topic`, default]; planner, pure pursuit, finish detector when passed as `pose_topic` |
| `/slam/odom` | `nav_msgs/Odometry` | `slam_node` (`slam:=gtsam`) | `path_planner_node`, `pure_pursuit_node`, `finish_detector_node` (when `slam:=gtsam`); `skidpad_planner`; `compare_node`; validation planner |
| `/planning/cones` | `mfe_msgs/Track` | `boundary_extractor` (`slam:=toolbox`, `none`); `slam_node` (`slam:=gtsam`); `bridge_node` (`use_sim_cones_directly`) | `path_planner_node`; `finish_detector_node` (gate and line modes); `supervisor_node`; `evaluator_node` |
| `/validation/cones` | `mfe_msgs/Track` | `slam_node` (validation chain, default `cones_topic`) | `path_planner_node` (remapped in validation) |
| `/planning/orange_cones` | `mfe_msgs/Track` | `slam_node` (`orange_only`, `yolo_lidar`) | `finish_detector_node` (when `orange_topic` is set to it) |
| `/planning/cones_camera_colours` | `mfe_msgs/Track` | `slam_node` | none (display) |
| `/validation/centerline` | `nav_msgs/Path` | `path_planner_node` (validation remap) | `compare_node` |
| `/planning/centerline` | `nav_msgs/Path` | `path_planner_node`; `skidpad_planner` | `pure_pursuit_node`; `lap_validator_node`; `compare_node` |
| `/planning/track_left`, `/planning/track_right` | `nav_msgs/Path` | `path_planner_node` | none |
| `/planning/path_horizon` | `std_msgs/Float32` | `path_planner_node` | `pure_pursuit_node` |
| `/planning/target_speeds` | `std_msgs/Float64MultiArray` | `path_planner_node` | `pure_pursuit_node` |
| `/planning/laps_completed` | `std_msgs/Int32` | `finish_detector_node` | `path_planner_node` |
| `/planning/mission_finished` | `std_msgs/Bool` (latched) | `finish_detector_node` | `pure_pursuit_node`; `lap_validator_node` |
| `/ros_can/mission_completed` | `std_msgs/Bool` | `finish_detector_node` | none in repo (external CAN bridge) |
| `/control/command` | `fs_msgs/ControlCommand` | `pure_pursuit_node`; `finish_detector_node` (full brake, 50 Hz) | `supervisor_node`; `bridge_node`; MCAP recorder |
| `/control/command_mpc` | `fs_msgs/ControlCommand` | `mpc_lateral_node` (not launched) | none |
| `/mfe/emergency_brake` | `std_msgs/Bool` | `supervisor_node` | none in repo (external ACU) |
| `/as/supervisor/status` | `std_msgs/String` | `supervisor_node` | none in repo (external) |
| `/cmd` | `ackermann_msgs/AckermannDriveStamped` | `bridge_node` | EUFS (external) |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` (external) | none in repo (TF `map` to `odom` is used instead) |
| `/skidpad/cones` | `mfe_msgs/Track` | none in repo | `skidpad_planner`; `finish_detector_node` (gate mode) |
| `/perception/accuracy` | `diagnostic_msgs/DiagnosticArray` | `evaluator_node` | none |
| `/perception/fusion_accuracy` | `diagnostic_msgs/DiagnosticArray` | `fusion_evaluator_node` | none |

### Transforms

| Transform | Producer | Consumer |
|---|---|---|
| `base_footprint` to `velodyne`, `base_footprint` to `zed_camera_center` | `mfe_eufs_sim` launch (sim) | `boundary_extractor`, `fusion_evaluator` |
| `map` to `odom` | `slam_toolbox` (`slam:=toolbox`) or EUFS ground truth (sim with `use_slam:=false`) | `boundary_extractor`, `evaluator_node` |
| `map` to `odom` | none under `slam:=gtsam` | none |

## TF tree

`map`, `odom` and `base_footprint` follow REP 105. The stack looks up `map` from the LiDAR frame
(`velodyne`) and from the camera frame, and `base_footprint` from the pose.

### Sim (`hardware:=false`, default)

| Transform | Published by |
|---|---|
| `base_footprint` to `velodyne` (0, 0, 0.3) | `mfe_eufs_sim.launch.py` static TF |
| `base_footprint` to `zed_camera_center` (0.3, 0, 0.5) | `mfe_eufs_sim.launch.py` static TF |
| `map` to `odom` | EUFS ground truth (`use_slam:=false`), or `slam_toolbox` (`slam:=toolbox`) |
| `odom` to `base_footprint` | EUFS ground truth |

The cone detector labels its cones with `camera_frame` `camera_base` (its default). Sim publishes
`zed_camera_center`, not `camera_base`, so those cones have no TF in sim.

### Hardware (`hardware:=true`)

Set on the perception machine (`perception.launch.py hardware:=true` or
`bringup.launch.py hardware:=true`). Nothing else in the repo publishes these transforms.

| Transform | Published by | Notes |
|---|---|---|
| `base_footprint` to `velodyne` | static TF, `bringup.launch.py` | Mounts from `mfe_bringup/config/sensor_mounts.yaml`. Sim values until measured. |
| `base_footprint` to `camera_base` | static TF, `bringup.launch.py` | Same file. `cone_detection_node` `camera_frame` is set to `camera_base`. |
| `map` to `odom` (identity) | static TF, `bringup.launch.py` | Only when `slam_toolbox` is not publishing it (`slam:=gtsam`, `slam:=none`, or `use_slam:=false`). |
| `map` to `odom` | `slam_toolbox` | `slam:=toolbox` with `use_slam:=true`. Replaces the identity transform. |
| `odom` to `base_footprint` | `ekf_node` (`publish_tf`) | EKF pose, which is GPS-origin ENU. Needs `use_ekf:=true`. |

The EKF pose is in the GPS-origin ENU frame, which `ekf_node` labels `map` in `/ekf/output`. The
TF uses `odom` as its parent instead, so `map` stays free for the SLAM correction (`slam_toolbox`)
on top of `odom`. This departs from REP 105: the EKF is GPS-fused, so `odom` can jump on a GPS
update and is not a locally smooth odometry frame.

Single-publisher rule: `map` to `odom` is published by `slam_toolbox` or by the identity static
TF, never both. Under `hardware:=true` the identity TF is skipped whenever `slam_toolbox`
publishes. Sensor and EKF transforms are in the perception group, so `compute.launch.py` (which
runs without perception) publishes no transforms.
