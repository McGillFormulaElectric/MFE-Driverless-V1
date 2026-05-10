# Troubleshooting

Every bug we hit, root cause, and exact fix.

---

## gzserver exits immediately with code 255

**Symptom**: `gzserver` process exits instantly. Log shows nothing useful, or exit code 255.

**Root cause**: Stale Gazebo lock directory `~/.gazebo/server-<port>/` left behind by a previous crashed or force-killed gzserver. When a new gzserver starts and finds this directory, it exits immediately.

**Fix**:
```bash
rm -rf ~/.gazebo/server-11350   # replace with your GAZEBO_PORT
```

Then re-run `launch_sim.sh`. If the port is different (e.g., 11355 for mfe-sim):
```bash
rm -rf ~/.gazebo/server-11355
```

**Prevention**: Always let `launch_sim.sh` handle cleanup (`pkill -f gzserver`) and wait for it to finish before restarting.

---

## Port conflict between two Docker containers (exit 255)

**Symptom**: Second sim container fails to start; gzserver exits with code 255 even after clearing lock files.

**Root cause**: Both containers use `--network host`. If both run with the same `GAZEBO_MASTER_URI` port (default 11350), the second gzserver cannot bind the port.

**Fix**: Set `GAZEBO_PORT` for one of the containers:

```bash
# Container 1 — use port 11355
docker run ... -e GAZEBO_PORT=11355 mfe-driverless-sim bash launch_sim.sh ...

# Container 2 — default port 11350
docker run ... mfe-driverless-sim bash launch_sim.sh ...
```

`launch_sim.sh` reads `${GAZEBO_PORT:-11350}` and exports `GAZEBO_MASTER_URI` accordingly.

---

## Car drives in circles / ignores path

**Symptom**: Car spins or drives erratically instead of following the track.

**Root cause A**: `publish_gt_tf:=false` in eufs_launcher. Without this, Gazebo doesn't publish the TF chain `map → odom → base_footprint`. The car has no pose in the map frame.

**Fix**: Ensure `publish_gt_tf:=true` (this is the default in `launch_sim.sh`).

**Root cause B**: `pose_topic` mismatch. In sim, path planner must use `/ground_truth/state_odom`; if it's set to `/ekf/output` and the EKF has no GPS signal in sim, the car has no pose.

**Fix**: Pass `pose_topic:=/ground_truth/state_odom` (set automatically in no_perception mode by `launch_sim.sh`).

---

## Car stops after 1 lap (multi-lap mode)

**Symptom**: Configured for N laps, but the car stops after the first lap.

**Root cause**: Finish detector triggered on lap 1 because the lap count wasn't passed through the launch chain.

**Fixed in**: `finish_detector_node.py` — added `num_laps` parameter; `bringup.launch.py` — added `num_laps_arg` and reads it in `_make_finish_detector`; `launch_sim.sh` — added 4th positional argument and routes to `endless:=true` or `num_laps:=$LAPS`.

**Verification**: Run `ros2 topic echo /planning/laps_completed` in Foxglove. You should see incrementing integers before the final stop.

---

## Peanut track: false lap trigger at figure-8 crossing

**Symptom**: Finish detector fires mid-track on the peanut figure-8, at the point where the path crosses near the origin.

**Root cause**: The figure-8 midpoint crossing briefly brings the car within `return_to_start_r` metres of the start position.

**Fix**: The `max_dist_from_start >= 15.0` guard. The detector only counts a crossing after the car has ventured at least 15 m from start. The figure-8 crossing happens at ~0 m from start, so it's ignored. Between laps, `_max_dist_from_start` is reset to 0 so the guard re-arms.

---

## No cones visible to path planner (car stops immediately)

**Symptom**: Path planner receives no cones; car does not move or moves straight then stops.

**Root cause A**: In no_perception mode, the bridge is not running. The bridge is what forwards `/ground_truth/track` → `/planning/cones`.

**Fix**: Ensure `mfe_eufs_sim` bridge is launched (pane 1 in tmux). Check `ros2 topic echo /planning/cones`.

**Root cause B**: In perception mode, `boundary_extractor` is not running or LiDAR detector crashed.

**Fix**: Check pane 2 logs. Run `ros2 topic list | grep perception`.

**Root cause C**: Orange cones mapped as `START_FINISH_AREA` in the path planner. The ft-fsd-path-planning library applies special logic to START_FINISH cones that deforms the path near the start gate.

**Fix** (already applied): Orange cones map to `ConeTypes.UNKNOWN` in `path_planner_node.py`.

---

## /camera/image_raw is empty

**Symptom**: `/camera/image_raw` topic publishes but images are black or zero-size.

**Root cause**: Gazebo's camera plugin requires OpenGL rendering. In headless (`nogui`) mode with no X11 display, the camera plugin cannot render frames.

**Fix options**:
1. Run with `gui` mode: `bash docker_run.sh peanut no_perception gui`
2. Run headless with virtual display: `Xvfb :99 -screen 0 1920x1080x24 &` then `DISPLAY=:99 docker run ...`
3. Accept no camera images in headless mode and use LiDAR only.

---

## docker exec process killed by pkill inside container

**Symptom**: SSH session or `docker exec` bash session is killed when running cleanup commands inside the container.

**Root cause**: `pkill -f ros2` matches the `ros2` substring in the docker exec process's command line, killing the exec session itself.

**Fix**: Use detached exec for kill commands:
```bash
docker exec -d mfe-sim bash -c "pkill -f ros2"
```
Or kill by specific PID from the host.

---

## DDS topic bleed-through between containers

**Symptom**: Topics from container 2 appear in container 1's `ros2 topic list`.

**Root cause**: Both containers share the host network and use the same `ROS_DOMAIN_ID` (default 0). DDS UDP multicast is visible across containers.

**Fix**: Set different `ROS_DOMAIN_ID` values:
```bash
docker run ... -e ROS_DOMAIN_ID=1 ...   # container 2
```

---

## ft-fsd-path-planning install fails from PyPI

**Symptom**: `pip install ft-fsd-path-planning` fails or installs a broken package.

**Root cause**: The PyPI package is not maintained. Only the GitHub source is reliable.

**Fix** (already in Dockerfile):
```bash
git clone --depth 1 https://github.com/papalotis/ft-fsd-path-planning.git /opt/ft-fsd-path-planning
pip3 install /opt/ft-fsd-path-planning
```

---

## YOLO training: GPU utilization drops to 3%

**Symptom**: GPU usage drops from ~80% to ~3% during training. Training takes 10× longer than expected.

**Root cause**: `workers=8` (or more) in `model.train()` causes data-loader processes to compete for memory. With sim containers also running, the system exhausts swap. Workers start thrashing, GPU starves.

**Fix**: Set `workers=2` in `train.py`. This is already the default. Do not increase it unless you're sure no sim containers are running.

---

## Car not spawned by eufs_launcher

**Symptom**: Gazebo starts but the car URDF doesn't appear. `/ros_can/state_str` has no output.

**Root cause**: EUFS state machine takes longer to initialize on complex tracks (peanut, small_track). The default 15 s wait in `launch_sim.sh` may not be enough.

**Fix**: `launch_sim.sh` has a fallback `spawn_entity.py` call that runs if `/ros_can/state_str` is not available after 15 s. It spawns the URDF manually and waits up to 20 s for the state machine to come up.

If this also fails, manually run:
```bash
source ~/Develop/MFE26-eufs-sim/install/setup.bash
ros2 run gazebo_ros spawn_entity.py \
  -entity eufs \
  -file ~/Develop/MFE26-eufs-sim/install/eufs_racecar/share/eufs_racecar/robots/eufs/robot.urdf \
  -x 0 -y 0 -z 0.1 -Y 0 -timeout 30.0
```

---

## Foxglove: cannot connect

**Symptom**: Foxglove WebSocket connection fails or times out.

**Root cause A**: Foxglove bridge not started. Check pane 3 of the tmux session.

**Root cause B**: Firewall blocking port 8765.

**Fix**:
```bash
# Check if bridge is listening
ss -tlnp | grep 8765

# Start bridge manually
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

URL format: `ws://localhost:8765` (not `wss://`, no path).

---

## colcon build fails with missing dependencies

**Symptom**: `colcon build` reports missing packages after a fresh clone.

**Fix**:
```bash
cd ~/Develop/MFE-Driverless-V1/ros2
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

For the EUFS workspace:
```bash
cd ~/Develop/MFE26-eufs-sim
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```
