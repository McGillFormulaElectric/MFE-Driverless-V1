# Jetson Setup

How to set up a Jetson (Orin Nano or similar) as a compute or perception node.

Script: `scripts/setup_jetson.sh`

---

## Overview

`setup_jetson.sh` runs six steps, each idempotent (safe to re-run):

1. Install ROS 2 Humble base
2. Install ROS dependencies + CycloneDDS
3. Install Python dependencies (`fsd_path_planning`, `scipy`, etc.)
4. Clone or update the MFE-Driverless-V1 repo
5. Build compute packages with colcon
6. Configure `~/.bashrc` with ROS environment variables

---

## Running the script

### Option A — run directly on the Jetson

```bash
git clone https://github.com/McGillFormulaElectric/MFE-Driverless-V1.git ~/Develop/MFE-Driverless-V1
bash ~/Develop/MFE-Driverless-V1/scripts/setup_jetson.sh
```

### Option B — pipe over SSH from host PC

```bash
ssh mfe@100.119.35.6 "bash -s" < scripts/setup_jetson.sh
```

### Option C — automatic via fleet.launch.py

`fleet.launch.py` runs the setup script automatically on first launch if the workspace is not already built. See [[fleet]] for details.

---

## What gets built

The script builds **only the packages needed for compute or perception** — no Gazebo, no GUI, no simulation.

| Package | Built | Purpose |
|---------|-------|---------|
| `mfe_msgs` | Yes | Custom message types (required by everything) |
| `mfe_bringup` | Yes | Launch files + vehicle.yaml |
| `mfe_path_planning` | Yes | Path planner + finish detector + boundary extractor |
| `mfe_control` | Yes | Pure pursuit controller |
| `lidar_cone_detector` | No | LiDAR C++ node (install separately on perception Jetson) |
| `vision_cone_detector` | No | YOLO Python node (install separately on perception Jetson) |
| `mfe_eufs_sim` | No | Sim-only (Gazebo not available on Jetson) |
| `mfe_mapping` | No | SLAM toolbox (install on perception Jetson if needed) |
| `mfe_state_estimation` | No | EKF (install on perception Jetson if needed) |
| `perception_evaluator` | No | Sim-only evaluation |

**For a perception Jetson** (Jetson 1), you need to also build `lidar_cone_detector`, `vision_cone_detector`, `mfe_mapping`, and `mfe_state_estimation`. Add them to the `--packages-select` list in the script.

---

## Python dependencies

```
numpy<2       # must stay below 2.x — NumPy 2 ABI breaks several packages
scipy
scikit-learn
icecream
fsd_path_planning   # cloned from github.com/papalotis/ft-fsd-path-planning
```

`fsd_path_planning` is cloned to `~/Develop/ft-fsd-path-planning` and installed with `pip install`.

---

## Shell environment

The script appends these lines to `~/.bashrc` (only once — guarded by `grep -qxF`):

```bash
source /opt/ros/humble/setup.bash
source ~/Develop/MFE-Driverless-V1/ros2/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclone_tailscale.xml
```

It also copies `cyclone_tailscale.xml` to `~/cyclone_tailscale.xml` (only if not already present, to avoid overwriting local edits).

After setup, source the new env:

```bash
source ~/.bashrc
```

---

## SSH key setup (from host PC)

```bash
# Generate key if you don't have one
ssh-keygen -t ed25519

# Copy to each Jetson
ssh-copy-id mfe@100.119.35.6   # Jetson 2 (compute)
ssh-copy-id mfe@100.84.7.80    # Jetson 1 (perception) — when online

# Verify
ssh mfe@100.119.35.6 echo "key auth works"
```

> Use Tailscale IPs — short hostnames like `mfe-driverless-2` don't resolve cross-org in MagicDNS.

---

## Verifying the setup

```bash
# On the Jetson, after sourcing ~/.bashrc:

# Check ROS2 is available
ros2 --help

# Check the workspace is built
ls ~/Develop/MFE-Driverless-V1/ros2/install/mfe_bringup

# Check ROS2 sees the host PC's topics (while sim is running on host)
ros2 topic list

# Launch compute manually (for testing without fleet.launch.py)
ros2 launch mfe_bringup compute.launch.py mission:=autocross
```

---

## Updating the stack on the Jetson

```bash
cd ~/Develop/MFE-Driverless-V1
git pull

cd ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF \
  --packages-select mfe_msgs mfe_bringup mfe_path_planning mfe_control
```

`fleet.launch.py` does `git pull --ff-only` automatically on every launch, but you still need to rebuild manually if source files changed.

---

## Troubleshooting

**`colcon build` fails with missing dependencies**
→ Run: `rosdep install --from-paths src --ignore-src -r -y`

**Python import errors at launch (`ModuleNotFoundError: fsd_path_planning`)**
→ Re-run: `pip3 install ~/Develop/ft-fsd-path-planning`

**Topics not visible from host after launch**
→ Check `CYCLONEDDS_URI` is set and `~/cyclone_tailscale.xml` exists with the host PC's Tailscale IP. See [[networking]].
