# Scripts

Every shell script in `scripts/`. These are the primary entry points for running the system.

---

## docker_build.sh

**Purpose**: Build the `mfe-driverless-sim` Docker image.

```bash
bash scripts/docker_build.sh
```

Runs `docker build` pointing at `Docker/fs-driverless-sim/Dockerfile`. Takes ~5–15 minutes on first run (downloads ROS 2 Humble desktop, Gazebo 11, and Python deps).

---

## docker_run.sh

**Purpose**: Run the simulation inside Docker with GPU + display passthrough. Auto-installs Docker and `nvidia-container-toolkit` if missing.

```bash
bash scripts/docker_run.sh [track] [mode] [gui]
```

| Argument | Default | Options |
|----------|---------|---------|
| track | `accel` | any track name |
| mode | `no_perception` | `no_perception`, `perception` |
| gui | `gui` | `gui`, `nogui` |
| laps | `1` | positive integer; `0` = endless |

**What it does**:
1. Checks for Docker → auto-installs via `get.docker.com` if missing.
2. Checks for `nvidia-container-toolkit` → auto-installs if missing.
3. Clones `MFE26-eufs-sim` and `eufs_msgs` if not present in `~/Develop/`.
4. Grants X11 access: `xhost +local:docker`.
5. Runs `mfe-driverless-sim` container with:
   - `--gpus all` + `NVIDIA_DRIVER_CAPABILITIES=all`
   - `--network host` + `--ipc host`
   - Volume mount: `~/Develop` → `/root/Develop`
   - Display passthrough via `DISPLAY` + `/tmp/.X11-unix`
6. Inside the container, calls `launch_sim.sh` with the same arguments.

**Note**: Because `docker_run.sh` is interactive (it attaches a tmux session), run it with the `!` prefix in Claude Code so it gets a real TTY:
```
! bash scripts/docker_run.sh peanut no_perception nogui 5
```
Once the tmux session opens, **press Enter in pane 4** (mid-right) to start the car.

---

## launch_sim.sh

**Purpose**: Full simulation launch. Opens a 6-pane tmux session and starts all components.

```bash
bash scripts/launch_sim.sh [track] [mode] [gui] [laps]
```

| Argument | Default | Options |
|----------|---------|---------|
| `track` | `accel` | `accel`, `skidpad`, `peanut`, `small_track`, `rectangle`, `autocross`, `garden_light`, `boa_constrictor`, `comp_2021`, `hairpins`, `rand`, `its_a_mess` |
| `mode` | `no_perception` | `no_perception`, `perception` |
| `gui` | `gui` | `gui`, `nogui` |
| `laps` | `1` | positive integer; `0` = endless loop |

**Environment variable**: `GAZEBO_PORT` (default `11350`) — set to a different port when running two sim containers simultaneously.

**Launch sequence**:
1. Parses track → sets `TRACK`, `MISSION`, `AMI_STATE`, `SPAWN_X/Y/YAW`.
2. Parses mode → sets `LAUNCH_GROUP`, `USE_SIM_CONES`, `PUBLISH_GT_TF`.
3. Kills any stale tmux session and ROS/Gazebo processes.
4. Restarts ROS 2 daemon.
5. Creates a new tmux session `mfe` (6 panes).
6. Starts EUFS sim in pane 0 and waits 15 s.
7. If car not spawned, does manual `spawn_entity.py`.
8. Starts MFE bridge (pane 1), MFE bringup (pane 2), Foxglove (pane 3).
9. Pre-fills mission control command in pane 4 (press Enter to start driving).
10. Starts logger in pane 5.
11. Attaches to the tmux session if running in a real terminal.

**Laps routing**:
```bash
if [ "$LAPS" = "0" ]; then
    BRINGUP_EXTRAS="$BRINGUP_EXTRAS endless:=true"
else
    BRINGUP_EXTRAS="$BRINGUP_EXTRAS num_laps:=$LAPS"
fi
```

---

## run_laps.sh

**Purpose**: Helper wrapper for running multiple laps (if present). Calls `launch_sim.sh` with a lap count argument.

---

## setup_linux.sh

**Purpose**: First-time setup for a new Linux developer machine (Ubuntu 22.04). Installs system packages, configures WezTerm, sets up shell aliases.

```bash
bash scripts/setup_linux.sh
```

Notable: restores WezTerm title bar so the window can be moved (removes the custom CSS that hides it).

---

## setup_mac.sh

**Purpose**: First-time setup for macOS. Installs Homebrew, development tools, and configures the shell environment.

---

## setup_remote.sh

**Purpose**: Sets up SSH access to a remote development machine (e.g., a lab desktop running the full sim stack). Configures SSH config entries and key forwarding.

---

## build_gazebo_from_source.sh

**Purpose**: Builds Gazebo 11 from source — used when the apt-installed version has bugs or missing features needed for the sim. Only required in exceptional circumstances; the Docker image uses the apt version.

---

## Common patterns

### Running multiple containers

```bash
# Container 1 — use non-default Gazebo port
docker exec mfe-sim bash -c "GAZEBO_PORT=11355 bash /root/Develop/MFE-Driverless-V1/scripts/launch_sim.sh peanut no_perception nogui 5"

# Container 2 — default port, isolated ROS domain
docker run ... -e ROS_DOMAIN_ID=1 mfe-driverless-sim bash launch_sim.sh small_track no_perception nogui 0
```

### Attaching to a running tmux session

```bash
# Inside the container
tmux attach-session -t mfe

# From host (via docker exec)
docker exec -it mfe-sim tmux attach-session -t mfe
```

### Watching logs

```bash
tail -f ~/mfe_logs/<track>_<timestamp>.log
cat ~/mfe_logs/perception_accuracy.csv
```
