# Simulation test harness

Drives the full autonomy stack headlessly in the EUFS Gazebo simulator and scores each run: did the car finish,
did it stay on the track, which cones did its footprint touch, how many laps, how fast. It also records bags,
plots, optional videos, speed maps and host CPU/GPU load.

The sim configuration is the `lidar_only` mode of `tools/validation/launch_sim_local.sh`: the EUFS car is driven
from `/slam/odom`, and the LiDAR cone detector and SLAM run in the loop. Ground truth is used for scoring only.

## Prerequisites

- **Host**: Linux with Docker. `pidstat` (sysstat) and `nvidia-smi` are needed only for `LOAD=1`.
- **GPU**: the GPU LiDAR path needs an NVIDIA GPU and `nvidia-container-toolkit` (`docker run --gpus all`).
  Without a GPU, run with `USE_GPU=false` to use the CPU path.
- **Image**: build the image once with `bash scripts/docker_build.sh` (builds `mfe-driverless-sim` from
  `Docker/fs-driverless-sim`).
- **Repos and workspace** (the exact steps the from-scratch check runs, with the defaults below):

  ```
  cd ~/Develop
  git clone --branch pr47-integration https://github.com/McGillFormulaElectric/MFE-Driverless-V1.git MFE-Driverless-V1
  git -C MFE-Driverless-V1 submodule update --init ros2/src/fs_msgs
  git clone --branch mfe-sim-realtime-cmd-fix git@github.com:McGillFormulaElectric/MFE26-eufs-sim.git
  git clone https://gitlab.com/eufs/eufs_msgs.git && git -C eufs_msgs checkout 9e91868
  mkdir -p simws/src && cd simws/src
  ln -s ~/Develop/MFE-Driverless-V1/ros2/src mfe
  ln -s ~/Develop/eufs_msgs eufs_msgs
  for p in eufs_models eufs_plugins eufs_racecar eufs_rqt eufs_sensors eufs_tracks eufs_launcher; do
    ln -s ~/Develop/MFE26-eufs-sim/$p $p; done
  ```

  `MFE26-eufs-sim` branch `mfe-sim-realtime-cmd-fix` has the race-car plugin's command-delivery and 1.0x
  real-time pacing fixes; without them commands stall and the sim runs at ~0.8x. `ros2/src/eufs_msgs` in this
  repo is an empty submodule placeholder with no `.gitmodules` entry, so `eufs_msgs` is cloned separately (pinned
  to the tested commit). The track CSVs are in `$EUFS_WS/eufs_tracks/csv`.
- **Build** the workspace inside the container (after "Starting the container" below), without
  `--symlink-install`:

  ```
  docker exec mfe-sim-gpu-test bash -c "source /opt/ros/humble/setup.bash && cd ~/Develop/simws && colcon build"
  ```

  All 24 packages should finish, including `eufs_launcher` (the tracked one at the repo root; without it the car
  never spawns). `build/lidar_cone_detector/CMakeCache.txt` should show
  `CMAKE_CUDA_COMPILER` set to nvcc; if not, LiDAR runs on the CPU.

## Starting the container

```
docker run -d --name mfe-sim-gpu-test --gpus all \
  -v "$HOME/Develop:$HOME/Develop" -v "$HOME/Develop:/root/Develop" \
  mfe-driverless-sim sleep infinity
```

This is the setup every result in this harness was produced with: the container runs as root on Docker's
default bridge network (so its ROS 2 traffic stays inside the container; the scripts also set
`ROS_DOMAIN_ID=12`), and the host `~/Develop` is mounted twice. The harness needs it at the **same path**
as on the host, so every path the scripts build resolves the same on host and in the container; the image's
`EUFS_MASTER` expects it at `/root/Develop`. `scripts/docker_run.sh` is for interactive use and only mounts
`/root/Develop`, so do not use it for the harness.

The scripts refuse to run outside a container (they check `/.dockerenv`). `run_batch.sh` runs on the host and
calls `docker exec` into `$SIM_CONTAINER`.

## Environment variables

Paths (all scripts; same names everywhere):

| Variable | Default | Meaning |
|---|---|---|
| `MFE_REPO` | derived from the script location | this repo checkout |
| `SIMWS` | `$HOME/Develop/simws` | colcon workspace with the stack + EUFS packages |
| `EUFS_WS` | `$HOME/Develop/MFE26-eufs-sim` | EUFS sim checkout |
| `EUFS_TRACKS_CSV` | `$EUFS_WS/eufs_tracks/csv` | track CSVs |
| `RESULTS` | `$HOME/mfe_sim_results` | bags, CSVs, plots, videos, load files |
| `SIM_CONTAINER` | `mfe-sim-gpu-test` | container name (`run_batch.sh`) |
| `GAZEBO_ROS_WS` | `$HOME/Develop/gazebo_ros_pkgs` | optional extra gazebo_ros install (launch script) |
| `LOG_DIR` | `$HOME/mfe_logs` | launch-script logs |
| `GAZEBO_PORT` | `11350` | Gazebo master port |

Run options (read by `run_sim_test.sh` and `launch_sim_local.sh`):

| Variable | Default | Meaning |
|---|---|---|
| `COLOR_MODE` | `geometric` (`unknown` in autocross batches) | cone colour mode given to the LiDAR detector |
| `MIN_OBS` | `2` | minimum observations before a cone is published |
| `NUM_LAPS` | `1` | laps before the finish gate (also the lap count the batch verdict requires) |
| `USE_GPU` | `true` | GPU LiDAR pipeline (`false` = CPU) |
| `MISSION_OVERRIDE` | unset | run the bringup under another mission, e.g. `autocross` for peanut |
| `PLANNER_BACKEND` | `fasttube` | planner backend |
| `STEER_PT2_TAU` | `0.05` | steering PT2 time constant |
| `SPEED_SMOOTH` | `5` | speed smoothing window (`speed_smooth_window`) |
| `MAX_SPEED` | bringup default | pure pursuit `max_speed` override |
| `LOOKAHEAD` | unset | pure pursuit lookahead override |
| `UNKNOWN_CONE_MODE` | stack default (`trace`) | cone sorting for uncoloured cones: `trace` = wall tracer (default); `trace_backend`; `fasttube` = ft-fsd's own colourless cone sorting; `heading` = legacy |
| `REC` | `$RESULTS/bags/single_<event>_<time>` | bag output directory (`run_sim_test.sh`) |
| `VIDEO_LABEL` | empty | label in the video title (`render_lap_video.py`) |

Batch options (`run_batch.sh`, host side):

| Variable | Default | Meaning |
|---|---|---|
| `TAG` | required | 2-letter run-id prefix |
| `EVENTS` | the six autocross events | space-separated event names (table below) |
| `N` | `3` | successful runs per event |
| `TIMEOUT` | per-event table | override the scorer timeout (s); needed for `NUM_LAPS=5`, e.g. `420` |
| `CSV` | `$RESULTS/$TAG.csv` | results file (appended to) |
| `LOAD` | `0` | `1` records host CPU (`pidstat`) and GPU (`nvidia-smi`) load per run |
| `ROS_DOMAIN_ID` | `12` | ROS domain used inside the container |

Batch defaults: `MAX_SPEED` is 8.0 for autocross events; skidpad and accel use the bringup default unless
`MAX_SPEED` is set. `SPEED_SMOOTH` is always passed (default 5).

Events for `EVENTS` (run ID letter in brackets):

| Event | Track | Scorer timeout |
|---|---|---|
| `ac_peanut` [P] | peanut | 240 s |
| `ac_mess` [M] | its_a_mess | 240 s |
| `ac_small` [S] | small_track | 240 s |
| `ac_comp` [C] | comp_2021 | 360 s |
| `ac_hairpins` [H] | hairpins_increasing_difficulty | 600 s |
| `ac_boa` [B] | boa_constrictor | 240 s |
| `ac_garden` [G] | garden_light | 300 s |
| `skidpad` [K] | skidpad | 240 s |
| `accel` [A] | acceleration | 120 s |

## Running

Single run (inside the container; the path is the repo checkout as seen in the container):

```
docker exec -e ROS_DOMAIN_ID=12 mfe-sim-gpu-test bash <repo>/tools/validation/sim_harness/run_sim_test.sh peanut 240
```

`run_sim_test.sh [event] [timeout_s]` prints the `score_run.py` result and the path of its log directory. It
does not compute plots, cone hits or the verdict; `run_batch.sh` does.

Batches (host). Run IDs are `TAG` + track letter + `F` + two-digit index, e.g. `AXPF01`.

```
# Autocross on all six tracks without colours (ac_* default to COLOR_MODE=unknown)
TAG=AX EVENTS="ac_peanut ac_mess ac_small ac_comp ac_hairpins ac_boa" N=3 bash tools/validation/sim_harness/run_batch.sh

# Skidpad and accel
TAG=SK EVENTS="skidpad accel" N=5 bash tools/validation/sim_harness/run_batch.sh

# 5-lap autocross runs
TAG=F5 NUM_LAPS=5 TIMEOUT=420 EVENTS="ac_peanut ac_mess ac_boa" N=3 bash tools/validation/sim_harness/run_batch.sh

# Speed test: different smoothing / speed cap
TAG=SP SPEED_SMOOTH=3 MAX_SPEED=10 EVENTS="ac_peanut ac_mess" N=3 bash tools/validation/sim_harness/run_batch.sh

# GPU vs CPU load comparison, then summarise both
TAG=GG USE_GPU=true  LOAD=1 EVENTS="ac_peanut" N=3 bash tools/validation/sim_harness/run_batch.sh
TAG=CC USE_GPU=false LOAD=1 EVENTS="ac_peanut" N=3 bash tools/validation/sim_harness/run_batch.sh
python3 tools/validation/sim_harness/viz/load_summary.py
```

Use a different `TAG` for each batch. The CSV is appended to, and re-running an ID overwrites its bag, plot and
load files.

### Outputs

Under `$RESULTS`:

- `<TAG>.csv`: one row per attempt. Columns: `id, event, index, verdict, mission_completed, wall_s, sim_s,
  clips, closest_m, bag, plot, logdir, note`. `clips` is the number of cones within 0.3 m of the car centre.
  `closest_m` is the closest car-centre approach to any cone. `note` holds the event-specific checks (laps,
  off-track, cones touched, planner config from the probe).
- `bags/<id>/`: ROS 2 bag with `/ground_truth/state_odom`, `/slam/odom`, `/planning/cones`,
  `/planning/centerline`, `/control/command`, `/cmd` and the LiDAR input. `bags/<id>.out` is the raw run output,
  `bags/<id>.probe` the config probe.
- `plots/<id>.png`: top-down plot of true cones, the true path coloured by time, and the SLAM pose.
- `load/<id>.pidstat` and `load/<id>.gpu`: only with `LOAD=1`.

Verdicts:

| Verdict | Meaning |
|---|---|
| `clean` | finished and passed every check, with no cone touched by the footprint |
| `clean_but_clips` | finished and passed the checks, but the footprint touched at least one cone |
| `off_track` | the ground-truth path left the track boundary (autocross) |
| `no_finish` | did not finish within the timeout, or fewer than `NUM_LAPS` laps |
| `infra_fail` | the sim or bag did not come up. Not counted; the batch retries (up to N+3 attempts) |
| `score_error` | autocross scoring produced no output |
| `false_finish` | skidpad finished, but the lap or exit check failed |
| `bad_finish` | accel finished, but the gate or lateral check failed |

## How runs are judged

- **Finish**: `MISSION_COMPLETED:TRUE` in the final `/ros_can/state_str` (printed by `score_run.py`). Time is
  wall-clock and sim time from first movement.
- **Autocross, on track** (`score/offtrack.py`): the ground-truth path, mapped into the csv frame, is compared
  with the outer and inner boundary polygons built from the blue and yellow cones. The car is on track when it is
  inside the outer line and outside the inner line, each with a 0.5 m margin measured as true distance. The run
  is off track when 5 consecutive (every 5th message) samples are outside. Laps are forward crossings of the start
  line. `run_batch.sh` sets `OFFTRACK_EXACT=1` (exact margin); `offtrack.py` on its own still defaults to the
  legacy rule, kept for older bags.
- **Cones touched** (`score/cone_hits.py`): the car footprint is the rectangle x in [-1.0, +1.4] m, y in
  [-0.8, +0.8] m around the centre of gravity. A cone (base radius 0.114 m) counts as touched if its circle
  overlaps the rectangle.
- **Skidpad** (`score/laps.py`): left and right laps from the integrated ground-truth heading, each in [1.75, 2.5],
  and the car ends more than 8 m from where it first turned.
- **Accel** (`score/accel_check.py`): maximum x in [78, 103] m (finish gate at 78, stop zone ends at 103) and
  maximum |y| at most 1.2 m.
- **Command timing** (`score/hold_rate.py`): median interval between command steps in the first 6 s, reported in
  the note.

## Videos, speed maps, lap comparisons

These need ROS and rosbag2 (the container). Run them there, with the output directory created first:

```
docker exec mfe-sim-gpu-test bash -c "source /opt/ros/humble/setup.bash && source \$SIMWS/install/setup.bash && \
  mkdir -p \$RESULTS/videos && \
  NUM_LAPS=5 python3 <repo>/tools/validation/sim_harness/viz/render_lap_video.py \
  \$RESULTS/bags/AXPF01 \$RESULTS/videos/AXPF01.mp4 peanut 1 'no cone colours'"
```

- `viz/render_lap_video.py <bag> <out.mp4> [track] [speed] [label]`: top-down real-time video. Needs `ffmpeg` in
  the container. Lap count from `NUM_LAPS`.
- `viz/speed_map.py <track> <old_bag> <new_bag> <out.png> [old_label] [new_label]`: speed by location, old and
  new run side by side on one colour scale.
- `viz/lap_compare.py <track> <old_bag> <new_bag> [old_label] [new_label]`: per-lap times and total for two runs
  (markdown table to stdout).
- `viz/speed_limits.py <bag> [<bag> ...]`: offline replay of the pure pursuit speed limiter for each bag.

## Pitfalls

- Run one batch at a time per container. A second batch or a manual run shares the Gazebo port and ROS domain.
- Never run `launch_sim_local.sh` or `run_sim_test.sh` on the host. Their cleanup kills tmux and ROS processes,
  which can include your own sessions.
- To stop a batch, kill the `run_batch.sh` process, then `docker restart mfe-sim-gpu-test`. The baseline
  teardown check at the start of the next batch aborts if the container is not clean.
- `LOAD=1` samples every process on the host. The "other ros2/python" group counts any host process with
  `ros2` or `python` in its command line, so keep the host otherwise idle.
- Each run is followed by a 30 s pause and a container restart, so batches are slow. Keep `N` small while
  iterating.
- `TIMEOUT` is a per-batch override. Without it, 5-lap runs on the longer tracks will time out.
