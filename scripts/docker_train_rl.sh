#!/bin/bash
# =============================================================================
# Run RL training inside the MFE Docker container (where ROS2 is installed).
#
# Usage: bash scripts/docker_train_rl.sh [--n-envs N] [--timesteps T] [--bc] [--resume]
#
# Prerequisites:
#   - Docker image built: bash scripts/docker_build.sh
#   - N Gazebo sims already running (via start_rl_envs.sh or docker_run.sh)
# =============================================================================

DEVELOP_DIR="$HOME/Develop"
MFE_DIR="$DEVELOP_DIR/MFE-Driverless-V1"

TTY_FLAGS=""
[ -t 0 ] && TTY_FLAGS="-it"

docker run --rm $TTY_FLAGS \
  --network host \
  --ipc host \
  --volume "$DEVELOP_DIR":/root/Develop \
  --name mfe-rl-train \
  mfe-driverless-sim \
  bash -c "
    source /opt/ros/humble/setup.bash && \
    source /root/Develop/MFE-Driverless-V1/ros2/install/setup.bash && \
    pip install gymnasium stable-baselines3 tensorboard --quiet --break-system-packages 2>/dev/null; \
    python3 /root/Develop/MFE-RL/training/rl/train_ppo.py $@
  "
