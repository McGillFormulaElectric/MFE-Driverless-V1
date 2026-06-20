#!/bin/bash
# Wrapper that sources ROS2 + workspace before launching RL training.
# Usage: bash scripts/train_rl.sh [--n-envs N] [--timesteps T] [--bc] [--resume]

MFE_WS=~/Develop/MFE-Driverless-V1/ros2

source /opt/ros/humble/setup.bash
source $MFE_WS/install/setup.bash

python3 "$(dirname "$0")/../training/rl/train_ppo.py" "$@"
