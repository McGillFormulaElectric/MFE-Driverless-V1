#!/usr/bin/env bash
set -euo pipefail

# ——— CONFIGURE ———
# Adjust these to match your ROS 2 distro and workspace layout:
ROS_DISTRO=humble
ROS_INSTALL=/opt/ros/$ROS_DISTRO/setup.bash

# Map menu keys to "<package> <launch_file>"
declare -A options=(
  ["1"]="vision_cone_detector vision_pipeline.launch.py"
  ["2"]="lidar_cone_detector lidar_pipeline.launch.py"
  ["3"]="mfe_eufs_sim mfe_eufs_sim.launch.py"
  # add or remove entries as needed...
)
# —————————————————

# 1) Source base ROS 2
if [ -f "$ROS_INSTALL" ]; then
  echo -e "\n=== Sourcing ROS 2 base ($ROS_DISTRO) ==="
  set +u
  source "$ROS_INSTALL"
  set -u
else
  echo "ERROR: cannot find ROS install at $ROS_INSTALL" >&2
  exit 1
fi

# 2) Ask user which packages to skip
echo -e "\n=== Skip Packages ==="
read -rp "Enter packages to skip (space-separated, or leave empty for none): " skip_input
skip_args=()
if [[ -n "$skip_input" ]]; then
  read -ra skip_args <<< "$skip_input"
fi

# 3) Build or Skip
echo -e "\n=== Build Options ==="
echo "1) Build workspace"
echo "2) Skip build (use existing install)"
read -rp "Enter choice [1/2]: " build_choice

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_ROOT"

if [[ "$build_choice" == "1" ]]; then
  echo "Building workspace..."
  if [ ${#skip_args[@]} -gt 0 ]; then
    echo "Skipping packages: ${skip_args[*]}"
    colcon build --symlink-install --packages-ignore "${skip_args[@]}"
  else
    colcon build --symlink-install
  fi
elif [[ "$build_choice" == "2" ]]; then
  echo "Skipping build step."
else
  echo "Invalid build choice: '$build_choice'" >&2
  exit 1
fi

# 4) Source your workspace overlay
echo -e "\n=== Sourcing workspace overlay ==="
set +u
source "$WS_ROOT/install/setup.bash"
set -u

# 5) Ask if sample data should be streamed
echo -e "\n=== Stream Sample Data ==="
read -rp "Stream sample data? [Y/N]: " stream_choice
if [[ "$stream_choice" =~ ^[Yy]$ ]]; then
  load_file_arg="load_file:=True"
else
  load_file_arg="load_file:=False"
fi

# 6) Show launch menu
echo -e "\n=== Select a launch file to run ==="
for key in "${!options[@]}"; do
  pkg_and_file=(${options[$key]})
  printf "%2s) %s/%s\n" "$key" "${pkg_and_file[0]}" "${pkg_and_file[1]}"
done

read -rp "Enter launch choice: " choice

# 7) Launch
if [[ -n "${options[$choice]:-}" ]]; then
  pkg_and_file=(${options[$choice]})
  echo -e "\nLaunching ➜ ros2 launch ${pkg_and_file[0]} ${pkg_and_file[1]} ${load_file_arg}\n"
  ros2 launch "${pkg_and_file[0]}" "${pkg_and_file[1]}" "${load_file_arg}"
else
  echo "Invalid choice: '$choice'" >&2
  exit 1
fi
