#!/bin/bash
# Replay accel bag through lidar_perception_node; report output rate. Usage: node_smoke.sh <use_gpu true|false> <rate>

USE_GPU=$1; RATE=$2
source /opt/ros/humble/setup.bash
source /home/neil/Develop/simws_gpu/install/setup.bash
export ROS_DOMAIN_ID=21
pkill -f lidar_perception_node 2>/dev/null; sleep 1
ros2 run lidar_cone_detector lidar_perception_node --ros-args -p use_gpu:=$USE_GPU > /tmp/node_$USE_GPU.log 2>&1 &
NODE=$!
sleep 4
ros2 topic hz /perception/cones_uncolored --window 200 > /tmp/hz_$USE_GPU.log 2>&1 &
HZ=$!
sleep 1
ros2 bag play /home/neil/Develop/simws_gpu/bags/accel_bag -r $RATE --topics /lidar/points_raw > /tmp/play_$USE_GPU.log 2>&1
sleep 2
kill -INT $HZ 2>/dev/null; kill -INT $NODE 2>/dev/null; sleep 2; pkill -f lidar_perception_node 2>/dev/null
echo "== use_gpu=$USE_GPU rate=${RATE}x"
grep -E "Mode|no CUDA|use_gpu" /tmp/node_$USE_GPU.log | head -3
grep -c "voxel=" /tmp/node_$USE_GPU.log | sed 's/^/frames processed (GPU branch logs): /'
tail -3 /tmp/hz_$USE_GPU.log
