# Shared ROS environment for the bench scripts. Source it, do not run it.
# Every bench process (and ros_record in ~/.bashrc on the Jetsons) must use the same DDS settings, or they do not see
# each other's topics.
BENCH_REPO=${BENCH_REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}
source /opt/ros/humble/setup.bash
for _ws in "$BENCH_REPO/ros2/install_bench" "$BENCH_REPO/ros2/install"; do
    if [ -f "$_ws/setup.bash" ]; then source "$_ws/setup.bash"; break; fi
done
unset _ws
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
# CycloneDDS where installed, else the ROS default (FastDDS). Default discovery on all
# interfaces: ~/cyclone_tailscale.xml binds to tailscale0 only, so it is not used here.
unset CYCLONEDDS_URI
if [ -f /opt/ros/humble/lib/librmw_cyclonedds_cpp.so ]; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
else
    unset RMW_IMPLEMENTATION
fi
