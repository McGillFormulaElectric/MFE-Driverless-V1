# ros_record: record ROS topics to ~/mfe_bags. Source this file from ~/.bashrc:
#   source ~/Develop/MFE-Driverless-V1/scripts/bench/ros_record.sh
#
# ros_record                          record every topic to ~/mfe_bags/bench_<date>
# ros_record /lidar/points_raw /tf    record only the topics you list
# ros_record -n accel_test ...        bag name prefix (default: bench)
# Other ros2 bag record flags pass through, e.g. ros_record --max-bag-size 2000000000
# Ctrl+C stops and closes the bag cleanly.
#
# Runs in a subshell with the bench DDS settings (scripts/bench/bench_env.sh), so it sees the
# bench stack's topics and does not change the calling shell's environment.
_ROS_RECORD_ENV=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/bench_env.sh

ros_record() {
    local name=bench topics=() flags=()
    while [ $# -gt 0 ]; do
        case "$1" in
            -n|--name) name=${2:?ros_record: -n needs a name}; shift 2 ;;
            -h|--help) sed -n '/^# ros_record  /,/^# Ctrl/p' "${_ROS_RECORD_ENV%/*}/ros_record.sh" | sed 's/^# //'; return 0 ;;
            /*) topics+=("$1"); shift ;;
            *) flags+=("$1"); shift ;;
        esac
    done
    (
        source "$_ROS_RECORD_ENV"
        mkdir -p "$HOME/mfe_bags"
        out=$HOME/mfe_bags/${name}_$(date +%Y%m%d_%H%M%S)
        if [ ${#topics[@]} -eq 0 ]; then
            echo "ros_record: ALL topics -> $out (Ctrl+C to stop)"
            exec ros2 bag record -a -o "$out" "${flags[@]}"
        else
            echo "ros_record: ${topics[*]} -> $out (Ctrl+C to stop)"
            exec ros2 bag record -o "$out" "${flags[@]}" "${topics[@]}"
        fi
    )
}
