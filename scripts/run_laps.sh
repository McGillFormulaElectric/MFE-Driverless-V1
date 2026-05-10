#!/bin/bash
# Automated lap runner for peanut figure-8 track
# Usage: bash run_laps.sh [start_lap] [end_lap] [speed]

START_LAP=${1:-2}
END_LAP=${2:-10}
SPEED=${3:-7}

SOURCE="export EUFS_MASTER=/root/Develop/MFE26-eufs-sim && source /opt/ros/humble/setup.bash && source /root/Develop/MFE26-eufs-sim/install/setup.bash && source /root/Develop/MFE-Driverless-V1/ros2/install/setup.bash"

RESULTS_FILE="/tmp/lap_results.txt"
echo "Lap results at speed ${SPEED} m/s" > $RESULTS_FILE
echo "=============================" >> $RESULTS_FILE

run_lap() {
    local LAP=$1
    echo "=== Starting lap $LAP / $END_LAP (speed=${SPEED} m/s) ==="

    # Kill and restart sim fresh
    bash /root/Develop/MFE-Driverless-V1/scripts/launch_sim.sh peanut no_perception nogui > /tmp/launch_sim_restart.log 2>&1 &

    # Wait for Gazebo ready
    local WAIT=0
    until grep -q 'Gazebo ready' /tmp/launch_sim_restart.log 2>/dev/null; do
        sleep 3
        WAIT=$((WAIT + 3))
        if [ $WAIT -ge 90 ]; then
            echo "ERROR: Gazebo did not become ready in 90s"
            echo "Lap $LAP: FAIL (sim startup timeout)" >> $RESULTS_FILE
            return 1
        fi
    done
    echo "Gazebo ready for lap $LAP (waited ${WAIT}s)"

    # Start mission
    eval "$SOURCE"
    local RETRIES=0
    while ! ros2 service call /ros_can/set_mission eufs_msgs/srv/SetCanState '{ami_state: 13, as_state: 1}' 2>/dev/null | grep -q 'success=True'; do
        sleep 2
        RETRIES=$((RETRIES + 1))
        if [ $RETRIES -ge 10 ]; then
            echo "ERROR: set_mission service call failed"
            echo "Lap $LAP: FAIL (service call timeout)" >> $RESULTS_FILE
            return 1
        fi
    done
    echo "Mission started for lap $LAP"

    local START=$(date +%s)
    local TIMEOUT=150
    local RESULT="TIMEOUT"
    while true; do
        local NOW=$(date +%s)
        local ELAPSED=$((NOW - START))
        if [ $ELAPSED -ge $TIMEOUT ]; then
            RESULT="FAIL (timeout ${ELAPSED}s)"
            break
        fi
        local STATE=$(timeout 2 ros2 topic echo /ros_can/state_str --once 2>/dev/null | head -3)
        if echo "$STATE" | grep -q 'MISSION_COMPLETED:TRUE'; then
            RESULT="SUCCESS in ${ELAPSED}s"
            break
        fi
        sleep 5
    done

    echo "Lap $LAP result: $RESULT"
    echo "Lap $LAP: $RESULT" >> $RESULTS_FILE
}

for i in $(seq $START_LAP $END_LAP); do
    run_lap $i
done

echo ""
echo "=== All laps done ==="
echo ""
cat $RESULTS_FILE
