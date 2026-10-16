#!/bin/bash
# Graceful shutdown of everything a sim test run started, inside the sim container.
# Run as: docker exec $SIM_CONTAINER bash <MFE_REPO>/tools/validation/sim_harness/teardown_inner.sh
# (A file, not bash -c, so pkill -f patterns can't match this script's own command line.)
# run_batch.sh calls this after every run, then restarts the container and checks it is clean.
[ -f /.dockerenv ] || { echo "ERROR: run inside the sim container" >&2; exit 1; }
MFE_TMUX_SOCKET=${MFE_TMUX_SOCKET:-mfe_test}

# 1. stop the recorder first so the bag gets its metadata.yaml
pkill -INT -f 'ros2 bag record'
for i in $(seq 1 20); do pgrep -f 'ros2 bag record' >/dev/null || break; sleep 1; done

# 2. ask launch files and the sim to exit
pkill -INT -f 'ros2 launch'
tmux -L "$MFE_TMUX_SOCKET" kill-server 2>/dev/null
pkill -TERM -x gzserver; pkill -TERM -x gzclient
for i in $(seq 1 10); do pgrep -x gzserver >/dev/null || pgrep -f 'ros2 launch' >/dev/null || break; sleep 1; done

# 3. sweep anything left, except PID 1 (sleep infinity) and this script
for p in $(ps -eo pid=,stat= | awk '$2 !~ /Z/ {print $1}'); do
  [ "$p" -eq 1 ] || [ "$p" -eq $$ ] && continue
  kill -KILL "$p" 2>/dev/null
done
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
echo "teardown_inner done"
