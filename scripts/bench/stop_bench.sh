#!/bin/bash
# Stop the bench run started by scripts/bench/start_bench.sh. Signals only the process groups in
# the run's pids file: SIGINT, wait up to 20 s for them to exit, then SIGTERM what is left (5 s),
# then SIGKILL. Returns when the run is down, so start_bench.sh can follow straight away.
RUN=$HOME/mfe_bench_runs/latest
[ -f "$RUN/pids" ] || { echo "no bench run recorded"; exit 0; }
alive() {  # number of processes left in the listed groups
    local n=0 name pgid
    while read -r name pgid; do
        n=$((n + $(ps -o pid= -g "$pgid" 2>/dev/null | wc -l)))
    done < "$RUN/pids"
    echo "$n"
}
signal() {  # signal <SIG>: send to every listed group that still has processes
    local name pgid
    while read -r name pgid; do
        if [ "$(ps -o pid= -g "$pgid" 2>/dev/null | wc -l)" -gt 0 ]; then
            kill -"$1" -- "-$pgid" 2>/dev/null && echo "SIG$1 $name (pgid $pgid)"
        fi
    done < "$RUN/pids"
}
signal INT
for _ in $(seq 40); do [ "$(alive)" -eq 0 ] && break; sleep 0.5; done
if [ "$(alive)" -gt 0 ]; then
    signal TERM
    for _ in $(seq 10); do [ "$(alive)" -eq 0 ] && break; sleep 0.5; done
fi
[ "$(alive)" -gt 0 ] && signal KILL
touch "$RUN/stopped"
if [ "$(alive)" -eq 0 ]; then echo "bench run stopped"; else echo "WARNING: $(alive) processes still running"; fi
