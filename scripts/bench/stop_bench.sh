#!/bin/bash
# Stop the bench run started by scripts/bench/start_bench.sh: SIGINT to each recorded process group only.
RUN=$HOME/mfe_bench_runs/latest
[ -f "$RUN/pids" ] || { echo "no bench run recorded"; exit 0; }
while read -r name pgid; do
    kill -INT -- "-$pgid" 2>/dev/null && echo "SIGINT $name (pgid $pgid)"
done < "$RUN/pids"
echo "check later with: ps -o pid,pgid,cmd -g <pgid>; escalate with kill -TERM -- -<pgid> only for groups listed in $RUN/pids"
touch "$RUN/stopped"
