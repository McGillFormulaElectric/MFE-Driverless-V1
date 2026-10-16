#!/bin/bash
# Batch of simulated runs, run on the HOST (it calls docker exec). One run at a time. After every run: graceful
# teardown inside the container, container restart, cleanliness check, then a 30 s pause before the next run.
# Appends one CSV row per run (see README.md for columns and verdicts).
#
#   TAG=XY EVENTS="ac_peanut ac_mess ac_boa ac_small ac_comp ac_hairpins" N=3 bash run_batch.sh
#   TAG=SK EVENTS="skidpad accel" N=5 bash run_batch.sh
#
# Run IDs = TAG + track letter + F + 2-digit index, e.g. XYPF01 = TAG XY, peanut, first run.
# Track letters: P peanut, M its_a_mess, S small_track, C comp_2021, H hairpins, B boa_constrictor,
# G garden_light, K skidpad, A accel.
# Do not run two batches against one container, and never run launch_sim_local.sh on the host.
set -u

# Paths (environment overrides; defaults assume the ~/Develop layout, same paths on host and container)
MFE_REPO=${MFE_REPO:-$(cd "$(dirname "$0")/../../.." && pwd)}
HARNESS=$MFE_REPO/tools/validation/sim_harness
SIMWS=${SIMWS:-$HOME/Develop/simws}
EUFS_WS=${EUFS_WS:-$HOME/Develop/MFE26-eufs-sim}
EUFS_TRACKS_CSV=${EUFS_TRACKS_CSV:-$EUFS_WS/eufs_tracks/csv}
RESULTS=${RESULTS:-$HOME/mfe_sim_results}
CT=${SIM_CONTAINER:-mfe-sim-gpu-test}

# Batch inputs
TAG=${TAG:?set TAG (2-letter run-id prefix, e.g. XY)}
EVENTS=${EVENTS:-ac_peanut ac_mess ac_small ac_boa ac_comp ac_hairpins}
N=${N:-3}
NUM_LAPS=${NUM_LAPS:-1}
SPEED_SMOOTH=${SPEED_SMOOTH:-5}
MAX_SPEED=${MAX_SPEED:-}           # empty: autocross uses 8.0, skidpad/accel keep the bringup default
USE_GPU=${USE_GPU:-true}
LOAD=${LOAD:-0}                    # 1: record host CPU (pidstat) and GPU (nvidia-smi) load during each run
CSV=${CSV:-$RESULTS/$TAG.csv}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-12}
PLANNER_BACKEND=${PLANNER_BACKEND:-fasttube}
STEER_PT2_TAU=${STEER_PT2_TAU:-0.05}
MIN_OBS=${MIN_OBS:-2}

mkdir -p "$RESULTS/bags" "$RESULTS/plots" "$RESULTS/load" "$(dirname "$CSV")"
[ -s "$CSV" ] || echo "id,event,index,verdict,mission_completed,wall_s,sim_s,clips,closest_m,bag,plot,logdir,note" > "$CSV"

SRC="source /opt/ros/humble/setup.bash && source $SIMWS/install/setup.bash && cd /tmp"
# Paths and scoring flag passed into every docker exec: the container sees the same absolute paths as the host.
DXE=(-e MFE_REPO="$MFE_REPO" -e SIMWS="$SIMWS" -e EUFS_WS="$EUFS_WS" -e EUFS_TRACKS_CSV="$EUFS_TRACKS_CSV"
     -e RESULTS="$RESULTS" -e OFFTRACK_EXACT=1)
RUNENV=(-e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -e REG_MODE=library -e STEER_PT2_TAU="$STEER_PT2_TAU"
        -e PLANNER_BACKEND="$PLANNER_BACKEND" -e NUM_LAPS="$NUM_LAPS" -e SPEED_SMOOTH="$SPEED_SMOOTH"
        -e USE_GPU="$USE_GPU" -e MIN_OBS="$MIN_OBS"
        -e UNKNOWN_CONE_MODE="${UNKNOWN_CONE_MODE:-}")

log() { echo "[$(date +%H:%M:%S)] $*"; }

teardown() {
  docker exec "${DXE[@]}" $CT bash "$HARNESS/teardown_inner.sh"
  docker restart -t 10 $CT >/dev/null
  sleep 3
  local left shm port
  left=$(docker exec $CT bash -c 'ps -eo pid=,stat=,comm= | awk "\$1!=1 && \$2!~/Z/ && \$3!=\"ps\" && \$3!=\"bash\" && \$3!=\"awk\""' | wc -l)
  shm=$(docker exec $CT bash -c 'ls /dev/shm | wc -l')
  port=$(docker exec $CT python3 -c 'import socket;s=socket.socket();s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
try:
    s.bind(("0.0.0.0",11350));print("free")
except OSError:
    print("busy")')
  log "teardown check: leftover procs=$left shm files=$shm gazebo port 11350=$port"
  [ "$left" -eq 0 ] && [ "$shm" -eq 0 ] && [ "$port" = free ]
}

log "baseline teardown"
teardown || { log "baseline not clean, aborting"; exit 1; }
first=1
for EV in $EVENTS; do
  # TO = scorer timeout (s). Lap lengths from the GT centreline: small ~103 m, boa ~89 m, comp ~262 m,
  # hairpins ~829 m, garden ~182 m, peanut ~100 m. KIND selects the scoring branch below.
  case $EV in
    ac_peanut)   KIND=autocross; L=P; SIMEV=peanut;          TRACK=peanut;                         TO=240 ;;
    ac_mess)     KIND=autocross; L=M; SIMEV=its_a_mess;      TRACK=its_a_mess;                     TO=240 ;;
    ac_small)    KIND=autocross; L=S; SIMEV=small_track;     TRACK=small_track;                    TO=240 ;;
    ac_comp)     KIND=autocross; L=C; SIMEV=comp_2021;       TRACK=comp_2021;                      TO=360 ;;
    ac_hairpins) KIND=autocross; L=H; SIMEV=hairpins;        TRACK=hairpins_increasing_difficulty; TO=600 ;;
    ac_boa)      KIND=autocross; L=B; SIMEV=boa_constrictor; TRACK=boa_constrictor;                TO=240 ;;
    ac_garden)   KIND=autocross; L=G; SIMEV=garden_light;    TRACK=garden_light;                   TO=300 ;;
    skidpad)     KIND=skidpad;   L=K; SIMEV=skidpad;         TRACK=skidpad;                        TO=240 ;;
    accel)       KIND=accel;     L=A; SIMEV=accel;           TRACK=acceleration;                   TO=120 ;;
    *) log "unknown event $EV, skipping"; continue ;;
  esac
  [ -n "${TIMEOUT:-}" ] && TO=$TIMEOUT      # e.g. TIMEOUT=420 for NUM_LAPS=5 runs on the longer tracks
  i=1; made=0; attempts=0
  while [ $made -lt $N ] && [ $attempts -lt $((N + 3)) ]; do
    attempts=$((attempts + 1))
    [ $first = 1 ] || { log "sleep 30"; sleep 30; }; first=0
    id=$(printf "%s%sF%02d" "$TAG" "$L" "$i")
    bag=$RESULTS/bags/$id
    plot=$RESULTS/plots/$id.png
    BK=$PLANNER_BACKEND
    MENV=()
    if [ "$KIND" = autocross ]; then
      MENV+=(-e MISSION_OVERRIDE=autocross -e COLOR_MODE="${COLOR_MODE:-unknown}")
    elif [ -n "${COLOR_MODE:-}" ]; then
      MENV+=(-e COLOR_MODE="$COLOR_MODE")
    fi
    MS=$MAX_SPEED; [ "$KIND" = autocross ] && MS=${MAX_SPEED:-8.0}
    [ -n "$MS" ] && MENV+=(-e MAX_SPEED="$MS")
    docker exec $CT rm -rf "$bag" "$bag.cmdlog"
    log "=== $id ($EV on $TRACK, backend $BK, timeout ${TO}s) start (attempt $attempts)"
    # Config probe, 60 s into the run (after the stack is up); results land in the run's .probe file.
    ( sleep 60; docker exec $CT timeout -s KILL 50 bash -c "$SRC && export ROS_DOMAIN_ID=$ROS_DOMAIN_ID && timeout 10 ros2 topic info /planning/centerline; timeout 10 ros2 param get /path_planner_node planner_backend; timeout 10 ros2 param get /path_planner_node mission; timeout 10 ros2 param get /lidar_perception_node use_gpu; timeout 10 ros2 param get /pure_pursuit_node max_speed; timeout 10 ros2 param get /pure_pursuit_node speed_smooth_window; grep -c 'no CUDA device' \$(ls -t ~/.ros/log/lidar_perception_node_*.log | head -1)" > "$bag.probe" 2>&1 ) &
    PROBE=$!
    # Host-side load sampling (sees the container's processes): per-process CPU every 1 s, GPU every 0.5 s.
    if [ "$LOAD" = 1 ]; then
      LC_ALL=C pidstat -u -r -h -l 1 > "$RESULTS/load/$id.pidstat" 2>/dev/null & PS_PID=$!
      nvidia-smi --query-gpu=timestamp,utilization.gpu,memory.used,power.draw --format=csv,noheader,nounits -lms 500 > "$RESULTS/load/$id.gpu" 2>/dev/null & NV_PID=$!
    fi
    out=$(timeout -s INT $((TO + 260)) docker exec "${DXE[@]}" "${RUNENV[@]}" "${MENV[@]+"${MENV[@]}"}" \
          -e REC="$bag" $CT bash "$HARNESS/run_sim_test.sh" $SIMEV $TO 2>&1)
    echo "$out" > "$bag.out"
    if [ "$LOAD" = 1 ]; then kill $PS_PID $NV_PID 2>/dev/null; wait $PS_PID $NV_PID 2>/dev/null; fi
    wait $PROBE 2>/dev/null
    logdir=$(echo "$out" | sed -n 's/^logs: //p' | head -1)
    if echo "$out" | grep -q "sim did not come up" || ! docker exec $CT test -f "$bag/metadata.yaml"; then
      log "$id infra failure (sim/bag missing), not counted"
      echo "${id}x,$EV,$i,infra_fail,,,,,,,,$logdir,sim or bag did not come up" >> "$CSV"
      teardown; continue
    fi
    done_=$(echo "$out" | grep -q "MISSION_COMPLETED:TRUE" && echo 1 || echo 0)
    wall=$(echo "$out" | sed -n 's/^TIME to finish: \([0-9.]*\) s wall.*/\1/p')
    sim=$(echo "$out" | sed -n 's/.*wall-clock, \([0-9.]*\) s SIM.*/\1/p')
    clips=$(echo "$out" | sed -n 's/.*cones within 0.3 m of car centre at some point: \([0-9]*\) of.*/\1/p'); clips=${clips:-0}
    closest=$(echo "$out" | sed -n 's/^closest car-centre approach to any cone: \([0-9.]*\) m.*/\1/p')
    docker exec "${DXE[@]}" $CT bash -c "$SRC && mkdir -p $RESULTS/plots && python3 $HARNESS/score/plot_trajectory.py $bag $plot $TRACK >/dev/null 2>&1"
    read NH FH COLS < <(docker exec "${DXE[@]}" $CT bash -c "$SRC && python3 $HARNESS/score/cone_hits.py $bag $TRACK 2>/dev/null")
    HOLD=$(docker exec "${DXE[@]}" -e WIN=6 $CT bash -c "$SRC && python3 $HARNESS/score/hold_rate.py $bag 2>/dev/null" | sed -n 's/.*median \([0-9.]*\)s.*/\1/p')
    case $KIND in
      skidpad)
        read LL RR EX < <(docker exec "${DXE[@]}" $CT bash -c "$SRC && python3 $HARNESS/score/laps.py $bag 2>/dev/null")
        ok=$(python3 -c "print(int(1.75<=$LL<=2.5 and 1.75<=$RR<=2.5 and $EX==1))" 2>/dev/null || echo 0)
        note="laps ${LL}L ${RR}R exit=${EX}"
        if [ "$done_" = 1 ] && [ "$ok" = 1 ]; then [ "${NH:-1}" = 0 ] && v=clean || v=clean_but_clips
        elif [ "$done_" = 1 ]; then v=false_finish; else v=no_finish; fi ;;
      accel)
        read MX MY TOP TG < <(docker exec "${DXE[@]}" $CT bash -c "$SRC && python3 $HARNESS/score/accel_check.py $bag 2>/dev/null")
        note="max x ${MX} m (gate 78; stop zone ends 103); max |y| ${MY} m; top speed ${TOP} m/s; gate time ${TG} s"
        ok=$(python3 -c "print(int($MX>=78 and $MX<=103 and $MY<=1.2))" 2>/dev/null || echo 0)
        if [ "$done_" = 1 ] && [ "$ok" = 1 ]; then [ "${NH:-1}" = 0 ] && v=clean || v=clean_but_clips
        elif [ "$done_" = 1 ]; then v=bad_finish; else v=no_finish; fi ;;
      *)
        read OFF FO LAPS PCT < <(docker exec "${DXE[@]}" $CT bash -c "$SRC && python3 $HARNESS/score/offtrack.py $bag $TRACK 2>/dev/null")
        note="off track=${OFF:-?} (first at ${FO:-?} m); laps before leaving ${LAPS:-?}; ${PCT:-?}% of samples on track (0.5 m margin)"
        if [ -z "${OFF:-}" ]; then v=score_error
        elif [ "$OFF" = 1 ]; then v=off_track
        elif [ "$done_" = 1 ] && [ "${LAPS:-0}" -ge "$NUM_LAPS" ]; then [ "${NH:-1}" = 0 ] && v=clean || v=clean_but_clips
        else v=no_finish; fi ;;
    esac
    gpu=$(grep -o 'Boolean value is: [a-zA-Z]*' "$bag.probe" 2>/dev/null | head -1 | awk '{print $4}')
    bkp=$(grep -o 'String value is: [a-z]*' "$bag.probe" 2>/dev/null | head -1 | awk '{print $4}')
    msn=$(grep -o 'String value is: [a-z]*' "$bag.probe" 2>/dev/null | sed -n 2p | awk '{print $4}')
    vmax=$(grep -o 'Double value is: [0-9.]*' "$bag.probe" 2>/dev/null | head -1 | awk '{print $4}')
    ssw=$(grep -o 'Integer value is: [0-9]*' "$bag.probe" 2>/dev/null | head -1 | awk '{print $4}')
    note="$note; cones touched (footprint) ${NH:-?} [${COLS:-}]; commands applied every ${HOLD:-?} s in first 6 s; planner_backend ${bkp:-?}; mission ${msn:-?}; pure_pursuit max_speed ${vmax:-?}; speed_smooth_window ${ssw:-?}; use_gpu=${gpu:-not probed}; num_laps $NUM_LAPS; colour mode ${COLOR_MODE:-default}; autocross speed cap ${MS:-bringup default}"
    echo "$id,$EV,$i,$v,$done_,${wall},${sim},${clips},${closest},$bag,$plot,$logdir,\"$note\"" >> "$CSV"
    log "$id ($EV, $BK) -> $v (done=$done_ sim=${sim}s touched=${NH:-?} hold=${HOLD:-?}s; $note)"
    i=$((i + 1)); made=$((made + 1))
    teardown || log "WARNING: teardown check not clean after $id"
  done
done
log "batch finished ($CSV)"
