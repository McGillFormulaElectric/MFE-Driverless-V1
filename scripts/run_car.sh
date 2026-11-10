#!/bin/bash
# =============================================================================
# MFE Driverless — start the car stack on the two Jetsons.
#
# Run on Jetson 1 (mfe-driverless-1, 192.168.10.1), where the sensors are attached. It starts
# the sensing half here (sensor drivers, LiDAR and camera perception, boundary extractor) and
# the estimation half on Jetson 2 (192.168.10.2) over SSH: SLAM, EKF, planner, control, supervisor.
# Only cone lists, poses and paths cross the wired link. Raw clouds and images stay on Jetson 1.
#
# Usage:
#   scripts/run_car.sh [--mission M] [--laps N] [--sensors LIST] [--replay BAG]
#                      [--single-host] [--dry-run]
#   scripts/run_car.sh --help
#
# Distributed is the default only when the wired car link is up on this host. Without it,
# pass --single-host to run both halves here (testing only).
# =============================================================================

WIRED_NET=192.168.10.
JETSON1_IP=192.168.10.1
JETSON2_IP=192.168.10.2
LIDAR_IP=${LIDAR_IP:-10.37.1.194}   # VLP-16 (docs/lidar-network-setup.md); sends UDP 2368 to 10.37.1.193
REMOTE_USER=${MFE_REMOTE_USER:-mfe}
ROS_DOMAIN=${ROS_DOMAIN_ID:-42}
LOCK_FILE=$HOME/jetson_busy.lock
MAX_RTT_MS=2
MAX_OFFSET_MS=1
LIDAR_PORT=2368

REPO=$(cd "$(dirname "$0")/.." && pwd)
CYCLONE_XML=$REPO/ros2/src/mfe_bringup/config/cyclone_car_wired.xml

MISSION=autocross
LAPS=1
SENSORS=lidar,camera,imu_gps,can
REPLAY_BAG=
SINGLE_HOST=false
DRY_RUN=false

usage() {
    cat <<USAGE
Usage: scripts/run_car.sh [options]

Starts the car stack (platform:=car). Run on Jetson 1 (mfe-driverless-1, $JETSON1_IP).

  --mission M       autocross | trackdrive | acceleration | skidpad | peanut   [autocross]
  --laps N          laps before the finish gate stops the car; 0 = endless     [1]
  --sensors LIST    comma list of lidar,camera,imu_gps,can                     [all]
  --replay BAG      play a recorded bag instead of starting the sensor drivers (bench tests)
  --single-host     run both halves on this Jetson (testing only; no SSH)
  --dry-run         print the preflight results and the commands; start nothing
  --help            this text

Distributed (default) needs the wired car link on this host ($WIRED_NET* up, peer
$JETSON2_IP reachable). Otherwise the script stops and asks for --single-host.

Preflight checks: wired link up; peer ping under ${MAX_RTT_MS} ms; chrony offset under ${MAX_OFFSET_MS} ms
on both hosts; same git commit on both; busy lock $LOCK_FILE free on both hosts; the sensors'
driver packages installed (not with --replay); LiDAR UDP $LIDAR_PORT streaming from $LIDAR_IP
(not with --replay or without lidar).

Stop: Ctrl+C in this terminal stops both halves (the remote half gets a TTY via ssh -t).
USAGE
}

die() { echo "[run_car] ERROR: $*" >&2; exit 1; }
info() { echo "[run_car] $*"; }

while [ $# -gt 0 ]; do
    case "$1" in
        --mission)      MISSION=${2:?--mission needs a value}; shift 2 ;;
        --laps)         LAPS=${2:?--laps needs a value}; shift 2 ;;
        --sensors)      SENSORS=${2:?--sensors needs a value}; shift 2 ;;
        --replay)       REPLAY_BAG=${2:?--replay needs a bag path}; shift 2 ;;
        --single-host)  SINGLE_HOST=true; shift ;;
        --dry-run)      DRY_RUN=true; shift ;;
        --help|-h)      usage; exit 0 ;;
        *)              echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

case "$MISSION" in
    autocross|trackdrive|acceleration|skidpad|peanut) ;;
    *) die "unknown mission '$MISSION'" ;;
esac
case "$LAPS" in
    ''|*[!0-9]*) die "--laps must be a whole number" ;;
esac
for s in ${SENSORS//,/ }; do
    case "$s" in lidar|camera|imu_gps|can) ;; *) die "unknown sensor '$s'" ;; esac
done
SENSORS=${SENSORS//,/,}
has_sensor() { case ",$SENSORS," in *",$1,"*) return 0 ;; *) return 1 ;; esac; }

ENDLESS=false
[ "$LAPS" = 0 ] && ENDLESS=true

# ----------------------------------------------------------------------------
# Local wired link
# ----------------------------------------------------------------------------
WIRED_IF=$(ip -o -4 addr show 2>/dev/null | awk -v net="$WIRED_NET" 'index($4, net) == 1 {print $2; exit}')
LOCAL_WIRED_IP=$(ip -o -4 addr show 2>/dev/null | awk -v net="$WIRED_NET" 'index($4, net) == 1 {split($4, a, "/"); print a[1]; exit}')
LINK_UP=false
if [ -n "$WIRED_IF" ] && ip link show "$WIRED_IF" 2>/dev/null | grep -q 'state UP'; then
    LINK_UP=true
fi

# Distributed needs this host to be Jetson 1 on the wired link.
if [ "$SINGLE_HOST" = false ]; then
    if [ "$LINK_UP" != true ]; then
        die "wired car link ($WIRED_NET*) not up on this host. Use --single-host for a one-Jetson test."
    fi
    if [ "$LOCAL_WIRED_IP" != "$JETSON1_IP" ]; then
        die "distributed run starts from Jetson 1 ($JETSON1_IP); this host is $LOCAL_WIRED_IP. Run here with --single-host, or run on Jetson 1."
    fi
fi

# ----------------------------------------------------------------------------
# Remote helpers (never used in dry-run or single-host)
# ----------------------------------------------------------------------------
REMOTE=$REMOTE_USER@$JETSON2_IP
ssh_opts=(-o BatchMode=yes -o ConnectTimeout=5)
remote() { ssh "${ssh_opts[@]}" "$REMOTE" "$@"; }

ROS_SETUP="source /opt/ros/humble/setup.bash && source $REPO/ros2/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=$ROS_DOMAIN CYCLONEDDS_URI=file://$CYCLONE_XML"

# ----------------------------------------------------------------------------
# Preflight
# ----------------------------------------------------------------------------
FAIL=0
check() { # check <true|false> <message>
    if [ "$1" = true ]; then info "ok    $2"; else info "FAIL  $2"; FAIL=1; fi
}
tf() { if [ "$1" = true ]; then echo true; else echo false; fi; }
skip() { info "skip  $1"; }

info "preflight"
if [ "$SINGLE_HOST" = false ]; then
    check "$LINK_UP" "wired link $WIRED_IF up ($LOCAL_WIRED_IP)"
    if [ "$DRY_RUN" = true ]; then
        skip "peer, clock, commit, image tag and lock checks on $JETSON2_IP (dry-run sends no ssh)"
    else
        rtt=$(ping -c 5 -i 0.2 -W 1 "$JETSON2_IP" 2>/dev/null | awk -F'[/ ]' '/rtt|round-trip/ {print $8}')
        rtt_ok=$(awk -v r="${rtt:-99999}" -v m="$MAX_RTT_MS" 'BEGIN {print (r+0 < m+0)}')
        check "$(tf "$([ "$rtt_ok" = 1 ] && echo true)")" "peer $JETSON2_IP ping avg ${rtt:-none} ms (< ${MAX_RTT_MS} ms)"

        local_off=$(chronyc tracking 2>/dev/null | awk -F': ' '/^System time/ {split($2, a, " "); v=a[1]*1000; if ($0 ~ /slow/) v=-v; print v}')
        remote_off=$(remote "chronyc tracking" 2>/dev/null | awk -F': ' '/^System time/ {split($2, a, " "); v=a[1]*1000; if ($0 ~ /slow/) v=-v; print v}')
        off_ok=$(awk -v a="${local_off:-99999}" -v b="${remote_off:-99999}" -v m="$MAX_OFFSET_MS" \
            'BEGIN { a=(a<0?-a:a); b=(b<0?-b:b); print (a < m && b < m) }')
        check "$(tf "$([ "$off_ok" = 1 ] && echo true)")" \
            "clock offset to chrony reference: this host ${local_off:-unknown} ms, $JETSON2_IP ${remote_off:-unknown} ms (< ${MAX_OFFSET_MS} ms)"

        local_commit=$(git -C "$REPO" rev-parse HEAD 2>/dev/null)
        remote_commit=$(remote "git -C $REPO rev-parse HEAD" 2>/dev/null)
        check "$(tf "$([ -n "$remote_commit" ] && [ "$remote_commit" = "$local_commit" ] && echo true)")" \
            "same git commit on both hosts (${local_commit:0:12} / ${remote_commit:-unreachable})"

        local_tag=${MFE_IMAGE_TAG:-unset}
        remote_tag=$(remote 'printf %s "${MFE_IMAGE_TAG:-unset}"' 2>/dev/null)
        check "$(tf "$([ "$local_tag" = "$remote_tag" ] && echo true)")" \
            "same image tag on both hosts (MFE_IMAGE_TAG: $local_tag / ${remote_tag:-unreachable})"

        remote_lock=$(remote "flock -n $LOCK_FILE true && echo free || echo busy" 2>/dev/null)
        check "$(tf "$([ "$remote_lock" = free ] && echo true)")" \
            "busy lock $LOCK_FILE free on $JETSON2_IP (${remote_lock:-unreachable})"
    fi
fi

# Local busy lock: taken for the run, only tested in dry-run.
exec 9>"$LOCK_FILE" || die "cannot open $LOCK_FILE"
if flock -n 9; then
    check true "busy lock $LOCK_FILE free on this host (taken for the run)"
else
    check false "busy lock $LOCK_FILE held on this host"
fi
if [ "$DRY_RUN" = true ]; then
    flock -u 9
fi

if [ -n "$REPLAY_BAG" ]; then
    if [ -e "$REPLAY_BAG" ]; then
        check true "replay bag $REPLAY_BAG present (sensor drivers skipped)"
    else
        check false "replay bag $REPLAY_BAG not found"
    fi
else
    for pair in lidar:velodyne_driver camera:realsense2_camera imu_gps:xsens_mti_ros2_driver; do
        sensor=${pair%%:*}
        pkg=${pair#*:}
        has_sensor "$sensor" || continue
        if [ "$DRY_RUN" = true ] && ! command -v ros2 >/dev/null 2>&1; then
            skip "driver package $pkg for $sensor (ROS not sourced, dry-run)"
        elif ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
            check true "driver package $pkg for $sensor installed"
        else
            check false "driver package $pkg for $sensor not installed"
        fi
    done
    if has_sensor lidar; then
        if [ "$DRY_RUN" = true ]; then
            skip "LiDAR UDP $LIDAR_PORT check (dry-run)"
        else
            streaming=$(timeout 4 python3 -c "
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('', $LIDAR_PORT)); s.settimeout(3)
try:
    data, addr = s.recvfrom(2048)
    print('yes' if addr[0] == '$LIDAR_IP' else 'other ' + addr[0])
except socket.timeout:
    print('no packets')
" 2>/dev/null)
            check "$(tf "$([ "$streaming" = yes ] && echo true)")" \
                "LiDAR UDP $LIDAR_PORT from $LIDAR_IP streaming (${streaming:-error})"
        fi
    fi
fi

if [ "$FAIL" = 1 ]; then
    if [ "$DRY_RUN" = true ]; then
        info "dry-run: preflight failures above would stop a real run"
    else
        die "preflight failed; nothing started"
    fi
fi

# ----------------------------------------------------------------------------
# Commands
# ----------------------------------------------------------------------------
DRIVERS=true
[ -n "$REPLAY_BAG" ] && DRIVERS=false
COMMON="platform:=car mission:=$MISSION sensors:=$SENSORS num_laps:=$LAPS endless:=$ENDLESS start_drivers:=$DRIVERS"
LAUNCH="ros2 launch mfe_bringup car.launch.py"

if [ "$SINGLE_HOST" = true ]; then
    LOCAL_CMD="$LAUNCH part:=all $COMMON"
    REMOTE_CMD=""
else
    LOCAL_CMD="$LAUNCH part:=sensing $COMMON"
    REMOTE_CMD="$LAUNCH part:=estimation $COMMON"
fi
PLAY_CMD=""
[ -n "$REPLAY_BAG" ] && PLAY_CMD="ros2 bag play $REPLAY_BAG"

info "commands"
[ -n "$PLAY_CMD" ] && echo "  [this host] $PLAY_CMD"
echo "  [this host] $LOCAL_CMD"
[ -n "$REMOTE_CMD" ] && echo "  [$JETSON2_IP] $REMOTE_CMD"

if [ "$DRY_RUN" = true ]; then
    info "dry run: nothing started"
    exit 0
fi

# ----------------------------------------------------------------------------
# Start. Each half holds its busy lock while it runs.
# ----------------------------------------------------------------------------
REMOTE_PID=
PLAY_PID=
LOCAL_PID=
cleanup() {
    trap - EXIT INT TERM
    [ -n "$LOCAL_PID" ] && kill "$LOCAL_PID" 2>/dev/null
    [ -n "$PLAY_PID" ] && kill "$PLAY_PID" 2>/dev/null
    [ -n "$REMOTE_PID" ] && kill "$REMOTE_PID" 2>/dev/null
    wait 2>/dev/null
    flock -u 9 2>/dev/null
    info "stopped; busy lock released on this host"
}
trap cleanup EXIT INT TERM

if [ -n "$REMOTE_CMD" ]; then
    REMOTE_SCRIPT="$ROS_SETUP && cd $REPO && $REMOTE_CMD"
    info "starting estimation half on $JETSON2_IP"
    ssh -t "${ssh_opts[@]}" "$REMOTE" \
        "bash -lc $(printf %q "flock -n $LOCK_FILE bash -c $(printf %q "$REMOTE_SCRIPT")")" &
    REMOTE_PID=$!
fi

LOCAL_SCRIPT="$ROS_SETUP && cd $REPO && $LOCAL_CMD"
info "starting sensing half on this host (Ctrl+C stops everything)"
bash -lc "$LOCAL_SCRIPT" &
LOCAL_PID=$!
if [ -n "$PLAY_CMD" ]; then
    info "playing $REPLAY_BAG"
    bash -lc "$ROS_SETUP && $PLAY_CMD" &
    PLAY_PID=$!
fi
wait "$LOCAL_PID"
