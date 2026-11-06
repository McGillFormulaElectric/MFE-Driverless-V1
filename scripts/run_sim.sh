#!/bin/bash
# =============================================================================
# MFE Driverless — run the EUFS simulation with the car stack (platform:=sim).
#
# Desktop script for the sim container. It runs scripts/launch_sim.sh, which opens the tmux
# session (Gazebo, sim bridge, stack, Foxglove), and passes platform:=sim and the sensor list.
#
# Usage:
#   scripts/run_sim.sh [--mission M | --track T] [--laps N] [--sensors LIST] [--headless]
#   scripts/run_sim.sh --help
# =============================================================================

REPO=$(cd "$(dirname "$0")/.." && pwd)

MISSION=autocross
TRACK=
LAPS=1
SENSORS=lidar,camera,imu_gps,can
GUI=gui

usage() {
    cat <<USAGE
Usage: scripts/run_sim.sh [options]

  --mission M     accel | acceleration | skidpad | peanut | autocross            [autocross]
  --track T       EUFS track name (small_track, rectangle, garden_light, boa_constrictor,
                  comp_2021, hairpins, its_a_mess, ...). Overrides --mission.
  --laps N        laps before the finish gate stops the car; 0 = endless        [1]
  --sensors LIST  sensors to emulate: lidar,camera,imu_gps,can                  [all]
                  Absent sensors are not emulated, and the stack runs without them.
  --headless      no Gazebo GUI (gazebo_gui:=false)
  --help          this text

The tmux session is named 'mfe'. Stop it with: tmux kill-session -t mfe
USAGE
}

die() { echo "[run_sim] ERROR: $*" >&2; exit 1; }

while [ $# -gt 0 ]; do
    case "$1" in
        --mission)  MISSION=${2:?--mission needs a value}; shift 2 ;;
        --track)    TRACK=${2:?--track needs a value}; shift 2 ;;
        --laps)     LAPS=${2:?--laps needs a value}; shift 2 ;;
        --sensors)  SENSORS=${2:?--sensors needs a value}; shift 2 ;;
        --headless) GUI=nogui; shift ;;
        --help|-h)  usage; exit 0 ;;
        *)          echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

case "$LAPS" in ''|*[!0-9]*) die "--laps must be a whole number" ;; esac
for s in ${SENSORS//,/ }; do
    case "$s" in lidar|camera|imu_gps|can) ;; *) die "unknown sensor '$s'" ;; esac
done

case "$MISSION" in
    accel|acceleration) EVENT=accel ;;
    autocross|skidpad|peanut) EVENT=$MISSION ;;
    *) die "unknown mission '$MISSION' (accel, acceleration, skidpad, peanut, autocross)" ;;
esac
[ -n "$TRACK" ] && EVENT=$TRACK

export MFE_SIM_ARGS="platform:=sim sensors:=$SENSORS"
exec bash "$REPO/scripts/launch_sim.sh" "$EVENT" perception "$GUI" "$LAPS"
