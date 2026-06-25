#!/usr/bin/env bash
# start_calibration.sh -- one command to run the guided C920->base calibration.
#
#   ./calibration/start_calibration.sh           # interactive guided calibration
#   ./calibration/start_calibration.sh --test-detect calibration/captures/pt5b_annot.jpg
#
# Sets up the ROS/container environment and runs calibration/calibrate_guided.py.
# Works on the HOST (docker-execs into the container) or already INSIDE
# multipanda-container. You move the arm with Franka native guiding; the tool
# only detects the socket + reads the EE pose (no robot motion).
#
# Env overrides: ROBO67_CONTAINER, ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY.
set -euo pipefail

CONTAINER="${ROBO67_CONTAINER:-multipanda-container}"
CAL="/host/Code/Robo67/calibration/calibrate_guided.py"
DOMAIN="${ROS_DOMAIN_ID:-1}"
# The real bringup runs localhost-only; the client MUST match or FrankaState
# never arrives and controller_manager service calls hang.
LOCALHOST="${ROS_LOCALHOST_ONLY:-1}"

banner() {
  cat <<'EOF'

============================================================
  Robo67 — guided C920 -> base calibration (socket-proxy)
============================================================
  Preconditions:
   - FCI active on Desk; cartesian impedance controller active
   - ONLY the socket cube in the camera view (hole up), e-stop in hand

  Per point (>= 4, spread out):
   1) Place socket (arm CLEAR) -> Enter   (detects the white-cube centroid)
   2) Guide the peg into that bore        -> Enter   (reads the EE pose)
   Type q to finish -> fits + saves config/c920_homography.npz + prints RMS.

  Move the arm ONLY with Franka native guiding (grip handles). If FrankaState
  goes missing, the bringup crashed -- relaunch it (see MANUAL_CALIBRATION.md).
============================================================

EOF
}

if [ -f /.dockerenv ] || [ "${ROBO67_IN_CONTAINER:-0}" = "1" ]; then
  banner
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source /home/developer/multipanda_ws/install/setup.bash
  export ROS_DOMAIN_ID="$DOMAIN" ROS_LOCALHOST_ONLY="$LOCALHOST"
  export LD_LIBRARY_PATH="/home/developer/Libraries/libfranka/lib:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="/host/Code/Robo67/robo67_insertion:${PYTHONPATH:-}"
  exec python3 "$CAL" "$@"
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "ERROR: docker not found and not inside the container." >&2
  echo "  Inside the container run: ROBO67_IN_CONTAINER=1 $CAL $*" >&2
  exit 1
fi
if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "ERROR: container '$CONTAINER' is not running." >&2
  exit 1
fi

banner
echo "[start_calibration] entering '$CONTAINER' (DOMAIN=$DOMAIN LOCALHOST_ONLY=$LOCALHOST)"
exec docker exec -it -e ROS_DOMAIN_ID="$DOMAIN" -e ROS_LOCALHOST_ONLY="$LOCALHOST" \
  "$CONTAINER" bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/developer/multipanda_ws/install/setup.bash
    export LD_LIBRARY_PATH="/home/developer/Libraries/libfranka/lib:${LD_LIBRARY_PATH:-}"
    export PYTHONPATH="/host/Code/Robo67/robo67_insertion:${PYTHONPATH:-}"
    exec python3 '"$CAL"' "$@"
  ' _ "$@"
