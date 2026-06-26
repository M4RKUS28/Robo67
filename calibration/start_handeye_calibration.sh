#!/usr/bin/env bash
# start_handeye_calibration.sh -- one command for the guided D405 (gripper cam)
# eye-in-hand (hand-to-eye) ChArUco calibration.
#
#   ./calibration/start_handeye_calibration.sh --square-length 0.025
#   ./calibration/start_handeye_calibration.sh --selftest
#   ./calibration/start_handeye_calibration.sh --test-detect some_board.jpg
#
# Sets up the ROS/container environment and runs calibration/calibrate_handeye.py.
# Works on the HOST (docker-execs into the container) or already INSIDE
# multipanda-container. You move the arm with Franka native guiding; the tool
# only detects the ChArUco board (D405 via pyrealsense2) + reads the EE pose
# (no robot motion). Needs pyrealsense2 + a NumPy-2 OpenCV in the container --
# see robo67_insertion/scripts/container_setup.sh.
#
# Env overrides: ROBO67_CONTAINER, ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY.
set -euo pipefail

CONTAINER="${ROBO67_CONTAINER:-multipanda-container}"
CAL="/host/Code/Robo67/calibration/calibrate_handeye.py"
DOMAIN="${ROS_DOMAIN_ID:-1}"
# The real bringup runs localhost-only; the client MUST match or FrankaState
# never arrives.
LOCALHOST="${ROS_LOCALHOST_ONLY:-1}"

banner() {
  cat <<'EOF'

============================================================
  Robo67 — guided D405 eye-in-hand (hand-to-eye) calibration
============================================================
  Preconditions:
   - FCI active on Desk; the D405 is plugged in and visible
   - A ChArUco board displayed/printed, WHOLE board in the D405 view
   - Pass the REAL square size: --square-length <meters> (default 0.025)

  Per view (>= 8, VARY tilt/yaw a lot between views):
   - Guide the arm so the D405 sees the whole board -> Enter (captures pose)
   Type q to finish -> solves + saves config/d405_handeye.npz + prints residual.

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
echo "[start_handeye_calibration] entering '$CONTAINER' (DOMAIN=$DOMAIN LOCALHOST_ONLY=$LOCALHOST)"
exec docker exec -it -e ROS_DOMAIN_ID="$DOMAIN" -e ROS_LOCALHOST_ONLY="$LOCALHOST" \
  "$CONTAINER" bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/developer/multipanda_ws/install/setup.bash
    export LD_LIBRARY_PATH="/home/developer/Libraries/libfranka/lib:${LD_LIBRARY_PATH:-}"
    export PYTHONPATH="/host/Code/Robo67/robo67_insertion:${PYTHONPATH:-}"
    exec python3 '"$CAL"' "$@"
  ' _ "$@"
