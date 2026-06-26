#!/usr/bin/env bash
# start_demo.sh -- ONE command: force-guided cable insertion from anywhere.
#
#   ./start_demo.sh                # auto-run (3-2-1 countdown, e-stop in hand)
#   ./start_demo.sh --dry-run      # perceive/plan + run the loop, publish NOTHING
#   ./start_demo.sh --confirm      # prompt YES before any motion
#   ./start_demo.sh --no-force-mode    # old fixed-equilibrium spiral
#
# Runs on the HOST and docker-execs into multipanda-container (or run INSIDE it).
# It sets up the ROS/container environment and runs the hardcoded force-guided
# seat (scripts/hw_cable_insert_fixed.py): from ANY start pose it MOVE_ABOVEs the
# taught seat point, DESCENDs to contact, force-guided SEARCH_SPIRAL (admittance
# press), then the firm PUSH_INSERT that plugs the connector home -- kept gripped.
#
# Tuning + paths are env-overridable (defaults = the params verified live
# 2026-06-26): ROBO67_CONTAINER, ROBO67_PKG, ROBO67_SEAT_XYZ="x y z",
# ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY, and SEARCH_PRESS/INSERT_PRESS/ADM_MAX_FORCE/
# INSERT_DEPTH/CONFIRM_DROP/SETTLE_S/TORQUE_ABORT/F_ABORT/WATCHDOG_S/V_MAX/STANDOFF.
set -euo pipefail

CONTAINER="${ROBO67_CONTAINER:-multipanda-container}"
# Container-side package path. This repo is mounted at /host/Code/robo67_cable_insertion
# (home -> /host). Override with ROBO67_PKG if your mount differs.
PKG="${ROBO67_PKG:-/host/Code/robo67_cable_insertion/robo67_insertion}"
SCRIPT="$PKG/scripts/hw_cable_insert_fixed.py"
DOMAIN="${ROS_DOMAIN_ID:-1}"
# The real bringup runs localhost-only; the client MUST match or FrankaState
# never arrives and the move never progresses.
LOCALHOST="${ROS_LOCALHOST_ONLY:-1}"

# --- verified force-guided seat params (override any via env) ----------------
SEARCH_PRESS="${SEARCH_PRESS:-6}"        # gentle press while finding the port
INSERT_PRESS="${INSERT_PRESS:-14}"       # firm final push that plugs it home
ADM_MAX_FORCE="${ADM_MAX_FORCE:-16}"     # soft clamp on the regulated force
INSERT_DEPTH="${INSERT_DEPTH:-0.012}"    # seat travel below contact (m)
CONFIRM_DROP="${CONFIRM_DROP:-0.002}"    # descent (m) that confirms the seat
SETTLE_S="${SETTLE_S:-0.6}"
TORQUE_ABORT="${TORQUE_ABORT:-15}"
F_ABORT="${F_ABORT:-28}"
WATCHDOG_S="${WATCHDOG_S:-0.5}"
V_MAX="${V_MAX:-0.025}"
STANDOFF="${STANDOFF:-0.06}"

ARGS=(
  --torque-abort "$TORQUE_ABORT" --f-abort "$F_ABORT" --watchdog-s "$WATCHDOG_S"
  --v-max "$V_MAX" --standoff "$STANDOFF"
  --search-press "$SEARCH_PRESS" --insert-press "$INSERT_PRESS"
  --adm-max-force "$ADM_MAX_FORCE" --insert-depth "$INSERT_DEPTH"
  --confirm-drop "$CONFIRM_DROP" --settle-s "$SETTLE_S"
)
# Optional taught-seat override: ROBO67_SEAT_XYZ="0.45 -0.09 0.21"
if [ -n "${ROBO67_SEAT_XYZ:-}" ]; then
  # shellcheck disable=SC2206
  ARGS+=( --seat-xyz ${ROBO67_SEAT_XYZ} )
fi
# Forward any extra user args (e.g. --dry-run, --confirm, --no-force-mode).
ARGS+=( "$@" )

banner() {
  cat <<'EOF'

============================================================
  Robo67 — cable insertion DEMO (force-guided, hardcoded)
============================================================
  Preconditions:
   - FCI active on Desk; cartesian impedance controller active
   - Arm in Move mode (robot_mode 2), e-stop in hand
   - LAN connector CLAMPED in the gripper, cable tail routed CLEAR
   - Box placed so the port matches the taught seat pose

  It moves above the taught seat from wherever the arm is, descends to
  contact, force-guided spiral-searches, then firmly pushes the connector
  home (kept gripped). 3-2-1 countdown before motion unless --confirm/--dry-run.
============================================================

EOF
}

# --- already inside the container --------------------------------------------
if [ -f /.dockerenv ] || [ "${ROBO67_IN_CONTAINER:-0}" = "1" ]; then
  banner
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source /home/developer/multipanda_ws/install/setup.bash
  export ROS_DOMAIN_ID="$DOMAIN" ROS_LOCALHOST_ONLY="$LOCALHOST"
  export LD_LIBRARY_PATH="/home/developer/Libraries/libfranka/lib:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="$PKG:${PYTHONPATH:-}"
  exec python3 -u "$SCRIPT" "${ARGS[@]}"
fi

# --- host: docker-exec into the container ------------------------------------
if ! command -v docker >/dev/null 2>&1; then
  echo "ERROR: docker not found and not inside the container." >&2
  echo "  Inside the container run: ROBO67_IN_CONTAINER=1 $0 $*" >&2
  exit 1
fi
if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "ERROR: container '$CONTAINER' is not running." >&2
  exit 1
fi

banner
echo "[start_demo] entering '$CONTAINER' (DOMAIN=$DOMAIN LOCALHOST_ONLY=$LOCALHOST)"
exec docker exec -it -e ROS_DOMAIN_ID="$DOMAIN" -e ROS_LOCALHOST_ONLY="$LOCALHOST" \
  "$CONTAINER" bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/developer/multipanda_ws/install/setup.bash
    export LD_LIBRARY_PATH="/home/developer/Libraries/libfranka/lib:${LD_LIBRARY_PATH:-}"
    export PYTHONPATH="'"$PKG"':${PYTHONPATH:-}"
    exec python3 -u "'"$SCRIPT"'" "$@"
  ' _ "${ARGS[@]}"
