#!/usr/bin/env bash
# run_live.sh — start the Robo67 insertion dashboard in LIVE mode.
#
# Ensures robo67_insertion is built in the container's ROS workspace (one-time,
# cached), then starts the logging graph (camera feeds + detection overlays) as
# a background docker exec and the passive ROS-observer dashboard in the
# foreground.  The logging graph is killed automatically when the dashboard
# exits (Ctrl-C or otherwise).
#
#   ./run_live.sh                          # defaults: socket_top_z=0.127, gripper=true
#   ./run_live.sh --socket-top-z 0.42     # measured socket Z in base frame (m)
#   ./run_live.sh --no-gripper            # disable D405 gripper feeds
#   ./run_live.sh --port 9000             # extra args forwarded to serve.py
#   CONTAINER=other ./run_live.sh         # override the container name
#   DASH_HOST=127.0.0.1 DASH_PORT=8088 ./run_live.sh
#
# The logging graph only OBSERVES — it never commands the arm.
set -euo pipefail

CONTAINER="${CONTAINER:-multipanda-container}"
DASH_HOST="${DASH_HOST:-0.0.0.0}"
DASH_PORT="${DASH_PORT:-8088}"
REPO="${REPO:-/host/Code/Robo67}"   # repo path as seen INSIDE the container
ROBO67_WS="${ROBO67_WS:-/home/developer/robo67_ws}"  # built workspace (container)

# --- parse our own args (rest forwarded to serve.py) ------------------------
SOCKET_TOP_Z="0.127"
GRIPPER="true"
SERVE_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --socket-top-z)   SOCKET_TOP_Z="$2"; shift 2 ;;
    --socket-top-z=*) SOCKET_TOP_Z="${1#*=}"; shift ;;
    --gripper)        GRIPPER="true"; shift ;;
    --no-gripper)     GRIPPER="false"; shift ;;
    *)                SERVE_ARGS+=("$1"); shift ;;
  esac
done

# --- preflight: container must be running -----------------------------------
if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "[run_live] ERROR: container '$CONTAINER' is not running." >&2
  echo "[run_live] running containers:" >&2
  docker ps --format '             {{.Names}}  ({{.Image}})' >&2
  exit 1
fi

# --- preflight: port free on the host (host networking => same namespace) ----
if ss -ltn "( sport = :$DASH_PORT )" 2>/dev/null | grep -q LISTEN; then
  echo "[run_live] ERROR: port $DASH_PORT is already in use on the host." >&2
  echo "[run_live] free it first, e.g.:  fuser -k ${DASH_PORT}/tcp" >&2
  exit 1
fi

# --- one-time build: register robo67_insertion with ament -------------------
# ros2 launch resolves Node(package=...) through the ament index; the package
# must be built with colcon for its executables to be findable.  We build once
# into ROBO67_WS with --symlink-install so source edits don't need a rebuild.
SENTINEL="$ROBO67_WS/install/robo67_insertion/lib/robo67_insertion/camera_publisher"
if ! docker exec "$CONTAINER" test -f "$SENTINEL" 2>/dev/null; then
  echo "[run_live] building robo67_insertion into $ROBO67_WS (one-time)..."
  docker exec "$CONTAINER" bash -lc "
    set -e
    source /opt/ros/humble/setup.bash
    source /home/developer/multipanda_ws/install/setup.bash
    mkdir -p '$ROBO67_WS/src'
    ln -sfn '$REPO/robo67_insertion' '$ROBO67_WS/src/robo67_insertion'
    cd '$ROBO67_WS'
    colcon build --symlink-install --packages-select robo67_insertion
  "
  echo "[run_live] build complete."
fi

# attach a TTY to the dashboard exec only when we actually have one
TTY=()
if [ -t 0 ] && [ -t 1 ]; then TTY=(-t); fi

# shared ROS environment (sourced inside the container)
read -r -d '' ROS_ENV <<'ROSENV' || true
  source /opt/ros/humble/setup.bash
  source /home/developer/multipanda_ws/install/setup.bash
  source "$ROBO67_WS/install/setup.bash"
  export ROS_DOMAIN_ID=1
  export ROS_LOCALHOST_ONLY=1
  export LD_LIBRARY_PATH=/home/developer/Libraries/libfranka/lib:${LD_LIBRARY_PATH:-}
  export PYTHONPATH="$REPO/robo67_insertion:${PYTHONPATH:-}"
ROSENV

echo "[run_live] $CONTAINER -> logging graph (socket_top_z=${SOCKET_TOP_Z}, gripper=${GRIPPER})"
echo "[run_live] $CONTAINER -> live dashboard on http://127.0.0.1:$DASH_PORT (Ctrl-C to stop)"

# Abort every logging-graph process (launch parent + spawned camera/detector
# nodes), escalating SIGTERM -> SIGKILL and only returning once they are gone.
#
# Two subtleties this guards against:
#  * SELF-MATCH: the kill pattern must NOT appear in this helper's own command
#    line. The container shares the host PID namespace, so `pkill -f` sees the
#    docker-exec/shell running it; if the pattern were in argv/env, pkill would
#    kill its own cleanup shell before reaching the actual nodes -- which is why
#    logging nodes kept surviving across runs. So we feed the pattern over STDIN
#    (a quoted heredoc to `bash -s`); pkill/pgrep already skip their own PIDs.
#  * STUCK NODES: a camera_publisher blocked in cv2.read() on a hung V4L2 device
#    (the D405 throws select() timeouts) can't run its Python SIGTERM handler
#    until the syscall returns, so plain SIGTERM leaves it alive. We wait briefly
#    for a clean exit, then SIGKILL (uncatchable) any survivor.
# The pattern matches the INSTALLED console-script names
# (lib/robo67_insertion/<entry_point>), NOT the *_node source filenames.
_kill_logging() {
  docker exec -i "$CONTAINER" bash -s >/dev/null 2>&1 <<'KILL' || true
pat='logging.launch.py|lib/robo67_insertion/(camera_publisher|socket_detector|d405_servo)'
pkill -TERM -f "$pat" || true
for _ in $(seq 1 10); do
  pgrep -f "$pat" >/dev/null 2>&1 || exit 0
  sleep 0.2
done
pkill -KILL -f "$pat" || true
KILL
}

# clean up any stale nodes from a previous run BEFORE launching (otherwise the
# new launch would race against leftovers holding /dev/videoN). _kill_logging
# blocks until they are actually gone, so the new graph starts from a clean slate.
_kill_logging

# --- start logging graph in background (separate docker exec) ---------------
docker exec -i \
  -e ROBO67_WS="$ROBO67_WS" -e REPO="$REPO" \
  -e SOCKET_TOP_Z="$SOCKET_TOP_Z" -e GRIPPER="$GRIPPER" \
  "$CONTAINER" bash -lc "
    $ROS_ENV
    exec ros2 launch robo67_insertion logging.launch.py \
      socket_top_z:=\"\$SOCKET_TOP_Z\" gripper:=\"\$GRIPPER\"
  " </dev/null &
LOGGING_PID=$!

cleanup() {
  trap - INT TERM EXIT   # avoid re-entrancy
  _kill_logging
  kill "$LOGGING_PID" 2>/dev/null || true
  wait "$LOGGING_PID" 2>/dev/null || true
}
# fire on Ctrl-C (INT) and TERM too, not just normal EXIT, so the in-container
# logging graph is always torn down (ros2 launch children don't die on their own).
trap cleanup INT TERM EXIT

# --- run dashboard in foreground --------------------------------------------
docker exec -i "${TTY[@]}" \
  -e ROBO67_WS="$ROBO67_WS" -e REPO="$REPO" \
  -e DASH_HOST="$DASH_HOST" -e DASH_PORT="$DASH_PORT" \
  "$CONTAINER" bash -lc "
    $ROS_ENV
    exec python3 -u \"\$REPO/dashboard/server/serve.py\" \
      --mode live --host \"\$DASH_HOST\" --port \"\$DASH_PORT\" \"\$@\"
  " bash "${SERVE_ARGS[@]}"
