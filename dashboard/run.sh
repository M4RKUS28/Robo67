#!/usr/bin/env bash
# Robo67 insertion dashboard launcher.
#
#   ./dashboard/run.sh              # mock mode, single process on :8088 (builds web if needed)
#   ./dashboard/run.sh --mode live  # passive ROS observer (run INSIDE multipanda-container)
#   ./dashboard/run.sh --dev        # Vite dev server (HMR) on :5173 + mock backend on :8088
#
# Any extra args after the recognised flags are forwarded to serve.py
# (e.g. --port 9000 --host 0.0.0.0 --c920-device 0).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB="$HERE/web"
SERVER="$HERE/server/serve.py"
MODE="mock"
DEV=0
PASS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode) MODE="$2"; shift 2 ;;
    --dev) DEV=1; shift ;;
    *) PASS+=("$1"); shift ;;
  esac
done

if [[ "$DEV" == "1" ]]; then
  echo "[run] dev mode: Vite (:5173) -> backend (:8088)"
  ( cd "$WEB" && [[ -d node_modules ]] || npm install --no-audit --no-fund )
  python3 -u "$SERVER" --mode "$MODE" --port 8088 "${PASS[@]}" &
  BACK=$!
  trap 'kill $BACK 2>/dev/null || true' EXIT
  ( cd "$WEB" && npm run dev )
  exit 0
fi

# single-process: ensure the SPA is built, then serve everything from :8088
if [[ ! -f "$WEB/dist/index.html" ]]; then
  echo "[run] building frontend (dist/ missing) ..."
  ( cd "$WEB" && { [[ -d node_modules ]] || npm install --no-audit --no-fund; } && npm run build )
fi

echo "[run] starting backend (mode=$MODE) -> http://127.0.0.1:8088"
exec python3 -u "$SERVER" --mode "$MODE" "${PASS[@]}"
