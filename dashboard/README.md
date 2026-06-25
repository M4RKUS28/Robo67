# Robo67 Insertion Dashboard

A self-hosted, real-time web dashboard for the Franka peg-in-hole insertion. It
shows **what the robot is doing and why**: the live phase state machine, EE
**speed** and contact **force (N)** graphs, both camera feeds (overhead **C920**
+ eye-in-hand **D405**) with **hole-detection** and **servo-vector** overlays,
the top-down alignment view, and a timestamped **decision log**.

It runs in two modes behind one API:

| mode   | data source                                                                 | where it runs |
|--------|------------------------------------------------------------------------------|---------------|
| `mock` | a synthetic insertion driven by the **real** `insertion_intent` seam + a virtual plant; cameras are the saved `robo67_insertion/captures/*.jpg` | anywhere (no ROS / no arm) |
| `live` | a passive ROS observer (`FrankaState` + detection topics) + GStreamer camera grabs | inside `multipanda-container` |

```
dashboard/
  server/        Python bridge (stdlib only + numpy): SSE telemetry + MJPEG cameras
    serve.py       HTTP server, routing, static SPA host
    common.py      hub, phase metadata, SSE helpers
    mock_provider.py   synthetic insertion + capture frames
    live_provider.py   rclpy observer + GStreamer grabbers (rclpy imported lazily)
  web/           Vite + React + TS + Tailwind + TanStack (Query/Router/Table) + visx/d3
  run.sh         launcher
```

## Quick start (mock — no robot needed)

```bash
./dashboard/run.sh            # builds the SPA if needed, serves it + the mock API on :8088
# open http://127.0.0.1:8088
```

The mock insertion loops forever (MOVE_ABOVE → DESCEND → SPIRAL → PUSH → CONFIRM
→ RETRACT → DONE, with a fresh random misalignment each cycle), so the graphs,
overlays, and decision log are always populated.

### Dev mode (hot reload)

```bash
./dashboard/run.sh --dev      # Vite HMR on :5173, proxying /api -> mock backend on :8088
# open http://127.0.0.1:5173
```

Equivalent manual steps:

```bash
python3 dashboard/server/serve.py --mode mock --port 8088     # terminal 1
cd dashboard/web && npm install && npm run dev                 # terminal 2
```

## Live mode (real arm)

Run the bridge **inside `multipanda-container`** with ROS sourced (see the
project `CLAUDE.md` runbook). It only *observes* — it never commands the arm.

```bash
# inside the container, ROS + ws sourced, domain 1
export PYTHONPATH=/host/Code/Robo67/robo67_insertion:$PYTHONPATH
python3 -u /host/Code/Robo67/dashboard/server/serve.py \
    --mode live --host 0.0.0.0 --port 8088
# then build the SPA once (on the host) and open http://<host>:8088,
# or use the host Vite dev server pointed at the container.
```

Subscribed topics:

- `/franka_robot_state_broadcaster/robot_state` → EE pose, external wrench (force **N**, `Fz`), `robot_mode`; EE **speed** is derived from successive poses.
- `/robo67/socket_detection` (`[u,v,r,score]`) → **C920** hole marker.
- `/robo67/socket_pose` → socket base XY/Z.
- `/robo67/servo_correction` (`[dx,dy]`) → **D405** eye-in-hand servo vector.
- `/robo67/insertion_phase` (`std_msgs/String`, **optional**) → the FSM phase.

Camera device numbers default to the values in `robo67_insertion/config/robo67.yaml`
(`camera.c920_device`, `camera.d405_color_device`); override with
`--c920-device N` / `--d405-device N`.

### Showing the real FSM phase in live mode (optional)

The orchestrator currently only *logs* phase transitions, so live mode shows the
coarse `robot_mode` (Idle/Move/Reflex) until the phase is published. To surface
the full decision phase, publish it from whichever runner drives the arm
(`insertion_orchestrator_node` or `hardware_insertion_node`) — additive, does not
touch the control law:

```python
from std_msgs.msg import String
# once, in __init__/setup:
self._phase_pub = self.create_publisher(String, "/robo67/insertion_phase", 10)
# on every transition (where the phase string changes):
self._phase_pub.publish(String(data=new_phase))   # e.g. "SEARCH_SPIRAL"
```

The dashboard subscribes to this topic automatically and degrades gracefully if
it is absent.

## HTTP API

| endpoint | description |
|----------|-------------|
| `GET /api/health` | mode, ROS/camera availability |
| `GET /api/config` | phase list, thresholds (contact/abort N, speed cap), camera metadata |
| `GET /api/stream` | **SSE** telemetry, one JSON snapshot per tick (see `web/src/api/types.ts`) |
| `GET /api/cam/c920`, `/api/cam/d405` | **MJPEG** camera streams |
| `GET /api/cam/<name>/jpg` | single JPEG snapshot |

Telemetry is pushed over plain Server-Sent Events and cameras over MJPEG, so the
backend needs **no** extra Python packages (no FastAPI/uvicorn/websockets) and
runs unchanged on the host or in the container.

## Notes

- The mock camera feeds are whatever `c920_*.jpg` / `d405_*.jpg` stills exist in
  `robo67_insertion/captures/` at startup; the detection marker is computed from
  that still with the real `detect_holes`.
- Charts use a fixed force/speed domain (contact 5 N, abort 25 N, speed cap from
  config) so the decision thresholds are always legible.
