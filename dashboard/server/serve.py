#!/usr/bin/env python3
"""Robo67 insertion dashboard backend.

A dependency-light bridge that exposes robot telemetry and the two camera feeds
to the React frontend over plain HTTP:

    GET /api/health          -> JSON status
    GET /api/config          -> phase list, thresholds, camera metadata
    GET /api/stream          -> Server-Sent Events telemetry (one JSON per tick)
    GET /api/cam/<name>      -> MJPEG stream  (name in {c920, d405})
    GET /api/cam/<name>/jpg  -> single JPEG snapshot

Two providers behind one interface (chosen with --mode):

    mock   synthetic insertion (real insertion_intent + a virtual plant) and the
           saved capture stills as camera feeds. Runs anywhere; no ROS/arm.
    live   passive ROS observer (FrankaState + detections) + GStreamer camera
           grabs. Run INSIDE multipanda-container (ROS sourced, domain 1).

If ``dashboard/web/dist`` exists it is served at ``/`` (SPA fallback), so a
single ``python3 serve.py`` hosts the whole thing on localhost.

Usage:
    python3 dashboard/server/serve.py --mode mock --port 8088
    # inside the container, after sourcing ROS:
    python3 dashboard/server/serve.py --mode live --port 8088 --host 0.0.0.0
"""
from __future__ import annotations

import argparse
import json
import os
import queue
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from common import DASHBOARD_DIR, sse_pack  # noqa: E402

WEB_DIST = os.path.join(DASHBOARD_DIR, "web", "dist")

_MIME = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".mjs": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json",
    ".svg": "image/svg+xml",
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
    ".ico": "image/x-icon",
    ".woff2": "font/woff2",
    ".map": "application/json",
}


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    # -- low-level helpers ----------------------------------------------

    def _provider(self):
        return self.server.provider  # type: ignore[attr-defined]

    def _insertion(self):
        return self.server.insertion  # type: ignore[attr-defined]

    def _bringup(self):
        return self.server.bringup  # type: ignore[attr-defined]

    def _home(self):
        return self.server.home  # type: ignore[attr-defined]

    def _fci(self):
        return self.server.fci  # type: ignore[attr-defined]

    def _gripper(self):
        return self.server.gripper  # type: ignore[attr-defined]

    def _cors(self):
        self.send_header("Access-Control-Allow-Origin", "*")

    def _send_json(self, obj, status=200):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self._cors()
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):  # quieter logs
        if os.environ.get("ROBO67_DASH_VERBOSE"):
            super().log_message(fmt, *args)

    # -- routing ---------------------------------------------------------

    def do_OPTIONS(self):
        self.send_response(204)
        self._cors()
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "*")
        self.send_header("Content-Length", "0")
        self.send_header("Connection", "close")
        self.end_headers()

    def do_POST(self):
        path = urlparse(self.path).path
        try:
            # read any request body (most actions need none; insertion start
            # accepts an optional {"force_mode": bool, "insertion_mode": "peg"|"cable"})
            n = int(self.headers.get("Content-Length", 0) or 0)
            raw = self.rfile.read(n) if n else b""
            if path == "/api/insertion/start":
                force_mode = False
                insertion_mode = "peg"
                if raw:
                    try:
                        body = json.loads(raw.decode("utf-8") or "{}")
                        force_mode = bool(body.get("force_mode", False))
                        m = str(body.get("insertion_mode", "peg")).lower()
                        insertion_mode = "cable" if m == "cable" else "peg"
                    except Exception:  # noqa: BLE001
                        force_mode = False
                        insertion_mode = "peg"
                return self._send_json(self._insertion().start(mode=insertion_mode, force_mode=force_mode))
            if path == "/api/insertion/stop":
                return self._send_json(self._insertion().stop())
            if path == "/api/bringup/relaunch":
                return self._send_json(self._bringup().relaunch())
            if path == "/api/home/run":
                return self._send_json(self._home().run())
            if path == "/api/home/stop":
                return self._send_json(self._home().stop())
            if path == "/api/fci/activate":
                return self._send_json(self._fci().activate())
            if path == "/api/fci/deactivate":
                return self._send_json(self._fci().deactivate())
            if path == "/api/gripper/open":
                return self._send_json(self._gripper().open())
            if path == "/api/gripper/close":
                return self._send_json(self._gripper().close())
            return self._send_json({"error": "not found", "path": path}, status=404)
        except (BrokenPipeError, ConnectionResetError):
            return
        except Exception as exc:  # noqa: BLE001
            try:
                self._send_json({"error": str(exc)}, status=500)
            except Exception:
                pass

    def do_GET(self):
        path = urlparse(self.path).path
        try:
            if path == "/api/health":
                return self._send_json(self._provider().health())
            if path == "/api/config":
                return self._send_json(self._provider().config())
            if path == "/api/insertion/status":
                return self._send_json(self._insertion().status())
            if path == "/api/bringup/status":
                return self._send_json(self._bringup().status())
            if path == "/api/home/status":
                return self._send_json(self._home().status())
            if path == "/api/fci/status":
                return self._send_json(self._fci().status())
            if path == "/api/gripper/status":
                return self._send_json(self._gripper().status())
            if path == "/api/stream":
                return self._stream_sse()
            if path.startswith("/api/cam/"):
                rest = path[len("/api/cam/"):]
                if rest.endswith("/jpg"):
                    return self._send_snapshot(rest[:-len("/jpg")])
                return self._stream_mjpeg(rest)
            if path.startswith("/api/"):
                return self._send_json({"error": "not found", "path": path}, status=404)
            return self._serve_static(path)
        except (BrokenPipeError, ConnectionResetError):
            return
        except Exception as exc:  # noqa: BLE001
            try:
                self._send_json({"error": str(exc)}, status=500)
            except Exception:
                pass

    # -- SSE telemetry ---------------------------------------------------

    def _stream_sse(self):
        prov = self._provider()
        q = prov.hub.subscribe()
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache, no-transform")
        self.send_header("Connection", "keep-alive")
        self._cors()
        self.end_headers()
        try:
            latest = prov.latest()
            if latest is not None:
                self.wfile.write(sse_pack(latest))
                self.wfile.flush()
            last_ping = time.time()
            while True:
                try:
                    snap = q.get(timeout=1.0)
                    self.wfile.write(sse_pack(snap))
                    self.wfile.flush()
                except queue.Empty:
                    now = time.time()
                    if now - last_ping > 5.0:
                        self.wfile.write(b": ping\n\n")
                        self.wfile.flush()
                        last_ping = now
        finally:
            prov.hub.unsubscribe(q)

    # -- camera streams --------------------------------------------------

    def _stream_mjpeg(self, name: str):
        prov = self._provider()
        if name not in prov.camera_names():
            return self._send_json({"error": "no such camera", "name": name}, status=404)
        boundary = "robo67frame"
        self.send_response(200)
        self.send_header(
            "Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
        self.send_header("Cache-Control", "no-cache, no-transform")
        self.send_header("Connection", "keep-alive")
        self._cors()
        self.end_headers()
        for frame in prov.camera_iter(name):
            if not frame:
                continue
            head = (f"--{boundary}\r\n"
                    f"Content-Type: image/jpeg\r\n"
                    f"Content-Length: {len(frame)}\r\n\r\n").encode("ascii")
            self.wfile.write(head)
            self.wfile.write(frame)
            self.wfile.write(b"\r\n")
            self.wfile.flush()

    def _send_snapshot(self, name: str):
        prov = self._provider()
        frame = prov.camera_jpeg(name)
        if not frame:
            return self._send_json({"error": "no frame", "name": name}, status=503)
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(frame)))
        self.send_header("Cache-Control", "no-cache")
        self._cors()
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(frame)

    # -- static frontend (built SPA) ------------------------------------

    def _serve_static(self, path: str):
        if not os.path.isdir(WEB_DIST):
            return self._send_json(
                {"error": "frontend not built",
                 "hint": "run `npm run build` in dashboard/web, or use the Vite dev server"},
                status=404)
        rel = path.lstrip("/") or "index.html"
        full = os.path.normpath(os.path.join(WEB_DIST, rel))
        if not full.startswith(WEB_DIST):
            return self._send_json({"error": "forbidden"}, status=403)
        if not os.path.isfile(full):
            full = os.path.join(WEB_DIST, "index.html")  # SPA fallback
            if not os.path.isfile(full):
                return self._send_json({"error": "index.html missing"}, status=404)
        ext = os.path.splitext(full)[1].lower()
        with open(full, "rb") as fh:
            body = fh.read()
        self.send_response(200)
        self.send_header("Content-Type", _MIME.get(ext, "application/octet-stream"))
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(body)


def build_provider(args):
    if args.mode == "live":
        from live_provider import LiveProvider

        return LiveProvider(c920_device=args.c920_device,
                            d405_device=args.d405_device)
    from mock_provider import MockProvider

    return MockProvider()


def main(argv=None):
    ap = argparse.ArgumentParser(description="Robo67 insertion dashboard backend")
    ap.add_argument("--mode", choices=["mock", "live"], default="mock")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8088)
    ap.add_argument("--c920-device", type=int, default=None,
                    help="override C920 /dev/video<N> (live mode)")
    ap.add_argument("--d405-device", type=int, default=None,
                    help="override D405 color /dev/video<N> (live mode)")
    args = ap.parse_args(argv)

    provider = build_provider(args)
    provider.start()

    from insertion_control import InsertionController
    from bringup_control import BringupController
    from home_control import HomeController
    from fci_control import FciController
    from gripper_control import GripperController

    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    httpd.daemon_threads = True
    httpd.provider = provider  # type: ignore[attr-defined]
    # The Start button spawns the real-arm insertion -> live mode only.
    httpd.insertion = InsertionController(enabled=(args.mode == "live"))  # type: ignore[attr-defined]
    # The Relaunch button stops + relaunches the bringup/gripper (real arm) and
    # reads robot_mode off the live provider to verify -> live mode only.
    httpd.bringup = BringupController(provider=provider,  # type: ignore[attr-defined]
                                      enabled=(args.mode == "live"))
    # The Home button holds the pose the arm is in right now -> live mode only.
    httpd.home = HomeController(enabled=(args.mode == "live"))  # type: ignore[attr-defined]
    # The FCI on/off button toggles the Franka Control Interface over the Desk
    # HTTP API (login + take control + activate/deactivate) -> live mode only.
    httpd.fci = FciController(enabled=(args.mode == "live"))  # type: ignore[attr-defined]
    # The gripper Open/Close buttons drive the franka_gripper Move/Grasp actions
    # via the ros2 CLI (needs the gripper node) -> live mode only.
    httpd.gripper = GripperController(enabled=(args.mode == "live"))  # type: ignore[attr-defined]

    web = "serving built SPA from web/dist" if os.path.isdir(WEB_DIST) else \
        "no built SPA (use Vite dev server)"
    print(f"[robo67-dashboard] mode={args.mode}  http://{args.host}:{args.port}  ({web})")
    print(f"[robo67-dashboard]   GET  /api/health  /api/config  /api/stream  "
          f"/api/cam/c920  /api/cam/d405  /api/insertion/status  /api/bringup/status  "
          f"/api/home/status  /api/fci/status  /api/gripper/status")
    print(f"[robo67-dashboard]   POST /api/insertion/start  /api/insertion/stop  "
          f"/api/bringup/relaunch  /api/home/run  /api/home/stop  /api/fci/activate  "
          f"/api/fci/deactivate  /api/gripper/open  /api/gripper/close  (live mode only)")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[robo67-dashboard] shutting down ...")
    finally:
        provider.stop()
        httpd.server_close()


if __name__ == "__main__":
    sys.exit(main())
