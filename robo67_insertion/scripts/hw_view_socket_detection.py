#!/usr/bin/env python3
"""hw_view_socket_detection.py -- live C920 feed with socket detection overlay.

Opens the overhead Logitech C920, runs detect_sockets on every frame, and draws
the result in a live OpenCV window. No ROS required.

Controls
--------
  q / ESC  quit
  s        save annotated snapshot to captures/detection_snap_<N>.jpg
  d        toggle debug overlay (shows ALL HoughCircles candidates, even rejected)
  +/-      increase/decrease exposure by 10 (manual exposure mode)

USAGE (no container needed -- pure Python + OpenCV):
    PYTHONPATH=/home/minga-08/Code/Robo67/robo67_insertion \
    python3 scripts/hw_view_socket_detection.py

Override camera / exposure:
    python3 scripts/hw_view_socket_detection.py --device /dev/video8 --exposure 80
"""
from __future__ import annotations

import argparse
import os
import sys
import time

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import cv2
import numpy as np

from robo67_insertion.lib.hole_detect import (
    HoleParams, WhiteCubeParams, WhiteSocketParams,
    detect_holes, detect_sockets, detect_white_cubes,
)

_DETECTORS = {
    "cube":   (detect_white_cubes, WhiteCubeParams),
    "socket": (detect_sockets,     WhiteSocketParams),
    "hole":   (detect_holes,       HoleParams),
}

DEFAULT_CONFIG = os.path.join(_PKG_ROOT, "config", "robo67.yaml")
SNAP_DIR = os.path.join(_PKG_ROOT, "captures")


# ---------------------------------------------------------------------------
# Annotate a frame with detection results.
# ---------------------------------------------------------------------------

def _draw_hole(out, hole, color_main, color_dim):
    """Draw a circle detection."""
    u, v, r = int(hole.u), int(hole.v), int(hole.radius_px)
    cv2.circle(out, (u, v), r, color_main, 2)
    arm = max(6, r // 3)
    cv2.line(out, (u - arm, v), (u + arm, v), color_main, 2)
    cv2.line(out, (u, v - arm), (u, v + arm), color_main, 2)
    return u, v, r


def _draw_cube(out, hole, color_main, color_dim):
    """Draw a rectangle detection (cube top)."""
    u, v, r = int(hole.u), int(hole.v), int(hole.radius_px)
    cv2.rectangle(out, (u - r, v - r), (u + r, v + r), color_main, 2)
    arm = max(6, r // 3)
    cv2.line(out, (u - arm, v), (u + arm, v), color_main, 2)
    cv2.line(out, (u, v - arm), (u, v + arm), color_main, 2)
    return u, v, r


def _draw_detection(img, holes, params, mode="cube", debug=False, debug_circles=None):
    out = img.copy()
    h, w = out.shape[:2]

    roi_margin = getattr(params, "roi_margin", 0.0)
    if roi_margin:
        mx, my = int(roi_margin * w), int(roi_margin * h)
        cv2.rectangle(out, (mx, my), (w - mx, h - my), (80, 80, 80), 1)
        cv2.putText(out, "ROI", (mx + 4, my + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 80, 80), 1)

    if debug and debug_circles is not None:
        for u, v, r in debug_circles:
            cv2.circle(out, (int(u), int(v)), int(r), (60, 60, 60), 1)

    if not holes:
        cv2.putText(out, "NO DETECTION", (12, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 220), 2)
        return out

    draw_fn = _draw_cube if mode == "cube" else _draw_hole

    best = holes[0]
    u, v, r = draw_fn(out, best, (0, 220, 0), (0, 140, 0))
    label = f"({u},{v}) r={r}px  score={best.score:.2f}"
    cv2.putText(out, label, (u - r, v - r - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 1)

    for hole in holes[1:]:
        draw_fn(out, hole, (0, 140, 0), (0, 100, 0))

    cv2.putText(out, f"[{mode}] DETECTED  {len(holes)} candidate(s)", (12, 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 0), 2)
    return out


def _draw_hud(img, fps, exposure, debug):
    out = img
    cv2.putText(out, f"FPS {fps:.1f}  exp={exposure}  d={int(debug)}",
                (8, img.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    cv2.putText(out, "q=quit  s=snap  d=debug  +/-=exposure",
                (8, img.shape[0] - 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
    return out


# ---------------------------------------------------------------------------
# Raw HoughCircles pass (to show debug candidates without filtering).
# ---------------------------------------------------------------------------

def _raw_hough(bgr, params):
    """Raw HoughCircles pass for debug overlay (only meaningful for circle detectors)."""
    if not all(hasattr(params, a) for a in ("dp", "min_dist_px", "hough_param1", "hough_param2")):
        return []
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    blur = cv2.medianBlur(gray, 5)
    circles = cv2.HoughCircles(
        blur, cv2.HOUGH_GRADIENT, dp=params.dp, minDist=params.min_dist_px,
        param1=params.hough_param1, param2=params.hough_param2,
        minRadius=int(params.min_radius_px), maxRadius=int(params.max_radius_px),
    )
    if circles is None:
        return []
    return np.round(circles[0]).astype(int).tolist()


# ---------------------------------------------------------------------------
# Camera open helpers.
# ---------------------------------------------------------------------------

def open_camera(device: str, width: int, height: int, exposure: int):
    for backend in (cv2.CAP_V4L2, cv2.CAP_ANY):
        cap = cv2.VideoCapture(device, backend)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        _apply_exposure(cap, exposure)
        # Flush stale buffered frames
        for _ in range(5):
            cap.grab()
        return cap
    return None


def _apply_exposure(cap, exposure: int):
    # Mode 1 = manual on V4L2 (some drivers use 1, some use 4=aperture-priority)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    cap.set(cv2.CAP_PROP_EXPOSURE, exposure)


# ---------------------------------------------------------------------------
# Main loop.
# ---------------------------------------------------------------------------

def run(args):
    detect_fn, params_cls = _DETECTORS[args.mode]
    params = params_cls()

    cap = open_camera(args.device, args.width, args.height, args.exposure)
    if cap is None:
        print(f"ERROR: could not open camera {args.device}", file=sys.stderr)
        return 1

    os.makedirs(SNAP_DIR, exist_ok=True)
    snap_n = 0
    debug = args.debug
    exposure = args.exposure

    print(f"Camera open: {args.device}  {args.width}x{args.height}  exposure={exposure}")
    print("Window: q/ESC=quit  s=save snapshot  d=toggle debug  +/-=exposure")

    cv2.namedWindow("socket detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("socket detection", args.width, args.height)

    t_last = time.monotonic()
    fps = 0.0

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("frame grab failed -- retrying", flush=True)
            time.sleep(0.05)
            continue

        holes = detect_fn(frame, params)
        raw_circles = _raw_hough(frame, params) if debug else None

        now = time.monotonic()
        fps = 0.9 * fps + 0.1 * (1.0 / max(1e-3, now - t_last))
        t_last = now

        vis = _draw_detection(frame, holes, params, mode=args.mode,
                              debug=debug, debug_circles=raw_circles)
        vis = _draw_hud(vis, fps, exposure, debug)

        cv2.imshow("socket detection", vis)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), 27):  # q or ESC
            break
        elif key == ord("s"):
            path = os.path.join(SNAP_DIR, f"detection_snap_{snap_n:03d}.jpg")
            cv2.imwrite(path, vis)
            print(f"saved {path}")
            snap_n += 1
        elif key == ord("d"):
            debug = not debug
            print(f"debug overlay: {debug}")
        elif key == ord("+") or key == ord("="):
            exposure = min(500, exposure + 10)
            _apply_exposure(cap, exposure)
            print(f"exposure -> {exposure}")
        elif key == ord("-"):
            exposure = max(10, exposure - 10)
            _apply_exposure(cap, exposure)
            print(f"exposure -> {exposure}")

    cap.release()
    cv2.destroyAllWindows()
    return 0


# ---------------------------------------------------------------------------
# CLI.
# ---------------------------------------------------------------------------

def build_parser():
    ap = argparse.ArgumentParser(description="Live socket detection viewer.")

    # Load config defaults lazily so --help works without the package installed
    try:
        sys.path.insert(0, _PKG_ROOT)
        from robo67_insertion.config_schema import load_config
        cfg = load_config(DEFAULT_CONFIG)
        default_device = cfg.camera.c920_device
        default_exposure = cfg.camera.c920_exposure
    except Exception:
        default_device = "/dev/video8"
        default_exposure = 100

    ap.add_argument("--device", default=default_device,
                    help=f"camera device path or index (default: {default_device})")
    ap.add_argument("--exposure", type=int, default=default_exposure,
                    help=f"initial manual exposure value (default: {default_exposure})")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--mode", choices=list(_DETECTORS), default="cube",
                    help="detector to use: cube (rectangle, default), socket (HoughCircles), hole (dark hole)")
    ap.add_argument("--debug", action="store_true",
                    help="start with debug overlay enabled")
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv if argv is not None else sys.argv[1:])
    # Resolve bare int index to /dev/videoN
    if args.device.lstrip("-").isdigit():
        args.device = f"/dev/video{args.device}"
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
