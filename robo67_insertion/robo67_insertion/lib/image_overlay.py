"""Detection overlay drawing for the camera logging feeds.

The dedicated ``camera_publisher`` node owns each ``/dev/videoN`` and publishes
the *raw* JPEG feed; the detector nodes subscribe to that feed, run detection,
and publish an *overlay* feed (the same frame annotated with where the socket /
hole is). This module holds the pure drawing logic so it can be unit-tested on
any host (numpy + OpenCV + stdlib only -- like :mod:`hole_detect`, it must NOT
import rclpy/ROS).

Every function returns a NEW annotated BGR array; the input is never mutated.
Passing an empty detection list returns an unmodified copy, so a node can always
publish *something* (the live feed never goes blank just because detection
missed a frame).
"""
from __future__ import annotations

from typing import Optional, Sequence, Tuple

import cv2
import numpy as np

from robo67_insertion.lib.hole_detect import Hole

__all__ = [
    "draw_socket_overlay",
    "draw_servo_overlay",
    "encode_jpeg",
    "decode_jpeg",
]

# BGR colours (OpenCV order).
_GREEN = (80, 220, 90)
_AMBER = (40, 170, 250)
_FAINT = (90, 90, 90)


def _as_bgr(img: np.ndarray) -> np.ndarray:
    """Return a 3-channel BGR copy (accepts grayscale or BGR uint8)."""
    if img.ndim == 2:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img.copy()


def _put_label(img: np.ndarray, text: str, org: Tuple[int, int],
               color: Tuple[int, int, int]) -> None:
    """Draw label text with a dark outline so it reads on any background."""
    x, y = int(org[0]), max(12, int(org[1]))
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3,
                cv2.LINE_AA)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
                cv2.LINE_AA)


def draw_socket_overlay(
    bgr: np.ndarray,
    holes: Sequence[Hole],
    *,
    color: Tuple[int, int, int] = _GREEN,
    label: str = "socket",
    base_xy: Optional[Tuple[float, float]] = None,
) -> np.ndarray:
    """Annotate an overhead frame with the detected socket(s).

    The best (first) hole gets a full-frame crosshair, a detection ring, a
    centre dot, and a text label (score, and the mapped base XY when known).
    Any further holes get a thin secondary ring. Returns a new BGR image.
    """
    out = _as_bgr(bgr)
    if not holes:
        return out
    h, w = out.shape[:2]

    # secondary detections first (so the primary draws on top)
    for hole in holes[1:]:
        cv2.circle(out, (int(hole.u), int(hole.v)),
                   max(3, int(hole.radius_px)), _FAINT, 1, cv2.LINE_AA)

    best = holes[0]
    u, v, r = int(best.u), int(best.v), max(4, int(best.radius_px))
    # faint full-frame crosshair through the detection
    cv2.line(out, (u, 0), (u, h), color, 1, cv2.LINE_AA)
    cv2.line(out, (0, v), (w, v), color, 1, cv2.LINE_AA)
    # detection ring + centre dot
    cv2.circle(out, (u, v), r, color, 2, cv2.LINE_AA)
    cv2.circle(out, (u, v), 2, color, -1, cv2.LINE_AA)

    txt = f"{label} {best.score:.2f}"
    if base_xy is not None:
        txt += f"  base ({base_xy[0]:+.3f}, {base_xy[1]:+.3f})"
    _put_label(out, txt, (u + r + 6, v - r - 6), color)
    return out


def draw_servo_overlay(
    bgr: np.ndarray,
    holes: Sequence[Hole],
    *,
    servo_dxy_m: Optional[Tuple[float, float]] = None,
    color: Tuple[int, int, int] = _AMBER,
) -> np.ndarray:
    """Annotate a gripper (eye-in-hand) frame with the hole + servo arrow.

    Draws the image-centre crosshair (the tool axis), a ring on the detected
    hole, and an arrow from the centre toward the hole (the direction the tool
    should move). ``servo_dxy_m`` (base-frame metres) is shown as a magnitude
    label when provided. Returns a new BGR image.
    """
    out = _as_bgr(bgr)
    h, w = out.shape[:2]
    cx, cy = w // 2, h // 2
    # tool-axis crosshair at image centre
    cv2.drawMarker(out, (cx, cy), color, cv2.MARKER_CROSS, 18, 1, cv2.LINE_AA)
    if not holes:
        return out
    best = holes[0]
    u, v, r = int(best.u), int(best.v), max(4, int(best.radius_px))
    cv2.circle(out, (u, v), r, color, 2, cv2.LINE_AA)
    if (u, v) != (cx, cy):
        cv2.arrowedLine(out, (cx, cy), (u, v), color, 2, cv2.LINE_AA,
                        tipLength=0.2)
    if servo_dxy_m is not None:
        mag_mm = float(np.hypot(servo_dxy_m[0], servo_dxy_m[1])) * 1000.0
        _put_label(out, f"servo {mag_mm:.1f} mm", (u + r + 6, v - r - 6), color)
    return out


def encode_jpeg(bgr: np.ndarray, quality: int = 80) -> Optional[bytes]:
    """Encode a BGR frame to JPEG bytes (for sensor_msgs/CompressedImage)."""
    quality = int(max(1, min(100, quality)))
    ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return buf.tobytes() if ok else None


def decode_jpeg(data: bytes) -> Optional[np.ndarray]:
    """Decode JPEG bytes (a CompressedImage payload) to a BGR ndarray."""
    if not data:
        return None
    arr = np.frombuffer(bytes(data), dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    return img
