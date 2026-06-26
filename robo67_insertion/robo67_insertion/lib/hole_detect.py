"""White-cube socket detection for the peg-in-hole vision pipeline.

An overhead Logitech C920 looks straight down at the workspace. The REAL Robo67
socket is a small WHITE 3D-printed cube with a recessed bore, sitting on a DARK
mat/carpet. When the C920 is even slightly overexposed the white-on-white bore
is too faint to detect reliably, so we instead detect the socket as a bright
WHITE SQUARE on a dark background and return its CENTROID -- a far more robust
feature (verified live 2026-06-25; this is what the socket-proxy homography was
calibrated against). The bore sits ~centred on the cube, so the centroid is a
stable proxy for it.

This module is pure-Python (numpy + OpenCV + stdlib only). It deliberately
avoids importing rclpy/ROS/scipy so it can be unit-tested on any host.

Pipeline
--------
1. Convert to grayscale + median blur.
2. Adaptive bright threshold (``max(bright_floor, percentile(gray, bright_pct) -
   bright_drop)``) -> binary white mask; morphologically close then open it.
3. Find external contours.
4. For each contour, reject by area (the cube top is bounded under the fixed
   overhead camera), then by aspect + fill ('extent') measured against the
   **rotated** min-area rectangle (``cv2.minAreaRect``) so a socket at any
   rotation still reads as a filled ~square.
5. Return centroids (image moments) as :class:`Hole`, sorted by contour area
   descending (largest cube first).
"""
from dataclasses import dataclass

import cv2
import numpy as np

__all__ = [
    "Hole",
    "WhiteCubeParams",
    "detect_white_cubes",
]


@dataclass
class Hole:
    """A detected socket feature, in image (pixel) coordinates.

    Attributes
    ----------
    u:
        Horizontal image coordinate of the feature center, in pixels.
    v:
        Vertical image coordinate of the feature center, in pixels.
    radius_px:
        Characteristic radius in pixels (half the mean rotated-rect side).
    score:
        Detection score in ``[0, 1]``; higher is a better/cleaner detection.
    """

    u: float
    v: float
    radius_px: float
    score: float


@dataclass
class WhiteCubeParams:
    """Tunable parameters for :func:`detect_white_cubes`.

    Detects the socket as a bright WHITE SQUARE on a dark background and returns
    its CENTROID -- a far more robust feature than the faint bore when the C920
    is even slightly overexposed (verified live 2026-06-25; this is what the
    socket-proxy homography was calibrated against). The bore sits ~centred on
    the cube, so the centroid is a stable proxy for it.

    NOTE: this finds white *cubes*, not bores, so it CANNOT tell the socket from
    an identical blank cube -- keep only the socket in view. It returns the
    largest qualifying square first.

    Attributes
    ----------
    bright_pct / bright_drop / bright_floor:
        Bright threshold ``max(bright_floor, percentile(gray, bright_pct) -
        bright_drop)`` -- adapts to exposure while staying above the carpet.
    min_area_px / max_area_px:
        Accepted contour area (the cube top), in px^2. The socket is a fixed
        ~6 cm cube under a FIXED overhead camera, so its apparent area is
        bounded (~3.9k px on the C920); ``max_area_px`` is the clutter guard
        that rejects much larger white blobs (packaging boxes, devices) that
        would otherwise be returned over the socket (they are bigger, and the
        result is area-sorted). Keep it just above the socket's apparent area.
    min_aspect / max_aspect / min_extent:
        Aspect and fill ('extent') bounds measured against the **rotated**
        min-area rectangle (``cv2.minAreaRect``), NOT the axis-aligned bbox --
        so a socket placed at any ROTATION still reads as a filled ~square
        (a 45 deg square fills only ~50% of its axis-aligned bbox, which the
        old axis-aligned test wrongly rejected). Keeps filled, roughly square
        blobs (rejects the elongated/irregular arm, cables, edges).
    roi_margin:
        Ignore detections whose centroid is within this fractional image border.
    """

    bright_pct: float = 99.5
    bright_drop: float = 55.0
    bright_floor: float = 150.0
    min_area_px: float = 1200.0
    max_area_px: float = 10000.0
    min_aspect: float = 0.55
    max_aspect: float = 1.8
    min_extent: float = 0.78
    roi_margin: float = 0.08


def detect_white_cubes(bgr: np.ndarray,
                       params: WhiteCubeParams = WhiteCubeParams()) -> list[Hole]:
    """Detect bright white square cube(s) and return centroid(s) as :class:`Hole`.

    Robust detector for the socket feature when the bore is too faint to detect
    (overexposed white-on-white). Returns :class:`Hole` (``u``/``v`` = centroid,
    ``radius_px`` = half the mean rotated-rect side, ``score`` = rotated-rect
    fill 'extent'), sorted by contour AREA descending (largest cube first).

    The aspect/extent are measured against the **rotated** min-area rectangle
    (``cv2.minAreaRect``), so a socket placed at any rotation is still accepted;
    the old axis-aligned bounding box made a rotated square look non-square and
    under-filled and dropped it entirely. See :class:`WhiteCubeParams`.
    """
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)
    h, w = gray.shape
    mx0, my0 = int(params.roi_margin * w), int(params.roi_margin * h)

    thr = max(params.bright_floor, float(np.percentile(gray, params.bright_pct)) - params.bright_drop)
    white = (gray > thr).astype(np.uint8) * 255
    white = cv2.morphologyEx(white, cv2.MORPH_CLOSE,
                             cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11)))
    white = cv2.morphologyEx(white, cv2.MORPH_OPEN,
                             cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
    contours, _ = cv2.findContours(white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    out: list[tuple[Hole, float]] = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < params.min_area_px or area > params.max_area_px:
            continue
        # Rotation-invariant shape test: aspect + fill against the min-area
        # (rotated) rectangle, so a rotated socket still reads as a filled square.
        (rw, rh) = cv2.minAreaRect(c)[1]
        if rw <= 0.0 or rh <= 0.0:
            continue
        aspect = rw / float(rh)
        extent = area / float(rw * rh)
        if not (params.min_aspect < aspect < params.max_aspect) or extent < params.min_extent:
            continue
        m = cv2.moments(c)
        if m["m00"] == 0:
            continue
        cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
        if not (mx0 < cx < w - mx0 and my0 < cy < h - my0):
            continue
        hole = Hole(u=float(cx), v=float(cy), radius_px=float((rw + rh) / 4.0),
                    score=float(extent))
        out.append((hole, float(area)))

    out.sort(key=lambda ha: ha[1], reverse=True)
    return [hole for hole, _area in out]
