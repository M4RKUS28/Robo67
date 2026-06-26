"""Dark circular hole detection for the peg-in-hole socket vision pipeline.

An overhead Logitech C920 looks straight down at the workspace. The socket is a
white 3D-printed cube with a **dark round hole** on its top face. The hole stays
dark even when the white cube is overexposed, which makes a simple intensity
threshold a robust first cut. The background is a gray grooved aluminum table
whose vertical-line texture is *not* circular, so a circularity filter rejects
it.

This module is pure-Python (numpy + OpenCV + stdlib only). It deliberately
avoids importing rclpy/ROS/scipy so it can be unit-tested on any host.

Pipeline
--------
1. Convert to grayscale.
2. Threshold dark pixels (``gray < dark_max_value``) into a binary mask.
3. Morphologically open the mask to remove speckle noise.
4. Find external contours.
5. For each contour, reject by area, enclosing-circle radius, and circularity
   ``(4*pi*area / perimeter^2)``.
6. Return surviving holes sorted by score (circularity) descending, with area
   as the tie-break.
"""
from dataclasses import dataclass
import math

import cv2
import numpy as np

__all__ = [
    "Hole",
    "HoleParams",
    "detect_holes",
    "WhiteSocketParams",
    "detect_sockets",
    "WhiteCubeParams",
    "detect_white_cubes",
]


@dataclass
class Hole:
    """A detected dark circular hole, in image (pixel) coordinates.

    Attributes
    ----------
    u:
        Horizontal image coordinate of the hole center, in pixels.
    v:
        Vertical image coordinate of the hole center, in pixels.
    radius_px:
        Radius of the minimum enclosing circle, in pixels.
    score:
        Detection score (circularity in ``[0, 1]``); higher is rounder.
    """

    u: float
    v: float
    radius_px: float
    score: float


@dataclass
class HoleParams:
    """Tunable parameters for :func:`detect_holes`.

    Attributes
    ----------
    min_radius_px:
        Reject enclosing circles smaller than this (pixels).
    max_radius_px:
        Reject enclosing circles larger than this (pixels).
    dark_max_value:
        Pixels with intensity strictly below this are 'hole' candidates.
    min_circularity:
        Minimum ``4*pi*area / perimeter^2`` for a contour to count as round.
    """

    min_radius_px: float = 8.0
    max_radius_px: float = 200.0
    dark_max_value: int = 90  # pixels darker than this are 'hole' candidates
    min_circularity: float = 0.6  # 4*pi*area / perimeter^2


def detect_holes(bgr: np.ndarray, params: HoleParams = HoleParams()) -> list[Hole]:
    """Detect dark circular holes in a BGR image.

    Parameters
    ----------
    bgr:
        Input image as an ``H x W x 3`` BGR ``uint8`` array (as returned by
        ``cv2.imread`` / a C920 capture).
    params:
        Detection parameters; see :class:`HoleParams`.

    Returns
    -------
    list of Hole
        Detected holes sorted by ``score`` (circularity) descending, with
        contour area as the secondary (tie-break) key. Empty if none found.
    """
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    # Dark pixels are hole candidates. The hole stays dark even when the white
    # socket body is overexposed, so a plain intensity threshold is robust.
    mask = (gray < params.dark_max_value).astype(np.uint8) * 255

    # Denoise: open (erode then dilate) with a small kernel to drop speckle.
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    min_area = math.pi * params.min_radius_px ** 2

    candidates: list[tuple[Hole, float]] = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue

        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0.0:
            continue

        circularity = 4.0 * math.pi * area / (perimeter * perimeter)

        (cx, cy), r = cv2.minEnclosingCircle(contour)
        if r < params.min_radius_px or r > params.max_radius_px:
            continue
        if circularity < params.min_circularity:
            continue

        hole = Hole(
            u=float(cx),
            v=float(cy),
            radius_px=float(r),
            score=float(circularity),
        )
        candidates.append((hole, float(area)))

    # Sort by score (circularity) descending, area as the secondary key.
    candidates.sort(key=lambda ha: (ha[0].score, ha[1]), reverse=True)

    return [hole for hole, _area in candidates]


@dataclass
class WhiteSocketParams:
    """Tunable parameters for :func:`detect_sockets`.

    The REAL Robo67 socket is a small WHITE 3D-printed cube with a bright
    recessed bore, sitting on a DARK surface (carpet/mat). This is the opposite
    of :func:`detect_holes`' assumption (a dark hole), so a separate detector is
    needed (see docs/cameras.md; verified on live C920 frames 2026-06-25).

    Attributes
    ----------
    min_radius_px, max_radius_px:
        Accepted bore radius range (the bore nearly fills the small cube top).
    roi_margin:
        Ignore detections inside this fractional image border (kills bright
        monitor/box edges and keeps the working area central).
    inner_med_min:
        The candidate interior must be bright (``>= inner_med_min``), i.e. on the
        white cube/bore rather than on dark carpet.
    inner_contrast_min:
        Minimum (max - 5th percentile) inside the disc. The bore's shadow/rim
        creates internal contrast; a FLAT hole-less decoy cube top is uniform
        (near-zero contrast) and is rejected by this. (Validated 7/7 on live
        C920 frames across exposures 2026-06-25; this is more robust than an
        upper median bound, which wrongly rejects a brightly-lit bore.)
    dark_surround_frac_min / dark_value:
        Fraction of an outer annulus that must be darker than ``dark_value``
        (the cube sits on dark carpet) -- rejects the WHITE robot arm, which is
        surrounded by more white.
    dp, min_dist_px, hough_param1, hough_param2:
        ``cv2.HoughCircles`` parameters.
    """

    min_radius_px: float = 14.0
    max_radius_px: float = 36.0
    roi_margin: float = 0.12
    inner_med_min: float = 185.0
    inner_contrast_min: float = 30.0
    dark_surround_frac_min: float = 0.6
    dark_value: int = 130
    dp: float = 1.2
    min_dist_px: float = 40.0
    hough_param1: float = 120.0
    hough_param2: float = 18.0


def detect_sockets(bgr: np.ndarray,
                   params: WhiteSocketParams = WhiteSocketParams()) -> list[Hole]:
    """Detect bright (white) circular socket bores in an overhead BGR image.

    Companion to :func:`detect_holes` for the real white-on-dark socket: finds
    bright circular candidates with ``cv2.HoughCircles``, then keeps only those
    that (a) lie in the central ROI, (b) have a bright interior (on the cube),
    (c) show internal contrast from the bore's rim shadow (rejecting a FLAT
    hole-less decoy cube), and (d) sit on a dark background (rejecting the white
    arm). Returns a list of :class:`Hole` sorted by ``score`` (descending) -- a
    drop-in replacement for :func:`detect_holes` for the real socket.
    """
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    blur = cv2.medianBlur(gray, 5)
    h, w = gray.shape
    mx0, my0 = int(params.roi_margin * w), int(params.roi_margin * h)

    circles = cv2.HoughCircles(
        blur, cv2.HOUGH_GRADIENT, dp=params.dp, minDist=params.min_dist_px,
        param1=params.hough_param1, param2=params.hough_param2,
        minRadius=int(params.min_radius_px), maxRadius=int(params.max_radius_px),
    )
    if circles is None:
        return []

    ys, xs = np.ogrid[:h, :w]
    out: list[Hole] = []
    for u, v, r in np.round(circles[0]).astype(int):
        if not (mx0 < u < w - mx0 and my0 < v < h - my0):
            continue
        d = np.sqrt((xs - u) ** 2 + (ys - v) ** 2)
        inner = gray[d <= 0.9 * r].astype(float)
        if inner.size < 20:
            continue
        med = float(np.percentile(inner, 50))
        contrast = float(inner.max() - np.percentile(inner, 5))
        annulus = (d >= 2.2 * r) & (d <= 3.2 * r)
        dark_frac = float((gray[annulus] < params.dark_value).mean()) if annulus.sum() else 0.0
        if not (med >= params.inner_med_min
                and contrast >= params.inner_contrast_min
                and dark_frac >= params.dark_surround_frac_min):
            continue
        score = dark_frac * min(1.0, contrast / 120.0)
        out.append(Hole(u=float(u), v=float(v), radius_px=float(r), score=float(score)))

    out.sort(key=lambda hole: hole.score, reverse=True)
    return out


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

    Robust drop-in for the socket feature when the bore is too faint to detect
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
