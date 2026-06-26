"""Industrial I/O-box detection for the cable-insertion overhead vision.

An overhead Logitech C920 looks straight down at the workspace. The cable task
targets a **dark, roughly rectangular industrial I/O box** whose top face is
covered in high-contrast features -- metallic ports (LAN/USB/CAN BUS), white
text labels, audio jacks, and a fan grille. The box BODY is near-black, but the
gray grooved-carpet background is a *similar* mid-gray intensity, so a plain
brightness threshold cannot separate them (verified on a live C920 frame
2026-06-26: carpet median ~116, box body similar).

The robust discriminator is **local texture energy**: the port face is a dense
island of high local variance (black body + bright ports + white text = large
local swings), whereas the carpet is a uniform low-variance field. We compute a
local standard-deviation map, threshold it, merge the port features into one
blob, and return the qualifying blob(s) as :class:`Box` (centroid + oriented
size + corner quad), sorted by score.

This module is pure-Python (numpy + OpenCV + stdlib only). Like
:mod:`hole_detect` it deliberately avoids importing rclpy/ROS/scipy so it can be
unit-tested on any host. The detected centroid maps to robot-base XY through the
same overhead homography the peg-in-hole socket uses
(:class:`~robo67_insertion.lib.pixel_mapping.HomographyMappingAdapter`).

Pipeline
--------
1. Convert to grayscale (float).
2. Compute a local standard-deviation map over a ``texture_win`` window
   (``std = sqrt(E[x^2] - E[x]^2)`` via box filters -- fast, separable).
3. Threshold ``std > min_texture_std`` into a binary texture mask.
4. Morphologically CLOSE (merge ports/labels into one box-face blob) then OPEN
   (drop speckle).
5. Find external contours; reject by area, ROI border, oriented aspect ratio,
   and fill (extent).
6. Return survivors as :class:`Box`, sorted by score (texture density * size)
   descending.
"""
from dataclasses import dataclass
import math

import cv2
import numpy as np

__all__ = ["Box", "BoxParams", "local_texture_std", "detect_gray_box"]


@dataclass
class Box:
    """A detected box, in image (pixel) coordinates.

    Attributes
    ----------
    u, v:
        Centroid of the box blob, in pixels.
    width_px, height_px:
        Side lengths of the oriented (min-area) bounding rectangle, in pixels
        (``width_px >= height_px`` is NOT guaranteed; they follow OpenCV's
        ``minAreaRect`` order).
    angle_deg:
        Orientation of the min-area rectangle, in degrees (OpenCV convention).
    score:
        Detection score; higher is a stronger box candidate (texture density
        times square-root of area, normalized). Used to sort candidates.
    corners:
        ``(4, 2)`` float array of the oriented rectangle's corner pixels
        (``cv2.boxPoints`` order), handy for drawing the quad overlay.
    """

    u: float
    v: float
    width_px: float
    height_px: float
    angle_deg: float
    score: float
    corners: np.ndarray


@dataclass
class BoxParams:
    """Tunable parameters for :func:`detect_gray_box`.

    Attributes
    ----------
    texture_win:
        Odd window size (px) for the local standard-deviation map. Larger =
        smoother/blobbier texture estimate.
    min_texture_std:
        Pixels whose local std exceeds this are 'busy' (box-face) candidates.
        Carpet sits well below this; the port face well above (live C920:
        carpet std median ~15, box face ~50-90). ~p90 of a typical frame.
    close_ks, open_ks:
        CLOSE kernel (merge ports/labels into one blob) and OPEN kernel (drop
        speckle), in pixels.
    min_area_px, max_area_px:
        Accepted blob contour area (px^2) -- rejects tiny texture flecks and
        full-frame floods.
    min_aspect, max_aspect:
        Oriented aspect ratio ``max(w,h)/min(w,h)`` bounds; the I/O box is a
        chunky rectangle, not a thin sliver.
    min_extent:
        Minimum fill ``contour_area / min_area_rect_area``; a solid blob fills
        most of its oriented box (rejects ragged carpet/cable wisps).
    roi_margin:
        Ignore blobs whose centroid is within this fractional image border
        (kills the keyboard/foot/bag at the frame edges).
    """

    texture_win: int = 21
    min_texture_std: float = 35.0
    close_ks: int = 35
    open_ks: int = 9
    min_area_px: float = 6000.0
    max_area_px: float = 300000.0
    min_aspect: float = 1.0
    max_aspect: float = 3.2
    min_extent: float = 0.55
    roi_margin: float = 0.04


def local_texture_std(gray: np.ndarray, win: int) -> np.ndarray:
    """Return the per-pixel local standard deviation over a ``win`` x ``win`` box.

    Uses the identity ``Var = E[x^2] - E[x]^2`` with normalized box filters, so
    it is O(1) per pixel regardless of window size. Input may be any dtype; the
    result is float32.
    """
    g = gray.astype(np.float32)
    win = int(win) | 1  # force odd
    mean = cv2.boxFilter(g, -1, (win, win), normalize=True)
    mean_sq = cv2.boxFilter(g * g, -1, (win, win), normalize=True)
    var = np.clip(mean_sq - mean * mean, 0.0, None)
    return np.sqrt(var)


def detect_gray_box(bgr: np.ndarray, params: BoxParams = BoxParams()) -> list[Box]:
    """Detect dark, port-covered I/O box(es) in an overhead BGR image.

    Returns a list of :class:`Box` sorted by ``score`` (texture density * size)
    descending; the best box is first. Empty if none qualifies. See the module
    docstring for the texture-energy rationale and :class:`BoxParams` for tuning.
    """
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    std = local_texture_std(gray, params.texture_win)

    mask = (std > params.min_texture_std).astype(np.uint8) * 255
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (params.close_ks, params.close_ks)))
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (params.open_ks, params.open_ks)))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    mx0, my0 = int(params.roi_margin * w), int(params.roi_margin * h)

    candidates: list[tuple[Box, float]] = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < params.min_area_px or area > params.max_area_px:
            continue

        rect = cv2.minAreaRect(c)  # ((cx, cy), (rw, rh), angle)
        (rcx, rcy), (rw, rh), angle = rect
        if rw <= 1.0 or rh <= 1.0:
            continue
        rect_area = rw * rh
        aspect = max(rw, rh) / max(1.0, min(rw, rh))
        extent = area / max(1.0, rect_area)
        if not (params.min_aspect <= aspect <= params.max_aspect):
            continue
        if extent < params.min_extent:
            continue

        m = cv2.moments(c)
        if m["m00"] == 0:
            continue
        cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
        if not (mx0 < cx < w - mx0 and my0 < cy < h - my0):
            continue

        # texture density inside the blob (mean local std over its pixels)
        blob = np.zeros(gray.shape, np.uint8)
        cv2.drawContours(blob, [c], -1, 255, -1)
        density = float(std[blob > 0].mean()) if (blob > 0).any() else 0.0
        score = density * math.sqrt(area)

        corners = cv2.boxPoints(rect).astype(float)
        box = Box(u=float(cx), v=float(cy), width_px=float(rw), height_px=float(rh),
                  angle_deg=float(angle), score=float(score), corners=corners)
        candidates.append((box, score))

    candidates.sort(key=lambda bs: bs[1], reverse=True)
    return [b for b, _ in candidates]
