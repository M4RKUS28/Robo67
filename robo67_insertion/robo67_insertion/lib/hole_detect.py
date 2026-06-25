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

__all__ = ["Hole", "HoleParams", "detect_holes"]


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
