"""Box-frame port offset: make a TAUGHT port location follow a moved/rotated box.

The overhead C920 + homography locate the box, but a *single* overhead camera
cannot resolve where on the box face the target port sits to sub-cm precision
(the homography is metric only on its calibration plane; the port panel rides a
few cm above it, so off-nadir parallax introduces a ~1-2 cm systematic offset).

Rather than measure the port with a ruler through that lossy mapping, we TEACH
it once: jog the EE tip onto the real port and read its base-frame XY straight
from ``FrankaState`` (parallax-free, ruler-free), then store it as an OFFSET in
the BOX'S OWN frame. At runtime the box is re-detected -- it may have been moved
AND rotated -- and the same offset is re-applied, so the port target follows the
box rigidly. Any residual is absorbed by the insertion spiral search.

The box frame is built from the ORB-matched template corners
(``OrbBoxMatcher`` projects the template quad ``[TL, TR, BR, BL]``, an order that
is preserved across rotations because it tracks template identity) mapped into
base XY -- NOT from ``minAreaRect``'s ``angle_deg`` (which carries a 90-degree /
flip ambiguity that would silently spin the offset).

Conventions
-----------
* ``corners_base`` is ``(4, 2)`` base-frame XY in ORB order ``[TL, TR, BR, BL]``.
* The box frame is right-handed in 2D: ``+x`` runs TL->TR (template ``+u``),
  ``+y`` runs TL->BL (template ``+v``), orthonormalised (Gram-Schmidt).
* An offset ``(dx, dy)`` is in METRES along those box axes, so it stays fixed as
  the physical box translates/rotates in the plane.

Pure: numpy + stdlib only. Imports NO rclpy and NO cv2.
"""
from __future__ import annotations

from typing import Callable, Tuple

import numpy as np

__all__ = [
    "box_frame_base",
    "to_box_frame",
    "from_box_frame",
    "teach_port_offset",
    "port_base_from_box",
    "map_corners_to_base",
]


def box_frame_base(corners_base: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Build the 2D box pose ``(center, R)`` from ordered base-frame corners.

    Parameters
    ----------
    corners_base:
        ``(4, 2)`` base-frame XY corners in ORB order ``[TL, TR, BR, BL]``.

    Returns
    -------
    center:
        ``(2,)`` box centroid (mean of the four corners), base-frame XY.
    R:
        ``(2, 2)`` orthonormal rotation whose columns are the box ``+x`` (TL->TR)
        and ``+y`` (TL->BL) axes in base frame. ``R @ offset + center`` maps a
        box-frame offset to base XY.

    Raises
    ------
    ValueError:
        If ``corners_base`` is not ``(4, 2)`` or the box edges are degenerate
        (zero-length), which would make the frame unobservable.
    """
    c = np.asarray(corners_base, dtype=float)
    if c.shape != (4, 2):
        raise ValueError(f"corners_base must be (4, 2); got {c.shape}")

    center = c.mean(axis=0)
    # Average the two parallel edges for each axis (more robust than one edge).
    ex = (c[1] - c[0]) + (c[2] - c[3])  # top (TL->TR) + bottom (BL->BR): box +x
    ey = (c[3] - c[0]) + (c[2] - c[1])  # left (TL->BL) + right (TR->BR): box +y

    nx = float(np.linalg.norm(ex))
    if nx < 1e-9:
        raise ValueError("degenerate box: zero-length +x edge")
    x_hat = ex / nx
    # Gram-Schmidt: force +y orthonormal to +x (the projected quad need not be
    # exactly rectangular after the homography).
    ey_perp = ey - float(ey @ x_hat) * x_hat
    ny = float(np.linalg.norm(ey_perp))
    if ny < 1e-9:
        raise ValueError("degenerate box: +x and +y edges are colinear")
    y_hat = ey_perp / ny

    R = np.column_stack([x_hat, y_hat])
    return center, R


def to_box_frame(point_base: np.ndarray, center: np.ndarray, R: np.ndarray) -> np.ndarray:
    """Express a base-frame XY ``point`` as an offset in the box frame.

    Inverse of :func:`from_box_frame`: ``offset = R.T @ (point - center)``.
    """
    p = np.asarray(point_base, dtype=float)
    return np.asarray(R, dtype=float).T @ (p - np.asarray(center, dtype=float))


def from_box_frame(offset: np.ndarray, center: np.ndarray, R: np.ndarray) -> np.ndarray:
    """Map a box-frame ``offset`` back to base-frame XY.

    ``point = center + R @ offset``.
    """
    o = np.asarray(offset, dtype=float)
    return np.asarray(center, dtype=float) + np.asarray(R, dtype=float) @ o


def teach_port_offset(corners_base: np.ndarray, port_base_xy: np.ndarray) -> np.ndarray:
    """TEACH: given the box corners and the jogged port base XY, return the
    box-frame ``offset`` to store. (One-shot ``box_frame_base`` + ``to_box_frame``.)
    """
    center, R = box_frame_base(corners_base)
    return to_box_frame(port_base_xy, center, R)


def port_base_from_box(corners_base: np.ndarray, offset: np.ndarray) -> np.ndarray:
    """RUNTIME: given the (possibly moved/rotated) box corners and the stored
    box-frame ``offset``, return the port's base-frame XY. (One-shot
    ``box_frame_base`` + ``from_box_frame``.)
    """
    center, R = box_frame_base(corners_base)
    return from_box_frame(offset, center, R)


def map_corners_to_base(
    corners_px: np.ndarray,
    map_xy: Callable[[float, float], Tuple[float, float]],
) -> np.ndarray:
    """Map ``(4, 2)`` pixel corners to base XY via a per-pixel ``map_xy`` callable.

    Keeps this seam free of any camera/homography dependency: the caller supplies
    ``map_xy(u, v) -> (x, y)`` (e.g. wrapping the overhead
    ``HomographyMappingAdapter``). Corner ORDER is preserved.
    """
    px = np.asarray(corners_px, dtype=float)
    if px.shape != (4, 2):
        raise ValueError(f"corners_px must be (4, 2); got {px.shape}")
    return np.array([list(map_xy(float(u), float(v))) for u, v in px], dtype=float)
