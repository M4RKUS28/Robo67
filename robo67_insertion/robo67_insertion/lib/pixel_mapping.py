"""Pixel-to-base mapping seam (Candidate 2): one interface, two adapters.

Two camera flows map detected pixels into the robot base XY plane, with
different camera models and sign conventions. This module unifies them behind
ONE seam (:class:`PixelToBaseMappingModule`) so node callers no longer carry
mapping-model conditionals. Each adapter COMPOSES an existing pure primitive --
it does NOT reimplement the homography projection or the pinhole math:

* :class:`HomographyMappingAdapter` -- overhead C920. Maps a pixel to an
  ABSOLUTE base-frame XY via the calibrated homography ``H``, composing
  :func:`robo67_insertion.lib.geometry.pixel_to_base`. The :class:`MappingContext`
  is ignored.
* :class:`PinholeMappingAdapter` -- eye-in-hand D405 (tool vertical). Maps a
  pixel error to an UNSCALED base-frame XY CORRECTION (delta), composing
  :func:`robo67_insertion.lib.servoing.ibvs_correction` with ``gain=1.0``. The
  proportional servo GAIN stays OUTSIDE the seam -- the node multiplies the
  result by its gain. The pinhole convention (incl. the negative sign) is
  inherited verbatim from ``servoing``.

Pure: numpy + stdlib only. Imports NO rclpy and NO cv2. Importing ``geometry``
and ``servoing`` is fine -- both are pure; ``geometry`` only imports cv2 lazily
inside ``fit_homography`` (which this module must NOT call).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from robo67_insertion.lib import geometry
from robo67_insertion.lib.servoing import ibvs_correction

__all__ = [
    "PixelObservation",
    "MappingContext",
    "PixelToBaseMappingModule",
    "HomographyMappingAdapter",
    "PinholeMappingAdapter",
]


@dataclass(frozen=True)
class PixelObservation:
    """A detected pixel in image coordinates ``(u, v)``."""

    u: float
    v: float


@dataclass(frozen=True)
class MappingContext:
    """Per-call mapping inputs beyond the pixel itself.

    The homography path ignores all fields. The pinhole path requires every
    field: the measured ``depth_m`` (m, along the optical axis), the focal
    lengths ``fx``/``fy`` (pixels), and the principal point ``(center_u,
    center_v)`` (per-frame; the node passes the image center).
    """

    depth_m: Optional[float] = None
    fx: Optional[float] = None
    fy: Optional[float] = None
    center_u: Optional[float] = None  # principal point (per-frame for pinhole)
    center_v: Optional[float] = None


class PixelToBaseMappingModule:
    """The mapping seam: pixel + context -> base-frame XY ``(x, y)``.

    Subclasses encapsulate the camera model. Callers depend only on this
    interface, never on which adapter (or camera model) is in use.
    """

    def map_xy(self, obs: PixelObservation, ctx: MappingContext) -> Tuple[float, float]:
        raise NotImplementedError


class HomographyMappingAdapter(PixelToBaseMappingModule):
    """Overhead C920: ABSOLUTE base XY via the calibrated homography ``H``.

    Composes :func:`geometry.pixel_to_base`; ``ctx`` is ignored. The returned
    tuple byte-matches ``geometry.pixel_to_base(H, (u, v))``.
    """

    def __init__(self, H: np.ndarray):
        self.H = np.asarray(H, dtype=float)

    def map_xy(self, obs: PixelObservation, ctx: MappingContext) -> Tuple[float, float]:
        xy = geometry.pixel_to_base(self.H, np.array([obs.u, obs.v], dtype=float))
        return (float(xy[0]), float(xy[1]))


class PinholeMappingAdapter(PixelToBaseMappingModule):
    """Eye-in-hand D405: UNSCALED base-frame XY CORRECTION (gain = 1.0).

    Composes :func:`servoing.ibvs_correction` with ``gain=1.0``; the
    proportional servo gain stays OUTSIDE this seam (the node applies it). The
    sign convention is inherited verbatim from ``servoing``.

    Requires ``depth_m``, ``fx``, ``fy``, ``center_u`` and ``center_v`` in the
    context; raises :class:`ValueError` if any is ``None`` (the pinhole path
    needs depth + intrinsics + principal point).
    """

    def map_xy(self, obs: PixelObservation, ctx: MappingContext) -> Tuple[float, float]:
        missing = [
            name
            for name in ("depth_m", "fx", "fy", "center_u", "center_v")
            if getattr(ctx, name) is None
        ]
        if missing:
            raise ValueError(
                "PinholeMappingAdapter requires depth + intrinsics + principal "
                f"point; missing/None in MappingContext: {', '.join(missing)}"
            )
        return ibvs_correction(
            (obs.u, obs.v),
            (ctx.center_u, ctx.center_v),
            ctx.depth_m,
            ctx.fx,
            ctx.fy,
            gain=1.0,
        )
