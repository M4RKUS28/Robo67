"""Tests for the pixel-to-base mapping seam (Task 8.4, Candidate 2).

Strict TDD: these are written before ``robo67_insertion.lib.pixel_mapping``.

The seam unifies the two camera flows behind ONE interface
(:class:`PixelToBaseMappingModule`) with two adapters that COMPOSE the existing
pure primitives (do NOT reimplement the math):

* :class:`HomographyMappingAdapter` -- overhead C920, ABSOLUTE base XY via the
  calibrated homography (composes ``geometry.pixel_to_base``).
* :class:`PinholeMappingAdapter` -- eye-in-hand D405, UNSCALED base-frame XY
  CORRECTION (gain = 1.0; the gain lives OUTSIDE the seam, in the node);
  composes ``servoing.ibvs_correction(..., gain=1.0)``.
"""
import subprocess
import sys

import numpy as np
import pytest

from robo67_insertion.lib import geometry
from robo67_insertion.lib.servoing import ibvs_correction
from robo67_insertion.lib.pixel_mapping import (
    HomographyMappingAdapter,
    MappingContext,
    PinholeMappingAdapter,
    PixelObservation,
    PixelToBaseMappingModule,
)


H_TRUE = np.array(
    [
        [0.0005, 0.0, 0.2],
        [0.0, 0.0005, -0.1],
        [0.0, 0.0, 1.0],
    ]
)

PIXELS = [
    (100.0, 100.0),
    (500.0, 100.0),
    (100.0, 400.0),
    (500.0, 400.0),
    (300.0, 250.0),
    (640.0, 480.0),
]


# --------------------------------------------------------------------------- #
# Purity / interface
# --------------------------------------------------------------------------- #
def test_pixel_mapping_is_pure_no_rclpy_no_cv2():
    """The seam must import NO rclpy and NO cv2 (geometry/servoing are pure)."""
    code = (
        "import robo67_insertion.lib.pixel_mapping as pm, sys; "
        "assert 'cv2' not in sys.modules, 'cv2 imported by pixel_mapping'; "
        "assert 'rclpy' not in sys.modules, 'rclpy imported by pixel_mapping'"
    )
    r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr


def test_adapters_implement_the_seam():
    assert issubclass(HomographyMappingAdapter, PixelToBaseMappingModule)
    assert issubclass(PinholeMappingAdapter, PixelToBaseMappingModule)


# --------------------------------------------------------------------------- #
# HomographyMappingAdapter: byte-match geometry.pixel_to_base
# --------------------------------------------------------------------------- #
def test_homography_adapter_matches_pixel_to_base():
    adapter = HomographyMappingAdapter(H_TRUE)
    for u, v in PIXELS:
        got = adapter.map_xy(PixelObservation(u, v), MappingContext())
        expected = geometry.pixel_to_base(H_TRUE, np.array([u, v], dtype=float))
        assert isinstance(got, tuple)
        assert len(got) == 2
        assert got[0] == pytest.approx(float(expected[0]), abs=0.0, rel=0.0)
        assert got[1] == pytest.approx(float(expected[1]), abs=0.0, rel=0.0)


def test_homography_adapter_ignores_context():
    adapter = HomographyMappingAdapter(H_TRUE)
    obs = PixelObservation(300.0, 250.0)
    a = adapter.map_xy(obs, MappingContext())
    b = adapter.map_xy(obs, MappingContext(depth_m=0.2, fx=600, fy=600,
                                           center_u=320, center_v=240))
    assert a == b


# --------------------------------------------------------------------------- #
# PinholeMappingAdapter: byte-match servoing.ibvs_correction(..., gain=1.0)
# --------------------------------------------------------------------------- #
def test_pinhole_adapter_center_returns_zero():
    adapter = PinholeMappingAdapter()
    ctx = MappingContext(depth_m=0.1, fx=600, fy=600, center_u=320, center_v=240)
    dx, dy = adapter.map_xy(PixelObservation(320, 240), ctx)
    assert dx == pytest.approx(0.0, abs=1e-12)
    assert dy == pytest.approx(0.0, abs=1e-12)


def test_pinhole_adapter_offset_signed_metric():
    adapter = PinholeMappingAdapter()
    ctx = MappingContext(depth_m=0.1, fx=600, fy=600, center_u=320, center_v=240)
    dx, dy = adapter.map_xy(PixelObservation(330, 240), ctx)
    assert dx == pytest.approx(-10 * 0.1 / 600)
    assert dy == pytest.approx(0.0, abs=1e-12)


def test_pinhole_adapter_equals_ibvs_gain_one():
    adapter = PinholeMappingAdapter()
    for (u, v, cu, cv, depth, fx, fy) in [
        (330.0, 240.0, 320.0, 240.0, 0.1, 600.0, 600.0),
        (320.0, 250.0, 320.0, 240.0, 0.2, 500.0, 400.0),
        (300.0, 300.0, 320.0, 240.0, 0.15, 700.0, 650.0),
    ]:
        ctx = MappingContext(depth_m=depth, fx=fx, fy=fy, center_u=cu, center_v=cv)
        got = adapter.map_xy(PixelObservation(u, v), ctx)
        expected = ibvs_correction((u, v), (cu, cv), depth, fx, fy, 1.0)
        assert got == expected


def test_pinhole_adapter_gain_externalized():
    """gain * map_xy(...) == ibvs_correction(..., gain) for a non-unit gain."""
    adapter = PinholeMappingAdapter()
    ctx = MappingContext(depth_m=0.1, fx=600, fy=600, center_u=320, center_v=240)
    gain = 0.6
    dx0, dy0 = adapter.map_xy(PixelObservation(330, 250), ctx)
    dx, dy = gain * dx0, gain * dy0
    exp_dx, exp_dy = ibvs_correction((330, 250), (320, 240), 0.1, 600, 600, gain)
    assert dx == pytest.approx(exp_dx)
    assert dy == pytest.approx(exp_dy)


@pytest.mark.parametrize(
    "missing",
    ["depth_m", "fx", "fy", "center_u", "center_v"],
)
def test_pinhole_adapter_requires_params(missing):
    adapter = PinholeMappingAdapter()
    kwargs = dict(depth_m=0.1, fx=600.0, fy=600.0, center_u=320.0, center_v=240.0)
    kwargs[missing] = None
    ctx = MappingContext(**kwargs)
    with pytest.raises(ValueError):
        adapter.map_xy(PixelObservation(330, 240), ctx)


# --------------------------------------------------------------------------- #
# Cross-adapter consistency (synthetic correspondence)
# --------------------------------------------------------------------------- #
def test_cross_adapter_consistency():
    """A homography whose local pixel->base scale matches the pinhole model's.

    Build a diagonal scale+translate homography with sx = -depth/fx,
    sy = -depth/fy. Then the homography's base-XY DIFFERENCE between an offset
    pixel and the center pixel equals the pinhole CORRECTION (sign + magnitude),
    because both equal (-du*depth/fx, -dv*depth/fy). Deterministic and
    well-conditioned (diagonal H).
    """
    depth, fx, fy = 0.12, 800.0, 600.0
    center_u, center_v = 640.0, 480.0
    sx, sy = -depth / fx, -depth / fy
    H = np.array([
        [sx, 0.0, 0.5],
        [0.0, sy, -0.3],
        [0.0, 0.0, 1.0],
    ])

    homog = HomographyMappingAdapter(H)
    pinhole = PinholeMappingAdapter()

    du, dv = 8.0, -5.0
    offset = PixelObservation(center_u + du, center_v + dv)
    center = PixelObservation(center_u, center_v)

    bx_off = homog.map_xy(offset, MappingContext())
    bx_ctr = homog.map_xy(center, MappingContext())
    homog_diff = (bx_off[0] - bx_ctr[0], bx_off[1] - bx_ctr[1])

    ctx = MappingContext(depth_m=depth, fx=fx, fy=fy,
                         center_u=center_u, center_v=center_v)
    correction = pinhole.map_xy(offset, ctx)

    assert correction[0] == pytest.approx(homog_diff[0], rel=1e-9, abs=1e-12)
    assert correction[1] == pytest.approx(homog_diff[1], rel=1e-9, abs=1e-12)
    # sanity: nonzero, opposite-pixel-direction sign baked into both
    assert correction[0] < 0.0  # +du -> negative base-x correction
    assert correction[1] > 0.0  # -dv -> positive base-y correction
