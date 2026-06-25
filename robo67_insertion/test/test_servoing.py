"""Tests for image-based visual servoing (IBVS) pixel-to-base corrections.

Strict TDD: these tests are written before the implementation in
``robo67_insertion.lib.servoing``.

Convention under test (eye-in-hand D405, tool pointing straight down)::

    du = hole_u - center_u ; dv = hole_v - center_v
    ex = du * depth_m / fx ; ey = dv * depth_m / fy
    dx_base = -gain * ex ; dy_base = -gain * ey
"""
import pytest

from robo67_insertion.lib.servoing import ibvs_correction


def test_hole_at_center_returns_zero():
    dx, dy = ibvs_correction((320, 240), (320, 240), 0.1, 600, 600, 1.0)
    assert dx == pytest.approx(0.0, abs=1e-12)
    assert dy == pytest.approx(0.0, abs=1e-12)


def test_u_offset_maps_to_dx():
    dx, dy = ibvs_correction((330, 240), (320, 240), 0.1, 600, 600, 1.0)
    assert dx == pytest.approx(-10 * 0.1 / 600)
    assert dy == pytest.approx(0.0, abs=1e-12)


def test_gain_scales_linearly():
    dx1, dy1 = ibvs_correction((330, 240), (320, 240), 0.1, 600, 600, 1.0)
    dx2, dy2 = ibvs_correction((330, 240), (320, 240), 0.1, 600, 600, 2.0)
    assert dx2 == pytest.approx(2.0 * dx1)
    assert dy2 == pytest.approx(2.0 * dy1)
    assert dx2 == pytest.approx(-2.0 * 10 * 0.1 / 600)


def test_v_offset_maps_to_dy():
    dx, dy = ibvs_correction((320, 250), (320, 240), 0.2, 500, 400, 1.0)
    assert dy == pytest.approx(-(10 * 0.2 / 400))
    assert dx == pytest.approx(0.0, abs=1e-12)
