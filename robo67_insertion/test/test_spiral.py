"""Tests for the Archimedean spiral search pattern (peg-in-hole XY search).

Strict TDD: these tests are written before the implementation in
``robo67_insertion.lib.spiral``.
"""
import math

import numpy as np
import pytest

from robo67_insertion.lib.spiral import archimedean_offset, spiral_waypoints


def test_archimedean_offset_at_zero():
    dx, dy = archimedean_offset(0.0, 0.002, 0.005)
    assert dx == pytest.approx(0.0, abs=1e-9)
    assert dy == pytest.approx(0.0, abs=1e-9)


def test_archimedean_offset_radius_grows_with_time():
    pitch_m = 0.002
    lin_speed_mps = 0.005
    r_early = math.hypot(*archimedean_offset(0.1, pitch_m, lin_speed_mps))
    r_late = math.hypot(*archimedean_offset(1.0, pitch_m, lin_speed_mps))
    assert r_late > r_early


def test_spiral_waypoints_shape_and_bounds():
    max_radius_m = 0.012
    pitch_m = 0.002
    pts_per_rev = 36
    wps = spiral_waypoints(max_radius_m=max_radius_m, pitch_m=pitch_m,
                           pts_per_rev=pts_per_rev)

    assert isinstance(wps, np.ndarray)
    assert wps.ndim == 2
    assert wps.shape[1] == 2
    assert wps.shape[0] >= 2

    # First waypoint is essentially the center.
    assert np.allclose(wps[0], [0.0, 0.0], atol=1e-6)

    # Radii are non-decreasing.
    radii = np.linalg.norm(wps, axis=1)
    assert np.all(np.diff(radii) >= -1e-9)

    # Max radius does not exceed the cap by more than one pitch step.
    one_step_radius = pitch_m / pts_per_rev
    assert radii.max() <= max_radius_m + one_step_radius + 1e-9


def test_spiral_waypoints_angular_spacing():
    pts_per_rev = 36
    wps = spiral_waypoints(max_radius_m=0.012, pitch_m=0.002,
                           pts_per_rev=pts_per_rev)
    expected = 2.0 * math.pi / pts_per_rev

    # Check a few interior points (skip the center where the angle is
    # undefined). Allow wraparound at +/- pi.
    angles = np.arctan2(wps[:, 1], wps[:, 0])
    for i in range(2, min(8, len(wps) - 1)):
        d = angles[i + 1] - angles[i]
        # Normalize to (-pi, pi] to handle wraparound.
        d = (d + math.pi) % (2.0 * math.pi) - math.pi
        assert d == pytest.approx(expected, abs=1e-6)
