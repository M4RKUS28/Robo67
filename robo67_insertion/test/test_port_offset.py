"""Tests for the box-frame port-offset seam (cable insertion, Phase 5).

Strict TDD partner of ``robo67_insertion.lib.port_offset``. The seam lets a
TAUGHT port location (jogged + read from FrankaState, base-frame truth) follow
the box when it is moved AND rotated, by storing the port as an offset in the
box's OWN frame (built from the ORB template corners ``[TL, TR, BR, BL]``).

Key invariants under test:
* round-trip teach->apply on the same corners returns the port;
* the port follows a rigid translation of the box;
* the port follows a rigid rotation of the box about its centre;
* an axis-aligned box yields a metric (dx, dy) offset from the centre;
* degenerate corners are rejected;
* the module imports NO rclpy and NO cv2.
"""
import subprocess
import sys

import numpy as np
import pytest

from robo67_insertion.lib.port_offset import (
    box_frame_base,
    from_box_frame,
    map_corners_to_base,
    port_base_from_box,
    teach_port_offset,
    to_box_frame,
)


# ORB order [TL, TR, BR, BL]; +u right, +v down -> here mapped to base axes.
def _rect_corners(cx, cy, w, h):
    return np.array([
        [cx - w / 2, cy - h / 2],   # TL
        [cx + w / 2, cy - h / 2],   # TR
        [cx + w / 2, cy + h / 2],   # BR
        [cx - w / 2, cy + h / 2],   # BL
    ], float)


def _rot(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]], float)


# --------------------------------------------------------------------------- #
# Purity
# --------------------------------------------------------------------------- #
def test_port_offset_is_pure_no_rclpy_no_cv2():
    code = (
        "import robo67_insertion.lib.port_offset as po, sys; "
        "assert 'cv2' not in sys.modules, 'cv2 imported by port_offset'; "
        "assert 'rclpy' not in sys.modules, 'rclpy imported by port_offset'"
    )
    r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr


# --------------------------------------------------------------------------- #
# box_frame_base
# --------------------------------------------------------------------------- #
def test_axis_aligned_box_frame_is_identity():
    corners = _rect_corners(0.40, -0.10, 0.12, 0.08)
    center, R = box_frame_base(corners)
    assert center == pytest.approx([0.40, -0.10])
    # +x axis is TL->TR (+base-x), +y axis is TL->BL (+base-y) -> identity.
    assert R == pytest.approx(np.eye(2), abs=1e-12)


def test_box_frame_R_is_orthonormal_under_rotation():
    corners = _rect_corners(0.5, 0.0, 0.10, 0.06)
    theta = np.deg2rad(37.0)
    rotated = (_rot(theta) @ (corners - corners.mean(0)).T).T + corners.mean(0)
    center, R = box_frame_base(rotated)
    assert center == pytest.approx(corners.mean(0))
    assert (R.T @ R) == pytest.approx(np.eye(2), abs=1e-9)
    assert R == pytest.approx(_rot(theta), abs=1e-9)


def test_degenerate_corners_raise():
    with pytest.raises(ValueError):
        box_frame_base(np.zeros((4, 2)))
    with pytest.raises(ValueError):
        box_frame_base(np.zeros((3, 2)))


# --------------------------------------------------------------------------- #
# teach / apply
# --------------------------------------------------------------------------- #
def test_teach_then_apply_roundtrip():
    corners = _rect_corners(0.45, -0.20, 0.14, 0.09)
    port = np.array([0.49, -0.235], float)  # somewhere on the box face
    offset = teach_port_offset(corners, port)
    got = port_base_from_box(corners, offset)
    assert got == pytest.approx(port, abs=1e-12)


def test_axis_aligned_offset_is_metric_from_centre():
    corners = _rect_corners(0.40, -0.10, 0.12, 0.08)
    port = np.array([0.40 + 0.03, -0.10 - 0.02], float)
    offset = teach_port_offset(corners, port)
    # identity frame -> offset == (port - centre)
    assert offset == pytest.approx([0.03, -0.02], abs=1e-12)


def test_port_follows_box_translation():
    corners = _rect_corners(0.45, -0.20, 0.14, 0.09)
    port = np.array([0.49, -0.235], float)
    offset = teach_port_offset(corners, port)

    t = np.array([0.05, 0.08], float)
    moved = corners + t
    got = port_base_from_box(moved, offset)
    assert got == pytest.approx(port + t, abs=1e-12)


def test_port_follows_box_rotation_about_centre():
    center0 = np.array([0.45, -0.20], float)
    corners = _rect_corners(center0[0], center0[1], 0.14, 0.09)
    port = np.array([0.49, -0.235], float)
    offset = teach_port_offset(corners, port)

    theta = np.deg2rad(25.0)
    R = _rot(theta)
    rotated = (R @ (corners - center0).T).T + center0
    got = port_base_from_box(rotated, offset)
    expected = center0 + R @ (port - center0)
    assert got == pytest.approx(expected, abs=1e-12)


def test_port_follows_box_translation_and_rotation():
    center0 = np.array([0.50, 0.05], float)
    corners = _rect_corners(center0[0], center0[1], 0.12, 0.10)
    port = np.array([0.535, 0.075], float)
    offset = teach_port_offset(corners, port)

    theta = np.deg2rad(-40.0)
    R = _rot(theta)
    t = np.array([-0.06, 0.04], float)
    moved = (R @ (corners - center0).T).T + center0 + t
    got = port_base_from_box(moved, offset)
    expected = center0 + t + R @ (port - center0)
    assert got == pytest.approx(expected, abs=1e-12)


# --------------------------------------------------------------------------- #
# to/from box frame are inverses
# --------------------------------------------------------------------------- #
def test_to_from_box_frame_inverse():
    corners = _rect_corners(0.5, 0.0, 0.10, 0.06)
    center, R = box_frame_base((_rot(0.3) @ (corners - corners.mean(0)).T).T + corners.mean(0))
    p = np.array([0.52, 0.01], float)
    off = to_box_frame(p, center, R)
    assert from_box_frame(off, center, R) == pytest.approx(p, abs=1e-12)


# --------------------------------------------------------------------------- #
# map_corners_to_base
# --------------------------------------------------------------------------- #
def test_map_corners_to_base_applies_callable_in_order():
    corners_px = _rect_corners(640, 360, 200, 120)
    # simple affine pixel->base: scale + offset
    sx, sy, ox, oy = 5e-4, 5e-4, 0.45, -0.10

    def map_xy(u, v):
        return (ox + sx * (u - 640), oy + sy * (v - 360))

    base = map_corners_to_base(corners_px, map_xy)
    assert base.shape == (4, 2)
    # TL pixel maps to TL base, order preserved
    assert base[0] == pytest.approx([ox + sx * (-100), oy + sy * (-60)])
    assert base[2] == pytest.approx([ox + sx * (100), oy + sy * (60)])


def test_map_corners_rejects_bad_shape():
    with pytest.raises(ValueError):
        map_corners_to_base(np.zeros((3, 2)), lambda u, v: (u, v))
