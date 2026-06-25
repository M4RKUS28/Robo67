"""Tests for dark circular hole detection (peg-in-hole socket vision).

Strict TDD: these tests are written before the implementation in
``robo67_insertion.lib.hole_detect``.

The overhead Logitech C920 looks down at a white 3D-printed cube socket with a
dark round hole on top. We detect the dark circular hole and return its image
center ``(u, v)`` and radius in pixels. A circularity filter rejects the gray
grooved aluminum table's vertical-line texture (lines are not circles).
"""
import os

import cv2
import numpy as np

from robo67_insertion.lib.hole_detect import (
    Hole,
    HoleParams,
    detect_holes,
)


def test_synthetic_positive_dark_circle_detected():
    img = np.full((480, 640, 3), 180, np.uint8)  # gray background
    # White 3D-printed cube top.
    cv2.rectangle(img, (250, 180), (390, 320), (255, 255, 255), -1)
    # Dark round hole on top of the cube.
    cv2.circle(img, (320, 250), 30, (0, 0, 0), -1)

    holes = detect_holes(img)
    assert len(holes) >= 1
    top = holes[0]
    assert isinstance(top, Hole)
    assert abs(top.u - 320) < 3
    assert abs(top.v - 250) < 3
    assert abs(top.radius_px - 30) < 4


def test_synthetic_negative_uniform_bright_returns_empty():
    img = np.full((480, 640, 3), 255, np.uint8)
    assert detect_holes(img) == []


def test_vertical_line_texture_not_detected_as_hole():
    img = np.full((480, 640, 3), 150, np.uint8)  # gray grooved table
    # Several thin black vertical grooves spanning full height.
    for x in range(60, 600, 40):
        cv2.rectangle(img, (x, 0), (x + 3, 479), (0, 0, 0), -1)

    params = HoleParams()
    holes = detect_holes(img, params)
    # Lines have low circularity, so none should pass the circularity filter.
    assert all(h.score < params.min_circularity for h in holes)


def test_returns_list_type():
    img = np.full((480, 640, 3), 180, np.uint8)
    res = detect_holes(img)
    assert isinstance(res, list)


def test_results_sorted_by_score_desc():
    img = np.full((480, 640, 3), 200, np.uint8)
    # Two dark circles of different sizes; both should be detected and the
    # results must be sorted by score (circularity) descending.
    cv2.circle(img, (150, 150), 25, (0, 0, 0), -1)
    cv2.circle(img, (450, 300), 40, (0, 0, 0), -1)

    holes = detect_holes(img)
    assert len(holes) >= 2
    scores = [h.score for h in holes]
    assert scores == sorted(scores, reverse=True)


def test_real_fixture_smoke():
    path = os.path.join(
        os.path.dirname(__file__), "fixtures", "c920_overexposed_no_socket.jpg"
    )
    if os.path.exists(path):
        img = cv2.imread(path)
        assert img is not None
        res = detect_holes(img)
        # No socket is present; just confirm the API returns a list.
        assert isinstance(res, list)
