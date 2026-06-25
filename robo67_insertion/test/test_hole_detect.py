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
    WhiteCubeParams,
    WhiteSocketParams,
    detect_holes,
    detect_sockets,
    detect_white_cubes,
)


def _white_socket_image(with_bore=True):
    """Synthetic overhead view: a small WHITE cube on a DARK background, with
    (or without) a bright recessed bore. Mirrors the real Robo67 socket that
    detect_sockets targets (white-on-dark, opposite of detect_holes)."""
    img = np.full((480, 640, 3), 35, np.uint8)                  # dark carpet
    cv2.rectangle(img, (290, 200), (350, 260), (252, 252, 252), -1)  # white cube top
    if with_bore:
        cv2.circle(img, (320, 230), 20, (150, 150, 150), 2)     # bore rim shadow (edge)
        cv2.circle(img, (320, 230), 18, (205, 205, 205), -1)    # bore bottom (bright < cube)
    return img


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


# --- detect_sockets: the real white-on-dark socket (bright bore) -------------

def test_white_socket_detected():
    holes = detect_sockets(_white_socket_image(with_bore=True))
    assert len(holes) >= 1
    top = holes[0]
    assert isinstance(top, Hole)
    assert abs(top.u - 320) < 6
    assert abs(top.v - 230) < 6
    assert abs(top.radius_px - 19) < 8


def test_white_socket_holeless_cube_rejected():
    # A flat white cube top (no bore) saturates uniformly -> must NOT be
    # reported as a socket (this is the on-table decoy cube).
    holes = detect_sockets(_white_socket_image(with_bore=False))
    near_cube = [h for h in holes if 290 < h.u < 350 and 200 < h.v < 260]
    assert near_cube == []


def test_white_socket_returns_sorted_list():
    res = detect_sockets(_white_socket_image(with_bore=True))
    assert isinstance(res, list)
    scores = [h.score for h in res]
    assert scores == sorted(scores, reverse=True)


def test_white_socket_empty_scene():
    img = np.full((480, 640, 3), 35, np.uint8)  # just dark background
    assert detect_sockets(img) == []


def test_white_socket_params_overridable():
    # Tightening the radius band below the bore size yields no detection.
    params = WhiteSocketParams(min_radius_px=30.0, max_radius_px=36.0)
    assert detect_sockets(_white_socket_image(with_bore=True), params) == []


# --- detect_white_cubes: robust cube-centroid feature (overexposure-proof) ---

def test_white_cube_centroid_detected():
    # Cube spans (290,200)-(350,260) -> centroid ~ (320, 230).
    holes = detect_white_cubes(_white_socket_image(with_bore=True))
    assert len(holes) >= 1
    top = holes[0]
    assert isinstance(top, Hole)
    assert abs(top.u - 320) < 6
    assert abs(top.v - 230) < 6


def test_white_cube_detected_even_without_bore():
    # The cube detector keys on the WHITE SQUARE, so a blank (bore-less) cube is
    # still found -- it cannot distinguish socket from blank (keep one in view).
    holes = detect_white_cubes(_white_socket_image(with_bore=False))
    assert len(holes) >= 1
    assert abs(holes[0].u - 320) < 6 and abs(holes[0].v - 230) < 6


def test_white_cube_empty_scene():
    img = np.full((480, 640, 3), 35, np.uint8)
    assert detect_white_cubes(img) == []


def test_white_cube_rejects_non_square_blob():
    # A long thin bright bar (like a cable/edge) must be rejected by aspect/extent.
    img = np.full((480, 640, 3), 35, np.uint8)
    cv2.rectangle(img, (100, 235), (540, 250), (252, 252, 252), -1)  # wide thin bar
    assert detect_white_cubes(img) == []
