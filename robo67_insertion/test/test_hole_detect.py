"""Tests for white-cube socket detection (peg-in-hole socket vision).

The overhead Logitech C920 looks down at a small WHITE 3D-printed cube socket on
a DARK mat. We detect the cube as a bright square and return its centroid
``(u, v)`` (plus a characteristic radius), robust to overexposure of the
white-on-white bore. Aspect/extent are measured against the rotated min-area
rectangle, so a socket placed at any rotation is still accepted; an area cap
rejects much larger white clutter (boxes, devices).
"""
import cv2
import numpy as np

from robo67_insertion.lib.hole_detect import (
    Hole,
    detect_white_cubes,
)


def _white_socket_image(with_bore=True):
    """Synthetic overhead view: a small WHITE cube on a DARK background, with
    (or without) a bright recessed bore. Mirrors the real Robo67 socket the
    cube detector targets (white-on-dark); the cube body is the keyed feature."""
    img = np.full((480, 640, 3), 35, np.uint8)                  # dark carpet
    cv2.rectangle(img, (290, 200), (350, 260), (252, 252, 252), -1)  # white cube top
    if with_bore:
        cv2.circle(img, (320, 230), 20, (150, 150, 150), 2)     # bore rim shadow (edge)
        cv2.circle(img, (320, 230), 18, (205, 205, 205), -1)    # bore bottom (bright < cube)
    return img


def _rotated_white_cube(center=(320, 240), side=60, angle_deg=35.0, value=252):
    """Synthetic overhead view: a WHITE square cube rotated by ``angle_deg`` on a
    DARK background. A rotated square's *axis-aligned* bounding box is much larger
    than the square (a 45 deg square fills only ~50% of it), so a detector that
    measures fill ('extent') against the axis-aligned bbox wrongly rejects it.
    The cube top is still a filled square, so a rotation-aware (minAreaRect)
    detector must find its centroid at ``center``."""
    img = np.full((480, 640, 3), 35, np.uint8)
    box = cv2.boxPoints((center, (side, side), angle_deg)).astype(np.int32)
    cv2.fillConvexPoly(img, box, (value, value, value))
    return img


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


def test_white_cube_rotated_detected():
    # The socket is often placed ROTATED. A rotated square fills only ~50-70% of
    # its axis-aligned bbox, so the old axis-aligned 'extent' filter rejected it
    # entirely (observed on live overhead frames). A rotation-aware detector must
    # still find the centroid. Exercise a range of angles, incl. the 45 deg worst case.
    for angle in (15.0, 30.0, 45.0, 60.0):
        holes = detect_white_cubes(_rotated_white_cube(angle_deg=angle))
        assert len(holes) >= 1, f"rotated cube missed at {angle} deg"
        assert abs(holes[0].u - 320) < 6 and abs(holes[0].v - 240) < 6


def test_white_cube_rejects_oversized_clutter():
    # The socket is a fixed ~6 cm cube under a fixed overhead camera, so its
    # apparent area is bounded. A much LARGER white square (a packaging box /
    # other clutter) must be rejected by the area cap, even though it is square
    # and bright -- otherwise it (being larger) would be returned over the socket.
    img = np.full((480, 640, 3), 35, np.uint8)
    cv2.rectangle(img, (200, 150), (350, 300), (252, 252, 252), -1)  # 150x150 box (~22.5k px)
    assert detect_white_cubes(img) == []


def test_white_cube_prefers_socket_over_large_box():
    # Socket-sized rotated cube AND a large white box in frame: only the socket
    # (small, square, within the size band) must be returned. The box is square
    # and bright, so it is rejected purely by the area cap (it is far larger than
    # a 6 cm cube can appear), not by aspect/extent.
    img = _rotated_white_cube(center=(420, 300), side=60, angle_deg=40.0)
    cv2.rectangle(img, (80, 60), (230, 210), (252, 252, 252), -1)  # 150x150 clutter box
    holes = detect_white_cubes(img)
    assert len(holes) >= 1
    assert abs(holes[0].u - 420) < 8 and abs(holes[0].v - 300) < 8
