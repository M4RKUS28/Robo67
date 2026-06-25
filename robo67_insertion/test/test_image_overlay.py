"""Tests for the pure detection-overlay seam (lib.image_overlay)."""
import numpy as np

from robo67_insertion.lib.hole_detect import Hole
from robo67_insertion.lib.image_overlay import (
    decode_jpeg,
    draw_servo_overlay,
    draw_socket_overlay,
    encode_jpeg,
)


def _img(h=120, w=160):
    # mid-grey so coloured annotations clearly differ from the background
    return np.full((h, w, 3), 60, dtype=np.uint8)


def test_socket_overlay_preserves_shape_and_dtype():
    img = _img()
    out = draw_socket_overlay(img, [Hole(80.0, 60.0, 18.0, 0.9)])
    assert out.shape == img.shape
    assert out.dtype == np.uint8


def test_socket_overlay_does_not_mutate_input():
    img = _img()
    before = img.copy()
    draw_socket_overlay(img, [Hole(80.0, 60.0, 18.0, 0.9)])
    assert np.array_equal(img, before)


def test_socket_overlay_actually_draws_something():
    img = _img()
    out = draw_socket_overlay(img, [Hole(80.0, 60.0, 18.0, 0.9)])
    assert not np.array_equal(out, img)  # pixels changed


def test_socket_overlay_empty_holes_returns_unmodified_copy():
    img = _img()
    out = draw_socket_overlay(img, [])
    assert out.shape == img.shape
    assert np.array_equal(out, img)
    assert out is not img  # a copy, safe to publish


def test_socket_overlay_accepts_grayscale():
    gray = np.full((120, 160), 60, dtype=np.uint8)
    out = draw_socket_overlay(gray, [Hole(80.0, 60.0, 18.0, 0.9)])
    assert out.ndim == 3 and out.shape[2] == 3


def test_socket_overlay_with_base_xy_label():
    img = _img()
    out = draw_socket_overlay(img, [Hole(80.0, 60.0, 18.0, 0.9)],
                              base_xy=(0.45, -0.02))
    assert not np.array_equal(out, img)


def test_servo_overlay_draws_arrow_and_ring():
    img = _img()
    out = draw_servo_overlay(img, [Hole(100.0, 40.0, 12.0, 0.7)],
                             servo_dxy_m=(0.01, -0.005))
    assert out.shape == img.shape
    assert not np.array_equal(out, img)


def test_servo_overlay_no_detection_still_draws_center_marker():
    img = _img()
    out = draw_servo_overlay(img, [])
    # centre crosshair is still drawn even with no hole
    assert not np.array_equal(out, img)


def test_jpeg_roundtrip():
    img = _img()
    data = encode_jpeg(img, quality=90)
    assert isinstance(data, (bytes, bytearray)) and len(data) > 0
    back = decode_jpeg(data)
    assert back is not None
    assert back.shape == img.shape
    assert back.dtype == np.uint8


def test_decode_jpeg_empty_returns_none():
    assert decode_jpeg(b"") is None
