"""Tests for industrial I/O-box detection (cable-insertion overhead vision).

The cable task targets a dark, port-covered I/O box on a similarly-mid-gray
carpet, so absolute brightness cannot separate them. The detector keys on LOCAL
TEXTURE ENERGY: the port face is a dense high-variance island; the carpet is a
uniform low-variance field. These tests lock that behavior on synthetic images
and on a real overhead C920 frame.
"""
import os

import cv2
import numpy as np

from robo67_insertion.lib.box_detect import (
    Box,
    BoxParams,
    detect_gray_box,
    local_texture_std,
)

FIXTURES = os.path.join(os.path.dirname(__file__), "fixtures")


def _low_texture_bg(h=720, w=1280, level=116, noise=4, seed=0):
    """Uniform mid-gray background with mild noise -> LOW local texture (carpet)."""
    rng = np.random.default_rng(seed)
    g = np.full((h, w), float(level)) + rng.normal(0.0, noise, size=(h, w))
    g = np.clip(g, 0, 255).astype(np.uint8)
    return cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)


def _busy_patch(img, x0, y0, bw, bh, seed=1):
    """Paint a HIGH-contrast (high local-variance) rectangle -> a 'port face'."""
    rng = np.random.default_rng(seed)
    patch = rng.integers(0, 256, size=(bh, bw), dtype=np.uint8)
    img[y0:y0 + bh, x0:x0 + bw] = cv2.cvtColor(patch, cv2.COLOR_GRAY2BGR)
    return img


def test_local_texture_std_high_on_noise_low_on_flat():
    flat = np.full((100, 100), 120, np.uint8)
    assert float(local_texture_std(flat, 21).mean()) < 1.0
    rng = np.random.default_rng(0)
    noisy = rng.integers(0, 256, size=(100, 100), dtype=np.uint8)
    assert float(local_texture_std(noisy, 21).mean()) > 40.0


def test_synthetic_busy_rectangle_detected_on_uniform_carpet():
    img = _low_texture_bg()
    cx, cy, bw, bh = 700, 560, 240, 150
    _busy_patch(img, cx - bw // 2, cy - bh // 2, bw, bh)

    boxes = detect_gray_box(img)
    assert len(boxes) >= 1
    top = boxes[0]
    assert isinstance(top, Box)
    assert abs(top.u - cx) < 25
    assert abs(top.v - cy) < 25
    # oriented size recovers the patch dimensions (order-agnostic)
    dims = sorted([top.width_px, top.height_px])
    assert abs(dims[0] - bh) < 30 and abs(dims[1] - bw) < 30
    assert top.corners.shape == (4, 2)


def test_uniform_low_texture_returns_empty():
    # pure carpet, no busy object -> nothing qualifies
    assert detect_gray_box(_low_texture_bg(noise=4)) == []


def test_picks_the_busier_of_two_rectangles():
    img = _low_texture_bg()
    # a big but MILD-texture rectangle (low std) ...
    rng = np.random.default_rng(2)
    big = np.clip(np.full((180, 300), 130.0) + rng.normal(0, 6, (180, 300)), 0, 255).astype(np.uint8)
    img[120:300, 150:450] = cv2.cvtColor(big, cv2.COLOR_GRAY2BGR)
    # ... vs a smaller but VERY busy rectangle (the real port face)
    _busy_patch(img, 760, 470, 220, 150, seed=3)

    boxes = detect_gray_box(img)
    assert boxes, "expected at least the busy rectangle"
    # the busy one wins on score (density * size), not the mild big one
    assert abs(boxes[0].u - (760 + 110)) < 30
    assert abs(boxes[0].v - (470 + 75)) < 30


def test_real_overhead_frame_locates_io_box():
    path = os.path.join(FIXTURES, "c920_io_box.jpg")
    img = cv2.imread(path)
    assert img is not None, f"missing fixture {path}"

    boxes = detect_gray_box(img)
    assert boxes, "no box detected in the real overhead frame"
    top = boxes[0]
    # the dark industrial I/O box (LAN/USB/CAN BUS + fan) sits bottom-center;
    # its texture-blob centroid was measured at ~(679, 580) on this frame.
    assert abs(top.u - 679) < 90, f"u={top.u}"
    assert abs(top.v - 580) < 90, f"v={top.v}"
