"""Tests for the offline homography-fit path of the calibration tool.

Only the pure-math helpers are exercised (no rclpy), so this runs on the host.
"""
import os
import tempfile

import numpy as np

from robo67_insertion.lib import geometry
from robo67_insertion.nodes.calibration_node import fit_and_save, load_correspondences


def _synth_correspondences():
    H_true = np.array([[0.0005, 0.0, 0.30], [0.0, 0.0005, -0.10], [0.0, 0.0, 1.0]])
    pixels = np.array([[100, 100], [540, 100], [100, 380], [540, 380],
                       [320, 240], [200, 300]], float)
    base_xy = geometry.pixel_to_base(H_true, pixels)
    return pixels, base_xy


def test_fit_and_save_roundtrip(tmp_path):
    pixels, base_xy = _synth_correspondences()
    out = os.path.join(tmp_path, "c920_homography.npz")
    H, rms = fit_and_save(pixels, base_xy, out)
    assert os.path.exists(out)
    assert rms < 1e-6
    reproj = geometry.pixel_to_base(H, pixels)
    assert np.allclose(reproj, base_xy, atol=1e-6)


def test_load_correspondences_csv(tmp_path):
    pixels, base_xy = _synth_correspondences()
    csv = os.path.join(tmp_path, "corr.csv")
    rows = np.hstack([pixels, base_xy])
    np.savetxt(csv, rows, delimiter=",")
    px, bxy = load_correspondences(csv)
    assert px.shape == pixels.shape and bxy.shape == base_xy.shape
    assert np.allclose(px, pixels) and np.allclose(bxy, base_xy)
