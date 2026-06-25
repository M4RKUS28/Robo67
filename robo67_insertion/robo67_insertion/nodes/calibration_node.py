#!/usr/bin/env python3
"""C920 -> robot-base homography calibration tool (robot-as-groundtruth).

Two modes:

* ``fit`` (default, works offline): read a correspondences file of
  ``pixel_u, pixel_v, base_x, base_y`` rows (CSV or .npz with arrays ``pixels``
  and ``base_xy``) and fit + save ``c920_homography.npz`` via
  :func:`~robo67_insertion.lib.geometry.fit_homography`. Prints reprojection error.

* ``capture`` (hardware, Phase 4): the operator jogs the EE (with a marker / the
  peg tip) to N known base XY points **at socket-top height**, and at each point
  a C920 frame is captured and the marker pixel recorded. Then it fits as above.
  Capturing the marker pixel reliably needs the camera path (see PHASE0_VERIFIED.md
  — the container currently lacks camera passthrough + a working cv2), so this mode
  is a documented procedure the hardware operator drives; the math below is shared.

Run (fit from a file):
    python3 -m robo67_insertion.nodes.calibration_node --ros-args \
        -p mode:=fit -p points_file:=/path/corr.csv -p out:=config/c920_homography.npz
"""
from __future__ import annotations

import os
import sys

import numpy as np

from robo67_insertion.lib import geometry


def load_correspondences(path: str):
    """Return (pixels[N,2], base_xy[N,2]) from a .npz or CSV file."""
    if path.endswith(".npz"):
        d = np.load(path)
        return np.asarray(d["pixels"], float), np.asarray(d["base_xy"], float)
    rows = np.loadtxt(path, delimiter=",", comments="#")
    rows = np.atleast_2d(rows)
    return rows[:, 0:2].astype(float), rows[:, 2:4].astype(float)


def fit_and_save(pixels: np.ndarray, base_xy: np.ndarray, out_path: str):
    """Fit a homography, save it, and return (H, rms_error_m)."""
    if len(pixels) < 4:
        raise ValueError("need >= 4 correspondences")
    H = geometry.fit_homography(pixels, base_xy)
    reproj = geometry.pixel_to_base(H, pixels)
    rms = float(np.sqrt(np.mean(np.sum((reproj - base_xy) ** 2, axis=1))))
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    np.savez(out_path, H=H, pixels=pixels, base_xy=base_xy, rms_error_m=rms)
    return H, rms


def main(args=None):
    # Allow plain CLI use without a full ROS context for the offline 'fit' mode.
    import rclpy

    rclpy.init(args=args)
    node = rclpy.create_node("calibration_node")
    node.declare_parameter("mode", "fit")
    node.declare_parameter("points_file", "")
    node.declare_parameter("out", "config/c920_homography.npz")

    mode = node.get_parameter("mode").value
    points_file = node.get_parameter("points_file").value
    out = node.get_parameter("out").value

    if mode == "fit":
        if not points_file or not os.path.exists(points_file):
            node.get_logger().error(f"points_file not found: {points_file!r}")
            rclpy.shutdown()
            sys.exit(1)
        pixels, base_xy = load_correspondences(points_file)
        H, rms = fit_and_save(pixels, base_xy, out)
        node.get_logger().info(
            f"fitted homography from {len(pixels)} points; RMS reproj error = {rms*1000:.2f} mm; saved {out}"
        )
    else:
        node.get_logger().error(
            "capture mode runs on hardware (Phase 4); see module docstring + PHASE0_VERIFIED.md"
        )
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
