"""Pure-Python geometry helpers for pixel <-> robot-base mapping and pose math.

This module is intentionally free of any ROS / rclpy dependency so it can be
unit-tested in isolation. It depends only on numpy and OpenCV (cv2).
"""

from __future__ import annotations

import cv2
import numpy as np


def fit_homography(pixels: np.ndarray, base_xy: np.ndarray) -> np.ndarray:
    """Fit a 3x3 homography mapping image pixels to robot-base (x, y) coords.

    Parameters
    ----------
    pixels : np.ndarray
        Shape ``(N, 2)`` image coordinates ``(u, v)``. ``N >= 4``.
    base_xy : np.ndarray
        Shape ``(N, 2)`` robot base coordinates ``(x, y)`` in meters.

    Returns
    -------
    np.ndarray
        A ``3x3`` homography ``H`` such that
        ``pixel_to_base(H, pixels) ~= base_xy``.
    """
    src = np.asarray(pixels, dtype=float)
    dst = np.asarray(base_xy, dtype=float)
    # method=0 -> least-squares (planar, no outlier rejection).
    H = cv2.findHomography(src, dst, 0)[0]
    return np.asarray(H, dtype=float)


def pixel_to_base(H: np.ndarray, uv: np.ndarray) -> np.ndarray:
    """Apply a homography to image points, returning robot-base coordinates.

    Handles both a single point of shape ``(2,)`` (returns ``(2,)``) and a
    batch of shape ``(..., 2)`` (returns the same leading shape).
    """
    H = np.asarray(H, dtype=float)
    uv = np.asarray(uv, dtype=float)
    shape = uv.shape

    flat = uv.reshape(-1, 2)
    ones = np.ones((flat.shape[0], 1), dtype=float)
    homog = np.concatenate([flat, ones], axis=1)  # (M, 3)

    proj = homog @ H.T  # (M, 3): [X, Y, W]
    xy = proj[:, :2] / proj[:, 2:3]

    return xy.reshape(shape)


def mat4_colmajor_to_xyz_quat(o_t_ee) -> tuple[np.ndarray, np.ndarray]:
    """Convert a column-major 4x4 homogeneous transform to (xyz, quat_xyzw).

    Parameters
    ----------
    o_t_ee : sequence of length 16
        A 4x4 homogeneous transform stored in COLUMN-MAJOR order, following the
        Franka ``O_T_EE`` convention.

    Returns
    -------
    tuple[np.ndarray, np.ndarray]
        ``(xyz, quat_xyzw)`` where ``xyz`` has shape ``(3,)`` and
        ``quat_xyzw`` is a unit quaternion in ``(x, y, z, w)`` order, shape
        ``(4,)``.
    """
    T = np.asarray(o_t_ee, dtype=float).reshape(4, 4, order="F")
    xyz = T[:3, 3].copy()
    R = T[:3, :3]
    quat_xyzw = _rotation_to_quat_xyzw(R)
    return xyz, quat_xyzw


def _rotation_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a unit quaternion (x, y, z, w).

    Uses Shepperd's method: select the computation branch based on the largest
    diagonal-related term for numerical stability. No scipy required.
    """
    R = np.asarray(R, dtype=float)
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]

    trace = m00 + m11 + m22

    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)  # s = 4 * w
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif (m00 > m11) and (m00 > m22):
        s = 2.0 * np.sqrt(1.0 + m00 - m11 - m22)  # s = 4 * x
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * np.sqrt(1.0 + m11 - m00 - m22)  # s = 4 * y
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m22 - m00 - m11)  # s = 4 * z
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s

    quat = np.array([x, y, z, w], dtype=float)
    norm = np.linalg.norm(quat)
    if norm > 0.0:
        quat = quat / norm
    return quat
