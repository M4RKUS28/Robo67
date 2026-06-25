"""Tests for robo67_insertion.lib.geometry (pure-Python geometry helpers)."""

import numpy as np

from robo67_insertion.lib.geometry import (
    fit_homography,
    mat4_colmajor_to_xyz_quat,
    pixel_to_base,
)


PIXELS = np.array(
    [
        [100, 100],
        [500, 100],
        [100, 400],
        [500, 400],
        [300, 250],
        [200, 350],
    ],
    dtype=float,
)

H_TRUE = np.array(
    [
        [0.0005, 0.0, 0.2],
        [0.0, 0.0005, -0.1],
        [0.0, 0.0, 1.0],
    ]
)


def test_homography_round_trip():
    base_xy = pixel_to_base(H_TRUE, PIXELS)
    assert base_xy.shape == (6, 2)

    H_fit = fit_homography(PIXELS, base_xy)
    assert H_fit.shape == (3, 3)

    recovered = pixel_to_base(H_fit, PIXELS)
    assert np.allclose(recovered, base_xy, atol=1e-6)


def test_pixel_to_base_single_point():
    pt = np.array([300.0, 250.0])
    out = pixel_to_base(H_TRUE, pt)
    assert out.shape == (2,)

    # Single point must equal the corresponding batch result.
    batch = pixel_to_base(H_TRUE, pt.reshape(1, 2))
    assert batch.shape == (1, 2)
    assert np.allclose(out, batch[0])

    # Compare against manual homogeneous transform.
    vec = H_TRUE @ np.array([300.0, 250.0, 1.0])
    expected = vec[:2] / vec[2]
    assert np.allclose(out, expected)


def test_mat4_identity_translation():
    o_t_ee = [
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0.3, 0.2, 0.5, 1,
    ]
    xyz, quat_xyzw = mat4_colmajor_to_xyz_quat(o_t_ee)
    assert isinstance(xyz, np.ndarray)
    assert isinstance(quat_xyzw, np.ndarray)
    assert xyz.shape == (3,)
    assert quat_xyzw.shape == (4,)
    assert np.allclose(xyz, [0.3, 0.2, 0.5])
    assert np.allclose(quat_xyzw, [0, 0, 0, 1], atol=1e-9)


def test_mat4_rot_z_90():
    theta = np.pi / 2.0
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [0.1, 0.2, 0.3]

    # Column-major flatten (Franka o_t_ee convention).
    o_t_ee = T.reshape(-1, order="F").tolist()

    xyz, quat_xyzw = mat4_colmajor_to_xyz_quat(o_t_ee)
    assert np.allclose(xyz, [0.1, 0.2, 0.3])

    # Unit quaternion.
    assert np.allclose(np.linalg.norm(quat_xyzw), 1.0, atol=1e-9)

    # 90deg about Z: w = cos(45deg) ~= 0.7071, z = sin(45deg) ~= 0.7071,
    # x = y = 0. Allow global sign flip (q and -q are the same rotation).
    expected = np.array([0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)])
    assert np.allclose(np.abs(quat_xyzw), np.abs(expected), atol=1e-6)
    assert np.isclose(abs(quat_xyzw[3]), np.sqrt(0.5), atol=1e-6)
    assert np.isclose(abs(quat_xyzw[2]), np.sqrt(0.5), atol=1e-6)
    assert np.isclose(quat_xyzw[0], 0.0, atol=1e-6)
    assert np.isclose(quat_xyzw[1], 0.0, atol=1e-6)
