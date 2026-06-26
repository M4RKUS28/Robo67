"""Eye-in-hand (hand-to-eye) calibration math for the wrist D405 camera.

Pure-Python helpers for the hand-eye calibration of the D405 mounted on the
gripper. Like :mod:`robo67_insertion.lib.geometry`, this module imports **no
rclpy** and imports ``cv2`` *lazily* (only inside the functions that need it),
so the matrix algebra stays host-testable and the control loop can import it
without pulling in OpenCV.

What hand-eye calibration gives us
----------------------------------
The D405 is bolted to the gripper, so the rigid transform between the camera
frame and the end-effector (EE) frame is FIXED but unknown. We recover it by
showing the camera a ChArUco board from several robot poses and pairing, per
view, the robot pose with the board pose seen by the camera. The result is the
camera's pose in the EE frame, ``T_ee_cam`` (a.k.a. OpenCV's ``cam2gripper``):
a point in the camera frame maps to the EE frame as ``X_ee = T_ee_cam @ X_cam``,
and then to the base frame via the live robot pose:

    X_base = T_base_ee @ T_ee_cam @ X_cam

which is exactly what the port detector needs (Phase 5) to turn a wrist-camera
pixel + depth into an ABSOLUTE base-frame XYZ (overhead can't give Z).

OpenCV frame-naming convention (used verbatim here to avoid ambiguity)
----------------------------------------------------------------------
``T_a2b`` is the transform that maps a point expressed in frame ``a`` into frame
``b`` (i.e. the pose of ``a`` expressed in ``b``). :func:`cv2.calibrateHandEye`
takes, per view:

* ``gripper2base`` -- pose of the gripper in the base frame. Franka's
  ``FrankaState.o_t_ee`` (``O_T_EE``, column-major 4x4) IS this transform
  (``T_base_ee``); reshape it with :func:`ee_pose_from_o_t_ee`.
* ``target2cam`` -- pose of the board in the camera frame. This is what
  :func:`estimate_board_pose` returns (``solvePnP`` rvec/tvec).

and returns ``cam2gripper`` = ``T_ee_cam``, which :func:`solve_hand_eye`
returns as a single 4x4.

Board-size scale note
---------------------
Only the TRANSLATION of ``T_ee_cam`` scales with ``square_length`` (the metric
size of one ChArUco square); its ROTATION is scale-invariant. Get
``square_length`` roughly right (a few % is fine -- the offset is only a few cm,
so a few % is sub-mm to low-mm, within the insertion's spiral search).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

__all__ = [
    "DEFAULT_DICTIONARY",
    "MIN_VIEWS",
    "HAND_EYE_METHODS",
    "CharucoBoardSpec",
    "make_charuco_board",
    "detect_charuco",
    "estimate_board_pose",
    "rvectvec_to_matrix",
    "matrix_to_rvectvec",
    "invert_transform",
    "ee_pose_from_o_t_ee",
    "solve_hand_eye",
    "hand_eye_residual",
]

# Default ArUco dictionary for the ChArUco board (plenty of unique markers for a
# small board; matches the typical online ChArUco generators / tablet images).
DEFAULT_DICTIONARY = "DICT_5X5_1000"

# calibrateHandEye needs >= 3 absolute poses (>= 2 relative motions) with varied
# rotation; we recommend 8-12 well-spread views for a stable solve.
MIN_VIEWS = 3

# Friendly name -> cv2.CALIB_HAND_EYE_* constant name. Resolved lazily so this
# module imports without cv2.
HAND_EYE_METHODS = {
    "tsai": "CALIB_HAND_EYE_TSAI",
    "park": "CALIB_HAND_EYE_PARK",
    "horaud": "CALIB_HAND_EYE_HORAUD",
    "andreff": "CALIB_HAND_EYE_ANDREFF",
    "daniilidis": "CALIB_HAND_EYE_DANIILIDIS",
}


@dataclass(frozen=True)
class CharucoBoardSpec:
    """A ChArUco board description (metric sizes in METERS).

    ``squares_x``/``squares_y`` are the number of chessboard squares across/down
    (the board's full grid). ``square_length`` is the side of one square and
    ``marker_length`` is the side of the ArUco marker printed inside the white
    squares -- both in meters (e.g. ``square_length=0.025`` for 2.5 cm squares).
    """

    squares_x: int = 5
    squares_y: int = 7
    square_length: float = 0.025
    marker_length: float = 0.01875  # 0.75 * square_length (typical generator ratio)
    dictionary: str = DEFAULT_DICTIONARY


def _aruco_dictionary(name: str):
    """Resolve a predefined ArUco dictionary by name (new or old cv2 API)."""
    import cv2

    const = getattr(cv2.aruco, name, None)
    if const is None:
        raise ValueError(
            f"unknown ArUco dictionary {name!r}; expected e.g. 'DICT_5X5_1000'"
        )
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(const)
    return cv2.aruco.Dictionary_get(const)  # cv2 < 4.7


def make_charuco_board(spec: CharucoBoardSpec):
    """Build the OpenCV ChArUco board object + its ArUco dictionary.

    Returns ``(board, dictionary)``. Supports both the new (>= 4.7
    ``CharucoBoard((cols, rows), ...)``) and old (``CharucoBoard_create``) APIs.
    """
    import cv2

    dictionary = _aruco_dictionary(spec.dictionary)
    if hasattr(cv2.aruco, "CharucoBoard") and hasattr(cv2.aruco, "CharucoDetector"):
        board = cv2.aruco.CharucoBoard(
            (spec.squares_x, spec.squares_y),
            float(spec.square_length),
            float(spec.marker_length),
            dictionary,
        )
    else:  # cv2 < 4.7
        board = cv2.aruco.CharucoBoard_create(
            spec.squares_x,
            spec.squares_y,
            float(spec.square_length),
            float(spec.marker_length),
            dictionary,
        )
    return board, dictionary


def _board_chessboard_corners(board) -> np.ndarray:
    """The board's inner-corner 3D coords (M, 3), across cv2 API versions."""
    getter = getattr(board, "getChessboardCorners", None)
    if getter is not None:
        return np.asarray(getter(), dtype=np.float32)
    return np.asarray(board.chessboardCorners, dtype=np.float32)  # cv2 < 4.7


def detect_charuco(image: np.ndarray, board, dictionary=None):
    """Detect ChArUco inner corners in a BGR/gray image.

    Returns ``(charuco_corners, charuco_ids)`` with shapes ``(N, 1, 2)`` /
    ``(N, 1)`` (OpenCV's native layout), or ``(None, None)`` if no corners were
    interpolated. ``dictionary`` is only needed on the old (< 4.7) cv2 API.
    """
    import cv2

    if hasattr(cv2.aruco, "CharucoDetector"):  # new API (>= 4.7)
        detector = cv2.aruco.CharucoDetector(board)
        charuco_corners, charuco_ids, _marker_corners, _marker_ids = detector.detectBoard(
            image
        )
        if charuco_corners is None or len(charuco_corners) == 0:
            return None, None
        return charuco_corners, charuco_ids

    # old API (< 4.7): detectMarkers -> interpolateCornersCharuco
    if dictionary is None:
        raise ValueError("the old cv2.aruco API requires the dictionary for detection")
    corners, ids, _rej = cv2.aruco.detectMarkers(image, dictionary)
    if ids is None or len(ids) == 0:
        return None, None
    n, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, image, board
    )
    if not n or charuco_corners is None or len(charuco_corners) == 0:
        return None, None
    return charuco_corners, charuco_ids


def estimate_board_pose(
    charuco_corners: np.ndarray,
    charuco_ids: np.ndarray,
    board,
    camera_matrix: np.ndarray,
    dist_coeffs: Optional[np.ndarray] = None,
    min_corners: int = 4,
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """Estimate the board pose in the camera frame (``target2cam``).

    Returns ``(rvec, tvec)`` (each shape ``(3, 1)``) such that a board point
    maps to the camera frame as ``X_cam = R(rvec) @ X_target + tvec``, or
    ``None`` if too few corners / the solve fails. Needs >= ``min_corners``
    (solvePnP needs >= 4 for a stable planar pose).
    """
    import cv2

    if charuco_corners is None or charuco_ids is None:
        return None
    if len(charuco_corners) < min_corners:
        return None

    camera_matrix = np.asarray(camera_matrix, dtype=np.float64)
    if dist_coeffs is None:
        dist_coeffs = np.zeros((5, 1), dtype=np.float64)
    else:
        dist_coeffs = np.asarray(dist_coeffs, dtype=np.float64)

    # Preferred path (works on both APIs): match charuco corners to board object
    # points, then solvePnP. matchImagePoints exists on the new board objects.
    matcher = getattr(board, "matchImagePoints", None)
    if matcher is not None:
        obj_points, img_points = matcher(charuco_corners, charuco_ids)
        if obj_points is None or len(obj_points) < min_corners:
            return None
        ok, rvec, tvec = cv2.solvePnP(
            np.asarray(obj_points, dtype=np.float64),
            np.asarray(img_points, dtype=np.float64),
            camera_matrix,
            dist_coeffs,
        )
        if not ok:
            return None
        return np.asarray(rvec, float).reshape(3, 1), np.asarray(tvec, float).reshape(3, 1)

    # old API fallback
    ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
        charuco_corners, charuco_ids, board, camera_matrix, dist_coeffs, None, None
    )
    if not ok:
        return None
    return np.asarray(rvec, float).reshape(3, 1), np.asarray(tvec, float).reshape(3, 1)


# --------------------------------------------------------------------------- #
# Rigid-transform helpers (4x4 homogeneous).
# --------------------------------------------------------------------------- #
def rvectvec_to_matrix(rvec, tvec) -> np.ndarray:
    """Rodrigues ``(rvec, tvec)`` -> 4x4 homogeneous transform."""
    import cv2

    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(tvec, dtype=float).reshape(3)
    return T


def matrix_to_rvectvec(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """4x4 homogeneous transform -> Rodrigues ``(rvec[3,1], tvec[3,1])``."""
    import cv2

    T = np.asarray(T, dtype=float)
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    return rvec.reshape(3, 1), T[:3, 3].reshape(3, 1)


def invert_transform(T: np.ndarray) -> np.ndarray:
    """Inverse of a 4x4 rigid transform (transpose R, ``-R^T t``)."""
    T = np.asarray(T, dtype=float)
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=float)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def ee_pose_from_o_t_ee(o_t_ee: Sequence[float]) -> np.ndarray:
    """Franka ``O_T_EE`` (len-16, COLUMN-major) -> 4x4 ``T_base_ee``.

    This is the gripper-in-base pose (``gripper2base``) that
    :func:`solve_hand_eye` expects.
    """
    return np.asarray(o_t_ee, dtype=float).reshape(4, 4, order="F")


def _split_rt(transforms: Sequence[np.ndarray]) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    R = [np.asarray(T, float)[:3, :3] for T in transforms]
    t = [np.asarray(T, float)[:3, 3].reshape(3, 1) for T in transforms]
    return R, t


def solve_hand_eye(
    gripper2base: Sequence[np.ndarray],
    target2cam: Sequence[np.ndarray],
    method: str = "tsai",
) -> np.ndarray:
    """Solve the eye-in-hand calibration -> ``T_ee_cam`` (``cam2gripper``, 4x4).

    Parameters
    ----------
    gripper2base : sequence of 4x4
        Per view, the gripper pose in the base frame (``T_base_ee``; from
        :func:`ee_pose_from_o_t_ee`).
    target2cam : sequence of 4x4
        Per view, the board pose in the camera frame (from
        :func:`estimate_board_pose` via :func:`rvectvec_to_matrix`).
    method : str
        One of :data:`HAND_EYE_METHODS` keys (default ``"tsai"``).

    Returns
    -------
    np.ndarray
        ``T_ee_cam`` (4x4): the camera pose in the EE frame, such that
        ``X_ee = T_ee_cam @ X_cam``.
    """
    import cv2

    if len(gripper2base) != len(target2cam):
        raise ValueError("gripper2base and target2cam must have equal length")
    if len(gripper2base) < MIN_VIEWS:
        raise ValueError(f"need >= {MIN_VIEWS} views, got {len(gripper2base)}")
    if method not in HAND_EYE_METHODS:
        raise ValueError(
            f"unknown method {method!r}; choose from {sorted(HAND_EYE_METHODS)}"
        )

    R_g2b, t_g2b = _split_rt(gripper2base)
    R_t2c, t_t2c = _split_rt(target2cam)
    flag = getattr(cv2, HAND_EYE_METHODS[method])
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_g2b, t_g2b, R_t2c, t_t2c, method=flag
    )
    T = np.eye(4, dtype=float)
    T[:3, :3] = np.asarray(R_cam2gripper, float)
    T[:3, 3] = np.asarray(t_cam2gripper, float).reshape(3)
    return T


def hand_eye_residual(
    gripper2base: Sequence[np.ndarray],
    target2cam: Sequence[np.ndarray],
    T_ee_cam: np.ndarray,
) -> Tuple[float, float]:
    """Consistency residual for a hand-eye solution.

    The board is fixed in the base frame, so the inferred board-in-base pose
    ``target2base_i = gripper2base_i @ T_ee_cam @ target2cam_i`` should be
    IDENTICAL across views. We return the spread of that pose:

    * ``trans_rms_m``  -- RMS distance of each view's inferred board ORIGIN from
      the mean origin (meters).
    * ``rot_rms_deg``  -- RMS geodesic rotation angle of each view's inferred
      board ORIENTATION from the mean orientation (degrees).

    Small values (sub-mm to low-mm, sub-degree) indicate a good calibration.
    """
    import cv2

    origins = []
    rots = []
    for g2b, t2c in zip(gripper2base, target2cam):
        target2base = np.asarray(g2b, float) @ np.asarray(T_ee_cam, float) @ np.asarray(
            t2c, float
        )
        origins.append(target2base[:3, 3])
        rots.append(target2base[:3, :3])
    origins = np.asarray(origins, float)
    mean_origin = origins.mean(axis=0)
    trans_rms = float(np.sqrt(np.mean(np.sum((origins - mean_origin) ** 2, axis=1))))

    # Mean rotation via SVD orthonormalization of the average matrix.
    R_mean = np.mean(np.asarray(rots, float), axis=0)
    U, _, Vt = np.linalg.svd(R_mean)
    R_mean = U @ Vt
    if np.linalg.det(R_mean) < 0:
        U[:, -1] *= -1
        R_mean = U @ Vt
    angles = []
    for R in rots:
        rvec, _ = cv2.Rodrigues(R.T @ R_mean)
        angles.append(np.linalg.norm(rvec))
    rot_rms_deg = float(np.degrees(np.sqrt(np.mean(np.square(angles)))))
    return trans_rms, rot_rms_deg
