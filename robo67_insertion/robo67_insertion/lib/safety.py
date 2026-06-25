"""Pure-Python safety clamps for Cartesian setpoints.

These are the non-negotiable safety clamps applied to every commanded
Cartesian setpoint before it is published to the robot. The functions here
are intentionally dependency-light (numpy + stdlib only) so they can be
unit-tested without rclpy/ROS and reused anywhere in the stack.

Functions
---------
clamp_to_workspace : clip a target position into an axis-aligned bounding box.
clamp_step         : cap the per-cycle Cartesian step (velocity limit).
force_exceeded     : check a measured wrench against absolute caps.
"""

import numpy as np


def clamp_to_workspace(xyz, aabb):
    """Clip a Cartesian position into an axis-aligned bounding box.

    Parameters
    ----------
    xyz : sequence/array, len 3
        (x, y, z) position in robot base frame, meters.
    aabb : array-like, shape (3, 2)
        [[xmin, xmax], [ymin, ymax], [zmin, zmax]].

    Returns
    -------
    np.ndarray, shape (3,)
        Each coordinate clipped into its [min, max] range.
    """
    xyz = np.asarray(xyz, dtype=float).reshape(3)
    aabb = np.asarray(aabb, dtype=float).reshape(3, 2)
    lo = aabb[:, 0]
    hi = aabb[:, 1]
    return np.clip(xyz, lo, hi)


def clamp_step(prev_xyz, target_xyz, max_step_m):
    """Limit a Cartesian move so the per-cycle step stays bounded.

    Acts as a per-cycle velocity cap: if the Euclidean distance from
    ``prev_xyz`` to ``target_xyz`` is within ``max_step_m`` the target is
    returned unchanged; otherwise the move is scaled down to exactly
    ``max_step_m`` along the same direction.

    Parameters
    ----------
    prev_xyz : sequence/array, len 3
        Current/previous commanded position, meters.
    target_xyz : sequence/array, len 3
        Desired target position, meters.
    max_step_m : float
        Maximum allowed Euclidean step per cycle, meters.

    Returns
    -------
    np.ndarray, shape (3,)
        The (possibly scaled) target position.
    """
    prev = np.asarray(prev_xyz, dtype=float).reshape(3)
    target = np.asarray(target_xyz, dtype=float).reshape(3)
    delta = target - prev
    dist = float(np.linalg.norm(delta))
    if dist <= max_step_m:
        return target
    return prev + delta * (max_step_m / dist)


def force_exceeded(wrench6, caps6):
    """Return True if any wrench component exceeds its absolute cap.

    Parameters
    ----------
    wrench6 : sequence/array, len 6
        Measured wrench [Fx, Fy, Fz, Mx, My, Mz].
    caps6 : sequence/array, len 6
        Absolute caps for each corresponding component.

    Returns
    -------
    bool
        True if abs(wrench6[i]) > caps6[i] for any i (strictly greater).
    """
    wrench = np.asarray(wrench6, dtype=float).reshape(6)
    caps = np.asarray(caps6, dtype=float).reshape(6)
    return bool(np.any(np.abs(wrench) > caps))
