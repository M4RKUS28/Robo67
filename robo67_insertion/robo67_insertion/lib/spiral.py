"""Archimedean spiral search patterns for peg-in-hole XY search.

Pure-Python (numpy + stdlib ``math`` only). This module deliberately avoids
importing rclpy/ROS/cv2/scipy so it can be unit-tested on any host.

An Archimedean spiral has the polar form::

    r = (pitch_m / (2 * pi)) * theta

so the radial distance between successive turns is exactly ``pitch_m``.

Two helpers are provided:

* :func:`archimedean_offset` -- a time-parameterized offset for driving a
  search at an approximately constant linear speed.
* :func:`spiral_waypoints` -- a discrete set of (x, y) waypoints sampled at a
  fixed angular resolution, growing outward until a maximum radius.
"""
import math

import numpy as np

__all__ = ["archimedean_offset", "spiral_waypoints"]


def archimedean_offset(t: float, pitch_m: float,
                       lin_speed_mps: float) -> tuple[float, float]:
    """Return the (dx, dy) spiral offset from the center at time ``t`` seconds.

    The point traverses an Archimedean spiral ``r = (pitch_m / (2*pi)) * theta``
    at an approximately constant linear speed ``lin_speed_mps``. We use the
    simple, well-behaved parameterization::

        theta(t) = sqrt(2 * lin_speed_mps * t * (2*pi) / pitch_m)

    which makes arc length grow ~linearly for small ``theta``. At ``t == 0`` the
    offset is exactly ``(0.0, 0.0)``.

    Parameters
    ----------
    t:
        Elapsed time in seconds (``t >= 0``).
    pitch_m:
        Radial distance between successive spiral turns, in meters (> 0).
    lin_speed_mps:
        Approximate linear traversal speed, in meters/second.

    Returns
    -------
    tuple of float
        ``(dx, dy)`` offset in meters.
    """
    theta = math.sqrt(2.0 * lin_speed_mps * t * (2.0 * math.pi) / pitch_m)
    r = (pitch_m / (2.0 * math.pi)) * theta
    dx = r * math.cos(theta)
    dy = r * math.sin(theta)
    return (dx, dy)


def spiral_waypoints(max_radius_m: float, pitch_m: float,
                     pts_per_rev: int) -> np.ndarray:
    """Sample Archimedean spiral waypoints from the center outward.

    Theta is sampled in steps of ``2*pi / pts_per_rev`` starting at 0, with
    ``r = (pitch_m / (2*pi)) * theta``. Sampling stops once ``r`` would exceed
    ``max_radius_m``. The first waypoint is ``(0.0, 0.0)``.

    Parameters
    ----------
    max_radius_m:
        Maximum radial extent of the spiral, in meters.
    pitch_m:
        Radial distance between successive spiral turns, in meters (> 0).
    pts_per_rev:
        Number of angular samples per full revolution (> 0).

    Returns
    -------
    numpy.ndarray
        Array of shape ``(N, 2)`` of ``(x, y)`` waypoints in meters.
    """
    d_theta = 2.0 * math.pi / pts_per_rev
    k = pitch_m / (2.0 * math.pi)

    points: list[tuple[float, float]] = []
    i = 0
    while True:
        theta = i * d_theta
        r = k * theta
        if r > max_radius_m:
            break
        points.append((r * math.cos(theta), r * math.sin(theta)))
        i += 1

    return np.asarray(points, dtype=float).reshape(-1, 2)
