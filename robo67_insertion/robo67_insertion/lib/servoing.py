"""Image-based visual servoing (IBVS) for eye-in-hand D405 peg-in-hole.

Pure-Python (stdlib only; numpy-compatible scalar inputs accepted). This
module deliberately avoids importing rclpy/ROS/cv2/scipy so it can be
unit-tested on any host.

Setup
-----
The D405 is mounted eye-in-hand and the robot tool always points straight
down (vertical). With a vertical tool the D405 image axes map to the
robot-base XY plane up to a sign convention. We detect the socket hole in
the D405 image at pixel ``hole_uv``, compare it to the image principal
point ``image_center``, and convert the pixel error to a base-frame XY
correction using the pinhole model at the measured depth.

Convention
----------
Given a detected hole pixel ``(hole_u, hole_v)``, the principal point
``(center_u, center_v)``, the measured depth ``depth_m`` (meters along the
camera optical axis), and the pinhole focal lengths ``fx``/``fy`` (pixels)::

    du = hole_u - center_u            # pixel error in u
    dv = hole_v - center_v            # pixel error in v

    ex = du * depth_m / fx            # metric error in camera frame X (m)
    ey = dv * depth_m / fy            # metric error in camera frame Y (m)

    dx_base = -gain * ex              # base-frame X correction (m)
    dy_base = -gain * ey              # base-frame Y correction (m)

The negative sign encodes that moving the camera in +x reduces a positive
``du`` error, driving the hole toward the image center. The exact sign
mapping will be validated (and flipped if needed) on hardware, but the
tests lock THIS convention.
"""

__all__ = ["ibvs_correction"]


def ibvs_correction(hole_uv, image_center, depth_m: float, fx: float,
                    fy: float, gain: float) -> tuple[float, float]:
    """Convert a hole pixel error into a base-frame XY correction.

    Args:
        hole_uv: Detected hole pixel as a len-2 ``(u, v)`` sequence.
        image_center: Principal point as a len-2 ``(u, v)`` sequence.
        depth_m: Measured depth along the camera optical axis, in meters.
        fx: Horizontal focal length, in pixels.
        fy: Vertical focal length, in pixels.
        gain: Proportional servo gain (dimensionless).

    Returns:
        ``(dx_base, dy_base)`` base-frame correction in meters, following
        the convention documented in the module docstring.
    """
    hole_u, hole_v = float(hole_uv[0]), float(hole_uv[1])
    center_u, center_v = float(image_center[0]), float(image_center[1])

    du = hole_u - center_u
    dv = hole_v - center_v

    ex = du * float(depth_m) / float(fx)
    ey = dv * float(depth_m) / float(fy)

    dx_base = -float(gain) * ex
    dy_base = -float(gain) * ey

    return (dx_base, dy_base)
