"""Wrench / force utilities for peg-in-hole contact detection.

Fz is the Z component of the Franka external wrench estimate
``o_f_ext_hat_k`` (index 2). In free space the external Fz sits at a
baseline (gravity/peg compensation residual). On contact, the magnitude
of the deviation ``|Fz - baseline|`` rises above a threshold.

numpy + stdlib only. This module must NOT import rclpy/ROS/cv2/scipy.
"""

__all__ = ["contact_detected", "BaselineEstimator"]


def contact_detected(fz: float, baseline: float, threshold_n: float) -> bool:
    """Return True when Fz deviates from baseline by at least threshold.

    Detection is symmetric: deviations in either direction (positive or
    negative) count once their magnitude reaches ``threshold_n``.

    Args:
        fz: Current Z component of the external wrench estimate, in N.
        baseline: Free-space baseline Fz, in N.
        threshold_n: Detection threshold (magnitude), in N.

    Returns:
        True if ``abs(fz - baseline) >= threshold_n``, else False.
    """
    return abs(fz - baseline) >= threshold_n


class BaselineEstimator:
    """Exponential moving average (EMA) estimator of the Fz baseline.

    The EMA is seeded with ``initial`` and updated as::

        ema = (1 - alpha) * ema + alpha * fz

    Args:
        alpha: Smoothing factor in (0, 1]. Larger values track inputs
            faster; smaller values smooth more heavily.
        initial: Starting value of the EMA.
    """

    def __init__(self, alpha: float = 0.1, initial: float = 0.0) -> None:
        if not (0.0 < alpha <= 1.0):
            raise ValueError("alpha must be in the interval (0, 1]")
        self._alpha = float(alpha)
        self._ema = float(initial)

    def update(self, fz: float) -> float:
        """Fold a new Fz sample into the EMA and return the new value."""
        self._ema = (1.0 - self._alpha) * self._ema + self._alpha * float(fz)
        return self._ema

    @property
    def value(self) -> float:
        """Current EMA value."""
        return self._ema

    def reset(self, value: float = 0.0) -> None:
        """Reset the EMA to ``value`` (default 0.0)."""
        self._ema = float(value)
