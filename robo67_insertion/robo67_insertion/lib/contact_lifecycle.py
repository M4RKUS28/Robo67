"""Contact lifecycle policy for peg-in-hole insertion.

Owns the free-space Fz baseline update/freeze policy that the sim
orchestrator used to glue inline: the EMA baseline is tracked ONLY while
in free space and FROZEN during every contact phase, where the frozen
baseline is the reference for symmetric contact detection.

This is the seam that decides WHEN to update vs. freeze the baseline,
given an explicit contact mode. The low-level primitives
(:class:`~robo67_insertion.lib.wrench.BaselineEstimator` EMA and
:func:`~robo67_insertion.lib.wrench.contact_detected` threshold compare)
are composed here, not reimplemented.

numpy + stdlib only. This module must NOT import rclpy/ROS/cv2/scipy.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from robo67_insertion.lib.wrench import BaselineEstimator, contact_detected

__all__ = ["ContactMode", "ContactOutcome", "ContactLifecycleModule"]

ContactMode = Literal["free_space", "contact_search", "insert", "confirm"]


@dataclass(frozen=True)
class ContactOutcome:
    """Result of folding one Fz sample through the lifecycle policy.

    Attributes:
        baseline_fz: The current baseline Fz (updated in free space,
            frozen otherwise) to feed into downstream contact detection.
        contact_detected: True when a contact-phase deviation reaches the
            threshold. Always False in free space.
    """

    baseline_fz: float
    contact_detected: bool


class ContactLifecycleModule:
    """Baseline update/freeze policy keyed on the contact mode.

    Args:
        threshold_n: Contact detection threshold (magnitude), in N.
        alpha: EMA smoothing factor in (0, 1] for the free-space baseline.
        initial: Starting value of the baseline EMA.
    """

    def __init__(self, threshold_n: float, alpha: float = 0.1, initial: float = 0.0) -> None:
        if threshold_n < 0:
            raise ValueError("threshold_n must be non-negative")
        self._threshold_n = float(threshold_n)
        self._baseline = BaselineEstimator(alpha=alpha, initial=initial)

    def observe(self, mode: ContactMode, fz: float) -> ContactOutcome:
        """Fold one Fz sample through the lifecycle and report the outcome.

        In ``free_space`` the baseline EMA tracks ``fz`` and contact is
        never reported (we are establishing the baseline). In any contact
        mode the baseline is frozen and contact is reported when
        ``abs(fz - baseline) >= threshold_n``.
        """
        if mode not in ("free_space", "contact_search", "insert", "confirm"):
            raise ValueError(f"unknown ContactMode: {mode!r}")
        if mode == "free_space":
            baseline = self._baseline.update(fz)
            return ContactOutcome(baseline_fz=baseline, contact_detected=False)

        baseline = self._baseline.value
        detected = contact_detected(fz, baseline, self._threshold_n)
        return ContactOutcome(baseline_fz=baseline, contact_detected=detected)
