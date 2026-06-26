"""Force-slacken + confirmed-descent insertion-event detector (pure).

Replaces the fragile absolute z-drop trigger: insertion is detected when the
filtered press MAGNITUDE drops by at least ``slacken_frac`` of the recently-held
press (the bore opened, so the surface stops resisting) AND the EE then descends
by at least ``confirm_drop_m`` within ``confirm_window_s`` while still being
driven down. Coupling the force-slacken with a confirmed descent rejects
momentary noise dips on the estimated external wrench.

``press_n`` is the press MAGNITUDE ``|fz_meas - fz_baseline|`` (N), the same
convention as :class:`~robo67_insertion.lib.force_regulator.AxialForceRegulator`.

See ADR-0002 and ``docs/architecture/force-guided-insertion-2026-06-26.md``.

numpy + stdlib only. This module must NOT import rclpy/ROS/cv2/scipy.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

__all__ = ["InsertionEventParams", "InsertionEvent", "InsertionEventDetector"]


@dataclass(frozen=True)
class InsertionEventParams:
    """Tunable parameters for :class:`InsertionEventDetector`."""

    fz_filter_alpha: float = 0.2     # EMA smoothing of the press estimate, (0, 1]
    slacken_frac: float = 0.4        # fraction of held press lost = slacken
    confirm_drop_m: float = 0.003    # EE descent to confirm entry
    confirm_window_s: float = 1.0    # confirm must occur within this of slacken
    min_press_n: float = 2.0         # ignore slacken until a real press was held


@dataclass(frozen=True)
class InsertionEvent:
    """One detector tick's verdict."""

    press_filt_n: float
    slacken: bool
    inserted: bool


class InsertionEventDetector:
    """Detect insertion from a force-slacken followed by a confirmed descent."""

    def __init__(self, params: InsertionEventParams) -> None:
        if not (0.0 < params.fz_filter_alpha <= 1.0):
            raise ValueError("fz_filter_alpha must be in (0, 1]")
        self.p = params
        self._press_filt: Optional[float] = None
        self._press_hold = 0.0
        self._slack_latched = False
        self._t_slack = 0.0
        self._z_slack = 0.0

    def observe(self, press_n: float, z_ee: float, descending: bool,
                t: float) -> InsertionEvent:
        """Fold one sample through the detector and return the verdict.

        Args:
            press_n: Press magnitude ``|fz - fz_baseline|`` (N).
            z_ee: Measured EE height (m).
            descending: True when the regulator is currently commanding the
                equilibrium downward (used to qualify the confirm).
            t: Monotonic time (s).
        """
        a = self.p.fz_filter_alpha
        pf = float(press_n) if self._press_filt is None else (
            (1.0 - a) * self._press_filt + a * float(press_n))
        self._press_filt = pf

        slacken = (self._press_hold >= self.p.min_press_n
                   and pf < (1.0 - self.p.slacken_frac) * self._press_hold)

        if slacken and not self._slack_latched:
            self._slack_latched = True
            self._t_slack = float(t)
            self._z_slack = float(z_ee)

        inserted = False
        if self._slack_latched:
            within = (float(t) - self._t_slack) <= self.p.confirm_window_s
            sank = (self._z_slack - float(z_ee)) >= self.p.confirm_drop_m
            if within and descending and sank:
                inserted = True
            elif not within:
                self._slack_latched = False        # expired -> resume tracking

        if not slacken:                            # don't let the drop raise the ref
            self._press_hold = max(self._press_hold, pf)

        return InsertionEvent(press_filt_n=pf, slacken=slacken, inserted=inserted)
