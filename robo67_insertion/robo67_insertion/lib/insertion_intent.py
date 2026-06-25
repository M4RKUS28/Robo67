"""Canonical, controller-agnostic peg-in-hole insertion intent (pure).

This is the single source of truth for insertion phase semantics (ADR-0001).
Given ``(phase, IntentSensors)`` :meth:`InsertionIntentModule.step` returns an
:class:`InsertionIntent`: the NEXT phase plus an absolute, base-frame
``target_xyz``. It owns ALL of the transition logic that used to be duplicated
across the sim FSM (``lib.insertion_fsm``) and the real-arm sequence
(``nodes.hardware_insertion_node.InsertionSequence``):

* contact detection (via :func:`lib.wrench.contact_detected`),
* the Archimedean spiral offset (via :func:`lib.spiral.archimedean_offset`),
* drop detection and ``hole_xy`` bookkeeping,
* spiral retry / exhaustion,
* confirmation + retract criteria,
* ``contact_z`` bookkeeping.

Controller QUIRKS (lead-clamp carrot, below-surface equilibrium gaps, the held
orientation) live in the command-path ADAPTERS (``lib.command_path_adapters``),
NOT here. The targets emitted here are absolute and controller-neutral:

Conventions
-----------
* Z is UP in the robot base frame, so *descending* means *decreasing z*.
* "Contact" records ``contact_z`` (the socket-top surface height). The peg
  drops INTO the hole when ``ee_z`` falls below ``contact_z``.

Only numpy + stdlib are used here, plus two sibling pure modules. This module
must NOT import rclpy/ROS/cv2/scipy so it stays unit-testable on any host.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, Optional, Tuple

import numpy as np

from robo67_insertion.lib.spiral import archimedean_offset
from robo67_insertion.lib.wrench import contact_detected

__all__ = [
    "PHASES",
    "Phase",
    "IntentParams",
    "IntentSensors",
    "InsertionIntent",
    "InsertionIntentModule",
]

Phase = Literal[
    "IDLE",
    "MOVE_ABOVE",
    "DESCEND_TO_CONTACT",
    "SEARCH_SPIRAL",
    "PUSH_INSERT",
    "CONFIRM",
    "RETRACT",
    "DONE",
    "ERROR",
]

# Canonical, ordered set of insertion phases.
PHASES = (
    "IDLE",
    "MOVE_ABOVE",
    "DESCEND_TO_CONTACT",
    "SEARCH_SPIRAL",
    "PUSH_INSERT",
    "CONFIRM",
    "RETRACT",
    "DONE",
    "ERROR",
)


@dataclass(frozen=True)
class IntentParams:
    """Controller-AGNOSTIC insertion parameters (SI units)."""

    standoff_m: float = 0.05
    approach_tol_m: float = 0.003
    contact_fz_threshold_n: float = 5.0
    insert_depth_m: float = 0.04
    z_drop_threshold_m: float = 0.004
    retry_limit: int = 3
    spiral_pitch_m: float = 0.002
    spiral_speed_mps: float = 0.005
    spiral_max_radius_m: float = 0.012


@dataclass(frozen=True)
class IntentSensors:
    """Normalized sensor snapshot consumed by :meth:`InsertionIntentModule.step`."""

    ee_xyz: Tuple[float, float, float]
    fz: float
    fz_baseline: float
    t: float


@dataclass(frozen=True)
class InsertionIntent:
    """Controller-agnostic insertion intent returned by the canonical module."""

    phase: Phase  # the NEXT phase
    target_xyz: Tuple[float, float, float]  # absolute base-frame goal
    contact_z: Optional[float]  # socket-top contact height once found
    done: bool = False
    error: Optional[str] = None


class InsertionIntentModule:
    """Canonical, controller-agnostic insertion transition model.

    Args:
        socket_xyz: Socket TOP center in the robot base frame, shape (3,).
        params: Tunable :class:`IntentParams`.
    """

    def __init__(self, socket_xyz, params: IntentParams = IntentParams()):
        self.socket = np.asarray(socket_xyz, float).reshape(3)
        self.params = params
        self.contact_z: Optional[float] = None
        self.hole_xy: Optional[Tuple[float, float]] = None
        self.spiral_t0: Optional[float] = None
        self.retries = 0
        self.error: Optional[str] = None

    # -- helpers ---------------------------------------------------------

    @staticmethod
    def _t3(x, y, z) -> Tuple[float, float, float]:
        """Return a finite 3-tuple of floats."""
        return (float(x), float(y), float(z))

    def _intent(self, target_xyz, phase, done=False, error=None) -> InsertionIntent:
        return InsertionIntent(
            phase=phase,
            target_xyz=self._t3(*target_xyz),
            contact_z=(None if self.contact_z is None else float(self.contact_z)),
            done=done,
            error=error,
        )

    # -- main entry point ------------------------------------------------

    def step(self, phase: str, s: IntentSensors) -> InsertionIntent:
        """Advance one tick: return the NEXT phase + an absolute canonical target."""
        p = self.params
        ee = np.asarray(s.ee_xyz, float).reshape(3)
        sx, sy, sz = self.socket

        if phase == "IDLE":
            return self._intent((sx, sy, sz + p.standoff_m), "MOVE_ABOVE")

        if phase == "MOVE_ABOVE":
            target = (sx, sy, sz + p.standoff_m)
            arrived = np.linalg.norm(ee - np.asarray(target)) <= p.approach_tol_m
            return self._intent(target, "DESCEND_TO_CONTACT" if arrived else "MOVE_ABOVE")

        if phase == "DESCEND_TO_CONTACT":
            if contact_detected(s.fz, s.fz_baseline, p.contact_fz_threshold_n):
                self.contact_z = float(ee[2])
                self.spiral_t0 = float(s.t)
                self.retries = 0
                # contact plane (spiral center); adapters add their press bias
                return self._intent((sx, sy, self.contact_z), "SEARCH_SPIRAL")
            # aim DOWN past the surface so both controllers keep descending
            return self._intent((sx, sy, sz - p.insert_depth_m), "DESCEND_TO_CONTACT")

        if phase == "SEARCH_SPIRAL":
            elapsed = s.t - (self.spiral_t0 if self.spiral_t0 is not None else s.t)
            dx, dy = archimedean_offset(elapsed, p.spiral_pitch_m, p.spiral_speed_mps)
            if (dx * dx + dy * dy) ** 0.5 > p.spiral_max_radius_m:
                # spiral grew past its bound: restart from the center
                self.retries += 1
                self.spiral_t0 = float(s.t)
                if self.retries > p.retry_limit:
                    self.error = "spiral search exhausted"
                    return self._intent(tuple(ee), "ERROR", done=True, error=self.error)
                dx = 0.0
                dy = 0.0
            cz = self.contact_z if self.contact_z is not None else float(sz)
            if self.contact_z is not None and ee[2] < self.contact_z - p.z_drop_threshold_m:
                # peg dropped into the hole: record where, keep emitting spiral target
                self.hole_xy = (float(ee[0]), float(ee[1]))
                nxt = "PUSH_INSERT"
            else:
                nxt = "SEARCH_SPIRAL"
            return self._intent((sx + dx, sy + dy, cz), nxt)

        if phase == "PUSH_INSERT":
            hx, hy = self.hole_xy if self.hole_xy is not None else (float(sx), float(sy))
            target_z = self.contact_z - p.insert_depth_m
            seated = ee[2] <= target_z + 0.5 * p.z_drop_threshold_m
            return self._intent((hx, hy, target_z), "CONFIRM" if seated else "PUSH_INSERT")

        if phase == "CONFIRM":
            if ee[2] <= self.contact_z - 0.8 * p.insert_depth_m:
                return self._intent((sx, sy, sz + p.standoff_m), "RETRACT")
            self.retries += 1
            self.spiral_t0 = float(s.t)
            if self.retries > p.retry_limit:
                self.error = "insertion not confirmed"
                return self._intent(tuple(ee), "ERROR", done=True, error=self.error)
            return self._intent(tuple(ee), "SEARCH_SPIRAL")

        if phase == "RETRACT":
            target = (sx, sy, sz + p.standoff_m)
            done_now = ee[2] >= self.contact_z
            return self._intent(target, "DONE" if done_now else "RETRACT")

        if phase == "DONE":
            return self._intent(tuple(ee), "DONE", done=True)

        if phase == "ERROR":
            return self._intent(tuple(ee), "ERROR", done=True, error=self.error)

        # Unknown phase -> fail safe into ERROR (holds position).
        self.error = "unknown phase: %r" % (phase,)
        return self._intent(tuple(ee), "ERROR", done=True, error=self.error)
