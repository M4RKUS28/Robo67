"""Safety envelope composition seam (Candidate 4) -- PURE.

Both orchestrator nodes apply the SAME three safety primitives to every
commanded Cartesian setpoint -- a workspace AABB clamp, a per-cycle step
(velocity) clamp, and a force abort -- but with a critical command-path
ANCHOR-POLICY difference that used to be inlined as node glue:

* the SIM / MMC path anchors the step clamp on the **measured EE** (the MMC
  controller rejects desired poses > 0.1 m from the current pose, so the lead
  must be measured from the actual arm -- "carrot on a stick");
* the REAL / impedance path anchors the step clamp on the **previous command**
  (the equilibrium ratchets down independent of the lagging arm, which is how
  the soft impedance controller builds contact force), and additionally folds a
  socket-top z-floor into the workspace box so the commanded equilibrium never
  goes more than ``max_press_depth_m`` below the socket top.

This module COMPOSES the existing :mod:`robo67_insertion.lib.safety` primitives
(``clamp_to_workspace``, ``clamp_step``, ``force_exceeded``) -- it does NOT
reimplement them -- behind ONE interface (:class:`SafetyEnvelopeModule`) driven
by one of two command-path :class:`profiles <MMCSafetyProfile>`. The clamp
ordering is standardized to **workspace then step**, with the step anchored per
the profile. Because the anchor (ee or prev_cmd) is itself inside the AABB in
normal operation and the box is convex, a workspace-clamped target stepped from
an in-box anchor stays in-box AND velocity-bounded -- so this ordering is safe
for both paths.

numpy + stdlib only. Must NOT import rclpy / ROS / cv2 / scipy.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence, Tuple

import numpy as np

from robo67_insertion.lib import safety

__all__ = [
    "SafetyInput",
    "SafetyOutput",
    "MMCSafetyProfile",
    "ImpedanceSafetyProfile",
    "SafetyEnvelopeModule",
]

# Moment caps are shared by both command paths (Mx, My, Mz).
_MOMENT_CAPS: Tuple[float, float, float] = (5.0, 5.0, 5.0)
# Translational force caps for the MMC (sim) path on the lateral axes.
_MMC_LATERAL_FORCE_CAP: float = 25.0


@dataclass(frozen=True)
class SafetyInput:
    """Everything the envelope needs to evaluate one setpoint.

    Each profile reads only the fields its anchor policy requires; the unused
    ones may carry placeholder values.
    """

    desired_xyz: Sequence[float]   # raw setpoint from the command-path adapter
    ee_xyz: Sequence[float]        # measured EE (MMC anchor)
    prev_cmd_xyz: Sequence[float]  # last published command (Impedance anchor)
    wrench6: Sequence[float]       # measured [Fx, Fy, Fz, Mx, My, Mz]


@dataclass(frozen=True)
class SafetyOutput:
    """The clamped, ready-to-publish setpoint plus the force-abort verdict."""

    safe_xyz: Tuple[float, float, float]
    abort: bool


@dataclass(frozen=True)
class MMCSafetyProfile:
    """SIM / MMC command-path profile: step anchored on the MEASURED EE.

    Args:
        workspace_aabb: Absolute workspace AABB, shape (3, 2).
        max_lead_m: Maximum per-cycle lead AHEAD of the measured EE (the step
            cap). The MMC controller discards desired poses > 0.1 m from the
            current pose, so this lead is measured from the arm, not the
            previous command.
        fz_abort_n: Absolute Fz abort cap; the lateral force caps are 25 N and
            the moment caps are 5 each (``[25, 25, fz_abort_n, 5, 5, 5]``).
    """

    workspace_aabb: Sequence[Sequence[float]]
    max_lead_m: float
    fz_abort_n: float

    @property
    def aabb(self) -> np.ndarray:
        return np.asarray(self.workspace_aabb, float).reshape(3, 2)

    @property
    def max_step(self) -> float:
        return float(self.max_lead_m)

    @property
    def caps6(self) -> Tuple[float, ...]:
        return (_MMC_LATERAL_FORCE_CAP, _MMC_LATERAL_FORCE_CAP,
                float(self.fz_abort_n)) + _MOMENT_CAPS

    def anchor(self, data: SafetyInput) -> Sequence[float]:
        """Step clamp is anchored on the measured EE (carrot-on-a-stick)."""
        return data.ee_xyz


@dataclass(frozen=True)
class ImpedanceSafetyProfile:
    """REAL / impedance command-path profile: step anchored on the PREVIOUS
    COMMAND, with the socket-top z-floor folded into the workspace AABB.

    Folding ``socket_top_z - max_press_depth_m`` into the effective workspace
    z-min keeps the impedance z-floor-below-socket a profile concern (it used
    to be node glue: ``cmd[2] = max(cmd[2], socket[2] - max_press_depth_m)``).
    The effective z-min is ``max(raw_zmin, socket_top_z - max_press_depth_m)``
    so a higher hard workspace floor still wins.

    Args:
        workspace_aabb: Absolute workspace AABB, shape (3, 2).
        max_step_m: Maximum per-cycle Euclidean step on the COMMAND
            (``v_max / rate``); a true command-velocity limit.
        f_abort_n: Absolute abort cap applied to Fx, Fy and Fz
            (``[f_abort, f_abort, f_abort, 5, 5, 5]``).
        socket_top_z: Resolved socket TOP z in the base frame (m).
        max_press_depth_m: How far below the socket top the commanded
            equilibrium may go.
    """

    workspace_aabb: Sequence[Sequence[float]]
    max_step_m: float
    f_abort_n: float
    socket_top_z: float
    max_press_depth_m: float

    @property
    def aabb(self) -> np.ndarray:
        a = np.asarray(self.workspace_aabb, float).reshape(3, 2).copy()
        a[2, 0] = max(float(a[2, 0]),
                      float(self.socket_top_z) - float(self.max_press_depth_m))
        return a

    @property
    def max_step(self) -> float:
        return float(self.max_step_m)

    @property
    def caps6(self) -> Tuple[float, ...]:
        f = float(self.f_abort_n)
        return (f, f, f) + _MOMENT_CAPS

    def anchor(self, data: SafetyInput) -> Sequence[float]:
        """Step clamp is anchored on the previous command (ratcheting)."""
        return data.prev_cmd_xyz


class SafetyEnvelopeModule:
    """Composes the :mod:`robo67_insertion.lib.safety` primitives for a profile.

    The clamp ordering is standardized to **workspace then step**: the raw
    desired setpoint is first clipped into the profile's (possibly z-folded)
    workspace AABB, then the per-cycle step is bounded relative to the profile's
    anchor. Force abort is evaluated against the profile's caps independently.
    """

    def __init__(self, profile):
        self.profile = profile

    def apply(self, data: SafetyInput) -> SafetyOutput:
        # workspace first, then step anchored per the profile (standardized).
        safe = safety.clamp_to_workspace(data.desired_xyz, self.profile.aabb)
        safe = safety.clamp_step(self.profile.anchor(data), safe,
                                 self.profile.max_step)
        abort = safety.force_exceeded(data.wrench6, self.profile.caps6)
        return SafetyOutput(
            safe_xyz=(float(safe[0]), float(safe[1]), float(safe[2])),
            abort=abort,
        )
