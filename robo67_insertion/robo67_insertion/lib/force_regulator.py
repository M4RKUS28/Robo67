"""Axial force regulator (admittance) for force-guided peg-in-hole (pure).

Converts a target press FORCE into a commanded equilibrium HEIGHT for the soft
Cartesian impedance controller (``Fz ~ pos_stiff * (z_ee - z_cmd)``). The
equilibrium ratchets from the previous command at an admittance rate so the arm
holds a constant gentle press AND chases the peg down when resistance slackens.

The law is BIDIRECTIONAL: ``v = k_adm * (f_target - press)`` ratchets the
equilibrium DOWN when under-pressed (drive the peg in) and UP when over-pressed
(reduce the force again), so the commanded force self-limits around the target
and never runs away toward the firmware reflex. See
``docs/architecture/force-guided-insertion-2026-06-26.md`` and ADR-0002.

``press_n`` is the press MAGNITUDE ``|fz_meas - fz_baseline|`` (N); the search
only ever presses down, so magnitude is sign-robust against the unknown sign of
``o_f_ext_hat_k[2]``.

numpy + stdlib only. This module must NOT import rclpy/ROS/cv2/scipy.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

__all__ = ["AxialForceParams", "AxialForceRegulator"]


@dataclass(frozen=True)
class AxialForceParams:
    """Tunable parameters for :class:`AxialForceRegulator` (SI units)."""

    pos_stiff: float                 # N/m, MUST match the running controller
    k_adm: float = 0.0008            # m/s per N (admittance gain)
    v_cap_mps: float = 0.01          # axial equilibrium speed cap (<= v_max)
    max_press_depth_m: float = 0.05  # z-floor = socket_top - this
    max_force_n: float = 12.0        # clamp on the force target


class AxialForceRegulator:
    """Force-target -> equilibrium-z, ratcheting from the previous command.

    Args:
        params: Tunable :class:`AxialForceParams`.
        socket_top_z: Socket TOP height in the base frame (m); the z-floor is
            ``socket_top_z - max_press_depth_m``.
    """

    def __init__(self, params: AxialForceParams, socket_top_z: float) -> None:
        self.params = params
        self.z_floor = float(socket_top_z) - float(params.max_press_depth_m)

    def _clamp_z(self, z: float, z_ee: Optional[float] = None) -> float:
        z = max(float(z), self.z_floor)            # never deeper than the floor
        if z_ee is not None:
            z = min(z, float(z_ee))                # never above the EE (press only)
        return z

    def seed(self, z_ee: float, press_n: float) -> float:
        """Initial equilibrium reproducing the measured press with NO jump.

        Used at the contact handoff so the equilibrium is not snapped (which
        would step the force and bounce the arm): the gap that produced the
        just-measured ``press_n`` is reproduced exactly.
        """
        return self._clamp_z(float(z_ee) - float(press_n) / self.params.pos_stiff, z_ee)

    def step(self, z_cmd_prev: float, z_ee: float, press_n: float,
             f_target_n: float, dt: float) -> float:
        """One admittance tick -> next commanded equilibrium height.

        ``err = f_target - press`` is positive when UNDER-pressed (descend to
        push the peg in) and negative when OVER-pressed (rise to bleed off
        force). The resulting equilibrium speed is capped at ``v_cap_mps`` in
        either direction and clamped to ``[z_floor, z_ee]``.
        """
        p = self.params
        f_target = min(float(f_target_n), p.max_force_n)
        err = f_target - float(press_n)            # >0 under-pressed, <0 over-pressed
        v = max(-p.v_cap_mps, min(p.v_cap_mps, p.k_adm * err))  # +v = descend
        z_next = float(z_cmd_prev) - v * float(dt)
        return self._clamp_z(z_next, z_ee)
