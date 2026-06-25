"""Insertion telemetry aggregation (pure).

The real-arm orchestrator (:mod:`nodes.hardware_insertion_node`) computes a rich
internal state every control tick -- phase, EE pose, commanded equilibrium,
external wrench, the Fz baseline, contact, retries -- but historically published
only the controller setpoint. This module turns that internal state into a
controller-neutral telemetry snapshot so the node stays a thin publisher and the
derivations (EE speed, the diagnostic key/value rollup) are unit-testable.

numpy + stdlib only -- like the other seams it must NOT import rclpy/ROS/cv2.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

__all__ = ["SpeedTracker", "InsertionTelemetry", "diagnostic_pairs"]


class SpeedTracker:
    """Finite-difference linear speed (m/s) from successive position samples.

    Mirrors the dashboard's derivation: speed is ``|x_k - x_{k-1}| / dt``. The
    first sample and non-advancing time stamps return the previous speed (0.0
    initially) rather than dividing by ~zero.
    """

    def __init__(self) -> None:
        self._prev_xyz: Optional[np.ndarray] = None
        self._prev_t: Optional[float] = None
        self.speed: float = 0.0

    def update(self, xyz: Sequence[float], t: float) -> float:
        cur = np.asarray(xyz, float).reshape(3)
        if self._prev_xyz is not None and self._prev_t is not None:
            dt = float(t) - self._prev_t
            if dt > 1e-6:
                self.speed = float(np.linalg.norm(cur - self._prev_xyz) / dt)
        self._prev_xyz = cur
        self._prev_t = float(t)
        return self.speed

    def reset(self) -> None:
        self._prev_xyz = None
        self._prev_t = None
        self.speed = 0.0


@dataclass
class InsertionTelemetry:
    """A single controller-neutral telemetry snapshot of the insertion loop."""

    t: float
    phase: str
    ee_xyz: Tuple[float, float, float]
    cmd_xyz: Tuple[float, float, float]
    speed: float
    wrench6: Tuple[float, float, float, float, float, float]
    fz: float
    fz_baseline: float
    contact: bool
    retries: int
    socket_xyz: Optional[Tuple[float, float, float]] = None
    contact_z: Optional[float] = None
    abort: bool = False
    done: bool = False
    error: Optional[str] = None

    @property
    def force_mag(self) -> float:
        """Euclidean magnitude of the translational wrench (N)."""
        fx, fy, fz = self.wrench6[:3]
        return float(np.sqrt(fx * fx + fy * fy + fz * fz))


def _xyz(label: str, v: Optional[Sequence[float]]) -> Tuple[str, str]:
    if v is None:
        return (label, "n/a")
    return (label, f"{v[0]:+.4f} {v[1]:+.4f} {v[2]:+.4f}")


def diagnostic_pairs(tel: InsertionTelemetry) -> List[Tuple[str, str]]:
    """Flatten a telemetry snapshot into ordered (key, value) string pairs.

    Used to fill a ``diagnostic_msgs/DiagnosticStatus.values`` array so the
    whole insertion state is visible in one ``ros2 topic echo`` / rqt monitor,
    and replays cleanly from a ``ros2 bag``.
    """
    pairs: List[Tuple[str, str]] = [
        ("phase", tel.phase),
        ("t_s", f"{tel.t:.3f}"),
    ]
    pairs.append(_xyz("ee_xyz_m", tel.ee_xyz))
    pairs.append(_xyz("cmd_xyz_m", tel.cmd_xyz))
    pairs.append(_xyz("socket_xyz_m", tel.socket_xyz))
    pairs += [
        ("speed_mps", f"{tel.speed:.4f}"),
        ("fz_n", f"{tel.fz:+.3f}"),
        ("fz_baseline_n", f"{tel.fz_baseline:+.3f}"),
        ("force_mag_n", f"{tel.force_mag:.3f}"),
        ("contact", "true" if tel.contact else "false"),
        ("retries", str(int(tel.retries))),
        ("contact_z_m", "n/a" if tel.contact_z is None else f"{tel.contact_z:+.4f}"),
        ("abort", "true" if tel.abort else "false"),
        ("done", "true" if tel.done else "false"),
        ("error", tel.error or ""),
    ]
    return pairs
