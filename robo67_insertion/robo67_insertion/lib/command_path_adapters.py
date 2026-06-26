"""Command-path adapters for the canonical insertion intent (pure).

Both adapters wrap the SAME :class:`~robo67_insertion.lib.insertion_intent.InsertionIntentModule`
and delegate every phase transition to it (ADR-0001). They contain NO
transition logic; they only translate the canonical, controller-agnostic
:class:`~robo67_insertion.lib.insertion_intent.InsertionIntent` into a specific
controller's command representation.

* :class:`MMCCommandPathAdapter` -- sim path. The MMC
  ``panda_cartesian_impedance_controller`` DISCARDS any desired pose > 0.1 m
  from the current pose, so the node lead-clamps every setpoint to a small lead
  ahead of the measured EE. The adapter therefore mostly passes the canonical
  target through (the node makes the carrot), holds a fixed orientation
  quaternion, and applies only a light downward press bias while searching.
* :class:`ImpedanceCommandPathAdapter` -- real-arm path. Pure Cartesian
  impedance with NO rejection window; contact force is produced by commanding
  an equilibrium BELOW the surface (force F needs a gap F/pos_stiff). The
  adapter maps canonical targets to below-surface equilibrium gaps, holds a
  ROW-MAJOR 3x3 R, and keeps px / R22 non-zero (controller quirk).

numpy + stdlib only. Must NOT import rclpy/ROS/cv2/scipy.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from robo67_insertion.lib.insertion_intent import (
    InsertionIntentModule,
    IntentParams,
    IntentSensors,
)

__all__ = [
    "MMCCommand",
    "ImpedanceCommand",
    "MMCCommandPathAdapter",
    "ImpedanceCommandPathAdapter",
    "pose_desired_data",
]

# Default tool-down orientation as a ROW-MAJOR 3x3 (R22 non-zero by construction).
_DEFAULT_R = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
_EPS = 1e-6


def pose_desired_data(xyz, R) -> List[float]:
    """Build the impedance controller's ``[px,py,pz, R00..R22]`` (row-major).

    px and R22 are forced non-zero: the real ``franka_controllers`` Cartesian
    impedance controller IGNORES updates whose px or R22 is zero.
    """
    xyz = np.asarray(xyz, float).reshape(3)
    R = np.asarray(R, float).reshape(3, 3)
    px = float(xyz[0]) if abs(float(xyz[0])) > _EPS else _EPS
    flat = [float(R[i][j]) for i in range(3) for j in range(3)]
    if abs(flat[8]) <= _EPS:  # R22
        flat[8] = -_EPS
    return [px, float(xyz[1]), float(xyz[2])] + flat


@dataclass(frozen=True)
class MMCCommand:
    """MMC (sim) Cartesian impedance command produced by the adapter."""

    desired_xyz: Tuple[float, float, float]
    desired_quat: Tuple[float, float, float, float]
    next_phase: str
    done: bool
    error: Optional[str]

    @property
    def next_state(self) -> str:
        """Alias of :attr:`next_phase` for node/FSM compatibility."""
        return self.next_phase


@dataclass(frozen=True)
class ImpedanceCommand:
    """Real-arm Cartesian impedance command produced by the adapter."""

    goal_xyz: Tuple[float, float, float]
    R: np.ndarray  # held ROW-MAJOR 3x3 orientation
    next_phase: str
    done: bool
    error: Optional[str]

    @property
    def next_state(self) -> str:
        """Alias of :attr:`next_phase` for node compatibility."""
        return self.next_phase

    def pose_desired(self) -> List[float]:
        """``[px,py,pz, R00..R22]`` with px / R22 forced non-zero."""
        return pose_desired_data(self.goal_xyz, self.R)


class MMCCommandPathAdapter:
    """Adapt canonical intent to the MMC (sim) command path.

    This is the SINGLE source of MMC command shaping: it turns each absolute
    canonical target into a per-tick carrot that ramps DOWN a few millimetres
    from the MEASURED EE while descending and pressing. The MMC controller
    discards setpoints far from the current pose, and the node's safety step
    clamp only engages past ``max_lead_m`` -- so the ramp (not the clamp) is
    what keeps DESCEND/PUSH gentle. :class:`~robo67_insertion.lib.insertion_fsm.InsertionFSM`
    delegates to this adapter so the shim and the orchestrator share one shaping.

    Args:
        socket_xyz: Socket TOP center in the base frame, shape (3,).
        params: Canonical :class:`IntentParams`.
        down_quat: Fixed tool-down orientation quaternion (xyzw) to hold.
        descend_step_m: Per-tick downward decrement of the carrot from the
            measured EE while descending toward contact.
        push_step_m: Light downward press bias used while searching/seating so
            the carrot keeps gentle contact instead of jumping to the deep
            equilibrium in one tick.
    """

    def __init__(self, socket_xyz, params: IntentParams = IntentParams(),
                 down_quat=(1.0, 0.0, 0.0, 0.0), descend_step_m: float = 0.0015,
                 push_step_m: float = 0.0015):
        self.module = InsertionIntentModule(socket_xyz, params)
        self.down_quat = tuple(float(v) for v in down_quat)
        self.descend_step_m = float(descend_step_m)
        self.push_step_m = float(push_step_m)
        self.insert_depth_m = float(self.module.params.insert_depth_m)

    def step(self, phase: str, s: IntentSensors) -> MMCCommand:
        intent = self.module.step(phase, s)
        ee = np.asarray(s.ee_xyz, float).reshape(3)
        tx, ty, tz = intent.target_xyz
        nxt = intent.phase
        cz = self.module.contact_z
        sx, sy, _sz = self.module.socket

        if phase in ("IDLE", "MOVE_ABOVE", "RETRACT"):
            # pass the absolute canonical target through; the node lead-clamps
            # it to a small lead ahead of the measured EE.
            desired = (tx, ty, tz)
        elif phase == "DESCEND_TO_CONTACT":
            if nxt == "SEARCH_SPIRAL":
                desired = (ee[0], ee[1], ee[2])  # contact recorded -> hold here
            else:
                # step straight down one descend_step from the MEASURED EE
                desired = (tx, ty, ee[2] - self.descend_step_m)
        elif phase == "SEARCH_SPIRAL":
            if nxt == "ERROR":
                desired = (ee[0], ee[1], ee[2])
            else:
                # gentle press a single push_step below the contact plane
                desired = (tx, ty, cz - self.push_step_m)
        elif phase == "PUSH_INSERT":
            target_z = cz - self.insert_depth_m
            # ramp toward the deep equilibrium one push_step at a time
            desired = (sx, sy, max(target_z, ee[2] - self.push_step_m))
        else:
            # CONFIRM / DONE / ERROR / unknown -> hold position
            desired = (ee[0], ee[1], ee[2])

        return MMCCommand(
            desired_xyz=(float(desired[0]), float(desired[1]), float(desired[2])),
            desired_quat=self.down_quat,
            next_phase=intent.phase,
            done=intent.done,
            error=intent.error,
        )


class ImpedanceCommandPathAdapter:
    """Adapt canonical intent to the real-arm Cartesian impedance command path.

    Args:
        socket_xyz: Socket TOP center in the base frame, shape (3,).
        params: Canonical :class:`IntentParams`.
        pos_stiff: Controller translational stiffness (N/m); MUST match the
            running controller so the equilibrium-gap force math is correct.
        press_force_n: Gentle press force target while searching.
        insert_press_n: Press force target while seating.
        max_press_depth_m: Bound on how far below the socket top to command an
            equilibrium while descending.
        R: Held ROW-MAJOR 3x3 orientation (defaults to tool-down).
    """

    def __init__(self, socket_xyz, params: IntentParams = IntentParams(),
                 pos_stiff: float = 200.0, press_force_n: float = 3.0,
                 insert_press_n: float = 6.0, max_press_depth_m: float = 0.05,
                 R=None, force_mode: bool = False):
        self.module = InsertionIntentModule(socket_xyz, params)
        self.pos_stiff = float(pos_stiff)
        self.press_force_n = float(press_force_n)
        self.insert_press_n = float(insert_press_n)
        self.max_press_depth_m = float(max_press_depth_m)
        self.R = np.array(_DEFAULT_R if R is None else R, dtype=float).reshape(3, 3)
        # When True, SEARCH_SPIRAL/PUSH_INSERT emit the bare contact plane as z
        # (no fixed gap); the node's AxialForceRegulator owns the axial z so a
        # constant press is REGULATED (and reduced if it overshoots) instead of
        # a fixed equilibrium that lets the force decay. ADR-0002.
        self.force_mode = bool(force_mode)

    @property
    def press_gap_m(self) -> float:
        return self.press_force_n / max(1.0, self.pos_stiff)

    @property
    def insert_gap_m(self) -> float:
        return self.insert_press_n / max(1.0, self.pos_stiff)

    def step(self, phase: str, s: IntentSensors) -> ImpedanceCommand:
        intent = self.module.step(phase, s)
        tx, ty, tz = intent.target_xyz
        cz = self.module.contact_z
        sx, sy, sz = self.module.socket

        if phase == "DESCEND_TO_CONTACT":
            if intent.phase == "SEARCH_SPIRAL" and cz is not None:
                # contact just detected -> first gentle press at the hole search
                goal = (tx, ty, cz - self.press_gap_m)
            else:
                # press an equilibrium below the surface to build contact force
                goal = (sx, sy, sz - self.max_press_depth_m)
        elif phase == "SEARCH_SPIRAL" and cz is not None:
            goal = (tx, ty, cz if self.force_mode else cz - self.press_gap_m)
        elif phase == "PUSH_INSERT" and cz is not None:
            goal = (tx, ty, cz if self.force_mode else cz - self.insert_gap_m)
        else:
            # MOVE_ABOVE / CONFIRM / RETRACT / DONE / ERROR pass through; these
            # canonical targets are already above-surface or hold positions.
            goal = (tx, ty, tz)

        return ImpedanceCommand(
            goal_xyz=(float(goal[0]), float(goal[1]), float(goal[2])),
            R=self.R,
            next_phase=intent.phase,
            done=intent.done,
            error=intent.error,
        )

    def pose_desired(self, xyz) -> List[float]:
        """``[px,py,pz, R00..R22]`` for an arbitrary (already-clamped) xyz."""
        return pose_desired_data(xyz, self.R)
