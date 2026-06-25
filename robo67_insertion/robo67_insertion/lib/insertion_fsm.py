"""Peg-in-hole insertion state machine (thin shim over the canonical seam).

Historically this module owned its own copy of the insertion transition graph.
The transition logic now lives in EXACTLY ONE place --
:class:`~robo67_insertion.lib.insertion_intent.InsertionIntentModule` (ADR-0001)
-- and :class:`InsertionFSM` is a thin shim that delegates every transition to
it. The shim preserves the legacy public API
(``InsertionFSM(socket_xyz, params).step(state, Sensors) -> Decision``) and the
MMC-flavored command shaping (per-tick descend/push carrot steps + held
orientation) so existing callers and tests keep working as a parity harness.

Conventions
-----------
* Z is UP in the robot base frame, so *descending* means *decreasing z*.
* "Contact" is when the peg bottom touches the socket top surface at height
  ``contact_z``. The peg drops INTO the hole when ``ee_z`` falls below
  ``contact_z`` (z decreases further).

This module imports only numpy + the canonical seam. It must NOT import
rclpy/ROS/cv2/scipy so it stays unit-testable on any host.
"""
from dataclasses import dataclass

import numpy as np

from robo67_insertion.lib.insertion_intent import (
    InsertionIntentModule,
    IntentParams,
    IntentSensors,
)

__all__ = ["FsmParams", "Sensors", "Decision", "InsertionFSM", "STATES"]


@dataclass
class FsmParams:
    """Tunable parameters for the insertion state machine (SI units)."""

    standoff_m: float = 0.05
    approach_tol_m: float = 0.003
    contact_fz_threshold_n: float = 5.0
    insert_depth_m: float = 0.04
    z_drop_threshold_m: float = 0.004
    retry_limit: int = 3
    spiral_pitch_m: float = 0.002
    spiral_speed_mps: float = 0.005
    spiral_max_radius_m: float = 0.012
    descend_step_m: float = 0.0015  # per-step downward setpoint decrement
    push_step_m: float = 0.0015
    down_quat: tuple = (1.0, 0.0, 0.0, 0.0)  # fixed tool-down orientation


@dataclass
class Sensors:
    """Sensor snapshot consumed by :meth:`InsertionFSM.step`."""

    ee_xyz: np.ndarray  # shape (3,)
    fz: float
    fz_baseline: float
    t: float  # seconds


@dataclass
class Decision:
    """Setpoint + next-state command returned by :meth:`InsertionFSM.step`."""

    desired_xyz: np.ndarray  # shape (3,)
    desired_quat: tuple  # 4 floats, always the fixed tool-down orientation
    next_state: str
    done: bool = False
    error: str = None


# Canonical set of FSM states (mirrors the canonical seam).
STATES = (
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


class InsertionFSM:
    """Pure peg-in-hole insertion FSM (MMC command shaping over the canonical seam).

    Args:
        socket_xyz: Socket TOP center in the robot base frame, shape (3,).
        params: Tunable :class:`FsmParams`.
    """

    def __init__(self, socket_xyz, params: FsmParams = FsmParams()):
        self.p = params
        self.module = InsertionIntentModule(
            socket_xyz,
            IntentParams(
                standoff_m=params.standoff_m,
                approach_tol_m=params.approach_tol_m,
                contact_fz_threshold_n=params.contact_fz_threshold_n,
                insert_depth_m=params.insert_depth_m,
                z_drop_threshold_m=params.z_drop_threshold_m,
                retry_limit=params.retry_limit,
                spiral_pitch_m=params.spiral_pitch_m,
                spiral_speed_mps=params.spiral_speed_mps,
                spiral_max_radius_m=params.spiral_max_radius_m,
            ),
        )

    # -- bookkeeping mirrored onto the canonical module ------------------

    @property
    def socket_xyz(self):
        return self.module.socket

    @property
    def contact_z(self):
        return self.module.contact_z

    @contact_z.setter
    def contact_z(self, value):
        self.module.contact_z = value

    @property
    def spiral_t0(self):
        return self.module.spiral_t0

    @spiral_t0.setter
    def spiral_t0(self, value):
        self.module.spiral_t0 = value

    @property
    def retries(self):
        return self.module.retries

    @retries.setter
    def retries(self, value):
        self.module.retries = value

    @property
    def error(self):
        return self.module.error

    @error.setter
    def error(self, value):
        self.module.error = value

    # -- helpers ---------------------------------------------------------

    def _xyz(self, *vals):
        """Return a finite float64 array of shape (3,)."""
        return np.asarray(vals, dtype=float).reshape(3)

    def _decision(self, desired_xyz, next_state, done=False, error=None):
        return Decision(
            desired_xyz=self._xyz(*desired_xyz),
            desired_quat=self.p.down_quat,
            next_state=next_state,
            done=done,
            error=error,
        )

    # -- main entry point ------------------------------------------------

    def step(self, state: str, s: Sensors) -> Decision:
        """Advance the FSM one tick and return the commanded :class:`Decision`.

        Transitions + contact/drop/retry bookkeeping come from the canonical
        seam; only the MMC-flavored desired_xyz shaping lives here.
        """
        p = self.p
        ee = np.asarray(s.ee_xyz, float).reshape(3)
        sx, sy, sz = self.module.socket

        intent = self.module.step(
            state,
            IntentSensors(
                ee_xyz=(float(ee[0]), float(ee[1]), float(ee[2])),
                fz=s.fz,
                fz_baseline=s.fz_baseline,
                t=s.t,
            ),
        )
        nxt = intent.phase
        tx, ty, _tz = intent.target_xyz
        cz = self.module.contact_z

        if state in ("IDLE", "MOVE_ABOVE", "RETRACT"):
            desired = intent.target_xyz
        elif state == "DESCEND_TO_CONTACT":
            if nxt == "SEARCH_SPIRAL":
                desired = ee  # contact recorded -> hold on the contact step
            else:
                desired = (tx, ty, ee[2] - p.descend_step_m)  # step straight down
        elif state == "SEARCH_SPIRAL":
            if nxt == "ERROR":
                desired = ee
            else:
                desired = (tx, ty, cz - p.push_step_m)  # gentle press while searching
        elif state == "PUSH_INSERT":
            target_z = cz - p.insert_depth_m
            desired = (sx, sy, max(target_z, ee[2] - p.push_step_m))
        else:
            # CONFIRM / DONE / ERROR / unknown -> hold position
            desired = ee

        return self._decision(desired, nxt, done=intent.done, error=intent.error)
