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

from robo67_insertion.lib.command_path_adapters import MMCCommandPathAdapter
from robo67_insertion.lib.insertion_intent import (
    PHASES,
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


# Canonical set of FSM states -- the SINGLE source is the intent seam (ADR-0001).
STATES = PHASES


class InsertionFSM:
    """Pure peg-in-hole insertion FSM (MMC command shaping over the canonical seam).

    Args:
        socket_xyz: Socket TOP center in the robot base frame, shape (3,).
        params: Tunable :class:`FsmParams`.
    """

    def __init__(self, socket_xyz, params: FsmParams = FsmParams()):
        self.p = params
        # The MMC command shaping (descend/push ramp + held orientation) lives
        # ONLY in MMCCommandPathAdapter now; the shim delegates to it. We expose
        # the adapter's intent module as ``self.module`` so the @property proxies
        # below (contact_z/spiral_t0/retries/error) operate on the SAME module.
        self.adapter = MMCCommandPathAdapter(
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
            down_quat=params.down_quat,
            descend_step_m=params.descend_step_m,
            push_step_m=params.push_step_m,
        )
        self.module = self.adapter.module

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

        Both the transitions AND the MMC command shaping now live in the
        adapter; this shim only translates the legacy ``Sensors`` into
        :class:`IntentSensors` and wraps the returned ``MMCCommand`` back into a
        :class:`Decision`.
        """
        ee = np.asarray(s.ee_xyz, float).reshape(3)
        cmd = self.adapter.step(
            state,
            IntentSensors(
                ee_xyz=(float(ee[0]), float(ee[1]), float(ee[2])),
                fz=s.fz,
                fz_baseline=s.fz_baseline,
                t=s.t,
            ),
        )
        return self._decision(
            cmd.desired_xyz, cmd.next_state, done=cmd.done, error=cmd.error
        )
