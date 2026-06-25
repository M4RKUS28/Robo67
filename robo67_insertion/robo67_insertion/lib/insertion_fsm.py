"""Peg-in-hole insertion state machine (pure, deterministic).

This module is PURE: given ``(current_state, Sensors)`` :meth:`InsertionFSM.step`
returns a :class:`Decision` (a desired Cartesian setpoint plus the next state).
The ROS node is expected to call :meth:`InsertionFSM.step` at ~50 Hz, command
the returned setpoint, and feed back the resulting sensor readings.

Conventions
-----------
* Z is UP in the robot base frame, so *descending* means *decreasing z*.
* "Contact" is when the peg bottom touches the socket top surface at height
  ``contact_z``. The peg drops INTO the hole when ``ee_z`` falls below
  ``contact_z`` (z decreases further).

Only numpy + stdlib are used here, plus two sibling pure modules
(:func:`~robo67_insertion.lib.spiral.archimedean_offset` and
:func:`~robo67_insertion.lib.wrench.contact_detected`). This module must NOT
import rclpy/ROS/cv2/scipy so it stays unit-testable on any host.
"""
from dataclasses import dataclass, field

import numpy as np

from robo67_insertion.lib.spiral import archimedean_offset
from robo67_insertion.lib.wrench import contact_detected

__all__ = ["FsmParams", "Sensors", "Decision", "InsertionFSM"]


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


# Canonical set of FSM states.
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
    """Pure peg-in-hole insertion state machine.

    Args:
        socket_xyz: Socket TOP center in the robot base frame, shape (3,).
        params: Tunable :class:`FsmParams`.
    """

    def __init__(self, socket_xyz, params: FsmParams = FsmParams()):
        self.socket_xyz = np.asarray(socket_xyz, float)  # socket TOP center
        self.p = params
        self.contact_z = None
        self.spiral_t0 = None
        self.retries = 0
        self.error = None

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
        """Advance the FSM one tick and return the commanded :class:`Decision`."""
        p = self.p
        ee = np.asarray(s.ee_xyz, float).reshape(3)
        sx, sy, sz = self.socket_xyz

        if state == "IDLE":
            return self._decision(
                (sx, sy, sz + p.standoff_m), "MOVE_ABOVE"
            )

        if state == "MOVE_ABOVE":
            target = self.socket_xyz + np.array([0.0, 0.0, p.standoff_m])
            if np.linalg.norm(ee - target) <= p.approach_tol_m:
                nxt = "DESCEND_TO_CONTACT"
            else:
                nxt = "MOVE_ABOVE"
            return self._decision(target, nxt)

        if state == "DESCEND_TO_CONTACT":
            if contact_detected(s.fz, s.fz_baseline, p.contact_fz_threshold_n):
                self.contact_z = float(ee[2])
                self.spiral_t0 = float(s.t)
                self.retries = 0
                # hold position on the contact step
                return self._decision(ee, "SEARCH_SPIRAL")
            # step straight down, keeping xy aligned with the socket
            return self._decision(
                (sx, sy, ee[2] - p.descend_step_m), "DESCEND_TO_CONTACT"
            )

        if state == "SEARCH_SPIRAL":
            elapsed = s.t - self.spiral_t0
            dx, dy = archimedean_offset(
                elapsed, p.spiral_pitch_m, p.spiral_speed_mps
            )
            r = (dx * dx + dy * dy) ** 0.5
            if r > p.spiral_max_radius_m:
                # spiral grew past its bound: restart from the center
                self.retries += 1
                self.spiral_t0 = float(s.t)
                if self.retries > p.retry_limit:
                    self.error = "spiral search exhausted"
                    return self._decision(
                        ee, "ERROR", done=True, error=self.error
                    )
                dx = 0.0
                dy = 0.0
            desired = (sx + dx, sy + dy, self.contact_z - p.push_step_m)
            if ee[2] < self.contact_z - p.z_drop_threshold_m:
                nxt = "PUSH_INSERT"  # peg dropped into the hole
            else:
                nxt = "SEARCH_SPIRAL"
            return self._decision(desired, nxt)

        if state == "PUSH_INSERT":
            target_z = self.contact_z - p.insert_depth_m
            desired_z = max(target_z, ee[2] - p.push_step_m)
            if ee[2] <= target_z + 0.5 * p.z_drop_threshold_m:
                nxt = "CONFIRM"
            else:
                nxt = "PUSH_INSERT"
            return self._decision((sx, sy, desired_z), nxt)

        if state == "CONFIRM":
            depth_ok = ee[2] <= self.contact_z - 0.8 * p.insert_depth_m
            if depth_ok:
                return self._decision(ee, "RETRACT")
            self.retries += 1
            self.spiral_t0 = float(s.t)
            if self.retries > p.retry_limit:
                self.error = "insertion not confirmed"
                return self._decision(
                    ee, "ERROR", done=True, error=self.error
                )
            return self._decision(ee, "SEARCH_SPIRAL")

        if state == "RETRACT":
            target = self.socket_xyz + np.array([0.0, 0.0, p.standoff_m])
            if ee[2] >= self.contact_z:
                nxt = "DONE"
            else:
                nxt = "RETRACT"
            return self._decision(target, nxt)

        if state == "DONE":
            return self._decision(ee, "DONE", done=True)

        if state == "ERROR":
            return self._decision(ee, "ERROR", done=True, error=self.error)

        # Unknown state -> fail safe into ERROR (holds position).
        self.error = "unknown state: %r" % (state,)
        return self._decision(ee, "ERROR", done=True, error=self.error)
