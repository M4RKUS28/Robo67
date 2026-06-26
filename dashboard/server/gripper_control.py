"""Open / close the franka_gripper from the dashboard.

The header gets two buttons:

  * **Open**  -> ``franka_msgs/action/Move``  (width -> open, no grip force)
  * **Close** -> ``franka_msgs/action/Grasp`` (close with a grip force, so it
    actually clamps and holds a peg -- matches the peg-in-hole workflow)

Like ``bringup_control`` we drive ROS through the ``ros2`` CLI as a subprocess
(``ros2 action send_goal``) rather than importing rclpy here: it inherits the
live-mode container's ROS env, keeps this controller dependency-light, and the
action call blocks until the gripper finishes so we can report success.

``ros2 action send_goal`` waits forever for an absent server, so each call is
wrapped in a subprocess timeout -> a missing ``/panda_gripper`` is reported as
an error instead of hanging. **Live mode only** (needs the gripper node).
"""
from __future__ import annotations

import collections
import os
import subprocess
import threading
import time
from typing import Dict, List, Optional

GRIPPER_NS = os.environ.get("ROBO67_GRIPPER_NS", "/panda_gripper")
MOVE_ACTION = f"{GRIPPER_NS}/move"
GRASP_ACTION = f"{GRIPPER_NS}/grasp"

# Geometry / force (env-overridable). 0.08 m ~= the Panda hand's full width.
OPEN_WIDTH = float(os.environ.get("ROBO67_GRIPPER_OPEN_WIDTH", "0.08"))
GRIPPER_SPEED = float(os.environ.get("ROBO67_GRIPPER_SPEED", "0.1"))
GRASP_FORCE = float(os.environ.get("ROBO67_GRIPPER_GRASP_FORCE", "20.0"))
# Grasp to ~0 width with a wide tolerance so a full close always "succeeds";
# with a peg in the jaws it clamps at the peg width with GRASP_FORCE.
GRASP_WIDTH = float(os.environ.get("ROBO67_GRIPPER_GRASP_WIDTH", "0.0"))
GRASP_EPSILON = float(os.environ.get("ROBO67_GRIPPER_GRASP_EPS", "0.08"))

# A gripper Move/Grasp typically completes in 1-3 s; cap so an absent server
# (ros2 action send_goal would otherwise wait forever) is reported as an error.
ACTION_TIMEOUT = float(os.environ.get("ROBO67_GRIPPER_TIMEOUT", "15"))

_MOVE_GOAL = "{{width: {w}, speed: {s}}}".format(w=OPEN_WIDTH, s=GRIPPER_SPEED)
_GRASP_GOAL = (
    "{{width: {w}, speed: {s}, force: {f}, "
    "epsilon: {{inner: {e}, outer: {e}}}}}"
).format(w=GRASP_WIDTH, s=GRIPPER_SPEED, f=GRASP_FORCE, e=GRASP_EPSILON)


class GripperController:
    """Owns at most one gripper Move/Grasp action call at a time."""

    def __init__(self, enabled: bool = True, log_lines: int = 200) -> None:
        self.enabled = enabled              # live mode only
        self._lock = threading.Lock()
        self._busy = False
        self._last_action: Optional[str] = None   # "open" / "close"
        self._ok: Optional[bool] = None
        self._error: Optional[str] = None
        self._started_at: Optional[float] = None
        self._log: "collections.deque[str]" = collections.deque(maxlen=log_lines)

    def _emit(self, line: str) -> None:
        msg = f"[{time.strftime('%H:%M:%S')}] {line}"
        self._log.append(msg)
        print(f"[gripper] {msg}", flush=True)

    def _run(self, action: str, act_name: str, act_type: str, goal: str) -> None:
        cmd = ["ros2", "action", "send_goal", act_name, act_type, goal]
        ok = False
        err: Optional[str] = None
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True,
                                  timeout=ACTION_TIMEOUT)
            out = (proc.stdout or "") + (proc.stderr or "")
            for line in out.splitlines():
                self._emit(f"  | {line.rstrip()}")
            # ros2 action send_goal exits 0 + prints SUCCEEDED on a finished goal
            if proc.returncode == 0 and (
                    "SUCCEEDED" in out or "success: true" in out.lower()):
                ok = True
            else:
                err = ("goal rejected / failed"
                       if proc.returncode == 0 else
                       f"ros2 exited {proc.returncode}")
        except subprocess.TimeoutExpired:
            err = (f"{act_name} timed out after {ACTION_TIMEOUT:.0f}s "
                   "(gripper action server unavailable?)")
            self._emit(err)
        except Exception as exc:  # noqa: BLE001
            err = f"spawn failed: {exc}"
            self._emit(err)
        with self._lock:
            self._ok = ok
            self._error = None if ok else err
            self._busy = False
        self._emit(f"{action} {'OK' if ok else 'FAILED'}"
                   + ("" if ok else f": {err}"))

    def _start(self, action: str, act_name: str, act_type: str, goal: str) -> Dict:
        if not self.enabled:
            return {"ok": False, "error": "gripper control is live-mode only"}
        with self._lock:
            if self._busy:
                return {"ok": False, "error": "gripper already moving"}
            self._busy = True
            self._last_action = action
            self._ok = None
            self._error = None
            self._started_at = time.time()
        self._emit(f"{action.upper()} -> {act_name} {goal}")
        threading.Thread(target=self._run,
                         args=(action, act_name, act_type, goal),
                         daemon=True).start()
        return {"ok": True, "started": True}

    # -- public API -----------------------------------------------------

    def open(self) -> Dict:
        return self._start("open", MOVE_ACTION, "franka_msgs/action/Move", _MOVE_GOAL)

    def close(self) -> Dict:
        return self._start("close", GRASP_ACTION, "franka_msgs/action/Grasp", _GRASP_GOAL)

    def status(self) -> Dict:
        with self._lock:
            busy = self._busy
            started = self._started_at
            return {
                "enabled": self.enabled,
                "busy": busy,
                "last_action": self._last_action,
                "ok": self._ok,
                "error": self._error,
                "elapsed_s": (round(time.time() - started, 1)
                              if (busy and started is not None) else None),
                "log": list(self._log),
            }
