"""Relaunch the Franka arm bringup + gripper from the dashboard.

In ``live`` mode the dashboard server runs INSIDE ``multipanda-container`` with
ROS sourced (domain 1, ``LD_LIBRARY_PATH`` including libfranka) -- see
``run_live.sh`` -- so it can spawn ``ros2 launch`` as subprocesses that inherit
that environment and command the real arm. This controller owns the bringup
relaunch sequence (the "Relaunch arm" button), the standard recovery for a
crashed / Idle / Reflex bringup documented in
``docs/runbooks/automated-insertion.md`` §5:

    1. KILL  any running bringup + gripper (broad ``pkill`` -> our tracked PIDs
       too), escalating SIGTERM -> SIGKILL.
    2. LAUNCH ``franka_bringup franka.launch.py`` (auto-activates the cartesian
       impedance controller). Tracked, own process group, stdout ring-buffered.
    3. WAIT for ``FrankaState`` to come back live (the dashboard's passive
       observer subscription auto-rediscovers the fresh publisher).
    4. RECOVER: if ``robot_mode != 2`` (Move) -- e.g. ``4`` (Reflex) -- call
       ``/panda_error_recovery_service_server/error_recovery`` and re-read.
    5. LAUNCH ``franka_gripper gripper.launch.py`` SEPARATELY (NOT
       ``load_gripper:=true``, which would shift the EE frame). Tracked.
    6. VERIFY ``robot_mode == 2`` (Move) and the ``/panda_gripper/move`` action
       server is present.

It deliberately does NOT touch the logging/camera graph or the dashboard
itself (scope = bringup + gripper only). The launched processes use
``start_new_session`` so they survive a dashboard restart, exactly like the
manual terminal launch they replace.
"""
from __future__ import annotations

import collections
import os
import signal
import subprocess
import threading
import time
from typing import Dict, List, Optional

# Runbook-verified bringup parameters (env-overridable). robot_ip/arm_id match
# docs/runbooks/automated-insertion.md §2 and the main CLAUDE.md runbook.
ROBOT_IP = os.environ.get("ROBO67_ROBOT_IP", "192.168.1.67")
ARM_ID = os.environ.get("ROBO67_ARM_ID", "panda")
GRIPPER_NS = os.environ.get("ROBO67_GRIPPER_NS", "/panda_gripper")

ERROR_RECOVERY_SRV = f"/{ARM_ID}_error_recovery_service_server/error_recovery"
GRIPPER_MOVE_ACTION = f"{GRIPPER_NS}/move"

BRINGUP_CMD: List[str] = [
    "ros2", "launch", "franka_bringup", "franka.launch.py",
    f"robot_ip:={ROBOT_IP}", "use_fake_hardware:=false", f"arm_id:={ARM_ID}",
]
GRIPPER_CMD: List[str] = [
    "ros2", "launch", "franka_gripper", "gripper.launch.py",
    f"robot_ip:={ROBOT_IP}", f"arm_id:={ARM_ID}", "use_fake_hardware:=false",
]

# Patterns matched by pkill -f to tear down a previous bringup / gripper. Fed
# over STDIN (never argv) so pkill can't match its own cleanup shell -- the
# container shares the host PID namespace, so a pattern in argv would let
# `pkill -f` kill the very shell running it (see run_live.sh for the same trap).
_KILL_PATTERN = (
    r"franka\.launch\.py|franka_control2_node|ros2_control_node|"
    r"controller_manager|gripper\.launch\.py|franka_gripper"
)

ROBOT_MODE_LABEL = {
    0: "Other", 1: "Idle", 2: "Move", 3: "Guiding",
    4: "Reflex", 5: "User stopped", 6: "Automatic recovery",
}

_PHASE_LABEL = {
    "idle": "Idle",
    "killing": "Stopping bringup",
    "launching_bringup": "Launching bringup",
    "waiting_state": "Waiting for robot state",
    "recovering": "Clearing reflex",
    "launching_gripper": "Launching gripper",
    "verifying": "Verifying",
    "done": "Ready",
    "error": "Error",
}

# Timings (s)
_STATE_TIMEOUT = 35.0     # bringup connect + controller spawn + FrankaState back
_RECOVER_SETTLE = 2.0
_GRIPPER_VERIFY_TIMEOUT = 15.0


class BringupController:
    """Owns the arm-bringup relaunch sequence (one at a time)."""

    def __init__(self, provider, enabled: bool = True, log_lines: int = 400) -> None:
        self.enabled = enabled              # live mode only
        self._provider = provider           # LiveProvider (robot_mode/state freshness)
        self._lock = threading.Lock()
        self._busy = False
        self._phase = "idle"
        self._started_at: Optional[float] = None
        self._ok: Optional[bool] = None
        self._error: Optional[str] = None
        self._mode_ok = False
        self._gripper_ok = False
        self._log: "collections.deque[str]" = collections.deque(maxlen=log_lines)
        # tracked long-running launches
        self._bringup: Optional[subprocess.Popen] = None
        self._gripper: Optional[subprocess.Popen] = None

    # -- helpers --------------------------------------------------------

    def _emit(self, line: str) -> None:
        self._log.append(f"[{time.strftime('%H:%M:%S')}] {line}")

    @staticmethod
    def _running(proc: Optional[subprocess.Popen]) -> bool:
        return proc is not None and proc.poll() is None

    def _drain(self, proc: subprocess.Popen, tag: str) -> None:
        try:
            assert proc.stdout is not None
            for line in proc.stdout:
                self._log.append(f"  {tag}| {line.rstrip()}")
        except Exception:  # noqa: BLE001
            pass

    def _robot_mode(self) -> Optional[int]:
        try:
            return int(self._provider.robot_mode())
        except Exception:  # noqa: BLE001
            return None

    # -- teardown / launch primitives -----------------------------------

    def _kill_existing(self) -> None:
        """SIGTERM (->SIGKILL) every bringup/gripper process, ours and stray."""
        # our own tracked launches first (clean process-group signal)
        for proc in (self._bringup, self._gripper):
            if self._running(proc):
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)  # type: ignore[union-attr]
                except Exception:  # noqa: BLE001
                    pass
        self._bringup = None
        self._gripper = None
        # broad sweep for any externally-launched bringup (pattern via STDIN so
        # pkill never matches its own shell -- shared host PID namespace).
        script = (
            f"pat='{_KILL_PATTERN}'\n"
            "pkill -TERM -f \"$pat\" || true\n"
            "for _ in $(seq 1 15); do pgrep -f \"$pat\" >/dev/null 2>&1 || exit 0; sleep 0.2; done\n"
            "pkill -KILL -f \"$pat\" || true\n"
        )
        try:
            subprocess.run(["bash", "-s"], input=script, text=True,
                           timeout=20, stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL)
        except Exception as exc:  # noqa: BLE001
            self._emit(f"kill sweep warning: {exc}")

    def _launch(self, cmd: List[str], tag: str) -> subprocess.Popen:
        proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1, start_new_session=True)
        threading.Thread(target=self._drain, args=(proc, tag), daemon=True).start()
        return proc

    def _wait_state_live(self, since: float, timeout: float) -> bool:
        """Block until a FrankaState newer than ``since`` arrives (or timeout)."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                if self._provider.last_state_wall() > since:
                    return True
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.25)
        return False

    def _error_recovery(self) -> None:
        try:
            subprocess.run(
                ["ros2", "service", "call", ERROR_RECOVERY_SRV,
                 "franka_msgs/srv/ErrorRecovery", "{}"],
                timeout=20, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as exc:  # noqa: BLE001
            self._emit(f"error_recovery call warning: {exc}")

    def _gripper_action_present(self) -> bool:
        try:
            out = subprocess.run(["ros2", "action", "list"], capture_output=True,
                                  text=True, timeout=10)
            return GRIPPER_MOVE_ACTION in (out.stdout or "")
        except Exception:  # noqa: BLE001
            return False

    # -- the relaunch sequence (runs in a background thread) ------------

    def _run(self) -> None:
        try:
            # 1. stop any running bringup/gripper
            with self._lock:
                self._phase = "killing"
            self._emit("stopping any running bringup + gripper ...")
            self._kill_existing()
            mark = time.time()

            # 2. launch bringup (auto-activates the cartesian impedance controller)
            with self._lock:
                self._phase = "launching_bringup"
            self._emit("launching " + " ".join(BRINGUP_CMD))
            self._bringup = self._launch(BRINGUP_CMD, "bringup")

            # 3. wait for FrankaState to come back live
            with self._lock:
                self._phase = "waiting_state"
            if not self._wait_state_live(mark, _STATE_TIMEOUT):
                raise RuntimeError(
                    f"no FrankaState within {_STATE_TIMEOUT:.0f}s "
                    "(bringup failed to connect / activate)")
            self._emit(f"robot state live (mode={self._robot_mode()})")

            # 4. clear reflex if not already in Move (2)
            mode = self._robot_mode()
            if mode != 2:
                with self._lock:
                    self._phase = "recovering"
                self._emit(f"mode={mode} ({ROBOT_MODE_LABEL.get(mode, '?')}) "
                           "!= Move -> error_recovery")
                self._error_recovery()
                time.sleep(_RECOVER_SETTLE)
                mode = self._robot_mode()
                self._emit(f"after recovery: mode={mode}")

            # 5. launch the gripper node (separate -- not load_gripper:=true)
            with self._lock:
                self._phase = "launching_gripper"
            self._emit("launching " + " ".join(GRIPPER_CMD))
            self._gripper = self._launch(GRIPPER_CMD, "gripper")

            # 6. verify mode==2 + /panda_gripper/move present
            with self._lock:
                self._phase = "verifying"
            deadline = time.time() + _GRIPPER_VERIFY_TIMEOUT
            gripper_ok = False
            while time.time() < deadline:
                if self._gripper_action_present():
                    gripper_ok = True
                    break
                time.sleep(0.5)
            mode = self._robot_mode()
            mode_ok = mode == 2

            with self._lock:
                self._mode_ok = mode_ok
                self._gripper_ok = gripper_ok
            self._emit(f"verify: robot_mode={mode} (Move? {mode_ok}) "
                       f"{GRIPPER_MOVE_ACTION} present? {gripper_ok}")

            ok = mode_ok and gripper_ok
            with self._lock:
                self._ok = ok
                self._error = None if ok else (
                    f"verification failed (mode={mode}, gripper={gripper_ok})")
                self._phase = "done" if ok else "error"
            self._emit("RELAUNCH COMPLETE -- arm ready" if ok
                       else "RELAUNCH finished with verification warnings")
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self._ok = False
                self._error = str(exc)
                self._phase = "error"
            self._emit(f"RELAUNCH FAILED: {exc}")
        finally:
            with self._lock:
                self._busy = False

    # -- public API -----------------------------------------------------

    def relaunch(self) -> Dict:
        if not self.enabled:
            return {"ok": False, "error": "bringup control is live-mode only"}
        with self._lock:
            if self._busy:
                return {"ok": False, "error": "relaunch already in progress"}
            self._busy = True
            self._phase = "killing"
            self._started_at = time.time()
            self._ok = None
            self._error = None
            self._mode_ok = False
            self._gripper_ok = False
            self._log.clear()
        threading.Thread(target=self._run, daemon=True).start()
        return {"ok": True, "started": True}

    def status(self) -> Dict:
        with self._lock:
            busy = self._busy
            phase = self._phase
            started = self._started_at
            mode = self._robot_mode()
            return {
                "enabled": self.enabled,
                "busy": busy,
                "phase": phase,
                "phase_label": _PHASE_LABEL.get(phase, phase),
                "bringup_running": self._running(self._bringup),
                "gripper_running": self._running(self._gripper),
                "robot_mode": mode,
                "robot_mode_label": ROBOT_MODE_LABEL.get(mode, "?") if mode is not None else "?",
                "mode_ok": self._mode_ok,
                "gripper_ok": self._gripper_ok,
                "ok": self._ok,
                "error": self._error,
                "elapsed_s": (round(time.time() - started, 1)
                              if (busy and started is not None) else None),
                "log": list(self._log),
            }
