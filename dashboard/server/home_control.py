"""Bring the arm to the defined HOME pose from the dashboard.

"Home" is a FIXED starting pose (taught by hand-guiding the arm to a good
default and reading its EE there). The Home button MOVES the arm to that pose so
you can always restore the start position after working / jogging the arm.

The pose below was captured from the live arm (the operator's chosen default);
override it with ``ROBO67_HOME_XYZ="x y z"`` (metres, base frame) if you re-teach
it. The move is tool-down (vertical, yaw-preserving) -- the insertion-ready
orientation -- via ``scripts/hw_move_to.py`` (gentle time-parameterised ramp +
overshoot settle, workspace clamp, force/reflex aborts).

Like the insertion + bringup controllers, this owns at most one such subprocess
(own process group, ring-buffered stdout) and is **live-mode only**.
"""
from __future__ import annotations

import collections
import os
import signal
import subprocess
import threading
import time
from typing import Dict, List, Optional

from common import INSERTION_PKG, REPO_ROOT

MOVE_SCRIPT = os.path.join(INSERTION_PKG, "scripts", "hw_move_to.py")

# Hard-coded HOME pose (base-frame EE XYZ, metres), captured live from the arm's
# taught default position. Override with ROBO67_HOME_XYZ="x y z".
_DEFAULT_HOME_XYZ = (0.2145, -0.0278, 0.4451)


def _home_xyz() -> List[str]:
    env = os.environ.get("ROBO67_HOME_XYZ")
    if env:
        parts = env.split()
        if len(parts) == 3:
            return parts
    return [f"{v:.4f}" for v in _DEFAULT_HOME_XYZ]


HOME_XYZ = _home_xyz()
# Gentle, vertical (tool-down) move on the auto-detected command path.
DEFAULT_ARGS: List[str] = (
    ["--xyz", *HOME_XYZ, "--tool-down", "--speed", "0.02", "--cmd-mode", "auto"]
)


class HomeController:
    """Owns at most one "go home" (move-to-defined-pose) subprocess."""

    def __init__(self, enabled: bool = True, log_lines: int = 300) -> None:
        self.enabled = enabled            # live mode only
        self._lock = threading.Lock()
        self._proc: Optional[subprocess.Popen] = None
        self._started_at: Optional[float] = None
        self._last_exit: Optional[int] = None
        self._log: "collections.deque[str]" = collections.deque(maxlen=log_lines)

    def _running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def _drain(self, proc: subprocess.Popen) -> None:
        try:
            assert proc.stdout is not None
            for line in proc.stdout:
                self._log.append(line.rstrip("\n"))
        except Exception:  # noqa: BLE001
            pass
        rc = proc.wait()
        self._last_exit = rc
        self._log.append(f"[dashboard] home process exited (rc={rc})")

    def run(self, extra_args: Optional[List[str]] = None) -> Dict:
        if not self.enabled:
            return {"ok": False, "error": "home control is live-mode only"}
        with self._lock:
            if self._running():
                return {"ok": False, "error": "home already running",
                        "pid": self._proc.pid}  # type: ignore[union-attr]
            cmd = ["python3", "-u", MOVE_SCRIPT] + DEFAULT_ARGS + list(extra_args or [])
            self._log.clear()
            self._last_exit = None
            try:
                self._proc = subprocess.Popen(
                    cmd, cwd=REPO_ROOT,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, bufsize=1, start_new_session=True)
            except Exception as exc:  # noqa: BLE001
                self._proc = None
                return {"ok": False, "error": f"spawn failed: {exc}"}
            self._started_at = time.time()
            self._log.append("[dashboard] HOME (move to " + " ".join(HOME_XYZ) + ") "
                             + " ".join(cmd))
            threading.Thread(target=self._drain, args=(self._proc,), daemon=True).start()
            return {"ok": True, "pid": self._proc.pid}

    def stop(self) -> Dict:
        with self._lock:
            if not self._running():
                return {"ok": False, "error": "no home running"}
            pid = self._proc.pid  # type: ignore[union-attr]
            self._log.append("[dashboard] STOP requested -> SIGINT")
            try:
                os.killpg(os.getpgid(pid), signal.SIGINT)
            except Exception as exc:  # noqa: BLE001
                return {"ok": False, "error": f"signal failed: {exc}"}
        for _ in range(30):
            if not self._running():
                break
            time.sleep(0.1)
        with self._lock:
            if self._running():
                try:
                    os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)  # type: ignore[union-attr]
                    self._log.append("[dashboard] process did not exit -> SIGKILL")
                except Exception:  # noqa: BLE001
                    pass
        return {"ok": True}

    def status(self) -> Dict:
        with self._lock:
            running = self._running()
            return {
                "enabled": self.enabled,
                "running": running,
                "pid": (self._proc.pid if (self._proc and running) else None),
                "elapsed_s": (round(time.time() - self._started_at, 1)
                              if (running and self._started_at is not None) else None),
                "last_exit": self._last_exit,
                "home_xyz": [float(v) for v in HOME_XYZ],
                "log": list(self._log),
            }
