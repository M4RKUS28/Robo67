"""Start / stop the automated insertion from the dashboard (peg-in-hole or cable).

In ``live`` mode the dashboard server runs INSIDE ``multipanda-container`` with
ROS sourced (domain 1) -- see ``run_live.sh`` -- so it can spawn
``hw_peg_in_hole_vision.py`` as a subprocess that inherits the ROS environment
and commands the real arm. This controller owns that subprocess:

  * one run at a time (no double-start),
  * SIGINT to cancel -- the insertion node's KeyboardInterrupt handler holds the
    last commanded pose (it does NOT drop the arm), then SIGKILL if it is stuck,
  * stdout ring-buffered so the UI can show phase transitions / the release.

The defaults below are the parameter set verified live on 2026-06-25 (see
``docs/runbooks/automated-insertion.md``): firm search press (the peg must sit
ON the surface, past the stiction deadband), wide spiral, raised moment cap, and
release-on-insert keyed to the DESCEND ``contact_z`` hole-top.
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

VISION_SCRIPT_PEG = os.path.join(INSERTION_PKG, "scripts", "hw_peg_in_hole_vision.py")
VISION_SCRIPT_CABLE = os.path.join(INSERTION_PKG, "scripts", "hw_cable_insertion_vision.py")

# Verified-live real-arm parameter set (pos_stiff MUST match the running
# controller; the firmware is the real guardrail). Keep in sync with the runbook.
# peg-in-hole args.
DEFAULT_ARGS_PEG: List[str] = [
    "--socket-top-z", "0.1465",      # taught hole-top; DESCEND force-probes the true Z
    "--pos-stiff", "2000",           # MUST match the controller's pos_stiff
    "--approach-tol", "0.015",       # >= free-space stiction deadband (~8 mm)
    "--v-max", "0.02",
    "--standoff", "0.05",
    "--contact-fz", "5",
    "--press-force", "18",           # firm enough to keep the peg ON the surface
    "--max-press-depth", "0.05",
    "--spiral-max-radius", "0.02",   # cover detection + deadband offset
    "--f-abort", "30",
    "--torque-abort", "12",          # headroom over the peg-weight torque offset
    "--release-on-insert",
    "--insert-drop-trigger", "0.003",  # release as soon as EE dips below contact_z
    "--gripper-open-width", "0.08",
    "--retract-after", "0.06",
]

# Cable-insertion args. The cable runner (hw_cable_insertion_vision.py) bakes in
# its own tuned defaults (pos_stiff 2000, press 18, spiral 0.02, torque-abort 10,
# seat-while-gripped) AND defaults force-mode ON, so here we only supply the taught
# box-top Z (an overhead camera can't measure it). Re-teach via hw_grab_box_template.py
# / hw_teach_port_offset.py and set ROBO67_BOX_TOP_Z if the rig moves.
BOX_TOP_Z = os.environ.get("ROBO67_BOX_TOP_Z", "0.211")
DEFAULT_ARGS_CABLE: List[str] = ["--box-top-z", BOX_TOP_Z]


class InsertionController:
    """Owns at most one automated-insertion subprocess."""

    def __init__(self, enabled: bool = True, log_lines: int = 500) -> None:
        self.enabled = enabled          # only true in live mode
        self._lock = threading.Lock()
        self._proc: Optional[subprocess.Popen] = None
        self._started_at: Optional[float] = None
        self._last_exit: Optional[int] = None
        self._force_mode: bool = False   # whether the current/last run used --force-mode
        self._mode: str = "peg"          # current/last run's insertion mode ("peg"|"cable")
        self._log: "collections.deque[str]" = collections.deque(maxlen=log_lines)

    # -- internal -------------------------------------------------------
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
        self._log.append(f"[dashboard] insertion process exited (rc={rc})")

    # -- public API -----------------------------------------------------
    def start(self, mode: str = "peg", force_mode: bool = False,
              extra_args: Optional[List[str]] = None) -> Dict:
        if not self.enabled:
            return {"ok": False, "error": "insertion control is live-mode only"}
        with self._lock:
            if self._running():
                return {"ok": False, "error": "insertion already running",
                        "pid": self._proc.pid}  # type: ignore[union-attr]
            # force_mode = regulate a constant gentle press + force-slacken detect
            # (ADR-0002); keeps the verified --press-force/etc. for now.
            mode = "cable" if str(mode).lower() == "cable" else "peg"
            if mode == "cable":
                script, base_args = VISION_SCRIPT_CABLE, DEFAULT_ARGS_CABLE
                # the cable runner defaults force-mode ON -> pass an explicit on/off flag
                mode_args = ["--force-mode"] if force_mode else ["--no-force-mode"]
            else:
                script, base_args = VISION_SCRIPT_PEG, DEFAULT_ARGS_PEG
                mode_args = ["--force-mode"] if force_mode else []
            cmd = (["python3", "-u", script] + base_args
                   + mode_args + list(extra_args or []))
            self._mode = mode
            self._force_mode = bool(force_mode)
            self._log.clear()
            self._last_exit = None
            try:
                # start_new_session => own process group, so a SIGINT to the
                # group reaches the node (and any children) without touching the
                # dashboard server itself.
                self._proc = subprocess.Popen(
                    cmd, cwd=REPO_ROOT,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, bufsize=1, start_new_session=True)
            except Exception as exc:  # noqa: BLE001
                self._proc = None
                return {"ok": False, "error": f"spawn failed: {exc}"}
            self._started_at = time.time()
            self._log.append("[dashboard] START " + " ".join(cmd))
            threading.Thread(target=self._drain, args=(self._proc,), daemon=True).start()
            return {"ok": True, "pid": self._proc.pid}

    def stop(self) -> Dict:
        with self._lock:
            if not self._running():
                return {"ok": False, "error": "no insertion running"}
            pid = self._proc.pid  # type: ignore[union-attr]
            self._log.append("[dashboard] STOP requested -> SIGINT (hold last pose)")
            try:
                os.killpg(os.getpgid(pid), signal.SIGINT)
            except Exception as exc:  # noqa: BLE001
                return {"ok": False, "error": f"signal failed: {exc}"}
        # let the node's interrupt handler hold the pose + exit; escalate if stuck
        for _ in range(40):
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
                "mode": self._mode,
                "force_mode": self._force_mode,
                "pid": (self._proc.pid if (self._proc and running) else None),
                "elapsed_s": (round(time.time() - self._started_at, 1)
                              if (running and self._started_at is not None) else None),
                "last_exit": self._last_exit,
                "log": list(self._log),
            }
