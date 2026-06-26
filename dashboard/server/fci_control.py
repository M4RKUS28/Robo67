"""Activate / deactivate the Franka Control Interface (FCI) from the dashboard.

The "FCI on/off" header button toggles the FCI over the Desk HTTP API (see
``desk_client.DeskClient``) instead of clicking the button in the Desk web UI.
This matters because **FCI active => the Desk UI is locked out**, so the
dashboard becoming the single point of control is exactly what we want during a
run.

Design (mirrors bringup_control's "owns a long operation, exposes status"):

  * **One persistent ``DeskClient``** is kept and reused across toggles. We log
    in + take control ONCE (a forced take needs a physical button tap on the
    robot per Single Point of Control); subsequent activate/deactivate reuse the
    held control token, so flipping FCI on/off does NOT keep asking for the tap.
  * Each toggle runs in a background thread; ``status()`` returns only in-memory
    tracked state (never hits the robot) so the 1 Hz UI poll is cheap.
  * A best-effort startup probe reads the current FCI state once so the button
    usually shows the right label; otherwise state is tracked from toggles.
  * **Live mode only** -- like the other real-arm controls -- though it only
    needs HTTPS to the robot (``ROS_LOCALHOST_ONLY`` affects DDS, not TCP).
"""
from __future__ import annotations

import collections
import os
import threading
import time
from typing import Dict, List, Optional

from desk_client import DeskClient, DeskError

# Desk credentials / host (env-overridable, like bringup_control's ROBOT_IP).
# Defaults match docs/CHALLENGE.md + CLAUDE.md (franka / frankaRSI @ the arm IP).
DESK_HOST = os.environ.get(
    "ROBO67_DESK_HOST", os.environ.get("ROBO67_ROBOT_IP", "192.168.1.67"))
DESK_USER = os.environ.get("ROBO67_DESK_USER", "franka")
DESK_PASS = os.environ.get("ROBO67_DESK_PASS", "frankaRSI")

# How long to wait for a forced control hand-off (physical button tap) (s).
TAKE_CONTROL_TIMEOUT = float(os.environ.get("ROBO67_FCI_TAKE_TIMEOUT", "30"))


class FciController:
    """Owns the Desk session and at most one FCI toggle at a time."""

    def __init__(self, enabled: bool = True, host: str = DESK_HOST,
                 user: str = DESK_USER, password: str = DESK_PASS,
                 log_lines: int = 300) -> None:
        self.enabled = enabled                 # live mode only
        self._host, self._user, self._password = host, user, password
        self._lock = threading.Lock()
        self._busy = False
        self._last_action: Optional[str] = None      # "activate" / "deactivate"
        self._ok: Optional[bool] = None
        self._error: Optional[str] = None
        self._awaiting_button = False           # forced take-control in progress
        self._fci_active: Optional[bool] = None  # tracked state (None = unknown)
        self._started_at: Optional[float] = None
        self._log: "collections.deque[str]" = collections.deque(maxlen=log_lines)
        # one persistent session, lazily created on first toggle and reused so
        # we keep holding control between toggles (no repeated button taps).
        self._client: Optional[DeskClient] = None
        if enabled:
            threading.Thread(target=self._probe_state, daemon=True).start()

    # -- helpers --------------------------------------------------------

    def _emit(self, line: str) -> None:
        msg = f"[{time.strftime('%H:%M:%S')}] {line}"
        self._log.append(msg)
        # also to stdout so it shows in the run_live.sh / docker exec terminal
        print(f"[fci] {msg}", flush=True)

    def _probe_state(self) -> None:
        """Best-effort startup probe: verify the Desk is reachable and log who
        currently holds control. (This firmware exposes no FCI-state endpoint,
        so ``fci_active`` stays unknown until the first toggle.)"""
        try:
            probe = DeskClient(self._host, self._user, self._password, timeout=6.0,
                               log=self._emit)
            probe.login()
            _, owner = probe.get_active_token()
            probe.close()
            self._emit(f"startup probe: Desk reachable @ {self._host}; "
                       f"control owner={owner if owner else 'none (free)'}")
        except Exception as exc:  # noqa: BLE001
            self._emit(f"startup probe failed: {exc}")

    def _client_or_new(self) -> DeskClient:
        if self._client is None:
            self._client = DeskClient(self._host, self._user, self._password,
                                      log=self._emit)
        return self._client

    # -- the toggle (runs in a background thread) -----------------------

    def _run(self, action: str) -> None:
        try:
            client = self._client_or_new()
            client.ensure_session()
            self._emit(f"logged in to Desk @ {self._host} as {self._user}")

            if not client.has_control():
                self._emit("acquiring control ...")

                # only fires when control is held by another user (force path)
                def _hint() -> None:
                    with self._lock:
                        self._awaiting_button = True
                    self._emit("control held by another user -> PRESS THE CIRCLE "
                               f"BUTTON ON THE ROBOT within {TAKE_CONTROL_TIMEOUT:.0f}s")

                got = client.take_control(
                    wait_timeout=TAKE_CONTROL_TIMEOUT, on_request=_hint)
                with self._lock:
                    self._awaiting_button = False
                if not got:
                    raise DeskError(
                        "could not acquire control (still held / no button tap)")
                self._emit("control acquired")

            if action == "activate":
                client.activate_fci()
                self._fci_active = True
                self._emit("FCI ACTIVATED (Desk UI is now locked out)")
            else:
                client.deactivate_fci()
                self._fci_active = False
                self._emit("FCI DEACTIVATED")

            with self._lock:
                self._ok = True
                self._error = None
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self._ok = False
                self._error = str(exc)
            self._emit(f"{action} FAILED: {exc}")
            # a failed session may be half-open; force a fresh login next time
            if self._client is not None:
                try:
                    self._client.close()
                except Exception:  # noqa: BLE001
                    pass
                self._client = None
        finally:
            with self._lock:
                self._busy = False
                self._awaiting_button = False

    def _start(self, action: str) -> Dict:
        if not self.enabled:
            return {"ok": False, "error": "FCI control is live-mode only"}
        with self._lock:
            if self._busy:
                return {"ok": False, "error": "FCI toggle already in progress"}
            self._busy = True
            self._last_action = action
            self._ok = None
            self._error = None
            self._awaiting_button = False
            self._started_at = time.time()
        self._emit(f"{action.upper()} requested")
        threading.Thread(target=self._run, args=(action,), daemon=True).start()
        return {"ok": True, "started": True}

    # -- public API -----------------------------------------------------

    def activate(self) -> Dict:
        return self._start("activate")

    def deactivate(self) -> Dict:
        return self._start("deactivate")

    def status(self) -> Dict:
        with self._lock:
            busy = self._busy
            started = self._started_at
            return {
                "enabled": self.enabled,
                "busy": busy,
                "awaiting_button": self._awaiting_button,
                "fci_active": self._fci_active,
                "last_action": self._last_action,
                "ok": self._ok,
                "error": self._error,
                "host": self._host,
                "elapsed_s": (round(time.time() - started, 1)
                              if (busy and started is not None) else None),
                "log": list(self._log),
            }
