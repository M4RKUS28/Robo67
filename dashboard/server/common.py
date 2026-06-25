"""Shared backend utilities for the Robo67 insertion dashboard.

Pure stdlib (no third-party deps beyond numpy, which the insertion libs already
require). Holds the telemetry fan-out hub, the canonical phase metadata mirrored
from ``robo67_insertion.lib.insertion_intent``, and small JSON/SSE helpers.

The whole backend deliberately avoids FastAPI/uvicorn/websockets: telemetry is
pushed over Server-Sent Events (plain ``text/event-stream``) and camera frames
over MJPEG (``multipart/x-mixed-replace``). Both are plain HTTP, so the bridge
runs unmodified on the host (mock mode) or inside ``multipanda-container`` (live
mode) with zero install.
"""
from __future__ import annotations

import json
import os
import queue
import threading
import time
from typing import Any, Dict, Iterator, List, Optional

# --- repo layout -----------------------------------------------------------

SERVER_DIR = os.path.dirname(os.path.abspath(__file__))
DASHBOARD_DIR = os.path.dirname(SERVER_DIR)
REPO_ROOT = os.path.dirname(DASHBOARD_DIR)
INSERTION_PKG = os.path.join(REPO_ROOT, "robo67_insertion")
CAPTURES_DIR = os.path.join(INSERTION_PKG, "captures")


def ensure_insertion_on_path() -> None:
    """Put ``robo67_insertion`` on sys.path so the pure libs import."""
    import sys

    if INSERTION_PKG not in sys.path:
        sys.path.insert(0, INSERTION_PKG)


# --- canonical phase metadata (mirrors lib.insertion_intent.PHASES) --------

PHASE_ORDER: List[str] = [
    "IDLE",
    "MOVE_ABOVE",
    "DESCEND_TO_CONTACT",
    "SEARCH_SPIRAL",
    "PUSH_INSERT",
    "CONFIRM",
    "RETRACT",
    "DONE",
    "ERROR",
]

PHASE_LABEL: Dict[str, str] = {
    "IDLE": "Idle",
    "MOVE_ABOVE": "Move above",
    "DESCEND_TO_CONTACT": "Descend to contact",
    "SEARCH_SPIRAL": "Spiral search",
    "PUSH_INSERT": "Push / insert",
    "CONFIRM": "Confirm seat",
    "RETRACT": "Retract",
    "DONE": "Done",
    "ERROR": "Error",
}

# Maps each FSM phase to the contact-lifecycle mode (mirrors the orchestrator's
# _PHASE_TO_CONTACT_MODE so the mock baseline behaves like the real loop).
PHASE_CONTACT_MODE: Dict[str, str] = {
    "IDLE": "free_space",
    "MOVE_ABOVE": "free_space",
    "DESCEND_TO_CONTACT": "contact_search",
    "SEARCH_SPIRAL": "contact_search",
    "PUSH_INSERT": "insert",
    "CONFIRM": "confirm",
    "RETRACT": "confirm",
    "DONE": "confirm",
    "ERROR": "confirm",
}

# Franka robot_mode enum (from CLAUDE.md): 1=Idle 2=Move 4=Reflex 5=UserStopped.
ROBOT_MODE_LABEL: Dict[int, str] = {
    0: "Other",
    1: "Idle",
    2: "Move",
    3: "Guiding",
    4: "Reflex",
    5: "User stopped",
    6: "Automatic recovery",
}


def phase_to_robot_mode(phase: str, aborted: bool) -> int:
    if aborted or phase == "ERROR":
        return 4
    if phase in ("IDLE", "DONE"):
        return 1
    return 2


# --- telemetry fan-out hub -------------------------------------------------

class Hub:
    """Thread-safe latest-value cache + multi-subscriber fan-out.

    The provider's worker thread calls :meth:`publish` with each telemetry
    snapshot. Each SSE connection calls :meth:`subscribe` to get its own bounded
    queue and drains it; slow clients drop the oldest frame rather than block
    the producer.
    """

    def __init__(self, maxsize: int = 8) -> None:
        self._lock = threading.Lock()
        self._subs: List["queue.Queue[dict]"] = []
        self._latest: Optional[dict] = None
        self._maxsize = maxsize

    def publish(self, snapshot: dict) -> None:
        with self._lock:
            self._latest = snapshot
            subs = list(self._subs)
        for q in subs:
            try:
                q.put_nowait(snapshot)
            except queue.Full:
                try:
                    q.get_nowait()  # drop oldest
                    q.put_nowait(snapshot)
                except queue.Empty:
                    pass

    def latest(self) -> Optional[dict]:
        with self._lock:
            return self._latest

    def subscribe(self) -> "queue.Queue[dict]":
        q: "queue.Queue[dict]" = queue.Queue(maxsize=self._maxsize)
        with self._lock:
            self._subs.append(q)
        return q

    def unsubscribe(self, q: "queue.Queue[dict]") -> None:
        with self._lock:
            if q in self._subs:
                self._subs.remove(q)


# --- HTTP helpers ----------------------------------------------------------

def sse_pack(payload: Any, event: Optional[str] = None) -> bytes:
    """Encode a payload as one SSE message."""
    out = []
    if event:
        out.append(f"event: {event}")
    data = json.dumps(payload, separators=(",", ":"))
    out.append(f"data: {data}")
    out.append("")  # blank line terminates the event
    out.append("")
    return ("\n".join(out)).encode("utf-8")


def now() -> float:
    return time.time()
