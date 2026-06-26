"""Minimal Franka Desk HTTP client (pure stdlib) for toggling the FCI.

The Desk web UI (``https://<robot>/desk/``) is backed by an HTTP+WebSocket API
that is not officially documented but is stable and widely used (panda-py,
franky). This module reimplements just the slice we need -- login, take control,
activate / deactivate the Franka Control Interface (FCI) -- with **stdlib only**
(``http.client`` + ``ssl`` + ``hashlib`` + ``base64``), matching the
dashboard's "stdlib + numpy" rule (no ``requests`` / ``panda_py`` dep).

Endpoints (verified against this Panda's firmware via the dashboard FCI log,
matching panda-py's ``platform='panda'`` path and frankaemika/franka_ros #221):

    POST   /admin/api/login                        -> session token (cookie)
    GET    /admin/api/control-token                -> {activeToken: {id, ownedBy}|null}
    POST   /admin/api/control-token/request[?force]-> {token, id} (control token)
    POST   /admin/api/control-token/fci            -> activate FCI   {token: <ctrl>}
    DELETE /admin/api/control-token/fci            -> deactivate FCI {token: <ctrl>}
    DELETE /admin/api/control-token                -> release control

NB: ``/admin/api/system-status`` does NOT exist on this firmware (it 404s -- it
is an FR3-only endpoint), so control is checked via ``/admin/api/control-token``.

Single Point of Control: if another client (e.g. an open Desk tab) holds
control, ``request_control(force=True)`` only *registers* our request -- control
is granted once someone **physically taps the button on the robot**, which is
why ``take_control`` polls ``has_control`` until a timeout.

The control token is kept in memory and reused across reconnects, so toggling
FCI on/off repeatedly does NOT require re-tapping the robot button (we keep
holding control between toggles; we never auto-release it).
"""
from __future__ import annotations

import base64
import hashlib
import http.client
import json
import ssl
from typing import Any, Callable, Dict, Optional


class DeskError(RuntimeError):
    """Raised when a Desk API call returns a non-2xx status."""


class DeskClient:
    """Stateful Desk session: holds the login cookie + control token."""

    def __init__(self, host: str, user: str, password: str, timeout: float = 10.0,
                 log: Optional[Callable[[str], None]] = None) -> None:
        self._host = host
        self._user = user
        self._password = password
        self._timeout = timeout
        self._log = log                                  # diagnostic log sink
        self._conn: Optional[http.client.HTTPSConnection] = None
        self._session_token: Optional[str] = None      # login cookie value
        self._control_token: Optional[str] = None       # control-token value
        self._control_token_id: Optional[Any] = None     # control-token id

    def _logmsg(self, line: str) -> None:
        if self._log is not None:
            try:
                self._log(line)
            except Exception:  # noqa: BLE001
                pass

    # -- low-level HTTP -------------------------------------------------

    @staticmethod
    def _encode_password(user: str, password: str) -> str:
        """Desk login password encoding: base64 of the comma-joined sha256 bytes
        of ``password#user@franka`` (the scheme the Desk frontend uses)."""
        digest = hashlib.sha256(
            (password + "#" + user + "@franka").encode("utf-8")).digest()
        joined = ",".join(str(b) for b in digest)
        return base64.b64encode(joined.encode("utf-8")).decode("utf-8")

    def _ensure_conn(self) -> http.client.HTTPSConnection:
        if self._conn is None:
            # the robot serves a self-signed cert -> unverified TLS context
            self._conn = http.client.HTTPSConnection(
                self._host, timeout=self._timeout,
                context=ssl._create_unverified_context())
        return self._conn

    def _drop_conn(self) -> None:
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:  # noqa: BLE001
                pass
        self._conn = None

    def _request(self, method: str, target: str,
                 headers: Optional[Dict[str, str]] = None,
                 body: Optional[str] = None,
                 control: bool = False) -> bytes:
        """One Desk API request. Adds the session cookie (and, when
        ``control`` is set, the X-Control-Token header). Retries once on a
        dropped keep-alive connection."""
        hdrs: Dict[str, str] = {}
        if self._session_token is not None:
            hdrs["Cookie"] = f"authorization={self._session_token}"
        if control:
            if self._control_token is None:
                raise DeskError("no control token -- take_control() first")
            hdrs["X-Control-Token"] = self._control_token
        if headers:
            hdrs.update(headers)

        last_exc: Optional[Exception] = None
        for attempt in range(2):
            conn = self._ensure_conn()
            try:
                conn.request(method, target, body=body, headers=hdrs)
                res = conn.getresponse()
                data = res.read()
                self._logmsg(f"HTTP {method} {target} -> {res.status} {res.reason} "
                             f"({len(data)}B)")
                if not (200 <= res.status < 300):
                    raise DeskError(
                        f"{method} {target} -> {res.status} {res.reason}: "
                        f"{data.decode('utf-8', 'replace')[:200]}")
                return data
            except (http.client.RemoteDisconnected, ConnectionError,
                    http.client.BadStatusLine) as exc:
                last_exc = exc
                self._logmsg(f"HTTP {method} {target} dropped ({exc!r}); "
                             f"{'retrying' if attempt == 0 else 'giving up'}")
                self._drop_conn()  # reconnect + retry once
        raise DeskError(f"{method} {target} failed: {last_exc}")

    # -- session --------------------------------------------------------

    def login(self) -> None:
        """(Re)establish the session. The control token (if any) is kept."""
        self._session_token = None
        payload = json.dumps({
            "login": self._user,
            "password": self._encode_password(self._user, self._password),
        })
        self._logmsg(f"login: POST /admin/api/login as user={self._user!r}")
        token = self._request(
            "POST", "/admin/api/login",
            headers={"content-type": "application/json"}, body=payload)
        self._session_token = token.decode("utf-8").strip().strip('"')
        self._logmsg(f"login: ok (session token len="
                     f"{len(self._session_token or '')})")

    def ensure_session(self) -> None:
        if self._session_token is None:
            self.login()

    # -- control token (Single Point of Control) ------------------------

    def get_active_token(self) -> "tuple[Optional[Any], Optional[str]]":
        """The robot's current control claim as ``(active_id, owned_by)`` via
        ``GET /admin/api/control-token`` -- the endpoint that exists on the
        Panda firmware (NOT ``/admin/api/system-status``, which is FR3-only and
        404s here). Returns ``(None, None)`` when nobody holds control."""
        self.ensure_session()
        data = self._request("GET", "/admin/api/control-token")
        resp = json.loads(data.decode("utf-8")) if data else {}
        active = resp.get("activeToken") if isinstance(resp, dict) else None
        if isinstance(active, dict):
            return active.get("id"), (active.get("ownedBy") or active.get("owned_by"))
        return None, None

    def has_control(self) -> bool:
        """True iff the robot's currently active control token is ours."""
        if self._control_token_id is None:
            self._logmsg("has_control: no control-token requested yet -> False")
            return False
        try:
            active_id, owner = self.get_active_token()
        except Exception as exc:  # noqa: BLE001
            self._logmsg(f"has_control: control-token read FAILED: {exc}")
            return False
        # ids may come back as int or str across firmwares -> compare as str
        held = active_id is not None and str(active_id) == str(self._control_token_id)
        self._logmsg(
            f"has_control: active_id={active_id!r} owner={owner!r} "
            f"our_id={self._control_token_id!r} -> {held}")
        return held

    def request_control(self, force: bool = False) -> None:
        self.ensure_session()
        target = "/admin/api/control-token/request" + ("?force" if force else "")
        self._logmsg(f"request_control: POST {target} requestedBy={self._user!r}")
        data = self._request(
            "POST", target, headers={"content-type": "application/json"},
            body=json.dumps({"requestedBy": self._user}))
        resp = json.loads(data.decode("utf-8"))
        self._control_token = resp.get("token")
        self._control_token_id = resp.get("id")
        self._logmsg(
            f"request_control: response keys={list(resp) if isinstance(resp, dict) else type(resp).__name__} "
            f"id={self._control_token_id!r} token_len={len(self._control_token or '')}")

    def take_control(self, wait_timeout: float = 30.0, poll: float = 1.0,
                     on_request: Optional[Callable[[], None]] = None) -> bool:
        """Acquire control. If control is **free** it is granted immediately on
        request (no button). If **another user** holds it, we force the request
        and the user must press the **circle button on the robot**; we then poll
        ``GET /admin/api/control-token`` until the claim transfers to us (or
        ``wait_timeout`` elapses). Returns True if control is held."""
        import time
        try:
            active_id, owner = self.get_active_token()
        except Exception as exc:  # noqa: BLE001
            self._logmsg(f"take_control: control-token read failed: {exc}")
            active_id, owner = None, None
        # already ours?
        if (active_id is not None and self._control_token_id is not None
                and str(active_id) == str(self._control_token_id)):
            self._logmsg("take_control: already holding control")
            return True
        held_by_other = active_id is not None
        self._logmsg(
            f"take_control: current owner={owner!r} active_id={active_id!r} -> "
            + ("held by another user (force + button needed)" if held_by_other
               else "control is FREE (no button needed)"))
        # force only when someone else holds control
        self.request_control(force=held_by_other)
        if not held_by_other:
            if self.has_control():
                self._logmsg("take_control: GRANTED (control was free)")
                return True
            self._logmsg("take_control: requested free control, not active yet -> polling")
        elif on_request is not None:
            on_request()  # caller logs the "press the circle button" hint
        deadline = time.time() + wait_timeout
        n = 0
        while time.time() < deadline:
            time.sleep(poll)
            n += 1
            if self.has_control():
                self._logmsg(f"take_control: GRANTED after {n} poll(s)")
                return True
            self._logmsg(
                "take_control: waiting "
                + ("(press the circle button) " if held_by_other else "")
                + f"(poll {n}, {max(0.0, deadline - time.time()):.0f}s left)")
        self._logmsg(f"take_control: TIMED OUT after {wait_timeout:.0f}s "
                     f"({n} polls) -- control never granted")
        return False

    def release_control(self) -> None:
        if self._control_token is None:
            return
        try:
            self._request(
                "DELETE", "/admin/api/control-token",
                headers={"content-type": "application/json"}, control=True,
                body=json.dumps({"token": self._control_token}))
        finally:
            self._control_token = None
            self._control_token_id = None

    # -- FCI ------------------------------------------------------------

    def activate_fci(self) -> None:
        """Activate the FCI (control token must be held; activating an already
        active FCI is a no-op on the robot side)."""
        self._logmsg("activate_fci: POST /admin/api/control-token/fci")
        self._request(
            "POST", "/admin/api/control-token/fci",
            headers={"content-type": "application/json"}, control=True,
            body=json.dumps({"token": self._control_token}))
        self._logmsg("activate_fci: ok")

    def deactivate_fci(self) -> None:
        self._logmsg("deactivate_fci: DELETE /admin/api/control-token/fci")
        self._request(
            "DELETE", "/admin/api/control-token/fci",
            headers={"content-type": "application/json"}, control=True,
            body=json.dumps({"token": self._control_token}))
        self._logmsg("deactivate_fci: ok")

    # NOTE: this Panda firmware exposes no system-status / FCI-state endpoint,
    # so FCI state can't be read back -- callers track it from the last
    # successful toggle.

    def close(self) -> None:
        self._drop_conn()
        self._session_token = None
