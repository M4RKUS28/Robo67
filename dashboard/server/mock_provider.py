"""Mock telemetry + camera provider for the insertion dashboard.

Drives a continuous peg-in-hole insertion using the REAL canonical seam
(:mod:`robo67_insertion.lib.insertion_intent` +
:mod:`robo67_insertion.lib.contact_lifecycle`) against a small virtual plant
(first-order EE + table/hole, the same shape as the offline self-tests). Every
tick it emits a telemetry snapshot (phase, EE pose, command, EE speed, external
wrench, contact baseline, detections) onto a :class:`~common.Hub`. When the run
reaches DONE it briefly holds, randomises the socket-alignment error, and loops
so the dashboard always shows activity.

Cameras are served from the saved stills in ``robo67_insertion/captures/`` (the
overhead C920 socket view + the D405 eye-in-hand view). Detection markers are
computed once with the real :func:`detect_white_cubes` and streamed in telemetry; the
frontend overlays them on the MJPEG feeds.

No ROS, no real camera, no robot -- runs anywhere ``numpy`` (and ideally
``opencv-python``) is importable.
"""
from __future__ import annotations

import glob
import math
import os
import random
import threading
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

from common import (
    CAPTURES_DIR,
    PHASE_CONTACT_MODE,
    PHASE_LABEL,
    PHASE_ORDER,
    ROBOT_MODE_LABEL,
    Hub,
    ensure_insertion_on_path,
    now,
    phase_to_robot_mode,
)

ensure_insertion_on_path()

from robo67_insertion.lib.contact_lifecycle import ContactLifecycleModule  # noqa: E402
from robo67_insertion.lib.insertion_intent import (  # noqa: E402
    InsertionIntentModule,
    IntentParams,
    IntentSensors,
)

# Preferred camera stills (most-likely-to-contain-a-clear-hole first). The exact
# set of capture files changes over time, so we glob what actually exists at
# startup and fall back gracefully -- never hard-depend on one filename.
C920_PREF = ["c920_holes.jpg", "c920_circles.jpg", "c920_live.jpg", "c920_peg.jpg"]
D405_PREF = ["d405_live.jpg", "d405_newfloor.jpg", "d405_vert.jpg"]


def _load_jpeg(path: Optional[str]) -> Optional[bytes]:
    if not path:
        return None
    try:
        with open(path, "rb") as fh:
            return fh.read()
    except OSError:
        return None


def _existing_stills(prefix: str, preferred: List[str]) -> List[str]:
    """Existing capture files for a camera prefix, preferred names first."""
    found = []
    for name in preferred:
        p = os.path.join(CAPTURES_DIR, name)
        if os.path.isfile(p):
            found.append(p)
    for p in sorted(glob.glob(os.path.join(CAPTURES_DIR, f"{prefix}*.jpg"))):
        if p not in found:
            found.append(p)
    return found


def _pick_still(prefix: str, preferred: List[str], fallback_size,
                fallback_det: dict) -> Tuple[Optional[str], tuple, dict]:
    """Pick the best existing still: first one with a detectable hole, else any.

    Returns ``(path, (w, h), detection)``. ``detection`` always has u/v/radius/
    score (synthesised at the frame centre when nothing is detected).
    """
    candidates = _existing_stills(prefix, preferred)
    if not candidates:
        return None, fallback_size, dict(fallback_det)

    best_path = candidates[0]
    best_size = fallback_size
    best_det = None
    try:
        import cv2

        from robo67_insertion.lib.hole_detect import WhiteCubeParams, detect_white_cubes

        for path in candidates:
            img = cv2.imread(path)
            if img is None:
                continue
            h, w = img.shape[:2]
            if best_det is None:  # remember first readable frame + its size
                best_path, best_size = path, (int(w), int(h))
            holes = detect_white_cubes(img, WhiteCubeParams())
            if holes:
                hole = holes[0]
                return path, (int(w), int(h)), dict(
                    u=float(hole.u), v=float(hole.v),
                    radius_px=float(hole.radius_px), score=float(hole.score))
    except Exception:
        pass

    # readable frame but no hole detected -> synthesise a centred marker
    w, h = best_size
    if best_det is None:
        best_det = dict(u=w / 2.0, v=h / 2.0,
                        radius_px=max(12.0, min(w, h) * 0.05), score=0.0)
    return best_path, best_size, best_det


class MockProvider:
    name = "mock"

    # plant / insertion constants
    RATE_HZ = 50.0
    SOCKET = np.array([0.45, 0.0, 0.10])
    EE_HOME = np.array([0.45, 0.0, 0.30])
    HOLE_R = 0.005
    V_MAX = 0.05
    POS_STIFF = 200.0
    CONTACT_FZ_N = 5.0
    F_ABORT_N = 25.0
    INSERT_DEPTH = 0.04
    # Below-surface press bias applied while spiralling (mirrors what the real
    # ImpedanceCommandPathAdapter does): the equilibrium sits below the surface
    # so the peg presses down and DROPS into the hole once XY-aligned. Without
    # it the spiral target z == contact_z and the peg never seats.
    SPIRAL_PRESS_M = 0.015
    WORKSPACE_AABB = [[0.05, 0.75], [-0.50, 0.50], [0.05, 1.20]]

    def __init__(self) -> None:
        self.hub = Hub()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._t0 = now()

        # camera assets -- pick whatever capture stills actually exist now
        c920_path, c920_size, self._c920_det = _pick_still(
            "c920", C920_PREF, (1920, 1080),
            dict(u=908.0, v=373.0, radius_px=99.0, score=0.69))
        d405_path, d405_size, self._d405_det = _pick_still(
            "d405", D405_PREF, (640, 480),
            dict(u=518.0, v=126.0, radius_px=22.0, score=0.68))
        self._cam_jpeg: Dict[str, Optional[bytes]] = {
            "c920": _load_jpeg(c920_path),
            "d405": _load_jpeg(d405_path),
        }
        self._cam_size: Dict[str, tuple] = {"c920": c920_size, "d405": d405_size}

    # -- lifecycle -------------------------------------------------------

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="mock-sim", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    # -- metadata --------------------------------------------------------

    def health(self) -> dict:
        return {
            "mode": self.name,
            "ros": False,
            "cameras": {n: (b is not None) for n, b in self._cam_jpeg.items()},
            "rate_hz": self.RATE_HZ,
        }

    def config(self) -> dict:
        return {
            "mode": self.name,
            "phases": [{"id": p, "label": PHASE_LABEL[p]} for p in PHASE_ORDER],
            "robot_modes": ROBOT_MODE_LABEL,
            "thresholds": {
                "contact_fz_n": self.CONTACT_FZ_N,
                "f_abort_n": self.F_ABORT_N,
                "speed_cap_mps": self.V_MAX,
                "insert_depth_m": self.INSERT_DEPTH,
            },
            "workspace_aabb": self.WORKSPACE_AABB,
            "cameras": {
                "c920": {"label": "C920 overhead", "size": self._cam_size["c920"],
                          "kind": "static-overhead"},
                "d405": {"label": "D405 eye-in-hand", "size": self._cam_size["d405"],
                          "kind": "eye-in-hand"},
            },
        }

    def latest(self) -> Optional[dict]:
        return self.hub.latest()

    # -- cameras ---------------------------------------------------------

    def camera_names(self) -> List[str]:
        return [n for n, b in self._cam_jpeg.items() if b is not None]

    def camera_jpeg(self, name: str) -> Optional[bytes]:
        return self._cam_jpeg.get(name)

    def camera_iter(self, name: str, fps: float = 12.0):
        period = 1.0 / max(1.0, fps)
        while not self._stop.is_set():
            frame = self._cam_jpeg.get(name)
            if frame is not None:
                yield frame
            time.sleep(period)

    # -- simulation ------------------------------------------------------

    def _make_run(self):
        """Fresh insertion run state (randomised alignment error each loop)."""
        params = IntentParams(
            standoff_m=0.05, approach_tol_m=0.010,
            contact_fz_threshold_n=self.CONTACT_FZ_N, insert_depth_m=self.INSERT_DEPTH,
            z_drop_threshold_m=0.004, retry_limit=3,
            spiral_pitch_m=0.002, spiral_speed_mps=0.005, spiral_max_radius_m=0.012,
        )
        # Misalignment MUST exceed the hole radius, otherwise the tool descends
        # straight into the hole at the socket centre and never contacts the
        # cube top (no contact -> stuck in DESCEND). Keep it below the spiral
        # max radius (0.012) so the search is always able to find the hole.
        ang = random.uniform(0, 2 * math.pi)
        mag = random.uniform(self.HOLE_R + 0.002, 0.010)
        hole_xy = self.SOCKET[:2] + mag * np.array([math.cos(ang), math.sin(ang)])
        return {
            "intent": InsertionIntentModule(self.SOCKET, params),
            "contact": ContactLifecycleModule(threshold_n=self.CONTACT_FZ_N, alpha=0.1),
            "params": params,
            "hole_xy": hole_xy,
            "hole_depth": self.INSERT_DEPTH + 0.005,
            "ee": self.EE_HOME.copy(),
            "cmd": self.EE_HOME.copy(),
            "phase": "MOVE_ABOVE",
            "retries": 0,
            "done": False,
            "error": None,
        }

    def _run(self) -> None:
        dt = 1.0 / self.RATE_HZ
        max_step = self.V_MAX / self.RATE_HZ
        run = self._make_run()
        prev_ee = run["ee"].copy()
        hold_until = 0.0

        while not self._stop.is_set():
            t = now() - self._t0
            phase = run["phase"]

            # hold briefly on DONE/ERROR, then restart a fresh run. Always
            # `continue` after handling DONE so the next iteration re-reads
            # `phase` from the (possibly fresh) run -- never fall through with a
            # stale local `phase`.
            if run["done"]:
                if hold_until == 0.0:
                    hold_until = time.time() + 2.5
                if time.time() >= hold_until:
                    run = self._make_run()
                    prev_ee = run["ee"].copy()
                    hold_until = 0.0
                else:
                    self.hub.publish(self._snapshot(run, t, 0.0, prev_ee, [], hold=True))
                    time.sleep(dt)
                continue

            ee = run["ee"]
            cmd = run["cmd"]
            hole_xy = run["hole_xy"]
            table_z = self.SOCKET[2]

            in_hole = math.hypot(ee[0] - hole_xy[0], ee[1] - hole_xy[1]) <= self.HOLE_R
            floor = table_z - (run["hole_depth"] if in_hole else 0.0)
            gap = max(0.0, ee[2] - cmd[2]) if ee[2] <= floor + 1e-6 else 0.0
            fz = self.POS_STIFF * gap  # reaction force magnitude (>= 0)

            mode = PHASE_CONTACT_MODE.get(phase, "confirm")
            outcome = run["contact"].observe(mode, fz)

            s = IntentSensors(ee_xyz=tuple(float(v) for v in ee), fz=fz,
                              fz_baseline=outcome.baseline_fz, t=t)
            intent_out = run["intent"].step(phase, s)
            target = np.asarray(intent_out.target_xyz, float)

            # press the equilibrium below the surface while searching so the peg
            # drops into the hole once aligned (the real adapter's press bias).
            if phase == "SEARCH_SPIRAL":
                target = target.copy()
                target[2] -= self.SPIRAL_PRESS_M

            # command chases the canonical target, velocity-limited
            step = target - cmd
            n = float(np.linalg.norm(step))
            if n > max_step:
                step = step * (max_step / n)
            cmd = cmd + step
            # plant: EE chases the command with first-order lag, blocked by floor
            ee_prev = ee
            ee_new = ee_prev + 0.25 * (cmd - ee_prev)
            ee_new[2] = max(ee_new[2], floor)
            # Cap the per-tick EE displacement so a floor change (e.g. the peg
            # sliding laterally out of the hole during RETRACT) can't teleport
            # the EE and produce an unphysical speed spike.
            max_ee_step = 3.0 * max_step
            delta = ee_new - ee_prev
            dn = float(np.linalg.norm(delta))
            if dn > max_ee_step:
                ee_new = ee_prev + delta * (max_ee_step / dn)
            ee = ee_new

            run["ee"], run["cmd"] = ee, cmd
            speed = float(np.linalg.norm(ee - prev_ee) / dt)

            # collect discrete decision events
            events = []
            next_phase = intent_out.phase
            if next_phase != phase:
                events.append({"t": t, "kind": "transition",
                               "from": phase, "to": next_phase,
                               "msg": f"{PHASE_LABEL[phase]} -> {PHASE_LABEL[next_phase]}"})
            if outcome.contact_detected and mode == "contact_search" \
                    and phase == "DESCEND_TO_CONTACT":
                events.append({"t": t, "kind": "contact",
                               "msg": f"contact at z={ee[2]:.3f} m (Fz {fz:.1f} N)"})
            if next_phase == "PUSH_INSERT" and phase == "SEARCH_SPIRAL":
                events.append({"t": t, "kind": "drop",
                               "msg": f"peg dropped into hole at "
                                      f"({ee[0]:.3f}, {ee[1]:.3f})"})
            if intent_out.error:
                events.append({"t": t, "kind": "error", "msg": intent_out.error})
            if next_phase == "DONE" and phase == "RETRACT":
                events.append({"t": t, "kind": "done", "msg": "insertion complete"})

            run["phase"] = next_phase
            run["retries"] = getattr(run["intent"], "retries", run["retries"])
            run["error"] = intent_out.error
            run["done"] = bool(intent_out.done)

            self.hub.publish(self._snapshot(run, t, speed, prev_ee, events,
                                            fz=fz, fz_baseline=outcome.baseline_fz,
                                            contact=outcome.contact_detected))
            prev_ee = ee.copy()
            time.sleep(dt)

    def _detections(self, run: dict) -> dict:
        """C920 (overhead, always sees socket) + D405 (eye-in-hand) markers."""
        ee = run["ee"]
        phase = run["phase"]
        hole_xy = run["hole_xy"]
        cw, ch = self._cam_size["c920"]
        dw, dh = self._cam_size["d405"]

        c920 = dict(self._c920_det)
        c920.update(present=True, img_w=cw, img_h=ch,
                    base_x=float(self.SOCKET[0]), base_y=float(self.SOCKET[1]))

        # D405 only sees the socket once the arm has descended near it
        near = phase in ("DESCEND_TO_CONTACT", "SEARCH_SPIRAL", "PUSH_INSERT", "CONFIRM")
        dxm = float(hole_xy[0] - ee[0])
        dym = float(hole_xy[1] - ee[1])
        u = dw / 2.0 + 6000.0 * dxm
        v = dh / 2.0 + 6000.0 * dym
        u = float(min(max(u, 8.0), dw - 8.0))
        v = float(min(max(v, 8.0), dh - 8.0))
        d405 = {
            "present": bool(near),
            "u": u, "v": v,
            "radius_px": float(self._d405_det.get("radius_px", 24.0)),
            "score": float(self._d405_det.get("score", 0.75)),
            "servo_dx": 0.6 * dxm, "servo_dy": 0.6 * dym,
            "img_w": dw, "img_h": dh,
        }
        return {"c920": c920, "d405": d405}

    def _snapshot(self, run, t, speed, prev_ee, events, *, fz=0.0, fz_baseline=0.0,
                  contact=False, hold=False) -> dict:
        ee = run["ee"]
        cmd = run["cmd"]
        phase = run["phase"]
        aborted = bool(run.get("error"))
        # synthesise small lateral forces during contact phases for a richer trace
        lateral = 0.0
        if phase in ("SEARCH_SPIRAL", "PUSH_INSERT"):
            lateral = 0.6 * math.sin(t * 6.0)
        fx = lateral
        fy = 0.6 * math.cos(t * 6.0) if phase == "SEARCH_SPIRAL" else 0.0
        force_mag = float(math.sqrt(fx * fx + fy * fy + fz * fz))
        return {
            "t": round(t, 4),
            "wall": now(),
            "mode": self.name,
            "phase": phase,
            "phase_label": PHASE_LABEL.get(phase, phase),
            "robot_mode": phase_to_robot_mode(phase, aborted),
            "robot_mode_label": ROBOT_MODE_LABEL.get(phase_to_robot_mode(phase, aborted), "?"),
            "ee": {"x": float(ee[0]), "y": float(ee[1]), "z": float(ee[2])},
            "cmd": {"x": float(cmd[0]), "y": float(cmd[1]), "z": float(cmd[2])},
            "socket": {"x": float(self.SOCKET[0]), "y": float(self.SOCKET[1]),
                       "z": float(self.SOCKET[2])},
            "speed": round(speed, 5),
            "speed_cap": self.V_MAX,
            "wrench": {"fx": round(fx, 3), "fy": round(fy, 3), "fz": round(fz, 3),
                       "tx": 0.0, "ty": 0.0, "tz": 0.0},
            "force_mag": round(force_mag, 3),
            "fz": round(fz, 3),
            "fz_baseline": round(float(fz_baseline), 3),
            "contact_threshold_n": self.CONTACT_FZ_N,
            "f_abort_n": self.F_ABORT_N,
            "contact": bool(contact),
            "retries": int(run.get("retries", 0)),
            "abort": aborted,
            "done": bool(run.get("done", False)),
            "error": run.get("error"),
            "detections": self._detections(run),
            "events": events,
        }
