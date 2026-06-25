"""Live ROS + camera provider for the insertion dashboard.

Runs INSIDE ``multipanda-container`` (ROS sourced, domain 1). It is a passive
observer -- it never commands the arm. It subscribes to:

* ``/franka_robot_state_broadcaster/robot_state`` (franka_msgs/FrankaState)
      EE pose (``o_t_ee``), external wrench (``o_f_ext_hat_k``), ``robot_mode``.
      EE speed is derived from successive pose samples.
* ``/robo67/socket_detection``  (std_msgs/Float64MultiArray = [u,v,r,score])  -> C920 hole
* ``/robo67/socket_pose``       (geometry_msgs/PoseStamped)                   -> socket base XY/Z
* ``/robo67/servo_correction``  (std_msgs/Float64MultiArray = [dx,dy])        -> D405 servo vector
* ``/robo67/insertion_phase``   (std_msgs/String, OPTIONAL)                   -> FSM phase
      Publish this from the orchestrator (``--publish-phase``) to see the real
      decisions; without it the phase shows UNKNOWN and we fall back to
      ``robot_mode`` for the high-level state.

Cameras are grabbed via GStreamer subprocesses (the proven path on these
cameras) into a latest-frame cache and streamed as MJPEG.

``rclpy`` is imported lazily in :meth:`start`, so this module imports fine on a
host without ROS (the server only instantiates it when ``--mode live``).
"""
from __future__ import annotations

import os
import subprocess
import tempfile
import threading
import time
from typing import Dict, List, Optional

import numpy as np

from common import (
    PHASE_LABEL,
    ROBOT_MODE_LABEL,
    Hub,
    ensure_insertion_on_path,
    now,
)

ensure_insertion_on_path()


def grab_jpeg_gst(device: int, width: int = 1280, height: int = 720,
                  timeout_s: float = 6.0) -> Optional[bytes]:
    """Grab one JPEG frame from /dev/video<device> via GStreamer; return bytes."""
    tmp = tempfile.NamedTemporaryFile(suffix=".jpg", delete=False)
    tmp.close()
    pipeline = (
        f"gst-launch-1.0 -q v4l2src device=/dev/video{device} num-buffers=1 "
        f"! image/jpeg,width={width},height={height} ! filesink location={tmp.name}"
    )
    data = None
    try:
        subprocess.run(pipeline, shell=True, timeout=timeout_s,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
        with open(tmp.name, "rb") as fh:
            data = fh.read() or None
    except Exception:
        data = None
    finally:
        try:
            os.unlink(tmp.name)
        except OSError:
            pass
    return data


def _o_t_ee_xyz(o_t_ee) -> np.ndarray:
    m = list(o_t_ee)
    return np.array([m[12], m[13], m[14]], float)


class LiveProvider:
    name = "live"

    def __init__(self, *, c920_device: Optional[int] = None,
                 d405_device: Optional[int] = None, cam_fps: float = 3.0) -> None:
        self.hub = Hub()
        self._stop = threading.Event()
        self._t0 = now()

        # config (device numbers + thresholds) from robo67.yaml when available
        self._cfg = self._load_cfg()
        self._dev = {
            "c920": c920_device if c920_device is not None
            else int(getattr(self._cfg.camera, "c920_device", 8)),
            "d405": d405_device if d405_device is not None
            else int(getattr(self._cfg.camera, "d405_color_device", 6)),
        }
        self._cam_size = {"c920": (1280, 720), "d405": (1280, 720)}
        self._cam_fps = cam_fps
        self._cam_frame: Dict[str, Optional[bytes]] = {"c920": None, "d405": None}
        self._cam_threads: List[threading.Thread] = []

        # ROS-fed state
        self._lock = threading.Lock()
        self._ee: Optional[np.ndarray] = None
        self._prev_ee: Optional[np.ndarray] = None
        self._prev_t: Optional[float] = None
        self._speed = 0.0
        self._wrench = [0.0] * 6
        self._robot_mode = 0
        self._phase = "UNKNOWN"
        self._prev_phase = "UNKNOWN"
        self._socket = None
        self._c920_det = None
        self._servo = None
        self._node = None

    def _load_cfg(self):
        try:
            from robo67_insertion.config_schema import RoboConfig, load_config

            path = os.path.join(
                os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
                "robo67_insertion", "config", "robo67.yaml")
            return load_config(path) if os.path.exists(path) else RoboConfig()
        except Exception:
            class _Empty:
                class camera:
                    c920_device = 8
                    d405_color_device = 6
            return _Empty()

    # -- lifecycle -------------------------------------------------------

    def start(self) -> None:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import Float64MultiArray, String
        from franka_msgs.msg import FrankaState

        prov = self

        class _Obs(Node):
            def __init__(self):
                super().__init__("robo67_dashboard_observer")
                self.create_subscription(FrankaState,
                                         "/franka_robot_state_broadcaster/robot_state",
                                         self._on_state, 10)
                self.create_subscription(Float64MultiArray, "/robo67/socket_detection",
                                         self._on_det, 10)
                self.create_subscription(PoseStamped, "/robo67/socket_pose",
                                         self._on_socket, 10)
                self.create_subscription(Float64MultiArray, "/robo67/servo_correction",
                                         self._on_servo, 10)
                self.create_subscription(String, "/robo67/insertion_phase",
                                         self._on_phase, 10)
                self.create_timer(1.0 / 30.0, self._publish)

            def _on_state(self, msg):
                ee = _o_t_ee_xyz(msg.o_t_ee)
                t = time.time()
                with prov._lock:
                    if prov._prev_ee is not None and prov._prev_t is not None:
                        dt = max(1e-3, t - prov._prev_t)
                        prov._speed = float(np.linalg.norm(ee - prov._prev_ee) / dt)
                    prov._prev_ee = ee.copy()
                    prov._prev_t = t
                    prov._ee = ee
                    prov._wrench = list(msg.o_f_ext_hat_k)
                    prov._robot_mode = int(getattr(msg, "robot_mode", 0))

            def _on_det(self, msg):
                d = list(msg.data)
                if len(d) >= 4:
                    with prov._lock:
                        prov._c920_det = d[:4]

            def _on_socket(self, msg):
                with prov._lock:
                    prov._socket = [msg.pose.position.x, msg.pose.position.y,
                                    msg.pose.position.z]

            def _on_servo(self, msg):
                d = list(msg.data)
                if len(d) >= 2:
                    with prov._lock:
                        prov._servo = d[:2]

            def _on_phase(self, msg):
                with prov._lock:
                    prov._phase = str(msg.data).strip().upper() or "UNKNOWN"

            def _publish(self):
                prov.hub.publish(prov._snapshot())

        rclpy.init()
        self._node = _Obs()

        def _spin():
            try:
                rclpy.spin(self._node)
            except Exception:
                pass

        threading.Thread(target=_spin, name="ros-spin", daemon=True).start()

        for cam in ("c920", "d405"):
            th = threading.Thread(target=self._cam_loop, args=(cam,),
                                  name=f"grab-{cam}", daemon=True)
            th.start()
            self._cam_threads.append(th)

    def stop(self) -> None:
        self._stop.set()
        try:
            import rclpy

            if self._node is not None:
                self._node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    # -- metadata --------------------------------------------------------

    def health(self) -> dict:
        with self._lock:
            return {
                "mode": self.name,
                "ros": self._ee is not None,
                "cameras": {n: (self._cam_frame[n] is not None) for n in self._cam_frame},
                "phase_topic": self._phase != "UNKNOWN",
                "devices": dict(self._dev),
            }

    def config(self) -> dict:
        from common import PHASE_ORDER

        ct = float(getattr(self._cfg.insertion, "contact_fz_threshold_n", 5.0)) \
            if hasattr(self._cfg, "insertion") else 5.0
        fa = float(getattr(self._cfg.safety, "fz_abort_n", 25.0)) \
            if hasattr(self._cfg, "safety") else 25.0
        return {
            "mode": self.name,
            "phases": [{"id": p, "label": PHASE_LABEL[p]} for p in PHASE_ORDER],
            "robot_modes": ROBOT_MODE_LABEL,
            "thresholds": {"contact_fz_n": ct, "f_abort_n": fa,
                           "speed_cap_mps": 0.05, "insert_depth_m": 0.04},
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

    def _cam_loop(self, name: str) -> None:
        period = 1.0 / max(0.5, self._cam_fps)
        dev = self._dev[name]
        while not self._stop.is_set():
            frame = grab_jpeg_gst(dev)
            if frame is not None:
                self._cam_frame[name] = frame
            time.sleep(period)

    def camera_names(self) -> List[str]:
        return ["c920", "d405"]

    def camera_jpeg(self, name: str) -> Optional[bytes]:
        return self._cam_frame.get(name)

    def camera_iter(self, name: str, fps: float = 8.0):
        period = 1.0 / max(1.0, fps)
        while not self._stop.is_set():
            frame = self._cam_frame.get(name)
            if frame is not None:
                yield frame
            time.sleep(period)

    # -- telemetry snapshot ---------------------------------------------

    def _snapshot(self) -> dict:
        with self._lock:
            ee = self._ee
            wrench = list(self._wrench)
            speed = self._speed
            robot_mode = self._robot_mode
            phase = self._phase
            socket = self._socket
            c920_det = self._c920_det
            servo = self._servo
            prev_phase = self._prev_phase
            self._prev_phase = phase

        t = now() - self._t0
        fx, fy, fz = (wrench + [0, 0, 0])[:3]
        force_mag = float(np.sqrt(fx * fx + fy * fy + fz * fz))
        events = []
        if phase != prev_phase and phase != "UNKNOWN":
            events.append({"t": t, "kind": "transition", "from": prev_phase, "to": phase,
                           "msg": f"{prev_phase} -> {phase}"})

        cw, ch = self._cam_size["c920"]
        dw, dh = self._cam_size["d405"]
        c920 = {"present": False, "img_w": cw, "img_h": ch}
        if c920_det is not None:
            c920 = {"present": True, "u": c920_det[0], "v": c920_det[1],
                    "radius_px": c920_det[2], "score": c920_det[3],
                    "img_w": cw, "img_h": ch,
                    "base_x": (socket[0] if socket else None),
                    "base_y": (socket[1] if socket else None)}
        d405 = {"present": False, "img_w": dw, "img_h": dh}
        if servo is not None:
            d405 = {"present": True, "u": dw / 2.0, "v": dh / 2.0,
                    "radius_px": 24.0, "score": None,
                    "servo_dx": servo[0], "servo_dy": servo[1],
                    "img_w": dw, "img_h": dh}

        return {
            "t": round(t, 4), "wall": now(), "mode": self.name,
            "phase": phase,
            "phase_label": PHASE_LABEL.get(phase, phase.title() if phase != "UNKNOWN" else "Unknown"),
            "robot_mode": robot_mode,
            "robot_mode_label": ROBOT_MODE_LABEL.get(robot_mode, "?"),
            "ee": ({"x": float(ee[0]), "y": float(ee[1]), "z": float(ee[2])}
                   if ee is not None else None),
            "cmd": None,
            "socket": ({"x": socket[0], "y": socket[1], "z": socket[2]} if socket else None),
            "speed": round(speed, 5),
            "speed_cap": 0.05,
            "wrench": {"fx": round(fx, 3), "fy": round(fy, 3), "fz": round(fz, 3),
                       "tx": round(wrench[3], 3) if len(wrench) > 3 else 0.0,
                       "ty": round(wrench[4], 3) if len(wrench) > 4 else 0.0,
                       "tz": round(wrench[5], 3) if len(wrench) > 5 else 0.0},
            "force_mag": round(force_mag, 3),
            "fz": round(fz, 3), "fz_baseline": 0.0,
            "contact_threshold_n": 5.0, "f_abort_n": 25.0,
            "contact": False, "retries": 0,
            "abort": robot_mode == 4, "done": phase == "DONE",
            "error": None,
            "detections": {"c920": c920, "d405": d405},
            "events": events,
        }
