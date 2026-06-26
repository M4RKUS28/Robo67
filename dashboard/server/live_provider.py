"""Live ROS provider for the insertion dashboard (topic-sourced).

Runs INSIDE ``multipanda-container`` (ROS sourced, domain 1). It is a passive
observer -- it never commands the arm, and (unlike before) it never opens a
camera device. Everything is read from rostopics so the dashboard is just one
more subscriber on the logging graph (no device contention with the detector):

Cameras (sensor_msgs/CompressedImage, "jpeg") -- published by camera_publisher
and the detector overlay feeds:
* ``topics.cam_overhead_raw``     -> dashboard feed ``c920``
* ``topics.cam_overhead_overlay`` -> dashboard feed ``c920_overlay``
* ``topics.cam_gripper_raw``      -> dashboard feed ``d405``
* ``topics.cam_gripper_overlay``  -> dashboard feed ``d405_overlay``

Robot state (always-on):
* ``topics.robot_state`` (franka_msgs/FrankaState) -> EE pose, wrench, robot_mode;
  EE speed is derived from successive poses.

Insertion telemetry (published by hardware_insertion_node during a run):
* ``topics.insertion_phase``         (std_msgs/String)   -> FSM phase
* ``topics.insertion_command_pose``  (geometry_msgs/PoseStamped) -> commanded equilibrium
* ``topics.insertion_fz_baseline``   (std_msgs/Float64)  -> free-space Fz baseline
* ``topics.insertion_contact``       (std_msgs/Bool)     -> contact detected
* ``topics.insertion_retries``       (std_msgs/Int32)    -> retry count

Detections (markers for the client-side overlay on the raw feed):
* ``topics.socket_detection`` ([u,v,r,score]) + ``topics.socket_pose`` (base XY/Z)
* ``topics.servo_correction`` ([dx,dy])  -> D405 servo vector

``rclpy`` is imported lazily in :meth:`start`, so this module imports fine on a
host without ROS (the server only instantiates it when ``--mode live``).
"""
from __future__ import annotations

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


# dashboard camera feed name -> (config topic attribute, fallback topic name).
# The fallback is used only when robo67.yaml/config_schema is unavailable.
CAM_FEEDS = {
    "c920": ("cam_overhead_raw", "/robo67/camera/overhead/image_raw/compressed"),
    "c920_overlay": ("cam_overhead_overlay", "/robo67/camera/overhead/overlay/compressed"),
    "d405": ("cam_gripper_raw", "/robo67/camera/gripper/image_raw/compressed"),
    "d405_overlay": ("cam_gripper_overlay", "/robo67/camera/gripper/overlay/compressed"),
}


def _o_t_ee_xyz(o_t_ee) -> np.ndarray:
    m = list(o_t_ee)
    return np.array([m[12], m[13], m[14]], float)


class LiveProvider:
    name = "live"

    def __init__(self, *, c920_device: Optional[int] = None,
                 d405_device: Optional[int] = None, cam_fps: float = 8.0) -> None:
        self.hub = Hub()
        self._stop = threading.Event()
        self._t0 = now()

        # config: topic names + thresholds from robo67.yaml when available.
        # (c920_device / d405_device are accepted for CLI compatibility but no
        # longer used -- the dashboard subscribes to camera topics now.)
        self._cfg = self._load_cfg()
        self._topics = getattr(self._cfg, "topics", None)
        self._cam_fps = cam_fps
        self._cam_size = {"c920": (1280, 720), "c920_overlay": (1280, 720),
                          "d405": (1280, 720), "d405_overlay": (1280, 720)}
        self._cam_jpeg: Dict[str, Optional[bytes]] = {n: None for n in CAM_FEEDS}

        # ROS-fed state
        self._lock = threading.Lock()
        self._ee: Optional[np.ndarray] = None
        self._prev_ee: Optional[np.ndarray] = None
        self._prev_t: Optional[float] = None
        self._speed = 0.0
        self._wrench = [0.0] * 6
        self._robot_mode = 0
        # wall-clock of the last FrankaState message -- lets the bringup
        # relaunch detect when a freshly-launched bringup starts publishing
        # again (the subscription auto-rediscovers the new publisher).
        self._last_state_wall = 0.0
        self._phase = "UNKNOWN"
        self._prev_phase = "UNKNOWN"
        self._cmd = None
        self._fz_baseline = 0.0
        self._contact = False
        self._retries = 0
        self._socket = None
        self._c920_det = None
        self._servo = None
        self._node = None

    def _load_cfg(self):
        import os

        try:
            from robo67_insertion.config_schema import RoboConfig, load_config

            path = os.path.join(
                os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
                "robo67_insertion", "config", "robo67.yaml")
            return load_config(path) if os.path.exists(path) else RoboConfig()
        except Exception:
            return None

    def _t(self, attr: str, default: str) -> str:
        """Topic name from config (yaml override) with a hard-coded fallback."""
        return getattr(self._topics, attr, default) if self._topics else default

    # -- lifecycle -------------------------------------------------------

    def start(self) -> None:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import Bool, Float64, Float64MultiArray, Int32, String
        from franka_msgs.msg import FrankaState

        from robo67_insertion.ros_qos import camera_qos

        prov = self

        class _Obs(Node):
            def __init__(self):
                super().__init__("robo67_dashboard_observer")
                # robot state (always-on)
                self.create_subscription(
                    FrankaState, prov._t("robot_state", "/franka_robot_state_broadcaster/robot_state"),
                    self._on_state, 10)
                # detections (client-side overlay markers)
                self.create_subscription(
                    Float64MultiArray, prov._t("socket_detection", "/robo67/socket_detection"),
                    self._on_det, 10)
                self.create_subscription(
                    PoseStamped, prov._t("socket_pose", "/robo67/socket_pose"),
                    self._on_socket, 10)
                self.create_subscription(
                    Float64MultiArray, prov._t("servo_correction", "/robo67/servo_correction"),
                    self._on_servo, 10)
                # insertion telemetry
                self.create_subscription(
                    String, prov._t("insertion_phase", "/robo67/insertion/phase"),
                    self._on_phase, 10)
                self.create_subscription(
                    PoseStamped, prov._t("insertion_command_pose", "/robo67/insertion/command_pose"),
                    self._on_cmd, 10)
                self.create_subscription(
                    Float64, prov._t("insertion_fz_baseline", "/robo67/insertion/fz_baseline"),
                    self._on_baseline, 10)
                self.create_subscription(
                    Bool, prov._t("insertion_contact", "/robo67/insertion/contact"),
                    self._on_contact, 10)
                self.create_subscription(
                    Int32, prov._t("insertion_retries", "/robo67/insertion/retries"),
                    self._on_retries, 10)
                # camera feeds (raw + overlay)
                for feed, (attr, default) in CAM_FEEDS.items():
                    topic = prov._t(attr, default)
                    # must match the publisher's BEST_EFFORT/depth-1 QoS, else
                    # no frames flow (incompatible QoS) -- and depth 1 keeps the
                    # dashboard on the freshest frame, never a stale backlog.
                    self.create_subscription(
                        CompressedImage, topic,
                        lambda msg, f=feed: self._on_image(f, msg), camera_qos())
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
                    prov._last_state_wall = time.time()

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

            def _on_cmd(self, msg):
                with prov._lock:
                    prov._cmd = [msg.pose.position.x, msg.pose.position.y,
                                 msg.pose.position.z]

            def _on_baseline(self, msg):
                with prov._lock:
                    prov._fz_baseline = float(msg.data)

            def _on_contact(self, msg):
                with prov._lock:
                    prov._contact = bool(msg.data)

            def _on_retries(self, msg):
                with prov._lock:
                    prov._retries = int(msg.data)

            def _on_image(self, feed, msg):
                prov._cam_jpeg[feed] = bytes(msg.data)

            def _publish(self):
                prov.hub.publish(prov._snapshot())

        rclpy.init()
        self._node = _Obs()

        def _spin():
            # Keep spinning even if a single callback raises: rclpy's default
            # executor propagates a callback exception out of spin() and would
            # otherwise leave the node permanently deaf (no camera/telemetry
            # callbacks ever fire again). Log it and continue instead.
            import traceback

            while rclpy.ok() and not self._stop.is_set():
                try:
                    rclpy.spin_once(self._node, timeout_sec=0.1)
                except Exception:
                    print("[live_provider] callback error (continuing):",
                          flush=True)
                    traceback.print_exc()

        threading.Thread(target=_spin, name="ros-spin", daemon=True).start()

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
                "cameras": {n: (self._cam_jpeg[n] is not None) for n in self._cam_jpeg},
                "phase_topic": self._phase != "UNKNOWN",
                "telemetry": self._cmd is not None,
            }

    def config(self) -> dict:
        from common import PHASE_ORDER

        ct = float(getattr(self._cfg.insertion, "contact_fz_threshold_n", 5.0)) \
            if getattr(self._cfg, "insertion", None) else 5.0
        fa = float(getattr(self._cfg.safety, "fz_abort_n", 25.0)) \
            if getattr(self._cfg, "safety", None) else 25.0
        return {
            "mode": self.name,
            "phases": [{"id": p, "label": PHASE_LABEL[p]} for p in PHASE_ORDER],
            "robot_modes": ROBOT_MODE_LABEL,
            "thresholds": {"contact_fz_n": ct, "f_abort_n": fa,
                           "speed_cap_mps": 0.05, "insert_depth_m": 0.04},
            "cameras": {
                "c920": {"label": "C920 overhead", "size": self._cam_size["c920"],
                          "kind": "static-overhead", "overlay": "c920_overlay"},
                "d405": {"label": "D405 eye-in-hand", "size": self._cam_size["d405"],
                          "kind": "eye-in-hand", "overlay": "d405_overlay"},
            },
        }

    def latest(self) -> Optional[dict]:
        return self.hub.latest()

    # -- robot-state accessors (used by the bringup relaunch verification) --

    def robot_mode(self) -> int:
        """Latest Franka robot_mode (1=Idle 2=Move 4=Reflex 5=UserStopped)."""
        with self._lock:
            return self._robot_mode

    def last_state_wall(self) -> float:
        """Wall-clock of the last FrankaState message (0.0 if never seen)."""
        with self._lock:
            return self._last_state_wall

    # -- cameras (topic-sourced) ----------------------------------------

    def camera_names(self) -> List[str]:
        return list(CAM_FEEDS.keys())

    def camera_jpeg(self, name: str) -> Optional[bytes]:
        return self._cam_jpeg.get(name)

    def camera_iter(self, name: str, fps: float = 30.0):
        # Poll fast, but only emit a frame when it actually CHANGES (each ROS
        # message is a fresh bytes object, so identity comparison works). Sending
        # duplicate JPEGs just builds a downstream TCP/browser buffer and adds
        # latency; emitting only new frames keeps the stream live and current.
        period = 1.0 / max(1.0, fps)
        last = None
        while not self._stop.is_set():
            frame = self._cam_jpeg.get(name)
            if frame is not None and frame is not last:
                last = frame
                yield frame
            time.sleep(period)

    # -- telemetry snapshot ---------------------------------------------

    def _snapshot(self) -> dict:
        with self._lock:
            ee = self._ee
            cmd = self._cmd
            wrench = list(self._wrench)
            speed = self._speed
            robot_mode = self._robot_mode
            phase = self._phase
            fz_baseline = self._fz_baseline
            contact = self._contact
            retries = self._retries
            socket = self._socket
            c920_det = self._c920_det
            servo = self._servo
            prev_phase = self._prev_phase
            self._prev_phase = phase

        ct = float(getattr(self._cfg.insertion, "contact_fz_threshold_n", 5.0)) \
            if getattr(self._cfg, "insertion", None) else 5.0
        fa = float(getattr(self._cfg.safety, "fz_abort_n", 25.0)) \
            if getattr(self._cfg, "safety", None) else 25.0

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
            "cmd": ({"x": float(cmd[0]), "y": float(cmd[1]), "z": float(cmd[2])}
                    if cmd is not None else None),
            "socket": ({"x": socket[0], "y": socket[1], "z": socket[2]} if socket else None),
            "speed": round(speed, 5),
            "speed_cap": 0.05,
            "wrench": {"fx": round(fx, 3), "fy": round(fy, 3), "fz": round(fz, 3),
                       "tx": round(wrench[3], 3) if len(wrench) > 3 else 0.0,
                       "ty": round(wrench[4], 3) if len(wrench) > 4 else 0.0,
                       "tz": round(wrench[5], 3) if len(wrench) > 5 else 0.0},
            "force_mag": round(force_mag, 3),
            "fz": round(fz, 3), "fz_baseline": round(fz_baseline, 3),
            "contact_threshold_n": ct, "f_abort_n": fa,
            "contact": bool(contact), "retries": int(retries),
            "abort": robot_mode == 4 or phase == "ERROR", "done": phase == "DONE",
            "error": None,
            "detections": {"c920": c920, "d405": d405},
            "events": events,
        }
