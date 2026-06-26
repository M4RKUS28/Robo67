#!/usr/bin/env python3
"""Dedicated camera publisher node (logging path).

Only ONE process may open a V4L2 ``/dev/videoN`` at a time, so this node is the
single OWNER of one camera. It captures frames continuously and publishes them
as ``sensor_msgs/CompressedImage`` (JPEG); everything else that needs the feed
(the detector nodes, the dashboard, ``rqt_image_view``, ``ros2 bag``) SUBSCRIBES
instead of grabbing the device itself. This removes the device contention that
previously existed between the detector and the dashboard.

One node instance per camera, selected with the ``camera`` parameter:

* ``overhead`` / ``c920``  -> ``camera.c920_device``       -> ``topics.cam_overhead_raw``
* ``gripper``  / ``d405``  -> ``camera.d405_color_device`` -> ``topics.cam_gripper_raw``

Capture reuses the proven ``grab_frame_gst`` path (GStreamer CLI with a
cv2/V4L2 fallback). A background thread fills a latest-frame buffer; a timer
publishes it at ``fps`` so a slow grab never stalls the executor. Set
``image_path`` to loop a still image instead of a camera (offline / CI smoke).
"""
from __future__ import annotations

import os
import threading
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

from robo67_insertion.config_schema import load_config
from robo67_insertion.lib.image_overlay import encode_jpeg
from robo67_insertion.ros_qos import camera_qos
from robo67_insertion.nodes.socket_detector_node import device_path, grab_frame_gst


def _default_config_path() -> str:
    here = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    return os.path.join(here, "config", "robo67.yaml")


# camera -> (config device attr, config raw-topic attr, default frame_id, uses_exposure)
_CAMERAS = {
    "overhead": ("c920_device", "cam_overhead_raw", "c920_overhead", True),
    "c920": ("c920_device", "cam_overhead_raw", "c920_overhead", True),
    "gripper": ("d405_color_device", "cam_gripper_raw", "d405_color", False),
    "d405": ("d405_color_device", "cam_gripper_raw", "d405_color", False),
}


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")
        self.declare_parameter("config_path", "")
        self.declare_parameter("camera", "overhead")
        self.declare_parameter("device", "")       # override; else from config
        self.declare_parameter("topic", "")        # override; else from config
        self.declare_parameter("fps", 0.0)         # 0 -> camera.publish_fps
        self.declare_parameter("jpeg_quality", 0)  # 0 -> camera.jpeg_quality
        self.declare_parameter("exposure", -1)     # -1 -> config (overhead) / auto
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("image_path", "")   # offline: loop a still image
        self.declare_parameter("frame_id", "")

        cfg_path = self.get_parameter("config_path").value or _default_config_path()
        self.cfg = load_config(cfg_path)
        cam = str(self.get_parameter("camera").value).lower()
        if cam not in _CAMERAS:
            raise ValueError(f"unknown camera {cam!r}; expected one of {list(_CAMERAS)}")
        dev_attr, topic_attr, default_frame, uses_exposure = _CAMERAS[cam]

        self.device = self.get_parameter("device").value or getattr(self.cfg.camera, dev_attr)
        self.topic = self.get_parameter("topic").value or getattr(self.cfg.topics, topic_attr)
        self.fps = float(self.get_parameter("fps").value) or float(self.cfg.camera.publish_fps)
        self.fps = max(0.5, self.fps)
        self.quality = int(self.get_parameter("jpeg_quality").value) or int(self.cfg.camera.jpeg_quality)
        exp_param = int(self.get_parameter("exposure").value)
        if uses_exposure:
            self.exposure = self.cfg.camera.c920_exposure if exp_param < 0 else exp_param
        else:
            self.exposure = None if exp_param < 0 else exp_param
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.image_path = self.get_parameter("image_path").value or ""
        self.frame_id = self.get_parameter("frame_id").value or default_frame

        self.pub = self.create_publisher(CompressedImage, self.topic, camera_qos())

        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._frames = 0
        self._stop = threading.Event()
        self._still = None
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

        self.timer = self.create_timer(1.0 / self.fps, self._publish)
        self.get_logger().info(
            f"camera_publisher up: camera={cam} device={device_path(self.device)} "
            f"-> {self.topic} @ {self.fps} Hz (jpeg q{self.quality}, "
            f"exposure={self.exposure}, frame_id={self.frame_id})")

    # -- capture (background thread) -------------------------------------

    def _open_capture(self):
        """Open a PERSISTENT VideoCapture with a minimal driver buffer.

        ``cv2`` cannot open ``/dev/v4l/by-id/...`` symlinks in a container (no
        udev), so resolve to the real ``/dev/videoN`` first. ``BUFFERSIZE=1``
        plus a tight read loop keeps us at the head of the stream so we always
        publish the FRESHEST frame instead of draining a stale FIFO.
        """
        import cv2

        dev = device_path(self.device)
        real = os.path.realpath(dev)
        target = real if os.path.exists(real) else dev
        for backend in (getattr(cv2, "CAP_V4L2", 200), getattr(cv2, "CAP_ANY", 0)):
            cap = cv2.VideoCapture(target, backend)
            if not cap.isOpened():
                cap.release()
                continue
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if self.exposure is not None:
                # aperture-priority auto-exposure (mode 3): on THIS C920 via the
                # OpenCV V4L2 backend the MANUAL modes blow the frame out to flat
                # white (verified live 2026-06-26) -- only mode 3 gives a usable
                # frame. (The GStreamer path in grab_frame_gst CAN do manual via
                # exposure_time_absolute; cv2's CAP_PROP_EXPOSURE here cannot.)
                cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
            return cap
        return None

    def _capture_loop(self):
        import cv2

        # still-image mode: loop a single decoded image at the publish rate.
        if self.image_path:
            still = cv2.imread(self.image_path)
            period = 1.0 / self.fps
            while not self._stop.is_set():
                if still is not None:
                    jpeg = encode_jpeg(still, self.quality)
                    if jpeg is not None:
                        with self._lock:
                            self._jpeg = jpeg
                            self._frames += 1
                self._stop.wait(period)
            return

        # device mode: one persistent capture, read in a tight loop so the V4L2
        # buffer never backs up (always the newest frame). Encode is throttled
        # to the publish rate to bound CPU; reads keep running to stay fresh.
        cap = None
        enc_period = 1.0 / self.fps
        last_enc = 0.0
        while not self._stop.is_set():
            if cap is None:
                cap = self._open_capture()
                if cap is None:
                    self.get_logger().warn("camera open failed; retrying",
                                           throttle_duration_sec=5.0)
                    self._stop.wait(1.0)
                    continue
                self.get_logger().info(f"opened {device_path(self.device)}")
            ok, frame = cap.read()
            if not ok or frame is None:
                self.get_logger().warn("frame read failed; reopening",
                                       throttle_duration_sec=5.0)
                cap.release()
                cap = None
                self._stop.wait(0.2)
                continue
            now = time.time()
            if now - last_enc < enc_period:
                continue  # drained a frame to stay fresh, but don't re-encode yet
            last_enc = now
            jpeg = encode_jpeg(frame, self.quality)
            if jpeg is not None:
                with self._lock:
                    self._jpeg = jpeg
                    self._frames += 1
        if cap is not None:
            cap.release()

    # -- publish (executor thread) ---------------------------------------

    def _publish(self):
        with self._lock:
            jpeg = self._jpeg
            n = self._frames
        if jpeg is None:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = "jpeg"
        msg.data = jpeg
        self.pub.publish(msg)
        if n and n % 100 == 0:
            self.get_logger().info(f"published {n} frames", throttle_duration_sec=10.0)

    def destroy_node(self):
        self._stop.set()
        return super().destroy_node()


def main(args=None):
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
