#!/usr/bin/env python3
"""Socket detector node.

Grabs overhead C920 frames, detects the dark circular hole
(:func:`~robo67_insertion.lib.hole_detect.detect_holes`), maps the best hole's
pixel to robot-base XY through the pixel-to-base mapping seam
(:class:`~robo67_insertion.lib.pixel_mapping.HomographyMappingAdapter`, which
composes the calibrated homography), and publishes:

* ``/robo67/socket_pose``      (geometry_msgs/PoseStamped, frame ``panda_link0``)
* ``/robo67/socket_detection`` (std_msgs/Float64MultiArray = [u, v, radius_px, score])

Frame grabbing uses a GStreamer subprocess (cv2.VideoCapture raw V4L2 times out
on these cameras; GStreamer is the reliable path). Detection runs at a low rate
(socket is static); the orchestrator only needs an occasional fresh pose.

The socket *top* Z cannot come from a single overhead camera, so the published
pose Z is a configured constant (``socket_top_z`` param, set after calibration);
the orchestrator finds the true contact Z by force-probing regardless.
"""
from __future__ import annotations

import os
import subprocess
import tempfile

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

from robo67_insertion.config_schema import load_config
from robo67_insertion.lib.hole_detect import HoleParams, detect_holes
from robo67_insertion.lib.pixel_mapping import (
    HomographyMappingAdapter,
    MappingContext,
    PixelObservation,
)


def _default_config_path() -> str:
    here = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    return os.path.join(here, "config", "robo67.yaml")


def grab_frame_gst(device: int, width: int = 1280, height: int = 720, timeout_s: float = 8.0):
    """Grab a single frame from /dev/video<device> via GStreamer; return a BGR ndarray or None."""
    import cv2

    tmp = tempfile.NamedTemporaryFile(suffix=".jpg", delete=False)
    tmp.close()
    pipeline = (
        f"gst-launch-1.0 -q v4l2src device=/dev/video{device} num-buffers=1 "
        f"! image/jpeg,width={width},height={height} ! jpegdec ! videoconvert "
        f"! jpegenc ! filesink location={tmp.name}"
    )
    try:
        subprocess.run(pipeline, shell=True, timeout=timeout_s,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
        img = cv2.imread(tmp.name)
    except Exception:
        img = None
    finally:
        try:
            os.unlink(tmp.name)
        except OSError:
            pass
    return img


class SocketDetector(Node):
    def __init__(self):
        super().__init__("socket_detector")
        self.declare_parameter("config_path", "")
        self.declare_parameter("homography_path", "")
        self.declare_parameter("socket_top_z", 0.0)
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("image_path", "")  # offline: detect on a still image instead of camera

        cfg_path = self.get_parameter("config_path").value or _default_config_path()
        self.cfg = load_config(cfg_path)
        self.device = int(self.cfg.camera.c920_device)
        self.socket_top_z = float(self.get_parameter("socket_top_z").value)
        self.image_path = self.get_parameter("image_path").value or ""

        hpath = self.get_parameter("homography_path").value or os.path.join(
            os.path.dirname(cfg_path), "c920_homography.npz"
        )
        self.H = None
        self.mapper = None
        if os.path.exists(hpath):
            data = np.load(hpath)
            self.H = data["H"]
            self.mapper = HomographyMappingAdapter(self.H)
            self.get_logger().info(f"loaded homography from {hpath}")
        else:
            self.get_logger().warn(
                f"no homography at {hpath}; publishing detections only (no base-frame pose)"
            )

        self.params = HoleParams()
        self.pub_pose = self.create_publisher(PoseStamped, self.cfg.topics.socket_pose, 10)
        self.pub_det = self.create_publisher(Float64MultiArray, self.cfg.topics.socket_detection, 10)

        rate = max(0.2, float(self.get_parameter("rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f"socket_detector up (device=/dev/video{self.device}, {rate} Hz)")

    def _grab(self):
        import cv2

        if self.image_path:
            return cv2.imread(self.image_path)
        return grab_frame_gst(self.device)

    def _tick(self):
        img = self._grab()
        if img is None:
            self.get_logger().warn("frame grab failed", throttle_duration_sec=5.0)
            return
        holes = detect_holes(img, self.params)
        if not holes:
            return
        h = holes[0]
        det = Float64MultiArray()
        det.data = [float(h.u), float(h.v), float(h.radius_px), float(h.score)]
        self.pub_det.publish(det)

        if self.mapper is not None:
            base_xy = self.mapper.map_xy(PixelObservation(h.u, h.v), MappingContext())
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "panda_link0"
            msg.pose.position.x = float(base_xy[0])
            msg.pose.position.y = float(base_xy[1])
            msg.pose.position.z = self.socket_top_z
            msg.pose.orientation.w = 1.0  # vertical (identity)
            self.pub_pose.publish(msg)


def main(args=None):
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = SocketDetector()
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
