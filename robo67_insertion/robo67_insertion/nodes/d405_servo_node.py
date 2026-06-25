#!/usr/bin/env python3
"""D405 eye-in-hand visual-servoing node (milestone B+ refinement).

When the EE is roughly above the socket (from the C920 coarse pose), this node
detects the hole in the D405 image, computes a base-frame XY correction with
:func:`~robo67_insertion.lib.servoing.ibvs_correction` (tool is vertical, so the
D405 image axes map to base XY via depth + intrinsics), and publishes the
correction on ``/robo67/servo_correction`` (std_msgs/Float64MultiArray = [dx, dy]).
The orchestrator can add this to the socket XY before descending.

Depth: with ``pyrealsense2`` the hole depth gives socket-top Z directly; without
it, fall back to a configured standoff. NOTE: the container currently lacks D405
passthrough + a working cv2 (see PHASE0_VERIFIED.md) — this node targets the
hardware bring-up where the D405 is accessible. The control law is unit-tested in
``test/test_servoing.py``.
"""
from __future__ import annotations

import os

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from robo67_insertion.config_schema import load_config
from robo67_insertion.lib.hole_detect import HoleParams, detect_holes
from robo67_insertion.lib.servoing import ibvs_correction
from robo67_insertion.nodes.socket_detector_node import grab_frame_gst


def _default_config_path() -> str:
    here = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    return os.path.join(here, "config", "robo67.yaml")


class D405Servo(Node):
    def __init__(self):
        super().__init__("d405_servo")
        self.declare_parameter("config_path", "")
        self.declare_parameter("depth_m", 0.10)   # standoff fallback if no pyrealsense2
        self.declare_parameter("gain", 0.6)
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("image_path", "")

        cfg_path = self.get_parameter("config_path").value or _default_config_path()
        self.cfg = load_config(cfg_path)
        self.device = int(self.cfg.camera.d405_color_device)
        self.depth_m = float(self.get_parameter("depth_m").value)
        self.gain = float(self.get_parameter("gain").value)
        self.image_path = self.get_parameter("image_path").value or ""
        self.params = HoleParams()

        self.pub = self.create_publisher(Float64MultiArray, "/robo67/servo_correction", 10)
        rate = max(0.2, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f"d405_servo up (device=/dev/video{self.device}, {rate} Hz)")

    def _grab(self):
        import cv2

        if self.image_path:
            return cv2.imread(self.image_path)
        return grab_frame_gst(self.device)

    def _tick(self):
        img = self._grab()
        if img is None:
            self.get_logger().warn("D405 frame grab failed", throttle_duration_sec=5.0)
            return
        h, w = img.shape[:2]
        holes = detect_holes(img, self.params)
        if not holes:
            return
        hole = holes[0]
        dx, dy = ibvs_correction(
            (hole.u, hole.v), (w / 2.0, h / 2.0), self.depth_m,
            self.cfg.camera.d405_fx, self.cfg.camera.d405_fy, self.gain,
        )
        msg = Float64MultiArray()
        msg.data = [float(dx), float(dy)]
        self.pub.publish(msg)


def main(args=None):
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = D405Servo()
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
