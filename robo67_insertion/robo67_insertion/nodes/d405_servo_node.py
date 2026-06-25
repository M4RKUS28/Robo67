#!/usr/bin/env python3
"""D405 eye-in-hand visual-servoing node (milestone B+ refinement).

When the EE is roughly above the socket (from the C920 coarse pose), this node
detects the hole in the D405 image and computes a base-frame XY correction
through the pixel-to-base mapping seam
(:class:`~robo67_insertion.lib.pixel_mapping.PinholeMappingAdapter`, which
composes the pinhole IBVS model; tool is vertical, so the D405 image axes map to
base XY via depth + intrinsics). The seam returns the UNSCALED correction
(gain = 1.0); this node applies the proportional servo gain and publishes the
correction on ``/robo67/servo_correction`` (std_msgs/Float64MultiArray = [dx, dy]).
The orchestrator can add this to the socket XY before descending.

Depth: with ``pyrealsense2`` the hole depth gives socket-top Z directly; without
it, fall back to a configured standoff. NOTE: the container currently lacks D405
passthrough + a working cv2 (see PHASE0_VERIFIED.md) — this node targets the
hardware bring-up where the D405 is accessible. The control law is unit-tested in
``test/test_servoing.py`` and the mapping seam in ``test/test_pixel_mapping.py``.
"""
from __future__ import annotations

import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from robo67_insertion.config_schema import load_config
from robo67_insertion.lib.hole_detect import HoleParams, detect_holes
from robo67_insertion.lib.pixel_mapping import (
    MappingContext,
    PinholeMappingAdapter,
    PixelObservation,
)
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
        self.mapper = PinholeMappingAdapter()

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
        # Seam returns the UNSCALED base-frame XY correction (gain = 1.0); the
        # proportional servo gain is applied HERE, in the node, never in the seam.
        dx0, dy0 = self.mapper.map_xy(
            PixelObservation(hole.u, hole.v),
            MappingContext(
                depth_m=self.depth_m,
                fx=self.cfg.camera.d405_fx,
                fy=self.cfg.camera.d405_fy,
                center_u=w / 2.0,
                center_v=h / 2.0,
            ),
        )
        dx, dy = self.gain * dx0, self.gain * dy0
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
