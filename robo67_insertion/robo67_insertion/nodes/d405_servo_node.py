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
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64MultiArray

from robo67_insertion.config_schema import load_config
from robo67_insertion.lib.hole_detect import HoleParams, detect_holes
from robo67_insertion.lib.image_overlay import decode_jpeg, draw_servo_overlay, encode_jpeg
from robo67_insertion.lib.pixel_mapping import (
    MappingContext,
    PinholeMappingAdapter,
    PixelObservation,
)
from robo67_insertion.nodes.socket_detector_node import device_path, grab_frame_gst


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
        # frame source: "topic" (subscribe to camera_publisher), "device", "image".
        self.declare_parameter("source", "topic")
        self.declare_parameter("image_topic", "")    # "" -> topics.cam_gripper_raw
        self.declare_parameter("overlay_topic", "")  # "" -> topics.cam_gripper_overlay

        cfg_path = self.get_parameter("config_path").value or _default_config_path()
        self.cfg = load_config(cfg_path)
        # Keep the raw device value (bare index OR /dev/v4l/by-id/... path);
        # device_path() in grab_frame_gst() resolves either form. Only used in
        # source="device"/"image"; in topic mode the camera_publisher owns it.
        self.device = self.cfg.camera.d405_color_device
        self.depth_m = float(self.get_parameter("depth_m").value)
        self.gain = float(self.get_parameter("gain").value)
        self.jpeg_quality = int(self.cfg.camera.jpeg_quality)
        self.image_path = self.get_parameter("image_path").value or ""
        self.source = str(self.get_parameter("source").value).lower()
        if self.image_path and self.source == "topic":
            self.source = "image"
        self.image_topic = (self.get_parameter("image_topic").value
                            or self.cfg.topics.cam_gripper_raw)
        self.overlay_topic = (self.get_parameter("overlay_topic").value
                              or self.cfg.topics.cam_gripper_overlay)
        self.params = HoleParams()
        self.mapper = PinholeMappingAdapter()
        self._last_proc = 0.0

        self.pub = self.create_publisher(
            Float64MultiArray, self.cfg.topics.servo_correction, 10)
        self.pub_overlay = self.create_publisher(CompressedImage, self.overlay_topic, 5)
        self.rate = max(0.2, float(self.get_parameter("rate_hz").value))
        if self.source == "topic":
            self.sub_img = self.create_subscription(
                CompressedImage, self.image_topic, self._on_image, 5)
            src = f"topic {self.image_topic}"
        else:
            self.create_timer(1.0 / self.rate, self._tick)
            src = f"device {device_path(self.device)}" if self.source == "device" \
                else f"image {self.image_path}"
        self.get_logger().info(
            f"d405_servo up (source={self.source} [{src}], {self.rate} Hz) "
            f"-> {self.cfg.topics.servo_correction}, overlay {self.overlay_topic}")

    def _grab(self):
        import cv2

        if self.source == "image":
            return cv2.imread(self.image_path)
        return grab_frame_gst(self.device)

    # -- frame handlers --------------------------------------------------

    def _tick(self):
        img = self._grab()
        if img is None:
            self.get_logger().warn("D405 frame grab failed", throttle_duration_sec=5.0)
            return
        self._process(img)

    def _on_image(self, msg: CompressedImage):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_proc < 1.0 / self.rate:
            return
        self._last_proc = now
        img = decode_jpeg(bytes(msg.data))
        if img is None:
            self.get_logger().warn("could not decode subscribed frame",
                                   throttle_duration_sec=5.0)
            return
        self._process(img)

    def _process(self, img):
        h, w = img.shape[:2]
        holes = detect_holes(img, self.params)
        servo_dxy = None
        if holes:
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
            servo_dxy = (self.gain * dx0, self.gain * dy0)
            msg = Float64MultiArray()
            msg.data = [float(servo_dxy[0]), float(servo_dxy[1])]
            self.pub.publish(msg)

        # always publish the overlay feed (annotated when locked, centre marker otherwise)
        overlay = draw_servo_overlay(img, holes, servo_dxy_m=servo_dxy)
        jpeg = encode_jpeg(overlay, self.jpeg_quality)
        if jpeg is not None:
            out = CompressedImage()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = "d405_color"
            out.format = "jpeg"
            out.data = jpeg
            self.pub_overlay.publish(out)


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
