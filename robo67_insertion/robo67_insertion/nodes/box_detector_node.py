#!/usr/bin/env python3
"""Box detector node (cable-insertion overhead vision).

Grabs overhead C920 frames, detects the dark, port-covered industrial I/O box
(:func:`~robo67_insertion.lib.box_detect.detect_gray_box`, a local-texture-energy
detector), maps the best box's centroid pixel to robot-base XY through the same
calibrated homography the peg-in-hole socket uses
(:class:`~robo67_insertion.lib.pixel_mapping.HomographyMappingAdapter`), and
publishes:

* ``topics.box_pose``      (geometry_msgs/PoseStamped, frame ``panda_link0``)
* ``topics.box_detection`` (std_msgs/Float64MultiArray = [u, v, w_px, h_px, angle_deg, score])
* the OVERLAY feed (sensor_msgs/CompressedImage): the frame annotated with the
  detected box quad + centroid + base-XY label. Defaults to
  ``topics.cam_overhead_overlay`` so the existing dashboard "Processed" toggle
  shows it with zero dashboard changes (run this INSTEAD of ``socket_detector``
  for the cable task -- they must not share an overlay topic).

This is the cable-task twin of ``socket_detector_node`` and reuses the exact
same frame-source machinery (``source`` = ``topic`` | ``device`` | ``image``),
camera-publisher subscription, throttling, and homography loading.

The box TOP Z cannot come from a single overhead camera, so the published pose Z
is a configured constant (``box_top_z`` param); the wrist D405 + insertion
force-probe resolve the true contact Z later.
"""
from __future__ import annotations

import os

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64MultiArray

from robo67_insertion.config_schema import load_config
from robo67_insertion.lib.box_detect import (
    BoxOrbParams,
    BoxParams,
    OrbBoxMatcher,
    detect_gray_box,
)
from robo67_insertion.lib.image_overlay import decode_jpeg, draw_box_overlay, encode_jpeg
from robo67_insertion.lib.pixel_mapping import (
    HomographyMappingAdapter,
    MappingContext,
    PixelObservation,
)
from robo67_insertion.nodes.socket_detector_node import device_path, grab_frame_gst
from robo67_insertion.ros_qos import camera_qos


def _default_config_path() -> str:
    here = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    return os.path.join(here, "config", "robo67.yaml")


class BoxDetector(Node):
    def __init__(self):
        super().__init__("box_detector")
        self.declare_parameter("config_path", "")
        self.declare_parameter("homography_path", "")
        self.declare_parameter("box_top_z", 0.0)
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("image_path", "")  # offline: detect on a still image
        # frame source: "topic" (subscribe to camera_publisher), "device", "image".
        self.declare_parameter("source", "topic")
        self.declare_parameter("image_topic", "")    # "" -> topics.cam_overhead_raw
        self.declare_parameter("overlay_topic", "")  # "" -> topics.cam_overhead_overlay
        self.declare_parameter("min_texture_std", BoxParams.min_texture_std)
        # detection method: "orb" (object-specific template match; robust to
        # clutter -- DEFAULT) or "texture" (busiest-blob heuristic). The texture
        # fallback is OFF by default: it is object-agnostic and will confidently
        # label the busiest distractor (a white box, a phone) as the I/O box on
        # a frame where the real box is absent. For insertion a wrong detection
        # is worse than no detection, so prefer ORB-only (opt back in with
        # fallback_texture:=true if you knowingly want the heuristic).
        self.declare_parameter("method", "orb")
        self.declare_parameter("template_path", "")  # "" -> config/box_template.jpg
        self.declare_parameter("fallback_texture", False)
        self.declare_parameter("orb_min_inliers", BoxOrbParams.min_inliers)

        cfg_path = self.get_parameter("config_path").value or _default_config_path()
        self.cfg = load_config(cfg_path)
        self.device = self.cfg.camera.c920_device
        self.exposure = self.cfg.camera.c920_exposure
        self.jpeg_quality = int(self.cfg.camera.jpeg_quality)
        self.box_top_z = float(self.get_parameter("box_top_z").value)
        self.image_path = self.get_parameter("image_path").value or ""
        self.source = str(self.get_parameter("source").value).lower()
        if self.image_path and self.source == "topic":
            self.source = "image"  # an explicit still overrides topic mode
        self.image_topic = (self.get_parameter("image_topic").value
                            or self.cfg.topics.cam_overhead_raw)
        self.overlay_topic = (self.get_parameter("overlay_topic").value
                              or self.cfg.topics.cam_overhead_overlay)
        self.params = BoxParams(
            min_texture_std=float(self.get_parameter("min_texture_std").value))
        self._last_proc = 0.0

        # detector: ORB template match (object-specific, default) + texture fallback
        self.method = str(self.get_parameter("method").value).lower()
        self.fallback_texture = bool(self.get_parameter("fallback_texture").value)
        self.matcher = None
        if self.method == "orb":
            import cv2
            tpath = self.get_parameter("template_path").value or os.path.join(
                os.path.dirname(cfg_path), "box_template.jpg")
            tmpl = cv2.imread(tpath)
            if tmpl is None:
                self.get_logger().warn(
                    f"no box template at {tpath}; falling back to texture detector")
                self.method = "texture"
            else:
                self.matcher = OrbBoxMatcher(tmpl, BoxOrbParams(
                    min_inliers=int(self.get_parameter("orb_min_inliers").value)))
                self.get_logger().info(
                    f"ORB box matcher ready (template {tpath}, "
                    f"min_inliers={self.matcher.params.min_inliers})")

        hpath = self.get_parameter("homography_path").value or os.path.join(
            os.path.dirname(cfg_path), "c920_homography.npz"
        )
        self.mapper = None
        if os.path.exists(hpath):
            data = np.load(hpath)
            self.mapper = HomographyMappingAdapter(data["H"])
            self.get_logger().info(f"loaded homography from {hpath}")
        else:
            self.get_logger().warn(
                f"no homography at {hpath}; publishing detections only (no base-frame pose)")

        self.pub_pose = self.create_publisher(PoseStamped, self.cfg.topics.box_pose, 10)
        self.pub_det = self.create_publisher(Float64MultiArray, self.cfg.topics.box_detection, 10)
        self.pub_overlay = self.create_publisher(CompressedImage, self.overlay_topic, camera_qos())

        self.rate = max(0.2, float(self.get_parameter("rate_hz").value))
        if self.source == "topic":
            # SUBSCRIBE to the camera_publisher feed with the matching BEST_EFFORT
            # camera QoS (a RELIABLE subscriber gets NO frames from it).
            self.sub_img = self.create_subscription(
                CompressedImage, self.image_topic, self._on_image, camera_qos())
            src = f"topic {self.image_topic}"
        else:
            self.timer = self.create_timer(1.0 / self.rate, self._tick)
            src = f"device {device_path(self.device)}" if self.source == "device" \
                else f"image {self.image_path}"
        self.get_logger().info(
            f"box_detector up (source={self.source} [{src}], "
            f"min_texture_std={self.params.min_texture_std}, {self.rate} Hz) "
            f"-> pose {self.cfg.topics.box_pose}, overlay {self.overlay_topic}")

    def _detect(self, img):
        """ORB template match (object-specific) with optional texture fallback."""
        if self.matcher is not None:
            boxes = self.matcher.detect(img)
            if boxes or not self.fallback_texture:
                return boxes
        return detect_gray_box(img, self.params)

    def _grab(self):
        import cv2

        if self.source == "image":
            return cv2.imread(self.image_path)
        return grab_frame_gst(self.device, exposure=self.exposure)

    # -- frame handlers --------------------------------------------------

    def _tick(self):
        img = self._grab()
        if img is None:
            self.get_logger().warn("frame grab failed", throttle_duration_sec=5.0)
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
        boxes = self._detect(img)
        base_xy = None
        if boxes:
            b = boxes[0]
            det = Float64MultiArray()
            det.data = [float(b.u), float(b.v), float(b.width_px), float(b.height_px),
                        float(b.angle_deg), float(b.score)]
            self.pub_det.publish(det)

            if self.mapper is not None:
                xy = self.mapper.map_xy(PixelObservation(b.u, b.v), MappingContext())
                base_xy = (float(xy[0]), float(xy[1]))
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "panda_link0"
                msg.pose.position.x = base_xy[0]
                msg.pose.position.y = base_xy[1]
                msg.pose.position.z = self.box_top_z
                msg.pose.orientation.w = 1.0  # vertical (identity)
                self.pub_pose.publish(msg)

        # always publish the overlay feed (annotated when detected, raw otherwise)
        overlay = draw_box_overlay(img, boxes, base_xy=base_xy)
        jpeg = encode_jpeg(overlay, self.jpeg_quality)
        if jpeg is not None:
            out = CompressedImage()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = "c920_overhead"
            out.format = "jpeg"
            out.data = jpeg
            self.pub_overlay.publish(out)


def main(args=None):
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = BoxDetector()
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
