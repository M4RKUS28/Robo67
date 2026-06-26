"""Shared ROS QoS profiles.

ROS-only, but ``rclpy`` is imported lazily inside the function so this module
is importable on a host without ROS (e.g. the dashboard in mock mode, or the
host test suite). Not a ``lib/`` seam -- those are pure (numpy/cv2/stdlib, no
rclpy); QoS is inherently ROS plumbing.
"""
from __future__ import annotations


def camera_qos():
    """QoS for live camera / overlay image streams: BEST_EFFORT, KEEP_LAST depth 1.

    Image frames are latest-value: dropping a frame is fine, showing a STALE
    queued one is not. The default RELIABLE / KEEP_LAST(5) profile *queues* old
    frames and delivers them in order, which manifests as growing latency,
    alternating fresh/stale frames, and a long "ghost" tail after motion stops
    (the queue draining). BEST_EFFORT + depth 1 always delivers the newest frame
    and drops the rest.

    Both the publisher and ALL subscribers must use this: a BEST_EFFORT publisher
    with a RELIABLE subscriber is an incompatible QoS pair and no data flows.
    """
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )
