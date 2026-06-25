#!/usr/bin/env python3
"""Command-path adapter for Panda Cartesian control scripts.

Supports two real-world controller paths:
- mmc: /panda/panda_cartesian_impedance_controller/desired_pose
       (multi_mode_control_msgs/CartesianImpedanceGoal)
- subscriber: /cartesian_impedance/pose_desired
       (std_msgs/Float64MultiArray = [px,py,pz,R00..R22] row-major)
"""
from __future__ import annotations

import time

import numpy as np
from multi_mode_control_msgs.msg import CartesianImpedanceGoal
from std_msgs.msg import Float64MultiArray

TOPIC_MMC = "/panda/panda_cartesian_impedance_controller/desired_pose"
TOPIC_SUBSCRIBER = "/cartesian_impedance/pose_desired"
PARAMETERS_MMC = "/panda/panda_cartesian_impedance_controller/parameters"


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


class CmdIface:
    def __init__(self, node, mode="auto"):
        self.node = node
        self.mode_req = mode
        self.mode = None
        self.pub_mmc = node.create_publisher(CartesianImpedanceGoal, TOPIC_MMC, 10)
        self.pub_sub = node.create_publisher(Float64MultiArray, TOPIC_SUBSCRIBER, 10)

    def detect(self, timeout=3.0):
        if self.mode_req in ("mmc", "subscriber"):
            self.mode = self.mode_req
            return self.mode

        t0 = time.time()
        mmc_subs = 0
        sub_subs = 0
        while time.time() - t0 < timeout:
            mmc_subs = self.node.count_subscribers(TOPIC_MMC)
            sub_subs = self.node.count_subscribers(TOPIC_SUBSCRIBER)
            if mmc_subs > 0 or sub_subs > 0:
                break
            # Allow graph discovery to settle.
            if hasattr(self.node, "ee_xyz"):
                import rclpy
                rclpy.spin_once(self.node, timeout_sec=0.05)
            else:
                time.sleep(0.05)

        if mmc_subs > 0 and sub_subs == 0:
            self.mode = "mmc"
        elif sub_subs > 0 and mmc_subs == 0:
            self.mode = "subscriber"
        elif mmc_subs > 0 and sub_subs > 0:
            self.mode = "mmc"
        else:
            self.mode = "mmc"

        self.node.get_logger().info(
            f"cmd iface mode={self.mode} (subs: mmc={mmc_subs}, subscriber={sub_subs})"
        )
        return self.mode

    def stiffness_supported(self):
        return self.mode == "mmc"

    def parameters_service(self):
        return PARAMETERS_MMC

    def publish(self, xyz, quat, q_n):
        if self.mode == "subscriber":
            R = quat_to_R(quat)
            px, py, pz = map(float, xyz)
            if abs(px) < 1e-6:
                px = 1e-6
            if abs(float(R[2, 2])) < 1e-6:
                R[2, 2] = -1e-6
            msg = Float64MultiArray()
            msg.data = [
                px, py, pz,
                float(R[0, 0]), float(R[0, 1]), float(R[0, 2]),
                float(R[1, 0]), float(R[1, 1]), float(R[1, 2]),
                float(R[2, 0]), float(R[2, 1]), float(R[2, 2]),
            ]
            self.pub_sub.publish(msg)
            return

        msg = CartesianImpedanceGoal()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, xyz)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(float, quat)
        msg.q_n = [float(v) for v in q_n]
        self.pub_mmc.publish(msg)
