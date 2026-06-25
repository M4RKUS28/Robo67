#!/usr/bin/env python3
"""Hand-guiding helper for the real Panda under the MMC cartesian controller.

Lets a human physically reposition the arm safely, then locks the new pose.

GUIDE phase (default):
  - Raises collision thresholds so a human push does NOT trip a collision reflex
    (which would crash the bringup).
  - Sets near-zero cartesian stiffness and continuously streams the equilibrium
    at the *current measured* EE -> the arm floats (gravity-compensated) with no
    spring resistance. The human moves it freely.
  - Prints `tool-Z . down` live (1.0 == tool pointing straight down) so the
    operator knows when the peg axis is vertical.
  - Watches for a sentinel file (default /tmp/robo67_guide_done). When it appears
    it transitions to LOCK.

LOCK phase:
  - Freezes the equilibrium at the current pose, ramps stiffness up to a firm
    hold (no jerk, since equilibrium == actual), restores protective collision
    thresholds, holds briefly, then exits leaving the controller holding the
    new (tool-down) pose firmly.

Run INSIDE multipanda-container (ROS + ws sourced, ROS_DOMAIN_ID=1), AFTER the
cartesian impedance controller is already active:
    PYTHONPATH=/host/Code/Robo67/robo67_insertion python3 \
        scripts/hw_handguide.py
Then, when the operator reports the tool is pointing down:
    touch /tmp/robo67_guide_done
"""
from __future__ import annotations

import argparse
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node

from franka_msgs.msg import FrankaState
from franka_msgs.srv import SetFullCollisionBehavior
from multi_mode_control_msgs.srv import SetCartesianImpedance

from robo67_insertion.lib import geometry
from hw_cmd_iface import CmdIface

ROBOT_STATE = "/franka_robot_state_broadcaster/robot_state"
COLLISION = "/panda_param_service_server/set_full_collision_behavior"

SENTINEL = "/tmp/robo67_guide_done"

# Loose thresholds for guiding (won't reflex when pushed). Torque caps near the
# Panda joint limits [87,87,87,87,12,12,12] Nm.
GUIDE_TQ_HI = [87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0]
GUIDE_TQ_LO = [25.0, 25.0, 22.0, 22.0, 10.0, 10.0, 8.0]
GUIDE_F_HI = [100.0] * 6
GUIDE_F_LO = [25.0] * 6

# Protective thresholds restored at LOCK (still clear the insertion's 25 N abort).
SAFE_TQ_HI = [40.0, 40.0, 38.0, 38.0, 12.0, 12.0, 12.0]
SAFE_TQ_LO = [20.0, 20.0, 18.0, 18.0, 10.0, 10.0, 8.0]
SAFE_F_HI = [45.0] * 6
SAFE_F_LO = [20.0] * 6


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def stiffness_colmajor(trans, rot):
    diag = [trans, trans, trans, rot, rot, rot]
    m = np.zeros((6, 6))
    for i in range(6):
        m[i, i] = diag[i]
    return m.flatten(order="F").tolist()


class Guide(Node):
    def __init__(self, cmd_mode="auto"):
        super().__init__("hw_handguide")
        self.ee_xyz = None
        self.ee_quat = None
        self.q = [0.0] * 7
        self.wrench = [0.0] * 6
        self.mode = 0
        self.stamp = None
        self.cmd = CmdIface(self, mode=cmd_mode)
        self.create_subscription(FrankaState, ROBOT_STATE, self._on_state, 10)
        self.cli_par = self.create_client(SetCartesianImpedance, self.cmd.parameters_service())
        self.cli_col = self.create_client(SetFullCollisionBehavior, COLLISION)

    def _on_state(self, m):
        self.stamp = self.get_clock().now().nanoseconds * 1e-9
        xyz, quat = geometry.mat4_colmajor_to_xyz_quat(list(m.o_t_ee))
        self.ee_xyz = np.asarray(xyz, float)
        self.ee_quat = tuple(float(v) for v in quat)
        self.q = list(m.q)
        self.wrench = list(m.o_f_ext_hat_k)
        self.mode = m.robot_mode

    def wait_state(self, timeout=8.0):
        t0 = time.time()
        while self.ee_xyz is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.ee_xyz is not None

    def set_collision(self, lo_tq, hi_tq, lo_f, hi_f):
        if not self.cli_col.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("collision service unavailable -- continuing without it")
            return
        req = SetFullCollisionBehavior.Request()
        req.lower_torque_thresholds_acceleration = lo_tq
        req.upper_torque_thresholds_acceleration = hi_tq
        req.lower_torque_thresholds_nominal = lo_tq
        req.upper_torque_thresholds_nominal = hi_tq
        req.lower_force_thresholds_acceleration = lo_f
        req.upper_force_thresholds_acceleration = hi_f
        req.lower_force_thresholds_nominal = lo_f
        req.upper_force_thresholds_nominal = hi_f
        fut = self.cli_col.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        self.get_logger().info(f"collision thresholds set (hi_f[2]={hi_f[2]})")

    def set_stiffness(self, trans, rot, nullspace):
        if not self.cmd.stiffness_supported():
            self.get_logger().warn("stiffness service not available in subscriber mode; continuing")
            return
        if not self.cli_par.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("parameters service unavailable")
        req = SetCartesianImpedance.Request()
        req.stiffness = stiffness_colmajor(trans, rot)
        req.damping_ratio = [0.9] * 6
        req.nullspace_stiffness = float(nullspace)
        fut = self.cli_par.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        self.get_logger().info(f"stiffness trans={trans} rot={rot} ns={nullspace}")

    def publish(self, xyz, quat):
        self.cmd.publish(xyz, quat, self.q)

    def tool_down(self):
        return float(np.dot(quat_to_R(self.ee_quat)[:, 2], [0, 0, -1]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sentinel", default=SENTINEL)
    ap.add_argument("--max-guide-s", type=float, default=300.0)
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--lock-trans", type=float, default=600.0)
    ap.add_argument("--lock-rot", type=float, default=30.0)
    ap.add_argument("--cmd-mode", choices=["auto", "mmc", "subscriber"], default="auto")
    args = ap.parse_args()

    if os.path.exists(args.sentinel):
        os.remove(args.sentinel)

    rclpy.init()
    n = Guide(cmd_mode=args.cmd_mode)
    if not n.wait_state():
        n.get_logger().error("no robot_state"); rclpy.shutdown(); return
    n.cmd.detect(timeout=3.0)

    print(f"[guide] start EE={[round(v,4) for v in n.ee_xyz]} tool_down={round(n.tool_down(),3)} cmd_mode={n.cmd.mode}")
    n.set_collision(GUIDE_TQ_LO, GUIDE_TQ_HI, GUIDE_F_LO, GUIDE_F_HI)
    n.set_stiffness(8.0, 2.0, 0.0)   # near-zero -> floats
    print("[guide] ARM IS NOW COMPLIANT. Move it slowly/smoothly; point the tool DOWN.")
    print(f"[guide] touch {args.sentinel} when tool_down ~ +0.95")

    dt = 1.0 / args.rate
    t0 = time.time()
    last_log = 0.0
    locked = False
    while time.time() - t0 < args.max_guide_s:
        rclpy.spin_once(n, timeout_sec=dt)
        if n.ee_xyz is None:
            continue
        now = time.time()
        if n.stamp is None or (n.get_clock().now().nanoseconds * 1e-9 - n.stamp) > 0.3:
            print("[guide] WARNING state stale");
        if n.mode in (4, 5):
            print(f"[guide] robot_mode={n.mode} (reflex/user-stop)! Guiding tripped a stop. Aborting.")
            break
        # equilibrium follows the actual EE -> no resistance
        n.publish(n.ee_xyz, n.ee_quat)
        if now - last_log >= 0.5:
            last_log = now
            print(f"  guiding  EE={[round(v,3) for v in n.ee_xyz]} tool_down={n.tool_down():+.3f} mode={n.mode}")
        if os.path.exists(args.sentinel):
            locked = True
            break

    if not locked:
        print("[guide] exited GUIDE without lock (timeout/abort). Leaving arm compliant-ish; re-run to retry.")
        n.destroy_node(); rclpy.shutdown(); return

    # LOCK
    frozen_xyz = n.ee_xyz.copy()
    frozen_quat = n.ee_quat
    print(f"[lock] freezing at EE={[round(v,4) for v in frozen_xyz]} tool_down={round(n.tool_down(),3)}")
    n.set_stiffness(args.lock_trans, args.lock_rot, 10.0)
    n.set_collision(SAFE_TQ_LO, SAFE_TQ_HI, SAFE_F_LO, SAFE_F_HI)
    t1 = time.time()
    while time.time() - t1 < 3.0:
        rclpy.spin_once(n, timeout_sec=dt)
        n.publish(frozen_xyz, frozen_quat)
    print(f"[lock] holding. final EE={[round(v,4) for v in n.ee_xyz]} tool_down={round(n.tool_down(),3)} mode={n.mode}")
    print("[lock] done -- controller now holds the new pose firmly.")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
