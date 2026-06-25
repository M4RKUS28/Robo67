#!/usr/bin/env python3
"""Clear a reflex/error and firmly hold the CURRENT pose (real Panda, MMC).

After a `robot_mode==4` reflex (e.g. self_collision_avoidance_violation or a
force threshold), the cartesian controller stops actuating until error recovery.
This script:
  1. Calls `/panda_error_recovery_service_server/error_recovery`.
  2. Restores protective collision thresholds.
  3. Sets a firm cartesian stiffness and streams the CURRENT measured EE as the
     equilibrium (no jerk) so the arm holds where it is.

Reusable during insertion tuning. Run INSIDE multipanda-container (sourced,
ROS_DOMAIN_ID=1), cartesian impedance controller already active:
    PYTHONPATH=/host/Code/Robo67/robo67_insertion python3 scripts/hw_recover.py
"""
from __future__ import annotations

import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node

from franka_msgs.msg import FrankaState
from franka_msgs.srv import ErrorRecovery, SetFullCollisionBehavior
from multi_mode_control_msgs.srv import SetCartesianImpedance

from robo67_insertion.lib import geometry
from hw_cmd_iface import CmdIface, quat_to_R

ROBOT_STATE = "/franka_robot_state_broadcaster/robot_state"
RECOVERY = "/panda_error_recovery_service_server/error_recovery"
COLLISION = "/panda_param_service_server/set_full_collision_behavior"

SAFE_TQ_HI = [40.0, 40.0, 38.0, 38.0, 12.0, 12.0, 12.0]
SAFE_TQ_LO = [20.0, 20.0, 18.0, 18.0, 10.0, 10.0, 8.0]
SAFE_F_HI = [45.0] * 6
SAFE_F_LO = [20.0] * 6


def stiffness_colmajor(trans, rot):
    diag = [trans, trans, trans, rot, rot, rot]
    m = np.zeros((6, 6))
    for i in range(6):
        m[i, i] = diag[i]
    return m.flatten(order="F").tolist()


class Recover(Node):
    def __init__(self, cmd_mode="auto"):
        super().__init__("hw_recover")
        self.ee_xyz = None
        self.ee_quat = None
        self.q = [0.0] * 7
        self.mode = 0
        self.cmd = CmdIface(self, mode=cmd_mode)
        self.create_subscription(FrankaState, ROBOT_STATE, self._on_state, 10)
        self.cli_rec = self.create_client(ErrorRecovery, RECOVERY)
        self.cli_par = self.create_client(SetCartesianImpedance, self.cmd.parameters_service())
        self.cli_col = self.create_client(SetFullCollisionBehavior, COLLISION)

    def _on_state(self, m):
        xyz, quat = geometry.mat4_colmajor_to_xyz_quat(list(m.o_t_ee))
        self.ee_xyz = np.asarray(xyz, float)
        self.ee_quat = tuple(float(v) for v in quat)
        self.q = list(m.q)
        self.mode = m.robot_mode

    def wait_state(self, t=6.0):
        t0 = time.time()
        while self.ee_xyz is None and time.time() - t0 < t:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.ee_xyz is not None

    def recover(self):
        if not self.cli_rec.wait_for_service(timeout_sec=6.0):
            raise RuntimeError("error_recovery service unavailable")
        fut = self.cli_rec.call_async(ErrorRecovery.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
        res = fut.result()
        self.get_logger().info(f"error_recovery -> success={getattr(res,'success',None)} err='{getattr(res,'error','')}'")

    def set_collision(self):
        if not self.cli_col.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("collision service unavailable"); return
        req = SetFullCollisionBehavior.Request()
        req.lower_torque_thresholds_acceleration = SAFE_TQ_LO
        req.upper_torque_thresholds_acceleration = SAFE_TQ_HI
        req.lower_torque_thresholds_nominal = SAFE_TQ_LO
        req.upper_torque_thresholds_nominal = SAFE_TQ_HI
        req.lower_force_thresholds_acceleration = SAFE_F_LO
        req.upper_force_thresholds_acceleration = SAFE_F_HI
        req.lower_force_thresholds_nominal = SAFE_F_LO
        req.upper_force_thresholds_nominal = SAFE_F_HI
        fut = self.cli_col.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        self.get_logger().info("protective collision thresholds restored")

    def set_stiffness(self, trans, rot):
        if not self.cmd.stiffness_supported():
            self.get_logger().warn("stiffness service not available in subscriber mode; continuing")
            return
        if not self.cli_par.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("parameters service unavailable")
        req = SetCartesianImpedance.Request()
        req.stiffness = stiffness_colmajor(trans, rot)
        req.damping_ratio = [0.9] * 6
        req.nullspace_stiffness = 10.0
        fut = self.cli_par.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        self.get_logger().info(f"stiffness trans={trans} rot={rot}")

    def publish(self, xyz, quat):
        self.cmd.publish(xyz, quat, self.q)

    def tool_down(self):
        return float(np.dot(quat_to_R(self.ee_quat)[:, 2], [0, 0, -1]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trans", type=float, default=500.0)
    ap.add_argument("--rot", type=float, default=30.0)
    ap.add_argument("--hold-s", type=float, default=3.0)
    ap.add_argument("--cmd-mode", choices=["auto", "mmc", "subscriber"], default="auto")
    args = ap.parse_args()

    rclpy.init()
    n = Recover(cmd_mode=args.cmd_mode)
    if not n.wait_state():
        n.get_logger().error("no robot_state"); rclpy.shutdown(); return
    n.cmd.detect(timeout=3.0)
    print(f"[recover] before: mode={n.mode} EE={[round(v,4) for v in n.ee_xyz]} tool_down={round(n.tool_down(),3)} cmd_mode={n.cmd.mode}")

    n.recover()
    time.sleep(0.5)
    for _ in range(10):
        rclpy.spin_once(n, timeout_sec=0.1)
    print(f"[recover] after recovery: mode={n.mode}")

    n.set_collision()
    # capture the pose to hold AFTER recovery
    for _ in range(5):
        rclpy.spin_once(n, timeout_sec=0.1)
    hold_xyz = n.ee_xyz.copy()
    hold_quat = n.ee_quat
    n.set_stiffness(args.trans, args.rot)

    dt = 0.02
    t0 = time.time()
    while time.time() - t0 < args.hold_s:
        rclpy.spin_once(n, timeout_sec=dt)
        n.publish(hold_xyz, hold_quat)
    print(f"[recover] holding. mode={n.mode} EE={[round(v,4) for v in n.ee_xyz]} tool_down={round(n.tool_down(),3)}")
    print("[recover] done.")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
