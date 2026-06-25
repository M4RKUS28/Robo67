#!/usr/bin/env python3
"""Move the real Panda EE to a target XYZ (tool-down) and hold. Reusable primitive.

Carrot-on-a-stick: each control tick commands a setpoint a small `max_lead` ahead
of the ACTUAL EE toward the target (the MMC controller discards desired poses
> 0.1 m from current). Holds the current (near-vertical) orientation. Applies a
workspace AABB clamp, a force abort, and on a reflex (robot_mode 4) it calls
error_recovery and resumes (bounded retries).

Used by calibration (move to known points), MOVE_ABOVE, and manual jogging.

Run INSIDE multipanda-container (sourced, ROS_DOMAIN_ID=1), cartesian impedance
controller already active and holding:
    PYTHONPATH=/host/Code/Robo67/robo67_insertion python3 scripts/hw_move_to.py \
        --xyz 0.45 0.0 0.25 --speed 0.04
"""
from __future__ import annotations

import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node

from franka_msgs.msg import FrankaState
from franka_msgs.srv import ErrorRecovery
from multi_mode_control_msgs.srv import SetCartesianImpedance

from robo67_insertion.lib import geometry, safety
from hw_cmd_iface import CmdIface, quat_to_R

ROBOT_STATE = "/franka_robot_state_broadcaster/robot_state"
RECOVERY = "/panda_error_recovery_service_server/error_recovery"

WORKSPACE_AABB = [[0.20, 0.65], [-0.45, 0.45], [0.06, 0.55]]
MAX_LEAD_M = 0.04
FZ_ABORT_N = 30.0          # gripper baseline ~10 N; abort on +20 N extra
WATCHDOG_S = 0.25
NO_MOTION_ABORT_S = 2.5
NO_MOTION_EPS_M = 0.001


def stiffness_colmajor(trans, rot):
    diag = [trans, trans, trans, rot, rot, rot]
    m = np.zeros((6, 6))
    for i in range(6):
        m[i, i] = diag[i]
    return m.flatten(order="F").tolist()


class Mover(Node):
    def __init__(self, cmd_mode="auto"):
        super().__init__("hw_move_to")
        self.ee_xyz = None
        self.ee_quat = None
        self.q = [0.0] * 7
        self.wrench = [0.0] * 6
        self.mode = 0
        self.stamp = None
        self.cmd = CmdIface(self, mode=cmd_mode)
        self.create_subscription(FrankaState, ROBOT_STATE, self._on_state, 10)
        self.cli_rec = self.create_client(ErrorRecovery, RECOVERY)
        self.cli_par = self.create_client(SetCartesianImpedance, self.cmd.parameters_service())

    def _on_state(self, m):
        self.stamp = self.get_clock().now().nanoseconds * 1e-9
        xyz, quat = geometry.mat4_colmajor_to_xyz_quat(list(m.o_t_ee))
        self.ee_xyz = np.asarray(xyz, float)
        self.ee_quat = tuple(float(v) for v in quat)
        self.q = list(m.q)
        self.wrench = list(m.o_f_ext_hat_k)
        self.mode = m.robot_mode

    def wait_state(self, t=6.0):
        t0 = time.time()
        while self.ee_xyz is None and time.time() - t0 < t:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.ee_xyz is not None

    def set_stiffness(self, trans, rot):
        if not self.cmd.stiffness_supported():
            self.get_logger().warn("stiffness service not available in subscriber mode; continuing")
            return
        if not self.cli_par.wait_for_service(timeout_sec=5.0):
            return
        req = SetCartesianImpedance.Request()
        req.stiffness = stiffness_colmajor(trans, rot)
        req.damping_ratio = [0.9] * 6
        req.nullspace_stiffness = 10.0
        fut = self.cli_par.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

    def recover(self):
        if not self.cli_rec.wait_for_service(timeout_sec=5.0):
            return False
        fut = self.cli_rec.call_async(ErrorRecovery.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
        time.sleep(0.3)
        return True

    def publish(self, xyz, quat):
        self.cmd.publish(xyz, quat, self.q)

    def tool_down(self):
        return float(np.dot(quat_to_R(self.ee_quat)[:, 2], [0, 0, -1]))

    def move_to(self, target, hold_quat, speed=0.015, tol=0.006, timeout=90.0,
                rate=50.0, min_duration=1.0):
        """Drive the equilibrium along a straight line to ``target`` at constant ``speed``.

        Instead of a carrot anchored a fixed lead ahead of the EE (which on the
        soft subscriber impedance controller either stalls -- too little force --
        or, if handed the raw target, twitches there in one step), the commanded
        equilibrium is TIME-PARAMETERISED:

            sp(t) = start + alpha * (target - start),  alpha = clamp(t / T, 0, 1)
            T     = max(min_duration, dist / speed)

        so the setpoint walks the line at a true, predictable command velocity
        (``speed`` m/s) and the arm tracks a gentle ramp. Returns True once the
        trajectory completes and the EE is within ``tol``.
        """
        target = np.asarray(safety.clamp_to_workspace(list(target), WORKSPACE_AABB), float)
        dt = 1.0 / rate

        # anchor the trajectory at the current measured EE
        while self.ee_xyz is None:
            rclpy.spin_once(self, timeout_sec=dt)
        start = self.ee_xyz.copy()
        dist = float(np.linalg.norm(target - start))
        T = max(min_duration, dist / max(1e-4, speed))

        t0 = time.time()
        ee_ref = start.copy()
        no_motion_t0 = time.time()
        reflex_retries = 0
        last_log = 0.0
        print(f"  [move] straight-line {[round(v,3) for v in start]} -> "
              f"{[round(v,3) for v in target]}  dist={dist:.3f}m speed={speed} m/s T={T:.1f}s")
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=dt)
            if self.ee_xyz is None:
                continue
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.stamp is None or now - self.stamp > WATCHDOG_S:
                print("  [move] state stale -> holding"); continue
            if self.mode in (4, 5):
                if reflex_retries < 3:
                    reflex_retries += 1
                    print(f"  [move] reflex (mode {self.mode}) -> error_recovery (retry {reflex_retries})")
                    self.recover()
                    continue
                print("  [move] reflex persists -> abort"); return False
            if abs(self.wrench[2]) > FZ_ABORT_N:
                print(f"  [move] FORCE ABORT Fz={self.wrench[2]:.1f}"); return False

            te = time.time() - t0
            alpha = min(1.0, te / T)
            sp = start + alpha * (target - start)
            sp = safety.clamp_to_workspace(list(sp), WORKSPACE_AABB)
            self.publish(sp, hold_quat)

            err = float(np.linalg.norm(target - self.ee_xyz))
            if alpha >= 1.0 and err < tol:
                self.publish(target, hold_quat)
                return True
            # catch a wholly-ignored command path (controller not actuating):
            # the moving setpoint should drag the EE; if it never budges, abort.
            if np.linalg.norm(self.ee_xyz - ee_ref) < NO_MOTION_EPS_M:
                if time.time() - no_motion_t0 > NO_MOTION_ABORT_S:
                    print(f"  [move] NO MOTION: command path ignored (cmd_mode={self.cmd.mode}). Check FCI/SPoC and controller path.")
                    return False
            else:
                ee_ref = self.ee_xyz.copy()
                no_motion_t0 = time.time()
            if time.time() - last_log >= 0.5:
                last_log = time.time()
                print(f"  [move] a={alpha:.2f} ee={[round(v,3) for v in self.ee_xyz]} "
                      f"sp={[round(v,3) for v in sp]} d={err:.3f} Fz={self.wrench[2]:.1f} mode={self.mode}")
        print("  [move] timeout"); return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xyz", type=float, nargs=3, required=True)
    ap.add_argument("--speed", type=float, default=0.015,
                    help="straight-line command velocity (m/s); kept gentle by default")
    ap.add_argument("--trans", type=float, default=500.0)
    ap.add_argument("--rot", type=float, default=30.0)
    ap.add_argument("--hold-after-s", type=float, default=1.5)
    ap.add_argument("--cmd-mode", choices=["auto", "mmc", "subscriber"], default="auto")
    args = ap.parse_args()

    rclpy.init()
    n = Mover(cmd_mode=args.cmd_mode)
    if not n.wait_state():
        print("no state"); rclpy.shutdown(); return
    n.cmd.detect(timeout=3.0)
    hold_quat = n.ee_quat
    print(f"[move_to] start ee={[round(v,4) for v in n.ee_xyz]} tool_down={round(n.tool_down(),3)} -> {args.xyz} cmd_mode={n.cmd.mode}")
    n.set_stiffness(args.trans, args.rot)
    ok = n.move_to(np.array(args.xyz), hold_quat, speed=args.speed)
    # Hold the pose we actually reached (publish the measured EE as the
    # equilibrium) -- NEVER re-publish the raw target, which would step the soft
    # controller to it in one tick and twitch the arm.
    hold_xyz = n.ee_xyz.copy()
    t0 = time.time()
    while time.time() - t0 < args.hold_after_s:
        rclpy.spin_once(n, timeout_sec=0.02)
        n.publish(hold_xyz, hold_quat)
    print(f"[move_to] reached={ok} final ee={[round(v,4) for v in n.ee_xyz]} tool_down={round(n.tool_down(),3)} mode={n.mode}")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
