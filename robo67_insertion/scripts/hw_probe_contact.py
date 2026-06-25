#!/usr/bin/env python3
"""Guarded descend-to-contact on the real Panda (MMC cartesian impedance).

Finds the surface height in the robot base frame by slowly lowering the tool
(carrot-on-a-stick) until the external Fz rises a threshold above its free-space
baseline, then retracts. This is the DESCEND_TO_CONTACT primitive and the first
real-hardware validation of force-based contact (sim wrench is identically zero).

Reports the contact Z (tool tip at the surface). Run INSIDE multipanda-container
(sourced, ROS_DOMAIN_ID=1), cartesian impedance active and holding tool-down:
    PYTHONPATH=/host/Code/Robo67/robo67_insertion python3 scripts/hw_probe_contact.py \
        --start-xyz 0.50 0.05 0.22 --max-drop 0.20 --contact-n 6
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
from hw_cmd_iface import CmdIface

ROBOT_STATE = "/franka_robot_state_broadcaster/robot_state"
RECOVERY = "/panda_error_recovery_service_server/error_recovery"

WORKSPACE_AABB = [[0.20, 0.65], [-0.45, 0.45], [0.02, 0.55]]
FZ_HARD_ABORT_N = 35.0     # over baseline -> hard stop
NO_MOTION_ABORT_S = 2.5
NO_MOTION_EPS_M = 0.001


def stiffness_colmajor(trans, rot):
    diag = [trans, trans, trans, rot, rot, rot]
    m = np.zeros((6, 6))
    for i in range(6):
        m[i, i] = diag[i]
    return m.flatten(order="F").tolist()


class Prober(Node):
    def __init__(self, cmd_mode="auto"):
        super().__init__("hw_probe_contact")
        self.ee_xyz = None
        self.ee_quat = None
        self.q = [0.0] * 7
        self.fz = 0.0
        self.mode = 0
        self.stamp = None
        self.cmd = CmdIface(self, mode=cmd_mode)
        self.create_subscription(FrankaState, ROBOT_STATE, self._on_state, 10)
        self.cli_par = self.create_client(SetCartesianImpedance, self.cmd.parameters_service())
        self.cli_rec = self.create_client(ErrorRecovery, RECOVERY)

    def _on_state(self, m):
        self.stamp = self.get_clock().now().nanoseconds * 1e-9
        xyz, quat = geometry.mat4_colmajor_to_xyz_quat(list(m.o_t_ee))
        self.ee_xyz = np.asarray(xyz, float)
        self.ee_quat = tuple(float(v) for v in quat)
        self.q = list(m.q)
        self.fz = float(m.o_f_ext_hat_k[2])
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
        if self.cli_par.wait_for_service(timeout_sec=5.0):
            req = SetCartesianImpedance.Request()
            req.stiffness = stiffness_colmajor(trans, rot)
            req.damping_ratio = [0.9] * 6
            req.nullspace_stiffness = 10.0
            fut = self.cli_par.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

    def recover(self):
        if self.cli_rec.wait_for_service(timeout_sec=5.0):
            fut = self.cli_rec.call_async(ErrorRecovery.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
            time.sleep(0.3)

    def publish(self, xyz, quat):
        self.cmd.publish(xyz, quat, self.q)

    def carrot_to(self, target, quat, lead=0.02, tol=0.006, timeout=40.0, rate=50.0):
        dt = 1.0 / rate; t0 = time.time()
        ee_ref = None
        no_motion_t0 = None
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=dt)
            if self.ee_xyz is None: continue
            if ee_ref is None:
                ee_ref = self.ee_xyz.copy()
                no_motion_t0 = time.time()
            if self.mode in (4, 5): self.recover(); continue
            if np.linalg.norm(np.asarray(target) - self.ee_xyz) < tol:
                self.publish(target, quat); return True
            if np.linalg.norm(self.ee_xyz - ee_ref) < NO_MOTION_EPS_M:
                if time.time() - no_motion_t0 > NO_MOTION_ABORT_S:
                    print(f"[probe] NO MOTION: command path ignored (cmd_mode={self.cmd.mode}). Check FCI/SPoC and controller path.")
                    return False
            else:
                ee_ref = self.ee_xyz.copy()
                no_motion_t0 = time.time()
            sp = safety.clamp_to_workspace(list(target), WORKSPACE_AABB)
            sp = safety.clamp_step(self.ee_xyz, sp, lead)
            self.publish(sp, quat)
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start-xyz", type=float, nargs=3, default=None,
                    help="move here first; default = current XY at current Z")
    ap.add_argument("--max-drop", type=float, default=0.20)
    ap.add_argument("--contact-n", type=float, default=6.0, help="Fz rise over baseline = contact")
    ap.add_argument("--descend-lead", type=float, default=0.004, help="per-tick downward carrot")
    ap.add_argument("--descend-step", type=float, default=0.03,
                    help="max equilibrium lead below the EE during descent (m); at high "
                         "stiffness keep this small to cap the contact force (F=pos_stiff*step)")
    ap.add_argument("--retract", type=float, default=0.03)
    ap.add_argument("--cmd-mode", choices=["auto", "mmc", "subscriber"], default="auto")
    args = ap.parse_args()

    rclpy.init()
    n = Prober(cmd_mode=args.cmd_mode)
    if not n.wait_state():
        print("no state"); rclpy.shutdown(); return
    n.cmd.detect(timeout=3.0)
    quat = n.ee_quat
    print(f"[probe] cmd_mode={n.cmd.mode}")
    n.set_stiffness(400.0, 30.0)

    if args.start_xyz:
        print(f"[probe] moving to start {args.start_xyz}")
        n.carrot_to(np.array(args.start_xyz), quat)
    start = n.ee_xyz.copy()
    print(f"[probe] start EE={[round(v,4) for v in start]}")

    # baseline Fz over ~1.5 s while holding
    samples = []
    t0 = time.time()
    while time.time() - t0 < 1.5:
        rclpy.spin_once(n, timeout_sec=0.02)
        n.publish(start, quat); samples.append(n.fz)
    baseline = float(np.median(samples))
    print(f"[probe] Fz baseline = {baseline:.2f} N (n={len(samples)})")

    # descend
    rate = 50.0; dt = 1.0 / rate
    target_z = start[2]
    contact_z = None
    ee_ref = n.ee_xyz.copy()
    no_motion_t0 = time.time()
    while start[2] - target_z < args.max_drop:
        rclpy.spin_once(n, timeout_sec=dt)
        if n.mode in (4, 5):
            print(f"[probe] reflex mode={n.mode} -> treating as contact, recovering")
            contact_z = n.ee_xyz[2]; n.recover(); break
        dfz = n.fz - baseline
        if abs(dfz) >= args.contact_n:
            contact_z = n.ee_xyz[2]
            print(f"[probe] CONTACT dFz={dfz:.2f} at z={contact_z:.4f}")
            break
        if abs(dfz) >= FZ_HARD_ABORT_N:
            print(f"[probe] HARD ABORT dFz={dfz:.2f}"); break
        target_z -= args.descend_lead
        sp = np.array([start[0], start[1], target_z])
        sp = safety.clamp_to_workspace(list(sp), WORKSPACE_AABB)
        sp = safety.clamp_step(n.ee_xyz, sp, args.descend_step)
        n.publish(sp, quat)
        if np.linalg.norm(n.ee_xyz - ee_ref) < NO_MOTION_EPS_M:
            if time.time() - no_motion_t0 > NO_MOTION_ABORT_S:
                print(f"[probe] NO MOTION during descend (ee not moving; deadband > descend-step?). "
                      f"cmd_mode={n.cmd.mode} gap={n.ee_xyz[2]-sp[2]:.4f} dFz={dfz:+.2f}")
                break
        else:
            ee_ref = n.ee_xyz.copy()
            no_motion_t0 = time.time()
        # periodic log (time-based): EE z, commanded gap below EE, dFz
        if time.time() - no_motion_t0 < 0.05 or int((start[2] - target_z) * 1000) % 5 == 0:
            print(f"  z={n.ee_xyz[2]:.4f} sp_z={sp[2]:.4f} gap={n.ee_xyz[2]-sp[2]:+.4f} dFz={dfz:+.2f} mode={n.mode}")

    if contact_z is None:
        print(f"[probe] no contact within {args.max_drop} m (mat lower than reach or peg missing)")
    # retract
    up = np.array([start[0], start[1], (contact_z if contact_z else n.ee_xyz[2]) + args.retract])
    print(f"[probe] retracting to z={up[2]:.4f}")
    n.carrot_to(up, quat, lead=0.01)
    t0 = time.time()
    while time.time() - t0 < 1.0:
        rclpy.spin_once(n, timeout_sec=0.02); n.publish(up, quat)
    print(f"[probe] DONE contact_z={contact_z} final_ee={[round(v,4) for v in n.ee_xyz]}")
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
