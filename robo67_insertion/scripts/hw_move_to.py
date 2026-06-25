#!/usr/bin/env python3
"""Move the real Panda EE to a target XYZ (tool-down) and hold. Reusable primitive.

Two-phase motion tuned for the soft real subscriber Cartesian-impedance controller:
  1. RAMP -- a straight-line, time-parameterised equilibrium trajectory at a slow
     constant command velocity (no twitch; the soft controller tracks a gentle ramp).
  2. SETTLE (``--overshoot``, default on) -- a software integral/overshoot outer
     loop that pushes the commanded equilibrium PAST the target along the residual
     error until the EE actually arrives within tol. This defeats the joint-stiction
     deadband of the pure-impedance controller (no integral term of its own), so the
     EE reaches the commanded pose instead of stopping ~cm short. The overshoot is
     anti-windup clamped, force-monitored, and the achieved (overshot) setpoint is
     held so the arm does not relax back when the script ends.

``--tool-down`` commands a vertical (z-down) orientation that preserves the current
yaw (minimal rotation), countering the rotational deadband that tilts the peg.

Applies a workspace AABB clamp + force abort; on a reflex (robot_mode 4/5) it calls
error_recovery and resumes (bounded retries). Used by calibration, MOVE_ABOVE, jogging.

Run INSIDE multipanda-container (sourced, ROS_DOMAIN_ID=1), cartesian impedance
controller already active and holding:
    PYTHONPATH=/host/Code/Robo67/robo67_insertion python3 scripts/hw_move_to.py \
        --xyz 0.45 0.0 0.25 --speed 0.015 --tool-down
"""
from __future__ import annotations

import argparse
import math
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


def _R_to_quat(R):
    """3x3 rotation matrix -> (x, y, z, w) quaternion (Shepperd's method)."""
    t = float(np.trace(R))
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def tool_down_quat(cur_quat):
    """(x,y,z,w) for a vertical (EE z-axis -> world -Z) orientation that keeps the
    CURRENT yaw -- the minimal rotation that just removes the tilt of the tool."""
    R = quat_to_R(cur_quat)
    new_z = np.array([0.0, 0.0, -1.0])
    cur_x = R[:, 0]
    x_h = np.array([cur_x[0], cur_x[1], 0.0])  # heading: current x flattened to floor
    if np.linalg.norm(x_h) < 1e-6:
        x_h = np.array([1.0, 0.0, 0.0])
    new_x = x_h / np.linalg.norm(x_h)
    new_y = np.cross(new_z, new_x)
    new_y /= np.linalg.norm(new_y)
    new_x = np.cross(new_y, new_z)  # re-orthonormalise
    Rn = np.column_stack([new_x, new_y, new_z])
    return _R_to_quat(Rn)


class Mover(Node):
    def __init__(self, cmd_mode="auto"):
        super().__init__("hw_move_to")
        self.ee_xyz = None
        self.ee_quat = None
        self.q = [0.0] * 7
        self.wrench = [0.0] * 6
        self.mode = 0
        self.stamp = None
        self.last_cmd_xyz = None   # last equilibrium we published (incl. overshoot)
        self.last_cmd_quat = None
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
        self.last_cmd_xyz = np.asarray(xyz, float).copy()
        self.last_cmd_quat = quat
        self.cmd.publish(xyz, quat, self.q)

    def tool_down(self):
        return float(np.dot(quat_to_R(self.ee_quat)[:, 2], [0, 0, -1]))

    def move_to(self, target, hold_quat, speed=0.015, tol=0.008, timeout=90.0,
                rate=50.0, min_duration=1.0, overshoot=True, overshoot_ki=0.5,
                overshoot_max=0.025, settle_timeout=12.0):
        """Drive the EE to ``target`` (RAMP) then close the residual (SETTLE).

        Phase 1 (RAMP): a TIME-PARAMETERISED straight line,
            sp(t) = start + alpha*(target-start),  alpha = clamp(t/T, 0, 1),
            T = max(min_duration, dist/speed),
        so the equilibrium walks the line at a true, slow command velocity and the
        soft controller tracks a gentle ramp (no twitch).

        Phase 2 (SETTLE, ``overshoot``): the pure-impedance controller has no
        integral term, so the arm stops short of the commanded pose once the
        restoring force can no longer beat joint stiction (a ~cm deadband). Here we
        add the missing integral: accumulate the residual into an overshoot vector
        ``I`` and command ``target + I`` so the equilibrium ratchets PAST the target
        until the EE arrives. ``I`` is anti-windup clamped to ``overshoot_max`` and
        every tick is force-aborted; the achieved (overshot) setpoint is the last
        thing published, so the caller can hold it without the EE relaxing back.
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

        # ---- Phase 1: time-parameterised straight-line ramp ----
        ramp_done = False
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
            if alpha >= 1.0:
                ramp_done = True
                break
            # no-motion guard (only while the setpoint is still moving): catch a
            # wholly-ignored command path. The end-of-ramp deadband stall is
            # EXPECTED and handled by the settle phase, so only check here.
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
        if not ramp_done:
            print("  [move] timeout (ramp)"); return False

        err = float(np.linalg.norm(target - self.ee_xyz))
        if err < tol:
            self.publish(target, hold_quat)
            print(f"  [move] reached on ramp, err={err:.4f}"); return True
        if not overshoot:
            print(f"  [move] ramp done, residual err={err:.4f} (overshoot disabled)")
            return False

        # ---- Phase 2: integral overshoot settle ----
        I = np.zeros(3)
        s0 = time.time()
        last_log = 0.0
        print(f"  [settle] overshoot integral start err={err:.4f} "
              f"ki={overshoot_ki} max={overshoot_max}m tol={tol}")
        while time.time() - s0 < settle_timeout:
            rclpy.spin_once(self, timeout_sec=dt)
            if self.ee_xyz is None:
                continue
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.stamp is None or now - self.stamp > WATCHDOG_S:
                print("  [settle] state stale -> holding"); continue
            if self.mode in (4, 5):
                if reflex_retries < 3:
                    reflex_retries += 1
                    print(f"  [settle] reflex (mode {self.mode}) -> error_recovery (retry {reflex_retries})")
                    self.recover()
                    continue
                print("  [settle] reflex persists -> abort"); return False
            if abs(self.wrench[2]) > FZ_ABORT_N:
                print(f"  [settle] FORCE ABORT Fz={self.wrench[2]:.1f}"); return False

            errv = target - self.ee_xyz
            errn = float(np.linalg.norm(errv))
            if errn < tol:
                self.publish(target + I, hold_quat)  # hold the overshot equilibrium
                print(f"  [settle] REACHED err={errn:.4f} overshoot={[round(v,4) for v in I]}")
                return True
            I = I + overshoot_ki * errv * dt
            n = float(np.linalg.norm(I))
            if n > overshoot_max:                    # anti-windup clamp
                I = I * (overshoot_max / n)
            sp = safety.clamp_to_workspace(list(target + I), WORKSPACE_AABB)
            self.publish(sp, hold_quat)
            if time.time() - last_log >= 0.5:
                last_log = time.time()
                print(f"  [settle] ee={[round(v,3) for v in self.ee_xyz]} err={errn:.4f} "
                      f"I={[round(v,4) for v in I]} Fz={self.wrench[2]:.1f} mode={self.mode}")
        final = float(np.linalg.norm(target - self.ee_xyz))
        print(f"  [settle] timeout, final err={final:.4f} (overshoot cap {overshoot_max}m)")
        return final < tol


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xyz", type=float, nargs=3, required=True)
    ap.add_argument("--speed", type=float, default=0.015,
                    help="straight-line command velocity (m/s); kept gentle by default")
    ap.add_argument("--trans", type=float, default=500.0)
    ap.add_argument("--rot", type=float, default=30.0)
    ap.add_argument("--hold-after-s", type=float, default=1.5)
    ap.add_argument("--cmd-mode", choices=["auto", "mmc", "subscriber"], default="auto")
    ap.add_argument("--tol", type=float, default=0.008, help="reach tolerance (m)")
    ap.add_argument("--tool-down", action="store_true",
                    help="command a vertical (z-down) orientation preserving current yaw")
    ap.add_argument("--overshoot", action=argparse.BooleanOptionalAction, default=True,
                    help="integral settle past target to beat the stiction deadband "
                         "(--no-overshoot to disable)")
    ap.add_argument("--overshoot-ki", type=float, default=0.5,
                    help="integral gain for the overshoot settle")
    ap.add_argument("--overshoot-max", type=float, default=0.025,
                    help="anti-windup cap on the overshoot magnitude (m)")
    ap.add_argument("--settle-timeout", type=float, default=12.0)
    args = ap.parse_args()

    rclpy.init()
    n = Mover(cmd_mode=args.cmd_mode)
    if not n.wait_state():
        print("no state"); rclpy.shutdown(); return
    n.cmd.detect(timeout=3.0)
    hold_quat = tool_down_quat(n.ee_quat) if args.tool_down else n.ee_quat
    print(f"[move_to] start ee={[round(v,4) for v in n.ee_xyz]} tool_down={round(n.tool_down(),3)} "
          f"-> {args.xyz} cmd_mode={n.cmd.mode} overshoot={args.overshoot} tool_down_cmd={args.tool_down}")
    n.set_stiffness(args.trans, args.rot)
    ok = n.move_to(np.array(args.xyz), hold_quat, speed=args.speed, tol=args.tol,
                   overshoot=args.overshoot, overshoot_ki=args.overshoot_ki,
                   overshoot_max=args.overshoot_max, settle_timeout=args.settle_timeout)
    # Hold the LAST commanded equilibrium (which includes the overshoot that keeps
    # the EE on target) -- NEVER re-publish the raw target or measured EE, either of
    # which would drop the overshoot and let the arm relax back into the deadband.
    hold_xyz = n.last_cmd_xyz.copy() if n.last_cmd_xyz is not None else n.ee_xyz.copy()
    t0 = time.time()
    while time.time() - t0 < args.hold_after_s:
        rclpy.spin_once(n, timeout_sec=0.02)
        n.publish(hold_xyz, hold_quat)
    print(f"[move_to] reached={ok} final ee={[round(v,4) for v in n.ee_xyz]} "
          f"tool_down={round(n.tool_down(),3)} mode={n.mode}")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
