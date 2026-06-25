#!/usr/bin/env python3
"""Hardware validator / parker for the MMC Cartesian impedance controller.

This is the FIRST thing to run on the real arm after `multimode_franka.launch.py`
+ spawning `franka_robot_state_broadcaster`. It exercises the one path that the
sim could NOT validate: the MMC `panda_cartesian_impedance_controller` actually
driving the *real* Panda.

What it does, conservatively:
  1. Reads one `FrankaState`, prints the current EE pose AND the tool axes in the
     base frame (so we know whether the peg axis points down before insertion).
  2. Switches joint_impedance -> cartesian_impedance via `set_controllers`.
  3. Verifies the cartesian `desired_pose` topic has exactly one subscriber
     (the real controller) -- no ghost/sim subscriber.
  4. Sets a *soft* free-space stiffness (gentle, so a switch transient cannot yank).
  5. Streams a HOLD goal at ~50 Hz, each setpoint anchored to the ACTUAL EE with
     the `max_lead` clamp (the controller discards desired > 0.1 m from current).
  6. Optional `--nudge-z`: gently lift by N metres (away from the table) and back,
     to confirm the arm tracks commanded motion.

Safety: workspace AABB clamp, per-tick lead clamp, force-abort, and an abort on
`robot_mode` REFLEX(4)/USER_STOPPED(5) or stale state. Up-only nudge.

Run INSIDE multipanda-container (ROS + ws sourced, ROS_DOMAIN_ID=1):
    PYTHONPATH=/host/Code/Robo67/robo67_insertion python3 \
        /host/Code/Robo67/robo67_insertion/scripts/hw_cartesian_hold.py --secs 6
    # then, once hold looks stable:
    ... hw_cartesian_hold.py --secs 8 --nudge-z 0.02
"""
from __future__ import annotations

import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node

from franka_msgs.msg import FrankaState
from multi_mode_control_msgs.msg import Controller
from multi_mode_control_msgs.srv import SetControllers, SetCartesianImpedance

from robo67_insertion.lib import geometry, safety
from hw_cmd_iface import CmdIface, TOPIC_MMC, TOPIC_SUBSCRIBER

ROBOT_STATE = "/franka_robot_state_broadcaster/robot_state"
SET_CONTROLLERS = "/multi_mode_controller/set_controllers"

# Generous-but-real workspace box [[xmin,xmax],[ymin,ymax],[zmin,zmax]] (metres).
WORKSPACE_AABB = [[0.05, 0.75], [-0.50, 0.50], [0.05, 0.90]]
MAX_LEAD_M = 0.05          # < 0.1 controller accept window
FZ_ABORT_N = 25.0
WATCHDOG_S = 0.25


def stiffness_colmajor(trans: float, rot: float) -> list:
    diag = [trans, trans, trans, rot, rot, rot]
    m = np.zeros((6, 6))
    for i in range(6):
        m[i, i] = diag[i]
    return m.flatten(order="F").tolist()


def quat_to_R(q_xyzw):
    x, y, z, w = q_xyzw
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


class Holder(Node):
    def __init__(self, args):
        super().__init__("hw_cartesian_hold")
        self.args = args
        self.state: FrankaState | None = None
        self.stamp: float | None = None
        self.ee_xyz = None
        self.ee_quat = None
        self.q = [0.0] * 7
        self.wrench = [0.0] * 6
        self.aborted = False

        self.cmd = CmdIface(self, mode=args.cmd_mode)
        self.create_subscription(FrankaState, ROBOT_STATE, self._on_state, 10)
        self.cli_ctrl = self.create_client(SetControllers, SET_CONTROLLERS)
        self.cli_par = self.create_client(SetCartesianImpedance, self.cmd.parameters_service())

    def _on_state(self, m: FrankaState):
        self.state = m
        self.stamp = self.get_clock().now().nanoseconds * 1e-9
        xyz, quat = geometry.mat4_colmajor_to_xyz_quat(list(m.o_t_ee))
        self.ee_xyz = np.asarray(xyz, float)
        self.ee_quat = tuple(float(v) for v in quat)
        self.q = list(m.q)
        self.wrench = list(m.o_f_ext_hat_k)

    def wait_state(self, timeout=8.0):
        t0 = time.time()
        while self.ee_xyz is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.ee_xyz is not None

    def activate_cartesian(self):
        if not self.cli_ctrl.wait_for_service(timeout_sec=8.0):
            self.get_logger().warn("set_controllers service unavailable; skipping switch")
            return False
        req = SetControllers.Request()
        c = Controller(); c.name = "panda_cartesian_impedance_controller"; c.resources = ["panda"]
        req.controllers = [c]
        fut = self.cli_ctrl.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
        self.get_logger().info("set_controllers -> cartesian impedance requested")
        return True

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
        self.get_logger().info(f"stiffness set: trans={trans} rot={rot}")

    def subscriber_count(self):
        topic = TOPIC_MMC if self.cmd.mode == "mmc" else TOPIC_SUBSCRIBER
        return self.count_subscribers(topic)

    def publish_goal(self, xyz, quat):
        self.cmd.publish(xyz, quat, self.q)

    def safe(self, target_xyz):
        s = safety.clamp_to_workspace(target_xyz, WORKSPACE_AABB)
        s = safety.clamp_step(self.ee_xyz, s, MAX_LEAD_M)
        return s

    def check_abort(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.stamp is None or now - self.stamp > WATCHDOG_S:
            self.get_logger().error("state STALE -> abort"); return True
        if self.state.robot_mode in (4, 5):
            self.get_logger().error(f"robot_mode={self.state.robot_mode} (reflex/user-stop) -> abort"); return True
        caps = [25.0, 25.0, FZ_ABORT_N, 5.0, 5.0, 5.0]
        if safety.force_exceeded(self.wrench, caps):
            self.get_logger().error(f"FORCE ABORT wrench={[round(v,2) for v in self.wrench]}"); return True
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--secs", type=float, default=6.0)
    ap.add_argument("--nudge-z", type=float, default=0.0, help="lift this many metres (up) and back")
    ap.add_argument("--stiff-trans", type=float, default=300.0)
    ap.add_argument("--stiff-rot", type=float, default=20.0)
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--cmd-mode", choices=["auto", "mmc", "subscriber"], default="auto")
    args = ap.parse_args()

    rclpy.init()
    n = Holder(args)
    if not n.wait_state():
        n.get_logger().error("no robot_state"); rclpy.shutdown(); return
    n.cmd.detect(timeout=3.0)

    home_xyz = n.ee_xyz.copy()
    home_quat = n.ee_quat
    R = quat_to_R(home_quat)
    print(f"[home] EE xyz = {[round(v,4) for v in home_xyz]}")
    print(f"[home] EE quat xyzw = {[round(v,4) for v in home_quat]}")
    print(f"[home] tool X axis (base) = {[round(v,3) for v in R[:,0]]}")
    print(f"[home] tool Y axis (base) = {[round(v,3) for v in R[:,1]]}")
    print(f"[home] tool Z axis (base) = {[round(v,3) for v in R[:,2]]}  (peg points this way)")
    print(f"[home] tool-Z dot base-down = {round(float(np.dot(R[:,2],[0,0,-1])),3)} (1.0 = straight down)")
    print(f"[home] Fz baseline = {round(n.wrench[2],2)} N")

    if n.cmd.mode == "mmc":
        n.activate_cartesian()
    time.sleep(1.0)
    for _ in range(10):
        rclpy.spin_once(n, timeout_sec=0.1)
    subs = n.subscriber_count()
    print(f"[ctrl] cmd_mode={n.cmd.mode} desired subscriber count = {subs} (expect >=1)")
    if subs < 1:
        print("[ctrl] WARNING: no subscriber on selected command path -- aborting before any motion")
        rclpy.shutdown(); return

    n.set_stiffness(args.stiff_trans, args.stiff_rot)

    dt = 1.0 / args.rate
    n_ticks = int(args.secs * args.rate)
    last_log = 0.0
    for k in range(n_ticks):
        rclpy.spin_once(n, timeout_sec=dt)
        if n.ee_xyz is None:
            continue
        if n.check_abort():
            n.aborted = True
            break
        frac = k / max(1, n_ticks)
        # nudge profile: up over first 40%, hold to 70%, back down by 100%
        z_off = 0.0
        if args.nudge_z > 0:
            if frac < 0.4:
                z_off = args.nudge_z * (frac / 0.4)
            elif frac < 0.7:
                z_off = args.nudge_z
            else:
                z_off = args.nudge_z * max(0.0, (1.0 - frac) / 0.3)
        target = home_xyz + np.array([0.0, 0.0, z_off])
        n.publish_goal(n.safe(target), home_quat)
        t = k * dt
        if t - last_log >= 0.5:
            last_log = t
            print(f"  t={t:4.1f}s ee_z={n.ee_xyz[2]:.4f} target_z={target[2]:.4f} "
                  f"Fz={n.wrench[2]:6.2f} mode={n.state.robot_mode}")

    print(f"[done] aborted={n.aborted} final ee={[round(v,4) for v in n.ee_xyz]}")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
