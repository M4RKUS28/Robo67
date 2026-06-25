#!/usr/bin/env python3
"""hardware_insertion_node.py -- real-arm peg-in-hole insertion orchestrator.

This drives the REAL Franka through the multipanda ``franka_controllers``
Cartesian impedance controller that the real ``franka.launch.py`` bringup
auto-activates:

    command : /cartesian_impedance/pose_desired   (std_msgs/Float64MultiArray)
    data    : [px, py, pz, R00, R01, R02, R10, R11, R12, R20, R21, R22]
              (position in metres, orientation as a ROW-MAJOR 3x3 matrix)
              -- px MUST be non-zero and R22 MUST be non-zero or the controller
              ignores the update (see cartesian_impedance_controller.cpp).
    state   : /franka_robot_state_broadcaster/robot_state  (franka_msgs/FrankaState)
              o_t_ee (4x4 COLUMN-MAJOR), o_f_ext_hat_k (ext wrench, Fz = idx 2).

WHY THIS IS DIFFERENT FROM insertion_orchestrator_node.py
---------------------------------------------------------
The sim MMC controller DISCARDS any desired pose > 0.1 m from current, so the
sim orchestrator clamps every setpoint to a small lead *ahead of the actual EE*.
The real ``franka_controllers`` controller has NO such rejection: it is a pure
Cartesian impedance (tau = J^T (-K e - D v)). Contact force is produced by the
gap between the commanded equilibrium and the actual EE. To press into the
surface we must therefore command an equilibrium that ratchets BELOW the
surface, rate-limited from the previous COMMAND (not from the EE). With the
live stiffness pos_stiff=200 N/m, a force F needs an equilibrium gap F/200 m
(e.g. 5 N -> 2.5 cm).

SAFETY (every published setpoint passes all of these):
  * absolute workspace AABB clamp + hard Z floor + reachable-radius clamp
  * per-tick Euclidean step cap on the COMMAND (a true command-velocity limit)
  * bounded max press depth below the socket top
  * force abort: if |wrench| exceeds caps -> stop commanding and hold
  * state watchdog: stale FrankaState -> hold last command
  * px/R22 kept non-zero (controller quirk)

USAGE
-----
Offline math/plant self-test (no ROS, no robot):
    python3 -m robo67_insertion.nodes.hardware_insertion_node --selftest

Dry run on the real robot (reads state, computes + logs setpoints, publishes
NOTHING -- always do this first):
    python3 -m robo67_insertion.nodes.hardware_insertion_node \
        --socket-from-current --socket-top-dz -0.15 --dry-run

Live insertion (socket top taught explicitly, gentle, human at the e-stop):
    python3 -m robo67_insertion.nodes.hardware_insertion_node \
        --socket-xyz 0.45 0.0 0.10 --confirm
"""
from __future__ import annotations

import argparse
import math
import sys
import time

# numpy is required for the libs; import lazily-friendly but it is always present.
import numpy as np

from robo67_insertion.lib.command_path_adapters import ImpedanceCommandPathAdapter
from robo67_insertion.lib.insertion_intent import IntentParams, IntentSensors
from robo67_insertion.lib.wrench import BaselineEstimator
from robo67_insertion.lib import safety


# ---------------------------------------------------------------------------
# Pose helpers (no ROS).
# ---------------------------------------------------------------------------

def o_t_ee_to_xyz_R(o_t_ee):
    """Column-major 4x4 -> (xyz (3,), R (3x3))."""
    m = list(o_t_ee)
    xyz = np.array([m[12], m[13], m[14]], float)
    R = np.array([[m[0], m[4], m[8]],
                  [m[1], m[5], m[9]],
                  [m[2], m[6], m[10]]], float)
    return xyz, R


def R_to_rowmajor(R):
    """3x3 -> flat list of 9 floats, row-major (R00,R01,R02,R10,...)."""
    return [float(R[i][j]) for i in range(3) for j in range(3)]


# ---------------------------------------------------------------------------
# Insertion intent (canonical seam) -> impedance command path.
# The phase-transition logic lives ONLY in lib.insertion_intent (ADR-0001).
# Here we build the ImpedanceCommandPathAdapter, which translates each
# canonical target into a below-surface equilibrium goal for the soft real
# impedance controller (force F needs a gap F/pos_stiff). The node still
# rate-limits the command toward that goal and feeds back sensors.
# ---------------------------------------------------------------------------

STATES = ("MOVE_ABOVE", "DESCEND_TO_CONTACT", "SEARCH_SPIRAL",
          "PUSH_INSERT", "CONFIRM", "RETRACT", "DONE", "ERROR")


def build_intent_adapter(socket_xyz, *, pos_stiff=200.0, press_force_n=3.0,
                         insert_press_n=6.0, max_press_depth_m=0.05,
                         standoff_m=0.05, contact_fz_n=4.0, insert_depth_m=0.03,
                         spiral_max_radius_m=0.012, R=None):
    """Construct the impedance command-path adapter with real-arm defaults.

    Canonical (controller-agnostic) params carry the hardware tunings that
    historically lived in ``InsertionParams``; controller quirks (stiffness,
    press forces, held R) are the adapter's own.
    """
    params = IntentParams(
        standoff_m=standoff_m,
        approach_tol_m=0.006,
        contact_fz_threshold_n=contact_fz_n,
        insert_depth_m=insert_depth_m,
        z_drop_threshold_m=0.004,
        retry_limit=3,
        spiral_pitch_m=0.0025,
        spiral_speed_mps=0.004,
        spiral_max_radius_m=spiral_max_radius_m,
    )
    return ImpedanceCommandPathAdapter(
        socket_xyz, params, pos_stiff=pos_stiff, press_force_n=press_force_n,
        insert_press_n=insert_press_n, max_press_depth_m=max_press_depth_m, R=R,
    )


# ---------------------------------------------------------------------------
# Offline self-test: a trivial spring + virtual-table plant. Verifies the full
# sequence (descend->contact->spiral->drop->insert->confirm->retract) and that
# every command stays inside the safety envelope. No ROS, no robot.
# ---------------------------------------------------------------------------

def selftest(args):
    socket = np.array([0.45, 0.0, 0.10])
    adapter = build_intent_adapter(socket, pos_stiff=200.0)
    pos_stiff = adapter.pos_stiff
    insert_depth_m = adapter.module.params.insert_depth_m

    rate = 100.0
    dt = 1.0 / rate
    vmax = 0.03
    max_step = vmax / rate

    # virtual plant: ee follows command with first-order lag; the cube top at
    # table_z blocks the peg; the actual hole is a small disk of radius hole_r
    # at hole_xy, OFFSET from the commanded socket xy by a realistic alignment
    # error so the peg first contacts the cube top, then the spiral must find
    # the hole before it can drop by hole_depth.
    table_z = socket[2]
    hole_xy = socket[:2] + np.array([0.006, 0.0])   # 6 mm alignment error
    hole_r = 0.005
    hole_depth = insert_depth_m + 0.005

    ee = np.array([0.45, 0.0, 0.30])
    cmd = ee.copy()
    base = BaselineEstimator(alpha=0.1, initial=0.0)

    aabb = np.array([[0.2, 0.65], [-0.4, 0.4], [0.02, 0.6]])
    z_floor = 0.03

    max_speed = 0.0
    min_z = 1e9
    prev_cmd = None
    seen = set()
    phase = "MOVE_ABOVE"
    error = None
    t = 0.0
    n = 0
    while n < 200000:
        # fake force: how far the equilibrium is pushed past where the ee can go
        in_hole = (math.hypot(ee[0] - hole_xy[0], ee[1] - hole_xy[1]) <= hole_r)
        floor = table_z - (hole_depth if in_hole else 0.0)
        gap = max(0.0, ee[2] - cmd[2]) if ee[2] <= floor + 1e-6 else 0.0
        fz = pos_stiff * gap  # reaction force magnitude (>=0)

        if phase == "MOVE_ABOVE":
            base.update(fz)
        s = IntentSensors(ee_xyz=tuple(float(v) for v in ee), fz=fz,
                          fz_baseline=base.value, t=t)
        out = adapter.step(phase, s)
        goal = np.asarray(out.goal_xyz, float)
        seen.add(phase)

        # rate-limit the command toward goal, then safety clamp
        cmd = safety.clamp_step(cmd, goal, max_step)
        cmd = safety.clamp_to_workspace(cmd, aabb)
        cmd[2] = max(cmd[2], z_floor)

        if prev_cmd is not None:
            max_speed = max(max_speed, np.linalg.norm(cmd - prev_cmd) / dt)
        prev_cmd = cmd.copy()

        # plant: ee chases cmd, but cannot go below the floor
        ee = ee + 0.25 * (cmd - ee)
        ee[2] = max(ee[2], floor)
        min_z = min(min_z, ee[2])

        n += 1
        t += dt
        phase = out.next_phase
        error = out.error
        if out.done:
            seen.add(phase)
            break

    ok = (phase == "DONE"
          and "DESCEND_TO_CONTACT" in seen
          and "SEARCH_SPIRAL" in seen
          and "PUSH_INSERT" in seen
          and "CONFIRM" in seen
          and max_speed <= vmax + 1e-6
          and min_z >= z_floor - 1e-9)
    print("=== hardware_insertion self-test ===")
    print(f"states visited : {sorted(seen)}")
    print(f"final state    : {phase} (done={out.done}, err={error})")
    print(f"ticks          : {n}  ({t:.2f}s sim)")
    print(f"max cmd speed  : {max_speed:.4f} m/s (cap {vmax})")
    print(f"min ee z       : {min_z:.4f} m (floor {z_floor})")
    print("RESULT         :", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# ROS node.
# ---------------------------------------------------------------------------

def run_ros(args):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray
    from franka_msgs.msg import FrankaState
    try:
        from franka_msgs.srv import ErrorRecovery
    except Exception:
        ErrorRecovery = None

    aabb = np.array(args.workspace_aabb, float).reshape(3, 2)
    caps = [args.f_abort, args.f_abort, args.f_abort, 5.0, 5.0, 5.0]
    max_step = args.v_max / args.rate

    class Node_(Node):
        def __init__(self):
            super().__init__("hardware_insertion")
            self.pub = self.create_publisher(Float64MultiArray, args.topic, 10)
            self.sub = self.create_subscription(FrankaState, args.state_topic,
                                                self._on_state, 10)
            self.ee = None
            self.R = None
            self.wrench = [0.0] * 6
            self.stamp = None
            self.recovery = None
            if ErrorRecovery is not None:
                self.recovery = self.create_client(ErrorRecovery, args.recovery_srv)

        def _on_state(self, msg):
            xyz, R = o_t_ee_to_xyz_R(msg.o_t_ee)
            self.ee = xyz
            if self.R is None:
                self.R = R  # capture & hold the initial (tool-down) orientation
            self.wrench = list(msg.o_f_ext_hat_k)
            self.stamp = time.time()

    rclpy.init()
    node = Node_()

    # wait for first state
    node.get_logger().info(f"waiting for {args.state_topic} ...")
    deadline = time.time() + 10.0
    while time.time() < deadline and node.ee is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.ee is None:
        node.get_logger().error("no robot_state received -- refusing to run.")
        node.destroy_node(); rclpy.shutdown(); return 1

    ee0 = node.ee.copy()
    node.get_logger().info(f"EE now: x={ee0[0]:.3f} y={ee0[1]:.3f} z={ee0[2]:.3f}  "
                           f"wrench={[round(w,2) for w in node.wrench]}")

    # ---- NUDGE MODE: safest possible first real motion -----------------
    # Gently move the EE by a tiny fixed offset from its current pose, hold,
    # then return. Rate-limited + workspace-clamped. Use this to confirm the
    # control path drives the real arm before attempting an insertion.
    if args.nudge is not None:
        target = ee0 + np.array([0.0, 0.0, float(args.nudge)])
        target = safety.clamp_to_workspace(target, aabb)
        node.get_logger().info(f"NUDGE: {ee0.tolist()} -> {target.tolist()} "
                               f"(dz={args.nudge:+.3f} m), v_max={args.v_max} m/s")
        if args.confirm and not args.dry_run:
            try:
                if input("Area clear, e-stop in hand? Type YES to nudge: ").strip() != "YES":
                    node.get_logger().info("not confirmed -- exiting."); 
                    node.destroy_node(); rclpy.shutdown(); return 0
            except EOFError:
                node.destroy_node(); rclpy.shutdown(); return 1
        cmd = ee0.copy()
        msg = Float64MultiArray()
        dt = 1.0 / args.rate
        phase_t = 3.0  # seconds out, then back
        t0 = time.time()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            t = time.time() - t0
            if node.stamp is None or (time.time() - node.stamp) > args.watchdog_s:
                node.get_logger().warn("state stale -> holding", throttle_duration_sec=1.0)
                time.sleep(dt); continue
            if safety.force_exceeded(node.wrench, caps):
                node.get_logger().error("FORCE ABORT during nudge"); break
            goal = target if t < phase_t else ee0
            cmd = safety.clamp_step(cmd, goal, max_step)
            cmd = safety.clamp_to_workspace(cmd, aabb)
            px = cmd[0] if abs(cmd[0]) > 1e-6 else 1e-6
            R = node.R
            data = [px, float(cmd[1]), float(cmd[2])] + R_to_rowmajor(R)
            if not args.dry_run:
                msg.data = [float(x) for x in data]; node.pub.publish(msg)
            node.get_logger().info(
                f"nudge t={t:4.1f} cmd_z={cmd[2]:.3f} ee_z={node.ee[2]:.3f} "
                f"fz={node.wrench[2]:.2f}", throttle_duration_sec=0.5)
            if t > 2 * phase_t:
                break
            sleep = dt - ((time.time() - t0) - t)
            if sleep > 0:
                time.sleep(sleep)
        node.get_logger().info("nudge complete -- holding start pose.")
        node.destroy_node(); rclpy.shutdown(); return 0

    # resolve socket pose
    if args.socket_from_current:
        socket = np.array([ee0[0], ee0[1], ee0[2] + args.socket_top_dz])
    elif args.socket_xyz:
        socket = np.array(args.socket_xyz, float)
    else:
        node.get_logger().error("must give --socket-xyz or --socket-from-current")
        node.destroy_node(); rclpy.shutdown(); return 1

    # Canonical insertion seam -> impedance command path. Holds the captured
    # (tool-down) orientation; the loop rate-limits the command toward each goal.
    adapter = build_intent_adapter(
        socket, pos_stiff=args.pos_stiff, press_force_n=args.press_force,
        insert_press_n=args.insert_press, max_press_depth_m=args.max_press_depth,
        standoff_m=args.standoff, contact_fz_n=args.contact_fz,
        insert_depth_m=args.insert_depth, spiral_max_radius_m=args.spiral_max_radius,
        R=node.R,
    )
    node.get_logger().info(f"socket TOP center: {socket.tolist()}  "
                           f"(press_gap={adapter.press_gap_m*1000:.1f}mm, "
                           f"insert_gap={adapter.insert_gap_m*1000:.1f}mm)")

    # sanity: socket must be inside workspace
    if not np.all((socket >= aabb[:, 0]) & (socket <= aabb[:, 1])):
        node.get_logger().error(f"socket {socket.tolist()} outside workspace AABB "
                                f"{aabb.tolist()} -- aborting.")
        node.destroy_node(); rclpy.shutdown(); return 1

    if args.confirm and not args.dry_run:
        try:
            ans = input("Area clear, peg clamped, e-stop in hand? Type YES: ").strip()
        except EOFError:
            ans = ""
        if ans != "YES":
            node.get_logger().info("not confirmed -- exiting without moving.")
            node.destroy_node(); rclpy.shutdown(); return 0

    if not args.dry_run:
        for c in range(args.countdown, 0, -1):
            node.get_logger().info(f"inserting in {c} ...")
            time.sleep(1.0)

    phase = "MOVE_ABOVE"
    base = BaselineEstimator(alpha=0.05, initial=float(node.wrench[2]))
    cmd = node.ee.copy()
    msg = Float64MultiArray()
    dt = 1.0 / args.rate
    last_state = None
    aborted = False
    t0 = time.time()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            t = time.time() - t0

            # watchdog
            if node.stamp is None or (time.time() - node.stamp) > args.watchdog_s:
                node.get_logger().warn("robot_state stale -> holding",
                                       throttle_duration_sec=1.0)
                time.sleep(dt)
                continue

            ee = node.ee
            fz = float(node.wrench[2])

            # force abort
            if safety.force_exceeded(node.wrench, caps):
                node.get_logger().error(f"FORCE ABORT wrench={[round(w,2) for w in node.wrench]}")
                aborted = True
                break

            if phase == "MOVE_ABOVE":
                base.update(fz)

            s = IntentSensors(ee_xyz=tuple(float(v) for v in ee), fz=fz,
                              fz_baseline=base.value, t=t)
            out = adapter.step(phase, s)
            goal = np.asarray(out.goal_xyz, float)
            st = phase

            # rate-limit the COMMAND toward the goal, then safety-clamp
            cmd = safety.clamp_step(cmd, goal, max_step)
            cmd = safety.clamp_to_workspace(cmd, aabb)
            # never command an equilibrium more than max_press_depth below socket top
            cmd[2] = max(cmd[2], socket[2] - args.max_press_depth)

            if st != last_state:
                node.get_logger().info(
                    f"[{st}] ee=({ee[0]:.3f},{ee[1]:.3f},{ee[2]:.3f}) "
                    f"fz={fz:.2f} base={base.value:.2f} cmd_z={cmd[2]:.3f}")
                last_state = st

            # adapter helper keeps px / R22 non-zero (controller quirk)
            data = adapter.pose_desired(cmd)

            if args.dry_run:
                node.get_logger().info(
                    f"DRY [{st}] cmd=({data[0]:.3f},{data[1]:.3f},{data[2]:.3f}) "
                    f"ee=({ee[0]:.3f},{ee[1]:.3f},{ee[2]:.3f}) fz={fz:.2f}",
                    throttle_duration_sec=0.5)
            else:
                msg.data = [float(x) for x in data]
                node.pub.publish(msg)

            phase = out.next_phase
            if out.done:
                node.get_logger().info(
                    f"sequence finished: state={phase} err={adapter.module.error}")
                break
            if args.dry_run and t > args.dry_run_seconds:
                node.get_logger().info("dry-run time limit reached.")
                break

            sleep = dt - ((time.time() - t0) - t)
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        node.get_logger().info("interrupted -- holding last commanded pose.")

    if aborted and not args.dry_run and node.recovery is not None:
        node.get_logger().info("attempting error recovery ...")
        try:
            req = ErrorRecovery.Request()
            fut = node.recovery.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        except Exception as e:
            node.get_logger().warn(f"error recovery failed: {e}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


def build_parser():
    ap = argparse.ArgumentParser(description="Real-arm peg-in-hole insertion.")
    ap.add_argument("--selftest", action="store_true", help="offline plant test, no ROS")
    ap.add_argument("--dry-run", action="store_true",
                    help="read state + compute setpoints but DO NOT publish")
    ap.add_argument("--dry-run-seconds", type=float, default=20.0)
    ap.add_argument("--confirm", action="store_true", help="prompt YES before motion")
    ap.add_argument("--countdown", type=int, default=3)
    ap.add_argument("--nudge", type=float, default=None,
                    help="first-motion test: gently move EE by this dz (m), hold, return")

    ap.add_argument("--topic", default="/cartesian_impedance/pose_desired")
    ap.add_argument("--state-topic", default="/franka_robot_state_broadcaster/robot_state")
    ap.add_argument("--recovery-srv",
                    default="/panda_error_recovery_service_server/error_recovery")

    # socket pose
    ap.add_argument("--socket-xyz", type=float, nargs=3, default=None,
                    help="socket TOP center x y z in base frame (m)")
    ap.add_argument("--socket-from-current", action="store_true",
                    help="use current EE xy; socket-top z = ee_z + --socket-top-dz")
    ap.add_argument("--socket-top-dz", type=float, default=-0.15,
                    help="with --socket-from-current: dz from current EE to socket top")

    # motion / force
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--v-max", type=float, default=0.03, help="command speed cap (m/s)")
    ap.add_argument("--standoff", type=float, default=0.05)
    ap.add_argument("--pos-stiff", type=float, default=200.0,
                    help="MUST match the controller's pos_stiff")
    ap.add_argument("--contact-fz", type=float, default=4.0)
    ap.add_argument("--press-force", type=float, default=3.0)
    ap.add_argument("--insert-press", type=float, default=6.0)
    ap.add_argument("--max-press-depth", type=float, default=0.05)
    ap.add_argument("--insert-depth", type=float, default=0.03)
    ap.add_argument("--spiral-max-radius", type=float, default=0.012)

    # safety
    ap.add_argument("--f-abort", type=float, default=20.0)
    ap.add_argument("--watchdog-s", type=float, default=0.25)
    ap.add_argument("--workspace-aabb", type=float, nargs=6,
                    default=[0.20, 0.65, -0.40, 0.40, 0.02, 0.60],
                    help="xmin xmax ymin ymax zmin zmax (m)")
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv if argv is not None else sys.argv[1:])
    if args.selftest:
        return selftest(args)
    return run_ros(args)


if __name__ == "__main__":
    raise SystemExit(main())
