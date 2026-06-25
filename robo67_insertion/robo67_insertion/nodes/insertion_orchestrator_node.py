#!/usr/bin/env python3
"""Insertion orchestrator node.

Thin rclpy wrapper around the canonical insertion seam via
:class:`~robo67_insertion.lib.command_path_adapters.MMCCommandPathAdapter`.
It activates the MMC Cartesian impedance controller, sets stiffness, reads
``FrankaState`` (EE pose + external wrench), runs the intent adapter at ~50 Hz,
applies the :mod:`~robo67_insertion.lib.safety` clamps to every setpoint, and
streams ``CartesianImpedanceGoal`` to the controller.

Run (in container, ROS + ws sourced, our package on PYTHONPATH):
    python3 -m robo67_insertion.nodes.insertion_orchestrator_node \
        --ros-args -p activate_controller:=true -p set_stiffness:=true

Phase-3 self-contained sim test (no detector): leave ``use_socket_topic`` false
and ``socket_xyz`` empty -> the socket is auto-placed ``auto_socket_below_m``
below the current EE so the arm descends and we can watch the loop + safety.
"""
from __future__ import annotations

import os

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from franka_msgs.msg import FrankaState
from multi_mode_control_msgs.msg import CartesianImpedanceGoal, Controller
from multi_mode_control_msgs.srv import SetControllers, SetCartesianImpedance

from robo67_insertion.config_schema import RoboConfig, load_config
from robo67_insertion.lib import geometry, safety
from robo67_insertion.lib.command_path_adapters import MMCCommandPathAdapter
from robo67_insertion.lib.insertion_intent import IntentParams, IntentSensors
from robo67_insertion.lib.wrench import BaselineEstimator


def _default_config_path() -> str:
    here = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    return os.path.join(here, "config", "robo67.yaml")


def stiffness_matrix(trans_xy: float, trans_z: float, rot: float) -> list:
    """Column-major 6x6 diagonal stiffness as a flat 36-float list."""
    diag = [trans_xy, trans_xy, trans_z, rot, rot, rot]
    m = np.zeros((6, 6), dtype=float)
    for i in range(6):
        m[i, i] = diag[i]
    return m.flatten(order="F").tolist()


class InsertionOrchestrator(Node):
    def __init__(self):
        super().__init__("insertion_orchestrator")
        self.declare_parameter("config_path", "")
        self.declare_parameter("use_socket_topic", False)
        self.declare_parameter("socket_xyz", [])
        self.declare_parameter("auto_socket_below_m", 0.15)
        self.declare_parameter("activate_controller", False)
        self.declare_parameter("set_stiffness", True)
        self.declare_parameter("dry_run", False)

        cfg_path = self.get_parameter("config_path").value or _default_config_path()
        self.cfg: RoboConfig = load_config(cfg_path)
        self.get_logger().info(f"loaded config from {cfg_path}")

        self.use_socket_topic = bool(self.get_parameter("use_socket_topic").value)
        self.socket_xyz_param = list(self.get_parameter("socket_xyz").value)
        self.auto_below = float(self.get_parameter("auto_socket_below_m").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)

        t = self.cfg.topics
        # State
        self.latest_state: FrankaState | None = None
        self.latest_stamp_s: float | None = None
        self.ee_xyz: np.ndarray | None = None
        self.ee_quat = None
        self.q = [0.0] * 7
        self.fz = 0.0
        self.wrench = [0.0] * 6
        self.baseline = BaselineEstimator(alpha=0.1, initial=0.0)
        self.adapter: MMCCommandPathAdapter | None = None
        self.state = "IDLE"
        self.prev_cmd: np.ndarray | None = None
        self.socket_xyz: np.ndarray | None = (
            np.asarray(self.socket_xyz_param, float) if len(self.socket_xyz_param) == 3 else None
        )
        self.contact_stiffness_set = False
        self.aborted = False

        # I/O
        self.pub_goal = self.create_publisher(CartesianImpedanceGoal, t.desired_pose, 10)
        self.create_subscription(FrankaState, t.robot_state, self._on_state, 10)
        if self.use_socket_topic:
            self.create_subscription(PoseStamped, t.socket_pose, self._on_socket, 10)

        self.cli_set_ctrl = self.create_client(SetControllers, t.set_controllers)
        self.cli_params = self.create_client(SetCartesianImpedance, t.parameters)

        if bool(self.get_parameter("activate_controller").value):
            self._activate_controller()
        if bool(self.get_parameter("set_stiffness").value):
            self._set_stiffness(contact=False)

        rate = max(1.0, float(self.cfg.insertion.control_rate_hz))
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f"orchestrator up @ {rate} Hz (dry_run={self.dry_run})")

    # -- service helpers -------------------------------------------------

    def _activate_controller(self):
        if not self.cli_set_ctrl.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("set_controllers service unavailable")
            return
        req = SetControllers.Request()
        c = Controller()
        c.name = self.cfg.topics.controller_name
        c.resources = [self.cfg.topics.arm_resource]
        req.controllers = [c]
        fut = self.cli_set_ctrl.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        self.get_logger().info("activated cartesian impedance controller")

    def _set_stiffness(self, contact: bool):
        # Fire-and-forget: NEVER spin here. This is also called from inside the
        # control timer (contact switch); spinning inside a callback deadlocks
        # the single-threaded executor. The async response is handled by spin().
        if not self.cli_params.service_is_ready():
            self.cli_params.wait_for_service(timeout_sec=2.0)
        s = self.cfg.stiffness
        req = SetCartesianImpedance.Request()
        if contact:
            req.stiffness = stiffness_matrix(
                s.contact_translational_xy, s.contact_z, s.contact_rotational
            )
        else:
            req.stiffness = stiffness_matrix(
                s.free_translational, s.free_translational, s.free_rotational
            )
        req.damping_ratio = [s.damping_ratio] * 6
        req.nullspace_stiffness = s.nullspace_stiffness
        self.cli_params.call_async(req)
        self.get_logger().info(f"requested stiffness (contact={contact})")

    # -- subscriptions ---------------------------------------------------

    def _on_state(self, msg: FrankaState):
        self.latest_state = msg
        self.latest_stamp_s = self.get_clock().now().nanoseconds * 1e-9
        xyz, quat = geometry.mat4_colmajor_to_xyz_quat(list(msg.o_t_ee))
        self.ee_xyz = xyz
        self.ee_quat = tuple(float(v) for v in quat)
        self.q = list(msg.q)
        self.wrench = list(msg.o_f_ext_hat_k)
        self.fz = float(msg.o_f_ext_hat_k[2])

    def _on_socket(self, msg: PoseStamped):
        self.socket_xyz = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], float
        )

    # -- control loop ----------------------------------------------------

    def _ensure_adapter(self) -> bool:
        if self.adapter is not None:
            return True
        if self.ee_xyz is None:
            return False
        if self.socket_xyz is None:
            if self.use_socket_topic:
                return False  # waiting for a detection
            # auto-place a virtual socket below the current EE (sim self-test)
            self.socket_xyz = self.ee_xyz + np.array([0.0, 0.0, -self.auto_below])
            self.get_logger().warn(f"auto socket_xyz = {self.socket_xyz.tolist()}")
        params = IntentParams(
            standoff_m=self.cfg.insertion.standoff_m,
            approach_tol_m=self.cfg.insertion.approach_tol_m,
            contact_fz_threshold_n=self.cfg.insertion.contact_fz_threshold_n,
            insert_depth_m=self.cfg.insertion.insert_depth_m,
            z_drop_threshold_m=self.cfg.insertion.z_drop_threshold_m,
            retry_limit=self.cfg.insertion.retry_limit,
            spiral_pitch_m=self.cfg.spiral.pitch_m,
            spiral_speed_mps=self.cfg.spiral.speed_mps,
            spiral_max_radius_m=self.cfg.spiral.max_radius_m,
        )
        # MMC command path: hold the current (down-pointing) orientation and let
        # the lead-clamp below produce the carrot step from the canonical target.
        self.adapter = MMCCommandPathAdapter(
            self.socket_xyz, params, down_quat=self.ee_quat
        )
        self.prev_cmd = self.ee_xyz.copy()
        self.get_logger().info(f"intent adapter created; socket={self.socket_xyz.tolist()}")
        return True

    def _tick(self):
        if self.aborted or self.ee_xyz is None or self.latest_stamp_s is None:
            return
        # watchdog
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.latest_stamp_s > self.cfg.safety.watchdog_s:
            self.get_logger().warn("robot_state stale -> holding", throttle_duration_sec=2.0)
            return
        if not self._ensure_adapter():
            return

        # force abort
        caps = [25.0, 25.0, self.cfg.safety.fz_abort_n, 5.0, 5.0, 5.0]
        if safety.force_exceeded(self.wrench, caps):
            self.get_logger().error(f"FORCE ABORT wrench={self.wrench}")
            self.aborted = True
            return

        # estimate free-space baseline only before contact
        if self.state in ("IDLE", "MOVE_ABOVE"):
            self.baseline.update(self.fz)

        s = IntentSensors(
            ee_xyz=tuple(float(v) for v in self.ee_xyz),
            fz=self.fz,
            fz_baseline=self.baseline.value,
            t=now,
        )
        cmd = self.adapter.step(self.state, s)

        # soften stiffness once we begin contact phase
        if self.state == "DESCEND_TO_CONTACT" and not self.contact_stiffness_set:
            self._set_stiffness(contact=True)
            self.contact_stiffness_set = True

        # safety: workspace clamp, then bound the command to a small lead AHEAD
        # OF THE ACTUAL EE. The MMC controller discards any desired pose > 0.1 m
        # from the current pose, so anchoring the clamp to ee (not the previous
        # command) keeps every setpoint inside the accept window and the arm
        # tracks smoothly (carrot-on-a-stick).
        safe = safety.clamp_to_workspace(cmd.desired_xyz, self.cfg.safety.workspace_aabb)
        safe = safety.clamp_step(self.ee_xyz, safe, self.cfg.safety.max_lead_m)

        if not self.dry_run:
            self._publish(safe, cmd.desired_quat)
        self.prev_cmd = np.asarray(safe, float)

        if cmd.next_state != self.state:
            self.get_logger().info(f"{self.state} -> {cmd.next_state}")
        self.state = cmd.next_state

        if cmd.done:
            if cmd.error:
                self.get_logger().error(f"insertion finished with error: {cmd.error}")
            else:
                self.get_logger().info("insertion DONE: complete")
            self.aborted = True  # stop commanding once finished

    def _publish(self, xyz, quat_xyzw):
        goal = CartesianImpedanceGoal()
        goal.pose.position.x = float(xyz[0])
        goal.pose.position.y = float(xyz[1])
        goal.pose.position.z = float(xyz[2])
        goal.pose.orientation.x = float(quat_xyzw[0])
        goal.pose.orientation.y = float(quat_xyzw[1])
        goal.pose.orientation.z = float(quat_xyzw[2])
        goal.pose.orientation.w = float(quat_xyzw[3])
        goal.q_n = [float(v) for v in self.q]
        self.pub_goal.publish(goal)


def main(args=None):
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = InsertionOrchestrator()
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
