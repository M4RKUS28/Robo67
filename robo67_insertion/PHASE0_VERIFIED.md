# Phase 0 — Verified ROS interfaces (against the running sim)

Captured 2026-06-25 by launching `franka_sim.launch.py` in `multipanda-container` and
inspecting the live system. These names are now the source of truth in `config/robo67.yaml`.

## Sim bring-up gotchas (IMPORTANT)

1. **`LD_LIBRARY_PATH` must include MuJoCo + libfranka**, or `mujoco_node` dies with
   `libmujoco.so.3.2.0: cannot open shared object file` and the control plugin dies with
   `libfranka.so.0.9: cannot open shared object file`:
   ```
   export LD_LIBRARY_PATH=/home/developer/Libraries/mujoco/lib:/home/developer/Libraries/libfranka/lib:$LD_LIBRARY_PATH
   ```
2. **`franka_sim.launch.py` only spawns `joint_state_broadcaster`.** The MMC and the
   franka state broadcaster must be spawned manually afterwards:
   ```
   ros2 run controller_manager spawner franka_robot_state_broadcaster -c /controller_manager
   ros2 run controller_manager spawner multi_mode_controller          -c /controller_manager
   ```
   (Use `scripts/sim_bringup.sh` then `scripts/activate_cartesian.sh`.)
3. Activate the Cartesian impedance controller via the **namespaced** service:
   ```
   ros2 service call /multi_mode_controller/set_controllers \
     multi_mode_control_msgs/srv/SetControllers \
     "{controllers: [{name: panda_cartesian_impedance_controller, resources: [panda]}]}"
   ```

## Verified names (sim)

| Purpose | Name | Type |
|---|---|---|
| Activate controllers | `/multi_mode_controller/set_controllers` | `multi_mode_control_msgs/srv/SetControllers` |
| Get controllers | `/multi_mode_controller/get_controllers` | `multi_mode_control_msgs/srv/GetControllers` |
| Command pose | `/panda/panda_cartesian_impedance_controller/desired_pose` | `multi_mode_control_msgs/msg/CartesianImpedanceGoal` |
| Set stiffness | `/panda/panda_cartesian_impedance_controller/parameters` | `multi_mode_control_msgs/srv/SetCartesianImpedance` |
| Robot state | `/franka_robot_state_broadcaster/robot_state` | `franka_msgs/msg/FrankaState` |
| Gripper actions | `/panda_gripper_sim_node/{homing,move,grasp,gripper_action}` | `franka_msgs/action/{Homing,Move,Grasp}`, `control_msgs/action/GripperCommand` |
| Error recovery | `/panda_error_recovery_service_server/error_recovery` | `franka_msgs/srv/ErrorRecovery` (**hardware only** — absent in sim) |

Confirmed live: `desired_pose` has the controller subscribed (sub count 1); `robot_state`
publishes `o_t_ee` (EE pose, col-major 4x4), `o_f_ext_hat_k` (ext wrench), `robot_mode: 1` (IDLE).

## Corrections vs. earlier assumptions

- `set_controllers` is **namespaced** under `/multi_mode_controller/...`, not global `/set_controllers`.
- robot_state topic is `/franka_robot_state_broadcaster/robot_state` (no `/panda/` segment in single-arm sim).
- The subscriber example controller `/cartesian_impedance/pose_desired`
  (`franka_controllers/CartesianImpedanceController`) is also present but is **real-only / unconfigured**;
  we do NOT use it.

## Phase 3 integration findings (orchestrator vs. live sim)

1. **Sim boots PAUSED.** `GetSimInfo` reports `paused=True` initially; physics won't
   step until you unpause: `ros2 service call /set_pause mujoco_ros_msgs/srv/SetPause "{paused: false}"`.
   (`activate_cartesian.sh` now does this.)
2. **The MMC cartesian impedance controller DISCARDS any desired pose > 0.1 m (position) OR
   > 0.15 rad (orientation) from the CURRENT pose** — see
   `panda_cartesian_impedance_controller.cpp:28-47`. Two consequences:
   - Clamp every commanded setpoint to a small **lead ahead of the ACTUAL EE**
     (`safety.max_lead_m`, default 0.05, MUST be < 0.1), NOT relative to the previous command.
     Anchoring to the previous command lets the setpoint outrun the lagging arm, after which the
     controller silently rejects everything and the arm freezes.
   - The commanded **orientation must be within 0.15 rad of current**. The orchestrator holds the
     current EE quaternion (computed with `geometry.mat4_colmajor_to_xyz_quat`, proper Shepperd's
     method) -> ~0 rotation error -> accepted. WARNING: a naive trace-only quaternion is degenerate
     for the Panda's ~180deg tool-down orientation and yields a 3.14 rad error -> ALL poses discarded
     -> arm frozen. Use the library's converter.
3. **Never call `rclpy.spin_until_future_complete` inside a timer/subscription callback**
   (re-entrant spin -> executor deadlock/hang). The orchestrator sets stiffness fire-and-forget
   (`call_async`, no spin) because the contact-stiffness switch happens inside the control loop.
4. **`o_f_ext_hat_k` is all zeros in sim** — MuJoCo here does not estimate the external wrench,
   so force-based contact detection CANNOT be validated in sim. Validate descend-to-contact +
   spiral on HARDWARE only (matches the design: sim = plumbing/logic, hardware = insertion).
5. Cartesian tracking is good from a well-conditioned pose, poor near singularities. From the
   sim home pose (EE z~0.59) the orchestrator descended ~10 cm and reached the MOVE_ABOVE target
   in <1 s. A stale sim instance whose arm had drifted to z~1.10 (near full vertical extension)
   tracked Z very slowly because that is near a singular direction. Start from a reasonable pose.
6. VALIDATED in sim (orchestrator, fresh sim, ROS_DOMAIN_ID isolated): controller activation,
   50 Hz `CartesianImpedanceGoal` streaming accepted by the controller, `FrankaState` parsing
   (`o_t_ee` -> xyz/quat, `o_f_ext_hat_k[2]` -> Fz), arm tracking a ~10 cm commanded descent,
   FSM `IDLE -> MOVE_ABOVE -> DESCEND_TO_CONTACT`, workspace + lead clamps, and the
   contact-stiffness switch without deadlock. (Force-gated SEARCH_SPIRAL/insert is HW-only since
   sim wrench is zero; that path is covered by `test_insertion_fsm.py`.)

## Hardware deltas to confirm in Phase 4

- Gripper namespace on real hardware (sim uses `/panda_gripper_sim_node`).
- Whether the real bringup auto-spawns MMC + state broadcaster, or needs the same manual spawn.
- `error_recovery` service is present on hardware (used after `ControlException`).
- Real launch is `franka.launch.py robot_ip:=<fci-ip>` (or `multimode_franka.launch.py`).
